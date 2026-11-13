// ControlLink.cpp -- see ControlLink.h.
//
// SPDX-License-Identifier: MIT
#include "ControlLink.h"

#include <string.h>

#include "../xbox_r4_protocol.h"
#include "Debug.h"
#include "hal.h"

namespace {

// One byte on the AT link, rounded up (TxQueue paces with the same figure).
constexpr uint32_t kByteUs = (UART_BITS_PER_BYTE * US_PER_S + LINK_BAUD - 1) / LINK_BAUD;
constexpr uint32_t kReplyUs = AT_REPLY_TIMEOUT_MS * US_PER_MS;
constexpr uint32_t kPollMs = MS_PER_S / CONTROL_PACKET_HZ;
constexpr uint32_t kDstMs = MS_PER_S / TELEMETRY_WIFI_HZ;
constexpr uint32_t kRateWindowMs = MS_PER_S;   // ctrlRxHz is "frames in the last second"

static_assert(kPollMs > 0 && kDstMs > 0, "CONTROL_PACKET_HZ / TELEMETRY_WIFI_HZ above 1000");
static_assert(XBOX_R4_STATE_LEN <= LINK_RX_PAYLOAD_BYTES, "AT reply buffer holds one XBOXSTATE");
static_assert(sizeof(ds_status_packet_t) == DS_STATUS_LEN, "AT+DSS fields = StatusPacket fields");

const char kPingLine[] = "AT\r\n";
const char kCapsLine[] = "AT" DS_AT_DSCAPS "?\r\n";
const char kStateLine[] = "AT" XBOX_R4_AT_STATE "?\r\n";
const char kBeginLine[] = "AT" XBOX_R4_AT_BEGIN "\r\n";
const char kDssHead[] = "AT" DS_AT_DSS "=";
const char kDstHead[] = "AT" DS_AT_DST "=";
const char kLineEnd[] = "\r\n";

// Appends text / an unsigned decimal at out[n]; the caller sized out for the longest line.
void appendStr(char* out, size_t& n, const char* text) {
  while (*text) out[n++] = *text++;
}

void appendU(char* out, size_t& n, uint32_t v) {
  char digits[sizeof(uint32_t) * BITS_PER_BYTE];   // more than the 10 a uint32_t needs
  size_t k = 0;
  do {
    digits[k++] = (char)('0' + v % DEC_BASE);
    v /= DEC_BASE;
  } while (v && k < sizeof digits);
  while (k) out[n++] = digits[--k];
}

// "a,b,c": exactly `count` unsigned decimal fields, each at most UINT8_MAX (every D-Sentry
// reply field is a u8). Anything else (spaces, signs, empty fields) is a malformed reply.
bool parseU8Fields(const uint8_t* data, size_t len, uint32_t* out, size_t count) {
  size_t field = 0;
  uint32_t value = 0;
  bool digits = false;
  for (size_t i = 0; i <= len; i++) {
    const bool end = i == len;
    const char c = end ? ',' : (char)data[i];
    if (c >= '0' && c <= '9') {
      value = value * DEC_BASE + (uint32_t)(c - '0');
      if (value > UINT8_MAX) return false;
      digits = true;
    } else if (c == ',') {
      if (!digits || field >= count) return false;
      out[field++] = value;
      value = 0;
      digits = false;
    } else {
      return false;
    }
  }
  return field == count;
}

uint16_t le16(const uint8_t* p, size_t ofs) {
  return (uint16_t)(p[ofs] | (uint16_t)(p[ofs + 1] << BITS_PER_BYTE));
}

// (raw - centre) >> shift: an arithmetic shift (floor), as the contract writes it; gcc and
// clang shift signed values arithmetically (C++20 makes it the rule).
int16_t axis(uint16_t raw) {
  const int32_t centred = (int32_t)raw - (int32_t)XSTATE_AXIS_CENTRE;
  return (int16_t)(centred >> XSTATE_AXIS_SHIFT);
}

uint16_t dpadBits(uint8_t code) {
  switch (code) {
    case XSTATE_DPAD_N: return DS_BTN_DPAD_UP;
    case XSTATE_DPAD_NE: return DS_BTN_DPAD_UP | DS_BTN_DPAD_RIGHT;
    case XSTATE_DPAD_E: return DS_BTN_DPAD_RIGHT;
    case XSTATE_DPAD_SE: return DS_BTN_DPAD_DOWN | DS_BTN_DPAD_RIGHT;
    case XSTATE_DPAD_S: return DS_BTN_DPAD_DOWN;
    case XSTATE_DPAD_SW: return DS_BTN_DPAD_DOWN | DS_BTN_DPAD_LEFT;
    case XSTATE_DPAD_W: return DS_BTN_DPAD_LEFT;
    case XSTATE_DPAD_NW: return DS_BTN_DPAD_UP | DS_BTN_DPAD_LEFT;
    default: return 0;   // XSTATE_DPAD_NONE or an unknown code
  }
}

bool sameStatus(const ds_status_packet_t& a, const ds_status_packet_t& b) {
  return memcmp(&a, &b, sizeof a) == 0;
}

uint8_t espModeFor(uint8_t radioMode) {
  return radioMode == DS_RADIO_WIFI ? (uint8_t)DS_ESP_MODE_WIFI : (uint8_t)DS_ESP_MODE_BLE;
}

}  // namespace

ControlLink::ControlLink() : at_(atPayload_, sizeof atPayload_) { ds_deframer_init(&usbDeframer_); }

void ControlLink::begin(Debug& debug, uint32_t nowMs, uint32_t /*nowUs*/) {
  debug_ = &debug;
  frame_ = ControlFrame();
  status_ = LinkStatus();
  status_.usbControlEnabled = usbControlEnabled_;
  stats_ = LinkStats();
  atPhase_ = AtPhase::PING;
  ds_deframer_init(&usbDeframer_);
  shadowLen_ = 0;
  lineLen_ = 0;
  line_[0] = 0;
  lineOverflow_ = linePoison_ = false;
  shellHead_ = shellCount_ = 0;
  usbJunk_ = 0;
  usb_ = SourceFrame();
  radio_ = SourceFrame();
  sourceChanged_ = false;
  windowStartMs_ = nowMs;
  memset(windowFrames_, 0, sizeof windowFrames_);
  at_.reset();
  req_ = Request::NONE;
  reqOnWire_ = false;
  lastEspReplyMs_ = nowMs;
  pingSent_ = false;
  lastPingMs_ = nowMs;
  capsNow_ = false;
  capsPrev_ = CapsOutcome();
  dssConfirming_ = false;
  dssPendingMode_ = DS_ESP_MODE_UNKNOWN;
  dssPendingBoot_ = 0;
  lastCapsMs_ = lastPollMs_ = nowMs;
  dssInFlight_ = dssAcked_ = ds_status_packet_t();
  dssAckedValid_ = dssFailed_ = resendDss_ = false;
  lastDssMs_ = nowMs;
  offDssEver_ = false;
  lastOffDssMs_ = nowMs;
  beginWanted_ = beginEver_ = false;
  lastBeginMs_ = nowMs;
  lastBeginCode_ = XBOX_R4_RC_OK;
  telemetryPending_ = false;
  memset(telemetryFrame_, 0, sizeof telemetryFrame_);
  lastDstMs_ = nowMs;
  wifiWarned_ = false;
  xboxCrcErrors_ = 0;
  rumbleStrong_ = rumbleWeak_ = 0;
  rumbleMs_ = 0;
  rumbleSeq_ = 0;
  turretState_ = DS_STATE_OFF;
  usbStatusSent_ = ds_status_packet_t();
  usbStatusValid_ = usbStatusRetry_ = false;
  lastUsbStatusMs_ = lastUsbStatusTryMs_ = nowMs;
}

void ControlLink::update(uint32_t nowMs, uint32_t nowUs) {
  // 1. Everything that arrived. The core's RX rings hold SERIAL_RX_BUFFER_BYTES each, so
  //    these loops end.
  uint8_t chunk[DS_MAX_FRAME_LEN];
  size_t n;
  while ((n = hal::usbRead(chunk, sizeof chunk)) > 0) {
    for (size_t i = 0; i < n; i++) usbByte(chunk[i], nowMs);
  }
  while ((n = hal::linkRead(chunk, sizeof chunk)) > 0) {
    for (size_t i = 0; i < n; i++) at_.feed(chunk[i]);
  }

  // 2. The AT client. The reply deadline counts from the request's last byte on the wire.
  if (req_ != Request::NONE && !reqOnWire_ && hal::linkTxIdle()) {
    at_.extend(nowUs, kReplyUs);
    reqOnWire_ = true;
  }
  const AsyncAt::Result r = at_.check(nowUs);
  if (r != AsyncAt::PENDING && r != AsyncAt::IDLE) onResult(r, nowMs);
  if (atPhase_ != AtPhase::PING && (uint32_t)(nowMs - lastEspReplyMs_) >= LINK_TIMEOUT_MS) {
    stats_.espSilences++;
    log(LOG_ERROR, "link: ESP32 silent for LINK_TIMEOUT_MS: pinging");
    enterPing(nowMs);
  }
  refreshWifiAvailability();
  if (!at_.busy()) schedule(nowMs, nowUs);

  // 3. What the turret sees, and what the PC hears.
  refreshSources(nowMs);
  rateWindow(nowMs);
  sendUsbStatus(nowMs);
  refreshStats(nowMs);
}

// ---- USB ------------------------------------------------------------------------------------

// The deframer drops bytes only from the front of what it holds, one at a time, counting each
// in junk_bytes, and reports a frame only at the front. shadow_ mirrors the held bytes that are
// not yet reported, so the bytes dropped by a feed()/poll() are exactly the first
// (junk_bytes delta) bytes of shadow_: those are text (or residue) for the line splitter. A
// reported frame's bytes are the next frame_len bytes: they leave shadow_ unseen by it.
void ControlLink::usbByte(uint8_t b, uint32_t nowMs) {
  if (shadowLen_ < sizeof shadow_) shadow_[shadowLen_++] = b;   // held <= DS_MAX_FRAME_LEN + 1
  uint32_t junkBefore = usbDeframer_.junk_bytes;
  uint8_t type = ds_deframer_feed(&usbDeframer_, b);
  for (;;) {
    size_t dropped = (size_t)(usbDeframer_.junk_bytes - junkBefore);
    if (dropped > shadowLen_) dropped = shadowLen_;   // cannot happen; never read past shadow_
    for (size_t i = 0; i < dropped; i++) shellByte(shadow_[i]);
    memmove(shadow_, shadow_ + dropped, shadowLen_ - dropped);
    shadowLen_ -= dropped;
    if (type == DS_PKT_NONE) break;
    size_t len = ds_deframer_frame_len(&usbDeframer_);
    usbFrame(type, ds_deframer_frame(&usbDeframer_), len, nowMs);
    if (len > shadowLen_) len = shadowLen_;
    memmove(shadow_, shadow_ + len, shadowLen_ - len);
    shadowLen_ -= len;
    junkBefore = usbDeframer_.junk_bytes;
    type = ds_deframer_poll(&usbDeframer_);
  }
}

void ControlLink::shellByte(uint8_t c) {
  if (c == '\n') {
    if (linePoison_) {
      stats_.shellJunkLines++;
    } else if (lineOverflow_) {
      stats_.shellOverflows++;
    } else if (lineLen_ > 0) {
      if (shellCount_ < SHELL_QUEUE_LINES) {
        char* slot = shellQ_[(shellHead_ + shellCount_) % SHELL_QUEUE_LINES];
        memcpy(slot, line_, lineLen_);
        slot[lineLen_] = 0;
        shellCount_++;
      } else {
        stats_.shellQueueFull++;
      }
    }
    lineLen_ = 0;
    lineOverflow_ = linePoison_ = false;
    return;
  }
  if (c == '\r') return;
  const bool text = (c >= ' ' && c <= '~') || c == '\t';
  if (!text) {
    // Binary residue (a frame that failed its CRC, bridge noise): the whole line is suspect.
    if (!linePoison_) usbJunk_ += (uint32_t)lineLen_;
    linePoison_ = true;
    lineLen_ = 0;
    usbJunk_++;
    return;
  }
  if (linePoison_ || lineOverflow_) {
    usbJunk_++;
    return;
  }
  if (lineLen_ >= SHELL_LINE_MAX) {
    lineOverflow_ = true;
    usbJunk_ += (uint32_t)lineLen_ + 1u;
    lineLen_ = 0;
    return;
  }
  line_[lineLen_++] = (char)c;
}

void ControlLink::usbFrame(uint8_t type, const uint8_t* frame, size_t len, uint32_t nowMs) {
  if (type != DS_PKT_CONTROL) return;   // the PC sends only ControlPackets; others are ignored
  ds_control_packet_t p;
  if (!ds_control_decode(frame, len, &p)) return;   // the deframer already checked it
  if (!usbControlEnabled_) {
    stats_.usbIgnored++;
    return;
  }
  if (usb_.have) {   // the same USB session: count the seq values that never arrived
    const uint8_t gap = (uint8_t)(p.seq - usb_.f.seq - 1u);
    if (gap != 0 && gap <= INT8_MAX) stats_.seqGaps += gap;   // larger: a restarted sender
  }
  ControlFrame f;
  f.valid = true;
  f.connected = (p.flags & DS_CTRL_FLAG_CONNECTED) != 0;
  f.espPairing = (p.flags & DS_CTRL_FLAG_ESP_PAIRING) != 0;
  f.keyboard = (p.flags & DS_CTRL_FLAG_KEYBOARD) != 0;
  f.source = DS_LINK_SRC_USB;
  f.rx = p.rx;
  f.ry = p.ry;
  f.rt = p.rt;
  f.buttons = p.buttons;
  f.seq = p.seq;
  f.rxMs = nowMs;
  // A ControlPacket has no left stick, left trigger or battery: lx/ly/lt stay 0, battery unknown.
  f.padState = f.connected ? (uint8_t)DS_PAD_STATE_CONNECTED : (uint8_t)DS_PAD_STATE_SCANNING;
  usb_.have = true;
  usb_.rxMs = nowMs;
  usb_.f = f;
  stats_.frames++;
  stats_.usbFrames++;
  windowFrames_[DS_LINK_SRC_USB]++;
  status_.lastFrameMs = nowMs;
}

bool ControlLink::takeShellLine(char* out, size_t cap) {
  if (shellCount_ == 0 || !out || cap == 0) return false;
  const char* slot = shellQ_[shellHead_];
  size_t len = strlen(slot);
  if (len > cap - 1) len = cap - 1;
  memcpy(out, slot, len);
  out[len] = 0;
  shellHead_ = (shellHead_ + 1) % SHELL_QUEUE_LINES;
  shellCount_--;
  stats_.shellLines++;
  return true;
}

void ControlLink::setUsbControlEnabled(bool on) {
  usbControlEnabled_ = on;
  status_.usbControlEnabled = on;
  if (!on) usb_ = SourceFrame();
}

// ---- AT client ------------------------------------------------------------------------------

bool ControlLink::issue(Request kind, const char* prompt, const char* line, size_t len,
                        uint32_t nowUs) {
  const size_t freeBytes = hal::linkTxFree();
  const size_t queued = freeBytes < LINK_TX_BUFFER_BYTES ? LINK_TX_BUFFER_BYTES - freeBytes : 0;
  if (!hal::linkWrite(reinterpret_cast<const uint8_t*>(line), len)) return false;   // next tick
  at_.start(prompt, nowUs, kReplyUs + (uint32_t)(queued + len) * kByteUs);
  req_ = kind;
  reqOnWire_ = false;
  return true;
}

void ControlLink::schedule(uint32_t nowMs, uint32_t nowUs) {
  switch (atPhase_) {
    case AtPhase::PING:
      if (!pingSent_ || (uint32_t)(nowMs - lastPingMs_) >= LINK_ESP_PING_MS) {
        if (issue(Request::PING, nullptr, kPingLine, sizeof kPingLine - 1, nowUs)) {
          pingSent_ = true;
          lastPingMs_ = nowMs;
        }
      }
      break;
    case AtPhase::CAPS:
      if (capsNow_ || (uint32_t)(nowMs - lastCapsMs_) >= LINK_ESP_PING_MS) {
        if (issue(Request::CAPS, DS_AT_DSCAPS, kCapsLine, sizeof kCapsLine - 1, nowUs)) {
          capsNow_ = false;
          lastCapsMs_ = nowMs;
        }
      }
      break;
    case AtPhase::RUN_DS:
      if (dssDue(nowMs))
        sendDss(nowMs, nowUs);
      else if (pollDue(nowMs))
        sendPoll(nowMs, nowUs);
      else if (dstDue(nowMs))
        sendDst(nowMs, nowUs);
      break;
    case AtPhase::RUN_LEGACY:
      if (beginDue(nowMs))
        sendBegin(nowMs, nowUs);
      else if (pollDue(nowMs))
        sendPoll(nowMs, nowUs);
      break;
  }
}

void ControlLink::onResult(AsyncAt::Result r, uint32_t nowMs) {
  const Request kind = req_;
  req_ = Request::NONE;
  reqOnWire_ = false;
  stats_.polls++;
  if (r == AsyncAt::TIMEOUT) {
    stats_.pollTimeouts++;
  } else {
    lastEspReplyMs_ = nowMs;   // any final, even ERROR, proves the ESP32 is alive
    status_.lastEspReplyMs = nowMs;
  }
  switch (kind) {
    case Request::PING:
      if (r != AsyncAt::TIMEOUT && atPhase_ == AtPhase::PING) enterCaps(nowMs);
      break;
    case Request::CAPS: onCaps(r, nowMs); break;
    case Request::DSS: onDss(r, nowMs); break;
    case Request::STATE: onState(r, nowMs); break;
    case Request::DST: onDst(r); break;
    case Request::BEGIN: onBegin(r); break;
    case Request::NONE: break;
  }
}

// AT text replies carry no CRC, and one garbled byte here would pick the wrong firmware for the
// whole session (a "dsProto 11" means legacy: no WiFi). So the decision needs the same outcome
// twice in a row; a different second answer starts the comparison again.
void ControlLink::onCaps(AsyncAt::Result r, uint32_t nowMs) {
  if (atPhase_ != AtPhase::CAPS) return;
  uint32_t v[DS_AT_DSCAPS_REPLY_COUNT] = {};
  CapsOutcome outcome;
  if (r == AsyncAt::OK && parseU8Fields(at_.data(), at_.length(), v, DS_AT_DSCAPS_REPLY_COUNT)) {
    outcome.kind = CapsOutcome::ANSWER;
    outcome.proto = (uint8_t)v[DS_AT_DSCAPS_REPLY_DS_PROTO];
    outcome.caps = (uint8_t)v[DS_AT_DSCAPS_REPLY_CAP_BITS];
  } else if (r == AsyncAt::ERR_BARE) {
    outcome.kind = CapsOutcome::BARE_ERROR;   // stock / pre-D-Sentry: unknown command
  } else {
    if (r != AsyncAt::TIMEOUT) stats_.pollErrors++;   // retried every LINK_ESP_PING_MS
    capsPrev_ = CapsOutcome();
    return;
  }
  const bool confirmed = capsPrev_.kind == outcome.kind && capsPrev_.proto == outcome.proto &&
                         capsPrev_.caps == outcome.caps;
  if (!confirmed) {
    capsPrev_ = outcome;
    capsNow_ = true;   // ask again at once
    return;
  }
  capsPrev_ = CapsOutcome();
  status_.espCapsKnown = true;
  if (outcome.kind == CapsOutcome::ANSWER) {
    status_.espProto = outcome.proto;
    status_.espCaps = outcome.caps;
    if (status_.espProto == DS_PROTO_VERSION) {
      status_.espLegacy = false;
      Debug::Line l;
      l.add("link: ESP32 D-Sentry firmware, dsProto ").addU(status_.espProto).add(" caps 0x")
          .addHex(status_.espCaps, sizeof(uint8_t) * BITS_PER_BYTE / HEX_DIGIT_BITS);
      log(LOG_STATE, l.c_str());
      enterRunDs(nowMs);
    } else {
      // Another protocol version: its AT+DSS / AT+DST may have another shape. Fall back to
      // the commands every fork answers the same way (BT only).
      status_.espLegacy = true;
      Debug::Line l;
      l.add("link: ESP32 dsProto ").addU(status_.espProto).add(", this build speaks ")
          .addU(DS_PROTO_VERSION).add(": treated as legacy (BT only)");
      log(LOG_ERROR, l.c_str());
      enterRunLegacy(nowMs);
    }
  } else {
    status_.espLegacy = true;
    status_.espProto = 0;
    status_.espCaps = 0;
    log(LOG_STATE, "link: ESP32 legacy firmware (AT+DSCAPS? -> ERROR): BT only");
    enterRunLegacy(nowMs);
  }
}

// A reply that would change what ControlLink believes about the ESP32 (its radio, its boot)
// is asked again at once and adopted only when the second reply says the same: AT text has no
// CRC, and a single garbled digit ("+DSS: 10,92,0") must not close the radio path or count a
// reboot. Values outside the enums are never adopted.
void ControlLink::onDss(AsyncAt::Result r, uint32_t nowMs) {
  if (atPhase_ != AtPhase::RUN_DS) return;
  uint32_t v[DS_AT_DSS_REPLY_COUNT];
  const bool parsed = r == AsyncAt::OK && parseU8Fields(at_.data(), at_.length(), v, DS_AT_DSS_REPLY_COUNT);
  const uint8_t mode = parsed ? (uint8_t)v[DS_AT_DSS_REPLY_ESP_MODE] : (uint8_t)DS_ESP_MODE_UNKNOWN;
  const uint8_t boot = parsed ? (uint8_t)v[DS_AT_DSS_REPLY_BOOT_ID] : (uint8_t)0;
  const uint8_t wifi = parsed ? (uint8_t)v[DS_AT_DSS_REPLY_WIFI_STATUS] : (uint8_t)DS_WIFI_STATUS_COUNT;
  const bool valid = parsed && boot != 0 && wifi < DS_WIFI_STATUS_COUNT &&
                     (mode == DS_ESP_MODE_BLE || mode == DS_ESP_MODE_WIFI || mode == DS_ESP_MODE_NONE ||
                      mode == DS_ESP_MODE_RESTARTING);
  if (!valid) {
    if (r != AsyncAt::TIMEOUT) stats_.pollErrors++;
    dssFailed_ = true;   // dssDue() retries after one poll period
    return;
  }
  dssFailed_ = false;
  dssAcked_ = dssInFlight_;
  dssAckedValid_ = true;
  const bool known = status_.espMode != DS_ESP_MODE_UNKNOWN;
  if (known && (mode != status_.espMode || boot != status_.bootId)) {
    const bool confirmed = dssConfirming_ && mode == dssPendingMode_ && boot == dssPendingBoot_;
    if (!confirmed) {
      dssConfirming_ = true;
      dssPendingMode_ = mode;
      dssPendingBoot_ = boot;
      resendDss_ = true;   // ask again at once
      return;
    }
  }
  dssConfirming_ = false;
  if (boot != status_.bootId) {
    if (status_.bootId != 0) {
      stats_.espRestarts++;
      resendDss_ = true;   // architecture.md 4.2: a changed bootId re-sends DSS
      Debug::Line l;
      l.add("link: ESP32 rebooted (bootId ").addU(status_.bootId).add(" -> ").addU(boot).add(')');
      log(LOG_STATE, l.c_str());
    }
    status_.bootId = boot;
  }
  if (mode != status_.espMode) {
    radio_ = SourceFrame();   // frames of the previous radio are void
    Debug::Line l;
    l.add("link: ESP32 radio ").add(ds_esp_mode_name(mode));
    log(LOG_STATE, l.c_str());
  }
  status_.espMode = mode;
  status_.wifiStatus = (uint8_t)v[DS_AT_DSS_REPLY_WIFI_STATUS];
  (void)nowMs;
}

void ControlLink::onState(AsyncAt::Result r, uint32_t nowMs) {
  if (atPhase_ != AtPhase::RUN_DS && atPhase_ != AtPhase::RUN_LEGACY) return;
  if (r != AsyncAt::OK) {
    if (r != AsyncAt::TIMEOUT) stats_.pollErrors++;
    return;
  }
  ControlFrame f;
  uint8_t conn = 0;
  if (!decodeXboxState(at_.data(), at_.length(), f, conn)) {
    if (at_.length() == XBOX_R4_STATE_LEN)
      xboxCrcErrors_++;
    else
      stats_.pollErrors++;
    return;
  }
  if (conn == XBOX_R4_OFF && status_.wantMode == DS_RADIO_BLE) {
    // The BLE client is not running. Legacy: AT+XBOXBEGIN. D-Sentry: re-send AT+DSS, which
    // starts the wanted radio. Both at most every LINK_BEGIN_RETRY_MS.
    if (atPhase_ == AtPhase::RUN_LEGACY) {
      beginWanted_ = true;
    } else if (!offDssEver_ || (uint32_t)(nowMs - lastOffDssMs_) >= LINK_BEGIN_RETRY_MS) {
      offDssEver_ = true;
      lastOffDssMs_ = nowMs;
      resendDss_ = true;
    }
  }
  if (!radioPathOpen()) return;   // not the radio TurretState asked for
  f.source = radioSource();
  f.rxMs = nowMs;
  radio_.have = true;
  radio_.rxMs = nowMs;
  radio_.f = f;
  stats_.frames++;
  stats_.radioFrames++;
  windowFrames_[f.source]++;
  status_.lastFrameMs = nowMs;
}

void ControlLink::onDst(AsyncAt::Result r) {
  uint32_t v[DS_AT_DST_REPLY_COUNT];
  if (r != AsyncAt::OK || !parseU8Fields(at_.data(), at_.length(), v, DS_AT_DST_REPLY_COUNT)) {
    if (r != AsyncAt::TIMEOUT) stats_.pollErrors++;
    return;
  }
  if (v[DS_AT_DST_REPLY_SENT] == DS_DST_SENT)
    stats_.telemetrySent++;
  else
    stats_.telemetryNotSent++;
}

void ControlLink::onBegin(AsyncAt::Result r) {
  uint32_t value = 0;   // "+XBOXBEGIN: <code>": one field
  if (r != AsyncAt::OK || !parseU8Fields(at_.data(), at_.length(), &value, 1)) {
    if (r != AsyncAt::TIMEOUT) stats_.pollErrors++;
    return;
  }
  const uint8_t code = (uint8_t)value;
  if (code != lastBeginCode_ && code != XBOX_R4_RC_OK) {
    Debug::Line l;
    l.add("link: AT+XBOXBEGIN refused, code ").addU(code);
    log(LOG_ERROR, l.c_str());
  }
  lastBeginCode_ = code;
}

void ControlLink::enterPing(uint32_t nowMs) {
  atPhase_ = AtPhase::PING;
  pingSent_ = false;   // ping at once
  lastPingMs_ = nowMs;
  status_.espAlive = false;
  status_.espCapsKnown = false;
  status_.espMode = DS_ESP_MODE_UNKNOWN;
  status_.wifiStatus = DS_WIFI_STATUS_OFF;
  radio_ = SourceFrame();
  telemetryPending_ = false;
}

void ControlLink::enterCaps(uint32_t nowMs) {
  atPhase_ = AtPhase::CAPS;
  capsNow_ = true;
  capsPrev_ = CapsOutcome();
  lastCapsMs_ = nowMs;
  lastEspReplyMs_ = nowMs;
  log(LOG_STATE, "link: ESP32 answered, probing AT+DSCAPS?");
}

void ControlLink::enterRunDs(uint32_t nowMs) {
  atPhase_ = AtPhase::RUN_DS;
  dssAckedValid_ = false;   // the first request is AT+DSS
  dssFailed_ = false;
  dssConfirming_ = false;
  resendDss_ = false;
  lastDssMs_ = nowMs;
  lastPollMs_ = nowMs - kPollMs;   // then a poll at once
  lastDstMs_ = nowMs - kDstMs;
  status_.espMode = DS_ESP_MODE_UNKNOWN;
}

void ControlLink::enterRunLegacy(uint32_t nowMs) {
  atPhase_ = AtPhase::RUN_LEGACY;
  status_.espMode = DS_ESP_MODE_UNKNOWN;
  beginWanted_ = false;
  beginEver_ = false;
  lastBeginCode_ = XBOX_R4_RC_OK;
  lastPollMs_ = nowMs - kPollMs;
  telemetryPending_ = false;
}

ds_status_packet_t ControlLink::currentStatus() const {
  ds_status_packet_t s;
  ds_status_init(&s);
  s.wantMode = status_.wantMode;
  s.rumbleStrong = rumbleStrong_;
  s.rumbleWeak = rumbleWeak_;
  s.rumbleMs = rumbleMs_;
  s.state = turretState_;
  s.rumbleSeq = rumbleSeq_;
  return s;
}

bool ControlLink::dssDue(uint32_t nowMs) const {
  const ds_status_packet_t cur = currentStatus();
  bool due = resendDss_ || !dssAckedValid_ || !sameStatus(cur, dssAcked_) ||
             (uint32_t)(nowMs - lastDssMs_) >= STATUS_HEARTBEAT_MS;
  if (due && dssFailed_ && (uint32_t)(nowMs - lastDssMs_) < kPollMs) due = false;
  return due;
}

bool ControlLink::pollDue(uint32_t nowMs) const { return (uint32_t)(nowMs - lastPollMs_) >= kPollMs; }

bool ControlLink::dstDue(uint32_t nowMs) const {
  return telemetryPending_ && wantsWifiTelemetry() && (uint32_t)(nowMs - lastDstMs_) >= kDstMs;
}

bool ControlLink::beginDue(uint32_t nowMs) const {
  return beginWanted_ && status_.wantMode == DS_RADIO_BLE &&
         (!beginEver_ || (uint32_t)(nowMs - lastBeginMs_) >= LINK_BEGIN_RETRY_MS);
}

void ControlLink::sendDss(uint32_t nowMs, uint32_t nowUs) {
  const ds_status_packet_t cur = currentStatus();
  size_t n = 0;
  appendStr(reqLine_, n, kDssHead);
  // Each argument at its protocol_spec.py index, so the order cannot drift.
  uint32_t args[DS_AT_DSS_ARGC] = {};
  args[DS_AT_DSS_ARG_WANT_MODE] = cur.wantMode;
  args[DS_AT_DSS_ARG_RUMBLE_SEQ] = cur.rumbleSeq;
  args[DS_AT_DSS_ARG_STRONG] = cur.rumbleStrong;
  args[DS_AT_DSS_ARG_WEAK] = cur.rumbleWeak;
  args[DS_AT_DSS_ARG_RUMBLE_MS] = cur.rumbleMs;
  args[DS_AT_DSS_ARG_STATE] = cur.state;
  for (size_t i = 0; i < DS_AT_DSS_ARGC; i++) {
    if (i) reqLine_[n++] = ',';
    appendU(reqLine_, n, args[i]);
  }
  appendStr(reqLine_, n, kLineEnd);
  if (!issue(Request::DSS, DS_AT_DSS, reqLine_, n, nowUs)) return;
  dssInFlight_ = cur;
  lastDssMs_ = nowMs;
  resendDss_ = false;
  stats_.dssSent++;
}

void ControlLink::sendPoll(uint32_t nowMs, uint32_t nowUs) {
  if (!issue(Request::STATE, XBOX_R4_AT_STATE, kStateLine, sizeof kStateLine - 1, nowUs)) return;
  // Keep a 1000/CONTROL_PACKET_HZ grid; after a longer gap (an AT+DST line) start a new one.
  lastPollMs_ += kPollMs;
  if ((uint32_t)(nowMs - lastPollMs_) >= kPollMs) lastPollMs_ = nowMs;
}

void ControlLink::sendDst(uint32_t nowMs, uint32_t nowUs) {
  size_t n = 0;
  appendStr(reqLine_, n, kDstHead);
  n += ds_hex_encode(telemetryFrame_, sizeof telemetryFrame_, reqLine_ + n);
  appendStr(reqLine_, n, kLineEnd);
  if (!issue(Request::DST, DS_AT_DST, reqLine_, n, nowUs)) return;
  telemetryPending_ = false;
  lastDstMs_ += kDstMs;
  if ((uint32_t)(nowMs - lastDstMs_) >= kDstMs) lastDstMs_ = nowMs;
}

void ControlLink::sendBegin(uint32_t nowMs, uint32_t nowUs) {
  if (!issue(Request::BEGIN, XBOX_R4_AT_BEGIN, kBeginLine, sizeof kBeginLine - 1, nowUs)) return;
  beginWanted_ = false;
  beginEver_ = true;
  lastBeginMs_ = nowMs;
  stats_.beginsSent++;
}

bool ControlLink::radioPathOpen() const {
  if (atPhase_ == AtPhase::RUN_DS)
    return !status_.wifiUnavailable && status_.espMode == espModeFor(status_.wantMode);
  if (atPhase_ == AtPhase::RUN_LEGACY) return status_.wantMode == DS_RADIO_BLE;
  return false;
}

uint8_t ControlLink::radioSource() const {
  return status_.espMode == DS_ESP_MODE_WIFI ? (uint8_t)DS_LINK_SRC_WIFI : (uint8_t)DS_LINK_SRC_BLE;
}

// ---- inputs ---------------------------------------------------------------------------------

bool ControlLink::takeSourceChanged() {
  bool was = sourceChanged_;
  sourceChanged_ = false;
  return was;
}

void ControlLink::setWantMode(uint8_t radioMode) {
  if (radioMode >= DS_RADIO_COUNT || radioMode == status_.wantMode) return;
  status_.wantMode = radioMode;
  radio_ = SourceFrame();   // the old radio's frames are void from now on
  beginWanted_ = false;
  wifiWarned_ = false;
  Debug::Line l;
  l.add("link: radio wanted ").add(ds_radio_mode_name(radioMode));
  log(LOG_STATE, l.c_str());
}

void ControlLink::setTurretState(uint8_t dsState) { turretState_ = dsState; }

void ControlLink::requestRumble(uint8_t strong, uint8_t weak, uint16_t ms) {
  rumbleStrong_ = strong;
  rumbleWeak_ = weak;
  rumbleMs_ = ms;
  rumbleSeq_++;
}

bool ControlLink::offerTelemetry(const uint8_t* frame, size_t len) {
  if (!frame || len != DS_TELEMETRY_FRAME_LEN || !ds_frame_check(frame, len, DS_PKT_TELEMETRY))
    return false;
  stats_.telemetryOffered++;
  if (!wantsWifiTelemetry()) return false;
  memcpy(telemetryFrame_, frame, len);
  telemetryPending_ = true;
  return true;
}

bool ControlLink::wantsWifiTelemetry() const {
  return atPhase_ == AtPhase::RUN_DS && status_.espMode == DS_ESP_MODE_WIFI &&
         (status_.espCaps & DS_CAP_TELEMETRY) != 0;
}

// ---- outputs --------------------------------------------------------------------------------

void ControlLink::refreshWifiAvailability() {
  const bool unavailable =
      status_.wantMode == DS_RADIO_WIFI && status_.espCapsKnown &&
      (atPhase_ == AtPhase::RUN_LEGACY || (status_.espCaps & DS_CAP_WIFI) == 0);
  if (unavailable && !wifiWarned_) {
    wifiWarned_ = true;
    log(LOG_ERROR, status_.espLegacy
                       ? "link: WiFi wanted, but the ESP32 runs legacy firmware (BT only): no link"
                       : "link: WiFi wanted, but the ESP32 firmware has no WiFi: no link");
  }
  status_.wifiUnavailable = unavailable;
}

void ControlLink::refreshSources(uint32_t nowMs) {
  if (usb_.have && (uint32_t)(nowMs - usb_.rxMs) >= LINK_TIMEOUT_MS) usb_.have = false;
  if (radio_.have && (uint32_t)(nowMs - radio_.rxMs) >= LINK_TIMEOUT_MS) radio_.have = false;
  const bool usbLive = usb_.have && usbControlEnabled_;
  const bool radioLive = radio_.have && radioPathOpen();
  const uint8_t active = usbLive ? (uint8_t)DS_LINK_SRC_USB
                                 : radioLive ? radio_.f.source : (uint8_t)DS_LINK_SRC_NONE;
  const uint8_t was = status_.activeSource;
  if (active != was && active != DS_LINK_SRC_NONE && was != DS_LINK_SRC_NONE &&
      status_.connected) {
    sourceChanged_ = true;
    stats_.sourceChanges++;
    Debug::Line l;
    l.add("link: source ").add(ds_link_source_name(was)).add(" -> ")
        .add(ds_link_source_name(active)).add(" while connected: link loss");
    log(LOG_STATE, l.c_str());
  }
  status_.activeSource = active;
  if (active == DS_LINK_SRC_USB)
    frame_ = usb_.f;
  else if (active != DS_LINK_SRC_NONE)
    frame_ = radio_.f;
  else
    frame_ = ControlFrame();
  status_.connected = frame_.connected;
}

void ControlLink::rateWindow(uint32_t nowMs) {
  const uint32_t elapsed = nowMs - windowStartMs_;
  if (elapsed < kRateWindowMs) return;
  const uint32_t hz = (uint32_t)((uint64_t)windowFrames_[status_.activeSource] * MS_PER_S / elapsed);
  stats_.ctrlRxHz = hz > UINT16_MAX ? (uint16_t)UINT16_MAX : (uint16_t)hz;
  memset(windowFrames_, 0, sizeof windowFrames_);
  windowStartMs_ = nowMs;
}

void ControlLink::sendUsbStatus(uint32_t nowMs) {
  const ds_status_packet_t cur = currentStatus();
  const bool due = !usbStatusValid_ || !sameStatus(cur, usbStatusSent_) ||
                   (uint32_t)(nowMs - lastUsbStatusMs_) >= STATUS_HEARTBEAT_MS;
  if (!due) return;
  // A full USB queue: try again after one poll period, not on every loop (each refusal
  // counts as a dropped message).
  if (usbStatusRetry_ && (uint32_t)(nowMs - lastUsbStatusTryMs_) < kPollMs) return;
  uint8_t f[DS_STATUS_FRAME_LEN];
  const size_t n = ds_status_encode(&cur, f);
  lastUsbStatusTryMs_ = nowMs;
  if (!hal::usbWrite(f, n)) {
    usbStatusRetry_ = true;
    return;
  }
  usbStatusRetry_ = false;
  usbStatusSent_ = cur;
  usbStatusValid_ = true;
  lastUsbStatusMs_ = nowMs;
  stats_.statusSent++;
}

void ControlLink::refreshStats(uint32_t nowMs) {
  status_.espAlive = atPhase_ != AtPhase::PING && (uint32_t)(nowMs - lastEspReplyMs_) < LINK_TIMEOUT_MS;
  stats_.crcErrors = usbDeframer_.crc_errors + xboxCrcErrors_;
  stats_.lenErrors = usbDeframer_.len_errors;
  stats_.staleReplies = at_.staleReplies();
  stats_.junkBytes = at_.junkLines() + usbJunk_;
}

void ControlLink::log(uint8_t level, const char* text) {
  if (debug_) debug_->log(level, text);
}

// ---- conversions ----------------------------------------------------------------------------

bool ControlLink::decodeXboxState(const uint8_t* payload, size_t len, ControlFrame& out,
                                  uint8_t& connState) {
  xbox_r4_state_t st;
  if (!payload || !xbox_r4_decode(payload, len, &st)) return false;
  out = ControlFrame();
  out.valid = true;
  connState = payload[XSTATE_OFS_CONN];
  out.connected = connState == XBOX_R4_CONNECTED;
  out.espPairing = connState == XBOX_R4_WAITING_FIRST_REPORT || connState == XBOX_R4_FOUND ||
                   connState == XBOX_R4_SCANNING;
  out.rx = axis(le16(payload, XSTATE_OFS_JOY_RH));
  out.ry = axis(le16(payload, XSTATE_OFS_JOY_RV));
  out.rt = le16(payload, XSTATE_OFS_TRIG_RT);
  out.lx = axis(le16(payload, XSTATE_OFS_JOY_LH));
  out.ly = axis(le16(payload, XSTATE_OFS_JOY_LV));
  out.lt = le16(payload, XSTATE_OFS_TRIG_LT);
  const uint8_t b0 = payload[XSTATE_OFS_BTN0];
  const uint8_t b1 = payload[XSTATE_OFS_BTN1];
  const uint8_t b2 = payload[XSTATE_OFS_BTN2];
  uint16_t buttons = dpadBits(payload[XSTATE_OFS_DPAD]);
  if (b0 & XSTATE_BTN0_A) buttons |= DS_BTN_A;
  if (b0 & XSTATE_BTN0_B) buttons |= DS_BTN_B;
  if (b0 & XSTATE_BTN0_X) buttons |= DS_BTN_X;
  if (b0 & XSTATE_BTN0_Y) buttons |= DS_BTN_Y;
  if (b0 & XSTATE_BTN0_LB) buttons |= DS_BTN_LB;
  if (b0 & XSTATE_BTN0_RB) buttons |= DS_BTN_RB;
  if (b1 & XSTATE_BTN1_MENU) buttons |= DS_BTN_START;
  if (b1 & XSTATE_BTN1_VIEW) buttons |= DS_BTN_BACK;
  if (b1 & XSTATE_BTN1_LS) buttons |= DS_BTN_LS_CLICK;
  if (b1 & XSTATE_BTN1_RS) buttons |= DS_BTN_RS_CLICK;
  if (b1 & XSTATE_BTN1_XBOX) buttons |= DS_BTN_XBOX;
  if (b2 & XSTATE_BTN2_SHARE) buttons |= DS_BTN_SHARE;
  out.buttons = buttons;
  out.seq = payload[XSTATE_OFS_SEQ];
  out.battery = payload[XSTATE_OFS_BATTERY];
  out.padState = connState;
  return true;
}

const char* ControlLink::phaseName(AtPhase p) {
  switch (p) {
    case AtPhase::PING: return "ping";
    case AtPhase::CAPS: return "caps";
    case AtPhase::RUN_DS: return "ds";
    case AtPhase::RUN_LEGACY: return "legacy";
  }
  return "?";
}

const char* ControlLink::requestName(Request r) {
  switch (r) {
    case Request::NONE: return "none";
    case Request::PING: return "AT";
    case Request::CAPS: return "AT+DSCAPS?";
    case Request::DSS: return "AT+DSS";
    case Request::STATE: return "AT+XBOXSTATE?";
    case Request::DST: return "AT+DST";
    case Request::BEGIN: return "AT+XBOXBEGIN";
  }
  return "?";
}

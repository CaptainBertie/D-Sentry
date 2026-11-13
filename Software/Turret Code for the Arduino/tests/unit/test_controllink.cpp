// test_controllink.cpp -- ControlLink (docs/architecture.md 4.2, 4.3) against a scripted ESP32.
//
// FakeEsp answers the AT link the way the protocol says (protocol_spec.py AT_COMMANDS, the
// 20-byte XBOXSTATE of xbox_r4_protocol.h), behind a byte-accurate wire model: every byte the
// RA4M1 writes reaches the ESP32 86.8 us after the previous one (115200 8N1), the ESP32 answers
// a complete line after its AT-task latency, and the reply comes back byte by byte at the same
// rate. ESP32 restarts, legacy firmware, silence, CRC errors and late replies are scripted.
// tests/chat/ runs the same ControlLink against the REAL chAT server.
//
// Key safety property (tests radio_link_loss_is_seen_within_link_timeout and
// usb_link_loss_is_seen_within_link_timeout): once the active source stops delivering valid
// frames, frame().connected is false no later than LINK_TIMEOUT_MS after the last one, and the
// frame published from then on is neutral (no stick, trigger or buttons).
//
// SPDX-License-Identifier: MIT
#include <deque>
#include <functional>
#include <random>
#include <string>
#include <vector>

#include "ControlLink.h"
#include "Debug.h"
#include "hal.h"
#include "hal_host.h"
#include "testkit.h"
#include "../xbox_r4_protocol.h"   // the sketch's symlink, found through -I src

namespace {

const double kByteUs = (double)UART_BITS_PER_BYTE * (double)US_PER_S / (double)LINK_BAUD;
constexpr uint32_t kPollMs = MS_PER_S / CONTROL_PACKET_HZ;

uint8_t espModeFor(uint8_t radio) { return radio == DS_RADIO_WIFI ? DS_ESP_MODE_WIFI : DS_ESP_MODE_BLE; }

std::vector<std::string> split(const std::string& s, char sep) {
  std::vector<std::string> out;
  std::string cur;
  for (char c : s) {
    if (c == sep) {
      out.push_back(cur);
      cur.clear();
    } else {
      cur += c;
    }
  }
  out.push_back(cur);
  return out;
}

bool startsWith(const std::string& s, const std::string& p) { return s.compare(0, p.size(), p) == 0; }

// ---- the scripted ESP32 ----------------------------------------------------------------------
struct FakeEsp {
  enum class Fw { DS, LEGACY };
  Fw fw = Fw::DS;
  bool powered = true;            // false: hears nothing, says nothing
  double processUs = 1000;        // AT task latency after a complete line
  // per-request reply override: return true and set reply (empty = no answer at all)
  std::function<bool(const std::string& req, std::string& reply)> hook;
  std::function<double(const std::string& req)> delayFor;   // extra latency per request
  // D-Sentry state
  uint8_t proto = DS_PROTO_VERSION;
  uint8_t caps = DS_CAP_ALL;
  uint8_t espMode = DS_ESP_MODE_NONE;
  uint8_t bootId = 0x5C;
  bool restartPending = false;
  double restartAtUs = 0;
  double bootSilenceUs = 300000;
  uint8_t pendingMode = DS_ESP_MODE_NONE;
  double poweredAtUs = 0;
  int lastRumbleSeq = -1;
  int rumbles = 0;
  std::vector<uint8_t> lastDst;
  // the pad
  xbox_r4_state_t st{};
  bool badCrc = false;
  bool beginConnects = true;
  // what was heard
  std::vector<std::string> wireLines;   // every complete request line on the wire
  std::vector<double> wireTimes;
  // wire model
  double inLast = 0, outLast = 0;
  std::deque<std::pair<double, uint8_t>> in, out;
  std::deque<std::pair<double, std::string>> todo;
  std::string line;

  FakeEsp() {
    xbox_r4_neutral_report(st.report);
    st.state = XBOX_R4_CONNECTED;
    st.battery = 80;
  }

  void powerOff() {
    powered = false;
    in.clear();
    out.clear();
    todo.clear();
    line.clear();
  }

  std::string xboxReply() {
    xbox_r4_state_t s = st;
    if (fw == Fw::DS && (espMode == DS_ESP_MODE_NONE || espMode == DS_ESP_MODE_RESTARTING)) {
      xbox_r4_neutral_report(s.report);
      s.state = XBOX_R4_OFF;
    }
    uint8_t enc[XBOX_R4_STATE_LEN];
    xbox_r4_encode(&s, enc);
    if (badCrc) enc[XBOX_R4_STATE_LEN - 1] ^= 0x5A;
    return "+XBOXSTATE: 20|" + std::string((const char*)enc, sizeof enc) + "\r\nOK\r\n";
  }

  std::string handle(const std::string& req, double now) {
    std::string reply;
    if (hook && hook(req, reply)) return reply;
    if (req == "AT") return "OK\r\n";
    if (req == "AT+XBOXSTATE?") return xboxReply();
    if (req == "AT+XBOXBEGIN") {
      if (st.state == XBOX_R4_OFF) st.state = beginConnects ? XBOX_R4_CONNECTED : XBOX_R4_SCANNING;
      return "+XBOXBEGIN: 0\r\nOK\r\n";
    }
    if (fw == Fw::LEGACY) return "ERROR\r\n";
    if (req == "AT+DSCAPS?")
      return "+DSCAPS: " + std::to_string(proto) + "," + std::to_string(caps) + "\r\nOK\r\n";
    if (startsWith(req, "AT+DSS=")) {
      std::vector<std::string> a = split(req.substr(7), ',');
      if (a.size() != DS_AT_DSS_ARGC) return "ERROR\r\n";
      const uint8_t want = espModeFor((uint8_t)std::stoi(a[DS_AT_DSS_ARG_WANT_MODE]));
      const int seq = std::stoi(a[DS_AT_DSS_ARG_RUMBLE_SEQ]);
      if (seq != lastRumbleSeq) {
        if (lastRumbleSeq >= 0) rumbles++;
        lastRumbleSeq = seq;
      }
      uint8_t replyMode = espMode;
      if (espMode == DS_ESP_MODE_NONE) {
        espMode = replyMode = want;
      } else if (espMode != want) {
        replyMode = DS_ESP_MODE_RESTARTING;
        restartPending = true;
        restartAtUs = now + ESP_RESTART_DELAY_MS * US_PER_MS;
        pendingMode = want;
      }
      const int wifi = espMode == DS_ESP_MODE_WIFI ? DS_WIFI_STATUS_RELAY_HEARD : DS_WIFI_STATUS_OFF;
      return "+DSS: " + std::to_string(replyMode) + "," + std::to_string(bootId) + "," +
             std::to_string(wifi) + "\r\nOK\r\n";
    }
    if (startsWith(req, "AT+DST=")) {
      const std::string hex = req.substr(7);
      std::vector<uint8_t> f(hex.size() / 2 + 1);
      const size_t n = ds_hex_decode(hex.data(), hex.size(), f.data(), f.size());
      f.resize(n);
      const bool ok = n == DS_TELEMETRY_FRAME_LEN && ds_frame_check(f.data(), n, DS_PKT_TELEMETRY) &&
                      espMode == DS_ESP_MODE_WIFI;
      if (ok) lastDst = f;
      return std::string("+DST: ") + (ok ? "1" : "0") + "\r\nOK\r\n";
    }
    return "ERROR\r\n";
  }

  void pump(double now) {
    // A scheduled restart: the radio switch of AT+DSS (architecture.md 4.2).
    if (restartPending && now >= restartAtUs) {
      restartPending = false;
      powerOff();
      poweredAtUs = now + bootSilenceUs;
      espMode = pendingMode;   // RTC_NOINIT: boots straight into the wanted radio
      bootId++;
    }
    if (!powered && poweredAtUs > 0 && now >= poweredAtUs) {
      powered = true;
      poweredAtUs = 0;
      inLast = outLast = now;
    }
    for (unsigned char c : hal_host::linkTakeTx()) {
      inLast = std::max(now, inLast) + kByteUs;
      in.push_back({inLast, c});
    }
    while (!in.empty() && in.front().first <= now) {
      const uint8_t c = in.front().second;
      const double t = in.front().first;
      in.pop_front();
      if (c == '\n') {
        std::string req = line;
        if (!req.empty() && req.back() == '\r') req.pop_back();
        wireLines.push_back(req);
        wireTimes.push_back(t);
        if (powered) todo.push_back({t + processUs + (delayFor ? delayFor(req) : 0.0), req});
        line.clear();
      } else {
        line += (char)c;
      }
    }
    while (!todo.empty() && todo.front().first <= now) {
      const auto job = todo.front();
      todo.pop_front();
      if (!powered) continue;
      for (unsigned char c : handle(job.second, job.first)) {
        outLast = std::max(job.first, outLast) + kByteUs;
        out.push_back({outLast, c});
      }
    }
    while (!out.empty() && out.front().first <= now) {
      const uint8_t c = out.front().second;
      out.pop_front();
      if (powered) hal_host::linkInject(&c, 1);
    }
  }

  size_t count(const std::string& prefix, size_t from = 0) const {
    size_t n = 0;
    for (size_t i = from; i < wireLines.size(); i++) n += startsWith(wireLines[i], prefix);
    return n;
  }
  size_t countSince(const std::string& prefix, double t0) const {
    size_t n = 0;
    for (size_t i = 0; i < wireLines.size(); i++) n += wireTimes[i] >= t0 && startsWith(wireLines[i], prefix);
    return n;
  }
  std::string last(const std::string& prefix) const {
    for (size_t i = wireLines.size(); i > 0; i--)
      if (startsWith(wireLines[i - 1], prefix)) return wireLines[i - 1];
    return "";
  }
};

// ---- the rig: hal_host + Debug + ControlLink + FakeEsp, stepped like loop() ----------------
struct Rig {
  Debug dbg;
  ControlLink link;
  FakeEsp esp;
  std::string usbOut;                          // everything the RA4M1 sent on USB
  std::function<void()> everyLoop;             // test hook, after update()

  explicit Rig(uint64_t startUs = 0) {
    hal_host::reset();
    if (startUs) hal_host::setTimeUs(startUs);
    hal::begin();
    dbg.begin(DebugSources());
    link.begin(dbg, hal::nowMs(), hal::nowUs());
  }
  double now() const { return (double)hal_host::timeUs(); }
  void step(uint32_t loopUs) {
    hal_host::advanceUs(loopUs);
    esp.pump(now());
    link.update(hal::nowMs(), hal::nowUs());
    if (everyLoop) everyLoop();
    hal::service(hal::nowUs());
    esp.pump(now());
    usbOut += hal_host::usbTakeTx();
  }
  void runUs(uint64_t us, uint32_t loopUs = 200) {
    const uint64_t end = hal_host::timeUs() + us;
    while (hal_host::timeUs() < end) step(loopUs);
  }
  void runMs(uint64_t ms, uint32_t loopUs = 200) { runUs(ms * US_PER_MS, loopUs); }
  // Until pred() or the timeout; true when pred() became true.
  bool runUntil(const std::function<bool()>& pred, uint64_t timeoutMs, uint32_t loopUs = 200) {
    const uint64_t end = hal_host::timeUs() + timeoutMs * US_PER_MS;
    while (hal_host::timeUs() < end) {
      step(loopUs);
      if (pred()) return true;
    }
    return pred();
  }
  // Serial has no FIFO: service() moves one byte per call, so give queued text time to leave.
  void drainUsb() { runMs(DEBUG_TX_BUFFER_BYTES * UART_BITS_PER_BYTE * US_PER_S / DEBUG_SERIAL_BAUD / US_PER_MS + 1, 50); }
  bool runToDs() {
    return runUntil([&] { return link.atPhase() == ControlLink::AtPhase::RUN_DS && link.connected(); }, 1000);
  }
  size_t countText(const std::string& needle) const {
    size_t n = 0;
    for (size_t p = usbOut.find(needle); p != std::string::npos; p = usbOut.find(needle, p + 1)) n++;
    return n;
  }
  std::vector<ds_status_packet_t> statusPackets() const {
    ds_deframer_t d;
    ds_deframer_init(&d);
    std::vector<ds_status_packet_t> out;
    for (unsigned char c : usbOut) {
      uint8_t t = ds_deframer_feed(&d, c);
      while (t != DS_PKT_NONE) {
        ds_status_packet_t p;
        if (t == DS_PKT_STATUS && ds_status_decode(ds_deframer_frame(&d), ds_deframer_frame_len(&d), &p))
          out.push_back(p);
        t = ds_deframer_poll(&d);
      }
    }
    return out;
  }
};

std::string controlFrame(uint8_t flags, int16_t rx, int16_t ry, uint16_t rt, uint16_t buttons, uint8_t seq) {
  ds_control_packet_t p;
  ds_control_init(&p);
  p.flags = flags;
  p.rx = rx;
  p.ry = ry;
  p.rt = rt;
  p.buttons = buttons;
  p.seq = seq;
  uint8_t f[DS_CONTROL_FRAME_LEN];
  const size_t n = ds_control_encode(&p, f);
  return std::string((const char*)f, n);
}

std::string telemetryFrame(uint8_t seq) {
  ds_telemetry_packet_t p;
  ds_telemetry_init(&p);
  p.seq = seq;
  p.uptimeMs = 1000u * seq;
  p.panCdeg = -4550;
  uint8_t f[DS_TELEMETRY_FRAME_LEN];
  const size_t n = ds_telemetry_encode(&p, f);
  return std::string((const char*)f, n);
}

std::vector<uint8_t> xstate(uint16_t rh, uint16_t rv, uint16_t rt, uint8_t dpad, uint8_t b0, uint8_t b1,
                            uint8_t conn, uint8_t seq) {
  xbox_r4_state_t s{};
  xbox_r4_neutral_report(s.report);
  s.report[XSTATE_OFS_JOY_RH] = (uint8_t)rh;
  s.report[XSTATE_OFS_JOY_RH + 1] = (uint8_t)(rh >> 8);
  s.report[XSTATE_OFS_JOY_RV] = (uint8_t)rv;
  s.report[XSTATE_OFS_JOY_RV + 1] = (uint8_t)(rv >> 8);
  s.report[XSTATE_OFS_TRIG_RT] = (uint8_t)rt;
  s.report[XSTATE_OFS_TRIG_RT + 1] = (uint8_t)(rt >> 8);
  s.report[XSTATE_OFS_DPAD] = dpad;
  s.report[XSTATE_OFS_BTN0] = b0;
  s.report[XSTATE_OFS_BTN1] = b1;
  s.state = conn;
  s.battery = 50;
  s.seq = seq;
  std::vector<uint8_t> out(XBOX_R4_STATE_LEN);
  xbox_r4_encode(&s, out.data());
  return out;
}

void putLe16(uint8_t* report, size_t ofs, uint16_t v) {
  report[ofs] = (uint8_t)v;
  report[ofs + 1] = (uint8_t)(v >> 8);
}

// Every byte of the 20: both sticks, both triggers, d-pad, the three button bytes, state, battery.
struct Sticks { uint16_t lh, lv, rh, rv, lt, rt; };
const Sticks kCentred = {32768, 32768, 32768, 32768, 0, 0};

std::vector<uint8_t> xstateFull(const Sticks& a, uint8_t dpad, uint8_t b0, uint8_t b1, uint8_t b2,
                                uint8_t conn, uint8_t battery, uint8_t seq) {
  xbox_r4_state_t s{};
  xbox_r4_neutral_report(s.report);
  putLe16(s.report, XSTATE_OFS_JOY_LH, a.lh);
  putLe16(s.report, XSTATE_OFS_JOY_LV, a.lv);
  putLe16(s.report, XSTATE_OFS_JOY_RH, a.rh);
  putLe16(s.report, XSTATE_OFS_JOY_RV, a.rv);
  putLe16(s.report, XSTATE_OFS_TRIG_LT, a.lt);
  putLe16(s.report, XSTATE_OFS_TRIG_RT, a.rt);
  s.report[XSTATE_OFS_DPAD] = dpad;
  s.report[XSTATE_OFS_BTN0] = b0;
  s.report[XSTATE_OFS_BTN1] = b1;
  s.report[XSTATE_OFS_BTN2] = b2;
  s.state = conn;
  s.battery = battery;
  s.seq = seq;
  std::vector<uint8_t> out(XBOX_R4_STATE_LEN);
  xbox_r4_encode(&s, out.data());
  return out;
}

void setPad(FakeEsp& esp, uint16_t rh, uint16_t rv, uint16_t rt, uint8_t b0) {
  esp.st.report[XSTATE_OFS_JOY_RH] = (uint8_t)rh;
  esp.st.report[XSTATE_OFS_JOY_RH + 1] = (uint8_t)(rh >> 8);
  esp.st.report[XSTATE_OFS_JOY_RV] = (uint8_t)rv;
  esp.st.report[XSTATE_OFS_JOY_RV + 1] = (uint8_t)(rv >> 8);
  esp.st.report[XSTATE_OFS_TRIG_RT] = (uint8_t)rt;
  esp.st.report[XSTATE_OFS_TRIG_RT + 1] = (uint8_t)(rt >> 8);
  esp.st.report[XSTATE_OFS_BTN0] = b0;
}

void expectNoBlockingWrites() {
  CHECK_EQ(hal_host::linkTxProbe().blockingWrites, 0u);
  CHECK_EQ(hal_host::usbTxProbe().blockingWrites, 0u);
  CHECK(hal_host::linkTxProbe().largestWrite <= (size_t)SCI_FIFO_DEPTH);
  CHECK(hal_host::usbTxProbe().largestWrite <= (size_t)SERIAL_TX_SLACK_BYTES);
}

}  // namespace

// =============================================================================================
// AT client sequencing
// =============================================================================================

TEST(pings_every_link_esp_ping_ms_until_any_final) {
  Rig rig;
  rig.esp.powered = false;
  rig.runMs(LINK_ESP_PING_MS * 3 + LINK_ESP_PING_MS / 2);
  CHECK_EQ(rig.esp.wireLines.size(), (size_t)4);   // at 0, 1, 2, 3 x LINK_ESP_PING_MS
  CHECK_EQ(rig.esp.count("AT", 0), rig.esp.wireLines.size());
  for (const std::string& l : rig.esp.wireLines) CHECK_EQ(l, std::string("AT"));
  CHECK(rig.link.atPhase() == ControlLink::AtPhase::PING);
  CHECK(!rig.link.status().espAlive);
  CHECK(rig.link.stats().pollTimeouts >= 3u);
  // An ERROR answer also proves the ESP32 is alive.
  rig.esp.powered = true;
  rig.esp.hook = [](const std::string& req, std::string& reply) {
    if (req != "AT") return false;
    reply = "ERROR\r\n";
    return true;
  };
  CHECK(rig.runUntil([&] { return rig.link.atPhase() != ControlLink::AtPhase::PING; }, 500));
  CHECK(rig.link.status().espAlive);
  expectNoBlockingWrites();
}

TEST(ds_firmware_sequence_caps_then_dss_then_polls) {
  Rig rig;
  CHECK(rig.runToDs());
  const auto& L = rig.esp.wireLines;
  CHECK(L.size() >= 5u);
  if (L.size() < 5u) return;
  CHECK_EQ(L[0], std::string("AT"));
  CHECK_EQ(L[1], std::string("AT+DSCAPS?"));
  CHECK_EQ(L[2], std::string("AT+DSCAPS?"));           // two agreeing answers decide
  CHECK_EQ(L[3], std::string("AT+DSS=0,0,0,0,0,0"));   // BLE, rumbleSeq 0, no rumble, state OFF
  CHECK_EQ(L[4], std::string("AT+XBOXSTATE?"));
  const LinkStatus& s = rig.link.status();
  CHECK(s.espCapsKnown);
  CHECK(!s.espLegacy);
  CHECK_EQ(s.espProto, (uint8_t)DS_PROTO_VERSION);
  CHECK_EQ(s.espCaps, (uint8_t)DS_CAP_ALL);
  CHECK_EQ(s.espMode, (uint8_t)DS_ESP_MODE_BLE);
  CHECK_EQ(s.bootId, (uint8_t)0x5C);
  CHECK_EQ(s.activeSource, (uint8_t)DS_LINK_SRC_BLE);
  CHECK(rig.link.frame().valid);
  CHECK_EQ(rig.link.frame().source, (uint8_t)DS_LINK_SRC_BLE);
  rig.drainUsb();
  CHECK_EQ(rig.countText("link: ESP32 D-Sentry firmware"), (size_t)1);
}

TEST(polls_run_at_control_packet_hz_without_timeouts) {
  Rig rig;
  CHECK(rig.runToDs());
  rig.runMs(MS_PER_S);   // settle: the first rate window
  const double t0 = rig.now();
  const uint32_t to0 = rig.link.stats().pollTimeouts;
  rig.runMs(MS_PER_S * 2);
  const size_t polls = rig.esp.countSince("AT+XBOXSTATE?", t0);
  CHECK(polls >= (size_t)(CONTROL_PACKET_HZ * 2 - 4) && polls <= (size_t)(CONTROL_PACKET_HZ * 2 + 1));
  CHECK_EQ(rig.link.stats().pollTimeouts, to0);
  CHECK(rig.link.stats().ctrlRxHz >= CONTROL_PACKET_HZ - 3 && rig.link.stats().ctrlRxHz <= CONTROL_PACKET_HZ + 1);
  CHECK(rig.link.connected());
  CHECK_EQ(rig.link.stats().pollErrors, 0u);
  expectNoBlockingWrites();
}

TEST(xboxstate_conversion_is_exactly_the_contract) {
  ControlFrame f;
  uint8_t conn = 0xFF;
  // Stick extremes: 0 -> -512 (left / UP), 65535 -> 511, centre -> 0, one below -> -1 (floor).
  auto v = xstate(0, 0, 1023, XSTATE_DPAD_NONE, 0, 0, XBOX_R4_CONNECTED, 7);
  CHECK(ControlLink::decodeXboxState(v.data(), v.size(), f, conn));
  CHECK_EQ(f.rx, (int16_t)-512);
  CHECK_EQ(f.ry, (int16_t)-512);   // stick up is negative on the wire
  CHECK_EQ(f.rt, (uint16_t)1023);
  CHECK_EQ(f.seq, (uint8_t)7);
  CHECK(f.connected);
  CHECK(!f.espPairing);
  CHECK_EQ(conn, (uint8_t)XBOX_R4_CONNECTED);
  v = xstate(65535, 65535, 0, XSTATE_DPAD_NONE, 0, 0, XBOX_R4_CONNECTED, 0);
  CHECK(ControlLink::decodeXboxState(v.data(), v.size(), f, conn));
  CHECK_EQ(f.rx, (int16_t)511);
  CHECK_EQ(f.ry, (int16_t)511);
  v = xstate(32768, 32767, 0, XSTATE_DPAD_NONE, 0, 0, XBOX_R4_CONNECTED, 0);
  CHECK(ControlLink::decodeXboxState(v.data(), v.size(), f, conn));
  CHECK_EQ(f.rx, (int16_t)0);
  CHECK_EQ(f.ry, (int16_t)-1);
  v = xstate(32768 + 64 * 100, 32768 - 64 * 100, 0, XSTATE_DPAD_NONE, 0, 0, XBOX_R4_CONNECTED, 0);
  CHECK(ControlLink::decodeXboxState(v.data(), v.size(), f, conn));
  CHECK_EQ(f.rx, (int16_t)100);
  CHECK_EQ(f.ry, (int16_t)-100);
  // Buttons, one at a time.
  struct B { uint8_t b0, b1; uint16_t want; };
  const B buttons[] = {{XSTATE_BTN0_A, 0, DS_BTN_A},   {XSTATE_BTN0_B, 0, DS_BTN_B},
                       {XSTATE_BTN0_X, 0, DS_BTN_X},   {XSTATE_BTN0_Y, 0, DS_BTN_Y},
                       {XSTATE_BTN0_LB, 0, DS_BTN_LB}, {XSTATE_BTN0_RB, 0, DS_BTN_RB},
                       {0, XSTATE_BTN1_MENU, DS_BTN_START}, {0, XSTATE_BTN1_VIEW, DS_BTN_BACK},
                       {0, XSTATE_BTN1_LS, DS_BTN_LS_CLICK}, {0, XSTATE_BTN1_RS, DS_BTN_RS_CLICK},
                       {0, XSTATE_BTN1_XBOX, DS_BTN_XBOX},
                       {0x24, 0x83, 0}};   // bits with no D-Sentry meaning map to nothing
  for (const B& b : buttons) {
    v = xstate(32768, 32768, 0, XSTATE_DPAD_NONE, b.b0, b.b1, XBOX_R4_CONNECTED, 0);
    CHECK(ControlLink::decodeXboxState(v.data(), v.size(), f, conn));
    CHECK_EQ(f.buttons, b.want);
  }
  // D-pad codes.
  struct D { uint8_t code; uint16_t want; };
  const D dpad[] = {{XSTATE_DPAD_NONE, 0},
                    {XSTATE_DPAD_N, DS_BTN_DPAD_UP},
                    {XSTATE_DPAD_NE, DS_BTN_DPAD_UP | DS_BTN_DPAD_RIGHT},
                    {XSTATE_DPAD_E, DS_BTN_DPAD_RIGHT},
                    {XSTATE_DPAD_SE, DS_BTN_DPAD_DOWN | DS_BTN_DPAD_RIGHT},
                    {XSTATE_DPAD_S, DS_BTN_DPAD_DOWN},
                    {XSTATE_DPAD_SW, DS_BTN_DPAD_DOWN | DS_BTN_DPAD_LEFT},
                    {XSTATE_DPAD_W, DS_BTN_DPAD_LEFT},
                    {XSTATE_DPAD_NW, DS_BTN_DPAD_UP | DS_BTN_DPAD_LEFT},
                    {42, 0}};
  for (const D& d : dpad) {
    v = xstate(32768, 32768, 0, d.code, 0, 0, XBOX_R4_CONNECTED, 0);
    CHECK(ControlLink::decodeXboxState(v.data(), v.size(), f, conn));
    CHECK_EQ(f.buttons, d.want);
  }
  // Connection states: only 0 is connected; 1..3 are the ESP32 pairing.
  const uint8_t states[] = {XBOX_R4_WAITING_FIRST_REPORT, XBOX_R4_FOUND, XBOX_R4_SCANNING, XBOX_R4_OFF,
                            XBOX_R4_BLOCKED_BY_HCI};
  for (uint8_t s : states) {
    v = xstate(32768, 32768, 0, 0, 0, 0, s, 0);
    CHECK(ControlLink::decodeXboxState(v.data(), v.size(), f, conn));
    CHECK(!f.connected);
    CHECK_EQ(f.espPairing, s <= XBOX_R4_SCANNING);
    CHECK_EQ(conn, s);
  }
  // Bad CRC and wrong length are refused.
  v = xstate(0, 0, 0, 0, 0, 0, XBOX_R4_CONNECTED, 0);
  v[XBOX_R4_STATE_LEN - 1] ^= 1;
  CHECK(!ControlLink::decodeXboxState(v.data(), v.size(), f, conn));
  v = xstate(0, 0, 0, 0, 0, 0, XBOX_R4_CONNECTED, 0);
  CHECK(!ControlLink::decodeXboxState(v.data(), v.size() - 1, f, conn));
  CHECK(!ControlLink::decodeXboxState(nullptr, v.size(), f, conn));
}

// The dashboard's controller block (TelemetryPacket v2): left stick, left trigger, the stick
// clicks, Xbox and Share, battery and the raw state byte, exactly as the fork's 20 bytes say.
TEST(xboxstate_controller_block_fields_are_exactly_the_payload) {
  ControlFrame f;
  uint8_t conn = 0xFF;
  // Left stick at the extremes and the floor below centre, the same (raw - 32768) >> 6 as the
  // right stick; the right stick and right trigger are unaffected.
  auto v = xstateFull({0, 65535, 32768, 32768, 1023, 0}, 0, 0, 0, 0, XBOX_R4_CONNECTED, 80, 3);
  CHECK(ControlLink::decodeXboxState(v.data(), v.size(), f, conn));
  CHECK_EQ(f.lx, (int16_t)-512);
  CHECK_EQ(f.ly, (int16_t)511);   // 65535 = full down: positive (wire: up negative)
  CHECK_EQ(f.lt, (uint16_t)1023);
  CHECK_EQ(f.rx, (int16_t)0);
  CHECK_EQ(f.ry, (int16_t)0);
  CHECK_EQ(f.rt, (uint16_t)0);
  CHECK_EQ(f.battery, (uint8_t)80);
  CHECK_EQ(f.padState, (uint8_t)DS_PAD_STATE_CONNECTED);
  CHECK_EQ(f.seq, (uint8_t)3);
  v = xstateFull({32767, 32768 + 64 * 7, 32768, 32768, 0, 0}, 0, 0, 0, 0, XBOX_R4_CONNECTED,
                 XBOX_R4_BATTERY_UNKNOWN, 0);
  CHECK(ControlLink::decodeXboxState(v.data(), v.size(), f, conn));
  CHECK_EQ(f.lx, (int16_t)-1);
  CHECK_EQ(f.ly, (int16_t)7);
  CHECK_EQ(f.lt, (uint16_t)0);
  CHECK_EQ(f.battery, (uint8_t)DS_PAD_BATTERY_UNKNOWN);
  // Byte 15: Share is bit 0; its other bits mean nothing.
  v = xstateFull(kCentred, 0, 0, 0, XSTATE_BTN2_SHARE, XBOX_R4_CONNECTED, 1, 0);
  CHECK(ControlLink::decodeXboxState(v.data(), v.size(), f, conn));
  CHECK_EQ(f.buttons, (uint16_t)DS_BTN_SHARE);
  v = xstateFull(kCentred, 0, 0, 0, 0xFE, XBOX_R4_CONNECTED, 1, 0);
  CHECK(ControlLink::decodeXboxState(v.data(), v.size(), f, conn));
  CHECK_EQ(f.buttons, (uint16_t)0);
  // Everything at once: all twelve control buttons, the four extra ones, a diagonal.
  v = xstateFull(kCentred, XSTATE_DPAD_SW, 0xFF, 0xFF, 0xFF, XBOX_R4_CONNECTED, 100, 0);
  CHECK(ControlLink::decodeXboxState(v.data(), v.size(), f, conn));
  CHECK_EQ(f.buttons, (uint16_t)(DS_BTN_A | DS_BTN_B | DS_BTN_X | DS_BTN_Y | DS_BTN_LB | DS_BTN_RB |
                                 DS_BTN_DPAD_DOWN | DS_BTN_DPAD_LEFT | DS_BTN_START | DS_BTN_BACK |
                                 DS_BTN_LS_CLICK | DS_BTN_RS_CLICK | DS_BTN_XBOX | DS_BTN_SHARE));
  // padState is the state byte as sent, every value of enum xbox_r4_conn_state (= DS_PAD_STATE_*).
  const uint8_t states[] = {XBOX_R4_CONNECTED, XBOX_R4_WAITING_FIRST_REPORT, XBOX_R4_FOUND,
                            XBOX_R4_SCANNING, XBOX_R4_OFF, XBOX_R4_BLOCKED_BY_HCI};
  const uint8_t padStates[] = {DS_PAD_STATE_CONNECTED, DS_PAD_STATE_WAITING_FIRST_REPORT,
                               DS_PAD_STATE_FOUND, DS_PAD_STATE_SCANNING, DS_PAD_STATE_OFF,
                               DS_PAD_STATE_BLOCKED_BY_HCI};
  for (size_t i = 0; i < sizeof states; i++) {
    v = xstateFull({1000, 2000, 32768, 32768, 300, 0}, 0, 0, 0, 0, states[i], 42, 0);
    CHECK(ControlLink::decodeXboxState(v.data(), v.size(), f, conn));
    CHECK_EQ(f.padState, padStates[i]);
    CHECK_EQ(f.battery, (uint8_t)42);
  }
}

TEST(usb_frames_have_no_left_stick_and_an_unknown_battery) {
  Rig rig;
  rig.esp.powered = false;
  hal_host::usbInject(controlFrame(DS_CTRL_FLAG_CONNECTED, 300, -300, 900,
                                   DS_BTN_LB | DS_BTN_XBOX | DS_BTN_SHARE, 1));
  rig.runMs(kPollMs);
  const ControlFrame& f = rig.link.frame();
  CHECK_EQ(f.source, (uint8_t)DS_LINK_SRC_USB);
  CHECK(f.connected);
  CHECK_EQ(f.rx, (int16_t)300);
  CHECK_EQ(f.ry, (int16_t)-300);
  CHECK_EQ(f.rt, (uint16_t)900);
  CHECK_EQ(f.buttons, (uint16_t)(DS_BTN_LB | DS_BTN_XBOX | DS_BTN_SHARE));   // kept, all 16 bits
  CHECK_EQ(f.lx, (int16_t)0);
  CHECK_EQ(f.ly, (int16_t)0);
  CHECK_EQ(f.lt, (uint16_t)0);
  CHECK_EQ(f.battery, (uint8_t)DS_PAD_BATTERY_UNKNOWN);
  CHECK_EQ(f.padState, (uint8_t)DS_PAD_STATE_CONNECTED);
  // A packet that says "not connected" (the relay with no input source) reads as 3.
  hal_host::usbInject(controlFrame(0, 0, 0, 0, 0, 2));
  rig.runMs(kPollMs);
  CHECK_EQ(rig.link.frame().source, (uint8_t)DS_LINK_SRC_USB);
  CHECK(!rig.link.frame().connected);
  CHECK_EQ(rig.link.frame().padState, (uint8_t)DS_PAD_STATE_SCANNING);
}

TEST(radio_frame_reaches_frame_with_the_pad_values) {
  Rig rig;
  CHECK(rig.runToDs());
  setPad(rig.esp, 65535, 0, 700, XSTATE_BTN0_A | XSTATE_BTN0_LB);
  rig.esp.st.seq = 99;
  rig.runMs(kPollMs * 3);
  const ControlFrame& f = rig.link.frame();
  CHECK(f.connected);
  CHECK_EQ(f.rx, (int16_t)511);
  CHECK_EQ(f.ry, (int16_t)-512);
  CHECK_EQ(f.rt, (uint16_t)700);
  CHECK_EQ(f.buttons, (uint16_t)(DS_BTN_A | DS_BTN_LB));
  CHECK_EQ(f.seq, (uint8_t)99);
  CHECK_EQ(f.source, (uint8_t)DS_LINK_SRC_BLE);
  CHECK((uint32_t)(hal::nowMs() - f.rxMs) <= kPollMs * 2);
}

TEST(bad_crc_frames_are_counted_and_never_obeyed) {
  Rig rig;
  rig.esp.badCrc = true;
  rig.runMs(MS_PER_S);
  CHECK(rig.link.atPhase() == ControlLink::AtPhase::RUN_DS);
  CHECK(!rig.link.connected());
  CHECK_EQ(rig.link.stats().radioFrames, 0u);
  CHECK(rig.link.stats().crcErrors >= (uint32_t)(CONTROL_PACKET_HZ / 2));
  CHECK(rig.link.status().espAlive);   // the ESP32 is answering, just not usefully
  rig.esp.badCrc = false;
  CHECK(rig.runUntil([&] { return rig.link.connected(); }, kPollMs * 3));
}

// KEY SAFETY PROPERTY (radio): frames stop -> not connected within LINK_TIMEOUT_MS, and the
// published frame is neutral from then on.
TEST(radio_link_loss_is_seen_within_link_timeout) {
  Rig rig;
  CHECK(rig.runToDs());
  setPad(rig.esp, 65535, 65535, 1023, XSTATE_BTN0_A);
  rig.runMs(kPollMs * 5);
  CHECK(rig.link.connected());
  CHECK_EQ(rig.link.frame().rt, (uint16_t)1023);
  // The ESP32 keeps answering every poll, but no frame is valid any more (a corrupted state
  // buffer): only the frame's own age can end the link; the ESP32 never looks silent.
  rig.esp.badCrc = true;
  const uint32_t lastFrame = rig.link.status().lastFrameMs;
  bool lost = false;
  uint32_t lostAt = 0;
  for (int i = 0; i < 1000 && !lost; i++) {
    rig.step(US_PER_MS / 4);
    if (!rig.link.connected()) {
      lost = true;
      lostAt = hal::nowMs();
    }
  }
  CHECK(lost);
  CHECK((uint32_t)(lostAt - lastFrame) <= LINK_TIMEOUT_MS);
  CHECK((uint32_t)(lostAt - lastFrame) >= LINK_TIMEOUT_MS - kPollMs);   // not spuriously early
  const ControlFrame& f = rig.link.frame();
  CHECK_EQ(f.rx, (int16_t)0);
  CHECK_EQ(f.ry, (int16_t)0);
  CHECK_EQ(f.rt, (uint16_t)0);
  CHECK_EQ(f.buttons, (uint16_t)0);
  CHECK_EQ(rig.link.status().activeSource, (uint8_t)DS_LINK_SRC_NONE);
  CHECK(rig.link.status().espAlive);   // it was the frame age, not ESP32 silence
  CHECK(rig.link.atPhase() == ControlLink::AtPhase::RUN_DS);
  rig.runMs(MS_PER_S);
  CHECK(!rig.link.connected());
}

TEST(pad_disconnect_reported_by_the_esp32_drops_at_the_next_poll) {
  Rig rig;
  CHECK(rig.runToDs());
  rig.esp.st.state = XBOX_R4_SCANNING;
  CHECK(rig.runUntil([&] { return !rig.link.connected(); }, kPollMs * 2));
  CHECK(rig.link.frame().espPairing);
}

TEST(esp_silent_for_link_timeout_goes_back_to_pinging_and_recovers) {
  Rig rig;
  CHECK(rig.runToDs());
  rig.esp.powerOff();
  const double t0 = rig.now();
  CHECK(rig.runUntil([&] { return rig.link.atPhase() == ControlLink::AtPhase::PING; }, LINK_TIMEOUT_MS * 2));
  CHECK(rig.now() - t0 <= (LINK_TIMEOUT_MS + kPollMs) * (double)US_PER_MS);
  CHECK(!rig.link.status().espAlive);
  CHECK_EQ(rig.link.stats().espSilences, 1u);
  CHECK(!rig.link.connected());
  const size_t pings0 = rig.esp.count("AT", 0) - rig.esp.count("AT+", 0);
  rig.runMs(LINK_ESP_PING_MS * 3);
  const size_t pings = rig.esp.count("AT", 0) - rig.esp.count("AT+", 0) - pings0;
  CHECK(pings >= 3u && pings <= 4u);
  const size_t dss0 = rig.esp.count("AT+DSS=");
  rig.esp.powered = true;
  CHECK(rig.runToDs());
  CHECK(rig.esp.count("AT+DSS=") > dss0);   // the new session starts with AT+DSS
  CHECK(rig.link.status().espAlive);
}

TEST(esp_reboot_seen_as_a_new_boot_id_resends_dss) {
  Rig rig;
  CHECK(rig.runToDs());
  rig.esp.bootId = 0x77;   // rebooted between two polls (fast boot, still BLE)
  const size_t dss0 = rig.esp.count("AT+DSS=");
  CHECK(rig.runUntil([&] { return rig.link.status().bootId == 0x77; }, STATUS_HEARTBEAT_MS + kPollMs * 5));
  rig.runMs(kPollMs * 3);
  CHECK_EQ(rig.link.stats().espRestarts, 1u);
  CHECK(rig.esp.count("AT+DSS=") >= dss0 + 2);   // the heartbeat that saw it, then the re-send
  rig.drainUsb();
  CHECK_EQ(rig.countText("link: ESP32 rebooted"), (size_t)1);
  CHECK(rig.link.connected());
}

TEST(off_while_ble_wanted_resends_dss_rate_limited) {
  Rig rig;
  CHECK(rig.runToDs());
  // Right after a DSS exchange, the ESP32 reports its BLE client OFF.
  rig.runUntil([&] { return rig.esp.last("AT").rfind("AT+DSS=", 0) == 0; }, STATUS_HEARTBEAT_MS * 2, 100);
  rig.runMs(kPollMs * 2);
  const size_t dss0 = rig.esp.count("AT+DSS=");
  rig.esp.st.state = XBOX_R4_OFF;
  rig.runMs(kPollMs * 4);
  CHECK(rig.esp.count("AT+DSS=") >= dss0 + 1);   // long before the next heartbeat
  CHECK(!rig.link.connected());
  const size_t dss1 = rig.esp.count("AT+DSS=");
  rig.runMs(MS_PER_S * 2);
  const size_t extra = rig.esp.count("AT+DSS=") - dss1;
  // At most the heartbeats plus one re-send per LINK_BEGIN_RETRY_MS (a re-send every poll would
  // be CONTROL_PACKET_HZ per second).
  CHECK(extra <= (size_t)(2 * MS_PER_S / STATUS_HEARTBEAT_MS + 2 * MS_PER_S / LINK_BEGIN_RETRY_MS + 1));
  CHECK_EQ(rig.link.stats().beginsSent, 0u);   // D-Sentry firmware never gets XBOXBEGIN
}

TEST(dss_goes_out_on_change_and_on_the_heartbeat) {
  Rig rig;
  CHECK(rig.runToDs());
  rig.runMs(kPollMs * 2);
  rig.link.setTurretState(DS_STATE_DISARMED);
  CHECK(rig.runUntil([&] { return rig.esp.last("AT+DSS=") == "AT+DSS=0,0,0,0,0,4"; }, kPollMs * 3));
  // Unchanged: the next one is a heartbeat, STATUS_HEARTBEAT_MS later.
  const double t0 = rig.esp.wireTimes.back();
  const size_t n0 = rig.esp.count("AT+DSS=");
  CHECK(rig.runUntil([&] { return rig.esp.count("AT+DSS=") > n0; }, STATUS_HEARTBEAT_MS * 2));
  const double gapMs = (rig.esp.wireTimes.back() - t0) / US_PER_MS;
  CHECK(gapMs >= STATUS_HEARTBEAT_MS - 2 && gapMs <= STATUS_HEARTBEAT_MS + kPollMs * 2);
  // A rumble: rumbleSeq increments every time, even with identical values.
  rig.link.requestRumble(RUMBLE_STRONG_255, RUMBLE_WEAK_255, RUMBLE_DURATION_MS);
  const std::string want1 = "AT+DSS=0,1," + std::to_string(RUMBLE_STRONG_255) + "," +
                            std::to_string(RUMBLE_WEAK_255) + "," + std::to_string(RUMBLE_DURATION_MS) + ",4";
  CHECK(rig.runUntil([&] { return rig.esp.last("AT+DSS=") == want1; }, kPollMs * 3));
  rig.link.requestRumble(RUMBLE_STRONG_255, RUMBLE_WEAK_255, RUMBLE_DURATION_MS);
  CHECK(rig.runUntil([&] { return rig.esp.last("AT+DSS=").rfind("AT+DSS=0,2,", 0) == 0; }, kPollMs * 3));
  rig.runMs(kPollMs);   // the ESP32 acts on a line after its AT-task latency
  CHECK_EQ(rig.link.rumbleSeq(), (uint8_t)2);
  CHECK_EQ(rig.esp.rumbles, 2);
}

TEST(failed_dss_is_retried_without_starving_the_polls) {
  Rig rig;
  CHECK(rig.runToDs());
  int failures = 30;
  rig.esp.hook = [&](const std::string& req, std::string& reply) {
    if (req.rfind("AT+DSS=", 0) != 0 || failures <= 0) return false;
    failures--;
    reply = "+DSS: ERROR\r\n";
    return true;
  };
  rig.link.setTurretState(DS_STATE_ARMED);
  bool everLost = false;
  rig.everyLoop = [&] { everLost = everLost || !rig.link.connected(); };
  rig.runMs(MS_PER_S);
  CHECK(!everLost);   // polls kept the radio frame fresh the whole time
  CHECK(rig.link.stats().pollErrors >= 20u);
  CHECK(rig.runUntil([&] { return rig.esp.last("AT+DSS=") == "AT+DSS=0,0,0,0,0,5" && failures <= 0; }, MS_PER_S));
  // Retries are at least one poll period apart.
  std::vector<double> t;
  for (size_t i = 0; i < rig.esp.wireLines.size(); i++)
    if (rig.esp.wireLines[i].rfind("AT+DSS=", 0) == 0) t.push_back(rig.esp.wireTimes[i]);
  for (size_t i = 1; i < t.size(); i++) CHECK(t[i] - t[i - 1] >= (kPollMs - 1) * (double)US_PER_MS);
}

TEST(a_single_odd_dscaps_answer_never_decides_the_firmware) {
  // One bare ERROR, then one garbled dsProto: neither is believed alone.
  for (const char* odd : {"ERROR\r\n", "+DSCAPS: 11,15\r\nOK\r\n"}) {
    Rig rig;
    int n = 0;
    rig.esp.hook = [&](const std::string& req, std::string& reply) {
      if (req != "AT+DSCAPS?" || n++ > 0) return false;
      reply = odd;
      return true;
    };
    CHECK(rig.runToDs());
    CHECK(!rig.link.status().espLegacy);
    CHECK_EQ(rig.esp.count("AT+DSCAPS?"), (size_t)3);   // odd, answer, agreeing answer
  }
}

// AT text has no CRC: one garbled +DSS reply must not close the radio path or count a reboot.
TEST(a_garbled_dss_reply_is_never_believed_alone) {
  Rig rig;
  CHECK(rig.runToDs());
  bool everLost = false;
  rig.everyLoop = [&] { everLost = everLost || !rig.link.connected(); };
  const char* garbled[] = {
      "+DSS: 10,92,0\r\nOK\r\n",    // espMode outside the enum
      "+DSS: 0,0,0\r\nOK\r\n",      // bootId 0 is never sent
      "+DSS: 0,92,9\r\nOK\r\n",     // wifiStatus outside the enum
      "+DSS: 1,92,0\r\nOK\r\n",     // a valid-looking radio change, said once
      "+DSS: 0,192,0\r\nOK\r\n",    // a valid-looking reboot, said once
  };
  for (const char* g : garbled) {
    bool once = true;
    rig.esp.hook = [&](const std::string& req, std::string& reply) {
      if (req.rfind("AT+DSS=", 0) != 0 || !once) return false;
      once = false;
      reply = g;
      return true;
    };
    rig.link.setTurretState((uint8_t)(rig.link.status().espMode == DS_ESP_MODE_BLE ? DS_STATE_DISARMED : DS_STATE_PAIRING));
    rig.runMs(STATUS_HEARTBEAT_MS / 5);
    CHECK(!once);   // the garbled reply was delivered
    CHECK_EQ(rig.link.status().espMode, (uint8_t)DS_ESP_MODE_BLE);
    CHECK_EQ(rig.link.status().bootId, (uint8_t)0x5C);
    rig.link.setTurretState(DS_STATE_ARMED);   // a fresh change for the next round
    rig.runMs(kPollMs * 2);
  }
  CHECK(!everLost);
  CHECK_EQ(rig.link.stats().espRestarts, 0u);
  // A real change is adopted once the repeat agrees.
  rig.esp.bootId = 0x77;
  CHECK(rig.runUntil([&] { return rig.link.status().bootId == 0x77; }, STATUS_HEARTBEAT_MS * 2));
  CHECK_EQ(rig.link.stats().espRestarts, 1u);
}

// =============================================================================================
// Legacy firmware
// =============================================================================================

TEST(legacy_firmware_bt_only_with_rate_limited_xboxbegin) {
  Rig rig;
  rig.esp.fw = FakeEsp::Fw::LEGACY;
  rig.esp.st.state = XBOX_R4_OFF;
  rig.esp.beginConnects = false;   // BEGIN starts scanning; the pad never shows up
  CHECK(rig.runUntil([&] { return rig.link.atPhase() == ControlLink::AtPhase::RUN_LEGACY; }, 1000));
  CHECK(rig.link.status().espLegacy);
  CHECK(rig.link.status().espCapsKnown);
  CHECK_EQ(rig.link.status().espMode, (uint8_t)DS_ESP_MODE_UNKNOWN);
  CHECK_EQ(rig.esp.count("AT+DSCAPS?"), (size_t)2);
  rig.runMs(kPollMs * 5);
  CHECK_EQ(rig.esp.count("AT+XBOXBEGIN"), (size_t)1);   // the first OFF frame: at once
  // Scanning: no more BEGIN. OFF again (BLE client died): at most one per LINK_BEGIN_RETRY_MS.
  rig.esp.hook = [](const std::string& req, std::string& reply) {
    if (req != "AT+XBOXBEGIN") return false;
    reply = "+XBOXBEGIN: 0\r\nOK\r\n";   // accepted, but the client stays OFF
    return true;
  };
  rig.esp.st.state = XBOX_R4_OFF;
  rig.runMs(LINK_BEGIN_RETRY_MS * 2 + LINK_BEGIN_RETRY_MS / 2);
  const size_t begins = rig.esp.count("AT+XBOXBEGIN");
  CHECK(begins >= 3u && begins <= 4u);
  CHECK_EQ(rig.link.stats().beginsSent, (uint32_t)begins);
  CHECK_EQ(rig.esp.count("AT+DSS"), (size_t)0);
  CHECK_EQ(rig.esp.count("AT+DST"), (size_t)0);
  CHECK(!rig.link.connected());
  // The pad connects: legacy BLE frames are obeyed.
  rig.esp.hook = nullptr;
  rig.esp.st.state = XBOX_R4_CONNECTED;
  CHECK(rig.runUntil([&] { return rig.link.connected(); }, kPollMs * 3));
  CHECK_EQ(rig.link.status().activeSource, (uint8_t)DS_LINK_SRC_BLE);
  CHECK(!rig.link.wantsWifiTelemetry());
  expectNoBlockingWrites();
}

TEST(legacy_firmware_with_wifi_wanted_never_connects_and_logs_once) {
  Rig rig;
  rig.esp.fw = FakeEsp::Fw::LEGACY;
  rig.esp.st.state = XBOX_R4_OFF;   // would earn an XBOXBEGIN in BT mode
  rig.link.setWantMode(DS_RADIO_WIFI);
  rig.runMs(MS_PER_S * 3);
  CHECK(rig.link.atPhase() == ControlLink::AtPhase::RUN_LEGACY);
  CHECK(!rig.link.connected());   // the fake's pad says connected: still not obeyed
  CHECK_EQ(rig.link.stats().radioFrames, 0u);
  CHECK(rig.link.status().wifiUnavailable);
  rig.drainUsb();
  CHECK_EQ(rig.countText("WiFi wanted, but the ESP32 runs legacy firmware"), (size_t)1);
  CHECK_EQ(rig.esp.count("AT+XBOXBEGIN"), (size_t)0);
  CHECK(rig.esp.count("AT+XBOXSTATE?") > 0u);   // still polled: espAlive stays meaningful
  CHECK(rig.link.status().espAlive);
  // Back to BT: it connects.
  rig.link.setWantMode(DS_RADIO_BLE);
  CHECK(rig.runUntil([&] { return rig.link.connected(); }, kPollMs * 3));
  CHECK(!rig.link.status().wifiUnavailable);
}

TEST(another_ds_proto_version_is_treated_as_legacy) {
  Rig rig;
  rig.esp.proto = DS_PROTO_VERSION + 1;
  CHECK(rig.runUntil([&] { return rig.link.atPhase() == ControlLink::AtPhase::RUN_LEGACY; }, 1000));
  CHECK(rig.link.status().espLegacy);
  rig.drainUsb();
  CHECK_EQ(rig.countText("treated as legacy"), (size_t)1);
  rig.runMs(kPollMs * 5);
  CHECK_EQ(rig.esp.count("AT+DSS"), (size_t)0);
}

TEST(no_wifi_capability_means_no_wifi_link) {
  Rig rig;
  rig.esp.caps = DS_CAP_RUMBLE | DS_CAP_TELEMETRY;
  rig.link.setWantMode(DS_RADIO_WIFI);
  rig.runMs(MS_PER_S);
  CHECK(rig.link.status().wifiUnavailable);
  CHECK(!rig.link.connected());
  rig.drainUsb();
  CHECK_EQ(rig.countText("firmware has no WiFi"), (size_t)1);
}

// =============================================================================================
// WiFi mode: the radio switch and AT+DST
// =============================================================================================

namespace {
// Boots in BLE, switches to WiFi, waits for the restarted ESP32 to report it.
bool toWifi(Rig& rig) {
  if (!rig.runToDs()) return false;
  rig.link.setWantMode(DS_RADIO_WIFI);
  return rig.runUntil([&] {
    return rig.link.status().espMode == DS_ESP_MODE_WIFI && rig.link.connected();
  }, MS_PER_S * 2);
}
}  // namespace

TEST(mode_switch_drops_the_radio_at_once_and_returns_in_the_new_mode) {
  Rig rig;
  CHECK(rig.runToDs());
  CHECK_EQ(rig.link.status().activeSource, (uint8_t)DS_LINK_SRC_BLE);
  rig.link.setWantMode(DS_RADIO_WIFI);
  rig.step(100);
  CHECK(!rig.link.connected());   // the old BLE pad is not obeyed for a single loop more
  bool everConnectedBle = false;
  rig.everyLoop = [&] {
    everConnectedBle = everConnectedBle || (rig.link.connected() && rig.link.frame().source == DS_LINK_SRC_BLE);
  };
  CHECK(rig.runUntil([&] { return rig.link.status().espMode == DS_ESP_MODE_WIFI && rig.link.connected(); },
                     MS_PER_S * 2));
  CHECK(!everConnectedBle);
  CHECK(rig.esp.count("AT+DSS=1,") >= 2u);   // the request (-> restarting) and the new session
  CHECK_EQ(rig.link.status().activeSource, (uint8_t)DS_LINK_SRC_WIFI);
  CHECK_EQ(rig.link.status().wifiStatus, (uint8_t)DS_WIFI_STATUS_RELAY_HEARD);
  CHECK_EQ(rig.link.stats().espRestarts, 1u);
  CHECK(rig.link.wantsWifiTelemetry());
}

TEST(dst_only_with_wifi_at_telemetry_wifi_hz_and_exact_bytes) {
  Rig rig;
  CHECK(rig.runToDs());
  const std::string f0 = telemetryFrame(1);
  CHECK(!rig.link.offerTelemetry((const uint8_t*)f0.data(), f0.size()));   // BLE: no WiFi path
  rig.runMs(MS_PER_S / 2);
  CHECK_EQ(rig.esp.count("AT+DST="), (size_t)0);
  CHECK(toWifi(rig));
  // Telemetry offers at TELEMETRY_USB_HZ, as Telemetry does.
  uint8_t seq = 0;
  uint32_t lastOffer = hal::nowMs();
  std::string lastOffered;
  rig.everyLoop = [&] {
    if ((uint32_t)(hal::nowMs() - lastOffer) < MS_PER_S / TELEMETRY_USB_HZ) return;
    lastOffer = hal::nowMs();
    lastOffered = telemetryFrame(++seq);
    CHECK(rig.link.offerTelemetry((const uint8_t*)lastOffered.data(), lastOffered.size()));
  };
  rig.runMs(MS_PER_S);
  const double t0 = rig.now();
  const uint32_t to0 = rig.link.stats().pollTimeouts;
  const uint32_t sent0 = rig.link.stats().telemetrySent;
  rig.runMs(MS_PER_S * 2);
  const size_t dst = rig.esp.countSince("AT+DST=", t0);
  CHECK(dst >= (size_t)(TELEMETRY_WIFI_HZ * 2 - 1) && dst <= (size_t)(TELEMETRY_WIFI_HZ * 2 + 1));
  CHECK_EQ(rig.link.stats().pollTimeouts, to0);
  CHECK(rig.link.stats().telemetrySent - sent0 >= (uint32_t)(TELEMETRY_WIFI_HZ * 2 - 2));
  CHECK_EQ(rig.link.stats().telemetryNotSent, 0u);
  // Every line is exactly one valid TelemetryPacket; the ESP32 got a recent offer.
  CHECK_EQ(rig.esp.last("AT+DST=").size() + 2, (size_t)DS_AT_DST_LINE_LEN);
  CHECK_EQ(rig.esp.lastDst.size(), (size_t)DS_TELEMETRY_FRAME_LEN);
  ds_telemetry_packet_t p;
  CHECK(ds_telemetry_decode(rig.esp.lastDst.data(), rig.esp.lastDst.size(), &p));
  CHECK((uint8_t)(seq - p.seq) <= (uint8_t)(TELEMETRY_USB_HZ / TELEMETRY_WIFI_HZ + 1));
  // The control polls keep going between DST lines.
  const size_t polls = rig.esp.countSince("AT+XBOXSTATE?", t0);
  CHECK(polls >= (size_t)(CONTROL_PACKET_HZ * 2 * 7 / 10));
  CHECK(rig.link.connected());
  expectNoBlockingWrites();
}

// The paced writer hands the UART one 16-byte burst per service() call: with a slow loop
// (2 ms here, an LCD write plus a long stall) the 229-byte line takes far longer than its
// 20 ms of wire time, and only a deadline counted from the last byte keeps the reply valid.
TEST(dst_deadline_counts_from_the_last_byte_on_the_wire) {
  Rig rig;
  CHECK(toWifi(rig));
  const uint32_t slowLoopUs = 2 * US_PER_MS;
  double lateMs = AT_REPLY_TIMEOUT_MS - 4;   // after the line's last byte reached the ESP32
  rig.esp.delayFor = [&](const std::string& req) {
    return req.rfind("AT+DST=", 0) == 0 ? lateMs * US_PER_MS - rig.esp.processUs : 0.0;
  };
  const std::string f = telemetryFrame(5);
  const uint32_t to0 = rig.link.stats().pollTimeouts;
  const uint32_t sent0 = rig.link.stats().telemetrySent;
  CHECK(rig.link.offerTelemetry((const uint8_t*)f.data(), f.size()));
  CHECK(rig.runUntil([&] { return rig.link.stats().telemetrySent > sent0; }, MS_PER_S / 2, slowLoopUs));
  CHECK_EQ(rig.link.stats().pollTimeouts, to0);   // ~26 ms of line + 16 ms of reply: no timeout
  // Later than AT_REPLY_TIMEOUT_MS after the last byte: a timeout, and the link recovers.
  lateMs = AT_REPLY_TIMEOUT_MS + 6;
  CHECK(rig.link.offerTelemetry((const uint8_t*)f.data(), f.size()));
  CHECK(rig.runUntil([&] { return rig.link.stats().pollTimeouts > to0; }, MS_PER_S / 2, slowLoopUs));
  rig.esp.delayFor = nullptr;
  CHECK(rig.runUntil([&] { return rig.link.connected(); }, MS_PER_S / 2));
  expectNoBlockingWrites();
}

TEST(offer_telemetry_rejects_anything_but_one_valid_frame) {
  Rig rig;
  CHECK(toWifi(rig));
  std::string f = telemetryFrame(3);
  CHECK(!rig.link.offerTelemetry((const uint8_t*)f.data(), f.size() - 1));
  CHECK(!rig.link.offerTelemetry(nullptr, f.size()));
  f[10] ^= 0x01;   // CRC no longer matches
  CHECK(!rig.link.offerTelemetry((const uint8_t*)f.data(), f.size()));
  const std::string c = controlFrame(1, 0, 0, 0, 0, 0);
  CHECK(!rig.link.offerTelemetry((const uint8_t*)c.data(), c.size()));
  CHECK_EQ(rig.link.stats().telemetryOffered, 0u);
}

// =============================================================================================
// USB: deframer, shell lines, source priority, StatusPackets
// =============================================================================================

TEST(usb_frames_survive_byte_at_a_time_random_splits_and_garbage) {
  Rig rig;
  rig.esp.powered = false;   // USB only
  std::mt19937 rng(7);
  std::string stream;
  const int kFrames = 300;
  for (int i = 0; i < kFrames; i++) {
    if (i % 7 == 0) stream += std::string("\xff\x00\xa5\x0b\x5a\x01junk", 10);   // garbage, fake starts
    stream += controlFrame(DS_CTRL_FLAG_CONNECTED, (int16_t)(i - 150), (int16_t)(150 - i), (uint16_t)i,
                           (uint16_t)(i & DS_BTN_ALL), (uint8_t)i);
  }
  size_t pos = 0;
  int mode = 0;
  while (pos < stream.size()) {
    size_t n = mode == 0 ? 1 : 1 + rng() % 40;
    if (pos > stream.size() / 2) mode = 1;
    n = std::min(n, stream.size() - pos);
    hal_host::usbInject((const uint8_t*)stream.data() + pos, n);
    pos += n;
    rig.step(100);
  }
  rig.step(100);
  CHECK_EQ(rig.link.stats().usbFrames, (uint32_t)kFrames);
  CHECK_EQ(rig.link.stats().crcErrors, 0u);
  CHECK(rig.link.connected());
  CHECK_EQ(rig.link.frame().source, (uint8_t)DS_LINK_SRC_USB);
  CHECK_EQ(rig.link.frame().seq, (uint8_t)(kFrames - 1));
  CHECK_EQ(rig.link.frame().rx, (int16_t)(kFrames - 1 - 150));
  CHECK_EQ(rig.link.frame().rt, (uint16_t)(kFrames - 1));
  CHECK_EQ(rig.link.stats().seqGaps, 0u);
  // A corrupted frame is counted, not obeyed.
  std::string bad = controlFrame(DS_CTRL_FLAG_CONNECTED, 400, 0, 0, DS_BTN_A, 77);
  bad[4] ^= 0x10;
  hal_host::usbInject(bad);
  rig.step(100);
  CHECK_EQ(rig.link.stats().crcErrors, 1u);
  CHECK(rig.link.frame().seq != 77);
}

TEST(shell_lines_and_frames_interleave_without_cutting_each_other) {
  Rig rig;
  rig.esp.powered = false;
  // Frames whose bytes contain '\n' (the len byte 0x0A, and rx = 0x0A0A) and printable text.
  const std::string fA = controlFrame(DS_CTRL_FLAG_CONNECTED, 0x0A0A, 0x7473, 0x0A61, 0x0A0A, 1);
  const std::string fB = controlFrame(DS_CTRL_FLAG_CONNECTED, 0x2020, 0x0A0D, 0x6C66, 0x0A79, 2);
  const std::string stream = "status\r\n" + fA + "fly on\n" + fB + "log 3\n" + fA.substr(0, 5) +
                             fA.substr(5) + "Zap Z\n" + fB + "pan -12.5\n";
  for (int round = 0; round < 3; round++) {
    std::mt19937 rng(round);
    size_t pos = 0;
    while (pos < stream.size()) {
      size_t n = round == 0 ? 1 : round == 1 ? stream.size() : 1 + rng() % 6;
      n = std::min(n, stream.size() - pos);
      hal_host::usbInject((const uint8_t*)stream.data() + pos, n);
      pos += n;
      rig.step(100);
    }
    std::vector<std::string> lines;
    char buf[SHELL_LINE_MAX + 1];
    while (rig.link.takeShellLine(buf, sizeof buf)) lines.push_back(buf);
    const std::vector<std::string> want = {"status", "fly on", "log 3", "Zap Z", "pan -12.5"};
    // The queue holds SHELL_QUEUE_LINES; take as we go is Debug's job, so check the head here.
    const size_t kept = std::min(want.size(), (size_t)SHELL_QUEUE_LINES);
    CHECK_EQ(lines.size(), kept);
    for (size_t i = 0; i < lines.size() && i < kept; i++) CHECK_EQ(lines[i], want[i]);
  }
  CHECK_EQ(rig.link.stats().usbFrames, 12u);   // four frames per round
  CHECK_EQ(rig.link.stats().crcErrors, 0u);
  CHECK_EQ(rig.link.stats().shellJunkLines, 0u);
  CHECK_EQ(rig.link.stats().shellOverflows, 0u);
}

TEST(shell_lines_are_handed_over_in_order_while_taken_every_loop) {
  Rig rig;
  rig.esp.powered = false;
  std::vector<std::string> got;
  rig.everyLoop = [&] {
    char buf[SHELL_LINE_MAX + 1];
    while (rig.link.takeShellLine(buf, sizeof buf)) got.push_back(buf);
  };
  std::string stream;
  for (int i = 0; i < 40; i++) {
    stream += "cmd " + std::to_string(i) + "\n";
    stream += controlFrame(0, 0, 0, 0, 0, (uint8_t)i);
  }
  for (size_t pos = 0; pos < stream.size(); pos += 11) {
    hal_host::usbInject((const uint8_t*)stream.data() + pos, std::min<size_t>(11, stream.size() - pos));
    rig.step(150);
  }
  rig.step(150);
  CHECK_EQ(got.size(), (size_t)40);
  for (size_t i = 0; i < got.size(); i++) CHECK_EQ(got[i], "cmd " + std::to_string(i));
  CHECK_EQ(rig.link.stats().shellLines, 40u);
  CHECK_EQ(rig.link.stats().shellQueueFull, 0u);
}

TEST(shell_overflow_junk_and_full_queue_are_dropped_and_counted) {
  Rig rig;
  rig.esp.powered = false;
  const std::string exact(SHELL_LINE_MAX, 'a');
  const std::string tooLong(SHELL_LINE_MAX + 1, 'b');
  hal_host::usbInject(exact + "\n" + tooLong + "\n" + std::string("x\x01y\n", 4) + "\n\r\n" + "ok\n");
  rig.step(100);
  char buf[SHELL_LINE_MAX + 1];
  CHECK(rig.link.takeShellLine(buf, sizeof buf));
  CHECK_EQ(std::string(buf), exact);
  CHECK(rig.link.takeShellLine(buf, sizeof buf));
  CHECK_EQ(std::string(buf), std::string("ok"));
  CHECK(!rig.link.takeShellLine(buf, sizeof buf));
  CHECK_EQ(rig.link.stats().shellOverflows, 1u);
  CHECK_EQ(rig.link.stats().shellJunkLines, 1u);
  // Nobody takes: the first SHELL_QUEUE_LINES wait, the rest are dropped and counted.
  std::string many;
  for (int i = 0; i < SHELL_QUEUE_LINES + 2; i++) many += "l" + std::to_string(i) + "\n";
  hal_host::usbInject(many);
  rig.step(100);
  CHECK_EQ(rig.link.shellLinesWaiting(), (size_t)SHELL_QUEUE_LINES);
  CHECK_EQ(rig.link.stats().shellQueueFull, 2u);
  char small[3];
  CHECK(rig.link.takeShellLine(small, sizeof small));   // cut to the caller's buffer
  CHECK_EQ(std::string(small), std::string("l0"));
  CHECK(rig.link.takeShellLine(buf, sizeof buf));
  CHECK_EQ(std::string(buf), std::string("l1"));
  CHECK(!rig.link.takeShellLine(buf, 0));
}

// KEY SAFETY PROPERTY (USB): the bench override expires exactly at LINK_TIMEOUT_MS.
TEST(usb_link_loss_is_seen_within_link_timeout) {
  Rig rig;
  rig.esp.powered = false;
  uint8_t seq = 0;
  bool sending = true;
  uint32_t lastSent = 0;
  rig.everyLoop = [&] {
    if (!sending || (uint32_t)(hal::nowMs() - lastSent) < kPollMs) return;
    lastSent = hal::nowMs();
    hal_host::usbInject(controlFrame(DS_CTRL_FLAG_CONNECTED, 300, -300, 900, DS_BTN_LB, seq++));
  };
  rig.runMs(MS_PER_S);
  CHECK(rig.link.connected());
  CHECK(rig.link.stats().ctrlRxHz >= CONTROL_PACKET_HZ - 2);
  CHECK_EQ(rig.link.frame().buttons, (uint16_t)DS_BTN_LB);
  sending = false;
  rig.step(US_PER_MS / 4);   // the frame injected by the last loop is read now
  const uint32_t last = rig.link.status().lastFrameMs;
  uint32_t lostAt = 0;
  for (int i = 0; i < 2000 && !lostAt; i++) {
    rig.step(US_PER_MS / 4);
    if (!rig.link.connected()) lostAt = hal::nowMs();
  }
  CHECK(lostAt != 0u);
  CHECK_EQ((uint32_t)(lostAt - last), (uint32_t)LINK_TIMEOUT_MS);
  CHECK_EQ(rig.link.frame().rt, (uint16_t)0);
  CHECK_EQ(rig.link.frame().buttons, (uint16_t)0);
  CHECK_EQ(rig.link.frame().rx, (int16_t)0);
  CHECK_EQ(rig.link.status().activeSource, (uint8_t)DS_LINK_SRC_NONE);
}

TEST(usb_wins_over_the_radio_and_a_switch_while_connected_is_a_link_loss) {
  Rig rig;
  CHECK(rig.runToDs());
  CHECK(!rig.link.takeSourceChanged());
  uint8_t seq = 0;
  bool sending = true;
  uint8_t flags = DS_CTRL_FLAG_CONNECTED;
  uint32_t lastSent = 0;
  rig.everyLoop = [&] {
    if (!sending || (uint32_t)(hal::nowMs() - lastSent) < kPollMs) return;
    lastSent = hal::nowMs();
    hal_host::usbInject(controlFrame(flags, 5, 5, 0, 0, seq++));
  };
  rig.step(200);
  rig.step(200);
  CHECK_EQ(rig.link.status().activeSource, (uint8_t)DS_LINK_SRC_USB);
  CHECK(rig.link.takeSourceChanged());
  CHECK(!rig.link.takeSourceChanged());   // cleared by reading
  CHECK_EQ(rig.link.stats().sourceChanges, 1u);
  rig.runMs(MS_PER_S / 2);
  CHECK(rig.link.connected());
  CHECK_EQ(rig.link.frame().rx, (int16_t)5);
  // USB stops: after LINK_TIMEOUT_MS the radio takes over, and that is a link loss too.
  sending = false;
  CHECK(rig.runUntil([&] { return rig.link.status().activeSource == DS_LINK_SRC_BLE; }, LINK_TIMEOUT_MS * 2, 100));
  CHECK(rig.link.takeSourceChanged());
  CHECK_EQ(rig.link.stats().sourceChanges, 2u);
  // A bench console holding the port with connected = 0 still overrides the radio.
  flags = 0;
  sending = true;
  rig.runMs(kPollMs * 3);
  CHECK_EQ(rig.link.status().activeSource, (uint8_t)DS_LINK_SRC_USB);
  CHECK(!rig.link.connected());
  // USB control disabled: frames are counted, never obeyed.
  rig.link.setUsbControlEnabled(false);
  flags = DS_CTRL_FLAG_CONNECTED;
  rig.runMs(kPollMs * 5);
  CHECK_EQ(rig.link.status().activeSource, (uint8_t)DS_LINK_SRC_BLE);
  CHECK(rig.link.stats().usbIgnored >= 3u);
  CHECK(!rig.link.status().usbControlEnabled);
}

TEST(usb_seq_gaps_are_counted_and_a_restarted_sender_is_not) {
  Rig rig;
  rig.esp.powered = false;
  const uint8_t seqs[] = {10, 11, 12, 15, 16, 20, 0, 1};   // gaps 2 and 3; then a restart
  for (uint8_t s : seqs) {
    hal_host::usbInject(controlFrame(DS_CTRL_FLAG_CONNECTED, 0, 0, 0, 0, s));
    rig.step(US_PER_MS);
  }
  CHECK_EQ(rig.link.stats().seqGaps, 5u);
  // After a timeout the next frame starts a new session: no gap counted.
  rig.runMs(LINK_TIMEOUT_MS + kPollMs);
  hal_host::usbInject(controlFrame(DS_CTRL_FLAG_CONNECTED, 0, 0, 0, 0, 100));
  rig.step(US_PER_MS);
  CHECK_EQ(rig.link.stats().seqGaps, 5u);
}

TEST(usb_status_packets_on_change_and_every_heartbeat) {
  Rig rig;
  rig.esp.powered = false;
  const uint32_t drainMs = 20;   // a log line + a StatusPacket, one byte per service() call
  rig.runMs(drainMs);
  auto sp = rig.statusPackets();
  CHECK_EQ(sp.size(), (size_t)1);   // at once after boot
  rig.runMs(STATUS_HEARTBEAT_MS * 4 + STATUS_HEARTBEAT_MS / 2);
  sp = rig.statusPackets();
  CHECK_EQ(sp.size(), (size_t)5);   // + one per heartbeat
  rig.link.setTurretState(DS_STATE_PAIRING);
  rig.link.setWantMode(DS_RADIO_WIFI);
  rig.step(100);
  CHECK_EQ(rig.link.stats().statusSent, 6u);   // queued on the very next update
  rig.runMs(drainMs);
  sp = rig.statusPackets();
  CHECK_EQ(sp.size(), (size_t)6);
  CHECK_EQ(sp.back().state, (uint8_t)DS_STATE_PAIRING);
  CHECK_EQ(sp.back().wantMode, (uint8_t)DS_RADIO_WIFI);
  rig.link.requestRumble(200, 120, 80);
  rig.runMs(drainMs);
  sp = rig.statusPackets();
  CHECK_EQ(sp.back().rumbleSeq, (uint8_t)1);
  CHECK_EQ(sp.back().rumbleStrong, (uint8_t)200);
  CHECK_EQ(sp.back().rumbleWeak, (uint8_t)120);
  CHECK_EQ(sp.back().rumbleMs, (uint16_t)80);
  // The heartbeat repeats the same rumbleSeq (so the relay does not rumble again).
  rig.runMs(STATUS_HEARTBEAT_MS + kPollMs);
  sp = rig.statusPackets();
  CHECK_EQ(sp.back().rumbleSeq, (uint8_t)1);
  CHECK_EQ(rig.link.stats().statusSent, (uint32_t)sp.size());
  expectNoBlockingWrites();
}

TEST(status_packets_survive_a_full_usb_queue) {
  Rig rig;
  rig.esp.powered = false;
  // Fill the USB queue so the status packet is refused...
  const std::string big(DEBUG_TX_BUFFER_BYTES - 4, 'x');
  CHECK(hal::usbWrite((const uint8_t*)big.data(), big.size()));
  rig.link.setTurretState(DS_STATE_DISARMED);
  rig.link.update(hal::nowMs(), hal::nowUs());   // no service(): the queue stays full
  rig.link.update(hal::nowMs(), hal::nowUs());
  CHECK_EQ(hal::usbTxDropped(), 1u);              // tried once, not on every update
  // ...and goes out once there is room.
  rig.runMs(DEBUG_TX_BUFFER_BYTES * 87 / US_PER_MS + kPollMs * 2, 50);
  auto sp = rig.statusPackets();
  CHECK(!sp.empty());
  if (!sp.empty()) CHECK_EQ(sp.back().state, (uint8_t)DS_STATE_DISARMED);
}

// =============================================================================================
// Faults and time
// =============================================================================================

TEST(garbage_on_the_link_rx_keeps_the_link_up) {
  Rig rig;
  CHECK(rig.runToDs());
  std::mt19937 rng(99);
  bool everLost = false;
  rig.everyLoop = [&] {
    if (rng() % 125 == 0) {   // ~40 garbage bytes/s at 200 us loops
      const uint8_t g = (uint8_t)rng();
      hal_host::linkInject(&g, 1);
    }
    everLost = everLost || !rig.link.connected();
  };
  rig.runMs(MS_PER_S * 5);
  CHECK(!everLost);
  CHECK(rig.link.stats().junkBytes > 0u);
  CHECK(rig.link.stats().radioFrames >= (uint32_t)(CONTROL_PACKET_HZ * 4));
}

TEST(late_replies_never_resurrect_an_old_frame) {
  Rig rig;
  CHECK(rig.runToDs());
  // Every 25th poll is answered 30 ms late (past its deadline); the pad's seq rises 1 per poll.
  int n = 0;
  rig.esp.delayFor = [&](const std::string& req) {
    if (req != "AT+XBOXSTATE?") return 0.0;
    rig.esp.st.seq++;
    return (++n % 25 == 0) ? 30.0 * US_PER_MS : 0.0;
  };
  int lastSeq = -1, backwards = 0;
  rig.everyLoop = [&] {
    if (!rig.link.connected()) return;
    const int s = rig.link.frame().seq;
    if (lastSeq >= 0 && (uint8_t)(s - lastSeq) > 128) backwards++;
    lastSeq = s;
  };
  rig.runMs(MS_PER_S * 3);
  CHECK_EQ(backwards, 0);
  CHECK(rig.link.stats().pollTimeouts >= 3u);
  CHECK(rig.link.connected());
}

TEST(micros_wrap_is_harmless) {
  // Start 300 ms before micros() wraps at 2^32 us.
  Rig rig(((uint64_t)UINT32_MAX + 1u) - 300u * US_PER_MS);
  CHECK(rig.runToDs());
  const uint32_t to0 = rig.link.stats().pollTimeouts;
  rig.runMs(MS_PER_S);
  CHECK((uint64_t)hal_host::timeUs() > (uint64_t)UINT32_MAX);   // crossed it
  CHECK_EQ(rig.link.stats().pollTimeouts, to0);
  CHECK(rig.link.connected());
  CHECK(rig.link.stats().ctrlRxHz >= CONTROL_PACKET_HZ - 3);
  expectNoBlockingWrites();
}

TEST(millis_wrap_is_harmless) {
  // Start 300 ms before millis() wraps at 2^32 ms (49.7 days).
  Rig rig((((uint64_t)UINT32_MAX + 1u) - 300u) * US_PER_MS);
  CHECK(rig.runToDs());
  const uint32_t to0 = rig.link.stats().pollTimeouts;
  rig.runMs(MS_PER_S);
  CHECK(hal::nowMs() < 1000u);   // crossed it
  CHECK_EQ(rig.link.stats().pollTimeouts, to0);
  CHECK(rig.link.connected());
  CHECK_EQ(rig.link.stats().espSilences, 0u);
}

TEST(names_for_logs) {
  CHECK_EQ(std::string(ControlLink::phaseName(ControlLink::AtPhase::RUN_DS)), std::string("ds"));
  CHECK_EQ(std::string(ControlLink::requestName(ControlLink::Request::DST)), std::string("AT+DST"));
}

TESTKIT_MAIN()

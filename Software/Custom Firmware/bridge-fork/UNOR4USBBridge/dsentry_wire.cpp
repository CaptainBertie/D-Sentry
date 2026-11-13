/*
 * dsentry_wire.cpp -- see dsentry_wire.h. Portable C++17: compiled into the ESP32-S3 sketch
 * and, unchanged, into the host tests in D-SentryCode/tests/esp32.
 *
 * SPDX-License-Identifier: MIT
 * Copyright (c) 2026 the D-Sentry project
 * The combined firmware is governed by the AGPL-3.0 (see DECISIONS.md D04).
 */
#include "dsentry_wire.h"

#include <string.h>

static_assert(WIFI_STA_RETRY_MS >= WIFI_STA_CONNECT_TIMEOUT_MS,
              "WIFI_STA_RETRY_MS must be at least WIFI_STA_CONNECT_TIMEOUT_MS: a new WiFi.begin() "
              "sooner than a join can take would abort every join");

namespace dsentry {

namespace {

/* Xbox BLE rumble output report (see dsentry_wire.h). Protocol facts, not tunables. */
constexpr size_t kRumbleEnable = 0;          /* byte offsets */
constexpr size_t kRumbleStrong = 3;          // lit-ok: HID report byte 3 = strong motor power
constexpr size_t kRumbleWeak = 4;            // lit-ok: HID report byte 4 = weak motor power
constexpr size_t kRumbleOnTime = 5;          // lit-ok: HID report byte 5 = on time, 10 ms units
constexpr size_t kRumbleRepeat = 7;          // lit-ok: HID report byte 7 = repeat count
constexpr uint8_t kRumbleEnableWeak = 0x01;  /* enable bits (Linux ENABLE_WEAK / ENABLE_STRONG) */
constexpr uint8_t kRumbleEnableStrong = 0x02;  // lit-ok: HID enable bit 1 = strong motor
constexpr uint32_t kRumblePowerMax = 100;    // lit-ok: HID motor power is 0..100 (hid-microsoft.c)
constexpr uint32_t kRumbleTickMs = 10;       // lit-ok: HID on time counts 10 ms units
constexpr uint32_t kRumbleTicksMax = UINT8_MAX;  /* the on-time and repeat fields are u8 */

uint8_t rumble_power(uint8_t v) {
  /* 0..255 -> 0..100, rounded to nearest */
  return (uint8_t)(((uint32_t)v * kRumblePowerMax + (BYTE_MASK >> 1)) / BYTE_MASK);
}

void put_u16(uint8_t* at, uint16_t v) {
  at[0] = (uint8_t)(v & BYTE_MASK);
  at[1] = (uint8_t)(v >> BITS_PER_BYTE);
}

uint16_t axis_to_hid(int16_t v) {
  int32_t x = v;
  if (x < -(int32_t)JOY_AXIS_RANGE) x = -(int32_t)JOY_AXIS_RANGE;
  if (x > (int32_t)JOY_AXIS_RANGE - 1) x = (int32_t)JOY_AXIS_RANGE - 1;
  return (uint16_t)(x * ((int32_t)1 << XSTATE_AXIS_SHIFT) + (int32_t)XSTATE_AXIS_CENTRE);
}

uint16_t trigger_to_hid(uint16_t v) {
  return v > (uint16_t)TRIGGER_AXIS_RANGE ? (uint16_t)TRIGGER_AXIS_RANGE : v;
}

uint8_t btn0_of(uint16_t b) {
  uint8_t out = 0;
  if (b & DS_BTN_A) out |= XSTATE_BTN0_A;
  if (b & DS_BTN_B) out |= XSTATE_BTN0_B;
  if (b & DS_BTN_X) out |= XSTATE_BTN0_X;
  if (b & DS_BTN_Y) out |= XSTATE_BTN0_Y;
  if (b & DS_BTN_LB) out |= XSTATE_BTN0_LB;
  if (b & DS_BTN_RB) out |= XSTATE_BTN0_RB;
  return out;
}

uint8_t btn1_of(uint16_t b) {
  uint8_t out = 0;
  if (b & DS_BTN_BACK) out |= XSTATE_BTN1_VIEW;
  if (b & DS_BTN_START) out |= XSTATE_BTN1_MENU;
  return out;
}

uint32_t store_check(const ModeStore& s) {
  return ~(s.magic ^ s.request ^ s.lastBootId);
}

bool is_radio_mode(uint32_t m) {
  return m == DS_ESP_MODE_BLE || m == DS_ESP_MODE_WIFI;
}

}  /* namespace */

/* ---- AT arguments and replies --------------------------------------------------------- */

bool parse_uint(const std::string& text, uint32_t max, uint32_t* out) {
  if (text.empty()) return false;
  uint32_t v = 0;
  for (char c : text) {
    if (c < '0' || c > '9') return false;
    uint32_t d = (uint32_t)(c - '0');
    if (d > max || v > (max - d) / DEC_BASE) return false;   /* v * 10 + d would exceed max */
    v = v * DEC_BASE + d;
  }
  *out = v;
  return true;
}

bool parse_dss(const std::vector<std::string>& args, DssArgs* out) {
  if (args.size() != DS_AT_DSS_ARGC) return false;
  uint32_t want, seq, strong, weak, ms, state;
  if (!parse_uint(args[DS_AT_DSS_ARG_WANT_MODE], DS_RADIO_COUNT - 1, &want)) return false;
  if (!parse_uint(args[DS_AT_DSS_ARG_RUMBLE_SEQ], UINT8_MAX, &seq)) return false;
  if (!parse_uint(args[DS_AT_DSS_ARG_STRONG], UINT8_MAX, &strong)) return false;
  if (!parse_uint(args[DS_AT_DSS_ARG_WEAK], UINT8_MAX, &weak)) return false;
  if (!parse_uint(args[DS_AT_DSS_ARG_RUMBLE_MS], UINT16_MAX, &ms)) return false;
  if (!parse_uint(args[DS_AT_DSS_ARG_STATE], UINT8_MAX, &state)) return false;
  out->wantMode = (uint8_t)want;
  out->rumbleSeq = (uint8_t)seq;
  out->strong = (uint8_t)strong;
  out->weak = (uint8_t)weak;
  out->rumbleMs = (uint16_t)ms;
  out->state = (uint8_t)state;
  return true;
}

uint8_t caps_bits() {
  return (uint8_t)(DS_CAP_RUMBLE | DS_CAP_WIFI | DS_CAP_MODE | DS_CAP_TELEMETRY);
}

std::string format_caps_reply() {
  return std::to_string((unsigned)DS_PROTO_VERSION) + "," + std::to_string((unsigned)caps_bits());
}

std::string format_dss_reply(uint8_t espMode, uint8_t bootId, uint8_t wifiStatus) {
  return std::to_string((unsigned)espMode) + "," + std::to_string((unsigned)bootId) + "," +
         std::to_string((unsigned)wifiStatus);
}

std::string format_dst_reply(uint8_t sent) {
  return std::to_string((unsigned)sent);
}

size_t parse_dst(const std::string& hex, uint8_t out[DS_TELEMETRY_FRAME_LEN]) {
  if (hex.size() != DS_AT_DST_HEX_LEN) return 0;
  uint8_t frame[DS_TELEMETRY_FRAME_LEN];
  if (ds_hex_decode(hex.data(), hex.size(), frame, sizeof frame) != DS_TELEMETRY_FRAME_LEN) return 0;
  if (!ds_frame_check(frame, sizeof frame, DS_PKT_TELEMETRY)) return 0;
  memcpy(out, frame, sizeof frame);
  return DS_TELEMETRY_FRAME_LEN;
}

/* ---- which radio runs ----------------------------------------------------------------- */

ModeAction decide_mode(uint8_t wantMode, const Radios& r) {
  if (wantMode >= DS_RADIO_COUNT) return ModeAction::INVALID;
  if (r.restartPending) return ModeAction::RESTART;
  if (wantMode == DS_RADIO_BLE) {
    if (r.dsWifiRunning || r.otherWifiRunning || r.hciUsed) return ModeAction::RESTART;
    return r.bleRunning ? ModeAction::ALREADY : ModeAction::START_BLE;
  }
  if (r.bleRunning || r.hciUsed || r.otherWifiRunning) return ModeAction::RESTART;
  return r.dsWifiRunning ? ModeAction::ALREADY : ModeAction::START_WIFI;
}

uint8_t esp_mode_of(const Radios& r) {
  if (r.restartPending) return DS_ESP_MODE_RESTARTING;
  if (r.dsWifiRunning) return DS_ESP_MODE_WIFI;
  if (r.bleRunning) return DS_ESP_MODE_BLE;
  return DS_ESP_MODE_NONE;
}

/* ---- rumble ----------------------------------------------------------------------------- */

bool RumbleSeqTracker::is_new(uint8_t seq) {
  if (!have_) {
    have_ = true;
    last_ = seq;
    return false;
  }
  if (seq == last_) return false;
  last_ = seq;
  return true;
}

void rumble_report(uint8_t strong, uint8_t weak, uint16_t ms, uint8_t out[XBOX_RUMBLE_REPORT_LEN]) {
  memset(out, 0, XBOX_RUMBLE_REPORT_LEN);
  out[kRumbleEnable] = (uint8_t)(kRumbleEnableWeak | kRumbleEnableStrong);
  if (ms == 0) return;   /* both grip motors to power 0: stop */
  const uint32_t ticks = ((uint32_t)ms + kRumbleTickMs - 1) / kRumbleTickMs;
  const uint32_t plays = (ticks + kRumbleTicksMax - 1) / kRumbleTicksMax;   /* >= 1 */
  out[kRumbleStrong] = rumble_power(strong);
  out[kRumbleWeak] = rumble_power(weak);
  out[kRumbleOnTime] = (uint8_t)((ticks + plays - 1) / plays);
  out[kRumbleRepeat] = (uint8_t)(plays - 1);
}

/* ---- WiFi mode: in -------------------------------------------------------------------- */

bool accept_control(const uint8_t* datagram, size_t len, ds_control_packet_t* out) {
  if (datagram == nullptr || len != DS_CONTROL_FRAME_LEN) return false;
  return ds_control_decode(datagram, len, out);
}

bool relay_heard(bool have, uint32_t rxMs, uint32_t nowMs) {
  return have && (uint32_t)(nowMs - rxMs) < (uint32_t)LINK_TIMEOUT_MS;
}

bool control_is_live(bool have, const ds_control_packet_t& packet, uint32_t rxMs, uint32_t nowMs) {
  return relay_heard(have, rxMs, nowMs) && (packet.flags & DS_CTRL_FLAG_CONNECTED) != 0;
}

uint8_t dpad_code(uint16_t buttons) {
  bool up = (buttons & DS_BTN_DPAD_UP) != 0;
  bool down = (buttons & DS_BTN_DPAD_DOWN) != 0;
  bool left = (buttons & DS_BTN_DPAD_LEFT) != 0;
  bool right = (buttons & DS_BTN_DPAD_RIGHT) != 0;
  if (up && down) up = down = false;
  if (left && right) left = right = false;
  if (up) return right ? XSTATE_DPAD_NE : (left ? XSTATE_DPAD_NW : XSTATE_DPAD_N);
  if (down) return right ? XSTATE_DPAD_SE : (left ? XSTATE_DPAD_SW : XSTATE_DPAD_S);
  if (right) return XSTATE_DPAD_E;
  if (left) return XSTATE_DPAD_W;
  return XSTATE_DPAD_NONE;
}

void control_to_xstate(const ds_control_packet_t* packet, bool live, uint8_t out[XBOX_R4_STATE_LEN]) {
  xbox_r4_state_t s;
  memset(&s, 0, sizeof s);
  xbox_r4_neutral_report(s.report);
  s.state = XBOX_R4_SCANNING;
  s.battery = XBOX_R4_BATTERY_UNKNOWN;
  s.seq = packet != nullptr ? packet->seq : 0;
  if (packet != nullptr && live) {
    s.state = XBOX_R4_CONNECTED;
    put_u16(s.report + XSTATE_OFS_JOY_LH, axis_to_hid(0));   /* the relay sends no left stick: */
    put_u16(s.report + XSTATE_OFS_JOY_LV, axis_to_hid(0));   /* centred, so the RA4M1 reads 0  */
    put_u16(s.report + XSTATE_OFS_JOY_RH, axis_to_hid(packet->rx));
    put_u16(s.report + XSTATE_OFS_JOY_RV, axis_to_hid(packet->ry));
    put_u16(s.report + XSTATE_OFS_TRIG_RT, trigger_to_hid(packet->rt));
    s.report[XSTATE_OFS_DPAD] = dpad_code(packet->buttons);
    s.report[XSTATE_OFS_BTN0] = btn0_of(packet->buttons);
    s.report[XSTATE_OFS_BTN1] = btn1_of(packet->buttons);
  }
  xbox_r4_encode(&s, out);
}

/* ---- WiFi mode: out ------------------------------------------------------------------- */

size_t build_status_frame(const DssArgs& a, uint8_t out[DS_STATUS_FRAME_LEN]) {
  ds_status_packet_t p;
  ds_status_init(&p);
  p.wantMode = a.wantMode;
  p.rumbleStrong = a.strong;
  p.rumbleWeak = a.weak;
  p.rumbleMs = a.rumbleMs;
  p.state = a.state;
  p.rumbleSeq = a.rumbleSeq;
  return ds_status_encode(&p, out);
}

size_t build_beacon_frame(const uint8_t ip[DS_BEACON_IP_LEN], uint8_t bootId, uint8_t wifiStatus,
                          uint8_t out[DS_BEACON_FRAME_LEN]) {
  ds_beacon_packet_t p;
  ds_beacon_init(&p);
  for (size_t i = 0; i < DS_BEACON_IP_LEN; i++) p.ip[i] = ip[i];
  p.udpPort = (uint16_t)WIFI_UDP_PORT;
  p.bootId = bootId;
  p.mode = wifiStatus;
  return ds_beacon_encode(&p, out);
}

/* ---- mode request across esp_restart() ------------------------------------------------- */

bool store_valid(const ModeStore& s) {
  if (s.magic != ESP_MODE_REQUEST_MAGIC || s.check != store_check(s)) return false;
  if (!is_radio_mode(s.request) && s.request != DS_ESP_MODE_NONE) return false;
  return s.lastBootId <= BYTE_MASK;
}

void store_write(ModeStore* s, uint8_t request, uint8_t lastBootId) {
  s->magic = ESP_MODE_REQUEST_MAGIC;
  s->request = request;
  s->lastBootId = lastBootId;
  s->check = store_check(*s);
}

uint8_t pick_boot_id(uint32_t random, uint8_t previous) {
  uint8_t id = (uint8_t)(random & BYTE_MASK);
  while (id == 0 || id == previous) id = (uint8_t)(id + 1);   /* at most two steps */
  return id;
}

BootPlan plan_boot(ModeStore* store, bool softwareReset, uint32_t random) {
  const bool valid = store_valid(*store);
  const uint8_t previous = valid ? (uint8_t)store->lastBootId : (uint8_t)0;
  BootPlan plan;
  plan.startMode = (softwareReset && valid && is_radio_mode(store->request))
                       ? (uint8_t)store->request
                       : (uint8_t)DS_ESP_MODE_NONE;
  plan.bootId = pick_boot_id(random, previous);
  store_write(store, DS_ESP_MODE_NONE, plan.bootId);   /* the request is used up */
  return plan;
}

/* ---- WiFi supervisor -------------------------------------------------------------------- */

WifiSupervisor::Action WifiSupervisor::start(uint32_t nowMs, bool haveSsid, bool apFallback) {
  apFallback_ = apFallback;
  t0_ = nowMs;
  if (haveSsid) {
    phase_ = JOINING;
    return STA_BEGIN;
  }
  if (apFallback) {
    phase_ = AP_UP;
    return AP_START;
  }
  phase_ = NO_NETWORK;
  return NONE;
}

WifiSupervisor::Action WifiSupervisor::step(uint32_t nowMs, bool staConnected) {
  const uint32_t elapsed = (uint32_t)(nowMs - t0_);
  switch (phase_) {
    case JOINING:
      if (staConnected) {
        phase_ = STA_UP;
        return NONE;
      }
      if (apFallback_) {
        if (elapsed >= (uint32_t)WIFI_STA_CONNECT_TIMEOUT_MS) {
          phase_ = AP_UP;
          return AP_START;
        }
        return NONE;
      }
      if (elapsed >= (uint32_t)WIFI_STA_RETRY_MS) {
        t0_ = nowMs;
        return STA_REBEGIN;
      }
      return NONE;
    case STA_UP:
      if (!staConnected) {
        phase_ = STA_LOST;
        t0_ = nowMs;
        return STA_REBEGIN;
      }
      return NONE;
    case STA_LOST:
      if (staConnected) {
        phase_ = STA_UP;
        return NONE;
      }
      if (elapsed >= (uint32_t)WIFI_STA_RETRY_MS) {
        t0_ = nowMs;
        return STA_REBEGIN;
      }
      return NONE;
    case IDLE:
    case AP_UP:
    case NO_NETWORK:
    default:
      return NONE;
  }
}

uint8_t wifi_status(bool wifiStarted, WifiSupervisor::Phase phase, bool relayHeard) {
  if (!wifiStarted) return DS_WIFI_STATUS_OFF;
  switch (phase) {
    case WifiSupervisor::STA_UP:
      return relayHeard ? DS_WIFI_STATUS_RELAY_HEARD : DS_WIFI_STATUS_STATION_UP;
    case WifiSupervisor::AP_UP:
      return relayHeard ? DS_WIFI_STATUS_RELAY_HEARD : DS_WIFI_STATUS_AP_UP;
    case WifiSupervisor::NO_NETWORK:
      return DS_WIFI_STATUS_OFF;
    case WifiSupervisor::IDLE:      /* the task was created but has not started the station yet */
    case WifiSupervisor::JOINING:
    case WifiSupervisor::STA_LOST:
    default:
      return DS_WIFI_STATUS_CONNECTING;
  }
}

}  /* namespace dsentry */

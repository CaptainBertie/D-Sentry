// test_dsentry_wire.cpp -- host tests of firmware/UNOR4USBBridge/dsentry_wire.cpp, the logic of
// the D-Sentry additions to the ESP32-S3 bridge, compiled unchanged from the firmware tree.
//
// Expected values come from the contract, not from the code: D-SentryCode/docs/architecture.md
// 4.1/4.2/4.4, protocol_spec.py's examples (the frames in docs/protocol.md, copied here as hex),
// protocol/xbox_r4_protocol.h (the 20-byte state) and the RA4M1's own decode formula
// rx = (joyRHori - 32768) >> 6.
//
// Safety properties pinned here (each test fails if the property is broken):
//   - radios_never_run_together: no AT+DSS decision ever starts a radio next to another radio
//     user (BLE + WiFi, or NimBLE + the HCI bridge) -- the owner's "never both" rule;
//   - silent_relay_reads_as_disconnected_after_link_timeout: in WiFi mode a relay that stops
//     sending (or says connected = 0) is reported to the RA4M1 as "scanning" with a neutral
//     report no later than LINK_TIMEOUT_MS after its last packet, so the turret's failsafe runs;
//   - corrupted_datagrams_never_drive_the_turret: every single-bit corruption of a ControlPacket
//     is refused.
//
// SPDX-License-Identifier: MIT
#include "testkit.h"

#include <set>
#include <string>
#include <vector>

#include "dsentry_wire.h"

using namespace dsentry;

namespace {

std::vector<uint8_t> unhex(const std::string& text) {
  std::vector<uint8_t> out;
  int hi = -1;
  for (char c : text) {
    int v = ds_hex_digit(c);
    if (v < 0) continue;   // spaces
    if (hi < 0) {
      hi = v;
    } else {
      out.push_back((uint8_t)(hi * 16 + v));
      hi = -1;
    }
  }
  return out;
}

std::string tohex(const uint8_t* data, size_t len) {
  std::string s(2 * len + 1, '\0');
  ds_hex_encode(data, len, &s[0]);
  s.resize(2 * len);
  return s;
}

// docs/protocol.md examples (generated from protocol_spec.py; copied, not computed)
const char* const kControlExample = "a5 0a 03 00 fe ff 01 ff 03 11 08 07 ac";
const char* const kStatusExample = "5a 07 01 c8 78 50 00 05 2a 9f";
const char* const kBeaconExample = "d6 08 c0 a8 04 01 72 10 5c 03 e0";
const char* const kTelemetryExample =
    "d56b02c840e2010005000101c7683aeeffffd2040000d8dc0000000000002c0100000c0f030071140002fe01"
    "07009426640002000100230007007f2f0041524d454420462020414d4d4f3a313242542020202020436f6e6e"
    "656374656400508000d4fe00feff010002ff031160ca";

uint16_t get_u16(const uint8_t* p) { return (uint16_t)(p[0] | (p[1] << 8)); }

// What the RA4M1 does with the 20 bytes (architecture.md 4.2).
int ra_axis(uint16_t raw) { return ((int)raw - 32768) >> 6; }

xbox_r4_state_t decode_state(const uint8_t out[XBOX_R4_STATE_LEN]) {
  xbox_r4_state_t s;
  memset(&s, 0xEE, sizeof s);
  CHECK(xbox_r4_decode(out, XBOX_R4_STATE_LEN, &s));
  return s;
}

ds_control_packet_t control(uint8_t flags, int16_t rx, int16_t ry, uint16_t rt, uint16_t buttons, uint8_t seq) {
  ds_control_packet_t p;
  ds_control_init(&p);
  p.flags = flags;
  p.rx = rx;
  p.ry = ry;
  p.rt = rt;
  p.buttons = buttons;
  p.seq = seq;
  return p;
}

}  // namespace

// ---------------------------------------------------------------------------------------------
// AT arguments and replies
// ---------------------------------------------------------------------------------------------

TEST(parse_uint_is_strict_decimal) {
  uint32_t v = 77;
  CHECK(parse_uint("0", 255, &v) && v == 0);
  CHECK(parse_uint("255", 255, &v) && v == 255);
  CHECK(parse_uint("007", 255, &v) && v == 7);
  CHECK(parse_uint("4294967295", UINT32_MAX, &v) && v == 4294967295u);
  v = 77;
  for (const char* bad : {"", "256", "-1", "+1", " 1", "1 ", "1a", "0x10", "1.0", "4294967296",
                          "99999999999999999999", "\"1\""}) {
    if (!CHECK(!parse_uint(bad, bad[0] == '4' || bad[0] == '9' ? UINT32_MAX : 255, &v)))
      std::printf("      accepted \"%s\"\n", bad);
  }
  CHECK_EQ(v, 77u);   // untouched on failure
}

TEST(parse_dss_takes_the_six_status_fields) {
  DssArgs a;
  CHECK(parse_dss({"1", "42", "200", "120", "80", "5"}, &a));   // docs/protocol.md example
  CHECK_EQ(a.wantMode, 1);
  CHECK_EQ(a.rumbleSeq, 42);
  CHECK_EQ(a.strong, 200);
  CHECK_EQ(a.weak, 120);
  CHECK_EQ(a.rumbleMs, 80);
  CHECK_EQ(a.state, 5);
  CHECK(parse_dss({"0", "255", "255", "255", "65535", "255"}, &a));
  CHECK_EQ(a.rumbleMs, 65535);
  CHECK_EQ(a.state, 255);   // any u8: only logged and forwarded
}

TEST(parse_dss_rejects_anything_else) {
  DssArgs a;
  const std::vector<std::vector<std::string>> bad = {
      {},
      {"1", "42", "200", "120", "80"},
      {"1", "42", "200", "120", "80", "5", "0"},
      {"2", "42", "200", "120", "80", "5"},       // wantMode beyond DS_RADIO_WIFI
      {"1", "256", "200", "120", "80", "5"},
      {"1", "42", "256", "120", "80", "5"},
      {"1", "42", "200", "256", "80", "5"},
      {"1", "42", "200", "120", "65536", "5"},
      {"1", "42", "200", "120", "80", "256"},
      {"1", "", "200", "120", "80", "5"},
      {"1", "42", "-1", "120", "80", "5"},
      {"1", "42", "200", "120", "8 0", "5"},
  };
  for (const auto& args : bad) CHECK(!parse_dss(args, &a));
}

TEST(reply_payloads_match_the_docs) {
  CHECK_EQ(format_caps_reply(), std::string("1,15"));   // "+DSCAPS: 1,15"
  CHECK_EQ(caps_bits(), DS_CAP_ALL);
  CHECK_EQ(format_dss_reply(1, 92, 4), std::string("1,92,4"));
  CHECK_EQ(format_dss_reply(255, 255, 255), std::string("255,255,255"));
  CHECK_EQ(format_dss_reply(0, 1, 0), std::string("0,1,0"));
  CHECK_EQ(format_dst_reply(1), std::string("1"));
  CHECK_EQ(format_dst_reply(0), std::string("0"));
}

TEST(parse_dst_accepts_exactly_one_telemetry_frame) {
  const std::vector<uint8_t> frame = unhex(kTelemetryExample);
  CHECK_EQ(frame.size(), (size_t)DS_TELEMETRY_FRAME_LEN);
  uint8_t out[DS_TELEMETRY_FRAME_LEN];
  CHECK_EQ(parse_dst(kTelemetryExample, out), (size_t)DS_TELEMETRY_FRAME_LEN);
  CHECK(memcmp(out, frame.data(), frame.size()) == 0);
  std::string upper = kTelemetryExample;
  for (char& c : upper) c = (char)toupper((unsigned char)c);
  CHECK_EQ(parse_dst(upper, out), (size_t)DS_TELEMETRY_FRAME_LEN);
  CHECK(memcmp(out, frame.data(), frame.size()) == 0);
}

TEST(parse_dst_rejects_bad_hex_lengths_and_frames) {
  uint8_t out[DS_TELEMETRY_FRAME_LEN];
  memset(out, 0x55, sizeof out);
  const std::string good = kTelemetryExample;
  CHECK_EQ(parse_dst("", out), 0u);
  CHECK_EQ(parse_dst(good.substr(0, good.size() - 1), out), 0u);   // odd
  CHECK_EQ(parse_dst(good.substr(0, good.size() - 2), out), 0u);   // one byte short
  CHECK_EQ(parse_dst(good + "00", out), 0u);                       // one byte long
  std::string g = good;
  g[10] = 'g';
  CHECK_EQ(parse_dst(g, out), 0u);
  // A ControlPacket frame, hex encoded, is not telemetry.
  const std::vector<uint8_t> ctrl = unhex(kControlExample);
  CHECK_EQ(parse_dst(tohex(ctrl.data(), ctrl.size()), out), 0u);
  // Every single-bit error in the 110 bytes is refused (start, len or CRC check).
  std::vector<uint8_t> frame = unhex(kTelemetryExample);
  int accepted = 0;
  for (size_t i = 0; i < frame.size(); i++) {
    for (int b = 0; b < 8; b++) {
      frame[i] ^= (uint8_t)(1u << b);
      if (parse_dst(tohex(frame.data(), frame.size()), out) != 0) accepted++;
      frame[i] ^= (uint8_t)(1u << b);
    }
  }
  CHECK_EQ(accepted, 0);
  // A different start byte with a correct CRC is still refused.
  frame[0] = DS_BEACON_START;
  frame.back() = ds_crc8(frame.data() + 1, frame.size() - 2);
  CHECK_EQ(parse_dst(tohex(frame.data(), frame.size()), out), 0u);
  CHECK_EQ(out[0], 0x55);   // never written on failure
}

// ---------------------------------------------------------------------------------------------
// Radio decision
// ---------------------------------------------------------------------------------------------

namespace {
Radios radios(bool ble, bool hci, bool dsWifi, bool otherWifi, bool pending = false) {
  Radios r;
  r.bleRunning = ble;
  r.hciUsed = hci;
  r.dsWifiRunning = dsWifi;
  r.otherWifiRunning = otherWifi;
  r.restartPending = pending;
  return r;
}
}  // namespace

TEST(mode_decisions_follow_the_contract) {
  // Nothing running: start in place.
  CHECK_EQ(decide_mode(DS_RADIO_BLE, radios(false, false, false, false)), ModeAction::START_BLE);
  CHECK_EQ(decide_mode(DS_RADIO_WIFI, radios(false, false, false, false)), ModeAction::START_WIFI);
  // Idempotent.
  CHECK_EQ(decide_mode(DS_RADIO_BLE, radios(true, false, false, false)), ModeAction::ALREADY);
  CHECK_EQ(decide_mode(DS_RADIO_WIFI, radios(false, false, true, false)), ModeAction::ALREADY);
  // The other radio runs: restart into the wanted one.
  CHECK_EQ(decide_mode(DS_RADIO_WIFI, radios(true, false, false, false)), ModeAction::RESTART);
  CHECK_EQ(decide_mode(DS_RADIO_BLE, radios(false, false, true, false)), ModeAction::RESTART);
  // Stock users of the radios (a previous sketch's WiFiS3 / ArduinoBLE) also force a restart.
  CHECK_EQ(decide_mode(DS_RADIO_BLE, radios(false, true, false, false)), ModeAction::RESTART);
  CHECK_EQ(decide_mode(DS_RADIO_BLE, radios(false, false, false, true)), ModeAction::RESTART);
  CHECK_EQ(decide_mode(DS_RADIO_BLE, radios(true, false, false, true)), ModeAction::RESTART);
  CHECK_EQ(decide_mode(DS_RADIO_WIFI, radios(false, false, false, true)), ModeAction::RESTART);
  CHECK_EQ(decide_mode(DS_RADIO_WIFI, radios(false, true, false, false)), ModeAction::RESTART);
  // While a restart is pending every valid request is RESTART (latest wins, no second restart).
  CHECK_EQ(decide_mode(DS_RADIO_WIFI, radios(false, false, true, false, true)), ModeAction::RESTART);
  CHECK_EQ(decide_mode(DS_RADIO_BLE, radios(false, false, false, false, true)), ModeAction::RESTART);
  // Not a radio.
  CHECK_EQ(decide_mode(DS_RADIO_COUNT, radios(false, false, false, false)), ModeAction::INVALID);
  CHECK_EQ(decide_mode(255, radios(true, false, false, false)), ModeAction::INVALID);
}

TEST(radios_never_run_together) {
  // KEY SAFETY PROPERTY. For every combination of radio users and both wanted modes, apply the
  // decision and check what would be running afterwards.
  int combos = 0;
  for (int bits = 0; bits < 32; bits++) {
    const Radios before = radios(bits & 1, bits & 2, bits & 4, bits & 8, bits & 16);
    for (uint8_t want = 0; want < DS_RADIO_COUNT; want++) {
      combos++;
      const ModeAction act = decide_mode(want, before);
      Radios after = before;
      switch (act) {
        case ModeAction::START_BLE:
          after.bleRunning = true;
          break;
        case ModeAction::START_WIFI:
          after.dsWifiRunning = true;
          break;
        case ModeAction::RESTART:   // the next boot starts only the wanted radio
          after = radios(want == DS_RADIO_BLE, false, want == DS_RADIO_WIFI, false);
          break;
        case ModeAction::ALREADY:
        case ModeAction::INVALID:
          break;
      }
      const bool bleSide = after.bleRunning || after.hciUsed;
      const bool wifiSide = after.dsWifiRunning || after.otherWifiRunning;
      if (act != ModeAction::ALREADY && act != ModeAction::INVALID) {
        if (!CHECK(!(bleSide && wifiSide))) std::printf("      bits %d want %u\n", bits, want);
        if (!CHECK(!(after.bleRunning && after.hciUsed))) std::printf("      bits %d want %u\n", bits, want);
      }
      // A start in place only ever happens with nothing else running and nothing pending.
      if (act == ModeAction::START_BLE || act == ModeAction::START_WIFI) {
        CHECK(!before.bleRunning && !before.hciUsed && !before.dsWifiRunning &&
              !before.otherWifiRunning && !before.restartPending);
      }
      // ALREADY means exactly the wanted radio, alone.
      if (act == ModeAction::ALREADY) {
        CHECK(!before.restartPending);
        if (want == DS_RADIO_BLE) CHECK(before.bleRunning && !wifiSide && !before.hciUsed);
        if (want == DS_RADIO_WIFI) CHECK(before.dsWifiRunning && !before.bleRunning && !before.hciUsed);
      }
      CHECK(act != ModeAction::INVALID);
    }
  }
  CHECK_EQ(combos, 64);
}

TEST(esp_mode_reports_in_priority_order) {
  CHECK_EQ(esp_mode_of(radios(false, false, false, false)), DS_ESP_MODE_NONE);
  CHECK_EQ(esp_mode_of(radios(true, false, false, false)), DS_ESP_MODE_BLE);
  CHECK_EQ(esp_mode_of(radios(false, false, true, false)), DS_ESP_MODE_WIFI);
  CHECK_EQ(esp_mode_of(radios(false, false, false, true)), DS_ESP_MODE_NONE);   // not ours
  CHECK_EQ(esp_mode_of(radios(false, true, false, false)), DS_ESP_MODE_NONE);
  CHECK_EQ(esp_mode_of(radios(true, false, false, false, true)), DS_ESP_MODE_RESTARTING);
  CHECK_EQ(esp_mode_of(radios(false, false, true, false, true)), DS_ESP_MODE_RESTARTING);
}

// ---------------------------------------------------------------------------------------------
// Rumble
// ---------------------------------------------------------------------------------------------

TEST(rumble_seq_first_value_is_only_a_baseline) {
  RumbleSeqTracker t;
  CHECK(!t.is_new(42));   // first AT+DSS after boot: an old shot, never replayed
  CHECK(!t.is_new(42));   // heartbeats
  CHECK(!t.is_new(42));
  CHECK(t.is_new(43));
  CHECK(!t.is_new(43));
  CHECK(t.is_new(255));
  CHECK(t.is_new(0));     // wraps
  CHECK(t.is_new(43));    // any change counts, not only +1
  t.reset();
  CHECK(!t.is_new(7));
  CHECK(t.is_new(8));
}

TEST(rumble_report_bytes_for_the_per_shot_rumble) {
  uint8_t r[XBOX_RUMBLE_REPORT_LEN];
  rumble_report(200, 120, 80, r);   // RUMBLE_STRONG_255 / WEAK_255 / DURATION_MS defaults
  const uint8_t want[XBOX_RUMBLE_REPORT_LEN] = {0x03, 0, 0, 78, 47, 8, 0, 0};
  CHECK(memcmp(r, want, sizeof want) == 0);
  rumble_report(255, 0, 10, r);
  const uint8_t want2[XBOX_RUMBLE_REPORT_LEN] = {0x03, 0, 0, 100, 0, 1, 0, 0};
  CHECK(memcmp(r, want2, sizeof want2) == 0);
  rumble_report(0, 255, 1, r);
  const uint8_t want3[XBOX_RUMBLE_REPORT_LEN] = {0x03, 0, 0, 0, 100, 1, 0, 0};
  CHECK(memcmp(r, want3, sizeof want3) == 0);
}

TEST(rumble_report_ms_zero_stops_both_motors) {
  uint8_t r[XBOX_RUMBLE_REPORT_LEN];
  memset(r, 0xAA, sizeof r);
  rumble_report(200, 120, 0, r);
  const uint8_t stop[XBOX_RUMBLE_REPORT_LEN] = {0x03, 0, 0, 0, 0, 0, 0, 0};
  CHECK(memcmp(r, stop, sizeof stop) == 0);
}

TEST(rumble_power_scaling_rounds_to_nearest_percent) {
  uint8_t r[XBOX_RUMBLE_REPORT_LEN];
  for (int v = 0; v <= 255; v++) {
    rumble_report((uint8_t)v, (uint8_t)(255 - v), 100, r);
    CHECK_NEAR(r[3], v * 100.0 / 255.0, 0.5);
    CHECK_NEAR(r[4], (255 - v) * 100.0 / 255.0, 0.5);
    CHECK(r[3] <= 100 && r[4] <= 100);
    CHECK_EQ(r[1], 0);   // trigger motors never driven
    CHECK_EQ(r[2], 0);
  }
}

TEST(rumble_duration_covers_the_request_for_every_ms) {
  uint8_t r[XBOX_RUMBLE_REPORT_LEN];
  int bad = 0;
  for (uint32_t ms = 1; ms <= 65535; ms++) {
    rumble_report(100, 100, (uint16_t)ms, r);
    const uint32_t active = r[5], plays = (uint32_t)r[7] + 1;
    const uint32_t total = active * plays * 10;
    if (active < 1 || r[6] != 0 || total < ms || total >= ms + 10 * (plays + 1)) bad++;
  }
  CHECK_EQ(bad, 0);
  rumble_report(1, 1, 2550, r);
  CHECK_EQ(r[5], 255);
  CHECK_EQ(r[7], 0);
  rumble_report(1, 1, 2551, r);   // one unit too long for one play: two equal plays
  CHECK_EQ(r[5], 128);
  CHECK_EQ(r[7], 1);
  rumble_report(1, 1, 65535, r);
  CHECK_EQ(r[7], 25);
  CHECK_EQ(r[5], 253);
}

// ---------------------------------------------------------------------------------------------
// WiFi mode: ControlPackets in, XBOXSTATE out
// ---------------------------------------------------------------------------------------------

TEST(accept_control_takes_the_docs_example) {
  const std::vector<uint8_t> f = unhex(kControlExample);
  ds_control_packet_t p;
  CHECK(accept_control(f.data(), f.size(), &p));
  CHECK_EQ(p.flags, 0x03);
  CHECK_EQ(p.rx, -512);
  CHECK_EQ(p.ry, 511);
  CHECK_EQ(p.rt, 1023);
  CHECK_EQ(p.buttons, 0x0811);
  CHECK_EQ(p.seq, 7);
}

TEST(corrupted_datagrams_never_drive_the_turret) {
  // SAFETY PROPERTY: only an exact, CRC-valid ControlPacket is accepted.
  std::vector<uint8_t> f = unhex(kControlExample);
  ds_control_packet_t p;
  int accepted = 0;
  for (size_t i = 0; i < f.size(); i++) {
    for (int b = 0; b < 8; b++) {
      f[i] ^= (uint8_t)(1u << b);
      if (accept_control(f.data(), f.size(), &p)) accepted++;
      f[i] ^= (uint8_t)(1u << b);
    }
  }
  CHECK_EQ(accepted, 0);
  CHECK(!accept_control(f.data(), f.size() - 1, &p));
  std::vector<uint8_t> longer = f;
  longer.push_back(0);
  CHECK(!accept_control(longer.data(), longer.size(), &p));
  CHECK(!accept_control(nullptr, f.size(), &p));
  const std::vector<uint8_t> status = unhex(kStatusExample);
  CHECK(!accept_control(status.data(), status.size(), &p));
  const char text[] = "Z\x0a hello there";   // text starting like a frame
  CHECK(!accept_control((const uint8_t*)text, DS_CONTROL_FRAME_LEN, &p));
  // Two frames glued in one datagram are refused too (exact length).
  std::vector<uint8_t> two = f;
  two.insert(two.end(), f.begin(), f.end());
  CHECK(!accept_control(two.data(), two.size(), &p));
}

TEST(xstate_from_the_docs_control_example) {
  const ds_control_packet_t p = control(0x03, -512, 511, 1023, 0x0811, 7);   // A, LB, back
  uint8_t out[XBOX_R4_STATE_LEN];
  control_to_xstate(&p, true, out);
  const xbox_r4_state_t s = decode_state(out);
  CHECK_EQ(s.state, XBOX_R4_CONNECTED);
  CHECK_EQ(s.seq, 7);
  CHECK_EQ(s.battery, XBOX_R4_BATTERY_UNKNOWN);
  // Left stick centred at 32768 so the RA4M1's (raw - 32768) >> 6 reads exactly 0: the relay sends
  // no left stick (review finding: the neutral report's 32767 read as -1 / -1 on the dashboard).
  CHECK_EQ(get_u16(s.report + 0), 32768);
  CHECK_EQ(get_u16(s.report + 2), 32768);
  CHECK_EQ(get_u16(s.report + 4), 0);       // rx -512 = full left
  CHECK_EQ(get_u16(s.report + 6), 65472);   // ry 511 = full down
  CHECK_EQ(get_u16(s.report + 8), 0);       // left trigger
  CHECK_EQ(get_u16(s.report + 10), 1023);
  CHECK_EQ(s.report[12], XSTATE_DPAD_NONE);
  CHECK_EQ(s.report[13], 0x41);             // A (bit0) + LB (bit6)
  CHECK_EQ(s.report[14], 0x04);             // View (back)
  CHECK_EQ(s.report[15], 0);
  CHECK_EQ(ra_axis(get_u16(s.report + 4)), -512);
  CHECK_EQ(ra_axis(get_u16(s.report + 6)), 511);
}

TEST(xstate_axes_round_trip_through_the_ra4m1_formula) {
  uint8_t out[XBOX_R4_STATE_LEN];
  int bad = 0;
  for (int v = -512; v <= 511; v++) {
    const ds_control_packet_t p = control(DS_CTRL_FLAG_CONNECTED, (int16_t)v, (int16_t)(-1 - v), 0, 0, 1);
    control_to_xstate(&p, true, out);
    const xbox_r4_state_t s = decode_state(out);
    if (ra_axis(get_u16(s.report + XSTATE_OFS_JOY_RH)) != v) bad++;
    if (ra_axis(get_u16(s.report + XSTATE_OFS_JOY_RV)) != -1 - v) bad++;
  }
  CHECK_EQ(bad, 0);
  // Out-of-range values are clamped, never wrapped (a wrapped -32768 would read full right).
  ds_control_packet_t p = control(DS_CTRL_FLAG_CONNECTED, -32768, 32767, 65535, 0, 1);
  control_to_xstate(&p, true, out);
  xbox_r4_state_t s = decode_state(out);
  CHECK_EQ(ra_axis(get_u16(s.report + 4)), -512);
  CHECK_EQ(ra_axis(get_u16(s.report + 6)), 511);
  CHECK_EQ(get_u16(s.report + 10), 1023);
  p = control(DS_CTRL_FLAG_CONNECTED, 600, -600, 1024, 0, 1);
  control_to_xstate(&p, true, out);
  s = decode_state(out);
  CHECK_EQ(ra_axis(get_u16(s.report + 4)), 511);
  CHECK_EQ(ra_axis(get_u16(s.report + 6)), -512);
  CHECK_EQ(get_u16(s.report + 10), 1023);
  p = control(DS_CTRL_FLAG_CONNECTED, 0, 0, 512, 0, 1);
  control_to_xstate(&p, true, out);
  s = decode_state(out);
  CHECK_EQ(get_u16(s.report + 4), 32768);
  CHECK_EQ(ra_axis(get_u16(s.report + 4)), 0);
  CHECK_EQ(get_u16(s.report + 10), 512);
}

TEST(xstate_buttons_map_one_to_one) {
  struct Map { uint16_t bit; size_t byte; uint8_t mask; };
  const Map maps[] = {
      {DS_BTN_A, 13, 0x01},  {DS_BTN_B, 13, 0x02},     {DS_BTN_X, 13, 0x08},
      {DS_BTN_Y, 13, 0x10},  {DS_BTN_LB, 13, 0x40},    {DS_BTN_RB, 13, 0x80},
      {DS_BTN_BACK, 14, 0x04}, {DS_BTN_START, 14, 0x08},
  };
  uint8_t out[XBOX_R4_STATE_LEN];
  for (const Map& m : maps) {
    const ds_control_packet_t p = control(DS_CTRL_FLAG_CONNECTED, 0, 0, 0, m.bit, 1);
    control_to_xstate(&p, true, out);
    const xbox_r4_state_t s = decode_state(out);
    CHECK_EQ(s.report[13], m.byte == 13 ? m.mask : 0);
    CHECK_EQ(s.report[14], m.byte == 14 ? m.mask : 0);
    CHECK_EQ(s.report[15], 0);
    CHECK_EQ(s.report[12], XSTATE_DPAD_NONE);
  }
  // Undefined button bits (12..15) are ignored.
  const ds_control_packet_t p = control(DS_CTRL_FLAG_CONNECTED, 0, 0, 0, 0xF000, 1);
  control_to_xstate(&p, true, out);
  const xbox_r4_state_t s = decode_state(out);
  CHECK_EQ(s.report[13], 0);
  CHECK_EQ(s.report[14], 0);
}

TEST(dpad_bits_map_to_the_eight_way_code) {
  const uint16_t U = DS_BTN_DPAD_UP, D = DS_BTN_DPAD_DOWN, L = DS_BTN_DPAD_LEFT, R = DS_BTN_DPAD_RIGHT;
  CHECK_EQ(dpad_code(0), XSTATE_DPAD_NONE);
  CHECK_EQ(dpad_code(U), XSTATE_DPAD_N);
  CHECK_EQ(dpad_code(U | R), XSTATE_DPAD_NE);
  CHECK_EQ(dpad_code(R), XSTATE_DPAD_E);
  CHECK_EQ(dpad_code(D | R), XSTATE_DPAD_SE);
  CHECK_EQ(dpad_code(D), XSTATE_DPAD_S);
  CHECK_EQ(dpad_code(D | L), XSTATE_DPAD_SW);
  CHECK_EQ(dpad_code(L), XSTATE_DPAD_W);
  CHECK_EQ(dpad_code(U | L), XSTATE_DPAD_NW);
  CHECK_EQ(dpad_code(U | D), XSTATE_DPAD_NONE);       // opposite directions cancel
  CHECK_EQ(dpad_code(L | R), XSTATE_DPAD_NONE);
  CHECK_EQ(dpad_code(U | D | L), XSTATE_DPAD_W);
  CHECK_EQ(dpad_code(U | L | R), XSTATE_DPAD_N);
  CHECK_EQ(dpad_code(U | D | L | R), XSTATE_DPAD_NONE);
  CHECK_EQ(dpad_code(DS_BTN_A | DS_BTN_START), XSTATE_DPAD_NONE);
  // Every one of the 16 combinations gives one of the nine defined codes.
  for (int combo = 0; combo < 16; combo++) {
    uint16_t b = (uint16_t)(((combo & 1) ? U : 0) | ((combo & 2) ? D : 0) | ((combo & 4) ? L : 0) |
                            ((combo & 8) ? R : 0));
    CHECK(dpad_code(b) <= XSTATE_DPAD_NW);
  }
}

TEST(silent_relay_reads_as_disconnected_after_link_timeout) {
  // KEY SAFETY PROPERTY (WiFi-mode failsafe, spec acceptance 13).
  const ds_control_packet_t p = control(DS_CTRL_FLAG_CONNECTED | DS_CTRL_FLAG_SOURCE_WIFI, 300, -300, 1023,
                                        DS_BTN_A | DS_BTN_LB, 9);
  const uint32_t rx = 1000;
  CHECK(control_is_live(true, p, rx, rx));
  CHECK(control_is_live(true, p, rx, rx + LINK_TIMEOUT_MS - 1));
  CHECK(!control_is_live(true, p, rx, rx + LINK_TIMEOUT_MS));
  CHECK(!control_is_live(true, p, rx, rx + LINK_TIMEOUT_MS + 100000));
  CHECK(!control_is_live(false, p, rx, rx));
  // millis() wrap: the packet arrived just before the wrap.
  const uint32_t rxw = 0xFFFFFF00u;
  CHECK(control_is_live(true, p, rxw, rxw + 100));
  CHECK(!control_is_live(true, p, rxw, rxw + LINK_TIMEOUT_MS));
  // The relay itself says nobody is driving.
  ds_control_packet_t idle = p;
  idle.flags = DS_CTRL_FLAG_SOURCE_WIFI;
  CHECK(!control_is_live(true, idle, rx, rx));
  CHECK(relay_heard(true, rx, rx));   // ...but the relay is heard
  CHECK(!relay_heard(true, rx, rx + LINK_TIMEOUT_MS));
  // What the RA4M1 then reads: scanning, neutral sticks, no trigger, no buttons, valid CRC.
  uint8_t out[XBOX_R4_STATE_LEN];
  control_to_xstate(&p, control_is_live(true, p, rx, rx + LINK_TIMEOUT_MS), out);
  xbox_r4_state_t s = decode_state(out);
  CHECK_EQ(s.state, XBOX_R4_SCANNING);
  uint8_t neutral[XBOX_R4_REPORT_LEN];
  xbox_r4_neutral_report(neutral);
  CHECK(memcmp(s.report, neutral, sizeof neutral) == 0);
  CHECK_EQ(s.seq, 9);
  // Nothing received yet.
  control_to_xstate(nullptr, true, out);
  s = decode_state(out);
  CHECK_EQ(s.state, XBOX_R4_SCANNING);
  CHECK_EQ(s.seq, 0);
  CHECK(memcmp(s.report, neutral, sizeof neutral) == 0);
}

// ---------------------------------------------------------------------------------------------
// Packets out
// ---------------------------------------------------------------------------------------------

TEST(status_frame_matches_the_docs_example) {
  DssArgs a;
  CHECK(parse_dss({"1", "42", "200", "120", "80", "5"}, &a));
  uint8_t f[DS_STATUS_FRAME_LEN];
  CHECK_EQ(build_status_frame(a, f), (size_t)DS_STATUS_FRAME_LEN);
  const std::vector<uint8_t> want = unhex(kStatusExample);
  CHECK(memcmp(f, want.data(), want.size()) == 0);
}

TEST(beacon_frame_matches_the_docs_example) {
  const uint8_t ip[DS_BEACON_IP_LEN] = {192, 168, 4, 1};
  uint8_t f[DS_BEACON_FRAME_LEN];
  CHECK_EQ(build_beacon_frame(ip, 0x5C, DS_WIFI_STATUS_AP_UP, f), (size_t)DS_BEACON_FRAME_LEN);
  ds_beacon_packet_t b;
  CHECK(ds_beacon_decode(f, sizeof f, &b));
  CHECK(memcmp(b.ip, ip, sizeof ip) == 0);
  CHECK_EQ(b.udpPort, WIFI_UDP_PORT);
  CHECK_EQ(b.bootId, 0x5C);
  CHECK_EQ(b.mode, DS_WIFI_STATUS_AP_UP);
  if (WIFI_UDP_PORT == 4210) {   // the docs example uses the default port
    const std::vector<uint8_t> want = unhex(kBeaconExample);
    CHECK(memcmp(f, want.data(), want.size()) == 0);
  }
}

// ---------------------------------------------------------------------------------------------
// RTC_NOINIT mode request and bootId
// ---------------------------------------------------------------------------------------------

TEST(mode_store_rejects_garbage) {
  ModeStore s;
  memset(&s, 0, sizeof s);
  CHECK(!store_valid(s));
  memset(&s, 0xFF, sizeof s);
  CHECK(!store_valid(s));
  uint32_t x = 12345;
  int accepted = 0;
  for (int i = 0; i < 100000; i++) {
    uint32_t w[4];
    for (uint32_t& v : w) {
      x = x * 1664525u + 1013904223u;
      v = x;
    }
    memcpy(&s, w, sizeof s);
    if (store_valid(s)) accepted++;
  }
  CHECK_EQ(accepted, 0);
  store_write(&s, DS_ESP_MODE_WIFI, 17);
  CHECK(store_valid(s));
  ModeStore t = s;
  t.request = DS_ESP_MODE_BLE;   // changed without the check word
  CHECK(!store_valid(t));
  t = s;
  t.lastBootId = 18;
  CHECK(!store_valid(t));
  // Values outside the domain are refused even with a matching check word.
  t = s;
  t.request = DS_ESP_MODE_RESTARTING;
  t.check = ~(t.magic ^ t.request ^ t.lastBootId);
  CHECK(!store_valid(t));
  t = s;
  t.lastBootId = 300;
  t.check = ~(t.magic ^ t.request ^ t.lastBootId);
  CHECK(!store_valid(t));
}

TEST(boot_starts_a_radio_only_after_our_software_restart_and_only_once) {
  ModeStore s;
  store_write(&s, DS_ESP_MODE_WIFI, 40);
  BootPlan p = plan_boot(&s, true, 0x1234);
  CHECK_EQ(p.startMode, DS_ESP_MODE_WIFI);
  CHECK_EQ(p.bootId, 0x34);
  CHECK(store_valid(s));
  CHECK_EQ(s.request, (uint32_t)DS_ESP_MODE_NONE);   // consumed
  CHECK_EQ(s.lastBootId, 0x34u);
  // The next software reset (AT+RESET, a panic is not ESP_RST_SW anyway) boots radio-less.
  p = plan_boot(&s, true, 0x99);
  CHECK_EQ(p.startMode, DS_ESP_MODE_NONE);
  // BLE request.
  store_write(&s, DS_ESP_MODE_BLE, 3);
  p = plan_boot(&s, true, 7);
  CHECK_EQ(p.startMode, DS_ESP_MODE_BLE);
  // A valid request after a power-on / watchdog / brownout reset is ignored and dropped.
  store_write(&s, DS_ESP_MODE_BLE, 3);
  p = plan_boot(&s, false, 7);
  CHECK_EQ(p.startMode, DS_ESP_MODE_NONE);
  p = plan_boot(&s, true, 7);
  CHECK_EQ(p.startMode, DS_ESP_MODE_NONE);
  // Power-on garbage, even if the reset reason said software.
  memset(&s, 0xA5, sizeof s);
  p = plan_boot(&s, true, 7);
  CHECK_EQ(p.startMode, DS_ESP_MODE_NONE);
  CHECK(store_valid(s));
  CHECK(p.bootId != 0);
}

TEST(boot_id_is_never_zero_and_always_changes) {
  int bad = 0;
  for (int prev = 0; prev <= 255; prev++) {
    for (int rnd = 0; rnd <= 255; rnd++) {
      uint8_t id = pick_boot_id((uint32_t)rnd | 0xABCD0000u, (uint8_t)prev);
      if (id == 0 || id == prev) bad++;
      if (rnd != 0 && rnd != prev && id != rnd) bad++;   // otherwise the random byte itself
    }
  }
  CHECK_EQ(bad, 0);
  // A thousand boots with a stuck random source still change bootId every time.
  ModeStore s;
  memset(&s, 0, sizeof s);
  uint8_t last = 0;
  int same = 0;
  for (int i = 0; i < 1000; i++) {
    BootPlan p = plan_boot(&s, true, 0x42);
    if (p.bootId == last || p.bootId == 0) same++;
    last = p.bootId;
  }
  CHECK_EQ(same, 0);
}

// ---------------------------------------------------------------------------------------------
// WiFi supervisor
// ---------------------------------------------------------------------------------------------

TEST(supervisor_station_joins) {
  WifiSupervisor w;
  CHECK_EQ(w.start(100, true, true), WifiSupervisor::STA_BEGIN);
  CHECK_EQ(w.phase(), WifiSupervisor::JOINING);
  CHECK(!w.network_up());
  CHECK_EQ(w.step(200, false), WifiSupervisor::NONE);
  CHECK_EQ(w.step(300, true), WifiSupervisor::NONE);
  CHECK_EQ(w.phase(), WifiSupervisor::STA_UP);
  CHECK(w.network_up());
}

TEST(supervisor_falls_back_to_own_ap_after_the_connect_timeout) {
  WifiSupervisor w;
  w.start(1000, true, true);
  CHECK_EQ(w.step(1000 + WIFI_STA_CONNECT_TIMEOUT_MS - 1, false), WifiSupervisor::NONE);
  CHECK_EQ(w.phase(), WifiSupervisor::JOINING);
  CHECK_EQ(w.step(1000 + WIFI_STA_CONNECT_TIMEOUT_MS, false), WifiSupervisor::AP_START);
  CHECK_EQ(w.phase(), WifiSupervisor::AP_UP);
  CHECK(w.network_up());
  // The AP stays until the next restart, whatever the station reports.
  CHECK_EQ(w.step(1000 + 10 * WIFI_STA_RETRY_MS, true), WifiSupervisor::NONE);
  CHECK_EQ(w.phase(), WifiSupervisor::AP_UP);
}

TEST(supervisor_without_fallback_retries_every_retry_interval) {
  WifiSupervisor w;
  w.start(0, true, false);
  CHECK_EQ(w.step(WIFI_STA_CONNECT_TIMEOUT_MS, false), WifiSupervisor::NONE);   // no AP
  CHECK_EQ(w.step(WIFI_STA_RETRY_MS - 1, false), WifiSupervisor::NONE);
  CHECK_EQ(w.step(WIFI_STA_RETRY_MS, false), WifiSupervisor::STA_REBEGIN);
  CHECK_EQ(w.step(2 * WIFI_STA_RETRY_MS - 1, false), WifiSupervisor::NONE);
  CHECK_EQ(w.step(2 * WIFI_STA_RETRY_MS, false), WifiSupervisor::STA_REBEGIN);
  CHECK_EQ(w.phase(), WifiSupervisor::JOINING);
}

TEST(supervisor_rejoins_at_once_after_a_drop) {
  // The 1200/2400-baud upload touch calls WiFi.disconnect(): ASSOC_LEAVE, no auto-reconnect.
  const uint32_t t0 = 0xFFFFF000u;   // across the millis() wrap
  WifiSupervisor w;
  w.start(t0, true, true);
  w.step(t0 + 10, true);
  CHECK_EQ(w.phase(), WifiSupervisor::STA_UP);
  CHECK_EQ(w.step(t0 + 20, false), WifiSupervisor::STA_REBEGIN);
  CHECK_EQ(w.phase(), WifiSupervisor::STA_LOST);
  CHECK(!w.network_up());
  // No AP fallback after the station has been up once; retries at the retry interval.
  CHECK_EQ(w.step(t0 + 20 + WIFI_STA_CONNECT_TIMEOUT_MS, false), WifiSupervisor::NONE);
  CHECK_EQ(w.step(t0 + 20 + WIFI_STA_RETRY_MS, false), WifiSupervisor::STA_REBEGIN);
  CHECK_EQ(w.step(t0 + 30 + WIFI_STA_RETRY_MS, true), WifiSupervisor::NONE);
  CHECK_EQ(w.phase(), WifiSupervisor::STA_UP);
}

TEST(supervisor_without_ssid) {
  WifiSupervisor w;
  CHECK_EQ(w.start(5, false, true), WifiSupervisor::AP_START);
  CHECK_EQ(w.phase(), WifiSupervisor::AP_UP);
  WifiSupervisor n;
  CHECK_EQ(n.start(5, false, false), WifiSupervisor::NONE);
  CHECK_EQ(n.phase(), WifiSupervisor::NO_NETWORK);
  CHECK_EQ(n.step(5 + 10 * WIFI_STA_RETRY_MS, true), WifiSupervisor::NONE);
}

TEST(wifi_status_values) {
  CHECK_EQ(wifi_status(false, WifiSupervisor::STA_UP, true), DS_WIFI_STATUS_OFF);
  CHECK_EQ(wifi_status(true, WifiSupervisor::IDLE, false), DS_WIFI_STATUS_CONNECTING);
  CHECK_EQ(wifi_status(true, WifiSupervisor::JOINING, false), DS_WIFI_STATUS_CONNECTING);
  CHECK_EQ(wifi_status(true, WifiSupervisor::STA_LOST, true), DS_WIFI_STATUS_CONNECTING);
  CHECK_EQ(wifi_status(true, WifiSupervisor::STA_UP, false), DS_WIFI_STATUS_STATION_UP);
  CHECK_EQ(wifi_status(true, WifiSupervisor::STA_UP, true), DS_WIFI_STATUS_RELAY_HEARD);
  CHECK_EQ(wifi_status(true, WifiSupervisor::AP_UP, false), DS_WIFI_STATUS_AP_UP);
  CHECK_EQ(wifi_status(true, WifiSupervisor::AP_UP, true), DS_WIFI_STATUS_RELAY_HEARD);
  CHECK_EQ(wifi_status(true, WifiSupervisor::NO_NETWORK, false), DS_WIFI_STATUS_OFF);
}

TESTKIT_MAIN()

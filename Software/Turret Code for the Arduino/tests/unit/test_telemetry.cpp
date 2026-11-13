// test_telemetry.cpp -- Telemetry against docs/architecture.md 4.1 / 4.3 and protocol_spec.py.
//
//   test_telemetry          the unit tests (tests/run.sh)
//   test_telemetry --dump   one line per named snapshot: "CASE <name> <frame hex>", for
//                           relay-pc/tests/test_telemetry_c.py, which decodes every frame with the
//                           generated Python codec (dsentry/protocol.py) and compares it with values
//                           derived there from the contract, byte for byte
//
// KEY SAFETY TEST: velocities_saturate_instead_of_wrapping. A pan rate beyond the i16 range must
// read as the largest rate, never wrap to the opposite sign (the dashboard and any future
// consumer would see the turret turning the wrong way).
//
// SPDX-License-Identifier: MIT
#include <algorithm>
#include <cmath>
#include <cstdio>
#include <cstring>
#include <limits>
#include <string>
#include <vector>

#include "ControlLink.h"
#include "Debug.h"
#include "Lcd.h"
#include "Leds.h"
#include "Telemetry.h"
#include "hal.h"
#include "hal_host.h"
#include "testkit.h"

namespace {

const uint32_t kPeriodMs = MS_PER_S / TELEMETRY_USB_HZ;
const float kInf = std::numeric_limits<float>::infinity();
const float kNan = std::numeric_limits<float>::quiet_NaN();

void setLcd(TelemetrySnapshot& s, const char* text) {
  std::memset(s.lcd, ' ', sizeof s.lcd);
  std::memcpy(s.lcd, text, std::min(std::strlen(text), sizeof s.lcd));
}

// One snapshot per flag, in DS_TELEM_FLAG_* bit order.
struct FlagCase {
  const char* name;
  bool TelemetrySnapshot::*member;
  uint16_t bit;
};
const FlagCase kFlags[] = {
    {"linkConnected", &TelemetrySnapshot::linkConnected, DS_TELEM_FLAG_LINK_CONNECTED},
    {"flywheelOn", &TelemetrySnapshot::flywheelOn, DS_TELEM_FLAG_FLYWHEEL_ON},
    {"flywheelReady", &TelemetrySnapshot::flywheelReady, DS_TELEM_FLAG_FLYWHEEL_READY},
    {"strokeActive", &TelemetrySnapshot::strokeActive, DS_TELEM_FLAG_STROKE_ACTIVE},
    {"triggerPulled", &TelemetrySnapshot::triggerPulled, DS_TELEM_FLAG_TRIGGER_PULLED},
    {"homing", &TelemetrySnapshot::homing, DS_TELEM_FLAG_HOMING},
    {"calibrated", &TelemetrySnapshot::calibrated, DS_TELEM_FLAG_CALIBRATED},
    {"powerToggleOn", &TelemetrySnapshot::powerToggleOn, DS_TELEM_FLAG_POWER_TOGGLE_ON},
    {"modeToggleWifi", &TelemetrySnapshot::modeToggleWifi, DS_TELEM_FLAG_MODE_TOGGLE_WIFI},
    {"stallEnabled", &TelemetrySnapshot::stallEnabled, DS_TELEM_FLAG_STALL_ENABLED},
    {"emptyMagBlink", &TelemetrySnapshot::emptyMagBlink, DS_TELEM_FLAG_EMPTY_MAG_BLINK},
    {"driversEnabled", &TelemetrySnapshot::driversEnabled, DS_TELEM_FLAG_DRIVERS_ENABLED},
    {"espLegacyFirmware", &TelemetrySnapshot::espLegacyFirmware, DS_TELEM_FLAG_ESP_LEGACY_FIRMWARE},
    {"lcdPresent", &TelemetrySnapshot::lcdPresent, DS_TELEM_FLAG_LCD_PRESENT},
    {"espAlive", &TelemetrySnapshot::espAlive, DS_TELEM_FLAG_ESP_ALIVE},
};

struct Case {
  std::string name;
  TelemetrySnapshot snap;
  uint8_t seq;
};

// The snapshots the Python test decodes. Keep in step with EXPECTED in test_telemetry_c.py.
std::vector<Case> dumpCases() {
  std::vector<Case> out;
  {  // protocol_spec.py's TelemetryPacket example, from module units
    TelemetrySnapshot s;
    s.uptimeMs = 123456;
    s.state = DS_STATE_ARMED;
    s.fault = DS_FAULT_NONE;
    s.fireMode = DS_FIRE_MODE_FULL;
    s.linkSource = DS_LINK_SRC_BLE;
    s.linkConnected = s.flywheelOn = s.flywheelReady = s.calibrated = s.powerToggleOn = true;
    s.driversEnabled = s.lcdPresent = s.espAlive = true;
    s.panDeg = -45.5f;
    s.tiltDeg = 12.34f;
    s.panVelDegS = -90.0f;
    s.tiltVelDegS = 0.0f;
    s.homePanDeg = 0.0f;
    s.homeTiltDeg = 3.0f;
    s.ammo = 12;
    s.magCapacity = 15;
    s.shotsTotal = 3;
    s.flywheelDuty = 113;
    s.servoDeg = 20.0f;
    s.stallSample = 512;
    s.stallBaseline = 510;
    s.stallPeak = 7;
    s.loopHz = 9876;
    s.ctrlRxHz = 100;
    s.crcErrors = 2;
    s.seqGaps = 1;
    s.stepJitterPermille = 35;
    s.lastCtrlSeq = 7;
    s.espMode = DS_ESP_MODE_BLE;
    s.led0.r = 127;
    s.led0.g = 47;
    s.led0.b = 0;
    setLcd(s, "ARMED F  AMMO:12BT     Connected");
    s.padState = DS_PAD_STATE_CONNECTED;
    s.padBattery = 80;
    s.padLx = 128;
    s.padLy = -300;
    s.padRx = -512;
    s.padRy = 511;
    s.padLt = 512;
    s.padRt = 1023;
    s.padButtons = DS_BTN_A | DS_BTN_LB | DS_BTN_RS_CLICK | DS_BTN_XBOX;
    out.push_back({"example", s, 200});
  }
  {  // the controller block at its extremes, every button bit
    TelemetrySnapshot s;
    s.padState = DS_PAD_STATE_BLOCKED_BY_HCI;
    s.padBattery = 100;
    s.padLx = -JOY_AXIS_RANGE;
    s.padLy = JOY_AXIS_RANGE - 1;
    s.padRx = JOY_AXIS_RANGE - 1;
    s.padRy = -JOY_AXIS_RANGE;
    s.padLt = TRIGGER_AXIS_RANGE;
    s.padRt = 0;
    s.padButtons = DS_BTN_ALL;
    out.push_back({"pad_extremes", s, 5});
  }
  {  // rounding: half away from zero, in both directions
    TelemetrySnapshot s;
    s.panDeg = 0.125f;        // 12.5 -> 13
    s.tiltDeg = -0.125f;      // -12.5 -> -13
    s.panVelDegS = 0.375f;    // 37.5 -> 38
    s.tiltVelDegS = -0.375f;  // -37.5 -> -38
    s.homePanDeg = 0.124f;    // 12.4 -> 12
    s.homeTiltDeg = -0.126f;  // -12.6 -> -13
    s.servoDeg = 179.5f;      // -> 180
    out.push_back({"rounding", s, 1});
  }
  {  // everything at or beyond its maximum
    TelemetrySnapshot s;
    s.uptimeMs = UINT32_MAX;
    s.panDeg = 1e9f;          // -> INT32_MAX
    s.tiltDeg = kInf;         // -> INT32_MAX
    s.panVelDegS = 400.0f;    // 40000 cdeg/s -> INT16_MAX, never negative
    s.tiltVelDegS = kInf;
    s.homePanDeg = 1e7f;      // 1e9 cdeg: fits, exactly
    s.homeTiltDeg = 3e7f;     // 3e9 cdeg -> INT32_MAX
    s.ammo = 255;
    s.magCapacity = 255;
    s.shotsTotal = 65535;
    s.flywheelDuty = 255;
    s.servoDeg = 300.0f;      // -> 255
    s.stallSample = s.stallBaseline = s.stallPeak = 1023;
    s.loopHz = 70000;         // saturates at 65535
    s.ctrlRxHz = 65535;
    s.crcErrors = 70000;      // wraps: 70000 - 65536 = 4464
    s.seqGaps = UINT32_MAX;   // wraps: 65535
    s.stepJitterPermille = 100000;   // saturates
    s.lastCtrlSeq = 255;
    s.espMode = DS_ESP_MODE_UNKNOWN;
    s.led0.r = s.led0.g = s.led0.b = 127;
    out.push_back({"saturate_hi", s, 255});
  }
  {  // everything at or beyond its minimum
    TelemetrySnapshot s;
    s.panDeg = -1e9f;
    s.tiltDeg = -kInf;
    s.panVelDegS = -400.0f;
    s.tiltVelDegS = -327.68f;   // exactly the i16 minimum
    s.homePanDeg = -0.004f;     // -0.4 -> 0 (not -0)
    s.homeTiltDeg = -1e7f;
    s.servoDeg = -5.0f;         // -> 0
    s.crcErrors = 65536;        // wraps to 0
    out.push_back({"saturate_lo", s, 0});
  }
  {  // NaN anywhere goes out as 0
    TelemetrySnapshot s;
    s.panDeg = s.tiltDeg = s.panVelDegS = s.tiltVelDegS = s.homePanDeg = s.homeTiltDeg = kNan;
    s.servoDeg = kNan;
    out.push_back({"nan", s, 2});
  }
  for (const FlagCase& f : kFlags) {
    TelemetrySnapshot s;
    s.*f.member = true;
    out.push_back({std::string("flag_") + f.name, s, 3});
  }
  {  // NULs in the LCD mirror go out as spaces
    TelemetrySnapshot s;
    std::memset(s.lcd, 0, sizeof s.lcd);
    std::memcpy(s.lcd, "SYSTEM OFF", 10);
    out.push_back({"lcd_nul", s, 4});
  }
  return out;
}

std::string hex(const uint8_t* p, size_t n) {
  static const char* digits = "0123456789abcdef";
  std::string s;
  for (size_t i = 0; i < n; i++) {
    s += digits[p[i] >> 4];
    s += digits[p[i] & 15];
  }
  return s;
}

int dump() {
  for (const Case& c : dumpCases()) {
    uint8_t frame[DS_TELEMETRY_FRAME_LEN];
    const size_t n = Telemetry::encode(c.snap, c.seq, frame);
    std::printf("CASE %s %s\n", c.name.c_str(), hex(frame, n).c_str());
  }
  return 0;
}

ds_telemetry_packet_t decodeCase(const char* name) {
  ds_telemetry_packet_t p;
  std::memset(&p, 0, sizeof p);
  for (const Case& c : dumpCases()) {
    if (c.name != name) continue;
    uint8_t frame[DS_TELEMETRY_FRAME_LEN];
    const size_t n = Telemetry::encode(c.snap, c.seq, frame);
    CHECK(ds_telemetry_decode(frame, n, &p));
  }
  return p;
}

// Every TelemetryPacket in a byte stream, through the generated C deframer.
std::vector<ds_telemetry_packet_t> deframe(const std::string& wire, uint32_t* junkOrOther = nullptr) {
  std::vector<ds_telemetry_packet_t> out;
  ds_deframer_t d;
  ds_deframer_init(&d);
  uint32_t other = 0;
  for (unsigned char b : wire) {
    uint8_t type = ds_deframer_feed(&d, b);
    while (type != DS_PKT_NONE) {
      if (type == DS_PKT_TELEMETRY) {
        ds_telemetry_packet_t p;
        if (ds_telemetry_decode(ds_deframer_frame(&d), ds_deframer_frame_len(&d), &p)) out.push_back(p);
      } else {
        other++;
      }
      type = ds_deframer_poll(&d);
    }
  }
  if (junkOrOther) *junkOrOther = other;
  return out;
}

struct Rig {
  Debug debug;
  Telemetry tel;
  Rig() {
    hal_host::reset();
    hal::begin();
    DebugSources none;
    debug.begin(none);
  }
  // `ms` loops of 1 ms each, draining USB as the HAL would between modules.
  std::string run(uint32_t ms) {
    std::string wire;
    for (uint32_t i = 0; i < ms; i++) {
      for (int k = 0; k < 10; k++) {   // service() every 100 us inside the millisecond
        hal_host::advanceUs(US_PER_MS / 10);
        hal::service(hal::nowUs());
      }
      tel.update(hal::nowMs());
      wire += hal_host::usbTakeTx();
    }
    return wire + drain();
  }
  // Let the paced USB queue empty (service() only, no update()): the last frame queued is
  // still on its way out when run() stops.
  static std::string drain() {
    std::string wire;
    while (hal::usbTxFree() < DEBUG_TX_BUFFER_BYTES) {
      hal_host::advanceUs(US_PER_MS / 10);
      hal::service(hal::nowUs());
      wire += hal_host::usbTakeTx();
    }
    hal_host::advanceUs(US_PER_MS);
    hal::service(hal::nowUs());
    return wire + hal_host::usbTakeTx();
  }
};

}  // namespace

// ---- conversions ----------------------------------------------------------------------------------------
TEST(centidegrees_round_half_away_from_zero) {
  CHECK_EQ(Telemetry::centi32(12.34f), 1234);
  CHECK_EQ(Telemetry::centi32(-45.5f), -4550);
  CHECK_EQ(Telemetry::centi32(0.125f), 13);
  CHECK_EQ(Telemetry::centi32(-0.125f), -13);
  CHECK_EQ(Telemetry::centi32(0.124f), 12);
  CHECK_EQ(Telemetry::centi32(-0.126f), -13);
  CHECK_EQ(Telemetry::centi32(0.0f), 0);
  CHECK_EQ(Telemetry::centi32(-0.004f), 0);
  CHECK_EQ(Telemetry::centi16(0.375f), 38);
  CHECK_EQ(Telemetry::centi16(-0.375f), -38);
  CHECK_EQ(Telemetry::centi16(-90.0f), -9000);
}

TEST(positions_saturate_and_nan_is_zero) {
  CHECK_EQ(Telemetry::centi32(1e9f), INT32_MAX);
  CHECK_EQ(Telemetry::centi32(-1e9f), INT32_MIN);
  CHECK_EQ(Telemetry::centi32(kInf), INT32_MAX);
  CHECK_EQ(Telemetry::centi32(-kInf), INT32_MIN);
  CHECK_EQ(Telemetry::centi32(kNan), 0);
  CHECK_EQ(Telemetry::centi32(1e7f), 1000000000);   // large but in range: exact
  // the largest float below 2^31 / 100 still converts without overflow (no UB in the cast)
  CHECK(Telemetry::centi32(21474835.0f) > 2000000000);
}

TEST(velocities_saturate_instead_of_wrapping) {
  CHECK_EQ(Telemetry::centi16(400.0f), INT16_MAX);    // 40000 would wrap to -25536
  CHECK_EQ(Telemetry::centi16(-400.0f), INT16_MIN);
  CHECK_EQ(Telemetry::centi16(327.67f), INT16_MAX);
  CHECK_EQ(Telemetry::centi16(-327.68f), INT16_MIN);
  CHECK_EQ(Telemetry::centi16(kInf), INT16_MAX);
  CHECK_EQ(Telemetry::centi16(kNan), 0);
  for (float v = 300.0f; v < 1e6f; v *= 1.37f) {
    if (!CHECK(Telemetry::centi16(v) > 0)) return;     // never the wrong sign
    if (!CHECK(Telemetry::centi16(-v) < 0)) return;
  }
  // through build(): the wire field, not just the helper
  TelemetrySnapshot s;
  s.panVelDegS = PAN_MAX_DEG_PER_S * 10.0f;
  ds_telemetry_packet_t p;
  Telemetry::build(s, 0, p);
  CHECK_EQ((int)p.panVelCdegS, (int)INT16_MAX);
}

// ---- build() --------------------------------------------------------------------------------------------
TEST(each_flag_sets_exactly_its_bit) {
  uint16_t all = 0;
  for (const FlagCase& f : kFlags) {
    TelemetrySnapshot s;
    s.*f.member = true;
    ds_telemetry_packet_t p;
    Telemetry::build(s, 0, p);
    CHECK_EQ((unsigned)p.flags, (unsigned)f.bit);
    all |= f.bit;
  }
  CHECK_EQ((unsigned)all, (unsigned)DS_TELEM_FLAG_ALL);   // every defined bit has a source
  TelemetrySnapshot none;
  ds_telemetry_packet_t p;
  Telemetry::build(none, 0, p);
  CHECK_EQ((unsigned)p.flags, 0u);
}

TEST(example_snapshot_builds_the_protocol_example) {
  const ds_telemetry_packet_t p = decodeCase("example");
  CHECK_EQ(p.version, DS_TELEMETRY_VERSION);
  CHECK_EQ(p.seq, 200);
  CHECK_EQ(p.uptimeMs, 123456u);
  CHECK_EQ(p.state, DS_STATE_ARMED);
  CHECK_EQ(p.fireMode, DS_FIRE_MODE_FULL);
  CHECK_EQ(p.linkSource, DS_LINK_SRC_BLE);
  CHECK_EQ((unsigned)p.flags, 0x68C7u);
  CHECK_EQ((int)p.panCdeg, -4550);
  CHECK_EQ((int)p.tiltCdeg, 1234);
  CHECK_EQ((int)p.panVelCdegS, -9000);
  CHECK_EQ((int)p.homeTiltCdeg, 300);
  CHECK_EQ((unsigned)p.servoDeg, 20u);
  CHECK_EQ((unsigned)p.loopHz, 9876u);
  CHECK_EQ((unsigned)p.ledG, 47u);
  CHECK_EQ(std::string(p.lcd, DS_TELEMETRY_LCD_LEN), "ARMED F  AMMO:12BT     Connected");
  CHECK_EQ((unsigned)p.padState, (unsigned)DS_PAD_STATE_CONNECTED);
  CHECK_EQ((unsigned)p.padBattery, 80u);
  CHECK_EQ((int)p.padLx, 128);
  CHECK_EQ((int)p.padLy, -300);
  CHECK_EQ((int)p.padRx, -512);
  CHECK_EQ((int)p.padRy, 511);
  CHECK_EQ((unsigned)p.padLt, 512u);
  CHECK_EQ((unsigned)p.padRt, 1023u);
  CHECK_EQ((unsigned)p.padButtons, 0x6011u);
}

TEST(controller_block_extremes_and_defaults) {
  const ds_telemetry_packet_t x = decodeCase("pad_extremes");
  CHECK_EQ((unsigned)x.padState, (unsigned)DS_PAD_STATE_BLOCKED_BY_HCI);
  CHECK_EQ((unsigned)x.padBattery, 100u);
  CHECK_EQ((int)x.padLx, -JOY_AXIS_RANGE);
  CHECK_EQ((int)x.padLy, JOY_AXIS_RANGE - 1);
  CHECK_EQ((int)x.padRx, JOY_AXIS_RANGE - 1);
  CHECK_EQ((int)x.padRy, -JOY_AXIS_RANGE);
  CHECK_EQ((unsigned)x.padLt, (unsigned)TRIGGER_AXIS_RANGE);
  CHECK_EQ((unsigned)x.padRt, 0u);
  CHECK_EQ((unsigned)x.padButtons, (unsigned)DS_BTN_ALL);
  CHECK_EQ((unsigned)DS_BTN_ALL, 0xFFFFu);   // bits 12-15 (stick clicks, Xbox, Share) defined
  // A snapshot with no source says "no source" and "unknown", not "connected" and 0 %.
  const ds_telemetry_packet_t d = decodeCase("nan");
  CHECK_EQ((unsigned)d.padState, (unsigned)DS_PAD_STATE_NONE);
  CHECK_EQ((unsigned)d.padBattery, (unsigned)DS_PAD_BATTERY_UNKNOWN);
  CHECK_EQ((unsigned)d.padButtons, 0u);
  ds_telemetry_packet_t init;
  ds_telemetry_init(&init);
  CHECK_EQ((unsigned)init.padState, (unsigned)DS_PAD_STATE_NONE);
  CHECK_EQ((unsigned)init.padBattery, (unsigned)DS_PAD_BATTERY_UNKNOWN);
}

// ---- the controller block (padBlock) ------------------------------------------------------------------------
namespace {
ControlFrame liveFrame(uint8_t source) {
  ControlFrame f;
  f.valid = true;
  f.connected = true;
  f.source = source;
  f.rx = 300;
  f.ry = -200;
  f.rt = 900;
  f.lx = -100;
  f.ly = 50;
  f.lt = 40;
  f.buttons = DS_BTN_A | DS_BTN_SHARE;
  f.battery = 64;
  f.padState = DS_PAD_STATE_CONNECTED;
  return f;
}

void dirty(TelemetrySnapshot& s) {   // padBlock() must set every field, never keep old values
  s.padState = DS_PAD_STATE_FOUND;
  s.padBattery = 1;
  s.padLx = s.padLy = s.padRx = s.padRy = 77;
  s.padLt = s.padRt = 77;
  s.padButtons = DS_BTN_B;
}
}  // namespace

TEST(pad_block_copies_a_connected_frame) {
  TelemetrySnapshot s;
  dirty(s);
  Telemetry::padBlock(liveFrame(DS_LINK_SRC_BLE), DS_LINK_SRC_BLE, s);
  CHECK_EQ(s.padState, (uint8_t)DS_PAD_STATE_CONNECTED);
  CHECK_EQ(s.padBattery, (uint8_t)64);
  CHECK_EQ(s.padLx, (int16_t)-100);
  CHECK_EQ(s.padLy, (int16_t)50);
  CHECK_EQ(s.padRx, (int16_t)300);
  CHECK_EQ(s.padRy, (int16_t)-200);
  CHECK_EQ(s.padLt, (uint16_t)40);
  CHECK_EQ(s.padRt, (uint16_t)900);
  CHECK_EQ(s.padButtons, (uint16_t)(DS_BTN_A | DS_BTN_SHARE));
}

TEST(pad_block_of_a_frame_that_is_not_connected_is_neutral_with_its_state) {
  ControlFrame f = liveFrame(DS_LINK_SRC_BLE);
  f.connected = false;
  f.padState = DS_PAD_STATE_SCANNING;
  TelemetrySnapshot s;
  dirty(s);
  Telemetry::padBlock(f, DS_LINK_SRC_BLE, s);
  CHECK_EQ(s.padState, (uint8_t)DS_PAD_STATE_SCANNING);
  CHECK_EQ(s.padBattery, (uint8_t)64);   // the ESP32 still knows the last level
  CHECK_EQ(s.padLx, (int16_t)0);
  CHECK_EQ(s.padLy, (int16_t)0);
  CHECK_EQ(s.padRx, (int16_t)0);
  CHECK_EQ(s.padRy, (int16_t)0);
  CHECK_EQ(s.padLt, (uint16_t)0);
  CHECK_EQ(s.padRt, (uint16_t)0);
  CHECK_EQ(s.padButtons, (uint16_t)0);
}

TEST(pad_block_with_no_source_is_none_and_neutral) {
  TelemetrySnapshot s;
  dirty(s);
  Telemetry::padBlock(liveFrame(DS_LINK_SRC_BLE), DS_LINK_SRC_NONE, s);   // stale, dropped
  CHECK_EQ(s.padState, (uint8_t)DS_PAD_STATE_NONE);
  CHECK_EQ(s.padBattery, (uint8_t)DS_PAD_BATTERY_UNKNOWN);
  CHECK_EQ(s.padRx, (int16_t)0);
  CHECK_EQ(s.padRt, (uint16_t)0);
  CHECK_EQ(s.padButtons, (uint16_t)0);
  dirty(s);
  Telemetry::padBlock(ControlFrame(), DS_LINK_SRC_USB, s);                 // no frame yet
  CHECK_EQ(s.padState, (uint8_t)DS_PAD_STATE_NONE);
  CHECK_EQ(s.padBattery, (uint8_t)DS_PAD_BATTERY_UNKNOWN);
  CHECK_EQ(s.padLx, (int16_t)0);
  CHECK_EQ(s.padButtons, (uint16_t)0);
}

TEST(collect_reports_the_active_usb_source_as_the_controller) {
  Rig rig;
  static ControlLink link;   // static: the link owns large buffers
  link.begin(rig.debug, hal::nowMs(), hal::nowUs());
  TelemetrySources src;
  src.link = &link;
  rig.tel.begin(src, link);
  TelemetrySnapshot s;
  rig.tel.collect(hal::nowMs(), s);
  CHECK_EQ(s.padState, (uint8_t)DS_PAD_STATE_NONE);   // nothing has arrived yet
  ds_control_packet_t p;
  ds_control_init(&p);
  p.flags = DS_CTRL_FLAG_CONNECTED;
  p.rx = -400;
  p.ry = 250;
  p.rt = 700;
  p.buttons = DS_BTN_LB | DS_BTN_XBOX;
  p.seq = 9;
  uint8_t f[DS_CONTROL_FRAME_LEN];
  hal_host::usbInject(f, ds_control_encode(&p, f));
  for (int i = 0; i < 10; i++) {
    hal_host::advanceUs(US_PER_MS);
    link.update(hal::nowMs(), hal::nowUs());
  }
  rig.tel.collect(hal::nowMs(), s);
  CHECK_EQ(s.linkSource, (uint8_t)DS_LINK_SRC_USB);
  CHECK_EQ(s.padState, (uint8_t)DS_PAD_STATE_CONNECTED);
  CHECK_EQ(s.padBattery, (uint8_t)DS_PAD_BATTERY_UNKNOWN);
  CHECK_EQ(s.padRx, (int16_t)-400);
  CHECK_EQ(s.padRy, (int16_t)250);
  CHECK_EQ(s.padRt, (uint16_t)700);
  CHECK_EQ(s.padLx, (int16_t)0);
  CHECK_EQ(s.padLt, (uint16_t)0);
  CHECK_EQ(s.padButtons, (uint16_t)(DS_BTN_LB | DS_BTN_XBOX));
  // LINK_TIMEOUT_MS later the source is gone: "no source", neutral.
  for (uint32_t i = 0; i <= LINK_TIMEOUT_MS; i++) {
    hal_host::advanceUs(US_PER_MS);
    link.update(hal::nowMs(), hal::nowUs());
  }
  rig.tel.collect(hal::nowMs(), s);
  CHECK_EQ(s.linkSource, (uint8_t)DS_LINK_SRC_NONE);
  CHECK_EQ(s.padState, (uint8_t)DS_PAD_STATE_NONE);
  CHECK_EQ(s.padRx, (int16_t)0);
  CHECK_EQ(s.padButtons, (uint16_t)0);
}

TEST(counters_wrap_and_rates_saturate) {
  const ds_telemetry_packet_t hi = decodeCase("saturate_hi");
  CHECK_EQ((unsigned)hi.loopHz, 65535u);
  CHECK_EQ((unsigned)hi.crcErrors, 70000u - 65536u);
  CHECK_EQ((unsigned)hi.seqGaps, 65535u);
  CHECK_EQ((unsigned)hi.stepJitterPermille, 65535u);
  CHECK_EQ((unsigned)hi.servoDeg, 255u);
  CHECK_EQ((int)hi.panVelCdegS, (int)INT16_MAX);
  const ds_telemetry_packet_t lo = decodeCase("saturate_lo");
  CHECK_EQ((unsigned)lo.crcErrors, 0u);
  CHECK_EQ((unsigned)lo.servoDeg, 0u);
  CHECK_EQ((int)lo.homePanCdeg, 0);
  CHECK_EQ((int)lo.tiltVelCdegS, (int)INT16_MIN);
  const ds_telemetry_packet_t nan = decodeCase("nan");
  CHECK_EQ((int)nan.panCdeg, 0);
  CHECK_EQ((int)nan.tiltVelCdegS, 0);
  CHECK_EQ((unsigned)nan.servoDeg, 0u);
  const ds_telemetry_packet_t nul = decodeCase("lcd_nul");
  CHECK_EQ(std::string(nul.lcd, DS_TELEMETRY_LCD_LEN), "SYSTEM OFF" + std::string(22, ' '));
}

TEST(snapshot_defaults_are_the_absent_module_values) {
  TelemetrySnapshot s;
  CHECK_EQ(std::string(s.lcd, sizeof s.lcd), std::string(DS_TELEMETRY_LCD_LEN, ' '));
  CHECK_EQ(s.espMode, DS_ESP_MODE_UNKNOWN);
  CHECK_EQ(s.state, DS_STATE_OFF);
  Telemetry tel;
  TelemetrySources none;
  tel.begin(none, nullptr);
  TelemetrySnapshot c;
  c.ammo = 99;   // collect() starts from a clean snapshot
  tel.collect(4321, c);
  CHECK_EQ(c.uptimeMs, 4321u);
  CHECK_EQ(c.ammo, 0);
  CHECK_EQ(c.magCapacity, MAG_CAPACITY);
  CHECK_EQ(c.espMode, DS_ESP_MODE_UNKNOWN);
}

// ---- collect() from the modules this stage owns -----------------------------------------------------------
TEST(collect_reads_led0_and_the_lcd_mirror) {
  Rig rig;
  Leds leds;
  Lcd lcd;
  leds.begin(rig.debug);
  lcd.begin(rig.debug);
  DisplayModel m;
  m.state = DS_STATE_ARMED;
  m.fireMode = DS_FIRE_MODE_FULL;
  m.linkConnected = true;
  m.ammo = 12;
  for (int i = 0; i < 300; i++) {
    hal_host::advanceUs(US_PER_MS);
    leds.update(hal::nowMs(), hal::nowUs(), m);
    lcd.update(hal::nowMs(), m);
  }
  TelemetrySources src;
  src.leds = &leds;
  src.lcd = &lcd;
  src.debug = &rig.debug;
  rig.tel.begin(src, nullptr);
  TelemetrySnapshot s;
  rig.tel.collect(hal::nowMs(), s);
  CHECK_EQ(s.led0.r, leds.pixel0().r);
  CHECK_EQ(s.led0.g, leds.pixel0().g);
  CHECK_EQ(s.led0.r, Leds::scale((uint8_t)((LED_COLOR_ARMED_FULL >> COLOR_SHIFT_R) & COLOR_BYTE_MASK)));
  char mirror[DS_TELEMETRY_LCD_LEN];
  lcd.mirror(mirror);
  CHECK_EQ(std::string(s.lcd, DS_TELEMETRY_LCD_LEN), std::string(mirror, DS_TELEMETRY_LCD_LEN));
  CHECK(s.lcdPresent);
}

// ---- update(): rate, atomic USB writes, the ControlLink offer ------------------------------------------------
TEST(usb_frames_at_telemetry_usb_hz_with_consecutive_seq) {
  Rig rig;
  TelemetrySources none;
  rig.tel.begin(none, nullptr);
  const uint32_t t0 = hal::nowMs();
  const std::string wire = rig.run(MS_PER_S);
  uint32_t other = 0;
  const std::vector<ds_telemetry_packet_t> frames = deframe(wire, &other);
  CHECK_EQ(frames.size(), (size_t)TELEMETRY_USB_HZ);
  CHECK_EQ(other, 0u);
  CHECK_EQ(wire.size(), frames.size() * DS_TELEMETRY_FRAME_LEN);   // nothing but whole frames
  for (size_t i = 0; i < frames.size(); i++) {
    CHECK_EQ(frames[i].seq, (uint8_t)i);
    CHECK_EQ(frames[i].uptimeMs, t0 + (uint32_t)(i + 1) * kPeriodMs);   // deadline schedule
  }
  CHECK_EQ(rig.tel.usbSent(), (uint32_t)TELEMETRY_USB_HZ);
  CHECK_EQ(rig.tel.usbDropped(), 0u);
  CHECK_EQ(rig.tel.seq(), (uint8_t)TELEMETRY_USB_HZ);
}

TEST(seq_wraps_after_255) {
  Rig rig;
  TelemetrySources none;
  rig.tel.begin(none, nullptr);
  const std::string wire = rig.run(300 * kPeriodMs);
  const std::vector<ds_telemetry_packet_t> frames = deframe(wire);
  CHECK_EQ(frames.size(), 300u);
  for (size_t i = 1; i < frames.size(); i++) CHECK_EQ(frames[i].seq, (uint8_t)(frames[i - 1].seq + 1));
}

TEST(no_burst_after_a_stalled_loop) {
  Rig rig;
  TelemetrySources none;
  rig.tel.begin(none, nullptr);
  hal_host::advanceUs((uint64_t)10 * kPeriodMs * US_PER_MS);   // loop() stalled ten periods
  rig.tel.update(hal::nowMs());
  rig.tel.update(hal::nowMs());
  CHECK_EQ(rig.tel.framesBuilt(), 1u);
  rig.run(kPeriodMs - 1);
  CHECK_EQ(rig.tel.framesBuilt(), 1u);
  rig.run(1);
  CHECK_EQ(rig.tel.framesBuilt(), 2u);
}

TEST(a_frame_that_does_not_fit_is_dropped_whole) {
  Rig rig;
  TelemetrySources none;
  rig.tel.begin(none, nullptr);
  // Fill the USB queue until less than one frame fits.
  const std::vector<uint8_t> junk(hal::usbTxFree() - (DS_TELEMETRY_FRAME_LEN - 1), 'x');
  CHECK(hal::usbWrite(junk.data(), junk.size()));
  CHECK(hal::usbTxFree() < DS_TELEMETRY_FRAME_LEN);
  hal_host::advanceUs((uint64_t)kPeriodMs * US_PER_MS);
  rig.tel.update(hal::nowMs());   // due, but cannot fit
  CHECK_EQ(rig.tel.usbDropped(), 1u);
  CHECK_EQ(rig.tel.usbSent(), 0u);
  CHECK_EQ(rig.tel.framesBuilt(), 1u);
  // drain, keep running: later frames go out whole, and no half frame ever appeared
  std::string wire = rig.run(2 * MS_PER_S);
  const std::vector<ds_telemetry_packet_t> frames = deframe(wire);
  CHECK_EQ(frames.size(), rig.tel.usbSent());
  CHECK(frames.size() >= 2u * TELEMETRY_USB_HZ - 2u);
  CHECK_EQ(wire.size(), junk.size() + frames.size() * DS_TELEMETRY_FRAME_LEN);
  CHECK_EQ(frames.front().seq, 1);   // seq 0 was the dropped one: seq counts frames built
}

TEST(every_frame_is_offered_to_control_link) {
  Rig rig;
  static ControlLink link;   // static: the link owns large buffers
  link.begin(rig.debug, hal::nowMs(), hal::nowUs());
  TelemetrySources src;
  src.link = &link;
  rig.tel.begin(src, link);
  rig.run(MS_PER_S);
  CHECK_EQ(rig.tel.framesBuilt(), (uint32_t)TELEMETRY_USB_HZ);
  CHECK_EQ(rig.tel.linkOffered(), rig.tel.framesBuilt());
  CHECK(rig.tel.linkAccepted() <= rig.tel.linkOffered());
  // without a link nothing is offered and USB is unaffected
  Rig solo;
  TelemetrySources none;
  solo.tel.begin(none, nullptr);
  solo.run(MS_PER_S);
  CHECK_EQ(solo.tel.linkOffered(), 0u);
  CHECK_EQ(solo.tel.usbSent(), (uint32_t)TELEMETRY_USB_HZ);
}

int main(int argc, char** argv) {
  if (argc > 1 && std::strcmp(argv[1], "--dump") == 0) return dump();
  return ::testkit::runAll(argc, argv);
}

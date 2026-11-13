// test_turretstate.cpp -- the WHOLE App (every RA4M1 module, wired by App) on the host HAL,
// driven the way the bench drives the board: ControlPackets and shell lines injected on the
// USB pipe at CONTROL_PACKET_HZ, the toggles and A2 on the fake pins, the tilt's bottom panel
// in the stepper physics model. What comes back is read the way the PC reads it: StatusPackets,
// TelemetryPackets and text lines split out of the USB wire bytes, plus the fake's captured
// outputs (EN, ENA duty, servo, LEDs as shown, LCD as displayed).
//
// One test per spec 11 acceptance item 1-17 that can be observed without hardware
// (accept_NN_...), then the failure paths and rules of TurretState (docs/architecture.md 6),
// the shell, and the boot warnings. Expected values come from the spec text and config.h,
// never from the module under test: colours are config.h colours through the one brightness
// function, speeds the config maxima through the documented stick curve, and so on.
//
// The Inputs baseline rule (Inputs.h): the first connected frame after boot, a drop or a source
// change produces no edges, so every rig starts sending neutral connected frames at once, and
// a "press" holds a button for a few packets.
//
// KEY SAFETY TESTS (each one failed when the rule it guards was broken in a scratch copy of
// TurretState / Debug / Lcd: 30 of 32 mutants killed; the two survivors are equivalent, see the
// integration report):
//   accept_13_link_loss_stops_everything_within_link_timeout_and_reconnects_disarmed
//   accept_06_a_arms_and_disarms_and_arm_refused_while_the_trigger_is_held
//   accept_08_flywheels_only_when_permitted_and_ramped
//   disarmed_never_fires_even_with_the_flywheel_gates_off
//   accept_01_power_off_idles_everything_and_power_on_reruns_boot
//   accept_15_stall_twist_stops_backs_off_blinks_and_clears
//   cal_stall_sense_without_a_stall_faults_until_power_cycled
//   accept_14_mode_toggle_changes_want_mode_lcd_and_goes_through_pairing, source_change_is_a_link_loss
//   accept_16_brightness_never_exceeds_the_limit_in_a_long_random_session
//   accept_17_loop_never_blocks_and_the_stats_prove_it
//   shell_matches_whole_words_only
// SERVO ADJUST (the last section; scratch mutants of TurretState / Trigger / Leds / Lcd / Debug:
// 19 of 22 killed alone; the 3 others are second layers: tryArm's teach guard and leaveTeach's
// abort were killed together with the layer in front of them, and the flywheel cut on entering
// is equivalent, because decideFlywheel cuts in the same update, and removing that is killed):
//   servo_adjust_never_spins_the_flywheels_whatever_lb_or_the_flags_say
//   servo_adjust_a_b_x_rt_and_the_d_pad_do_not_do_their_normal_actions
//   servo_adjust_ends_on_link_loss_power_off_mode_toggle_fault_and_state_change
//   servo_adjust_random_session_keeps_every_rule_and_the_brightness_clamp
//
// SPDX-License-Identifier: MIT
#include <algorithm>
#include <cmath>
#include <cstring>
#include <deque>
#include <functional>
#include <memory>
#include <random>
#include <set>
#include <string>
#include <vector>

#include "App.h"
#include "Leds.h"
#include "hal.h"
#include "hal_host.h"
#include "testkit.h"
#include "../xbox_r4_protocol.h"   // the sketch's symlink, found through -I src

// The servo angle in config.h terms: the host HAL records what the servo receives, which
// TRIGGER_SERVO_REVERSED mirrors; the mirror is its own inverse.
static float servoLogical() { return Trigger::physicalDeg(hal_host::servoAngle()); }

namespace {

constexpr uint64_t kStepUs = 100;                        // one loop() every 100 us of virtual time
constexpr uint32_t kCtrlMs = MS_PER_S / CONTROL_PACKET_HZ;
const int16_t kStickMax = JOY_AXIS_RANGE - 1;            // full deflection on the wire
const uint16_t kPulled = (uint16_t)(TRIGGER_AXIS_RANGE * (TRIGGER_PRESS_THRESHOLD + 1.0f) / 2.0f);
const uint16_t kBetween = (uint16_t)(TRIGGER_AXIS_RANGE * (TRIGGER_PRESS_THRESHOLD + TRIGGER_RELEASE_THRESHOLD) / 2.0f);
const uint16_t kReleased = (uint16_t)(TRIGGER_AXIS_RANGE * TRIGGER_RELEASE_THRESHOLD / 2.0f);
// The trigger stroke at config.h's angles (Trigger.h): a sweep out at TRIGGER_SWEEP_DEG_PER_S that
// lands at the first whole ms whose distance at the rate covers REST -> FIRE, FIRE held
// TRIGGER_FORWARD_MS, the sweep back, REST held TRIGGER_RETURN_MS; then, in semi auto,
// TRIGGER_COOLDOWN_MS before the next stroke may start.
uint32_t sweepMsFor(float spanDeg) {
  if (!(TRIGGER_SWEEP_DEG_PER_S > 0.0f) || !(spanDeg > 0.0f)) return 0;
  uint32_t ms = 0;
  while (TRIGGER_SWEEP_DEG_PER_S * (float)ms / (float)MS_PER_S < spanDeg) ms++;
  return ms;
}
const uint32_t kSweepMs = sweepMsFor(std::fabs((float)TRIGGER_FIRE_ANGLE - (float)TRIGGER_REST_ANGLE));
const uint32_t kStrokeMs = kSweepMs + TRIGGER_FORWARD_MS + kSweepMs + TRIGGER_RETURN_MS;
const uint32_t kCoolMs = TRIGGER_COOLDOWN_MS;
// A stroke's first phase (no sweep configured: straight to the FIRE hold).
const Trigger::Phase kFirstPhase = kSweepMs ? Trigger::Phase::SWEEP_OUT : Trigger::Phase::HOLD_FIRE;
// Where the bottom panel is: this far below the power-on position (the barrel starts
// part-way up, as it would after a power cycle).
const float kStopBelowDeg = 30.0f;
const uint16_t kA2Baseline = 400;                        // stall-sense level at rest

int8_t tiltDown() { return TILT_DIR_INVERT ? 1 : -1; }   // motor-step sign of "barrel down"
int64_t stopSteps() { return (int64_t)tiltDown() * (int64_t)std::lround(kStopBelowDeg * TILT_STEPS_PER_DEG); }
bool toggleLevel(bool closed) { return TOGGLE_ACTIVE_LOW ? !closed : closed; }

Rgb scaled(uint32_t rgb) {
  Rgb c;
  c.r = Leds::scale((uint8_t)((rgb >> COLOR_SHIFT_R) & COLOR_BYTE_MASK));
  c.g = Leds::scale((uint8_t)((rgb >> COLOR_SHIFT_G) & COLOR_BYTE_MASK));
  c.b = Leds::scale((uint8_t)(rgb & COLOR_BYTE_MASK));
  return c;
}
bool same(const Rgb& a, const Rgb& b) { return a.r == b.r && a.g == b.g && a.b == b.b; }
// All LED_COUNT LEDs show the same colour (Leds sends one colour to all of them).
bool allLeds(const Rgb& c) {
  for (uint8_t i = 0; i < LED_COUNT; i++)
    if (!same(hal_host::ledShown(i), c)) return false;
  return true;
}
std::string padRow(const std::string& left, const std::string& right) {
  std::string l = left.substr(0, LCD_COLS);
  const size_t room = l.size() + (l.empty() || right.empty() ? 0 : 1) <= (size_t)LCD_COLS
                          ? LCD_COLS - l.size() - (l.empty() || right.empty() ? 0 : 1) : 0;
  const std::string r = right.substr(0, room);
  return l + std::string(LCD_COLS - l.size() - r.size(), ' ') + r;
}
std::string ammoText(unsigned n) { return std::string(LCD_TXT_AMMO_PREFIX) + std::to_string(n); }
bool startsWith(const std::string& s, const std::string& p) { return s.compare(0, p.size(), p) == 0; }

// ---- a scripted ESP32 on the AT link (the same model as test_controllink.cpp's FakeEsp,
// reduced to D-Sentry firmware in BLE mode): byte-timed wire, AT-task latency, a connected pad.
struct FakeEsp {
  double byteUs = (double)UART_BITS_PER_BYTE * (double)US_PER_S / (double)LINK_BAUD;
  double processUs = 1000;
  uint8_t espMode = DS_ESP_MODE_NONE;
  xbox_r4_state_t st{};
  double inLast = 0, outLast = 0;
  std::deque<std::pair<double, uint8_t>> in, out;
  std::deque<std::pair<double, std::string>> todo;
  std::string line;

  FakeEsp() {
    xbox_r4_neutral_report(st.report);
    st.state = XBOX_R4_CONNECTED;
    st.battery = XBOX_R4_BATTERY_UNKNOWN;
  }
  std::string handle(const std::string& req) {
    if (req == "AT") return "OK\r\n";
    if (req == "AT+XBOXSTATE?") {
      xbox_r4_state_t s = st;
      if (espMode != DS_ESP_MODE_BLE) {
        xbox_r4_neutral_report(s.report);
        s.state = XBOX_R4_OFF;
      }
      uint8_t enc[XBOX_R4_STATE_LEN];
      xbox_r4_encode(&s, enc);
      return "+XBOXSTATE: " + std::to_string(XBOX_R4_STATE_LEN) + "|" +
             std::string((const char*)enc, sizeof enc) + "\r\nOK\r\n";
    }
    if (req == "AT+DSCAPS?")
      return "+DSCAPS: " + std::to_string(DS_PROTO_VERSION) + "," + std::to_string(DS_CAP_ALL) + "\r\nOK\r\n";
    if (startsWith(req, "AT+DSS=")) {
      const int want = req[7] - '0';
      espMode = want == DS_RADIO_WIFI ? DS_ESP_MODE_WIFI : DS_ESP_MODE_BLE;
      return "+DSS: " + std::to_string(espMode) + ",77," + std::to_string(DS_WIFI_STATUS_OFF) + "\r\nOK\r\n";
    }
    return "ERROR\r\n";
  }
  void pump(double now) {
    for (unsigned char c : hal_host::linkTakeTx()) {
      inLast = std::max(now, inLast) + byteUs;
      in.push_back({inLast, c});
    }
    while (!in.empty() && in.front().first <= now) {
      const uint8_t c = in.front().second;
      const double t = in.front().first;
      in.pop_front();
      if (c == '\n') {
        std::string req = line;
        if (!req.empty() && req.back() == '\r') req.pop_back();
        todo.push_back({t + processUs, req});
        line.clear();
      } else {
        line += (char)c;
      }
    }
    while (!todo.empty() && todo.front().first <= now) {
      const auto job = todo.front();
      todo.pop_front();
      for (unsigned char c : handle(job.second)) {
        outLast = std::max(job.first, outLast) + byteUs;
        out.push_back({outLast, c});
      }
    }
    while (!out.empty() && out.front().first <= now) {
      const uint8_t c = out.front().second;
      out.pop_front();
      hal_host::linkInject(&c, 1);
    }
  }
};

// ---- the rig ---------------------------------------------------------------------------------------
struct Pad {             // what the PC relay / bench console sends over USB
  bool send = true;      // a ControlPacket every 1000/CONTROL_PACKET_HZ ms
  bool connected = true; // flags.connected
  int16_t rx = 0;        // + = stick right
  int16_t ry = 0;        // + = stick DOWN (the wire convention; JOY_INVERT_Y makes up = +tilt)
  uint16_t rt = 0;
  uint16_t buttons = 0;
};

struct Rig {
  std::unique_ptr<App> app;
  std::unique_ptr<FakeEsp> esp;     // null: the ESP32 is silent (USB bench only)
  Pad pad;
  uint8_t seq = 0;
  uint64_t lastSendUs = 0;
  bool sentOnce = false;
  uint32_t framesSent = 0;
  std::string pending;              // USB wire bytes not yet split
  std::string cur;                  // the text line being received
  std::vector<std::string> lines;   // every complete text line from the RA4M1
  std::vector<ds_status_packet_t> statuses;
  std::vector<uint64_t> statusTimes;
  std::vector<ds_telemetry_packet_t> telemetry;
  std::function<void()> everyLoop;  // test hook, before each loop()
  uint32_t ledShows = 0;            // hal_host::ledShowCount() at the last step
  std::function<void()> onShow;     // test hook, after a step that showed a LED frame

  // keepStore: hal_host::reboot() instead of reset(), so the App boots on the settings store
  // the previous one left (a power cycle of the board).
  explicit Rig(const App::Config& cfg = App::defaults(), bool powerOn = true, bool wifi = false,
               bool keepStore = false) {
    if (keepStore)
      hal_host::reboot();
    else
      hal_host::reset();
    setPower(powerOn);
    setWifi(wifi);
    hal_host::setTiltBottomStop(true, stopSteps());
    hal_host::setAnalog(PIN_STALL_SENSE, kA2Baseline);
    app.reset(new App(cfg));
    app->begin();
    drain();
  }

  void setPower(bool on) { hal_host::setDigitalInput(PIN_TOGGLE_POWER, toggleLevel(on)); }
  void setWifi(bool wifi) {
    const bool closed = TOGGLE_MODE_WIFI_WHEN_CLOSED ? wifi : !wifi;
    hal_host::setDigitalInput(PIN_TOGGLE_MODE, toggleLevel(closed));
  }
  uint64_t us() const { return hal_host::timeUs(); }
  uint32_t ms() const { return hal::nowMs(); }

  void sendPacket() {
    ds_control_packet_t p;
    ds_control_init(&p);
    p.flags = (uint8_t)((pad.connected ? DS_CTRL_FLAG_CONNECTED : 0) | DS_CTRL_FLAG_SOURCE_WIFI);
    p.rx = pad.rx;
    p.ry = pad.ry;
    p.rt = pad.rt;
    p.buttons = pad.buttons;
    p.seq = seq++;
    uint8_t f[DS_CONTROL_FRAME_LEN];
    const size_t n = ds_control_encode(&p, f);
    hal_host::usbInject(f, n);
    framesSent++;
  }

  void step() {
    hal_host::advanceUs(kStepUs);
    if (esp) esp->pump((double)us());
    if (pad.send && (!sentOnce || us() - lastSendUs >= (uint64_t)kCtrlMs * US_PER_MS)) {
      sendPacket();
      lastSendUs = us();
      sentOnce = true;
    }
    if (everyLoop) everyLoop();
    app->loop();
    if (esp) esp->pump((double)us());
    drain();
    if (hal_host::ledShowCount() != ledShows) {
      ledShows = hal_host::ledShowCount();
      if (onShow) onShow();
    }
  }
  void runMs(uint64_t ms) {
    const uint64_t end = us() + ms * US_PER_MS;
    while (us() < end) step();
  }
  bool runUntil(const std::function<bool()>& pred, uint64_t timeoutMs) {
    const uint64_t end = us() + timeoutMs * US_PER_MS;
    while (us() < end) {
      step();
      if (pred()) return true;
    }
    return pred();
  }

  // Splits the USB wire bytes the way the PC does: StatusPackets and TelemetryPackets (valid
  // frames, exact length and CRC), everything else is text lines.
  void drain() {
    pending += hal_host::usbTakeTx();
    size_t i = 0;
    while (i < pending.size()) {
      const uint8_t b = (uint8_t)pending[i];
      const uint8_t type = ds_packet_type_for_start(b);
      if (type == DS_PKT_STATUS || type == DS_PKT_TELEMETRY) {
        const size_t flen = ds_frame_len_for_type(type);
        if (pending.size() - i < 2) break;   // wait for the len byte
        if ((uint8_t)pending[i + 1] == ds_payload_len_for_type(type)) {
          if (pending.size() - i < flen) break;   // wait for the rest
          const uint8_t* f = (const uint8_t*)pending.data() + i;
          if (type == DS_PKT_STATUS) {
            ds_status_packet_t p;
            if (ds_status_decode(f, flen, &p)) {
              statuses.push_back(p);
              statusTimes.push_back(us());
              i += flen;
              continue;
            }
          } else {
            ds_telemetry_packet_t p;
            if (ds_telemetry_decode(f, flen, &p)) {
              telemetry.push_back(p);
              i += flen;
              continue;
            }
          }
        }
      }
      if (b == '\n') {
        lines.push_back(cur);
        cur.clear();
      } else {
        cur += (char)b;
      }
      i++;
    }
    pending.erase(0, i);
  }

  uint8_t state() const { return app->turret().state(); }
  const Motion& motion() const { return app->motion(); }
  const Trigger& trigger() const { return app->trigger(); }
  std::string lcd(uint8_t row) const { return hal_host::lcdRow(row); }
  size_t countLines(const std::string& needle, size_t from = 0) const {
    size_t n = 0;
    for (size_t k = from; k < lines.size(); k++) n += lines[k].find(needle) != std::string::npos;
    return n;
  }
  // The text after the uptime of the last line containing needle ("" when none).
  std::string lastLine(const std::string& needle) const {
    for (size_t k = lines.size(); k > 0; k--)
      if (lines[k - 1].find(needle) != std::string::npos) return lines[k - 1];
    return "";
  }
  // Serial has no FIFO: the HAL sends one USB byte per hal::service() call that finds the UART
  // idle, which here is one byte per loop. Running this long lets everything queued so far
  // (DEBUG_TX_BUFFER_BYTES at most) reach the wire, so its text lines are in `lines`.
  void flush() { runMs(DEBUG_TX_BUFFER_BYTES * kStepUs / US_PER_MS + 1); }
  // A shell line on USB, and enough time for its reply to leave (one byte per loop). A command
  // that saved SERVO ADJUST's angles blocked one loop for hal_host::storeSaveUs() of virtual time,
  // so its reply leaves after that: the same time again.
  std::string shell(const std::string& line) {
    const size_t from = lines.size();
    const uint32_t saves = hal_host::storeSaves();
    hal_host::usbInject(line + "\n");
    runMs(kCtrlMs * 4);
    if (hal_host::storeSaves() != saves) runMs(kCtrlMs * 4);
    std::string reply;
    for (size_t k = from; k < lines.size(); k++) reply += lines[k] + "\n";
    return reply;
  }
  // Hold a button for a few packets, then release it past the debounce lockout.
  void press(uint16_t button) {
    pad.buttons = (uint16_t)(pad.buttons | button);
    runMs(kCtrlMs * 3);
    pad.buttons = (uint16_t)(pad.buttons & ~button);
    runMs(BUTTON_DEBOUNCE_MS + kCtrlMs * 2);
  }
  // Power on (already on in the constructor) -> BOOT -> CALIBRATING -> PAIRING -> DISARMED.
  bool toDisarmed() {
    return runUntil([&] { return state() == DS_STATE_DISARMED; },
                    (uint64_t)(TILT_CAL_MAX_TRAVEL_DEG / TILT_CAL_SPEED_DEG_PER_S * MS_PER_S) +
                        TILT_CAL_SETTLE_MS + 3 * MS_PER_S);
  }
  bool toArmed() {
    if (!toDisarmed()) return false;
    press(DS_BTN_A);
    return state() == DS_STATE_ARMED;
  }
  // ARMED with the flywheels held on (LB) past the spin-up time.
  bool toArmedSpunUp() {
    if (!toArmed()) return false;
    pad.buttons = (uint16_t)(pad.buttons | DS_BTN_LB);
    runMs(FLYWHEEL_SPINUP_MS + kCtrlMs * 3);
    return app->flywheel().ready(ms());
  }
  // The newest StatusPacket.
  ds_status_packet_t lastStatus() const { return statuses.empty() ? ds_status_packet_t() : statuses.back(); }
};

uint32_t blockingWrites() {
  return hal_host::usbTxProbe().blockingWrites + hal_host::linkTxProbe().blockingWrites;
}

// An everyLoop hook: sets endedIn to the phase the running stroke was in when it ended, i.e. the
// phase it was aborted in (HOLD_REST also when it completed; the ammo tells the two apart).
std::function<void()> watchStrokeEnd(const Rig& rig, Trigger::Phase& endedIn) {
  Trigger::Phase was = Trigger::Phase::IDLE;
  return [&rig, &endedIn, was]() mutable {
    const Trigger::Phase p = rig.trigger().phase();
    if (p == Trigger::Phase::IDLE && was != Trigger::Phase::IDLE) endedIn = was;
    was = p;
  };
}

App::Config stallOn() {
  App::Config c = App::defaults();
  c.stall.enabled = true;   // stands in for STALL_DETECT_ENABLED = true with a pot on A2
  return c;
}

}  // namespace

// =====================================================================================================
// Spec 11 acceptance items 1-17
// =====================================================================================================

// 1. Power toggle open -> everything idle, LCD SYSTEM OFF, LEDs dark; closing it reruns BOOT ->
//    CALIBRATING -> PAIRING. Spec 3.1 OFF: EN high, flywheel PWM 0, servo detached, controller
//    input ignored, link packets still consumed.
TEST(accept_01_power_off_idles_everything_and_power_on_reruns_boot) {
  Rig rig;
  CHECK(rig.toArmedSpunUp());
  rig.pad.rx = kStickMax;                              // turning, flywheels on, armed
  rig.runMs(MS_PER_S / 2);
  CHECK(rig.motion().isMoving());
  CHECK(hal_host::pwmDuty(PIN_FLY_ENA) > 0);
  CHECK(hal_host::servoOn());

  rig.setPower(false);
  const uint64_t t0 = rig.us();
  CHECK(rig.runUntil([&] { return rig.state() == DS_STATE_OFF; }, TOGGLE_DEBOUNCE_MS + kCtrlMs));
  CHECK(rig.us() - t0 <= (uint64_t)(TOGGLE_DEBOUNCE_MS + 1) * US_PER_MS);   // debounced, no later
  CHECK(!hal::steppersEnabled());
  CHECK_EQ(hal_host::pinLevel(PIN_STEPPER_EN), true);   // EN high = drivers off
  CHECK_EQ(hal_host::pwmDuty(PIN_FLY_ENA), (uint8_t)0);
  CHECK(!hal_host::servoOn());                           // parked = detached
  CHECK(!rig.motion().isMoving());

  // Controller input is ignored, packets are still consumed, the shell still answers.
  const uint32_t frames0 = rig.app->link().stats().frames;
  const int32_t pan0 = hal::stepPosition(AXIS_PAN);
  rig.pad.rx = -kStickMax;
  rig.pad.rt = kPulled;
  rig.press(DS_BTN_A);
  rig.press(DS_BTN_LB);
  rig.runMs(2 * MS_PER_S);
  CHECK_EQ(rig.state(), (uint8_t)DS_STATE_OFF);
  CHECK_EQ(hal::stepPosition(AXIS_PAN), pan0);
  CHECK_EQ(hal_host::pwmDuty(PIN_FLY_ENA), (uint8_t)0);
  CHECK_EQ(rig.trigger().shotsTotal(), (uint16_t)0);
  CHECK(rig.app->link().stats().frames - frames0 >= 2 * CONTROL_PACKET_HZ - 2);
  CHECK_EQ(hal_host::usbRxDropped(), 0u);
  CHECK(allLeds(Rgb()));                                 // dark
  CHECK_EQ(rig.lcd(0), padRow(LCD_TXT_OFF_ROW0, ""));
  CHECK_EQ(rig.lcd(1), padRow(LCD_TXT_OFF_ROW1, ""));
  CHECK_EQ(hal_host::lcdBacklightOn(), !LCD_BACKLIGHT_OFF_WHEN_SYSTEM_OFF);
  CHECK_EQ(rig.lastStatus().state, (uint8_t)DS_STATE_OFF);
  CHECK(rig.shell("arm").find("arm: refused") != std::string::npos);
  // Not even the actions that work in every other state (B / X equivalents, fly, cal).
  const uint8_t ammo = rig.trigger().ammo();
  CHECK(rig.shell("mode full").find("mode full: refused") != std::string::npos);
  CHECK_EQ(rig.trigger().fireMode(), (uint8_t)DS_FIRE_MODE_SEMI);
  CHECK(rig.shell("mag").find("mag: refused") != std::string::npos);
  CHECK_EQ(rig.trigger().ammo(), ammo);
  CHECK(rig.shell("cal").find("cal: refused") != std::string::npos);
  CHECK(rig.shell("fly on").find("fly on: refused") != std::string::npos);
  CHECK(rig.shell("stop").find("stop: ok") != std::string::npos);   // stopping is always allowed
  CHECK_EQ(rig.state(), (uint8_t)DS_STATE_OFF);
  CHECK(rig.shell("status").find("state=OFF") != std::string::npos);

  // Close it: BOOT -> CALIBRATING -> PAIRING, then the connected source lands in DISARMED.
  rig.pad = Pad();
  const size_t from = rig.lines.size();
  rig.setPower(true);
  CHECK(rig.toDisarmed());
  rig.flush();
  std::vector<std::string> order;
  for (size_t k = from; k < rig.lines.size(); k++)
    for (const char* s : {"state BOOT", "state CALIBRATING", "state PAIRING", "state DISARMED"})
      if (rig.lines[k].find(s) != std::string::npos) order.push_back(s);
  CHECK(order == std::vector<std::string>({"state BOOT", "state CALIBRATING", "state PAIRING", "state DISARMED"}));
  CHECK(hal_host::servoOn());
  CHECK(rig.motion().calibrated());
  CHECK(hal_host::lcdBacklightOn());
}

// 2. Boot: LEDs pulse white; tilt drives down, stops at the panel, lifts TILT_CAL_BACKOFF_DEG,
//    reports tilt = 0; LCD shows Calibrating, then the pairing row.
TEST(accept_02_boot_pulses_white_and_calibrates_against_the_panel) {
  Rig rig;
  rig.pad.send = false;   // nothing connected: stays in PAIRING afterwards
  uint8_t lo = UINT8_MAX, hi = 0;
  bool allWhite = true;
  rig.onShow = [&] {
    if (rig.state() != DS_STATE_CALIBRATING) return;
    const Rgb c = hal_host::ledShown(0);
    allWhite = allWhite && c.r == c.g && c.g == c.b && allLeds(c);
    lo = std::min(lo, c.r);
    hi = std::max(hi, c.r);
  };
  rig.runMs(LED_PULSE_PERIOD_MS);
  CHECK_EQ(rig.state(), (uint8_t)DS_STATE_CALIBRATING);
  CHECK_EQ(rig.lcd(0), padRow(LCD_TXT_CALIBRATING, ""));
  CHECK_EQ(rig.lcd(1), padRow(LCD_TXT_MODE_BT, ""));
  CHECK(hal::steppersEnabled());
  CHECK(rig.motion().velocityDeg(AXIS_TILT) < 0.0f);     // driving down
  CHECK(std::fabs(rig.motion().velocityDeg(AXIS_TILT)) <= TILT_CAL_SPEED_DEG_PER_S + 0.01f);
  CHECK(rig.runUntil([&] { return rig.state() == DS_STATE_PAIRING; },
                     (uint64_t)(TILT_CAL_MAX_TRAVEL_DEG / TILT_CAL_SPEED_DEG_PER_S * MS_PER_S) + 2 * MS_PER_S));
  CHECK(allWhite);
  CHECK_EQ(hi, scaled(LED_COLOR_STARTUP).r);              // the pulse reaches full (half) brightness
  CHECK(lo <= (uint8_t)(scaled(LED_COLOR_STARTUP).r * LED_PULSE_MIN_FRACTION) + 1);
  CHECK(hal_host::lostSteps(AXIS_TILT) > 0u);             // it rammed the panel
  CHECK(rig.motion().calibrated());
  CHECK_EQ(hal::stepPosition(AXIS_TILT), 0);              // tilt = 0 ...
  CHECK_EQ(hal_host::trueSteps(AXIS_TILT),                // ... TILT_CAL_BACKOFF_DEG above the panel
           stopSteps() + Motion::degToSteps(AXIS_TILT, TILT_CAL_BACKOFF_DEG));
  CHECK(rig.motion().limitsActive(AXIS_TILT));
  rig.runMs(LCD_REFRESH_MS * 4);
  CHECK(startsWith(rig.lcd(1), std::string(LCD_TXT_MODE_BT) + " " + LCD_TXT_PAIRING));
  CHECK_EQ(rig.lcd(0), padRow(LCD_TXT_DISARMED, ammoText(MAG_CAPACITY)));
  CHECK(!rig.telemetry.empty() && (rig.telemetry.back().flags & DS_TELEM_FLAG_CALIBRATED));
  CHECK_EQ(rig.telemetry.back().tiltCdeg, 0);
}

// 3. Pairing: LEDs pulse blue/white; LCD row 1 "BT Pairing" with dots cycling at
//    LCD_PAIRING_DOT_MS; the first valid connected packet -> DISARMED, green, "Connected".
TEST(accept_03_pairing_pulses_blue_white_with_dots_then_connects) {
  Rig rig;
  rig.pad.send = false;
  CHECK(rig.runUntil([&] { return rig.state() == DS_STATE_PAIRING; }, 15 * MS_PER_S));
  const Rgb a = scaled(LED_COLOR_PAIRING_A), b = scaled(LED_COLOR_PAIRING_B);
  bool between = true;
  bool sawA = false, sawB = false;
  rig.onShow = [&] {
    const Rgb c = hal_host::ledShown(0);
    for (uint8_t ch = 0; ch < 3; ch++) {
      const uint8_t v = ch == 0 ? c.r : ch == 1 ? c.g : c.b;
      const uint8_t x = ch == 0 ? a.r : ch == 1 ? a.g : a.b, y = ch == 0 ? b.r : ch == 1 ? b.g : b.b;
      between = between && v >= std::min(x, y) && v <= std::max(x, y);
    }
    sawA = sawA || same(c, a);
    sawB = sawB || (std::abs(c.r - b.r) <= 1 && std::abs(c.g - b.g) <= 1 && std::abs(c.b - b.b) <= 1);
  };
  std::vector<std::pair<uint32_t, std::string>> rows;
  std::string prev;
  const uint32_t since = rig.app->turret().stateSinceMs();
  rig.everyLoop = [&] {   // the rows Lcd composes (the display follows a few updates later)
    const std::string r = rig.app->lcd().row(1);
    if (r != prev) rows.push_back({rig.ms() - since, r}), prev = r;
  };
  rig.runMs(4 * LCD_PAIRING_DOT_MS * 2);
  rig.everyLoop = nullptr;
  rig.onShow = nullptr;
  CHECK(between && sawA && sawB);
  CHECK_EQ(rig.lcd(1), std::string(rig.app->lcd().row(1)));   // and the display holds it
  // The dots: "", ".", ". .", ". . ." advancing every LCD_PAIRING_DOT_MS.
  const std::vector<std::string> dots = {"", ".", ". .", ". . ."};
  std::vector<std::string> seen;
  for (const auto& r : rows) {
    const std::string t = r.second;
    const std::string head = std::string(LCD_TXT_MODE_BT) + " " + LCD_TXT_PAIRING;
    if (!startsWith(t, head)) continue;
    std::string tail = t.substr(head.size());
    while (!tail.empty() && tail.back() == ' ') tail.pop_back();
    if (!tail.empty() && tail[0] == ' ') tail.erase(0, 1);
    seen.push_back(tail);
    const size_t phase = (r.first / LCD_PAIRING_DOT_MS) % dots.size();
    CHECK_EQ(tail, dots[phase]);
  }
  CHECK(seen.size() >= 2 * dots.size() - 1);
  if (std::strcmp(LCD_TXT_MODE_BT, "BT") == 0 && std::strcmp(LCD_TXT_PAIRING, "Pairing") == 0)
    CHECK(startsWith(rig.lcd(1), "BT Pairing"));
  // First packet: DISARMED within one packet, green, Connected.
  rig.pad.send = true;
  const uint64_t t0 = rig.us();
  CHECK(rig.runUntil([&] { return rig.state() == DS_STATE_DISARMED; }, kCtrlMs * 3));
  CHECK(rig.us() - t0 <= (uint64_t)kCtrlMs * US_PER_MS);
  rig.runMs(LCD_REFRESH_MS * 4);
  CHECK(allLeds(scaled(LED_COLOR_DISARMED)));
  CHECK_EQ(rig.lcd(1), padRow(LCD_TXT_MODE_BT, LCD_TXT_CONNECTED));
}

// 4. Right stick: speed proportional to deflection (expo curve), stops in the dead zone, never
//    above the max speeds, never across a soft limit, slowing near limits.
TEST(accept_04_stick_speed_follows_the_curve_and_respects_limits) {
  Rig rig;
  CHECK(rig.toDisarmed());
  float maxPan = 0.0f, maxTilt = 0.0f;
  rig.everyLoop = [&] {
    maxPan = std::max(maxPan, std::fabs(rig.motion().velocityDeg(AXIS_PAN)));
    maxTilt = std::max(maxTilt, std::fabs(rig.motion().velocityDeg(AXIS_TILT)));
  };
  // Full right: clockwise = negative pan (architecture.md 4.1), at the shaped full speed.
  rig.pad.rx = kStickMax;
  rig.runMs(MS_PER_S);
  const float full = Inputs::shapeAxis(kStickMax) * PAN_MAX_DEG_PER_S;
  CHECK_NEAR(rig.motion().velocityDeg(AXIS_PAN), -full, 0.5f);
  CHECK(rig.motion().positionDeg(AXIS_PAN) < -10.0f);
  // Half deflection: the expo curve, not half speed.
  const int16_t half = JOY_AXIS_RANGE / 2;
  rig.pad.rx = half;
  rig.runMs(MS_PER_S);
  const float halfSpeed = Inputs::shapeAxis(half) * PAN_MAX_DEG_PER_S;
  CHECK_NEAR(rig.motion().velocityDeg(AXIS_PAN), -halfSpeed, 0.5f);
  if (JOY_EXPO > 0.0f) CHECK(halfSpeed < 0.5f * PAN_MAX_DEG_PER_S);
  // Left: counter-clockwise.
  rig.pad.rx = -kStickMax;
  rig.runMs(MS_PER_S);
  CHECK(rig.motion().velocityDeg(AXIS_PAN) > 0.0f);
  // Inside the dead zone: stops.
  rig.pad.rx = (int16_t)(JOY_DEADZONE * JOY_AXIS_RANGE * 0.9f);
  rig.runMs(MS_PER_S);
  CHECK_EQ(rig.motion().velocityDeg(AXIS_PAN), 0.0f);
  CHECK(!rig.motion().isMoving());
  // Tilt up (stick up = negative on the wire) into the top soft limit and hold it there.
  rig.pad.ry = -kStickMax;
  bool slowedNearLimit = false;
  rig.everyLoop = [&] {
    maxTilt = std::max(maxTilt, std::fabs(rig.motion().velocityDeg(AXIS_TILT)));
    const float p = rig.motion().positionDeg(AXIS_TILT);
    if (p > TILT_RANGE_DEG - SOFT_LIMIT_SLOWDOWN_DEG / 4 && rig.motion().velocityDeg(AXIS_TILT) > 0.0f &&
        rig.motion().velocityDeg(AXIS_TILT) < 0.5f * TILT_MAX_DEG_PER_S)
      slowedNearLimit = true;
    CHECK(p <= TILT_RANGE_DEG + 1.0f / TILT_STEPS_PER_DEG);
  };
  rig.runMs((uint64_t)(TILT_RANGE_DEG / TILT_MAX_DEG_PER_S * MS_PER_S) + 2 * MS_PER_S);
  CHECK(slowedNearLimit);
  CHECK_NEAR(rig.motion().positionDeg(AXIS_TILT), TILT_RANGE_DEG, 0.5f);
  // ... and down to the bottom limit (0 = calibrated bottom + backoff).
  rig.pad.ry = kStickMax;
  rig.everyLoop = [&] { CHECK(rig.motion().positionDeg(AXIS_TILT) >= -1.0f / TILT_STEPS_PER_DEG); };
  rig.runMs((uint64_t)(TILT_RANGE_DEG / TILT_MAX_DEG_PER_S * MS_PER_S) + 2 * MS_PER_S);
  CHECK_NEAR(rig.motion().positionDeg(AXIS_TILT), 0.0f, 0.5f);
  rig.everyLoop = nullptr;
  CHECK(maxPan <= PAN_MAX_DEG_PER_S + 0.01f);
  CHECK(maxTilt <= TILT_MAX_DEG_PER_S + 0.01f);
  CHECK_EQ(hal_host::totalRefusals(AXIS_TILT), 0u);    // the planner braked; the bound never had to
  CHECK_EQ(hal_host::lostSteps(AXIS_TILT) > 0u, true); // (the calibration's, against the panel)
}

// 5. D-pad left, move away, D-pad down -> back at the stored position at HOME_SPEED_DEG_PER_S;
//    a stick nudge mid-move cancels it.
TEST(accept_05_home_set_go_home_and_stick_cancels) {
  Rig rig;
  CHECK(rig.toDisarmed());
  rig.pad.rx = -kStickMax;
  rig.pad.ry = -kStickMax;
  rig.runMs(MS_PER_S / 2);
  rig.pad.rx = rig.pad.ry = 0;
  rig.runMs(MS_PER_S / 2);
  const float homePan = rig.motion().positionDeg(AXIS_PAN), homeTilt = rig.motion().positionDeg(AXIS_TILT);
  CHECK(homePan > 10.0f && homeTilt > 5.0f);
  rig.press(DS_BTN_DPAD_LEFT);
  CHECK_NEAR(rig.motion().homeDeg(AXIS_PAN), homePan, 0.01f);
  CHECK_NEAR(rig.motion().homeDeg(AXIS_TILT), homeTilt, 0.01f);
  rig.pad.rx = kStickMax;
  rig.runMs(MS_PER_S);
  rig.pad.rx = 0;
  rig.runMs(MS_PER_S / 2);
  CHECK(rig.motion().positionDeg(AXIS_PAN) < homePan - 30.0f);
  float top = 0.0f;
  rig.everyLoop = [&] { top = std::max(top, std::fabs(rig.motion().velocityDeg(AXIS_PAN))); };
  rig.press(DS_BTN_DPAD_DOWN);
  CHECK(rig.motion().homing());
  CHECK(rig.runUntil([&] { return !rig.motion().homing(); }, 5 * MS_PER_S));
  rig.everyLoop = nullptr;
  CHECK_NEAR(rig.motion().positionDeg(AXIS_PAN), homePan, 1.5f / PAN_STEPS_PER_DEG);
  CHECK_NEAR(rig.motion().positionDeg(AXIS_TILT), homeTilt, 1.5f / TILT_STEPS_PER_DEG);
  CHECK(top <= HOME_SPEED_DEG_PER_S + 0.01f);
  CHECK(top >= 0.95f * HOME_SPEED_DEG_PER_S);
  // Again, and nudge the stick half-way.
  rig.pad.rx = kStickMax;
  rig.runMs(MS_PER_S);
  rig.pad.rx = 0;
  rig.runMs(MS_PER_S / 2);
  rig.press(DS_BTN_DPAD_DOWN);
  CHECK(rig.motion().homing());
  rig.runMs(MS_PER_S / 5);
  rig.pad.rx = (int16_t)(JOY_AXIS_RANGE / 2);
  rig.runMs(kCtrlMs * 3);
  CHECK(!rig.motion().homing());
  rig.pad.rx = 0;
  rig.runMs(MS_PER_S);
  CHECK(std::fabs(rig.motion().positionDeg(AXIS_PAN) - homePan) > 5.0f);   // stopped short
}

// Found on the bench (2026-11-03): the power toggle is a new boot (spec 3.1). While OFF the
// drivers are released and the turret can be turned by hand, so the old pan count means
// nothing: pan zero = wherever it is when the toggle closes, and home goes back to (0, 0).
// Without this a go-home after a power cycle swung the turret by the stale count.
TEST(power_toggle_reboot_rezeroes_pan_and_resets_home) {
  Rig rig;
  CHECK(rig.toDisarmed());
  rig.pad.rx = -kStickMax;
  rig.runMs(MS_PER_S);
  rig.pad.rx = 0;
  rig.runMs(MS_PER_S / 2);
  CHECK(std::fabs(rig.motion().positionDeg(AXIS_PAN)) > 30.0f);
  rig.press(DS_BTN_DPAD_LEFT);
  CHECK(std::fabs(rig.motion().homeDeg(AXIS_PAN)) > 30.0f);

  rig.setPower(false);
  CHECK(rig.runUntil([&] { return rig.state() == DS_STATE_OFF; }, TOGGLE_DEBOUNCE_MS + kCtrlMs));
  rig.setPower(true);
  CHECK(rig.runUntil([&] { return rig.state() == DS_STATE_CALIBRATING; }, TOGGLE_DEBOUNCE_MS + kCtrlMs));
  CHECK_EQ(hal::stepPosition(AXIS_PAN), (int32_t)0);
  CHECK_NEAR(rig.motion().homeDeg(AXIS_PAN), 0.0f, 0.001f);
  CHECK_NEAR(rig.motion().homeDeg(AXIS_TILT), 0.0f, 0.001f);
  CHECK(rig.toDisarmed());
  CHECK_NEAR(rig.motion().positionDeg(AXIS_PAN), 0.0f, 0.001f);
  CHECK(rig.motion().calibrated());
}

// 6. A toggles ARMED/DISARMED; LEDs yellow / orange / green; LCD row 0 follows; arming is
//    refused while the trigger is pulled (ARM_REQUIRES_TRIGGER_RELEASED).
TEST(accept_06_a_arms_and_disarms_and_arm_refused_while_the_trigger_is_held) {
  Rig rig;
  CHECK(rig.toDisarmed());
  rig.press(DS_BTN_A);
  CHECK_EQ(rig.state(), (uint8_t)DS_STATE_ARMED);
  rig.runMs(LCD_REFRESH_MS * 4);
  // Armed with the flywheels still: yellow (a pull would not fire) unless the trigger may fire
  // without them.
  CHECK(allLeds(scaled(FIRE_REQUIRES_FLYWHEEL ? LED_COLOR_ARMED_WAIT : LED_COLOR_ARMED_READY)));
  CHECK(startsWith(rig.lcd(0), LCD_TXT_ARMED));
  CHECK(rig.lcd(0).find(ammoText(MAG_CAPACITY)) != std::string::npos);
  rig.press(DS_BTN_A);
  CHECK_EQ(rig.state(), (uint8_t)DS_STATE_DISARMED);
  rig.runMs(LCD_REFRESH_MS * 4);
  CHECK(allLeds(scaled(LED_COLOR_DISARMED)));
  CHECK_EQ(rig.lcd(0), padRow(LCD_TXT_DISARMED, ammoText(MAG_CAPACITY)));
  // With RT pulled, A is refused (and says why).
  rig.pad.rt = kPulled;
  rig.runMs(kCtrlMs * 3);
  CHECK(rig.app->inputs().triggerPulled());
  const size_t from = rig.lines.size();
  rig.press(DS_BTN_A);
  CHECK_EQ(rig.state(), (uint8_t)(ARM_REQUIRES_TRIGGER_RELEASED ? DS_STATE_DISARMED : DS_STATE_ARMED));
  if (ARM_REQUIRES_TRIGGER_RELEASED) {
    CHECK(rig.countLines("refused: trigger held", from) == 1u);
    CHECK_EQ(rig.trigger().shotsTotal(), (uint16_t)0);
  }
  // Released: A arms again.
  rig.pad.rt = 0;
  rig.runMs(kCtrlMs * 3);
  if (rig.state() == DS_STATE_ARMED) rig.press(DS_BTN_A);
  rig.press(DS_BTN_A);
  CHECK_EQ(rig.state(), (uint8_t)DS_STATE_ARMED);
}

// The same rule with the option off: arming with the trigger held is allowed, but that pull
// never fires (architecture.md 6: a pull counts only if it started while ARMED).
TEST(arm_with_the_trigger_held_never_fires_that_pull) {
  App::Config cfg = App::defaults();
  cfg.turret.armRequiresTriggerReleased = false;
  Rig rig(cfg);
  CHECK(rig.toDisarmed());
  rig.pad.buttons = DS_BTN_LB;   // ignored while DISARMED (FLYWHEEL_REQUIRES_ARMED)
  rig.pad.rt = kPulled;
  rig.runMs(kCtrlMs * 3);
  rig.press(DS_BTN_A);
  CHECK_EQ(rig.state(), (uint8_t)DS_STATE_ARMED);
  rig.runMs(FLYWHEEL_SPINUP_MS + MS_PER_S);
  CHECK(rig.app->flywheel().ready(rig.ms()));
  CHECK_EQ(rig.trigger().shotsTotal(), (uint16_t)0);
  rig.pad.rt = kReleased;
  rig.runMs(kCtrlMs * 3);
  rig.pad.rt = kPulled;
  rig.runMs(kStrokeMs + kCtrlMs * 3);
  CHECK_EQ(rig.trigger().shotsTotal(), (uint16_t)1);
}

// 7. B toggles semi/full (the LCD shows ARMED S / F; since the owner's 2026-11-03 scheme the LEDs
//    show readiness, not the mode, so they do not change); B works in any state.
TEST(accept_07_b_toggles_semi_full_in_any_state) {
  Rig rig;
  CHECK(rig.toDisarmed());
  rig.press(DS_BTN_B);
  CHECK_EQ(rig.trigger().fireMode(), (uint8_t)DS_FIRE_MODE_FULL);   // DISARMED too
  rig.press(DS_BTN_A);
  rig.runMs(LCD_REFRESH_MS * 4);
  const Rgb armedColour = scaled(FIRE_REQUIRES_FLYWHEEL ? LED_COLOR_ARMED_WAIT : LED_COLOR_ARMED_READY);
  CHECK(allLeds(armedColour));
  if (LCD_TXT_SHOW_FIRE_MODE) CHECK(startsWith(rig.lcd(0), std::string(LCD_TXT_ARMED) + LCD_TXT_FIRE_FULL));
  rig.press(DS_BTN_B);
  rig.runMs(LCD_REFRESH_MS * 4);
  CHECK_EQ(rig.trigger().fireMode(), (uint8_t)DS_FIRE_MODE_SEMI);
  CHECK(allLeds(armedColour));                                      // same colour in both modes
  if (LCD_TXT_SHOW_FIRE_MODE) CHECK(startsWith(rig.lcd(0), std::string(LCD_TXT_ARMED) + LCD_TXT_FIRE_SEMI));
  // During a calibration (shell `cal` from DISARMED) B still works.
  rig.press(DS_BTN_A);
  CHECK(rig.shell("cal").find("cal: ok") != std::string::npos);
  CHECK_EQ(rig.state(), (uint8_t)DS_STATE_CALIBRATING);
  rig.press(DS_BTN_B);
  CHECK_EQ(rig.state(), (uint8_t)DS_STATE_CALIBRATING);
  CHECK_EQ(rig.trigger().fireMode(), (uint8_t)DS_FIRE_MODE_FULL);
}

// 8. LB spins the flywheels only when permitted; the duty ramps over FLYWHEEL_RAMP_MS to
//    FLYWHEEL_DUTY_255 (ENA = FLYWHEEL_DUTY_255 / 255).
TEST(accept_08_flywheels_only_when_permitted_and_ramped) {
  Rig rig;
  CHECK(rig.toDisarmed());
  rig.pad.buttons = DS_BTN_LB;
  rig.runMs(FLYWHEEL_RAMP_MS * 2);
  CHECK_EQ(hal_host::pwmDuty(PIN_FLY_ENA), (uint8_t)(FLYWHEEL_REQUIRES_ARMED ? 0 : FLYWHEEL_DUTY_255));
  rig.pad.buttons = 0;
  rig.runMs(kCtrlMs * 3);
  rig.press(DS_BTN_A);
  CHECK_EQ(rig.state(), (uint8_t)DS_STATE_ARMED);
  CHECK_EQ(hal_host::pwmDuty(PIN_FLY_ENA), (uint8_t)0);
  rig.pad.buttons = DS_BTN_LB;
  CHECK(rig.runUntil([&] { return rig.app->flywheel().on(); }, kCtrlMs * 2));
  const uint32_t on = rig.app->flywheel().onSinceMs();
  rig.runUntil([&] { return rig.ms() - on >= FLYWHEEL_RAMP_MS / 2; }, FLYWHEEL_RAMP_MS);
  const int mid = hal_host::pwmDuty(PIN_FLY_ENA);
  CHECK(std::abs(mid - FLYWHEEL_DUTY_255 / 2) <= 2);   // soft start, linear
  rig.runUntil([&] { return rig.ms() - on >= FLYWHEEL_RAMP_MS; }, FLYWHEEL_RAMP_MS);
  rig.runMs(kCtrlMs);
  CHECK_EQ(hal_host::pwmDuty(PIN_FLY_ENA), (uint8_t)FLYWHEEL_DUTY_255);
  CHECK_EQ(hal_host::pinLevel(PIN_FLY_IN1), FLYWHEEL_FORWARD_IN1_HIGH);
  CHECK_EQ(hal_host::pinLevel(PIN_FLY_IN2), !FLYWHEEL_FORWARD_IN1_HIGH);
  // Hold mode: release = off; disarm with LB held = off at once.
  if (FLYWHEEL_HOLD_MODE) {
    rig.pad.buttons = 0;
    rig.runMs(kCtrlMs * 2);
    CHECK_EQ(hal_host::pwmDuty(PIN_FLY_ENA), (uint8_t)0);
    rig.pad.buttons = DS_BTN_LB;
    rig.runMs(FLYWHEEL_RAMP_MS);
  }
  rig.pad.buttons = (uint16_t)(rig.pad.buttons | DS_BTN_A);
  rig.runMs(kCtrlMs * 2);
  CHECK_EQ(rig.state(), (uint8_t)DS_STATE_DISARMED);
  if (FLYWHEEL_REQUIRES_ARMED) CHECK_EQ(hal_host::pwmDuty(PIN_FLY_ENA), (uint8_t)0);
}

TEST(flywheel_toggle_mode_and_not_requiring_armed) {
  App::Config cfg = App::defaults();
  cfg.turret.flywheelHoldMode = false;
  cfg.turret.flywheelRequiresArmed = false;
  Rig rig(cfg);
  CHECK(rig.toDisarmed());
  rig.press(DS_BTN_LB);                               // toggle: on, in DISARMED
  rig.runMs(FLYWHEEL_RAMP_MS);
  CHECK(rig.app->flywheel().on());
  CHECK_EQ(hal_host::pwmDuty(PIN_FLY_ENA), (uint8_t)FLYWHEEL_DUTY_255);
  rig.press(DS_BTN_LB);                               // toggle: off
  CHECK(!rig.app->flywheel().on());
  rig.press(DS_BTN_LB);
  CHECK(rig.app->flywheel().on());
  // The link drops (FAILSAFE_STOP_FLYWHEEL): cut, and the toggle is forgotten.
  rig.pad.send = false;
  rig.runMs(LINK_TIMEOUT_MS + kCtrlMs);
  CHECK_EQ(rig.app->flywheel().on(), !FAILSAFE_STOP_FLYWHEEL);
  rig.pad.send = true;
  rig.runMs(MS_PER_S / 2);
  CHECK_EQ(rig.state(), (uint8_t)DS_STATE_DISARMED);
  if (FAILSAFE_STOP_FLYWHEEL) CHECK(!rig.app->flywheel().on());
}

// 9. Semi auto: one stroke per pull above TRIGGER_PRESS_THRESHOLD; holding does not re-fire;
//    release below TRIGGER_RELEASE_THRESHOLD then pull again fires again; ammo - 1 per stroke.
TEST(accept_09_semi_auto_one_stroke_per_pull_with_hysteresis) {
  Rig rig;
  // Not armed: a pull does nothing.
  CHECK(rig.toDisarmed());
  rig.pad.rt = kPulled;
  rig.runMs(kStrokeMs * 2);
  CHECK_EQ(rig.trigger().shotsTotal(), (uint16_t)0);
  CHECK_EQ(servoLogical(), (float)TRIGGER_REST_ANGLE);
  rig.pad.rt = 0;
  rig.runMs(kCtrlMs * 3);
  CHECK(rig.toArmedSpunUp());
  bool sawFire = false;
  rig.everyLoop = [&] { sawFire = sawFire || servoLogical() == (float)TRIGGER_FIRE_ANGLE; };
  rig.pad.rt = kPulled;
  rig.runMs(kStrokeMs * 4);   // held well past one stroke
  CHECK(sawFire);
  CHECK_EQ(rig.trigger().shotsTotal(), (uint16_t)1);
  CHECK_EQ(rig.trigger().ammo(), (uint8_t)(MAG_CAPACITY - 1));
  CHECK_EQ(servoLogical(), (float)TRIGGER_REST_ANGLE);
  // Between the thresholds is still "pulled": back up past the press threshold, no new pull.
  rig.pad.rt = kBetween;
  rig.runMs(kCtrlMs * 5);
  rig.pad.rt = kPulled;
  rig.runMs(kStrokeMs * 2);
  CHECK_EQ(rig.trigger().shotsTotal(), (uint16_t)1);
  // Below the release threshold, then pulled: the next stroke.
  rig.pad.rt = kReleased;
  rig.runMs(kCtrlMs * 3);
  rig.pad.rt = kPulled;
  rig.runMs(kStrokeMs * 2);
  CHECK_EQ(rig.trigger().shotsTotal(), (uint16_t)2);
  CHECK_EQ(rig.trigger().ammo(), (uint8_t)(MAG_CAPACITY - 2));
  rig.everyLoop = nullptr;
}

// Semi cooldown (owner, 2026-11-03): after a stroke completes, no new one starts for
// TRIGGER_COOLDOWN_MS; a pull still held when it ends fires on the spot, a tap inside it is
// dropped. The magazine loses one dart at each completion and never while a stroke runs.
TEST(semi_cooldown_after_each_stroke_and_ammo_only_on_completion) {
  Rig rig;
  CHECK(rig.toArmedSpunUp());
  std::vector<int64_t> starts, ends;
  std::vector<uint8_t> ammoAtEnd;
  bool active = false, ammoMidStroke = false;
  uint8_t ammoAtStart = 0;
  rig.everyLoop = [&] {   // sees the state the previous loop left
    const bool a = rig.trigger().strokeActive();
    if (a && !active) {
      starts.push_back(rig.ms());
      ammoAtStart = rig.trigger().ammo();
    }
    if (!a && active) {
      ends.push_back(rig.ms());
      ammoAtEnd.push_back(rig.trigger().ammo());
    }
    ammoMidStroke = ammoMidStroke || (a && rig.trigger().ammo() != ammoAtStart);
    active = a;
  };
  // Pull, let go mid-stroke, pull again and hold: the second stroke waits out the cooldown.
  rig.pad.rt = kPulled;
  rig.runMs(kStrokeMs / 2);
  rig.pad.rt = kReleased;
  rig.runMs(kCtrlMs * 3);
  rig.pad.rt = kPulled;
  CHECK(rig.runUntil([&] { return ends.size() >= 2; }, (kStrokeMs + kCoolMs) * 2));
  // A tap inside the cooldown after the second stroke: dropped.
  rig.pad.rt = kReleased;
  rig.runMs(kCtrlMs * 3);
  rig.pad.rt = kPulled;
  rig.runMs(kCtrlMs * 3);
  rig.pad.rt = kReleased;
  CHECK(rig.ms() - ends.back() + kCtrlMs < kCoolMs);   // the tap really was inside it
  rig.runMs(kCoolMs + kStrokeMs);
  rig.everyLoop = nullptr;
  CHECK_EQ(starts.size(), (size_t)2);
  CHECK_EQ(ends.size(), (size_t)2);
  if (starts.size() == 2 && ends.size() == 2) {
    // The first pull lands anywhere inside a ms; the cooldown and the second stroke are exact.
    CHECK(ends[0] - starts[0] >= (int64_t)kStrokeMs - 1 && ends[0] - starts[0] <= (int64_t)kStrokeMs);
    CHECK_EQ(starts[1] - ends[0], (int64_t)kCoolMs);
    CHECK_EQ(ends[1] - starts[1], (int64_t)kStrokeMs);
    CHECK_EQ(ammoAtEnd[0], (uint8_t)(MAG_CAPACITY - 1));
    CHECK_EQ(ammoAtEnd[1], (uint8_t)(MAG_CAPACITY - 2));
  }
  CHECK(!ammoMidStroke);
  CHECK_EQ(rig.trigger().shotsTotal(), (uint16_t)2);
  CHECK_EQ(rig.trigger().ammo(), (uint8_t)(MAG_CAPACITY - 2));
}

// The trigger fires only while ARMED, also with the two gates that could otherwise hide a wrong
// "armed" switched off: no flywheel requirement, and flywheels allowed while DISARMED.
TEST(disarmed_never_fires_even_with_the_flywheel_gates_off) {
  App::Config cfg = App::defaults();
  cfg.trigger.requiresFlywheel = false;
  cfg.turret.flywheelRequiresArmed = false;
  Rig rig(cfg);
  CHECK(rig.toDisarmed());
  rig.pad.buttons = DS_BTN_LB;
  rig.runMs(FLYWHEEL_SPINUP_MS + kCtrlMs * 3);
  CHECK(rig.app->flywheel().ready(rig.ms()));          // spinning while DISARMED (this config)
  rig.pad.rt = kPulled;
  rig.runMs(kStrokeMs * 3);
  CHECK_EQ(rig.trigger().shotsTotal(), (uint16_t)0);
  CHECK_EQ(servoLogical(), (float)TRIGGER_REST_ANGLE);
  CHECK(rig.shell("fire").find("fire: refused: not in this state") != std::string::npos);
  CHECK_EQ(rig.trigger().shotsTotal(), (uint16_t)0);
  rig.pad.rt = kReleased;
  rig.runMs(kCtrlMs * 3);
  rig.press(DS_BTN_A);
  CHECK_EQ(rig.state(), (uint8_t)DS_STATE_ARMED);
  rig.pad.rt = kPulled;
  rig.runMs(kStrokeMs + kCtrlMs * 3);
  CHECK_EQ(rig.trigger().shotsTotal(), (uint16_t)1);
}

// 10. Full auto: strokes back-to-back while held, no gap; release finishes the current stroke.
TEST(accept_10_full_auto_back_to_back_and_release_finishes_the_stroke) {
  Rig rig;
  CHECK(rig.toArmedSpunUp());
  rig.press(DS_BTN_B);
  CHECK_EQ(rig.trigger().fireMode(), (uint8_t)DS_FIRE_MODE_FULL);
  std::vector<uint64_t> starts;
  bool forward = false;
  rig.everyLoop = [&] {
    const bool f = rig.trigger().phase() == kFirstPhase;
    if (f && !forward) starts.push_back(rig.us());
    forward = f;
  };
  rig.pad.rt = kPulled;
  const uint32_t shots = 4;
  rig.runUntil([&] { return starts.size() >= shots + 1; }, (shots + 2) * kStrokeMs);
  CHECK(starts.size() >= shots + 1);
  // No gap: consecutive strokes start one stroke period apart (whole-ms phase timing, so
  // within one millisecond either side).
  for (size_t k = 1; k < starts.size(); k++) {
    CHECK(starts[k] - starts[k - 1] + US_PER_MS >= (uint64_t)kStrokeMs * US_PER_MS);
    CHECK(starts[k] - starts[k - 1] <= (uint64_t)kStrokeMs * US_PER_MS + US_PER_MS);
  }
  // Release mid-stroke: that stroke completes (ammo - 1), no new one starts.
  rig.runMs(TRIGGER_FORWARD_MS / 2);
  CHECK(rig.trigger().strokeActive());
  const uint16_t before = rig.trigger().shotsTotal();
  rig.pad.rt = 0;
  rig.runMs(kStrokeMs * 3);
  CHECK_EQ(rig.trigger().shotsTotal(), (uint16_t)(before + 1));
  CHECK(!rig.trigger().strokeActive());
  CHECK_EQ(rig.trigger().ammo(), (uint8_t)(MAG_CAPACITY - rig.trigger().shotsTotal()));
  rig.everyLoop = nullptr;
}

// 11. Each stroke: all LEDs show it for at least LED_FIRE_FLASH_MIN_MS (the owner's scheme of
//     2026-11-03: ARMED yellow "next shot not ready" from the stroke's start for the whole stroke,
//     its sweeps and holds, and the semi cooldown after it; red again once a pull would fire); a
//     rumble command of the configured strength / duration goes out (USB StatusPacket, new
//     rumbleSeq per stroke).
TEST(accept_11_each_stroke_shows_yellow_then_red_and_sends_one_rumble) {
  Rig rig;
  CHECK(rig.toArmedSpunUp());
  const Rgb red = scaled(LED_COLOR_ARMED_READY);
  const Rgb yellow = scaled(LED_COLOR_ARMED_WAIT);
  const uint64_t frameUs = (uint64_t)MS_PER_S / LED_REFRESH_HZ * US_PER_MS;
  rig.runMs(kCtrlMs * 3);
  CHECK(allLeds(red));                                 // spun up, idle: a pull would fire
  uint64_t start = 0, end = 0, firstYellow = 0, lastYellow = 0;
  bool yellowWhileStroke = true, yellowWhileCooling = true, redAfter = true;
  uint32_t showsAfter = 0, showsCooling = 0;
  // After the stroke: the semi cooldown (and whatever is left of the LED hold) keeps it yellow.
  const uint64_t yellowAfterUs = (uint64_t)std::max<uint32_t>(kCoolMs, LED_FIRE_FLASH_MIN_MS) * US_PER_MS;
  rig.onShow = [&] {
    const bool isYellow = allLeds(yellow);
    if (isYellow && !firstYellow) firstYellow = rig.us();
    if (isYellow) lastYellow = rig.us();
    if (rig.trigger().strokeActive() && start && rig.us() - start > frameUs)
      yellowWhileStroke = yellowWhileStroke && isYellow;
    if (end && rig.us() - end + frameUs < (uint64_t)kCoolMs * US_PER_MS) {
      yellowWhileCooling = yellowWhileCooling && isYellow;
      showsCooling++;
    }
    if (end && rig.us() - end > frameUs + yellowAfterUs) {
      redAfter = redAfter && allLeds(red);   // semi, still held: a fresh pull would fire
      showsAfter++;
    }
  };
  rig.everyLoop = [&] {
    if (!start && rig.trigger().strokeActive()) start = rig.us();
    if (start && !end && !rig.trigger().strokeActive()) end = rig.us();
  };
  const size_t st0 = rig.statuses.size();
  const uint8_t seq0 = rig.lastStatus().rumbleSeq;
  rig.pad.rt = kPulled;
  rig.runMs(kStrokeMs + LED_FIRE_FLASH_MIN_MS + kCtrlMs * 5);
  rig.runMs(std::max<uint32_t>(kCoolMs, LED_FIRE_FLASH_MIN_MS) + 4 * MS_PER_S / LED_REFRESH_HZ);   // still held, idle
  CHECK(start > 0u);
  CHECK(end > start);
  CHECK(firstYellow > 0u);
  CHECK(firstYellow - start <= (uint64_t)(MS_PER_S / LED_REFRESH_HZ + LED_SHOW_MAX_DEFER_MS) * US_PER_MS);
  CHECK(lastYellow - firstYellow >= (uint64_t)LED_FIRE_FLASH_MIN_MS * US_PER_MS - frameUs);
  CHECK(yellowWhileStroke);
  CHECK(yellowWhileCooling && (kCoolMs <= 2 * frameUs / US_PER_MS || showsCooling > 0u));
  CHECK(showsAfter > 0u && redAfter);
  CHECK(allLeds(red));
  // The rumble: exactly one new rumbleSeq, with the configured strength and duration.
  std::vector<ds_status_packet_t> rumbles;
  uint8_t prev = seq0;
  for (size_t k = st0; k < rig.statuses.size(); k++)
    if (rig.statuses[k].rumbleSeq != prev) rumbles.push_back(rig.statuses[k]), prev = rig.statuses[k].rumbleSeq;
  CHECK_EQ(rumbles.size(), (size_t)1);
  if (!rumbles.empty()) {
    CHECK_EQ(rumbles[0].rumbleStrong, (uint8_t)RUMBLE_STRONG_255);
    CHECK_EQ(rumbles[0].rumbleWeak, (uint8_t)RUMBLE_WEAK_255);
    CHECK_EQ(rumbles[0].rumbleMs, (uint16_t)RUMBLE_DURATION_MS);
    CHECK_EQ(rumbles[0].state, (uint8_t)DS_STATE_ARMED);
  }
  // Three more pulls: three more rumbles (each one held through the semi cooldown).
  for (int k = 0; k < 3; k++) {
    rig.pad.rt = kReleased;
    rig.runMs(kCtrlMs * 3);
    rig.pad.rt = kPulled;
    rig.runMs(kCoolMs + kStrokeMs + kCtrlMs * 3);
  }
  CHECK_EQ((uint8_t)(rig.lastStatus().rumbleSeq - seq0), (uint8_t)4);
  CHECK_EQ(rig.trigger().shotsTotal(), (uint16_t)4);
  rig.onShow = nullptr;
  rig.everyLoop = nullptr;
}

// 12. Ammo 0 (the owner's change of 2026-11-03: FIRE_LOCKOUT_WHEN_EMPTY false): the trigger still
//     fires, semi and full auto, and the count stays at 0; the LEDs double-flash purple over the
//     colour they show anyway (ARMED red / yellow, DISARMED green), whether or not the trigger is
//     pulled, until X restores MAG_CAPACITY.
TEST(accept_12_empty_magazine_keeps_firing_flashes_purple_and_x_restores) {
  Rig rig;
  CHECK(rig.toArmedSpunUp());
  for (unsigned k = 0; k < MAG_CAPACITY; k++) {   // each pull: one stroke, then the semi cooldown
    rig.pad.rt = kPulled;
    rig.runMs(kStrokeMs + kCoolMs + kCtrlMs * 2);
    rig.pad.rt = kReleased;
    rig.runMs(kCtrlMs * 3);
  }
  CHECK_EQ(rig.trigger().ammo(), (uint8_t)0);
  CHECK_EQ(rig.trigger().shotsTotal(), (uint16_t)MAG_CAPACITY);
  CHECK(rig.app->turret().display().emptyMagBlink);   // at once, trigger released
  rig.runMs(LCD_REFRESH_MS * 4);
  CHECK(rig.lcd(0).find(ammoText(0)) != std::string::npos);
  CHECK(!rig.telemetry.empty() && (rig.telemetry.back().flags & DS_TELEM_FLAG_EMPTY_MAG_BLINK));
  const Rgb purple = scaled(LED_COLOR_EMPTY_MAG);
  const Rgb red = scaled(LED_COLOR_ARMED_READY);
  const Rgb yellow = scaled(LED_COLOR_ARMED_WAIT);
  const Rgb green = scaled(LED_COLOR_DISARMED);
  struct Count {
    uint32_t purple = 0, red = 0, yellow = 0, green = 0, other = 0;
  } n;
  rig.onShow = [&] {
    if (allLeds(purple)) n.purple++;
    else if (allLeds(red)) n.red++;
    else if (allLeds(yellow)) n.yellow++;
    else if (allLeds(green)) n.green++;
    else n.other++;
  };
  // Idle at 0: purple flashes over red (a pull would fire), most of each period red.
  rig.runMs(2 * LED_EMPTY_FLASH_PERIOD_MS);
  CHECK(n.purple > 0u && n.red > n.purple);
  CHECK_EQ(n.yellow + n.green + n.other, 0u);
  // Pulled at 0: it fires, the count stays 0, and the stroke shows yellow under the flashes.
  n = Count();
  rig.pad.rt = kPulled;
  rig.runMs(kStrokeMs + kCtrlMs * 2);
  CHECK_EQ(rig.trigger().shotsTotal(), (uint16_t)(MAG_CAPACITY + 1));
  CHECK_EQ(rig.trigger().ammo(), (uint8_t)0);
  CHECK(n.yellow > 0u);
  CHECK_EQ(n.green + n.other, 0u);
  rig.pad.rt = kReleased;
  rig.runMs(kCtrlMs * 3);
  // Full auto at 0, held: back to back, the count still 0, never a red frame while it cycles.
  rig.press(DS_BTN_B);
  CHECK_EQ(rig.trigger().fireMode(), (uint8_t)DS_FIRE_MODE_FULL);
  rig.pad.rt = kPulled;
  CHECK(rig.runUntil([&] { return rig.trigger().strokeActive(); }, kCtrlMs * 3));
  n = Count();
  rig.runMs(3 * kStrokeMs);
  CHECK(rig.trigger().shotsTotal() >= (uint16_t)(MAG_CAPACITY + 1 + 3));
  CHECK_EQ(rig.trigger().ammo(), (uint8_t)0);
  CHECK(n.yellow > 0u && n.purple > 0u);
  CHECK_EQ(n.red + n.green + n.other, 0u);
  rig.pad.rt = kReleased;
  rig.runMs(kStrokeMs * 2);
  CHECK(!rig.trigger().strokeActive());
  // DISARMED at 0: still flashing, over green.
  rig.press(DS_BTN_A);
  CHECK_EQ(rig.state(), (uint8_t)DS_STATE_DISARMED);
  CHECK(rig.app->turret().display().emptyMagBlink);
  rig.runMs(kCtrlMs * 3);
  n = Count();
  rig.runMs(2 * LED_EMPTY_FLASH_PERIOD_MS);
  CHECK(n.purple > 0u && n.green > n.purple);
  CHECK_EQ(n.red + n.yellow + n.other, 0u);
  // X: MAG_CAPACITY, the flashing stops at once, plain green.
  rig.press(DS_BTN_X);
  CHECK_EQ(rig.trigger().ammo(), (uint8_t)MAG_CAPACITY);
  CHECK(!rig.app->turret().display().emptyMagBlink);
  n = Count();
  rig.runMs(2 * LED_EMPTY_FLASH_PERIOD_MS);
  CHECK(n.green > 0u);
  CHECK_EQ(n.purple + n.red + n.yellow + n.other, 0u);
  CHECK(!(rig.telemetry.back().flags & DS_TELEM_FLAG_EMPTY_MAG_BLINK));
  rig.onShow = nullptr;
  // Armed again (LB still held): firing goes on from the full magazine.
  rig.press(DS_BTN_A);
  CHECK_EQ(rig.state(), (uint8_t)DS_STATE_ARMED);
  rig.runMs(FLYWHEEL_SPINUP_MS + kCtrlMs * 3);
  const uint16_t before = rig.trigger().shotsTotal();
  rig.pad.rt = kPulled;
  rig.runMs(kStrokeMs + kCtrlMs * 3);
  CHECK(rig.trigger().shotsTotal() > before);
  CHECK(rig.trigger().ammo() < (uint8_t)MAG_CAPACITY);
}

// Full auto with the trigger held (the owner's rule, 2026-11-03): the LEDs stay yellow from
// stroke to stroke, never one red frame in between. TurretState publishes before Trigger runs,
// and Trigger starts the next stroke in the update that completes the last, so every display
// model while it cycles says "not ready"; checked after every loop and on every shown frame.
TEST(full_auto_held_stays_yellow_between_strokes) {
  Rig rig;
  CHECK(rig.toArmedSpunUp());
  rig.press(DS_BTN_B);
  CHECK_EQ(rig.trigger().fireMode(), (uint8_t)DS_FIRE_MODE_FULL);
  rig.runMs(kCtrlMs * 3);
  CHECK(allLeds(scaled(LED_COLOR_ARMED_READY)));
  rig.pad.rt = kPulled;
  CHECK(rig.runUntil([&] { return rig.trigger().strokeActive(); }, kCtrlMs * 3));
  rig.runMs(1);   // the loop after the start has published "not ready"
  const uint16_t shots0 = rig.trigger().shotsTotal();
  bool readySeen = false, redShown = false, otherShown = false;
  uint32_t yellowShows = 0, loops = 0;
  rig.everyLoop = [&] {   // the model the previous loop() left for the LEDs
    loops++;
    readySeen = readySeen || rig.app->turret().display().fireReady;
  };
  rig.onShow = [&] {
    if (allLeds(scaled(LED_COLOR_ARMED_WAIT))) yellowShows++;
    else if (allLeds(scaled(LED_COLOR_ARMED_READY))) redShown = true;
    else otherShown = true;
  };
  const uint32_t strokes = 4;
  rig.runMs(strokes * kStrokeMs);
  rig.everyLoop = nullptr;
  rig.onShow = nullptr;
  CHECK(rig.trigger().shotsTotal() >= (uint16_t)(shots0 + strokes - 1));
  CHECK(rig.trigger().ammo() > 0u);   // not into the empty flash
  CHECK(loops > 0u && yellowShows > 0u);
  CHECK(!readySeen);
  CHECK(!redShown);
  CHECK(!otherShown);
  // Released: the running stroke finishes, then red.
  rig.pad.rt = kReleased;
  rig.runMs(kStrokeMs + LED_FIRE_FLASH_MIN_MS + kCtrlMs * 5);
  CHECK(!rig.trigger().strokeActive());
  CHECK(rig.app->turret().display().fireReady);
  CHECK(allLeds(scaled(LED_COLOR_ARMED_READY)));
}

// Yellow while the flywheels spin up (FIRE_REQUIRES_FLYWHEEL), red once a pull would fire, and
// yellow for the stroke of a shell `fire` and the semi cooldown after it too.
TEST(armed_is_yellow_through_spin_up_and_shell_fire_then_red) {
  Rig rig;
  CHECK(rig.toArmed());
  rig.runMs(kCtrlMs * 3);
  const Rgb red = scaled(LED_COLOR_ARMED_READY);
  const Rgb yellow = scaled(LED_COLOR_ARMED_WAIT);
  if (FIRE_REQUIRES_FLYWHEEL) {
    CHECK(allLeds(yellow));
    CHECK(!rig.app->turret().display().fireReady);
    rig.pad.buttons = DS_BTN_LB;
    CHECK(rig.runUntil([&] { return rig.app->flywheel().on(); }, kCtrlMs * 3));
    const uint32_t on = rig.app->flywheel().onSinceMs();
    bool redEarly = false;
    rig.onShow = [&] { redEarly = redEarly || (allLeds(red) && rig.ms() - on + 1 < FLYWHEEL_SPINUP_MS); };
    rig.runMs(FLYWHEEL_SPINUP_MS / 2);
    CHECK(allLeds(yellow));                          // still spinning up
    rig.runMs(FLYWHEEL_SPINUP_MS / 2 + 2 * MS_PER_S / LED_REFRESH_HZ + kCtrlMs);
    rig.onShow = nullptr;
    CHECK(!redEarly);
  }
  CHECK(allLeds(red));                               // a pull would fire now
  CHECK(rig.shell("fire").find("fire: ok") != std::string::npos);   // the shell's stroke
  CHECK(rig.trigger().strokeActive());
  CHECK(!rig.app->turret().display().fireReady);
  CHECK(allLeds(yellow));
  CHECK(rig.runUntil([&] { return !rig.trigger().strokeActive(); }, kStrokeMs * 2));
  CHECK_EQ(rig.trigger().shotsTotal(), (uint16_t)1);
  const uint32_t frameMs = MS_PER_S / LED_REFRESH_HZ;
  if (kCoolMs > 2 * frameMs + LED_SHOW_MAX_DEFER_MS) {   // the semi cooldown: still yellow
    rig.runMs(frameMs + LED_SHOW_MAX_DEFER_MS);
    CHECK(rig.trigger().coolingDown());
    CHECK(!rig.app->turret().display().fireReady);
    CHECK(allLeds(yellow));
  }
  rig.runMs(std::max<uint32_t>(kCoolMs, LED_FIRE_FLASH_MIN_MS) + 2 * frameMs + kCtrlMs);
  CHECK(allLeds(red));
}

// 13. Stopping the relay -> within LINK_TIMEOUT_MS motion stops, flywheels stop, a stroke is
//     aborted (rest, no decrement), state PAIRING ("Pairing" for LINK_RECONNECT_GRACE_MS, then
//     "No Link"); reconnecting restores DISARMED only, even with A held.
TEST(accept_13_link_loss_stops_everything_within_link_timeout_and_reconnects_disarmed) {
  Rig rig;
  CHECK(rig.toArmedSpunUp());
  rig.pad.rx = kStickMax;
  rig.runMs(MS_PER_S / 2);
  rig.pad.rt = kPulled;
  rig.runUntil([&] { return rig.trigger().phase() == kFirstPhase; }, kStrokeMs);
  CHECK(rig.trigger().strokeActive());
  const uint8_t ammo = rig.trigger().ammo();
  Trigger::Phase endedIn = Trigger::Phase::IDLE;
  rig.everyLoop = watchStrokeEnd(rig, endedIn);
  rig.pad.send = false;
  const uint64_t lastPacket = rig.lastSendUs;
  CHECK(rig.runUntil([&] { return rig.state() == DS_STATE_PAIRING; }, LINK_TIMEOUT_MS * 2));
  rig.everyLoop();   // once more: runUntil stops right after the loop that aborted it
  rig.everyLoop = nullptr;
  CHECK(rig.us() - lastPacket <= (uint64_t)LINK_TIMEOUT_MS * US_PER_MS + kStepUs);
  CHECK(!rig.trigger().strokeActive());                 // aborted ...
  if (LINK_TIMEOUT_MS + kCtrlMs < kSweepMs) CHECK_EQ(endedIn, Trigger::Phase::SWEEP_OUT);   // ... mid sweep ...
  CHECK_EQ(servoLogical(), (float)TRIGGER_REST_ANGLE);   // ... at rest now ...
  CHECK_EQ(rig.trigger().ammo(), ammo);                 // ... and not counted
  if (FAILSAFE_STOP_FLYWHEEL) CHECK_EQ(hal_host::pwmDuty(PIN_FLY_ENA), (uint8_t)0);
  const float v = std::fabs(rig.motion().velocityDeg(AXIS_PAN));
  CHECK(rig.runUntil([&] { return !rig.motion().isMoving(); }, (uint64_t)(v / PAN_ACCEL_DEG_PER_S2 * MS_PER_S) + 50));
  rig.runMs(LCD_REFRESH_MS * 4);
  CHECK_EQ(rig.app->turret().display().pairing, PairingPhase::GRACE);
  CHECK(rig.lcd(1).find(LCD_TXT_PAIRING) != std::string::npos);
  rig.runMs(LINK_RECONNECT_GRACE_MS);
  CHECK_EQ(rig.app->turret().display().pairing, PairingPhase::NO_LINK);
  CHECK_EQ(rig.lcd(1), padRow(LCD_TXT_MODE_BT, LCD_TXT_NO_CONNECTION));
  // Reconnect with A, LB and RT all held: DISARMED, nothing armed, spun or fired.
  const uint16_t shots = rig.trigger().shotsTotal();
  rig.pad.buttons = DS_BTN_A | DS_BTN_LB;
  rig.pad.send = true;
  rig.runMs(MS_PER_S);
  CHECK_EQ(rig.state(), (uint8_t)DS_STATE_DISARMED);
  CHECK_EQ(hal_host::pwmDuty(PIN_FLY_ENA), (uint8_t)0);
  CHECK_EQ(rig.trigger().shotsTotal(), shots);
  CHECK(rig.countLines("state ARMED") == 1u);           // only the original arming
}

// A stream of connected = 0 frames is a link loss too (the relay's source-change hold).
TEST(connected_zero_frames_are_a_link_loss) {
  Rig rig;
  CHECK(rig.toArmed());
  rig.pad.connected = false;
  const uint64_t t0 = rig.us();
  CHECK(rig.runUntil([&] { return rig.state() == DS_STATE_PAIRING; }, LINK_TIMEOUT_MS));
  CHECK(rig.us() - t0 <= (uint64_t)kCtrlMs * US_PER_MS + kStepUs);
  rig.pad.connected = true;
  rig.runMs(kCtrlMs * 3);
  CHECK_EQ(rig.state(), (uint8_t)DS_STATE_DISARMED);
}

// 14. Mode toggle -> StatusPacket.wantMode changes, the LCD mode label changes, state ->
//     PAIRING (a link loss), and back to DISARMED (never ARMED) while USB still drives.
TEST(accept_14_mode_toggle_changes_want_mode_lcd_and_goes_through_pairing) {
  Rig rig;
  CHECK(rig.toArmed());
  CHECK_EQ(rig.lastStatus().wantMode, (uint8_t)DS_RADIO_BLE);
  const size_t from = rig.lines.size();
  rig.setWifi(true);
  rig.runMs(TOGGLE_DEBOUNCE_MS + STATUS_HEARTBEAT_MS);
  CHECK_EQ(rig.app->inputs().radioMode(), (uint8_t)DS_RADIO_WIFI);
  CHECK_EQ(rig.lastStatus().wantMode, (uint8_t)DS_RADIO_WIFI);
  CHECK_EQ(rig.app->link().status().wantMode, (uint8_t)DS_RADIO_WIFI);
  CHECK(rig.countLines("link lost: mode toggle", from) == 1u);
  CHECK(rig.countLines("state PAIRING", from) == 1u);
  CHECK_EQ(rig.state(), (uint8_t)DS_STATE_DISARMED);    // USB is still connected: reconnects
  CHECK(startsWith(rig.lcd(1), LCD_TXT_MODE_WIFI));
  // With nothing connected the switch leaves it pairing, with the new label.
  rig.pad.send = false;
  rig.runMs(LINK_TIMEOUT_MS * 2);
  rig.setWifi(false);
  rig.runMs(TOGGLE_DEBOUNCE_MS + LCD_REFRESH_MS * 4);
  CHECK_EQ(rig.state(), (uint8_t)DS_STATE_PAIRING);
  CHECK(startsWith(rig.lcd(1), LCD_TXT_MODE_BT));
  CHECK_EQ(rig.lastStatus().wantMode, (uint8_t)DS_RADIO_BLE);
}

// 15. With a pot on A2 and detection enabled, a sudden twist while moving -> both axes stop,
//     the moving one backs off, red blink, LCD FAULT: STALL, clears after STALL_FAULT_HOLD_MS
//     back to the previous arm state.
TEST(accept_15_stall_twist_stops_backs_off_blinks_and_clears) {
  Rig rig(stallOn());
  CHECK(rig.toArmed());
  rig.press(DS_BTN_B);   // full auto, to see it survives the fault
  rig.pad.rx = kStickMax;   // pan clockwise (negative)
  rig.runMs(MS_PER_S / 2);
  CHECK(rig.motion().velocityDeg(AXIS_PAN) < 0.0f);
  const float before = rig.motion().positionDeg(AXIS_PAN);
  (void)before;
  hal_host::setAnalog(PIN_STALL_SENSE, (uint16_t)(kA2Baseline + STALL_DELTA_COUNTS + STALL_DELTA_COUNTS / 2));
  const uint64_t twist = rig.us();
  CHECK(rig.runUntil([&] { return rig.state() == DS_STATE_FAULT; }, 50));
  const uint64_t detectUs = rig.us() - twist;
  CHECK(detectUs <= (uint64_t)(STALL_CONFIRM_SAMPLES + 2) * US_PER_S / STALL_SAMPLE_HZ);
  hal_host::setAnalog(PIN_STALL_SENSE, kA2Baseline);   // the twist is over
  rig.pad.rx = 0;
  const float atStall = rig.motion().positionDeg(AXIS_PAN);
  CHECK_EQ(rig.app->turret().fault(), (uint8_t)DS_FAULT_STALL);
  // The back-off: pan reverses STALL_BACKOFF_DEG; tilt (not moving) stays.
  const int32_t tilt0 = hal::stepPosition(AXIS_TILT);
  uint32_t redShows = 0, darkShows = 0;
  rig.onShow = [&] { (allLeds(scaled(LED_COLOR_FAULT)) ? redShows : darkShows)++; };
  rig.runMs(STALL_FAULT_HOLD_MS / 2);
  CHECK(!rig.motion().isMoving());
  CHECK_NEAR(rig.motion().positionDeg(AXIS_PAN), atStall + STALL_BACKOFF_DEG, 2.5f / PAN_STEPS_PER_DEG);
  CHECK_EQ(hal::stepPosition(AXIS_TILT), tilt0);
  CHECK(redShows > 0u && darkShows > 0u);              // hard blink
  rig.runMs(LCD_REFRESH_MS * 2);
  CHECK_EQ(rig.lcd(0), padRow(LCD_TXT_FAULT_STALL, ""));
  CHECK_EQ(hal_host::pwmDuty(PIN_FLY_ENA), (uint8_t)0);
  // The stick is ignored in FAULT.
  rig.pad.rx = -kStickMax;
  rig.runMs(kCtrlMs * 5);
  CHECK(!rig.motion().isMoving());
  rig.pad.rx = 0;
  // Clears STALL_FAULT_HOLD_MS after the stall, back to ARMED (full auto kept).
  const uint64_t stallAt = twist + detectUs;
  CHECK(rig.runUntil([&] { return rig.state() != DS_STATE_FAULT; }, STALL_FAULT_HOLD_MS));
  CHECK(rig.us() - stallAt >= (uint64_t)STALL_FAULT_HOLD_MS * US_PER_MS - kStepUs);
  CHECK(rig.us() - stallAt <= (uint64_t)STALL_FAULT_HOLD_MS * US_PER_MS + 2 * kStepUs);
  CHECK_EQ(rig.state(), (uint8_t)DS_STATE_ARMED);
  CHECK_EQ(rig.trigger().fireMode(), (uint8_t)DS_FIRE_MODE_FULL);
  rig.onShow = nullptr;
  rig.flush();   // the stall log line names the axes and the peak delta (for tuning)
  CHECK(rig.countLines("STALL axes") == 1u);
  CHECK(rig.lastLine("STALL axes").find("peak delta") != std::string::npos);
}

// A link that drops during the stall fault sends it to PAIRING instead; from DISARMED it
// returns to DISARMED; B and X still work in FAULT.
TEST(stall_fault_returns_to_pairing_if_the_link_dropped_and_ignores_a) {
  Rig rig(stallOn());
  CHECK(rig.toDisarmed());
  rig.pad.ry = -kStickMax;
  rig.runMs(MS_PER_S / 2);
  hal_host::setAnalog(PIN_STALL_SENSE, (uint16_t)(kA2Baseline - STALL_DELTA_COUNTS - STALL_DELTA_COUNTS / 2));
  CHECK(rig.runUntil([&] { return rig.state() == DS_STATE_FAULT; }, 50));
  hal_host::setAnalog(PIN_STALL_SENSE, kA2Baseline);
  rig.pad.ry = 0;
  rig.press(DS_BTN_A);
  CHECK_EQ(rig.state(), (uint8_t)DS_STATE_FAULT);
  rig.press(DS_BTN_B);
  CHECK_EQ(rig.trigger().fireMode(), (uint8_t)DS_FIRE_MODE_FULL);
  rig.runUntil([&] { return rig.state() != DS_STATE_FAULT; }, STALL_FAULT_HOLD_MS);
  CHECK_EQ(rig.state(), (uint8_t)DS_STATE_DISARMED);
  // Again, and drop the link during the fault: PAIRING, then DISARMED on reconnection.
  rig.press(DS_BTN_A);
  CHECK_EQ(rig.state(), (uint8_t)DS_STATE_ARMED);
  rig.pad.rx = kStickMax;
  rig.runMs(MS_PER_S / 2);
  hal_host::setAnalog(PIN_STALL_SENSE, (uint16_t)(kA2Baseline + STALL_DELTA_COUNTS * 2));
  CHECK(rig.runUntil([&] { return rig.state() == DS_STATE_FAULT; }, 50));
  hal_host::setAnalog(PIN_STALL_SENSE, kA2Baseline);
  rig.pad.rx = 0;
  rig.pad.send = false;
  rig.runMs(STALL_FAULT_HOLD_MS + kCtrlMs);
  CHECK_EQ(rig.state(), (uint8_t)DS_STATE_PAIRING);
  rig.pad.send = true;
  rig.runMs(kCtrlMs * 3);
  CHECK_EQ(rig.state(), (uint8_t)DS_STATE_DISARMED);   // never back to ARMED
}

// A fault mid-stroke (architecture.md 6), here while FIRE is held: the pusher goes to rest at
// once, the stroke is aborted and not counted, the flywheels are cut.
TEST(stall_mid_stroke_aborts_without_a_decrement) {
  Rig rig(stallOn());
  CHECK(rig.toArmedSpunUp());
  rig.pad.rx = kStickMax;
  rig.runMs(MS_PER_S / 2);
  rig.pad.rt = kPulled;
  CHECK(rig.runUntil([&] { return rig.trigger().phase() == Trigger::Phase::HOLD_FIRE; }, kStrokeMs));
  CHECK_EQ(servoLogical(), (float)TRIGGER_FIRE_ANGLE);
  const uint8_t ammo = rig.trigger().ammo();
  Trigger::Phase endedIn = Trigger::Phase::IDLE;
  rig.everyLoop = watchStrokeEnd(rig, endedIn);
  hal_host::setAnalog(PIN_STALL_SENSE, (uint16_t)(kA2Baseline + STALL_DELTA_COUNTS * 2));
  CHECK(rig.runUntil([&] { return rig.state() == DS_STATE_FAULT; }, 50));
  rig.everyLoop();   // once more: runUntil stops right after the loop that aborted it
  rig.everyLoop = nullptr;
  hal_host::setAnalog(PIN_STALL_SENSE, kA2Baseline);
  CHECK(!rig.trigger().strokeActive());
  CHECK_EQ(endedIn, Trigger::Phase::HOLD_FIRE);
  CHECK_EQ(servoLogical(), (float)TRIGGER_REST_ANGLE);
  CHECK_EQ(hal_host::pwmDuty(PIN_FLY_ENA), (uint8_t)0);
  rig.runMs(kStrokeMs);
  CHECK_EQ(rig.trigger().ammo(), ammo);
  CHECK_EQ(rig.trigger().shotsTotal(), (uint16_t)(MAG_CAPACITY - ammo));
}

// Spec 3.1 FAULT: "flywheels off" for the whole fault, not only at its start: LB held (hold
// mode) or pressed again (toggle mode) never spins them, and the shell cannot latch them on.
TEST(flywheels_stay_cut_for_the_whole_fault_whatever_lb_does) {
  for (const bool holdMode : {true, false}) {
    App::Config cfg = stallOn();
    cfg.turret.flywheelHoldMode = holdMode;
    Rig rig(cfg);
    CHECK(rig.toArmedSpunUp());   // LB held from here on (toggle mode: that press latched them)
    rig.pad.rx = kStickMax;
    rig.runMs(MS_PER_S / 2);
    hal_host::setAnalog(PIN_STALL_SENSE, (uint16_t)(kA2Baseline + STALL_DELTA_COUNTS * 2));
    CHECK(rig.runUntil([&] { return rig.state() == DS_STATE_FAULT; }, 50));
    hal_host::setAnalog(PIN_STALL_SENSE, kA2Baseline);
    rig.pad.rx = 0;
    bool spun = false;
    rig.everyLoop = [&] {
      if (rig.state() == DS_STATE_FAULT)
        spun = spun || rig.app->flywheel().on() || rig.app->flywheel().duty() > 0;
    };
    rig.runMs(FLYWHEEL_RAMP_MS);                                // LB still held
    rig.pad.buttons = (uint16_t)(rig.pad.buttons & ~DS_BTN_LB);
    rig.runMs(BUTTON_DEBOUNCE_MS + kCtrlMs * 2);
    rig.press(DS_BTN_LB);                                       // a fresh press
    CHECK(rig.shell("fly on").find("fly on: refused") != std::string::npos);
    CHECK_EQ(rig.state(), (uint8_t)DS_STATE_FAULT);             // all of it inside the fault
    rig.everyLoop = nullptr;
    CHECK(!spun);
  }
}

// Stall events during a CAL_RAM calibration are expected (the barrel rams the panel): ignored.
TEST(stalls_during_cal_ram_are_ignored) {
  Rig rig(stallOn());
  rig.runMs(MS_PER_S);
  CHECK_EQ(rig.state(), (uint8_t)DS_STATE_CALIBRATING);
  hal_host::setAnalog(PIN_STALL_SENSE, (uint16_t)(kA2Baseline + STALL_DELTA_COUNTS * 2));
  rig.runMs(MS_PER_S / 2);
  CHECK(rig.app->stall().stallCount() >= 1u);           // declared ...
  CHECK_EQ(rig.state(), (uint8_t)DS_STATE_CALIBRATING); // ... and ignored
  hal_host::setAnalog(PIN_STALL_SENSE, kA2Baseline);
  CHECK(rig.toDisarmed());
  CHECK(rig.motion().calibrated());
  CHECK_EQ(rig.app->turret().status().stalls, 0u);
}

// 16. LED brightness never exceeds LED_MAX_BRIGHTNESS in any state, through a long random
//     session of every input (power, mode, link drops, buttons, sticks, trigger, A2).
TEST(accept_16_brightness_never_exceeds_the_limit_in_a_long_random_session) {
  Rig rig(stallOn());
  std::mt19937 rnd(20261103);
  std::uniform_int_distribution<int> stick(-JOY_AXIS_RANGE, JOY_AXIS_RANGE - 1);
  std::uniform_int_distribution<int> trig(0, TRIGGER_AXIS_RANGE);
  std::uniform_int_distribution<int> pct(0, 99);
  std::set<uint8_t> states;
  rig.everyLoop = [&] { states.insert(rig.state()); };
  for (int k = 0; k < 600; k++) {   // 600 x 100 ms = 60 s of virtual time
    const int r = pct(rnd);
    if (r < 2) rig.setPower(pct(rnd) < 70);
    else if (r < 4) rig.setWifi(pct(rnd) < 50);
    else if (r < 8) rig.pad.send = pct(rnd) < 80;
    else if (r < 10) hal_host::setAnalog(PIN_STALL_SENSE, (uint16_t)(pct(rnd) * ADC_MAX_COUNTS / 100));
    rig.pad.rx = (int16_t)stick(rnd);
    rig.pad.ry = (int16_t)stick(rnd);
    rig.pad.rt = (uint16_t)trig(rnd);
    rig.pad.buttons = (uint16_t)(rnd() & DS_BTN_ALL);
    rig.runMs(100);
    if (k % 100 == 99) rig.setPower(true), rig.pad.send = true, hal_host::setAnalog(PIN_STALL_SENSE, kA2Baseline);
  }
  CHECK(states.size() >= 5u);   // the session really went through the states
  CHECK(hal_host::ledMaxChannel() <= LED_MAX_BRIGHTNESS);
  CHECK(LED_MAX_BRIGHTNESS <= LED_BRIGHTNESS_LIMIT);
  CHECK_EQ(hal_host::ledOverLimit(), 0u);
  CHECK_EQ(rig.app->leds().overLimit(), 0u);
  for (const auto& t : rig.telemetry) {
    CHECK(t.ledR <= LED_MAX_BRIGHTNESS);
    CHECK(t.ledG <= LED_MAX_BRIGHTNESS);
    CHECK(t.ledB <= LED_MAX_BRIGHTNESS);
  }
  CHECK_EQ(blockingWrites(), 0u);
}

// 17. Loop timing: LCD + LEDs active and both steppers at max speed for 60 s: no write ever
//     blocks, the stats line prints every DEBUG_STATS_INTERVAL_MS with jitter < 10 % and packet
//     loss < 1 % (the debug stats print is the proof).
TEST(accept_17_loop_never_blocks_and_the_stats_prove_it) {
  Rig rig;
  CHECK(rig.toArmed());
  const size_t from = rig.lines.size();
  const uint32_t sent0 = rig.framesSent;
  const uint32_t usb0 = rig.app->link().stats().usbFrames;
  const uint64_t t0 = rig.us();
  rig.pad.rx = kStickMax;   // pan at full speed all the time (no pan limits by default)
  for (int k = 0; k < 60; k++) {
    rig.pad.ry = (k / 2) % 2 ? kStickMax : -kStickMax;   // tilt end to end
    rig.pad.rt = k % 3 ? 0 : kPulled;                     // strokes: LEDs and the LCD change
    rig.runMs(MS_PER_S);
  }
  CHECK_EQ(rig.us() - t0, (uint64_t)60 * US_PER_S);   // only the test moved time
  CHECK_EQ(blockingWrites(), 0u);
  CHECK_EQ(hal_host::ledUnsafeShows(), rig.app->leds().forcedShows());
  const uint32_t got = rig.app->link().stats().usbFrames - usb0;
  CHECK(got + 2 >= rig.framesSent - sent0);              // nothing lost on the way in
  CHECK_EQ(rig.app->link().stats().seqGaps, 0u);
  CHECK_EQ(rig.app->link().stats().crcErrors, 0u);
  size_t statsLines = 0;
  for (size_t k = from; k < rig.lines.size(); k++) {
    const std::string& l = rig.lines[k];
    if (l.find(" stats loop=") == std::string::npos) continue;
    statsLines++;
    CHECK(l.find("loss=0.0%") != std::string::npos);
    CHECK(l.find('!') == std::string::npos);              // nothing over its limit
    CHECK(l.find(" rx=" + std::to_string(CONTROL_PACKET_HZ)) != std::string::npos ||
          l.find(" rx=" + std::to_string(CONTROL_PACKET_HZ - 1)) != std::string::npos);
  }
  CHECK(statsLines >= 58u);
  CHECK(rig.app->debug().stats().jitterMaxPermille < STEP_JITTER_LIMIT_PERMILLE);
  CHECK(rig.app->debug().stats().loopHz >= US_PER_S / kStepUs - 1);
  CHECK(hal_host::totalSteps(AXIS_PAN) > (uint32_t)(50 * PAN_MAX_DEG_PER_S * PAN_STEPS_PER_DEG));
}

// =====================================================================================================
// Failure paths and rules where the spec is silent (architecture.md 6)
// =====================================================================================================

// CAL_STALL_SENSE without a stall: FAULT CAL after TILT_CAL_TIMEOUT_MS; only the power toggle
// clears it (not the link, not A, not the shell, not time).
TEST(cal_stall_sense_without_a_stall_faults_until_power_cycled) {
  App::Config cfg = App::defaults();
  cfg.turret.calMode = CAL_STALL_SENSE;
  Rig rig(cfg);
  rig.runMs(MS_PER_S);
  CHECK_EQ(rig.state(), (uint8_t)DS_STATE_CALIBRATING);
  CHECK(rig.app->stall().forced());                     // detection forced on for the search
  CHECK(rig.runUntil([&] { return rig.state() == DS_STATE_FAULT; }, TILT_CAL_TIMEOUT_MS + MS_PER_S));
  CHECK(rig.ms() >= TILT_CAL_TIMEOUT_MS);
  CHECK_EQ(rig.app->turret().fault(), (uint8_t)DS_FAULT_CAL);
  CHECK(!rig.app->stall().forced());
  CHECK(!rig.motion().calibrated());
  rig.flush();
  CHECK(rig.countLines("calibration failed") == 1u);
  rig.runMs(LCD_REFRESH_MS * 4);
  CHECK_EQ(rig.lcd(0), padRow(LCD_TXT_FAULT_CAL, ""));
  uint32_t redShows = 0, darkShows = 0;
  rig.onShow = [&] { (allLeds(scaled(LED_COLOR_FAULT)) ? redShows : darkShows)++; };
  rig.press(DS_BTN_A);
  rig.runMs(STALL_FAULT_HOLD_MS * 2);
  rig.onShow = nullptr;
  CHECK(redShows > 0u && darkShows > 0u);
  CHECK(rig.shell("cal").find("cal: refused") != std::string::npos);
  CHECK(rig.shell("arm").find("arm: refused") != std::string::npos);
  CHECK_EQ(rig.state(), (uint8_t)DS_STATE_FAULT);
  rig.pad.send = false;                                 // a link drop does not clear it either
  rig.runMs(LINK_TIMEOUT_MS * 2);
  rig.pad.send = true;
  rig.runMs(MS_PER_S);
  CHECK_EQ(rig.state(), (uint8_t)DS_STATE_FAULT);
  // Power cycle: OFF, then a fresh boot and calibration; this time A2 shows the panel.
  rig.setPower(false);
  rig.runMs(TOGGLE_DEBOUNCE_MS + kCtrlMs);
  CHECK_EQ(rig.state(), (uint8_t)DS_STATE_OFF);
  CHECK_EQ(rig.app->turret().fault(), (uint8_t)DS_FAULT_NONE);
  rig.everyLoop = [&] {
    if (rig.state() == DS_STATE_CALIBRATING && hal_host::lostSteps(AXIS_TILT) > 0u &&
        rig.motion().calPhase() == Motion::CalPhase::SENSE_DOWN)
      hal_host::setAnalog(PIN_STALL_SENSE, (uint16_t)(kA2Baseline + STALL_DELTA_COUNTS * 2));
    else
      hal_host::setAnalog(PIN_STALL_SENSE, kA2Baseline);
  };
  const uint32_t lost0 = hal_host::lostSteps(AXIS_TILT);
  (void)lost0;
  rig.setPower(true);
  CHECK(rig.toDisarmed());
  rig.everyLoop = nullptr;
  CHECK(rig.motion().calibrated());
  CHECK_EQ(hal::stepPosition(AXIS_TILT), 0);
  CHECK(rig.countLines("calibration: stall at the bottom stop") == 1u);
}

// No step timer (hal::begin failed to claim it): the calibration cannot run -> FAULT CAL.
TEST(no_tilt_timer_is_a_cal_fault) {
  hal_host::reset();
  hal_host::setBeginTimerFailure(true);
  hal_host::setDigitalInput(PIN_TOGGLE_POWER, toggleLevel(true));
  App app;
  app.begin();
  for (int k = 0; k < 100; k++) {
    hal_host::advanceUs(kStepUs);
    app.loop();
  }
  CHECK_EQ(app.turret().state(), (uint8_t)DS_STATE_FAULT);
  CHECK_EQ(app.turret().fault(), (uint8_t)DS_FAULT_CAL);
  CHECK(app.debug().warnings() >= 1u);
}

// A source change while connected (USB bench override -> the radio) is a link loss: PAIRING,
// then DISARMED from the new source, never ARMED.
TEST(source_change_is_a_link_loss) {
  Rig rig;
  rig.esp.reset(new FakeEsp());
  CHECK(rig.toArmed());
  CHECK_EQ(rig.app->link().status().activeSource, (uint8_t)DS_LINK_SRC_USB);
  CHECK(rig.app->link().status().espAlive);
  const size_t from = rig.lines.size();
  rig.pad.send = false;   // the bench console stops: the pad on the radio takes over
  CHECK(rig.runUntil([&] { return rig.app->link().status().activeSource == DS_LINK_SRC_BLE; }, LINK_TIMEOUT_MS * 2));
  rig.runMs(kCtrlMs * 3);
  CHECK(rig.countLines("link lost: source changed", from) == 1u);
  CHECK(rig.countLines("state PAIRING", from) == 1u);
  CHECK_EQ(rig.state(), (uint8_t)DS_STATE_DISARMED);
  CHECK(rig.app->link().connected());
  CHECK_EQ(rig.app->turret().status().linkLosses, 1u);
}

// FAILSAFE_STOP_FLYWHEEL false: the flywheels keep spinning through PAIRING, and the DISARMED
// rules cut them on reconnection (FLYWHEEL_REQUIRES_ARMED).
TEST(failsafe_stop_flywheel_false_keeps_them_through_pairing) {
  App::Config cfg = App::defaults();
  cfg.turret.failsafeStopFlywheel = false;
  Rig rig(cfg);
  CHECK(rig.toArmedSpunUp());
  rig.pad.send = false;
  rig.runMs(LINK_TIMEOUT_MS * 2);
  CHECK_EQ(rig.state(), (uint8_t)DS_STATE_PAIRING);
  CHECK(rig.app->flywheel().on());
  CHECK_EQ(hal_host::pwmDuty(PIN_FLY_ENA), (uint8_t)FLYWHEEL_DUTY_255);
  rig.pad.buttons = 0;
  rig.pad.send = true;
  rig.runMs(kCtrlMs * 3);
  CHECK_EQ(rig.state(), (uint8_t)DS_STATE_DISARMED);
  if (FLYWHEEL_REQUIRES_ARMED) CHECK_EQ(hal_host::pwmDuty(PIN_FLY_ENA), (uint8_t)0);
}

// FAILSAFE_DISARM_ON_LOSS false is reported once at boot as ignored, and is ignored.
TEST(failsafe_disarm_false_is_reported_once_and_ignored) {
  App::Config cfg = App::defaults();
  cfg.turret.failsafeDisarmOnLoss = false;
  Rig rig(cfg);
  rig.runMs(MS_PER_S);
  CHECK_EQ(rig.countLines("FAILSAFE_DISARM_ON_LOSS"), (size_t)1);
  CHECK(rig.toArmed());
  rig.pad.send = false;
  rig.runMs(LINK_TIMEOUT_MS * 2);
  rig.pad.send = true;
  rig.runMs(MS_PER_S);
  CHECK_EQ(rig.state(), (uint8_t)DS_STATE_DISARMED);
  CHECK_EQ(rig.countLines("FAILSAFE_DISARM_ON_LOSS"), (size_t)1);
  // With the shipped default (true) there is nothing to report.
  Rig plain;
  plain.runMs(MS_PER_S / 2);
  CHECK_EQ(plain.countLines("FAILSAFE_DISARM_ON_LOSS"), (size_t)(FAILSAFE_DISARM_ON_LOSS ? 0 : 1));
}

// Boot warnings (Debug::bootWarnings): no LCD at either address, and the configured LCD texts
// that will be cut (Lcd lists each one; Debug adds the count). Both once, and the turret still
// runs without an LCD (rows go to the telemetry mirror only).
TEST(boot_warnings_name_a_missing_lcd_and_cut_texts) {
  hal_host::reset();
  hal_host::setLcdPresent(false, 0);
  hal_host::setDigitalInput(PIN_TOGGLE_POWER, toggleLevel(true));
  hal_host::setTiltBottomStop(true, stopSteps());
  std::unique_ptr<App> app(new App());
  app->begin();
  std::string text;
  for (int k = 0; k < 20000; k++) {   // 2 s
    hal_host::advanceUs(kStepUs);
    app->loop();
    text += hal_host::usbTakeTx();
  }
  auto count = [&](const std::string& needle) {
    size_t n = 0;
    for (size_t p = text.find(needle); p != std::string::npos; p = text.find(needle, p + 1)) n++;
    return n;
  };
  CHECK_EQ(count("warning: no LCD at LCD_I2C_ADDR or LCD_I2C_ADDR_FALLBACK"), (size_t)1);
  CHECK_EQ(count("LCD text combination(s) will be cut"), (size_t)(Lcd::overflowCount() ? 1 : 0));
  CHECK_EQ(app->debug().warnings(), (uint8_t)(1 + (Lcd::overflowCount() ? 1 : 0)));
  CHECK_EQ(hal_host::lcdTransactions(), 0u);
  CHECK_EQ(app->turret().state(), (uint8_t)DS_STATE_CALIBRATING);
}

// Power off mid-stroke, here in the sweep back: rest now, no decrement; the stroke's rumble is
// stopped (rumbleMs 0).
TEST(power_off_mid_stroke_aborts_without_a_decrement) {
  Rig rig;
  CHECK(rig.toArmedSpunUp());
  const Trigger::Phase inPhase = kSweepMs ? Trigger::Phase::SWEEP_BACK : Trigger::Phase::HOLD_REST;
  rig.pad.rt = kPulled;
  CHECK(rig.runUntil([&] { return rig.trigger().phase() == inPhase; }, kStrokeMs));
  const uint8_t ammo = rig.trigger().ammo();
  Trigger::Phase endedIn = Trigger::Phase::IDLE;
  rig.everyLoop = watchStrokeEnd(rig, endedIn);
  rig.setPower(false);
  CHECK(rig.runUntil([&] { return rig.state() == DS_STATE_OFF; }, TOGGLE_DEBOUNCE_MS * 2 + kCtrlMs));
  CHECK_EQ(servoLogical(), (float)TRIGGER_REST_ANGLE);   // at once
  rig.everyLoop();   // once more: runUntil stops right after the loop that aborted it
  rig.everyLoop = nullptr;
  CHECK_EQ(endedIn, inPhase);
  rig.runMs(STATUS_HEARTBEAT_MS);
  CHECK_EQ(rig.state(), (uint8_t)DS_STATE_OFF);
  CHECK_EQ(rig.trigger().ammo(), ammo);
  CHECK_EQ(servoLogical(), (float)TRIGGER_REST_ANGLE);
  CHECK_EQ(rig.lastStatus().rumbleMs, (uint16_t)0);
}

// Disarm (A) mid-stroke, here in the REST hold just before the stroke would count: rest now, no
// decrement, flywheels cut.
TEST(disarm_mid_stroke_aborts_and_cuts_the_flywheels) {
  Rig rig;
  CHECK(rig.toArmedSpunUp());
  rig.pad.rt = kPulled;
  CHECK(rig.runUntil([&] { return rig.trigger().phase() == Trigger::Phase::HOLD_REST; }, kStrokeMs));
  const uint8_t ammo = rig.trigger().ammo();
  Trigger::Phase endedIn = Trigger::Phase::IDLE;
  rig.everyLoop = watchStrokeEnd(rig, endedIn);
  rig.pad.buttons = (uint16_t)(rig.pad.buttons | DS_BTN_A);
  rig.runMs(kCtrlMs * 2);
  rig.everyLoop = nullptr;
  CHECK_EQ(rig.state(), (uint8_t)DS_STATE_DISARMED);
  CHECK(!rig.trigger().strokeActive());
  CHECK_EQ(endedIn, Trigger::Phase::HOLD_REST);
  CHECK_EQ(rig.trigger().ammo(), ammo);                 // aborted, not completed
  rig.runMs(kStrokeMs);
  CHECK_EQ(rig.trigger().ammo(), ammo);
  CHECK_EQ(servoLogical(), (float)TRIGGER_REST_ANGLE);
  if (FLYWHEEL_REQUIRES_ARMED) CHECK_EQ(hal_host::pwmDuty(PIN_FLY_ENA), (uint8_t)0);
}

// Boot with the power toggle open: straight to OFF, nothing moves until it closes.
TEST(boot_with_the_power_toggle_open_stays_off) {
  Rig rig(App::defaults(), false);
  rig.runMs(2 * MS_PER_S);
  CHECK_EQ(rig.state(), (uint8_t)DS_STATE_OFF);
  CHECK_EQ(hal_host::totalSteps(AXIS_TILT), 0u);
  CHECK(!hal_host::servoOn());
  CHECK(!hal::steppersEnabled());
  rig.setPower(true);
  CHECK(rig.toDisarmed());
}

// =====================================================================================================
// The shell (architecture.md 5): same permissions as the controller, whole-word commands
// =====================================================================================================

TEST(shell_actions_obey_the_controller_permissions) {
  Rig rig;
  rig.pad.send = false;
  CHECK(rig.runUntil([&] { return rig.state() == DS_STATE_PAIRING; }, 15 * MS_PER_S));
  CHECK(rig.shell("arm").find("arm: refused: not in this state") != std::string::npos);
  CHECK(rig.shell("pan 30").find("pan 30.0: refused: not in this state") != std::string::npos);
  CHECK(rig.shell("fire").find("fire: refused") != std::string::npos);
  rig.pad.send = true;
  rig.runMs(kCtrlMs * 3);
  CHECK_EQ(rig.state(), (uint8_t)DS_STATE_DISARMED);
  CHECK(rig.shell("fire").find("fire: refused: not in this state") != std::string::npos);
  CHECK(rig.shell("fly on").find(FLYWHEEL_REQUIRES_ARMED ? "fly on: refused" : "fly on: ok") != std::string::npos);
  CHECK(rig.shell("fly off").find("fly off: ok") != std::string::npos);
  // Arm refused while RT is held, like A.
  rig.pad.rt = kPulled;
  rig.runMs(kCtrlMs * 3);
  if (ARM_REQUIRES_TRIGGER_RELEASED) CHECK(rig.shell("arm").find("arm: refused: trigger held") != std::string::npos);
  rig.pad.rt = 0;
  rig.runMs(kCtrlMs * 3);
  CHECK(rig.shell("arm").find("arm: ok") != std::string::npos);
  CHECK_EQ(rig.state(), (uint8_t)DS_STATE_ARMED);
  // Fire needs the flywheels spun up (FIRE_REQUIRES_FLYWHEEL).
  if (FIRE_REQUIRES_FLYWHEEL) CHECK(rig.shell("fire").find("fire: refused: not ready") != std::string::npos);
  CHECK(rig.shell("fly on").find("fly on: ok") != std::string::npos);
  rig.runMs(FLYWHEEL_SPINUP_MS);
  CHECK(rig.app->flywheel().on());   // the shell latch holds them on in hold mode too
  CHECK(rig.shell("fire").find("fire: ok") != std::string::npos);
  rig.runMs(kStrokeMs + kCtrlMs);
  CHECK_EQ(rig.trigger().shotsTotal(), (uint16_t)1);
  CHECK_EQ(rig.trigger().ammo(), (uint8_t)(MAG_CAPACITY - 1));
  // Semi auto: `fire` obeys the cooldown after the stroke, like the trigger (the stroke completed
  // about kCtrlMs * 5 ago: the shell's own wait plus the kCtrlMs above).
  CHECK(kCtrlMs * 5 < kCoolMs);
  CHECK(rig.shell("fire").find("fire: refused: semi-auto cooldown") != std::string::npos);
  CHECK_EQ(rig.trigger().shotsTotal(), (uint16_t)1);
  rig.runMs(kCoolMs);
  CHECK(rig.shell("fire").find("fire: ok") != std::string::npos);
  rig.runMs(kStrokeMs + kCtrlMs);
  CHECK_EQ(rig.trigger().shotsTotal(), (uint16_t)2);
  CHECK_EQ(rig.trigger().ammo(), (uint8_t)(MAG_CAPACITY - 2));
  // Modes, magazine, positions.
  CHECK(rig.shell("mode full").find("mode full: ok") != std::string::npos);
  CHECK_EQ(rig.trigger().fireMode(), (uint8_t)DS_FIRE_MODE_FULL);
  CHECK(rig.shell("mode semi").find("mode semi: ok") != std::string::npos);
  CHECK(rig.shell("mag").find("mag: ok") != std::string::npos);
  CHECK_EQ(rig.trigger().ammo(), (uint8_t)MAG_CAPACITY);
  CHECK(rig.shell("tilt 20").find("tilt 20.0: ok") != std::string::npos);
  CHECK(rig.shell("pan -45.5").find("pan -45.5: ok") != std::string::npos);
  rig.runMs(3 * MS_PER_S);
  CHECK_NEAR(rig.motion().positionDeg(AXIS_TILT), 20.0f, 1.0f / TILT_STEPS_PER_DEG);
  CHECK_NEAR(rig.motion().positionDeg(AXIS_PAN), -45.5f, 1.0f / PAN_STEPS_PER_DEG);
  CHECK(rig.shell("tilt " + std::to_string((int)TILT_RANGE_DEG + 5)).find("refused: outside the limits") != std::string::npos);
  CHECK(rig.shell("tilt -5").find("refused: outside the limits") != std::string::npos);
  CHECK(rig.shell("sethome").find("sethome: ok") != std::string::npos);
  CHECK_NEAR(rig.motion().homeDeg(AXIS_PAN), -45.5f, 1.0f / PAN_STEPS_PER_DEG);
  CHECK(rig.shell("pan 10").find("ok") != std::string::npos);
  rig.runMs(MS_PER_S / 4);
  CHECK(rig.shell("stop").find("stop: ok") != std::string::npos);
  rig.runMs(MS_PER_S);
  CHECK(!rig.motion().isMoving());
  CHECK(rig.motion().positionDeg(AXIS_PAN) < 10.0f - 1.0f);   // stopped short of 10
  CHECK(rig.shell("home").find("home: ok") != std::string::npos);
  rig.runMs(3 * MS_PER_S);
  CHECK_NEAR(rig.motion().positionDeg(AXIS_PAN), -45.5f, 1.0f / PAN_STEPS_PER_DEG);
  CHECK(rig.shell("disarm").find("disarm: ok") != std::string::npos);
  CHECK_EQ(rig.state(), (uint8_t)DS_STATE_DISARMED);
  CHECK(!rig.app->flywheel().on() || !FLYWHEEL_REQUIRES_ARMED);
  // `cal` from DISARMED re-runs the calibration, then PAIRING -> DISARMED.
  CHECK(rig.shell("cal").find("cal: ok") != std::string::npos);
  CHECK_EQ(rig.state(), (uint8_t)DS_STATE_CALIBRATING);
  CHECK(rig.toDisarmed());
  CHECK_EQ(hal::stepPosition(AXIS_TILT), 0);
}

TEST(shell_tilt_before_calibration_is_refused) {
  Rig rig;
  rig.runMs(MS_PER_S);
  CHECK_EQ(rig.state(), (uint8_t)DS_STATE_CALIBRATING);
  CHECK(rig.shell("tilt 10").find("refused: not in this state") != std::string::npos);
  CHECK(rig.shell("stop").find("stop: ok") != std::string::npos);   // stop never refused
  CHECK_EQ(rig.state(), (uint8_t)DS_STATE_CALIBRATING);            // and it does not stop cal
  CHECK(rig.toDisarmed());
}

// Whole-word matching: the ESP32's boot banner and typos never run a command.
TEST(shell_matches_whole_words_only) {
  Rig rig;
  CHECK(rig.toDisarmed());
  const std::vector<std::string> junk = {
      "ESP-ROM:esp32s3-20210327", "rst:0x1 (POWERON),boot:0x8 (SPI_FAST_FLASH_BOOT)",
      "mode:DIO, clock div:1", "load:0x3fce3818,len:0x508", "entry 0x403c98d8",
      "arming", "armed", "arm now", "fire!", "fly", "fly up", "mode auto", "pan", "pan 30deg",
      "pan 1e3", "pan --5", "tilt .", "calibrate", "stop it", "log 9", "log", "STATUS", "stall"};
  for (const std::string& j : junk) {
    const std::string r = rig.shell(j);
    CHECK(r.find(": ok") == std::string::npos);
    CHECK(r.find("unknown command") != std::string::npos || r.find("not run") != std::string::npos);
  }
  CHECK_EQ(rig.state(), (uint8_t)DS_STATE_DISARMED);
  CHECK(!rig.motion().isMoving());
  CHECK_EQ(rig.trigger().fireMode(), (uint8_t)DS_FIRE_MODE_SEMI);
  CHECK_EQ(rig.app->debug().level(), (uint8_t)DEBUG_LOG_LEVEL);
  CHECK(rig.app->debug().shellRejected() >= junk.size());
  // Leading / trailing blanks and CR are fine.
  CHECK(rig.shell("  arm \r").find("arm: ok") != std::string::npos);
}

TEST(shell_reports_help_status_stats_and_stall) {
  Rig rig;
  CHECK(rig.toDisarmed());
  rig.runMs(DEBUG_STATS_INTERVAL_MS + kCtrlMs);
  const std::string help = rig.shell("help");
  for (const char* c : {"status", "stats", "stall?", "log", "arm", "disarm", "fire", "fly", "mode", "mag",
                        "pan", "tilt", "home", "sethome", "stop", "cal"})
    CHECK(help.find(c) != std::string::npos);
  const std::string status = rig.shell("status");
  CHECK(status.find("state=DISARMED") != std::string::npos);
  CHECK(status.find("ammo=" + std::to_string(MAG_CAPACITY) + "/" + std::to_string(MAG_CAPACITY)) != std::string::npos);
  CHECK(status.find("link=USB conn=1") != std::string::npos);
  CHECK(status.find("cal=1") != std::string::npos);
  const std::string stats = rig.shell("stats");
  CHECK(stats.find("stats loop=") != std::string::npos);
  CHECK(stats.find("totals frames=") != std::string::npos);
  const std::string stall = rig.shell("stall?");
  CHECK(stall.find("stall? on=" + std::string(STALL_DETECT_ENABLED ? "1" : "0")) != std::string::npos);
  CHECK(stall.find(" s=" + std::to_string(kA2Baseline)) != std::string::npos);
  // Every reply line fits DEBUG_LOG_LINE_MAX (nothing was cut mid-word by the builder).
  for (const std::string& l : rig.lines) CHECK(l.size() <= (size_t)DEBUG_LOG_LINE_MAX);
}

// Level 3: the stall trace at STALL_LOG_HZ (spec 3.6: how the threshold gets tuned); level 2:
// the stats line every DEBUG_STATS_INTERVAL_MS; level 1: neither.
TEST(log_levels_gate_the_stats_line_and_the_stall_trace) {
  Rig rig;
  CHECK(rig.toDisarmed());
  CHECK(rig.shell("log 3").find("log: level 3") != std::string::npos);
  rig.pad.rx = kStickMax;
  size_t from = rig.lines.size();
  rig.runMs(MS_PER_S);
  const size_t traces = rig.countLines(" stall s=", from);
  CHECK(traces >= STALL_LOG_HZ - 2 && traces <= STALL_LOG_HZ + 1);
  CHECK(rig.lastLine(" stall s=").find(" mv1") != std::string::npos);
  CHECK(rig.countLines(" stats loop=", from) >= 1u);
  rig.shell("log 1");
  from = rig.lines.size();
  rig.runMs(2 * DEBUG_STATS_INTERVAL_MS);
  CHECK_EQ(rig.countLines(" stall s=", from), (size_t)0);
  CHECK_EQ(rig.countLines(" stats loop=", from), (size_t)0);
  rig.shell("log 2");
  from = rig.lines.size();
  rig.runMs(3 * DEBUG_STATS_INTERVAL_MS + kCtrlMs);
  CHECK(rig.countLines(" stats loop=", from) >= 3u);
  CHECK_EQ(rig.countLines(" stall s=", from), (size_t)0);
}

// =====================================================================================================
// SERVO ADJUST (TurretState.h): the owner sets the trigger servo's REST and FIRE angles with the pad,
// the turret records the angle it commands and logs ready-to-paste config.h lines
// =====================================================================================================
namespace {

int32_t servoNow() { return (int32_t)std::lround(servoLogical()); }
bool teaching(const Rig& rig) { return rig.app->turret().servoTeach(); }
int32_t clampTeach(int32_t deg) { return std::min<int32_t>(std::max<int32_t>(deg, SERVO_ADJUST_MIN_DEG), SERVO_ADJUST_MAX_DEG); }
bool toTeach(Rig& rig) {
  if (!rig.toDisarmed()) return false;
  rig.press(DS_BTN_START);
  return teaching(rig) && rig.state() == DS_STATE_DISARMED;
}
std::string restLine(int32_t deg) {
  return "servo adjust: REST recorded -> #define TRIGGER_REST_ANGLE " + std::to_string(deg);
}
std::string fireLine(int32_t deg) {
  return "servo adjust: FIRE recorded -> #define TRIGGER_FIRE_ANGLE " + std::to_string(deg);
}
std::string summaryLine(int32_t rest, int32_t fire) {
  return "servo adjust: done; config.h lines: #define TRIGGER_REST_ANGLE " + std::to_string(rest) +
         " / #define TRIGGER_FIRE_ANGLE " + std::to_string(fire);
}
std::string restFire(int32_t rest, int32_t fire) {
  return "REST " + std::to_string(rest) + " FIRE " + std::to_string(fire);
}
std::string unchangedLine(int32_t rest, int32_t fire) {
  return "servo adjust: " + restFire(rest, fire) + " unchanged, nothing written";
}
// The saved line ends with the time the save took, which the test does not pin down.
std::string savedPrefix(int32_t rest, int32_t fire) {
  return "servo adjust: saved " + restFire(rest, fire) + " (survives power off), ";
}
// Lines from `from` whose text after "<uptime> " is exactly `text` (so a cut line never counts).
size_t countExact(const Rig& rig, const std::string& text, size_t from = 0) {
  size_t n = 0;
  for (size_t k = from; k < rig.lines.size(); k++) {
    const std::string& l = rig.lines[k];
    const size_t sp = l.find(' ');
    if (sp == std::string::npos || sp == 0) continue;
    bool digits = true;
    for (size_t i = 0; i < sp; i++) digits = digits && l[i] >= '0' && l[i] <= '9';
    n += digits && l.compare(sp + 1, std::string::npos, text) == 0;
  }
  return n;
}
std::string teachRow1(const std::string& rest, const std::string& fire) {
  return padRow(std::string(LCD_TXT_SERVO_REST) + rest + " " + LCD_TXT_SERVO_FIRE + fire, "");
}
bool found(const std::string& haystack, const std::string& needle) { return haystack.find(needle) != std::string::npos; }

}  // namespace

// Menu (DS_BTN_START; owner, 2026-11-03, it was Y) enters adjust mode only from DISARMED (refused
// and logged in CALIBRATING and ARMED), the state stays DISARMED everywhere it is reported, the
// servo starts at the configured REST, and Menu again leaves it with the summary.
TEST(servo_adjust_enters_only_from_disarmed_on_menu) {
  Rig rig;
  rig.runMs(MS_PER_S);
  CHECK_EQ(rig.state(), (uint8_t)DS_STATE_CALIBRATING);
  size_t from = rig.lines.size();
  rig.press(DS_BTN_START);
  rig.flush();
  CHECK(!teaching(rig));
  CHECK_EQ(rig.countLines("Menu (servo adjust) while calibrating: refused: not in this state", from), (size_t)1);
  CHECK(rig.toArmed());
  from = rig.lines.size();
  rig.press(DS_BTN_START);                                    // never from ARMED
  rig.flush();
  CHECK(!teaching(rig));
  CHECK_EQ(rig.state(), (uint8_t)DS_STATE_ARMED);
  CHECK_EQ(rig.countLines("Menu (servo adjust): refused: not in this state", from), (size_t)1);
  CHECK(found(rig.shell("servo adjust"), "servo adjust: refused: not in this state"));
  CHECK(!teaching(rig));
  rig.press(DS_BTN_A);
  CHECK_EQ(rig.state(), (uint8_t)DS_STATE_DISARMED);
  from = rig.lines.size();
  rig.press(DS_BTN_START);
  CHECK(teaching(rig));
  CHECK_EQ(rig.state(), (uint8_t)DS_STATE_DISARMED);     // a mode of DISARMED, not a state
  CHECK_EQ(servoNow(), clampTeach(TRIGGER_REST_ANGLE));   // starts at the configured REST
  rig.runMs(STATUS_HEARTBEAT_MS + LCD_REFRESH_MS * 4);
  CHECK_EQ(rig.lastStatus().state, (uint8_t)DS_STATE_DISARMED);
  CHECK(!rig.telemetry.empty() && rig.telemetry.back().state == DS_STATE_DISARMED);
  CHECK_EQ(rig.telemetry.back().servoDeg, (uint8_t)clampTeach(TRIGGER_REST_ANGLE));
  CHECK_EQ(rig.lcd(0), padRow(LCD_TXT_SERVO_ADJUST, std::to_string(clampTeach(TRIGGER_REST_ANGLE))));
  CHECK_EQ(rig.lcd(1), teachRow1(std::to_string(TRIGGER_REST_ANGLE), std::to_string(TRIGGER_FIRE_ANGLE)));
  CHECK(allLeds(scaled(LED_COLOR_SERVO_ADJUST)));
  rig.flush();
  CHECK_EQ(rig.countLines("servo adjust: on (Menu) at " + std::to_string(clampTeach(TRIGGER_REST_ANGLE)) + " deg", from),
           (size_t)1);
  // Menu again leaves it, with the summary; nothing changed, so nothing is written.
  from = rig.lines.size();
  rig.press(DS_BTN_START);
  rig.flush();
  CHECK(!teaching(rig));
  CHECK_EQ(countExact(rig, summaryLine(TRIGGER_REST_ANGLE, TRIGGER_FIRE_ANGLE), from), (size_t)1);
  CHECK_EQ(countExact(rig, unchangedLine(TRIGGER_REST_ANGLE, TRIGGER_FIRE_ANGLE), from), (size_t)1);
  CHECK_EQ(hal_host::storeSaves(), 0u);
  CHECK_EQ(servoNow(), (int32_t)TRIGGER_REST_ANGLE);
  rig.runMs(LCD_REFRESH_MS * 4);
  CHECK_EQ(rig.lcd(0), padRow(LCD_TXT_DISARMED, ammoText(MAG_CAPACITY)));
  CHECK(allLeds(scaled(LED_COLOR_DISARMED)));
}

// d-pad up / down and RB / LB move by the configured steps, one step per press (holding never
// repeats), clamped to SERVO_ADJUST_MIN_DEG..MAX; the LCD and the telemetry show the angle.
TEST(servo_adjust_nudges_by_the_configured_steps_clamp_and_never_repeat) {
  Rig rig;
  CHECK(toTeach(rig));
  const int32_t start = servoNow();
  int32_t want = start;
  rig.press(DS_BTN_DPAD_UP);
  CHECK_EQ(servoNow(), want = clampTeach(want + SERVO_ADJUST_STEP_SMALL_DEG));
  rig.press(DS_BTN_RB);
  CHECK_EQ(servoNow(), want = clampTeach(want + SERVO_ADJUST_STEP_LARGE_DEG));
  rig.press(DS_BTN_DPAD_DOWN);
  CHECK_EQ(servoNow(), want = clampTeach(want - SERVO_ADJUST_STEP_SMALL_DEG));
  rig.press(DS_BTN_LB);
  CHECK_EQ(servoNow(), want = clampTeach(want - SERVO_ADJUST_STEP_LARGE_DEG));
  // Held for a whole second: still one step.
  rig.pad.buttons = DS_BTN_DPAD_UP;
  rig.runMs(MS_PER_S);
  rig.pad.buttons = 0;
  rig.runMs(BUTTON_DEBOUNCE_MS + kCtrlMs * 2);
  CHECK_EQ(servoNow(), want = clampTeach(want + SERVO_ADJUST_STEP_SMALL_DEG));
  rig.pad.buttons = DS_BTN_RB;
  rig.runMs(MS_PER_S);
  rig.pad.buttons = 0;
  rig.runMs(BUTTON_DEBOUNCE_MS + kCtrlMs * 2);
  CHECK_EQ(servoNow(), want = clampTeach(want + SERVO_ADJUST_STEP_LARGE_DEG));
  // The bottom of the range: more presses than it takes, then it stays there.
  const int32_t downs = (servoNow() - SERVO_ADJUST_MIN_DEG) / SERVO_ADJUST_STEP_LARGE_DEG + 2;
  for (int32_t k = 0; k < downs; k++) rig.press(DS_BTN_LB);
  CHECK_EQ(servoNow(), (int32_t)SERVO_ADJUST_MIN_DEG);
  rig.press(DS_BTN_DPAD_DOWN);
  CHECK_EQ(servoNow(), (int32_t)SERVO_ADJUST_MIN_DEG);
  rig.flush();
  CHECK(found(rig.lastLine("servo adjust: "), "(limit)"));
  // The top (the shell's absolute angle gets near it), and past it.
  CHECK(found(rig.shell("servo " + std::to_string(SERVO_ADJUST_MAX_DEG - SERVO_ADJUST_STEP_SMALL_DEG)), ": ok"));
  rig.press(DS_BTN_RB);
  CHECK_EQ(servoNow(), (int32_t)SERVO_ADJUST_MAX_DEG);
  rig.press(DS_BTN_DPAD_UP);
  CHECK_EQ(servoNow(), (int32_t)SERVO_ADJUST_MAX_DEG);
  CHECK(found(rig.shell("servo " + std::to_string(SERVO_ADJUST_MAX_DEG + 1)), "refused: outside the limits"));
  CHECK(found(rig.shell("servo " + std::to_string(SERVO_ADJUST_MIN_DEG - 1)), "refused: outside the limits"));
  CHECK_EQ(servoNow(), (int32_t)SERVO_ADJUST_MAX_DEG);
  // The LCD and the telemetry follow the commanded angle.
  rig.runMs(LCD_REFRESH_MS * 4);
  CHECK_EQ(rig.lcd(0), padRow(LCD_TXT_SERVO_ADJUST, std::to_string(SERVO_ADJUST_MAX_DEG)));
  CHECK_EQ(rig.telemetry.back().servoDeg, (uint8_t)SERVO_ADJUST_MAX_DEG);
  CHECK(teaching(rig));
}

// A records the commanded angle as REST, B as FIRE, each with its ready-to-paste line (printed
// whatever the log level); the LCD and `status` show them; leaving prints the summary, whole.
TEST(servo_adjust_records_rest_and_fire_and_prints_the_paste_lines) {
  Rig rig;
  CHECK(toTeach(rig));
  for (int k = 0; k < 3; k++) rig.press(DS_BTN_DPAD_UP);
  const int32_t rest = servoNow();
  CHECK_EQ(rest, clampTeach(clampTeach(TRIGGER_REST_ANGLE) + 3 * SERVO_ADJUST_STEP_SMALL_DEG));
  size_t from = rig.lines.size();
  rig.press(DS_BTN_A);
  rig.flush();
  CHECK_EQ(countExact(rig, restLine(rest), from), (size_t)1);
  CHECK_EQ(rig.state(), (uint8_t)DS_STATE_DISARMED);     // A recorded; it did not arm
  rig.runMs(LCD_REFRESH_MS * 4);
  CHECK_EQ(rig.lcd(1), teachRow1(std::to_string(rest), std::to_string(TRIGGER_FIRE_ANGLE)));
  for (int k = 0; k < 4; k++) rig.press(DS_BTN_RB);
  const int32_t fire = servoNow();
  from = rig.lines.size();
  rig.press(DS_BTN_B);
  rig.flush();
  CHECK_EQ(countExact(rig, fireLine(fire), from), (size_t)1);
  CHECK_EQ(rig.trigger().fireMode(), (uint8_t)DS_FIRE_MODE_SEMI);   // B recorded; no fire mode change
  CHECK_EQ(rig.trigger().restDeg(), (float)rest);
  CHECK_EQ(rig.trigger().fireDeg(), (float)fire);
  rig.runMs(LCD_REFRESH_MS * 4);
  CHECK_EQ(rig.lcd(1), teachRow1(std::to_string(rest), std::to_string(fire)));
  CHECK(found(rig.shell("status"), "status servo=" + std::to_string(fire) + " adjust=1 rest=" + std::to_string(rest) +
                                       " fire=" + std::to_string(fire) + " (not saved)"));
  // With logging off the paste lines still come; the nudge line (LOG_STATE) does not.
  CHECK(found(rig.shell("log 0"), "log: level 0"));
  from = rig.lines.size();
  CHECK(found(rig.shell("servo " + std::to_string(rest)), "servo " + std::to_string(rest) + ": ok"));
  rig.press(DS_BTN_A);
  rig.flush();
  CHECK_EQ(countExact(rig, restLine(rest), from), (size_t)1);
  CHECK_EQ(rig.countLines("servo adjust: " + std::to_string(rest) + " deg", from), (size_t)0);
  from = rig.lines.size();
  rig.press(DS_BTN_START);
  rig.flush();
  CHECK_EQ(countExact(rig, summaryLine(rest, fire), from), (size_t)1);   // whole, on one line
  CHECK_EQ(rig.countLines(savedPrefix(rest, fire), from), (size_t)1);    // and saved, log 0 or not
  CHECK(found(rig.shell("log 2"), "log: level 2"));
  for (const std::string& l : rig.lines) CHECK(l.size() <= (size_t)DEBUG_LOG_LINE_MAX);
  // Adjust mode is over; the recorded values stay in force, now saved.
  CHECK(!teaching(rig));
  CHECK(found(rig.shell("status"), "adjust=0 rest=" + std::to_string(rest) + " fire=" + std::to_string(fire) + " (saved)"));
  CHECK_EQ(servoNow(), rest);
  CHECK_EQ(rig.trigger().restDeg(), (float)rest);
  CHECK_EQ(rig.trigger().fireDeg(), (float)fire);
}

// Teach nudges toward the side of the configured REST that has room: config.h's REST may sit at
// either end of the teach range (the owner's racked REST is 180 with TRIGGER_SERVO_REVERSED).
const bool kRoomUp = (SERVO_ADJUST_MAX_DEG - TRIGGER_REST_ANGLE) >= (TRIGGER_REST_ANGLE - SERVO_ADJUST_MIN_DEG);
const uint16_t kAwaySmall = kRoomUp ? DS_BTN_DPAD_UP : DS_BTN_DPAD_DOWN;
const uint16_t kAwayLarge = kRoomUp ? DS_BTN_RB : DS_BTN_LB;
const uint16_t kBackSmall = kRoomUp ? DS_BTN_DPAD_DOWN : DS_BTN_DPAD_UP;

// X: one test stroke -> FIRE -> REST with the angles in force (config.h's at once, the recorded
// ones after A / B), the same sweeps (from wherever the servo is) and holds as a real stroke, LEDs
// cyan throughout (SERVO ADJUST wins; the owner dropped the FIRING red on 2026-11-03), no ammo
// change, no shot counted, no rumble; while it runs, records, View and nudges are refused.
TEST(servo_adjust_test_stroke_uses_the_recorded_angles_and_timing_and_costs_nothing) {
  Rig rig;
  CHECK(toTeach(rig));
  size_t from = rig.lines.size();
  rig.pad.buttons = DS_BTN_X;                     // nothing recorded: config.h's angles in force
  CHECK(rig.runUntil([&] { return rig.trigger().testStrokeActive(); }, kCtrlMs * 3));
  rig.pad.buttons = 0;
  bool reachedFire = false;
  rig.everyLoop = [&] { reachedFire = reachedFire || servoNow() == (int32_t)TRIGGER_FIRE_ANGLE; };
  CHECK(rig.runUntil([&] { return !rig.trigger().strokeActive(); }, kStrokeMs * 2));
  rig.everyLoop = nullptr;
  CHECK(reachedFire);
  CHECK_EQ(servoNow(), (int32_t)TRIGGER_REST_ANGLE);
  rig.runMs(BUTTON_DEBOUNCE_MS + kCtrlMs * 2);
  rig.flush();
  CHECK_EQ(rig.countLines("servo adjust: test stroke REST " + std::to_string(TRIGGER_REST_ANGLE) + " -> FIRE " +
                              std::to_string(TRIGGER_FIRE_ANGLE), from), (size_t)1);
  rig.press(kAwaySmall);
  rig.press(DS_BTN_A);
  const int32_t rest = servoNow();
  for (int k = 0; k < 3; k++) rig.press(kAwayLarge);
  rig.press(DS_BTN_B);
  const int32_t fire = servoNow();
  CHECK(rest != fire);
  rig.press(kBackSmall);                          // somewhere else: the stroke starts anyway
  const uint8_t ammo = rig.trigger().ammo();
  const uint16_t shots = rig.trigger().shotsTotal();
  rig.runMs(STATUS_HEARTBEAT_MS);
  const uint8_t rumbleSeq = rig.lastStatus().rumbleSeq;
  const size_t statuses0 = rig.statuses.size();
  // Each phase's first ms, and every commanded angle, as the loops see them.
  const float startDeg = rig.trigger().servoDeg();   // the sweep starts at the nudged angle
  int64_t startAt = -1, holdFireAt = -1, sweepBackAt = -1, holdRestAt = -1, doneAt = -1;
  bool fireExact = true, restExact = true, monotonic = true, inside = true, cyan = false, notCyanMidStroke = false;
  float last = startDeg;
  Trigger::Phase was = Trigger::Phase::IDLE;
  const float lo = std::min({startDeg, (float)rest, (float)fire}), hi = std::max({startDeg, (float)rest, (float)fire});
  rig.everyLoop = [&] {
    const Trigger::Phase p = rig.trigger().phase();
    const float d = rig.trigger().servoDeg();
    const int64_t now = rig.ms();
    if (p != was) {
      if (p == kFirstPhase && startAt < 0) startAt = now;
      if (p == Trigger::Phase::HOLD_FIRE && holdFireAt < 0) holdFireAt = now;
      if (p == Trigger::Phase::SWEEP_BACK && sweepBackAt < 0) sweepBackAt = now;
      if (p == Trigger::Phase::HOLD_REST && holdRestAt < 0) holdRestAt = now;
      if (p == Trigger::Phase::IDLE && startAt >= 0 && doneAt < 0) doneAt = now;
    }
    if (p == Trigger::Phase::HOLD_FIRE) fireExact = fireExact && d == (float)fire;
    if (p == Trigger::Phase::HOLD_REST) restExact = restExact && d == (float)rest;
    if (p == Trigger::Phase::SWEEP_OUT) monotonic = monotonic && std::fabs(d - (float)fire) <= std::fabs(last - (float)fire);
    if (p == Trigger::Phase::SWEEP_BACK) monotonic = monotonic && std::fabs(d - (float)rest) <= std::fabs(last - (float)rest);
    if (startAt >= 0) inside = inside && d >= lo && d <= hi;
    last = d;
    was = p;
  };
  rig.onShow = [&] {
    if (!rig.trigger().testStrokeActive()) return;
    if (allLeds(scaled(LED_COLOR_SERVO_ADJUST))) cyan = true;
    else notCyanMidStroke = true;
  };
  rig.pad.buttons = DS_BTN_X;
  CHECK(rig.runUntil([&] { return rig.trigger().testStrokeActive(); }, kCtrlMs * 3));
  rig.pad.buttons = 0;
  rig.runMs(BUTTON_DEBOUNCE_MS + kCtrlMs * 2);
  // Mid stroke: A, B, View, a nudge and X again are refused (not ready).
  from = rig.lines.size();
  rig.pad.buttons = DS_BTN_A | DS_BTN_B | DS_BTN_BACK | kAwaySmall | DS_BTN_X;
  rig.runMs(kCtrlMs * 3);
  rig.pad.buttons = 0;
  CHECK(rig.runUntil([&] { return doneAt >= 0; }, kStrokeMs * 2));
  rig.everyLoop = nullptr;
  rig.onShow = nullptr;
  rig.flush();
  for (const char* what : {"A (record REST): refused: not ready", "B (record FIRE): refused: not ready",
                           "View (config.h angles): refused: not ready",
                           kRoomUp ? "d-pad up (servo +): refused: not ready" : "d-pad down (servo -): refused: not ready",
                           "X (test stroke): refused: not ready"})
    CHECK_EQ(rig.countLines(what, from), (size_t)1);
  CHECK(startAt >= 0 && holdFireAt >= 0 && sweepBackAt >= 0 && holdRestAt >= 0 && doneAt >= 0);
  // The X press lands anywhere inside a ms, the later phases on its first loop: the first gap
  // may read one ms short, the others are exact.
  const int64_t out = sweepMsFor(std::fabs((float)fire - startDeg));
  CHECK(holdFireAt - startAt >= out - 1 && holdFireAt - startAt <= out);
  CHECK_EQ(sweepBackAt - holdFireAt, (int64_t)TRIGGER_FORWARD_MS);
  CHECK_EQ(holdRestAt - sweepBackAt, (int64_t)sweepMsFor(std::fabs((float)rest - (float)fire)));
  CHECK_EQ(doneAt - holdRestAt, (int64_t)TRIGGER_RETURN_MS);
  CHECK(fireExact && restExact);                        // landed exactly, held
  CHECK(monotonic && inside);                           // glided, never past an end
  CHECK(cyan && !notCyanMidStroke);                     // cyan for the whole test stroke
  CHECK_EQ(rig.trigger().ammo(), ammo);                 // no dart counted
  CHECK_EQ(rig.trigger().shotsTotal(), shots);
  CHECK_EQ(rig.trigger().restDeg(), (float)rest);       // the records were not touched mid stroke
  CHECK_EQ(rig.trigger().fireDeg(), (float)fire);
  rig.runMs(STATUS_HEARTBEAT_MS * 2);
  CHECK(rig.statuses.size() > statuses0);
  for (size_t k = statuses0; k < rig.statuses.size(); k++) CHECK_EQ(rig.statuses[k].rumbleSeq, rumbleSeq);   // no rumble
  CHECK_EQ(servoNow(), rest);                           // ends at REST, still in adjust mode
  CHECK(teaching(rig));
  rig.runMs(LCD_REFRESH_MS * 4);
  CHECK(allLeds(scaled(LED_COLOR_SERVO_ADJUST)));
  CHECK_EQ(rig.lcd(0), padRow(LCD_TXT_SERVO_ADJUST, std::to_string(rest)));
}

// KEY SAFETY TEST: the flywheels never turn in adjust mode, also when DISARMED may spin them
// (FLYWHEEL_REQUIRES_ARMED false), with LB held into and through it, LB pressed (toggle mode)
// or the shell's `fly on`; nothing fires either.
TEST(servo_adjust_never_spins_the_flywheels_whatever_lb_or_the_flags_say) {
  struct Case {
    bool requiresArmed;
    bool holdMode;
  };
  for (const Case c : {Case{true, true}, Case{false, true}, Case{false, false}}) {
    App::Config cfg = App::defaults();
    cfg.turret.flywheelRequiresArmed = c.requiresArmed;
    cfg.turret.flywheelHoldMode = c.holdMode;
    Rig rig(cfg);
    CHECK(rig.toDisarmed());
    if (!c.requiresArmed) {   // DISARMED may spin them here: have them spinning first
      if (c.holdMode)
        rig.pad.buttons = DS_BTN_LB;
      else
        rig.press(DS_BTN_LB);
      rig.runMs(FLYWHEEL_RAMP_MS);
      CHECK(rig.app->flywheel().on());
    }
    bool spun = false;
    uint32_t teachLoops = 0;
    rig.everyLoop = [&] {
      if (!teaching(rig)) return;
      teachLoops++;
      spun = spun || rig.app->flywheel().on() || rig.app->flywheel().duty() > 0 ||
             (hal_host::pwmDuty(PIN_FLY_ENA) > 0 && !rig.app->flywheel().braking());
    };
    rig.press(DS_BTN_START);                 // into adjust mode (hold mode: LB still held)
    CHECK(teaching(rig));
    rig.runMs(FLYWHEEL_SPINUP_MS + FLYWHEEL_RAMP_MS);
    rig.pad.buttons = 0;
    rig.runMs(BUTTON_DEBOUNCE_MS + kCtrlMs * 2);
    for (int k = 0; k < 3; k++) rig.press(DS_BTN_LB);   // fresh presses (the toggle latch)
    rig.pad.buttons = DS_BTN_LB;                          // and held again
    rig.runMs(FLYWHEEL_RAMP_MS);
    rig.pad.buttons = 0;
    rig.runMs(BUTTON_DEBOUNCE_MS + kCtrlMs * 2);
    CHECK(found(rig.shell("fly on"), "fly on: refused: in servo adjust"));
    rig.pad.rt = kPulled;
    rig.runMs(kStrokeMs * 2);
    CHECK(found(rig.shell("fire"), "fire: refused"));
    rig.pad.rt = 0;
    rig.runMs(kCtrlMs * 3);
    CHECK(teaching(rig));
    rig.everyLoop = nullptr;
    CHECK(teachLoops > 0u);
    CHECK(!spun);
    CHECK_EQ(hal_host::pwmDuty(PIN_FLY_ENA), (uint8_t)0);
    CHECK_EQ(rig.trigger().shotsTotal(), (uint16_t)0);
    CHECK(!rig.app->turret().status().flywheelAllowed);
  }
}

// KEY SAFETY TEST: in adjust mode A does not arm, B does not toggle the fire mode, X does not
// reset the magazine, d-pad left / down do not set / go home, RT does not fire, and the shell
// refuses the same actions; the stick (and shell pan / tilt) still aim.
TEST(servo_adjust_a_b_x_rt_and_the_d_pad_do_not_do_their_normal_actions) {
  Rig rig;
  CHECK(rig.toArmedSpunUp());
  rig.pad.rt = kPulled;
  rig.runMs(kStrokeMs + kCtrlMs * 3);
  rig.pad = Pad();
  rig.runMs(kCtrlMs * 3);
  CHECK_EQ(rig.trigger().ammo(), (uint8_t)(MAG_CAPACITY - 1));   // so X's reset would show
  rig.press(DS_BTN_A);
  CHECK_EQ(rig.state(), (uint8_t)DS_STATE_DISARMED);
  rig.pad.rx = -kStickMax;                                        // a home away from here
  rig.runMs(MS_PER_S / 2);
  rig.pad.rx = 0;
  rig.runMs(MS_PER_S / 2);
  rig.press(DS_BTN_DPAD_LEFT);
  const float homePan = rig.motion().homeDeg(AXIS_PAN);
  rig.pad.rx = kStickMax;
  rig.runMs(MS_PER_S / 2);
  rig.pad.rx = 0;
  rig.runMs(MS_PER_S / 2);
  const float panHere = rig.motion().positionDeg(AXIS_PAN);
  CHECK(std::fabs(panHere - homePan) > 5.0f);
  rig.press(DS_BTN_START);
  CHECK(teaching(rig));
  rig.flush();
  const size_t armings = rig.countLines("state ARMED");
  rig.press(DS_BTN_A);
  CHECK_EQ(rig.state(), (uint8_t)DS_STATE_DISARMED);
  rig.press(DS_BTN_B);
  CHECK_EQ(rig.trigger().fireMode(), (uint8_t)DS_FIRE_MODE_SEMI);
  rig.press(DS_BTN_X);                                            // FIRE is recorded: a test stroke
  rig.runMs(kStrokeMs * 2);
  CHECK_EQ(rig.trigger().ammo(), (uint8_t)(MAG_CAPACITY - 1));
  rig.press(DS_BTN_DPAD_LEFT);
  CHECK_NEAR(rig.motion().homeDeg(AXIS_PAN), homePan, 0.001f);
  rig.press(DS_BTN_DPAD_DOWN);
  CHECK(!rig.motion().homing());
  rig.runMs(MS_PER_S / 2);
  CHECK_NEAR(rig.motion().positionDeg(AXIS_PAN), panHere, 1.0f / PAN_STEPS_PER_DEG);
  rig.pad.rt = kPulled;
  rig.runMs(kStrokeMs * 2);
  rig.pad.rt = 0;
  rig.runMs(kCtrlMs * 3);
  CHECK_EQ(rig.trigger().shotsTotal(), (uint16_t)1);              // only the shot before adjust mode
  // The stick still aims.
  rig.pad.rx = kStickMax;
  rig.runMs(MS_PER_S / 2);
  CHECK(rig.motion().velocityDeg(AXIS_PAN) < 0.0f);
  rig.pad.rx = 0;
  rig.runMs(MS_PER_S / 2);
  // The shell: the same actions refused, aiming and stopping allowed.
  for (const std::string cmd : {"arm", "fire", "fly on", "mode full", "mag", "home", "sethome"})
    CHECK(found(rig.shell(cmd), cmd + ": refused: in servo adjust"));
  CHECK(found(rig.shell("pan 10"), "pan 10.0: ok"));
  CHECK(found(rig.shell("stop"), "stop: ok"));
  CHECK(found(rig.shell("disarm"), "disarm: ok"));
  rig.flush();
  CHECK_EQ(rig.countLines("state ARMED"), armings);
  CHECK_EQ(rig.state(), (uint8_t)DS_STATE_DISARMED);
  CHECK_EQ(rig.trigger().fireMode(), (uint8_t)DS_FIRE_MODE_SEMI);
  CHECK_EQ(rig.trigger().ammo(), (uint8_t)(MAG_CAPACITY - 1));
  CHECK_NEAR(rig.motion().homeDeg(AXIS_PAN), homePan, 0.001f);
  CHECK(teaching(rig));
}

// KEY SAFETY TEST: every way out of adjust mode (link stops, connected = 0, mode toggle, power off,
// a stall fault, a state change by the shell, `servo off`) ends it, mid test stroke too: the servo
// back at the REST in force, no dart counted, the summary printed, the adjust display gone. Only
// `servo off` (like Menu) saves; every other exit says the angles stay for this run only.
TEST(servo_adjust_ends_on_link_loss_power_off_mode_toggle_fault_and_state_change) {
  enum Exit { LINK_STOPS, NOT_CONNECTED, MODE_TOGGLE, POWER_OFF, STALL_FAULT, SHELL_CAL, SHELL_OFF, EXITS };
  const char* const names[] = {"link stops", "not connected", "mode toggle", "power off", "stall fault",
                               "shell cal", "servo off"};
  // The test-stroke phase each exit is made in, so that between them they abort it in every phase
  // (each exit acts well inside its phase; the link timeout is too slow to pin to one).
  using P = Trigger::Phase;
  const P target[] = {P::HOLD_FIRE, P::HOLD_REST, P::HOLD_FIRE, P::HOLD_REST, P::HOLD_FIRE, P::SWEEP_OUT, P::SWEEP_BACK};
  for (int e = LINK_STOPS; e < EXITS; e++) {
    std::printf("      case %s\n", names[e]);
    Rig rig(stallOn());
    CHECK(toTeach(rig));
    rig.press(kAwaySmall);
    rig.press(DS_BTN_A);                    // REST recorded one step up
    const int32_t rest = servoNow();
    rig.press(kAwayLarge);
    rig.press(DS_BTN_B);                    // FIRE recorded
    const int32_t fire = servoNow();
    rig.press(kAwayLarge);                   // and the servo off both of them
    CHECK(servoNow() != rest);
    if (e == STALL_FAULT) {                 // a stall needs a moving axis
      rig.pad.rx = kStickMax;
      rig.runMs(MS_PER_S / 2);
    }
    const uint8_t ammo = rig.trigger().ammo();
    const uint32_t out = sweepMsFor(std::fabs((float)servoNow() - (float)fire));
    const uint32_t back = sweepMsFor(std::fabs((float)fire - (float)rest));
    rig.flush();                            // before the stroke: flushing runs the clock on
    const size_t from = rig.lines.size();
    rig.pad.buttons = DS_BTN_X;             // the test stroke runs when the exit comes
    CHECK(rig.runUntil([&] { return rig.trigger().testStrokeActive(); }, kCtrlMs * 3));
    rig.pad.buttons = 0;
    CHECK(rig.runUntil([&] { return rig.trigger().phase() == target[e]; }, kStrokeMs));
    if (target[e] == P::SWEEP_OUT) rig.runMs(out / 2);    // mid sweep: off both ends
    if (target[e] == P::SWEEP_BACK) rig.runMs(back / 2);
    if (target[e] == P::HOLD_FIRE) CHECK_EQ(servoNow(), fire);
    if (target[e] == P::SWEEP_OUT || target[e] == P::SWEEP_BACK) CHECK(servoNow() != fire && servoNow() != rest);
    CHECK_EQ(rig.trigger().phase(), target[e]);
    // The phase the test stroke was in the loop before adjust mode ended (IDLE: it had completed).
    bool wasTeach = true;
    P wasPhase = rig.trigger().phase(), exitPhase = P::IDLE;
    rig.everyLoop = [&] {
      const bool t = teaching(rig);
      if (wasTeach && !t) exitPhase = wasPhase;
      wasTeach = t;
      wasPhase = rig.trigger().phase();
    };
    switch (e) {
      case LINK_STOPS: rig.pad.send = false; break;
      case NOT_CONNECTED: rig.pad.connected = false; break;
      case MODE_TOGGLE: rig.setWifi(true); break;
      case POWER_OFF: rig.setPower(false); break;
      case STALL_FAULT:
        hal_host::setAnalog(PIN_STALL_SENSE, (uint16_t)(kA2Baseline + STALL_DELTA_COUNTS * 2));
        break;
      case SHELL_CAL: hal_host::usbInject(std::string("cal\n")); break;
      case SHELL_OFF: hal_host::usbInject(std::string("servo off\n")); break;
    }
    CHECK(rig.runUntil([&] { return !teaching(rig); }, LINK_TIMEOUT_MS * 2));
    rig.everyLoop();   // once more: runUntil stops right after the loop that ended it
    rig.everyLoop = nullptr;
    hal_host::setAnalog(PIN_STALL_SENSE, kA2Baseline);
    rig.pad.rx = 0;
    CHECK(!rig.trigger().strokeActive());
    if (e == LINK_STOPS)
      CHECK(exitPhase != P::IDLE);           // still running when the link timed out
    else
      CHECK_EQ(exitPhase, target[e]);        // aborted in the phase it was made in
    CHECK_EQ(servoNow(), e == POWER_OFF ? (int32_t)TRIGGER_REST_ANGLE : rest);   // power off: a new boot
    CHECK_EQ(rig.trigger().ammo(), ammo);
    CHECK_EQ(rig.trigger().shotsTotal(), (uint16_t)0);
    rig.flush();
    CHECK_EQ(countExact(rig, summaryLine(rest, fire), from), (size_t)1);
    if (e == SHELL_OFF) {
      CHECK_EQ(hal_host::storeSaves(), 1u);
      CHECK_EQ(rig.countLines(savedPrefix(rest, fire), from), (size_t)1);
    } else {
      CHECK_EQ(hal_host::storeSaves(), 0u);
      CHECK_EQ(rig.countLines("servo adjust: not saved (", from), (size_t)1);
      CHECK_EQ(rig.countLines(restFire(rest, fire) + " stay for this run only", from), (size_t)1);
    }
    rig.runMs(LCD_REFRESH_MS * 4);
    CHECK(!startsWith(rig.lcd(0), LCD_TXT_SERVO_ADJUST));
    CHECK(!allLeds(scaled(LED_COLOR_SERVO_ADJUST)));
    switch (e) {
      case LINK_STOPS:
      case NOT_CONNECTED:
        CHECK_EQ(rig.state(), (uint8_t)DS_STATE_PAIRING);
        break;
      case POWER_OFF:
        CHECK_EQ(rig.state(), (uint8_t)DS_STATE_OFF);
        CHECK_EQ(rig.trigger().restDeg(), (float)TRIGGER_REST_ANGLE);   // the records are gone
        CHECK_EQ(rig.trigger().fireDeg(), (float)TRIGGER_FIRE_ANGLE);
        rig.setPower(true);
        CHECK(rig.toDisarmed());
        CHECK_EQ(servoNow(), (int32_t)TRIGGER_REST_ANGLE);
        CHECK(found(rig.shell("status"), "adjust=0 rest=" + std::to_string(TRIGGER_REST_ANGLE) + " fire=" +
                                             std::to_string(TRIGGER_FIRE_ANGLE) + " (config.h)"));
        break;
      case STALL_FAULT: {
        CHECK_EQ(rig.state(), (uint8_t)DS_STATE_FAULT);
        const size_t f = rig.lines.size();
        rig.press(DS_BTN_START);            // and Menu cannot start it in FAULT
        rig.flush();
        CHECK(!teaching(rig));
        CHECK_EQ(rig.countLines("Menu (servo adjust) in FAULT: refused", f), (size_t)1);
        break;
      }
      case SHELL_CAL:
        CHECK_EQ(rig.state(), (uint8_t)DS_STATE_CALIBRATING);
        break;
      case MODE_TOGGLE:                     // through PAIRING; USB still drives, so DISARMED again
        CHECK(rig.countLines("link lost: mode toggle", from) == 1u);
        CHECK(rig.countLines("state PAIRING", from) == 1u);
        break;
      default:
        CHECK_EQ(rig.state(), (uint8_t)DS_STATE_DISARMED);
        break;
    }
    if (e != POWER_OFF) {                   // the records stay for the rest of this power-on
      CHECK_EQ(rig.trigger().restDeg(), (float)rest);
      CHECK_EQ(rig.trigger().fireDeg(), (float)fire);
    }
  }
}

// The recorded angles, saved by Menu, drive every real stroke after adjust mode (and an abort rests
// at the recorded REST), and still do after the power toggle's reboot (OFF -> BOOT).
TEST(servo_adjust_recorded_angles_drive_real_strokes_until_power_off) {
  Rig rig;
  CHECK(toTeach(rig));
  for (int k = 0; k < 3; k++) rig.press(kAwaySmall);
  rig.press(DS_BTN_A);
  const int32_t rest = servoNow();
  for (int k = 0; k < 4; k++) rig.press(kAwayLarge);
  rig.press(DS_BTN_B);
  const int32_t fire = servoNow();
  CHECK(rest != (int32_t)TRIGGER_REST_ANGLE && fire != (int32_t)TRIGGER_FIRE_ANGLE);
  rig.press(DS_BTN_START);
  CHECK(!teaching(rig));
  CHECK_EQ(servoNow(), rest);
  // Armed and spun up, one pull: the recorded FIRE, back to the recorded REST, one dart.
  rig.press(DS_BTN_A);
  CHECK_EQ(rig.state(), (uint8_t)DS_STATE_ARMED);
  rig.pad.buttons = DS_BTN_LB;
  rig.runMs(FLYWHEEL_SPINUP_MS + kCtrlMs * 3);
  // The commanded angles swept between the two records, both ends reached, never past either.
  auto sweptBetween = [](const std::set<int32_t>& a, int32_t r, int32_t f) {
    return !a.empty() && *a.begin() == std::min(r, f) && *a.rbegin() == std::max(r, f) &&
           (a.size() > 2 || std::abs(r - f) <= 1);
  };
  std::set<int32_t> angles;
  rig.everyLoop = [&] { angles.insert(servoNow()); };
  rig.pad.rt = kPulled;
  rig.runMs(kStrokeMs * 2);
  rig.everyLoop = nullptr;
  CHECK(sweptBetween(angles, rest, fire));
  CHECK_EQ(rig.trigger().shotsTotal(), (uint16_t)1);
  CHECK_EQ(rig.trigger().ammo(), (uint8_t)(MAG_CAPACITY - 1));
  CHECK_EQ(servoNow(), rest);
  // A disarm mid stroke rests at the recorded REST too.
  rig.pad.rt = kReleased;
  rig.runMs(kCtrlMs * 3);
  rig.pad.rt = kPulled;
  CHECK(rig.runUntil([&] { return rig.trigger().phase() == Trigger::Phase::HOLD_FIRE; }, kStrokeMs + kCoolMs));
  CHECK_EQ(servoNow(), fire);
  rig.pad.buttons = (uint16_t)(rig.pad.buttons | DS_BTN_A);
  CHECK(rig.runUntil([&] { return rig.state() == DS_STATE_DISARMED; }, kCtrlMs * 3));
  CHECK_EQ(servoNow(), rest);
  CHECK_EQ(rig.trigger().shotsTotal(), (uint16_t)1);
  // Power off and on: a new boot, on the saved angles.
  rig.pad = Pad();
  rig.setPower(false);
  CHECK(rig.runUntil([&] { return rig.state() == DS_STATE_OFF; }, TOGGLE_DEBOUNCE_MS + kCtrlMs));
  CHECK_EQ(servoNow(), rest);
  rig.setPower(true);
  CHECK(rig.toArmedSpunUp());
  CHECK_EQ(servoNow(), rest);
  angles.clear();
  rig.everyLoop = [&] { angles.insert(servoNow()); };
  rig.pad.rt = kPulled;
  rig.runMs(kStrokeMs * 2);
  rig.everyLoop = nullptr;
  CHECK(sweptBetween(angles, rest, fire));
  CHECK_EQ(hal_host::storeSaves(), 1u);
}

// The shell equivalents: servo adjust|<deg>|rest|fire|stroke|off, only from DISARMED, through the
// same rules; `status` shows the teach state and the angles; malformed lines never run.
TEST(servo_adjust_shell_commands_follow_the_pad_rules) {
  Rig rig;
  rig.pad.send = false;
  CHECK(rig.runUntil([&] { return rig.state() == DS_STATE_PAIRING; }, 15 * MS_PER_S));
  CHECK(found(rig.shell("servo adjust"), "servo adjust: refused: not in this state"));
  rig.pad.send = true;
  rig.runMs(kCtrlMs * 3);
  CHECK_EQ(rig.state(), (uint8_t)DS_STATE_DISARMED);
  for (const std::string w : {"30", "rest", "fire", "stroke"})
    CHECK(found(rig.shell("servo " + w), "servo " + w + ": refused: not in servo adjust"));
  CHECK(found(rig.shell("servo off"), "servo off: ok"));   // nothing to leave
  CHECK(!teaching(rig));
  CHECK(found(rig.shell("servo adjust"), "servo adjust: ok"));
  CHECK(teaching(rig));
  const int32_t rest = clampTeach(TRIGGER_REST_ANGLE + 3);
  const int32_t fire = clampTeach(TRIGGER_FIRE_ANGLE - 2);
  CHECK(found(rig.shell("servo " + std::to_string(rest)), "servo " + std::to_string(rest) + ": ok"));
  CHECK_EQ(servoNow(), rest);
  size_t from = rig.lines.size();
  CHECK(found(rig.shell("servo rest"), "servo rest: ok"));
  rig.flush();
  CHECK_EQ(countExact(rig, restLine(rest), from), (size_t)1);
  CHECK(found(rig.shell("servo " + std::to_string(fire)), ": ok"));
  from = rig.lines.size();
  CHECK(found(rig.shell("servo fire"), "servo fire: ok"));
  rig.flush();
  CHECK_EQ(countExact(rig, fireLine(fire), from), (size_t)1);
  const uint8_t ammo = rig.trigger().ammo();
  CHECK(found(rig.shell("servo stroke"), "servo stroke: ok"));
  CHECK(rig.trigger().testStrokeActive());
  CHECK(found(rig.shell("servo rest"), "servo rest: refused: not ready"));   // mid stroke
  rig.runMs(kStrokeMs);
  CHECK(!rig.trigger().strokeActive());
  CHECK_EQ(servoNow(), rest);
  CHECK_EQ(rig.trigger().ammo(), ammo);
  CHECK(found(rig.shell("status"), "status servo=" + std::to_string(rest) + " adjust=1 rest=" + std::to_string(rest) +
                                       " fire=" + std::to_string(fire) + " (not saved)"));
  from = rig.lines.size();
  CHECK(found(rig.shell("servo off"), "servo off: ok"));   // saves, as Menu does
  rig.flush();
  CHECK(!teaching(rig));
  CHECK_EQ(countExact(rig, summaryLine(rest, fire), from), (size_t)1);
  CHECK_EQ(rig.countLines(savedPrefix(rest, fire), from), (size_t)1);
  CHECK(found(rig.shell("status"), "adjust=0 rest=" + std::to_string(rest) + " fire=" + std::to_string(fire) + " (saved)"));
  // Toggle on and off again with `servo adjust` and its old name `servo teach`; nothing changed,
  // nothing written.
  for (const std::string word : {"adjust", "teach"}) {
    CHECK(found(rig.shell("servo " + word), "servo " + word + ": ok"));
    CHECK(teaching(rig));
    from = rig.lines.size();
    CHECK(found(rig.shell("servo " + word), "servo " + word + ": ok"));
    CHECK(!teaching(rig));
    rig.flush();
    CHECK_EQ(countExact(rig, unchangedLine(rest, fire), from), (size_t)1);
  }
  CHECK_EQ(hal_host::storeSaves(), 1u);
  // `servo reset` is View: config.h's angles in force, outside adjust mode refused.
  CHECK(found(rig.shell("servo reset"), "servo reset: refused: not in servo adjust"));
  CHECK(found(rig.shell("servo adjust"), "servo adjust: ok"));
  CHECK(found(rig.shell("servo reset"), "servo reset: ok"));
  CHECK_EQ(servoNow(), clampTeach(TRIGGER_REST_ANGLE));
  CHECK_EQ(rig.trigger().restDeg(), (float)TRIGGER_REST_ANGLE);
  CHECK_EQ(rig.trigger().fireDeg(), (float)TRIGGER_FIRE_ANGLE);
  CHECK(found(rig.shell("servo off"), "servo off: ok"));
  CHECK_EQ(hal_host::storeSaves(), 2u);
  CHECK(found(rig.shell("status"), "fire=" + std::to_string(TRIGGER_FIRE_ANGLE) + " (saved)"));
  // Never from ARMED.
  CHECK(found(rig.shell("arm"), "arm: ok"));
  CHECK(found(rig.shell("servo adjust"), "servo adjust: refused: not in this state"));
  CHECK(found(rig.shell("servo " + std::to_string(rest)), "refused: not in servo adjust"));
  CHECK(!teaching(rig));
  CHECK_EQ(rig.state(), (uint8_t)DS_STATE_ARMED);
  // Malformed lines never run.
  for (const std::string j : {"servo", "servo 12.5", "servo up", "servo 1 2", "servo adjust now", "servo reset now",
                               "servos", "SERVO adjust", "servo adjusts"}) {
    const std::string r = rig.shell(j);
    CHECK(r.find(": ok") == std::string::npos);
    CHECK(found(r, "not run") || found(r, "unknown command"));
  }
  CHECK(found(rig.shell("help"), "servo adjust|<deg>|rest|fire|reset|stroke|off"));
}

// KEY SAFETY TEST: a long random session (every button, Menu included, sticks, trigger, link drops,
// mode and power toggles, stall spikes), with DISARMED allowed to spin the flywheels: whenever
// adjust mode is on the state is DISARMED, the flywheels are still, no real stroke runs and no
// dart is counted; the LED brightness clamp holds throughout.
TEST(servo_adjust_random_session_keeps_every_rule_and_the_brightness_clamp) {
  App::Config cfg = stallOn();
  cfg.turret.flywheelRequiresArmed = false;
  Rig rig(cfg);
  std::mt19937 rnd(20261104);
  std::uniform_int_distribution<int> stick(-JOY_AXIS_RANGE, JOY_AXIS_RANGE - 1);
  std::uniform_int_distribution<int> trig(0, TRIGGER_AXIS_RANGE);
  std::uniform_int_distribution<int> pct(0, 99);
  uint32_t entries = 0, teachLoops = 0, testStrokes = 0;
  bool was = false, bad = false, wasTest = false;
  uint8_t ammo0 = 0;
  uint16_t shots0 = 0;
  rig.everyLoop = [&] {
    const bool t = teaching(rig);
    if (t && !was) {
      entries++;
      ammo0 = rig.trigger().ammo();
      shots0 = rig.trigger().shotsTotal();
    }
    if (t) {
      teachLoops++;
      const bool test = rig.trigger().testStrokeActive();
      if (test && !wasTest) testStrokes++;
      wasTest = test;
      bad = bad || rig.state() != DS_STATE_DISARMED || rig.app->flywheel().on() || rig.app->flywheel().duty() > 0 ||
            rig.trigger().ammo() != ammo0 || rig.trigger().shotsTotal() != shots0 ||
            (rig.trigger().strokeActive() && !test) || rig.app->turret().triggerInput().armed;
    } else {
      bad = bad || rig.trigger().testStrokeActive();   // a test stroke never outlives adjust mode
      wasTest = false;
    }
    was = t;
  };
  for (int k = 0; k < 600; k++) {   // 600 x 100 ms = 60 s of virtual time
    const int r = pct(rnd);
    if (r < 1) rig.setPower(pct(rnd) < 70);
    else if (r < 2) rig.setWifi(pct(rnd) < 50);
    else if (r < 5) rig.pad.send = pct(rnd) < 80;
    else if (r < 7) hal_host::setAnalog(PIN_STALL_SENSE, (uint16_t)(pct(rnd) * ADC_MAX_COUNTS / 100));
    rig.pad.rx = (int16_t)stick(rnd);
    rig.pad.ry = (int16_t)stick(rnd);
    if (pct(rnd) < 40) rig.pad.rx = rig.pad.ry = 0;   // often still, so Menu can save and leave
    rig.pad.rt = (uint16_t)trig(rnd);
    rig.pad.buttons = (uint16_t)(rnd() & DS_BTN_ALL);
    rig.runMs(100);
    if (k % 50 == 49) rig.setPower(true), rig.pad.send = true, hal_host::setAnalog(PIN_STALL_SENSE, kA2Baseline);
  }
  rig.everyLoop = nullptr;
  std::printf("info servo adjust random session: %u entries, %u loops in adjust mode, %u test strokes\n", entries, teachLoops,
              testStrokes);
  CHECK(entries >= 10u);
  CHECK(testStrokes >= 1u);
  CHECK(!bad);
  CHECK(hal_host::ledMaxChannel() <= LED_MAX_BRIGHTNESS);
  CHECK_EQ(hal_host::ledOverLimit(), 0u);
  CHECK_EQ(rig.app->leds().overLimit(), 0u);
  for (const auto& t : rig.telemetry) {
    CHECK(t.ledR <= LED_MAX_BRIGHTNESS);
    CHECK(t.ledG <= LED_MAX_BRIGHTNESS);
    CHECK(t.ledB <= LED_MAX_BRIGHTNESS);
  }
  CHECK_EQ(blockingWrites(), 0u);
}

// The frame whose Menu switches adjust mode acts on nothing else, LB included: with DISARMED allowed
// to spin (toggle mode), a Menu + LB press that leaves adjust mode does not flip the flywheel latch
// on, and one that enters it leaves nothing behind; a later LB press toggles as usual.
TEST(servo_adjust_menu_with_lb_in_the_same_frame_never_flips_the_flywheel_latch) {
  App::Config cfg = App::defaults();
  cfg.turret.flywheelRequiresArmed = false;
  cfg.turret.flywheelHoldMode = false;
  Rig rig(cfg);
  CHECK(rig.toDisarmed());
  rig.press((uint16_t)(DS_BTN_START | DS_BTN_LB));   // enters
  CHECK(teaching(rig));
  CHECK(!rig.app->flywheel().on());
  rig.press((uint16_t)(DS_BTN_START | DS_BTN_LB));   // leaves
  CHECK(!teaching(rig));
  CHECK_EQ(rig.state(), (uint8_t)DS_STATE_DISARMED);
  rig.runMs(FLYWHEEL_RAMP_MS);
  CHECK(!rig.app->flywheel().on());
  CHECK_EQ(hal_host::pwmDuty(PIN_FLY_ENA), (uint8_t)0);
  rig.press(DS_BTN_LB);                          // DISARMED rules again: LB toggles the latch
  CHECK(rig.app->flywheel().on());
}

// Y is unused since Menu took SERVO ADJUST (owner, 2026-11-03): in CALIBRATING, DISARMED and ARMED
// a Y press does nothing at all, is never refused and never logged.
TEST(y_does_nothing_anywhere) {
  Rig rig;
  rig.runMs(MS_PER_S);
  CHECK_EQ(rig.state(), (uint8_t)DS_STATE_CALIBRATING);
  rig.flush();
  size_t from = rig.lines.size();
  uint32_t refusals = rig.app->turret().status().refusals;
  rig.press(DS_BTN_Y);
  rig.flush();
  CHECK_EQ(rig.app->turret().status().refusals, refusals);
  CHECK_EQ(rig.countLines("refused", from) + rig.countLines("servo adjust", from), (size_t)0);
  CHECK(rig.toDisarmed());
  for (uint8_t want : {DS_STATE_DISARMED, DS_STATE_ARMED}) {
    if (want == DS_STATE_ARMED) rig.press(DS_BTN_A);
    CHECK_EQ(rig.state(), want);
    rig.flush();
    from = rig.lines.size();
    refusals = rig.app->turret().status().refusals;
    const Trigger::Phase phase = rig.trigger().phase();
    const uint8_t mode = rig.trigger().fireMode();
    rig.press(DS_BTN_Y);
    rig.pad.buttons = DS_BTN_Y;                    // and held
    rig.runMs(MS_PER_S / 2);
    rig.pad.buttons = 0;
    rig.runMs(BUTTON_DEBOUNCE_MS + kCtrlMs * 2);
    rig.flush();
    CHECK(!teaching(rig));
    CHECK_EQ(rig.state(), want);
    CHECK_EQ(rig.app->turret().status().refusals, refusals);
    CHECK_EQ(rig.trigger().phase(), phase);
    CHECK_EQ(rig.trigger().fireMode(), mode);
    CHECK_EQ(rig.trigger().ammo(), (uint8_t)MAG_CAPACITY);
    CHECK_EQ(rig.countLines("refused", from) + rig.countLines("servo adjust", from) +
                 rig.countLines("state ", from), (size_t)0);   // nothing logged about it
  }
}

// SERVO ADJUST's cyan wins over the empty-magazine flash: at ammo 0 adjust mode shows solid cyan (the
// telemetry flag drops too), and leaving it brings the purple flash back over green at once.
TEST(servo_adjust_cyan_wins_over_the_empty_flash) {
  Rig rig;
  CHECK(rig.toArmedSpunUp());
  for (unsigned k = 0; k < MAG_CAPACITY; k++) {   // semi: one stroke per pull, then the cooldown
    rig.pad.rt = kPulled;
    rig.runMs(kStrokeMs + kCtrlMs * 2);
    rig.pad.rt = kReleased;
    rig.runMs(kCoolMs + kCtrlMs * 3);
  }
  CHECK_EQ(rig.trigger().ammo(), (uint8_t)0);
  rig.pad.buttons = 0;
  rig.press(DS_BTN_A);
  CHECK_EQ(rig.state(), (uint8_t)DS_STATE_DISARMED);
  CHECK(rig.app->turret().display().emptyMagBlink);
  rig.press(DS_BTN_START);
  CHECK(teaching(rig));
  CHECK(!rig.app->turret().display().emptyMagBlink);
  const Rgb cyan = scaled(LED_COLOR_SERVO_ADJUST);
  const Rgb purple = scaled(LED_COLOR_EMPTY_MAG);
  const Rgb green = scaled(LED_COLOR_DISARMED);
  uint32_t cyanShows = 0, otherShows = 0;
  rig.onShow = [&] { (allLeds(cyan) ? cyanShows : otherShows)++; };
  rig.runMs(2 * LED_EMPTY_FLASH_PERIOD_MS);
  CHECK(cyanShows > 0u);
  CHECK_EQ(otherShows, 0u);
  CHECK(!(rig.telemetry.back().flags & DS_TELEM_FLAG_EMPTY_MAG_BLINK));
  // Leave: the flash is back (its first flash at once), over green.
  rig.onShow = nullptr;
  rig.pad.buttons = DS_BTN_START;
  CHECK(rig.runUntil([&] { return !teaching(rig); }, kCtrlMs * 3));
  rig.pad.buttons = 0;
  CHECK(rig.app->turret().display().emptyMagBlink);
  uint32_t purpleShows = 0, greenShows = 0;
  otherShows = 0;
  bool firstIsPurple = false;
  rig.onShow = [&] {
    if (purpleShows + greenShows + otherShows == 0) firstIsPurple = allLeds(purple);
    if (allLeds(purple)) purpleShows++;
    else if (allLeds(green)) greenShows++;
    else otherShows++;
  };
  rig.runMs(2 * LED_EMPTY_FLASH_PERIOD_MS);
  rig.onShow = nullptr;
  CHECK(firstIsPurple);
  CHECK(purpleShows > 0u && greenShows > purpleShows);
  CHECK_EQ(otherShows, 0u);
}

// =====================================================================================================
// SERVO ADJUST's saved angles (owner, 2026-11-03: the data flash record, ServoStore.h), through the
// host HAL's fake store: hal_host::reset() blanks it, reboot() (Rig keepStore) keeps it
// =====================================================================================================
namespace {

// Angles that differ from config.h's and from each other, inside the adjust range.
int32_t awayFrom(int32_t deg, int32_t by) {
  return deg + by <= SERVO_ADJUST_MAX_DEG ? deg + by : deg - by;
}
const int32_t kSavedRest = awayFrom(TRIGGER_REST_ANGLE, 7);
const int32_t kSavedFire = awayFrom(TRIGGER_FIRE_ANGLE, 9);

void writeStore(const uint8_t* rec, size_t n) {
  for (size_t i = 0; i < n; i++) hal_host::storeSetByte(i, rec[i]);
}
// A valid record in the fake store (call after hal_host::reset(), before a keepStore Rig).
void storeRecord(int32_t rest, int32_t fire) {
  servostore::Angles a;
  a.rest = (int16_t)rest;
  a.fire = (int16_t)fire;
  uint8_t rec[servostore::REC_BYTES];
  servostore::encode(a, rec);
  writeStore(rec, sizeof rec);
}
// What the fake store holds, decoded.
servostore::Load storedAngles(servostore::Angles& out) {
  uint8_t rec[servostore::REC_BYTES];
  for (size_t i = 0; i < sizeof rec; i++) rec[i] = hal_host::storeByte(i);
  return servostore::decode(rec, out);
}
bool storeHolds(int32_t rest, int32_t fire) {
  servostore::Angles a;
  return storedAngles(a) == servostore::Load::OK && a.rest == rest && a.fire == fire;
}
std::string loadedLine(int32_t rest, int32_t fire) {
  std::string l = "servo adjust: loaded saved " + restFire(rest, fire);
  if (rest != TRIGGER_REST_ANGLE || fire != TRIGGER_FIRE_ANGLE)
    l += " (config.h has " + restFire(TRIGGER_REST_ANGLE, TRIGGER_FIRE_ANGLE) + ", not used)";
  return l;
}
std::string noSavedLine(bool invalid) {
  return "servo adjust: no saved angles, using config.h " + restFire(TRIGGER_REST_ANGLE, TRIGGER_FIRE_ANGLE) +
         (invalid ? " (stored record invalid)" : "");
}
bool anglesInForce(const Rig& rig, int32_t rest, int32_t fire) {
  return rig.trigger().restDeg() == (float)rest && rig.trigger().fireDeg() == (float)fire;
}

}  // namespace

// The record's codec on its own: every angle pair round-trips; all 0xFF is BLANK; any single bit
// flipped anywhere, a wrong magic or version, or an angle past SERVO_SPAN_DEG is INVALID.
TEST(servo_store_record_round_trips_and_every_damage_is_invalid) {
  using servostore::Load;
  uint32_t pairs = 0;
  for (int32_t r = 0; r <= SERVO_SPAN_DEG; r++)
    for (int32_t f = 0; f <= SERVO_SPAN_DEG; f++) {
      servostore::Angles a, back;
      a.rest = (int16_t)r;
      a.fire = (int16_t)f;
      uint8_t rec[servostore::REC_BYTES];
      servostore::encode(a, rec);
      if (!CHECK(servostore::decode(rec, back) == Load::OK && servostore::sameAngles(a, back))) return;
      pairs++;
    }
  CHECK_EQ(pairs, (uint32_t)((SERVO_SPAN_DEG + 1) * (SERVO_SPAN_DEG + 1)));
  uint8_t blank[servostore::REC_BYTES];
  std::memset(blank, BYTE_MASK, sizeof blank);
  servostore::Angles untouched;
  untouched.rest = 1;
  CHECK(servostore::decode(blank, untouched) == Load::BLANK);
  CHECK_EQ(untouched.rest, (int16_t)1);                        // left alone unless OK
  servostore::Angles a;
  a.rest = (int16_t)kSavedRest;
  a.fire = (int16_t)kSavedFire;
  uint8_t good[servostore::REC_BYTES];
  servostore::encode(a, good);
  for (size_t i = 0; i < sizeof good; i++)
    for (int bit = 0; bit < BITS_PER_BYTE; bit++) {
      uint8_t bad[servostore::REC_BYTES];
      std::memcpy(bad, good, sizeof bad);
      bad[i] = (uint8_t)(bad[i] ^ (1u << bit));
      servostore::Angles out;
      if (!CHECK(servostore::decode(bad, out) == Load::INVALID)) return;
    }
  // Wrong magic / version / an angle the servo cannot take, each with a CRC that matches.
  const size_t fields[] = {servostore::REC_MAGIC_HI, servostore::REC_VERSION, servostore::REC_REST, servostore::REC_FIRE};
  for (size_t field : fields) {
    uint8_t bad[servostore::REC_BYTES];
    std::memcpy(bad, good, sizeof bad);
    bad[field] = field == servostore::REC_REST || field == servostore::REC_FIRE ? (uint8_t)(SERVO_SPAN_DEG + 1)
                                                                              : (uint8_t)(bad[field] + 1);
    bad[servostore::REC_CRC] = ds_crc8(bad, servostore::REC_CRC);
    servostore::Angles out;
    CHECK(servostore::decode(bad, out) == Load::INVALID);
  }
  // Half-written (the start blank, the rest not): invalid, never blank.
  uint8_t half[servostore::REC_BYTES];
  std::memcpy(half, good, sizeof half);
  half[servostore::REC_MAGIC_LO] = BYTE_MASK;
  CHECK(servostore::decode(half, a) == Load::INVALID);
}

// Boot: a valid record's angles replace config.h's for the run (strokes, the servo's rest, adjust
// mode's start and LCD row 1), logged whatever the level; a blank store or any invalid record leaves
// config.h's, and says which.
TEST(servo_adjust_boot_loads_a_valid_record_and_falls_back_on_blank_or_corrupt) {
  {
    Rig rig;                                       // a new board: blank store
    rig.flush();
    CHECK_EQ(countExact(rig, noSavedLine(false)), (size_t)1);
    CHECK_EQ(hal_host::storeLoads(), 1u);          // read once, at boot
    CHECK(anglesInForce(rig, TRIGGER_REST_ANGLE, TRIGGER_FIRE_ANGLE));
    CHECK_EQ(servoNow(), (int32_t)TRIGGER_REST_ANGLE);
    CHECK(found(rig.shell("status"), "rest=" + std::to_string(TRIGGER_REST_ANGLE) + " fire=" +
                                         std::to_string(TRIGGER_FIRE_ANGLE) + " (config.h)"));
  }
  {
    hal_host::reset();
    storeRecord(kSavedRest, kSavedFire);
    Rig rig(App::defaults(), true, false, true);
    rig.flush();
    CHECK_EQ(countExact(rig, loadedLine(kSavedRest, kSavedFire)), (size_t)1);
    CHECK(anglesInForce(rig, kSavedRest, kSavedFire));
    CHECK(rig.app->turret().status().savedRecord);
    CHECK_EQ(servoNow(), kSavedRest);              // the servo waits at the saved REST from boot
    CHECK(toTeach(rig));
    CHECK_EQ(servoNow(), clampTeach(kSavedRest));  // adjust mode starts from the angles in force
    rig.runMs(LCD_REFRESH_MS * 4);
    CHECK_EQ(rig.lcd(1), teachRow1(std::to_string(kSavedRest), std::to_string(kSavedFire)));
    CHECK(found(rig.shell("status"), "rest=" + std::to_string(kSavedRest) + " fire=" + std::to_string(kSavedFire) +
                                         " (saved)"));
    rig.press(DS_BTN_START);                       // nothing changed: nothing written
    CHECK_EQ(hal_host::storeSaves(), 0u);
  }
  // Damaged records: a flipped CRC, another magic, another version, an angle past the span.
  for (int damage = 0; damage < 4; damage++) {
    hal_host::reset();
    servostore::Angles a;
    a.rest = (int16_t)kSavedRest;
    a.fire = (int16_t)kSavedFire;
    uint8_t rec[servostore::REC_BYTES];
    servostore::encode(a, rec);
    if (damage == 0) rec[servostore::REC_CRC] = (uint8_t)(rec[servostore::REC_CRC] ^ 1u);
    if (damage == 1) rec[servostore::REC_MAGIC_LO] = (uint8_t)(rec[servostore::REC_MAGIC_LO] + 1);
    if (damage == 2) rec[servostore::REC_VERSION] = (uint8_t)(SERVO_STORE_VERSION + 1);
    if (damage == 3) rec[servostore::REC_FIRE] = (uint8_t)(SERVO_SPAN_DEG + 1);
    if (damage != 0) rec[servostore::REC_CRC] = ds_crc8(rec, servostore::REC_CRC);
    writeStore(rec, sizeof rec);
    Rig rig(App::defaults(), true, false, true);
    rig.flush();
    CHECK_EQ(countExact(rig, noSavedLine(true)), (size_t)1);
    CHECK(anglesInForce(rig, TRIGGER_REST_ANGLE, TRIGGER_FIRE_ANGLE));
    CHECK(!rig.app->turret().status().savedRecord);
  }
}

// KEY SAFETY TEST (the one blocking call): leaving with Menu writes the record only when the angles
// in force differ from the saved ones, only in DISARMED with no stroke (a test stroke is aborted
// first) and no axis moving (Menu is refused while one moves); a link loss never writes. The fake
// save blocks for the typical 50 ms and the link survives it.
TEST(servo_adjust_menu_saves_only_changed_angles_and_link_loss_never_saves) {
  Rig rig;
  CHECK(toTeach(rig));
  size_t from = rig.lines.size();
  rig.press(DS_BTN_START);                         // unchanged
  rig.flush();
  CHECK_EQ(countExact(rig, unchangedLine(TRIGGER_REST_ANGLE, TRIGGER_FIRE_ANGLE), from), (size_t)1);
  CHECK_EQ(hal_host::storeSaves(), 0u);
  // Changed, then the link drops: not saved, but in force for the rest of this run.
  CHECK(toTeach(rig));
  rig.press(kAwaySmall);
  rig.press(DS_BTN_A);
  const int32_t rest = servoNow();
  CHECK(rest != (int32_t)TRIGGER_REST_ANGLE);
  from = rig.lines.size();
  rig.pad.send = false;
  CHECK(rig.runUntil([&] { return rig.state() == DS_STATE_PAIRING; }, LINK_TIMEOUT_MS * 2));
  rig.flush();
  CHECK(!teaching(rig));
  CHECK_EQ(hal_host::storeSaves(), 0u);
  CHECK_EQ(countExact(rig, "servo adjust: not saved (link lost): " + restFire(rest, TRIGGER_FIRE_ANGLE) +
                               " stay for this run only", from), (size_t)1);
  CHECK(anglesInForce(rig, rest, TRIGGER_FIRE_ANGLE));
  servostore::Angles none;
  CHECK(storedAngles(none) == servostore::Load::BLANK);
  rig.pad.send = true;
  CHECK(rig.runUntil([&] { return rig.state() == DS_STATE_DISARMED; }, LINK_TIMEOUT_MS * 2));
  CHECK(found(rig.shell("status"), "rest=" + std::to_string(rest) + " fire=" + std::to_string(TRIGGER_FIRE_ANGLE) +
                                       " (not saved)"));
  // Back in, and Menu while the stick moves the turret: refused, still in adjust mode, nothing
  // written; the shell's servo off too.
  CHECK(toTeach(rig));
  rig.pad.rx = kStickMax;
  rig.runMs(MS_PER_S / 4);
  CHECK(rig.motion().isMoving());
  from = rig.lines.size();
  rig.press(DS_BTN_START);
  CHECK(found(rig.shell("servo off"), "servo off: refused: turret moving"));
  rig.flush();
  CHECK_EQ(rig.countLines("Menu (save and leave servo adjust): refused: turret moving", from), (size_t)1);
  CHECK(teaching(rig));
  CHECK_EQ(hal_host::storeSaves(), 0u);
  rig.pad.rx = 0;
  CHECK(rig.runUntil([&] { return !rig.motion().isMoving(); }, MS_PER_S * 2));
  // Menu mid test stroke: the stroke is aborted (servo to REST), then the record is written.
  rig.pad.buttons = DS_BTN_X;
  CHECK(rig.runUntil([&] { return rig.trigger().phase() == Trigger::Phase::HOLD_FIRE; }, kStrokeMs * 2));
  rig.pad.buttons = 0;
  const uint8_t ammo = rig.trigger().ammo();
  bool savedMidStroke = false, savedOffDisarmed = false;
  uint32_t savesSeen = 0;
  rig.everyLoop = [&] {   // after each loop that wrote, the stroke was already over, still DISARMED
    if (hal_host::storeSaves() == savesSeen) return;
    savesSeen = hal_host::storeSaves();
    savedMidStroke = savedMidStroke || rig.trigger().strokeActive();
    savedOffDisarmed = savedOffDisarmed || rig.state() != DS_STATE_DISARMED;
  };
  from = rig.lines.size();
  const uint32_t losses = rig.app->turret().status().linkLosses;
  rig.pad.buttons = DS_BTN_START;
  CHECK(rig.runUntil([&] { return !teaching(rig); }, kCtrlMs * 3));
  rig.pad.buttons = 0;
  rig.runMs(BUTTON_DEBOUNCE_MS + LINK_TIMEOUT_MS * 2);   // the 50 ms block costs no link loss
  rig.everyLoop();
  rig.everyLoop = nullptr;
  rig.flush();
  CHECK_EQ(hal_host::storeSaves(), 1u);
  CHECK(!savedMidStroke && !savedOffDisarmed);
  CHECK(!rig.trigger().strokeActive());
  CHECK_EQ(servoNow(), rest);
  CHECK_EQ(rig.trigger().ammo(), ammo);
  CHECK(storeHolds(rest, TRIGGER_FIRE_ANGLE));
  CHECK_EQ(rig.countLines(savedPrefix(rest, TRIGGER_FIRE_ANGLE) +
                              std::to_string(hal_host::kStoreSaveTypicalUs / US_PER_MS) + " ms", from), (size_t)1);
  CHECK_EQ(countExact(rig, summaryLine(rest, TRIGGER_FIRE_ANGLE), from), (size_t)1);
  CHECK_EQ(rig.app->turret().status().linkLosses, losses);
  CHECK_EQ(rig.state(), (uint8_t)DS_STATE_DISARMED);
  CHECK_EQ(blockingWrites(), 0u);
  // Saved now: in and out again writes nothing.
  CHECK(toTeach(rig));
  rig.press(DS_BTN_START);
  CHECK_EQ(hal_host::storeSaves(), 1u);
  CHECK(found(rig.shell("status"), " (saved)"));
}

// View in adjust mode: config.h's REST and FIRE in force at once (the servo to that REST, LCD row 1,
// the log), and Menu then saves them over the stored record. Outside adjust mode View does nothing.
TEST(servo_adjust_view_restores_config_angles_and_menu_saves_them) {
  hal_host::reset();
  storeRecord(kSavedRest, kSavedFire);
  Rig rig(App::defaults(), true, false, true);
  CHECK(rig.toDisarmed());
  rig.flush();
  size_t from = rig.lines.size();
  const uint32_t refusals = rig.app->turret().status().refusals;
  rig.press(DS_BTN_BACK);                          // DISARMED, not adjusting: nothing at all
  rig.flush();
  CHECK_EQ(rig.lines.size(), from);
  CHECK_EQ(rig.app->turret().status().refusals, refusals);
  CHECK(anglesInForce(rig, kSavedRest, kSavedFire));
  CHECK(toTeach(rig));
  CHECK_EQ(servoNow(), clampTeach(kSavedRest));
  from = rig.lines.size();
  rig.press(DS_BTN_BACK);
  rig.flush();
  CHECK(anglesInForce(rig, TRIGGER_REST_ANGLE, TRIGGER_FIRE_ANGLE));
  CHECK_EQ(servoNow(), clampTeach(TRIGGER_REST_ANGLE));
  CHECK_EQ(countExact(rig, "servo adjust: config.h angles restored: " + restFire(TRIGGER_REST_ANGLE, TRIGGER_FIRE_ANGLE) +
                               " (Menu saves them)", from), (size_t)1);
  rig.runMs(LCD_REFRESH_MS * 4);
  CHECK_EQ(rig.lcd(1), teachRow1(std::to_string(TRIGGER_REST_ANGLE), std::to_string(TRIGGER_FIRE_ANGLE)));
  CHECK(teaching(rig));
  CHECK(storeHolds(kSavedRest, kSavedFire));       // View alone writes nothing
  from = rig.lines.size();
  rig.press(DS_BTN_START);
  rig.flush();
  CHECK_EQ(hal_host::storeSaves(), 1u);
  CHECK(storeHolds(TRIGGER_REST_ANGLE, TRIGGER_FIRE_ANGLE));
  CHECK_EQ(rig.countLines(savedPrefix(TRIGGER_REST_ANGLE, TRIGGER_FIRE_ANGLE), from), (size_t)1);
  CHECK(found(rig.shell("status"), " (saved)"));
}

// Saved angles survive a reboot of the board (a new App on the same store) and the power toggle's
// OFF -> BOOT; angles changed but not saved are dropped by both.
TEST(servo_adjust_saved_angles_survive_a_reboot_and_unsaved_ones_do_not) {
  {
    Rig rig;
    CHECK(toTeach(rig));
    CHECK(found(rig.shell("servo " + std::to_string(kSavedRest)), ": ok"));
    rig.press(DS_BTN_A);
    CHECK(found(rig.shell("servo " + std::to_string(kSavedFire)), ": ok"));
    rig.press(DS_BTN_B);
    rig.press(DS_BTN_START);
    CHECK_EQ(hal_host::storeSaves(), 1u);
    CHECK(storeHolds(kSavedRest, kSavedFire));
  }
  {
    Rig rig(App::defaults(), true, false, true);   // the board reboots
    rig.flush();
    CHECK_EQ(countExact(rig, loadedLine(kSavedRest, kSavedFire)), (size_t)1);
    CHECK(anglesInForce(rig, kSavedRest, kSavedFire));
    CHECK_EQ(servoNow(), kSavedRest);
    // Changed in adjust mode, left by the power toggle: not saved, and the toggle's reboot is back
    // on the saved angles.
    CHECK(toTeach(rig));
    rig.press(kAwayLarge);
    rig.press(DS_BTN_A);
    const int32_t unsaved = servoNow();
    CHECK(unsaved != kSavedRest);
    rig.pad = Pad();
    rig.setPower(false);
    CHECK(rig.runUntil([&] { return rig.state() == DS_STATE_OFF; }, TOGGLE_DEBOUNCE_MS + kCtrlMs));
    CHECK(anglesInForce(rig, kSavedRest, kSavedFire));
    rig.setPower(true);
    CHECK(rig.toDisarmed());
    CHECK(anglesInForce(rig, kSavedRest, kSavedFire));
    CHECK_EQ(servoNow(), kSavedRest);
    CHECK_EQ(hal_host::storeSaves(), 0u);
    // Changed again, left by a link loss (RAM only), then the board reboots: the saved angles.
    CHECK(toTeach(rig));
    rig.press(kAwayLarge);
    rig.press(DS_BTN_B);
    rig.pad.send = false;
    CHECK(rig.runUntil([&] { return rig.state() == DS_STATE_PAIRING; }, LINK_TIMEOUT_MS * 2));
    CHECK(!anglesInForce(rig, kSavedRest, kSavedFire));
  }
  {
    Rig rig(App::defaults(), true, false, true);
    CHECK(anglesInForce(rig, kSavedRest, kSavedFire));
    CHECK(storeHolds(kSavedRest, kSavedFire));
  }
}

// A save whose read-back fails (worn flash): logged, counted, the angles stay in force for this run
// and the store's next boot is not trusted with them.
TEST(servo_adjust_failed_save_keeps_the_angles_for_this_run_only) {
  Rig rig;
  hal_host::failStoreSaves(true);
  CHECK(toTeach(rig));
  rig.press(kAwaySmall);
  rig.press(DS_BTN_A);
  const int32_t rest = servoNow();
  const size_t from = rig.lines.size();
  rig.press(DS_BTN_START);
  rig.flush();
  CHECK(!teaching(rig));
  CHECK_EQ(countExact(rig, "servo adjust: save FAILED (data flash read-back), " + restFire(rest, TRIGGER_FIRE_ANGLE) +
                               " stay for this run only", from), (size_t)1);
  CHECK_EQ(rig.app->turret().status().saveFailures, 1u);
  CHECK(anglesInForce(rig, rest, TRIGGER_FIRE_ANGLE));
  CHECK(found(rig.shell("status"), " (not saved)"));
  hal_host::failStoreSaves(false);
  Rig again(App::defaults(), true, false, true);
  CHECK(anglesInForce(again, TRIGGER_REST_ANGLE, TRIGGER_FIRE_ANGLE));
}

TESTKIT_MAIN()

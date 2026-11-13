// test_inputs.cpp -- Inputs (docs/architecture.md 5, spec 3.2): toggle debounce in both
// polarities and both mode senses, the stick curve (exact values, dead zone, expo, clamping,
// monotonicity, JOY_INVERT_Y), trigger hysteresis, button edges with the BUTTON_DEBOUNCE_MS
// lockout, and the link rules (no edges from a frame that is not connected, held state cleared
// on a drop, a baseline frame after every drop and source change).
//
// Safety property: a frame that is not connected never produces an edge and never leaves a
// level held (disconnected_frame_produces_no_edges_and_clears_held_state), and a button held
// through a link drop cannot fire on reconnect (button_held_through_a_drop_is_not_an_edge).
//
// SPDX-License-Identifier: MIT
#include <cmath>

#include "Inputs.h"
#include "hal.h"
#include "hal_host.h"
#include "testkit.h"

namespace {

void boot() {
  hal_host::reset();
  hal::begin();
}

// A connected frame from `source`.
ControlFrame live(int16_t rx = 0, int16_t ry = 0, uint16_t rt = 0, uint16_t buttons = 0,
                  uint8_t source = DS_LINK_SRC_USB) {
  ControlFrame f;
  f.valid = true;
  f.connected = true;
  f.source = source;
  f.rx = rx;
  f.ry = ry;
  f.rt = rt;
  f.buttons = buttons;
  return f;
}

// The pin level a toggle reads when closed/open under a polarity.
bool level(bool closed, bool activeLow) { return activeLow ? !closed : closed; }

Inputs::Config cfgWith(bool activeLow, bool wifiWhenClosed, bool invertY = JOY_INVERT_Y) {
  Inputs::Config c = Inputs::defaults();
  c.toggleActiveLow = activeLow;
  c.modeWifiWhenClosed = wifiWhenClosed;
  c.invertY = invertY;
  return c;
}

// The contract's curve, written out independently in double precision.
double expectedShape(double s, double dz, double e) {
  double mag = std::fabs(s);
  if (mag > 1.0) mag = 1.0;
  if (mag <= dz) return 0.0;
  const double m = (mag - dz) / (1.0 - dz);
  const double out = (1.0 - e) * m + e * m * m * m;
  return s < 0 ? -out : out;
}

// Smallest trigger reading the firmware counts as above / below a fraction, found with the
// same float arithmetic the module uses, so the boundary tests are exact for any config.
uint16_t firstRtAbove(float fraction) {
  for (uint32_t rt = 0; rt <= TRIGGER_AXIS_RANGE; rt++)
    if ((float)rt / (float)TRIGGER_AXIS_RANGE > fraction) return (uint16_t)rt;
  return TRIGGER_AXIS_RANGE;
}
uint16_t lastRtBelow(float fraction) {
  uint16_t last = 0;
  for (uint32_t rt = 0; rt <= TRIGGER_AXIS_RANGE; rt++)
    if ((float)rt / (float)TRIGGER_AXIS_RANGE < fraction) last = (uint16_t)rt;
  return last;
}

// Readings well inside each band, from the config.
uint16_t rtBetween() {
  return (uint16_t)((TRIGGER_PRESS_THRESHOLD + TRIGGER_RELEASE_THRESHOLD) * MATH_HALF * TRIGGER_AXIS_RANGE);
}
uint16_t rtAbove() {
  return (uint16_t)((TRIGGER_PRESS_THRESHOLD + 1.0f) * MATH_HALF * TRIGGER_AXIS_RANGE);
}
uint16_t rtBelow() { return (uint16_t)(TRIGGER_RELEASE_THRESHOLD * MATH_HALF * TRIGGER_AXIS_RANGE); }

}  // namespace

// ---------------------------------------------------------------- toggles

TEST(defaults_come_from_config_h) {
  const Inputs::Config c = Inputs::defaults();
  CHECK_EQ(c.toggleActiveLow, TOGGLE_ACTIVE_LOW);
  CHECK_EQ(c.modeWifiWhenClosed, TOGGLE_MODE_WIFI_WHEN_CLOSED);
  CHECK_EQ(c.invertY, JOY_INVERT_Y);
  Inputs plain;
  CHECK_EQ(plain.config().toggleActiveLow, TOGGLE_ACTIVE_LOW);
}

TEST(begin_reads_the_real_toggle_state_without_debounce) {
  for (int pol = 0; pol < 2; pol++) {
    const bool activeLow = pol == 0;
    boot();
    hal_host::setDigitalInput(PIN_TOGGLE_POWER, level(true, activeLow));
    hal_host::setDigitalInput(PIN_TOGGLE_MODE, level(true, activeLow));
    Inputs in(cfgWith(activeLow, true));
    in.begin(0);
    CHECK(in.powerOn());
    CHECK(!in.powerChanged());
    CHECK_EQ(in.radioMode(), DS_RADIO_WIFI);
    CHECK(!in.radioModeChanged());

    boot();
    hal_host::setDigitalInput(PIN_TOGGLE_POWER, level(false, activeLow));
    hal_host::setDigitalInput(PIN_TOGGLE_MODE, level(false, activeLow));
    Inputs in2(cfgWith(activeLow, true));
    in2.begin(0);
    CHECK(!in2.powerOn());
    CHECK_EQ(in2.radioMode(), DS_RADIO_BLE);
  }
}

TEST(host_reset_toggles_read_open_with_the_config_polarity) {
  boot();
  Inputs in;
  in.begin(0);
  CHECK(!in.powerOn());   // hal_host::reset leaves the toggles open: OFF state at boot
}

TEST(power_toggle_debounce_both_polarities) {
  for (int pol = 0; pol < 2; pol++) {
    const bool activeLow = pol == 0;
    boot();
    hal_host::setDigitalInput(PIN_TOGGLE_POWER, level(false, activeLow));
    Inputs in(cfgWith(activeLow, true));
    in.begin(0);
    const ControlFrame none;
    in.update(500, none);
    CHECK(!in.powerOn());

    // Close at t = 1000: accepted exactly TOGGLE_DEBOUNCE_MS later, flagged for one update.
    hal_host::setDigitalInput(PIN_TOGGLE_POWER, level(true, activeLow));
    for (uint32_t t = 1000; t < 1000 + TOGGLE_DEBOUNCE_MS; t++) {
      in.update(t, none);
      CHECK(!in.powerOn());
      CHECK(!in.powerChanged());
      CHECK(in.powerRaw());
    }
    in.update(1000 + TOGGLE_DEBOUNCE_MS, none);
    CHECK(in.powerOn());
    CHECK(in.powerChanged());
    in.update(1001 + TOGGLE_DEBOUNCE_MS, none);
    CHECK(in.powerOn());
    CHECK(!in.powerChanged());

    // Open again: same rule the other way.
    hal_host::setDigitalInput(PIN_TOGGLE_POWER, level(false, activeLow));
    in.update(2000, none);
    CHECK(in.powerOn());
    CHECK(!in.powerRaw());
    in.update(2000 + TOGGLE_DEBOUNCE_MS - 1, none);
    CHECK(in.powerOn());
    in.update(2000 + TOGGLE_DEBOUNCE_MS, none);
    CHECK(!in.powerOn());
    CHECK(in.powerChanged());
  }
}

TEST(toggle_bounce_restarts_the_debounce_timer) {
  boot();
  Inputs in;
  in.begin(0);
  const ControlFrame none;
  const bool closed = level(true, TOGGLE_ACTIVE_LOW);
  const bool open = level(false, TOGGLE_ACTIVE_LOW);
  hal_host::setDigitalInput(PIN_TOGGLE_POWER, closed);
  in.update(1000, none);
  hal_host::setDigitalInput(PIN_TOGGLE_POWER, open);
  in.update(1010, none);
  hal_host::setDigitalInput(PIN_TOGGLE_POWER, closed);
  in.update(1020, none);
  in.update(1000 + TOGGLE_DEBOUNCE_MS, none);   // would have been accepted without the bounce
  CHECK(!in.powerOn());
  in.update(1020 + TOGGLE_DEBOUNCE_MS - 1, none);
  CHECK(!in.powerOn());
  in.update(1020 + TOGGLE_DEBOUNCE_MS, none);
  CHECK(in.powerOn());
  CHECK(in.powerChanged());
}

TEST(toggle_glitch_shorter_than_debounce_is_ignored) {
  boot();
  Inputs in;
  in.begin(0);
  const ControlFrame none;
  hal_host::setDigitalInput(PIN_TOGGLE_POWER, level(true, TOGGLE_ACTIVE_LOW));
  for (uint32_t t = 1000; t < 1000 + TOGGLE_DEBOUNCE_MS; t++) in.update(t, none);
  hal_host::setDigitalInput(PIN_TOGGLE_POWER, level(false, TOGGLE_ACTIVE_LOW));
  bool everOn = false;
  bool everChanged = false;
  for (uint32_t t = 1000 + TOGGLE_DEBOUNCE_MS; t < 1000 + 4 * TOGGLE_DEBOUNCE_MS; t++) {
    in.update(t, none);
    everOn = everOn || in.powerOn();
    everChanged = everChanged || in.powerChanged();
  }
  CHECK(!everOn);
  CHECK(!everChanged);
}

TEST(mode_toggle_sense_and_polarity_all_four_combinations) {
  for (int pol = 0; pol < 2; pol++) {
    for (int sense = 0; sense < 2; sense++) {
      const bool activeLow = pol == 0;
      const bool wifiWhenClosed = sense == 0;
      boot();
      hal_host::setDigitalInput(PIN_TOGGLE_MODE, level(false, activeLow));
      Inputs in(cfgWith(activeLow, wifiWhenClosed));
      in.begin(0);
      CHECK_EQ(in.radioMode(), wifiWhenClosed ? DS_RADIO_BLE : DS_RADIO_WIFI);
      hal_host::setDigitalInput(PIN_TOGGLE_MODE, level(true, activeLow));
      const ControlFrame none;
      in.update(100, none);
      CHECK_EQ(in.radioMode(), wifiWhenClosed ? DS_RADIO_BLE : DS_RADIO_WIFI);   // not yet
      CHECK_EQ(in.modeRaw(), wifiWhenClosed);
      in.update(100 + TOGGLE_DEBOUNCE_MS, none);
      CHECK_EQ(in.radioMode(), wifiWhenClosed ? DS_RADIO_WIFI : DS_RADIO_BLE);
      CHECK(in.radioModeChanged());
      in.update(101 + TOGGLE_DEBOUNCE_MS, none);
      CHECK(!in.radioModeChanged());
    }
  }
}

TEST(toggles_debounce_independently_of_the_link) {
  boot();
  Inputs in;
  in.begin(0);
  hal_host::setDigitalInput(PIN_TOGGLE_POWER, level(true, TOGGLE_ACTIVE_LOW));
  in.update(10, live());
  in.update(10 + TOGGLE_DEBOUNCE_MS, live());
  CHECK(in.powerOn());
}

// ---------------------------------------------------------------- stick curve

TEST(shape_exact_values) {
  // Hand values (dead zone 0.12, expo 0.4): m = (0.5-0.12)/0.88; 0.6 m + 0.4 m^3.
  CHECK_NEAR(Inputs::shape(0.5f, 0.12f, 0.4f), 0.2912988354620586, 1e-6);
  // m = 0.75 exactly: 0.6*0.75 + 0.4*0.421875 = 0.61875, sign kept.
  CHECK_NEAR(Inputs::shape(-0.78f, 0.12f, 0.4f), -0.61875, 1e-6);
  // Linear (expo 0) and fully cubic (expo 1): m = 0.5.
  CHECK_NEAR(Inputs::shape(0.56f, 0.12f, 0.0f), 0.5, 1e-6);
  CHECK_NEAR(Inputs::shape(0.56f, 0.12f, 1.0f), 0.125, 1e-6);
  // m = 0.125, expo 0.5: 0.0625 + 0.0009765625.
  CHECK_NEAR(Inputs::shape(0.3f, 0.2f, 0.5f), 0.0634765625, 1e-6);
  // Ends and the dead zone itself (|s| == dz is inside).
  CHECK_EQ(Inputs::shape(0.0f, 0.12f, 0.4f), 0.0f);
  CHECK_EQ(Inputs::shape(0.12f, 0.12f, 0.4f), 0.0f);
  CHECK_EQ(Inputs::shape(-0.12f, 0.12f, 0.4f), 0.0f);
  CHECK_NEAR(Inputs::shape(1.0f, 0.12f, 0.4f), 1.0, 1e-6);
  CHECK_NEAR(Inputs::shape(-1.0f, 0.12f, 0.4f), -1.0, 1e-6);
  // Clamped input, NaN, and a degenerate dead zone never divide by zero.
  CHECK_NEAR(Inputs::shape(7.0f, 0.12f, 0.4f), 1.0, 1e-6);
  CHECK_NEAR(Inputs::shape(-7.0f, 0.12f, 0.4f), -1.0, 1e-6);
  CHECK_EQ(Inputs::shape(NAN, 0.12f, 0.4f), 0.0f);
  CHECK_EQ(Inputs::shape(1.0f, 1.0f, 0.4f), 0.0f);
}

TEST(shape_axis_matches_the_contract_with_config_values) {
  for (int raw = -JOY_AXIS_RANGE; raw < JOY_AXIS_RANGE; raw += 7) {
    const double expected = expectedShape((double)raw / JOY_AXIS_RANGE, JOY_DEADZONE, JOY_EXPO);
    CHECK_NEAR(Inputs::shapeAxis((int16_t)raw), expected, 1e-5);
  }
  CHECK_EQ(Inputs::shapeAxis((int16_t)(JOY_AXIS_RANGE / 2)),
           Inputs::shape(MATH_HALF, JOY_DEADZONE, JOY_EXPO));
}

TEST(dead_zone_boundary_in_raw_counts) {
  // Largest raw with |raw|/range <= dz, and the next one.
  int inside = 0;
  while ((float)(inside + 1) / (float)JOY_AXIS_RANGE <= JOY_DEADZONE) inside++;
  CHECK_EQ(Inputs::shapeAxis((int16_t)inside), 0.0f);
  CHECK_EQ(Inputs::shapeAxis((int16_t)-inside), 0.0f);
  CHECK(!Inputs::outsideDeadzone((int16_t)inside));
  CHECK(!Inputs::outsideDeadzone((int16_t)-inside));
  CHECK(Inputs::shapeAxis((int16_t)(inside + 1)) > 0.0f);
  CHECK(Inputs::shapeAxis((int16_t)(-inside - 1)) < 0.0f);
  CHECK(Inputs::outsideDeadzone((int16_t)(inside + 1)));
  CHECK(Inputs::outsideDeadzone((int16_t)(-inside - 1)));
  // Just outside, the output starts near 0 (rescaled), not at the dead-zone value.
  CHECK(Inputs::shapeAxis((int16_t)(inside + 1)) < 0.01f);
}

TEST(shape_axis_is_monotonic_bounded_and_odd_over_the_whole_int16_range) {
  float prev = Inputs::shapeAxis(INT16_MIN);
  bool monotonic = true;
  bool bounded = true;
  for (int raw = INT16_MIN + 1; raw <= INT16_MAX; raw++) {
    const float v = Inputs::shapeAxis((int16_t)raw);
    if (v < prev) monotonic = false;
    if (v > 1.0f || v < -1.0f) bounded = false;
    prev = v;
  }
  CHECK(monotonic);
  CHECK(bounded);
  bool odd = true;
  for (int raw = -JOY_AXIS_RANGE + 1; raw < JOY_AXIS_RANGE; raw++)
    if (Inputs::shapeAxis((int16_t)raw) != -Inputs::shapeAxis((int16_t)-raw)) odd = false;
  CHECK(odd);
}

TEST(out_of_range_axis_values_are_clamped) {
  CHECK_NEAR(Inputs::shapeAxis(INT16_MIN), -1.0, 1e-6);
  CHECK_NEAR(Inputs::shapeAxis(INT16_MAX), 1.0, 1e-6);
  CHECK_NEAR(Inputs::shapeAxis((int16_t)(JOY_AXIS_RANGE + 100)), 1.0, 1e-6);
  CHECK_NEAR(Inputs::shapeAxis((int16_t)-JOY_AXIS_RANGE), -1.0, 1e-6);
  CHECK(Inputs::outsideDeadzone(INT16_MIN));
  boot();
  Inputs in;
  in.begin(0);
  in.update(1, live(INT16_MAX, INT16_MIN, UINT16_MAX));
  CHECK_NEAR(in.stickX(), 1.0, 1e-6);
  CHECK_NEAR(std::fabs(in.stickY()), 1.0, 1e-6);
  CHECK_EQ(in.triggerFraction(), 1.0f);
  CHECK(in.triggerPulled());
}

TEST(joy_invert_y_makes_stick_up_positive) {
  // Wire convention: stick up is NEGATIVE (architecture.md 4.1).
  const int16_t up = (int16_t)-JOY_AXIS_RANGE;
  const int16_t right = (int16_t)(JOY_AXIS_RANGE / 2);
  boot();
  Inputs inverted(cfgWith(TOGGLE_ACTIVE_LOW, TOGGLE_MODE_WIFI_WHEN_CLOSED, true));
  inverted.begin(0);
  inverted.update(1, live(right, up));
  CHECK_NEAR(inverted.stickY(), 1.0, 1e-6);
  CHECK(inverted.stickX() > 0.0f);   // X is never inverted
  CHECK_NEAR(inverted.stickX(), Inputs::shapeAxis(right), 1e-7);
  Inputs plain(cfgWith(TOGGLE_ACTIVE_LOW, TOGGLE_MODE_WIFI_WHEN_CLOSED, false));
  plain.begin(0);
  plain.update(1, live(right, up));
  CHECK_NEAR(plain.stickY(), -1.0, 1e-6);
  // The shipped config: JOY_INVERT_Y true, so stick up = +tilt.
  Inputs shipped;
  shipped.begin(0);
  shipped.update(1, live(0, up));
  CHECK_EQ(shipped.stickY() > 0.0f, JOY_INVERT_Y);
}

TEST(stick_active_when_either_axis_leaves_the_dead_zone) {
  int inside = 0;
  while ((float)(inside + 1) / (float)JOY_AXIS_RANGE <= JOY_DEADZONE) inside++;
  boot();
  Inputs in;
  in.begin(0);
  in.update(1, live((int16_t)inside, (int16_t)-inside));
  CHECK(!in.stickActive());
  CHECK_EQ(in.stickX(), 0.0f);
  CHECK_EQ(in.stickY(), 0.0f);
  in.update(2, live((int16_t)(inside + 1), 0));
  CHECK(in.stickActive());
  in.update(3, live(0, (int16_t)(-inside - 1)));
  CHECK(in.stickActive());
  in.update(4, live(0, 0));
  CHECK(!in.stickActive());
  in.update(5, live((int16_t)JOY_AXIS_RANGE / 2, 0));
  CHECK(in.stickActive());
  const ControlFrame none;
  in.update(6, none);
  CHECK(!in.stickActive());
  CHECK_EQ(in.stickX(), 0.0f);
}

// ---------------------------------------------------------------- trigger hysteresis

TEST(trigger_hysteresis_between_thresholds_keeps_the_previous_state) {
  boot();
  Inputs in;
  in.begin(0);
  uint32_t t = 1;
  in.update(t++, live(0, 0, 0));          // baseline
  CHECK(!in.triggerPulled());
  in.update(t++, live(0, 0, rtBetween()));
  CHECK(!in.triggerPulled());             // between, was released: stays released
  CHECK(!in.triggerPullStarted());
  in.update(t++, live(0, 0, rtAbove()));
  CHECK(in.triggerPulled());
  CHECK(in.triggerPullStarted());
  in.update(t++, live(0, 0, rtAbove()));
  CHECK(in.triggerPulled());
  CHECK(!in.triggerPullStarted());        // one edge per pull
  in.update(t++, live(0, 0, rtBetween()));
  CHECK(in.triggerPulled());              // between, was pulled: stays pulled
  CHECK(!in.triggerPullStarted());
  in.update(t++, live(0, 0, rtBelow()));
  CHECK(!in.triggerPulled());
  in.update(t++, live(0, 0, rtBetween()));
  CHECK(!in.triggerPulled());
  in.update(t++, live(0, 0, rtAbove()));
  CHECK(in.triggerPulled());
  CHECK(in.triggerPullStarted());
  CHECK_NEAR(in.triggerFraction(), (float)rtAbove() / (float)TRIGGER_AXIS_RANGE, 1e-7);
}

TEST(trigger_threshold_boundaries_are_strict) {
  const uint16_t press = firstRtAbove(TRIGGER_PRESS_THRESHOLD);
  const uint16_t release = lastRtBelow(TRIGGER_RELEASE_THRESHOLD);
  CHECK(press > release);
  boot();
  Inputs in;
  in.begin(0);
  in.update(1, live(0, 0, 0));
  in.update(2, live(0, 0, (uint16_t)(press - 1)));   // at or below 60 %: not pulled
  CHECK(!in.triggerPulled());
  in.update(3, live(0, 0, press));                   // first reading above 60 %
  CHECK(in.triggerPulled());
  in.update(4, live(0, 0, (uint16_t)(release + 1))); // at or above 40 %: still pulled
  CHECK(in.triggerPulled());
  in.update(5, live(0, 0, release));                 // first reading below 40 %
  CHECK(!in.triggerPulled());
  // With the shipped numbers (1023 range, 60 / 40 %): 614 pulls, 613 does not; 409 releases.
  if (TRIGGER_AXIS_RANGE == 1023 && TRIGGER_PRESS_THRESHOLD == 0.60f && TRIGGER_RELEASE_THRESHOLD == 0.40f) {
    CHECK_EQ(press, 614);
    CHECK_EQ(release, 409);
  }
}

// ---------------------------------------------------------------- buttons

TEST(button_edge_is_immediate_and_holds_one_update) {
  boot();
  Inputs in;
  in.begin(0);
  in.update(1000, live());                       // baseline: nothing held
  in.update(1001, live(0, 0, 0, DS_BTN_A));
  CHECK(in.pressed(DS_BTN_A));
  CHECK(in.held(DS_BTN_A));
  CHECK_EQ(in.pressedEdges(), DS_BTN_A);
  in.update(1002, live(0, 0, 0, DS_BTN_A));      // still held: no new edge
  CHECK(!in.pressed(DS_BTN_A));
  CHECK(in.held(DS_BTN_A));
  in.update(1003, live(0, 0, 0, 0));
  CHECK(!in.held(DS_BTN_A));
  CHECK(!in.pressed(DS_BTN_A));
}

TEST(button_chatter_inside_the_lockout_is_ignored_and_a_later_press_counts) {
  boot();
  Inputs in;
  in.begin(0);
  in.update(999, live());
  const uint32_t t0 = 1000;
  int edges = 0;
  in.update(t0, live(0, 0, 0, DS_BTN_B));
  edges += in.pressed(DS_BTN_B);
  // Bounce: release / press every ms for the whole lockout.
  for (uint32_t t = t0 + 1; t < t0 + BUTTON_DEBOUNCE_MS; t++) {
    in.update(t, live(0, 0, 0, (t % 2) ? 0 : DS_BTN_B));
    edges += in.pressed(DS_BTN_B);
  }
  CHECK_EQ(edges, 1);
  // Make sure the last chatter sample is "released", then a real press exactly at the
  // lockout's end counts.
  in.update(t0 + BUTTON_DEBOUNCE_MS - 1, live());
  in.update(t0 + BUTTON_DEBOUNCE_MS, live(0, 0, 0, DS_BTN_B));
  CHECK(in.pressed(DS_BTN_B));
}

TEST(button_held_out_of_a_chatter_burst_does_not_make_a_late_edge) {
  boot();
  Inputs in;
  in.begin(0);
  in.update(0, live());
  in.update(100, live(0, 0, 0, DS_BTN_X));
  CHECK(in.pressed(DS_BTN_X));
  in.update(105, live());
  in.update(110, live(0, 0, 0, DS_BTN_X));   // chatter press, ignored
  CHECK(!in.pressed(DS_BTN_X));
  bool late = false;
  for (uint32_t t = 111; t < 100 + 3 * BUTTON_DEBOUNCE_MS; t++) {
    in.update(t, live(0, 0, 0, DS_BTN_X));   // held through the lockout's end
    late = late || in.pressed(DS_BTN_X);
  }
  CHECK(!late);
}

TEST(lockout_is_per_button) {
  boot();
  Inputs in;
  in.begin(0);
  in.update(0, live());
  in.update(10, live(0, 0, 0, DS_BTN_A));
  CHECK(in.pressed(DS_BTN_A));
  in.update(11, live(0, 0, 0, DS_BTN_A | DS_BTN_B));   // B inside A's lockout: still an edge
  CHECK(in.pressed(DS_BTN_B));
  CHECK(!in.pressed(DS_BTN_A));
  in.update(12, live(0, 0, 0, DS_BTN_DPAD_LEFT | DS_BTN_DPAD_DOWN));
  CHECK(in.pressed(DS_BTN_DPAD_LEFT));
  CHECK(in.pressed(DS_BTN_DPAD_DOWN));
  CHECK_EQ(in.heldLevels(), DS_BTN_DPAD_LEFT | DS_BTN_DPAD_DOWN);
}

TEST(lockout_survives_the_millisecond_wrap) {
  boot();
  Inputs in;
  in.begin(0);
  const uint32_t t0 = UINT32_MAX - 10;
  in.update(t0 - 1, live());
  in.update(t0, live(0, 0, 0, DS_BTN_A));
  CHECK(in.pressed(DS_BTN_A));
  in.update(t0 + 2, live());
  in.update(t0 + 5, live(0, 0, 0, DS_BTN_A));                        // wrapped past 0: chatter
  CHECK(!in.pressed(DS_BTN_A));
  in.update(t0 + BUTTON_DEBOUNCE_MS - 1, live());
  in.update(t0 + BUTTON_DEBOUNCE_MS, live(0, 0, 0, DS_BTN_A));       // after the lockout
  CHECK(in.pressed(DS_BTN_A));
}

TEST(lb_is_a_level_and_undefined_bits_are_ignored) {
  boot();
  Inputs in;
  in.begin(0);
  in.update(0, live());
  in.update(1, live(0, 0, 0, DS_BTN_LB));
  CHECK(in.held(DS_BTN_LB));
  in.update(2, live(0, 0, 0, DS_BTN_LB));
  CHECK(in.held(DS_BTN_LB));
  in.update(3, live());
  CHECK(!in.held(DS_BTN_LB));
  // Bits 12-15 reach the RA4M1 for the dashboard only; the turret must never act on them.
  const uint16_t displayOnly = (uint16_t)(DS_BTN_LS_CLICK | DS_BTN_RS_CLICK | DS_BTN_XBOX | DS_BTN_SHARE);
  in.update(4, live(0, 0, 0, displayOnly));
  CHECK_EQ(in.pressedEdges(), 0);
  CHECK_EQ(in.heldLevels(), 0);
}

// ---------------------------------------------------------------- link rules

TEST(disconnected_frame_produces_no_edges_and_clears_held_state) {
  boot();
  Inputs in;
  in.begin(0);
  in.update(0, live());
  in.update(10, live((int16_t)JOY_AXIS_RANGE / 2, (int16_t)-JOY_AXIS_RANGE / 2, rtAbove(), DS_BTN_LB | DS_BTN_A));
  CHECK(in.connected());
  CHECK(in.held(DS_BTN_LB));
  CHECK(in.triggerPulled());
  CHECK(in.stickActive());

  // Valid but not connected (flags.connected = 0), with everything still "pressed".
  ControlFrame lost = live((int16_t)JOY_AXIS_RANGE / 2, 0, rtAbove(), DS_BTN_LB | DS_BTN_B | DS_BTN_X);
  lost.connected = false;
  in.update(20, lost);
  CHECK(!in.connected());
  CHECK_EQ(in.pressedEdges(), 0);
  CHECK_EQ(in.heldLevels(), 0);
  CHECK(!in.triggerPulled());
  CHECK(!in.triggerPullStarted());
  CHECK_EQ(in.triggerFraction(), 0.0f);
  CHECK_EQ(in.stickX(), 0.0f);
  CHECK_EQ(in.stickY(), 0.0f);
  CHECK(!in.stickActive());
  // Not valid at all (no frame since the link came up): same.
  ControlFrame invalid = lost;
  invalid.valid = false;
  invalid.connected = true;
  for (uint32_t t = 30; t < 60; t++) {
    in.update(t, invalid);
    CHECK_EQ(in.pressedEdges(), 0);
    CHECK_EQ(in.heldLevels(), 0);
    CHECK(!in.triggerPulled());
  }
}

TEST(button_held_through_a_drop_is_not_an_edge) {
  boot();
  Inputs in;
  in.begin(0);
  in.update(0, live());
  in.update(10, live(0, 0, 0, DS_BTN_A));
  CHECK(in.pressed(DS_BTN_A));
  const ControlFrame none;
  in.update(500, none);                               // link lost
  in.update(900, live(0, 0, rtAbove(), DS_BTN_A));    // back, A and the trigger still held
  CHECK(in.connected());
  CHECK(!in.pressed(DS_BTN_A));
  CHECK(in.held(DS_BTN_A));                           // the level is reported
  CHECK(in.triggerPulled());
  CHECK(!in.triggerPullStarted());                    // but not as a fresh pull
  in.update(901, live(0, 0, rtAbove(), DS_BTN_A));
  CHECK(!in.pressed(DS_BTN_A));
  in.update(902, live());
  in.update(903, live(0, 0, rtAbove(), DS_BTN_A));    // released and pressed again: counts
  CHECK(in.pressed(DS_BTN_A));
  CHECK(in.triggerPullStarted());
}

TEST(first_connected_frame_after_begin_is_a_baseline) {
  boot();
  Inputs in;
  in.begin(0);
  in.update(5, live(0, 0, 0, DS_BTN_A | DS_BTN_B));
  CHECK_EQ(in.pressedEdges(), 0);
  CHECK_EQ(in.heldLevels(), DS_BTN_A | DS_BTN_B);
  in.update(6, live(0, 0, 0, DS_BTN_A | DS_BTN_B | DS_BTN_X));
  CHECK_EQ(in.pressedEdges(), DS_BTN_X);   // only the newly pressed one
}

TEST(source_change_is_a_fresh_baseline) {
  boot();
  Inputs in;
  in.begin(0);
  in.update(0, live(0, 0, 0, 0, DS_LINK_SRC_USB));
  in.update(1, live(0, 0, 0, DS_BTN_A, DS_LINK_SRC_USB));
  CHECK(in.pressed(DS_BTN_A));
  // The radio takes over while connected with A and B held: no edges from the switch frame.
  in.update(200, live(0, 0, 0, DS_BTN_A | DS_BTN_B, DS_LINK_SRC_BLE));
  CHECK_EQ(in.pressedEdges(), 0);
  CHECK_EQ(in.heldLevels(), DS_BTN_A | DS_BTN_B);
  in.update(201, live(0, 0, 0, DS_BTN_A | DS_BTN_B | DS_BTN_X, DS_LINK_SRC_BLE));
  CHECK_EQ(in.pressedEdges(), DS_BTN_X);
}

TEST(clear_held_resets_lockouts) {
  boot();
  Inputs in;
  in.begin(0);
  in.update(0, live());
  in.update(10, live(0, 0, 0, DS_BTN_A));
  CHECK(in.pressed(DS_BTN_A));
  in.clearHeld();
  CHECK_EQ(in.heldLevels(), 0);
  in.update(11, live());                         // baseline after the clear
  in.update(12, live(0, 0, 0, DS_BTN_A));        // inside the old lockout, but it was reset
  CHECK(in.pressed(DS_BTN_A));
}

TESTKIT_MAIN()

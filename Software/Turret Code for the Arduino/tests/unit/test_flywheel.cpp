// test_flywheel.cpp -- Flywheel (spec 3.7, architecture.md 5): the soft-start ramp (exact
// profile, monotonic, exact endpoint), direction pins per FLYWHEEL_FORWARD_IN1_HIGH, ready()
// after FLYWHEEL_SPINUP_MS, cut() immediate, coast vs brake (L298N truth table: brake needs
// ENA high with IN1 = IN2), and PWM only ever on PIN_FLY_ENA. Outputs are observed through the
// host HAL (pin levels, the ENA duty, refused PWM writes).
//
// Safety property: the ENA duty never exceeds the linear soft-start profile, not even when
// update() is handed a timestamp older than setOn() (ramp_never_jumps_ahead_of_the_profile),
// and cut() stops the drive at once with no update() (cut_is_immediate_mid_ramp).
//
// SPDX-License-Identifier: MIT
#include "Flywheel.h"
#include "hal.h"
#include "hal_host.h"
#include "testkit.h"

namespace {

void boot() {
  hal_host::reset();
  hal::begin();
}

Flywheel::Config cfgWith(bool brake, bool in1High) {
  Flywheel::Config c = Flywheel::defaults();
  c.brakeOnStop = brake;
  c.forwardIn1High = in1High;
  return c;
}

uint8_t ena() { return hal_host::pwmDuty(PIN_FLY_ENA); }
bool in1() { return hal_host::pinLevel(PIN_FLY_IN1); }
bool in2() { return hal_host::pinLevel(PIN_FLY_IN2); }

// The contract's profile: target * elapsed / ramp, rounded down, target from elapsed >= ramp.
uint32_t profile(uint32_t elapsedMs) {
  if (FLYWHEEL_RAMP_MS == 0 || elapsedMs >= FLYWHEEL_RAMP_MS) return FLYWHEEL_DUTY_255;
  return (uint32_t)FLYWHEEL_DUTY_255 * elapsedMs / FLYWHEEL_RAMP_MS;
}

}  // namespace

TEST(defaults_come_from_config_h) {
  const Flywheel::Config c = Flywheel::defaults();
  CHECK_EQ(c.duty, FLYWHEEL_DUTY_255);
  CHECK_EQ(c.rampMs, FLYWHEEL_RAMP_MS);
  CHECK_EQ(c.spinupMs, FLYWHEEL_SPINUP_MS);
  CHECK_EQ(c.brakeOnStop, FLYWHEEL_BRAKE_ON_STOP);
  CHECK_EQ(c.forwardIn1High, FLYWHEEL_FORWARD_IN1_HIGH);
  CHECK(c.brakeMs > 0);
}

TEST(begin_coasts_and_never_brakes) {
  for (int brake = 0; brake < 2; brake++) {
    boot();
    Flywheel f(cfgWith(brake == 1, true));
    f.begin();
    CHECK(!f.on());
    CHECK(!f.braking());
    CHECK_EQ(f.duty(), 0);
    CHECK_EQ(ena(), 0);
    CHECK(!in1());
    CHECK(!in2());
    CHECK(!f.ready(100000));
    f.cut();                                // cutting stopped wheels: still just coast
    CHECK(!f.braking());
    CHECK_EQ(ena(), 0);
    f.update(5000);
    CHECK_EQ(ena(), 0);
  }
}

TEST(ramp_profile_is_exact_monotonic_and_ends_at_the_target) {
  boot();
  Flywheel f;
  f.begin();
  const uint32_t t0 = 7000;
  f.setOn(true, t0);
  CHECK(f.on());
  CHECK_EQ(f.onSinceMs(), t0);
  CHECK_EQ(f.duty(), 0);                    // starts from 0, not the target
  CHECK_EQ(ena(), 0);
  uint8_t prev = 0;
  bool exact = true;
  bool monotonic = true;
  bool matchesHal = true;
  bool neverAbove = true;
  for (uint32_t t = t0; t <= t0 + FLYWHEEL_RAMP_MS + 100; t++) {
    f.update(t);
    if (f.duty() != profile(t - t0)) exact = false;
    if (f.duty() < prev) monotonic = false;
    if (ena() != f.duty()) matchesHal = false;
    if (f.duty() > FLYWHEEL_DUTY_255) neverAbove = false;
    prev = f.duty();
  }
  CHECK(exact);
  CHECK(monotonic);
  CHECK(matchesHal);
  CHECK(neverAbove);
  CHECK_EQ(f.duty(), FLYWHEEL_DUTY_255);
  CHECK_EQ(f.enaDuty(), FLYWHEEL_DUTY_255);
  // Endpoint: one ms short is below the target, the ramp time is exactly the target.
  boot();
  Flywheel g;
  g.begin();
  g.setOn(true, 0);
  g.update(FLYWHEEL_RAMP_MS - 1);
  CHECK(g.duty() < FLYWHEEL_DUTY_255);
  g.update(FLYWHEEL_RAMP_MS);
  CHECK_EQ(g.duty(), FLYWHEEL_DUTY_255);
  // Half way is half the duty (rounded down).
  boot();
  Flywheel h;
  h.begin();
  h.setOn(true, 0);
  h.update(FLYWHEEL_RAMP_MS / 2);
  CHECK_EQ(h.duty(), (uint32_t)FLYWHEEL_DUTY_255 * (FLYWHEEL_RAMP_MS / 2) / FLYWHEEL_RAMP_MS);
  // Measured ENA duty ~ FLYWHEEL_DUTY_255 / 255 (acceptance 8): the HAL holds exactly the target.
  h.update(FLYWHEEL_RAMP_MS * 3);
  CHECK_EQ(ena(), FLYWHEEL_DUTY_255);
}

TEST(ramp_never_jumps_ahead_of_the_profile) {
  boot();
  Flywheel f;
  f.begin();
  f.setOn(true, 10000);
  f.update(9990);                           // a stale timestamp (older than setOn)
  CHECK_EQ(f.duty(), 0);
  CHECK_EQ(ena(), 0);
  CHECK(!f.ready(9990));
  f.update(10000 + FLYWHEEL_RAMP_MS / 4);
  CHECK_EQ(f.duty(), profile(FLYWHEEL_RAMP_MS / 4));
}

TEST(ramp_and_spinup_across_the_millisecond_wrap) {
  boot();
  Flywheel f;
  f.begin();
  const uint32_t t0 = UINT32_MAX - FLYWHEEL_RAMP_MS / 2;
  f.setOn(true, t0);
  f.update(t0 + FLYWHEEL_RAMP_MS / 4);
  CHECK_EQ(f.duty(), profile(FLYWHEEL_RAMP_MS / 4));
  f.update(t0 + FLYWHEEL_RAMP_MS);          // wrapped past 0
  CHECK_EQ(f.duty(), FLYWHEEL_DUTY_255);
  CHECK(!f.ready(t0 + FLYWHEEL_SPINUP_MS - 1));
  CHECK(f.ready(t0 + FLYWHEEL_SPINUP_MS));
}

TEST(direction_pins_follow_forward_in1_high) {
  for (int dir = 0; dir < 2; dir++) {
    const bool in1High = dir == 0;
    boot();
    Flywheel f(cfgWith(false, in1High));
    f.begin();
    f.setOn(true, 100);
    CHECK_EQ(in1(), in1High);
    CHECK_EQ(in2(), !in1High);
    f.update(100 + FLYWHEEL_RAMP_MS);
    CHECK_EQ(in1(), in1High);               // the ramp never touches the direction
    CHECK_EQ(in2(), !in1High);
  }
}

TEST(ready_exactly_after_spinup_and_never_when_off) {
  boot();
  Flywheel f;
  f.begin();
  CHECK(!f.ready(0));
  f.setOn(true, 2000);
  for (uint32_t t = 2000; t < 2000 + FLYWHEEL_SPINUP_MS; t += 7) {
    f.update(t);
    CHECK(!f.ready(t));
  }
  f.update(2000 + FLYWHEEL_SPINUP_MS - 1);
  CHECK(!f.ready(2000 + FLYWHEEL_SPINUP_MS - 1));
  f.update(2000 + FLYWHEEL_SPINUP_MS);
  CHECK(f.ready(2000 + FLYWHEEL_SPINUP_MS));
  f.cut();
  CHECK(!f.ready(2000 + FLYWHEEL_SPINUP_MS + 1));
  // On again: the spin-up time starts over.
  f.setOn(true, 5000);
  CHECK(!f.ready(5000 + FLYWHEEL_SPINUP_MS - 1));
  CHECK(f.ready(5000 + FLYWHEEL_SPINUP_MS));
}

TEST(set_on_twice_does_not_restart_the_ramp) {
  boot();
  Flywheel f;
  f.begin();
  f.setOn(true, 1000);
  f.update(1000 + FLYWHEEL_RAMP_MS / 2);
  const uint8_t mid = f.duty();
  f.setOn(true, 1000 + FLYWHEEL_RAMP_MS / 2);
  CHECK_EQ(f.onSinceMs(), 1000u);
  CHECK_EQ(f.duty(), mid);
  CHECK_EQ(ena(), mid);
}

TEST(cut_is_immediate_mid_ramp) {
  for (int dir = 0; dir < 2; dir++) {
    boot();
    Flywheel f(cfgWith(false, dir == 0));
    f.begin();
    f.setOn(true, 0);
    f.update(FLYWHEEL_RAMP_MS / 2);
    CHECK(ena() > 0);
    f.cut();                                // no update() after this
    CHECK(!f.on());
    CHECK_EQ(f.duty(), 0);
    CHECK_EQ(ena(), 0);
    CHECK(!in1());                          // coast: IN1 = IN2 = LOW
    CHECK(!in2());
    CHECK(!f.ready(FLYWHEEL_SPINUP_MS * 2));
    CHECK(!f.braking());
    f.update(FLYWHEEL_RAMP_MS * 2);         // and nothing restarts it
    CHECK_EQ(ena(), 0);
  }
}

TEST(set_on_false_is_cut) {
  boot();
  Flywheel f;
  f.begin();
  f.setOn(true, 0);
  f.update(FLYWHEEL_RAMP_MS);
  f.setOn(false, FLYWHEEL_RAMP_MS);
  CHECK(!f.on());
  CHECK_EQ(f.duty(), 0);
  CHECK_EQ(ena(), FLYWHEEL_BRAKE_ON_STOP ? PWM_MAX : 0);
}

TEST(brake_holds_enable_high_with_both_inputs_high_then_coasts) {
  boot();
  Flywheel f(cfgWith(true, true));
  f.begin();
  f.setOn(true, 0);
  f.update(FLYWHEEL_RAMP_MS);
  f.cut();                                  // at once, before any update()
  CHECK(f.braking());
  CHECK(!f.on());
  CHECK_EQ(f.duty(), 0);                    // no drive
  CHECK(in1());
  CHECK(in2());
  CHECK_EQ(ena(), PWM_MAX);                 // the L298N brakes only with its enable high
  CHECK_EQ(f.enaDuty(), PWM_MAX);
  const uint32_t t1 = FLYWHEEL_RAMP_MS + 5;
  f.update(t1);                             // the brake's timer starts at the first update
  f.update(t1 + f.config().brakeMs - 1);
  CHECK(f.braking());
  CHECK_EQ(ena(), PWM_MAX);
  f.cut();                                  // a second cut keeps the running brake and timer
  CHECK(f.braking());
  f.update(t1 + f.config().brakeMs);
  CHECK(!f.braking());
  CHECK_EQ(ena(), 0);
  CHECK(!in1());
  CHECK(!in2());
}

TEST(brake_timed_from_set_on_false) {
  boot();
  Flywheel f(cfgWith(true, false));
  f.begin();
  f.setOn(true, 0);
  f.update(FLYWHEEL_RAMP_MS);
  f.setOn(false, 1000);
  CHECK(f.braking());
  f.update(1000 + f.config().brakeMs - 1);
  CHECK(f.braking());
  f.update(1000 + f.config().brakeMs);
  CHECK(!f.braking());
  CHECK_EQ(ena(), 0);
}

TEST(set_on_during_a_brake_releases_it_and_ramps_from_zero) {
  boot();
  Flywheel f(cfgWith(true, false));
  f.begin();
  f.setOn(true, 0);
  f.update(FLYWHEEL_RAMP_MS);
  f.cut();
  f.update(FLYWHEEL_RAMP_MS + 1);
  CHECK(f.braking());
  f.setOn(true, FLYWHEEL_RAMP_MS + 10);
  CHECK(!f.braking());
  CHECK_EQ(ena(), 0);                       // from 0, not from the brake's full enable
  CHECK(!in1());                            // forwardIn1High false: IN1 low, IN2 high
  CHECK(in2());
  f.update(FLYWHEEL_RAMP_MS + 10 + FLYWHEEL_RAMP_MS / 2);
  CHECK_EQ(f.duty(), profile(FLYWHEEL_RAMP_MS / 2));
}

TEST(zero_ramp_and_zero_spinup_take_effect_at_once) {
  boot();
  Flywheel::Config c = Flywheel::defaults();
  c.rampMs = 0;
  c.spinupMs = 0;
  Flywheel f(c);
  f.begin();
  f.setOn(true, 42);
  CHECK_EQ(f.duty(), c.duty);
  CHECK_EQ(ena(), c.duty);
  CHECK(f.ready(42));
}

TEST(cut_writes_the_stop_outputs_again_even_if_they_were_disturbed) {
  // cut() is the safety stop: it never trusts what it last wrote.
  for (int brake = 0; brake < 2; brake++) {
    boot();
    Flywheel f(cfgWith(brake == 1, true));
    f.begin();
    hal::pwmWrite(PIN_FLY_ENA, 77);           // something else touched the outputs
    hal::writePin(PIN_FLY_IN1, true);
    hal::writePin(PIN_FLY_IN2, false);
    f.cut();                                  // stopped wheels: coast in either mode
    CHECK_EQ(ena(), 0);
    CHECK(!in1());
    CHECK(!in2());
  }
  boot();
  Flywheel f(cfgWith(true, true));
  f.begin();
  f.setOn(true, 0);
  f.update(FLYWHEEL_RAMP_MS);
  f.cut();
  f.update(FLYWHEEL_RAMP_MS + 1);
  hal::pwmWrite(PIN_FLY_ENA, 0);              // disturbed mid-brake
  hal::writePin(PIN_FLY_IN2, false);
  f.cut();
  CHECK(f.braking());
  CHECK_EQ(ena(), PWM_MAX);
  CHECK(in1());
  CHECK(in2());
  f.update(FLYWHEEL_RAMP_MS + 1 + f.config().brakeMs);   // the original timer still ends it
  CHECK(!f.braking());
  CHECK_EQ(ena(), 0);
}

TEST(pwm_is_only_ever_written_to_the_enable_pin) {
  for (int brake = 0; brake < 2; brake++) {
    boot();
    Flywheel f(cfgWith(brake == 1, true));
    f.begin();
    f.setOn(true, 0);
    for (uint32_t t = 0; t <= FLYWHEEL_RAMP_MS; t += 3) f.update(t);
    f.cut();
    f.update(FLYWHEEL_RAMP_MS + 1);
    f.update(FLYWHEEL_RAMP_MS + 1 + f.config().brakeMs);
    f.setOn(true, 50000);
    f.setOn(false, 50001);
    CHECK_EQ(hal_host::pwmRefused(), 0u);
  }
}

TESTKIT_MAIN()

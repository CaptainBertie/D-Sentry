// test_motion.cpp -- the real Motion module (firmware-ra4m1/dsentry/src/Motion.cpp) driven
// through the host HAL (sim/hal_host.cpp): virtual time, the stepper model (exact periods,
// buffered rate changes, the STEP_MIN_RATE_HZ floor, bounds in the "interrupt") and the tilt's
// physical bottom stop, where steps are lost exactly as a stepper rams a panel.
//
// What is proved, against spec 3.3 / 3.5 / 11 items 2, 4, 5 and architecture.md 5:
//   - degrees <-> steps with *_STEPS_PER_DEG and *_DIR_INVERT (both signs);
//   - velocity mode: proportional, capped at *_MAX_DEG_PER_S, accel-limited both ways, a STOP
//     decelerates at the accel limit, a stick below the step floor does not move;
//   - PROPERTY (randomised stick sequences, long runs, jittery loop): the speed never exceeds
//     *_MAX_DEG_PER_S, the change between two planner ticks never exceeds the accel limit, no
//     reversal skips the stopped tick, the step counter never leaves an active soft limit and
//     the interrupt's hard bound never has to refuse a step (KEY SAFETY PROPERTY: remove the
//     soft-limit cap from Motion::tickAxis and random_sticks_never_break_speed_accel_or_limits
//     fails, ~80,000 steps refused at the bound; drop the tilt bound after calibration and
//     tilt_hard_bounds_are_set_after_calibration fails);
//   - full speed starts braking at least SOFT_LIMIT_SLOWDOWN_DEG before a limit;
//   - go-home cruises at HOME_SPEED_DEG_PER_S and stops on the stored step; a stick cancels it;
//   - position moves of any length land exactly on their target step, never stall short;
//   - CAL_RAM from several heights (including already on the stop) ends with tilt 0 exactly
//     TILT_CAL_BACKOFF_DEG above the real stop; CAL_STALL_SENSE stops on the event and faults
//     after TILT_CAL_TIMEOUT_MS; the bounds are on afterwards;
//   - drivers: enabled before the first step, held or released per the idle policy;
//   - stall back-off: STALL_BACKOFF_DEG against the motion of each axis that was moving.
//
// Tolerance: velocities are compared within kEps = 1e-3 deg/s (float rounding of deg/s values;
// the planner's own changes are multiples of accel / CONTROL_LOOP_HZ).
//
// SPDX-License-Identifier: MIT
#include <math.h>
#include <stdint.h>

#include "Motion.h"
#include "StallDetector.h"
#include "hal.h"
#include "hal_host.h"
#include "testkit.h"

namespace {

constexpr float kEps = 1e-3f;
constexpr float kT = 1.0f / CONTROL_LOOP_HZ;
constexpr uint32_t kTickUs = US_PER_S / CONTROL_LOOP_HZ;

float spd(Axis a) { return a == AXIS_PAN ? PAN_STEPS_PER_DEG : TILT_STEPS_PER_DEG; }
bool inv(Axis a) { return a == AXIS_PAN ? PAN_DIR_INVERT : TILT_DIR_INVERT; }
float vmax(Axis a) { return a == AXIS_PAN ? PAN_MAX_DEG_PER_S : TILT_MAX_DEG_PER_S; }
float accel(Axis a) { return a == AXIS_PAN ? PAN_ACCEL_DEG_PER_S2 : TILT_ACCEL_DEG_PER_S2; }
float stepDeg(Axis a) { return 1.0f / spd(a); }

// The jumps the planner cannot avoid (the HAL's floor and the final-approach creep) must fit in
// one tick's accel budget, or the accel property below would be claimed for a config where it
// cannot hold.
constexpr float kCreepStepsPerS = MOTION_CREEP_STEPS_PER_S;
static_assert(STEP_MIN_RATE_HZ / TILT_STEPS_PER_DEG <= TILT_ACCEL_DEG_PER_S2 / CONTROL_LOOP_HZ, "floor");
static_assert(STEP_MIN_RATE_HZ / PAN_STEPS_PER_DEG <= PAN_ACCEL_DEG_PER_S2 / CONTROL_LOOP_HZ, "floor");
static_assert(kCreepStepsPerS / TILT_STEPS_PER_DEG <= TILT_ACCEL_DEG_PER_S2 / CONTROL_LOOP_HZ, "creep");
static_assert(kCreepStepsPerS / PAN_STEPS_PER_DEG <= PAN_ACCEL_DEG_PER_S2 / CONTROL_LOOP_HZ, "creep");

// Turret-frame step counter / true mechanism position.
int32_t tsteps(Axis a) { return inv(a) ? -hal::stepPosition(a) : hal::stepPosition(a); }
int32_t tstepsFor(Axis a, float deg) {
  const float s = deg * spd(a);
  return (int32_t)(s < 0.0f ? s - 0.5f : s + 0.5f);
}
// Velocity the HAL is running, deg/s, turret frame.
float halVel(Axis a) {
  const float v = hal::stepRate(a) / spd(a);
  return inv(a) ? -v : v;
}

struct Rng {
  uint64_t s;
  explicit Rng(uint64_t seed) : s(seed * 0x9E3779B97F4A7C15ull + 0x1234567ull) {}
  uint32_t next() {
    s ^= s << 13;
    s ^= s >> 7;
    s ^= s << 17;
    return (uint32_t)(s >> 32);
  }
  float uniform(float lo, float hi) { return lo + (hi - lo) * (float)(next() >> 8) / 16777216.0f; }
  uint32_t range(uint32_t lo, uint32_t hi) { return lo + next() % (hi - lo + 1); }
};

// Invariants checked on every loop pass (limits) and every planner tick (speed, accel, sign).
struct Watch {
  uint32_t lastTicks = 0;
  uint64_t lastTickUs = 0;
  float lastV[AXIS_COUNT] = {};
  float maxSpeed[AXIS_COUNT] = {};
  float maxDv[AXIS_COUNT] = {};
  uint32_t ticks = 0;
  uint32_t speedBad = 0;
  uint32_t accelBad = 0;
  uint32_t reversalBad = 0;
  uint32_t limitBad = 0;
  uint32_t halMismatch = 0;
  uint32_t nearLimit = 0;   // ticks spent within 1 degree of an active limit (coverage)

  bool tickTimeKnown = false;   // start() is not a tick: the first spacing is unknown

  void start(const Motion& m) {
    lastTicks = m.ticks();
    lastTickUs = hal_host::timeUs();
    tickTimeKnown = false;
    for (uint8_t i = 0; i < AXIS_COUNT; i++) lastV[i] = m.velocityDeg((Axis)i);
  }
  void observe(const Motion& m) {
    for (uint8_t i = 0; i < AXIS_COUNT; i++) {
      const Axis a = (Axis)i;
      if (!m.limitsActive(a)) continue;
      const int32_t s = tsteps(a);
      if (s < tstepsFor(a, m.limitMinDeg(a)) || s > tstepsFor(a, m.limitMaxDeg(a))) limitBad++;
    }
    if (m.ticks() == lastTicks) return;
    const uint64_t now = hal_host::timeUs();
    const float dtS = (float)(now - lastTickUs) / (float)US_PER_S;
    ticks++;
    for (uint8_t i = 0; i < AXIS_COUNT; i++) {
      const Axis a = (Axis)i;
      const float v = m.velocityDeg(a);
      if (fabsf(v - halVel(a)) > kEps) halMismatch++;
      if (fabsf(v) > vmax(a) + kEps) speedBad++;
      if (fabsf(v) > maxSpeed[i]) maxSpeed[i] = fabsf(v);
      const float dv = fabsf(v - lastV[i]);
      if (dv > maxDv[i]) maxDv[i] = dv;
      if (dv > accel(a) * kT + kEps || (tickTimeKnown && dv > accel(a) * dtS + kEps)) accelBad++;
      if (v * lastV[i] < 0.0f) reversalBad++;
      if (m.limitsActive(a)) {
        const float p = m.positionDeg(a);
        if (p - m.limitMinDeg(a) < 1.0f || m.limitMaxDeg(a) - p < 1.0f) nearLimit++;
      }
      lastV[i] = v;
    }
    lastTicks = m.ticks();
    lastTickUs = now;
    tickTimeKnown = true;
  }
  bool clean() const {
    return speedBad == 0 && accelBad == 0 && reversalBad == 0 && limitBad == 0 && halMismatch == 0;
  }
};

// Each invariant on its own line, so a failure names the one that broke.
#define CHECK_WATCH(w)              \
  do {                              \
    CHECK_EQ((w).speedBad, 0u);     \
    CHECK_EQ((w).accelBad, 0u);     \
    CHECK_EQ((w).reversalBad, 0u);  \
    CHECK_EQ((w).limitBad, 0u);     \
    CHECK_EQ((w).halMismatch, 0u);  \
  } while (0)

void step(Motion& m, uint32_t us, Watch* w) {
  hal_host::advanceUs(us);
  m.update(hal::nowMs(), hal::nowUs());
  if (w) w->observe(m);
}

void runFor(Motion& m, uint64_t us, Watch* w = nullptr, uint32_t loopUs = 1000) {
  const uint64_t end = hal_host::timeUs() + us;
  while (hal_host::timeUs() < end) step(m, loopUs, w);
}

template <typename Pred>
bool runUntil(Motion& m, Pred done, uint64_t maxUs, Watch* w = nullptr, uint32_t loopUs = 1000) {
  const uint64_t end = hal_host::timeUs() + maxUs;
  while (hal_host::timeUs() < end) {
    step(m, loopUs, w);
    if (done()) return true;
  }
  return false;
}

struct Rig {
  StallDetector stall;
  Motion m;
  explicit Rig(bool timerFailure = false) {
    hal_host::reset();
    hal_host::setBeginTimerFailure(timerFailure);
    hal::begin();
    stall.begin(hal::nowUs());
    m.begin(stall);
  }
};

// The tilt's bottom stop `heightDeg` below where the barrel is now. Returns the stop (true
// steps). "Down" in motor steps is -1, or +1 with TILT_DIR_INVERT (hal_host.h).
int64_t placeStop(float heightDeg) {
  const int64_t down = TILT_DIR_INVERT ? 1 : -1;
  const int64_t stop = hal_host::trueSteps(AXIS_TILT) + down * (int64_t)tstepsFor(AXIS_TILT, heightDeg);
  hal_host::setTiltBottomStop(true, stop);
  return stop;
}
// How far the barrel truly is above the stop, in steps.
int64_t aboveStop(int64_t stop) {
  const int64_t down = TILT_DIR_INVERT ? 1 : -1;
  return (hal_host::trueSteps(AXIS_TILT) - stop) * -down;
}

bool finishCal(Motion& m, Watch* w = nullptr) {
  return runUntil(m, [&] { return m.calibrationStatus() != Motion::CalStatus::RUNNING; },
                  (uint64_t)(TILT_CAL_MAX_TRAVEL_DEG / TILT_CAL_SPEED_DEG_PER_S * 2.0f + 5.0f) * US_PER_S, w);
}

// A calibrated rig: the tilt starts 20 degrees above its stop, CAL_RAM runs.
int64_t calibrate(Rig& r) {
  const int64_t stop = placeStop(20.0f);
  r.m.startCalibration(hal::nowMs(), CAL_RAM);
  finishCal(r.m);
  return stop;
}

bool settled(const Motion& m) { return !m.isMoving() && !m.positionMoveActive() && !m.backingOff(); }

}  // namespace

// ---- conversions -------------------------------------------------------------------------------

TEST(conversions_use_steps_per_degree_and_inversion) {
  // 1.8 deg motor, 1/8 microsteps, 5:1 pan: 22.22 steps/deg -> 90 deg = 2000 steps (spec 3.3).
  CHECK_EQ(Motion::degToStepsWith(90.0f, PAN_STEPS_PER_DEG, false), 2000);
  CHECK_EQ(Motion::degToStepsWith(90.0f, PAN_STEPS_PER_DEG, true), -2000);
  CHECK_EQ(Motion::degToStepsWith(60.0f, TILT_STEPS_PER_DEG, false), 1067);   // 1066.67
  CHECK_NEAR(Motion::stepsToDegWith(-2000, PAN_STEPS_PER_DEG, true), 90.0f, 1e-4f);
  CHECK_NEAR(Motion::stepsToDegWith(2000, PAN_STEPS_PER_DEG, false), 90.0f, 1e-4f);
  // Round half away from zero, symmetric.
  CHECK_EQ(Motion::degToStepsWith(1.5f, 1.0f, false), 2);
  CHECK_EQ(Motion::degToStepsWith(-1.5f, 1.0f, false), -2);
  CHECK_EQ(Motion::degToStepsWith(1.49f, 1.0f, false), 1);
  CHECK_EQ(Motion::degToStepsWith(-1.49f, 1.0f, true), 1);
  // The config wiring.
  CHECK_EQ(Motion::degToSteps(AXIS_PAN, 33.3f), Motion::degToStepsWith(33.3f, PAN_STEPS_PER_DEG, PAN_DIR_INVERT));
  CHECK_EQ(Motion::degToSteps(AXIS_TILT, -7.7f),
           Motion::degToStepsWith(-7.7f, TILT_STEPS_PER_DEG, TILT_DIR_INVERT));
  // Round trip over a range of steps, both axes, both signs of inversion.
  int bad = 0;
  for (int32_t s = -5000; s <= 5000; s += 7) {
    for (int k = 0; k < 2; k++) {
      const bool iv = k == 1;
      if (Motion::degToStepsWith(Motion::stepsToDegWith(s, PAN_STEPS_PER_DEG, iv), PAN_STEPS_PER_DEG, iv) != s) bad++;
      if (Motion::degToStepsWith(Motion::stepsToDegWith(s, TILT_STEPS_PER_DEG, iv), TILT_STEPS_PER_DEG, iv) != s) bad++;
    }
  }
  CHECK_EQ(bad, 0);
}

TEST(a_move_lands_in_motor_steps_with_the_configured_direction) {
  Rig r;
  CHECK(r.m.moveTo(AXIS_PAN, 45.0f));
  CHECK(runUntil(r.m, [&] { return !r.m.positionMoveActive(); }, 5 * US_PER_S));
  CHECK_EQ(hal::stepPosition(AXIS_PAN), Motion::degToSteps(AXIS_PAN, 45.0f));
  CHECK_EQ(hal_host::trueSteps(AXIS_PAN), (int64_t)Motion::degToSteps(AXIS_PAN, 45.0f));
  // + turret = DIR high unless inverted (hal.h: + = DIR pin high).
  CHECK_EQ(hal::stepPosition(AXIS_PAN) > 0, !PAN_DIR_INVERT);
  CHECK_NEAR(r.m.positionDeg(AXIS_PAN), 45.0f, stepDeg(AXIS_PAN));
}

TEST(begin_leaves_everything_stopped_and_uncalibrated) {
  Rig r;
  CHECK(!r.m.driversEnabled());
  CHECK(!r.m.isMoving());
  CHECK_EQ(r.m.movingAxes(), 0);
  CHECK(!r.m.calibrated());
  CHECK(r.m.calibrationStatus() == Motion::CalStatus::IDLE);
  CHECK(!r.m.limitsActive(AXIS_TILT));
  CHECK_EQ(r.m.limitsActive(AXIS_PAN), PAN_LIMITS_ENABLED);
  CHECK_NEAR(r.m.limitMaxDeg(AXIS_TILT), TILT_RANGE_DEG, 0.0f);
  CHECK_NEAR(r.m.limitMinDeg(AXIS_PAN), PAN_MIN_DEG, 0.0f);
  CHECK_NEAR(r.m.homeDeg(AXIS_PAN), 0.0f, 0.0f);   // spec 3.3: home defaults to (0, 0)
  CHECK_NEAR(r.m.homeDeg(AXIS_TILT), 0.0f, 0.0f);
  CHECK(!r.m.homing());
  runFor(r.m, US_PER_S);
  CHECK_EQ(hal_host::totalSteps(AXIS_PAN), 0u);
  CHECK_EQ(hal_host::totalSteps(AXIS_TILT), 0u);
  CHECK(!r.m.driversEnabled());   // nothing asked to move: EN stays released
}

// ---- velocity mode -----------------------------------------------------------------------------

TEST(planner_ticks_at_control_loop_hz) {
  Rig r;
  runFor(r.m, US_PER_S, nullptr, 1000);
  CHECK_EQ(r.m.ticks(), (uint32_t)CONTROL_LOOP_HZ);
  // A loop slower than the tick: one tick per pass, never a burst after a gap.
  const uint32_t before = r.m.ticks();
  hal_host::advanceUs(5 * kTickUs);
  r.m.update(hal::nowMs(), hal::nowUs());
  CHECK_EQ(r.m.ticks(), before + 1);
  r.m.update(hal::nowMs(), hal::nowUs());
  CHECK_EQ(r.m.ticks(), before + 1);
}

TEST(full_stick_ramps_at_the_accel_limit_to_max_speed_and_no_further) {
  Rig r;
  Watch w;
  w.start(r.m);
  r.m.setVelocity(1.0f, 1.0f);   // tilt uncalibrated: no limits yet
  runFor(r.m, US_PER_S, &w);
  CHECK_WATCH(w);
  CHECK_NEAR(r.m.velocityDeg(AXIS_PAN), PAN_MAX_DEG_PER_S, kEps);
  CHECK_NEAR(r.m.velocityDeg(AXIS_TILT), TILT_MAX_DEG_PER_S, kEps);
  CHECK_NEAR(w.maxDv[AXIS_PAN], PAN_ACCEL_DEG_PER_S2 * kT, kEps);   // the limiter was the limit
  CHECK(hal::stepRate(AXIS_PAN) <= STEP_MAX_RATE_HZ);
  CHECK(r.m.driversEnabled());
  CHECK_EQ(hal_host::lostSteps(AXIS_PAN), 0u);    // EN went low before the first step
  CHECK_EQ(hal_host::lostSteps(AXIS_TILT), 0u);
  // Ramp time: vmax / accel, to within a tick (90 / 400 = 0.225 s).
  Rig r2;
  r2.m.setVelocity(1.0f, 0.0f);
  const uint64_t t0 = hal_host::timeUs();
  CHECK(runUntil(r2.m, [&] { return r2.m.velocityDeg(AXIS_PAN) >= PAN_MAX_DEG_PER_S - kEps; }, US_PER_S));
  CHECK_NEAR((double)(hal_host::timeUs() - t0) / US_PER_S, PAN_MAX_DEG_PER_S / PAN_ACCEL_DEG_PER_S2, kT);
}

TEST(stick_speed_is_proportional_and_the_steps_follow) {
  Rig r;
  r.m.setVelocity(0.5f, -0.25f);
  runFor(r.m, US_PER_S);   // ramps done
  CHECK_NEAR(r.m.velocityDeg(AXIS_PAN), 0.5f * PAN_MAX_DEG_PER_S, kEps);
  CHECK_NEAR(r.m.velocityDeg(AXIS_TILT), -0.25f * TILT_MAX_DEG_PER_S, kEps);
  const int32_t p0 = tsteps(AXIS_PAN), t0 = tsteps(AXIS_TILT);
  runFor(r.m, 2 * US_PER_S);
  CHECK_NEAR((tsteps(AXIS_PAN) - p0) / 2.0f, 0.5f * PAN_MAX_DEG_PER_S * PAN_STEPS_PER_DEG, 1.0f);
  CHECK_NEAR((tsteps(AXIS_TILT) - t0) / 2.0f, -0.25f * TILT_MAX_DEG_PER_S * TILT_STEPS_PER_DEG, 1.0f);
  CHECK_EQ(r.m.movingAxes(), axisBit(AXIS_PAN) | axisBit(AXIS_TILT));
  // Fractions are clamped; NaN means stop.
  r.m.setVelocity(7.0f, NAN);
  runFor(r.m, US_PER_S);
  CHECK_NEAR(r.m.velocityDeg(AXIS_PAN), PAN_MAX_DEG_PER_S, kEps);
  CHECK_NEAR(r.m.velocityDeg(AXIS_TILT), 0.0f, 0.0f);
}

TEST(stop_decelerates_at_the_accel_limit_and_reversal_passes_through_a_stopped_tick) {
  Rig r;
  Watch w;
  r.m.setVelocity(1.0f, 0.0f);
  runFor(r.m, US_PER_S);
  w.start(r.m);
  const float p0 = r.m.positionDeg(AXIS_PAN);
  r.m.stop();
  CHECK(r.m.mode(AXIS_PAN) == Motion::Mode::STOP);
  CHECK(runUntil(r.m, [&] { return !r.m.isMoving(); }, US_PER_S, &w));
  CHECK_WATCH(w);
  CHECK_NEAR(w.maxDv[AXIS_PAN], PAN_ACCEL_DEG_PER_S2 * kT, kEps);
  // Stopping distance ~ v^2 / 2a (10.1 deg), less the discrete ramp's last half-tick.
  const float v = PAN_MAX_DEG_PER_S;
  CHECK_NEAR(r.m.positionDeg(AXIS_PAN) - p0, v * v / (2.0f * PAN_ACCEL_DEG_PER_S2), v * kT);
  // Full reversal: through a stopped tick, accel-limited throughout.
  r.m.setVelocity(1.0f, 0.0f);
  runFor(r.m, US_PER_S);
  w.start(r.m);
  r.m.setVelocity(-1.0f, 0.0f);
  bool sawZero = false;
  CHECK(runUntil(r.m, [&] {
          if (r.m.velocityDeg(AXIS_PAN) == 0.0f) sawZero = true;
          return r.m.velocityDeg(AXIS_PAN) <= -PAN_MAX_DEG_PER_S + kEps;
        }, US_PER_S, &w));
  CHECK(sawZero);
  CHECK_WATCH(w);
}

TEST(a_stick_below_the_step_floor_does_not_move) {
  Rig r;
  const float floorFrac = STEP_MIN_RATE_HZ / PAN_STEPS_PER_DEG / PAN_MAX_DEG_PER_S;   // 1.25 %
  r.m.setVelocity(floorFrac * 0.9f, 0.0f);
  runFor(r.m, US_PER_S);
  CHECK_EQ(hal_host::totalSteps(AXIS_PAN), 0u);
  CHECK_NEAR(r.m.velocityDeg(AXIS_PAN), 0.0f, 0.0f);
  CHECK(!r.m.isMoving());
  r.m.setVelocity(floorFrac * 1.2f, 0.0f);
  runFor(r.m, US_PER_S);
  CHECK(r.m.isMoving());
  CHECK(hal_host::totalSteps(AXIS_PAN) > 0u);
}

// ---- the property test (KEY SAFETY PROPERTY) ---------------------------------------------------

// Randomised stick sequences over long virtual runs with a jittery loop, after a real CAL_RAM
// calibration and with pan limits switched on: speed, accel, sign changes and soft limits hold
// on every tick, and the interrupt's hard bounds never refuse a step.
TEST(random_sticks_never_break_speed_accel_or_limits) {
  for (uint64_t seed = 1; seed <= 4; seed++) {
    Rig r;
    Rng rng(seed);
    const int64_t stop = calibrate(r);
    CHECK(r.m.calibrated());
    r.m.setPanLimits(true, PAN_MIN_DEG, PAN_MAX_DEG);
    const uint32_t lostAfterCal = hal_host::lostSteps(AXIS_TILT);
    const uint32_t refusalsBefore = hal_host::totalRefusals(AXIS_PAN) + hal_host::totalRefusals(AXIS_TILT);
    Watch w;
    w.start(r.m);
    const uint64_t end = hal_host::timeUs() + 150ull * US_PER_S;
    while (hal_host::timeUs() < end) {
      float s[AXIS_COUNT];
      for (float& x : s) {
        const uint32_t kind = rng.range(0, 5);
        x = kind == 0 ? 1.0f : kind == 1 ? -1.0f : kind == 2 ? 0.0f
          : kind == 3 ? rng.uniform(-0.05f, 0.05f) : rng.uniform(-1.0f, 1.0f);
      }
      r.m.setVelocity(s[AXIS_PAN], s[AXIS_TILT]);
      const uint64_t hold = (uint64_t)rng.range(20, 2500) * US_PER_MS;
      const uint64_t until = hal_host::timeUs() + hold;
      while (hal_host::timeUs() < until) step(r.m, rng.range(40, 700), &w);
    }
    CHECK_EQ(w.speedBad, 0u);
    CHECK_EQ(w.accelBad, 0u);
    CHECK_EQ(w.reversalBad, 0u);
    CHECK_EQ(w.limitBad, 0u);
    CHECK_EQ(w.halMismatch, 0u);
    CHECK_EQ(hal_host::totalRefusals(AXIS_PAN) + hal_host::totalRefusals(AXIS_TILT), refusalsBefore);
    CHECK_EQ(hal_host::lostSteps(AXIS_TILT), lostAfterCal);   // never pushed into the panel
    CHECK(aboveStop(stop) >= (int64_t)tstepsFor(AXIS_TILT, TILT_CAL_BACKOFF_DEG));
    CHECK(w.nearLimit > 100u);   // the run really lived at the limits
    CHECK(w.maxSpeed[AXIS_PAN] > PAN_MAX_DEG_PER_S - kEps);
    CHECK(w.maxSpeed[AXIS_TILT] > TILT_MAX_DEG_PER_S - kEps);
  }
}

// Pan limits off (the default): pan is unbounded, both ways.
TEST(pan_without_limits_turns_freely) {
  if (PAN_LIMITS_ENABLED) return;
  Rig r;
  r.m.setVelocity(1.0f, 0.0f);
  runFor(r.m, 5 * US_PER_S);   // 5 s at 90 deg/s = 400+ degrees
  CHECK(r.m.positionDeg(AXIS_PAN) > 400.0f);
  CHECK_EQ(hal_host::totalRefusals(AXIS_PAN), 0u);
}

// ---- soft-limit braking -----------------------------------------------------------------------

// Drives one axis at full stick into a limit; returns the distance from the limit at the first
// tick where the speed fell while the stick was still held, and where it came to rest.
struct BrakeResult {
  float brakeStartGap = -1.0f;
  float restGap = -1.0f;
  Watch w;
  uint32_t refusals = 0;
};

BrakeResult brakeInto(Rig& r, Axis a, float dir, uint32_t loopUs) {
  BrakeResult out;
  Watch& w = out.w;
  w.start(r.m);
  const uint32_t ref0 = hal_host::totalRefusals(a);
  const float limit = dir > 0.0f ? r.m.limitMaxDeg(a) : r.m.limitMinDeg(a);
  r.m.setVelocity(a == AXIS_PAN ? dir : 0.0f, a == AXIS_TILT ? dir : 0.0f);
  float prev = 0.0f;
  bool atFull = false;
  const uint64_t end = hal_host::timeUs() + 20ull * US_PER_S;
  while (hal_host::timeUs() < end) {
    const uint32_t t = r.m.ticks();
    step(r.m, loopUs, &w);
    if (r.m.ticks() == t) continue;
    const float v = fabsf(r.m.velocityDeg(a));
    if (v >= vmax(a) - kEps) atFull = true;
    if (atFull && out.brakeStartGap < 0.0f && v < prev - kEps)
      out.brakeStartGap = fabsf(limit - r.m.positionDeg(a));
    prev = v;
    if (atFull && v == 0.0f) break;
  }
  runFor(r.m, US_PER_S / 2, &w, loopUs);   // stick still held against the limit
  out.restGap = fabsf(limit - r.m.positionDeg(a));
  out.refusals = hal_host::totalRefusals(a) - ref0;
  r.m.setVelocity(0.0f, 0.0f);
  return out;
}

TEST(full_speed_brakes_at_least_slowdown_degrees_before_every_limit) {
  for (uint32_t loopUs : {200u, 1000u, 2700u}) {   // fast board loop .. a sluggish one
    Rig r;
    calibrate(r);
    r.m.setPanLimits(true, PAN_MIN_DEG, PAN_MAX_DEG);
    const BrakeResult up = brakeInto(r, AXIS_TILT, 1.0f, loopUs);
    const BrakeResult down = brakeInto(r, AXIS_TILT, -1.0f, loopUs);
    const BrakeResult right = brakeInto(r, AXIS_PAN, 1.0f, loopUs);
    const BrakeResult left = brakeInto(r, AXIS_PAN, -1.0f, loopUs);
    for (const BrakeResult* b : {&up, &down, &right, &left}) {
      CHECK(b->brakeStartGap >= SOFT_LIMIT_SLOWDOWN_DEG);
      CHECK(b->restGap >= 0.0f);
      CHECK(b->restGap < 0.25f);   // and still reaches the limit (within ~4 steps)
      CHECK_WATCH(b->w);
      CHECK_EQ(b->refusals, 0u);
    }
  }
  // The braking accel itself: min(accel, vmax^2 / (2 * slowdown)), never above the accel limit.
  for (Axis a : {AXIS_PAN, AXIS_TILT}) {
    const float ab = Motion::brakeAccel(a);
    CHECK(ab <= accel(a));
    CHECK(vmax(a) * vmax(a) / (2.0f * ab) >= SOFT_LIMIT_SLOWDOWN_DEG - kEps);
  }
}

TEST(brake_cap_is_the_discrete_sqrt_2ad) {
  // Zero / negative distance: no speed. Large distance: -> sqrt(2 a d) from below.
  CHECK_NEAR(Motion::brakeCap(0.0f, 300.0f), 0.0f, 0.0f);
  CHECK_NEAR(Motion::brakeCap(-1.0f, 300.0f), 0.0f, 0.0f);
  const float d = 6.0f, a = 300.0f;
  const float cap = Motion::brakeCap(d, a);
  CHECK(cap < sqrtf(2.0f * a * d));
  CHECK_NEAR(cap, sqrtf(2.0f * a * d), a * kT);
  // Held for a tick, then cut by a*T per tick: the distance covered is exactly d.
  float v = cap, dist = 0.0f;
  while (v > 0.0f) {
    dist += v * kT;
    v -= a * kT;
  }
  // A speed that is not a whole number of a*T steps overshoots the formula by at most
  // a T^2 / 8 (0.004 deg here, a fraction of a step; MOTION_BRAKE_MARGIN_STEPS covers it).
  CHECK(dist <= d + a * kT * kT / 8.0f + 1e-4f);
}

TEST(tilt_hard_bounds_are_set_after_calibration) {
  // The interrupt's backstop, bypassing the planner: steps below 0 are refused.
  Rig r;
  calibrate(r);
  CHECK_EQ(hal::stepPosition(AXIS_TILT), 0);
  const uint32_t ref0 = hal_host::totalRefusals(AXIS_TILT);
  hal::stepSetRate(AXIS_TILT, TILT_DIR_INVERT ? 500.0f : -500.0f);   // straight down
  hal_host::advanceMs(200);
  hal::stepSetRate(AXIS_TILT, 0.0f);
  CHECK_EQ(hal::stepPosition(AXIS_TILT), 0);
  CHECK(hal_host::totalRefusals(AXIS_TILT) > ref0);
}

// ---- position moves and home ---------------------------------------------------------------------

TEST(go_home_cruises_at_home_speed_and_stops_on_the_stored_step) {
  Rig r;
  calibrate(r);
  r.m.moveTo(30.0f, 20.0f);
  CHECK(runUntil(r.m, [&] { return settled(r.m); }, 5 * US_PER_S));
  r.m.setHomeHere();
  CHECK_NEAR(r.m.homeDeg(AXIS_PAN), 30.0f, stepDeg(AXIS_PAN));
  CHECK_NEAR(r.m.homeDeg(AXIS_TILT), 20.0f, stepDeg(AXIS_TILT));
  const int32_t homePan = tsteps(AXIS_PAN), homeTilt = tsteps(AXIS_TILT);
  // Away with the stick.
  // Pan about 140 deg (under half a turn, so PAN_HOME_SHORTEST_PATH comes back the same way).
  const uint64_t panUs = (uint64_t)(140.0f / PAN_MAX_DEG_PER_S * US_PER_S);
  r.m.setVelocity(-1.0f, 1.0f);
  runFor(r.m, panUs);
  r.m.setVelocity(0.0f, 1.0f);
  if (panUs < 1500 * US_PER_MS) runFor(r.m, 1500 * US_PER_MS - panUs);
  r.m.setVelocity(0.0f, 0.0f);
  CHECK(runUntil(r.m, [&] { return settled(r.m); }, 5 * US_PER_S));
  CHECK(r.m.positionDeg(AXIS_PAN) < -60.0f);
  // Well away from home (20 deg): past halfway from home to the top of the tilt range.
  CHECK(r.m.positionDeg(AXIS_TILT) > 20.0f + (TILT_RANGE_DEG - 20.0f) / 2.0f);
  // Home.
  Watch w;
  w.start(r.m);
  CHECK(r.m.goHome());
  CHECK(r.m.homing());
  bool reversed = false;
  float lastPan = 0.0f, lastTilt = 0.0f;
  CHECK(runUntil(r.m, [&] {
          const float vp = r.m.velocityDeg(AXIS_PAN), vt = r.m.velocityDeg(AXIS_TILT);
          if (vp * lastPan < 0.0f || vt * lastTilt < 0.0f) reversed = true;
          if (vp != 0.0f) lastPan = vp;
          if (vt != 0.0f) lastTilt = vt;
          return !r.m.homing();
        }, 10 * US_PER_S, &w));
  CHECK_WATCH(w);
  CHECK(!reversed);   // no overshoot and come-back
  CHECK_NEAR(w.maxSpeed[AXIS_PAN], HOME_SPEED_DEG_PER_S, kEps);   // spec 11.5: at HOME speed
  CHECK_NEAR(w.maxSpeed[AXIS_TILT], HOME_SPEED_DEG_PER_S, kEps);
  CHECK_EQ(tsteps(AXIS_PAN), homePan);   // exactly the stored step
  CHECK_EQ(tsteps(AXIS_TILT), homeTilt);
  CHECK(!r.m.isMoving());
  CHECK(r.m.mode(AXIS_PAN) == Motion::Mode::STOP);
  CHECK_EQ(hal_host::totalRefusals(AXIS_TILT), 0u);
}

// Owner, 2026-11-03: nothing crosses the pan joint, so go-home takes the short way round
// (PAN_HOME_SHORTEST_PATH): 370 deg from home turns 10 deg, not 370 back.
TEST(nearest_turn_is_at_most_half_a_turn_away) {
  CHECK_NEAR(Motion::nearestTurn(370.0f, 0.0f), 360.0f, kEps);
  CHECK_NEAR(Motion::nearestTurn(190.0f, 0.0f), 360.0f, kEps);    // +170, not -190
  CHECK_NEAR(Motion::nearestTurn(170.0f, 0.0f), 0.0f, kEps);      // -170
  CHECK_NEAR(Motion::nearestTurn(-190.0f, 0.0f), -360.0f, kEps);  // -170, not +190
  CHECK_NEAR(Motion::nearestTurn(180.0f, 0.0f), 360.0f, kEps);    // a tie goes the positive way
  CHECK_NEAR(Motion::nearestTurn(5.0f, 725.0f), 5.0f, kEps);      // already there, modulo turns
  CHECK_NEAR(Motion::nearestTurn(30.0f, 32.9f), 32.9f, kEps);     // ordinary case unchanged
}

TEST(go_home_takes_the_short_way_round_on_an_unlimited_pan) {
  if (!PAN_HOME_SHORTEST_PATH || PAN_LIMITS_ENABLED) return;   // the rule is off in this build
  Rig r;
  calibrate(r);
  CHECK_NEAR(r.m.homeDeg(AXIS_PAN), 0.0f, kEps);
  CHECK(r.m.moveTo(AXIS_PAN, 370.0f));
  CHECK(runUntil(r.m, [&] { return settled(r.m); }, 15 * US_PER_S));
  CHECK_NEAR(r.m.positionDeg(AXIS_PAN), 370.0f, stepDeg(AXIS_PAN));
  float minPan = r.m.positionDeg(AXIS_PAN), maxPan = minPan;
  CHECK(r.m.goHome());
  CHECK(runUntil(r.m, [&] {
          const float p = r.m.positionDeg(AXIS_PAN);
          if (p < minPan) minPan = p;
          if (p > maxPan) maxPan = p;
          return !r.m.homing();
        }, 5 * US_PER_S));
  CHECK_NEAR(r.m.positionDeg(AXIS_PAN), 360.0f, stepDeg(AXIS_PAN));   // home, one turn on
  CHECK(maxPan - minPan <= 10.0f + stepDeg(AXIS_PAN));                // 10 deg of travel, not 370
  // With pan limits on, the stored angle is the only legal home: no wrapping.
  Rig lim;
  calibrate(lim);
  lim.m.setPanLimits(true, PAN_MIN_DEG, PAN_MAX_DEG);
  CHECK(lim.m.moveTo(AXIS_PAN, PAN_MAX_DEG - 10.0f));
  CHECK(runUntil(lim.m, [&] { return settled(lim.m); }, 15 * US_PER_S));
  lim.m.setHome(PAN_MIN_DEG + 10.0f, 0.0f);
  CHECK(lim.m.goHome());
  CHECK(runUntil(lim.m, [&] { return !lim.m.homing(); }, 20 * US_PER_S));
  CHECK_NEAR(lim.m.positionDeg(AXIS_PAN), PAN_MIN_DEG + 10.0f, stepDeg(AXIS_PAN));
}

TEST(go_home_defaults_to_the_calibrated_zero) {
  Rig r;
  calibrate(r);
  r.m.setVelocity(0.6f, 0.6f);
  runFor(r.m, US_PER_S);
  r.m.setVelocity(0.0f, 0.0f);
  r.m.goHome();
  CHECK(runUntil(r.m, [&] { return !r.m.homing() && !r.m.isMoving(); }, 10 * US_PER_S));
  CHECK_EQ(hal::stepPosition(AXIS_PAN), 0);
  CHECK_EQ(hal::stepPosition(AXIS_TILT), 0);   // home at the soft minimum: reached, not refused
  CHECK_EQ(hal_host::totalRefusals(AXIS_TILT), 0u);
}

TEST(a_stick_nudge_cancels_go_home) {
  Rig r;
  calibrate(r);
  r.m.setHome(-90.0f, 40.0f);
  CHECK(r.m.goHome());
  runFor(r.m, 300 * US_PER_MS);
  CHECK(r.m.homing());
  // TurretState's rule: setVelocity only while the stick is active (or no move runs).
  r.m.setVelocity(0.4f, 0.0f);
  CHECK(!r.m.homing());
  CHECK(r.m.mode(AXIS_PAN) == Motion::Mode::VELOCITY);
  CHECK(r.m.mode(AXIS_TILT) == Motion::Mode::VELOCITY);
  runFor(r.m, US_PER_S);
  CHECK_NEAR(r.m.velocityDeg(AXIS_PAN), 0.4f * PAN_MAX_DEG_PER_S, kEps);
  CHECK_NEAR(r.m.velocityDeg(AXIS_TILT), 0.0f, 0.0f);
}

TEST(random_position_moves_land_exactly_and_never_stall_short) {
  Rig r;
  Rng rng(99);
  calibrate(r);
  Watch w;
  w.start(r.m);
  int missed = 0, timeouts = 0, reversals = 0;
  for (int i = 0; i < 160; i++) {
    float pan, tilt;
    const uint32_t kind = rng.range(0, 3);
    if (kind == 0) {   // tiny: 0 .. 3 steps
      pan = r.m.positionDeg(AXIS_PAN) + (float)rng.range(0, 3) * stepDeg(AXIS_PAN) * (rng.range(0, 1) ? 1.0f : -1.0f);
      tilt = r.m.positionDeg(AXIS_TILT) + rng.uniform(-0.2f, 0.2f);
    } else {
      pan = rng.uniform(-170.0f, 170.0f);
      tilt = rng.uniform(-5.0f, 65.0f);   // some outside: clamped
    }
    CHECK(r.m.moveTo(pan, tilt));
    const int32_t wantPan = tstepsFor(AXIS_PAN, pan);
    const int32_t wantTilt = tstepsFor(AXIS_TILT, tilt < 0.0f ? 0.0f : tilt > TILT_RANGE_DEG ? TILT_RANGE_DEG : tilt);
    float lp = 0.0f, lt = 0.0f;
    if (!runUntil(r.m, [&] {
          const float vp = r.m.velocityDeg(AXIS_PAN), vt = r.m.velocityDeg(AXIS_TILT);
          if (vp * lp < 0.0f || vt * lt < 0.0f) reversals++;
          if (vp != 0.0f) lp = vp;
          if (vt != 0.0f) lt = vt;
          return !r.m.positionMoveActive();
        }, 15 * US_PER_S, &w, rng.range(50, 900)))
      timeouts++;
    if (tsteps(AXIS_PAN) != wantPan || tsteps(AXIS_TILT) != wantTilt) missed++;
    if (r.m.isMoving()) missed++;
  }
  CHECK_EQ(timeouts, 0);
  CHECK_EQ(missed, 0);
  CHECK_EQ(reversals, 0);
  CHECK_WATCH(w);
  CHECK_EQ(hal_host::totalRefusals(AXIS_PAN) + hal_host::totalRefusals(AXIS_TILT), 0u);
}

TEST(a_move_against_the_current_motion_stops_then_returns) {
  Rig r;
  Watch w;
  r.m.setVelocity(-1.0f, 0.0f);
  runFor(r.m, US_PER_S);
  w.start(r.m);
  CHECK(r.m.moveTo(AXIS_PAN, 50.0f));
  bool sawStop = false;
  CHECK(runUntil(r.m, [&] {
          if (r.m.velocityDeg(AXIS_PAN) == 0.0f && hal::stepRate(AXIS_PAN) == 0.0f) sawStop = true;
          return !r.m.positionMoveActive();
        }, 10 * US_PER_S, &w));
  CHECK(sawStop);
  CHECK_WATCH(w);
  CHECK_EQ(tsteps(AXIS_PAN), tstepsFor(AXIS_PAN, 50.0f));
  CHECK_EQ(hal_host::trueSteps(AXIS_PAN), (int64_t)hal::stepPosition(AXIS_PAN));   // no step lost
}

TEST(move_targets_are_clamped_to_the_active_limits) {
  Rig r;
  calibrate(r);
  CHECK(r.m.moveTo(AXIS_TILT, TILT_RANGE_DEG + 30.0f));
  CHECK(runUntil(r.m, [&] { return !r.m.positionMoveActive(); }, 5 * US_PER_S));
  CHECK_EQ(tsteps(AXIS_TILT), tstepsFor(AXIS_TILT, TILT_RANGE_DEG));
  CHECK(r.m.moveTo(AXIS_TILT, -30.0f));
  CHECK(runUntil(r.m, [&] { return !r.m.positionMoveActive(); }, 5 * US_PER_S));
  CHECK_EQ(tsteps(AXIS_TILT), 0);
  CHECK_EQ(hal_host::totalRefusals(AXIS_TILT), 0u);
  CHECK(!r.m.moveTo(AXIS_PAN, NAN));   // refused, nothing changes
  CHECK(r.m.mode(AXIS_PAN) == Motion::Mode::STOP);
}

// ---- calibration (spec 3.5, 11.2) -------------------------------------------------------------

TEST(cal_ram_finds_the_stop_from_any_height) {
  const int32_t backoff = tstepsFor(AXIS_TILT, TILT_CAL_BACKOFF_DEG);
  for (float h : {0.0f, 0.4f, 10.0f, 30.0f, TILT_RANGE_DEG + TILT_CAL_BACKOFF_DEG, TILT_CAL_MAX_TRAVEL_DEG - 0.5f}) {
    Rig r;
    hal_host::setTrueSteps(AXIS_PAN, 123);   // pan is not calibrated: untouched
    hal::stepSetPosition(AXIS_PAN, 77);
    const int64_t stop = placeStop(h);
    Watch w;
    w.start(r.m);
    r.m.startCalibration(hal::nowMs(), CAL_RAM);
    CHECK(r.m.calibrationStatus() == Motion::CalStatus::RUNNING);
    CHECK(r.m.calPhase() == Motion::CalPhase::RAM_DOWN);
    CHECK(r.m.driversEnabled());
    CHECK(!r.m.limitsActive(AXIS_TILT));
    CHECK(!r.m.calibrationWantsStall());   // CAL_RAM: stall events are not wanted
    // Watch the phases: the deepest count, the down speed, the settle pause.
    int32_t deepest = 0;
    float downSpeed = 0.0f;
    uint64_t downEndUs = 0, liftStartUs = 0;
    Motion::CalPhase last = r.m.calPhase();
    const bool ok = runUntil(r.m, [&] {
      const int32_t s = tsteps(AXIS_TILT);
      if (s < deepest) deepest = s;
      if (r.m.calPhase() == Motion::CalPhase::RAM_DOWN && -r.m.velocityDeg(AXIS_TILT) > downSpeed)
        downSpeed = -r.m.velocityDeg(AXIS_TILT);
      if (last == Motion::CalPhase::RAM_DOWN && r.m.calPhase() == Motion::CalPhase::SETTLE) downEndUs = hal_host::timeUs();
      if (last == Motion::CalPhase::SETTLE && r.m.calPhase() == Motion::CalPhase::LIFT) liftStartUs = hal_host::timeUs();
      last = r.m.calPhase();
      return r.m.calibrationStatus() != Motion::CalStatus::RUNNING;
    }, 30 * US_PER_S, &w);
    CHECK(ok);
    CHECK(r.m.calibrationStatus() == Motion::CalStatus::DONE);
    CHECK(r.m.calibrated());
    CHECK_WATCH(w);
    // The ram: TILT_CAL_MAX_TRAVEL_DEG commanded down, at TILT_CAL_SPEED_DEG_PER_S.
    CHECK_EQ(deepest, -tstepsFor(AXIS_TILT, TILT_CAL_MAX_TRAVEL_DEG));
    CHECK_NEAR(downSpeed, TILT_CAL_SPEED_DEG_PER_S, kEps);
    CHECK(liftStartUs - downEndUs >= (uint64_t)TILT_CAL_SETTLE_MS * US_PER_MS);
    CHECK(liftStartUs - downEndUs <= (uint64_t)TILT_CAL_SETTLE_MS * US_PER_MS + 2 * kTickUs);
    // The result: tilt reads 0 and is truly TILT_CAL_BACKOFF_DEG above the panel.
    CHECK_EQ(hal::stepPosition(AXIS_TILT), 0);
    CHECK_NEAR(r.m.positionDeg(AXIS_TILT), 0.0f, 0.0f);
    CHECK_EQ(aboveStop(stop), (int64_t)backoff);
    CHECK_NEAR((double)hal_host::lostSteps(AXIS_TILT), (double)tstepsFor(AXIS_TILT, TILT_CAL_MAX_TRAVEL_DEG - h), 1.0);
    CHECK(r.m.limitsActive(AXIS_TILT));
    CHECK(r.m.mode(AXIS_TILT) == Motion::Mode::STOP);
    CHECK(!r.m.isMoving());
    CHECK_EQ(hal_host::trueSteps(AXIS_PAN), 123);
    CHECK_EQ(hal::stepPosition(AXIS_PAN), 77);
    // Afterwards full stick down holds at the soft minimum: never back onto the panel.
    const uint32_t lost = hal_host::lostSteps(AXIS_TILT);
    r.m.setVelocity(0.0f, -1.0f);
    runFor(r.m, 2 * US_PER_S, &w);
    CHECK_EQ(hal_host::lostSteps(AXIS_TILT), lost);
    CHECK(aboveStop(stop) >= (int64_t)backoff);
    CHECK_WATCH(w);
  }
}

TEST(cal_ram_duration_matches_travel_settle_and_backoff) {
  Rig r;
  placeStop(30.0f);
  const uint64_t t0 = hal_host::timeUs();
  r.m.startCalibration(hal::nowMs(), CAL_RAM);
  CHECK(finishCal(r.m));
  const double s = (double)(hal_host::timeUs() - t0) / US_PER_S;
  const double expect = (TILT_CAL_MAX_TRAVEL_DEG + TILT_CAL_BACKOFF_DEG) / TILT_CAL_SPEED_DEG_PER_S +
                        TILT_CAL_SETTLE_MS / 1000.0;
  CHECK(s >= expect);
  CHECK(s <= expect + 0.3);   // ramps and the creep at both ends
}

// The precondition the owner must keep: TILT_CAL_MAX_TRAVEL_DEG > the real range. A barrel that
// starts higher than that above the panel never reaches it, and the zero is wrong.
TEST(cal_ram_cannot_find_a_stop_beyond_its_travel) {
  Rig r;
  const int64_t stop = placeStop(TILT_CAL_MAX_TRAVEL_DEG + 5.0f);
  r.m.startCalibration(hal::nowMs(), CAL_RAM);
  CHECK(finishCal(r.m));
  CHECK(r.m.calibrated());   // CAL_RAM has no way to know
  CHECK_EQ(hal_host::lostSteps(AXIS_TILT), 0u);
  CHECK_EQ(aboveStop(stop), (int64_t)tstepsFor(AXIS_TILT, 5.0f + TILT_CAL_BACKOFF_DEG));
}

TEST(cal_mode_defaults_to_the_config_and_can_be_chosen) {
  Rig r;
  r.m.startCalibration(hal::nowMs());
  CHECK_EQ(r.m.calMode(), TILT_CAL_MODE);
  CHECK(r.m.calPhase() == (TILT_CAL_MODE == CAL_STALL_SENSE ? Motion::CalPhase::SENSE_DOWN
                                                            : Motion::CalPhase::RAM_DOWN));
  r.m.abortCalibration();
  r.m.startCalibration(hal::nowMs(), CAL_STALL_SENSE);
  CHECK_EQ(r.m.calMode(), CAL_STALL_SENSE);
  r.m.abortCalibration();
  r.m.startCalibration(hal::nowMs(), 77);   // unknown: CAL_RAM
  CHECK_EQ(r.m.calMode(), CAL_RAM);
}

TEST(cal_stall_sense_stops_on_the_event_and_zeroes_above_the_stop) {
  Rig r;
  const int64_t stop = placeStop(25.0f);
  r.m.startCalibration(hal::nowMs(), CAL_STALL_SENSE);
  CHECK(r.m.calibrationWantsStall());
  CHECK(r.stall.forced());   // detection forced on for the drive down
  CHECK(r.stall.active());
  CHECK(!r.m.limitsActive(AXIS_TILT));
  // Drive down until the barrel meets the panel (the model starts losing steps), then a few
  // more ticks, as a real detector needs STALL_CONFIRM_SAMPLES to confirm.
  float downSpeed = 0.0f;
  CHECK(runUntil(r.m, [&] {
          if (-r.m.velocityDeg(AXIS_TILT) > downSpeed) downSpeed = -r.m.velocityDeg(AXIS_TILT);
          return hal_host::lostSteps(AXIS_TILT) > 0;
        }, 10 * US_PER_S));
  CHECK_NEAR(downSpeed, TILT_CAL_SPEED_DEG_PER_S, kEps);
  runFor(r.m, 20 * US_PER_MS);
  CHECK(r.m.isMoving());
  CHECK(r.m.notifyCalibrationStall(hal::nowMs()));
  // "Stop the instant the stall detector fires": the step generator is off in the same call.
  CHECK_NEAR(hal::stepRate(AXIS_TILT), 0.0f, 0.0f);
  CHECK(!r.m.isMoving());
  CHECK(r.m.calPhase() == Motion::CalPhase::SETTLE);
  CHECK(!r.stall.forced());
  CHECK(!r.m.calibrationWantsStall());
  CHECK(!r.m.notifyCalibrationStall(hal::nowMs()));   // a second event is not wanted
  CHECK(finishCal(r.m));
  CHECK(r.m.calibrationStatus() == Motion::CalStatus::DONE);
  CHECK_EQ(hal::stepPosition(AXIS_TILT), 0);
  CHECK_EQ(aboveStop(stop), (int64_t)tstepsFor(AXIS_TILT, TILT_CAL_BACKOFF_DEG));
  CHECK(r.m.limitsActive(AXIS_TILT));
}

TEST(cal_stall_sense_faults_after_the_timeout) {
  Rig r;
  placeStop(25.0f);
  const uint32_t t0 = hal::nowMs();
  r.m.startCalibration(t0, CAL_STALL_SENSE);
  CHECK(runUntil(r.m, [&] { return r.m.calibrationStatus() != Motion::CalStatus::RUNNING; },
                 (TILT_CAL_TIMEOUT_MS + 1000) * US_PER_MS));
  CHECK(r.m.calibrationStatus() == Motion::CalStatus::FAILED);
  CHECK(r.m.calFailure() == Motion::CalFailure::TIMEOUT);
  const uint32_t took = hal::nowMs() - t0;
  CHECK(took >= TILT_CAL_TIMEOUT_MS);
  CHECK(took <= TILT_CAL_TIMEOUT_MS + 2 * kTickUs / US_PER_MS);
  CHECK(!r.stall.forced());
  CHECK(!r.m.calibrated());
  CHECK(!r.m.limitsActive(AXIS_TILT));
  CHECK(!r.m.calibrationWantsStall());
  // The tilt decelerates to rest and stays there.
  CHECK(runUntil(r.m, [&] { return !r.m.isMoving(); }, US_PER_S));
  CHECK(r.m.mode(AXIS_TILT) == Motion::Mode::STOP);
  const int32_t s = hal::stepPosition(AXIS_TILT);
  runFor(r.m, US_PER_S);
  CHECK_EQ(hal::stepPosition(AXIS_TILT), s);
}

TEST(cal_ram_ignores_stall_events) {
  Rig r;
  placeStop(5.0f);
  r.m.startCalibration(hal::nowMs(), CAL_RAM);
  runFor(r.m, US_PER_S);
  CHECK(!r.m.notifyCalibrationStall(hal::nowMs()));
  CHECK(r.m.calPhase() == Motion::CalPhase::RAM_DOWN);
  CHECK(r.m.isMoving());
}

TEST(abort_and_halt_end_a_running_calibration) {
  {
    Rig r;
    placeStop(30.0f);
    r.m.startCalibration(hal::nowMs(), CAL_STALL_SENSE);
    runFor(r.m, US_PER_S);
    r.m.abortCalibration();
    CHECK(r.m.calibrationStatus() == Motion::CalStatus::IDLE);
    CHECK(!r.stall.forced());
    CHECK(runUntil(r.m, [&] { return !r.m.isMoving(); }, US_PER_S));   // decelerated
  }
  {
    Rig r;
    placeStop(30.0f);
    r.m.startCalibration(hal::nowMs(), CAL_RAM);
    runFor(r.m, US_PER_S);
    r.m.haltNow();
    CHECK(r.m.calibrationStatus() == Motion::CalStatus::IDLE);
    CHECK_NEAR(hal::stepRate(AXIS_TILT), 0.0f, 0.0f);   // at once
    CHECK(!r.m.isMoving());
  }
  {   // stop() leaves a calibration alone; moveTo / setVelocity cannot take the tilt from it
    Rig r;
    placeStop(30.0f);
    r.m.startCalibration(hal::nowMs(), CAL_RAM);
    runFor(r.m, US_PER_S);
    r.m.stop();
    r.m.setVelocity(0.0f, 1.0f);
    CHECK(!r.m.moveTo(AXIS_TILT, 10.0f));
    CHECK(r.m.mode(AXIS_TILT) == Motion::Mode::CALIBRATE);
    CHECK(finishCal(r.m));
    CHECK(r.m.calibrated());
  }
}

TEST(calibration_fails_at_once_without_a_tilt_timer) {
  Rig r(true);
  r.m.startCalibration(hal::nowMs(), CAL_RAM);
  CHECK(r.m.calibrationStatus() == Motion::CalStatus::FAILED);
  CHECK(r.m.calFailure() == Motion::CalFailure::NO_TIMER);
}

// ---- drivers ----------------------------------------------------------------------------------

TEST(drivers_hold_when_idle_by_default_and_release_by_policy) {
  {
    Rig r;
    r.m.setVelocity(0.5f, 0.0f);
    runFor(r.m, 300 * US_PER_MS);
    CHECK(r.m.driversEnabled());
    r.m.stop();
    runFor(r.m, 10 * US_PER_S);
    CHECK_EQ(r.m.driversEnabled(), STEPPER_HOLD_WHEN_IDLE || false);
  }
  {
    Rig r;
    r.m.setIdlePolicy(false, 500);
    r.m.setVelocity(0.5f, 0.0f);
    runFor(r.m, 300 * US_PER_MS);
    r.m.stop();
    CHECK(runUntil(r.m, [&] { return !r.m.isMoving(); }, US_PER_S));
    const uint64_t stoppedUs = hal_host::timeUs();
    CHECK(runUntil(r.m, [&] { return !r.m.driversEnabled(); }, 2 * US_PER_S));
    const double after = (double)(hal_host::timeUs() - stoppedUs) / US_PER_MS;
    CHECK(after >= 500.0 - 10.0);   // counted from the tick that found the axes idle
    CHECK(after <= 500.0 + 20.0);
    // The next move enables EN again before its first step.
    const uint32_t lost = hal_host::lostSteps(AXIS_PAN);
    r.m.moveTo(AXIS_PAN, r.m.positionDeg(AXIS_PAN) + 10.0f);
    CHECK(runUntil(r.m, [&] { return !r.m.positionMoveActive(); }, 2 * US_PER_S));
    CHECK_EQ(hal_host::lostSteps(AXIS_PAN), lost);
  }
  {
    Rig r;
    r.m.setIdlePolicy(false, 0);
    r.m.setVelocity(0.5f, 0.0f);
    runFor(r.m, 300 * US_PER_MS);
    r.m.stop();
    CHECK(runUntil(r.m, [&] { return !r.m.isMoving(); }, US_PER_S));
    runFor(r.m, 2 * kTickUs);
    CHECK(!r.m.driversEnabled());
  }
  {   // power off: halt and release
    Rig r;
    r.m.setVelocity(1.0f, 1.0f);
    runFor(r.m, 500 * US_PER_MS);
    r.m.haltNow();
    r.m.enableDrivers(false);
    CHECK_NEAR(hal::stepRate(AXIS_PAN), 0.0f, 0.0f);
    CHECK_NEAR(hal::stepRate(AXIS_TILT), 0.0f, 0.0f);
    CHECK(!r.m.isMoving());
    CHECK(!r.m.driversEnabled());
    runFor(r.m, US_PER_S);
    CHECK(!r.m.driversEnabled());
  }
}

// ---- stall back-off ------------------------------------------------------------------------------

TEST(back_off_stops_the_stalled_axis_and_reverses_it_by_the_backoff_angle) {
  Rig r;
  calibrate(r);
  r.m.moveTo(AXIS_TILT, 30.0f);
  CHECK(runUntil(r.m, [&] { return settled(r.m); }, 5 * US_PER_S));
  r.m.setVelocity(0.5f, -0.3f);   // pan +, tilt down
  runFor(r.m, 400 * US_PER_MS);
  const float pan0 = r.m.positionDeg(AXIS_PAN), tilt0 = r.m.positionDeg(AXIS_TILT);
  r.m.backOff(axisBit(AXIS_PAN) | axisBit(AXIS_TILT), STALL_BACKOFF_DEG);
  CHECK_NEAR(hal::stepRate(AXIS_PAN), 0.0f, 0.0f);    // pulses stop at once
  CHECK_NEAR(hal::stepRate(AXIS_TILT), 0.0f, 0.0f);
  CHECK(r.m.backingOff());
  r.m.setVelocity(1.0f, 1.0f);   // ignored while backing off
  CHECK(r.m.mode(AXIS_PAN) == Motion::Mode::BACKOFF);
  Watch w;
  w.start(r.m);
  CHECK(runUntil(r.m, [&] { return !r.m.backingOff(); }, 3 * US_PER_S, &w));
  CHECK_WATCH(w);   // the back-off itself starts from rest, accel-limited
  CHECK_NEAR(r.m.positionDeg(AXIS_PAN), pan0 - STALL_BACKOFF_DEG, stepDeg(AXIS_PAN));
  CHECK_NEAR(r.m.positionDeg(AXIS_TILT), tilt0 + STALL_BACKOFF_DEG, stepDeg(AXIS_TILT));
  CHECK(!r.m.isMoving());
  CHECK(r.m.mode(AXIS_PAN) == Motion::Mode::STOP);
}

TEST(back_off_only_reverses_the_axes_in_the_mask) {
  Rig r;
  r.m.setVelocity(-1.0f, 0.5f);
  runFor(r.m, 500 * US_PER_MS);
  const float tilt0 = r.m.positionDeg(AXIS_TILT);
  const float panV = r.m.velocityDeg(AXIS_PAN);
  r.m.backOff(axisBit(AXIS_TILT), STALL_BACKOFF_DEG);
  CHECK_NEAR(hal::stepRate(AXIS_TILT), 0.0f, 0.0f);
  CHECK_NEAR(r.m.velocityDeg(AXIS_PAN), panV, 0.0f);   // not stalled: decelerates, not cut
  CHECK(r.m.mode(AXIS_PAN) == Motion::Mode::STOP);
  Watch w;
  w.start(r.m);
  CHECK(runUntil(r.m, [&] { return settled(r.m); }, 3 * US_PER_S, &w));
  CHECK_WATCH(w);
  CHECK_NEAR(r.m.positionDeg(AXIS_TILT), tilt0 - STALL_BACKOFF_DEG, stepDeg(AXIS_TILT));
}

TEST(back_off_is_clamped_to_the_soft_limits) {
  Rig r;
  calibrate(r);
  r.m.setVelocity(0.0f, 0.05f);   // tilt creeping up from 0: a back-off would go below 0
  runFor(r.m, 300 * US_PER_MS);
  CHECK(r.m.isMoving());
  CHECK(r.m.positionDeg(AXIS_TILT) < STALL_BACKOFF_DEG);
  r.m.backOff(axisBit(AXIS_TILT), STALL_BACKOFF_DEG);
  CHECK(runUntil(r.m, [&] { return settled(r.m); }, 3 * US_PER_S));
  CHECK_EQ(hal::stepPosition(AXIS_TILT), 0);
  CHECK_EQ(hal_host::totalRefusals(AXIS_TILT), 0u);
}

TEST(move_start_times_follow_each_axis_start) {
  Rig r;
  runFor(r.m, 100 * US_PER_MS);
  r.m.setVelocity(0.5f, 0.0f);
  CHECK(runUntil(r.m, [&] { return r.m.isMoving(); }, US_PER_S));
  const uint32_t panStart = r.m.moveStartMs(AXIS_PAN);
  CHECK_EQ(r.m.moveStartMs(), panStart);
  CHECK_EQ(r.m.movingAxes(), axisBit(AXIS_PAN));
  runFor(r.m, 300 * US_PER_MS);
  r.m.setVelocity(0.5f, 0.5f);
  CHECK(runUntil(r.m, [&] { return r.m.movingAxes() == (axisBit(AXIS_PAN) | axisBit(AXIS_TILT)); }, US_PER_S));
  CHECK_EQ(r.m.moveStartMs(AXIS_PAN), panStart);   // pan kept moving
  CHECK(r.m.moveStartMs(AXIS_TILT) >= panStart + 300);
  CHECK_EQ(r.m.moveStartMs(), r.m.moveStartMs(AXIS_TILT));   // the latest start re-masks
}

TESTKIT_MAIN()

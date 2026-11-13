// test_stall.cpp -- the real StallDetector (firmware-ra4m1/dsentry/src/StallDetector.cpp) fed
// synthetic A2 signals through the host HAL's analog input (hal_host::setAnalog): steps, ramps,
// noise, a potentiometer twist; then wired to the real Motion as App and TurretState will wire
// it, for spec 11 item 15 (motion part) and a CAL_STALL_SENSE calibration in the physics model.
//
// What is proved, against spec 3.6 and architecture.md 5 / 6:
//   - sampling at STALL_SAMPLE_HZ from update(nowUs), at most one catch-up sample per call,
//     the backlog dropped (and counted) after a long gap, wrap-safe;
//   - the baseline is frozen at rest; while moving, in-band samples follow the EMA
//     (STALL_BASELINE_ALPHA) and over-threshold samples never do; nothing is counted in the
//     first STALL_IGNORE_AFTER_START_MS of a move, so a small start-level change is learnt and a
//     crash already present at the start is reported straight after the window;
//   - KEY SAFETY PROPERTY: |sample - baseline| > STALL_DELTA_COUNTS for STALL_CONFIRM_SAMPLES in a
//     row while moving is a stall, after exactly that many samples (a_step_while_moving_...
//     fails if the confirm count, the threshold or the moving-axes report is broken), and wired
//     to Motion it stops the moving axis at once and backs it off STALL_BACKOFF_DEG
//     (a_pot_twist_stops_the_turret_and_backs_it_off);
//   - exactly at the threshold is not a stall; short spikes reset the run; in-band noise and
//     slow ramps never stall; a fast ramp does;
//   - takeStall() clears; stallAxes(); peak delta per move; the level-3 trace window;
//   - inactive (STALL_DETECT_ENABLED false and not forced): never declares, still measures.
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

constexpr uint32_t kPeriodUs = US_PER_S / STALL_SAMPLE_HZ;
constexpr uint8_t kBoth = (uint8_t)((1u << AXIS_PAN) | (1u << AXIS_TILT));
constexpr uint32_t kIgnoreSamples = STALL_IGNORE_AFTER_START_MS * STALL_SAMPLE_HZ / MS_PER_S;

struct Rng {
  uint64_t s;
  explicit Rng(uint64_t seed) : s(seed * 0x9E3779B97F4A7C15ull + 0x9876ull) {}
  uint32_t next() {
    s ^= s << 13;
    s ^= s >> 7;
    s ^= s << 17;
    return (uint32_t)(s >> 32);
  }
  float uniform(float lo, float hi) { return lo + (hi - lo) * (float)(next() >> 8) / 16777216.0f; }
};

void setA2(float counts) {
  if (counts < 0.0f) counts = 0.0f;
  hal_host::setAnalog(PIN_STALL_SENSE, (uint16_t)(counts + 0.5f));
}

// A detector at `level` counts, forced active (STALL_DETECT_ENABLED is off in config.h).
struct Det {
  StallDetector d;
  explicit Det(uint16_t level, bool forced = true) {
    hal_host::reset();
    hal::begin();
    hal_host::setAnalog(PIN_STALL_SENSE, level);
    hal_host::advanceMs(5);   // nonzero clock, so a move can start "at t = now"
    d.begin(hal::nowUs());
    d.setForced(forced);
  }
  // One sample period with this motion state.
  void sample(uint8_t moving, uint32_t moveStartMs) {
    hal_host::advanceUs(kPeriodUs);
    d.update(hal::nowUs(), hal::nowMs(), moving, moveStartMs);
  }
  // `n` periods.
  void samples(uint32_t n, uint8_t moving, uint32_t moveStartMs) {
    for (uint32_t i = 0; i < n; i++) sample(moving, moveStartMs);
  }
  // A move that started long enough ago that the ignore window is over.
  uint32_t oldMove() const { return hal::nowMs() - STALL_IGNORE_AFTER_START_MS - 1; }
};

}  // namespace

// ---- sampling schedule ------------------------------------------------------------------------

TEST(samples_at_stall_sample_hz) {
  Det t(500);
  const uint32_t s0 = t.d.samples();
  for (int i = 0; i < 10000; i++) {   // a 100 us loop for 1 s
    hal_host::advanceUs(100);
    t.d.update(hal::nowUs(), hal::nowMs(), 0, 0);
  }
  CHECK_EQ(t.d.samples() - s0, (uint32_t)STALL_SAMPLE_HZ);
  CHECK_EQ(t.d.droppedSamples(), 0u);
}

TEST(a_late_loop_takes_one_catch_up_sample_and_drops_the_backlog) {
  Det t(500);
  uint32_t s = t.d.samples();
  hal_host::advanceUs(kPeriodUs * 5 / 2);   // 2.5 periods late
  t.d.update(hal::nowUs(), hal::nowMs(), 0, 0);
  CHECK_EQ(t.d.samples() - s, 2u);           // the due one + one catch-up
  CHECK_EQ(t.d.droppedSamples(), 0u);
  s = t.d.samples();
  hal_host::advanceUs(kPeriodUs * 10);       // a 10-period stall of the loop
  t.d.update(hal::nowUs(), hal::nowMs(), 0, 0);
  CHECK_EQ(t.d.samples() - s, 2u);           // never a burst
  CHECK(t.d.droppedSamples() >= 7u);
  // The schedule restarts one period from now.
  s = t.d.samples();
  hal_host::advanceUs(kPeriodUs - 1);
  t.d.update(hal::nowUs(), hal::nowMs(), 0, 0);
  CHECK_EQ(t.d.samples() - s, 0u);
  hal_host::advanceUs(1);
  t.d.update(hal::nowUs(), hal::nowMs(), 0, 0);
  CHECK_EQ(t.d.samples() - s, 1u);
  // Called twice at the same instant: no second sample.
  t.d.update(hal::nowUs(), hal::nowMs(), 0, 0);
  CHECK_EQ(t.d.samples() - s, 1u);
}

TEST(the_schedule_survives_the_micros_wrap) {
  Det t(500);
  const uint32_t start = UINT32_MAX - 1500u;
  t.d.begin(start);
  const uint32_t s = t.d.samples();
  t.d.update(start + kPeriodUs - 1u, 0, 0, 0);
  CHECK_EQ(t.d.samples(), s);
  t.d.update(start + kPeriodUs, 0, 0, 0);              // UINT32_MAX - 500
  CHECK_EQ(t.d.samples(), s + 1u);
  t.d.update(start + kPeriodUs * 2u, 0, 0, 0);         // wrapped past 0
  CHECK_EQ(t.d.samples(), s + 2u);
  CHECK_EQ(t.d.droppedSamples(), 0u);
}

// ---- baseline -----------------------------------------------------------------------------------

TEST(begin_seeds_the_baseline_from_the_first_reading) {
  Det t(612);
  CHECK_EQ(t.d.baseline(), 612);
  CHECK_EQ(t.d.sample(), 612);
  CHECK_EQ(t.d.samples(), 1u);
}

TEST(the_baseline_is_frozen_at_rest) {
  Det t(500);
  setA2(900);
  t.samples(2000, 0, 0);
  CHECK_EQ(t.d.baseline(), 500);
  CHECK_EQ(t.d.sample(), 900);
  CHECK(!t.d.stallPending());
  CHECK_EQ(t.d.triggers(), 0u);   // a big change at rest is never even a would-be stall
  CHECK_EQ(t.d.overCount(), 0);
  CHECK(!t.d.moving());
}

TEST(the_ignore_window_learns_a_small_level_change_and_counts_nothing) {
  Det t(500);
  const uint32_t start = hal::nowMs();
  const float jump = STALL_DELTA_COUNTS * 0.6f;   // in band: rail sag or drift since last move
  setA2(500.0f + jump);
  for (uint32_t i = 0; i < kIgnoreSamples - 1; i++) {
    t.sample(axisBit(AXIS_PAN), start);
    CHECK(t.d.ignoring());
  }
  CHECK(!t.d.stallPending());
  CHECK_EQ(t.d.overCount(), 0);
  CHECK_EQ(t.d.peakDelta(), 0);   // the window's deltas are not counted ones
  // The EMA: b_n = level - jump (1 - alpha)^n.
  const double level = (double)(uint16_t)(500.0f + jump + 0.5f);
  const double expect = level - (level - 500.0) * pow(1.0 - STALL_BASELINE_ALPHA, (double)(kIgnoreSamples - 1));
  CHECK_NEAR(t.d.baselineExact(), expect, 0.05);
  t.samples(3000, axisBit(AXIS_PAN), start);
  CHECK(!t.d.ignoring());
  CHECK(!t.d.stallPending());
  CHECK_EQ(t.d.triggers(), 0u);
  CHECK_NEAR(t.d.baselineExact(), level, 0.5);
}

// A crash already pressing when the move starts (the barrel on the panel as a CAL_STALL_SENSE
// calibration begins): never learnt as the baseline, reported straight after the window.
TEST(a_crash_present_at_the_move_start_is_reported_after_the_window) {
  Det t(500);
  const uint32_t start = hal::nowMs();
  setA2(500 + STALL_DELTA_COUNTS * 3);
  uint32_t n = 0;
  while (!t.d.stallPending() && n < 2000) {
    t.sample(kBoth, start);
    n++;
  }
  CHECK(t.d.stallPending());
  CHECK_EQ(t.d.baseline(), 500);   // over-threshold samples never fed it
  // First sample outside the window is the one STALL_IGNORE_AFTER_START_MS after the start.
  CHECK_EQ(n, kIgnoreSamples + STALL_CONFIRM_SAMPLES - 1u);
}

TEST(in_band_samples_follow_the_ema_after_the_window) {
  Det t(500);
  const uint32_t move = t.oldMove();
  setA2(500 + STALL_DELTA_COUNTS / 2);
  const uint32_t n = 40;
  t.samples(n, axisBit(AXIS_TILT), move);
  const double expect = 500.0 + (STALL_DELTA_COUNTS / 2) * (1.0 - pow(1.0 - STALL_BASELINE_ALPHA, (double)n));
  CHECK_NEAR(t.d.baselineExact(), expect, 0.05);
  CHECK_EQ(t.d.peakDelta(), STALL_DELTA_COUNTS / 2);
}

// ---- declaring a stall (KEY SAFETY PROPERTY) ----------------------------------------------------

TEST(a_step_while_moving_is_a_stall_after_exactly_confirm_samples) {
  for (int sign : {1, -1}) {
    Det t(500);
    const uint32_t move = t.oldMove();
    t.samples(10, kBoth, move);   // moving, in band
    setA2(500.0f + sign * (STALL_DELTA_COUNTS + 1));
    t.samples(STALL_CONFIRM_SAMPLES - 1, kBoth, move);
    CHECK(!t.d.stallPending());
    CHECK_EQ(t.d.overCount(), STALL_CONFIRM_SAMPLES - 1);
    CHECK_EQ(t.d.baseline(), 500);   // over-threshold samples do not move the baseline
    t.sample(kBoth, move);
    CHECK(t.d.stallPending());
    CHECK_EQ(t.d.stallAxes(), kBoth);
    CHECK_EQ(t.d.stallCount(), 1u);
    CHECK_EQ(t.d.lastStallPeak(), STALL_DELTA_COUNTS + 1);
    CHECK(t.d.takeStall());
    CHECK(!t.d.takeStall());   // cleared by reading
    CHECK_EQ(t.d.stallAxes(), kBoth);   // still says which axes it was
  }
}

TEST(stall_axes_are_the_axes_moving_at_the_confirming_sample) {
  Det t(500);
  const uint32_t move = t.oldMove();
  setA2(500 + STALL_DELTA_COUNTS * 2);
  t.samples(STALL_CONFIRM_SAMPLES, axisBit(AXIS_TILT), move);
  CHECK(t.d.takeStall());
  CHECK_EQ(t.d.stallAxes(), axisBit(AXIS_TILT));
}

TEST(exactly_at_the_threshold_is_not_a_stall) {
  Det t(500);
  const uint32_t move = t.oldMove();
  setA2(500 + STALL_DELTA_COUNTS);   // |delta| == STALL_DELTA_COUNTS: not "more than"
  t.samples(STALL_CONFIRM_SAMPLES * 3, kBoth, move);
  CHECK(!t.d.stallPending());
  CHECK_EQ(t.d.triggers(), 0u);
}

TEST(a_spike_shorter_than_confirm_resets_the_run) {
  Det t(500);
  const uint32_t move = t.oldMove();
  for (int rep = 0; rep < 50; rep++) {
    setA2(500 + STALL_DELTA_COUNTS * 3);
    t.samples(STALL_CONFIRM_SAMPLES - 1, kBoth, move);
    setA2(500);
    t.sample(kBoth, move);
    CHECK_EQ(t.d.overCount(), 0);
  }
  CHECK(!t.d.stallPending());
  CHECK_EQ(t.d.triggers(), 0u);
  CHECK_EQ(t.d.peakDelta(), STALL_DELTA_COUNTS * 3);   // but the peaks are on record
}

TEST(a_continuing_fault_needs_a_fresh_run_for_the_next_stall) {
  Det t(500);
  const uint32_t move = t.oldMove();
  setA2(500 + STALL_DELTA_COUNTS * 2);
  t.samples(STALL_CONFIRM_SAMPLES, kBoth, move);
  CHECK_EQ(t.d.stallCount(), 1u);
  t.samples(STALL_CONFIRM_SAMPLES - 1, kBoth, move);
  CHECK_EQ(t.d.stallCount(), 1u);
  t.sample(kBoth, move);
  CHECK_EQ(t.d.stallCount(), 2u);
}

TEST(nothing_counts_during_the_ignore_window_even_a_big_kick) {
  Det t(500);
  const uint32_t start = hal::nowMs();
  setA2(1000);   // the start-up kick, shorter than the window
  t.samples(kIgnoreSamples - 2, kBoth, start);
  setA2(500);
  t.samples(1000, kBoth, start);
  CHECK(!t.d.stallPending());
  CHECK_EQ(t.d.triggers(), 0u);
  CHECK_EQ(t.d.baseline(), 500);   // nor did it move the baseline
}

// ---- signals --------------------------------------------------------------------------------------

TEST(noise_inside_the_band_never_stalls) {
  Det t(500);
  Rng rng(7);
  const uint32_t move = t.oldMove();
  const float amp = STALL_DELTA_COUNTS * 0.75f;
  for (int i = 0; i < 20 * STALL_SAMPLE_HZ; i++) {   // 20 s of moving
    setA2(500.0f + rng.uniform(-amp, amp));
    t.sample(kBoth, move);
  }
  CHECK(!t.d.stallPending());
  CHECK_EQ(t.d.triggers(), 0u);
  CHECK_NEAR(t.d.baselineExact(), 500.0, 15.0);
  CHECK(t.d.peakDelta() > STALL_DELTA_COUNTS / 2);
}

TEST(a_slow_ramp_is_followed_but_a_fast_ramp_is_a_stall) {
  {
    // 0.5 counts per sample: the EMA lags by r / alpha = 25 counts, inside the band.
    Det t(300);
    const uint32_t move = t.oldMove();
    float level = 300.0f;
    for (int i = 0; i < 1000; i++) {
      level += 0.5f;
      setA2(level);
      t.sample(axisBit(AXIS_PAN), move);
    }
    CHECK(!t.d.stallPending());
    CHECK_EQ(t.d.triggers(), 0u);
    CHECK_NEAR(t.d.baselineExact(), level - 0.5 * (1.0 - STALL_BASELINE_ALPHA) / STALL_BASELINE_ALPHA, 2.0);
  }
  {
    // 5 counts per sample: the lag would be 250 counts, so it crosses the band and stays out.
    Det t(300);
    const uint32_t move = t.oldMove();
    float level = 300.0f;
    int samplesToStall = -1;
    for (int i = 0; i < 100; i++) {
      level += 5.0f;
      setA2(level);
      t.sample(axisBit(AXIS_PAN), move);
      if (t.d.stallPending()) {
        samplesToStall = i + 1;
        break;
      }
    }
    CHECK(samplesToStall > 0);
    CHECK(samplesToStall < 60);
    CHECK(t.d.lastStallPeak() > STALL_DELTA_COUNTS);
  }
}

TEST(a_pot_twist_while_moving_is_a_stall_with_its_peak) {
  Det t(512);
  Rng rng(3);
  const uint32_t start = hal::nowMs();
  for (int i = 0; i < 1000; i++) {   // 1 s of moving with +-10 counts of pot noise
    setA2(512.0f + rng.uniform(-10.0f, 10.0f));
    t.sample(kBoth, start);
  }
  CHECK(!t.d.stallPending());
  // A sudden twist: 512 -> 900 over 30 ms, then held.
  int at = -1;
  for (int i = 0; i < 200; i++) {
    const float level = i < 30 ? 512.0f + (900.0f - 512.0f) * (float)(i + 1) / 30.0f : 900.0f;
    setA2(level + rng.uniform(-10.0f, 10.0f));
    t.sample(kBoth, start);
    if (t.d.stallPending()) {
      at = i;
      break;
    }
  }
  CHECK(at >= 0);
  // The band (80 counts) is crossed about 6 samples in; confirmed STALL_CONFIRM_SAMPLES later.
  CHECK(at <= 30);
  CHECK(t.d.lastStallPeak() > STALL_DELTA_COUNTS);
  CHECK_EQ(t.d.stallAxes(), kBoth);
}

// ---- peaks, moves, trace, enable ---------------------------------------------------------------

TEST(a_new_move_resets_the_peak_and_reopens_the_window) {
  Det t(500);
  uint32_t move = t.oldMove();
  setA2(560);
  t.samples(5, kBoth, move);
  CHECK_EQ(t.d.peakDelta(), 60);
  setA2(500);
  t.samples(500, 0, move);        // rest: peak kept for the record
  CHECK_EQ(t.d.peakDelta(), 60);
  move = hal::nowMs();            // a new move
  t.sample(axisBit(AXIS_PAN), move);
  CHECK_EQ(t.d.peakDelta(), 0);
  CHECK(t.d.ignoring());
}

TEST(a_second_axis_starting_masks_its_own_kick) {
  Det t(500);
  const uint32_t panStart = t.oldMove();
  t.samples(100, axisBit(AXIS_PAN), panStart);
  CHECK(!t.d.ignoring());
  // Tilt starts: Motion::moveStartMs() moves to now, and the kick is masked.
  const uint32_t tiltStart = hal::nowMs();
  setA2(500 + STALL_DELTA_COUNTS * 3);
  t.samples(kIgnoreSamples / 2, kBoth, tiltStart);
  CHECK(t.d.ignoring());
  CHECK(!t.d.stallPending());
}

TEST(the_trace_reports_the_window_extremes_and_resets) {
  Det t(500);
  StallTrace tr;
  CHECK(!t.d.takeTrace(tr));   // nothing sampled since begin
  const uint32_t move = t.oldMove();
  setA2(520);
  t.samples(10, axisBit(AXIS_PAN), move);
  setA2(470);
  t.samples(3, axisBit(AXIS_PAN), move);
  setA2(505);
  t.sample(axisBit(AXIS_PAN), move);
  CHECK(t.d.takeTrace(tr));
  CHECK_EQ(tr.samples, 14);
  CHECK_EQ(tr.minSample, 470);
  CHECK_EQ(tr.maxSample, 520);
  CHECK_EQ(tr.sample, 505);
  CHECK_EQ(tr.baseline, t.d.baseline());
  CHECK(tr.maxDelta >= 29);   // 470 against a baseline near 500
  CHECK(tr.moving);
  CHECK(!tr.ignoring);
  CHECK(tr.active);
  CHECK(!t.d.takeTrace(tr));   // the window was reset
  t.sample(0, move);
  CHECK(t.d.takeTrace(tr));
  CHECK_EQ(tr.samples, 1);
  CHECK(!tr.moving);
}

TEST(inactive_it_measures_but_never_declares) {
  Det t(500, false);
  CHECK_EQ(t.d.active(), STALL_DETECT_ENABLED);
  if (STALL_DETECT_ENABLED) return;   // config.h has it on: nothing inactive to test
  const uint32_t move = t.oldMove();
  setA2(500 + STALL_DELTA_COUNTS * 2);
  t.samples(STALL_CONFIRM_SAMPLES * 4, kBoth, move);
  CHECK(!t.d.stallPending());
  CHECK(!t.d.takeStall());
  CHECK_EQ(t.d.stallCount(), 0u);
  CHECK_EQ(t.d.triggers(), 4u);   // what detection would have done
  CHECK_EQ(t.d.lastStallPeak(), STALL_DELTA_COUNTS * 2);
  CHECK_EQ(t.d.peakDelta(), STALL_DELTA_COUNTS * 2);
  // Forced on (CAL_STALL_SENSE), the same signal is a stall.
  t.d.setForced(true);
  CHECK(t.d.active());
  t.samples(STALL_CONFIRM_SAMPLES, kBoth, move);
  CHECK(t.d.takeStall());
}

// ---- wired to Motion ---------------------------------------------------------------------------------

namespace {

// App's order (App::loop): StallDetector, then TurretState (here: the stall rule of
// architecture.md 6 and the calibration forwarding of decision a), then Motion.
struct Turret {
  StallDetector stall;
  Motion m;
  uint32_t backOffs = 0;
  uint32_t forwarded = 0;
  Turret() {
    hal_host::reset();
    hal::begin();
    hal_host::setAnalog(PIN_STALL_SENSE, 512);
    stall.begin(hal::nowUs());
    m.begin(stall);
  }
  void loopOnce(uint32_t us) {
    hal_host::advanceUs(us);
    const uint32_t nowMs = hal::nowMs();
    stall.update(hal::nowUs(), nowMs, m.movingAxes(), m.moveStartMs());
    if (stall.takeStall()) {
      if (m.calibrationWantsStall()) {
        if (m.notifyCalibrationStall(nowMs)) forwarded++;
      } else if (m.calibrationStatus() != Motion::CalStatus::RUNNING) {
        m.backOff(stall.stallAxes(), STALL_BACKOFF_DEG);
        backOffs++;
      }   // CAL_RAM: stall events are expected and ignored
    }
    m.update(nowMs, hal::nowUs());
  }
};

}  // namespace

// Spec 11 item 15 (the motion half; FAULT, LEDs and LCD are TurretState's): with the detector on
// and a pot on A2, a sudden twist while moving stops the moving axis and backs it off.
TEST(a_pot_twist_stops_the_turret_and_backs_it_off) {
  Turret t;
  t.stall.setForced(true);   // stands in for STALL_DETECT_ENABLED = true
  Rng rng(11);
  t.m.setVelocity(1.0f, 0.0f);
  for (int i = 0; i < 10000; i++) {   // 1 s at full pan speed, 100 us loop, pot noise
    setA2(512.0f + rng.uniform(-8.0f, 8.0f));
    t.loopOnce(100);
  }
  CHECK(t.m.isMoving());
  CHECK_EQ(t.backOffs, 0u);
  // Twist: a step to 800 (as fast as a hand can turn a pot).
  const uint64_t twistUs = hal_host::timeUs();
  setA2(800);
  float stallPos = 0.0f;
  uint64_t stoppedUs = 0;
  for (int i = 0; i < 20000 && stoppedUs == 0; i++) {
    t.loopOnce(100);
    if (t.backOffs == 1) {
      stallPos = t.m.positionDeg(AXIS_PAN);
      stoppedUs = hal_host::timeUs();
      CHECK_NEAR(hal::stepRate(AXIS_PAN), 0.0f, 0.0f);   // pulses stopped in the same pass
    }
  }
  CHECK_EQ(t.backOffs, 1u);
  // STALL_CONFIRM_SAMPLES samples at STALL_SAMPLE_HZ, plus one sample period of phase.
  CHECK(stoppedUs - twistUs <= (uint64_t)(STALL_CONFIRM_SAMPLES + 1) * kPeriodUs);
  CHECK_EQ(t.stall.stallAxes(), axisBit(AXIS_PAN));
  setA2(512);   // let go of the pot
  for (int i = 0; i < 30000 && t.m.backingOff(); i++) t.loopOnce(100);
  CHECK(!t.m.backingOff());
  CHECK(!t.m.isMoving());
  CHECK_NEAR(t.m.positionDeg(AXIS_PAN), stallPos - STALL_BACKOFF_DEG, 1.0f / PAN_STEPS_PER_DEG);
  CHECK_EQ(t.backOffs, 1u);   // the back-off's own start kick was masked
}

// CAL_STALL_SENSE in the physics model: A2 jumps while the barrel is pushing against the panel
// (steps are being lost), the forced detector confirms it, TurretState forwards it, and the
// calibration ends exactly TILT_CAL_BACKOFF_DEG above the stop.
TEST(cal_stall_sense_end_to_end_with_a_sensed_stop) {
  for (float h : {20.0f, 0.0f}) {   // 0: the barrel rests on the panel when calibration starts
  Turret t;
  const int64_t down = TILT_DIR_INVERT ? 1 : -1;
  const int64_t stop = down * (int64_t)(h * TILT_STEPS_PER_DEG + 0.5f);
  hal_host::setTiltBottomStop(true, stop);
  t.m.startCalibration(hal::nowMs(), CAL_STALL_SENSE);
  CHECK(t.stall.active());
  uint32_t lostSeen = 0;
  uint64_t lastLossUs = 0;
  bool sawSettle = false;
  for (int i = 0; i < 400000 && t.m.calibrationStatus() == Motion::CalStatus::RUNNING; i++) {
    if (hal_host::lostSteps(AXIS_TILT) != lostSeen) {
      lostSeen = hal_host::lostSteps(AXIS_TILT);
      lastLossUs = hal_host::timeUs();
    }
    const bool pushing = lastLossUs != 0 && hal_host::timeUs() - lastLossUs < 20000;
    setA2(pushing ? 512.0f + 250.0f : 512.0f);
    t.loopOnce(100);
    if (t.m.calPhase() == Motion::CalPhase::SETTLE) sawSettle = true;
  }
  CHECK(sawSettle);
  CHECK_EQ(t.forwarded, 1u);
  CHECK_EQ(t.backOffs, 0u);
  CHECK(t.m.calibrationStatus() == Motion::CalStatus::DONE);
  const int64_t above = (hal_host::trueSteps(AXIS_TILT) - stop) * -down;
  CHECK_EQ(above, (int64_t)(TILT_CAL_BACKOFF_DEG * TILT_STEPS_PER_DEG + 0.5f));
  CHECK_EQ(hal::stepPosition(AXIS_TILT), 0);
  CHECK(!t.stall.forced());
  CHECK_EQ(t.stall.active(), STALL_DETECT_ENABLED);
  // It stopped within a few steps of meeting the panel (plus the ignore window when it began
  // there), not after a long ram.
  CHECK(hal_host::lostSteps(AXIS_TILT) < 20u + (uint32_t)(TILT_CAL_SPEED_DEG_PER_S * TILT_STEPS_PER_DEG *
                                                          STALL_IGNORE_AFTER_START_MS / MS_PER_S));
  }
}

TESTKIT_MAIN()

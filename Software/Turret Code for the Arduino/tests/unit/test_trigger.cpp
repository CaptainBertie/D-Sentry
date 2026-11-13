// test_trigger.cpp -- Trigger (spec 3.4, architecture.md 5 and 6): the stroke timeline (sweep out
// at TRIGGER_SWEEP_DEG_PER_S, hold FIRE, sweep back, hold REST), semi and full auto, the semi
// cooldown, abort in every phase, the magazine (floor 0, lockout, reset), the flywheel gate, "a
// pull counts only if it started while ARMED", rumble requests, the firing-LED hold and the shot
// counter.
//
// Time is the module's own nowMs argument; the servo is observed through the host HAL
// (hal_host::servoAngle / servoOn), i.e. what the PWM would really be told. Every expected time
// is computed here from config.h (sweepFor below states the sweep rule), never read from the
// module under test.
//
// Safety property: a stroke never keeps running once armed goes false, and an aborted stroke
// never costs a dart, in whatever phase it is aborted
// (abort_in_every_phase_rests_now_never_decrements_and_starts_no_cooldown,
// disarm_mid_stroke_rests_at_once_without_an_abort_call).
//
// SPDX-License-Identifier: MIT
#include <algorithm>
#include <cmath>
#include <vector>

#include "Trigger.h"
#include "hal.h"
#include "hal_host.h"
#include "testkit.h"

// The servo angle in config.h terms: the host HAL records what the servo receives, which
// TRIGGER_SERVO_REVERSED mirrors; the mirror is its own inverse.
static float servoLogical() { return Trigger::physicalDeg(hal_host::servoAngle()); }

namespace {

using Phase = Trigger::Phase;

constexpr uint32_t kFwd = TRIGGER_FORWARD_MS;
constexpr uint32_t kRet = TRIGGER_RETURN_MS;
constexpr uint32_t kCool = TRIGGER_COOLDOWN_MS;
constexpr float kRate = TRIGGER_SWEEP_DEG_PER_S;
constexpr double kDegTol = 1e-3;   // float rounding of a commanded angle mid sweep

// The sweep rule (Trigger.h): with an update every ms, a sweep of spanDeg lands at the first
// whole ms whose distance at the rate covers the span; 0 without a sweep (rate 0) or a span.
uint32_t sweepFor(float spanDeg, float degPerS = kRate) {
  if (!(degPerS > 0.0f) || !(spanDeg > 0.0f)) return 0;
  uint32_t ms = 0;
  while (degPerS * (float)ms / (float)MS_PER_S < spanDeg) ms++;
  return ms;
}
float spanOf(float a, float b) { return std::fabs(a - b); }
// The angle a sweep from `from` toward `to` commands `ms` after it started (before it lands).
float sweptTo(float from, float to, uint32_t ms, float degPerS = kRate) {
  const float travel = degPerS * (float)ms / (float)MS_PER_S;
  return to > from ? from + travel : from - travel;
}

const uint32_t kSweep = sweepFor(spanOf(TRIGGER_FIRE_ANGLE, TRIGGER_REST_ANGLE));
const uint32_t kStroke = kSweep + kFwd + kSweep + kRet;   // config.h's angles, from REST

// Where each phase starts, counted from the stroke's start, and how long it lasts.
uint32_t phaseStart(Phase p, uint32_t sweep = kSweep) {
  switch (p) {
    case Phase::SWEEP_OUT: return 0;
    case Phase::HOLD_FIRE: return sweep;
    case Phase::SWEEP_BACK: return sweep + kFwd;
    case Phase::HOLD_REST: return sweep + kFwd + sweep;
    case Phase::IDLE: break;
  }
  return sweep + kFwd + sweep + kRet;
}
uint32_t phaseLength(Phase p, uint32_t sweep = kSweep) {
  switch (p) {
    case Phase::SWEEP_OUT:
    case Phase::SWEEP_BACK: return sweep;
    case Phase::HOLD_FIRE: return kFwd;
    case Phase::HOLD_REST: return kRet;
    case Phase::IDLE: break;
  }
  return 0;
}
const Phase kStrokePhases[] = {Phase::SWEEP_OUT, Phase::HOLD_FIRE, Phase::SWEEP_BACK, Phase::HOLD_REST};
const char* phaseName(Phase p) {
  switch (p) {
    case Phase::IDLE: return "IDLE";
    case Phase::SWEEP_OUT: return "SWEEP_OUT";
    case Phase::HOLD_FIRE: return "HOLD_FIRE";
    case Phase::SWEEP_BACK: return "SWEEP_BACK";
    case Phase::HOLD_REST: return "HOLD_REST";
  }
  return "?";
}

Trigger::Input in(bool armed, bool pulled, bool ready = true) {
  Trigger::Input i;
  i.armed = armed;
  i.pulled = pulled;
  i.flywheelReady = ready;
  return i;
}

// A booted host, a Trigger begun with its servo powered (as TurretState does on leaving OFF).
struct Rig {
  Trigger t;
  uint32_t now = 1000;
  explicit Rig(const Trigger::Config& c = Trigger::defaults()) : t(c) {
    hal_host::reset();
    hal::begin();
    t.begin();
    t.setServoPower(true);
  }
  // One update at the current time.
  void step(const Trigger::Input& i) { t.update(now, i); }
  // Updates every millisecond for `ms` ms (the first one at now + 1).
  void run(uint32_t ms, const Trigger::Input& i) {
    for (uint32_t k = 0; k < ms; k++) {
      now++;
      t.update(now, i);
    }
  }
  // Fires `n` semi strokes, each pulled, held to completion, released and the cooldown waited out.
  void fireSemi(int n) {
    for (int k = 0; k < n; k++) {
      step(in(true, true));
      run(kStroke, in(true, true));
      run(1, in(true, false));
      run(kCool, in(true, false));
    }
  }
  bool servoAtFire() const { return servoLogical() == (float)TRIGGER_FIRE_ANGLE; }
  bool servoAtRest() const { return servoLogical() == (float)TRIGGER_REST_ANGLE; }
};

Trigger::Config cfg(bool lockout, bool requiresFly, bool continuous) {
  Trigger::Config c = Trigger::defaults();
  c.lockoutWhenEmpty = lockout;
  c.requiresFlywheel = requiresFly;
  c.rumbleFullAutoContinuous = continuous;
  return c;
}

// Lockout on and the flywheel gate on, whatever config.h ships: the tests that rely on either.
Trigger::Config strict() { return cfg(true, true, RUMBLE_FULL_AUTO_CONTINUOUS); }

// One stroke as it happened: the commanded angle (config.h terms) and the phase after the
// update that started it (index 0) and after every later millisecond, until idle again.
struct Trace {
  std::vector<float> deg;
  std::vector<Phase> phase;
  std::vector<uint8_t> ammo;
  bool halAgrees = true;   // the HAL got the mirrored angle every time (TRIGGER_SERVO_REVERSED)
};
void record(Rig& r, Trace& tr) {
  tr.deg.push_back(r.t.servoDeg());
  tr.phase.push_back(r.t.phase());
  tr.ammo.push_back(r.t.ammo());
  tr.halAgrees = tr.halAgrees && std::fabs(servoLogical() - r.t.servoDeg()) <= kDegTol;
}
// Runs `i` until the stroke that `start` begins is over (at most `maxMs`).
template <typename Start>
Trace traceStroke(Rig& r, const Trigger::Input& i, uint32_t maxMs, Start start) {
  Trace tr;
  start();
  record(r, tr);
  for (uint32_t k = 0; k < maxMs && r.t.strokeActive(); k++) {
    r.run(1, i);
    record(r, tr);
  }
  return tr;
}

// The stroke shape (Trigger.h): from `from` a sweep at `rate` to `fire`, `fire` exactly for
// TRIGGER_FORWARD_MS, a sweep back to `rest`, `rest` exactly for TRIGGER_RETURN_MS, then idle.
// Every sweep step is the rate's distance from the sweep's start, monotonic, never past its end.
bool strokeShapeIs(const Trace& tr, float from, float fire, float rest, float rate = kRate) {
  const uint32_t out = sweepFor(spanOf(fire, from), rate);
  const uint32_t back = sweepFor(spanOf(rest, fire), rate);
  const size_t total = out + kFwd + back + kRet;
  bool ok = tr.deg.size() == total + 1 && tr.halAgrees;
  if (!CHECK(ok)) {
    std::printf("      stroke lasted %zu ms, expected %zu; HAL agreed: %d\n", tr.deg.size() - 1, total, tr.halAgrees);
    return false;
  }
  for (size_t k = 0; k <= total && ok; k++) {
    const float d = tr.deg[k];
    if (k < out) {   // sweeping out: k ms after the start
      ok = tr.phase[k] == Phase::SWEEP_OUT && std::fabs(d - sweptTo(from, fire, (uint32_t)k, rate)) <= kDegTol &&
           spanOf(d, fire) > 0.0f && spanOf(d, from) < spanOf(fire, from);
      if (k > 0) ok = ok && spanOf(d, fire) < spanOf(tr.deg[k - 1], fire);   // strictly toward FIRE
    } else if (k < out + kFwd) {
      ok = tr.phase[k] == Phase::HOLD_FIRE && d == fire;   // landed exactly, held
    } else if (k < out + kFwd + back) {
      const uint32_t j = (uint32_t)(k - out - kFwd);
      ok = tr.phase[k] == Phase::SWEEP_BACK && std::fabs(d - sweptTo(fire, rest, j, rate)) <= kDegTol &&
           spanOf(d, rest) > 0.0f && spanOf(d, fire) < spanOf(rest, fire);
      if (j > 0) ok = ok && spanOf(d, rest) < spanOf(tr.deg[k - 1], rest);   // strictly toward REST
    } else if (k < total) {
      ok = tr.phase[k] == Phase::HOLD_REST && d == rest;
    } else {
      ok = tr.phase[k] == Phase::IDLE && d == rest;
    }
    if (!ok) std::printf("      stroke shape broken at %zu ms: %s at %.4f\n", k, phaseName(tr.phase[k]), (double)d);
  }
  return CHECK(ok);
}

}  // namespace

TEST(defaults_come_from_config_h) {
  const Trigger::Config c = Trigger::defaults();
  CHECK_EQ(c.lockoutWhenEmpty, FIRE_LOCKOUT_WHEN_EMPTY);
  CHECK_EQ(c.requiresFlywheel, FIRE_REQUIRES_FLYWHEEL);
  CHECK_EQ(c.rumbleFullAutoContinuous, RUMBLE_FULL_AUTO_CONTINUOUS);
  CHECK_EQ(c.sweepDegPerS, TRIGGER_SWEEP_DEG_PER_S);
  CHECK_EQ(c.semiCooldownMs, (uint32_t)TRIGGER_COOLDOWN_MS);
  // The sweep rule agrees with the plain arithmetic for config.h's whole-degree angles.
  if (kRate > 0.0f)
    CHECK_EQ(kSweep, (uint32_t)std::ceil(std::fabs((double)TRIGGER_FIRE_ANGLE - (double)TRIGGER_REST_ANGLE) *
                                         (double)MS_PER_S / (double)kRate));
  std::printf("info stroke at config.h angles: sweep %u ms each way, stroke %u ms, semi cycle %u ms\n",
              (unsigned)kSweep, (unsigned)kStroke, (unsigned)(kStroke + kCool));
}

TEST(begin_parks_the_servo_at_rest_with_a_full_magazine) {
  hal_host::reset();
  hal::begin();
  Trigger t;
  t.begin();
  CHECK(!hal_host::servoOn());
  CHECK_EQ(servoLogical(), (float)TRIGGER_REST_ANGLE);
  CHECK_EQ(t.phase(), Phase::IDLE);
  CHECK_EQ(t.ammo(), MAG_CAPACITY);
  CHECK_EQ(t.capacity(), MAG_CAPACITY);
  CHECK_EQ(t.shotsTotal(), 0);
  CHECK_EQ(t.fireMode(), DS_FIRE_MODE_SEMI);
  CHECK(!t.coolingDown());
  // Unpowered servo: armed and ready still cannot fire.
  CHECK_EQ(t.gate(in(true, true)), Trigger::Gate::SERVO_OFF);
  t.update(1000, in(true, false));
  t.update(1001, in(true, true));
  CHECK(!t.strokeActive());
  t.setServoPower(true);
  CHECK(hal_host::servoOn());
  CHECK_EQ(servoLogical(), (float)TRIGGER_REST_ANGLE);
  CHECK_EQ(t.gate(in(true, true)), Trigger::Gate::OPEN);
}

// The whole stroke at config.h's angles, ms by ms: sweep out, FIRE held, sweep back, REST held;
// the magazine and the shot counter change exactly once, at the completing update, never before.
TEST(one_stroke_timeline_exactly_and_ammo_only_at_completion) {
  Rig r;
  r.step(in(true, false));
  r.now = 5000;
  const uint32_t t0 = r.now;
  const Trace tr = traceStroke(r, in(true, true), 2 * kStroke, [&] { r.step(in(true, true)); });
  CHECK(strokeShapeIs(tr, TRIGGER_REST_ANGLE, TRIGGER_FIRE_ANGLE, TRIGGER_REST_ANGLE));
  CHECK_EQ(r.now, t0 + kStroke);
  bool fullUntilTheEnd = tr.ammo.size() == kStroke + 1;
  for (size_t k = 0; k + 1 < tr.ammo.size(); k++) fullUntilTheEnd = fullUntilTheEnd && tr.ammo[k] == MAG_CAPACITY;
  CHECK(fullUntilTheEnd);                        // not one ms early, not mid hold
  CHECK_EQ(r.t.phase(), Phase::IDLE);
  CHECK_EQ(r.t.ammo(), MAG_CAPACITY - 1);
  CHECK_EQ(r.t.shotsTotal(), 1);
  CHECK(r.servoAtRest());
  CHECK_EQ(r.t.strokeMs(), kStroke);             // what the module reports for rumble / docs
  CHECK_EQ(r.t.sweepMs(TRIGGER_REST_ANGLE, TRIGGER_FIRE_ANGLE), kSweep);
  CHECK_EQ(r.t.sweepMs(TRIGGER_FIRE_ANGLE, TRIGGER_REST_ANGLE), kSweep);
}

// The sweep for either sign of FIRE - REST, the worst-case span and a short one: at the configured
// rate, monotonic, landing exactly on FIRE and REST, each hold its configured time.
TEST(sweep_glides_at_the_rate_for_either_direction_and_lands_exactly) {
  struct Angles {
    float rest, fire;
  };
  const Angles cases[] = {{TRIGGER_REST_ANGLE, TRIGGER_FIRE_ANGLE},
                          {TRIGGER_FIRE_ANGLE, TRIGGER_REST_ANGLE},
                          {0.0f, (float)SERVO_SPAN_DEG},
                          {(float)SERVO_SPAN_DEG, 0.0f},
                          {90.0f, 93.0f}};
  for (const Angles& a : cases) {
    Rig r;
    r.t.setAngles(a.rest, a.fire);
    r.t.abort();                                   // the servo to this REST first
    CHECK_EQ(servoLogical(), a.rest);
    r.step(in(true, false));
    const Trace tr = traceStroke(r, in(true, true), 4 * MS_PER_S, [&] { r.step(in(true, true)); });
    CHECK(strokeShapeIs(tr, a.rest, a.fire, a.rest));
    CHECK_EQ(r.t.shotsTotal(), 1);
    CHECK_EQ(r.t.strokeMs(), sweepFor(spanOf(a.fire, a.rest)) * 2 + kFwd + kRet);
  }
}

// TRIGGER_SWEEP_DEG_PER_S 0: no sweep, each angle commanded at once (the old two-step stroke).
TEST(no_sweep_at_rate_zero_jumps_to_each_angle_as_before) {
  Trigger::Config c = Trigger::defaults();
  c.sweepDegPerS = 0.0f;
  Rig r(c);
  r.step(in(true, false));
  r.run(1, in(true, true));
  CHECK_EQ(r.t.phase(), Phase::HOLD_FIRE);       // FIRE in the update that starts the stroke
  CHECK(r.servoAtFire());
  r.run(kFwd - 1, in(true, true));
  CHECK(r.servoAtFire());
  r.run(1, in(true, true));
  CHECK_EQ(r.t.phase(), Phase::HOLD_REST);
  CHECK(r.servoAtRest());
  r.run(kRet - 1, in(true, true));
  CHECK_EQ(r.t.ammo(), MAG_CAPACITY);
  r.run(1, in(true, true));
  CHECK_EQ(r.t.phase(), Phase::IDLE);
  CHECK_EQ(r.t.ammo(), MAG_CAPACITY - 1);
  CHECK_EQ(r.t.strokeMs(), kFwd + kRet);
  CHECK_EQ(r.t.sweepMs(TRIGGER_REST_ANGLE, TRIGGER_FIRE_ANGLE), 0u);
}

TEST(semi_one_stroke_per_pull_and_held_trigger_does_not_refire) {
  Rig r;
  r.step(in(true, false));
  r.run(1, in(true, true));
  CHECK(r.t.strokeActive());
  int starts = 0;
  bool wasActive = true;
  for (uint32_t k = 0; k < 10 * (kStroke + kCool); k++) {   // held across the stroke's end and long after
    r.run(1, in(true, true));
    if (r.t.strokeActive() && !wasActive) starts++;
    wasActive = r.t.strokeActive();
  }
  CHECK_EQ(starts, 0);
  CHECK_EQ(r.t.ammo(), MAG_CAPACITY - 1);
  CHECK(r.servoAtRest());
}

TEST(semi_release_rearms_and_each_pull_costs_exactly_one_dart) {
  Rig r;
  r.step(in(true, false));
  for (int k = 1; k <= 3; k++) {
    r.run(1, in(true, true));
    CHECK(r.t.strokeActive());
    r.run(kStroke, in(true, true));
    CHECK(!r.t.strokeActive());
    CHECK_EQ(r.t.ammo(), MAG_CAPACITY - k);
    r.run(5, in(true, false));                    // released below the release threshold
    r.run(kCool, in(true, false));                // and the cooldown waited out
  }
  CHECK_EQ(r.t.shotsTotal(), 3);
}

// Semi cooldown: after a stroke completes, nothing starts for TRIGGER_COOLDOWN_MS (gate COOLDOWN);
// a counting pull still held when it ends fires in that very update, also one that started
// during the stroke itself.
TEST(semi_cooldown_a_held_pull_fires_the_moment_it_ends) {
  for (int when = 0; when < 2; when++) {             // re-pulled during the stroke / the cooldown
    Rig r;
    r.step(in(true, false));
    r.run(1, in(true, true));
    const uint32_t t0 = r.now;
    if (when == 0) {
      r.run(kStroke / 2, in(true, true));
      r.run(10, in(true, false));                    // released mid-stroke
      r.run(t0 + kStroke - r.now, in(true, true));   // pulled again, held to the completion
    } else {
      r.run(kStroke, in(true, true));
    }
    CHECK_EQ(r.now, t0 + kStroke);
    CHECK_EQ(r.t.phase(), Phase::IDLE);              // completed ...
    CHECK_EQ(r.t.ammo(), MAG_CAPACITY - 1);
    CHECK(r.t.coolingDown());                        // ... and cooling down
    CHECK_EQ(r.t.gate(in(true, true)), Trigger::Gate::COOLDOWN);
    CHECK_EQ(r.t.cooldownMs(), kCool);
    if (when == 1) {
      r.run(kCool / 2, in(true, false));             // released ...
      r.run(1, in(true, true));                      // ... and pulled inside the cooldown
    }
    bool early = false;
    while (r.now < t0 + kStroke + kCool - 1) {
      r.run(1, in(true, true));
      early = early || r.t.strokeActive();
    }
    CHECK(!early);
    CHECK_EQ(r.t.gate(in(true, true)), Trigger::Gate::COOLDOWN);
    r.run(1, in(true, true));                        // completion + TRIGGER_COOLDOWN_MS
    CHECK_EQ(r.t.phase(), kSweep ? Phase::SWEEP_OUT : Phase::HOLD_FIRE);
    CHECK(!r.t.coolingDown());
    CHECK_EQ(r.t.ammo(), MAG_CAPACITY - 1);          // the new stroke has not completed yet
    r.run(kStroke, in(true, true));
    CHECK_EQ(r.t.ammo(), MAG_CAPACITY - 2);
    r.run(3 * (kStroke + kCool), in(true, true));    // held on: no third
    CHECK_EQ(r.t.ammo(), MAG_CAPACITY - 2);
  }
}

// A tap that starts and ends inside the cooldown is dropped, also a re-pull during the stroke
// released before the cooldown ends.
TEST(semi_cooldown_a_tap_inside_it_never_fires) {
  for (int when = 0; when < 2; when++) {
    Rig r;
    r.step(in(true, false));
    r.run(1, in(true, true));
    const uint32_t t0 = r.now;
    if (when == 0) {
      r.run(kStroke + 1, in(true, false));           // released, stroke completes
      r.run(kCool / 4, in(true, true));              // a tap inside the cooldown
      r.run(1, in(true, false));
    } else {
      r.run(kStroke / 2, in(true, false));
      r.run(t0 + kStroke + kCool - 1 - r.now, in(true, true));   // re-pulled, let go 1 ms early
      r.run(1, in(true, false));
    }
    r.run(3 * (kStroke + kCool), in(true, false));
    CHECK(!r.t.strokeActive());
    CHECK_EQ(r.t.ammo(), MAG_CAPACITY - 1);
    CHECK_EQ(r.t.shotsTotal(), 1);
    r.run(1, in(true, true));                        // a fresh pull after it: fires
    CHECK(r.t.strokeActive());
  }
}

// Only a completed real stroke starts a cooldown: not begin(), not an aborted stroke, not a
// teach-mode test stroke.
TEST(cooldown_starts_only_when_a_real_stroke_completes) {
  Rig r;
  CHECK_EQ(r.t.gate(in(true, false)), Trigger::Gate::OPEN);
  r.step(in(true, false));
  r.run(1, in(true, true));
  r.run(kStroke - 1, in(true, true));               // its last ms
  CHECK(r.t.strokeActive());
  r.t.abort();
  CHECK(!r.t.coolingDown());
  r.run(1, in(true, false));
  r.run(1, in(true, true));                         // fires at once
  CHECK(r.t.strokeActive());
  r.t.abort();
  // A test stroke run to completion.
  Trigger::Input teach;
  teach.servoTeach = true;
  r.run(1, teach);
  CHECK(r.t.testStroke(r.now, teach));
  r.run(kStroke, teach);
  CHECK(!r.t.strokeActive());
  CHECK(!r.t.coolingDown());
  r.run(1, in(true, false));
  r.run(1, in(true, true));
  CHECK(r.t.strokeActive());
  CHECK_EQ(r.t.shotsTotal(), 0);
}

TEST(full_auto_back_to_back_with_no_gap_and_release_finishes_the_stroke) {
  Rig r;
  r.t.setFireMode(DS_FIRE_MODE_FULL);
  CHECK_EQ(r.t.cooldownMs(), 0u);
  r.step(in(true, false));
  std::vector<uint32_t> starts;
  bool everIdleWhileHeld = false, everCooldown = false;
  Phase was = Phase::IDLE;
  const Phase first = kSweep ? Phase::SWEEP_OUT : Phase::HOLD_FIRE;   // a stroke's first phase
  auto watch = [&] {
    const Phase p = r.t.phase();
    if (p == first && was != first) starts.push_back(r.now);
    was = p;
    everCooldown = everCooldown || r.t.coolingDown() || r.t.gate(in(true, true)) == Trigger::Gate::COOLDOWN;
  };
  r.run(1, in(true, true));
  watch();
  const uint32_t t0 = r.now;
  while (r.now < t0 + 3 * kStroke + kStroke / 4) {   // three strokes and a quarter of the fourth
    r.run(1, in(true, true));
    if (!r.t.strokeActive()) everIdleWhileHeld = true;
    watch();
  }
  CHECK(!everIdleWhileHeld);                        // never a gap between strokes
  CHECK(!everCooldown);                             // full auto has no cooldown
  CHECK_EQ(starts.size(), 4u);
  for (size_t k = 1; k < starts.size(); k++) CHECK_EQ(starts[k] - starts[k - 1], kStroke);
  CHECK_EQ(r.t.ammo(), MAG_CAPACITY - 3);
  // Release in the fourth stroke: it finishes, then nothing more.
  r.run(1, in(true, false));
  CHECK(r.t.strokeActive());
  r.run(kStroke, in(true, false));
  CHECK(!r.t.strokeActive());
  CHECK_EQ(r.t.ammo(), MAG_CAPACITY - 4);
  r.run(3 * kStroke, in(true, false));
  CHECK_EQ(r.t.ammo(), MAG_CAPACITY - 4);
  CHECK_EQ(r.t.shotsTotal(), 4);
}

// The cooldown belongs to semi: after a semi stroke, a fresh full-auto pull fires at once; after a
// full-auto stroke, a fresh semi pull waits out TRIGGER_COOLDOWN_MS from that completion.
TEST(the_cooldown_is_semi_only_and_follows_the_mode_in_force) {
  Rig r;
  r.step(in(true, false));
  r.run(1, in(true, true));
  r.run(kStroke, in(true, true));                   // a semi stroke completes
  CHECK(r.t.coolingDown());
  r.t.setFireMode(DS_FIRE_MODE_FULL);
  r.run(1, in(true, false));
  r.run(1, in(true, true));
  CHECK(r.t.strokeActive());                        // full auto: no cooldown
  r.run(1, in(true, false));                        // released: this one stroke finishes
  r.run(kStroke, in(true, false));
  CHECK(!r.t.strokeActive());
  CHECK_EQ(r.t.ammo(), MAG_CAPACITY - 2);
  const uint32_t done = r.now - 1;                   // when it completed (it started 1 ms before the release)
  r.t.setFireMode(DS_FIRE_MODE_SEMI);
  r.run(1, in(true, true));
  CHECK_EQ(r.t.gate(in(true, true)), Trigger::Gate::COOLDOWN);
  r.run(done + kCool - 1 - r.now, in(true, true));
  CHECK(!r.t.strokeActive());
  r.run(1, in(true, true));
  CHECK(r.t.strokeActive());                        // held through it: fires as it ends
}

// KEY SAFETY TEST: an abort in any phase (abort(), armed dropping without an abort() call, the
// servo powered off) commands REST at once, no sweep, never costs a dart or counts a shot and
// starts no cooldown; the servo stays at REST afterwards.
TEST(abort_in_every_phase_rests_now_never_decrements_and_starts_no_cooldown) {
  enum Kind { ABORT_CALL, DISARMED, POWER_OFF, KINDS };
  const char* const kinds[] = {"abort()", "armed false", "servo power off"};
  for (const Phase p : kStrokePhases) {
    if (phaseLength(p) == 0) continue;               // no sweep configured: no sweep phases
    for (uint32_t at : {phaseStart(p) + phaseLength(p) / 2, phaseStart(p) + phaseLength(p) - 1}) {
      for (int kind = ABORT_CALL; kind < KINDS; kind++) {
        Rig r;
        r.step(in(true, false));
        r.run(1, in(true, true));
        r.run(at, in(true, true));
        if (!CHECK_EQ(r.t.phase(), p)) std::printf("      %s at +%u ms, %s\n", phaseName(p), (unsigned)at, kinds[kind]);
        Trigger::Rumble ignore;
        r.t.takeRumbleRequest(ignore);
        if (kind == ABORT_CALL) r.t.abort();
        if (kind == DISARMED) r.run(1, in(false, true));
        if (kind == POWER_OFF) r.t.setServoPower(false);
        CHECK(r.servoAtRest());                      // in that very call: no sweep back
        CHECK_EQ(r.t.servoDeg(), (float)TRIGGER_REST_ANGLE);
        CHECK_EQ(r.t.phase(), Phase::IDLE);
        CHECK_EQ(r.t.ammo(), MAG_CAPACITY);
        CHECK_EQ(r.t.shotsTotal(), 0);
        CHECK(!r.t.coolingDown());
        CHECK(!r.t.firingLedActive(r.now));
        Trigger::Rumble stop;
        CHECK(r.t.takeRumbleRequest(stop));          // the stroke's rumble is stopped
        CHECK_EQ(stop.ms, 0);
        bool stayedAtRest = true;
        for (uint32_t k = 0; k < kStroke + kCool; k++) {   // still armed and held (or disarmed)
          r.run(1, in(kind != DISARMED, true));
          stayedAtRest = stayedAtRest && r.servoAtRest() && !r.t.strokeActive();
        }
        CHECK(stayedAtRest);                          // the aborted pull does not restart it
        CHECK_EQ(r.t.ammo(), MAG_CAPACITY);
        if (kind == POWER_OFF) r.t.setServoPower(true);
        r.run(1, in(true, false));                    // release and pull: fires again
        r.run(1, in(true, true));
        CHECK(r.t.strokeActive());
      }
    }
  }
}

TEST(abort_in_full_auto_stops_cycling_until_a_fresh_pull) {
  Rig r;
  r.t.setFireMode(DS_FIRE_MODE_FULL);
  r.step(in(true, false));
  r.run(1, in(true, true));
  r.run(kStroke + kStroke / 2, in(true, true));        // into the second stroke
  CHECK_EQ(r.t.ammo(), MAG_CAPACITY - 1);
  r.t.abort();                                         // still armed, still held
  r.run(5 * kStroke, in(true, true));
  CHECK(!r.t.strokeActive());
  CHECK_EQ(r.t.ammo(), MAG_CAPACITY - 1);
  r.run(1, in(true, false));
  r.run(1, in(true, true));
  CHECK(r.t.strokeActive());
}

TEST(abort_cancels_a_blocked_semi_pull) {
  Rig r(strict());
  r.step(in(true, false, false));
  r.run(1, in(true, true, false));                     // waiting for spin-up
  r.t.abort();
  r.run(5 * kStroke, in(true, true, true));            // ready now, still held
  CHECK(!r.t.strokeActive());
  CHECK_EQ(r.t.ammo(), MAG_CAPACITY);
  // A pull waiting on the cooldown too.
  Rig c;
  c.step(in(true, false));
  c.fireSemi(1);
  c.run(1, in(true, true));
  c.run(kStroke, in(true, true));                      // completed, cooling down
  c.run(1, in(true, false));
  c.run(1, in(true, true));                            // a new pull, blocked by the cooldown
  CHECK_EQ(c.t.gate(in(true, true)), Trigger::Gate::COOLDOWN);
  c.t.abort();
  c.run(kCool + kStroke, in(true, true));
  CHECK(!c.t.strokeActive());
  CHECK_EQ(c.t.ammo(), MAG_CAPACITY - 2);
}

TEST(abort_while_idle_requests_no_rumble) {
  Rig r;
  r.t.abort();
  Trigger::Rumble out;
  CHECK(!r.t.takeRumbleRequest(out));
  CHECK(r.servoAtRest());
}

TEST(disarm_mid_stroke_rests_at_once_without_an_abort_call) {
  Rig r;
  r.step(in(true, false));
  r.run(1, in(true, true));
  r.run(kSweep + kFwd / 2, in(true, true));
  CHECK(r.servoAtFire());
  r.run(1, in(false, true));                         // TurretState: no longer ARMED
  CHECK(r.servoAtRest());
  CHECK(!r.t.strokeActive());
  CHECK_EQ(r.t.ammo(), MAG_CAPACITY);
  r.run(3 * kStroke, in(false, true));
  CHECK(!r.t.strokeActive());
  CHECK_EQ(r.t.ammo(), MAG_CAPACITY);
}

TEST(servo_power_off_mid_stroke_aborts) {
  Rig r;
  r.step(in(true, false));
  r.run(1, in(true, true));
  r.t.setServoPower(false);
  CHECK(!r.t.strokeActive());
  CHECK(!hal_host::servoOn());
  CHECK_EQ(servoLogical(), (float)TRIGGER_REST_ANGLE);
  CHECK_EQ(r.t.ammo(), MAG_CAPACITY);
  CHECK_EQ(r.t.gate(in(true, true)), Trigger::Gate::SERVO_OFF);
}

TEST(ammo_floor_is_zero_when_lockout_is_off) {
  Rig r(cfg(false, FIRE_REQUIRES_FLYWHEEL, RUMBLE_FULL_AUTO_CONTINUOUS));
  r.step(in(true, false));
  r.fireSemi(MAG_CAPACITY + 3);
  CHECK_EQ(r.t.ammo(), 0);                            // never wraps to 255
  CHECK_EQ(r.t.shotsTotal(), MAG_CAPACITY + 3);       // kept cycling at 0
  CHECK_EQ(r.t.gate(in(true, true)), Trigger::Gate::OPEN);
  CHECK(r.t.fireReady(r.now, in(true, false)));       // ready at 0: the count blocks nothing
  r.run(1, in(true, true));
  CHECK(r.t.strokeActive());
  CHECK_EQ(r.t.gate(in(true, false)), Trigger::Gate::STROKE_RUNNING);
}

// The owner's choice (2026-11-03, FIRE_LOCKOUT_WHEN_EMPTY false): with config.h's defaults the
// turret keeps firing at 0, semi and full auto, and the count stays at 0 (never negative).
TEST(defaults_keep_firing_at_zero_and_the_count_stays_zero) {
  CHECK(!FIRE_LOCKOUT_WHEN_EMPTY);
  CHECK(!Trigger::defaults().lockoutWhenEmpty);
  Rig r;   // Trigger::defaults()
  r.step(in(true, false));
  r.fireSemi(MAG_CAPACITY);
  CHECK_EQ(r.t.ammo(), 0);
  CHECK_EQ(r.t.shotsTotal(), MAG_CAPACITY);
  r.fireSemi(3);                                      // semi at 0: each pull still strokes
  CHECK_EQ(r.t.ammo(), 0);
  CHECK_EQ(r.t.shotsTotal(), MAG_CAPACITY + 3);
  r.t.setFireMode(DS_FIRE_MODE_FULL);
  r.run(1, in(true, false));
  r.run(1, in(true, true));
  r.run(4 * kStroke - 1, in(true, true));             // full auto at 0: back to back
  CHECK_EQ(r.t.ammo(), 0);
  CHECK_EQ(r.t.shotsTotal(), MAG_CAPACITY + 3 + 3);
  CHECK(r.t.strokeActive());                          // the fourth is running
  r.run(1, in(true, false));
  r.run(2 * kStroke, in(true, false));
  CHECK_EQ(r.t.shotsTotal(), MAG_CAPACITY + 3 + 4);
  CHECK_EQ(r.t.ammo(), 0);
  r.t.resetMagazine();                                // X still reloads
  CHECK_EQ(r.t.ammo(), MAG_CAPACITY);
}

TEST(empty_magazine_with_lockout_on_never_strokes) {
  Rig r(cfg(true, FIRE_REQUIRES_FLYWHEEL, RUMBLE_FULL_AUTO_CONTINUOUS));
  r.step(in(true, false));
  r.fireSemi(MAG_CAPACITY);
  CHECK_EQ(r.t.ammo(), 0);
  CHECK_EQ(r.t.gate(in(true, true)), Trigger::Gate::EMPTY);
  CHECK(!r.t.fireReady(r.now, in(true, false)));      // the EMPTY gate counts for the LEDs too
  r.run(1, in(true, true));
  CHECK(!r.t.strokeActive());
  CHECK(r.servoAtRest());
  r.run(500, in(true, true));
  CHECK(!r.t.strokeActive());
  CHECK_EQ(r.t.shotsTotal(), MAG_CAPACITY);
  r.run(1, in(true, false));
  r.run(1, in(false, true));
  Trigger::Gate g = r.t.gate(in(false, true));
  CHECK_EQ(g, Trigger::Gate::NOT_ARMED);
  // X: magazine back, a new pull fires.
  r.run(1, in(true, false));
  r.t.resetMagazine();
  CHECK_EQ(r.t.ammo(), MAG_CAPACITY);
  CHECK(r.t.fireReady(r.now, in(true, false)));
  r.run(1, in(true, true));
  CHECK(r.t.strokeActive());
}

// Empty during the cooldown after the last dart: EMPTY is reported, the reason that lasts.
TEST(the_last_dart_reports_empty_rather_than_the_cooldown) {
  Rig r(strict());
  r.step(in(true, false));
  r.fireSemi(MAG_CAPACITY - 1);
  r.run(1, in(true, true));
  r.run(kStroke, in(true, true));
  CHECK_EQ(r.t.ammo(), 0);
  CHECK(r.t.coolingDown());
  CHECK_EQ(r.t.gate(in(true, true)), Trigger::Gate::EMPTY);
}

TEST(held_pull_blocked_by_an_empty_magazine_fires_once_after_reset) {
  Rig r(strict());
  r.step(in(true, false));
  r.fireSemi(MAG_CAPACITY);
  r.run(1, in(true, true));                           // pulled on empty: nothing
  CHECK(!r.t.strokeActive());
  r.t.resetMagazine();                                // X while still holding
  r.run(1, in(true, true));
  CHECK(r.t.strokeActive());
  r.run(5 * kStroke, in(true, true));
  CHECK_EQ(r.t.ammo(), MAG_CAPACITY - 1);             // once, not continuously
}

TEST(full_auto_stops_on_the_last_dart) {
  Rig r(strict());
  r.step(in(true, false));
  r.fireSemi(MAG_CAPACITY - 2);
  r.t.setFireMode(DS_FIRE_MODE_FULL);
  r.run(1, in(true, false));
  r.run(1, in(true, true));
  r.run(10 * kStroke, in(true, true));
  CHECK_EQ(r.t.ammo(), 0);
  CHECK_EQ(r.t.shotsTotal(), MAG_CAPACITY);
  CHECK(!r.t.strokeActive());
  CHECK_EQ(r.t.gate(in(true, true)), Trigger::Gate::EMPTY);
}

TEST(flywheel_gate_follows_fire_requires_flywheel) {
  {
    Rig r(cfg(FIRE_LOCKOUT_WHEN_EMPTY, true, RUMBLE_FULL_AUTO_CONTINUOUS));
    r.step(in(true, false, false));
    CHECK_EQ(r.t.gate(in(true, true, false)), Trigger::Gate::FLYWHEEL_NOT_READY);
    r.run(1, in(true, true, false));
    r.run(2 * kStroke, in(true, true, false));
    CHECK(!r.t.strokeActive());
    CHECK_EQ(r.t.ammo(), MAG_CAPACITY);
    CHECK(!r.t.fireOnce(r.now, in(true, false, false)));
  }
  {
    Rig r(cfg(FIRE_LOCKOUT_WHEN_EMPTY, false, RUMBLE_FULL_AUTO_CONTINUOUS));
    r.step(in(true, false, false));
    CHECK_EQ(r.t.gate(in(true, true, false)), Trigger::Gate::OPEN);
    r.run(1, in(true, true, false));                   // flywheel never ready: fires anyway
    CHECK(r.t.strokeActive());
  }
}

TEST(a_pull_counts_only_if_it_started_while_armed) {
  Rig r;
  r.step(in(false, false));
  r.run(10, in(false, true));                          // pulled while disarmed
  r.run(5 * kStroke, in(true, true));                  // then armed while still held
  CHECK(!r.t.strokeActive());
  CHECK_EQ(r.t.ammo(), MAG_CAPACITY);
  r.t.setFireMode(DS_FIRE_MODE_FULL);                  // full auto: same rule
  r.run(5 * kStroke, in(true, true));
  CHECK_EQ(r.t.ammo(), MAG_CAPACITY);
  r.run(1, in(true, false));                           // released ...
  r.run(1, in(true, true));                            // ... and pulled while armed: fires
  CHECK(r.t.strokeActive());

  // Armed pull, disarm while held, re-arm while still held: no stroke.
  Rig q;
  q.step(in(true, false));
  q.run(1, in(true, true));
  q.run(kStroke, in(true, true));                      // one semi stroke
  q.run(1, in(true, false));
  q.run(kCool, in(true, false));                       // the cooldown waited out
  q.run(1, in(true, true));
  CHECK(q.t.strokeActive());
  q.run(10, in(false, true));                          // disarmed: aborted
  q.run(5 * kStroke, in(true, true));                  // re-armed, still holding
  CHECK(!q.t.strokeActive());
  CHECK_EQ(q.t.ammo(), MAG_CAPACITY - 1);

  // Pulled in the very loop before arming (one disarmed update, then armed): no stroke.
  Rig p;
  p.step(in(false, false));
  p.run(1, in(false, true));
  p.run(5 * kStroke, in(true, true));
  CHECK(!p.t.strokeActive());
  CHECK_EQ(p.t.ammo(), MAG_CAPACITY);

  // A counting pull that is still blocked (no stroke yet), disarmed while held, re-armed while
  // held and then unblocked: it stopped counting at the disarm, so no stroke.
  Rig b(strict());
  b.step(in(true, false, false));
  b.run(1, in(true, true, false));                     // armed pull, flywheel not ready
  b.run(1, in(false, true, false));                    // disarmed, still held
  b.run(1, in(true, true, false));                     // armed again, still held
  b.run(5 * kStroke, in(true, true, true));            // flywheel ready
  CHECK(!b.t.strokeActive());
  CHECK_EQ(b.t.ammo(), MAG_CAPACITY);
}

TEST(semi_pull_held_through_spin_up_fires_once_when_ready) {
  Rig r(strict());
  r.step(in(true, false, false));
  r.run(1, in(true, true, false));                     // pulled armed, flywheel spinning up
  r.run(FLYWHEEL_SPINUP_MS, in(true, true, false));
  CHECK(!r.t.strokeActive());
  r.run(1, in(true, true, true));                      // ready, still held: fires now
  CHECK(r.t.strokeActive());
  r.run(10 * kStroke, in(true, true, true));
  CHECK_EQ(r.t.ammo(), MAG_CAPACITY - 1);              // once
  // Released while not ready: nothing queued.
  Rig q(strict());
  q.step(in(true, false, false));
  q.run(1, in(true, true, false));
  q.run(10, in(true, false, false));
  q.run(1, in(true, false, true));
  CHECK(!q.t.strokeActive());
}

TEST(rumble_request_per_stroke_semi) {
  Rig r;
  r.step(in(true, false));
  Trigger::Rumble out;
  CHECK(!r.t.takeRumbleRequest(out));
  r.run(1, in(true, true));
  CHECK(r.t.takeRumbleRequest(out));                   // raised at the stroke's start
  CHECK_EQ(out.strong, RUMBLE_STRONG_255);
  CHECK_EQ(out.weak, RUMBLE_WEAK_255);
  CHECK_EQ(out.ms, RUMBLE_DURATION_MS);
  CHECK(!r.t.takeRumbleRequest(out));                  // cleared by reading
  r.run(kStroke, in(true, true));
  CHECK(!r.t.takeRumbleRequest(out));                  // none at the stroke's end
}

TEST(rumble_full_auto_continuous_and_per_shot) {
  for (int continuous = 0; continuous < 2; continuous++) {
    Rig r(cfg(FIRE_LOCKOUT_WHEN_EMPTY, FIRE_REQUIRES_FLYWHEEL, continuous == 1));
    r.t.setFireMode(DS_FIRE_MODE_FULL);
    r.step(in(true, false));
    int requests = 0;
    bool allRight = true;
    std::vector<uint32_t> at;
    for (uint32_t k = 0; k < 4 * kStroke; k++) {
      r.run(1, in(true, true));
      Trigger::Rumble out;
      if (r.t.takeRumbleRequest(out)) {
        requests++;
        at.push_back(r.now);
        const uint16_t want = continuous ? (uint16_t)(kStroke + RUMBLE_DURATION_MS) : (uint16_t)RUMBLE_DURATION_MS;
        allRight = allRight && out.ms == want && out.strong == RUMBLE_STRONG_255 && out.weak == RUMBLE_WEAK_255;
      }
    }
    CHECK_EQ(requests, 4);                             // renewed every stroke
    CHECK(allRight);
    // Continuous: each request lasts past the next renewal, so the pad never stops while
    // cycling; per shot: each ends before the next stroke.
    for (size_t k = 1; k < at.size(); k++) {
      const uint32_t gap = at[k] - at[k - 1];
      if (continuous)
        CHECK(kStroke + RUMBLE_DURATION_MS > gap);
      else
        CHECK_EQ(gap, kStroke);
    }
  }
  // Semi with the continuous switch on: still the per-shot length.
  Rig s(cfg(FIRE_LOCKOUT_WHEN_EMPTY, FIRE_REQUIRES_FLYWHEEL, true));
  s.step(in(true, false));
  s.run(1, in(true, true));
  Trigger::Rumble out;
  CHECK(s.t.takeRumbleRequest(out));
  CHECK_EQ(out.ms, RUMBLE_DURATION_MS);
}

// The continuous rumble is one stroke at the angles in force (SERVO ADJUST records change it), and
// the worst case, a sweep over the whole SERVO_SPAN_DEG, still fits StatusPacket.rumbleMs (u16).
TEST(continuous_rumble_follows_the_angles_in_force_and_fits_u16) {
  struct Angles {
    float rest, fire;
  };
  for (const Angles a : {Angles{100.0f, 90.0f}, Angles{0.0f, (float)SERVO_SPAN_DEG}}) {
    Rig r(cfg(FIRE_LOCKOUT_WHEN_EMPTY, FIRE_REQUIRES_FLYWHEEL, true));
    r.t.setAngles(a.rest, a.fire);
    r.t.abort();
    r.t.setFireMode(DS_FIRE_MODE_FULL);
    r.step(in(true, false));
    Trigger::Rumble ignore;
    r.t.takeRumbleRequest(ignore);
    r.run(1, in(true, true));
    Trigger::Rumble out;
    CHECK(r.t.takeRumbleRequest(out));
    const uint32_t stroke = sweepFor(spanOf(a.fire, a.rest)) * 2 + kFwd + kRet;
    CHECK(stroke + RUMBLE_DURATION_MS <= UINT16_MAX);
    CHECK_EQ(out.ms, (uint16_t)(stroke + RUMBLE_DURATION_MS));
    // ... and it covers the gap to the next stroke's start.
    r.run(stroke, in(true, true));
    CHECK(r.t.takeRumbleRequest(out));
  }
}

TEST(firing_led_covers_the_stroke_and_at_least_the_minimum_flash) {
  Trigger::Config fast = Trigger::defaults();
  fast.sweepDegPerS = 0.0f;                           // the shortest stroke there is
  for (const Trigger::Config& c : {Trigger::defaults(), fast}) {
    Rig r(c);
    const uint32_t stroke = sweepFor(spanOf(TRIGGER_FIRE_ANGLE, TRIGGER_REST_ANGLE), c.sweepDegPerS) * 2 + kFwd + kRet;
    r.step(in(true, false));
    r.run(1, in(true, true));
    const uint32_t t0 = r.now;
    const uint32_t holdEnd = std::max<uint32_t>(stroke, LED_FIRE_FLASH_MIN_MS);
    bool alwaysOn = true;
    r.run(1, in(true, false));                          // released at once: the stroke still runs
    while (r.now < t0 + holdEnd - 1) {
      if (!r.t.firingLedActive(r.now)) alwaysOn = false;
      r.run(1, in(true, false));
    }
    CHECK(r.t.firingLedActive(r.now));                  // t0 + holdEnd - 1
    CHECK(alwaysOn);
    CHECK(r.t.firingLedActive(t0 + LED_FIRE_FLASH_MIN_MS - 1));
    r.run(1, in(true, false));                          // t0 + holdEnd
    CHECK(!r.t.firingLedActive(r.now));
    r.run(stroke, in(true, false));
    CHECK(!r.t.firingLedActive(r.now));
  }
}

// ---- fireReady: the ARMED LED colour (owner, 2026-11-03) --------------------------------------------
// It is exactly "gate() of a fresh pull is OPEN and the shot's LED hold is over": every gate
// counts (not armed, stroke running, servo off, the flywheel spin-up, EMPTY only with lockout, the
// semi cooldown), whatever the trigger does now, and asking never fires.
TEST(fire_ready_follows_every_gate_and_the_led_hold) {
  Rig r(strict());
  CHECK(!r.t.fireReady(r.now, in(false, false)));                 // NOT_ARMED
  CHECK(!r.t.fireReady(r.now, in(true, false, false)));           // FLYWHEEL_NOT_READY
  CHECK(r.t.fireReady(r.now, in(true, false, true)));
  CHECK(r.t.fireReady(r.now, in(true, true, true)));              // held or not: a pull would fire
  CHECK(!r.t.strokeActive());                                     // asking fired nothing
  CHECK_EQ(r.t.shotsTotal(), 0);
  r.t.setServoPower(false);
  CHECK(!r.t.fireReady(r.now, in(true, false)));                  // SERVO_OFF
  r.t.setServoPower(true);
  // A semi stroke: not ready from its start until the stroke, the cooldown after it and the LED
  // hold are all over.
  CHECK_EQ(r.t.fireMode(), (uint8_t)DS_FIRE_MODE_SEMI);
  r.step(in(true, false));
  r.run(1, in(true, true));
  const uint32_t t0 = r.now;
  CHECK(r.t.strokeActive());
  const uint32_t notReadyFor = std::max<uint32_t>(kStroke + kCool, LED_FIRE_FLASH_MIN_MS);
  bool readyEarly = false, mismatch = false, sawCooldown = false;
  while (r.now < t0 + notReadyFor - 1) {
    r.run(1, in(true, false));
    const Trigger::Input i = in(true, false);
    readyEarly = readyEarly || r.t.fireReady(r.now, i);
    sawCooldown = sawCooldown || r.t.gate(in(true, true)) == Trigger::Gate::COOLDOWN;
    // the definition, every millisecond
    const bool want = r.t.gate(in(true, true)) == Trigger::Gate::OPEN && !r.t.firingLedActive(r.now);
    mismatch = mismatch || r.t.fireReady(r.now, i) != want;
  }
  CHECK(!readyEarly);
  CHECK(!mismatch);
  CHECK(sawCooldown == (kCool > 0));                              // the cooldown kept it yellow
  r.run(1, in(true, false));                                      // t0 + max(stroke + cooldown, hold)
  CHECK(r.t.fireReady(r.now, in(true, false)));
  // Semi with the trigger still held after its shot: once the cooldown is over a fresh pull
  // would fire, so ready.
  r.run(1, in(true, true));
  r.run(kStroke + kCool + LED_FIRE_FLASH_MIN_MS, in(true, true));
  CHECK_EQ(r.t.shotsTotal(), 2);
  CHECK(!r.t.strokeActive());
  CHECK(r.t.fireReady(r.now, in(true, true)));
  // An abort ends the hold at once (and starts no cooldown).
  r.run(1, in(true, false));
  r.run(1, in(true, true));
  CHECK(!r.t.fireReady(r.now, in(true, true)));
  r.t.abort();
  CHECK(r.t.fireReady(r.now, in(true, true)));
}

// Full auto with the trigger held: after every update the next stroke is already running, so
// fireReady never comes true between strokes (no red flicker); released, it does.
TEST(fire_ready_stays_false_while_full_auto_cycles) {
  Rig r(strict());
  r.t.setFireMode(DS_FIRE_MODE_FULL);
  r.step(in(true, false));
  CHECK(r.t.fireReady(r.now, in(true, false)));
  r.run(1, in(true, true));
  const uint16_t shots0 = r.t.shotsTotal();
  bool ready = false;
  for (uint32_t k = 0; k < 5 * kStroke; k++) {
    r.run(1, in(true, true));
    ready = ready || r.t.fireReady(r.now, in(true, true));
  }
  CHECK(!ready);
  CHECK_EQ(r.t.shotsTotal(), (uint16_t)(shots0 + 5));
  r.run(1, in(true, false));
  r.run(kStroke + LED_FIRE_FLASH_MIN_MS, in(true, false));
  CHECK(r.t.fireReady(r.now, in(true, false)));
}

TEST(shot_counter_counts_completed_strokes_only) {
  Rig r;
  r.step(in(true, false));
  r.fireSemi(2);
  CHECK_EQ(r.t.shotsTotal(), 2);
  r.run(1, in(true, true));
  r.run(kSweep + kFwd + 1, in(true, true));
  r.t.abort();                                        // aborted: not counted
  CHECK_EQ(r.t.shotsTotal(), 2);
  CHECK_EQ(r.t.ammo(), MAG_CAPACITY - 2);
  r.t.resetMagazine();
  CHECK_EQ(r.t.ammo(), MAG_CAPACITY);
  CHECK_EQ(r.t.shotsTotal(), 2);                      // the counter is not the magazine
}

// The shell's `fire`: every gate, the semi cooldown included (refused until the very ms it ends).
TEST(fire_once_obeys_every_gate_and_the_cooldown) {
  Rig r(strict());
  CHECK(!r.t.fireOnce(r.now, in(false, false)));
  CHECK_EQ(r.t.gate(in(false, false)), Trigger::Gate::NOT_ARMED);
  CHECK(!r.t.fireOnce(r.now, in(true, false, false)));
  CHECK(r.t.fireOnce(r.now, in(true, false)));        // no trigger needed
  CHECK(r.t.strokeActive());
  Trigger::Rumble out;
  CHECK(r.t.takeRumbleRequest(out));
  CHECK_EQ(out.ms, RUMBLE_DURATION_MS);
  CHECK(!r.t.fireOnce(r.now, in(true, false)));       // busy
  CHECK_EQ(r.t.gate(in(true, false)), Trigger::Gate::STROKE_RUNNING);
  const uint32_t t0 = r.now;
  r.run(kStroke, in(true, false));
  CHECK_EQ(r.t.ammo(), MAG_CAPACITY - 1);
  CHECK(!r.t.strokeActive());                         // exactly one stroke
  const uint32_t done = t0 + kStroke;
  CHECK(!r.t.fireOnce(done + 1, in(true, false)));
  CHECK_EQ(r.t.gate(in(true, false)), Trigger::Gate::COOLDOWN);
  CHECK(!r.t.fireOnce(done + kCool - 1, in(true, false)));   // no update in between: fireOnce reads the clock
  CHECK_EQ(r.t.gate(in(true, false)), Trigger::Gate::COOLDOWN);
  CHECK(r.t.fireOnce(done + kCool, in(true, false)));
  CHECK(r.t.strokeActive());
  // Full auto: back-to-back, so no cooldown for `fire` either.
  Rig f(strict());
  f.t.setFireMode(DS_FIRE_MODE_FULL);
  CHECK(f.t.fireOnce(f.now, in(true, false)));
  f.run(kStroke, in(true, false));
  CHECK(f.t.fireOnce(f.now, in(true, false)));
}

TEST(fire_mode_change_needs_a_fresh_pull) {
  Rig r;
  r.t.setFireMode(DS_FIRE_MODE_FULL);
  r.step(in(true, false));
  r.run(1, in(true, true));
  r.run(kStroke / 2, in(true, true));
  r.t.toggleFireMode();                               // B while cycling: -> semi
  CHECK_EQ(r.t.fireMode(), DS_FIRE_MODE_SEMI);
  r.run(5 * kStroke, in(true, true));
  CHECK_EQ(r.t.ammo(), MAG_CAPACITY - 1);             // the running stroke finished, no more
  r.t.toggleFireMode();                               // -> full while still held
  r.run(5 * kStroke, in(true, true));
  CHECK_EQ(r.t.ammo(), MAG_CAPACITY - 1);
  r.run(1, in(true, false));
  r.run(1, in(true, true));
  CHECK(r.t.strokeActive());
  r.t.setFireMode(DS_FIRE_MODE_COUNT);                // not a mode: ignored
  CHECK_EQ(r.t.fireMode(), DS_FIRE_MODE_FULL);
}

// ---- stroke angles and SERVO ADJUST support (Trigger.h) ---------------------------------------------
namespace {
Trigger::Input teachIn(bool on = true, bool armed = false) {
  Trigger::Input i;
  i.servoTeach = on;
  i.armed = armed;
  return i;
}
// Recorded angles that differ from config.h's (3 and 2 degrees away, inside the servo's span).
float teachRest() {
  return TRIGGER_REST_ANGLE + 3 <= SERVO_SPAN_DEG ? TRIGGER_REST_ANGLE + 3.0f : TRIGGER_REST_ANGLE - 3.0f;
}
float teachFire() { return TRIGGER_FIRE_ANGLE >= 2 ? TRIGGER_FIRE_ANGLE - 2.0f : TRIGGER_FIRE_ANGLE + 2.0f; }
}  // namespace

// setAngles() changes what every stroke, its sweep back and abort() command (it moves nothing by
// itself: the first stroke sweeps from where the servo is); resetAngles() brings config.h's back.
TEST(set_angles_drive_every_stroke_and_abort_until_reset) {
  Rig r;
  r.t.setAngles(teachRest(), teachFire());
  CHECK(r.servoAtRest());                                  // nothing moved yet
  r.step(in(true, false));
  const float from = r.t.servoDeg();
  Trace tr = traceStroke(r, in(true, true), 4 * MS_PER_S, [&] { r.step(in(true, true)); });
  CHECK(strokeShapeIs(tr, from, teachFire(), teachRest()));
  CHECK_EQ(r.t.shotsTotal(), (uint16_t)1);
  CHECK_EQ(servoLogical(), teachRest());
  r.run(1, in(true, false));
  r.run(kCool, in(true, false));
  tr = traceStroke(r, in(true, true), 4 * MS_PER_S, [&] { r.step(in(true, true)); });
  CHECK(strokeShapeIs(tr, teachRest(), teachFire(), teachRest()));   // from the recorded REST now
  r.run(1, in(true, false));
  r.run(kCool, in(true, false));
  r.step(in(true, true));
  r.run(sweepFor(spanOf(teachFire(), teachRest())), in(true, true));
  CHECK_EQ(servoLogical(), teachFire());
  r.t.abort();
  CHECK_EQ(servoLogical(), teachRest());
  r.t.resetAngles();
  CHECK_EQ(r.t.restDeg(), (float)TRIGGER_REST_ANGLE);
  CHECK_EQ(r.t.fireDeg(), (float)TRIGGER_FIRE_ANGLE);
  r.t.abort();
  CHECK(r.servoAtRest());
}

TEST(teach_move_only_in_teach_mode_never_armed_never_mid_stroke) {
  Rig r;
  const float a = teachRest();
  CHECK(!r.t.teachMove(a, in(false, false)));               // not in adjust mode
  CHECK(!r.t.teachMove(a, teachIn(true, true)));            // armed
  CHECK(r.servoAtRest());
  CHECK(!r.t.offRest());
  CHECK(r.t.teachMove(a, teachIn()));
  CHECK_EQ(servoLogical(), a);
  CHECK_EQ(r.t.servoDeg(), a);
  CHECK(r.t.offRest());
  r.t.setAngles(teachRest(), teachFire());
  CHECK(r.t.testStroke(r.now, teachIn()));
  CHECK(!r.t.teachMove(teachFire(), teachIn()));            // a stroke runs
  CHECK_EQ(servoLogical(), a);                              // the sweep starts where the servo is
  r.run(sweepFor(spanOf(teachFire(), a)), teachIn());
  CHECK_EQ(servoLogical(), teachFire());
}

// The test stroke: from the teach angle the servo is at, the same sweeps and holds as a real
// stroke, firing LED, no rumble, no ammo change, no shot counted, no cooldown; a real stroke
// cannot start over it.
TEST(test_stroke_sweeps_like_a_stroke_costs_nothing_and_needs_teach_mode) {
  Rig r;
  r.t.setAngles(teachRest(), teachFire());
  CHECK(!r.t.testStroke(r.now, in(false, false)));
  CHECK(!r.t.testStroke(r.now, teachIn(true, true)));
  r.t.setServoPower(false);
  CHECK(!r.t.testStroke(r.now, teachIn()));
  r.t.setServoPower(true);
  const float from = (teachRest() + teachFire()) / 2.0f + 0.5f;   // somewhere between, off-grid
  CHECK(r.t.teachMove(from, teachIn()));
  bool ledAlways = true, gateClosed = true;
  const Trace tr = traceStroke(r, teachIn(), 4 * MS_PER_S, [&] { CHECK(r.t.testStroke(r.now, teachIn())); });
  CHECK(strokeShapeIs(tr, from, teachFire(), teachRest()));
  CHECK(!r.t.offRest());                                     // the stroke ended at rest by itself
  // Run it again, watching the LED and the gate.
  CHECK(r.t.testStroke(r.now, teachIn()));
  CHECK(r.t.testStrokeActive());
  CHECK(r.t.gate(in(true, true)) == Trigger::Gate::STROKE_RUNNING);
  while (r.t.strokeActive()) {
    ledAlways = ledAlways && r.t.firingLedActive(r.now);
    gateClosed = gateClosed && r.t.gate(in(true, true)) == Trigger::Gate::STROKE_RUNNING;
    r.run(1, teachIn());
  }
  CHECK(ledAlways);
  CHECK(gateClosed);
  CHECK_EQ(r.t.ammo(), (uint8_t)MAG_CAPACITY);
  CHECK_EQ(r.t.shotsTotal(), (uint16_t)0);
  CHECK(!r.t.coolingDown());
  Trigger::Rumble rumble;
  CHECK(!r.t.takeRumbleRequest(rumble));
  CHECK_EQ(servoLogical(), teachRest());
}

// KEY SAFETY TEST: a test stroke never outlives adjust mode, in whatever phase adjust mode drops
// (or armed rises), and a teach angle never outlives it either, even without an abort() call; an
// aborted test stroke requests no rumble stop and goes to REST at once (no sweep).
TEST(teach_mode_dropping_rests_the_servo_without_an_abort_call) {
  const uint32_t sweep = sweepFor(spanOf(teachFire(), teachRest()));
  for (const Phase p : kStrokePhases) {
    if (phaseLength(p, sweep) == 0) continue;
    for (int k = 0; k < 2; k++) {
      Rig r;
      r.t.setAngles(teachRest(), teachFire());
      r.t.abort();
      CHECK(r.t.testStroke(r.now, teachIn()));
      r.run(phaseStart(p, sweep) + phaseLength(p, sweep) / 2, teachIn());
      CHECK_EQ(r.t.phase(), p);
      r.step(k == 0 ? in(false, false) : teachIn(true, true));   // teach dropped / armed rose
      CHECK(!r.t.strokeActive());
      CHECK_EQ(servoLogical(), teachRest());
      CHECK_EQ(r.t.shotsTotal(), (uint16_t)0);
      CHECK_EQ(r.t.ammo(), (uint8_t)MAG_CAPACITY);
      CHECK(!r.t.coolingDown());
      Trigger::Rumble rumble;
      CHECK(!r.t.takeRumbleRequest(rumble));
    }
  }
  Rig r;
  CHECK(r.t.teachMove(teachFire(), teachIn()));
  r.run(10, teachIn());
  CHECK_EQ(servoLogical(), teachFire());            // held while in adjust mode
  r.step(in(false, false));
  CHECK(r.servoAtRest());
  CHECK(!r.t.offRest());
  CHECK(r.t.teachMove(teachFire(), teachIn()));
  r.t.setServoPower(false);                                 // parked; the next update rests it
  r.step(in(false, false));
  CHECK(r.servoAtRest());
}

// TRIGGER_SERVO_REVERSED (owner, 2026-11-03): the servo receives SERVO_SPAN_DEG - angle, so the
// racked REST (180) is the servo's own 0 end and every move toward FIRE turns it the other way.
TEST(reversed_servo_receives_the_mirrored_angle) {
  Rig r;   // begin() and servo power on
  const float sent = hal_host::servoAngle();
  if (TRIGGER_SERVO_REVERSED) {
    CHECK_NEAR(sent, (float)SERVO_SPAN_DEG - (float)TRIGGER_REST_ANGLE, 0.001f);
  } else {
    CHECK_NEAR(sent, (float)TRIGGER_REST_ANGLE, 0.001f);
  }
  CHECK_NEAR(Trigger::physicalDeg(Trigger::physicalDeg(37.0f)), 37.0f, 0.001f);   // its own inverse
}

TESTKIT_MAIN()

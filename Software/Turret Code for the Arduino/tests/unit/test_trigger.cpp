// test_trigger.cpp -- Trigger (spec 3.4, architecture.md 5 and 6): the shot sequence (PRE-FIRE ->
// FIRE held -> REST held -> pause -> the shot counts -> PRE-FIRE held, or stay at REST at ammo 0),
// with and without sweeps, semi and full auto, the semi cooldown, abort in every phase (home
// now), the magazine (boot ammo, reload to PRE-FIRE, chambering, floor 0, lockout), the flywheel
// gate, "a pull counts only if it started while ARMED", rumble requests, the firing-LED hold and
// the shot counter.
//
// Time is the module's own nowMs argument; the servo is observed through the host HAL
// (hal_host::servoAngle / servoOn), i.e. what the PWM would really be told. Every expected time
// is computed here from config.h (sweepFor and segments below state the rules), never read from
// the module under test.
//
// Safety property: a stroke never keeps running once armed goes false, and an aborted stroke
// never costs a dart, in whatever phase it is aborted
// (abort_in_every_phase_goes_home_now_never_decrements_and_starts_no_cooldown,
// disarm_mid_stroke_goes_home_at_once_without_an_abort_call).
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
constexpr uint32_t kClearHold = TRIGGER_CLEAR_HOLD_MS;
constexpr uint32_t kChamber = TRIGGER_CHAMBER_MS;
constexpr uint32_t kCool = TRIGGER_COOLDOWN_MS;
constexpr float kRate = TRIGGER_SWEEP_DEG_PER_S;
constexpr float kTestRate = 300.0f;   // a glide for the sweep tests, whatever config.h ships
constexpr double kDegTol = 1e-3;      // float rounding of a commanded angle mid sweep
constexpr float kPre = TRIGGER_PREFIRE_ANGLE;
constexpr float kFire = TRIGGER_FIRE_ANGLE;
constexpr float kRest = TRIGGER_REST_ANGLE;
constexpr float kClear = TRIGGER_CLEAR_ANGLE;

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

// One shot's sequence (Trigger.h) as segments: each phase, whether it sweeps, its end angle and
// its length. From `from`: [lead-in: sweep to PRE-FIRE, hold TRIGGER_CHAMBER_MS] sweep to FIRE,
// hold TRIGGER_FORWARD_MS, sweep to REST, hold TRIGGER_RETURN_MS, sweep to CLEAR, hold
// TRIGGER_CLEAR_HOLD_MS, sweep to REST, hold TRIGGER_RETURN_MS (the shot counts at its end), [chamber: sweep to PRE-FIRE, hold TRIGGER_CHAMBER_MS], idle.
struct Shot {
  float from = kPre, fire = kFire, rest = kRest, prefire = kPre, clear = kClear;
  bool clearStop = TRIGGER_CLEAR_ENABLED;   // the CLEAR visit and second REST hold
  bool chamber = true;    // ammo left after it (or a test stroke)
  bool leadIn = false;    // a test stroke from off PRE-FIRE
  float rate = kRate;
};
struct Seg {
  Phase phase;
  bool sweep;
  float from, to;
  uint32_t len;
};
std::vector<Seg> segments(const Shot& s) {
  std::vector<Seg> v;
  float at = s.from;
  auto sweep = [&](Phase p, float to) {
    v.push_back({p, true, at, to, sweepFor(spanOf(to, at), s.rate)});
    at = to;
  };
  auto hold = [&](Phase p, uint32_t ms) { v.push_back({p, false, at, at, ms}); };
  if (s.leadIn) {
    sweep(Phase::SWEEP_CHAMBER, s.prefire);
    hold(Phase::HOLD_CHAMBER, kChamber);
  }
  sweep(Phase::SWEEP_OUT, s.fire);
  hold(Phase::HOLD_FIRE, kFwd);
  sweep(Phase::SWEEP_BACK, s.rest);
  hold(Phase::HOLD_REST, kRet);
  if (s.clearStop) {
    sweep(Phase::SWEEP_CLEAR, s.clear);
    hold(Phase::HOLD_CLEAR, kClearHold);
    sweep(Phase::SWEEP_RACK, s.rest);
    hold(Phase::HOLD_RACK, kRet);
  }
  if (s.chamber) {
    sweep(Phase::SWEEP_CHAMBER, s.prefire);
    hold(Phase::HOLD_CHAMBER, kChamber);
  }
  return v;
}
uint32_t totalMs(const Shot& s) {
  uint32_t ms = 0;
  for (const Seg& g : segments(s)) ms += g.len;
  return ms;
}
float endDeg(const Shot& s) { return segments(s).back().to; }
// When the shot counts: the end of the last REST hold (HOLD_RACK, or HOLD_REST without the
// CLEAR visit), from the start.
uint32_t countMs(const Shot& s) {
  uint32_t ms = 0;
  for (const Seg& g : segments(s)) {
    ms += g.len;
    if (g.phase == (s.clearStop ? Phase::HOLD_RACK : Phase::HOLD_REST)) break;
  }
  return ms;
}
// Where the (first) phase p starts and how long it lasts; a shot without a lead-in.
uint32_t phaseStart(Phase p, const Shot& s = Shot()) {
  uint32_t ms = 0;
  for (const Seg& g : segments(s)) {
    if (g.phase == p) return ms;
    ms += g.len;
  }
  return ms;
}
uint32_t phaseLength(Phase p, const Shot& s = Shot()) {
  for (const Seg& g : segments(s))
    if (g.phase == p) return g.len;
  return 0;
}

// config.h's angles, from PRE-FIRE with darts left after it; the last dart (no chambering).
const uint32_t kOut = sweepFor(spanOf(kFire, kPre));
const uint32_t kCount = countMs(Shot());
const uint32_t kShot = totalMs(Shot());
Shot lastShot() {
  Shot s;
  s.chamber = false;
  return s;
}
const uint32_t kLast = totalMs(lastShot());
const Phase kFirst = kOut ? Phase::SWEEP_OUT : Phase::HOLD_FIRE;   // a shot's first phase
const Phase kShotPhases[] = {Phase::SWEEP_OUT, Phase::HOLD_FIRE,     Phase::SWEEP_BACK,  Phase::HOLD_REST,
                             Phase::SWEEP_CLEAR, Phase::HOLD_CLEAR,   Phase::SWEEP_RACK,  Phase::HOLD_RACK,
                             Phase::SWEEP_CHAMBER, Phase::HOLD_CHAMBER};
const char* phaseName(Phase p) {
  switch (p) {
    case Phase::IDLE: return "IDLE";
    case Phase::SWEEP_OUT: return "SWEEP_OUT";
    case Phase::HOLD_FIRE: return "HOLD_FIRE";
    case Phase::SWEEP_BACK: return "SWEEP_BACK";
    case Phase::HOLD_REST: return "HOLD_REST";
    case Phase::SWEEP_CLEAR: return "SWEEP_CLEAR";
    case Phase::HOLD_CLEAR: return "HOLD_CLEAR";
    case Phase::SWEEP_RACK: return "SWEEP_RACK";
    case Phase::HOLD_RACK: return "HOLD_RACK";
    case Phase::SWEEP_CHAMBER: return "SWEEP_CHAMBER";
    case Phase::HOLD_CHAMBER: return "HOLD_CHAMBER";
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

// A booted host, a Trigger begun with its servo powered (as TurretState does on leaving OFF) and,
// unless `loaded` is false, X pressed (MAG_CAPACITY, servo at PRE-FIRE) and its chambering over.
struct Rig {
  Trigger t;
  uint32_t now = 1000;
  uint32_t shot;   // one shot's whole sequence at this Config's rate and config.h's angles
  explicit Rig(const Trigger::Config& c = Trigger::defaults(), bool loaded = true) : t(c) {
    Shot s;
    s.rate = c.sweepDegPerS;
    s.clearStop = c.clearStop;
    shot = totalMs(s);
    hal_host::reset();
    hal::begin();
    t.begin();
    t.setServoPower(true);
    if (loaded) load();
  }
  // X, then the chambering waited out (it starts at the next update and lasts TRIGGER_CHAMBER_MS).
  void load() {
    t.resetMagazine();
    settle();
  }
  void settle() { run(kChamber + 1, in(false, false)); }
  // One update at the current time.
  void step(const Trigger::Input& i) { t.update(now, i); }
  // Updates every millisecond for `ms` ms (the first one at now + 1).
  void run(uint32_t ms, const Trigger::Input& i) {
    for (uint32_t k = 0; k < ms; k++) {
      now++;
      t.update(now, i);
    }
  }
  // Fires `n` semi shots, each pulled, held to the sequence's end, released and the cooldown waited out.
  void fireSemi(int n) {
    for (int k = 0; k < n; k++) {
      step(in(true, true));
      run(shot, in(true, true));
      run(1, in(true, false));
      run(kCool, in(true, false));
    }
  }
  bool servoAtFire() const { return servoLogical() == kFire; }
  bool servoAtRest() const { return servoLogical() == kRest; }
  bool servoAtPrefire() const { return servoLogical() == kPre; }
  bool servoAtHome() const { return servoLogical() == (t.ammo() > 0 ? kPre : kRest); }
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
// config.h's switches at another rate: 0 (jump), config.h's, or a test glide.
Trigger::Config rated(float rate, bool clear = TRIGGER_CLEAR_ENABLED) {
  Trigger::Config c = Trigger::defaults();
  c.sweepDegPerS = rate;
  c.clearStop = clear;
  return c;
}
Shot shotAt(float rate, bool clear = TRIGGER_CLEAR_ENABLED) {
  Shot s;
  s.rate = rate;
  s.clearStop = clear;
  return s;
}
Trigger::Config gliding() { return rated(kTestRate); }
Shot glidingShot() { return shotAt(kTestRate); }
// Every rate with and without the CLEAR visit (TRIGGER_CLEAR_ENABLED either way).
struct Variant {
  float rate;
  bool clear;
};
const Variant kVariants[] = {{0.0f, false}, {kRate, false}, {kTestRate, false},
                             {0.0f, true},  {kRate, true},  {kTestRate, true}};

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

// The sequence shape (segments): every sweep step is the rate's distance from the sweep's start,
// monotonic, never past its end; every hold exactly its angle for its time; then idle at the end.
bool strokeShapeIs(const Trace& tr, const Shot& s) {
  const std::vector<Seg> segs = segments(s);
  const size_t total = totalMs(s);
  bool ok = tr.deg.size() == total + 1 && tr.halAgrees;
  if (!CHECK(ok)) {
    std::printf("      stroke lasted %zu ms, expected %zu; HAL agreed: %d\n", tr.deg.size() - 1, total, tr.halAgrees);
    return false;
  }
  size_t k = 0;
  for (const Seg& g : segs) {
    for (uint32_t j = 0; j < g.len && ok; j++, k++) {
      const float d = tr.deg[k];
      if (g.sweep) {
        ok = tr.phase[k] == g.phase && std::fabs(d - sweptTo(g.from, g.to, j, s.rate)) <= kDegTol &&
             spanOf(d, g.to) > 0.0f && spanOf(d, g.from) < spanOf(g.to, g.from);
        if (j > 0) ok = ok && spanOf(d, g.to) < spanOf(tr.deg[k - 1], g.to);   // strictly toward its end
      } else {
        ok = tr.phase[k] == g.phase && d == g.to;   // landed exactly, held
      }
      if (!ok) std::printf("      stroke shape broken at %zu ms: %s at %.4f\n", k, phaseName(tr.phase[k]), (double)d);
    }
    if (!ok) break;
  }
  if (ok) {
    ok = tr.phase[total] == Phase::IDLE && tr.deg[total] == endDeg(s);
    if (!ok) std::printf("      not idle at %.4f after %zu ms\n", (double)tr.deg[total], total);
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
  CHECK_EQ(c.clearStop, TRIGGER_CLEAR_ENABLED);
  // The sweep rule agrees with the plain arithmetic for config.h's whole-degree angles.
  if (kRate > 0.0f)
    CHECK_EQ(kOut, (uint32_t)std::ceil(std::fabs((double)kFire - (double)kPre) * (double)MS_PER_S / (double)kRate));
  CHECK_EQ(kShot, kLast + sweepFor(spanOf(kRest, kPre)) + kChamber);
  const uint32_t atRest = kOut + kFwd + sweepFor(spanOf(kRest, kFire));
  std::printf("info shot at config.h angles, CLEAR %s (ms from the pull): at FIRE %u, leaves %u, at REST %u, "
              "counts %u, at PRE-FIRE %u, next shot %u (last dart ends at REST at %u), semi cycle %u\n",
              TRIGGER_CLEAR_ENABLED ? "on" : "off", (unsigned)kOut, (unsigned)(kOut + kFwd), (unsigned)atRest,
              (unsigned)kCount, (unsigned)(kShot - kChamber), (unsigned)kShot, (unsigned)kLast,
              (unsigned)(kShot + kCool));
  Shot burst;
  burst.from = kRest;
  burst.chamber = false;
  std::printf("info full auto: first shot counts %u, then one shot every %u (REST -> FIRE -> REST), after "
              "the release + %u to PRE-FIRE\n",
              (unsigned)kLast, (unsigned)totalMs(burst), (unsigned)(kShot - kLast));
  Shot other;
  other.clearStop = !TRIGGER_CLEAR_ENABLED;
  std::printf("info with CLEAR %s: counts %u, next shot %u\n", other.clearStop ? "on" : "off",
              (unsigned)countMs(other), (unsigned)totalMs(other));
}

TEST(begin_parks_the_servo_at_home_with_the_boot_ammo) {
  hal_host::reset();
  hal::begin();
  Trigger t;
  t.begin();
  CHECK(!hal_host::servoOn());
  CHECK_EQ(t.ammo(), (uint8_t)TRIGGER_AMMO_AT_BOOT);
  CHECK_EQ(servoLogical(), TRIGGER_AMMO_AT_BOOT > 0 ? kPre : kRest);   // config.h: 0, REST
  CHECK_EQ(t.homeDeg(), TRIGGER_AMMO_AT_BOOT > 0 ? kPre : kRest);
  CHECK_EQ(t.phase(), Phase::IDLE);
  CHECK_EQ(t.capacity(), MAG_CAPACITY);
  CHECK_EQ(t.shotsTotal(), 0);
  CHECK_EQ(t.fireMode(), DS_FIRE_MODE_SEMI);
  CHECK(!t.coolingDown());
  CHECK(!t.chambering());
  CHECK_EQ(t.prefireDeg(), kPre);
  // Unpowered servo: armed and ready still cannot fire.
  CHECK_EQ(t.gate(in(true, true)), Trigger::Gate::SERVO_OFF);
  t.update(1000, in(true, false));
  t.update(1001, in(true, true));
  CHECK(!t.strokeActive());
  t.setServoPower(true);
  CHECK(hal_host::servoOn());
  CHECK_EQ(servoLogical(), TRIGGER_AMMO_AT_BOOT > 0 ? kPre : kRest);
  const bool empty = TRIGGER_AMMO_AT_BOOT == 0 && FIRE_LOCKOUT_WHEN_EMPTY;
  CHECK_EQ(t.gate(in(true, true)), empty ? Trigger::Gate::EMPTY : Trigger::Gate::OPEN);
}

// X: ammo MAG_CAPACITY and the servo to PRE-FIRE at once; no shot until it has had
// TRIGGER_CHAMBER_MS to arrive (from the next update), a held pull then fires.
TEST(reload_moves_to_prefire_and_chambers_before_a_shot) {
  Rig r(strict(), false);
  r.step(in(true, false));
  CHECK(r.servoAtRest());
  CHECK_EQ(r.t.gate(in(true, true)), Trigger::Gate::EMPTY);
  r.t.resetMagazine();
  CHECK_EQ(r.t.ammo(), (uint8_t)MAG_CAPACITY);
  CHECK(r.servoAtPrefire());                       // in that very call
  CHECK_EQ(r.t.gate(in(true, true)), Trigger::Gate::CHAMBERING);
  r.run(1, in(true, true));                        // pulled: the chambering starts at this update
  const uint32_t t0 = r.now;
  bool early = false;
  while (r.now < t0 + kChamber - 1) {
    r.run(1, in(true, true));
    early = early || r.t.strokeActive();
  }
  CHECK(!early);
  CHECK_EQ(r.t.gate(in(true, true)), Trigger::Gate::CHAMBERING);
  r.run(1, in(true, true));                        // t0 + TRIGGER_CHAMBER_MS: fires
  CHECK(r.t.strokeActive());
  CHECK(!r.t.chambering());
  // X again at PRE-FIRE (nothing moves): no chambering. X mid-stroke: the stroke carries on.
  r.run(kShot, in(true, false));
  r.run(kCool, in(true, false));
  r.t.resetMagazine();
  CHECK(!r.t.chambering());
  r.run(1, in(true, true));
  CHECK(r.t.strokeActive());
  r.run(kOut + kFwd / 2, in(true, true));
  CHECK(r.servoAtFire());
  r.t.resetMagazine();
  CHECK(r.servoAtFire());                          // not pulled back mid-stroke
  CHECK_EQ(r.t.phase(), Phase::HOLD_FIRE);
  r.run(kShot, in(true, true));
  CHECK_EQ(r.t.ammo(), (uint8_t)(MAG_CAPACITY - 1));   // counted against the new magazine
  CHECK(r.servoAtPrefire());
}

// Power-toggle reboot: TRIGGER_AMMO_AT_BOOT and home (REST at 0), a running stroke aborted.
TEST(boot_magazine_empties_and_goes_home) {
  Rig r;
  r.step(in(true, false));
  r.run(1, in(true, true));
  r.run(kOut + kFwd / 2, in(true, true));
  CHECK(r.servoAtFire());
  r.t.bootMagazine();
  CHECK(!r.t.strokeActive());
  CHECK_EQ(r.t.ammo(), (uint8_t)TRIGGER_AMMO_AT_BOOT);
  CHECK(r.servoAtHome());
  CHECK_EQ(r.t.shotsTotal(), 0);
}

// One shot at config.h's angles, ms by ms (no glide: each angle at once): FIRE held
// TRIGGER_FORWARD_MS, REST held TRIGGER_RETURN_MS, [with the CLEAR visit: CLEAR held
// TRIGGER_CLEAR_HOLD_MS, REST held TRIGGER_RETURN_MS again,] the shot counts, PRE-FIRE held
// TRIGGER_CHAMBER_MS, then the next shot is available.
TEST(one_shot_exact_timeline_without_a_glide) {
  for (const bool clear : {false, true}) {
    Rig r(rated(0.0f, clear));
    const uint32_t count = kFwd + kRet + (clear ? kClearHold + kRet : 0u), shot = count + kChamber;
    r.step(in(true, false));
    CHECK(r.servoAtPrefire());
    r.run(1, in(true, true));                        // t0
    const uint32_t t0 = r.now;
    CHECK_EQ(r.t.phase(), Phase::HOLD_FIRE);         // FIRE in the update that starts the shot
    CHECK(r.servoAtFire());
    r.run(kFwd - 1, in(true, true));
    CHECK(r.servoAtFire());
    r.run(1, in(true, true));                        // t0 + 180
    CHECK_EQ(r.now, t0 + kFwd);
    CHECK_EQ(r.t.phase(), Phase::HOLD_REST);
    CHECK(r.servoAtRest());
    if (clear) {
      r.run(kRet, in(true, true));                   // t0 + 360: CLEAR
      CHECK_EQ(r.t.phase(), Phase::HOLD_CLEAR);
      CHECK_EQ(servoLogical(), kClear);
      CHECK_EQ(r.t.ammo(), (uint8_t)MAG_CAPACITY);
      r.run(kClearHold - 1, in(true, true));
      CHECK_EQ(servoLogical(), kClear);
      r.run(1, in(true, true));                      // t0 + 460: REST again
      CHECK_EQ(r.t.phase(), Phase::HOLD_RACK);
      CHECK(r.servoAtRest());
      r.run(kRet - 1, in(true, true));
    } else {
      r.run(kRet - 1, in(true, true));
      CHECK_EQ(r.t.phase(), Phase::HOLD_REST);
    }
    CHECK_EQ(r.t.ammo(), (uint8_t)MAG_CAPACITY);     // not one ms early
    r.run(1, in(true, true));                        // counted, PRE-FIRE commanded
    CHECK_EQ(r.now, t0 + count);
    CHECK_EQ(r.t.ammo(), (uint8_t)(MAG_CAPACITY - 1));
    CHECK_EQ(r.t.shotsTotal(), 1);
    CHECK_EQ(r.t.phase(), Phase::HOLD_CHAMBER);
    CHECK(r.servoAtPrefire());
    r.run(kChamber - 1, in(true, true));
    CHECK(r.t.strokeActive());
    CHECK_EQ(r.t.gate(in(true, true)), Trigger::Gate::STROKE_RUNNING);
    r.run(1, in(true, true));                        // the next shot is available
    CHECK_EQ(r.now, t0 + shot);
    CHECK_EQ(r.t.phase(), Phase::IDLE);
    CHECK(r.t.coolingDown());                        // semi: the cooldown starts here
    CHECK_EQ(r.t.strokeMs(), shot);
    CHECK_EQ(r.t.sweepMs(kRest, kFire), 0u);
  }
}

// The whole sequence through the trace (both rates), and the magazine changing exactly once, at
// the end of the second REST hold.
TEST(one_shot_shape_and_ammo_only_where_it_counts) {
  for (const Variant v : kVariants) {
    const float rate = v.rate;
    const bool clear = v.clear;
    Rig r(rated(rate, clear));
    const Shot s = shotAt(rate, clear);
    r.step(in(true, false));
    r.now = 5000;
    const uint32_t t0 = r.now;
    const Trace tr = traceStroke(r, in(true, true), 4 * MS_PER_S, [&] { r.step(in(true, true)); });
    CHECK(strokeShapeIs(tr, s));
    CHECK_EQ(r.now, t0 + totalMs(s));
    bool once = tr.ammo.size() == totalMs(s) + 1;
    for (size_t k = 0; k < tr.ammo.size(); k++)
      once = once && tr.ammo[k] == (k < countMs(s) ? MAG_CAPACITY : MAG_CAPACITY - 1);
    CHECK(once);
    CHECK_EQ(r.t.shotsTotal(), 1);
    CHECK(r.servoAtPrefire());
    CHECK_EQ(r.t.strokeMs(), totalMs(s));         // what the module reports for rumble / docs
  }
}

// The last dart: counted at the end of the pause, then the servo stays at REST (the reload
// position) and the sequence is over at once.
TEST(the_last_dart_stays_at_rest) {
  for (const Variant v : kVariants) {
    const float rate = v.rate;
    const bool clear = v.clear;
    Rig r(rated(rate, clear));
    Shot s = shotAt(rate, clear);
    s.chamber = false;
    r.step(in(true, false));
    r.fireSemi(MAG_CAPACITY - 1);
    CHECK_EQ(r.t.ammo(), 1);
    CHECK(r.servoAtPrefire());
    const Trace tr = traceStroke(r, in(true, true), 4 * MS_PER_S, [&] { r.step(in(true, true)); });
    CHECK(strokeShapeIs(tr, s));
    CHECK_EQ(r.t.ammo(), 0);
    CHECK(r.servoAtRest());
    r.run(kShot + kCool, in(true, true));
    CHECK(r.servoAtRest());
  }
}

// The sweep for either sign of the angles, the worst-case span and short ones: at the configured
// rate, monotonic, landing exactly on each angle, each hold its configured time.
TEST(sweep_glides_at_the_rate_for_either_direction_and_lands_exactly) {
  struct Angles {
    float rest, fire, prefire, clear;
  };
  const Angles cases[] = {{kRest, kFire, kPre, kClear},
                          {kFire, kRest, (kFire + kRest) / 2.0f, kFire + 10.0f},
                          {0.0f, (float)SERVO_SPAN_DEG, 90.0f, (float)SERVO_SPAN_DEG},
                          {(float)SERVO_SPAN_DEG, 0.0f, (float)SERVO_SPAN_DEG, 0.0f},
                          {90.0f, 93.0f, 91.0f, 90.0f}};
  for (const Angles& a : cases) {
    Rig r(gliding());
    r.t.setAngles(a.rest, a.fire, a.prefire, a.clear);
    r.t.abort();                                   // the servo to this PRE-FIRE first
    CHECK_EQ(servoLogical(), a.prefire);
    r.settle();
    r.step(in(true, false));
    const Trace tr = traceStroke(r, in(true, true), 4 * MS_PER_S, [&] { r.step(in(true, true)); });
    Shot s = glidingShot();
    s.from = a.prefire;
    s.rest = a.rest;
    s.fire = a.fire;
    s.prefire = a.prefire;
    s.clear = a.clear;
    CHECK(strokeShapeIs(tr, s));
    CHECK_EQ(r.t.shotsTotal(), 1);
    CHECK_EQ(r.t.strokeMs(), totalMs(s));
    CHECK_EQ(r.t.sweepMs(a.rest, a.fire), sweepFor(spanOf(a.fire, a.rest), kTestRate));
  }
}

TEST(semi_one_stroke_per_pull_and_held_trigger_does_not_refire) {
  Rig r;
  r.step(in(true, false));
  r.run(1, in(true, true));
  CHECK(r.t.strokeActive());
  int starts = 0;
  bool wasActive = true;
  for (uint32_t k = 0; k < 10 * (kShot + kCool); k++) {   // held across the sequence's end and long after
    r.run(1, in(true, true));
    if (r.t.strokeActive() && !wasActive) starts++;
    wasActive = r.t.strokeActive();
  }
  CHECK_EQ(starts, 0);
  CHECK_EQ(r.t.ammo(), MAG_CAPACITY - 1);
  CHECK(r.servoAtPrefire());
}

TEST(semi_release_rearms_and_each_pull_costs_exactly_one_dart) {
  Rig r;
  r.step(in(true, false));
  for (int k = 1; k <= 3; k++) {
    r.run(1, in(true, true));
    CHECK(r.t.strokeActive());
    r.run(kShot, in(true, true));
    CHECK(!r.t.strokeActive());
    CHECK_EQ(r.t.ammo(), MAG_CAPACITY - k);
    r.run(5, in(true, false));                    // released below the release threshold
    r.run(kCool, in(true, false));                // and the cooldown waited out
  }
  CHECK_EQ(r.t.shotsTotal(), 3);
}

// Semi cooldown: counted from the end of the sequence (the next shot would be available),
// nothing starts for TRIGGER_COOLDOWN_MS (gate COOLDOWN); a counting pull still held when it ends
// fires in that very update, also one that started during the stroke itself.
TEST(semi_cooldown_a_held_pull_fires_the_moment_it_ends) {
  for (int when = 0; when < 2; when++) {             // re-pulled during the stroke / the cooldown
    Rig r;
    r.step(in(true, false));
    r.run(1, in(true, true));
    const uint32_t t0 = r.now;
    if (when == 0) {
      r.run(kShot / 2, in(true, true));
      r.run(10, in(true, false));                    // released mid-stroke
      r.run(t0 + kShot - r.now, in(true, true));     // pulled again, held to the end
    } else {
      r.run(kShot, in(true, true));
    }
    CHECK_EQ(r.now, t0 + kShot);
    CHECK_EQ(r.t.phase(), Phase::IDLE);              // over ...
    CHECK_EQ(r.t.ammo(), MAG_CAPACITY - 1);
    CHECK(r.t.coolingDown());                        // ... and cooling down
    CHECK_EQ(r.t.gate(in(true, true)), Trigger::Gate::COOLDOWN);
    CHECK_EQ(r.t.cooldownMs(), kCool);
    if (when == 1) {
      r.run(kCool / 2, in(true, false));             // released ...
      r.run(1, in(true, true));                      // ... and pulled inside the cooldown
    }
    bool early = false;
    while (r.now < t0 + kShot + kCool - 1) {
      r.run(1, in(true, true));
      early = early || r.t.strokeActive();
    }
    CHECK(!early);
    CHECK_EQ(r.t.gate(in(true, true)), Trigger::Gate::COOLDOWN);
    r.run(1, in(true, true));                        // end + TRIGGER_COOLDOWN_MS
    CHECK_EQ(r.t.phase(), kFirst);
    CHECK(!r.t.coolingDown());
    CHECK_EQ(r.t.ammo(), MAG_CAPACITY - 1);          // the new shot has not counted yet
    r.run(kShot, in(true, true));
    CHECK_EQ(r.t.ammo(), MAG_CAPACITY - 2);
    r.run(3 * (kShot + kCool), in(true, true));      // held on: no third
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
      r.run(kShot + 1, in(true, false));             // released, the sequence ends
      r.run(kCool / 4, in(true, true));              // a tap inside the cooldown
      r.run(1, in(true, false));
    } else {
      r.run(kShot / 2, in(true, false));
      r.run(t0 + kShot + kCool - 1 - r.now, in(true, true));   // re-pulled, let go 1 ms early
      r.run(1, in(true, false));
    }
    r.run(3 * (kShot + kCool), in(true, false));
    CHECK(!r.t.strokeActive());
    CHECK_EQ(r.t.ammo(), MAG_CAPACITY - 1);
    CHECK_EQ(r.t.shotsTotal(), 1);
    r.run(1, in(true, true));                        // a fresh pull after it: fires
    CHECK(r.t.strokeActive());
  }
}

// Only a real shot's sequence starts a cooldown: not begin(), not an aborted stroke, not a
// teach-mode test stroke.
TEST(cooldown_starts_only_after_a_real_shot) {
  Rig r;
  CHECK_EQ(r.t.gate(in(true, false)), Trigger::Gate::OPEN);
  r.step(in(true, false));
  r.run(1, in(true, true));
  r.run(kCount - 1, in(true, true));                // the last ms before the shot counts
  CHECK(r.t.strokeActive());
  r.t.abort();
  CHECK(!r.t.coolingDown());
  CHECK_EQ(r.t.shotsTotal(), 0);
  r.run(kChamber + 1, in(true, false));             // back to PRE-FIRE: chambering
  r.run(1, in(true, true));                         // fires at once
  CHECK(r.t.strokeActive());
  r.t.abort();
  // A test stroke run to its end.
  Trigger::Input teach;
  teach.servoTeach = true;
  r.run(kChamber + 1, teach);
  CHECK(r.t.testStroke(r.now, teach));
  r.run(kShot, teach);
  CHECK(!r.t.strokeActive());
  CHECK(!r.t.coolingDown());
  r.run(1, in(true, false));
  r.run(1, in(true, true));
  CHECK(r.t.strokeActive());
  CHECK_EQ(r.t.shotsTotal(), 0);
}

// Full auto (owner, 2026-11-04), an exact 3-shot burst and the release, for every rate with and
// without CLEAR: shot 1 from PRE-FIRE; while held, each next shot starts in the very update its
// predecessor's last REST hold ends (the dart counted), straight from REST, never visiting
// PRE-FIRE or holding the chamber; released during shot 3, that shot finishes to REST, counts,
// then chambers to PRE-FIRE and the sequence ends. No cooldown anywhere.
TEST(full_auto_burst_goes_rest_to_fire_and_chambers_only_after_the_release) {
  for (const Variant v : kVariants) {
    Rig r(rated(v.rate, v.clear));
    r.t.setFireMode(DS_FIRE_MODE_FULL);
    CHECK_EQ(r.t.cooldownMs(), 0u);
    Shot first = shotAt(v.rate, v.clear);
    first.chamber = false;                        // PRE-FIRE -> ... -> the count at REST
    Shot next = first;
    next.from = kRest;                            // REST -> FIRE -> ... -> the count at REST
    const uint32_t firstMs = totalMs(first), cycleMs = totalMs(next);
    const uint32_t tailMs = sweepFor(spanOf(kPre, kRest), v.rate) + kChamber;   // REST -> PRE-FIRE, held
    const Phase head = sweepFor(spanOf(kFire, kPre), v.rate) ? Phase::SWEEP_OUT : Phase::HOLD_FIRE;
    const Phase headNext = sweepFor(spanOf(kFire, kRest), v.rate) ? Phase::SWEEP_OUT : Phase::HOLD_FIRE;
    r.step(in(true, false));
    std::vector<uint32_t> starts, counts;
    std::vector<float> startDeg;
    bool chamberedWhileHeld = false, idleWhileHeld = false, everCooldown = false;
    Phase was = Phase::IDLE;
    uint8_t ammoWas = r.t.ammo();
    bool held = true;
    auto watch = [&] {
      const Phase p = r.t.phase();
      if ((p == head || p == headNext) && p != was && (was == Phase::IDLE || was == Phase::HOLD_REST ||
                                                     was == Phase::HOLD_RACK)) {
        starts.push_back(r.now);
        startDeg.push_back(r.t.servoDeg());
      }
      if (r.t.ammo() != ammoWas) counts.push_back(r.now);
      ammoWas = r.t.ammo();
      if (held) {
        chamberedWhileHeld = chamberedWhileHeld || p == Phase::SWEEP_CHAMBER || p == Phase::HOLD_CHAMBER;
        idleWhileHeld = idleWhileHeld || !r.t.strokeActive();
      }
      everCooldown = everCooldown || r.t.coolingDown();
      was = p;
    };
    r.run(1, in(true, true));
    const uint32_t t0 = r.now;
    watch();
    // Held through shots 1 and 2 and into shot 3, released before shot 3 counts.
    const uint32_t releaseAt = t0 + firstMs + cycleMs + cycleMs / 2;
    while (r.now < releaseAt) {
      r.run(1, in(true, true));
      watch();
    }
    held = false;
    while (r.now < releaseAt + 2 * (cycleMs + tailMs)) {
      r.run(1, in(true, false));
      watch();
    }
    CHECK(!chamberedWhileHeld);
    CHECK(!idleWhileHeld);
    CHECK(!everCooldown);
    if (CHECK_EQ(starts.size(), (size_t)3)) {
      CHECK_EQ(starts[0], t0);
      CHECK_EQ(starts[1], t0 + firstMs);
      CHECK_EQ(starts[2], t0 + firstMs + cycleMs);
      if (v.rate > 0.0f) {                           // where each stroke's glide set out from
        CHECK_EQ(startDeg[1], kRest);
        CHECK_EQ(startDeg[2], kRest);
      }
    }
    if (CHECK_EQ(counts.size(), (size_t)3)) {
      CHECK_EQ(counts[0], t0 + firstMs);
      CHECK_EQ(counts[1], t0 + firstMs + cycleMs);
      CHECK_EQ(counts[2], t0 + firstMs + 2 * cycleMs);   // shot 3 finished after the release
    }
    CHECK_EQ(r.t.ammo(), MAG_CAPACITY - 3);
    CHECK_EQ(r.t.shotsTotal(), 3);
    CHECK(!r.t.strokeActive());
    CHECK(r.servoAtPrefire());                       // chambered after the release
    // The sequence ended exactly one chamber after shot 3 counted.
    Rig q(rated(v.rate, v.clear));
    q.t.setFireMode(DS_FIRE_MODE_FULL);
    q.step(in(true, false));
    q.run(1, in(true, true));
    const uint32_t q0 = q.now;
    q.run(firstMs + cycleMs + cycleMs / 2 - 1, in(true, true));
    while (q.t.strokeActive()) q.run(1, in(true, false));
    CHECK_EQ(q.now, q0 + firstMs + 2 * cycleMs + tailMs);
  }
}

// The cooldown belongs to semi: after a semi shot, a fresh full-auto pull fires at once; after a
// full-auto shot, a fresh semi pull waits out TRIGGER_COOLDOWN_MS from that sequence's end.
TEST(the_cooldown_is_semi_only_and_follows_the_mode_in_force) {
  Rig r;
  r.step(in(true, false));
  r.run(1, in(true, true));
  r.run(kShot, in(true, true));                     // a semi shot ends
  CHECK(r.t.coolingDown());
  r.t.setFireMode(DS_FIRE_MODE_FULL);
  r.run(1, in(true, false));
  r.run(1, in(true, true));
  CHECK(r.t.strokeActive());                        // full auto: no cooldown
  r.run(1, in(true, false));                        // released: this one shot finishes
  r.run(kShot, in(true, false));
  CHECK(!r.t.strokeActive());
  CHECK_EQ(r.t.ammo(), MAG_CAPACITY - 2);
  const uint32_t done = r.now - 1;                   // when it ended (it started 1 ms before the release)
  r.t.setFireMode(DS_FIRE_MODE_SEMI);
  r.run(1, in(true, true));
  CHECK_EQ(r.t.gate(in(true, true)), Trigger::Gate::COOLDOWN);
  r.run(done + kCool - 1 - r.now, in(true, true));
  CHECK(!r.t.strokeActive());
  r.run(1, in(true, true));
  CHECK(r.t.strokeActive());                        // held through it: fires as it ends
}

// KEY SAFETY TEST: an abort in any phase (abort(), armed dropping without an abort() call, the
// servo powered off) commands home (PRE-FIRE with darts left) at once, no sweep, never costs a
// dart or counts a shot and starts no cooldown; the servo stays home afterwards.
TEST(abort_in_every_phase_goes_home_now_never_decrements_and_starts_no_cooldown) {
  enum Kind { ABORT_CALL, DISARMED, POWER_OFF, KINDS };
  const char* const kinds[] = {"abort()", "armed false", "servo power off"};
  for (const Variant v : kVariants) {
    const float rate = v.rate;
    const bool clear = v.clear;
    const Shot s = shotAt(rate, clear);
    const uint32_t shot = totalMs(s);
    for (const Phase p : kShotPhases) {
      if (phaseLength(p, s) == 0) continue;            // no sweep configured: no sweep phases
      const bool counted = phaseStart(p, s) >= countMs(s);   // the chambering after the count
      for (uint32_t at : {phaseStart(p, s) + phaseLength(p, s) / 2, phaseStart(p, s) + phaseLength(p, s) - 1}) {
        for (int kind = ABORT_CALL; kind < KINDS; kind++) {
          Rig r(rated(rate, clear));
          r.step(in(true, false));
          r.run(1, in(true, true));
          r.run(at, in(true, true));
          if (!CHECK_EQ(r.t.phase(), p))
            std::printf("      %s at +%u ms, %s\n", phaseName(p), (unsigned)at, kinds[kind]);
          const uint8_t ammo = counted ? MAG_CAPACITY - 1 : MAG_CAPACITY;
          const uint16_t shots = counted ? 1 : 0;
          Trigger::Rumble ignore;
          r.t.takeRumbleRequest(ignore);
          if (kind == ABORT_CALL) r.t.abort();
          if (kind == DISARMED) r.run(1, in(false, true));
          if (kind == POWER_OFF) r.t.setServoPower(false);
          CHECK(r.servoAtPrefire());                   // in that very call: no sweep
          CHECK_EQ(r.t.servoDeg(), kPre);
          CHECK_EQ(r.t.phase(), Phase::IDLE);
          CHECK_EQ(r.t.ammo(), ammo);
          CHECK_EQ(r.t.shotsTotal(), shots);
          CHECK(!r.t.coolingDown());
          CHECK(!r.t.firingLedActive(r.now));
          Trigger::Rumble stop;
          CHECK(r.t.takeRumbleRequest(stop));          // the stroke's rumble is stopped
          CHECK_EQ(stop.ms, 0);
          bool stayedHome = true;
          for (uint32_t k = 0; k < shot + kCool; k++) {   // still armed and held (or disarmed)
            r.run(1, in(kind != DISARMED, true));
            stayedHome = stayedHome && r.servoAtPrefire() && !r.t.strokeActive();
          }
          CHECK(stayedHome);                            // the aborted pull does not restart it
          CHECK_EQ(r.t.ammo(), ammo);
          if (kind == POWER_OFF) r.t.setServoPower(true);
          r.run(1, in(true, false));                    // release and pull: fires again
          r.run(1, in(true, true));
          CHECK(r.t.strokeActive());
        }
      }
    }
  }
}

// At ammo 0 home is REST: an abort of a lockout-off dry stroke goes there.
TEST(abort_at_zero_ammo_goes_to_rest) {
  Trigger::Config c = cfg(false, FIRE_REQUIRES_FLYWHEEL, RUMBLE_FULL_AUTO_CONTINUOUS);
  c.sweepDegPerS = 0.0f;
  Rig r(c, false);
  r.step(in(true, false));
  r.run(1, in(true, true));                          // ammo 0, no lockout: a dry stroke
  CHECK(r.servoAtFire());
  r.t.abort();
  CHECK(r.servoAtRest());
  CHECK_EQ(r.t.ammo(), 0);
}

TEST(abort_in_full_auto_stops_cycling_until_a_fresh_pull) {
  Rig r;
  r.t.setFireMode(DS_FIRE_MODE_FULL);
  r.step(in(true, false));
  r.run(1, in(true, true));
  r.run(kShot + kCount / 2, in(true, true));           // into the second shot, before it counts
  CHECK_EQ(r.t.ammo(), MAG_CAPACITY - 1);
  r.t.abort();                                         // still armed, still held
  r.run(5 * kShot, in(true, true));
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
  r.run(5 * kShot, in(true, true, true));              // ready now, still held
  CHECK(!r.t.strokeActive());
  CHECK_EQ(r.t.ammo(), MAG_CAPACITY);
  // A pull waiting on the cooldown too.
  Rig c;
  c.step(in(true, false));
  c.fireSemi(1);
  c.run(1, in(true, true));
  c.run(kShot, in(true, true));                        // ended, cooling down
  c.run(1, in(true, false));
  c.run(1, in(true, true));                            // a new pull, blocked by the cooldown
  CHECK_EQ(c.t.gate(in(true, true)), Trigger::Gate::COOLDOWN);
  c.t.abort();
  c.run(kCool + kShot, in(true, true));
  CHECK(!c.t.strokeActive());
  CHECK_EQ(c.t.ammo(), MAG_CAPACITY - 2);
}

TEST(abort_while_idle_requests_no_rumble) {
  Rig r;
  r.t.abort();
  Trigger::Rumble out;
  CHECK(!r.t.takeRumbleRequest(out));
  CHECK(r.servoAtPrefire());
  CHECK(!r.t.chambering());                            // already home: nothing moved
}

TEST(disarm_mid_stroke_goes_home_at_once_without_an_abort_call) {
  Rig r;
  r.step(in(true, false));
  r.run(1, in(true, true));
  r.run(kOut + kFwd / 2, in(true, true));
  CHECK(r.servoAtFire());
  r.run(1, in(false, true));                         // TurretState: no longer ARMED
  CHECK(r.servoAtPrefire());
  CHECK(!r.t.strokeActive());
  CHECK_EQ(r.t.ammo(), MAG_CAPACITY);
  r.run(3 * kShot, in(false, true));
  CHECK(!r.t.strokeActive());
  CHECK_EQ(r.t.ammo(), MAG_CAPACITY);
}

// The abort's move home chambers too: re-armed at once, a fresh pull waits TRIGGER_CHAMBER_MS.
TEST(abort_from_fire_chambers_before_the_next_shot) {
  Rig r;
  r.step(in(true, false));
  r.run(1, in(true, true));
  r.run(kOut + kFwd / 2, in(true, true));
  r.t.abort();
  CHECK(r.t.chambering());
  r.run(1, in(true, false));                         // chambering starts here
  r.run(1, in(true, true));
  CHECK_EQ(r.t.gate(in(true, true)), Trigger::Gate::CHAMBERING);
  CHECK(!r.t.strokeActive());
  r.run(kChamber - 1, in(true, true));
  CHECK(r.t.strokeActive());                         // the held pull fires the moment it ends
}

TEST(servo_power_off_mid_stroke_aborts) {
  Rig r;
  r.step(in(true, false));
  r.run(1, in(true, true));
  r.t.setServoPower(false);
  CHECK(!r.t.strokeActive());
  CHECK(!hal_host::servoOn());
  CHECK_EQ(servoLogical(), kPre);
  CHECK_EQ(r.t.ammo(), MAG_CAPACITY);
  CHECK_EQ(r.t.gate(in(true, true)), Trigger::Gate::SERVO_OFF);
}

TEST(ammo_floor_is_zero_when_lockout_is_off) {
  Rig r(cfg(false, FIRE_REQUIRES_FLYWHEEL, RUMBLE_FULL_AUTO_CONTINUOUS));
  r.step(in(true, false));
  r.fireSemi(MAG_CAPACITY + 3);
  CHECK_EQ(r.t.ammo(), 0);                            // never wraps to 255
  CHECK_EQ(r.t.shotsTotal(), MAG_CAPACITY + 3);       // kept cycling at 0
  CHECK(r.servoAtRest());                             // dry strokes end at REST
  CHECK(!r.t.emptyMagIndicator());
  r.run(1, in(true, true));
  CHECK(r.t.strokeActive());
  CHECK(!r.t.emptyMagIndicator());                    // no lockout, no blink
  CHECK_EQ(r.t.gate(in(true, false)), Trigger::Gate::STROKE_RUNNING);
}

TEST(empty_magazine_with_lockout_never_strokes_and_blinks_only_while_armed_and_pulled) {
  Rig r(cfg(true, FIRE_REQUIRES_FLYWHEEL, RUMBLE_FULL_AUTO_CONTINUOUS));
  r.step(in(true, false));
  r.fireSemi(MAG_CAPACITY);
  CHECK_EQ(r.t.ammo(), 0);
  CHECK_EQ(r.t.gate(in(true, true)), Trigger::Gate::EMPTY);
  CHECK(!r.t.emptyMagIndicator());                    // armed, not pulled
  r.run(1, in(true, true));
  CHECK(!r.t.strokeActive());
  CHECK(r.servoAtRest());
  CHECK(r.t.emptyMagIndicator());                     // armed + pulled + empty + lockout
  r.run(500, in(true, true));
  CHECK(!r.t.strokeActive());
  CHECK_EQ(r.t.shotsTotal(), MAG_CAPACITY);
  r.run(1, in(true, false));
  CHECK(!r.t.emptyMagIndicator());                    // released
  r.run(1, in(false, true));
  CHECK(!r.t.emptyMagIndicator());                    // disarmed
  Trigger::Gate g = r.t.gate(in(false, true));
  CHECK_EQ(g, Trigger::Gate::NOT_ARMED);
  // X: magazine back, servo to PRE-FIRE; after the chambering a new pull fires.
  r.run(1, in(true, false));
  r.t.resetMagazine();
  CHECK_EQ(r.t.ammo(), MAG_CAPACITY);
  CHECK(r.servoAtPrefire());
  r.run(1, in(true, true));
  CHECK(!r.t.strokeActive());
  CHECK(!r.t.emptyMagIndicator());
  r.run(kChamber, in(true, true));
  CHECK(r.t.strokeActive());
}

// Empty during the cooldown after the last dart: EMPTY is reported, the reason that lasts.
TEST(the_last_dart_reports_empty_rather_than_the_cooldown) {
  Rig r(strict());
  r.step(in(true, false));
  r.fireSemi(MAG_CAPACITY - 1);
  r.run(1, in(true, true));
  r.run(kLast, in(true, true));
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
  r.run(kChamber + 1, in(true, true));                // fires once chambered
  CHECK(r.t.strokeActive());
  r.run(5 * kShot, in(true, true));
  CHECK_EQ(r.t.ammo(), MAG_CAPACITY - 1);             // once, not continuously
}

TEST(full_auto_stops_on_the_last_dart_at_rest) {
  Rig r(strict());
  r.step(in(true, false));
  r.fireSemi(MAG_CAPACITY - 2);
  r.t.setFireMode(DS_FIRE_MODE_FULL);
  r.run(1, in(true, false));
  r.run(1, in(true, true));
  r.run(10 * kShot, in(true, true));
  CHECK_EQ(r.t.ammo(), 0);
  CHECK_EQ(r.t.shotsTotal(), MAG_CAPACITY);
  CHECK(!r.t.strokeActive());
  CHECK(r.servoAtRest());
  CHECK(r.t.emptyMagIndicator());
}

TEST(flywheel_gate_follows_fire_requires_flywheel) {
  {
    Rig r(cfg(FIRE_LOCKOUT_WHEN_EMPTY, true, RUMBLE_FULL_AUTO_CONTINUOUS));
    r.step(in(true, false, false));
    CHECK_EQ(r.t.gate(in(true, true, false)), Trigger::Gate::FLYWHEEL_NOT_READY);
    r.run(1, in(true, true, false));
    r.run(2 * kShot, in(true, true, false));
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
  r.run(5 * kShot, in(true, true));                    // then armed while still held
  CHECK(!r.t.strokeActive());
  CHECK_EQ(r.t.ammo(), MAG_CAPACITY);
  r.t.setFireMode(DS_FIRE_MODE_FULL);                  // full auto: same rule
  r.run(5 * kShot, in(true, true));
  CHECK_EQ(r.t.ammo(), MAG_CAPACITY);
  r.run(1, in(true, false));                           // released ...
  r.run(1, in(true, true));                            // ... and pulled while armed: fires
  CHECK(r.t.strokeActive());

  // Armed pull, disarm while held, re-arm while still held: no stroke.
  Rig q;
  q.step(in(true, false));
  q.run(1, in(true, true));
  q.run(kShot, in(true, true));                        // one semi shot
  q.run(1, in(true, false));
  q.run(kCool, in(true, false));                       // the cooldown waited out
  q.run(1, in(true, true));
  CHECK(q.t.strokeActive());
  q.run(10, in(false, true));                          // disarmed: aborted
  q.run(5 * kShot, in(true, true));                    // re-armed, still holding
  CHECK(!q.t.strokeActive());
  CHECK_EQ(q.t.ammo(), MAG_CAPACITY - 1);

  // Pulled in the very loop before arming (one disarmed update, then armed): no stroke.
  Rig p;
  p.step(in(false, false));
  p.run(1, in(false, true));
  p.run(5 * kShot, in(true, true));
  CHECK(!p.t.strokeActive());
  CHECK_EQ(p.t.ammo(), MAG_CAPACITY);

  // A counting pull that is still blocked (no stroke yet), disarmed while held, re-armed while
  // held and then unblocked: it stopped counting at the disarm, so no stroke.
  Rig b(strict());
  b.step(in(true, false, false));
  b.run(1, in(true, true, false));                     // armed pull, flywheel not ready
  b.run(1, in(false, true, false));                    // disarmed, still held
  b.run(1, in(true, true, false));                     // armed again, still held
  b.run(5 * kShot, in(true, true, true));              // flywheel ready
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
  r.run(10 * kShot, in(true, true, true));
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
  r.run(kShot, in(true, true));
  CHECK(!r.t.takeRumbleRequest(out));                  // none at the sequence's end
}

TEST(rumble_full_auto_continuous_and_per_shot) {
  for (int continuous = 0; continuous < 2; continuous++) {
    Rig r(cfg(FIRE_LOCKOUT_WHEN_EMPTY, FIRE_REQUIRES_FLYWHEEL, continuous == 1));
    r.t.setFireMode(DS_FIRE_MODE_FULL);
    r.step(in(true, false));
    int requests = 0;
    bool allRight = true;
    std::vector<uint32_t> at;
    Shot first;
    first.chamber = false;
    Shot next = first;
    next.from = kRest;
    const uint32_t firstMs = totalMs(first), cycleMs = totalMs(next);   // the burst (full auto)
    r.run(1, in(true, true));                        // shot 1
    Trigger::Rumble out0;
    if (r.t.takeRumbleRequest(out0)) {
      requests++;
      at.push_back(r.now);
    }
    for (uint32_t k = 0; k < firstMs + 2 * cycleMs + cycleMs / 2; k++) {   // shots 2, 3 and 4
      r.run(1, in(true, true));
      Trigger::Rumble out;
      if (r.t.takeRumbleRequest(out)) {
        requests++;
        at.push_back(r.now);
        const uint16_t want = continuous ? (uint16_t)(kShot + RUMBLE_DURATION_MS) : (uint16_t)RUMBLE_DURATION_MS;
        allRight = allRight && out.ms == want && out.strong == RUMBLE_STRONG_255 && out.weak == RUMBLE_WEAK_255;
      }
    }
    CHECK_EQ(requests, 4);                             // renewed every shot
    CHECK(allRight);
    // Continuous: each request lasts past the next renewal, so the pad never stops while
    // cycling; per shot: each ends before the next stroke.
    for (size_t k = 1; k < at.size(); k++) {
      const uint32_t gap = at[k] - at[k - 1];
      if (continuous)
        CHECK(kShot + RUMBLE_DURATION_MS > gap);
      else
        CHECK_EQ(gap, k == 1 ? firstMs : cycleMs);
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

// The continuous rumble is one shot's sequence at the angles in force (SERVO TEACH records change
// it), and the worst case, sweeps over the whole SERVO_SPAN_DEG, still fits StatusPacket.rumbleMs (u16).
TEST(continuous_rumble_follows_the_angles_in_force_and_fits_u16) {
  struct Angles {
    float rest, fire, prefire, clear;
  };
  for (const Angles a :
       {Angles{100.0f, 90.0f, 95.0f, 98.0f}, Angles{0.0f, (float)SERVO_SPAN_DEG, 0.0f, (float)SERVO_SPAN_DEG}}) {
    Trigger::Config c = gliding();
    c.rumbleFullAutoContinuous = true;
    Rig r(c);
    r.t.setAngles(a.rest, a.fire, a.prefire, a.clear);
    r.t.abort();
    r.settle();
    r.t.setFireMode(DS_FIRE_MODE_FULL);
    r.step(in(true, false));
    Trigger::Rumble ignore;
    r.t.takeRumbleRequest(ignore);
    r.run(1, in(true, true));
    Trigger::Rumble out;
    CHECK(r.t.takeRumbleRequest(out));
    Shot s = glidingShot();
    s.from = s.prefire = a.prefire;
    s.clear = a.clear;
    s.rest = a.rest;
    s.fire = a.fire;
    const uint32_t shot = totalMs(s);
    CHECK(shot + RUMBLE_DURATION_MS <= UINT16_MAX);
    CHECK_EQ(out.ms, (uint16_t)(shot + RUMBLE_DURATION_MS));
    // ... and it covers the gap to the next shot's start.
    r.run(shot, in(true, true));
    CHECK(r.t.takeRumbleRequest(out));
  }
}

TEST(firing_led_covers_the_whole_sequence_and_at_least_the_minimum_flash) {
  for (const Variant v : kVariants) {
    const float rate = v.rate;
    const bool clear = v.clear;
    Rig r(rated(rate, clear));
    const uint32_t shot = totalMs(shotAt(rate, clear));
    r.step(in(true, false));
    r.run(1, in(true, true));
    const uint32_t t0 = r.now;
    const uint32_t holdEnd = std::max<uint32_t>(shot, LED_FIRE_FLASH_MIN_MS);
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
    r.run(shot, in(true, false));
    CHECK(!r.t.firingLedActive(r.now));
  }
}

TEST(shot_counter_counts_counted_shots_only) {
  Rig r;
  r.step(in(true, false));
  r.fireSemi(2);
  CHECK_EQ(r.t.shotsTotal(), 2);
  r.run(1, in(true, true));
  r.run(kOut + kFwd + 1, in(true, true));
  r.t.abort();                                        // aborted: not counted
  CHECK_EQ(r.t.shotsTotal(), 2);
  CHECK_EQ(r.t.ammo(), MAG_CAPACITY - 2);
  r.t.resetMagazine();
  CHECK_EQ(r.t.ammo(), MAG_CAPACITY);
  CHECK_EQ(r.t.shotsTotal(), 2);                      // the counter is not the magazine
}

// The shell's `fire`: every gate, the chambering and the semi cooldown included (refused until
// the very ms each ends).
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
  r.run(kShot, in(true, false));
  CHECK_EQ(r.t.ammo(), MAG_CAPACITY - 1);
  CHECK(!r.t.strokeActive());                         // exactly one shot
  const uint32_t done = t0 + kShot;
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
  f.run(kShot, in(true, false));
  CHECK(f.t.fireOnce(f.now, in(true, false)));
  // Right after X from REST: refused until the chambering is over, also without an update.
  Rig x(strict(), false);
  x.t.resetMagazine();
  CHECK(!x.t.fireOnce(x.now, in(true, false)));       // chambering starts at this call
  CHECK_EQ(x.t.gate(in(true, false)), Trigger::Gate::CHAMBERING);
  CHECK(!x.t.fireOnce(x.now + kChamber - 1, in(true, false)));
  CHECK(x.t.fireOnce(x.now + kChamber, in(true, false)));
}

TEST(fire_mode_change_needs_a_fresh_pull) {
  Rig r;
  r.t.setFireMode(DS_FIRE_MODE_FULL);
  r.step(in(true, false));
  r.run(1, in(true, true));
  r.run(kShot / 2, in(true, true));
  r.t.toggleFireMode();                               // B while cycling: -> semi
  CHECK_EQ(r.t.fireMode(), DS_FIRE_MODE_SEMI);
  r.run(5 * kShot, in(true, true));
  CHECK_EQ(r.t.ammo(), MAG_CAPACITY - 1);             // the running shot finished, no more
  r.t.toggleFireMode();                               // -> full while still held
  r.run(5 * kShot, in(true, true));
  CHECK_EQ(r.t.ammo(), MAG_CAPACITY - 1);
  r.run(1, in(true, false));
  r.run(1, in(true, true));
  CHECK(r.t.strokeActive());
  r.t.setFireMode(DS_FIRE_MODE_COUNT);                // not a mode: ignored
  CHECK_EQ(r.t.fireMode(), DS_FIRE_MODE_FULL);
}

// ---- stroke angles and SERVO TEACH support (Trigger.h) ---------------------------------------------
namespace {
Trigger::Input teachIn(bool on = true, bool armed = false) {
  Trigger::Input i;
  i.servoTeach = on;
  i.armed = armed;
  return i;
}
// Recorded angles that differ from config.h's (3, 2, 4 and 5 degrees away, inside the servo's span).
float teachRest() {
  return TRIGGER_REST_ANGLE + 3 <= SERVO_SPAN_DEG ? TRIGGER_REST_ANGLE + 3.0f : TRIGGER_REST_ANGLE - 3.0f;
}
float teachFire() { return TRIGGER_FIRE_ANGLE >= 2 ? TRIGGER_FIRE_ANGLE - 2.0f : TRIGGER_FIRE_ANGLE + 2.0f; }
float teachPre() {
  return TRIGGER_PREFIRE_ANGLE + 4 <= SERVO_SPAN_DEG ? TRIGGER_PREFIRE_ANGLE + 4.0f : TRIGGER_PREFIRE_ANGLE - 4.0f;
}
float teachClear() {
  return TRIGGER_CLEAR_ANGLE + 5 <= SERVO_SPAN_DEG ? TRIGGER_CLEAR_ANGLE + 5.0f : TRIGGER_CLEAR_ANGLE - 5.0f;
}
Shot teachShot(float from, float rate = kTestRate, bool clear = TRIGGER_CLEAR_ENABLED) {
  Shot s;
  s.clearStop = clear;
  s.from = from;
  s.rest = teachRest();
  s.fire = teachFire();
  s.prefire = teachPre();
  s.clear = teachClear();
  s.rate = rate;
  return s;
}
}  // namespace

// setAngles() changes what every stroke, each sweep and abort() command (it moves nothing by
// itself: the first stroke starts from where the servo is); resetAngles() brings config.h's back.
TEST(set_angles_drive_every_stroke_and_abort_until_reset) {
  Rig r(gliding());
  r.t.setAngles(teachRest(), teachFire(), teachPre(), teachClear());
  CHECK(r.servoAtPrefire());                               // nothing moved yet
  r.step(in(true, false));
  const float from = r.t.servoDeg();
  Trace tr = traceStroke(r, in(true, true), 4 * MS_PER_S, [&] { r.step(in(true, true)); });
  CHECK(strokeShapeIs(tr, teachShot(from)));
  CHECK_EQ(r.t.shotsTotal(), (uint16_t)1);
  CHECK_EQ(servoLogical(), teachPre());
  r.run(1, in(true, false));
  r.run(kCool, in(true, false));
  tr = traceStroke(r, in(true, true), 4 * MS_PER_S, [&] { r.step(in(true, true)); });
  CHECK(strokeShapeIs(tr, teachShot(teachPre())));         // from the recorded PRE-FIRE now
  r.run(1, in(true, false));
  r.run(kCool, in(true, false));
  r.step(in(true, true));
  r.run(sweepFor(spanOf(teachFire(), teachPre()), kTestRate), in(true, true));
  CHECK_EQ(servoLogical(), teachFire());
  r.t.abort();
  CHECK_EQ(servoLogical(), teachPre());
  r.t.resetAngles();
  CHECK_EQ(r.t.restDeg(), kRest);
  CHECK_EQ(r.t.fireDeg(), kFire);
  CHECK_EQ(r.t.prefireDeg(), kPre);
  CHECK_EQ(r.t.clearDeg(), kClear);
  r.t.abort();
  CHECK(r.servoAtPrefire());
}

TEST(teach_move_only_in_teach_mode_never_armed_never_mid_stroke) {
  Rig r(rated(0.0f));
  const float a = teachRest();
  CHECK(!r.t.teachMove(a, in(false, false)));               // not in teach mode
  CHECK(!r.t.teachMove(a, teachIn(true, true)));            // armed
  CHECK(r.servoAtPrefire());
  CHECK(!r.t.offRest());
  CHECK(r.t.teachMove(a, teachIn()));
  CHECK_EQ(servoLogical(), a);
  CHECK_EQ(r.t.servoDeg(), a);
  CHECK(r.t.offRest());
  r.t.setAngles(teachRest(), teachFire(), teachPre(), teachClear());
  CHECK(r.t.testStroke(r.now, teachIn()));
  CHECK(!r.t.teachMove(teachFire(), teachIn()));            // a stroke runs
  CHECK_EQ(r.t.phase(), Phase::HOLD_CHAMBER);               // the lead-in: PRE-FIRE first (no glide)
  CHECK_EQ(servoLogical(), teachPre());
  r.run(kChamber, teachIn());
  CHECK_EQ(servoLogical(), teachFire());
}

// The test stroke: the whole shot sequence PRE-FIRE -> FIRE -> REST -> pause -> PRE-FIRE (from an
// angle off PRE-FIRE it first sweeps there and holds TRIGGER_CHAMBER_MS), the same sweeps and
// holds as a real shot, firing LED, no rumble, no ammo change, no shot counted, no cooldown; a
// real stroke cannot start over it.
TEST(test_stroke_runs_the_whole_sequence_costs_nothing_and_needs_teach_mode) {
  for (const Variant v : kVariants) {
    const float rate = v.rate;
    const bool clear = v.clear;
    Rig r(rated(rate, clear));
    r.t.setAngles(teachRest(), teachFire(), teachPre(), teachClear());
    CHECK(!r.t.testStroke(r.now, in(false, false)));
    CHECK(!r.t.testStroke(r.now, teachIn(true, true)));
    r.t.setServoPower(false);
    CHECK(!r.t.testStroke(r.now, teachIn()));
    r.t.setServoPower(true);
    const float from = (teachRest() + teachFire()) / 2.0f + 0.5f;   // somewhere between, off-grid
    CHECK(r.t.teachMove(from, teachIn()));
    Shot lead = teachShot(from, rate, clear);
    lead.leadIn = true;
    const Trace tr = traceStroke(r, teachIn(), 4 * MS_PER_S, [&] { CHECK(r.t.testStroke(r.now, teachIn())); });
    CHECK(strokeShapeIs(tr, lead));
    CHECK(!r.t.offRest());                                     // it ended at PRE-FIRE by itself
    // Again from PRE-FIRE (no lead-in), watching the LED and the gate.
    bool ledAlways = true, gateClosed = true;
    const Trace again = traceStroke(r, teachIn(), 4 * MS_PER_S, [&] { CHECK(r.t.testStroke(r.now, teachIn())); });
    CHECK(strokeShapeIs(again, teachShot(teachPre(), rate, clear)));
    CHECK(r.t.testStroke(r.now, teachIn()));
    CHECK(r.t.testStrokeActive());
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
    CHECK_EQ(servoLogical(), teachPre());
  }
  // At ammo 0 too (no dart, no count): still the whole sequence, ending at PRE-FIRE.
  Rig z(Trigger::defaults(), false);
  z.t.setAngles(teachRest(), teachFire(), teachPre(), teachClear());
  CHECK(z.t.teachMove(teachPre(), teachIn()));
  const Trace tr = traceStroke(z, teachIn(), 4 * MS_PER_S, [&] { CHECK(z.t.testStroke(z.now, teachIn())); });
  CHECK(strokeShapeIs(tr, teachShot(teachPre(), kRate)));
  CHECK_EQ(z.t.ammo(), 0);
  CHECK(z.t.offRest());                                      // PRE-FIRE is not home at ammo 0 ...
  z.step(in(false, false));                                  // ... so leaving teach mode sends it to REST
  CHECK_EQ(servoLogical(), teachRest());
  CHECK(!z.t.offRest());
}

// KEY SAFETY TEST: a test stroke never outlives teach mode, in whatever phase teach mode drops
// (or armed rises), and a teach angle never outlives it either, even without an abort() call; an
// aborted test stroke requests no rumble stop and goes home at once (no sweep).
TEST(teach_mode_dropping_sends_the_servo_home_without_an_abort_call) {
  for (const Variant v : kVariants) {
    const float rate = v.rate;
    const bool clear = v.clear;
    const Shot s = teachShot(teachPre(), rate, clear);
    for (const Phase p : kShotPhases) {
      if (phaseLength(p, s) == 0) continue;
      for (int k = 0; k < 2; k++) {
        Rig r(rated(rate, clear));
        r.t.setAngles(teachRest(), teachFire(), teachPre(), teachClear());
        r.t.abort();
        CHECK(r.t.testStroke(r.now, teachIn()));
        r.run(phaseStart(p, s) + phaseLength(p, s) / 2, teachIn());
        CHECK_EQ(r.t.phase(), p);
        r.step(k == 0 ? in(false, false) : teachIn(true, true));   // teach dropped / armed rose
        CHECK(!r.t.strokeActive());
        CHECK_EQ(servoLogical(), teachPre());
        CHECK_EQ(r.t.shotsTotal(), (uint16_t)0);
        CHECK_EQ(r.t.ammo(), (uint8_t)MAG_CAPACITY);
        CHECK(!r.t.coolingDown());
        Trigger::Rumble rumble;
        CHECK(!r.t.takeRumbleRequest(rumble));
      }
    }
  }
  Rig r;
  CHECK(r.t.teachMove(teachFire(), teachIn()));
  r.run(10, teachIn());
  CHECK_EQ(servoLogical(), teachFire());            // held while in teach mode
  r.step(in(false, false));
  CHECK(r.servoAtPrefire());
  CHECK(!r.t.offRest());
  CHECK(r.t.teachMove(teachFire(), teachIn()));
  r.t.setServoPower(false);                                 // parked; the next update sends it home
  r.step(in(false, false));
  CHECK(r.servoAtPrefire());
}

// TRIGGER_SERVO_REVERSED (owner, 2026-11-03): the servo receives SERVO_SPAN_DEG - angle, so the
// racked REST (180) is the servo's own 0 end and every move toward FIRE turns it the other way.
TEST(reversed_servo_receives_the_mirrored_angle) {
  Rig r(Trigger::defaults(), false);   // begin() and servo power on, the boot ammo
  const float sent = hal_host::servoAngle();
  const float home = TRIGGER_AMMO_AT_BOOT > 0 ? kPre : kRest;
  if (TRIGGER_SERVO_REVERSED) {
    CHECK_NEAR(sent, (float)SERVO_SPAN_DEG - home, 0.001f);
  } else {
    CHECK_NEAR(sent, home, 0.001f);
  }
  CHECK_NEAR(Trigger::physicalDeg(Trigger::physicalDeg(37.0f)), 37.0f, 0.001f);   // its own inverse
}

TESTKIT_MAIN()

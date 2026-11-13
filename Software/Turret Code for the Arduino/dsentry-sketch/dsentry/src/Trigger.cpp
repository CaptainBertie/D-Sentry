// Trigger.cpp -- see Trigger.h.
//
// SPDX-License-Identifier: MIT
#include "Trigger.h"

#include "hal.h"

static_assert(TRIGGER_REST_ANGLE >= 0 && TRIGGER_REST_ANGLE <= SERVO_SPAN_DEG,
              "TRIGGER_REST_ANGLE must be within the servo's 0..SERVO_SPAN_DEG");
static_assert(TRIGGER_FIRE_ANGLE >= 0 && TRIGGER_FIRE_ANGLE <= SERVO_SPAN_DEG,
              "TRIGGER_FIRE_ANGLE must be within the servo's 0..SERVO_SPAN_DEG");
static_assert(MAG_CAPACITY > 0 && MAG_CAPACITY <= UINT8_MAX, "MAG_CAPACITY must fit the u8 ammo count");
static_assert(RUMBLE_STRONG_255 <= UINT8_MAX && RUMBLE_WEAK_255 <= UINT8_MAX, "rumble strengths are 0..255");
static_assert(TRIGGER_SWEEP_DEG_PER_S >= 0.0f, "TRIGGER_SWEEP_DEG_PER_S must be 0 (no sweep) or positive");

namespace {

inline bool elapsed(uint32_t nowMs, uint32_t sinceMs, uint32_t periodMs) {
  return (uint32_t)(nowMs - sinceMs) >= periodMs;
}

// The longest sweep any two angles can need (the whole SERVO_SPAN_DEG at TRIGGER_SWEEP_DEG_PER_S),
// rounded up past its last whole ms; 0 without a sweep. Teach angles stay inside the span.
constexpr uint32_t kWorstSweepMs =
    TRIGGER_SWEEP_DEG_PER_S > 0.0f
        ? (uint32_t)((float)SERVO_SPAN_DEG / TRIGGER_SWEEP_DEG_PER_S * (float)MS_PER_S) + 1u
        : 0u;

}  // namespace

static_assert(kWorstSweepMs + TRIGGER_FORWARD_MS + kWorstSweepMs + TRIGGER_RETURN_MS + RUMBLE_DURATION_MS <=
                  UINT16_MAX,
              "the continuous full-auto rumble (a whole stroke at the worst-case SERVO_SPAN_DEG sweep, plus "
              "RUMBLE_DURATION_MS) must fit StatusPacket.rumbleMs (u16): raise TRIGGER_SWEEP_DEG_PER_S");

Trigger::Config Trigger::defaults() {
  Config c;
  c.lockoutWhenEmpty = FIRE_LOCKOUT_WHEN_EMPTY;
  c.requiresFlywheel = FIRE_REQUIRES_FLYWHEEL;
  c.rumbleFullAutoContinuous = RUMBLE_FULL_AUTO_CONTINUOUS;
  c.sweepDegPerS = TRIGGER_SWEEP_DEG_PER_S;
  c.semiCooldownMs = TRIGGER_COOLDOWN_MS;
  return c;
}

Trigger::Trigger() : cfg_(defaults()) {}

Trigger::Trigger(const Config& cfg) : cfg_(cfg) {}

void Trigger::begin() {
  fireMode_ = DS_FIRE_MODE_SEMI;
  phase_ = Phase::IDLE;
  phaseStartMs_ = 0;
  sweepFromDeg_ = TRIGGER_REST_ANGLE;
  sweepToDeg_ = TRIGGER_REST_ANGLE;
  cooling_ = false;
  recentShot_ = false;
  completedMs_ = 0;
  ledHoldStartMs_ = 0;
  ledHold_ = false;
  ammo_ = MAG_CAPACITY;
  shotsTotal_ = 0;
  prevPulled_ = false;
  pullArmed_ = false;
  pullUsed_ = false;
  rumblePending_ = false;
  rumble_ = Rumble();
  testStroke_ = false;
  teachMoved_ = false;
  resetAngles();
  command(restDeg_);
  hal::servoEnable(false);
}

// TRIGGER_SERVO_REVERSED mirrors every angle in this one place, just before the servo. Everything
// above it (config.h angles, SERVO ADJUST, the LCD, telemetry's servoDeg) uses the same mirrored
// numbers, so the owner never has to think about the servo's own direction.
float Trigger::physicalDeg(float deg) {
  return TRIGGER_SERVO_REVERSED ? (float)SERVO_SPAN_DEG - deg : deg;
}

void Trigger::command(float deg) {
  servoDeg_ = deg;
  hal::servoWriteDeg(physicalDeg(deg));
}

Trigger::Gate Trigger::gate(const Input& in) const {
  if (!in.armed) return Gate::NOT_ARMED;
  if (strokeActive()) return Gate::STROKE_RUNNING;
  if (!servoPowered()) return Gate::SERVO_OFF;
  if (ammo_ == 0 && cfg_.lockoutWhenEmpty) return Gate::EMPTY;
  if (cfg_.requiresFlywheel && !in.flywheelReady) return Gate::FLYWHEEL_NOT_READY;
  if (cooling_) return Gate::COOLDOWN;
  return Gate::OPEN;
}

uint32_t Trigger::cooldownMs() const {
  // Full auto cycles back-to-back (spec 3.4). A full-auto pause would be returned here (and
  // counted in longestCooldownMs()).
  return fireMode_ == DS_FIRE_MODE_SEMI ? cfg_.semiCooldownMs : 0;
}

uint32_t Trigger::longestCooldownMs() const { return cfg_.semiCooldownMs; }

// The cooldown runs from the last completed stroke for cooldownMs() of the mode in force now, so
// switching full -> semi straight after a burst still waits it out. `recentShot_` drops once no
// mode's cooldown can still be running, so the 32-bit ms clock can wrap safely.
void Trigger::refreshCooldown(uint32_t nowMs) {
  if (recentShot_ && elapsed(nowMs, completedMs_, longestCooldownMs())) recentShot_ = false;
  cooling_ = recentShot_ && !elapsed(nowMs, completedMs_, cooldownMs());
}

// ---- the sweep: the commanded angle glides at Config::sweepDegPerS --------------------------------
float Trigger::sweptDeg(uint32_t elapsedMs) const {
  return cfg_.sweepDegPerS * (float)elapsedMs / (float)MS_PER_S;
}

uint32_t Trigger::sweepMs(float fromDeg, float toDeg) const {
  const float span = toDeg > fromDeg ? toDeg - fromDeg : fromDeg - toDeg;
  if (!(cfg_.sweepDegPerS > 0.0f) || !(span > 0.0f)) return 0;
  // Within a ms or two of the answer; then exactly sweepStep()'s landing test.
  uint32_t ms = (uint32_t)(span / cfg_.sweepDegPerS * (float)MS_PER_S);
  while (ms > 0 && sweptDeg(ms - 1) >= span) ms--;
  while (sweptDeg(ms) < span) ms++;
  return ms;
}

uint32_t Trigger::strokeMs() const {
  return sweepMs(restDeg_, fireDeg_) + TRIGGER_FORWARD_MS + sweepMs(fireDeg_, restDeg_) + TRIGGER_RETURN_MS;
}

void Trigger::beginSweep(Phase phase, float toDeg, uint32_t nowMs) {
  phase_ = phase;
  phaseStartMs_ = nowMs;
  sweepFromDeg_ = servoDeg_;   // from where the servo is commanded now
  sweepToDeg_ = toDeg;
}

bool Trigger::sweepStep(uint32_t nowMs) {
  const float span = sweepToDeg_ > sweepFromDeg_ ? sweepToDeg_ - sweepFromDeg_ : sweepFromDeg_ - sweepToDeg_;
  const float travel = sweptDeg((uint32_t)(nowMs - phaseStartMs_));
  if (!(cfg_.sweepDegPerS > 0.0f) || travel >= span) {
    command(sweepToDeg_);   // lands exactly on the end angle (no sweep: straight there)
    return true;
  }
  const float deg = sweepToDeg_ > sweepFromDeg_ ? sweepFromDeg_ + travel : sweepFromDeg_ - travel;
  if (deg != servoDeg_) command(deg);   // at most once per ms: nowMs is whole ms
  return false;
}

void Trigger::advance(uint32_t nowMs) {
  // Each phase starts at the update() that commands it; a zero-length one (no sweep) passes
  // straight on in the same update().
  if (phase_ == Phase::SWEEP_OUT && sweepStep(nowMs)) {
    phase_ = Phase::HOLD_FIRE;
    phaseStartMs_ = nowMs;
  }
  if (phase_ == Phase::HOLD_FIRE && elapsed(nowMs, phaseStartMs_, TRIGGER_FORWARD_MS))
    beginSweep(Phase::SWEEP_BACK, restDeg_, nowMs);
  if (phase_ == Phase::SWEEP_BACK && sweepStep(nowMs)) {
    phase_ = Phase::HOLD_REST;
    phaseStartMs_ = nowMs;
  }
  if (phase_ == Phase::HOLD_REST && elapsed(nowMs, phaseStartMs_, TRIGGER_RETURN_MS)) completeStroke(nowMs);
}

void Trigger::beginStroke(uint32_t nowMs) {
  beginSweep(Phase::SWEEP_OUT, fireDeg_, nowMs);
  ledHold_ = true;
  ledHoldStartMs_ = nowMs;
  advance(nowMs);   // without a sweep FIRE is commanded in this update(), as before
}

void Trigger::startStroke(uint32_t nowMs, bool continuousRumble) {
  testStroke_ = false;
  teachMoved_ = false;
  beginStroke(nowMs);
  pullUsed_ = true;
  rumble_.strong = RUMBLE_STRONG_255;
  rumble_.weak = RUMBLE_WEAK_255;
  // A whole stroke with the angles in force, so the next stroke's request overlaps this one.
  // config.h's worst case fits u16 (static_assert above); a test Config is clamped.
  const uint32_t ms = continuousRumble ? strokeMs() + RUMBLE_DURATION_MS : (uint32_t)RUMBLE_DURATION_MS;
  rumble_.ms = (uint16_t)(ms < (uint32_t)UINT16_MAX ? ms : (uint32_t)UINT16_MAX);
  rumblePending_ = true;
}

// The one place the magazine and the shot counter change: the stroke has swept out, held FIRE,
// swept back and held REST.
void Trigger::completeStroke(uint32_t nowMs) {
  phase_ = Phase::IDLE;
  if (testStroke_) {   // a teach-mode test stroke: no dart, nothing counted, no cooldown
    testStroke_ = false;
    return;
  }
  if (ammo_ > 0) ammo_--;
  shotsTotal_++;
  recentShot_ = true;
  completedMs_ = nowMs;
  refreshCooldown(nowMs);
}

void Trigger::update(uint32_t nowMs, const Input& in) {
  // 1. A stroke runs only while armed (a test stroke only in adjust mode and never armed):
  //    disarm, link loss, power off, a fault or leaving adjust mode mid-stroke means rest now
  //    and no decrement, even if the caller forgot abort(). A teach-mode angle does not
  //    outlive adjust mode either.
  if (strokeActive() && (testStroke_ ? !teachAllowed(in) : !in.armed)) abort();
  if (teachMoved_ && !teachAllowed(in)) abort();

  // 2. Which pull is this? It counts only if it started while armed and armed has held since.
  if (!in.armed) pullArmed_ = false;
  if (in.pulled && !prevPulled_) {
    pullArmed_ = in.armed;
    pullUsed_ = false;
  }
  if (!in.pulled) {
    pullArmed_ = false;
    pullUsed_ = false;
  }
  prevPulled_ = in.pulled;

  // 3. Advance the running stroke (sweep, hold, sweep back, hold, complete).
  advance(nowMs);

  // 4. The cooldown after the last completed stroke (full auto: none, so it is over in the same
  //    update() that completed the stroke).
  refreshCooldown(nowMs);

  // 5. Start the next stroke: full auto as long as the pull counts (back-to-back, no gap,
  //    because step 3 may have just completed the previous one); semi once per pull, after the
  //    cooldown (a counting pull still held when it ends fires then).
  const bool full = fireMode_ == DS_FIRE_MODE_FULL;
  if (in.pulled && pullArmed_ && (full || !pullUsed_) && gate(in) == Gate::OPEN)
    startStroke(nowMs, full && cfg_.rumbleFullAutoContinuous);

  // 6. Indications.
  if (ledHold_ && elapsed(nowMs, ledHoldStartMs_, LED_FIRE_FLASH_MIN_MS)) ledHold_ = false;
}

// REST at once, with no sweep, whatever phase the stroke is in: safety first, the pusher leaves
// the dart path at the servo's full speed. Nothing is counted and no cooldown starts.
void Trigger::abort() {
  if (strokeActive() && !testStroke_) {   // a test stroke never started a rumble
    rumble_ = Rumble();   // all zero: stop the stroke's rumble
    rumblePending_ = true;
  }
  phase_ = Phase::IDLE;
  testStroke_ = false;
  teachMoved_ = false;
  ledHold_ = false;
  pullArmed_ = false;   // the trigger must be released and pulled again
  command(restDeg_);
}

bool Trigger::fireOnce(uint32_t nowMs, const Input& in) {
  refreshCooldown(nowMs);   // the cooldown as of now, for this gate() and the caller's
  if (gate(in) != Gate::OPEN) return false;
  startStroke(nowMs, false);
  return true;
}

void Trigger::setAngles(float restDeg, float fireDeg) {
  restDeg_ = restDeg;
  fireDeg_ = fireDeg;
}

bool Trigger::teachMove(float deg, const Input& in) {
  if (!teachAllowed(in) || strokeActive()) return false;
  command(deg);
  teachMoved_ = true;
  return true;
}

bool Trigger::testStroke(uint32_t nowMs, const Input& in) {
  if (!teachAllowed(in) || strokeActive() || !servoPowered()) return false;
  testStroke_ = true;
  teachMoved_ = false;   // the stroke ends at rest by itself (or abort() puts it there)
  beginStroke(nowMs);    // from the teach angle the servo is at, the same sweeps and holds
  return true;
}

void Trigger::setFireMode(uint8_t mode) {
  if (mode != DS_FIRE_MODE_SEMI && mode != DS_FIRE_MODE_FULL) return;
  if (mode == fireMode_) return;
  fireMode_ = mode;
  pullArmed_ = false;   // a held trigger does not carry over into the new mode
}

void Trigger::toggleFireMode() {
  setFireMode(fireMode_ == DS_FIRE_MODE_SEMI ? (uint8_t)DS_FIRE_MODE_FULL : (uint8_t)DS_FIRE_MODE_SEMI);
}

void Trigger::setServoPower(bool on) {
  if (!on && strokeActive()) abort();
  if (on) hal::servoWriteDeg(physicalDeg(servoDeg_));
  hal::servoEnable(on);
}

bool Trigger::servoPowered() const { return hal::servoEnabled(); }

bool Trigger::firingLedActive(uint32_t nowMs) const {
  return strokeActive() || (ledHold_ && !elapsed(nowMs, ledHoldStartMs_, LED_FIRE_FLASH_MIN_MS));
}

bool Trigger::fireReady(uint32_t nowMs, const Input& in) const {
  Input pull = in;
  pull.pulled = true;   // a fresh pull, now; it counts only while armed, which gate() checks
  return gate(pull) == Gate::OPEN && !firingLedActive(nowMs);
}

bool Trigger::takeRumbleRequest(Rumble& out) {
  if (!rumblePending_) return false;
  out = rumble_;
  rumblePending_ = false;
  return true;
}

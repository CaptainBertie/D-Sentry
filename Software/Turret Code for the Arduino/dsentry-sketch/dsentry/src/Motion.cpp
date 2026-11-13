// Motion.cpp -- see Motion.h for the planner, the position profile and the exceptions to the
// accel limit.
//
// SPDX-License-Identifier: MIT
#include "Motion.h"

#include <math.h>

#include "StallDetector.h"
#include "hal.h"

// The HAL caps every axis at STEP_MAX_RATE_HZ; full stick must not ask for more.
static_assert(PAN_MAX_DEG_PER_S * PAN_STEPS_PER_DEG <= STEP_MAX_RATE_HZ,
              "PAN_MAX_DEG_PER_S needs more steps/s than STEP_MAX_RATE_HZ allows");
static_assert(TILT_MAX_DEG_PER_S * TILT_STEPS_PER_DEG <= STEP_MAX_RATE_HZ,
              "TILT_MAX_DEG_PER_S needs more steps/s than STEP_MAX_RATE_HZ allows");
static_assert(CONTROL_LOOP_HZ > 0 && CONTROL_LOOP_HZ <= US_PER_S, "CONTROL_LOOP_HZ out of range");
static_assert(MOTION_CREEP_STEPS_PER_S > STEP_MIN_RATE_HZ,
              "MOTION_CREEP_STEPS_PER_S must be above STEP_MIN_RATE_HZ or a move stalls short");
static_assert(MOTION_CREEP_STEPS_PER_S < CONTROL_LOOP_HZ,
              "MOTION_CREEP_STEPS_PER_S must be below CONTROL_LOOP_HZ (one step per tick at most)");
static_assert(MOTION_BRAKE_MARGIN_STEPS >= 0, "MOTION_BRAKE_MARGIN_STEPS must not be negative");
static_assert(MOTION_BRAKE_PLAN_FRACTION > 0.0f && MOTION_BRAKE_PLAN_FRACTION <= 1.0f,
              "MOTION_BRAKE_PLAN_FRACTION must be in (0, 1]");
static_assert(PAN_ACCEL_DEG_PER_S2 > 0.0f && TILT_ACCEL_DEG_PER_S2 > 0.0f, "accel must be > 0");
static_assert(SOFT_LIMIT_SLOWDOWN_DEG > 0.0f, "SOFT_LIMIT_SLOWDOWN_DEG must be > 0");
static_assert(TILT_CAL_SPEED_DEG_PER_S > 0.0f && HOME_SPEED_DEG_PER_S > 0.0f, "speeds must be > 0");

namespace {

constexpr uint32_t kTickUs = US_PER_S / CONTROL_LOOP_HZ;
constexpr float kTickS = (float)kTickUs / (float)US_PER_S;

float stepsPerDeg(Axis a) { return a == AXIS_PAN ? PAN_STEPS_PER_DEG : TILT_STEPS_PER_DEG; }
bool inverted(Axis a) { return a == AXIS_PAN ? PAN_DIR_INVERT : TILT_DIR_INVERT; }
float maxSpeed(Axis a) { return a == AXIS_PAN ? PAN_MAX_DEG_PER_S : TILT_MAX_DEG_PER_S; }
float accelOf(Axis a) { return a == AXIS_PAN ? PAN_ACCEL_DEG_PER_S2 : TILT_ACCEL_DEG_PER_S2; }
float minOf(float x, float y) { return x < y ? x : y; }
float cruiseSpeed(Axis a) { return minOf(HOME_SPEED_DEG_PER_S, maxSpeed(a)); }
float calSpeed() { return minOf(TILT_CAL_SPEED_DEG_PER_S, TILT_MAX_DEG_PER_S); }

// NaN and +-inf fail (x - x is NaN for both); no <math.h> isfinite, whose macro / function
// form differs between newlib and libc++.
bool finite(float x) { return x - x == 0.0f; }

int32_t roundSteps(float s) { return (int32_t)(s < 0.0f ? s - MATH_HALF : s + MATH_HALF); }
int32_t turretStepsFor(Axis a, float deg) { return roundSteps(deg * stepsPerDeg(a)); }

// The fastest speed (deg/s) that stops within `d` degrees when the planner holds it for
// `holdS` seconds and then removes `dvTick` deg/s per tick, each new speed again held for
// `holdS`. Distance covered from v = n * dvTick: holdS * dvTick * n (n + 1) / 2
//   = holdS * (v^2 + v * dvTick) / (2 * dvTick)  <=  d
//   =>  v = sqrt((dvTick / 2)^2 + 2 * (dvTick / holdS) * d) - dvTick / 2.
// With holdS = T it is the discrete form of sqrt(2 a d) for a = dvTick / T; with the measured
// tick spacing (holdS > T when the loop lands late) it brakes for the deceleration the
// limiter actually delivers, so a slow loop never makes the axis run past its profile.
float capFor(float d, float dvTick, float holdS) {
  if (!(d > 0.0f)) return 0.0f;
  const float half = MATH_HALF * dvTick;
  return sqrtf(half * half + KINEMATICS_2AD_FACTOR * (dvTick / holdS) * d) - half;
}

}  // namespace

// ---- conversions -------------------------------------------------------------------------------
int32_t Motion::degToStepsWith(float deg, float stepsPerDegree, bool invert) {
  const float s = deg * stepsPerDegree;
  return roundSteps(invert ? -s : s);
}

float Motion::stepsToDegWith(int32_t steps, float stepsPerDegree, bool invert) {
  const float deg = (float)steps / stepsPerDegree;
  return invert ? -deg : deg;
}

int32_t Motion::degToSteps(Axis axis, float deg) {
  return degToStepsWith(deg, stepsPerDeg(axis), inverted(axis));
}

float Motion::stepsToDeg(Axis axis, int32_t steps) {
  return stepsToDegWith(steps, stepsPerDeg(axis), inverted(axis));
}

float Motion::brakeCap(float distDeg, float accel) { return capFor(distDeg, accel * kTickS, kTickS); }

float Motion::brakeAccel(Axis axis) {
  const float vmax = maxSpeed(axis);
  return minOf(accelOf(axis), vmax * vmax / (KINEMATICS_2AD_FACTOR * SOFT_LIMIT_SLOWDOWN_DEG));
}

// ---- lifecycle ---------------------------------------------------------------------------------
void Motion::begin(StallDetector& stall) {
  stall_ = &stall;
  const uint32_t nowMs = hal::nowMs();
  calStatus_ = CalStatus::IDLE;   // first, so haltNow() below has no calibration to abort
  calPhase_ = CalPhase::NONE;
  calFailure_ = CalFailure::NONE;
  calMode_ = TILT_CAL_MODE;
  calStartMs_ = calPhaseMs_ = nowMs;
  homing_ = false;
  for (uint8_t i = 0; i < AXIS_COUNT; i++) {
    const Axis a = (Axis)i;
    mode_[a] = Mode::STOP;
    velDeg_[a] = 0.0f;
    targetVelDeg_[a] = 0.0f;
    targetSteps_[a] = 0;
    cruiseDeg_[a] = cruiseSpeed(a);
    lastDir_[a] = 0;
    homeDeg_[a] = 0.0f;
    axisStartMs_[a] = nowMs;
    hal::stepSetRate(a, 0.0f);
    hal::stepSetPosition(a, 0);
  }
  limMin_[AXIS_PAN] = PAN_MIN_DEG;
  limMax_[AXIS_PAN] = PAN_MAX_DEG;
  limMin_[AXIS_TILT] = 0.0f;
  limMax_[AXIS_TILT] = TILT_RANGE_DEG;
  panLimitsOn_ = PAN_LIMITS_ENABLED;
  applyPanBounds();
  applyTiltBounds();   // off until calibrated
  holdWhenIdle_ = STEPPER_HOLD_WHEN_IDLE;
  idleDisableMs_ = STEPPER_IDLE_DISABLE_MS;
  wasIdle_ = true;
  idleSinceMs_ = nowMs;
  moveStartMs_ = nowMs;
  ticks_ = 0;
  prevTickUs_ = hal::nowUs();
  holdS_ = kTickS;
  begun_ = true;
  enableDrivers(false);
}

void Motion::update(uint32_t nowMs) { update(nowMs, hal::nowUs()); }

void Motion::update(uint32_t nowMs, uint32_t nowUs) {
  if (!begun_) return;
  // A tick lands on the first update() at least one period after the previous tick, never
  // earlier: every speed change is then held for >= T, so a change of accel * T per tick is
  // never more than the accel limit. The measured spacing feeds the braking cap (capFor).
  const uint32_t elapsed = nowUs - prevTickUs_;
  if (elapsed < kTickUs) return;
  prevTickUs_ = nowUs;
  holdS_ = (float)elapsed / (float)US_PER_S;
  tick(nowMs);
}

// ---- the planner -------------------------------------------------------------------------------
void Motion::tick(uint32_t nowMs) {
  ticks_++;
  calStep(nowMs);
  for (uint8_t i = 0; i < AXIS_COUNT; i++) tickAxis((Axis)i, nowMs);

  // Idle driver policy: with STEPPER_HOLD_WHEN_IDLE false, release EN after
  // STEPPER_IDLE_DISABLE_MS without motion (the next commanded move enables it again).
  bool idle = movingAxes() == 0;
  for (uint8_t i = 0; i < AXIS_COUNT; i++)
    if (positional((Axis)i) || mode_[i] == Mode::CALIBRATE) idle = false;
  if (idle && !wasIdle_) idleSinceMs_ = nowMs;
  wasIdle_ = idle;
  if (idle && !holdWhenIdle_ && hal::steppersEnabled() &&
      (uint32_t)(nowMs - idleSinceMs_) >= idleDisableMs_)
    enableDrivers(false);
}

void Motion::tickAxis(Axis a, uint32_t nowMs) {
  const float spd = stepsPerDeg(a);
  const float v0 = velDeg_[a];
  const int32_t pos = turretSteps(a);
  const float posDeg = (float)pos / spd;
  const float margin = (float)MOTION_BRAKE_MARGIN_STEPS / spd;
  const float dvTick = accelOf(a) * kTickS;              // the limiter's budget per tick
  const float dvPlan = dvTick * MOTION_BRAKE_PLAN_FRACTION;  // what braking curves assume

  // 1. What this mode wants.
  float vt = 0.0f;
  int32_t rem = 0;
  const bool seek = positional(a);
  if (seek) {
    rem = targetSteps_[a] - pos;
    if (rem != 0) {
      const float dist = (float)(rem < 0 ? -rem : rem) / spd - margin;
      const float v = minOf(cruiseDeg_[a], capFor(dist, dvPlan, holdS_));
      vt = rem > 0 ? v : -v;
    }
  } else if (mode_[a] == Mode::VELOCITY ||
             (mode_[a] == Mode::CALIBRATE && calPhase_ == CalPhase::SENSE_DOWN)) {
    vt = targetVelDeg_[a];
  }

  // 2. Soft limits: brake at a_brake so the stop lands MOTION_BRAKE_MARGIN_STEPS inside.
  if (limitsActive(a)) {
    const float dvBrake = brakeAccel(a) * kTickS * MOTION_BRAKE_PLAN_FRACTION;
    const float up = capFor(limMax_[a] - posDeg - margin, dvBrake, holdS_);
    const float down = capFor(posDeg - limMin_[a] - margin, dvBrake, holdS_);
    if (vt > up) vt = up;
    if (vt < -down) vt = -down;
  }

  // 3. Accel limiter, 4. speed limit.
  float v1 = vt;
  if (v1 > v0 + dvTick) v1 = v0 + dvTick;
  if (v1 < v0 - dvTick) v1 = v0 - dvTick;
  const float vmax = maxSpeed(a);
  if (v1 > vmax) v1 = vmax;
  if (v1 < -vmax) v1 = -vmax;

  // Final approach: never slower than the creep rate while steps remain (the HAL would stop
  // the axis below STEP_MIN_RATE_HZ and the move would stall short), unless the axis is still
  // moving away from the target (then it first stops, below).
  if (seek && rem != 0) {
    const float dir = rem > 0 ? 1.0f : -1.0f;
    const float creep = MOTION_CREEP_STEPS_PER_S / spd;
    if (v0 * dir >= 0.0f && v1 * dir < creep) v1 = dir * creep;
  }

  // 5. A reversal passes through one stopped tick.
  if (v0 * v1 < 0.0f) v1 = 0.0f;

  // 6. The HAL's floor and ceiling, applied here so velocityDeg() is what actually runs. Below
  // the floor the axis stops, but only if stopping from v0 fits this tick's accel budget;
  // otherwise it runs one more tick at the creep rate (above the floor, below v0) and stops
  // from there, so the floor never turns a deceleration into a jump bigger than accel * T.
  float rate = v1 * spd;
  if (inverted(a)) rate = -rate;
  const float mag = rate < 0.0f ? -rate : rate;
  if (!(mag >= STEP_MIN_RATE_HZ)) {
    if ((v0 < 0.0f ? -v0 : v0) <= dvTick) {
      rate = 0.0f;
      v1 = 0.0f;
    } else {
      v1 = v1 < 0.0f ? -MOTION_CREEP_STEPS_PER_S / spd : MOTION_CREEP_STEPS_PER_S / spd;
      rate = inverted(a) ? -v1 * spd : v1 * spd;
    }
  } else if (mag > STEP_MAX_RATE_HZ) {
    rate = rate < 0.0f ? -STEP_MAX_RATE_HZ : STEP_MAX_RATE_HZ;
    v1 = v1 < 0.0f ? -STEP_MAX_RATE_HZ / spd : STEP_MAX_RATE_HZ / spd;
  }

  // 7. Command it.
  if (v1 != 0.0f && !hal::steppersEnabled()) enableDrivers(true);   // before the first step
  hal::stepSetRate(a, rate);
  if (v0 == 0.0f && v1 != 0.0f) {
    axisStartMs_[a] = nowMs;
    moveStartMs_ = nowMs;
  }
  if (v1 != 0.0f) lastDir_[a] = v1 > 0.0f ? 1 : -1;
  velDeg_[a] = v1;
  if (seek && rem == 0 && v1 == 0.0f) arrived(a, nowMs);
}

void Motion::arrived(Axis a, uint32_t nowMs) {
  switch (mode_[a]) {
    case Mode::POSITION:
      mode_[a] = Mode::STOP;
      if (!positionMoveActive()) homing_ = false;
      break;
    case Mode::BACKOFF:
      mode_[a] = Mode::STOP;
      break;
    case Mode::CALIBRATE:
      if (calPhase_ == CalPhase::RAM_DOWN) {
        calPhase_ = CalPhase::SETTLE;   // the counter is past the panel: the barrel is on it
        calPhaseMs_ = nowMs;
      } else if (calPhase_ == CalPhase::LIFT) {
        hal::stepSetPosition(AXIS_TILT, 0);   // TILT_CAL_BACKOFF_DEG above the stop = tilt 0
        endCalibration(CalStatus::DONE);
      }
      break;
    default:
      break;
  }
}

// Calibration phases that run on time rather than on arrival.
void Motion::calStep(uint32_t nowMs) {
  if (calStatus_ != CalStatus::RUNNING) return;
  if (calPhase_ == CalPhase::SENSE_DOWN) {
    if ((uint32_t)(nowMs - calStartMs_) >= TILT_CAL_TIMEOUT_MS) {
      calFailure_ = CalFailure::TIMEOUT;
      endCalibration(CalStatus::FAILED);
    }
  } else if (calPhase_ == CalPhase::SETTLE) {
    if ((uint32_t)(nowMs - calPhaseMs_) >= TILT_CAL_SETTLE_MS) {
      hal::stepSetPosition(AXIS_TILT, 0);   // this is the hard stop
      startMove(AXIS_TILT, Mode::CALIBRATE, turretStepsFor(AXIS_TILT, TILT_CAL_BACKOFF_DEG),
                calSpeed());
      calPhase_ = CalPhase::LIFT;
      calPhaseMs_ = nowMs;
    }
  }
}

// ---- commands ----------------------------------------------------------------------------------
void Motion::setVelocity(float panFrac, float tiltFrac) {
  const float frac[AXIS_COUNT] = {panFrac, tiltFrac};
  homing_ = false;
  for (uint8_t i = 0; i < AXIS_COUNT; i++) {
    const Axis a = (Axis)i;
    if (mode_[a] == Mode::CALIBRATE || mode_[a] == Mode::BACKOFF) continue;
    float f = frac[i];
    if (!(f == f)) f = 0.0f;   // NaN
    if (f > 1.0f) f = 1.0f;
    if (f < -1.0f) f = -1.0f;
    mode_[a] = Mode::VELOCITY;
    targetVelDeg_[a] = f * maxSpeed(a);
  }
}

bool Motion::moveTo(Axis axis, float deg) {
  if (axis >= AXIS_COUNT) return false;
  if (mode_[axis] == Mode::CALIBRATE || mode_[axis] == Mode::BACKOFF) return false;
  if (!finite(deg)) return false;
  homing_ = false;
  startMove(axis, Mode::POSITION, turretStepsFor(axis, clampToLimits(axis, deg)), cruiseSpeed(axis));
  return true;
}

bool Motion::moveTo(float panDeg, float tiltDeg) {
  const bool pan = moveTo(AXIS_PAN, panDeg);
  const bool tilt = moveTo(AXIS_TILT, tiltDeg);
  return pan && tilt;
}

bool Motion::goHome() {
  // Pan has no end stops when its limits are off, so the stored home and every whole turn away
  // from it are the same place: take the nearest one (owner: nothing crosses the pan joint).
  float panTarget = homeDeg_[AXIS_PAN];
  if (PAN_HOME_SHORTEST_PATH && !panLimitsOn_) panTarget = nearestTurn(positionDeg(AXIS_PAN), panTarget);
  const bool ok = moveTo(panTarget, homeDeg_[AXIS_TILT]);
  homing_ = positionMoveActive();
  return ok;
}

float Motion::nearestTurn(float fromDeg, float toDeg) {
  float d = fmodf(toDeg - fromDeg, DEG_PER_REV);   // (-DEG_PER_REV, DEG_PER_REV)
  if (d < 0.0f) d += DEG_PER_REV;                  // [0, DEG_PER_REV)
  if (d > DEG_PER_REV - d) d -= DEG_PER_REV;       // more than half a turn: go the other way
  return fromDeg + d;
}

void Motion::setHomeHere() {
  for (uint8_t i = 0; i < AXIS_COUNT; i++) homeDeg_[i] = positionDeg((Axis)i);
}

void Motion::setHome(float panDeg, float tiltDeg) {
  if (finite(panDeg)) homeDeg_[AXIS_PAN] = panDeg;
  if (finite(tiltDeg)) homeDeg_[AXIS_TILT] = tiltDeg;
}

void Motion::stop() {
  homing_ = false;
  for (uint8_t i = 0; i < AXIS_COUNT; i++) {
    if (mode_[i] == Mode::CALIBRATE) continue;
    mode_[i] = Mode::STOP;
    targetVelDeg_[i] = 0.0f;
  }
}

void Motion::haltNow() {
  if (calStatus_ == CalStatus::RUNNING) endCalibration(CalStatus::IDLE);
  for (uint8_t i = 0; i < AXIS_COUNT; i++) {
    stopAxisNow((Axis)i);
    mode_[i] = Mode::STOP;
    targetVelDeg_[i] = 0.0f;
  }
  homing_ = false;
}

void Motion::rebootPosition() {
  haltNow();
  hal::stepSetPosition(AXIS_PAN, 0);
  homeDeg_[AXIS_PAN] = 0.0f;
  homeDeg_[AXIS_TILT] = 0.0f;
  calStatus_ = CalStatus::IDLE;
  applyPanBounds();
}

void Motion::backOff(uint8_t axesMask, float deg) {
  homing_ = false;
  for (uint8_t i = 0; i < AXIS_COUNT; i++) {
    const Axis a = (Axis)i;
    if (mode_[a] == Mode::CALIBRATE) continue;
    targetVelDeg_[a] = 0.0f;
    if (!(axesMask & axisBit(a))) {
      mode_[a] = Mode::STOP;   // not stalled: decelerate
      continue;
    }
    stopAxisNow(a);
    if (lastDir_[a] == 0 || !(deg > 0.0f) || !finite(deg)) {
      mode_[a] = Mode::STOP;
      continue;
    }
    const float target = positionDeg(a) - (float)lastDir_[a] * deg;
    startMove(a, Mode::BACKOFF, turretStepsFor(a, clampToLimits(a, target)), cruiseSpeed(a));
  }
}

void Motion::startCalibration(uint32_t nowMs, uint8_t calMode) {
  calMode_ = calMode == CAL_STALL_SENSE ? (uint8_t)CAL_STALL_SENSE : (uint8_t)CAL_RAM;
  calStartMs_ = calPhaseMs_ = nowMs;
  calFailure_ = CalFailure::NONE;
  if (stall_) stall_->setForced(false);
  if (!hal::beginReport().tiltTimer) {   // the tilt cannot move: the sequence could never end
    calFailure_ = CalFailure::NO_TIMER;
    calStatus_ = CalStatus::FAILED;
    calPhase_ = CalPhase::NONE;
    return;
  }
  calStatus_ = CalStatus::RUNNING;   // tilt soft limits off (limitsActive) ...
  applyTiltBounds();                 // ... and the interrupt's bounds too
  if (homing_ && mode_[AXIS_TILT] == Mode::POSITION) homing_ = false;
  enableDrivers(true);
  targetVelDeg_[AXIS_TILT] = 0.0f;
  if (calMode_ == CAL_STALL_SENSE) {
    mode_[AXIS_TILT] = Mode::CALIBRATE;
    calPhase_ = CalPhase::SENSE_DOWN;
    targetVelDeg_[AXIS_TILT] = -calSpeed();   // down until the stall event or the timeout
    if (stall_) stall_->setForced(true);
  } else {
    startMove(AXIS_TILT, Mode::CALIBRATE,
              turretSteps(AXIS_TILT) - turretStepsFor(AXIS_TILT, TILT_CAL_MAX_TRAVEL_DEG), calSpeed());
    calPhase_ = CalPhase::RAM_DOWN;
  }
}

void Motion::abortCalibration() {
  if (calStatus_ == CalStatus::RUNNING) endCalibration(CalStatus::IDLE);
}

bool Motion::notifyCalibrationStall(uint32_t nowMs) {
  if (!calibrationWantsStall()) return false;
  stopAxisNow(AXIS_TILT);   // the panel has already stopped the barrel
  if (stall_) stall_->setForced(false);
  calPhase_ = CalPhase::SETTLE;
  calPhaseMs_ = nowMs;
  return true;
}

void Motion::enableDrivers(bool on) { hal::stepperEnable(on); }

void Motion::setPanLimits(bool enabled, float minDeg, float maxDeg) {
  if (!finite(minDeg) || !finite(maxDeg) || minDeg > maxDeg) return;
  panLimitsOn_ = enabled;
  limMin_[AXIS_PAN] = minDeg;
  limMax_[AXIS_PAN] = maxDeg;
  applyPanBounds();
  if (enabled && positional(AXIS_PAN)) {   // a running move's target must stay inside
    const float t = clampToLimits(AXIS_PAN, (float)targetSteps_[AXIS_PAN] / stepsPerDeg(AXIS_PAN));
    targetSteps_[AXIS_PAN] = turretStepsFor(AXIS_PAN, t);
  }
}

void Motion::setIdlePolicy(bool holdWhenIdle, uint32_t idleDisableMs) {
  holdWhenIdle_ = holdWhenIdle;
  idleDisableMs_ = idleDisableMs;
}

// ---- helpers -----------------------------------------------------------------------------------
void Motion::startMove(Axis a, Mode m, int32_t targetTurretSteps, float cruiseDeg) {
  mode_[a] = m;
  targetSteps_[a] = targetTurretSteps;
  cruiseDeg_[a] = cruiseDeg;
  targetVelDeg_[a] = 0.0f;
}

void Motion::stopAxisNow(Axis a) {
  hal::stepSetRate(a, 0.0f);
  velDeg_[a] = 0.0f;
}

void Motion::endCalibration(CalStatus status) {
  calStatus_ = status;
  calPhase_ = CalPhase::NONE;
  if (stall_) stall_->setForced(false);
  if (mode_[AXIS_TILT] == Mode::CALIBRATE) {
    mode_[AXIS_TILT] = Mode::STOP;   // decelerates (at rest already on DONE)
    targetVelDeg_[AXIS_TILT] = 0.0f;
  }
  applyTiltBounds();   // on only when DONE
}

void Motion::applyTiltBounds() {
  if (!calibrated()) {
    hal::stepSetBounds(AXIS_TILT, false, 0, 0);
    return;
  }
  const int32_t lo = degToSteps(AXIS_TILT, limMin_[AXIS_TILT]);
  const int32_t hi = degToSteps(AXIS_TILT, limMax_[AXIS_TILT]);
  hal::stepSetBounds(AXIS_TILT, true, lo < hi ? lo : hi, lo < hi ? hi : lo);
}

void Motion::applyPanBounds() {
  if (!panLimitsOn_) {
    hal::stepSetBounds(AXIS_PAN, false, 0, 0);
    return;
  }
  const int32_t lo = degToSteps(AXIS_PAN, limMin_[AXIS_PAN]);
  const int32_t hi = degToSteps(AXIS_PAN, limMax_[AXIS_PAN]);
  hal::stepSetBounds(AXIS_PAN, true, lo < hi ? lo : hi, lo < hi ? hi : lo);
}

int32_t Motion::turretSteps(Axis a) const {
  const int32_t s = hal::stepPosition(a);
  return inverted(a) ? -s : s;
}

bool Motion::positional(Axis a) const {
  return mode_[a] == Mode::POSITION || mode_[a] == Mode::BACKOFF ||
         (mode_[a] == Mode::CALIBRATE &&
          (calPhase_ == CalPhase::RAM_DOWN || calPhase_ == CalPhase::LIFT));
}

float Motion::clampToLimits(Axis a, float deg) const {
  if (!limitsActive(a)) return deg;
  if (deg < limMin_[a]) return limMin_[a];
  if (deg > limMax_[a]) return limMax_[a];
  return deg;
}

// ---- state -------------------------------------------------------------------------------------
bool Motion::driversEnabled() const { return hal::steppersEnabled(); }

bool Motion::homing() const { return homing_ && positionMoveActive(); }

bool Motion::positionMoveActive() const {
  for (uint8_t i = 0; i < AXIS_COUNT; i++)
    if (mode_[i] == Mode::POSITION) return true;
  return false;
}

bool Motion::backingOff() const {
  for (uint8_t i = 0; i < AXIS_COUNT; i++)
    if (mode_[i] == Mode::BACKOFF) return true;
  return false;
}

float Motion::positionDeg(Axis axis) const { return stepsToDeg(axis, hal::stepPosition(axis)); }
int32_t Motion::steps(Axis axis) const { return hal::stepPosition(axis); }

float Motion::targetDeg(Axis axis) const {
  return (float)targetSteps_[axis] / stepsPerDeg(axis);
}

bool Motion::limitsActive(Axis axis) const {
  return axis == AXIS_TILT ? calibrated() : panLimitsOn_;
}

uint8_t Motion::movingAxes() const {
  uint8_t mask = 0;
  for (uint8_t i = 0; i < AXIS_COUNT; i++)
    if (velDeg_[i] != 0.0f) mask |= axisBit((Axis)i);
  return mask;
}

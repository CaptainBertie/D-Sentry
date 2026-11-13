// Motion.h -- both steppers: velocity and position modes, soft limits, calibration, home
// (docs/architecture.md 5 and 6, spec 3.3 and 3.5).
//
// Coordinates: degrees in the turret frame. Pan + = counter-clockwise seen from above; tilt
// + = barrel up, 0 = calibrated bottom stop + TILT_CAL_BACKOFF_DEG. Motion converts to motor
// steps with PAN_STEPS_PER_DEG / TILT_STEPS_PER_DEG and *_DIR_INVERT, and is the only module
// that calls hal::stepSetRate / stepSetBounds / stepSetPosition / stepperEnable.
//
// Planner, one tick every 1/CONTROL_LOOP_HZ s from update(), per axis:
//   1. target velocity from the axis mode (stick, position profile, calibration phase, 0);
//   2. soft-limit cap  v <= vcap(d, a_brake), a_brake = min(accel,
//      vmax^2 / (KINEMATICS_2AD_FACTOR * SOFT_LIMIT_SLOWDOWN_DEG)), d = distance to the limit
//      less MOTION_BRAKE_MARGIN_STEPS. vcap is the discrete form of sqrt(2 a d): the fastest
//      speed that, held for one tick of length h and then cut by a*T every tick, stops within d:
//        vcap = sqrt((a T / 2)^2 + 2 (a T / h) d) - a T / 2,   T = 1 / CONTROL_LOOP_HZ,
//      h = the measured spacing of the last two ticks (>= T; = T on time), so a late loop
//      brakes for the deceleration it can really deliver. The curve is planned for
//      MOTION_BRAKE_PLAN_FRACTION of a_brake: the limiter keeps the rest of the accel budget to
//      pull the speed back onto the curve after loop jitter or a late step (with no slack an
//      axis that falls behind its curve only falls further behind). Full speed therefore starts
//      braking at least SOFT_LIMIT_SLOWDOWN_DEG before a limit and never needs more than the
//      accel limit;
//   3. accel limiter  |v - v_prev| <= accel * T. Ticks are never closer than T (see update()),
//      so the change between two ticks never exceeds the accel limit and one jump never
//      exceeds a*T (CODE/ lesson: never start a stepper at speed);
//   4. |v| <= *_MAX_DEG_PER_S;
//   5. a sign change always passes through one stopped tick (the HAL applies a direction
//      change only after one more step in the old direction; stopping first avoids it);
//   6. below STEP_MIN_RATE_HZ the HAL stops the axis, so v is snapped to 0 there too (or, when
//      stopping from v_prev would exceed a*T, held one tick at MOTION_CREEP_STEPS_PER_S first):
//      what velocityDeg() reports is exactly what the step generator runs;
//   7. hal::stepSetRate (never above STEP_MAX_RATE_HZ).
//
// Position moves (go-home, shell pan/tilt, the stall back-off, the calibration moves) are
// trapezoids: cruise speed, accel ramp, the same vcap braking toward the target step with the
// axis accel (times MOTION_BRAKE_PLAN_FRACTION). The last MOTION_BRAKE_MARGIN_STEPS steps run
// at MOTION_CREEP_STEPS_PER_S (above STEP_MIN_RATE_HZ, so the axis cannot stall short; below
// CONTROL_LOOP_HZ, so at most one step lands between two ticks and the move stops exactly on
// its target step).
//
// Hard bounds in the step interrupt (hal::stepSetBounds) are the backstop: tilt
// 0..TILT_RANGE_DEG once calibrated; pan PAN_MIN_DEG..PAN_MAX_DEG if PAN_LIMITS_ENABLED. The
// planner brakes before them, so in normal running the interrupt never refuses a step.
//
// Exceptions to the accel limit, all deliberate: haltNow() (power off), backOff() (a stalled
// axis has lost sync with its steps, so its pulses stop at once), the CAL_STALL_SENSE stop (the
// panel has already stopped the barrel), and the HAL's own floor: an axis cannot run slower than
// STEP_MIN_RATE_HZ, so starting or stopping from that rate is a jump of STEP_MIN_RATE_HZ (and of
// MOTION_CREEP_STEPS_PER_S at the end of a position move). Both are below accel / CONTROL_LOOP_HZ
// at the default config; tests/unit/test_motion.cpp checks it.
//
// Ownership (architecture.md rule 5 and the stage-2 decisions): TurretState is the only caller
// of the commands below and the only consumer of StallDetector::takeStall(). During a
// CAL_STALL_SENSE calibration TurretState forwards the stall event with
// notifyCalibrationStall(); Motion never reads takeStall() itself. Motion only switches the
// detector's forced flag on and off around that calibration phase.
//
// SPDX-License-Identifier: MIT
#pragma once
#include <stdint.h>
#include "Types.h"

class StallDetector;

class Motion {
 public:
  enum class Mode : uint8_t {
    STOP,       // decelerating to rest at the accel limit, then idle
    VELOCITY,   // following setVelocity() (the stick)
    POSITION,   // trapezoid move to a target at HOME_SPEED_DEG_PER_S (go-home, shell pan/tilt)
    CALIBRATE,  // tilt only: the startup homing sequence (calPhase())
    BACKOFF     // stall back-off: a STALL_BACKOFF_DEG position move against the motion
  };
  enum class CalStatus : uint8_t {
    IDLE,       // not started, or aborted (power off)
    RUNNING,
    DONE,       // tilt zeroed, soft limits and hard bounds 0..TILT_RANGE_DEG active
    FAILED      // see calFailure()
  };
  enum class CalPhase : uint8_t {
    NONE,        // no calibration running
    RAM_DOWN,    // CAL_RAM: position move TILT_CAL_MAX_TRAVEL_DEG down, bounds off
    SENSE_DOWN,  // CAL_STALL_SENSE: down at TILT_CAL_SPEED_DEG_PER_S until the stall event
    SETTLE,      // at the stop, still, for TILT_CAL_SETTLE_MS; then the counter is zeroed
    LIFT         // TILT_CAL_BACKOFF_DEG up; on arrival the counter is zeroed again
  };
  enum class CalFailure : uint8_t {
    NONE,
    TIMEOUT,     // CAL_STALL_SENSE: no stall event within TILT_CAL_TIMEOUT_MS of the start
    NO_TIMER     // hal::begin() could not claim the tilt step timer: the axis cannot move
  };

  // Positions 0, drivers disabled, tilt uncalibrated (no tilt bounds), pan bounds per
  // PAN_LIMITS_ENABLED, idle policy from STEPPER_HOLD_WHEN_IDLE / STEPPER_IDLE_DISABLE_MS.
  void begin(StallDetector& stall);

  // Once per loop(). Runs one planner tick on the first call at least 1/CONTROL_LOOP_HZ s after
  // the previous tick (never earlier, never a burst after a long gap), so the tick rate is
  // CONTROL_LOOP_HZ less the loop's own granularity (a 200 us loop: about 2 % slower). Phase
  // timing (settle, timeouts) uses nowMs. The one-argument form reads hal::nowUs().
  void update(uint32_t nowMs, uint32_t nowUs);
  void update(uint32_t nowMs);

  // ---- commands (TurretState is the only caller; it has already checked permissions) ----
  // All of them take effect at the next planner tick unless they say "at once".

  // Stick: fractions of full speed, -1..1 (clamped; NaN = 0): pan + = counter-clockwise,
  // tilt + = up. Puts both axes in VELOCITY and cancels any POSITION move (and homing):
  // TurretState calls it while the stick is active or no position move is running, so a stick
  // nudge cancels go-home (spec 3.2). Axes in CALIBRATE or BACKOFF ignore it.
  void setVelocity(float panFrac, float tiltFrac);
  // POSITION move of one axis (the other unchanged) or both, cruising at
  // min(HOME_SPEED_DEG_PER_S, *_MAX_DEG_PER_S). The target is clamped to the active soft
  // limits. Returns false (nothing changes) for an axis in CALIBRATE or BACKOFF; the
  // two-axis form returns true only if both were accepted.
  bool moveTo(Axis axis, float deg);
  bool moveTo(float panDeg, float tiltDeg);
  bool goHome();                            // moveTo(home); homing() until both arrive. Pan goes
                                            // the short way (PAN_HOME_SHORTEST_PATH, limits off)
  void setHomeHere();                       // home = current position (RAM only)
  void setHome(float panDeg, float tiltDeg);  // explicit home (shell); RAM only
  // Decelerate both axes to rest at the accel limit, then STOP. Cancels VELOCITY, POSITION
  // and BACKOFF; a running calibration continues (abortCalibration() stops it).
  void stop();
  // At once: rates 0 on both axes, every mode STOP, homing cancelled, a running calibration
  // aborted (status IDLE, the stall detector's forced flag cleared). For power off; the drivers
  // stay as they are (enableDrivers(false) releases them).
  void haltNow();
  // A new boot (power toggle closed, spec 3.1 "restarts from BOOT"): haltNow(), then pan zero =
  // wherever the turret is now (spec 3.5: pan is not calibrated; while OFF the drivers were
  // released, so the old count means nothing), home back to the default (0, 0), and the tilt
  // calibration forgotten until CALIBRATING runs again.
  void rebootPosition();
  // Stall response (spec 3.6, architecture.md 6). For each axis in axesMask (normally
  // StallDetector::stallAxes()): its pulses stop at once, then a BACKOFF move of `deg` against
  // the direction it was last moving, cruising at min(HOME_SPEED_DEG_PER_S, max), clamped to
  // the soft limits. Every other axis decelerates to rest (STOP). Homing is cancelled. An
  // axis in CALIBRATE is left alone (stalls during calibration are TurretState's to route).
  void backOff(uint8_t axesMask, float deg);

  // Tilt homing (spec 3.5). calMode: CAL_RAM or CAL_STALL_SENSE (default TILT_CAL_MODE; any
  // other value runs CAL_RAM). Enables the drivers, turns the tilt bounds off and puts tilt in
  // CALIBRATE; pan is not touched. CAL_STALL_SENSE also forces the stall detector on until
  // the stall event arrives or the calibration ends.
  void startCalibration(uint32_t nowMs, uint8_t calMode = TILT_CAL_MODE);
  // A running calibration stops (tilt decelerates), status IDLE, detector no longer forced.
  void abortCalibration();
  // TurretState forwards StallDetector::takeStall() here while calibrationWantsStall(). Stops
  // the tilt at once and moves on to SETTLE. Returns false (and does nothing) otherwise.
  bool notifyCalibrationStall(uint32_t nowMs);
  bool calibrationWantsStall() const {
    return calStatus_ == CalStatus::RUNNING && calPhase_ == CalPhase::SENSE_DOWN;
  }

  // EN line (shared by both drivers). A planner tick that needs steps enables the drivers
  // itself, so after enableDrivers(false) the next commanded move turns them back on.
  void enableDrivers(bool on);

  // ---- bench / test overrides (config.h stays the default; TurretState never calls these) ----
  // Pan soft limits and hard bounds at run time (PAN_LIMITS_ENABLED and PAN_MIN/MAX_DEG at
  // begin()). Enable them only while pan is inside the range: the interrupt refuses every step
  // that would leave it, including steps back towards it.
  void setPanLimits(bool enabled, float minDeg, float maxDeg);
  // Idle driver policy (STEPPER_HOLD_WHEN_IDLE, STEPPER_IDLE_DISABLE_MS at begin()).
  void setIdlePolicy(bool holdWhenIdle, uint32_t idleDisableMs);

  // ---- state ----
  CalStatus calibrationStatus() const { return calStatus_; }
  CalPhase calPhase() const { return calPhase_; }
  CalFailure calFailure() const { return calFailure_; }
  uint8_t calMode() const { return calMode_; }   // mode of the current / last calibration
  bool calibrated() const { return calStatus_ == CalStatus::DONE; }
  Mode mode(Axis axis) const { return mode_[axis]; }
  bool homing() const;                      // a goHome() move is running
  bool positionMoveActive() const;          // any axis in POSITION (go-home or shell move)
  bool backingOff() const;                  // any axis in BACKOFF
  float positionDeg(Axis axis) const;       // from the step counter (what was commanded)
  float velocityDeg(Axis axis) const { return velDeg_[axis]; }   // in force now, deg/s
  float targetDeg(Axis axis) const;         // POSITION / BACKOFF / calibration move target
  float homeDeg(Axis axis) const { return homeDeg_[axis]; }
  // The axis' configured soft-limit range (tilt 0..TILT_RANGE_DEG, pan PAN_MIN..PAN_MAX or
  // the setPanLimits() values); limitsActive() says whether it is enforced right now (tilt:
  // only once calibrated).
  float limitMinDeg(Axis axis) const { return limMin_[axis]; }
  float limitMaxDeg(Axis axis) const { return limMax_[axis]; }
  bool limitsActive(Axis axis) const;
  bool isMoving() const { return movingAxes() != 0; }
  uint8_t movingAxes() const;               // axisBit() mask of axes with nonzero velocity
  // Last rest -> moving transition of any axis / of one axis (hal::nowMs()). StallDetector's
  // ignore window runs from moveStartMs(), so a second axis starting re-masks the kick.
  uint32_t moveStartMs() const { return moveStartMs_; }
  uint32_t moveStartMs(Axis axis) const { return axisStartMs_[axis]; }
  bool driversEnabled() const;
  int32_t steps(Axis axis) const;           // raw step counter (motor coordinates)
  uint32_t ticks() const { return ticks_; } // planner ticks since begin()

  // Pure conversions (unit tests, shell): turret degrees <-> motor steps, inversion applied.
  // degToSteps rounds to the nearest step (half away from zero).
  static int32_t degToSteps(Axis axis, float deg);
  static float stepsToDeg(Axis axis, int32_t steps);
  // The same with explicit parameters, so both signs of *_DIR_INVERT are testable.
  static int32_t degToStepsWith(float deg, float stepsPerDeg, bool invert);
  static float stepsToDegWith(int32_t steps, float stepsPerDeg, bool invert);
  // The planner's braking cap (deg/s) for `distDeg` left at deceleration `accel` (deg/s^2).
  static float brakeCap(float distDeg, float accel);
  // The angle that equals `toDeg` modulo a full turn and is nearest to `fromDeg`: at most half a
  // turn away (an exact half turn goes the positive way). Used for go-home on an unlimited pan.
  static float nearestTurn(float fromDeg, float toDeg);
  // Soft-limit braking deceleration of an axis: min(accel, vmax^2 / (2 * slowdown)).
  static float brakeAccel(Axis axis);

 private:
  void tick(uint32_t nowMs);
  void tickAxis(Axis a, uint32_t nowMs);
  void arrived(Axis a, uint32_t nowMs);
  void calStep(uint32_t nowMs);
  void startMove(Axis a, Mode m, int32_t targetTurretSteps, float cruiseDeg);
  void stopAxisNow(Axis a);
  void endCalibration(CalStatus status);
  void applyTiltBounds();
  void applyPanBounds();
  int32_t turretSteps(Axis a) const;        // step counter in the turret frame (+ = turret +)
  bool positional(Axis a) const;            // mode follows targetSteps_ (not a velocity)
  float clampToLimits(Axis a, float deg) const;

  StallDetector* stall_ = nullptr;
  bool begun_ = false;
  Mode mode_[AXIS_COUNT] = {Mode::STOP, Mode::STOP};
  float velDeg_[AXIS_COUNT] = {};        // in force, deg/s, turret frame
  float targetVelDeg_[AXIS_COUNT] = {};  // VELOCITY / SENSE_DOWN target, deg/s
  int32_t targetSteps_[AXIS_COUNT] = {}; // positional target, turret-frame steps
  float cruiseDeg_[AXIS_COUNT] = {};     // positional cruise speed, deg/s
  int8_t lastDir_[AXIS_COUNT] = {};      // sign of the last nonzero velocity (back-off)
  float homeDeg_[AXIS_COUNT] = {};
  float limMin_[AXIS_COUNT] = {};
  float limMax_[AXIS_COUNT] = {};
  bool panLimitsOn_ = false;
  bool homing_ = false;
  CalStatus calStatus_ = CalStatus::IDLE;
  CalPhase calPhase_ = CalPhase::NONE;
  CalFailure calFailure_ = CalFailure::NONE;
  uint8_t calMode_ = TILT_CAL_MODE;
  uint32_t calStartMs_ = 0;
  uint32_t calPhaseMs_ = 0;
  bool holdWhenIdle_ = STEPPER_HOLD_WHEN_IDLE;
  uint32_t idleDisableMs_ = STEPPER_IDLE_DISABLE_MS;
  bool wasIdle_ = true;
  uint32_t idleSinceMs_ = 0;
  uint32_t prevTickUs_ = 0;   // when the last tick ran (hal::nowUs())
  float holdS_ = 0.0f;        // spacing of the last two ticks, s (>= 1/CONTROL_LOOP_HZ)
  uint32_t ticks_ = 0;
  uint32_t moveStartMs_ = 0;
  uint32_t axisStartMs_[AXIS_COUNT] = {};
};

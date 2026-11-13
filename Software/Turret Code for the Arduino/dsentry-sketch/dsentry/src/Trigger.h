// Trigger.h -- the dart-pusher servo stroke, semi/full logic and the magazine (spec 3.4,
// docs/architecture.md 5 and 6).
//
// Stroke: IDLE -> SWEEP_OUT (the commanded angle glides from where the servo is, REST in normal
// use, to the FIRE angle at Config::sweepDegPerS = TRIGGER_SWEEP_DEG_PER_S, re-commanded every
// update()) -> HOLD_FIRE (exactly at FIRE for TRIGGER_FORWARD_MS) -> SWEEP_BACK (FIRE -> REST at
// the same rate) -> HOLD_REST (exactly at REST for TRIGGER_RETURN_MS) -> complete: ammo - 1
// (floor 0), shotsTotal + 1 -> IDLE. FIRE and REST are angles TRIGGER_FIRE_ANGLE /
// TRIGGER_REST_ANGLE unless TurretState set others (SERVO ADJUST's saved or adjusted angles);
// either sign of FIRE - REST works.
// A sweep lands exactly on its end angle at the first update() whose elapsed time covers the
// distance at the rate; each hold starts at that update(), so the servo always gets at least
// the configured hold. sweepDegPerS 0 = no sweep: each angle is commanded at once (the old
// two-step stroke, FORWARD then RETURN). A hobby servo has no position feedback: gliding
// slower than the servo's own top speed is what makes "the stroke completed" true.
// The ammo count changes in one place only, at completion. abort() (any phase) commands REST
// at once and never decrements or counts a shot.
//
// A stroke may start only when every gate is open (gate()): armed, no stroke running, the
// servo powered (setServoPower), ammo > 0 unless lockout is off (Config::lockoutWhenEmpty =
// FIRE_LOCKOUT_WHEN_EMPTY, off since 2026-11-03: the turret keeps firing at 0, the count stays
// 0), Input::flywheelReady unless Config::requiresFlywheel (FIRE_REQUIRES_FLYWHEEL) is false,
// and the cooldown over. Trigger applies FIRE_REQUIRES_FLYWHEEL itself, so TurretState passes
// Flywheel::ready() as it is. fireReady() asks the same gates about a fresh pull (the ARMED LED
// colour), so the cooldown makes ARMED yellow too.
// Cooldown: after a real stroke completes, no new stroke starts for cooldownMs() of the fire
// mode in force: semi Config::semiCooldownMs (TRIGGER_COOLDOWN_MS), full auto 0. Aborted strokes
// and test strokes start none.
//
// Firing rules (architecture.md 6):
//   - a pull counts only if it started while armed, and it stops counting the moment armed
//     goes false, the fire mode changes or abort() runs: arming (or re-arming) with the trigger
//     held never fires until it is released and pulled again;
//   - semi: one stroke per pull. A counting pull that is blocked (flywheel not spun up, empty
//     magazine, a stroke still running, the cooldown) fires once when the block clears, if
//     still held; one let go before then never fires. A trigger held across a stroke's end
//     does not re-fire;
//   - full: strokes back-to-back while held and permitted: the next stroke starts in the same
//     update() that completes the previous one. Releasing finishes the current stroke;
//   - armed false while a stroke runs (disarm, link loss, power off, fault): update() aborts
//     the stroke itself, so a missing abort() call can never leave the pusher forward;
//   - empty magazine with lockout: no stroke (TurretState's LEDs show ammo 0 either way).
// Each stroke start raises a rumble request (takeRumbleRequest): RUMBLE_STRONG_255 /
// RUMBLE_WEAK_255 for RUMBLE_DURATION_MS; in full auto with RUMBLE_FULL_AUTO_CONTINUOUS the
// duration is one whole stroke (strokeMs()) plus RUMBLE_DURATION_MS, so each stroke's request
// overlaps the next one's and the pad rumbles without a break while cycling, then for
// RUMBLE_DURATION_MS after the last stroke ends. abort() of a running stroke requests a stop
// (all zero).
// The shot LED hold (firingLedActive) is on from each stroke start for the whole stroke (both
// sweeps and holds) and at least LED_FIRE_FLASH_MIN_MS; abort() turns it off at once.
// fireReady() is false while it is on, so ARMED shows yellow for at least that long per shot (a
// test stroke holds it too).
//
// Stroke angles: TRIGGER_REST_ANGLE / TRIGGER_FIRE_ANGLE after begin() and resetAngles();
// setAngles() replaces them (TurretState: the angles SERVO ADJUST saved, loaded at boot, and its
// records and View). Every stroke, abort() and the sweep back use the angles in force at that moment.
// SERVO ADJUST support (TurretState decides; Input::servoTeach says the mode is on):
//   - teachMove(deg): command an angle now, only while idle, in adjust mode and not armed;
//   - testStroke(): one stroke -> FIRE -> REST with the angles in force, the same sweeps and
//     holds, LED hold as a normal stroke, but no rumble, no ammo change, no shot counted and
//     no cooldown (no dart is meant to leave: the flywheels are off);
//   - update() aborts a test stroke, and brings a teach-moved servo back to rest, the moment
//     Input::servoTeach drops or Input::armed rises, so leaving adjust mode without an abort()
//     call can never leave the pusher off its rest.
//
// SPDX-License-Identifier: MIT
#pragma once
#include <stdint.h>
#include "Types.h"

class Trigger {
 public:
  // TRIGGER_SERVO_REVERSED applied: the angle the servo itself receives for a config.h angle
  // (SERVO_SPAN_DEG - deg when reversed). Its own inverse.
  static float physicalDeg(float deg);
  enum class Phase : uint8_t {
    IDLE,
    SWEEP_OUT,    // commanded angle gliding toward FIRE
    HOLD_FIRE,    // at FIRE, TRIGGER_FORWARD_MS
    SWEEP_BACK,   // gliding back toward REST
    HOLD_REST     // at REST, TRIGGER_RETURN_MS; then complete
  };

  // Why a stroke cannot start now (the first gate that is closed, in this order).
  enum class Gate : uint8_t {
    OPEN,                 // a stroke may start
    NOT_ARMED,            // Input::armed is false
    STROKE_RUNNING,       // the previous stroke has not completed
    SERVO_OFF,            // setServoPower(false) (OFF state) or the servo PWM did not start
    EMPTY,                // ammo 0 with Config::lockoutWhenEmpty
    FLYWHEEL_NOT_READY,   // Config::requiresFlywheel and !Input::flywheelReady
    COOLDOWN              // cooldownMs() since the last completed stroke not over yet (last:
                          // it opens by itself, so a longer-lasting reason is reported first)
  };

  // Behaviour switches, from config.h by default; tests flip them.
  struct Config {
    bool lockoutWhenEmpty;          // FIRE_LOCKOUT_WHEN_EMPTY
    bool requiresFlywheel;          // FIRE_REQUIRES_FLYWHEEL
    bool rumbleFullAutoContinuous;  // RUMBLE_FULL_AUTO_CONTINUOUS
    float sweepDegPerS;             // TRIGGER_SWEEP_DEG_PER_S; 0 (or less) = no sweep
    uint32_t semiCooldownMs;        // TRIGGER_COOLDOWN_MS
  };
  static Config defaults();

  // What TurretState feeds every loop().
  struct Input {
    bool armed = false;          // turret state is ARMED
    bool pulled = false;         // Inputs::triggerPulled() (after hysteresis)
    bool flywheelReady = false;  // Flywheel::ready(nowMs); ignored when !Config::requiresFlywheel
    bool servoTeach = false;     // TurretState's SERVO ADJUST mode is on (never together with armed)
  };

  // One rumble command for ControlLink::requestRumble(strong, weak, ms); ms 0 = stop.
  struct Rumble {
    uint8_t strong = 0;
    uint8_t weak = 0;
    uint16_t ms = 0;
  };

  Trigger();                                    // defaults()
  explicit Trigger(const Config& cfg);
  const Config& config() const { return cfg_; }

  // Boot defaults: semi, magazine full, shot counter 0, config.h angles, servo commanded to rest
  // and parked (disabled). TurretState calls setServoPower(true) when it leaves OFF.
  void begin();
  // Every loop(), after TurretState::update() (App passes TurretState::triggerInput()).
  void update(uint32_t nowMs, const Input& in);

  void abort();                                 // rest now, stroke cancelled, no decrement
  // Shell `fire`: one stroke if gate(in) is OPEN at nowMs, whatever the trigger does. Returns
  // whether a stroke started; on false, gate(in) says why.
  bool fireOnce(uint32_t nowMs, const Input& in);
  // The cooldown part is as of the last update() / fireOnce() (every loop).
  Gate gate(const Input& in) const;
  // DS_FIRE_MODE_*; any other value is ignored. A change makes the current pull stop counting.
  void setFireMode(uint8_t mode);
  void toggleFireMode();
  void resetMagazine() { ammo_ = MAG_CAPACITY; }   // X button / shell `mag`; any state
  // false = servo PWM parked (detached, OFF state); a running stroke is aborted first.
  // true = PWM resumes at the commanded angle (rest unless a stroke is running).
  void setServoPower(bool on);
  bool servoPowered() const;

  // ---- stroke angles and SERVO ADJUST (see the header comment) ----
  // The angles strokes use from now on. Neither call moves the servo: the next stroke, its
  // sweep back, abort() or teachMove() does.
  void setAngles(float restDeg, float fireDeg);
  void resetAngles() { setAngles(TRIGGER_REST_ANGLE, TRIGGER_FIRE_ANGLE); }   // config.h's
  float restDeg() const { return restDeg_; }
  float fireDeg() const { return fireDeg_; }
  // Commands `deg` now. Refused (false) unless in.servoTeach, !in.armed and no stroke runs.
  bool teachMove(float deg, const Input& in);
  // Starts a test stroke. Refused (false) unless in.servoTeach, !in.armed, no stroke runs and
  // the servo is powered.
  bool testStroke(uint32_t nowMs, const Input& in);
  bool testStrokeActive() const { return strokeActive() && testStroke_; }
  bool offRest() const { return teachMoved_; }   // teachMove() left the servo off its rest

  uint8_t fireMode() const { return fireMode_; }
  Phase phase() const { return phase_; }
  bool strokeActive() const { return phase_ != Phase::IDLE; }
  // Whole ms a stroke takes from REST with the angles in force (updates every ms): both sweeps
  // (sweepMs) plus TRIGGER_FORWARD_MS and TRIGGER_RETURN_MS.
  uint32_t strokeMs() const;
  // Whole ms a sweep of fromDeg -> toDeg takes: the first update() after its start whose
  // elapsed ms cover the distance at Config::sweepDegPerS; 0 without a sweep.
  uint32_t sweepMs(float fromDeg, float toDeg) const;
  // The pause after a completed stroke in the current fire mode: semi Config::semiCooldownMs,
  // full auto 0 (back-to-back, spec 3.4).
  uint32_t cooldownMs() const;
  bool coolingDown() const { return cooling_; }          // as of the last update() / fireOnce()
  uint8_t ammo() const { return ammo_; }
  uint8_t capacity() const { return MAG_CAPACITY; }
  uint16_t shotsTotal() const { return shotsTotal_; }    // completed strokes since boot, wraps
  float servoDeg() const { return servoDeg_; }           // commanded angle
  bool firingLedActive(uint32_t nowMs) const;            // stroke running or LED hold not over
  // A pull would start a stroke right now: gate() of `in` with the trigger pulled (a fresh pull,
  // started while armed) is OPEN, so every gate counts without a list here, and the last shot's
  // LED hold (firingLedActive) is over. TurretState::publish() -> DisplayModel::fireReady.
  bool fireReady(uint32_t nowMs, const Input& in) const;
  // A rumble command raised since the last call (cleared by reading; a newer one replaces an
  // unread one). TurretState forwards it to ControlLink::requestRumble.
  bool takeRumbleRequest(Rumble& out);

 private:
  void command(float deg);
  void beginStroke(uint32_t nowMs);    // SWEEP_OUT from the commanded angle, LED hold on
  void startStroke(uint32_t nowMs, bool continuousRumble);
  void beginSweep(Phase phase, float toDeg, uint32_t nowMs);
  bool sweepStep(uint32_t nowMs);      // re-commands the running sweep; true once landed
  float sweptDeg(uint32_t elapsedMs) const;   // distance covered after elapsedMs
  void advance(uint32_t nowMs);        // runs the stroke's phases as far as nowMs allows
  void completeStroke(uint32_t nowMs);
  void refreshCooldown(uint32_t nowMs);
  uint32_t longestCooldownMs() const;   // the longest cooldownMs() of any fire mode
  static bool teachAllowed(const Input& in) { return in.servoTeach && !in.armed; }

  Config cfg_;
  uint8_t fireMode_ = DS_FIRE_MODE_SEMI;
  Phase phase_ = Phase::IDLE;
  uint32_t phaseStartMs_ = 0;
  float sweepFromDeg_ = TRIGGER_REST_ANGLE;   // the running sweep's start and end angles
  float sweepToDeg_ = TRIGGER_REST_ANGLE;
  bool cooling_ = false;       // a real stroke completed less than cooldownMs() ago
  bool recentShot_ = false;    // ... less than longestCooldownMs() ago
  uint32_t completedMs_ = 0;   // when it completed
  uint32_t ledHoldStartMs_ = 0;
  bool ledHold_ = false;
  uint8_t ammo_ = MAG_CAPACITY;
  uint16_t shotsTotal_ = 0;
  float servoDeg_ = TRIGGER_REST_ANGLE;
  float restDeg_ = TRIGGER_REST_ANGLE;   // the angles in force (setAngles / resetAngles)
  float fireDeg_ = TRIGGER_FIRE_ANGLE;
  bool testStroke_ = false;    // the running stroke is a teach-mode test stroke
  bool teachMoved_ = false;    // teachMove() commanded an angle; cleared by abort() / a test stroke
  bool prevPulled_ = false;    // Input::pulled at the previous update (pull-start edge)
  bool pullArmed_ = false;     // the current pull started while armed and still counts
  bool pullUsed_ = false;      // the current pull has fired a stroke (semi: no more)
  bool rumblePending_ = false;
  Rumble rumble_;
};

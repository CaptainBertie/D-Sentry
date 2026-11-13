// Trigger.h -- the dart-pusher servo stroke, semi/full logic and the magazine (spec 3.4,
// docs/architecture.md 5 and 6).
//
// Four angles: REST (TRIGGER_REST_ANGLE, the reload position), FIRE (TRIGGER_FIRE_ANGLE, dart
// pushed), PRE-FIRE (TRIGGER_PREFIRE_ANGLE, the waiting place close to the dart) and CLEAR
// (TRIGGER_CLEAR_ANGLE, a stop just off REST that replaces the old pause there), unless SERVO
// TEACH recorded others. The servo's home is PRE-FIRE while ammo > 0 and REST at ammo 0.
//
// One shot (owner, 2026-11-04), from home: SWEEP_OUT (to FIRE) -> HOLD_FIRE (TRIGGER_FORWARD_MS)
// -> SWEEP_BACK (to REST) -> HOLD_REST (TRIGGER_RETURN_MS) -> [only with Config::clearStop =
// TRIGGER_CLEAR_ENABLED: SWEEP_CLEAR (to CLEAR) -> HOLD_CLEAR (TRIGGER_CLEAR_HOLD_MS) ->
// SWEEP_RACK (to REST again) -> HOLD_RACK (TRIGGER_RETURN_MS, time to arrive)] -> the shot counts: ammo - 1 (floor 0), shotsTotal + 1 -> if ammo > 0
// SWEEP_CHAMBER (to PRE-FIRE) -> HOLD_CHAMBER (TRIGGER_CHAMBER_MS, time to arrive) -> IDLE (the
// next shot is available); at ammo 0 straight to IDLE at REST. strokeActive() (the firing LED,
// telemetry's strokeActive) covers the whole sequence.
// Sweeps: the commanded angle glides at Config::sweepDegPerS (TRIGGER_SWEEP_DEG_PER_S),
// re-commanded every update(), landing exactly on its end angle at the first update() whose
// elapsed time covers the distance; each hold starts at that update(). sweepDegPerS 0 = no
// glide: each angle is commanded at once and the servo moves at its own top speed; a hobby servo
// has no position feedback, so the holds are then its time to arrive.
// The ammo count changes only where the shot counts (and on resetMagazine / bootMagazine).
// abort() (any phase) commands home at once and never decrements or counts a shot.
// Chambering: whenever abort() or resetMagazine() (X) moves the idle servo to a different home,
// no stroke starts for TRIGGER_CHAMBER_MS (counted from the next update()), so a shot never
// starts before the servo arrived (gate CHAMBERING).
//
// A stroke may start only when every gate is open (gate()): armed, no stroke running, not
// chambering, the servo powered (setServoPower), ammo > 0 unless lockout is off (Config::lockoutWhenEmpty =
// FIRE_LOCKOUT_WHEN_EMPTY), Input::flywheelReady unless Config::requiresFlywheel
// (FIRE_REQUIRES_FLYWHEEL) is false, and the cooldown over. Trigger applies
// FIRE_REQUIRES_FLYWHEEL itself, so TurretState passes Flywheel::ready() as it is.
// Cooldown: after a real shot's sequence ends (IDLE again: the next shot would be available), no
// new stroke starts for cooldownMs() of the fire mode in force: semi Config::semiCooldownMs
// (TRIGGER_COOLDOWN_MS), full auto 0. Aborted strokes and test strokes start none.
//
// Firing rules (architecture.md 6):
//   - a pull counts only if it started while armed, and it stops counting the moment armed
//     goes false, the fire mode changes or abort() runs: arming (or re-arming) with the trigger
//     held never fires until it is released and pulled again;
//   - semi: one stroke per pull. A counting pull that is blocked (flywheel not spun up, empty
//     magazine, a stroke still running, chambering, the cooldown) fires once when the block clears, if
//     still held; one let go before then never fires. A trigger held across a stroke's end
//     does not re-fire;
//   - full (owner, 2026-11-04): the first stroke starts from PRE-FIRE as in semi. While the trigger
//     is still held and every gate is open when a stroke's last REST hold ends (the shot
//     counted), the next stroke starts in that same update() straight from REST (REST -> FIRE ->
//     REST ...), with no PRE-FIRE visit and no chamber hold; the CLEAR visit stays in each cycle
//     with TRIGGER_CLEAR_ENABLED. Released (or a gate closed), that stroke chambers to PRE-FIRE
//     and the sequence ends as in semi; ammo reaching 0 ends it at REST;
//   - armed false while a stroke runs (disarm, link loss, power off, fault): update() aborts
//     the stroke itself, so a missing abort() call can never leave the pusher forward;
//   - empty magazine with lockout: no stroke; emptyMagIndicator() while armed and pulled.
// Each stroke start raises a rumble request (takeRumbleRequest): RUMBLE_STRONG_255 /
// RUMBLE_WEAK_255 for RUMBLE_DURATION_MS; in full auto with RUMBLE_FULL_AUTO_CONTINUOUS the
// duration is one whole stroke (strokeMs()) plus RUMBLE_DURATION_MS, so each stroke's request
// overlaps the next one's and the pad rumbles without a break while cycling, then for
// RUMBLE_DURATION_MS after the last stroke ends. abort() of a running stroke requests a stop
// (all zero).
// The firing LED (firingLedActive) is on from each stroke start for the whole stroke and at
// least LED_FIRE_FLASH_MIN_MS; abort() turns it off at once.
//
// Stroke angles: config.h's REST / FIRE / PRE-FIRE / CLEAR after begin() and resetAngles(); setAngles()
// replaces them (TurretState's SERVO TEACH records, RAM only). Every stroke, abort() and each
// sweep use the angles in force at that moment.
// SERVO TEACH support (TurretState decides; Input::servoTeach says the mode is on):
//   - teachMove(deg): command an angle now, only while idle, in teach mode and not armed;
//   - testStroke(): the whole shot sequence with the angles in force, PRE-FIRE -> FIRE -> REST ->
//     [CLEAR -> REST ->] PRE-FIRE (from a servo off PRE-FIRE it first sweeps there and holds
//     TRIGGER_CHAMBER_MS), the same sweeps and holds, firing LED as a normal stroke, but no
//     rumble, no ammo change, no shot counted and no cooldown; it always ends at PRE-FIRE (at
//     ammo 0 that counts as off home, like a teachMove());
//   - update() aborts a test stroke, and brings a teach-moved servo back home, the moment
//     Input::servoTeach drops or Input::armed rises, so leaving teach mode without an abort()
//     call can never leave the pusher off its home.
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
    HOLD_REST,    // at REST, TRIGGER_RETURN_MS (time to arrive)
    SWEEP_CLEAR,  // gliding toward CLEAR
    HOLD_CLEAR,   // at CLEAR, TRIGGER_CLEAR_HOLD_MS
    SWEEP_RACK,   // gliding back to REST
    HOLD_RACK,    // at REST, TRIGGER_RETURN_MS again; then the shot counts
    SWEEP_CHAMBER,   // gliding to PRE-FIRE (ammo > 0, or a test stroke)
    HOLD_CHAMBER     // at PRE-FIRE, TRIGGER_CHAMBER_MS (time to arrive); then IDLE
  };

  // Why a stroke cannot start now (the first gate that is closed, in this order).
  enum class Gate : uint8_t {
    OPEN,                 // a stroke may start
    NOT_ARMED,            // Input::armed is false
    STROKE_RUNNING,       // the previous shot's sequence has not ended
    SERVO_OFF,            // setServoPower(false) (OFF state) or the servo PWM did not start
    EMPTY,                // ammo 0 with Config::lockoutWhenEmpty
    FLYWHEEL_NOT_READY,   // Config::requiresFlywheel and !Input::flywheelReady
    CHAMBERING,           // the idle servo was just sent to a new home (X, abort):
                          // TRIGGER_CHAMBER_MS to arrive
    COOLDOWN              // cooldownMs() since the last shot's sequence ended not over yet (last:
                          // it opens by itself, so a longer-lasting reason is reported first)
  };

  // Behaviour switches, from config.h by default; tests flip them.
  struct Config {
    bool lockoutWhenEmpty;          // FIRE_LOCKOUT_WHEN_EMPTY
    bool requiresFlywheel;          // FIRE_REQUIRES_FLYWHEEL
    bool rumbleFullAutoContinuous;  // RUMBLE_FULL_AUTO_CONTINUOUS
    float sweepDegPerS;             // TRIGGER_SWEEP_DEG_PER_S; 0 (or less) = no sweep
    bool clearStop;                 // TRIGGER_CLEAR_ENABLED: the CLEAR visit and second REST hold
    uint32_t semiCooldownMs;        // TRIGGER_COOLDOWN_MS
  };
  static Config defaults();

  // What TurretState feeds every loop().
  struct Input {
    bool armed = false;          // turret state is ARMED
    bool pulled = false;         // Inputs::triggerPulled() (after hysteresis)
    bool flywheelReady = false;  // Flywheel::ready(nowMs); ignored when !Config::requiresFlywheel
    bool servoTeach = false;     // TurretState's SERVO TEACH mode is on (never together with armed)
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

  // Boot defaults: semi, ammo TRIGGER_AMMO_AT_BOOT, shot counter 0, config.h angles, servo
  // commanded home (REST at ammo 0) and parked (disabled). TurretState calls setServoPower(true)
  // when it leaves OFF.
  void begin();
  // Every loop(), after TurretState::update() (App passes TurretState::triggerInput()).
  void update(uint32_t nowMs, const Input& in);

  void abort();                                 // home now, stroke cancelled, no decrement
  // Shell `fire`: one stroke if gate(in) is OPEN at nowMs, whatever the trigger does. Returns
  // whether a stroke started; on false, gate(in) says why.
  bool fireOnce(uint32_t nowMs, const Input& in);
  // The cooldown part is as of the last update() / fireOnce() (every loop).
  Gate gate(const Input& in) const;
  // DS_FIRE_MODE_*; any other value is ignored. A change makes the current pull stop counting.
  void setFireMode(uint8_t mode);
  void toggleFireMode();
  // X button / shell `mag`; any state: ammo MAG_CAPACITY and, when idle and not teach-moved,
  // the servo to PRE-FIRE (chambering). A running stroke carries on (it chambers at its end).
  void resetMagazine();
  void unloadMagazine();   // ammo to 0 and the idle servo to REST, for taking the mag out
  // Power-toggle reboot: ammo TRIGGER_AMMO_AT_BOOT, then abort() (home: REST at ammo 0).
  void bootMagazine();
  // false = servo PWM parked (detached, OFF state); a running stroke is aborted first.
  // true = PWM resumes at the commanded angle (home unless a stroke is running).
  void setServoPower(bool on);
  bool servoPowered() const;

  // ---- stroke angles and SERVO TEACH (see the header comment) ----
  // The angles strokes use from now on. Neither call moves the servo: the next stroke, its
  // sweeps, abort() or teachMove() does.
  void setAngles(float restDeg, float fireDeg, float prefireDeg, float clearDeg);
  void resetAngles() {
    setAngles(TRIGGER_REST_ANGLE, TRIGGER_FIRE_ANGLE, TRIGGER_PREFIRE_ANGLE, TRIGGER_CLEAR_ANGLE);
  }
  float restDeg() const { return restDeg_; }
  float fireDeg() const { return fireDeg_; }
  float prefireDeg() const { return prefireDeg_; }
  float clearDeg() const { return clearDeg_; }
  float homeDeg() const { return ammo_ > 0 ? prefireDeg_ : restDeg_; }   // where the idle servo waits
  // Commands `deg` now. Refused (false) unless in.servoTeach, !in.armed and no stroke runs.
  bool teachMove(float deg, const Input& in);
  // Starts a test stroke. Refused (false) unless in.servoTeach, !in.armed, no stroke runs and
  // the servo is powered.
  bool testStroke(uint32_t nowMs, const Input& in);
  bool testStrokeActive() const { return strokeActive() && testStroke_; }
  bool offRest() const { return teachMoved_; }   // teachMove() left the servo off its home

  uint8_t fireMode() const { return fireMode_; }
  Phase phase() const { return phase_; }
  bool strokeActive() const { return phase_ != Phase::IDLE; }
  // Whole ms one shot's whole sequence takes from PRE-FIRE with the angles in force (updates
  // every ms), chambering included: its sweeps (sweepMs) plus TRIGGER_FORWARD_MS,
  // TRIGGER_RETURN_MS, TRIGGER_CHAMBER_MS and, with Config::clearStop, the CLEAR visit
  // (two sweeps, TRIGGER_CLEAR_HOLD_MS, TRIGGER_RETURN_MS again).
  uint32_t strokeMs() const;
  bool chambering() const { return chamberPending_ || chambering_; }   // gate CHAMBERING
  // Whole ms a sweep of fromDeg -> toDeg takes: the first update() after its start whose
  // elapsed ms cover the distance at Config::sweepDegPerS; 0 without a sweep.
  uint32_t sweepMs(float fromDeg, float toDeg) const;
  // The pause after a shot's sequence in the current fire mode: semi Config::semiCooldownMs,
  // full auto 0 (back-to-back, spec 3.4).
  uint32_t cooldownMs() const;
  bool coolingDown() const { return cooling_; }          // as of the last update() / fireOnce()
  uint8_t ammo() const { return ammo_; }
  uint8_t capacity() const { return MAG_CAPACITY; }
  uint16_t shotsTotal() const { return shotsTotal_; }    // shots counted since boot, wraps
  float servoDeg() const { return servoDeg_; }           // commanded angle
  bool firingLedActive(uint32_t nowMs) const;            // stroke running or LED hold not over
  bool emptyMagIndicator() const { return emptyMag_; }   // armed, ammo 0, lockout, pulled
  // A rumble command raised since the last call (cleared by reading; a newer one replaces an
  // unread one). TurretState forwards it to ControlLink::requestRumble.
  bool takeRumbleRequest(Rumble& out);

 private:
  void command(float deg);
  void beginStroke(uint32_t nowMs, bool leadIn);   // SWEEP_OUT (or the lead-in), firing LED on
  void goHome();                       // command homeDeg(); a change starts chambering
  void startStroke(uint32_t nowMs, bool continuousRumble);
  void beginSweep(Phase phase, float toDeg, uint32_t nowMs);
  bool sweepStep(uint32_t nowMs);      // re-commands the running sweep; true once landed
  float sweptDeg(uint32_t elapsedMs) const;   // distance covered after elapsedMs
  void advance(uint32_t nowMs);        // runs the stroke's phases as far as nowMs allows
  void restReached(uint32_t nowMs);    // the last REST hold is over: count, then burst, chamber or end
  bool burstContinues() const;         // full auto, still held and permitted: REST -> FIRE again
  void countShot();                    // after the last REST hold: ammo - 1, shotsTotal + 1 (real only)
  void endSequence(uint32_t nowMs);    // IDLE: the next shot is available; cooldown starts
  void refreshCooldown(uint32_t nowMs);
  void refreshChamber(uint32_t nowMs);
  uint32_t longestCooldownMs() const;   // the longest cooldownMs() of any fire mode
  static bool teachAllowed(const Input& in) { return in.servoTeach && !in.armed; }

  Config cfg_;
  uint8_t fireMode_ = DS_FIRE_MODE_SEMI;
  Phase phase_ = Phase::IDLE;
  uint32_t phaseStartMs_ = 0;
  float sweepFromDeg_ = TRIGGER_REST_ANGLE;   // the running sweep's start and end angles
  float sweepToDeg_ = TRIGGER_REST_ANGLE;
  bool cooling_ = false;       // a real shot's sequence ended less than cooldownMs() ago
  bool recentShot_ = false;    // ... less than longestCooldownMs() ago
  uint32_t completedMs_ = 0;   // when it completed
  uint32_t ledHoldStartMs_ = 0;
  bool ledHold_ = false;
  uint8_t ammo_ = TRIGGER_AMMO_AT_BOOT;
  uint16_t shotsTotal_ = 0;
  float servoDeg_ = TRIGGER_REST_ANGLE;
  float restDeg_ = TRIGGER_REST_ANGLE;   // the angles in force (setAngles / resetAngles)
  float fireDeg_ = TRIGGER_FIRE_ANGLE;
  float prefireDeg_ = TRIGGER_PREFIRE_ANGLE;
  float clearDeg_ = TRIGGER_CLEAR_ANGLE;
  bool leadIn_ = false;        // the running SWEEP_CHAMBER / HOLD_CHAMBER leads into SWEEP_OUT
  bool chamberPending_ = false;   // goHome() moved the idle servo; chambering starts next update()
  bool chambering_ = false;    // ... TRIGGER_CHAMBER_MS from chamberStartMs_
  uint32_t chamberStartMs_ = 0;
  bool testStroke_ = false;    // the running stroke is a teach-mode test stroke
  bool teachMoved_ = false;    // teachMove() commanded an angle; cleared by abort() / a test stroke
  bool prevPulled_ = false;    // Input::pulled at the previous update (pull-start edge)
  Input in_;                   // the current update()'s input (burstContinues)
  bool pullArmed_ = false;     // the current pull started while armed and still counts
  bool pullUsed_ = false;      // the current pull has fired a stroke (semi: no more)
  bool emptyMag_ = false;
  bool rumblePending_ = false;
  Rumble rumble_;
};

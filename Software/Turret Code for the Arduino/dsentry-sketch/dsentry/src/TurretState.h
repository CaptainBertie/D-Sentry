// TurretState.h -- the top-level state machine (spec 3.1) and the ONLY place that decides who
// may act (docs/architecture.md rule 5, section 6). Every other module executes; this one
// reads Inputs / ControlLink / Motion / StallDetector / Flywheel / Trigger, applies the
// permission rules, commands them, and publishes the DisplayModel for Leds and Lcd.
//
//   OFF --power on--> BOOT --next tick--> CALIBRATING --done--> PAIRING --connected-->
//   DISARMED <--A--> ARMED;  link lost -> PAIRING;  stall / calibration failure -> FAULT.
//
// States (what each one lets act):
//   OFF          power toggle open: drivers released (EN high), flywheel cut (PWM 0 once a
//                configured brake ends), servo parked (suspended), LEDs dark, LCD SYSTEM OFF.
//                Controller input and shell actions are ignored; ControlLink still runs, so
//                link packets are consumed and StatusPackets still carry wantMode. Closing
//                the toggle restarts from BOOT (a fresh calibration).
//   BOOT         one tick (architecture.md 6), then CALIBRATING.
//   CALIBRATING  Motion's tilt homing (Config::calMode). DONE -> PAIRING; FAILED -> FAULT CAL.
//                Stall events: forwarded to Motion while it waits for one (CAL_STALL_SENSE),
//                ignored otherwise (CAL_RAM rams the panel on purpose).
//   PAIRING      waits for a connected frame (the tick after the loss at the earliest) ->
//                DISARMED. Row 1: from boot "<mode> Pairing<dots>" indefinitely; after a link
//                loss "Pairing<dots>" for LINK_RECONNECT_GRACE_MS, then "No Link".
//   DISARMED     stick motion, set/go home, B, X; flywheels only if !FLYWHEEL_REQUIRES_ARMED.
//   ARMED        all of the above plus flywheels and the trigger.
//   FAULT        STALL: the axes that were moving back off STALL_BACKOFF_DEG, the others stop;
//                after STALL_FAULT_HOLD_MS (from the latest stall) back to the arm state it
//                came from, or PAIRING if the link dropped meanwhile. CAL: only the power
//                toggle clears it. B and X still work (they only change counters / mode).
//
// Rules where the spec is silent (architecture.md 6), all decided here:
//   - Link loss = the active source's frame stops being connected, a source change while
//     connected (ControlLink::takeSourceChanged), or a mode-toggle change. Effects: motion
//     stopped (decelerated; a stall back-off in FAULT finishes), stroke aborted (rest now, no
//     decrement; Trigger's home: PRE-FIRE while ammo > 0, else REST), flywheels cut if FAILSAFE_STOP_FLYWHEEL, DISARMED/ARMED -> PAIRING. During
//     CALIBRATING the calibration continues; during FAULT the fault continues.
//   - Reconnection always lands in DISARMED: FAILSAFE_DISARM_ON_LOSS = false cannot restore
//     ARMED, so Debug reports it at boot as ignored.
//   - FAILSAFE_STOP_FLYWHEEL false: the flywheels keep the state they had through PAIRING; on
//     reconnection the DISARMED rules apply (cut if FLYWHEEL_REQUIRES_ARMED).
//   - The mode toggle (debounced A1) is passed to ControlLink::setWantMode in every state
//     (OFF included, so StatusPacket.wantMode always mirrors the switch); outside OFF it also
//     counts as a link loss.
//   - Flywheels: hold mode = on while LB is held (or the shell latch `fly on` is set); toggle
//     mode = each LB press flips the latch. In ARMED a d-pad up press also flips the latch (owner,
//     2026-11-04), so the wheels stay on after LB is let go until d-pad up again, a disarm (cut
//     with FLYWHEEL_REQUIRES_ARMED), a link loss, power off or a fault; in DISARMED d-pad up does
//     nothing. Allowed in ARMED, and in DISARMED when
//     !FLYWHEEL_REQUIRES_ARMED; anywhere else (and whenever not allowed) they are cut and
//     the latch cleared. LB presses that are not allowed are ignored, not remembered.
//   - A arms only from DISARMED, refused while RT is pulled (ARM_REQUIRES_TRIGGER_RELEASED);
//     A in ARMED disarms. D-pad up: the flywheel latch in ARMED (above). B (semi/full) and X (magazine: MAG_CAPACITY, servo to PRE-FIRE) act in
//     every state but OFF.
//   - Power toggle: every OFF is a new boot for the magazine too: ammo TRIGGER_AMMO_AT_BOOT
//     (Trigger::bootMagazine), servo home (REST at ammo 0).
//     D-pad left / down (set / go home) and the stick act in DISARMED and ARMED; any stick
//     input past the dead zone cancels a go-home (or a shell pan/tilt move).
//   - Trigger::Input: armed = state ARMED, pulled = Inputs::triggerPulled(), flywheelReady =
//     Flywheel::ready() (Trigger applies FIRE_REQUIRES_FLYWHEEL and the other gates itself).
//   - Rumble: each Trigger::takeRumbleRequest goes to ControlLink::requestRumble (the stroke
//     starts in Trigger::update, so the request leaves on the next update, < 1 ms later).
//
// SERVO TEACH (the owner sets the trigger servo's REST, FIRE, PRE-FIRE and CLEAR angles with the pad; a hobby
// servo reports no position, so what is recorded is the angle being commanded):
//   - A rising edge of Y enters it, only from DISARMED (Y anywhere else is refused and logged),
//     and leaves it again. The state stays DISARMED for every purpose: no stroke but the test
//     stroke, A does not arm, B does not toggle the fire mode, X does not reset the magazine,
//     d-pad left / down do not set / go home, LB does not touch the flywheels. The stick still aims.
//   - In it: d-pad up / down +- SERVO_TEACH_STEP_SMALL_DEG, RB / LB +- SERVO_TEACH_STEP_LARGE_DEG
//     (rising edges with the BUTTON_DEBOUNCE_MS lockout, no auto-repeat), clamped to
//     SERVO_TEACH_MIN_DEG..SERVO_TEACH_MAX_DEG, starting from the REST in force (config.h's, or
//     the last recorded one). A records the commanded angle as REST, B as FIRE, d-pad right as
//     PRE-FIRE, View as CLEAR (each refused while the test stroke runs); X runs one test stroke,
//     the whole shot sequence PRE-FIRE -> FIRE -> REST -> CLEAR -> REST -> PRE-FIRE with the
//     angles in force and no ammo change (refused until FIRE was recorded, and while the REST,
//     PRE-FIRE or CLEAR in force is outside the teach range). D-pad left toggles the flywheels on / off (the normal ramp), in teach mode
//     only and whatever FLYWHEEL_REQUIRES_ARMED says; they are cut whenever teach mode ends.
//     Recorded angles are RAM only (the spec forbids stored settings) and every stroke uses
//     them until the power toggle opens (OFF = a new boot: config.h's angles again). Each
//     record logs, whatever the log level (Debug::logAlways):
//       servo teach: REST recorded -> #define TRIGGER_REST_ANGLE 23
//     and leaving teach mode logs the four lines to paste into config.h (two log lines):
//       servo teach: done; config.h lines: #define TRIGGER_REST_ANGLE 23 / #define TRIGGER_FIRE_ANGLE 108
//       servo teach: done; config.h lines: #define TRIGGER_PREFIRE_ANGLE 60 / #define TRIGGER_CLEAR_ANGLE 40
//   - Teach mode ends, with the servo back home (PRE-FIRE while ammo > 0, else REST; a test stroke
//     aborted) and the flywheels cut, on Y, the shell, a link loss (the mode toggle included),
//     power off, a fault and any state change.
//   - Shown as LCD "SERVO TEACH <angle>" / "R<rest>F<fire>P<prefire>C<clear>" ("-" = not recorded) and
//     LEDs LED_COLOR_SERVO_TEACH (below FAULT and FIRING). Telemetry is unchanged: it reports
//     DISARMED and the live servoDeg.
//
// Shell actions (Debug) go through shellCommand() and obey the same rules: arm / disarm / fire
// (one stroke through Trigger's gates, the semi cooldown included) / fly on|off (the latch) / mode semi|full / mag /
// pan|tilt <deg> (DISARMED or ARMED; tilt only once calibrated; outside the active soft limits
// refused) / home / sethome / stop (any state: it only ever stops) / cal (from DISARMED or
// PAIRING: re-runs the calibration, then PAIRING -> DISARMED). SERVO TEACH: servo teach (the
// pad's Y: enter from DISARMED, or leave) / servo <whole deg> (go to that angle, refused outside
// SERVO_TEACH_MIN_DEG..MAX) / servo rest (A) / servo fire (B) / servo setprefire (d-pad right) /
// servo prefire (go to the PRE-FIRE in force) / servo setclear (View) / servo clear (go to the
// CLEAR in force) / servo stroke (X) / servo off
// (leave; ok when not in it). In teach mode arm, fire, fly on, mode, mag, home and sethome are
// refused (REFUSED_TEACHING), as their pad buttons are; pan, tilt, stop, disarm and fly off (it
// also turns d-pad left's flywheels off) work;
// cal leaves teach mode (a state change).
//
// SPDX-License-Identifier: MIT
#pragma once
#include <stdint.h>
#include "Trigger.h"
#include "Types.h"

class Inputs;
class ControlLink;
class Motion;
class StallDetector;
class Flywheel;
class Debug;

class TurretState {
 public:
  // Bench shell commands that act on the turret (Debug parses the text).
  enum class ShellCmd : uint8_t {
    ARM, DISARM, FIRE, FLY_ON, FLY_OFF, MODE_SEMI, MODE_FULL, MAG, PAN, TILT, HOME, SETHOME,
    STOP, CAL,
    SERVO_TEACH, SERVO_SET, SERVO_REST, SERVO_FIRE, SERVO_STROKE, SERVO_OFF,   // SERVO TEACH
    SERVO_PREFIRE, SERVO_SETPREFIRE, SERVO_CLEAR, SERVO_SETCLEAR
  };
  // Why a command (shell or controller) was or was not carried out.
  enum class CmdResult : uint8_t {
    OK,
    REFUSED_STATE,          // not allowed in the current state (e.g. fire while DISARMED)
    REFUSED_TRIGGER_HELD,   // arm refused: ARM_REQUIRES_TRIGGER_RELEASED and RT pulled
    REFUSED_NOT_READY,      // fire refused: flywheel not spun up / stroke running / chambering
    REFUSED_EMPTY,          // fire refused: magazine empty with FIRE_LOCKOUT_WHEN_EMPTY
    REFUSED_RANGE,          // pan/tilt target outside the active soft limits (or not a number);
                            // servo angle outside SERVO_TEACH_MIN_DEG..SERVO_TEACH_MAX_DEG
    REFUSED_NOT_CALIBRATED, // tilt move before calibration finished
    REFUSED_TEACHING,       // a normal action while SERVO TEACH is on (arm, mode, mag, home ...)
    REFUSED_NOT_TEACHING,   // a SERVO TEACH action while it is off
    REFUSED_NO_FIRE,        // test stroke before FIRE was recorded
    REFUSED_COOLDOWN        // fire refused: semi auto, TRIGGER_COOLDOWN_MS since the last
                            // completed stroke not over yet (Trigger::Gate::COOLDOWN)
  };

  // Behaviour switches, from config.h by default; the App tests flip them.
  struct Config {
    bool armRequiresTriggerReleased;   // ARM_REQUIRES_TRIGGER_RELEASED
    bool flywheelHoldMode;             // FLYWHEEL_HOLD_MODE
    bool flywheelRequiresArmed;        // FLYWHEEL_REQUIRES_ARMED
    bool failsafeStopFlywheel;         // FAILSAFE_STOP_FLYWHEEL
    bool failsafeDisarmOnLoss;         // FAILSAFE_DISARM_ON_LOSS (reported only: see above)
    uint8_t calMode;                   // TILT_CAL_MODE: CAL_RAM or CAL_STALL_SENSE
  };
  static Config defaults();

  // The compact picture Debug's `status` and the tests read.
  struct Status {
    uint8_t state = DS_STATE_OFF;
    uint8_t fault = DS_FAULT_NONE;
    uint32_t stateSinceMs = 0;
    PairingPhase pairing = PairingPhase::FROM_BOOT;
    bool linkConnected = false;
    bool flywheelAllowed = false;      // the state permits the flywheels now
    bool flywheelLatch = false;        // toggle-mode state / shell `fly on`
    uint8_t stateBeforeFault = DS_STATE_DISARMED;   // where a stall fault returns
    bool linkDroppedInFault = false;   // ... unless this is set: then PAIRING
    uint32_t faultSinceMs = 0;
    uint32_t linkLosses = 0;           // since begin()
    uint32_t stalls = 0;               // stall events acted on (FAULT entries and repeats)
    uint32_t refusals = 0;             // refused controller / shell actions
    CmdResult lastRefusal = CmdResult::OK;
    uint32_t transitions = 0;          // state changes since begin()
    bool servoTeach = false;           // SERVO TEACH is on
    int16_t servoDeg = 0;              // the angle the trigger servo is commanded to, whole deg
    int16_t restDeg = 0;               // the REST / FIRE / PRE-FIRE angles strokes use now
    int16_t fireDeg = 0;
    int16_t prefireDeg = 0;
    int16_t clearDeg = 0;
    bool restRecorded = false;         // ... recorded by SERVO TEACH this power-on (else config.h)
    bool fireRecorded = false;
    bool prefireRecorded = false;
    bool clearRecorded = false;
    bool teachFlywheel = false;        // SERVO TEACH's d-pad-left flywheel toggle is on
  };

  TurretState();                        // defaults()
  explicit TurretState(const Config& cfg);
  const Config& config() const { return cfg_; }

  // After every module's begin(). Enters BOOT (or OFF if the power toggle reads open).
  void begin(Inputs& inputs, ControlLink& link, Motion& motion, StallDetector& stall,
             Flywheel& flywheel, Trigger& trigger, Debug& debug, uint32_t nowMs);

  // Once per loop(), after Inputs / StallDetector (and the shell lines) and before Trigger /
  // Flywheel / Motion.
  void update(uint32_t nowMs);

  // Same permission rules as the controller. arg: degrees for PAN / TILT, ignored otherwise.
  CmdResult shellCommand(ShellCmd cmd, float arg, uint32_t nowMs);
  static const char* resultName(CmdResult r);

  uint8_t state() const { return state_; }              // DS_STATE_*
  uint8_t fault() const { return fault_; }              // DS_FAULT_*
  uint32_t stateSinceMs() const { return stateSinceMs_; }
  const DisplayModel& display() const { return display_; }
  // What Trigger::update() gets this loop (App passes it on right after update()): armed, the
  // trigger level and the flywheel gate, already decided here.
  const Trigger::Input& triggerInput() const { return triggerInput_; }
  Status status() const;
  bool everConnected() const { return everConnected_; }
  uint32_t linkLostMs() const { return linkLostMs_; }   // when the link last dropped
  uint32_t transitions() const { return transitions_; } // state changes since begin()
  bool servoTeach() const { return teach_; }             // SERVO TEACH is on (state DISARMED)

 private:
  void enter(uint8_t state, uint32_t nowMs);
  void enterOff(uint32_t nowMs);
  void leaveOff(uint32_t nowMs);
  void enterCalibrating(uint32_t nowMs);
  void enterFault(uint8_t fault, uint32_t nowMs);
  void onLinkLost(uint32_t nowMs, const char* why);
  void onStall(uint32_t nowMs);
  void controls(uint32_t nowMs);        // DISARMED / ARMED: buttons and stick
  void normalButtons(uint32_t nowMs);   // A, B, X, d-pad left / down
  void anyStateButtons();               // B and X
  // SERVO TEACH
  void teachButtons(uint32_t nowMs);    // d-pad, RB / LB, A, B, X, View
  CmdResult enterTeach(uint32_t nowMs, const char* via);
  void leaveTeach(const char* why);     // no-op unless on: flywheels cut, servo home + summary
  CmdResult teachGoTo(int32_t deg, bool clamp, uint32_t nowMs);
  enum class TeachAngle : uint8_t { REST, FIRE, PREFIRE, CLEAR };
  CmdResult teachRecord(TeachAngle which);
  CmdResult teachTestStroke(uint32_t nowMs);
  void forgetRecordedAngles();          // power off: config.h's angles again
  int16_t commandedDeg() const;         // Trigger::servoDeg(), whole degrees
  void decideFlywheel(uint32_t nowMs);
  void publish(uint32_t nowMs);         // triggerInput_, display_, ControlLink state
  void forwardRumble();
  bool flywheelAllowed() const;
  CmdResult tryArm(uint32_t nowMs);
  void disarm(uint32_t nowMs);
  CmdResult moveAxis(Axis axis, float deg);
  CmdResult refuse(CmdResult why, const char* what);
  Trigger::Input freshTriggerInput(uint32_t nowMs) const;
  PairingPhase pairingPhase(uint32_t nowMs) const;

  Config cfg_;
  Inputs* inputs_ = nullptr;
  ControlLink* link_ = nullptr;
  Motion* motion_ = nullptr;
  StallDetector* stall_ = nullptr;
  Flywheel* flywheel_ = nullptr;
  Trigger* trigger_ = nullptr;
  Debug* debug_ = nullptr;
  uint8_t state_ = DS_STATE_OFF;
  uint8_t fault_ = DS_FAULT_NONE;
  uint8_t stateBeforeFault_ = DS_STATE_DISARMED;
  bool linkDroppedInFault_ = false;
  uint32_t stateSinceMs_ = 0;
  uint32_t faultSinceMs_ = 0;
  uint32_t linkLostMs_ = 0;
  bool lossSinceBoot_ = false;     // a link loss happened since the last BOOT (pairing phase)
  bool wasConnected_ = false;      // link connected at the previous update
  bool lostThisTick_ = false;      // PAIRING may not end in the update that entered it
  bool everConnected_ = false;
  bool flyLatch_ = false;
  bool teach_ = false;             // SERVO TEACH is on (only ever in DISARMED)
  bool teachFly_ = false;          // SERVO TEACH's d-pad-left flywheel toggle
  bool teachSwitchedThisTick_ = false;   // Y switched SERVO TEACH this update (LB ignored too)
  bool restRecorded_ = false;      // SERVO TEACH recorded REST / FIRE this power-on
  bool fireRecorded_ = false;
  bool prefireRecorded_ = false;
  bool clearRecorded_ = false;
  uint32_t transitions_ = 0;
  uint32_t linkLosses_ = 0;
  uint32_t stalls_ = 0;
  uint32_t refusals_ = 0;
  CmdResult lastRefusal_ = CmdResult::OK;
  DisplayModel display_;
  Trigger::Input triggerInput_;
};

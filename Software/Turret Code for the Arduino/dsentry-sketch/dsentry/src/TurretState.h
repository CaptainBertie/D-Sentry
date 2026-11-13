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
//     decrement), flywheels cut if FAILSAFE_STOP_FLYWHEEL, DISARMED/ARMED -> PAIRING. During
//     CALIBRATING the calibration continues; during FAULT the fault continues.
//   - Reconnection always lands in DISARMED: FAILSAFE_DISARM_ON_LOSS = false cannot restore
//     ARMED, so Debug reports it at boot as ignored.
//   - FAILSAFE_STOP_FLYWHEEL false: the flywheels keep the state they had through PAIRING; on
//     reconnection the DISARMED rules apply (cut if FLYWHEEL_REQUIRES_ARMED).
//   - The mode toggle (debounced A1) is passed to ControlLink::setWantMode in every state
//     (OFF included, so StatusPacket.wantMode always mirrors the switch); outside OFF it also
//     counts as a link loss.
//   - Flywheels: hold mode = on while LB is held (or the shell latch `fly on` is set); toggle
//     mode = each LB press flips the latch. Allowed in ARMED, and in DISARMED when
//     !FLYWHEEL_REQUIRES_ARMED; anywhere else (and whenever not allowed) they are cut and
//     the latch cleared. LB presses that are not allowed are ignored, not remembered.
//   - A arms only from DISARMED, refused while RT is pulled (ARM_REQUIRES_TRIGGER_RELEASED);
//     A in ARMED disarms. B (semi/full) and X (magazine) act in every state but OFF.
//     D-pad left / down (set / go home) and the stick act in DISARMED and ARMED; any stick
//     input past the dead zone cancels a go-home (or a shell pan/tilt move).
//   - Trigger::Input: armed = state ARMED, pulled = Inputs::triggerPulled(), flywheelReady =
//     Flywheel::ready() (Trigger applies FIRE_REQUIRES_FLYWHEEL and the other gates itself).
//   - Rumble: each Trigger::takeRumbleRequest goes to ControlLink::requestRumble (the stroke
//     starts in Trigger::update, so the request leaves on the next update, < 1 ms later).
//   - LEDs (owner, 2026-11-03): DisplayModel::fireReady = Trigger::fireReady(triggerInput()), so
//     ARMED is red when a pull would fire now and yellow while any gate is closed or the shot's
//     LED_FIRE_FLASH_MIN_MS hold runs. publish() runs before Trigger::update(), and Trigger starts
//     a full-auto stroke in the update that completes the last one, so a held trigger stays
//     yellow from stroke to stroke. DisplayModel::emptyMagBlink = ammo 0 while DISARMED or ARMED
//     outside SERVO ADJUST (the purple double flash, until X); it never blocks anything.
//   - Y: unused (no action, never refused).
//
// SERVO ADJUST (the owner sets the trigger servo's REST and FIRE angles with the pad; a hobby
// servo reports no position, so what is recorded is the angle being commanded). The code calls
// it "teach" (teach_, enterTeach, Trigger::teachMove), its name until 2026-11-03.
//   - Boot: begin() loads the record ServoStore keeps in the data flash (owner, 2026-11-03,
//     overriding the spec's "settings RAM only"). A valid record's REST and FIRE replace config.h's
//     for this run ("servo adjust: loaded saved REST n FIRE m"); a blank or invalid one leaves
//     config.h's ("servo adjust: no saved angles, using config.h REST n FIRE m"). These boot angles
//     are what the power toggle's OFF -> BOOT returns to, and they follow every save.
//   - A rising edge of Menu (DS_BTN_START, the three-line button) enters it, only from DISARMED
//     (Menu in CALIBRATING, ARMED or FAULT is refused and logged); Menu again saves and leaves.
//     A Menu press acts on nothing else in its frame. The state stays DISARMED for every purpose:
//     flywheels cut and refused whatever FLYWHEEL_REQUIRES_ARMED or LB say, no stroke but the test
//     stroke, A does not arm, B does not toggle the fire mode, X does not reset the magazine,
//     d-pad left / down do not set / go home. The stick still aims.
//   - In it, starting from the angles in force (servo at their REST, clamped to the adjust range):
//     d-pad up / down +- SERVO_ADJUST_STEP_SMALL_DEG, RB / LB +- SERVO_ADJUST_STEP_LARGE_DEG
//     (rising edges with the BUTTON_DEBOUNCE_MS lockout, no auto-repeat), clamped to
//     SERVO_ADJUST_MIN_DEG..SERVO_ADJUST_MAX_DEG. A records the commanded angle as REST, B as FIRE;
//     View (DS_BTN_BACK) puts config.h's REST and FIRE back in force and moves the servo to that
//     REST; X runs one test stroke between the REST and FIRE in force (refused while either is
//     outside the adjust range). A, B, View and the nudges are refused while the test stroke runs.
//     Every stroke uses the angles in force. Each record and View logs, whatever the log level
//     (Debug::logAlways):
//       servo adjust: REST recorded -> #define TRIGGER_REST_ANGLE 23
//   - Leaving with Menu (or the shell's servo adjust / servo off) SAVES: when the angles in force
//     differ from the boot angles, ServoStore writes them ("servo adjust: saved REST n FIRE m
//     (survives power off), <t> ms") and they become the boot angles; otherwise nothing is written
//     ("... unchanged, nothing written"). The save is the one blocking call in loop() (hal.h,
//     about 50 ms; the step interrupt keeps running): it happens only in DISARMED with no stroke
//     running (the test stroke is aborted first) and no axis moving, so Menu is refused
//     (REFUSED_MOVING) while an axis moves, because the motion planner stops for the save.
//   - Leaving any other way does NOT save: a link loss (the mode toggle included), power off, a
//     fault, a state change (shell cal). Changed angles stay in force for this run ("servo
//     adjust: not saved (link lost): REST n FIRE m stay for this run only"); power off returns to
//     the boot angles. Every exit puts the servo back at the REST in force (a test stroke
//     aborted) and logs the two lines to paste into config.h:
//       servo adjust: done; config.h lines: #define TRIGGER_REST_ANGLE 23 / #define TRIGGER_FIRE_ANGLE 108
//   - Shown as LCD "SERVO ADJUST <angle>" / "R:<rest> F:<fire>" (the angles in force) and LEDs
//     LED_COLOR_SERVO_ADJUST (only FAULT wins over it; it wins over the empty-magazine flash, a
//     test stroke included). Telemetry is unchanged: it reports DISARMED and the live servoDeg.
//
// Shell actions (Debug) go through shellCommand() and obey the same rules: arm / disarm / fire
// (one stroke through Trigger's gates, the semi cooldown included) / fly on|off (the latch) / mode semi|full / mag /
// pan|tilt <deg> (DISARMED or ARMED; tilt only once calibrated; outside the active soft limits
// refused) / home / sethome / stop (any state: it only ever stops) / cal (from DISARMED or
// PAIRING: re-runs the calibration, then PAIRING -> DISARMED). SERVO ADJUST: servo adjust (alias
// servo teach; the pad's Menu: enter from DISARMED, or save and leave) / servo <whole deg> (go to
// that angle, refused outside SERVO_ADJUST_MIN_DEG..MAX) / servo rest (A) / servo fire (B) /
// servo reset (View) / servo stroke (X) / servo off (save and leave; ok when not in it). In adjust
// mode arm, fire, fly on, mode, mag, home and sethome are refused (REFUSED_TEACHING), as their pad
// buttons are; pan, tilt, stop, disarm and fly off work; cal leaves adjust mode (a state change).
//
// SPDX-License-Identifier: MIT
#pragma once
#include <stdint.h>
#include "ServoStore.h"
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
    SERVO_TEACH, SERVO_SET, SERVO_REST, SERVO_FIRE, SERVO_RESET, SERVO_STROKE, SERVO_OFF   // SERVO ADJUST
  };
  // Why a command (shell or controller) was or was not carried out.
  enum class CmdResult : uint8_t {
    OK,
    REFUSED_STATE,          // not allowed in the current state (e.g. fire while DISARMED)
    REFUSED_TRIGGER_HELD,   // arm refused: ARM_REQUIRES_TRIGGER_RELEASED and RT pulled
    REFUSED_NOT_READY,      // fire refused: flywheel not spun up / stroke running
    REFUSED_EMPTY,          // fire refused: magazine empty with FIRE_LOCKOUT_WHEN_EMPTY
    REFUSED_RANGE,          // pan/tilt target outside the active soft limits (or not a number);
                            // servo angle outside SERVO_ADJUST_MIN_DEG..SERVO_ADJUST_MAX_DEG
    REFUSED_NOT_CALIBRATED, // tilt move before calibration finished
    REFUSED_TEACHING,       // a normal action while SERVO ADJUST is on (arm, mode, mag, home ...)
    REFUSED_NOT_TEACHING,   // a SERVO ADJUST action while it is off
    REFUSED_COOLDOWN,       // fire refused: semi auto, TRIGGER_COOLDOWN_MS since the last
                            // completed stroke not over yet (Trigger::Gate::COOLDOWN)
    REFUSED_MOVING          // SERVO ADJUST save and leave while an axis moves (the save blocks)
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
    bool servoTeach = false;           // SERVO ADJUST is on
    int16_t servoDeg = 0;              // the angle the trigger servo is commanded to, whole deg
    int16_t restDeg = 0;               // the REST / FIRE angles strokes use now
    int16_t fireDeg = 0;
    servostore::Angles bootAngles;     // what a power-toggle reboot uses: the saved record's, or
    bool savedRecord = false;          // ... config.h's when no valid record (savedRecord false)
    uint32_t saves = 0;                // records written since begin()
    uint32_t saveFailures = 0;         // saves whose read-back did not match
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
  bool servoTeach() const { return teach_; }             // SERVO ADJUST is on (state DISARMED)
  const servostore::Angles& bootAngles() const { return bootAngles_; }

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
  // SERVO ADJUST
  void loadBootAngles();                // begin(): the saved record, else config.h's
  void teachButtons(uint32_t nowMs);    // d-pad up / down, RB / LB, A, B, View, X
  CmdResult enterTeach(uint32_t nowMs, const char* via);
  CmdResult saveAndLeaveTeach(const char* via);   // Menu / shell: refused while an axis moves
  // No-op unless on: servo to the REST in force, then the save (save) or the "not saved" line,
  // then the summary.
  void leaveTeach(const char* why, bool save = false);
  void saveAngles(const servostore::Angles& a);   // the one blocking call (hal::storeSave)
  CmdResult teachGoTo(int32_t deg, bool clamp, uint32_t nowMs);
  CmdResult teachRecord(bool fire);
  CmdResult teachRestoreConfig(uint32_t nowMs);   // View: config.h's REST and FIRE in force
  CmdResult teachTestStroke(uint32_t nowMs);
  void restoreBootAngles();             // power off: the boot angles again
  servostore::Angles anglesInForce() const;   // Trigger's, whole degrees
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
  bool teach_ = false;             // SERVO ADJUST is on (only ever in DISARMED)
  bool menuThisTick_ = false;      // a Menu press this update: it acts on nothing else (LB too)
  servostore::Angles bootAngles_;  // the saved record's angles, or config.h's (savedRecord_ false)
  bool savedRecord_ = false;       // a valid record in the data flash holds bootAngles_
  uint32_t saves_ = 0;
  uint32_t saveFailures_ = 0;
  uint32_t transitions_ = 0;
  uint32_t linkLosses_ = 0;
  uint32_t stalls_ = 0;
  uint32_t refusals_ = 0;
  CmdResult lastRefusal_ = CmdResult::OK;
  DisplayModel display_;
  Trigger::Input triggerInput_;
};

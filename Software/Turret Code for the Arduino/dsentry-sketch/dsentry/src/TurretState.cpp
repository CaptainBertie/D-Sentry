// TurretState.cpp -- see TurretState.h for the states and every rule decided here.
//
// SPDX-License-Identifier: MIT
#include "TurretState.h"

#include <math.h>

#include "ControlLink.h"
#include "Debug.h"
#include "Flywheel.h"
#include "Inputs.h"
#include "Motion.h"
#include "ServoStore.h"
#include "StallDetector.h"
#include "Trigger.h"
#include "hal.h"

static_assert(STALL_FAULT_HOLD_MS > 0, "STALL_FAULT_HOLD_MS must be positive");
static_assert(STALL_BACKOFF_DEG >= 0.0f, "STALL_BACKOFF_DEG must not be negative");
static_assert(TILT_CAL_MODE == CAL_RAM || TILT_CAL_MODE == CAL_STALL_SENSE,
              "TILT_CAL_MODE must be CAL_RAM or CAL_STALL_SENSE");
static_assert(SERVO_ADJUST_MIN_DEG >= 0 && SERVO_ADJUST_MAX_DEG <= SERVO_SPAN_DEG &&
                  SERVO_ADJUST_MIN_DEG < SERVO_ADJUST_MAX_DEG,
              "SERVO ADJUST range must be inside the servo's 0..SERVO_SPAN_DEG");
static_assert(SERVO_ADJUST_STEP_SMALL_DEG > 0 && SERVO_ADJUST_STEP_LARGE_DEG > 0 &&
                  SERVO_ADJUST_STEP_SMALL_DEG <= SERVO_ADJUST_MAX_DEG - SERVO_ADJUST_MIN_DEG &&
                  SERVO_ADJUST_STEP_LARGE_DEG <= SERVO_ADJUST_MAX_DEG - SERVO_ADJUST_MIN_DEG,
              "SERVO ADJUST steps must be positive and fit the adjust range");

namespace {

// SERVO ADJUST log texts. The summary must fit one log line (uptime, a space, the text) at its
// widest, so the owner can paste both #define lines from one place.
const char kTeachTag[] = "servo adjust: ";
const char kRestDefine[] = "#define TRIGGER_REST_ANGLE ";
const char kFireDefine[] = "#define TRIGGER_FIRE_ANGLE ";
const char kSummaryHead[] = "servo adjust: done; config.h lines: ";
const char kRestWord[] = "REST ";
const char kFireWord[] = " FIRE ";
const char kSummarySep[] = " / ";
constexpr size_t textLen(size_t sizeofLiteral) { return sizeofLiteral - 1; }
constexpr size_t digitsOf(uint32_t v) { return v < DEC_BASE ? 1 : 1 + digitsOf(v / DEC_BASE); }
constexpr size_t kSummaryMax = digitsOf(UINT32_MAX) + 1 + textLen(sizeof kSummaryHead) +
                               textLen(sizeof kRestDefine) + digitsOf(SERVO_SPAN_DEG) +
                               textLen(sizeof kSummarySep) + textLen(sizeof kFireDefine) +
                               digitsOf(SERVO_SPAN_DEG);
static_assert(kSummaryMax <= DEBUG_LOG_LINE_MAX,
              "DEBUG_LOG_LINE_MAX is too short for the SERVO ADJUST summary line");

inline int16_t wholeDeg(float deg) { return (int16_t)lroundf(deg); }

inline int32_t clampTeach(int32_t deg) {
  if (deg < SERVO_ADJUST_MIN_DEG) return SERVO_ADJUST_MIN_DEG;
  if (deg > SERVO_ADJUST_MAX_DEG) return SERVO_ADJUST_MAX_DEG;
  return deg;
}

inline bool elapsed(uint32_t nowMs, uint32_t sinceMs, uint32_t periodMs) {
  return (uint32_t)(nowMs - sinceMs) >= periodMs;
}

inline bool finiteDeg(float v) { return v == v && v - v == 0.0f; }   // not NaN, not +-inf

const char* calFailureName(Motion::CalFailure f) {
  switch (f) {
    case Motion::CalFailure::TIMEOUT: return "no stall within TILT_CAL_TIMEOUT_MS";
    case Motion::CalFailure::NO_TIMER: return "no tilt step timer";
    case Motion::CalFailure::NONE: break;
  }
  return "stopped";
}

}  // namespace

TurretState::Config TurretState::defaults() {
  Config c;
  c.armRequiresTriggerReleased = ARM_REQUIRES_TRIGGER_RELEASED;
  c.flywheelHoldMode = FLYWHEEL_HOLD_MODE;
  c.flywheelRequiresArmed = FLYWHEEL_REQUIRES_ARMED;
  c.failsafeStopFlywheel = FAILSAFE_STOP_FLYWHEEL;
  c.failsafeDisarmOnLoss = FAILSAFE_DISARM_ON_LOSS;
  c.calMode = TILT_CAL_MODE;
  return c;
}

TurretState::TurretState() : cfg_(defaults()) {}

TurretState::TurretState(const Config& cfg) : cfg_(cfg) {}

void TurretState::begin(Inputs& inputs, ControlLink& link, Motion& motion, StallDetector& stall,
                        Flywheel& flywheel, Trigger& trigger, Debug& debug, uint32_t nowMs) {
  inputs_ = &inputs;
  link_ = &link;
  motion_ = &motion;
  stall_ = &stall;
  flywheel_ = &flywheel;
  trigger_ = &trigger;
  debug_ = &debug;
  state_ = DS_STATE_OFF;
  fault_ = DS_FAULT_NONE;
  stateBeforeFault_ = DS_STATE_DISARMED;
  linkDroppedInFault_ = false;
  stateSinceMs_ = nowMs;
  faultSinceMs_ = nowMs;
  linkLostMs_ = nowMs;
  lossSinceBoot_ = false;
  lostThisTick_ = false;
  everConnected_ = false;
  flyLatch_ = false;
  teach_ = false;
  menuThisTick_ = false;
  saves_ = 0;
  saveFailures_ = 0;
  transitions_ = 0;
  linkLosses_ = 0;
  stalls_ = 0;
  refusals_ = 0;
  lastRefusal_ = CmdResult::OK;
  display_ = DisplayModel();
  link_->setWantMode(inputs_->radioMode());   // the radio the switch asks for, from the start
  wasConnected_ = link_->connected();
  loadBootAngles();                // before the servo PWM resumes: it starts at their REST
  if (inputs_->powerOn()) {
    trigger_->setServoPower(true);
    enter(DS_STATE_BOOT, nowMs);
  } else {
    enterOff(nowMs);
  }
  publish(nowMs);
}

// ---- the loop ------------------------------------------------------------------------------------
void TurretState::update(uint32_t nowMs) {
  lostThisTick_ = false;
  menuThisTick_ = false;
  forwardRumble();

  // The mode switch reaches ControlLink in every state (StatusPacket.wantMode mirrors it).
  const bool modeChanged = inputs_->radioModeChanged();
  if (modeChanged) link_->setWantMode(inputs_->radioMode());

  // Power toggle.
  if (state_ != DS_STATE_OFF && !inputs_->powerOn()) enterOff(nowMs);
  if (state_ == DS_STATE_OFF) {
    if (inputs_->powerOn()) {
      leaveOff(nowMs);           // -> BOOT; CALIBRATING follows on the next tick
    } else {
      stall_->takeStall();       // nothing moves in OFF: drop any late event
      link_->takeSourceChanged();
      wasConnected_ = link_->connected();
    }
    publish(nowMs);
    return;
  }

  // Link: a connected frame stopped arriving, the source changed, or the mode switch moved.
  const bool connected = link_->connected();
  const bool sourceChanged = link_->takeSourceChanged();
  if (connected) everConnected_ = true;
  if (sourceChanged)
    onLinkLost(nowMs, "source changed");
  else if (wasConnected_ && !connected)
    onLinkLost(nowMs, "no connected frame for LINK_TIMEOUT_MS");
  else if (modeChanged)
    onLinkLost(nowMs, "mode toggle");
  wasConnected_ = connected;

  // Stall events: TurretState is their only consumer.
  if (stall_->takeStall()) onStall(nowMs);

  switch (state_) {
    case DS_STATE_BOOT:
      enterCalibrating(nowMs);
      break;
    case DS_STATE_CALIBRATING: {
      anyStateButtons();
      if (inputs_->pressed(DS_BTN_A)) refuse(CmdResult::REFUSED_STATE, "A (arm) while calibrating");
      if (inputs_->pressed(DS_BTN_START))
        refuse(CmdResult::REFUSED_STATE, "Menu (servo adjust) while calibrating");
      const Motion::CalStatus cal = motion_->calibrationStatus();
      if (cal == Motion::CalStatus::DONE) {
        debug_->log(LOG_STATE, "calibration done: tilt 0 = bottom stop + TILT_CAL_BACKOFF_DEG");
        enter(DS_STATE_PAIRING, nowMs);
      } else if (cal != Motion::CalStatus::RUNNING) {
        Debug::Line l;
        l.add("calibration failed: ").add(calFailureName(motion_->calFailure()));
        debug_->log(LOG_ERROR, l);
        enterFault(DS_FAULT_CAL, nowMs);
      }
      break;
    }
    case DS_STATE_PAIRING:
      if (connected && !lostThisTick_) enter(DS_STATE_DISARMED, nowMs);   // never ARMED
      break;
    case DS_STATE_DISARMED:
    case DS_STATE_ARMED:
      controls(nowMs);
      break;
    case DS_STATE_FAULT:
      anyStateButtons();
      if (inputs_->pressed(DS_BTN_A)) refuse(CmdResult::REFUSED_STATE, "A (arm) in FAULT");
      if (inputs_->pressed(DS_BTN_START))
        refuse(CmdResult::REFUSED_STATE, "Menu (servo adjust) in FAULT");
      if (fault_ == DS_FAULT_STALL && elapsed(nowMs, faultSinceMs_, STALL_FAULT_HOLD_MS)) {
        fault_ = DS_FAULT_NONE;
        const bool toPairing = linkDroppedInFault_ || !connected;
        linkDroppedInFault_ = false;
        enter(toPairing ? (uint8_t)DS_STATE_PAIRING : stateBeforeFault_, nowMs);
      }
      break;
    default:
      break;
  }

  decideFlywheel(nowMs);
  publish(nowMs);
}

void TurretState::controls(uint32_t nowMs) {
  // Menu: SERVO ADJUST on (from DISARMED only) / save and off. A Menu press is the only button
  // this update acts on, so a button pressed together with it is neither an adjust action nor its
  // normal one, whether Menu switched the mode or was refused.
  menuThisTick_ = inputs_->pressed(DS_BTN_START);   // decideFlywheel ignores LB in this frame too
  if (menuThisTick_) {
    if (teach_) {
      const CmdResult r = saveAndLeaveTeach("Menu");
      if (r != CmdResult::OK) refuse(r, "Menu (save and leave servo adjust)");
    } else {
      const CmdResult r = enterTeach(nowMs, "Menu");
      if (r != CmdResult::OK) refuse(r, "Menu (servo adjust)");
    }
  } else if (teach_) {
    teachButtons(nowMs);
  } else {
    normalButtons(nowMs);
  }
  // The stick: any deflection past the dead zone takes over (and so cancels a go-home or a
  // shell move); a centred stick leaves a running position move alone. It aims in adjust mode too.
  if (inputs_->stickActive() || !motion_->positionMoveActive())
    motion_->setVelocity(-inputs_->stickX(), inputs_->stickY());   // pan + = counter-clockwise
}

void TurretState::normalButtons(uint32_t nowMs) {
  anyStateButtons();
  if (inputs_->pressed(DS_BTN_A)) {
    if (state_ == DS_STATE_ARMED) {
      disarm(nowMs);
    } else {
      const CmdResult r = tryArm(nowMs);
      if (r != CmdResult::OK) refuse(r, "A (arm)");
    }
  }
  if (inputs_->pressed(DS_BTN_DPAD_LEFT)) {
    motion_->setHomeHere();
    Debug::Line l;
    l.add("home set: pan ").add(motion_->homeDeg(AXIS_PAN), 1).add(" tilt ")
        .add(motion_->homeDeg(AXIS_TILT), 1);
    debug_->log(LOG_STATE, l);
  }
  if (inputs_->pressed(DS_BTN_DPAD_DOWN)) {
    if (motion_->goHome())
      debug_->log(LOG_STATE, "going home");
    else
      refuse(CmdResult::REFUSED_STATE, "d-pad down (go home)");
  }
}

void TurretState::anyStateButtons() {
  if (inputs_->pressed(DS_BTN_B)) {
    trigger_->toggleFireMode();
    debug_->log(LOG_STATE, trigger_->fireMode() == DS_FIRE_MODE_FULL ? "fire mode full" : "fire mode semi");
  }
  if (inputs_->pressed(DS_BTN_X)) {
    trigger_->resetMagazine();
    debug_->log(LOG_STATE, "magazine reset");
  }
}

bool TurretState::flywheelAllowed() const {
  if (teach_) return false;   // SERVO ADJUST: never, whatever FLYWHEEL_REQUIRES_ARMED says
  return state_ == DS_STATE_ARMED || (state_ == DS_STATE_DISARMED && !cfg_.flywheelRequiresArmed);
}

void TurretState::decideFlywheel(uint32_t nowMs) {
  // PAIRING keeps what the link loss left: cut by FAILSAFE_STOP_FLYWHEEL, or (with it false)
  // still spinning until the reconnection's DISARMED rules decide.
  if (state_ == DS_STATE_PAIRING) return;
  if (!flywheelAllowed()) {
    if (flywheel_->on()) flywheel_->cut();
    flyLatch_ = false;
    return;
  }
  if (!cfg_.flywheelHoldMode && inputs_->pressed(DS_BTN_LB) && !menuThisTick_)
    flyLatch_ = !flyLatch_;   // not in a frame with a Menu press (controls())
  const bool want = flyLatch_ || (cfg_.flywheelHoldMode && inputs_->held(DS_BTN_LB));
  if (want && !flywheel_->on())
    flywheel_->setOn(true, nowMs);
  else if (!want && flywheel_->on())
    flywheel_->cut();
}

// ---- transitions -----------------------------------------------------------------------------------
void TurretState::enter(uint8_t state, uint32_t nowMs) {
  if (state != DS_STATE_DISARMED) leaveTeach("state change");   // the callers below leave it first
  if (state != state_) transitions_++;
  state_ = state;
  stateSinceMs_ = nowMs;
  display_.state = state;
  display_.fault = fault_;
  display_.stateSinceMs = nowMs;
  link_->setTurretState(state);
  Debug::Line l;
  l.add("state ").add(ds_turret_state_name(state));
  if (state == DS_STATE_FAULT) l.add(' ').add(ds_fault_name(fault_));
  debug_->log(LOG_STATE, l);
}

void TurretState::enterOff(uint32_t nowMs) {
  motion_->haltNow();              // power off: no deceleration, a running calibration aborted
  motion_->enableDrivers(false);   // EN high
  flywheel_->cut();                // PWM 0 (after FLYWHEEL_BRAKE_MS when braking is configured)
  flyLatch_ = false;
  leaveTeach("power off");         // not saved; the summary still names the angles in force ...
  restoreBootAngles();             // ... then a new boot: the boot angles, servo at their rest
  trigger_->setServoPower(false);  // a running stroke is aborted first (rest, no decrement)
  fault_ = DS_FAULT_NONE;          // OFF clears every fault, CAL included
  linkDroppedInFault_ = false;
  enter(DS_STATE_OFF, nowMs);
}

void TurretState::leaveOff(uint32_t nowMs) {
  trigger_->setServoPower(true);   // Trigger's servo PWM resumes at rest
  motion_->rebootPosition();       // a new boot: pan zero = here, home = (0, 0) (spec 3.3, 3.5)
  lossSinceBoot_ = false;          // a new boot: PAIRING shows "<mode> Pairing" again
  wasConnected_ = link_->connected();
  enter(DS_STATE_BOOT, nowMs);
}

void TurretState::enterCalibrating(uint32_t nowMs) {
  leaveTeach("calibration");
  motion_->stop();                 // nothing else may keep moving (e.g. pan after shell `cal`)
  trigger_->abort();
  if (flywheel_->on()) flywheel_->cut();
  flyLatch_ = false;
  motion_->startCalibration(nowMs, cfg_.calMode);
  enter(DS_STATE_CALIBRATING, nowMs);
}

void TurretState::enterFault(uint8_t fault, uint32_t nowMs) {
  leaveTeach("fault");
  fault_ = fault;
  faultSinceMs_ = nowMs;
  trigger_->abort();               // fault mid-stroke: rest now, no decrement
  if (flywheel_->on()) flywheel_->cut();
  flyLatch_ = false;
  if (fault == DS_FAULT_CAL) motion_->stop();
  enter(DS_STATE_FAULT, nowMs);
}

void TurretState::onLinkLost(uint32_t nowMs, const char* why) {
  linkLosses_++;
  linkLostMs_ = nowMs;
  lossSinceBoot_ = true;
  lostThisTick_ = true;
  Debug::Line l;
  l.add("link lost: ").add(why);
  debug_->log(LOG_STATE, l);
  leaveTeach("link lost");
  trigger_->abort();               // stroke aborted: rest now, no decrement
  if (cfg_.failsafeStopFlywheel) {
    if (flywheel_->on()) flywheel_->cut();
    flyLatch_ = false;
  }
  switch (state_) {
    case DS_STATE_PAIRING:
    case DS_STATE_DISARMED:
    case DS_STATE_ARMED:
      motion_->stop();             // decelerate at the accel limit, never past a limit
      enter(DS_STATE_PAIRING, nowMs);
      break;
    case DS_STATE_FAULT:
      linkDroppedInFault_ = true;  // a stall fault then ends in PAIRING (its back-off finishes)
      break;
    default:
      break;                       // BOOT / CALIBRATING carry on (calibration is not stopped)
  }
}

void TurretState::onStall(uint32_t nowMs) {
  const uint8_t axes = stall_->stallAxes();
  switch (state_) {
    case DS_STATE_CALIBRATING:
      if (motion_->calibrationWantsStall()) {
        motion_->notifyCalibrationStall(nowMs);   // CAL_STALL_SENSE: the bottom stop is here
        debug_->log(LOG_STATE, "calibration: stall at the bottom stop");
      } else {
        debug_->log(LOG_VERBOSE, "stall ignored: calibration rams the stop");   // CAL_RAM
      }
      return;
    case DS_STATE_PAIRING:
    case DS_STATE_DISARMED:
    case DS_STATE_ARMED: {
      stalls_++;
      stateBeforeFault_ = state_ == DS_STATE_ARMED ? (uint8_t)DS_STATE_ARMED : (uint8_t)DS_STATE_DISARMED;
      linkDroppedInFault_ = state_ == DS_STATE_PAIRING;
      motion_->backOff(axes, STALL_BACKOFF_DEG);  // moving axes reverse, the others stop
      Debug::Line l;
      l.add("STALL axes ").addU(axes).add(" peak delta ").addU(stall_->lastStallPeak())
          .add(" counts: backing off");
      debug_->log(LOG_ERROR, l);
      enterFault(DS_FAULT_STALL, nowMs);
      return;
    }
    case DS_STATE_FAULT:
      if (fault_ == DS_FAULT_STALL) {   // the back-off ran into something too: just stop
        stalls_++;
        motion_->stop();
        faultSinceMs_ = nowMs;
        debug_->log(LOG_ERROR, "STALL again during the fault: stopped");
      }
      return;
    default:
      return;                           // OFF / BOOT: nothing is moving
  }
}

// ---- permissions shared by the controller and the shell ------------------------------------------
TurretState::CmdResult TurretState::tryArm(uint32_t nowMs) {
  if (state_ == DS_STATE_ARMED) return CmdResult::OK;
  if (state_ != DS_STATE_DISARMED) return CmdResult::REFUSED_STATE;
  if (teach_) return CmdResult::REFUSED_TEACHING;   // SERVO ADJUST stays DISARMED
  if (cfg_.armRequiresTriggerReleased && inputs_->triggerPulled())
    return CmdResult::REFUSED_TRIGGER_HELD;
  enter(DS_STATE_ARMED, nowMs);
  return CmdResult::OK;
}

void TurretState::disarm(uint32_t nowMs) {
  if (state_ != DS_STATE_ARMED) return;
  trigger_->abort();               // disarm mid-stroke: rest now, no decrement
  if (cfg_.flywheelRequiresArmed) {
    if (flywheel_->on()) flywheel_->cut();
    flyLatch_ = false;
  }
  enter(DS_STATE_DISARMED, nowMs);
}

TurretState::CmdResult TurretState::moveAxis(Axis axis, float deg) {
  if (state_ != DS_STATE_DISARMED && state_ != DS_STATE_ARMED) return CmdResult::REFUSED_STATE;
  if (!finiteDeg(deg)) return CmdResult::REFUSED_RANGE;
  if (axis == AXIS_TILT && !motion_->calibrated()) return CmdResult::REFUSED_NOT_CALIBRATED;
  if (motion_->limitsActive(axis) &&
      (deg < motion_->limitMinDeg(axis) || deg > motion_->limitMaxDeg(axis)))
    return CmdResult::REFUSED_RANGE;
  return motion_->moveTo(axis, deg) ? CmdResult::OK : CmdResult::REFUSED_STATE;
}

TurretState::CmdResult TurretState::refuse(CmdResult why, const char* what) {
  refusals_++;
  lastRefusal_ = why;
  Debug::Line l;
  l.add(what).add(": ").add(resultName(why));
  debug_->log(LOG_STATE, l);
  return why;
}

TurretState::CmdResult TurretState::shellCommand(ShellCmd cmd, float arg, uint32_t nowMs) {
  CmdResult r = CmdResult::OK;
  const bool controlState = state_ == DS_STATE_DISARMED || state_ == DS_STATE_ARMED;
  // SERVO ADJUST blocks what its pad buttons no longer do (A, B, X, d-pad left / down, LB) and
  // firing; aiming, stopping, disarm and fly off still work.
  const bool blockedByTeach =
      teach_ && (cmd == ShellCmd::ARM || cmd == ShellCmd::FIRE || cmd == ShellCmd::FLY_ON ||
                 cmd == ShellCmd::MODE_SEMI || cmd == ShellCmd::MODE_FULL || cmd == ShellCmd::MAG ||
                 cmd == ShellCmd::HOME || cmd == ShellCmd::SETHOME);
  if (state_ == DS_STATE_OFF && cmd != ShellCmd::STOP) {
    r = CmdResult::REFUSED_STATE;   // power off: the turret takes no orders
  } else if (blockedByTeach) {
    r = CmdResult::REFUSED_TEACHING;
  } else {
    switch (cmd) {
      case ShellCmd::ARM:
        r = tryArm(nowMs);
        break;
      case ShellCmd::DISARM:
        disarm(nowMs);              // a no-op unless ARMED
        break;
      case ShellCmd::FIRE: {
        const Trigger::Input in = freshTriggerInput(nowMs);
        if (!trigger_->fireOnce(nowMs, in)) {
          switch (trigger_->gate(in)) {
            case Trigger::Gate::EMPTY: r = CmdResult::REFUSED_EMPTY; break;
            case Trigger::Gate::STROKE_RUNNING:
            case Trigger::Gate::FLYWHEEL_NOT_READY: r = CmdResult::REFUSED_NOT_READY; break;
            case Trigger::Gate::COOLDOWN: r = CmdResult::REFUSED_COOLDOWN; break;
            default: r = CmdResult::REFUSED_STATE; break;   // NOT_ARMED, SERVO_OFF
          }
        }
        break;
      }
      case ShellCmd::FLY_ON:
        if (!flywheelAllowed()) {
          r = CmdResult::REFUSED_STATE;
        } else {
          flyLatch_ = true;
          flywheel_->setOn(true, nowMs);
        }
        break;
      case ShellCmd::FLY_OFF:       // releases the latch; in hold mode a held LB still spins
        flyLatch_ = false;
        if (flywheel_->on()) flywheel_->cut();
        break;
      case ShellCmd::MODE_SEMI:
        trigger_->setFireMode(DS_FIRE_MODE_SEMI);
        break;
      case ShellCmd::MODE_FULL:
        trigger_->setFireMode(DS_FIRE_MODE_FULL);
        break;
      case ShellCmd::MAG:
        trigger_->resetMagazine();
        break;
      case ShellCmd::PAN:
        r = moveAxis(AXIS_PAN, arg);
        break;
      case ShellCmd::TILT:
        r = moveAxis(AXIS_TILT, arg);
        break;
      case ShellCmd::HOME:
        r = controlState && motion_->goHome() ? CmdResult::OK : CmdResult::REFUSED_STATE;
        break;
      case ShellCmd::SETHOME:
        if (controlState)
          motion_->setHomeHere();
        else
          r = CmdResult::REFUSED_STATE;
        break;
      case ShellCmd::STOP:          // any state: it only ever stops
        motion_->stop();
        break;
      case ShellCmd::CAL:
        if (state_ == DS_STATE_DISARMED || state_ == DS_STATE_PAIRING)
          enterCalibrating(nowMs);   // leaves SERVO ADJUST first
        else
          r = CmdResult::REFUSED_STATE;
        break;
      case ShellCmd::SERVO_TEACH:   // the pad's Menu: enter, or save and leave
        r = teach_ ? saveAndLeaveTeach("shell") : enterTeach(nowMs, "shell");
        break;
      case ShellCmd::SERVO_SET:
        if (!teach_)
          r = CmdResult::REFUSED_NOT_TEACHING;
        else if (!finiteDeg(arg) || arg < (float)SERVO_ADJUST_MIN_DEG || arg > (float)SERVO_ADJUST_MAX_DEG)
          r = CmdResult::REFUSED_RANGE;
        else
          r = teachGoTo((int32_t)lroundf(arg), false, nowMs);
        break;
      case ShellCmd::SERVO_REST:    // the pad's A in adjust mode
        r = teachRecord(false);
        break;
      case ShellCmd::SERVO_FIRE:    // B
        r = teachRecord(true);
        break;
      case ShellCmd::SERVO_RESET:   // View
        r = teachRestoreConfig(nowMs);
        break;
      case ShellCmd::SERVO_STROKE:  // X
        r = teachTestStroke(nowMs);
        break;
      case ShellCmd::SERVO_OFF:     // save and leave; a no-op when not in adjust mode
        r = saveAndLeaveTeach("shell");
        break;
    }
  }
  if (r != CmdResult::OK) {
    refusals_++;
    lastRefusal_ = r;
  }
  publish(nowMs);
  return r;
}

const char* TurretState::resultName(CmdResult r) {
  switch (r) {
    case CmdResult::OK: return "ok";
    case CmdResult::REFUSED_STATE: return "refused: not in this state";
    case CmdResult::REFUSED_TRIGGER_HELD: return "refused: trigger held";
    case CmdResult::REFUSED_NOT_READY: return "refused: not ready";
    case CmdResult::REFUSED_EMPTY: return "refused: magazine empty";
    case CmdResult::REFUSED_RANGE: return "refused: outside the limits";
    case CmdResult::REFUSED_NOT_CALIBRATED: return "refused: not calibrated";
    case CmdResult::REFUSED_TEACHING: return "refused: in servo adjust (Menu or servo off leaves it)";
    case CmdResult::REFUSED_NOT_TEACHING: return "refused: not in servo adjust (Menu or servo adjust)";
    case CmdResult::REFUSED_COOLDOWN: return "refused: semi-auto cooldown after the last shot";
    case CmdResult::REFUSED_MOVING: return "refused: turret moving (centre the stick, then save)";
  }
  return "?";
}

// ---- outputs ------------------------------------------------------------------------------------------
void TurretState::forwardRumble() {
  Trigger::Rumble r;
  if (trigger_->takeRumbleRequest(r)) link_->requestRumble(r.strong, r.weak, r.ms);
}

Trigger::Input TurretState::freshTriggerInput(uint32_t nowMs) const {
  Trigger::Input in;
  in.armed = state_ == DS_STATE_ARMED;
  in.pulled = inputs_->triggerPulled();
  in.flywheelReady = flywheel_->ready(nowMs);
  in.servoTeach = teach_;
  return in;
}

PairingPhase TurretState::pairingPhase(uint32_t nowMs) const {
  if (!lossSinceBoot_) return PairingPhase::FROM_BOOT;
  return elapsed(nowMs, linkLostMs_, LINK_RECONNECT_GRACE_MS) ? PairingPhase::NO_LINK
                                                                : PairingPhase::GRACE;
}

void TurretState::publish(uint32_t nowMs) {
  triggerInput_ = freshTriggerInput(nowMs);
  display_.state = state_;
  display_.fault = fault_;
  display_.fireMode = trigger_->fireMode();
  display_.radioMode = inputs_->radioMode();
  display_.ammo = trigger_->ammo();
  display_.linkConnected = link_->connected();
  // ARMED red / yellow: would a pull fire now? Trigger asks its own gates (all of them) and the
  // last shot's LED hold. Published before Trigger::update() runs, so in full auto the stroke that
  // starts as the last one completes is already running in the next publish: no red frame between.
  display_.fireReady = trigger_->fireReady(nowMs, triggerInput_);
  // The empty-magazine purple double flash: ammo 0 while DISARMED or ARMED, until X; SERVO ADJUST's
  // cyan wins over it. Display only: firing at 0 is Trigger's EMPTY gate (FIRE_LOCKOUT_WHEN_EMPTY).
  display_.emptyMagBlink =
      (state_ == DS_STATE_DISARMED || state_ == DS_STATE_ARMED) && !teach_ && trigger_->ammo() == 0;
  display_.pairing = pairingPhase(nowMs);
  display_.stateSinceMs = stateSinceMs_;
  display_.servoTeach = teach_;
  display_.servoDeg = commandedDeg();
  display_.restDeg = wholeDeg(trigger_->restDeg());
  display_.fireDeg = wholeDeg(trigger_->fireDeg());
  link_->setTurretState(state_);
}

TurretState::Status TurretState::status() const {
  Status s;
  s.state = state_;
  s.fault = fault_;
  s.stateSinceMs = stateSinceMs_;
  s.pairing = display_.pairing;
  s.linkConnected = link_ ? link_->connected() : false;
  s.flywheelAllowed = flywheelAllowed();
  s.flywheelLatch = flyLatch_;
  s.stateBeforeFault = stateBeforeFault_;
  s.linkDroppedInFault = linkDroppedInFault_;
  s.faultSinceMs = faultSinceMs_;
  s.linkLosses = linkLosses_;
  s.stalls = stalls_;
  s.refusals = refusals_;
  s.lastRefusal = lastRefusal_;
  s.transitions = transitions_;
  s.servoTeach = teach_;
  if (trigger_) {
    s.servoDeg = commandedDeg();
    s.restDeg = wholeDeg(trigger_->restDeg());
    s.fireDeg = wholeDeg(trigger_->fireDeg());
  }
  s.bootAngles = bootAngles_;
  s.savedRecord = savedRecord_;
  s.saves = saves_;
  s.saveFailures = saveFailures_;
  return s;
}

// ---- SERVO ADJUST (TurretState.h) --------------------------------------------------------------------
int16_t TurretState::commandedDeg() const { return wholeDeg(trigger_->servoDeg()); }

servostore::Angles TurretState::anglesInForce() const {
  servostore::Angles a;
  a.rest = wholeDeg(trigger_->restDeg());
  a.fire = wholeDeg(trigger_->fireDeg());
  return a;
}

namespace {
// "REST <n> FIRE <m>"
void addAngles(Debug::Line& l, const servostore::Angles& a) {
  l.add(kRestWord).add((int32_t)a.rest).add(kFireWord).add((int32_t)a.fire);
}
}  // namespace

void TurretState::loadBootAngles() {
  servostore::Angles saved;
  const servostore::Load r = servostore::load(saved);
  const servostore::Angles cfgAngles = servostore::configAngles();
  savedRecord_ = r == servostore::Load::OK;
  bootAngles_ = savedRecord_ ? saved : cfgAngles;
  trigger_->setAngles((float)bootAngles_.rest, (float)bootAngles_.fire);
  trigger_->abort();   // idle: commands the boot REST (the PWM resumes there)
  Debug::Line l;
  l.add(kTeachTag);
  if (savedRecord_) {
    l.add("loaded saved ");
    addAngles(l, bootAngles_);
    if (!servostore::sameAngles(bootAngles_, cfgAngles)) {
      l.add(" (config.h has ");
      addAngles(l, cfgAngles);
      l.add(", not used)");
    }
  } else {
    l.add("no saved angles, using config.h ");
    addAngles(l, bootAngles_);
    if (r == servostore::Load::INVALID) l.add(" (stored record invalid)");
  }
  debug_->logAlways(l);
}

TurretState::CmdResult TurretState::enterTeach(uint32_t nowMs, const char* via) {
  if (teach_) return CmdResult::OK;
  if (state_ != DS_STATE_DISARMED) return CmdResult::REFUSED_STATE;   // never from ARMED
  if (trigger_->strokeActive()) return CmdResult::REFUSED_NOT_READY;
  teach_ = true;
  if (flywheel_->on()) flywheel_->cut();   // DISARMED with the flywheels off, whatever the flags
  flyLatch_ = false;
  const int32_t start = clampTeach(wholeDeg(trigger_->restDeg()));
  trigger_->teachMove((float)start, freshTriggerInput(nowMs));
  Debug::Line l;
  l.add(kTeachTag).add("on (").add(via).add(") at ").add(start).add(" deg, ");
  addAngles(l, anglesInForce());
  l.add("; A=REST B=FIRE X=test View=config.h Menu=save");
  debug_->log(LOG_STATE, l);
  return CmdResult::OK;
}

TurretState::CmdResult TurretState::saveAndLeaveTeach(const char* via) {
  if (!teach_) return CmdResult::OK;
  // The save blocks loop(), the motion planner with it: never while an axis moves.
  if (motion_->isMoving()) return CmdResult::REFUSED_MOVING;
  leaveTeach(via, true);
  return CmdResult::OK;
}

void TurretState::leaveTeach(const char* why, bool save) {
  if (!teach_) return;
  teach_ = false;
  trigger_->abort();   // a test stroke cancelled; the servo back to the REST in force
  const servostore::Angles a = anglesInForce();
  Debug::Line l;
  l.add(kTeachTag).add("off (").add(why).add("), servo at REST ").add((int32_t)a.rest);
  debug_->log(LOG_STATE, l);
  if (save) {
    saveAngles(a);
  } else if (!servostore::sameAngles(a, bootAngles_)) {
    l.clear();
    l.add(kTeachTag).add("not saved (").add(why).add("): ");
    addAngles(l, a);
    l.add(" stay for this run only");
    debug_->logAlways(l);
  }
  l.clear();
  l.add(kSummaryHead).add(kRestDefine).add((int32_t)a.rest).add(kSummarySep).add(kFireDefine)
      .add((int32_t)a.fire);
  debug_->logAlways(l);
}

void TurretState::saveAngles(const servostore::Angles& a) {
  Debug::Line l;
  l.add(kTeachTag);
  if (servostore::sameAngles(a, bootAngles_)) {
    addAngles(l, a);
    l.add(" unchanged, nothing written");
    debug_->logAlways(l);
    return;
  }
  // The one blocking call in loop(): hal::storeSave erases and programs a 1 KB data flash block,
  // about 50 ms (hal.h; the datasheet maximum is 0.6-0.75 s). The step interrupt, the servo PWM and
  // the UART receive rings keep running; the planner, the LEDs, the LCD and the link polls wait.
  // Allowed only here: DISARMED, no stroke running, no axis moving (saveAndLeaveTeach and the abort
  // in leaveTeach make sure; checked again so no other caller can save mid-stroke or mid-move).
  if (state_ != DS_STATE_DISARMED || trigger_->strokeActive() || motion_->isMoving()) {
    l.add("not saved (turret busy): ");
    addAngles(l, a);
    l.add(" stay for this run only");
    debug_->logAlways(l);
    return;
  }
  const uint32_t startUs = hal::nowUs();
  const bool ok = servostore::save(a);
  const uint32_t tookMs = (hal::nowUs() - startUs) / US_PER_MS;
  if (ok) {
    saves_++;
    savedRecord_ = true;
    bootAngles_ = a;
    l.add("saved ");
    addAngles(l, a);
    l.add(" (survives power off), ").addU(tookMs).add(" ms");
  } else {
    saveFailures_++;
    l.add("save FAILED (data flash read-back), ");
    addAngles(l, a);
    l.add(" stay for this run only");
  }
  debug_->logAlways(l);
}

TurretState::CmdResult TurretState::teachGoTo(int32_t deg, bool clamp, uint32_t nowMs) {
  if (!teach_) return CmdResult::REFUSED_NOT_TEACHING;
  if (trigger_->strokeActive()) return CmdResult::REFUSED_NOT_READY;   // the test stroke runs
  const int32_t to = clampTeach(deg);
  if (to != deg && !clamp) return CmdResult::REFUSED_RANGE;
  if (!trigger_->teachMove((float)to, freshTriggerInput(nowMs))) return CmdResult::REFUSED_NOT_READY;
  Debug::Line l;
  l.add(kTeachTag).add(to).add(" deg");
  if (to != deg) l.add(" (limit)");
  debug_->log(LOG_STATE, l);
  return CmdResult::OK;
}

TurretState::CmdResult TurretState::teachRecord(bool fire) {
  if (!teach_) return CmdResult::REFUSED_NOT_TEACHING;
  if (trigger_->strokeActive()) return CmdResult::REFUSED_NOT_READY;   // mid test stroke
  const int32_t deg = commandedDeg();
  if (fire)
    trigger_->setAngles(trigger_->restDeg(), (float)deg);
  else
    trigger_->setAngles((float)deg, trigger_->fireDeg());
  Debug::Line l;
  l.add(kTeachTag).add(fire ? "FIRE" : "REST").add(" recorded -> ").add(fire ? kFireDefine : kRestDefine)
      .add(deg);
  debug_->logAlways(l);
  return CmdResult::OK;
}

TurretState::CmdResult TurretState::teachRestoreConfig(uint32_t nowMs) {
  if (!teach_) return CmdResult::REFUSED_NOT_TEACHING;
  if (trigger_->strokeActive()) return CmdResult::REFUSED_NOT_READY;   // mid test stroke
  const servostore::Angles cfgAngles = servostore::configAngles();
  trigger_->setAngles((float)cfgAngles.rest, (float)cfgAngles.fire);
  trigger_->teachMove((float)clampTeach(cfgAngles.rest), freshTriggerInput(nowMs));
  Debug::Line l;
  l.add(kTeachTag).add("config.h angles restored: ");
  addAngles(l, cfgAngles);
  l.add(" (Menu saves them)");
  debug_->logAlways(l);
  return CmdResult::OK;
}

TurretState::CmdResult TurretState::teachTestStroke(uint32_t nowMs) {
  if (!teach_) return CmdResult::REFUSED_NOT_TEACHING;
  // It sweeps between the angles in force; config.h's (View) may lie outside the adjust range.
  const servostore::Angles a = anglesInForce();
  if (clampTeach(a.rest) != a.rest || clampTeach(a.fire) != a.fire) return CmdResult::REFUSED_RANGE;
  if (!trigger_->testStroke(nowMs, freshTriggerInput(nowMs))) return CmdResult::REFUSED_NOT_READY;
  Debug::Line l;
  l.add(kTeachTag).add("test stroke REST ").add((int32_t)a.rest).add(" -> FIRE ").add((int32_t)a.fire)
      .add(" -> REST (no dart counted)");
  debug_->log(LOG_STATE, l);
  return CmdResult::OK;
}

void TurretState::teachButtons(uint32_t nowMs) {
  struct Nudge {
    uint16_t button;
    int32_t deg;
    const char* name;
  };
  const Nudge nudges[] = {
      {DS_BTN_DPAD_UP, SERVO_ADJUST_STEP_SMALL_DEG, "d-pad up (servo +)"},
      {DS_BTN_DPAD_DOWN, -SERVO_ADJUST_STEP_SMALL_DEG, "d-pad down (servo -)"},
      {DS_BTN_RB, SERVO_ADJUST_STEP_LARGE_DEG, "RB (servo ++)"},
      {DS_BTN_LB, -SERVO_ADJUST_STEP_LARGE_DEG, "LB (servo --)"},
  };
  for (const Nudge& n : nudges) {
    if (!inputs_->pressed(n.button)) continue;
    const CmdResult r = teachGoTo(commandedDeg() + n.deg, true, nowMs);
    if (r != CmdResult::OK) refuse(r, n.name);
  }
  if (inputs_->pressed(DS_BTN_A)) {
    const CmdResult r = teachRecord(false);
    if (r != CmdResult::OK) refuse(r, "A (record REST)");
  }
  if (inputs_->pressed(DS_BTN_B)) {
    const CmdResult r = teachRecord(true);
    if (r != CmdResult::OK) refuse(r, "B (record FIRE)");
  }
  if (inputs_->pressed(DS_BTN_BACK)) {
    const CmdResult r = teachRestoreConfig(nowMs);
    if (r != CmdResult::OK) refuse(r, "View (config.h angles)");
  }
  if (inputs_->pressed(DS_BTN_X)) {
    const CmdResult r = teachTestStroke(nowMs);
    if (r != CmdResult::OK) refuse(r, "X (test stroke)");
  }
  // d-pad left / right: nothing in adjust mode (no set home).
}

void TurretState::restoreBootAngles() {
  if (!servostore::sameAngles(anglesInForce(), bootAngles_)) {
    Debug::Line l;
    l.add(kTeachTag).add("unsaved angles dropped (power off): back to ").add(savedRecord_ ? "saved " : "config.h ");
    addAngles(l, bootAngles_);
    debug_->log(LOG_STATE, l);
  }
  trigger_->setAngles((float)bootAngles_.rest, (float)bootAngles_.fire);
  trigger_->abort();   // idle here: commands the boot REST, where the PWM resumes at power-on
}

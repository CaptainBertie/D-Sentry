// Debug.cpp -- see Debug.h.
//
// SPDX-License-Identifier: MIT
#include "Debug.h"

#include <math.h>
#include <string.h>

#include "ControlLink.h"
#include "Flywheel.h"
#include "Inputs.h"
#include "Lcd.h"
#include "Leds.h"
#include "Motion.h"
#include "StallDetector.h"
#include "Trigger.h"
#include "TurretState.h"
#include "hal.h"

static_assert(DEBUG_STATS_INTERVAL_MS > 0, "DEBUG_STATS_INTERVAL_MS must be positive");
static_assert(STALL_LOG_HZ > 0 && STALL_LOG_HZ <= MS_PER_S, "STALL_LOG_HZ must be 1..1000");
static_assert(DEBUG_LOG_LEVEL >= LOG_OFF && DEBUG_LOG_LEVEL <= LOG_VERBOSE, "DEBUG_LOG_LEVEL is 0..3");

// ---- Line ------------------------------------------------------------------------------------
void Debug::Line::clear() {
  len_ = 0;
  buf_[0] = 0;
  truncated_ = false;
}

Debug::Line& Debug::Line::add(char c) {
  if (len_ < DEBUG_LOG_LINE_MAX) {
    buf_[len_++] = c;
    buf_[len_] = 0;
  } else {
    truncated_ = true;
  }
  return *this;
}

Debug::Line& Debug::Line::add(const char* text) {
  if (!text) return *this;
  while (*text) add(*text++);
  return *this;
}

Debug::Line& Debug::Line::addU(uint32_t v) {
  char digits[sizeof(uint32_t) * BITS_PER_BYTE];   // more than the 10 a uint32_t needs
  size_t n = 0;
  do {
    digits[n++] = (char)('0' + v % DEC_BASE);
    v /= DEC_BASE;
  } while (v && n < sizeof digits);
  while (n) add(digits[--n]);
  return *this;
}

Debug::Line& Debug::Line::add(int32_t v) {
  if (v < 0) {
    add('-');
    // -(v + 1) + 1 avoids overflowing on INT32_MIN
    return addU((uint32_t)(-(v + 1)) + 1u);
  }
  return addU((uint32_t)v);
}

Debug::Line& Debug::Line::add(float v, uint8_t decimals) {
  if (v != v) return add("nan");
  if (v < 0.0f) {
    add('-');
    v = -v;
  }
  uint32_t scale = 1;
  for (uint8_t i = 0; i < decimals; i++) scale *= DEC_BASE;
  float scaled = v * (float)scale + MATH_HALF;          // round half up
  if (scaled >= (float)UINT32_MAX) return add("big");
  uint32_t fixed = (uint32_t)scaled;
  addU(fixed / scale);
  if (decimals) {
    add('.');
    uint32_t frac = fixed % scale;
    for (uint32_t div = scale / DEC_BASE; div; div /= DEC_BASE) {
      add((char)('0' + (frac / div) % DEC_BASE));
    }
  }
  return *this;
}

Debug::Line& Debug::Line::addHex(uint32_t v, uint8_t digits) {
  static const char kHex[] = "0123456789abcdef";
  const uint8_t maxDigits = (uint8_t)(sizeof(uint32_t) * BITS_PER_BYTE / HEX_DIGIT_BITS);
  if (digits > maxDigits) digits = maxDigits;
  for (uint8_t i = digits; i > 0; i--) {
    add(kHex[(v >> ((i - 1u) * HEX_DIGIT_BITS)) % HEX_BASE]);
  }
  return *this;
}

Debug::Line& Debug::Line::addBool(bool v) { return add(v ? '1' : '0'); }

// ---- shell words -----------------------------------------------------------------------------
namespace {

struct Word {
  const char* p = "";
  size_t n = 0;
};

inline bool isBlank(char c) { return c == ' ' || c == '\t' || c == '\r'; }

// The next blank-separated word of *s (empty at the end); *s moves past it.
Word nextWord(const char*& s) {
  while (*s && isBlank(*s)) s++;
  Word w;
  w.p = s;
  while (*s && !isBlank(*s)) s++;
  w.n = (size_t)(s - w.p);
  return w;
}

bool is(const Word& w, const char* name) {
  const size_t n = strlen(name);
  return w.n == n && memcmp(w.p, name, n) == 0;
}

void addWord(Debug::Line& l, const Word& w) {
  for (size_t i = 0; i < w.n; i++) l.add(w.p[i]);
}

// A plain decimal number, the whole word: [+-]digits[.digits] or [+-].digits. No exponent,
// no hex, no trailing junk ("30deg" is refused, so a typo never moves the turret).
bool parseDeg(const Word& w, float& out) {
  size_t i = 0;
  bool neg = false;
  if (i < w.n && (w.p[i] == '-' || w.p[i] == '+')) neg = w.p[i++] == '-';
  float v = 0.0f;
  float scale = 1.0f;
  bool digits = false;
  bool point = false;
  for (; i < w.n; i++) {
    const char c = w.p[i];
    if (c == '.' && !point) {
      point = true;
      continue;
    }
    if (c < '0' || c > '9') return false;
    digits = true;
    if (point) {
      scale /= (float)DEC_BASE;
      v += (float)(c - '0') * scale;
    } else {
      v = v * (float)DEC_BASE + (float)(c - '0');
    }
  }
  if (!digits) return false;
  out = neg ? -v : v;
  return true;
}

struct Action {
  const char* name;
  TurretState::ShellCmd cmd;
};
// The actions that take no argument.
const Action kPlainActions[] = {
    {"arm", TurretState::ShellCmd::ARM},         {"disarm", TurretState::ShellCmd::DISARM},
    {"fire", TurretState::ShellCmd::FIRE},       {"mag", TurretState::ShellCmd::MAG},
    {"home", TurretState::ShellCmd::HOME},       {"sethome", TurretState::ShellCmd::SETHOME},
    {"stop", TurretState::ShellCmd::STOP},       {"cal", TurretState::ShellCmd::CAL},
};

// The `servo` words that take no number (SERVO ADJUST; "teach" is its old name, kept as an alias);
// `servo <whole degrees>` is SERVO_SET.
const Action kServoActions[] = {
    {"adjust", TurretState::ShellCmd::SERVO_TEACH},  {"teach", TurretState::ShellCmd::SERVO_TEACH},
    {"rest", TurretState::ShellCmd::SERVO_REST},     {"fire", TurretState::ShellCmd::SERVO_FIRE},
    {"reset", TurretState::ShellCmd::SERVO_RESET},   {"stroke", TurretState::ShellCmd::SERVO_STROKE},
    {"off", TurretState::ShellCmd::SERVO_OFF},
};

const char* onOff(bool v) { return v ? "on" : "off"; }

uint16_t permille(uint32_t part, uint32_t whole) {
  if (whole == 0) return 0;
  const uint64_t pm = (uint64_t)part * PERMILLE / whole;
  return pm > PERMILLE ? (uint16_t)PERMILLE : (uint16_t)pm;
}

}  // namespace

// ---- Debug -----------------------------------------------------------------------------------
void Debug::begin(const DebugSources& sources) {
  src_ = sources;
  level_ = DEBUG_LOG_LEVEL;
  stats_ = RuntimeStats();
  windowStartMs_ = hal::nowMs();
  loopsInWindow_ = 0;
  loopMaxUsInWindow_ = 0;
  linesLogged_ = 0;
  linesDropped_ = 0;
  lastStallLogMs_ = windowStartMs_;
  warnings_ = 0;
  shellCommands_ = 0;
  shellRejected_ = 0;
  linkAtStart_ = LinkStats();
  forcedShowsAtStart_ = 0;
  usbDropsAtStart_ = hal::usbTxDropped();
  begun_ = true;
  StepStats discard;
  hal::stepTakeStats(discard);   // start the first window now
}

void Debug::bootBanner(bool halOk) {
  const hal::BeginReport& r = hal::beginReport();
  Line l;
  l.add("D-Sentry RA4M1 boot: pan timer ").addBool(r.panTimer).add(" tilt timer ").addBool(r.tiltTimer)
      .add(" servo ").addBool(r.servo).add(" lcd ").addBool(r.lcd);
  if (r.lcd) l.add(" 0x").addHex(r.lcdAddress, sizeof(uint8_t) * BITS_PER_BYTE / HEX_DIGIT_BITS);
  log(halOk ? LOG_STATE : LOG_ERROR, l);
}

void Debug::warn(const char* text) {
  warnings_++;
  Line l;
  l.add("warning: ").add(text);
  log(LOG_ERROR, l);
}

void Debug::bootWarnings() {
  const hal::BeginReport& r = hal::beginReport();
  if (!r.panTimer || !r.tiltTimer) warn("a step timer was not claimed: that axis cannot move");
  if (!r.servo) warn("the trigger servo PWM did not start: no firing");
  // architecture.md 3: both LCD addresses silent -> run without the LCD and log an error.
  if (!r.lcd) warn("no LCD at LCD_I2C_ADDR or LCD_I2C_ADDR_FALLBACK: running without it");
  if (src_.lcd && src_.lcd->overflowWarnings()) {
    Line l;
    l.addU(src_.lcd->overflowWarnings()).add(" LCD text combination(s) will be cut (listed above)");
    warn(l.c_str());
  }
  if (src_.turret && !src_.turret->config().failsafeDisarmOnLoss)
    warn("FAILSAFE_DISARM_ON_LOSS false is ignored: a reconnect always lands in DISARMED");
}

void Debug::update(uint32_t nowMs, uint32_t /*nowUs*/) {
  if (!begun_) return;
  if (level_ >= LOG_VERBOSE && (uint32_t)(nowMs - lastStallLogMs_) >= MS_PER_S / STALL_LOG_HZ) {
    lastStallLogMs_ = nowMs;
    stallTrace();
  }
  const uint32_t elapsed = nowMs - windowStartMs_;
  if (elapsed < DEBUG_STATS_INTERVAL_MS) return;
  closeWindow(nowMs);
  if (enabled(LOG_STATE)) {
    Line l;
    statsLine(l);
    log(LOG_STATE, l);
  }
}

void Debug::closeWindow(uint32_t nowMs) {
  const uint32_t elapsed = nowMs - windowStartMs_;
  stats_.windowMs = elapsed;
  uint32_t hz = elapsed ? (uint32_t)((uint64_t)loopsInWindow_ * MS_PER_S / elapsed) : 0;
  stats_.loopHz = hz > UINT16_MAX ? UINT16_MAX : (uint16_t)hz;
  stats_.loopMaxUs = loopMaxUsInWindow_;
  hal::stepTakeStats(stats_.steps);
  uint16_t worst = 0;
  uint64_t weighted = 0;
  uint32_t samples = 0;
  for (uint8_t a = 0; a < AXIS_COUNT; a++) {
    if (stats_.steps.jitterMaxPermille[a] > worst) worst = stats_.steps.jitterMaxPermille[a];
    weighted += (uint64_t)stats_.steps.jitterMeanPermille[a] * stats_.steps.samples[a];
    samples += stats_.steps.samples[a];
  }
  stats_.jitterMaxPermille = worst;
  stats_.jitterMeanPermille = samples ? (uint16_t)(weighted / samples) : 0;
  stats_.jitterOverLimit = worst > STEP_JITTER_LIMIT_PERMILLE;
  stats_.usbTxDropped = hal::usbTxDropped();
  stats_.linkTxDropped = hal::linkTxDropped();
  stats_.usbTxDrops = stats_.usbTxDropped - usbDropsAtStart_;
  usbDropsAtStart_ = stats_.usbTxDropped;

  if (src_.link) {
    const LinkStats& now = src_.link->stats();
    const LinkStats& was = linkAtStart_;
    stats_.ctrlRxHz = now.ctrlRxHz;
    stats_.frames = now.frames - was.frames;
    stats_.pollTimeouts = now.pollTimeouts - was.pollTimeouts;
    stats_.crcErrors = now.crcErrors - was.crcErrors;
    stats_.seqGaps = now.seqGaps - was.seqGaps;
    if (src_.link->status().activeSource == DS_LINK_SRC_USB) {
      const uint32_t usb = now.usbFrames - was.usbFrames;
      stats_.lossPermille = permille(stats_.seqGaps, usb + stats_.seqGaps);
    } else {
      const uint32_t failed = stats_.pollTimeouts + (now.pollErrors - was.pollErrors);
      stats_.lossPermille = permille(failed, now.polls - was.polls);
    }
    linkAtStart_ = now;
  }
  stats_.lossOverLimit = stats_.lossPermille >= PACKET_LOSS_LIMIT_PERMILLE;
  if (src_.leds) {
    stats_.ledForcedShows = src_.leds->forcedShows() - forcedShowsAtStart_;
    forcedShowsAtStart_ = src_.leds->forcedShows();
  }
  windowStartMs_ = nowMs;
  loopsInWindow_ = 0;
  loopMaxUsInWindow_ = 0;
}

void Debug::statsLine(Line& l) const {
  const RuntimeStats& s = stats_;
  l.add("stats loop=").addU(s.loopHz).add(" rx=").addU(s.ctrlRxHz).add(" to=").addU(s.pollTimeouts)
      .add(" crc=").addU(s.crcErrors).add(" gap=").addU(s.seqGaps).add(" loss=")
      .add((float)s.lossPermille / (float)(PERMILLE / PERCENT), 1).add('%');
  if (s.lossOverLimit) l.add('!');
  l.add(" jit=").addU(s.jitterMaxPermille).add('/').addU(s.jitterMeanPermille).add("pm");
  if (s.jitterOverLimit) l.add('!');
  l.add(" led=").addU(s.ledForcedShows).add(" drop=").addU(s.usbTxDrops);
}

void Debug::stallTrace() {
  if (!src_.stall) return;
  StallTrace t;
  if (!src_.stall->takeTrace(t)) return;
  Line l;
  l.add("stall s=").addU(t.sample).add(" b=").addU(t.baseline).add(" lo=").addU(t.minSample)
      .add(" hi=").addU(t.maxSample).add(" d=").addU(t.maxDelta).add(" n=").addU(t.samples)
      .add(" run=").addU(t.overCount).add(" mv").addBool(t.moving).add(" ig").addBool(t.ignoring)
      .add(" on").addBool(t.active);
  log(LOG_VERBOSE, l);
}

void Debug::countLoop(uint32_t loopUs) {
  loopsInWindow_++;
  if (loopUs > loopMaxUsInWindow_) loopMaxUsInWindow_ = loopUs;
}

void Debug::setLevel(uint8_t level) { level_ = level > LOG_VERBOSE ? (uint8_t)LOG_VERBOSE : level; }

void Debug::log(uint8_t level, const char* text) {
  if (!enabled(level)) return;
  Line l;
  l.addU(hal::nowMs()).add(' ').add(text);
  emit(l.c_str(), l.length());
}

void Debug::log(uint8_t level, const Line& line) { log(level, line.c_str()); }

void Debug::logAlways(const Line& line) { reply(line); }

void Debug::reply(const Line& line) {
  Line l;
  l.addU(hal::nowMs()).add(' ').add(line.c_str());
  emit(l.c_str(), l.length());
}

void Debug::emit(const char* text, size_t len) {
  char out[DEBUG_LOG_LINE_MAX + 1];
  if (len > DEBUG_LOG_LINE_MAX) len = DEBUG_LOG_LINE_MAX;
  memcpy(out, text, len);
  out[len] = '\n';
  if (hal::usbWrite(reinterpret_cast<const uint8_t*>(out), len + 1))
    linesLogged_++;
  else
    linesDropped_++;
}

// ---- the shell ---------------------------------------------------------------------------------
void Debug::handleShellLine(const char* line) {
  if (!line) return;
  const char* s = line;
  const Word cmd = nextWord(s);
  const Word arg = nextWord(s);
  const Word extra = nextWord(s);
  if (cmd.n == 0) return;   // a blank line is no command
  const bool noArg = arg.n == 0;
  const bool oneArg = arg.n != 0 && extra.n == 0;
  Line l;

  for (const Action& a : kPlainActions) {
    if (!is(cmd, a.name)) continue;
    if (!noArg) {
      shellRejected_++;
      l.add(a.name).add(": takes no argument (not run)");
      reply(l);
      return;
    }
    runAction(a.name, (uint8_t)a.cmd, 0.0f);
    return;
  }
  if (is(cmd, "fly") || is(cmd, "mode")) {
    const bool fly = is(cmd, "fly");
    const char* first = fly ? "on" : "semi";
    const char* second = fly ? "off" : "full";
    if (oneArg && (is(arg, first) || is(arg, second))) {
      const bool isFirst = is(arg, first);
      const TurretState::ShellCmd c =
          fly ? (isFirst ? TurretState::ShellCmd::FLY_ON : TurretState::ShellCmd::FLY_OFF)
              : (isFirst ? TurretState::ShellCmd::MODE_SEMI : TurretState::ShellCmd::MODE_FULL);
      l.add(fly ? "fly " : "mode ").add(isFirst ? first : second);
      runAction(l.c_str(), (uint8_t)c, 0.0f);
      return;
    }
    shellRejected_++;
    l.add("usage: ").add(fly ? "fly on|off" : "mode semi|full").add(" (not run)");
    reply(l);
    return;
  }
  if (is(cmd, "pan") || is(cmd, "tilt")) {
    const bool pan = is(cmd, "pan");
    float deg = 0.0f;
    if (oneArg && parseDeg(arg, deg)) {
      l.add(pan ? "pan " : "tilt ").add(deg, 1);
      runAction(l.c_str(), (uint8_t)(pan ? TurretState::ShellCmd::PAN : TurretState::ShellCmd::TILT), deg);
      return;
    }
    shellRejected_++;
    l.add("usage: ").add(pan ? "pan" : "tilt").add(" <degrees> (not run)");
    reply(l);
    return;
  }
  if (is(cmd, "servo")) {
    if (oneArg) {
      for (const Action& a : kServoActions) {
        if (!is(arg, a.name)) continue;
        l.add("servo ").add(a.name);
        runAction(l.c_str(), (uint8_t)a.cmd, 0.0f);
        return;
      }
      float deg = 0.0f;
      if (parseDeg(arg, deg) && deg == floorf(deg)) {   // whole degrees only: what gets recorded
        l.add("servo ").add(deg, 0);
        runAction(l.c_str(), (uint8_t)TurretState::ShellCmd::SERVO_SET, deg);
        return;
      }
    }
    shellRejected_++;
    l.add("usage: servo adjust|<whole degrees>|rest|fire|reset|stroke|off (not run)");
    reply(l);
    return;
  }
  if (is(cmd, "log")) {
    if (oneArg && arg.n == 1 && arg.p[0] >= '0' && arg.p[0] < (char)('0' + LOG_LEVEL_COUNT)) {
      shellCommands_++;
      setLevel((uint8_t)(arg.p[0] - '0'));
      l.add("log: level ").addU(level_);
      reply(l);
      return;
    }
    shellRejected_++;
    l.add("usage: log <0-").addU(LOG_VERBOSE).add("> (not run)");
    reply(l);
    return;
  }
  if (noArg && (is(cmd, "help") || is(cmd, "status") || is(cmd, "stats") || is(cmd, "stall?"))) {
    shellCommands_++;
    if (is(cmd, "help"))
      help();
    else if (is(cmd, "status"))
      status();
    else if (is(cmd, "stats"))
      totals();
    else
      stallReport();
    return;
  }
  shellRejected_++;
  if (is(cmd, "help") || is(cmd, "status") || is(cmd, "stats") || is(cmd, "stall?")) {
    addWord(l, cmd);
    l.add(": takes no argument (not run)");
  } else {
    l.add("unknown command: ");
    addWord(l, cmd);
    l.add(" (help lists them)");
  }
  reply(l);
}

void Debug::runAction(const char* name, uint8_t cmd, float arg) {
  shellCommands_++;
  Line l;
  l.add(name).add(": ");
  if (!src_.turret) {
    l.add("no turret");
  } else {
    const TurretState::CmdResult r =
        src_.turret->shellCommand((TurretState::ShellCmd)cmd, arg, hal::nowMs());
    l.add(TurretState::resultName(r));
  }
  reply(l);
}

void Debug::help() {
  Line l;
  l.add("help: status stats stall? log <0-3> | arm disarm fire fly on|off mode semi|full");
  reply(l);
  l.clear();
  l.add("help: mag pan <deg> tilt <deg> home sethome stop cal");
  reply(l);
  l.clear();
  l.add("help: servo adjust|<deg>|rest|fire|reset|stroke|off (SERVO ADJUST from DISARMED; pad: Menu)");
  reply(l);
}

void Debug::status() {
  Line l;
  if (src_.turret) {
    const TurretState::Status t = src_.turret->status();
    l.add("status state=").add(ds_turret_state_name(t.state)).add(" fault=").add(ds_fault_name(t.fault))
        .add(" losses=").addU(t.linkLosses).add(" stalls=").addU(t.stalls).add(" refused=")
        .addU(t.refusals).add(" warn=").addU(warnings_);
    reply(l);
    l.clear();
    // Where the angles in force come from: the saved record, config.h's (no record), or neither
    // (adjusted, not saved: Menu in SERVO ADJUST saves them).
    const bool atBoot = t.restDeg == t.bootAngles.rest && t.fireDeg == t.bootAngles.fire;
    l.add("status servo=").add((int32_t)t.servoDeg).add(" adjust=").addBool(t.servoTeach).add(" rest=")
        .add((int32_t)t.restDeg).add(" fire=").add((int32_t)t.fireDeg)
        .add(!atBoot ? " (not saved)" : t.savedRecord ? " (saved)" : " (config.h)");
    reply(l);
    l.clear();
  }
  if (src_.trigger || src_.flywheel) {
    l.add("status");
    if (src_.trigger)
      l.add(" fire=").add(ds_fire_mode_name(src_.trigger->fireMode())).add(" ammo=")
          .addU(src_.trigger->ammo()).add('/').addU(src_.trigger->capacity()).add(" shots=")
          .addU(src_.trigger->shotsTotal());
    if (src_.flywheel)
      l.add(" fly=").add(onOff(src_.flywheel->on())).add(" ready=")
          .addBool(src_.flywheel->ready(hal::nowMs())).add(" duty=").addU(src_.flywheel->duty());
    reply(l);
    l.clear();
  }
  if (src_.motion) {
    const Motion& m = *src_.motion;
    l.add("status pan=").add(m.positionDeg(AXIS_PAN), 1).add(" tilt=").add(m.positionDeg(AXIS_TILT), 1)
        .add(" home=").add(m.homeDeg(AXIS_PAN), 1).add(',').add(m.homeDeg(AXIS_TILT), 1)
        .add(" cal=").addBool(m.calibrated()).add(" homing=").addBool(m.homing()).add(" drv=")
        .addBool(m.driversEnabled());
    reply(l);
    l.clear();
  }
  if (src_.link) {
    const LinkStatus& st = src_.link->status();
    l.add("status link=").add(ds_link_source_name(st.activeSource)).add(" conn=").addBool(st.connected)
        .add(" want=").add(ds_radio_mode_name(st.wantMode)).add(" esp=")
        .add(st.espAlive ? (st.espLegacy ? "legacy" : "alive") : "silent").add(" espMode=")
        .add(ds_esp_mode_name(st.espMode)).add(" usb=").add(onOff(st.usbControlEnabled));
    reply(l);
    l.clear();
  }
  if (src_.inputs) {
    l.add("status power=").add(onOff(src_.inputs->powerOn())).add(" modeSwitch=")
        .add(ds_radio_mode_name(src_.inputs->radioMode())).add(" rt=").addBool(src_.inputs->triggerPulled())
        .add(" lcd=").addBool(hal::lcdPresent()).add(" lcdErrors=").addU(hal::lcdErrors())
        .add(" log=").addU(level_);
    reply(l);
  }
}

void Debug::totals() {
  Line l;
  statsLine(l);   // the last closed window
  reply(l);
  l.clear();
  if (src_.link) {
    const LinkStats& s = src_.link->stats();
    l.add("totals frames=").addU(s.frames).add(" usb=").addU(s.usbFrames).add(" crc=").addU(s.crcErrors)
        .add(" gaps=").addU(s.seqGaps).add(" polls=").addU(s.polls).add(" to=").addU(s.pollTimeouts)
        .add(" err=").addU(s.pollErrors).add(" srcChg=").addU(s.sourceChanges);
    reply(l);
    l.clear();
  }
  l.add("totals usbDrop=").addU(hal::usbTxDropped()).add(" linkDrop=").addU(hal::linkTxDropped())
      .add(" lines=").addU(linesLogged_).add(" lost=").addU(linesDropped_).add(" loopMax=")
      .addU(stats_.loopMaxUs).add("us shell=").addU(shellCommands_).add('/').addU(shellRejected_);
  reply(l);
}

void Debug::stallReport() {
  Line l;
  if (!src_.stall) {
    l.add("stall?: no detector");
    reply(l);
    return;
  }
  const StallDetector& d = *src_.stall;
  l.add("stall? on=").addBool(d.active()).add(" s=").addU(d.sample()).add(" b=").addU(d.baseline())
      .add(" peak=").addU(d.peakDelta()).add(" last=").addU(d.lastStallPeak()).add(" n=")
      .addU(d.stallCount()).add(" would=").addU(d.triggers()).add(" skipped=").addU(d.droppedSamples());
  reply(l);
}

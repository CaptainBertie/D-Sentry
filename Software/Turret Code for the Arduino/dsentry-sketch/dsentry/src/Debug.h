// Debug.h -- debug log, periodic stats line and the bench shell (docs/architecture.md 5).
//
// Output is text lines on the USB port, queued whole through hal::usbWrite (never printf,
// never a blocking write, never from an interrupt). A line that does not fit the USB queue is
// dropped and counted by the HAL. Lines are built with Debug::Line, a fixed-size builder:
//
//   Debug::Line l;
//   l.add("pan ").add(motion.positionDeg(AXIS_PAN), 1).add(" deg");
//   debug.log(LOG_STATE, l);
//
// Every line starts with the uptime in ms and a space. Log lines obey the level (log <0-3>);
// shell replies are always sent (they answer a request).
//
// Periodic output (update()):
//   - every DEBUG_STATS_INTERVAL_MS a window closes (RuntimeStats, read by Telemetry too) and,
//     at level >= LOG_STATE, the stats line is printed:
//       stats loop=<Hz> rx=<frames/s> to=<poll timeouts> crc=<CRC errors> gap=<seq gaps>
//             loss=<x.x>% jit=<max>/<mean>pm led=<forced shows> drop=<USB TX drops>
//     counts are the window's (not totals); loss is the active source's (USB: seq gaps against
//     the frames expected; radio: AT polls that timed out or failed); jit is the step jitter
//     in permille of the period, max/mean over both axes. A value past its acceptance-17 limit
//     (STEP_JITTER_LIMIT_PERMILLE, PACKET_LOSS_LIMIT_PERMILLE) is followed by '!';
//   - at level LOG_VERBOSE, STALL_LOG_HZ times a second, the stall trace (spec 3.6: how the
//     threshold gets tuned): s sample, b baseline, lo..hi the window's range, d its largest
//     |sample - baseline|, n samples, run over-threshold samples in a row, mv moving, ig in the
//     start-up ignore window, on detection active.
//
// The shell: one command per line from ControlLink (which owns the USB byte stream). The first
// word must be a command exactly (whole-word match, so the ESP32's boot banner, which reaches
// the shell as text lines, never runs one) and the argument count must match; anything else
// is answered and not run. Commands: help, status, stats, stall?, log <0-3>, and the actions
// arm, disarm, fire, fly on|off, mode semi|full, mag, pan <deg>, tilt <deg>, home, sethome,
// stop, cal, and SERVO ADJUST's servo adjust|<whole deg>|rest|fire|reset|stroke|off ("servo teach"
// is an alias of "servo adjust"; the pad's Menu, d-pad / bumper nudges, A, B, View, X and Menu
// again, which saves), which go through TurretState::shellCommand(),
// so they obey exactly the permissions the controller does (architecture.md rule 5). Each
// reply is "<cmd>: ok" or "<cmd>: refused: <reason>".
//
// Boot: bootBanner() right after begin() (what hal::begin brought up), bootWarnings() once
// every module has begun: no LCD, the LCD texts that will be cut (Lcd lists each), and
// FAILSAFE_DISARM_ON_LOSS = false, which is ignored (reconnection always lands in DISARMED).
//
// SPDX-License-Identifier: MIT
#pragma once
#include <stddef.h>
#include <stdint.h>
#include "Types.h"

class TurretState;
class Inputs;
class ControlLink;
class Motion;
class StallDetector;
class Flywheel;
class Trigger;
class Leds;
class Lcd;

// Log levels, matching config.h DEBUG_LOG_LEVEL (0 off, 1 errors, 2 state changes, 3 verbose).
enum LogLevel : uint8_t {
  LOG_OFF = 0,
  LOG_ERROR,     // something failed or was refused
  LOG_STATE,     // state changes, boot report, the stats line
  LOG_VERBOSE,   // per-event detail and the STALL_LOG_HZ stall trace
  LOG_LEVEL_COUNT
};

// Measured over the last DEBUG_STATS_INTERVAL_MS window; read by Telemetry as well.
struct RuntimeStats {
  uint32_t windowMs = 0;        // length of the window these numbers cover
  uint16_t loopHz = 0;          // loop() iterations per second, saturates at UINT16_MAX
  uint32_t loopMaxUs = 0;       // longest single loop() in the window
  StepStats steps;              // hal::stepTakeStats() for the window
  uint16_t jitterMaxPermille = 0;   // worst of both axes (TelemetryPacket.stepJitterPermille)
  uint16_t jitterMeanPermille = 0;  // mean of both axes, weighted by samples
  uint32_t usbTxDropped = 0;    // hal::usbTxDropped() at the end of the window (cumulative)
  uint32_t linkTxDropped = 0;   // hal::linkTxDropped() likewise
  // The window's own counts (the stats line):
  uint16_t ctrlRxHz = 0;        // LinkStats.ctrlRxHz at the end of the window
  uint32_t frames = 0;          // valid control frames accepted (all sources)
  uint32_t pollTimeouts = 0;    // AT requests without a final in time
  uint32_t crcErrors = 0;
  uint32_t seqGaps = 0;
  uint16_t lossPermille = 0;    // the active source's packet loss (see the header comment)
  uint32_t ledForcedShows = 0;  // LED frames shown without a step-quiet window
  uint32_t usbTxDrops = 0;      // USB messages refused by the TX queue
  bool jitterOverLimit = false; // jitterMaxPermille > STEP_JITTER_LIMIT_PERMILLE
  bool lossOverLimit = false;   // lossPermille >= PACKET_LOSS_LIMIT_PERMILLE
};

// The modules Debug reports on. TurretState is non-const: shell commands act through it;
// StallDetector is non-const for its trace window (takeTrace).
struct DebugSources {
  TurretState* turret = nullptr;
  const Inputs* inputs = nullptr;
  const ControlLink* link = nullptr;
  const Motion* motion = nullptr;
  StallDetector* stall = nullptr;
  const Flywheel* flywheel = nullptr;
  const Trigger* trigger = nullptr;
  const Leds* leds = nullptr;
  const Lcd* lcd = nullptr;
};

class Debug {
 public:
  // Fixed-size text line builder. Text past DEBUG_LOG_LINE_MAX is cut (and truncated() says
  // so); the newline is added by log().
  class Line {
   public:
    Line& add(const char* text);
    Line& add(char c);
    Line& add(int32_t v);
    Line& addU(uint32_t v);
    Line& add(float v, uint8_t decimals);   // fixed point, rounded; no exponent form
    Line& addHex(uint32_t v, uint8_t digits);  // zero-padded to `digits`, lowercase
    Line& addBool(bool v);                  // "1" / "0"
    const char* c_str() const { return buf_; }
    size_t length() const { return len_; }
    bool truncated() const { return truncated_; }
    void clear();

   private:
    char buf_[DEBUG_LOG_LINE_MAX + 1] = {};
    size_t len_ = 0;
    bool truncated_ = false;
  };

  void begin(const DebugSources& sources);
  void bootBanner(bool halOk);      // "D-Sentry RA4M1 boot: ..." (what hal::begin brought up)
  void bootWarnings();              // after every module's begin(); see the header comment
  uint8_t warnings() const { return warnings_; }   // lines bootWarnings() logged

  // Once per loop(): stats window, STALL_LOG_HZ trace at LOG_VERBOSE.
  void update(uint32_t nowMs, uint32_t nowUs);

  // App calls this once per loop() with that loop's duration, for loopHz / loopMaxUs.
  void countLoop(uint32_t loopUs);

  bool enabled(uint8_t level) const { return level != LOG_OFF && level <= level_; }
  void log(uint8_t level, const char* text);   // one line, prefixed by the uptime
  void log(uint8_t level, const Line& line);
  // A LOG_STATE line the owner must not miss whatever the level (SERVO ADJUST's ready-to-paste
  // "#define" lines): always sent, like a shell reply, prefixed by the uptime.
  void logAlways(const Line& line);
  void setLevel(uint8_t level);                // clamped to LOG_VERBOSE
  uint8_t level() const { return level_; }

  // One complete shell line (no newline). See the header comment.
  void handleShellLine(const char* line);

  const RuntimeStats& stats() const { return stats_; }
  uint32_t linesLogged() const { return linesLogged_; }
  uint32_t linesDropped() const { return linesDropped_; }   // refused by hal::usbWrite
  uint32_t shellCommands() const { return shellCommands_; } // lines that ran a command
  uint32_t shellRejected() const { return shellRejected_; } // unknown words / bad arguments

 private:
  void emit(const char* text, size_t len);
  void reply(const Line& line);                // always sent, prefixed by the uptime
  void closeWindow(uint32_t nowMs);
  void statsLine(Line& l) const;
  void stallTrace();
  void warn(const char* text);
  void help();
  void status();
  void totals();
  void stallReport();
  void runAction(const char* name, uint8_t cmd, float arg);

  DebugSources src_;
  uint8_t level_ = DEBUG_LOG_LEVEL;
  RuntimeStats stats_;
  uint32_t windowStartMs_ = 0;
  uint32_t loopsInWindow_ = 0;
  uint32_t loopMaxUsInWindow_ = 0;
  uint32_t linesLogged_ = 0;
  uint32_t linesDropped_ = 0;
  uint32_t lastStallLogMs_ = 0;
  uint8_t warnings_ = 0;
  uint32_t shellCommands_ = 0;
  uint32_t shellRejected_ = 0;
  // counters at the start of the window (the stats line shows differences)
  LinkStats linkAtStart_;
  uint32_t forcedShowsAtStart_ = 0;
  uint32_t usbDropsAtStart_ = 0;
  bool begun_ = false;
};

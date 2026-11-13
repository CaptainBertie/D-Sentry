// StallDetector.h -- stall / crash detection on PIN_STALL_SENSE (spec 3.6, architecture.md 5).
//
// Sampling: hal::readAnalog(PIN_STALL_SENSE) every 1/STALL_SAMPLE_HZ s from update() on a
// micros() schedule (analogRead is not interrupt-safe). When a call is late it takes the due
// sample plus at most one catch-up sample; if it is still behind after that, the backlog is
// dropped and the schedule restarts one period from now, so a long loop never becomes a burst.
//
// Per sample, by what the axes are doing (movingAxes / moveStartMs from Motion):
//   - at rest: the baseline is frozen, nothing is counted;
//   - moving: a sample within STALL_DELTA_COUNTS of the baseline ("in band") feeds the EMA
//     baseline (STALL_BASELINE_ALPHA), so slow drift (battery discharge, a small level change
//     when the motors start) is followed; a sample beyond it never does, so the threshold is
//     exact: a step just over STALL_DELTA_COUNTS is not absorbed before it is confirmed;
//   - in the first STALL_IGNORE_AFTER_START_MS of a move (moveStartMs; Motion moves it on
//     every axis start, so a second axis starting re-masks its kick) nothing is counted;
//   - after it, |sample - baseline| > STALL_DELTA_COUNTS counts one over-threshold sample, and
//     STALL_CONFIRM_SAMPLES of them in a row is a stall.
// So a crash that is already pressing when a move starts (the barrel resting on the panel when a
// CAL_STALL_SENSE calibration begins) is reported straight after the ignore window, and a level
// change of more than STALL_DELTA_COUNTS between rest and motion would be too: the level-3 trace
// (takeTrace) shows the moving level, the noise band and the peaks for tuning the threshold.
//
// A stall is declared only while active(): Config::enabled (STALL_DETECT_ENABLED), or forced on
// by Motion during a CAL_STALL_SENSE calibration. Inactive, the detector still samples, tracks
// the baseline, the
// peaks and counts would-be stalls (triggers()), so the threshold can be tuned with detection
// off. TurretState is the only caller of takeStall() (architecture stage-2 decision a); during a
// CAL_STALL_SENSE calibration it forwards the event to Motion::notifyCalibrationStall().
//
// SPDX-License-Identifier: MIT
#pragma once
#include <stdint.h>
#include "Types.h"

// Aggregates since the previous takeTrace(): what Debug prints at level 3, STALL_LOG_HZ times a
// second (the window's extremes, so a spike between two lines still shows).
struct StallTrace {
  uint16_t sample = 0;      // latest reading, ADC counts
  uint16_t baseline = 0;    // latest baseline (rounded)
  uint16_t minSample = 0;   // smallest reading in the window
  uint16_t maxSample = 0;   // largest reading in the window
  uint16_t maxDelta = 0;    // largest |sample - baseline| in the window (any phase)
  uint16_t samples = 0;     // readings in the window (saturates at UINT16_MAX)
  uint8_t overCount = 0;    // consecutive over-threshold samples at the latest reading
  bool moving = false;      // an axis was moving at the latest reading
  bool ignoring = false;    // ...within the start-up ignore window
  bool active = false;      // detection on (enabled or forced)
};

class StallDetector {
 public:
  // From config.h by default; tests (and the App tests) switch detection on with a pot on A2.
  struct Config {
    bool enabled;   // STALL_DETECT_ENABLED: declare stalls outside CAL_STALL_SENSE calibration
  };
  static Config defaults();

  StallDetector();                          // defaults()
  explicit StallDetector(const Config& cfg);
  const Config& config() const { return cfg_; }

  // First reading seeds the baseline; the next sample is due one period after nowUs.
  void begin(uint32_t nowUs);

  // Once per loop(). movingAxes: Motion::movingAxes(); moveStartMs: Motion::moveStartMs().
  void update(uint32_t nowUs, uint32_t nowMs, uint8_t movingAxes, uint32_t moveStartMs);

  void setForced(bool on) { forced_ = on; }   // Motion: CAL_STALL_SENSE calibration only
  bool forced() const { return forced_; }
  bool active() const;                        // Config::enabled or forced

  // A stall was declared since the last call (cleared by reading). stallAxes() then tells
  // which axes were moving when it was confirmed; lastStallPeak() its largest delta.
  bool takeStall();
  bool stallPending() const { return pending_; }   // peek without clearing (tests, shell)
  uint8_t stallAxes() const { return stallAxes_; }
  uint32_t stallCount() const { return stallCount_; }   // declared stalls since begin()
  // Times the confirm condition was met, active or not: what detection would have done.
  uint32_t triggers() const { return triggers_; }

  // Live values (telemetry, shell `stall?`), ADC counts.
  uint16_t sample() const { return sample_; }
  uint16_t baseline() const;                                // rounded EMA
  float baselineExact() const { return baseline_; }
  uint16_t peakDelta() const { return peakDelta_; }         // largest counted |delta| of the
                                                            // current / last move
  uint16_t lastStallPeak() const { return lastStallPeak_; } // peak delta of the last trigger
  uint8_t overCount() const { return overCount_; }
  bool moving() const { return moving_; }                   // at the latest sample
  bool ignoring() const { return ignoring_; }               // at the latest sample
  uint32_t samples() const { return samples_; }             // readings since begin()
  uint32_t droppedSamples() const { return dropped_; }      // due samples skipped (late loop)

  // The level-3 hook: copies the window since the previous call into `out` and starts a new
  // one. Returns false (out untouched) when no sample was taken in the window.
  bool takeTrace(StallTrace& out);

 private:
  void takeSample(uint32_t nowMs, uint8_t movingAxes, uint32_t moveStartMs);

  Config cfg_;
  bool forced_ = false;
  bool pending_ = false;
  uint8_t stallAxes_ = 0;
  uint32_t stallCount_ = 0;
  uint32_t triggers_ = 0;
  uint16_t sample_ = 0;
  float baseline_ = 0.0f;
  uint16_t peakDelta_ = 0;
  uint16_t runPeak_ = 0;        // largest delta of the current over-threshold run
  uint16_t lastStallPeak_ = 0;
  uint8_t overCount_ = 0;       // consecutive samples beyond STALL_DELTA_COUNTS
  uint32_t nextSampleUs_ = 0;
  uint32_t samples_ = 0;
  uint32_t dropped_ = 0;
  bool moving_ = false;
  bool ignoring_ = false;
  uint32_t lastMoveStartMs_ = 0;
  StallTrace win_;              // the trace window being accumulated
};

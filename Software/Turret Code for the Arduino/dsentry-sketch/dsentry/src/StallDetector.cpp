// StallDetector.cpp -- see StallDetector.h.
//
// SPDX-License-Identifier: MIT
#include "StallDetector.h"

#include "hal.h"

static_assert(STALL_SAMPLE_HZ > 0 && STALL_SAMPLE_HZ <= US_PER_S, "STALL_SAMPLE_HZ out of range");
static_assert(STALL_BASELINE_ALPHA > 0.0f && STALL_BASELINE_ALPHA <= 1.0f,
              "STALL_BASELINE_ALPHA must be in (0, 1]");
static_assert(STALL_CONFIRM_SAMPLES >= 1 && STALL_CONFIRM_SAMPLES <= UINT8_MAX,
              "STALL_CONFIRM_SAMPLES must fit the uint8_t run counter");
static_assert(STALL_DELTA_COUNTS >= 0 && STALL_DELTA_COUNTS < ADC_MAX_COUNTS,
              "STALL_DELTA_COUNTS out of the ADC range");

namespace {
constexpr uint32_t kPeriodUs = US_PER_S / STALL_SAMPLE_HZ;

uint16_t roundCounts(float v) {
  if (!(v > 0.0f)) return 0;
  return (uint16_t)(v + MATH_HALF);
}
}  // namespace

StallDetector::Config StallDetector::defaults() {
  Config c;
  c.enabled = STALL_DETECT_ENABLED;
  return c;
}

StallDetector::StallDetector() : cfg_(defaults()) {}

StallDetector::StallDetector(const Config& cfg) : cfg_(cfg) {}

void StallDetector::begin(uint32_t nowUs) {
  forced_ = false;
  pending_ = false;
  stallAxes_ = 0;
  stallCount_ = 0;
  triggers_ = 0;
  sample_ = hal::readAnalog(PIN_STALL_SENSE);
  baseline_ = (float)sample_;
  peakDelta_ = 0;
  runPeak_ = 0;
  lastStallPeak_ = 0;
  overCount_ = 0;
  nextSampleUs_ = nowUs + kPeriodUs;
  samples_ = 1;
  dropped_ = 0;
  moving_ = false;
  ignoring_ = false;
  lastMoveStartMs_ = 0;
  win_ = StallTrace();
}

bool StallDetector::active() const { return cfg_.enabled || forced_; }

uint16_t StallDetector::baseline() const { return roundCounts(baseline_); }

void StallDetector::update(uint32_t nowUs, uint32_t nowMs, uint8_t movingAxes, uint32_t moveStartMs) {
  if ((int32_t)(nowUs - nextSampleUs_) < 0) return;   // not due (wrap-safe)
  takeSample(nowMs, movingAxes, moveStartMs);
  nextSampleUs_ += kPeriodUs;
  if ((int32_t)(nowUs - nextSampleUs_) >= 0) {        // late: one catch-up sample
    takeSample(nowMs, movingAxes, moveStartMs);
    nextSampleUs_ += kPeriodUs;
  }
  if ((int32_t)(nowUs - nextSampleUs_) >= 0) {        // still behind: drop the backlog
    dropped_ += (nowUs - nextSampleUs_) / kPeriodUs + 1u;
    nextSampleUs_ = nowUs + kPeriodUs;
  }
}

void StallDetector::takeSample(uint32_t nowMs, uint8_t movingAxes, uint32_t moveStartMs) {
  const uint16_t s = hal::readAnalog(PIN_STALL_SENSE);
  sample_ = s;
  samples_++;
  const bool moving = movingAxes != 0;
  if (moving && (!moving_ || moveStartMs != lastMoveStartMs_)) {   // a new move
    peakDelta_ = 0;
    overCount_ = 0;
    runPeak_ = 0;
  }
  moving_ = moving;
  lastMoveStartMs_ = moveStartMs;
  ignoring_ = moving && (uint32_t)(nowMs - moveStartMs) < STALL_IGNORE_AFTER_START_MS;

  const float diff = (float)s - baseline_;
  const float absDiff = diff < 0.0f ? -diff : diff;
  const uint16_t delta = roundCounts(absDiff);
  const bool over = absDiff > (float)STALL_DELTA_COUNTS;
  if (!moving) {
    overCount_ = 0;   // at rest: baseline frozen, nothing counted
  } else {
    if (!over) baseline_ += STALL_BASELINE_ALPHA * diff;   // in band: follow the level
    if (ignoring_) {
      overCount_ = 0;   // the start-up kick: counted as nothing
    } else {
      if (delta > peakDelta_) peakDelta_ = delta;
      if (!over) {
        overCount_ = 0;
      } else {
        if (overCount_ == 0) runPeak_ = 0;
        if (delta > runPeak_) runPeak_ = delta;
        if (overCount_ < UINT8_MAX) overCount_++;
        if (overCount_ >= STALL_CONFIRM_SAMPLES) {
          triggers_++;
          lastStallPeak_ = runPeak_;
          if (active()) {
            pending_ = true;
            stallAxes_ = movingAxes;
            stallCount_++;
          }
          overCount_ = 0;   // a further stall needs a fresh run of STALL_CONFIRM_SAMPLES
          runPeak_ = 0;
        }
      }
    }
  }

  // Trace window.
  if (win_.samples == 0) {
    win_.minSample = win_.maxSample = s;
    win_.maxDelta = 0;
  }
  if (s < win_.minSample) win_.minSample = s;
  if (s > win_.maxSample) win_.maxSample = s;
  if (delta > win_.maxDelta) win_.maxDelta = delta;
  if (win_.samples < UINT16_MAX) win_.samples++;
  win_.sample = s;
  win_.baseline = baseline();
  win_.overCount = overCount_;
  win_.moving = moving_;
  win_.ignoring = ignoring_;
  win_.active = active();
}

bool StallDetector::takeStall() {
  const bool was = pending_;
  pending_ = false;
  return was;
}

bool StallDetector::takeTrace(StallTrace& out) {
  if (win_.samples == 0) return false;
  out = win_;
  win_ = StallTrace();
  return true;
}

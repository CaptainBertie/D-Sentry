// Flywheel.cpp -- see Flywheel.h.
//
// SPDX-License-Identifier: MIT
#include "Flywheel.h"

#include "hal.h"

static_assert(FLYWHEEL_DUTY_255 >= 0 && FLYWHEEL_DUTY_255 <= PWM_MAX, "FLYWHEEL_DUTY_255 is 0..PWM_MAX");
static_assert(PWM_MAX <= UINT8_MAX, "hal::pwmWrite takes an 8-bit duty");

namespace {

inline bool elapsed(uint32_t nowMs, uint32_t sinceMs, uint32_t periodMs) {
  return (uint32_t)(nowMs - sinceMs) >= periodMs;
}

// now - since as a signed span: negative when `now` is older than `since` (a caller passing a
// stale timestamp), which must read as "no time yet", never as a huge span.
inline int32_t span(uint32_t nowMs, uint32_t sinceMs) { return (int32_t)(nowMs - sinceMs); }

}  // namespace

Flywheel::Config Flywheel::defaults() {
  Config c;
  c.duty = FLYWHEEL_DUTY_255;
  c.rampMs = FLYWHEEL_RAMP_MS;
  c.spinupMs = FLYWHEEL_SPINUP_MS;
  c.brakeMs = FLYWHEEL_BRAKE_MS;
  c.brakeOnStop = FLYWHEEL_BRAKE_ON_STOP;
  c.forwardIn1High = FLYWHEEL_FORWARD_IN1_HIGH;
  return c;
}

Flywheel::Flywheel() : cfg_(defaults()) {}

Flywheel::Flywheel(const Config& cfg) : cfg_(cfg) {}

void Flywheel::begin() {
  on_ = false;
  onSinceMs_ = 0;
  duty_ = 0;
  rampDone_ = false;
  spunUp_ = false;
  braking_ = false;
  brakeTimed_ = false;
  coast();
}

void Flywheel::writeEna(uint8_t duty) {
  ena_ = duty;
  hal::pwmWrite(PIN_FLY_ENA, duty);
}

// L298N "free running motor stop": enable low. Inputs low too, the state hal::begin() leaves.
void Flywheel::coast() {
  writeEna(0);
  hal::writePin(PIN_FLY_IN1, false);
  hal::writePin(PIN_FLY_IN2, false);
}

uint8_t Flywheel::rampDuty(uint32_t elapsedMs) const {
  if (cfg_.rampMs == 0 || elapsedMs >= cfg_.rampMs) return cfg_.duty;
  return (uint8_t)((uint32_t)cfg_.duty * elapsedMs / cfg_.rampMs);
}

void Flywheel::update(uint32_t nowMs) {
  if (on_) {
    if (!rampDone_) {
      const int32_t t = span(nowMs, onSinceMs_);
      const uint8_t d = rampDuty(t < 0 ? 0u : (uint32_t)t);
      if (d != duty_) {
        duty_ = d;
        writeEna(d);
      }
      rampDone_ = duty_ == cfg_.duty;
    }
    if (!spunUp_ && span(nowMs, onSinceMs_) >= (int32_t)cfg_.spinupMs) spunUp_ = true;
    return;
  }
  if (braking_) {
    if (!brakeTimed_) {
      brakeStartMs_ = nowMs;
      brakeTimed_ = true;
    }
    if (elapsed(nowMs, brakeStartMs_, cfg_.brakeMs)) {
      braking_ = false;
      brakeTimed_ = false;
      coast();
    }
  }
}

void Flywheel::setOn(bool on, uint32_t nowMs) {
  if (!on) {
    const bool wasOn = on_;
    cut();
    if (wasOn && braking_) {   // a fresh brake: time it from this call
      brakeStartMs_ = nowMs;
      brakeTimed_ = true;
    }
    return;
  }
  if (on_) return;
  // From a stop or a brake: enable low before the direction changes, then ramp from 0.
  braking_ = false;
  brakeTimed_ = false;
  writeEna(0);
  hal::writePin(PIN_FLY_IN1, cfg_.forwardIn1High);
  hal::writePin(PIN_FLY_IN2, !cfg_.forwardIn1High);
  on_ = true;
  onSinceMs_ = nowMs;
  duty_ = 0;
  rampDone_ = false;
  spunUp_ = false;
  update(nowMs);   // FLYWHEEL_RAMP_MS 0 or FLYWHEEL_SPINUP_MS 0 take effect in this call
}

void Flywheel::cut() {
  const bool wasDriving = on_;
  on_ = false;
  onSinceMs_ = 0;
  duty_ = 0;
  rampDone_ = false;
  spunUp_ = false;
  if (!cfg_.brakeOnStop || (!wasDriving && !braking_)) {
    braking_ = false;
    brakeTimed_ = false;
    coast();
    return;
  }
  // Fast stop: from driving, enable low while both inputs go high, then enable fully on. A
  // brake already holding keeps its timer; its outputs are written again all the same.
  if (wasDriving) {
    writeEna(0);
    braking_ = true;
    brakeTimed_ = false;
  }
  hal::writePin(PIN_FLY_IN1, true);
  hal::writePin(PIN_FLY_IN2, true);
  writeEna(PWM_MAX);
}

bool Flywheel::ready(uint32_t nowMs) const {
  return on_ && (spunUp_ || span(nowMs, onSinceMs_) >= (int32_t)cfg_.spinupMs);
}

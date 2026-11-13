// Inputs.cpp -- see Inputs.h.
//
// SPDX-License-Identifier: MIT
#include "Inputs.h"

#include "hal.h"

namespace {
// The buttons the turret acts on (spec 3.2); the display-only bits are masked off.
constexpr uint16_t kControlButtons =
    (uint16_t)(DS_BTN_ALL & ~(DS_BTN_LS_CLICK | DS_BTN_RS_CLICK | DS_BTN_XBOX | DS_BTN_SHARE));
}  // namespace

// The curve is monotonic and stays inside -1..1 only for these ranges (d/dm of
// (1-e)m + e m^3 is (1-e) + 3e m^2, negative near m = 0 when e > 1).
static_assert(JOY_DEADZONE >= 0.0f && JOY_DEADZONE < 1.0f, "JOY_DEADZONE must be in [0, 1)");
static_assert(JOY_EXPO >= 0.0f && JOY_EXPO <= 1.0f, "JOY_EXPO must be in [0, 1]");
static_assert(JOY_AXIS_RANGE > 0, "JOY_AXIS_RANGE must be positive");
static_assert(TRIGGER_AXIS_RANGE > 0, "TRIGGER_AXIS_RANGE must be positive");
static_assert(TRIGGER_RELEASE_THRESHOLD >= 0.0f && TRIGGER_RELEASE_THRESHOLD < TRIGGER_PRESS_THRESHOLD &&
                  TRIGGER_PRESS_THRESHOLD < 1.0f,
              "trigger hysteresis needs 0 <= TRIGGER_RELEASE_THRESHOLD < TRIGGER_PRESS_THRESHOLD < 1");

namespace {

inline bool elapsed(uint32_t nowMs, uint32_t sinceMs, uint32_t periodMs) {
  return (uint32_t)(nowMs - sinceMs) >= periodMs;
}

}  // namespace

Inputs::Config Inputs::defaults() {
  Config c;
  c.toggleActiveLow = TOGGLE_ACTIVE_LOW;
  c.modeWifiWhenClosed = TOGGLE_MODE_WIFI_WHEN_CLOSED;
  c.invertY = JOY_INVERT_Y;
  return c;
}

Inputs::Inputs() : cfg_(defaults()) {}

Inputs::Inputs(const Config& cfg) : cfg_(cfg) {}

bool Inputs::toggleClosed(uint8_t pin) const {
  const bool high = hal::readPin(pin);
  return cfg_.toggleActiveLow ? !high : high;
}

bool Inputs::readPowerOn() const { return toggleClosed(PIN_TOGGLE_POWER); }

bool Inputs::readModeWifi() const {
  const bool closed = toggleClosed(PIN_TOGGLE_MODE);
  return cfg_.modeWifiWhenClosed ? closed : !closed;
}

void Inputs::begin(uint32_t nowMs) {
  powerRaw_ = readPowerOn();
  powerOn_ = powerRaw_;
  powerChanged_ = false;
  powerSinceMs_ = nowMs;
  modeRaw_ = readModeWifi();
  radioMode_ = modeRaw_ ? (uint8_t)DS_RADIO_WIFI : (uint8_t)DS_RADIO_BLE;
  radioModeChanged_ = false;
  modeSinceMs_ = nowMs;
  connected_ = false;
  source_ = DS_LINK_SRC_NONE;
  clearHeld();
}

void Inputs::updateToggles(uint32_t nowMs) {
  powerChanged_ = false;
  radioModeChanged_ = false;

  // Each toggle: restart the timer whenever the raw level changes; accept the raw level once
  // it has held for TOGGLE_DEBOUNCE_MS.
  const bool power = readPowerOn();
  if (power != powerRaw_) {
    powerRaw_ = power;
    powerSinceMs_ = nowMs;
  }
  if (powerRaw_ != powerOn_ && elapsed(nowMs, powerSinceMs_, TOGGLE_DEBOUNCE_MS)) {
    powerOn_ = powerRaw_;
    powerChanged_ = true;
  }

  const bool wifi = readModeWifi();
  if (wifi != modeRaw_) {
    modeRaw_ = wifi;
    modeSinceMs_ = nowMs;
  }
  const bool modeWifi = radioMode_ == DS_RADIO_WIFI;
  if (modeRaw_ != modeWifi && elapsed(nowMs, modeSinceMs_, TOGGLE_DEBOUNCE_MS)) {
    radioMode_ = modeRaw_ ? (uint8_t)DS_RADIO_WIFI : (uint8_t)DS_RADIO_BLE;
    radioModeChanged_ = true;
  }
}

void Inputs::update(uint32_t nowMs, const ControlFrame& frame) {
  updateToggles(nowMs);
  triggerPullStarted_ = false;
  pressedEdges_ = 0;

  if (!(frame.valid && frame.connected)) {
    clearHeld();
    connected_ = false;
    return;
  }
  // The same physical link can switch sources (USB bench override <-> radio); the new
  // source's levels are a fresh baseline, never edges.
  if (connected_ && frame.source != source_) clearHeld();
  connected_ = true;
  source_ = frame.source;

  // ---- sticks ----
  stickX_ = shapeAxis(frame.rx);
  const float y = shapeAxis(frame.ry);
  stickY_ = cfg_.invertY ? -y : y;
  stickActive_ = outsideDeadzone(frame.rx) || outsideDeadzone(frame.ry);

  // ---- trigger, with hysteresis ----
  float f = (float)frame.rt / (float)TRIGGER_AXIS_RANGE;
  if (f > 1.0f) f = 1.0f;
  triggerFraction_ = f;
  const bool wasPulled = triggerPulled_;
  if (f > TRIGGER_PRESS_THRESHOLD)
    triggerPulled_ = true;
  else if (f < TRIGGER_RELEASE_THRESHOLD)
    triggerPulled_ = false;

  // ---- buttons ----
  // Bits 12-15 (stick clicks, Xbox, Share) are carried for the dashboard's controller view only:
  // the turret never acts on them, so they never become a level or an edge here.
  const uint16_t levels = (uint16_t)(frame.buttons & kControlButtons);
  // Expire finished lockouts first, so a timer can never look active again after the
  // millisecond counter wraps.
  for (uint8_t i = 0; lockoutActive_ != 0 && i < kButtonBits; i++) {
    const uint16_t bit = (uint16_t)(1u << i);
    if ((lockoutActive_ & bit) && elapsed(nowMs, lockoutStartMs_[i], BUTTON_DEBOUNCE_MS))
      lockoutActive_ = (uint16_t)(lockoutActive_ & ~bit);
  }
  if (baselinePending_) {
    baselinePending_ = false;   // levels as they are; no button or trigger edge
  } else {
    triggerPullStarted_ = triggerPulled_ && !wasPulled;
    const uint16_t rising = (uint16_t)(levels & ~prevButtons_);
    for (uint8_t i = 0; i < kButtonBits; i++) {
      const uint16_t bit = (uint16_t)(1u << i);
      if (!(rising & bit) || (lockoutActive_ & bit)) continue;
      pressedEdges_ = (uint16_t)(pressedEdges_ | bit);
      lockoutActive_ = (uint16_t)(lockoutActive_ | bit);
      lockoutStartMs_[i] = nowMs;
    }
  }
  prevButtons_ = levels;
  heldLevels_ = levels;
}

void Inputs::clearHeld() {
  stickX_ = 0.0f;
  stickY_ = 0.0f;
  stickActive_ = false;
  triggerFraction_ = 0.0f;
  triggerPulled_ = false;
  triggerPullStarted_ = false;
  pressedEdges_ = 0;
  heldLevels_ = 0;
  prevButtons_ = 0;
  lockoutActive_ = 0;
  for (uint32_t& t : lockoutStartMs_) t = 0;
  baselinePending_ = true;
}

float Inputs::shape(float s, float deadzone, float expo) {
  float mag = s < 0.0f ? -s : s;
  if (mag > 1.0f) mag = 1.0f;
  if (!(mag > deadzone)) return 0.0f;   // inside the dead zone (also catches NaN, and dz >= 1)
  const float m = (mag - deadzone) / (1.0f - deadzone);
  float out = (1.0f - expo) * m + expo * m * m * m;
  if (out > 1.0f) out = 1.0f;     // only reachable with expo outside 0..1
  if (out < 0.0f) out = 0.0f;
  return s < 0.0f ? -out : out;
}

float Inputs::shapeAxis(int16_t raw) {
  return shape((float)raw / (float)JOY_AXIS_RANGE, JOY_DEADZONE, JOY_EXPO);
}

bool Inputs::outsideDeadzone(int16_t raw) {
  float mag = (float)raw / (float)JOY_AXIS_RANGE;
  if (mag < 0.0f) mag = -mag;
  return mag > JOY_DEADZONE;
}

// Inputs.h -- the two physical toggles and the shaped controller input (architecture.md 5).
//
// Toggles (A0 power, A1 radio mode): read through hal::readPin, debounced over
// TOGGLE_DEBOUNCE_MS (a new level is accepted once the pin has read it for that long without
// a change), polarity TOGGLE_ACTIVE_LOW, mode sense TOGGLE_MODE_WIFI_WHEN_CLOSED. Power on =
// the power toggle is closed (spec 3.1: OFF = toggle open).
//
// Sticks: normalise by JOY_AXIS_RANGE, clamp to -1..1, dead zone rescaled (|x|-dz)/(1-dz),
// expo (1-e)s + e s^3, sign restored. Y is flipped by JOY_INVERT_Y so stickY() > 0 = stick up.
// |x| equal to the dead zone is still inside it.
// Trigger: pulled when the fraction of TRIGGER_AXIS_RANGE is strictly above
// TRIGGER_PRESS_THRESHOLD, released when strictly below TRIGGER_RELEASE_THRESHOLD; a value in
// between (or on a threshold) keeps the previous state. Fractions above 1 are clamped to 1.
// Buttons: a rising edge is accepted at once, then BUTTON_DEBOUNCE_MS of lockout on that button
// ignores further rising edges (chatter); the first rising edge at or after the lockout's end
// counts again. held() is the frame's level, unfiltered.
//
// Link rules (architecture.md 5): edges are generated only from a connected frame
// (frame.valid && frame.connected). A frame that is not connected clears every held level
// (sticks 0, stickActive false, trigger released, buttons up, lockouts reset) without producing
// edges. The first connected frame after that (and the first after begin(), and the first after
// the frame's source changes) is a BASELINE: its levels are taken as they are and produce no
// edges, so a button or trigger already held when the link comes up must be released and
// pressed again to count. This keeps a reconnect, which always lands in DISARMED, from arming
// on an A that was held through the drop.
//
// SPDX-License-Identifier: MIT
#pragma once
#include <stdint.h>
#include "Types.h"

class Inputs {
 public:
  // Hardware senses, from config.h by default; tests flip them to cover both polarities.
  struct Config {
    bool toggleActiveLow;      // TOGGLE_ACTIVE_LOW: a closed toggle reads LOW
    bool modeWifiWhenClosed;   // TOGGLE_MODE_WIFI_WHEN_CLOSED: closed mode toggle = WiFi
    bool invertY;              // JOY_INVERT_Y: negate the wire's Y so + = stick up
  };
  static Config defaults();

  Inputs();                             // defaults()
  explicit Inputs(const Config& cfg);
  const Config& config() const { return cfg_; }

  // Reads the toggles once, without debouncing, so boot starts from their real state
  // (powerChanged()/radioModeChanged() stay false). Clears all controller state.
  void begin(uint32_t nowMs);

  // Once per loop(), after ControlLink::update(). Edge flags hold until the next update().
  void update(uint32_t nowMs, const ControlFrame& frame);

  // ---- toggles (debounced) ----
  bool powerOn() const { return powerOn_; }               // power toggle closed
  bool powerChanged() const { return powerChanged_; }     // flipped in this update()
  uint8_t radioMode() const { return radioMode_; }        // DS_RADIO_* from the A1 toggle
  bool radioModeChanged() const { return radioModeChanged_; }
  bool powerRaw() const { return powerRaw_; }             // undebounced powerOn(), for debug
  bool modeRaw() const { return modeRaw_; }               // undebounced "A1 says WiFi"

  // ---- link ----
  bool connected() const { return connected_; }           // last frame was valid + connected

  // ---- sticks, shaped, -1..1 ----
  float stickX() const { return stickX_; }    // + = stick right
  float stickY() const { return stickY_; }    // + = stick up (JOY_INVERT_Y applied)
  bool stickActive() const { return stickActive_; }   // either axis outside the dead zone

  // ---- trigger ----
  float triggerFraction() const { return triggerFraction_; }   // 0..1 of TRIGGER_AXIS_RANGE
  bool triggerPulled() const { return triggerPulled_; }        // after hysteresis
  bool triggerPullStarted() const { return triggerPullStarted_; }  // released -> pulled now

  // ---- buttons (DS_BTN_* masks) ----
  bool pressed(uint16_t mask) const { return (pressedEdges_ & mask) != 0; }  // rising edge now
  bool held(uint16_t mask) const { return (heldLevels_ & mask) != 0; }       // level (e.g. LB)
  uint16_t pressedEdges() const { return pressedEdges_; }
  uint16_t heldLevels() const { return heldLevels_; }

  // Forget held state (link loss, source change): every level released, lockouts reset, and
  // the next connected frame is a baseline (see the header comment).
  void clearHeld();

  // Pure shaping functions, exposed for the unit tests and the shell.
  static float shapeAxis(int16_t raw);              // normalise, clamp, dead zone, expo
  static bool outsideDeadzone(int16_t raw);
  // The curve on an already normalised value s (clamped to -1..1 here): 0 for |s| <= deadzone
  // (and for NaN), else sign(s) * ((1-expo) m + expo m^3) with m = (|s|-deadzone)/(1-deadzone),
  // capped at 1. shapeAxis(raw) == shape(raw / JOY_AXIS_RANGE, JOY_DEADZONE, JOY_EXPO).
  static float shape(float s, float deadzone, float expo);

  // Bits of ControlFrame::buttons that get edge detection and a lockout timer.
  static constexpr uint8_t kButtonBits = sizeof(uint16_t) * BITS_PER_BYTE;

 private:
  bool toggleClosed(uint8_t pin) const;
  bool readPowerOn() const;
  bool readModeWifi() const;
  void updateToggles(uint32_t nowMs);

  Config cfg_;
  bool powerOn_ = false;
  bool powerChanged_ = false;
  bool powerRaw_ = false;
  uint32_t powerSinceMs_ = 0;      // when powerRaw_ last changed (debounce)
  uint8_t radioMode_ = DS_RADIO_BLE;
  bool radioModeChanged_ = false;
  bool modeRaw_ = false;
  uint32_t modeSinceMs_ = 0;
  bool connected_ = false;
  bool baselinePending_ = true;    // the next connected frame sets levels without edges
  uint8_t source_ = DS_LINK_SRC_NONE;   // source of the last connected frame
  float stickX_ = 0.0f;
  float stickY_ = 0.0f;
  bool stickActive_ = false;
  float triggerFraction_ = 0.0f;
  bool triggerPulled_ = false;
  bool triggerPullStarted_ = false;
  uint16_t pressedEdges_ = 0;
  uint16_t heldLevels_ = 0;
  uint16_t prevButtons_ = 0;
  // per bit of ControlFrame::buttons: when the BUTTON_DEBOUNCE_MS lockout began
  uint32_t lockoutStartMs_[kButtonBits] = {};
  uint16_t lockoutActive_ = 0;     // bits whose lockout is running
};

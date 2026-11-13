// Leds.h -- the 8 WS2812 LEDs (spec 3.8, docs/architecture.md 3, 5 and 6).
//
// What they show, highest priority first (all LED_COUNT LEDs show the same colour; the scheme
// the owner set on 2026-11-03):
//   OFF state        dark, whatever else the model says (spec 3.1: power off = LEDs dark)
//   FAULT state      LED_COLOR_FAULT hard blink: LED_FAULT_BLINK_MS on, LED_FAULT_BLINK_MS off,
//                    starting ON when FAULT is entered
//   servoTeach       LED_COLOR_SERVO_ADJUST solid (DISARMED in SERVO ADJUST; a test stroke too)
//   emptyMagBlink    in DISARMED or ARMED only: a double flash of LED_COLOR_EMPTY_MAG over the
//                    state colour, repeating every LED_EMPTY_FLASH_PERIOD_MS from the moment the
//                    flag rose: LED_EMPTY_FLASH_ON_MS flash, LED_EMPTY_FLASH_GAP_MS state colour,
//                    LED_EMPTY_FLASH_ON_MS flash, then the state colour for the rest of the period
//                    (so ARMED's ready / wait colour stays readable while the magazine reads 0)
//   state colour     BOOT / CALIBRATING: LED_COLOR_STARTUP pulsing between
//                    LED_PULSE_MIN_FRACTION and full; PAIRING: sinusoidal crossfade
//                    LED_COLOR_PAIRING_A -> LED_COLOR_PAIRING_B -> A; DISARMED: LED_COLOR_DISARMED;
//                    ARMED: LED_COLOR_ARMED_READY when DisplayModel::fireReady (a pull would fire
//                    now), else LED_COLOR_ARMED_WAIT (a stroke and its LED_FIRE_FLASH_MIN_MS hold,
//                    the flywheel spin-up, any closed Trigger gate). Semi / full is not shown.
// Pulses and the crossfade are sinusoidal with period LED_PULSE_PERIOD_MS, phase 0 when the
// state was entered (DisplayModel::stateSinceMs): dim (or colour A) at 0, bright (or B) at
// half a period.
//
// Frames: one every 1000/LED_REFRESH_HZ ms, and all LED_COUNT colours are re-sent every time,
// even when unchanged (CODE/ lesson: a corrupted pixel then heals itself at the next frame).
// hal::ledsShow() masks every interrupt for ~241 us, so a rendered frame is shown only when
// hal::stepQuietUs() >= LED_SHOW_GUARD_US (no step edge due in that window). Until then it
// waits, and every update() tries again; after LED_SHOW_MAX_DEFER_MS without such a window it
// is shown anyway and counted in forcedShows(). A frame that comes due while an older one is
// still waiting replaces its colours but keeps the older frame's deadline.
//
// Brightness (architecture.md rule 3, acceptance 16): every channel byte that reaches the HAL
// goes through scale() -- the one brightness function -- and then a runtime check that counts
// (overLimit()) and logs once any byte above LED_MAX_BRIGHTNESS, cutting it to the limit.
// Leds.cpp static_asserts LED_MAX_BRIGHTNESS <= LED_BRIGHTNESS_LIMIT. The HAL's own guard
// (hal::ledsOverLimit) is a further, independent backstop.
//
// SPDX-License-Identifier: MIT
#pragma once
#include <stdint.h>
#include "Types.h"

class Debug;

class Leds {
 public:
  void begin(Debug& debug);       // all dark, shown at once (setup(): no step is running)
  // Once per loop(). nowUs is unused: the quiet window comes from hal::stepQuietUs().
  void update(uint32_t nowMs, uint32_t nowUs, const DisplayModel& model);

  // THE brightness function: 0..LED_CHANNEL_MAX -> 0..LED_MAX_BRIGHTNESS (floor of
  // channel * LED_MAX_BRIGHTNESS / LED_CHANNEL_MAX, so LED_CHANNEL_MAX -> LED_MAX_BRIGHTNESS).
  static uint8_t scale(uint8_t channel);

  // Pure rendering (unit tests). The colour the model asks for at nowMs, at full scale
  // (0..LED_CHANNEL_MAX per channel, before scale()). emptyMagSinceMs: when emptyMagBlink
  // last rose (the double flash's phase 0); update() tracks it.
  static Rgb colorFor(const DisplayModel& model, uint32_t nowMs, uint32_t emptyMagSinceMs);
  // Sinusoid over LED_PULSE_PERIOD_MS: 0 at elapsed 0 and at every whole period, 1 at half.
  static float pulseWave(uint32_t elapsedMs);
  // The BOOT/CALIBRATING brightness factor: LED_PULSE_MIN_FRACTION .. 1.
  static float pulseLevel(uint32_t elapsedMs);
  // A hard blink of LED_FAULT_BLINK_MS on / LED_FAULT_BLINK_MS off, on at elapsed 0.
  static bool blinkOn(uint32_t elapsedMs);
  // The empty-magazine double flash, elapsed since the flag rose: true during the two
  // LED_EMPTY_FLASH_ON_MS flashes of every LED_EMPTY_FLASH_PERIOD_MS (the first at elapsed 0).
  static bool emptyFlashOn(uint32_t elapsedMs);

  // ---- telemetry / debug ----
  Rgb pixel0() const { return shown_; }         // LED 0 as last SHOWN, after scale()
  Rgb pending0() const { return frame_; }       // LED 0 of the newest rendered frame
  bool framePending() const { return framePending_; }   // rendered, waiting for a quiet window
  uint32_t frames() const { return frames_; }           // frames rendered since begin()
  uint32_t shows() const { return shows_; }             // hal::ledsShow() calls since begin()
  uint32_t forcedShows() const { return forcedShows_; } // of which after LED_SHOW_MAX_DEFER_MS
  uint32_t overLimit() const { return overLimit_; }     // runtime brightness check hits

 private:
  uint8_t checked(uint8_t scaledChannel);   // the runtime check after scale()
  void render(const Rgb& full);             // scale + check + hal::ledsSetPixel for every LED
  void show(bool forced);

  Debug* debug_ = nullptr;
  Rgb frame_;                    // newest rendered frame, after scale()
  Rgb shown_;                    // last frame actually shown
  bool framePending_ = false;    // a rendered frame is waiting for a quiet window
  uint32_t pendingSinceMs_ = 0;  // when the oldest unshown frame was rendered
  uint32_t nextFrameMs_ = 0;     // deadline of the next frame
  bool emptyMagWas_ = false;
  uint32_t emptyMagSinceMs_ = 0;
  uint32_t frames_ = 0;
  uint32_t shows_ = 0;
  uint32_t forcedShows_ = 0;
  uint32_t overLimit_ = 0;
};

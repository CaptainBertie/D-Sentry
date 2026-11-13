// Leds.cpp -- see Leds.h.
//
// SPDX-License-Identifier: MIT
#include "Leds.h"

#include <math.h>
#include "Debug.h"
#include "hal.h"

// Spec 3.8 / acceptance 16: half brightness at most, to protect the 5 V rail.
static_assert(LED_MAX_BRIGHTNESS <= LED_BRIGHTNESS_LIMIT,
              "LED_MAX_BRIGHTNESS must never exceed LED_BRIGHTNESS_LIMIT (spec 3.8)");
static_assert(LED_MAX_BRIGHTNESS <= LED_CHANNEL_MAX, "LED_MAX_BRIGHTNESS is one channel byte");
static_assert(LED_REFRESH_HZ > 0 && LED_REFRESH_HZ <= MS_PER_S, "LED frame period is whole ms");
static_assert(LED_PULSE_PERIOD_MS > 0, "LED_PULSE_PERIOD_MS must be positive");
static_assert(LED_FAULT_BLINK_MS > 0, "LED_FAULT_BLINK_MS must be positive");
static_assert(LED_PULSE_MIN_FRACTION >= 0.0f && LED_PULSE_MIN_FRACTION <= 1.0f,
              "LED_PULSE_MIN_FRACTION is a fraction of full brightness");
static_assert(LED_EMPTY_FLASH_ON_MS > 0 && LED_EMPTY_FLASH_GAP_MS > 0,
              "the empty-magazine flashes and the gap between them must be positive");
static_assert(LED_EMPTY_FLASH_ON_MS + LED_EMPTY_FLASH_GAP_MS + LED_EMPTY_FLASH_ON_MS <
                  LED_EMPTY_FLASH_PERIOD_MS,
              "two LED_EMPTY_FLASH_ON_MS flashes and the LED_EMPTY_FLASH_GAP_MS gap must fit inside "
              "LED_EMPTY_FLASH_PERIOD_MS, with a pause left before the next pair");

namespace {

const uint32_t kFrameMs = MS_PER_S / LED_REFRESH_HZ;

Rgb unpack(uint32_t rgb) {
  Rgb c;
  c.r = (uint8_t)((rgb >> COLOR_SHIFT_R) & COLOR_BYTE_MASK);
  c.g = (uint8_t)((rgb >> COLOR_SHIFT_G) & COLOR_BYTE_MASK);
  c.b = (uint8_t)(rgb & COLOR_BYTE_MASK);
  return c;
}

// A float channel value to a byte, rounded and clamped (a float above 255 cast to uint8_t is
// undefined behaviour, so the clamp comes first).
uint8_t toChannel(float v) {
  if (!(v > 0.0f)) return 0;   // also NaN
  if (v >= (float)LED_CHANNEL_MAX) return (uint8_t)LED_CHANNEL_MAX;
  return (uint8_t)(v + MATH_HALF);
}

Rgb dim(const Rgb& c, float level) {
  Rgb out;
  out.r = toChannel((float)c.r * level);
  out.g = toChannel((float)c.g * level);
  out.b = toChannel((float)c.b * level);
  return out;
}

uint8_t mixChannel(uint8_t a, uint8_t b, float w) {
  return toChannel((float)a + ((float)b - (float)a) * w);
}

Rgb mix(const Rgb& a, const Rgb& b, float w) {
  Rgb out;
  out.r = mixChannel(a.r, b.r, w);
  out.g = mixChannel(a.g, b.g, w);
  out.b = mixChannel(a.b, b.b, w);
  return out;
}

}  // namespace

uint8_t Leds::scale(uint8_t channel) {
  return (uint8_t)((uint16_t)channel * LED_MAX_BRIGHTNESS / LED_CHANNEL_MAX);
}

float Leds::pulseWave(uint32_t elapsedMs) {
  const uint32_t t = elapsedMs % LED_PULSE_PERIOD_MS;   // keeps the float phase precise
  const float phase = MATH_TWO_PI * (float)t / (float)LED_PULSE_PERIOD_MS;
  float w = (1.0f - cosf(phase)) * MATH_HALF;
  if (w < 0.0f) w = 0.0f;
  if (w > 1.0f) w = 1.0f;
  return w;
}

float Leds::pulseLevel(uint32_t elapsedMs) {
  return LED_PULSE_MIN_FRACTION + (1.0f - LED_PULSE_MIN_FRACTION) * pulseWave(elapsedMs);
}

bool Leds::blinkOn(uint32_t elapsedMs) { return ((elapsedMs / LED_FAULT_BLINK_MS) & 1u) == 0; }

bool Leds::emptyFlashOn(uint32_t elapsedMs) {
  const uint32_t t = elapsedMs % LED_EMPTY_FLASH_PERIOD_MS;
  const uint32_t secondStart = LED_EMPTY_FLASH_ON_MS + LED_EMPTY_FLASH_GAP_MS;
  return t < LED_EMPTY_FLASH_ON_MS || (t >= secondStart && t < secondStart + LED_EMPTY_FLASH_ON_MS);
}

Rgb Leds::colorFor(const DisplayModel& m, uint32_t nowMs, uint32_t emptyMagSinceMs) {
  const uint32_t inState = nowMs - m.stateSinceMs;   // wrap-safe elapsed time
  if (m.state == DS_STATE_OFF) return Rgb();
  if (m.state == DS_STATE_FAULT) return blinkOn(inState) ? unpack(LED_COLOR_FAULT) : Rgb();
  if (m.servoTeach && m.state == DS_STATE_DISARMED) return unpack(LED_COLOR_SERVO_ADJUST);
  // The empty-magazine double flash, over the state colour (which shows between the flashes).
  const bool emptyShown =
      m.emptyMagBlink && (m.state == DS_STATE_DISARMED || m.state == DS_STATE_ARMED);
  if (emptyShown && emptyFlashOn(nowMs - emptyMagSinceMs)) return unpack(LED_COLOR_EMPTY_MAG);
  switch (m.state) {
    case DS_STATE_BOOT:
    case DS_STATE_CALIBRATING:
      return dim(unpack(LED_COLOR_STARTUP), pulseLevel(inState));
    case DS_STATE_PAIRING:
      return mix(unpack(LED_COLOR_PAIRING_A), unpack(LED_COLOR_PAIRING_B), pulseWave(inState));
    case DS_STATE_DISARMED:
      return unpack(LED_COLOR_DISARMED);
    case DS_STATE_ARMED:   // a pull fires now: red; the next shot is not available yet: yellow
      return unpack(m.fireReady ? LED_COLOR_ARMED_READY : LED_COLOR_ARMED_WAIT);
    default:
      return Rgb();   // not a state: dark
  }
}

void Leds::begin(Debug& debug) {
  debug_ = &debug;
  frame_ = Rgb();
  shown_ = Rgb();
  framePending_ = false;
  pendingSinceMs_ = 0;
  emptyMagWas_ = false;
  emptyMagSinceMs_ = 0;
  frames_ = 0;
  shows_ = 0;
  forcedShows_ = 0;
  overLimit_ = 0;
  const uint32_t nowMs = hal::nowMs();
  nextFrameMs_ = nowMs + kFrameMs;
  render(Rgb());
  show(false);   // setup(): no step is running yet, the interrupt mask costs nothing
}

uint8_t Leds::checked(uint8_t v) {
  if (v <= LED_MAX_BRIGHTNESS) return v;
  overLimit_++;
  if (overLimit_ == 1 && debug_) {
    Debug::Line l;
    l.add("LED brightness check: channel ").addU(v).add(" > LED_MAX_BRIGHTNESS ")
        .addU(LED_MAX_BRIGHTNESS).add(", cut (counted in Leds::overLimit)");
    debug_->log(LOG_ERROR, l);
  }
  return (uint8_t)LED_MAX_BRIGHTNESS;
}

void Leds::render(const Rgb& full) {
  Rgb out;
  out.r = checked(scale(full.r));
  out.g = checked(scale(full.g));
  out.b = checked(scale(full.b));
  for (uint8_t i = 0; i < LED_COUNT; i++) hal::ledsSetPixel(i, out.r, out.g, out.b);
  frame_ = out;
  frames_++;
}

void Leds::show(bool forced) {
  hal::ledsShow();
  shows_++;
  if (forced) forcedShows_++;
  shown_ = frame_;
  framePending_ = false;
}

void Leds::update(uint32_t nowMs, uint32_t /*nowUs*/, const DisplayModel& model) {
  // The empty-magazine double flash starts ON at the moment the flag rises (immediate feedback
  // when the last dart goes), so remember when that was.
  if (model.emptyMagBlink && !emptyMagWas_) emptyMagSinceMs_ = nowMs;
  emptyMagWas_ = model.emptyMagBlink;

  if ((int32_t)(nowMs - nextFrameMs_) >= 0) {
    nextFrameMs_ += kFrameMs;
    // A long stall (or the first frame after begin) must not turn into a burst of catch-up
    // frames: start the schedule again from now.
    if ((int32_t)(nowMs - nextFrameMs_) >= 0) nextFrameMs_ = nowMs + kFrameMs;
    render(colorFor(model, nowMs, emptyMagSinceMs_));
    if (!framePending_) {
      framePending_ = true;
      pendingSinceMs_ = nowMs;
    }
  }
  if (!framePending_) return;
  // Check the window immediately before showing: show() masks interrupts for ~241 us and a
  // step edge due inside that would be delayed.
  if (hal::stepQuietUs() >= LED_SHOW_GUARD_US) {
    show(false);
  } else if ((uint32_t)(nowMs - pendingSinceMs_) >= LED_SHOW_MAX_DEFER_MS) {
    show(true);
  }
}

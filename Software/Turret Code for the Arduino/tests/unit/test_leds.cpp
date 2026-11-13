// test_leds.cpp -- Leds against spec 3.8, acceptance 11/12/16 and docs/architecture.md 3, 5, 6,
// with the owner's scheme of 2026-11-03: ARMED red when a pull would fire now, yellow while the
// next shot is not available; a purple double flash over the state colour at ammo 0; SERVO
// TEACH cyan over it; FAULT over everything.
//
// KEY SAFETY TEST: brightness_never_exceeds_limit_through_the_module drives the real module
// through every state x fire mode x flag x fault x pairing phase over a whole pulse period and
// checks the bytes that reached the HAL. It fails if any path bypasses Leds::scale() (the
// host HAL records the largest raw byte ever passed to ledsSetPixel, before its own guard).
//
// SPDX-License-Identifier: MIT
#include <algorithm>
#include <cmath>
#include <cstdio>
#include <vector>

#include "Debug.h"
#include "Leds.h"
#include "hal.h"
#include "hal_host.h"
#include "testkit.h"

namespace {

const uint32_t kFrameMs = MS_PER_S / LED_REFRESH_HZ;

Rgb rgbOf(uint32_t c) {
  Rgb x;
  x.r = (uint8_t)((c >> COLOR_SHIFT_R) & COLOR_BYTE_MASK);
  x.g = (uint8_t)((c >> COLOR_SHIFT_G) & COLOR_BYTE_MASK);
  x.b = (uint8_t)(c & COLOR_BYTE_MASK);
  return x;
}
bool same(const Rgb& a, const Rgb& b) { return a.r == b.r && a.g == b.g && a.b == b.b; }
bool dark(const Rgb& a) { return a.r == 0 && a.g == 0 && a.b == 0; }
Rgb scaled(const Rgb& a) {
  Rgb x;
  x.r = Leds::scale(a.r);
  x.g = Leds::scale(a.g);
  x.b = Leds::scale(a.b);
  return x;
}

DisplayModel model(uint8_t state, uint32_t sinceMs = 0) {
  DisplayModel m;
  m.state = state;
  m.stateSinceMs = sinceMs;
  m.linkConnected = state == DS_STATE_DISARMED || state == DS_STATE_ARMED;
  m.ammo = MAG_CAPACITY;
  return m;
}

// The empty-magazine double flash from config.h alone (not Leds::emptyFlashOn): within each
// LED_EMPTY_FLASH_PERIOD_MS, purple for LED_EMPTY_FLASH_ON_MS, the colour underneath for
// LED_EMPTY_FLASH_GAP_MS, purple again for LED_EMPTY_FLASH_ON_MS, then the colour underneath.
bool purpleExpected(uint32_t sinceRiseMs) {
  const uint32_t t = sinceRiseMs % LED_EMPTY_FLASH_PERIOD_MS;
  if (t < LED_EMPTY_FLASH_ON_MS) return true;
  if (t < LED_EMPTY_FLASH_ON_MS + LED_EMPTY_FLASH_GAP_MS) return false;
  return t < 2u * LED_EMPTY_FLASH_ON_MS + LED_EMPTY_FLASH_GAP_MS;
}

// What ARMED / DISARMED show without the empty overlay.
uint32_t stateColour(const DisplayModel& m) {
  if (m.state == DS_STATE_DISARMED) return LED_COLOR_DISARMED;
  return m.fireReady ? LED_COLOR_ARMED_READY : LED_COLOR_ARMED_WAIT;
}

struct Rig {
  Debug debug;
  Leds leds;
  Rig() {
    hal_host::reset();
    hal::begin();
    DebugSources none;
    debug.begin(none);
    leds.begin(debug);
  }
  void step(uint64_t us, const DisplayModel& m) {
    hal_host::advanceUs(us);
    leds.update(hal::nowMs(), hal::nowUs(), m);
  }
};

}  // namespace

// ---- the one brightness function ------------------------------------------------------------------
TEST(scale_maps_full_range_onto_the_limit) {
  CHECK_EQ(Leds::scale(0), 0);
  CHECK_EQ(Leds::scale(LED_CHANNEL_MAX), LED_MAX_BRIGHTNESS);
  uint8_t prev = 0;
  for (uint32_t v = 0; v <= LED_CHANNEL_MAX; v++) {
    const uint8_t s = Leds::scale((uint8_t)v);
    CHECK(s <= LED_MAX_BRIGHTNESS);
    CHECK(s >= prev);   // monotonic: a brighter request is never dimmer
    prev = s;
  }
  // The config the spec demands (spec 3.8: half brightness, acceptance 16).
  CHECK(LED_MAX_BRIGHTNESS <= LED_BRIGHTNESS_LIMIT);
}

// ---- acceptance 16, through the real module and the HAL -------------------------------------------------
TEST(brightness_never_exceeds_limit_through_the_module) {
  Rig rig;
  const uint8_t faults[] = {DS_FAULT_NONE, DS_FAULT_CAL, DS_FAULT_STALL};
  const PairingPhase phases[] = {PairingPhase::FROM_BOOT, PairingPhase::GRACE, PairingPhase::NO_LINK};
  const uint32_t stepMs = kFrameMs + 1;   // not a divisor of the pulse period: many phases
  uint32_t combos = 0;
  uint8_t worstShown = 0;
  // The empty double flash's whole period too, so its purple frames are in the sweep.
  const uint32_t sweepMs = std::max<uint32_t>(LED_PULSE_PERIOD_MS, LED_EMPTY_FLASH_PERIOD_MS);
  for (uint8_t state = 0; state <= DS_STATE_COUNT; state++) {   // DS_STATE_COUNT: not a state
    for (uint8_t fire = 0; fire < DS_FIRE_MODE_COUNT; fire++) {
      for (uint8_t flags = 0; flags < 16; flags++) {
        for (uint8_t fault : faults) {
          for (PairingPhase phase : phases) {
            DisplayModel m = model(state, hal::nowMs());
            m.fireMode = fire;
            m.fireReady = flags & 1;
            m.emptyMagBlink = flags & 2;
            m.linkConnected = flags & 4;
            m.servoTeach = flags & 8;
            m.ammo = m.emptyMagBlink ? 0 : MAG_CAPACITY;
            m.fault = fault;
            m.pairing = phase;
            combos++;
            for (uint32_t t = 0; t <= sweepMs; t += stepMs) {
              rig.step((uint64_t)stepMs * US_PER_MS, m);
              const Rgb c = hal_host::ledShown(0);
              worstShown = std::max(worstShown, std::max(c.r, std::max(c.g, c.b)));
            }
          }
        }
      }
    }
  }
  CHECK_EQ(combos, (DS_STATE_COUNT + 1u) * DS_FIRE_MODE_COUNT * 16u * 3u * 3u);
  CHECK(hal_host::ledMaxChannel() <= LED_MAX_BRIGHTNESS);   // raw bytes given to the HAL
  CHECK_EQ(hal_host::ledOverLimit(), 0u);                   // the HAL's guard never had to cut
  CHECK_EQ(rig.leds.overLimit(), 0u);                       // nor Leds' own runtime check
  CHECK(worstShown <= LED_MAX_BRIGHTNESS);
  // Not vacuous: full-scale white / red / green did reach the LEDs, at exactly the cap.
  CHECK_EQ(hal_host::ledMaxChannel(), LED_MAX_BRIGHTNESS);
  CHECK_EQ(worstShown, LED_MAX_BRIGHTNESS);
}

// ---- priorities (owner 2026-11-03), swept over every state, flag and many timestamps --------------------
// FAULT blink > SERVO ADJUST cyan > EMPTY purple double flash (over the state colour) > state colour.
TEST(priority_rules_hold_for_every_state_flag_and_time) {
  const Rgb fault = rgbOf(LED_COLOR_FAULT);
  const Rgb teach = rgbOf(LED_COLOR_SERVO_ADJUST);
  const Rgb purple = rgbOf(LED_COLOR_EMPTY_MAG);
  const Rgb startup = rgbOf(LED_COLOR_STARTUP);
  // stateSinceMs values on both sides of the millis() wrap; the empty flag rose a little later
  const uint32_t sinces[] = {0u, 12345u, UINT32_MAX - LED_PULSE_PERIOD_MS / 2u};
  const uint32_t riseAfter = LED_EMPTY_FLASH_PERIOD_MS / 3u;
  uint32_t samples = 0, purpleSeen = 0, underneathSeen = 0;
  for (uint8_t state = 0; state <= DS_STATE_COUNT; state++) {
    for (uint8_t fire = 0; fire < DS_FIRE_MODE_COUNT; fire++) {
      for (uint8_t flags = 0; flags < 8; flags++) {
        for (uint32_t since : sinces) {
          DisplayModel m = model(state, since);
          m.fireMode = fire;
          m.fireReady = flags & 1;
          m.emptyMagBlink = flags & 2;
          m.servoTeach = flags & 4;
          m.ammo = m.emptyMagBlink ? 0 : MAG_CAPACITY;
          m.fault = DS_FAULT_STALL;
          const uint32_t rose = since + riseAfter;
          const uint32_t span = 2u * std::max<uint32_t>(LED_PULSE_PERIOD_MS, LED_EMPTY_FLASH_PERIOD_MS);
          for (uint32_t t = 0; t < span; t += 3) {
            const uint32_t now = since + t;
            const Rgb c = Leds::colorFor(m, now, rose);
            samples++;
            const bool control = state == DS_STATE_DISARMED || state == DS_STATE_ARMED;
            if (state == DS_STATE_OFF) {
              if (!CHECK(dark(c))) return;                      // OFF is dark, whatever the flags
            } else if (state == DS_STATE_FAULT) {
              if (!CHECK(same(c, fault) || dark(c))) return;    // FAULT beats TEACH and EMPTY
            } else if (m.servoTeach && state == DS_STATE_DISARMED) {
              if (!CHECK(same(c, teach))) return;               // TEACH beats EMPTY, solid
            } else if (m.emptyMagBlink && control && purpleExpected(now - rose)) {
              if (!CHECK(same(c, purple))) return;              // a flash: purple
              purpleSeen++;
            } else if (state == DS_STATE_BOOT || state == DS_STATE_CALIBRATING) {
              // pulse: never above the colour, never below its floor (never fully dark); the
              // empty flag shows nothing outside DISARMED / ARMED
              const float floorR = (float)startup.r * LED_PULSE_MIN_FRACTION;
              if (!CHECK(c.r <= startup.r && c.r + 1.0f >= floorR)) return;
              if (!CHECK(c.r == c.g && c.g == c.b)) return;     // white stays white
            } else if (state == DS_STATE_PAIRING) {
              if (!CHECK(!same(c, purple) && !same(c, teach))) return;
            } else if (control) {
              // the state colour, between the flashes too: green, or ARMED red (ready) / yellow
              // (wait), whatever the fire mode
              if (!CHECK(same(c, rgbOf(stateColour(m))))) return;
              if (m.emptyMagBlink) underneathSeen++;
            } else if (state == DS_STATE_COUNT) {
              if (!CHECK(dark(c))) return;                      // not a state: dark
            }
          }
        }
      }
    }
  }
  CHECK(samples > 100000u);
  CHECK(purpleSeen > 0u);
  CHECK(underneathSeen > purpleSeen);   // most of each period is the colour underneath
}

TEST(fault_blink_is_hard_on_off_starting_on_and_beats_everything) {
  DisplayModel m = model(DS_STATE_FAULT, 1000);
  m.fireReady = true;       // FAULT still wins: the off phase is dark, never another colour
  m.emptyMagBlink = true;
  m.ammo = 0;
  m.servoTeach = true;
  const Rgb fault = rgbOf(LED_COLOR_FAULT);
  CHECK(same(Leds::colorFor(m, 1000, 1000), fault));
  CHECK(same(Leds::colorFor(m, 1000 + LED_FAULT_BLINK_MS - 1, 1000), fault));
  CHECK(dark(Leds::colorFor(m, 1000 + LED_FAULT_BLINK_MS, 1000)));
  CHECK(dark(Leds::colorFor(m, 1000 + 2 * LED_FAULT_BLINK_MS - 1, 1000)));
  CHECK(same(Leds::colorFor(m, 1000 + 2 * LED_FAULT_BLINK_MS, 1000), fault));
  for (uint32_t t = 0; t < 2 * LED_EMPTY_FLASH_PERIOD_MS; t++) {
    const Rgb c = Leds::colorFor(m, 1000 + t, 1000);
    if (!CHECK(same(c, fault) || dark(c))) return;
  }
}

// ARMED: red when a pull would fire now, yellow while the next shot is not available; the fire
// mode is not shown (the LCD's ARMED S / F is).
TEST(state_colours_match_config) {
  CHECK(same(Leds::colorFor(model(DS_STATE_DISARMED), 0, 0), rgbOf(LED_COLOR_DISARMED)));
  for (uint8_t fire = 0; fire < DS_FIRE_MODE_COUNT; fire++) {
    DisplayModel armed = model(DS_STATE_ARMED);
    armed.fireMode = fire;
    armed.fireReady = true;
    CHECK(same(Leds::colorFor(armed, 0, 0), rgbOf(LED_COLOR_ARMED_READY)));
    armed.fireReady = false;
    CHECK(same(Leds::colorFor(armed, 0, 0), rgbOf(LED_COLOR_ARMED_WAIT)));
  }
  DisplayModel disarmed = model(DS_STATE_DISARMED);
  disarmed.fireReady = true;   // meaningless outside ARMED
  CHECK(same(Leds::colorFor(disarmed, 0, 0), rgbOf(LED_COLOR_DISARMED)));
  CHECK(dark(Leds::colorFor(model(DS_STATE_OFF), 0, 0)));
  // PAIRING crossfade: colour A when entered, colour B half a period later, A again after one.
  const DisplayModel p = model(DS_STATE_PAIRING, 500);
  CHECK(same(Leds::colorFor(p, 500, 0), rgbOf(LED_COLOR_PAIRING_A)));
  CHECK(same(Leds::colorFor(p, 500 + LED_PULSE_PERIOD_MS / 2, 0), rgbOf(LED_COLOR_PAIRING_B)));
  CHECK(same(Leds::colorFor(p, 500 + LED_PULSE_PERIOD_MS, 0), rgbOf(LED_COLOR_PAIRING_A)));
  // a quarter period in: halfway between A and B on every channel
  const Rgb a = rgbOf(LED_COLOR_PAIRING_A), b = rgbOf(LED_COLOR_PAIRING_B);
  const Rgb q = Leds::colorFor(p, 500 + LED_PULSE_PERIOD_MS / 4, 0);
  CHECK_NEAR(q.r, (a.r + b.r) / 2.0, 1.0);
  CHECK_NEAR(q.g, (a.g + b.g) / 2.0, 1.0);
  CHECK_NEAR(q.b, (a.b + b.b) / 2.0, 1.0);
}

// ---- pulse shape: sinusoidal, period LED_PULSE_PERIOD_MS, floor LED_PULSE_MIN_FRACTION ---------------------
TEST(pulse_has_the_configured_period_and_floor) {
  CHECK_NEAR(Leds::pulseLevel(0), LED_PULSE_MIN_FRACTION, 1e-6);
  CHECK_NEAR(Leds::pulseLevel(LED_PULSE_PERIOD_MS / 2), 1.0, 1e-6);
  CHECK_NEAR(Leds::pulseLevel(LED_PULSE_PERIOD_MS), LED_PULSE_MIN_FRACTION, 1e-6);
  CHECK_NEAR(Leds::pulseLevel(LED_PULSE_PERIOD_MS / 4), (1.0 + LED_PULSE_MIN_FRACTION) / 2.0, 1e-3);
  float lo = 2.0f, hi = -1.0f;
  for (uint32_t t = 0; t < 3 * LED_PULSE_PERIOD_MS; t++) {
    const float v = Leds::pulseLevel(t);
    lo = std::min(lo, v);
    hi = std::max(hi, v);
    // the exact sinusoid the contract names
    const double want = LED_PULSE_MIN_FRACTION + (1.0 - LED_PULSE_MIN_FRACTION) *
                        (1.0 - std::cos(2.0 * M_PI * (double)(t % LED_PULSE_PERIOD_MS) / LED_PULSE_PERIOD_MS)) / 2.0;
    if (!CHECK_NEAR(v, want, 1e-4)) return;
    if (!CHECK_NEAR(v, Leds::pulseLevel(t + LED_PULSE_PERIOD_MS), 1e-5)) return;   // periodic
  }
  CHECK_NEAR(lo, LED_PULSE_MIN_FRACTION, 1e-5);
  CHECK_NEAR(hi, 1.0, 1e-5);
  // The BOOT colour at its dimmest and brightest, and the wrap of millis() mid-pulse.
  const Rgb startup = rgbOf(LED_COLOR_STARTUP);
  const DisplayModel boot = model(DS_STATE_BOOT, UINT32_MAX - 10);
  const Rgb dim = Leds::colorFor(boot, UINT32_MAX - 10, 0);
  CHECK_EQ(dim.r, (uint8_t)std::lround(startup.r * LED_PULSE_MIN_FRACTION));
  const Rgb bright = Leds::colorFor(boot, (UINT32_MAX - 10) + LED_PULSE_PERIOD_MS / 2, 0);
  CHECK(same(bright, startup));
}

// ---- frames: every 1000/LED_REFRESH_HZ ms, always re-sent --------------------------------------------------
TEST(frames_are_resent_every_refresh_even_when_unchanged) {
  Rig rig;
  const DisplayModel m = model(DS_STATE_DISARMED, 0);
  const uint32_t showsBefore = hal_host::ledShowCount();
  const uint32_t framesBefore = rig.leds.frames();
  for (uint32_t i = 0; i < MS_PER_S; i++) rig.step(US_PER_MS, m);   // one second, steppers still
  const uint32_t shows = hal_host::ledShowCount() - showsBefore;
  CHECK_EQ(shows, (uint32_t)LED_REFRESH_HZ);
  CHECK_EQ(rig.leds.frames() - framesBefore, (uint32_t)LED_REFRESH_HZ);
  CHECK_EQ(rig.leds.forcedShows(), 0u);
  CHECK_EQ(hal_host::ledUnsafeShows(), 0u);
  for (uint8_t i = 0; i < LED_COUNT; i++)   // every LED, not just LED 0
    CHECK(same(hal_host::ledShown(i), scaled(rgbOf(LED_COLOR_DISARMED))));
  CHECK(same(rig.leds.pixel0(), scaled(rgbOf(LED_COLOR_DISARMED))));
}

TEST(pixel0_is_led0_as_sent_after_the_clamp) {
  Rig rig;
  DisplayModel m = model(DS_STATE_ARMED, 0);
  m.fireMode = DS_FIRE_MODE_FULL;
  for (uint32_t i = 0; i < 2 * kFrameMs; i++) rig.step(US_PER_MS, m);
  Rgb p = rig.leds.pixel0();
  CHECK(same(p, scaled(rgbOf(LED_COLOR_ARMED_WAIT))));
  CHECK(same(p, hal_host::ledShown(0)));
  if (LED_COLOR_ARMED_WAIT == 0xFFFF00 && LED_MAX_BRIGHTNESS == 127) {   // yellow at half
    CHECK_EQ(p.r, 127);
    CHECK_EQ(p.g, 127);
    CHECK_EQ(p.b, 0);
  }
  m.fireReady = true;
  for (uint32_t i = 0; i < 2 * kFrameMs; i++) rig.step(US_PER_MS, m);
  p = rig.leds.pixel0();
  CHECK(same(p, scaled(rgbOf(LED_COLOR_ARMED_READY))));
  if (LED_COLOR_ARMED_READY == 0xFF0000 && LED_MAX_BRIGHTNESS == 127) {  // red at half
    CHECK_EQ(p.r, 127);
    CHECK_EQ(p.g, 0);
    CHECK_EQ(p.b, 0);
  }
}

// The owner's purple 0x8000FF through the one brightness function: floor(128 * 127 / 255) = 63
// red, 127 blue, on every LED.
TEST(purple_reaches_the_leds_through_the_clamp) {
  Rig rig;
  DisplayModel m = model(DS_STATE_DISARMED, 0);
  m.ammo = 0;
  m.emptyMagBlink = true;
  for (uint32_t i = 0; i <= kFrameMs; i++) rig.step(US_PER_MS, m);   // the first flash
  const Rgb want = scaled(rgbOf(LED_COLOR_EMPTY_MAG));
  for (uint8_t k = 0; k < LED_COUNT; k++) CHECK(same(hal_host::ledShown(k), want));
  CHECK(want.r <= LED_MAX_BRIGHTNESS && want.g <= LED_MAX_BRIGHTNESS && want.b <= LED_MAX_BRIGHTNESS);
  if (LED_COLOR_EMPTY_MAG == 0x8000FF && LED_MAX_BRIGHTNESS == 127) {
    CHECK_EQ(want.r, 63);
    CHECK_EQ(want.g, 0);
    CHECK_EQ(want.b, 127);
  }
  CHECK_EQ(rig.leds.overLimit(), 0u);
  CHECK(hal_host::ledMaxChannel() <= LED_MAX_BRIGHTNESS);
}

// TurretState holds fireReady false for a stroke and at least LED_FIRE_FLASH_MIN_MS; the LEDs
// must turn yellow within one frame of it, stay yellow that long (to within one frame) on every
// LED, then go back to red.
TEST(armed_shows_yellow_while_not_ready_then_red_again) {
  Rig rig;
  DisplayModel m = model(DS_STATE_ARMED, 0);
  m.fireReady = true;
  for (uint32_t i = 0; i < 33; i++) rig.step(US_PER_MS, m);
  const Rgb red = scaled(rgbOf(LED_COLOR_ARMED_READY));
  const Rgb yellow = scaled(rgbOf(LED_COLOR_ARMED_WAIT));
  CHECK(same(hal_host::ledShown(0), red));
  const uint32_t start = hal::nowMs();
  uint32_t firstYellow = 0, lastYellow = 0;
  for (uint32_t i = 0; i < 3 * LED_FIRE_FLASH_MIN_MS; i++) {
    m.fireReady = hal::nowMs() - start >= LED_FIRE_FLASH_MIN_MS;
    rig.step(US_PER_MS, m);
    bool allYellow = true;
    for (uint8_t k = 0; k < LED_COUNT; k++) allYellow = allYellow && same(hal_host::ledShown(k), yellow);
    if (allYellow) {
      if (!firstYellow) firstYellow = hal::nowMs();
      lastYellow = hal::nowMs();
    }
  }
  CHECK(firstYellow > 0u);
  CHECK(firstYellow - start <= kFrameMs + 1);
  CHECK(lastYellow - firstYellow + 1 >= LED_FIRE_FLASH_MIN_MS - kFrameMs);
  CHECK(lastYellow - start <= LED_FIRE_FLASH_MIN_MS + kFrameMs + 1);
  CHECK(same(hal_host::ledShown(0), red));   // back to red
}

// The double flash, millisecond by millisecond against config.h: purple, the colour underneath,
// purple, the colour underneath, every LED_EMPTY_FLASH_PERIOD_MS from the rise; the colour
// underneath is green (DISARMED), red (ARMED, ready) or yellow (ARMED, wait).
TEST(empty_double_flash_timing_is_exact_with_the_state_colour_between) {
  const Rgb purple = rgbOf(LED_COLOR_EMPTY_MAG);
  struct Case {
    uint8_t state;
    bool ready;
  };
  const uint32_t rises[] = {0u, 777u, UINT32_MAX - LED_EMPTY_FLASH_PERIOD_MS / 2u};   // wrap too
  for (const Case c : {Case{DS_STATE_DISARMED, false}, Case{DS_STATE_ARMED, true}, Case{DS_STATE_ARMED, false}}) {
    for (uint32_t rose : rises) {
      DisplayModel m = model(c.state, rose - LED_PULSE_PERIOD_MS);
      m.fireReady = c.ready;
      m.ammo = 0;
      m.emptyMagBlink = true;
      const Rgb under = rgbOf(stateColour(m));
      uint32_t purpleMs = 0;
      for (uint32_t t = 0; t < 3 * LED_EMPTY_FLASH_PERIOD_MS; t++) {
        const Rgb got = Leds::colorFor(m, rose + t, rose);
        const bool flash = purpleExpected(t);
        if (!CHECK(same(got, flash ? purple : under))) return;
        if (!CHECK_EQ(Leds::emptyFlashOn(t), flash)) return;
        purpleMs += flash;
      }
      CHECK_EQ(purpleMs, 3u * 2u * LED_EMPTY_FLASH_ON_MS);   // two flashes a period, exactly
    }
  }
  // The edges, spelled out.
  CHECK(Leds::emptyFlashOn(0));
  CHECK(Leds::emptyFlashOn(LED_EMPTY_FLASH_ON_MS - 1));
  CHECK(!Leds::emptyFlashOn(LED_EMPTY_FLASH_ON_MS));
  CHECK(!Leds::emptyFlashOn(LED_EMPTY_FLASH_ON_MS + LED_EMPTY_FLASH_GAP_MS - 1));
  CHECK(Leds::emptyFlashOn(LED_EMPTY_FLASH_ON_MS + LED_EMPTY_FLASH_GAP_MS));
  CHECK(Leds::emptyFlashOn(2 * LED_EMPTY_FLASH_ON_MS + LED_EMPTY_FLASH_GAP_MS - 1));
  CHECK(!Leds::emptyFlashOn(2 * LED_EMPTY_FLASH_ON_MS + LED_EMPTY_FLASH_GAP_MS));
  CHECK(!Leds::emptyFlashOn(LED_EMPTY_FLASH_PERIOD_MS - 1));
  CHECK(Leds::emptyFlashOn(LED_EMPTY_FLASH_PERIOD_MS));
  CHECK(2 * LED_EMPTY_FLASH_ON_MS + LED_EMPTY_FLASH_GAP_MS < LED_EMPTY_FLASH_PERIOD_MS);   // a pause
}

// Not in OFF, BOOT, CALIBRATING, PAIRING or FAULT, whatever the flag says.
TEST(empty_flash_shows_only_in_disarmed_and_armed) {
  const Rgb purple = rgbOf(LED_COLOR_EMPTY_MAG);
  for (uint8_t state : {DS_STATE_OFF, DS_STATE_BOOT, DS_STATE_CALIBRATING, DS_STATE_PAIRING, DS_STATE_FAULT}) {
    DisplayModel m = model(state, 0);
    m.ammo = 0;
    m.emptyMagBlink = true;
    DisplayModel noFlag = m;
    noFlag.emptyMagBlink = false;
    for (uint32_t t = 0; t < 2 * LED_EMPTY_FLASH_PERIOD_MS; t += 7) {
      const Rgb c = Leds::colorFor(m, t, 0);
      if (!CHECK(same(c, Leds::colorFor(noFlag, t, 0)))) return;   // the flag changes nothing
      if (!CHECK(!same(c, purple))) return;
    }
  }
}

// Through the module: the first flash comes within one frame of the rise (not waiting for a
// global phase), the colour underneath shows in the gap and after the second flash.
TEST(empty_flash_starts_on_when_the_flag_rises) {
  Rig rig;
  DisplayModel m = model(DS_STATE_ARMED, 0);
  m.fireReady = true;
  m.ammo = 0;
  for (uint32_t i = 0; i < 77; i++) rig.step(US_PER_MS, m);   // an arbitrary phase
  m.emptyMagBlink = true;
  const uint32_t rose = hal::nowMs() + 1;
  const Rgb on = scaled(rgbOf(LED_COLOR_EMPTY_MAG));
  const Rgb red = scaled(rgbOf(LED_COLOR_ARMED_READY));
  for (uint32_t i = 0; i <= kFrameMs; i++) rig.step(US_PER_MS, m);
  CHECK(same(hal_host::ledShown(0), on));
  // Every shown frame is purple exactly when the timeline says so (frames render on whole ms).
  uint32_t purpleFrames = 0, redFrames = 0;
  uint32_t lastShows = rig.leds.shows();
  while (hal::nowMs() < rose + 2 * LED_EMPTY_FLASH_PERIOD_MS) {
    rig.step(US_PER_MS, m);
    if (rig.leds.shows() == lastShows) continue;
    lastShows = rig.leds.shows();
    const bool flash = purpleExpected(hal::nowMs() - rose);
    if (!CHECK(same(hal_host::ledShown(0), flash ? on : red))) return;
    (flash ? purpleFrames : redFrames)++;
  }
  CHECK(purpleFrames > 0u && redFrames > purpleFrames);
  // X: the flag drops, the colour underneath stays.
  m.emptyMagBlink = false;
  m.ammo = MAG_CAPACITY;
  for (uint32_t i = 0; i < LED_EMPTY_FLASH_PERIOD_MS; i++) {
    rig.step(US_PER_MS, m);
    if (i > kFrameMs && !CHECK(same(hal_host::ledShown(0), red))) return;
  }
}

// ---- the step-quiet window (architecture.md 3) ------------------------------------------------------------
TEST(show_waits_for_a_quiet_window_and_never_cuts_a_step) {
  Rig rig;
  hal::stepperEnable(true);
  // An edge every 1 ms, 100 us after each whole ms (the first edge comes one period after the
  // call): a frame rendered on a whole ms has its next edge 100 us away and must wait.
  hal_host::advanceUs(100);
  hal::stepSetRate(AXIS_PAN, 1000.0f);
  const DisplayModel m = model(DS_STATE_DISARMED, 0);
  const uint32_t before = hal_host::ledShowCount();
  uint32_t deferredLoops = 0;
  for (uint32_t i = 0; i < 20000; i++) {   // 1 s in 50 us loops
    rig.step(50, m);
    if (rig.leds.framePending()) deferredLoops++;
  }
  CHECK_EQ(hal_host::ledUnsafeShows(), 0u);   // no show ever overlapped a due step edge
  CHECK_EQ(rig.leds.forcedShows(), 0u);
  CHECK_EQ(hal_host::ledShowCount() - before, (uint32_t)LED_REFRESH_HZ);
  CHECK(deferredLoops > 0u);                  // some frames did have to wait
  CHECK(hal_host::totalSteps(AXIS_PAN) >= 990u);
}

TEST(show_is_forced_after_max_defer_and_counted) {
  Rig rig;
  // Two axes at the top rate, half a period apart: the gap to the next edge never reaches
  // LED_SHOW_GUARD_US, so every show has to be forced.
  const uint32_t periodUs = (uint32_t)(US_PER_S / STEP_MAX_RATE_HZ);
  if (periodUs / 2 + 2 >= LED_SHOW_GUARD_US) {
    std::printf("info show_is_forced_after_max_defer_and_counted: config leaves quiet windows "
                "at STEP_MAX_RATE_HZ; skipped\n");
    return;
  }
  hal::stepperEnable(true);
  hal::stepSetRate(AXIS_PAN, STEP_MAX_RATE_HZ);
  hal_host::advanceUs(periodUs / 2);
  hal::stepSetRate(AXIS_TILT, STEP_MAX_RATE_HZ);
  hal_host::advanceUs(2 * periodUs);   // both running
  const DisplayModel m = model(DS_STATE_ARMED, 0);
  const uint32_t showsBefore = hal_host::ledShowCount();
  std::vector<uint64_t> showTimes;
  uint64_t lastSeen = hal_host::lastShowUs();
  uint32_t quietSeen = 0;
  for (uint32_t i = 0; i < 20000; i++) {   // 1 s in 50 us loops
    rig.step(50, m);
    if (hal::stepQuietUs() >= LED_SHOW_GUARD_US) quietSeen++;
    if (hal_host::lastShowUs() != lastSeen) {
      lastSeen = hal_host::lastShowUs();
      showTimes.push_back(lastSeen);
    }
  }
  CHECK_EQ(quietSeen, 0u);   // the scenario really has no quiet window
  const uint32_t shows = hal_host::ledShowCount() - showsBefore;
  CHECK(shows > 0u);
  CHECK_EQ(rig.leds.forcedShows(), shows);              // every show was a forced one
  CHECK_EQ(hal_host::ledUnsafeShows(), shows);          // and only forced ones were unsafe
  // A forced show comes LED_SHOW_MAX_DEFER_MS after its frame was rendered, never earlier, and
  // the next frame is rendered within one frame period of it: gaps of DEFER .. DEFER + frame.
  for (size_t i = 1; i < showTimes.size(); i++) {
    const uint64_t gapMs = (showTimes[i] - showTimes[i - 1]) / US_PER_MS;
    CHECK(gapMs >= LED_SHOW_MAX_DEFER_MS);
    CHECK(gapMs <= LED_SHOW_MAX_DEFER_MS + kFrameMs + 1);
  }
  CHECK(shows >= MS_PER_S / (LED_SHOW_MAX_DEFER_MS + kFrameMs + 1));
  CHECK(shows <= MS_PER_S / LED_SHOW_MAX_DEFER_MS + 1);
  // the newest colours went out (frames kept rendering while waiting)
  CHECK(same(hal_host::ledShown(0), scaled(rgbOf(LED_COLOR_ARMED_WAIT))));
}

TEST(no_catch_up_burst_after_a_long_stall) {
  Rig rig;
  const DisplayModel m = model(DS_STATE_DISARMED, 0);
  rig.step(US_PER_MS, m);
  const uint32_t frames = rig.leds.frames();
  rig.step((uint64_t)10 * kFrameMs * US_PER_MS, m);   // loop() stalled for ten frame periods
  CHECK_EQ(rig.leds.frames(), frames + 1);             // one frame, not ten
  rig.step(US_PER_MS, m);
  CHECK_EQ(rig.leds.frames(), frames + 1);
  rig.step((uint64_t)kFrameMs * US_PER_MS, m);
  CHECK_EQ(rig.leds.frames(), frames + 2);
}

TEST(begin_leaves_every_led_dark_and_shown) {
  Rig rig;
  for (uint8_t i = 0; i < LED_COUNT; i++) CHECK(dark(hal_host::ledShown(i)));
  CHECK_EQ(rig.leds.shows(), 1u);
  CHECK(dark(rig.leds.pixel0()));
}

// ---- SERVO ADJUST colour (Leds.h) ---------------------------------------------------------------------
// LED_COLOR_SERVO_ADJUST in DISARMED's adjust mode: below FAULT only, over the empty-magazine
// flash (solid, every millisecond of its period), through the clamp.
TEST(servo_adjust_colour_sits_below_fault_over_the_empty_flash_and_obeys_the_clamp) {
  DisplayModel m = model(DS_STATE_DISARMED);
  m.servoTeach = true;
  CHECK(same(Leds::colorFor(m, 0, 0), rgbOf(LED_COLOR_SERVO_ADJUST)));
  m.ammo = 0;
  m.emptyMagBlink = true;              // never together from TurretState, but cyan would win
  for (uint32_t t = 0; t < 2 * LED_EMPTY_FLASH_PERIOD_MS; t++)
    if (!CHECK(same(Leds::colorFor(m, t, 0), rgbOf(LED_COLOR_SERVO_ADJUST)))) return;
  m.state = DS_STATE_FAULT;            // never together, but FAULT would win
  CHECK(same(Leds::colorFor(m, 0, 0), rgbOf(LED_COLOR_FAULT)));
  m.emptyMagBlink = false;
  m.ammo = MAG_CAPACITY;
  m.state = DS_STATE_ARMED;            // the flag means nothing outside DISARMED
  CHECK(!same(Leds::colorFor(m, 0, 0), rgbOf(LED_COLOR_SERVO_ADJUST)) ||
        LED_COLOR_SERVO_ADJUST == LED_COLOR_ARMED_WAIT);
  // Through the module: scaled, never above LED_MAX_BRIGHTNESS.
  Rig rig;
  m.state = DS_STATE_DISARMED;
  for (int k = 0; k < 4; k++) rig.step((uint64_t)kFrameMs * US_PER_MS, m);
  CHECK(same(hal_host::ledShown(0), scaled(rgbOf(LED_COLOR_SERVO_ADJUST))));
  CHECK(hal_host::ledMaxChannel() <= LED_MAX_BRIGHTNESS);
  CHECK_EQ(rig.leds.overLimit(), 0u);
}

TESTKIT_MAIN()

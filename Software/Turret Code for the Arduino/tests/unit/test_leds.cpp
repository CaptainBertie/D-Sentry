// test_leds.cpp -- Leds against spec 3.8, acceptance 11/12/16 and docs/architecture.md 3, 5, 6.
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
  for (uint8_t state = 0; state <= DS_STATE_COUNT; state++) {   // DS_STATE_COUNT: not a state
    for (uint8_t fire = 0; fire < DS_FIRE_MODE_COUNT; fire++) {
      for (uint8_t flags = 0; flags < 8; flags++) {
        for (uint8_t fault : faults) {
          for (PairingPhase phase : phases) {
            DisplayModel m = model(state, hal::nowMs());
            m.fireMode = fire;
            m.firing = flags & 1;
            m.emptyMagBlink = flags & 2;
            m.linkConnected = flags & 4;
            m.fault = fault;
            m.pairing = phase;
            combos++;
            for (uint32_t t = 0; t <= LED_PULSE_PERIOD_MS; t += stepMs) {
              rig.step((uint64_t)stepMs * US_PER_MS, m);
              const Rgb c = hal_host::ledShown(0);
              worstShown = std::max(worstShown, std::max(c.r, std::max(c.g, c.b)));
            }
          }
        }
      }
    }
  }
  CHECK_EQ(combos, (DS_STATE_COUNT + 1u) * DS_FIRE_MODE_COUNT * 8u * 3u * 3u);
  CHECK(hal_host::ledMaxChannel() <= LED_MAX_BRIGHTNESS);   // raw bytes given to the HAL
  CHECK_EQ(hal_host::ledOverLimit(), 0u);                   // the HAL's guard never had to cut
  CHECK_EQ(rig.leds.overLimit(), 0u);                       // nor Leds' own runtime check
  CHECK(worstShown <= LED_MAX_BRIGHTNESS);
  // Not vacuous: full-scale white / red / green did reach the LEDs, at exactly the cap.
  CHECK_EQ(hal_host::ledMaxChannel(), LED_MAX_BRIGHTNESS);
  CHECK_EQ(worstShown, LED_MAX_BRIGHTNESS);
}

// ---- priorities (spec 3.8), swept over every state, flag and many timestamps -------------------------------
TEST(priority_rules_hold_for_every_state_flag_and_time) {
  const Rgb fault = rgbOf(LED_COLOR_FAULT);
  const Rgb firing = rgbOf(LED_COLOR_FIRING);
  const Rgb empty = rgbOf(LED_COLOR_EMPTY_MAG);
  const Rgb startup = rgbOf(LED_COLOR_STARTUP);
  // stateSinceMs values on both sides of the millis() wrap
  const uint32_t sinces[] = {0u, 12345u, UINT32_MAX - LED_PULSE_PERIOD_MS / 2u};
  uint32_t samples = 0;
  for (uint8_t state = 0; state <= DS_STATE_COUNT; state++) {
    for (uint8_t fire = 0; fire < DS_FIRE_MODE_COUNT; fire++) {
      for (uint8_t flags = 0; flags < 4; flags++) {
        for (uint32_t since : sinces) {
          DisplayModel m = model(state, since);
          m.fireMode = fire;
          m.firing = flags & 1;
          m.emptyMagBlink = flags & 2;
          m.fault = DS_FAULT_STALL;
          for (uint32_t t = 0; t < 2 * LED_PULSE_PERIOD_MS; t += 3) {
            const uint32_t now = since + t;
            const Rgb c = Leds::colorFor(m, now, since);
            samples++;
            if (state == DS_STATE_OFF) {
              if (!CHECK(dark(c))) return;                      // OFF is dark, whatever the flags
            } else if (state == DS_STATE_FAULT) {
              if (!CHECK(same(c, fault) || dark(c))) return;    // FAULT beats FIRING and EMPTY
            } else if (m.firing) {
              if (!CHECK(same(c, firing))) return;              // FIRING beats EMPTY_MAG
            } else if (m.emptyMagBlink) {
              if (!CHECK(same(c, empty) || dark(c))) return;    // EMPTY beats the state colour
            } else if (state == DS_STATE_BOOT || state == DS_STATE_CALIBRATING) {
              // pulse: never above the colour, never below its floor (never fully dark)
              const float floorR = (float)startup.r * LED_PULSE_MIN_FRACTION;
              if (!CHECK(c.r <= startup.r && c.r + 1.0f >= floorR)) return;
              if (!CHECK(c.r == c.g && c.g == c.b)) return;     // white stays white
            } else if (state == DS_STATE_DISARMED) {
              if (!CHECK(same(c, rgbOf(LED_COLOR_DISARMED)))) return;
            } else if (state == DS_STATE_ARMED) {
              const uint32_t want = fire == DS_FIRE_MODE_FULL ? LED_COLOR_ARMED_FULL : LED_COLOR_ARMED_SEMI;
              if (!CHECK(same(c, rgbOf(want)))) return;
            } else if (state == DS_STATE_COUNT) {
              if (!CHECK(dark(c))) return;                      // not a state: dark
            }
          }
        }
      }
    }
  }
  CHECK(samples > 100000u);
}

TEST(fault_blink_is_hard_on_off_starting_on) {
  DisplayModel m = model(DS_STATE_FAULT, 1000);
  m.firing = true;   // FAULT still wins: the off phase is dark, not red-for-firing
  const Rgb fault = rgbOf(LED_COLOR_FAULT);
  CHECK(same(Leds::colorFor(m, 1000, 0), fault));
  CHECK(same(Leds::colorFor(m, 1000 + LED_FAULT_BLINK_MS - 1, 0), fault));
  CHECK(dark(Leds::colorFor(m, 1000 + LED_FAULT_BLINK_MS, 0)));
  CHECK(dark(Leds::colorFor(m, 1000 + 2 * LED_FAULT_BLINK_MS - 1, 0)));
  CHECK(same(Leds::colorFor(m, 1000 + 2 * LED_FAULT_BLINK_MS, 0), fault));
}

TEST(firing_red_beats_empty_mag_and_state_colour) {
  for (uint8_t state : {DS_STATE_BOOT, DS_STATE_CALIBRATING, DS_STATE_PAIRING, DS_STATE_DISARMED,
                        DS_STATE_ARMED}) {
    DisplayModel m = model(state, 0);
    m.firing = true;
    m.emptyMagBlink = true;
    for (uint32_t t = 0; t < 4 * LED_FAULT_BLINK_MS; t += LED_FAULT_BLINK_MS / 2)
      CHECK(same(Leds::colorFor(m, t, 0), rgbOf(LED_COLOR_FIRING)));
  }
}

TEST(state_colours_match_config) {
  CHECK(same(Leds::colorFor(model(DS_STATE_DISARMED), 0, 0), rgbOf(LED_COLOR_DISARMED)));
  DisplayModel armed = model(DS_STATE_ARMED);
  armed.fireMode = DS_FIRE_MODE_SEMI;
  CHECK(same(Leds::colorFor(armed, 0, 0), rgbOf(LED_COLOR_ARMED_SEMI)));
  armed.fireMode = DS_FIRE_MODE_FULL;
  CHECK(same(Leds::colorFor(armed, 0, 0), rgbOf(LED_COLOR_ARMED_FULL)));
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
  const Rgb p = rig.leds.pixel0();
  CHECK(same(p, scaled(rgbOf(LED_COLOR_ARMED_FULL))));
  CHECK(same(p, hal_host::ledShown(0)));
  if (LED_COLOR_ARMED_FULL == 0xFF6000 && LED_MAX_BRIGHTNESS == 127) {
    // the protocol_spec.py TelemetryPacket example (ledR 127, ledG 47, ledB 0)
    CHECK_EQ(p.r, 127);
    CHECK_EQ(p.g, 47);
    CHECK_EQ(p.b, 0);
  }
}

TEST(a_stroke_shows_red_for_its_whole_flash_hold) {
  // acceptance 11: TurretState holds `firing` for LED_FIRE_FLASH_MIN_MS; the LEDs must show red
  // within one frame of it and for that long (to within one frame), on every LED.
  Rig rig;
  DisplayModel m = model(DS_STATE_ARMED, 0);
  for (uint32_t i = 0; i < 33; i++) rig.step(US_PER_MS, m);
  const Rgb red = scaled(rgbOf(LED_COLOR_FIRING));
  const uint32_t start = hal::nowMs();
  uint32_t firstRed = 0, lastRed = 0;
  for (uint32_t i = 0; i < 3 * LED_FIRE_FLASH_MIN_MS; i++) {
    m.firing = hal::nowMs() - start < LED_FIRE_FLASH_MIN_MS;
    rig.step(US_PER_MS, m);
    bool allRed = true;
    for (uint8_t k = 0; k < LED_COUNT; k++) allRed = allRed && same(hal_host::ledShown(k), red);
    if (allRed) {
      if (!firstRed) firstRed = hal::nowMs();
      lastRed = hal::nowMs();
    }
  }
  CHECK(firstRed > 0u);
  CHECK(firstRed - start <= kFrameMs + 1);
  CHECK(lastRed - firstRed + 1 >= LED_FIRE_FLASH_MIN_MS - kFrameMs);
  CHECK(lastRed - start <= LED_FIRE_FLASH_MIN_MS + kFrameMs + 1);
  CHECK(same(hal_host::ledShown(0), scaled(rgbOf(LED_COLOR_ARMED_SEMI))));   // back to yellow
}

TEST(empty_mag_blink_starts_on_when_the_flag_rises) {
  Rig rig;
  DisplayModel m = model(DS_STATE_ARMED, 0);
  m.ammo = 0;
  for (uint32_t i = 0; i < 77; i++) rig.step(US_PER_MS, m);   // an arbitrary blink phase
  m.emptyMagBlink = true;
  const uint32_t rose = hal::nowMs() + 1;
  const Rgb on = scaled(rgbOf(LED_COLOR_EMPTY_MAG));
  // within one frame of the rise the LEDs are magenta, not waiting for a global blink phase
  for (uint32_t i = 0; i <= kFrameMs; i++) rig.step(US_PER_MS, m);
  CHECK(same(hal_host::ledShown(0), on));
  while (hal::nowMs() < rose + LED_FAULT_BLINK_MS + kFrameMs) rig.step(US_PER_MS, m);
  CHECK(dark(hal_host::ledShown(0)));
  while (hal::nowMs() < rose + 2 * LED_FAULT_BLINK_MS + kFrameMs) rig.step(US_PER_MS, m);
  CHECK(same(hal_host::ledShown(0), on));
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
  CHECK(same(hal_host::ledShown(0), scaled(rgbOf(LED_COLOR_ARMED_SEMI))));
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

// ---- SERVO TEACH colour (Leds.h) ---------------------------------------------------------------------
// LED_COLOR_SERVO_TEACH in DISARMED's teach mode, below FAULT and FIRING, through the clamp.
TEST(servo_teach_colour_sits_below_fault_and_firing_and_obeys_the_clamp) {
  DisplayModel m = model(DS_STATE_DISARMED);
  m.servoTeach = true;
  CHECK(same(Leds::colorFor(m, 0, 0), rgbOf(LED_COLOR_SERVO_TEACH)));
  m.firing = true;                     // a test stroke: red as a normal stroke
  CHECK(same(Leds::colorFor(m, 0, 0), rgbOf(LED_COLOR_FIRING)));
  m.firing = false;
  m.state = DS_STATE_FAULT;            // never together, but FAULT would win
  CHECK(same(Leds::colorFor(m, 0, 0), rgbOf(LED_COLOR_FAULT)));
  m.state = DS_STATE_ARMED;            // the flag means nothing outside DISARMED
  CHECK(!same(Leds::colorFor(m, 0, 0), rgbOf(LED_COLOR_SERVO_TEACH)) ||
        LED_COLOR_SERVO_TEACH == LED_COLOR_ARMED_SEMI);
  // Through the module: scaled, never above LED_MAX_BRIGHTNESS.
  Rig rig;
  m.state = DS_STATE_DISARMED;
  for (int k = 0; k < 4; k++) rig.step((uint64_t)kFrameMs * US_PER_MS, m);
  CHECK(same(hal_host::ledShown(0), scaled(rgbOf(LED_COLOR_SERVO_TEACH))));
  CHECK(hal_host::ledMaxChannel() <= LED_MAX_BRIGHTNESS);
  CHECK_EQ(rig.leds.overLimit(), 0u);
}

TESTKIT_MAIN()

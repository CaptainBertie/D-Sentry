// App.h -- owns every RA4M1 module and wires them together (docs/architecture.md rule 5: no
// cross-module globals; modules are members here, connected by reference in begin()).
// dsentry.ino creates one App; sim/sim_main.cpp and the App tests create theirs against the
// host HAL (the tests pass an App::Config to flip behaviour switches without editing config.h).
//
// Construction: members in the order below; every constructor only stores its Config (no
// hardware is touched before setup()). begin(): hal::begin() first (it claims the step timers
// GPT0/GPT1 before anything else can take a timer and leaves every output safe), then Debug
// (so every later begin() can log), ControlLink, Inputs, StallDetector, Motion, Flywheel,
// Trigger, Leds, Lcd, TurretState (the decider, last: it reads and commands all of the above),
// Telemetry, and finally Debug's boot warnings.
// loop(): hal::service() between modules (the USB queue drains at most one byte per call, so
// it must run often), ControlLink, Inputs, StallDetector, one shell line (Debug -> TurretState,
// so a shell action lands before this loop's decisions), TurretState, Trigger, Flywheel,
// Motion, Leds, Lcd, Telemetry, Debug. Nothing in it blocks.
//
// SPDX-License-Identifier: MIT
#pragma once
#include <stdint.h>
#include "ControlLink.h"
#include "Debug.h"
#include "Flywheel.h"
#include "Inputs.h"
#include "Lcd.h"
#include "Leds.h"
#include "Motion.h"
#include "StallDetector.h"
#include "Telemetry.h"
#include "TurretState.h"
#include "Trigger.h"
#include "Types.h"

class App {
 public:
  // The behaviour switches of the modules that have them (config.h by default).
  struct Config {
    Inputs::Config inputs;
    StallDetector::Config stall;
    Flywheel::Config flywheel;
    Trigger::Config trigger;
    TurretState::Config turret;
  };
  static Config defaults();

  App();                           // defaults()
  explicit App(const Config& cfg);
  App(const App&) = delete;        // ControlLink is not copyable
  App& operator=(const App&) = delete;

  // setup(): may block (LCD init ~1 s); nothing after it does.
  void begin();
  // loop(): one pass over every module, never blocking.
  void loop();

  // For the host tests and dsentry_sim.
  const TurretState& turret() const { return turret_; }
  const ControlLink& link() const { return link_; }
  const Inputs& inputs() const { return inputs_; }
  const Motion& motion() const { return motion_; }
  const StallDetector& stall() const { return stall_; }
  const Flywheel& flywheel() const { return flywheel_; }
  const Trigger& trigger() const { return trigger_; }
  const Leds& leds() const { return leds_; }
  const Lcd& lcd() const { return lcd_; }
  const Telemetry& telemetry() const { return telemetry_; }
  const Debug& debug() const { return debug_; }
  bool halOk() const { return halOk_; }
  uint32_t loops() const { return loops_; }

 private:
  Debug debug_;
  ControlLink link_;
  Inputs inputs_;
  Motion motion_;
  StallDetector stall_;
  Flywheel flywheel_;
  Trigger trigger_;
  Leds leds_;
  Lcd lcd_;
  TurretState turret_;
  Telemetry telemetry_;
  bool halOk_ = false;
  uint32_t loops_ = 0;
};

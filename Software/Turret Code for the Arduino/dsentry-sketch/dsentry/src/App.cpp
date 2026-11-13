// App.cpp -- see App.h. The order of begin() and loop() is the contract the modules rely on.
//
// SPDX-License-Identifier: MIT
#include "App.h"

#include "hal.h"

App::Config App::defaults() {
  Config c;
  c.inputs = Inputs::defaults();
  c.stall = StallDetector::defaults();
  c.flywheel = Flywheel::defaults();
  c.trigger = Trigger::defaults();
  c.turret = TurretState::defaults();
  return c;
}

App::App() : App(defaults()) {}

App::App(const Config& cfg)
    : inputs_(cfg.inputs), stall_(cfg.stall), flywheel_(cfg.flywheel), trigger_(cfg.trigger),
      turret_(cfg.turret) {}

void App::begin() {
  // 1. Hardware first: hal::begin() claims GPT0/GPT1 for the steppers before any library can
  //    allocate a timer, and leaves every output safe (drivers off, flywheel cut, servo parked).
  halOk_ = hal::begin();
  const uint32_t nowMs = hal::nowMs();
  const uint32_t nowUs = hal::nowUs();

  // 2. Debug next, so every later begin() can log. It only needs pointers that stay valid.
  DebugSources dbg;
  dbg.turret = &turret_;
  dbg.inputs = &inputs_;
  dbg.link = &link_;
  dbg.motion = &motion_;
  dbg.stall = &stall_;
  dbg.flywheel = &flywheel_;
  dbg.trigger = &trigger_;
  dbg.leds = &leds_;
  dbg.lcd = &lcd_;
  debug_.begin(dbg);
  debug_.bootBanner(halOk_);

  // 3. The modules, inputs before the ones that act on them.
  link_.begin(debug_, nowMs, nowUs);
  inputs_.begin(nowMs);
  stall_.begin(nowUs);
  motion_.begin(stall_);
  flywheel_.begin();
  trigger_.begin();
  leds_.begin(debug_);
  lcd_.begin(debug_);

  // 4. The decider last: it reads and commands all of the above.
  turret_.begin(inputs_, link_, motion_, stall_, flywheel_, trigger_, debug_, nowMs);

  TelemetrySources tel;
  tel.turret = &turret_;
  tel.inputs = &inputs_;
  tel.link = &link_;
  tel.motion = &motion_;
  tel.stall = &stall_;
  tel.flywheel = &flywheel_;
  tel.trigger = &trigger_;
  tel.leds = &leds_;
  tel.lcd = &lcd_;
  tel.debug = &debug_;
  telemetry_.begin(tel, link_);

  // 5. What the owner should know before trusting this boot.
  debug_.bootWarnings();
}

void App::loop() {
  const uint32_t startUs = hal::nowUs();
  const uint32_t nowMs = hal::nowMs();
  hal::service(startUs);

  // Inputs first: what arrived, what the toggles and stick say.
  link_.update(nowMs, startUs);
  hal::service(hal::nowUs());
  inputs_.update(nowMs, link_.frame());
  stall_.update(hal::nowUs(), nowMs, motion_.movingAxes(), motion_.moveStartMs());
  hal::service(hal::nowUs());

  // One bench-shell line per loop (ControlLink queues up to SHELL_QUEUE_LINES): its action goes
  // through TurretState's permissions before this loop's decisions are made.
  char line[SHELL_LINE_MAX + 1];
  if (link_.takeShellLine(line, sizeof line)) {
    debug_.handleShellLine(line);
    hal::service(hal::nowUs());
  }

  // Decide, then act.
  turret_.update(nowMs);
  trigger_.update(nowMs, turret_.triggerInput());
  flywheel_.update(nowMs);
  motion_.update(nowMs);
  hal::service(hal::nowUs());

  // Show and report.
  leds_.update(nowMs, hal::nowUs(), turret_.display());
  hal::service(hal::nowUs());
  lcd_.update(nowMs, turret_.display());
  hal::service(hal::nowUs());
  telemetry_.update(nowMs);
  hal::service(hal::nowUs());
  debug_.update(nowMs, hal::nowUs());
  hal::service(hal::nowUs());

  loops_++;
  debug_.countLoop(hal::nowUs() - startUs);
}

// test_app_skeleton.cpp -- the whole App on the host HAL: boots, loops in virtual time, and keeps
// the invariants that must hold at every stage of the build (not the stubs' behaviour, which
// the module stages replace): the boot report is logged, no LED channel ever exceeds
// LED_MAX_BRIGHTNESS, the HAL never refuses a PWM pin, the loop-rate statistics run, and
// nothing in loop() blocks virtual time.
//
// SPDX-License-Identifier: MIT
#include <string>

#include "App.h"
#include "hal.h"
#include "hal_host.h"
#include "testkit.h"

TEST(app_boots_and_loops) {
  hal_host::reset();
  static App app;   // static: the App holds every module's buffers (one App per process)
  app.begin();
  CHECK(app.halOk());
  std::string usb;
  const uint64_t loopUs = 200;
  const uint64_t loopsPerSecond = US_PER_S / loopUs;
  for (uint64_t i = 0; i < 2 * loopsPerSecond; i++) {
    hal_host::advanceUs(loopUs);
    app.loop();
    usb += hal_host::usbTakeTx();
  }
  CHECK_EQ(app.loops(), 2 * loopsPerSecond);
  CHECK(usb.find("D-Sentry RA4M1 boot") != std::string::npos);
  CHECK(hal_host::ledMaxChannel() <= LED_MAX_BRIGHTNESS);
  CHECK_EQ(hal_host::ledOverLimit(), 0u);
  CHECK_EQ(hal_host::pwmRefused(), 0u);
  // Virtual time only moved by the test: loop() itself never waited.
  CHECK_EQ(hal_host::timeUs(), 2 * US_PER_S);
  // A stats window has closed: loopHz is the 5000 loops/s the test ran.
  CHECK_EQ(app.debug().stats().loopHz, loopsPerSecond);
}

TESTKIT_MAIN()

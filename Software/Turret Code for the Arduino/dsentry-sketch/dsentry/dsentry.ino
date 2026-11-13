// dsentry.ino -- D-Sentry turret firmware for the UNO R4 WiFi's RA4M1.
// Everything lives in src/ (docs/architecture.md section 5); the tunables in config.h (a
// symlink to D-SentryCode/config.h). Build: D-SentryCode/tools/build.sh, or
//   arduino-cli compile --fqbn arduino:renesas_uno:unor4wifi D-SentryCode/firmware-ra4m1/dsentry
// or `pio run` in D-SentryCode/firmware-ra4m1.
//
// SPDX-License-Identifier: MIT
#include "src/App.h"

static App app;   // constructors only store defaults; nothing touches hardware before setup()

void setup() { app.begin(); }

void loop() { app.loop(); }

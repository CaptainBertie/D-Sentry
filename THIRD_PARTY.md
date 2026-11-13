# Licences and third-party parts

## Our licences

| Part | Licence | Why |
|---|---|---|
| Mechanical design, wiring, the RA4M1 sketch, the `XboxControllerR4` library, the PC relay and dashboard, D-Sentry's protocol code | **MIT** | Our own work. |
| The compiled ESP32-S3 firmware (`Software/Custom Firmware/bridge-fork/`) | **AGPL-3.0** | It links SudoMaker's `chAT`, which is AGPL-3.0-or-later, so the whole binary is AGPL-3.0 even though most of its code is MIT, Apache or BSD. |

You may build and flash the firmware for your own turret freely. If you give the compiled
image to anyone else, the AGPL makes you offer them the full source too; this repo already
holds it beside the images in `Software/Custom Firmware/artifacts/`. The RA4M1 code never
links `chAT`, so it stays MIT. `DECISIONS.md` D04 sets out the reasoning.

## Third-party parts

One row per part. "Use" says what we take from it. We read each licence from the LICENSE file
or source headers at the pinned revision.

### ESP32-S3 firmware, in `Software/Custom Firmware/bridge-fork/`

| Part | URL | Pinned revision | Licence | Use |
|---|---|---|---|---|
| uno-r4-wifi-usb-bridge | https://github.com/arduino/uno-r4-wifi-usb-bridge | 94d5bb2e8c2cb5492345bfb84d787fde65d1c183 (tag 0.6.0 era, main @ 2026-10-11) | No repo-level LICENSE; per file, see the rows below | The whole stock firmware: USB CDC bridge, CMSIS-DAP, AT server, WiFi, HCI bridge, OTA, build scripts |
| chAT (in the bridge sketch: chAT.hpp, parser.cpp, server.cpp) | https://github.com/SudoMaker/chAT | as vendored in the bridge repo | AGPL-3.0-or-later | The AT command server the RA4M1 talks to; our new commands register in it |
| OTA.cpp/.h, Arduino SA (in the bridge sketch) | same repo | same | GPL-3.0 | unchanged |
| SSE.cpp/.h, Arduino SA (in the bridge sketch) | same repo | same | MPL-2.0 | unchanged |
| freedap.c/.h, dap_config.h (Alex Taradov free-dap) | https://github.com/ataradov/free-dap | as vendored | BSD-3-Clause | CMSIS-DAP debug probe, unchanged |
| BossaUnoR4WiFi.cpp/.h, Arduino SA | same repo | same | LGPL-2.1-or-later | unchanged |
| arduino-esp32 (Arduino fork, patched) | https://github.com/arduino/arduino-esp32 branch unor4wifi-2.0.9 | a2b82168bed7e43636271e757bde9d28e8014977 | LGPL-2.1 | ESP32-S3 Arduino core, precompiled ESP-IDF 4.4 libraries, toolchain downloader |
| ArduinoBLE | https://github.com/arduino-libraries/ArduinoBLE | 56dbfb33aef47489cfbfb26a4e32c8e6d123c9ff (1.3.6) | LGPL-2.1 | HCIVirtualTransport: passes raw HCI between the RA4M1 and the ESP32 BT controller (stock) |
| ArduinoHttpClient | https://github.com/arduino-libraries/ArduinoHttpClient | 6f2659de4c0523f0a5fa4bc1154d1e5dc1b01546 (0.6.0) | No LICENSE file in the pinned tree; the library metadata says Apache-2.0 (not checked here) | OTA download (stock) |
| Arduino_DebugUtils | https://github.com/arduino-libraries/Arduino_DebugUtils | af1530c7da403998b96ee90b83532ff3a50dafd3 (1.4.0) | GPL (Arduino SA dual-licence note) | debug printing (stock) |
| Arduino_ESP32_OTA | https://github.com/arduino-libraries/Arduino_ESP32_OTA | 7b509784ce4dd988b5213b14babec2f8338f6619 (0.3.1) | GPL (Arduino SA dual-licence note) | OTA (stock) |
| BOSSA (Arduino fork) | https://github.com/arduino/BOSSA branch unor4wifi | ce7b6844a21e6bd871388a0736fd186208f28ce6 | BSD-3-Clause (ShumaTech) | flashing the RA4M1 through the bridge (stock) |
| hid (pennam fork of karalabe/hid) | https://github.com/pennam/hid | 931f677e7f1728dfd652e146654caebb901ee85c | hidapi BSD-3 / libusb LGPL-2.1 | the unor4wifi-reboot host tool (stock) |

### Added for the Xbox controller

| Part | URL | Pinned revision | Licence | Use |
|---|---|---|---|---|
| XboxSeriesXControllerESP32_asukiaaa | https://github.com/asukiaaa/arduino-XboxSeriesXControllerESP32 | 1.1.3 @ 00234325be391543e7df914fc85a82bcb274c045, plain copy with 3 marked patches (see its UPSTREAM.md) | MIT | BLE client: scan, connect, bond, subscribe to the controller's HID report |
| XboxControllerNotificationParser | https://github.com/asukiaaa/arduino-XboxControllerNotificationParser | 1.0.5 @ 77a449d77970166a01c25ef7c26c084a42c3a5d6 (submodule; also copied verbatim into the RA4M1 library's src/vendor) | MIT | Decodes the 16-byte HID report into sticks, triggers, d-pad and buttons, on both chips |
| XboxSeriesXHIDReportBuilder_asukiaaa | https://github.com/asukiaaa/arduino-XboxSeriesXHIDReportBuilder | 1.0.1 @ 4afd097b198097d2b66f892529488b7c9f8a2898 (submodule) | MIT | Needed by the client library; only rumble uses it (not built) |
| NimBLE-Arduino | https://github.com/h2zero/NimBLE-Arduino | 2.5.1 @ 5ae2c43f62c5a588a25c1de68f7d17b6f3b75adb (submodule; the tag is on branch release/2.5) | Apache-2.0; bundles Apache Mynewt NimBLE (Apache-2.0) with BSD-3 parts (queue.h, os_mbuf.c, tinycrypt) and Nordic/CMSIS files, see src/nimble/LICENSE and NOTICE | BLE host stack on the ESP32-S3, central role with bonding |

### Read for design only (nothing copied)

| Part | URL | Revision read | Licence |
|---|---|---|---|
| ControllerAsI2c_asukiaaa | https://github.com/asukiaaa/arduino-ControllerAsI2c | 1fb4b8d (1.0.7) | MIT |
| ArduinoCore-renesas (WiFiS3 Modem) | https://github.com/arduino/ArduinoCore-renesas | 424e86e (main, 2026-05-28); installed core arduino:renesas_uno 1.6.0 | LGPL-2.1 |

### Host tools (not built into any binary)

Paths are relative to `Software/Custom Firmware/`.

| Tool | Version | Licence | Use |
|---|---|---|---|
| arduino-cli | 1.5.1 (Homebrew) | GPL-3.0 | builds the ESP32 firmware (`bridge-fork/compile.sh`) and the RA4M1 examples |
| esptool | 5.5.0 (pip) | GPL-2.0-or-later | `tools/r4flash.py`: flash-id, read-flash, write-flash, verify-flash, image-info |
| espflash | 4.6.0 (Homebrew) | MIT OR Apache-2.0 | a documented alternative only (in 4.x, `-B` sets the baud rate) |
| cryptography (Python) | 50.0.2 (pip) | Apache-2.0 OR BSD-3 | the `bridge-fork/export.sh` certificate bundle (`bridge-fork/certificates/gen_crt_bundle.py`) |
| hidapi (cython-hidapi, Python) | 0.15.0 (pip) | your choice of GPL-3.0, BSD-style, or the original cython-hidapi licence (per its LICENSE.txt) | `tools/r4flash.py enter-download`: sends the bridge's 0xAA HID feature report |
| pyserial | 3.5 (pip) | BSD-3-Clause | `tools/r4flash.py ports`: lists ports by VID:PID without opening them |

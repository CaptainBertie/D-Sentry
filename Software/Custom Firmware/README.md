<!-- SPDX-License-Identifier: MIT AND AGPL-3.0-or-later -->
# Custom Firmware (ESP32-S3)

The Arduino UNO R4 WiFi carries two chips. This folder is everything that runs on the
second one, the **ESP32-S3**: the radio and USB bridge. (The main turret logic, which runs
on the RA4M1, is in `../Turret Code for the Arduino/`.)

It is built in two layers:

1. **`bridge-fork/`** — a fork of Arduino's own [`uno-r4-wifi-usb-bridge`](https://github.com/arduino/uno-r4-wifi-usb-bridge)
   firmware, extended to also act as a BLE host for an Xbox controller. This is the Xbox-R4
   project: it keeps the stock USB-CDC bridge (so sketch uploads and the Serial Monitor still
   work) and adds an Xbox client on top. It is vendored here as a near-verbatim copy with
   every change marked — see `bridge-fork/UPSTREAM.md` and `../../DECISIONS.md` D01–D18.
2. **D-Sentry's own additions**, inside `bridge-fork/UNOR4USBBridge/`, which add the two
   turret radio modes (BT / WiFi) on top of the Xbox client — see "What D-Sentry adds" below
   and `../../DECISIONS.md` D19.

## License — read this before you flash or share a build

The compiled firmware image is **AGPL-3.0**, because it links SudoMaker's `chAT` AT-command
server. Building and flashing it for your own turret has no obligations. Giving the compiled
image to anyone else requires offering them this complete source, which is exactly what's
sitting in `artifacts/` next to its prebuilt `.bin` files. The D-Sentry-specific files are
individually MIT. Full breakdown: `../../THIRD_PARTY.md`.

## Folder map

| Path | What |
|---|---|
| `bridge-fork/` | The fork's source: `UNOR4USBBridge/` (the sketch), `libraries/` (vendored + git submodules, pinned), `hardware/esp32-patched/esp32` (the patched Arduino-ESP32 core, submodule), build scripts. |
| `protocol/` | The wire formats: `xbox_r4_protocol.h` (controller state, base Xbox-R4 project) and `dsentry_protocol.h` / `protocol_spec.py` (D-Sentry's ControlPacket/StatusPacket, generated from the Python spec). |
| `radio-ra4m1-library/` | `XboxControllerR4`, the Arduino library the RA4M1 side uses to talk to this firmware over the AT link. |
| `docs/` | `recon.md` (everything learned about the stock bridge before forking it), `flashing.md` (back up, flash, restore), `protocol.md` (generated AT-command + packet reference). |
| `sim/` | Emulation: the fork in Espressif QEMU, the RA4M1 sketch in Renode. What each proves is in `sim/README.md`. |
| `tests/` | Host-side tests for the fork and the base protocol (`./tests/run.sh`). D-Sentry's own ESP32-side tests (`dsentry_wire`, `dsentry_at`) live with the rest of the turret's tests, in `../Turret Code for the Arduino/tests/esp32/`. |
| `tools/` | `r4flash.py` (flash/backup/restore with safety guards, see `tools/README.md`) and a bench dashboard (`tools/dashboard.py`, `tools/DASHBOARD.md`). |
| `artifacts/` | Prebuilt `.bin` images, both the plain Xbox-R4 fork and the D-Sentry build, with SHA256 sums. |

## Build the firmware from a clean checkout

```sh
git clone --recurse-submodules <this repo> d-sentry && cd d-sentry
python3 -m venv .venv && .venv/bin/pip install esptool pyserial cryptography hidapi
brew install arduino-cli            # 1.5.x; any recent version works
cd "Software/Custom Firmware/bridge-fork/hardware/esp32-patched/esp32/tools" && python3 get.py && cd -   # ESP32 toolchain, ~1 GB, once
cd "Software/Custom Firmware/bridge-fork" && ./build.sh    # compile + size gate + export; about 1 minute
```

A checkout path with spaces works (upstream's `compile.sh` was fixed for it — `../../../DECISIONS.md` D02).

Outputs land in `bridge-fork/UNOR4USBBridge/build/esp32-patched.esp32.arduino_unor4wifi_usb_bridge/`:
`S3-BOOT-APP.bin` (flash at 0x0, keeps NVS/SPIFFS), `S3-ALL.bin` (full image, wipes NVS/SPIFFS —
factory reset only), `UNOR4USBBridge.ino.bin` (the app alone).

## Flash it

`docs/flashing.md`, in short: `tools/r4flash.py check`, put the board in ESP download mode,
`backup`, `flash <image> --backup <file>`, unplug, replug. `flash` refuses to run without a
verified full backup.

## What D-Sentry adds on top of the fork

| | BT mode (`wantMode` 0) | WiFi mode (`wantMode` 1) |
|---|---|---|
| Pad | pairs with the turret over BLE (the fork's Xbox client, exactly as `AT+XBOXBEGIN`) | pairs with the PC; the PC relay sends ControlPackets |
| `AT+XBOXSTATE?` | the pad's state (unchanged) | built from the relay's latest valid ControlPacket |
| Rumble (`rumbleSeq` in `AT+DSS`) | 8-byte output report to the pad, written by the client task | a StatusPacket by UDP to the relay, which rumbles the pad |
| `AT+DST=<hex>` | answers `0` (not forwarded) | the TelemetryPacket bytes, unchanged, by UDP to the relay |
| Network | none | station on `WIFI_SSID`, else own AP `WIFI_AP_SSID`; UDP `WIFI_UDP_PORT`; beacon broadcast every `WIFI_BEACON_MS` |

BLE and WiFi never run in the same boot — switching stores the wanted mode and restarts
(`../../DECISIONS.md` D19). A board that never receives a D-Sentry command runs exactly the
fork's own code paths.

New AT commands: `AT+DSCAPS?`, `AT+DSS=<wantMode>,<rumbleSeq>,<strong>,<weak>,<rumbleMs>,<state>`,
`AT+DST=<hex>`. Full command reference: `docs/protocol.md`.

New files (all in `bridge-fork/UNOR4USBBridge/`, each carrying an MIT header and the AGPL
combined-work note): `dsentry_wire.h/.cpp` (pure logic, host-tested), `dsentry_at.h/.cpp` (the
chAT handler bodies), `dsentry_radio.h/.cpp` (ESP32-only: NimBLE, the WiFi task, RTC memory,
restart), `cmds_dsentry.h` (registers the three commands), `dsentry_protocol.h` /
`dsentry_config.h` (generated, never hand-edited). Edited: `at_handler.h/.cpp`,
`cmds_esp_generic.h`, `xbox_client.h/.cpp`, `UNOR4USBBridge.ino` — every change is marked
`D-Sentry:` in the source.

## Build and test (no board)

```sh
cd "../Turret Code for the Arduino/tests/esp32" && ./run.sh            # host tests: dsentry_wire, AT handlers on real chAT, literals
cd "../Turret Code for the Arduino/tests/esp32" && ./build_firmware.sh # the firmware in a scratch copy, size gate, -Wall -Wextra
```

For the fork's own (non-D-Sentry) tests: `cd tests && ./run.sh` and `cd sim && cat README.md`.

<!-- SPDX-License-Identifier: MIT -->
# Turret Code for the Arduino (RA4M1)

This is D-Sentry's main microcontroller sketch — the RA4M1 half of the Arduino UNO R4 WiFi.
It owns every turret behavior: the state machine, pan/tilt motion, firing, the stall detector,
LEDs, the LCD, and the link to the radio chip. You upload it like any Arduino sketch.

The radio chip (ESP32-S3) runs separate firmware that this sketch talks to over a serial AT
link — see `../Custom Firmware/`. The full system picture is in `../System Block Diagram/`.

This is synced from the `turret-live` branch of the original development repo — the one
actually flashed to the turret — not `d-sentry`, which still carries an LED scheme and a
servo flash-save feature that were built and then rolled back before ever being flashed.
See `../../DECISIONS.md` D26 if a build or a decision here looks inconsistent with an older
description elsewhere in this repo; `turret-live` is the one that's real.

`dsentry-sketch/dsentry/config.h`, `dsentry_protocol.h` and `xbox_r4_protocol.h` are plain
copies of `config.h` (above) and the files in `../Custom Firmware/protocol/` — the original
repo wired them up as symlinks, which don't survive this folder's structure or a Windows
checkout reliably, so they're flattened here. If you change one, re-copy it into this
sketch folder too.

## Folder map

| Path | What |
|---|---|
| `config.h` | The **only** tunables file. Every pin, speed, angle, color, timeout and threshold comes from here — nothing else hard-codes one. |
| `dsentry-sketch/dsentry/` | The Arduino sketch (`dsentry.ino` + `src/`, one module per subsystem: `ControlLink`, `Inputs`, `Motion`, `StallDetector`, `Flywheel`, `Trigger`, `Leds`, `Lcd`, `TurretState`, `Debug`). |
| `dsentry-sketch/platformio.ini`, `libraries.txt` | Pinned build config and library versions. |
| `tools/` | `bench_console.py` (typed console on the RA4M1's USB port), `i2c_scan/` (finds the LCD's I2C address), `sync_config.py`/`gen_protocol.py`/`check_literals.py` (generators and the no-hard-coded-literals checker). |
| `docs/tuning.md` | Every tunable: what it does, its safe range, how to see its effect, the order to tune in. |
| `docs/bench-checklist.md` | First power-up, then every acceptance-criteria item from `../../D-SENTRY_SPEC.md` §11, run on the real turret. |
| `tests/` | Host unit tests (one per spec §11 item), the chAT/ESP32 host tests, `run.sh`. |
| `sim/` | The real RA4M1 logic, built for a Mac against a fake hardware layer — lets you test turret behavior with no board at all. |
| `early-prototypes/` | Bench sketches from before D-Sentry existed (stepper/LED/display tests) — kept for project history, superseded by the sketch above. |

## What the system is

D-Sentry is a pan/tilt foam-dart turret. An Xbox controller aims and fires it. Full behavior
spec (state machine, controller mapping, firing logic, calibration, stall detection, link
protocol): `../../D-SENTRY_SPEC.md`. Why things are built the way they are:
`../../DECISIONS.md`.

The A1 toggle picks how the controller reaches the turret:

| Mode | Where the pad pairs | How its state reaches the RA4M1 |
|---|---|---|
| **BT** (toggle open) | with the turret, over Bluetooth Low Energy | the radio chip, then the AT link inside the board |
| **WiFi** (toggle closed) | with a PC | the PC relay (`../Local Web Server for WiFi Connect Mode/`), then UDP over WiFi to the radio chip, then the AT link |

On the bench there's a third path: the PC relay can send the same packets down the USB cable,
which always wins over the radio (while `DEBUG_ACCEPT_USB_CONTROL` is true, the default).

## Build and test everything (no board)

Tools needed: `arduino-cli` 1.5.x with core `arduino:renesas_uno@1.6.0`, the libraries
`Adafruit NeoPixel@1.15.5` and `LiquidCrystal I2C@1.1.2` (see `dsentry-sketch/libraries.txt`),
PlatformIO if you use it, and Python with `pygame`/`pyserial`/`esptool`.

```sh
./tools/build.sh
```

Runs four steps, never uploads anything: checks generated files match their sources
(`config.h` changes need `python3 tools/sync_config.py` to re-copy into the radio-firmware
build at `../Custom Firmware/bridge-fork/UNOR4USBBridge/dsentry_config.h`), runs the host test
suite, compiles the sketch with `arduino-cli --warnings all`, and runs the same build under
PlatformIO. Last line must read `Summary: N passed, 0 failed`.

## Upload the sketch

```sh
arduino-cli board list                 # find the port — "Arduino UNO R4 WiFi"
arduino-cli compile --fqbn arduino:renesas_uno:unor4wifi --upload -p <port> \
  dsentry-sketch/dsentry
```

The upload "touches" the port at 1200 baud, which resets the RA4M1 into its bootloader —
that's why every D-Sentry tool opens the port at exactly 115200 (`DEBUG_SERIAL_BAUD`); opening
it at 1200/2400 baud would reset the turret mid-use.

The radio chip needs its own firmware image once — see `../Custom Firmware/README.md` and
`../Custom Firmware/docs/flashing.md`. Back up first; the flash tool refuses otherwise.

## Drive it and watch it

| Situation | Command | What happens |
|---|---|---|
| Bench, pad on a PC, USB cable | `python3 "../Local Web Server for WiFi Connect Mode/dsentry_relay.py"` | Sends the pad's state down USB at 100 Hz, shows telemetry, opens a dashboard at `http://127.0.0.1:8766/`. |
| BT mode (pad paired with the turret) | add `--monitor` | Read-only telemetry; sends nothing, never overrides the pad. |
| WiFi mode (pad paired with a PC) | add `--udp` | Finds the turret by its beacon, or dial it directly. |
| No hardware, real logic | add `--sim` | Talks to `sim/build/dsentry_sim` (build first: `bash sim/build.sh`) — the real RA4M1 code on a fake board. |
| Typed control on the bench | `python3 tools/bench_console.py` | Sends neutral packets at 100 Hz; accepts `connect`, `tap a`, `hold lb`, `stick 1 0`, `trigger 0.8`, `!status`, `help`. |

## The bench shell

Text lines typed on the USB port reach a small shell on the RA4M1 (prefix with `!` from the
bench console, e.g. `!status`). Every action obeys the same rules as the controller: `arm` is
refused while the trigger is pulled, `fire` needs ARMED and spun-up flywheels, and so on.

| Command | Does |
|---|---|
| `help`, `status`, `stats`, `stall?` | commands; state/link/position/flywheel; counters since boot; the stall sensor now |
| `log 0`..`log 3` | log level: 0 off, 1 errors, 2 state changes + stats/sec, 3 + stall trace |
| `arm`, `disarm`, `fire`, `fly on`, `fly off`, `mode semi`, `mode full`, `mag` | the controller's actions |
| `pan <deg>`, `tilt <deg>`, `home`, `sethome`, `stop` | position moves; out-of-limit targets refused |
| `cal` | re-runs tilt calibration |

## Where to tune

Every number lives in `config.h`; `docs/tuning.md` lists each one with its safe range and how
to see its effect. After editing `config.h`: re-upload the sketch, and if the name you changed
also appears in the radio firmware's config (WiFi credentials, `LINK_TIMEOUT_MS`, etc.), run
`python3 tools/sync_config.py` and reflash the radio chip.

## License

MIT.

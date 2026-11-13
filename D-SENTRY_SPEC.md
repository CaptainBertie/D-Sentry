# D-SENTRY — Software Implementation Spec

**Audience:** Claude Code, plan mode. Build a plan from this document, then implement.
**Owner:** Bigman. He tunes every number; you build every function.
**Companion file:** `config.h` — the single tunables file. It is already written. Every pin, speed, angle, colour, timeout and threshold in the system must come from it. No other file may hard-code one.

> This is the original spec the build started from. Where the finished turret differs —
> the 5V power rail, the tilt reduction's ratio, the firing sequence, the controller
> mapping — `DECISIONS.md` has the full history, and `Software/Turret Code for the
> Arduino/` is the code that's actually running.

---

## 1. What the system is

D-Sentry is a pan/tilt foam-dart turret. An Xbox controller aims it and fires it. The brain is an Arduino UNO R4 WiFi: the RA4M1 runs all turret logic; the on-board ESP32-S3 coprocessor runs custom firmware that talks to the controller (Bluetooth) or to a PC relay (WiFi) and streams a normalised control packet to the RA4M1 over a serial link.

### Hardware (fixed — do not redesign)

| Part | Detail |
|---|---|
| Main board | Arduino UNO R4 WiFi (Renesas RA4M1 + ESP32-S3-MINI-1-N8), powered by its own 9 V battery on the barrel jack |
| Motion | 2× NEMA17 steppers through a CNC Shield V3 with A4988/DRV8825 drivers. Stepper rail = 6S LiPo on the shield's terminal block. X axis = pan (5:1 planetary), Y axis = tilt (4:1 spur, 220 mm arm) |
| Flywheels | Two DC motors on one wire pair, one L298N channel (A: OUT1/OUT2), fed from the 6S LiPo. Must see ~9 V → PWM duty on ENA |
| Trigger | Hobby servo on D12 (dart pusher) |
| LEDs | Freenove 8× WS2812 (addressable RGB) module, data on D13, powered from the 5 V rail |
| LCD | 16×2 character LCD on an I2C backpack (A4/A5) |
| Switches | Two toggles on A0 (power) and A1 (BT/WiFi), both to GND with internal pull-ups |
| Controller | Xbox Wireless Controller over Bluetooth **Low Energy** (see §9 prerequisites) |

### Final pin map

| RA4M1 pin | Shield header | Wired to | Direction |
|---|---|---|---|
| D2 / D5 | X.STEP / X.DIR | Pan stepper driver | out |
| D3 / D6 | Y.STEP / Y.DIR | Tilt stepper driver | out |
| D8 | EN | All driver enables (LOW = on) | out |
| D9 | X± limit | L298N IN1 | out |
| D10 | Y± limit | L298N ENA (PWM) | out |
| D11 | Z± limit | L298N IN2 | out |
| D12 | SpnEn | Trigger servo signal | out |
| D13 | SpnDir | LED module data (S) | out |
| A0 | Abort | Power toggle | in, pull-up |
| A1 | Hold | BT/WiFi toggle | in, pull-up |
| A2 | Resume | Stall-sense analog (tap TBD, §8) | in |
| A3 | CoolEn | spare | — |
| A4 / A5 | SDA / SCL | LCD I2C | — |
| D0 / D1 | — | Serial1, free (fallback link to an external ESP32) | — |
| internal UART | — | ESP32-S3 link (`LINK_SERIAL`, verify name) | both |

---

## 2. Repository layout to produce

```
dsentry/
  config.h                      # given; shared tunables (RA4M1 includes it directly,
                                #   ESP32 build mirrors sections 10–11 via a copy script)
  firmware-ra4m1/               # Arduino sketch for the main MCU
    dsentry.ino
    src/ ...                    # one module per subsystem (see §4)
  firmware-esp32s3/             # custom radio firmware for the on-board ESP32-S3
  relay-pc/                     # Python: reads an Xbox controller on a PC, sends packets
  tools/
    sync_config.py              # copies sections 10–11 of config.h into the ESP32 build
    i2c_scan.ino                # finds the LCD address
    bench_console.py            # types packets at the RA4M1 over USB serial
  docs/
    protocol.md                 # the link packet format (§5), generated from the structs
    flashing.md                 # how to flash both MCUs and how to restore stock firmware
```

Build with `arduino-cli`. Pin every library version in a `libraries.txt` or sketch.yaml.

---

## 3. Behaviour spec (the contract)

### 3.1 Top-level state machine (RA4M1)

```
            power toggle OFF ──────────────► OFF
OFF ──power ON──► BOOT ──► CALIBRATING ──► PAIRING ──► DISARMED ◄──A──► ARMED
                              │ fail                      ▲             │
                              ▼                           │ link lost   │ link lost
                            FAULT ◄───── stall ───────────┴─────────────┘
```

- **OFF** — power toggle open. Steppers disabled (EN HIGH), flywheel PWM 0, servo detached, LEDs dark, LCD shows `SYSTEM OFF` (backlight off if configured), controller input ignored, link packets still consumed so the buffer never overflows. Closing the toggle restarts from BOOT.
- **BOOT** — LEDs pulse white↔off. Initialise peripherals, LCD shows `Calibrating`.
- **CALIBRATING** — tilt homing (§3.5). On failure → FAULT with `FAULT: CAL`.
- **PAIRING** — LEDs pulse blue↔white, LCD row 1 shows mode + `Pairing` with the cycling dots. Leaves when a valid control packet with `connected = 1` arrives.
- **DISARMED** (default after pairing) — LEDs solid green. Pan/tilt, home set/go, mode toggle (B), mag reset (X) all work. Trigger and flywheels do nothing (flywheels do nothing only if `FLYWHEEL_REQUIRES_ARMED`).
- **ARMED** — LEDs solid yellow (semi) or orange (full). Everything above plus flywheels and firing.
- **FAULT** — all motion stopped, flywheels off, servo to rest, LEDs hard-blink red, LCD shows the fault. Stall faults clear themselves after `STALL_FAULT_HOLD_MS` and return to the previous arm state. Calibration faults clear only by toggling power.

Link loss (no packet for `LINK_TIMEOUT_MS`): stop motion, apply the `FAILSAFE_*` options, return to PAIRING (LCD shows `Pairing` for `LINK_RECONNECT_GRACE_MS`, then `No Link`). Reconnection restores DISARMED, never ARMED.

### 3.2 Controller mapping

| Input | Action | Notes |
|---|---|---|
| Right stick X | Pan velocity | deadzone + expo + max speed from config |
| Right stick Y | Tilt velocity | `JOY_INVERT_Y` so stick-up = barrel up |
| Left bumper | Flywheels | hold or toggle per `FLYWHEEL_HOLD_MODE` |
| Right trigger | Fire | analog; "pulled" only above `TRIGGER_PRESS_THRESHOLD`, released below `TRIGGER_RELEASE_THRESHOLD` |
| A | Toggle ARMED/DISARMED | rising edge; arming refused if trigger is pulled and `ARM_REQUIRES_TRIGGER_RELEASED` |
| B | Toggle semi/full auto | rising edge; allowed in any state |
| X | Reset magazine to `MAG_CAPACITY` | rising edge |
| D-pad left | Set home = current pan/tilt | rising edge |
| D-pad down | Go to home | cancelled by any stick input past deadzone |
| (physical) toggle A1 | BT ↔ WiFi | changing it drops the link, tells the ESP32 to switch, re-enters PAIRING |
| (physical) toggle A0 | System on/off | §3.1 |

All button actions are edge-triggered with `BUTTON_DEBOUNCE_MS`.

### 3.3 Motion

- Velocity mode: stick deflection → degrees/second, clamped to `*_MAX_DEG_PER_S`, rate-limited by `*_ACCEL_DEG_PER_S2`. Position is tracked in steps at all times; expose it in degrees.
- Soft limits: tilt always (`0 .. TILT_RANGE_DEG` after calibration); pan only if `PAN_LIMITS_ENABLED`. Begin decelerating `SOFT_LIMIT_SLOWDOWN_DEG` before a limit; never command a step past one. During go-home, use trapezoidal position moves at `HOME_SPEED_DEG_PER_S`.
- Step generation must not stutter when the LCD or LEDs update. Budget: at `MICROSTEPS = 8`, pan at 90°/s is 2000 steps/s, tilt at 60°/s is 1067 steps/s. Use a timer-interrupt step generator or a tight non-blocking scheduler; `AccelStepper` in `runSpeed()` mode is acceptable only if measured jitter is under 10 % at those rates. Rate-limit LCD writes to changes only, max every `LCD_REFRESH_MS`. WS2812 `show()` masks interrupts for ~250 µs per 8 LEDs — schedule it, don't call it every loop.
- Home defaults to pan = 0, tilt = 0 (the post-calibration position) until D-pad-left overrides it. Home is RAM-only; it does not survive a reboot.

### 3.4 Firing

A **shot** is one servo stroke: command `TRIGGER_FIRE_ANGLE`, wait `TRIGGER_FORWARD_MS`, command `TRIGGER_REST_ANGLE`, wait `TRIGGER_RETURN_MS`, decrement ammo (floor 0). During the stroke: all 8 LEDs red for at least `LED_FIRE_FLASH_MIN_MS`, rumble per `RUMBLE_*`.

A shot is permitted only when **all** hold: state is ARMED; ammo > 0 or `FIRE_LOCKOUT_WHEN_EMPTY` is false; flywheels have been on ≥ `FLYWHEEL_SPINUP_MS` or `FIRE_REQUIRES_FLYWHEEL` is false; no stroke is already in progress.

- **Full auto:** while the trigger reads pulled, run strokes back-to-back with no gap. Releasing the trigger lets the current stroke finish, then stops.
- **Semi auto:** one stroke per pull. The next stroke needs the trigger to have dropped below `TRIGGER_RELEASE_THRESHOLD` **and** the previous stroke to have completed. A trigger held across the stroke's end does not re-fire.
- Disarm (A), link loss, power off, or a fault mid-stroke: command rest immediately, abort the stroke, do not decrement ammo.
- Empty magazine with lockout on: trigger does nothing; LEDs blink `LED_COLOR_EMPTY_MAG` while the trigger is held so the operator knows why.

### 3.5 Tilt calibration (startup)

Goal: find the bottom mechanical stop and set tilt zero a little above it.

- `CAL_RAM` (default until A2 is wired): enable drivers, drive tilt downward at `TILT_CAL_SPEED_DEG_PER_S` for `TILT_CAL_MAX_TRAVEL_DEG`. The stepper skips steps harmlessly against the bottom panel. Wait `TILT_CAL_SETTLE_MS`, define that position as the hard stop, move up `TILT_CAL_BACKOFF_DEG`, zero the tilt counter, set soft limits `0..TILT_RANGE_DEG`.
- `CAL_STALL_SENSE`: same motion, but stop the instant the stall detector (§3.6) fires. If it does not fire within `TILT_CAL_TIMEOUT_MS` → FAULT `CAL`.
- Pan is not calibrated; pan zero = wherever it was at boot.

### 3.6 Stall / crash detection

Runs whenever a stepper is moving and `STALL_DETECT_ENABLED`. Sample `PIN_STALL_SENSE` at `STALL_SAMPLE_HZ`; keep an exponential rolling baseline (`STALL_BASELINE_ALPHA`), frozen while no motor moves; ignore the first `STALL_IGNORE_AFTER_START_MS` of any move; declare a stall when `|sample − baseline| > STALL_DELTA_COUNTS` for `STALL_CONFIRM_SAMPLES` in a row. On stall: stop both axes, back off `STALL_BACKOFF_DEG` on the axis that was moving, enter FAULT for `STALL_FAULT_HOLD_MS`, log the peak delta so the threshold can be tuned. Expose the live reading and baseline on the debug serial at log level 3 — this is how the threshold gets tuned.

### 3.7 Flywheels

ENA PWM ramps 0 → `FLYWHEEL_DUTY_255` over `FLYWHEEL_RAMP_MS` (soft start). Direction pins per `FLYWHEEL_FORWARD_IN1_HIGH`. Stop = coast or brake per config. Flywheels cut immediately on disarm (if `FLYWHEEL_REQUIRES_ARMED`), link loss (if `FAILSAFE_STOP_FLYWHEEL`), power off, or fault. Track `flywheelOnSince` for the spin-up gate.

### 3.8 LEDs

Priority, highest first: FAULT blink → FIRING red → EMPTY_MAG blink (trigger held, lockout) → state colour (OFF dark / BOOT white pulse / PAIRING blue↔white pulse / DISARMED green / ARMED yellow or orange). Global brightness is clamped to `LED_MAX_BRIGHTNESS = 127` in one place; nothing may exceed it. Pulses are sinusoidal with period `LED_PULSE_PERIOD_MS`. Library: **Adafruit NeoPixel ≥ 1.12.1** (first version with UNO R4 support). Older WS2812 libraries, including Freenove's, are not confirmed to run on the Renesas core.

### 3.9 LCD

16×2. Row 0: arm state left-justified (`ARMED` / `DISARMED`, optional ` S`/` F` suffix), ammo right-justified (`AMMO:15`). Row 1: link mode left (`BT` / `WIFI`), status right (`Connected` / `Pairing` + dots / `No Link`). Dots cycle `""`, `"."`, `". ."`, `". . ."` every `LCD_PAIRING_DOT_MS`. Redraw only rows that changed; never clear the whole display per frame (it flickers). The spec string `AMMO COUNT: xx` does not fit beside `DISARMED` in 16 columns, so the config supplies shorter defaults; keep every row ≤ 16 chars including the dots.

---

## 4. RA4M1 firmware modules

One file pair per module, no cross-module globals, each module exposes `begin()` / `update()` / small API.

| Module | Owns |
|---|---|
| `ControlLink` | Parses packets from `LINK_SERIAL` (and USB serial when `DEBUG_ACCEPT_USB_CONTROL`). Sends status/rumble packets back. Tracks connected state and link timeout. |
| `Inputs` | Toggle switches (debounced), controller edge detection, stick shaping (deadzone, expo), trigger hysteresis. |
| `Motion` | Both steppers: velocity + position modes, soft limits, calibration, home. |
| `StallDetector` | §3.6 |
| `Flywheel` | §3.7 |
| `Trigger` | Servo stroke state machine, semi/full logic, ammo count. |
| `Leds` | §3.8 |
| `Lcd` | §3.9 |
| `TurretState` | The top-level state machine (§3.1) and the only place that decides who may act. |
| `Debug` | Serial logging at `DEBUG_LOG_LEVEL`, periodic loop-rate / packet-rate stats, a tiny command shell (`arm`, `fire`, `fly on`, `pan 30`, `cal`, `stall?`) for bench use. |

The main loop is non-blocking. Nothing may call `delay()` outside `setup()`.

---

## 5. Link protocol (ESP32-S3 ↔ RA4M1, and PC relay → either)

Fixed-size binary, little-endian, framed with a start byte and CRC-8. Generate `docs/protocol.md` from the struct definitions so the three implementations cannot drift.

**ControlPacket** (radio → RA4M1, at `CONTROL_PACKET_HZ`):

| Field | Type | Meaning |
|---|---|---|
| `start` | u8 | `0xA5` |
| `len` | u8 | bytes after this field, excluding crc |
| `flags` | u8 | bit0 = controller connected, bit1 = source (0 BLE, 1 WiFi), bit2 = ESP32 is in pairing mode |
| `rx`, `ry` | i16 | right stick, −512..511 |
| `rt` | u16 | right trigger, 0..1023 |
| `buttons` | u16 | bit0 A, 1 B, 2 X, 3 Y, 4 LB, 5 RB, 6 dpad-up, 7 dpad-down, 8 dpad-left, 9 dpad-right, 10 start, 11 back |
| `seq` | u8 | wraps; RA4M1 counts gaps for the stats |
| `crc` | u8 | CRC-8 over `len..seq` |

**StatusPacket** (RA4M1 → radio, on change or every 500 ms):

| Field | Type | Meaning |
|---|---|---|
| `start` | u8 | `0x5A` |
| `len` | u8 | |
| `wantMode` | u8 | 0 BLE, 1 WiFi — mirrors the physical toggle |
| `rumbleStrong`, `rumbleWeak` | u8 | 0..255 |
| `rumbleMs` | u16 | 0 = stop |
| `state` | u8 | turret state enum, for the ESP32's own debug log |
| `crc` | u8 | |

The PC relay sends the exact same `ControlPacket` bytes, over UDP to `WIFI_UDP_PORT` or over USB serial to the RA4M1 directly.

---

## 6. ESP32-S3 radio firmware

**This is the highest-risk part of the build. Do it last, after the RA4M1 firmware works over USB with the PC relay.**

Facts that constrain the design (verified 3 Nov 2026):

- The ESP32-S3 on the UNO R4 WiFi is **BLE only** — no Bluetooth Classic. Only BLE-capable Xbox controllers can pair (§9).
- Bluepad32 supports ESP32-S3 with a **subset** of controllers; Xbox over BLE, including rumble on controller firmware 5.x, is in that subset. Bluepad32 ships an Arduino IDE board package ("Arduino + Bluepad32 Boards") and an ESP-IDF template; either targets ESP32-S3.
- Bluepad32's NINA/AirLift drop-in firmware does **not** target the UNO R4 WiFi — the R4's bridge protocol is different. So the ESP32-S3 needs a bespoke sketch, not a pre-built binary.
- Flashing any custom sketch to the ESP32-S3 replaces Arduino's `uno-r4-wifi-usb-bridge` firmware. Consequences: `WiFiS3` stops working (irrelevant — we own WiFi now), **and the USB-C port stops bridging uploads to the RA4M1** unless the custom firmware re-implements the bridge.

Design:

1. **Fork `arduino/uno-r4-wifi-usb-bridge`** (Arduino-ESP32 based). Keep its USB-CDC → RA4M1 serial passthrough and the RA4M1 reset/boot handling so normal sketch uploads still work. Strip the `WiFiS3` modem command handler.
2. Add **Bluepad32** as the BLE gamepad host. On each gamepad update, pack a `ControlPacket` and write it to the RA4M1 UART at `CONTROL_PACKET_HZ` (send on a timer, not per-event, so the rate is constant and absence = link loss).
3. Add **WiFi mode**: station with `WIFI_SSID`, AP fallback if enabled; a UDP server on `WIFI_UDP_PORT` that accepts `ControlPacket`s from the PC relay and forwards them to the UART. `connected` flag = a valid packet in the last `LINK_TIMEOUT_MS`.
4. Honour `StatusPacket.wantMode`: switching tears down the other radio (BLE host off / WiFi off), sets `flags.bit2 = pairing`, and starts the requested one. Honour rumble fields via Bluepad32's dual-rumble call.
5. Mirror the needed `config.h` constants with `tools/sync_config.py` so the two builds share one source of truth.
6. `docs/flashing.md` must cover: flashing the ESP32-S3 through the board's ESP header / `esptool`, backing up the stock bridge first, and restoring the stock bridge with Arduino IDE's firmware updater if the fork breaks uploads.

**De-risk fallback, same protocol:** if the bridge fork proves unworkable, an external ESP32 (original, dual-mode BT) dev board running the same sketch minus the bridge, wired to `Serial1` (D0/D1), satisfies every requirement including classic-BT controllers. `LINK_SERIAL` in `config.h` is the only RA4M1 change. Stock bridge firmware stays on the R4. Note this option in `docs/flashing.md`; do not build it unless asked.

---

## 7. PC relay (`relay-pc/`)

Python 3, `pygame` (or `inputs`) to read an Xbox controller plugged into or paired with the PC. Maps to the same `ControlPacket` fields. Two transports, selectable by flag: `--udp host:port` and `--serial /dev/ttyACM0`. Sends at `CONTROL_PACKET_HZ`. Prints the packet rate and any CRC errors reported back. This script is both the WiFi-mode controller path and the bench tool for Stage 1.

---

## 8. Open hardware decisions (flag to Bigman; do not guess silently)

1. **Stall-sense tap on A2.** The A4988/DRV8825 boards on a CNC shield do not expose their current-sense signal on any header, and battery-voltage sensing was ruled out earlier. Candidates: (a) a resistor divider from the 6S stepper rail to A2 (catches the supply dip/spike on a stall; this is the only "voltage spike" source available without hardware changes), (b) a shunt + divider in the stepper rail ground return, (c) an external current sensor module. Until one is wired, `STALL_DETECT_ENABLED = false` and `TILT_CAL_MODE = CAL_RAM`. Software must be complete and testable with a potentiometer on A2.
2. **Which stepper is on X vs Y** on the shield. Config assumes X = pan, Y = tilt.
3. **Microstep jumper setting** on the shield must be read off the board and entered as `MICROSTEPS`.
4. **`LINK_SERIAL` object name** for the RA4M1↔ESP32 UART (`Serial2` assumed; verify in the Renesas core variant for `UNOWIFIR4`).
5. **LCD text** that does not fit 16 columns — defaults chosen in config.
6. **Pan soft limits** — off by default; turn on if wiring restricts rotation.

---

## 9. Developer prerequisites (human, before Claude Code builds)

Ordered by when they are needed.

1. Read the microstep jumpers on the CNC shield; set `MICROSTEPS`. Set driver Vref for the NEMA17 current rating.
2. Confirm which driver socket (X/Y) each stepper plugs into; adjust the two `PIN_*_STEP/DIR` pairs if swapped.
3. Run `tools/i2c_scan.ino`; set `LCD_I2C_ADDR`.
4. Install `arduino-cli`, the `arduino:renesas_uno` core, and libraries: Adafruit NeoPixel ≥ 1.12.1, Servo (bundled), an I2C LCD library confirmed on R4 (e.g. `hd44780` or `LiquidCrystal_I2C`).
5. Xbox controller: must be a BLE model — Series X|S controller (model 1914) or an older model 1708 **updated to firmware 5.x** via the Xbox Accessories app. Pre-5.x 1708 controllers are Bluetooth Classic and will never pair to the ESP32-S3.
6. WiFi SSID/password for station mode; a PC with Python 3 and the controller for relay mode.
7. For the ESP32-S3 stage: install the Bluepad32 Arduino board package (board-manager URL from the Bluepad32 docs) or ESP-IDF 5.3/5.4; `esptool`; **back up the stock bridge firmware before flashing anything**.
8. Decide item 8.1 (stall tap). Not blocking — the rest builds without it.

---

## 10. Codebase assumptions for Claude Code to verify before planning

- `config.h` is present at the repo root and compiles as a C++ header on `arduino:renesas_uno`.
- The Renesas core variant for `UNOWIFIR4` defines the UART object wired to the ESP32-S3 (confirm the name; update `LINK_SERIAL`).
- `Adafruit_NeoPixel` installed version ≥ 1.12.1 (check `library.properties`).
- `Servo` compiles for `renesas_uno` and `attach(pin, min, max)` is available.
- The chosen LCD library compiles for `renesas_uno` and does not use AVR-only registers.
- `arduino-cli compile --fqbn arduino:renesas_uno:unor4wifi` succeeds on an empty sketch (toolchain sanity).
- `pygame` can enumerate an Xbox controller on the developer's OS.
- The `arduino/uno-r4-wifi-usb-bridge` repo builds with the Arduino-ESP32 core version it pins (check its README/CI before forking).

If any of these fail, stop and report before writing a plan.

---

## 11. Acceptance criteria

Each is independently testable on the bench with the PC relay over USB unless marked *(radio)*.

1. Power toggle open → everything idle, LCD `SYSTEM OFF`, LEDs dark; closing it reruns BOOT → CALIBRATING → PAIRING.
2. Boot: LEDs pulse white; tilt drives down, stops at the panel, lifts `TILT_CAL_BACKOFF_DEG`, reports tilt = 0; LCD shows `Calibrating` then the pairing row.
3. Pairing: LEDs pulse blue/white; LCD row 1 shows `BT Pairing` with dots cycling at `LCD_PAIRING_DOT_MS`; first valid packet → DISARMED, green, `Connected`.
4. Right stick: pan/tilt move with speed proportional to deflection (expo curve), stop in the deadzone, never exceed max speeds, never cross a soft limit, decelerate near limits.
5. D-pad left then move away then D-pad down → returns to the stored position at `HOME_SPEED_DEG_PER_S`; a stick nudge mid-move cancels it.
6. A toggles ARMED/DISARMED; LEDs yellow/orange/green accordingly; LCD row 0 updates; arming is refused while the trigger is pulled (if configured).
7. B toggles semi/full; LEDs yellow↔orange when armed.
8. LB spins flywheels only when permitted; duty ramps over `FLYWHEEL_RAMP_MS`; measured ENA duty ≈ `FLYWHEEL_DUTY_255 / 255`.
9. Semi auto: one stroke per pull above 60 %; holding the trigger does not re-fire; release below 40 % then pull again fires again; ammo decrements by 1 per stroke.
10. Full auto: strokes back-to-back while held, no gap; release finishes the current stroke and stops.
11. Each stroke: LEDs red ≥ `LED_FIRE_FLASH_MIN_MS`; a rumble command of the configured strength/duration goes out on the link (*radio*: controller rumbles).
12. Ammo reaches 0 → trigger ignored (lockout on), magenta blink while held; X restores `MAG_CAPACITY` and firing resumes.
13. Pulling the USB cable / stopping the relay → within `LINK_TIMEOUT_MS` motion stops, flywheels stop, state DISARMED, PAIRING shown; reconnecting restores DISARMED only.
14. Flipping the mode toggle → `StatusPacket.wantMode` changes, LCD mode label changes, state → PAIRING (*radio*: ESP32 switches radios).
15. With a potentiometer on A2 and `STALL_DETECT_ENABLED`, a sudden twist while moving → both axes stop, back-off, red blink, LCD `FAULT: STALL`, auto-clear after `STALL_FAULT_HOLD_MS`.
16. LED brightness never exceeds 127 in any state (assert in code; inspect with a scope or current meter).
17. Loop timing: with LCD + LEDs active and both steppers at max speed, step jitter < 10 % and packet loss < 1 % over 60 s (debug stats print proves it).
18. *(radio)* Xbox controller pairs over BLE, drives everything above, and the RA4M1 still accepts sketch uploads over USB-C with the forked bridge.
19. *(radio)* WiFi mode: PC relay over UDP drives everything above.
20. `grep` for numeric literals in any source outside `config.h` and the protocol struct returns only loop counters and array indices.

---

## 12. Build stages (sequence, not detail)

1. **RA4M1 core, bench-driven.** `config.h` wiring, `TurretState`, `Inputs`, `Motion` (velocity, limits, calibration in `CAL_RAM`), `Flywheel`, `Trigger`, `Leds`, `Lcd`, `Debug` shell, `ControlLink` accepting packets over USB serial. Deliverable: everything in §11 items 1–13 and 16–17 passes with `tools/bench_console.py` and the PC relay over USB.
2. **PC relay.** Real Xbox controller on the PC → packets over USB serial. Re-run §11 with a real stick. Add the UDP transport.
3. **Stall detector + `CAL_STALL_SENSE` path.** Software complete and tested with a pot on A2 (§11 item 15). Leave disabled in config.
4. **ESP32-S3 firmware.** Fork the bridge, add Bluepad32 BLE + WiFi UDP + UART link + mode switching + rumble. `docs/flashing.md`. §11 items 18–19.
5. **Integration pass.** Link-loss, mode-switch, and power-toggle behaviour end to end on the real turret; tune nothing — hand the tunables list back to Bigman.

---

## 13. Do not

- Do not hard-code any tunable outside `config.h`.
- Do not use `delay()` in the main loop or in any module `update()`.
- Do not raise LED brightness above `LED_MAX_BRIGHTNESS` anywhere, including "briefly".
- Do not add battery-voltage monitoring, a camera, hall sensors, or buck converters — all previously ruled out.
- Do not build the external-ESP32 fallback unless asked; document it only.
- Do not persist home position or settings to flash (RAM only, by design).
- Do not implement a web UI, OTA, telemetry logging, or multi-controller support — out of scope.
- Do not attempt classic-Bluetooth pairing on the ESP32-S3; it is BLE-only hardware.
- Do not write code that only compiles for AVR; the target is Renesas RA4M1.

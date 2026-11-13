<!-- SPDX-License-Identifier: MIT -->
# Turret dashboard

A live web page that shows what the Xbox controller is doing to the Nerf blaster turret:
where the right stick is aiming, how far the right trigger is pulled, the fire-mode and
armed switches, and whether the flywheels are spinning. The board prints one text line per
controller report over USB; `tools/dashboard.py` reads those lines, works out the turret
logic and serves the page at <http://127.0.0.1:8765/>.

| Control | Turret action | On the page |
|---|---|---|
| Right stick | aims the turret | TURRET AIM square |
| Right trigger (RT) | fires | TRIGGER bar and FIRING lamp |
| A | toggles SEMI / FULL AUTO | FIRE MODE switch |
| X | toggles SAFE / ARMED | ARMED switch |
| Left bumper (LB), held | spins the flywheels | FLYWHEELS |

## 1. Upload the TurretTelemetry sketch

The sketch is `ra4m1/libraries/XboxControllerR4/examples/TurretTelemetry`. It needs the
forked ESP32-S3 bridge firmware, like the other examples.

```sh
cd "/Users/jwilson/Downloads/University/Meche Projects/A4"
SKETCH=ra4m1/libraries/XboxControllerR4/examples/TurretTelemetry
arduino-cli board list                      # the UNO R4 WiFi shows as 2341:1002
arduino-cli compile --fqbn arduino:renesas_uno:unor4wifi --libraries ra4m1/libraries "$SKETCH"
arduino-cli upload  --fqbn arduino:renesas_uno:unor4wifi -p /dev/cu.usbmodemD885ACA7B66C2 "$SKETCH"
```

In the Arduino IDE (with the library installed as the library README describes):
File > Examples > XboxControllerR4 > TurretTelemetry, then Upload.

## 2. One program on the port at a time

A serial port can be open in only one program. Close the IDE's Serial Monitor (and any
other terminal on the port) before you start the dashboard. The other way round matters
too: **stop the dashboard (Ctrl+C) before uploading a sketch**, or the upload fails because
the port is busy. Ctrl+Z only pauses the dashboard and it keeps holding the port; the open
page then shows NO UPDATES.

## 3. Run the dashboard

```sh
cd "/Users/jwilson/Downloads/University/Meche Projects/A4"
PY="$PWD/.venv/bin/python"
"$PY" tools/dashboard.py
```

It finds the one port with USB ID 2341:1002 and opens your browser at the page. It refuses
to guess when there is no such port or more than one; then pass `--port`. If the board is
unplugged while the dashboard runs, the dashboard retries every second and carries on when
the board comes back.

| Option | Effect |
|---|---|
| `--demo` | No board needed: a pretend controller (see below). |
| `--port PORT` | Use this serial port instead of auto-detecting. |
| `--baud N` | Serial speed, default 115200 (must match the sketch). |
| `--http-port N` | Web port, default 8765. |
| `--host 0.0.0.0` | Let a phone on the same network open the page (default: this computer only). |
| `--no-browser` | Do not open a browser tab. |
| `--log FILE` | Append every received line to FILE, each prefixed with the computer's time. |

### Preview without a controller: `--demo`

```sh
"$PY" tools/dashboard.py --demo
```

The demo never touches a serial port. Its pretend controller connects after 2 s, moves the
right stick round a slow circle and pauses in each quadrant, pulses the right trigger every
3 s, presses A every 4 s and X every 9 s, and holds LB for 3 s of every 6 s. Every 40 s the
link drops for 3 s, alternating between the controller disconnecting and the board going
silent, so you can watch both safety resets happen.

### Record a session: `--log`

```sh
"$PY" tools/dashboard.py --log tools/logs/turret-session.csv
```

Each line is the computer's timestamp, a comma, then the line exactly as the board sent it:

```
2026-10-12T13:31:25.574+11:00,XR4,1258,0,87,12,32768,32768,61234,4096,0,512,0011,0
2026-10-12T13:31:26.002+11:00,# state: scanning
```

## The line format

The sketch prints, at 115200 baud, one line per new controller report plus a heartbeat at
least every 100 ms:

```
XR4,<millis>,<state>,<battery>,<seq>,<lx>,<ly>,<rx>,<ry>,<lt>,<rt>,<buttons_hex4>,<dpad>
```

| Field | Meaning |
|---|---|
| `millis` | The board's clock when the line was printed, in ms. |
| `state` | 0 connected, 1 waiting for first report, 2 found (connecting), 3 scanning, 4 off, 5 blocked by ArduinoBLE. |
| `battery` | 0..100 %, or 255 unknown. |
| `seq` | 0..255, the ESP32-S3's report counter (low byte). |
| `lx ly rx ry` | Sticks, raw 0..65535, centre about 32768. Vertical 0 is **up**; the dashboard flips it so up is positive. |
| `lt rt` | Triggers, raw 0..1023. |
| `buttons_hex4` | 4 hex digits; bit 0 A, 1 B, 2 X, 3 Y, 4 LB, 5 RB, 6 View, 7 Menu, 8 Xbox, 9 LS click, 10 RS click, 11 Share. Example `0011` = A + LB. |
| `dpad` | 0 none, 1 N, 2 NE, 3 E, 4 SE, 5 S, 6 SW, 7 W, 8 NW. |

Lines that start with `#` are messages for people, for example
`# begin ok: firmware 0.6.99 protocol 1`, `# begin failed: lastError 1 (stock bridge firmware)`
or `# link lost: no good poll for 200 ms`. They appear in the event log.

The sketch prints no XR4 lines while the ESP32-S3 is not answering (the values would be
stale), so a silent board and an unplugged cable look the same to the dashboard: LINK LOST.

Typing `scan` in the Serial Monitor (with the dashboard stopped) prints
`# scan: <...>`: the Bluetooth devices the ESP32-S3 saw last. Useful when the controller
will not pair.

## What each panel means

- **Header strip**: the serial port (or `demo`), the controller state (green CONNECTED;
  amber SCANNING, CONNECTING or WAITING; red OFF or BLOCKED), battery, reports per second
  (XR4 lines received in the last second), and the link. A DEMO badge shows in demo mode.
  While the values are stale, the controller state turns grey and reads, for example,
  `CONNECTED (stale)`: it is the last state the board reported, not the current one.
- **LINK LOST banner**: no valid XR4 line for 300 ms. It shows how long ago the last one
  came and the board's last `#` message. The panels grey out because the values are stale.
- **NO UPDATES banner**: the page has heard nothing from the dashboard program for 2.5 s
  (it sends at least one message a second), for example because the program is paused or
  hung, or a phone lost Wi-Fi. The display is frozen and the turret state is unknown; the
  page reconnects every 5 s. **DASHBOARD OFFLINE** means the page cannot reach the program.
- **TURRET AIM**: the dot is the right stick; the trail shows where it has just been. The
  shaded circle in the middle is the 15 % deadzone: a stick resting a few percent off centre
  still counts as CENTRE. Outside it, pan and tilt are read separately, because they are
  separate motions: LEFT or RIGHT lights once the stick is 15 % or more to that side, and UP
  or DOWN likewise, so a stick pushed right and a little up reads AIM: RIGHT · UP. Just
  outside the circle, where neither axis reaches 15 %, the larger one is named.
- **TRIGGER**: the right trigger from 0 to 100 %. The dashed line is the 10 % firing
  threshold. The FIRING lamp lights red only when armed and past the line. While disarmed
  the panel is dimmed and stamped SAFE.
- **FIRE MODE** switch: flips between SEMI and FULL AUTO each time A is pressed.
- **ARMED** switch: flips between SAFE and ARMED each time X is pressed; red when armed.
- **FLYWHEELS**: SPINNING while LB is held, idle otherwise.
- **RAW INPUTS**: all 12 buttons, the d-pad, the left stick and LT, plus the report counter,
  the board clock, the raw button bits and counts of good and bad lines. Useful for checking
  wiring and mapping before trusting the turret panels.
- **EVENT LOG**: the last 50 board messages, switch flips and safety resets, newest first.

The page is dark by default and follows the computer's setting: with the system in light
mode it uses a light palette with the same colour meanings.

## Safety rules (computed in `TurretState`, in `dashboard.py`)

1. A and X act on the **press** (the moment the button goes down), not while held: holding
   A flips the mode once. A button already held when the dashboard starts, or held through
   a dropout, must be released and pressed again before it counts.
2. Firing needs **armed** and the right trigger **past 10 %**.
3. Whenever the controller is not connected (state not 0) or the link is lost (no valid
   line for 300 ms), everything resets: fire mode SEMI, SAFE, flywheels off. The reset is
   logged once.

These rules live in the dashboard so they can be seen and tested now. The turret firmware
must apply the same rules itself on the board, because the dashboard is only a display.

## Tests

```sh
"$PY" tools/tests/test_dashboard.py -v
```

They cover line parsing, the axis and d-pad conversions, the toggle and safety logic, the
serial reader against a fake port, the demo, and a run of the real server in `--demo`
mode. Nothing in them opens a real serial port.

## Firing cycle (added 2026-10-12)

The dashboard models the blaster, not just the buttons:

- **Flywheels** need **1.0 s** of spin (LB held) before a dart can fire. The FLYWHEELS panel
  shows SPINNING UP with a progress bar, then READY in green.
- **Semi-auto**: one dart per trigger pull past 10%; release the trigger below 10% to reload.
  Never more than three darts per second: a tap during the ⅓ s cooldown is queued and fires
  when the cooldown ends (one queued dart at most; disarming or dropping the wheels cancels it).
- **Full auto**: three darts per second while the trigger is held past 10%; the first dart
  leaves immediately.
- The red dot in the trigger panel flashes for 0.1 s every time a dart fires: one blink per pull
  in semi, and in full auto one blink per dart, so it blinks three times a second. The note under
  the trigger says why nothing fires (disarmed, wheels spinning up).
- The dart counter under the lamp counts every dart since the dashboard started; `/events`
  frames carry `shots`, `flywheels_ready`, `spinup`, `trigger_pulled` and `last_shot_age` so a
  turret controller can mirror the same cycle.
- Everything still resets to SAFE (semi, disarmed, wheels off, no shots pending) when the
  controller disconnects or the board goes quiet.

Constants: `SPINUP_S`, `FULL_AUTO_RPS` and `FIRE_THRESHOLD` at the top of `dashboard.py`.

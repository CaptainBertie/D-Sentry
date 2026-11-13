# D-Sentry PC relay and dashboard

The PC side of D-Sentry (`docs/architecture.md` section 7). The relay reads a game pad on the
Mac, sends ControlPackets to the turret at `CONTROL_PACKET_HZ`, and serves a local web dashboard
of the turret's telemetry. The dashboard can also drive the turret from the keyboard.
Every tunable comes from `D-SentryCode/config.h`; nothing here copies a number.

Run everything with the project venv (`pygame==2.6.1`, `pyserial==3.5`, see `requirements.txt`):

```sh
PY="<repo>/.venv/bin/python"
"$PY" D-SentryCode/relay-pc/dsentry_relay.py --demo     # no hardware: a Python stand-in turret
```

The dashboard opens at `http://127.0.0.1:8766/` (`DASHBOARD_HTTP_PORT`).

## Links

| Flag | What it does |
|---|---|
| `--serial [PORT\|auto]` (the default) | USB serial to the RA4M1. `auto` picks the one port with USB ID 2341:1002. The relay opens it at exactly `DEBUG_SERIAL_BAUD`, because 1200 or 2400 baud resets the RA4M1. It sets `write_timeout` to `RELAY_SERIAL_WRITE_TIMEOUT_MS`, reads all the time, and after any error closes the port and reopens it every `RELAY_REOPEN_MS`. |
| `--udp [HOST[:PORT]\|auto]` | WiFi mode. With `auto`, the relay first listens for the turret's beacon on `WIFI_DISCOVERY_PORT`. If none arrives and this Mac is on the turret's own AP subnet, it uses `WIFI_AP_IP`. Replies come back to the relay's own socket. |
| `--monitor [PORT\|auto]` | BT mode. The relay reads telemetry over USB and **sends nothing**, so the RA4M1 never sees two control sources. |
| `--sim` | Starts the simulator (`sim/build/dsentry_sim`, or `--sim-bin`, or `$DSENTRY_SIM_BIN`) as `<bin> --udp-port PORT [--sim-arg ...]`, then talks UDP to it on 127.0.0.1. The simulator's stdout appears in the dashboard as RA4M1 text. If the simulator exits, the relay stops with code 1. |
| `--demo` | A small Python stand-in turret, labelled DEMO everywhere, for previewing the page. It is **not** the firmware; use `--sim` to run the real logic. A scripted input drives it until a pad or the keyboard takes over; `--no-demo-input` turns the script off. |

Other flags: `--no-pad`, `--http-port N` (0 picks any free port), `--host ADDR`, `--no-browser`,
`--quiet` (no stats line), `--config PATH`.

`--host 0.0.0.0` lets a phone **view** the dashboard. Keyboard control still works only in a
browser on the relay's own computer.

Once a second the terminal shows the tx rate, the telemetry rate (and packets lost), the
turret's CRC errors (from telemetry), the relay's own deframer CRC errors, the round trip from
`lastCtrlSeq`, the input source and the link.

## Input

- **Pad.** The relay uses SDL through `pygame._sdl2.controller` on the main thread, headless
  (`SDL_VIDEODRIVER=dummy`), and pumps events at `RELAY_PAD_POLL_HZ`. Sticks are scaled with
  `>> 6` and the trigger with `>> 5`, both shifts derived from `JOY_AXIS_RANGE` and
  `TRIGGER_AXIS_RANGE`. Y is sent as SDL reports it (up is negative). Rumble follows the turret's
  StatusPacket, but only when its `rumbleSeq` is new; `rumbleMs` 0 stops the rumble.
- **One source at a time:** a connected pad wins, then the dashboard keyboard, then (demo only)
  the script. A pad locks the keyboard out. **Any** change of source sends `connected=0` for
  `RELAY_SOURCE_CHANGE_HOLD_MS`, which is longer than `LINK_TIMEOUT_MS`. The RA4M1 therefore
  sees a link loss and comes back DISARMED, never ARMED.
- **Flags sent:** `connected` while a source is live; `source` = WiFi always (the relay is the
  WiFi side of the source bit, whatever cable it uses); `keyboard` while the keyboard drives.

## Keyboard control (dashboard)

Keyboard control is off at every start. Click **Enable keyboard** in the tab you want to use.
A red KEYBOARD LIVE banner shows while it is on. Keys:

| Keys | Action |
|---|---|
| W / S / Up / Down | tilt |
| A / D / Left / Right | pan (with Shift: full deflection; without: `KEYBOARD_DEFLECTION`) |
| Space | right trigger (fire) |
| F | left bumper (flywheels) |
| Enter | A: arm / disarm |
| M | B: semi / full |
| R | X: magazine reset |
| H / G | d-pad left (set home) / d-pad down (go home) |
| Esc | panic: neutral, keyboard off |

Safety:

- The relay accepts key state only with the per-boot token in an `X-DSentry-Token` header, a
  `Host` header of `127.0.0.1:port` or `localhost:port`, a loopback client and a JSON body.
- It never answers a CORS preflight, so a page from another site cannot send key state.
- With no key state for `LINK_TIMEOUT_MS`, the output goes neutral (the deadman). After
  `KEYBOARD_REVOKE_MS`, the session is revoked.
- When the tab loses focus or is hidden or closed, or Cmd is pressed, every key is released.
- A tap is held for `KEYBOARD_PRESS_MS` and then released for the same time, so the RA4M1's
  debounce sees every tap. At most `KEYBOARD_PRESS_QUEUE_MAX` taps per key wait in line.

## Dashboard

The dashboard is one file (`web/index.html`) with no network resources. Its light or dark theme
follows the system. It shows:

- the turret in 3D (drag to orbit, wheel to zoom), with the home direction and the pan and tilt
  limits;
- beside it, a CONTROLLER MAP: a drawing of the Xbox controller with what each control does on
  this turret (spec 3.2 as built). A held button glows, the sticks move (with the `JOY_DEADZONE`
  ring), LT and RT fill (RT with its pull and release points). The labels come from `config.h`
  (`FLYWHEEL_HOLD_MODE`, the `SERVO_TEACH_STEP_*` nudges, `TRIGGER_REST_ANGLE` /
  `TRIGGER_FIRE_ANGLE`). The map switches to the servo teach mapping when LCD row 0 starts with
  `LCD_TXT_SERVO_TEACH`. It greys what the turret's state refuses, for example RT while not ARMED.
  It is lit from the same pad as the CONTROLLER panel. On a narrow screen it stacks below the
  turret and lists the labels under the drawing;
- state, fault, fire mode, ammo, flywheel and trigger;
- mirrors of the LCD and the LED ring;
- link and latency, loop rate and step jitter;
- a graph of the stall sensor;
- the input being sent, and the keyboard panel;
- a CONTROLLER panel with the full pad state: both sticks (with the `JOY_DEADZONE` ring), both
  triggers (RT with its pull and release points), the d-pad, all twelve buttons, the battery and
  the connection state. The panel says which pad it describes. While this relay drives the turret
  with its own pad, it shows that pad as SDL reads it ("relay pad"). With the keyboard or the
  demo script it shows that input. Otherwise (`--monitor`, or no input here) it shows what the
  turret reports in its telemetry: in BT mode, the pad paired with the turret ("pad on the turret
  (BLE)"). An RA4M1 built before the controller block (TelemetryPacket version 1) still shows on
  every other panel; this one then says "not reported by this firmware";
- an event log built from telemetry changes, and the RA4M1's text.

A value that is not live is greyed and stamped. This happens when the telemetry is stale (none
for `DASHBOARD_STALE_MS`), when no telemetry has arrived, when the telemetry version is wrong,
or when the relay has stopped responding. The dashboard never shows such a value as live.

## Bench console

`tools/bench_console.py [--port PORT|auto] [--disconnected]` is a line console on the USB port.
It sends neutral ControlPackets at `CONTROL_PACKET_HZ` and offers these commands:

- `tap` / `hold` / `release` a button (or `rt`);
- `stick x y` and `trigger f`;
- `connect` / `disconnect` and `neutral`;
- `!<line>` sends a line to the RA4M1 shell;
- `status`, `watch on|off`, `quit`.

It prints the RA4M1's text and summaries of its Status and Telemetry packets.

## Tests

```sh
"$PY" -m unittest discover -s D-SentryCode/relay-pc/tests -v
```

The tests are stdlib only and need no hardware:

- codec use, SDL scaling, the button map and the SdlPad logic against a fake SDL;
- the keyboard rules, press stretching, and the source-change hold;
- the transports over a pty pair and loopback UDP with a fake turret, beacon discovery and the
  AP fallback;
- `--sim` with a stand-in simulator;
- the HTTP server in-process and the whole relay as a subprocess;
- the bench console against a pty.

The relay against the real `dsentry_sim` (`--sim`, `/events`, the keyboard POSTs, `--serial` on
the simulator's pty) is covered by `D-SentryCode/tests/acceptance`, which `tests/run.sh` runs.

## Not done here

- Checking rumble on a real BLE pad on macOS 26.

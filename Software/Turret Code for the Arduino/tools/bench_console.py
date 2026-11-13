#!/usr/bin/env python3
# SPDX-License-Identifier: MIT
"""bench_console.py -- a line-based console on the RA4M1's USB port (docs/architecture.md 7).

It keeps sending ControlPackets at CONTROL_PACKET_HZ (neutral, connected, unless told otherwise),
lets you tap/hold buttons, set the stick and trigger, connect/disconnect, and send shell lines
('!<line>'). It prints the RA4M1's text lines and short summaries of its Status and Telemetry
packets. The port is opened at exactly DEBUG_SERIAL_BAUD and always read (dsentry.transports).

    "<repo>/.venv/bin/python" D-SentryCode/tools/bench_console.py            # auto-detect the board
    "<repo>/.venv/bin/python" D-SentryCode/tools/bench_console.py --port /dev/cu.usbmodem1101

Type 'help' for the commands. Output lines start with:  '< ' RA4M1 text, 'status:' and
'telemetry:' decoded packets, '# ' console notes, '? ' a refused command.
"""

import argparse
import sys
import threading
import time
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "relay-pc"))

from dsentry import hub as H                 # noqa: E402
from dsentry import protocol as P            # noqa: E402
from dsentry import relay as R               # noqa: E402
from dsentry import settings as S            # noqa: E402
from dsentry import transports as T          # noqa: E402

BUTTONS = {
    "a": P.DS_BTN_A, "b": P.DS_BTN_B, "x": P.DS_BTN_X, "y": P.DS_BTN_Y, "lb": P.DS_BTN_LB,
    "rb": P.DS_BTN_RB, "up": P.DS_BTN_DPAD_UP, "down": P.DS_BTN_DPAD_DOWN, "left": P.DS_BTN_DPAD_LEFT,
    "right": P.DS_BTN_DPAD_RIGHT, "start": P.DS_BTN_START, "back": P.DS_BTN_BACK,
}
TRIGGER_NAME = "rt"           # tap/hold rt = trigger fully pulled

HELP = """commands:
  connect | disconnect       ControlPacket flags.connected 1 / 0 (packets keep flowing either way)
  neutral                    stick centred, trigger released, every button released
  tap <button> [ms]          press, then release after ms (default KEYBOARD_PRESS_MS)
  hold <button>              press until 'release'
  release <button>|all
  stick <x> <y>              right stick, -1..1 each; x right positive, y UP positive
  trigger <0..1>             right trigger fraction
  !<line>                    send <line> to the RA4M1 shell (e.g. !status, !pan 30)
  status                     print the latest telemetry in full
  watch on|off               a telemetry summary every RELAY_STATS_MS
  quit | exit
buttons: a b x y lb rb up down left right start back, and rt (the trigger, fully pulled)"""


class ConsoleInput:
    """The input the console sends. Thread-safe: stdin thread writes, sender thread reads."""

    def __init__(self, settings, connected: bool = True, clock=time.monotonic) -> None:
        self.s = settings
        self._clock = clock
        self._lock = threading.Lock()
        self.connected = connected
        self.rx = self.ry = self.rt = 0
        self.held = 0                  # buttons held until released
        self.rt_held = False
        self._taps = {}                # name -> release time

    def build(self, now: float) -> R.InputSnapshot:
        with self._lock:
            for name, until in list(self._taps.items()):
                if now >= until:
                    del self._taps[name]
            buttons = self.held
            rt = self.rt
            for name in self._taps:
                if name == TRIGGER_NAME:
                    rt = self.s.trigger_axis_range
                else:
                    buttons |= BUTTONS[name]
            if self.rt_held:
                rt = self.s.trigger_axis_range
            if not self.connected:
                return R.InputSnapshot("console", False, False, 0, 0, 0, 0, 0.0)
            return R.InputSnapshot("console", True, False, self.rx, self.ry, rt, buttons, 0.0)

    def tap(self, name: str, ms: float) -> None:
        with self._lock:
            self._taps[name] = self._clock() + ms / 1000.0

    def hold(self, name: str, on: bool) -> None:
        with self._lock:
            if name == TRIGGER_NAME:
                self.rt_held = on
            elif on:
                self.held |= BUTTONS[name]
            else:
                self.held &= ~BUTTONS[name]

    def neutral(self) -> None:
        with self._lock:
            self.rx = self.ry = self.rt = 0
            self.held = 0
            self.rt_held = False
            self._taps.clear()

    def stick(self, x: float, y: float) -> None:
        r = self.s.joy_axis_range
        with self._lock:
            self.rx = max(-r, min(r - 1, round(x * r)))
            self.ry = max(-r, min(r - 1, round(-y * r)))     # the wire is up-negative

    def trigger(self, f: float) -> None:
        with self._lock:
            self.rt = max(0, min(self.s.trigger_axis_range, round(f * self.s.trigger_axis_range)))

    def set_connected(self, on: bool) -> None:
        with self._lock:
            self.connected = on


def telemetry_summary(t: P.TelemetryPacket) -> str:
    v = H.telemetry_view(t)
    fb = v["flagBits"]
    return ("telemetry: %s%s %s ammo %d/%d pan %.2f tilt %.2f link %s%s fly %d%s%s lcd |%s|%s|" % (
        v["stateName"], "" if t.fault == P.DS_FAULT_NONE else "(%s)" % v["faultName"], v["fireModeName"],
        t.ammo, t.magCapacity, v["panDeg"], v["tiltDeg"], v["linkSourceName"],
        "" if fb["linkConnected"] else " (not connected)", t.flywheelDuty,
        " ready" if fb["flywheelReady"] else "", " homing" if fb["homing"] else "",
        v["lcdRows"][0], v["lcdRows"][1]))


class Printer:
    """The transport sink: prints what the board sends."""

    KEY_FIELDS = ("state", "fault", "fireMode", "linkSource", "ammo", "homeTiltCdeg", "homePanCdeg")

    def __init__(self, settings, out=sys.stdout) -> None:
        self.s = settings
        self.out = out
        self._lock = threading.Lock()
        self.telemetry = None
        self.status = None
        self.watch = False
        self._last_key = None
        self._last_watch = 0.0

    def say(self, text: str) -> None:
        with self._lock:
            self.out.write(text + "\n")
            self.out.flush()

    def note(self, kind: str, text: str) -> None:
        self.say("# " + text)

    def on_text(self, text: str, via: str = "") -> None:
        self.say("< " + text)

    def on_frame(self, frame, via: str = "") -> None:
        pkt = frame.packet
        if frame.type == P.PacketType.STATUS:
            if self.status is None or pkt.payload() != self.status.payload():
                self.say("status: want %s state %s rumble #%d (%d/%d, %d ms)" % (
                    H.enum_name(P.RadioMode, pkt.wantMode), H.enum_name(P.TurretState, pkt.state),
                    pkt.rumbleSeq, pkt.rumbleStrong, pkt.rumbleWeak, pkt.rumbleMs))
            self.status = pkt
        elif frame.type == P.PacketType.TELEMETRY:
            self.telemetry = pkt
            if not H.telemetry_layout_ok(pkt, len(frame.raw) - P.DS_FRAME_OVERHEAD if frame.raw else None):
                self.say("telemetry: version %d, expected %d: ignored" % (pkt.version, P.DS_TELEMETRY_VERSION))
                return
            key = (tuple(getattr(pkt, f) for f in self.KEY_FIELDS), pkt.flags & P.DS_TELEM_FLAG_LINK_CONNECTED)
            now = time.monotonic()
            due = self.watch and now - self._last_watch >= self.s.stats_s
            if key != self._last_key or due:
                self._last_key = key
                self._last_watch = now
                self.say(telemetry_summary(pkt))

    def full(self) -> str:
        t = self.telemetry
        if t is None:
            return "telemetry: none received yet"
        v = H.telemetry_view(t)
        lines = ["telemetry (full):"]
        for k, val in v.items():
            lines.append("  %-20s %s" % (k, val))
        return "\n".join(lines)


def run_command(line: str, inp: ConsoleInput, printer: Printer, transport, settings) -> bool:
    """One console line. Returns False to quit."""
    line = line.rstrip("\r\n")
    if not line.strip():
        return True
    if line.startswith("!"):
        shell = line[1:]
        limit = settings.value("SHELL_LINE_MAX")
        if limit is not None and len(shell) > limit:
            printer.say("? shell line is %d characters; the RA4M1 drops lines over SHELL_LINE_MAX %d" % (len(shell), limit))
            return True
        if transport.send((shell + "\n").encode("ascii", "replace")):
            printer.say("# sent shell line: %s" % shell)
        else:
            printer.say("? not sent: the port is not open")
        return True
    words = line.split()
    cmd, args = words[0].lower(), words[1:]
    try:
        if cmd in ("quit", "exit"):
            return False
        if cmd == "help":
            printer.say(HELP)
        elif cmd == "connect":
            inp.set_connected(True)
            printer.say("# connected=1")
        elif cmd == "disconnect":
            inp.set_connected(False)
            printer.say("# connected=0")
        elif cmd == "neutral":
            inp.neutral()
            printer.say("# neutral")
        elif cmd in ("tap", "hold", "release"):
            if not args:
                raise ValueError("%s needs a button" % cmd)
            name = args[0].lower()
            if cmd == "release" and name == "all":
                inp.neutral()
                printer.say("# released everything")
                return True
            if name not in BUTTONS and name != TRIGGER_NAME:
                raise ValueError("unknown button %r" % name)
            if cmd == "tap":
                ms = float(args[1]) if len(args) > 1 else settings.keyboard_press_s * 1000.0
                if ms <= settings.button_debounce_s * 1000.0:
                    printer.say("# note: %g ms is not longer than BUTTON_DEBOUNCE_MS" % ms)
                inp.tap(name, ms)
                printer.say("# tap %s %g ms" % (name, ms))
            else:
                inp.hold(name, cmd == "hold")
                printer.say("# %s %s" % (cmd, name))
        elif cmd == "stick":
            if len(args) != 2:
                raise ValueError("stick needs x and y")
            x, y = float(args[0]), float(args[1])
            if not (-1.0 <= x <= 1.0 and -1.0 <= y <= 1.0):
                raise ValueError("stick values are -1..1")
            inp.stick(x, y)
            printer.say("# stick rx %d ry %d" % (inp.rx, inp.ry))
        elif cmd == "trigger":
            if len(args) != 1:
                raise ValueError("trigger needs one value 0..1")
            f = float(args[0])
            if not 0.0 <= f <= 1.0:
                raise ValueError("trigger is 0..1")
            inp.trigger(f)
            printer.say("# trigger %d" % inp.rt)
        elif cmd == "status":
            printer.say(printer.full())
        elif cmd == "watch":
            if not args or args[0] not in ("on", "off"):
                raise ValueError("watch on|off")
            printer.watch = args[0] == "on"
            printer.say("# watch %s" % args[0])
        else:
            raise ValueError("unknown command %r (try help)" % cmd)
    except ValueError as error:
        printer.say("? %s" % error)
    return True


def main(argv=None) -> int:
    p = argparse.ArgumentParser(description="D-Sentry bench console on the RA4M1's USB port")
    p.add_argument("--port", default="auto", help="serial port (default auto: the one USB 2341:1002)")
    p.add_argument("--disconnected", action="store_true", help="start with connected=0")
    p.add_argument("--config", help="config.h to read")
    args = p.parse_args(argv)
    try:
        settings = S.Settings.load(args.config)
    except (OSError, S.SettingsError) as error:
        print("bench_console: cannot use config.h: %s" % error, file=sys.stderr)
        return 2
    printer = Printer(settings)
    transport = T.SerialTransport(settings, printer, port=None if args.port == "auto" else args.port)
    inp = ConsoleInput(settings, connected=not args.disconnected)
    sender = R.Sender(settings, transport, inp.build)
    stop = threading.Event()
    transport.start(stop)
    th = threading.Thread(target=sender.run, args=(stop,), daemon=True, name="sender")
    th.start()
    printer.say("# bench console: ControlPackets at %g Hz, %s; type help" % (
        settings.control_hz, "connected" if inp.connected else "disconnected"))
    try:
        for line in sys.stdin:
            if not run_command(line, inp, printer, transport, settings):
                break
    except KeyboardInterrupt:
        pass
    finally:
        stop.set()
        th.join(1.0)                 # the sender sends one connected=0 packet on its way out
        transport.join(1.0)
        printer.say("# bench console stopped")
    return 0


if __name__ == "__main__":
    sys.exit(main())

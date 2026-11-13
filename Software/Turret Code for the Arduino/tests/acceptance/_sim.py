# SPDX-License-Identifier: MIT
"""Shared helpers for the acceptance tests (docs/architecture.md 8.4): start dsentry_sim, read
its stdout (the RA4M1's text) and stderr (the simulator's own "sim: t=<s> ..." lines), send it
stdin commands, and drive it with the PC relay's own code (relay-pc/dsentry: transports,
relay.Sender, protocol).

Times: every number the tests judge is VIRTUAL time (the RA4M1's millis() in its log lines and
in TelemetryPacket.uptimeMs, the sim's "t=" stamps and status fields), so the result does not
depend on how fast the Mac runs. Wall-clock waits are only timeouts: virtual seconds / speed,
plus slack.

The binary: $DSENTRY_SIM_BIN (tests/run.sh sets it), else it is built once with sim/build.sh
into $DSENTRY_TESTS_BUILD_DIR/acceptance-sim (default D-SentryCode/tests/build/acceptance-sim).
"""

import dataclasses
import math
import os
import re
import signal
import socket
import subprocess
import sys
import threading
import time
from pathlib import Path
from typing import Callable, List, Optional

HERE = Path(__file__).resolve().parent
DSC = HERE.parents[1]                 # D-SentryCode
REPO = DSC.parent
RELAY = DSC / "relay-pc"
TOOLS = DSC / "tools"
FORK = REPO / "firmware" / "UNOR4USBBridge"
PYTHON = sys.executable
if str(RELAY) not in sys.path:
    sys.path.insert(0, str(RELAY))

from dsentry import protocol as P     # noqa: E402
from dsentry import relay as R        # noqa: E402
from dsentry import settings as ST    # noqa: E402
from dsentry import transports as T   # noqa: E402

SET = ST.Settings.load(DSC / "config.h")


def cfg(name):
    value = SET.value(name)
    if value is None:
        raise KeyError("config.h has no %s" % name)
    return value


def sweep_ms(span_deg):
    """Trigger.h: with an update every ms, a sweep of span_deg lands at the first whole ms whose
    distance at TRIGGER_SWEEP_DEG_PER_S covers it; 0 without a sweep (rate 0)."""
    rate = cfg("TRIGGER_SWEEP_DEG_PER_S")
    if rate <= 0 or span_deg <= 0:
        return 0
    return math.ceil(span_deg * cfg("MS_PER_S") / rate)


# One shot at config.h's angles (Trigger.h), from PRE-FIRE with darts left: the sweep to FIRE,
# FIRE held TRIGGER_FORWARD_MS, the sweep to REST, REST held TRIGGER_RETURN_MS, [with
# TRIGGER_CLEAR_ENABLED: the sweep to CLEAR, held TRIGGER_CLEAR_HOLD_MS, the sweep to REST, held
# TRIGGER_RETURN_MS again] (the shot counts: COUNT_MS), the sweep to PRE-FIRE, held
# TRIGGER_CHAMBER_MS (the next shot available: STROKE_MS); semi auto then waits
# TRIGGER_COOLDOWN_MS before the next stroke may start. The last dart ends at REST (COUNT_MS).
# X (reload) sends the idle servo to PRE-FIRE and the next shot waits CHAMBER_S for it.
SWEEP_MS = sweep_ms(abs(cfg("TRIGGER_FIRE_ANGLE") - cfg("TRIGGER_PREFIRE_ANGLE")))
_CLEAR_MS = sweep_ms(abs(cfg("TRIGGER_CLEAR_ANGLE") - cfg("TRIGGER_REST_ANGLE")))
COUNT_MS = (SWEEP_MS + cfg("TRIGGER_FORWARD_MS") + sweep_ms(abs(cfg("TRIGGER_FIRE_ANGLE") - cfg("TRIGGER_REST_ANGLE")))
            + cfg("TRIGGER_RETURN_MS")
            + ((_CLEAR_MS + cfg("TRIGGER_CLEAR_HOLD_MS") + _CLEAR_MS + cfg("TRIGGER_RETURN_MS"))
               if cfg("TRIGGER_CLEAR_ENABLED") else 0))
STROKE_MS = (COUNT_MS + sweep_ms(abs(cfg("TRIGGER_REST_ANGLE") - cfg("TRIGGER_PREFIRE_ANGLE")))
             + cfg("TRIGGER_CHAMBER_MS"))
STROKE_S = STROKE_MS / cfg("MS_PER_S")
# Full auto while held (owner, 2026-11-04): every shot after the first starts from REST, so one
# shot per BURST_MS (REST -> FIRE -> REST, the count), no PRE-FIRE visit in between.
BURST_MS = COUNT_MS - SWEEP_MS + sweep_ms(abs(cfg("TRIGGER_FIRE_ANGLE") - cfg("TRIGGER_REST_ANGLE")))
COOLDOWN_S = cfg("TRIGGER_COOLDOWN_MS") / cfg("MS_PER_S")
CHAMBER_S = cfg("TRIGGER_CHAMBER_MS") / cfg("MS_PER_S")


# Wall-clock slack added to every timeout (process start, Python scheduling).
WALL_SLACK_S = 3.0
_BUILT = {}
_LOCK = threading.Lock()


def sim_binary() -> str:
    env = os.environ.get("DSENTRY_SIM_BIN")
    if env:
        if not os.access(env, os.X_OK):
            raise RuntimeError("DSENTRY_SIM_BIN=%s is not an executable" % env)
        return env
    with _LOCK:
        if "bin" not in _BUILT:
            out = Path(os.environ.get("DSENTRY_TESTS_BUILD_DIR", DSC / "tests" / "build")) / "acceptance-sim"
            r = subprocess.run(["bash", str(DSC / "sim" / "build.sh")], capture_output=True, text=True,
                               env=dict(os.environ, DSENTRY_SIM_BUILD_DIR=str(out)))
            if r.returncode != 0:
                raise RuntimeError("sim/build.sh failed:\n%s%s" % (r.stdout, r.stderr))
            _BUILT["bin"] = str(out / "dsentry_sim")
        return _BUILT["bin"]


def free_udp_port() -> int:
    return T.free_udp_port()


def free_tcp_port() -> int:
    with socket.socket() as s:
        s.bind(("127.0.0.1", 0))
        return s.getsockname()[1]


_STATUS_FIELD = re.compile(r'(\w+)=("[^"]*"|\S+)')


def parse_fields(text: str) -> dict:
    out = {}
    for key, value in _STATUS_FIELD.findall(text):
        out[key] = value[1:-1] if value.startswith('"') else value
    return out


class Lines:
    """A pipe read line by line on a thread; every line kept with its arrival index."""

    def __init__(self, stream) -> None:
        self.lines: List[str] = []
        self.cond = threading.Condition()
        self.closed = False
        self._thread = threading.Thread(target=self._run, args=(stream,), daemon=True)
        self._thread.start()

    def _run(self, stream) -> None:
        for line in iter(stream.readline, b""):
            with self.cond:
                self.lines.append(line.decode("latin-1").rstrip("\r\n"))
                self.cond.notify_all()
        with self.cond:
            self.closed = True
            self.cond.notify_all()

    def mark(self) -> int:
        with self.cond:
            return len(self.lines)

    def wait(self, pattern, timeout: float, after: int = 0):
        """The first line from index `after` on that matches; AssertionError on timeout."""
        rx = re.compile(pattern)
        end = time.monotonic() + timeout
        with self.cond:
            i = after
            while True:
                while i < len(self.lines):
                    m = rx.search(self.lines[i])
                    if m:
                        return m, i
                    i += 1
                left = end - time.monotonic()
                if left <= 0 or self.closed:
                    tail = "\n".join(self.lines[-15:])
                    raise AssertionError("no line matching %r within %.1f s; last lines:\n%s" % (pattern, timeout, tail))
                self.cond.wait(min(left, 0.05))

    def all(self, pattern, after: int = 0):
        rx = re.compile(pattern)
        with self.cond:
            return [m for m in (rx.search(l) for l in self.lines[after:]) if m]

    def join(self, timeout: float) -> None:
        self._thread.join(timeout)


class Sim:
    """One dsentry_sim process. args are extra command-line options."""

    def __init__(self, *args, speed: float = 1.0, udp_port: Optional[int] = None) -> None:
        self.speed = speed
        self.udp_port = free_udp_port() if udp_port is None else udp_port
        self.argv = [sim_binary(), "--udp-port", str(self.udp_port), "--speed", repr(speed), *args]
        self.proc = subprocess.Popen(self.argv, stdin=subprocess.PIPE, stdout=subprocess.PIPE,
                                     stderr=subprocess.PIPE, bufsize=0)
        self.out = Lines(self.proc.stdout)
        self.err = Lines(self.proc.stderr)
        self.pty_path: Optional[str] = None
        if "--pty" in args or "--pty-link" in args:
            m, _ = self.err.wait(r"sim: t=\S+ pty (\S+) ", WALL_SLACK_S)
            self.pty_path = m.group(1)

    # -- time -------------------------------------------------------------------------------
    def wall(self, virtual_s: float) -> float:
        """A wall-clock timeout for something due within virtual_s of virtual time."""
        return virtual_s / self.speed + WALL_SLACK_S

    def sleep(self, virtual_s: float) -> None:
        time.sleep(virtual_s / self.speed)

    # -- stdin / stderr ---------------------------------------------------------------------
    def send(self, line: str) -> None:
        self.proc.stdin.write((line + "\n").encode())
        self.proc.stdin.flush()

    def command(self, line: str, reply_pattern: str, timeout: float = WALL_SLACK_S):
        mark = self.err.mark()
        self.send(line)
        m, _ = self.err.wait(r"^sim: t=\S+ " + reply_pattern, timeout, mark)
        return m

    def status(self) -> dict:
        m = self.command("status", r"status (.*)$")
        line = m.group(0)
        d = parse_fields(m.group(1))
        d["t"] = float(re.search(r"t=(\S+)", line).group(1))
        return d

    # -- stdout: the RA4M1's log lines "<millis> <text>" -------------------------------------
    def wait_log(self, pattern: str, virtual_s: float, after: int = 0):
        """(millis, match, index) of the first RA4M1 line matching `pattern` from `after` on."""
        m, i = self.out.wait(r"^(\d+) .*?" + pattern, self.wall(virtual_s), after)
        return int(m.group(1)), m, i

    def states(self, after: int = 0):
        """[(millis, state name)] from the RA4M1's 'state X' lines."""
        return [(int(m.group(1)), m.group(2)) for m in self.out.all(r"^(\d+) state (\w+)", after)]

    # -- end --------------------------------------------------------------------------------
    def stop(self, sig=signal.SIGTERM, timeout: float = 10.0):
        if self.proc.poll() is None:
            self.proc.send_signal(sig)
        try:
            self.proc.wait(timeout)
        except subprocess.TimeoutExpired:
            self.proc.kill()
            self.proc.wait()
        try:
            self.proc.stdin.close()
        except OSError:
            pass
        self.out.join(2)
        self.err.join(2)
        for pipe in (self.proc.stdout, self.proc.stderr):
            pipe.close()
        return self.proc.returncode

    def __enter__(self):
        return self

    def __exit__(self, *exc):
        self.stop()


class Collector:
    """The sink every relay transport delivers to: frames and text, in arrival order."""

    def __init__(self) -> None:
        self.cond = threading.Condition()
        self.telemetry: List[P.TelemetryPacket] = []
        self.status: List[P.StatusPacket] = []
        self.other: List[P.Frame] = []
        self.text: List[str] = []
        self.notes: List[str] = []

    def on_frame(self, frame, via: str = "") -> None:
        with self.cond:
            if frame.type == P.PacketType.TELEMETRY:
                self.telemetry.append(frame.packet)
            elif frame.type == P.PacketType.STATUS:
                self.status.append(frame.packet)
            else:
                self.other.append(frame)
            self.cond.notify_all()

    def on_text(self, text: str, via: str = "") -> None:
        with self.cond:
            self.text.append(text)
            self.cond.notify_all()

    def note(self, kind: str, text: str) -> None:
        with self.cond:
            self.notes.append("%s: %s" % (kind, text))

    def mark(self) -> int:
        with self.cond:
            return len(self.telemetry)

    def latest(self) -> Optional[P.TelemetryPacket]:
        with self.cond:
            return self.telemetry[-1] if self.telemetry else None

    def since(self, mark: int) -> List[P.TelemetryPacket]:
        with self.cond:
            return list(self.telemetry[mark:])

    def wait(self, pred: Callable[[P.TelemetryPacket], bool], timeout: float, after: int = 0,
             what: str = "telemetry") -> P.TelemetryPacket:
        end = time.monotonic() + timeout
        with self.cond:
            i = after
            while True:
                while i < len(self.telemetry):
                    t = self.telemetry[i]
                    if t.version == P.DS_TELEMETRY_VERSION and pred(t):
                        return t
                    i += 1
                left = end - time.monotonic()
                if left <= 0:
                    last = self.telemetry[-1] if self.telemetry else None
                    raise AssertionError("%s not seen within %.1f s; last: %s" % (what, timeout, describe(last)))
                self.cond.wait(min(left, 0.05))


def describe(t: Optional[P.TelemetryPacket]) -> str:
    if t is None:
        return "none"
    return ("%s fault=%d up=%d ms ammo=%d shots=%d duty=%d pan=%d tilt=%d vel=%d/%d src=%d flags=0x%04x "
            "led=%d,%d,%d lcd=%r" % (P.TurretState(t.state).name, t.fault, t.uptimeMs, t.ammo, t.shotsTotal,
                                    t.flywheelDuty, t.panCdeg, t.tiltCdeg, t.panVelCdegS, t.tiltVelCdegS,
                                    t.linkSource, t.flags, t.ledR, t.ledG, t.ledB, t.lcd))


def rows(t: P.TelemetryPacket):
    cols = cfg("LCD_COLS")
    return t.lcd[:cols], t.lcd[cols:2 * cols]


def lcd_row(left: str, right: str = "") -> str:
    """Spec 3.9 / architecture 5 Lcd: left text, right text right-justified, >= 1 space between."""
    cols = cfg("LCD_COLS")
    room = cols - len(left) - (1 if left and right else 0)
    right = right[:max(0, room)]
    return left + " " * (cols - len(left) - len(right)) + right


def scaled(color: int):
    """LED 0 as sent for a config colour: every channel through the brightness clamp."""
    limit, full = cfg("LED_MAX_BRIGHTNESS"), cfg("LED_CHANNEL_MAX")
    return tuple(((color >> s) & 0xFF) * limit // full for s in (16, 8, 0))


def flag(t: P.TelemetryPacket, bit: int) -> bool:
    return bool(t.flags & bit)


def state_is(*names):
    want = {getattr(P, "DS_STATE_" + n) for n in names}
    return lambda t: t.state in want


class Driver:
    """The relay's side of WiFi mode, built from the relay's own parts: UdpTransport (socket,
    reader thread, LinkReader) and relay.Sender (packet_for / flags_for, seq), stepped at
    CONTROL_PACKET_HZ of VIRTUAL time (wall rate = CONTROL_PACKET_HZ * speed).

    pause() stops sending without a farewell packet (a cable pulled / relay frozen);
    stop_clean() is what the relay does on exit (Sender.send_final: one connected=0 packet)."""

    def __init__(self, sim: Sim, target=None, discovery_port: Optional[int] = None) -> None:
        self.sim = sim
        self.settings = dataclasses.replace(SET, control_hz=SET.control_hz * sim.speed)
        self.sink = Collector()
        if target is None and discovery_port is None:
            target = (T.LOOPBACK, sim.udp_port)
        if target is not None:
            self.udp = T.UdpTransport(self.settings, self.sink, target=target, bind_host=T.LOOPBACK,
                                      discovery=False, ap_fallback=False)
        else:
            self.udp = T.UdpTransport(self.settings, self.sink, bind_host=T.LOOPBACK, discovery=True,
                                      discovery_port=discovery_port, ap_fallback=False)
        self._lock = threading.Lock()
        self.connected = False
        self.rx = self.ry = self.rt = self.buttons = 0
        self.sender = R.Sender(self.settings, self.udp, self._build)
        self._stop = threading.Event()
        self._pause = threading.Event()
        self._final = threading.Event()
        self._final_sent = threading.Event()
        self.udp.start(self._stop)
        self._thread = threading.Thread(target=self._run, daemon=True, name="driver")
        self._thread.start()

    def _build(self, now) -> R.InputSnapshot:
        with self._lock:
            return R.InputSnapshot("pad" if self.connected else None, self.connected, False,
                                   self.rx, self.ry, self.rt, self.buttons, 0.0)

    def _run(self) -> None:
        period = self.settings.control_period_s
        due = time.monotonic()
        while not self._stop.is_set():
            if self._final.is_set() and not self._pause.is_set():
                self._pause.set()                 # in the sending thread: nothing slips in after it
                self.sender.send_final()
                self._final_sent.set()
            if not self._pause.is_set():
                self.sender.step()
            due += period
            now = time.monotonic()
            if now - due > period:
                due = now
            self._stop.wait(max(0.0, due - now))

    # -- input -------------------------------------------------------------------------------
    def set(self, **kw) -> None:
        with self._lock:
            for k, v in kw.items():
                if not hasattr(self, k):
                    raise AttributeError(k)
                setattr(self, k, v)

    def hold(self, button: int, on: bool = True) -> None:
        with self._lock:
            self.buttons = (self.buttons | button) if on else (self.buttons & ~button)

    def tap(self, button: int, virtual_ms: Optional[float] = None) -> None:
        """Press for KEYBOARD_PRESS_MS (the relay's shortest press), then release for as long."""
        ms = cfg("KEYBOARD_PRESS_MS") if virtual_ms is None else virtual_ms
        self.hold(button, True)
        self.sim.sleep(ms / 1000.0)
        self.hold(button, False)
        self.sim.sleep(ms / 1000.0)

    def pause(self) -> None:
        self._pause.set()

    def resume(self) -> None:
        self._pause.clear()

    def stop_clean(self) -> None:
        """The relay's exit: the next packet slot carries Sender.send_final's connected=0, then silence."""
        self._pause.clear()
        self._final.set()
        if not self._final_sent.wait(2.0):
            raise AssertionError("the driver did not send its final packet")

    # -- what came back ------------------------------------------------------------------------
    def wait(self, pred, virtual_s: float, after: int = 0, what: str = "telemetry"):
        return self.sink.wait(pred, self.sim.wall(virtual_s), after, what)

    def close(self) -> None:
        self._stop.set()
        self._thread.join(2)
        self.udp.join(2)
        self.udp.close()


def boot_to_pairing(test, sim: Sim, drv: Optional[Driver] = None):
    """Waits for PAIRING after the calibration (CAL_RAM drives TILT_CAL_MAX_TRAVEL_DEG at
    TILT_CAL_SPEED_DEG_PER_S, settles, backs off). Returns the PAIRING log time in ms."""
    cal_s = cfg("TILT_CAL_MAX_TRAVEL_DEG") / cfg("TILT_CAL_SPEED_DEG_PER_S") + cfg("TILT_CAL_SETTLE_MS") / 1000.0
    ms, _, _ = sim.wait_log(r"state PAIRING", cal_s + 5.0)
    if drv is not None:
        drv.wait(state_is("PAIRING"), 2.0, what="PAIRING telemetry")
    return ms

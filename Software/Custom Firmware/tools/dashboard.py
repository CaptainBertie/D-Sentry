#!/usr/bin/env python3
# dashboard.py - live web dashboard for the Nerf blaster turret's Xbox controller.
# It reads the XR4 CSV lines that the TurretTelemetry sketch prints over USB serial, works out
# the turret logic (aim direction, fire mode, armed, flywheels, firing) and serves one page at
# http://127.0.0.1:8765/ that updates through server-sent events (/events).
#
# Run it with the project venv (stdlib + pyserial; pyserial is only needed for a real board):
#   "<repo>/.venv/bin/python" "<repo>/tools/dashboard.py"           # auto-detect the UNO R4 WiFi
#   "<repo>/.venv/bin/python" "<repo>/tools/dashboard.py" --demo    # synthetic controller
# Only one program can hold the serial port: stop the dashboard before uploading a sketch.
# See tools/DASHBOARD.md.
#
# SPDX-License-Identifier: MIT

import argparse
import collections
import json
import math
import re
import secrets
import signal
import sys
import threading
import time
import webbrowser
from datetime import datetime
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path

PAGE_PATH = Path(__file__).resolve().parent / "dashboard" / "index.html"

UNO_R4_WIFI_VID = 0x2341
UNO_R4_WIFI_PID = 0x1002

DEADZONE = 0.15          # right-stick radius that still counts as "centre"
FIRE_THRESHOLD = 0.10    # right trigger must pass 10 % so a resting trigger never fires
SPINUP_S = 1.0           # flywheels must have been spinning this long before a dart can fire
FULL_AUTO_RPS = 3.0      # darts per second while the trigger is held in full auto
LINK_TIMEOUT_S = 0.3     # no valid XR4 line for this long = link lost (sketch heartbeat: 100 ms)
RATE_WINDOW_S = 1.0      # reports per second are counted over this sliding window
MAX_FRAMES_PER_S = 50    # /events never sends faster than this per client
KEEPALIVE_S = 1.0        # an idle /events stream gets a "ping" event this often, so the page
                         # can tell a quiet board from a frozen or unreachable server
TICK_S = 0.05            # how often link loss and the rate are re-checked without new lines
LOG_KEEP = 50            # event log entries kept for the page
MAX_LINE_BYTES = 4096    # a "line" longer than this without a newline is garbage; drop it
RETRY_S = 1.0            # serial port reopen interval

AXIS_CENTRE = 32767.5    # midpoint of 0..65535, so both ends map to exactly -1 and +1
TRIGGER_MAX = 1023
FIELD_COUNT = 13

STATE_WORDS = ("connected", "waiting", "connecting", "scanning", "off", "blocked")
BUTTON_NAMES = ("a", "b", "x", "y", "lb", "rb", "view", "menu", "xbox", "ls", "rs", "share")
DPAD_WORDS = ("none", "up", "up-right", "right", "down-right", "down", "down-left", "left",
              "up-left")

# (lowest, highest) for fields 1..10 of an XR4 line, in order
DECIMAL_RANGES = (
    (0, 0xFFFFFFFF),  # millis
    (0, 5),           # state
    (0, 255),         # battery (0..100 or 255, checked below)
    (0, 255),         # seq
    (0, 65535), (0, 65535), (0, 65535), (0, 65535),  # lx ly rx ry
    (0, TRIGGER_MAX), (0, TRIGGER_MAX),               # lt rt
)
BATTERY_UNKNOWN = 255
DECIMAL_RE = re.compile(r"[0-9]{1,10}\Z")   # ASCII digits only: int() would accept " 7" or "٣"
HEX4_RE = re.compile(r"[0-9a-fA-F]{4}\Z")

Report = collections.namedtuple(
    "Report", "millis state battery seq lx ly rx ry lt rt buttons dpad")


class PortError(Exception):
    """No usable serial port; the message says what to do."""


# ---------------------------------------------------------------------------
# Parsing and conversions (pure functions)
# ---------------------------------------------------------------------------

def parse_xr4(line):
    """Returns a Report for a well-formed XR4 line, None for anything else. Never raises."""
    if not isinstance(line, str):
        return None
    fields = line.strip().split(",")
    if len(fields) != FIELD_COUNT or fields[0] != "XR4":
        return None
    values = []
    for text, (lowest, highest) in zip(fields[1:11], DECIMAL_RANGES):
        if not DECIMAL_RE.match(text):
            return None
        value = int(text)
        if not lowest <= value <= highest:
            return None
        values.append(value)
    battery = values[2]
    if battery > 100 and battery != BATTERY_UNKNOWN:
        return None
    if not HEX4_RE.match(fields[11]):
        return None
    buttons = int(fields[11], 16)
    if buttons >> len(BUTTON_NAMES):
        return None  # bits above Share are never set by the sketch: the line is corrupt
    if not DECIMAL_RE.match(fields[12]) or int(fields[12]) >= len(DPAD_WORDS):
        return None
    return Report(*values, buttons, int(fields[12]))


def axis(raw, invert=False):
    """Raw stick 0..65535 to -1..+1. invert=True for vertical axes, whose raw 0 is up."""
    value = max(-1.0, min(1.0, (raw - AXIS_CENTRE) / AXIS_CENTRE))
    return -value if invert else value


def trigger(raw):
    """Raw trigger 0..1023 to 0..1."""
    return max(0.0, min(1.0, raw / TRIGGER_MAX))


def dpad_word(code):
    return DPAD_WORDS[code] if 0 <= code < len(DPAD_WORDS) else "none"


def state_word(state):
    if state is None:
        return "no data"
    return STATE_WORDS[state] if 0 <= state < len(STATE_WORDS) else "unknown"


def button_dict(mask):
    return {name: bool(mask >> bit & 1) for bit, name in enumerate(BUTTON_NAMES)}


def aim_direction(x, y, deadzone=DEADZONE):
    """(word, horizontal, vertical) for a stick at x, y (up positive), e.g. ("up-left", "left",
    "up"). Inside the deadzone circle it is ("centre", "", ""). Outside it each axis is named
    on its own, because pan and tilt are separate motions: a stick pushed right and 20 % up
    reads "up-right", so a live tilt command is never hidden behind "right"."""
    if math.hypot(x, y) < deadzone:
        return "centre", "", ""
    horizontal = _axis_word(x, y, deadzone, "right", "left")
    vertical = _axis_word(y, x, deadzone, "up", "down")
    word = "-".join(part for part in (vertical, horizontal) if part)
    return word, horizontal, vertical


def _axis_word(value, other, deadzone, positive, negative):
    """One axis's word outside the centre circle. The axis counts once it passes the deadzone
    by itself, or when it is the larger of the two, so a stick just outside the circle always
    names a direction even when neither axis alone reaches the deadzone."""
    if abs(value) < deadzone and abs(value) < abs(other):
        return ""
    return positive if value > 0 else negative


def tidy(value):
    """Rounded for JSON; adding 0.0 turns the -0.0 of an inverted centred axis into 0.0."""
    return round(value, 4) + 0.0


def clean_text(text, limit=300):
    """Printable copy of a status line: control characters become spaces, length capped."""
    return "".join(ch if ch.isprintable() else " " for ch in text).strip()[:limit]


# ---------------------------------------------------------------------------
# Turret logic
# ---------------------------------------------------------------------------

class TurretState:
    """The turret's switches and the firing cycle, driven by controller samples and a clock.
    No I/O, so it is unit-testable.

    A toggles semi / full auto and X toggles armed, each on the press (rising edge) only, so
    holding a button flips once. LB spins the flywheels while held; they are "ready" once they
    have been spinning for spinup_s. A dart fires only when armed, the wheels are ready and the
    right trigger is past the threshold: in semi, one dart per pull (release below the
    threshold to fire again), never faster than full_auto_rps, a pull during the cooldown is
    queued; in full auto, one dart every 1/full_auto_rps while held.
    Whenever the controller is not connected or the link is lost, everything drops to the
    safe state: semi, disarmed, flywheels off."""

    def __init__(self, fire_threshold=FIRE_THRESHOLD, spinup_s=SPINUP_S,
                 full_auto_rps=FULL_AUTO_RPS, clock=time.monotonic):
        self.fire_threshold = fire_threshold
        self.spinup_s = spinup_s
        self.shot_interval = 1.0 / full_auto_rps
        self._clock = clock
        self.fire_mode = "semi"
        self.armed = False
        self.flywheels = False          # LB held
        self.flywheels_ready = False    # LB held for spinup_s or longer
        self.flywheels_since = None     # clock reading when LB went down
        self.trigger_pulled = False     # right trigger past the threshold
        self.firing = False             # armed + ready + trigger pulled: darts are leaving
        self.shots = 0                  # darts fired since start-up
        self.last_shot = None           # clock reading of the last dart
        # True means "seen held": a button already down at start-up, or held through a
        # dropout, must be released and pressed again before it counts.
        self._a_was_down = True
        self._x_was_down = True
        self._semi_shot_done = False    # semi: one dart per pull
        self._semi_pending = False      # semi: a pull is waiting for the cooldown to end
        self._next_auto_shot = None     # full auto: when the next dart is due
        self._burst_shots = 0
        self._lb = False
        self._rt = 0.0
        self._live = False

    def cooldown_left(self, now):
        """Seconds until the pusher can cycle again (0 when it can)."""
        if self.last_shot is None:
            return 0.0
        return max(0.0, self.last_shot + self.shot_interval - now)

    def update(self, a, x, lb, rt, live, reason="", now=None):
        """Applies one controller sample. live = controller connected and link ok.
        Returns a list of (kind, text) events for the log."""
        now = self._clock() if now is None else now
        if not live:
            return self._make_safe(reason)
        events = []
        if a and not self._a_was_down:
            old = self.fire_mode
            self.fire_mode = "full" if old == "semi" else "semi"
            events.append(("turret", "A: fire mode %s -> %s" % (mode_label(old),
                                                                 mode_label(self.fire_mode))))
        if x and not self._x_was_down:
            self.armed = not self.armed
            events.append(("turret", "X: ARMED" if self.armed else "X: disarmed (SAFE)"))
        self._a_was_down = bool(a)
        self._x_was_down = bool(x)
        self._lb, self._rt, self._live = bool(lb), float(rt), True
        return events + self._evaluate(now)

    def tick(self, now=None):
        """Advances the clock-driven parts (spin-up, full-auto cadence) when no new sample
        arrives: an idle controller sends no reports while LB or the trigger is simply held."""
        now = self._clock() if now is None else now
        if not self._live:
            return []
        return self._evaluate(now)

    def spinup(self, now=None):
        """Seconds the flywheels have been spinning, capped at spinup_s."""
        if self.flywheels_since is None:
            return 0.0
        now = self._clock() if now is None else now
        return min(self.spinup_s, max(0.0, now - self.flywheels_since))

    def _evaluate(self, now):
        events = []
        if self._lb:
            if self.flywheels_since is None:
                self.flywheels_since = now
            self.flywheels = True
            ready = now - self.flywheels_since >= self.spinup_s
        else:
            self.flywheels = False
            self.flywheels_since = None
            ready = False
        if ready and not self.flywheels_ready:
            events.append(("turret", "flywheels up to speed"))
        self.flywheels_ready = ready

        self.trigger_pulled = self._rt > self.fire_threshold
        if not self.trigger_pulled:
            if self.firing and self.fire_mode == "full" and self._burst_shots:
                events.append(("turret", "burst ended: %d darts" % self._burst_shots))
            self._semi_shot_done = False
            self._next_auto_shot = None
            self._burst_shots = 0
        ready_to_fire = self.armed and self.flywheels_ready
        if self.fire_mode == "semi":
            # A pull registers one dart request; it is served when the pusher's cooldown
            # (1/full_auto_rps) has passed, even if the trigger was released by then. That caps
            # semi at the same rate as full auto without losing a quick tap.
            if self.trigger_pulled and not self._semi_shot_done:
                self._semi_shot_done = True
                self._semi_pending = True
            if not self.armed:
                self._semi_pending = False       # disarmed: the request is dropped
            if not self.flywheels_ready and not self.trigger_pulled:
                self._semi_pending = False       # released before the wheels were ready: no dart
            self.firing = ready_to_fire and (self.trigger_pulled or self._semi_pending)
            if self._semi_pending and ready_to_fire and self.cooldown_left(now) == 0.0:
                self._semi_pending = False
                self._shot(now)
                events.append(("turret", "FIRE: one dart (semi), dart #%d" % self.shots))
            return events
        self._semi_pending = False
        self.firing = ready_to_fire and self.trigger_pulled
        if not self.firing:
            return events
        if self._next_auto_shot is None or now >= self._next_auto_shot:
            self._shot(now)
            self._burst_shots += 1
            self._next_auto_shot = now + self.shot_interval   # no catch-up after a late tick
            if self._burst_shots == 1:
                events.append(("turret", "FIRE: full auto, %d darts/s" % round(1 / self.shot_interval)))
        return events

    def _shot(self, now):
        self.shots += 1
        self.last_shot = now

    def _make_safe(self, reason):
        was_unsafe = self.armed or self.fire_mode != "semi" or self.flywheels or self.firing
        self.fire_mode = "semi"
        self.armed = False
        self.flywheels = False
        self.flywheels_ready = False
        self.flywheels_since = None
        self.trigger_pulled = False
        self.firing = False
        self._a_was_down = True
        self._x_was_down = True
        self._semi_shot_done = False
        self._semi_pending = False
        self._next_auto_shot = None
        self._burst_shots = 0
        self._lb, self._rt, self._live = False, 0.0, False
        if not was_unsafe:
            return []
        why = " (%s)" % reason if reason else ""
        return [("safety", "safety reset: SAFE, semi, flywheels off" + why)]


def mode_label(mode):
    return "FULL AUTO" if mode == "full" else "SEMI"


# ---------------------------------------------------------------------------
# Shared latest state
# ---------------------------------------------------------------------------

class Hub:
    """Thread-safe latest state. Reader threads feed lines in; HTTP threads read frames out.
    `version` is the event counter: it rises on every change a page should see."""

    def __init__(self, source, demo=False, clock=time.monotonic, wall=time.time):
        self._cond = threading.Condition()
        self._clock = clock
        self._wall = wall
        self.source = source
        self.demo = demo
        self.boot = secrets.token_hex(4)   # lets a page notice that the server restarted
        self.version = 0
        self.lines = 0           # valid XR4 lines
        self.bad_lines = 0       # lines that started with XR4 but failed parsing
        self.turret = TurretState(clock=clock)
        self._report = None
        self._last_good = None   # clock() of the last valid XR4 line
        self._arrivals = collections.deque()
        self._link_ok = False
        self._last_rate = 0
        self._log = collections.deque(maxlen=LOG_KEEP)
        self._next_log_id = 1

    # -- input ------------------------------------------------------------

    def feed_line(self, text, now=None):
        """One line from the board (or the demo). XR4 lines update the state, '#' lines go
        to the event log, anything else (boot noise, half lines) is ignored."""
        now = self._clock() if now is None else now
        text = text.strip()
        if text.startswith("XR4"):
            self._feed_report(parse_xr4(text), now)
        elif text.startswith("#"):
            with self._cond:
                self._add_log("board", clean_text(text))
                self._changed()

    def note(self, text):
        """A message from the dashboard itself (port opened, port lost)."""
        with self._cond:
            self._add_log("dashboard", clean_text(text))
            self._changed()

    def tick(self, now=None):
        """Re-checks link loss and the rate when no lines arrive. Called every TICK_S."""
        now = self._clock() if now is None else now
        with self._cond:
            changed = self._check_link(now)
            changed |= self._tick_turret(now)
            rate = self._rate(now)
            if rate != self._last_rate:
                self._last_rate = rate
                changed = True
            if changed:
                self._changed()

    def _tick_turret(self, now):
        """Spin-up time and full-auto cadence advance with the clock, not only with reports
        (a held button sends no new report). Returns True when something visible changed."""
        turret = self.turret
        before = (turret.shots, turret.flywheels_ready, turret.firing)
        for kind, text in turret.tick(now):
            self._add_log(kind, text)
        return before != (turret.shots, turret.flywheels_ready, turret.firing)

    def _check_link(self, now):
        """Notices link loss and forces the turret safe. Returns True when something changed.
        frame() runs it too, so no frame can say link lost and armed at the same time."""
        changed = False
        link_ok = self._link_alive(now)
        if link_ok != self._link_ok:
            self._link_ok = link_ok
            changed = True
            if not link_ok:
                self._add_log("dashboard", "link lost: no XR4 line for %d ms"
                              % round(LINK_TIMEOUT_S * 1000))
        if not link_ok:
            changed |= self._apply_turret(None, live=False, reason="link lost", now=now)
        return changed

    def _feed_report(self, report, now):
        with self._cond:
            if report is None:
                self.bad_lines += 1
                return
            self.lines += 1
            self._report = report
            self._last_good = now
            self._arrivals.append(now)
            if not self._link_ok:
                self._link_ok = True
                self._add_log("dashboard", "link ok: XR4 lines arriving")
            live = report.state == 0
            self._apply_turret(report, live, reason="controller " + state_word(report.state), now=now)
            self._last_rate = self._rate(now)
            self._changed()

    def _apply_turret(self, report, live, reason, now=None):
        """Runs the turret logic; returns True when it logged an event."""
        if report is None:
            events = self.turret.update(False, False, False, 0.0, live=False, reason=reason, now=now)
        else:
            pressed = button_dict(report.buttons)
            events = self.turret.update(pressed["a"], pressed["x"], pressed["lb"],
                                        trigger(report.rt), live=live, reason=reason, now=now)
        for kind, text in events:
            self._add_log(kind, text)
        return bool(events)

    def _link_alive(self, now):
        return self._last_good is not None and now - self._last_good <= LINK_TIMEOUT_S

    def _rate(self, now):
        while self._arrivals and self._arrivals[0] <= now - RATE_WINDOW_S:
            self._arrivals.popleft()
        return len(self._arrivals)

    def _add_log(self, kind, text):
        self._log.append({"id": self._next_log_id, "t": round(self._wall(), 3),
                          "kind": kind, "text": text})
        self._next_log_id += 1

    def _changed(self):
        self.version += 1
        self._cond.notify_all()

    # -- output -----------------------------------------------------------

    def wait_for_change(self, after_version, timeout):
        """Blocks until version differs from after_version or timeout; returns version."""
        with self._cond:
            self._cond.wait_for(lambda: self.version != after_version, timeout)
            return self.version

    def frame(self, now=None, log_after=0):
        """Everything the page shows, as a JSON-ready dict. log holds entries with
        id > log_after, so a stream sends each log line once."""
        now = self._clock() if now is None else now
        with self._cond:
            if self._check_link(now):
                self._changed()
            return self._build_frame(now, log_after)

    def _build_frame(self, now, log_after):
        report = self._report or Report(0, None, BATTERY_UNKNOWN, 0, 32768, 32768, 32768,
                                        32768, 0, 0, 0, 0)
        have_data = self._report is not None
        lx, ly = axis(report.lx), axis(report.ly, invert=True)
        rx, ry = axis(report.rx), axis(report.ry, invert=True)
        aim, aim_x, aim_y = aim_direction(rx, ry)
        turret = self.turret
        return {
            "version": self.version,
            "boot": self.boot,
            "t": round(self._wall(), 3),
            "source": self.source,
            "demo": self.demo,
            "have_data": have_data,
            "link_ok": self._link_alive(now),
            "link_age": None if self._last_good is None else round(now - self._last_good, 3),
            "state": report.state if have_data else None,
            "state_word": state_word(report.state if have_data else None),
            "battery": (None if not have_data or report.battery == BATTERY_UNKNOWN
                        else report.battery),
            "seq": report.seq if have_data else None,
            "board_ms": report.millis if have_data else None,
            "rate": self._rate(now),
            "lx": tidy(lx), "ly": tidy(ly), "rx": tidy(rx), "ry": tidy(ry),
            "lt": tidy(trigger(report.lt)), "rt": tidy(trigger(report.rt)),
            "buttons": button_dict(report.buttons),
            "buttons_hex": "%04x" % report.buttons,
            "dpad": dpad_word(report.dpad),
            "aim": aim, "aim_x": aim_x, "aim_y": aim_y,
            "deadzone": DEADZONE,
            "fire_threshold": FIRE_THRESHOLD,
            "fire_mode": turret.fire_mode,
            "armed": turret.armed,
            "flywheels": turret.flywheels,
            "flywheels_ready": turret.flywheels_ready,
            "spinup": tidy(turret.spinup(now)),
            "spinup_s": turret.spinup_s,
            "trigger_pulled": turret.trigger_pulled,
            "firing": turret.firing,
            "shots": turret.shots,
            "shot_interval": tidy(turret.shot_interval),
            "cooldown_left": tidy(turret.cooldown_left(now)),
            "last_shot_age": (None if turret.last_shot is None else tidy(max(0.0, now - turret.last_shot))),
            "lines": self.lines,
            "bad_lines": self.bad_lines,
            "log": [dict(entry) for entry in self._log if entry["id"] > log_after],
        }


# ---------------------------------------------------------------------------
# Line sources: serial port and demo
# ---------------------------------------------------------------------------

class LineSplitter:
    """Bytes in, text lines out. Undecodable bytes become U+FFFD instead of raising. A line
    longer than MAX_LINE_BYTES is garbage and is dropped whole, up to its newline, so its
    tail can never come out looking like a line of its own."""

    def __init__(self, max_bytes=MAX_LINE_BYTES):
        self._buffer = bytearray()
        self._max = max_bytes
        self._discarding = False   # inside an over-long line: drop bytes up to its newline

    def feed(self, data):
        self._buffer += data
        lines = []
        while True:
            end = self._buffer.find(b"\n")
            if end < 0:
                break
            raw = bytes(self._buffer[:end])
            del self._buffer[:end + 1]
            if self._discarding or len(raw) > self._max:
                self._discarding = False
                continue
            lines.append(raw.decode("utf-8", errors="replace"))
        if len(self._buffer) > self._max:
            self._buffer.clear()
            self._discarding = True
        return lines


class LineLog:
    """--log FILE: appends every XR4 and '#' line with a host timestamp. Off when path is None."""

    def __init__(self, path):
        self._file = open(path, "a", encoding="utf-8", buffering=1) if path else None
        self._lock = threading.Lock()

    def write(self, line):
        text = line.strip()
        if self._file is None or not (text.startswith("XR4") or text.startswith("#")):
            return
        stamp = datetime.now().astimezone().isoformat(timespec="milliseconds")
        with self._lock:
            self._file.write("%s,%s\n" % (stamp, text))

    def close(self):
        if self._file is not None:
            self._file.close()


def detect_port(ports=None):
    """The one serial port with the UNO R4 WiFi's VID:PID. Listing ports never opens them."""
    if ports is None:
        from serial.tools import list_ports
        ports = list_ports.comports()
    matches = sorted(p.device for p in ports
                     if p.vid == UNO_R4_WIFI_VID and p.pid == UNO_R4_WIFI_PID)
    if not matches:
        raise PortError("no UNO R4 WiFi found (USB 2341:1002). Plug it in, pass --port, "
                        "or preview with --demo.")
    if len(matches) > 1:
        raise PortError("several UNO R4 WiFi boards found (%s); choose one with --port."
                        % ", ".join(matches))
    return matches[0]


def open_serial(name, baud):
    """Opens the board's port. pyserial is imported here so --demo never needs it."""
    import serial
    return serial.Serial(name, baud, timeout=0.1)


def serial_reader(hub, port, baud, auto, stop, line_log, open_port=None):
    """Reads lines from the board until stop is set. If the port vanishes or cannot be
    opened, it tries again every RETRY_S; with auto=True it re-detects the board each time,
    because a replugged board can come back under another name on some systems."""
    open_port = open_port or (lambda name: open_serial(name, baud))
    last_problem = None
    while not stop.is_set():
        try:
            name = detect_port() if auto else port
            with open_port(name) as link:
                hub.note("serial port %s open at %d baud" % (name, baud))
                last_problem = None
                _read_until_error(link, hub, stop, line_log)
        except PortError as error:
            problem = str(error)
        except Exception as error:  # noqa: BLE001 (deliberately broad, see below)
            # SerialException, OSError, and also termios.error, which pyserial lets out when a
            # USB device is half re-enumerated and which is not an OSError. If this thread died,
            # the page would show LINK LOST until the dashboard was restarted.
            problem = "serial port problem: %s: %s; retrying every %g s" % (
                type(error).__name__, error, RETRY_S)
        else:
            problem = None
        if problem and problem != last_problem:
            hub.note(problem)   # once per distinct problem, not once a second
            last_problem = problem
        stop.wait(RETRY_S)


def _read_until_error(link, hub, stop, line_log):
    splitter = LineSplitter()
    while not stop.is_set():
        data = link.read(link.in_waiting or 1)
        for line in splitter.feed(data):
            line_log.write(line)
            hub.feed_line(line)


class DemoController:
    """A synthetic controller for previewing the page without hardware. Time t is seconds
    since the demo started. Timeline: scanning, then connected at 2 s; the right stick circles
    slowly and pauses in each quadrant; RT pulses every 3 s; A is pressed every 4 s, X every
    9 s; LB is held 3 s of every 6 s. Every 40 s the link drops for 3 s, alternating a
    controller disconnect (state scanning) and a silent board (no lines at all)."""

    CONNECT_AT = 2.0
    DROP_EVERY = 40.0
    DROP_FOR = 3.0
    STICK_RADIUS = 0.8
    QUADRANT_S = 2.0      # per quarter turn: move MOVE_S, then hold still
    MOVE_S = 1.2

    def __init__(self):
        self._seq = 0
        self._last_phase = None

    def phase(self, t):
        """'scanning', 'connecting', 'waiting', 'connected' or 'silent'."""
        if t < self.CONNECT_AT:
            return self._reconnect_phase(t / self.CONNECT_AT)
        cycle, into = divmod(t, self.DROP_EVERY)
        if into < self.DROP_EVERY - self.DROP_FOR:
            return "connected"
        if int(cycle) % 2 == 1:
            return "silent"
        return self._reconnect_phase((into - (self.DROP_EVERY - self.DROP_FOR)) / self.DROP_FOR)

    @staticmethod
    def _reconnect_phase(fraction):
        if fraction < 0.6:
            return "scanning"
        return "connecting" if fraction < 0.8 else "waiting"

    def right_stick(self, t):
        """(x, y) in -1..1, up positive: quarter turns that stop on 45, 135, 225, 315 degrees."""
        quarter, into = divmod(max(0.0, t - self.CONNECT_AT), self.QUADRANT_S)
        progress = min(1.0, into / self.MOVE_S)
        progress = progress * progress * (3 - 2 * progress)   # ease in and out
        angle = math.radians(45.0 + 90.0 * (quarter - 1 + progress))
        radius = self.STICK_RADIUS * min(1.0, max(0.0, t - self.CONNECT_AT))  # grow from centre
        return radius * math.cos(angle), radius * math.sin(angle)

    def sample(self, t):
        """Raw XR4 field values at time t as a dict, or None while the board is silent."""
        phase = self.phase(t)
        if phase == "silent":
            return None
        values = {"millis": int(t * 1000) + 1234, "state": STATE_WORDS.index(phase),
                  "battery": BATTERY_UNKNOWN, "seq": self._seq % 256,
                  "lx": 32768, "ly": 32768, "rx": 32768, "ry": 32768,
                  "lt": 0, "rt": 0, "buttons": 0, "dpad": 0}
        if phase != "connected":
            return values  # the library reads neutral while no controller is connected
        x, y = self.right_stick(t)
        rt_into = t % 3.0
        rt = math.sin(math.pi * rt_into / 1.2) ** 2 if rt_into < 1.2 else 0.0
        pressed = {"a": t % 4.0 < 0.3, "x": t % 9.0 < 0.3, "lb": t % 6.0 < 3.0}
        mask = sum(1 << BUTTON_NAMES.index(name) for name, down in pressed.items() if down)
        values.update({
            "battery": max(5, 87 - int(t // 60)),
            "lx": raw_axis(0.35 * math.sin(2 * math.pi * t / 5.0)),
            "ly": raw_axis(0.25 * math.sin(4 * math.pi * t / 5.0), invert=True),
            "rx": raw_axis(x), "ry": raw_axis(y, invert=True),
            "lt": round(0.3 * (1 - math.cos(2 * math.pi * t / 7.0)) / 2 * TRIGGER_MAX),
            "rt": round(rt * TRIGGER_MAX),
            "buttons": mask,
            "dpad": 1 + int(t // 2.0) % 8 if t % 2.0 < 0.3 else 0,   # tap round the d-pad
        })
        return values

    def lines(self, t):
        """The lines the board would print at time t: '#' notes on phase changes, then one
        XR4 line (none while silent)."""
        out = []
        phase = self.phase(t)
        if phase != self._last_phase:
            out.extend(self._phase_notes(self._last_phase, phase))
            self._last_phase = phase
        values = self.sample(t)
        if values is not None:
            if phase == "connected":
                self._seq += 1
            out.append(format_xr4(values))
        return out

    @staticmethod
    def _phase_notes(old, new):
        notes = []
        if old is None:
            notes.append("# demo: synthetic controller, no board attached")
            notes.append("# begin ok: firmware demo protocol 1")
        if new == "silent":
            return notes + ["# demo: board goes silent for 3 s (link-loss test)"]
        if old == "silent":
            notes.append("# demo: board back")
        if old == "connected" and new == "scanning":
            notes.append("# demo: controller drops for 3 s (disconnect test)")
        notes.append("# state: " + new)
        return notes


def raw_axis(value, invert=False):
    """Inverse of axis(): -1..1 back to the raw 0..65535 the board sends."""
    value = -value if invert else value
    return max(0, min(65535, round(value * AXIS_CENTRE + AXIS_CENTRE)))


def format_xr4(v):
    return "XR4,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%04x,%d" % (
        v["millis"], v["state"], v["battery"], v["seq"], v["lx"], v["ly"], v["rx"], v["ry"],
        v["lt"], v["rt"], v["buttons"], v["dpad"])


def demo_reader(hub, stop, line_log, rate_hz=50):
    """Feeds the demo controller's lines at rate_hz until stop is set."""
    demo = DemoController()
    start = time.monotonic()
    step = 1.0 / rate_hz
    due = start
    while not stop.is_set():
        for line in demo.lines(time.monotonic() - start):
            line_log.write(line)
            hub.feed_line(line)
        due += step
        lag = due - time.monotonic()
        if lag < -0.5:
            due = time.monotonic()  # fell far behind (laptop slept): restart the schedule
        stop.wait(max(0.0, lag))


def ticker(hub, stop):
    while not stop.wait(TICK_S):
        hub.tick()


# ---------------------------------------------------------------------------
# HTTP
# ---------------------------------------------------------------------------

def make_handler(hub, stop, page_path=PAGE_PATH):
    min_gap = 1.0 / MAX_FRAMES_PER_S

    class Handler(BaseHTTPRequestHandler):
        server_version = "TurretDashboard/1"
        timeout = 30  # a client that stops reading cannot block a thread forever

        def log_message(self, fmt, *args):  # quiet: the terminal is for the dashboard URL
            pass

        def do_GET(self):
            path = self.path.split("?", 1)[0]
            try:
                if path == "/":
                    self._send_page()
                elif path == "/state":
                    self._send_bytes(200, "application/json",
                                     json.dumps(hub.frame(), separators=(",", ":")).encode())
                elif path == "/events":
                    self._send_events()
                else:
                    self._send_bytes(404, "text/plain; charset=utf-8", b"not found\n")
            except (BrokenPipeError, ConnectionResetError, ConnectionAbortedError, TimeoutError):
                pass  # the browser went away; nothing to clean up

        def _send_bytes(self, status, content_type, body):
            self.send_response(status)
            self.send_header("Content-Type", content_type)
            self.send_header("Content-Length", str(len(body)))
            self.send_header("Cache-Control", "no-store")
            self.end_headers()
            self.wfile.write(body)

        def _send_page(self):
            try:
                body = page_path.read_bytes()   # read per request so edits show on reload
            except OSError as error:
                self._send_bytes(500, "text/plain; charset=utf-8",
                                 ("cannot read %s: %s\n" % (page_path, error)).encode())
                return
            self._send_bytes(200, "text/html; charset=utf-8", body)

        def _send_events(self):
            self.send_response(200)
            self.send_header("Content-Type", "text/event-stream; charset=utf-8")
            self.send_header("Cache-Control", "no-store")
            self.end_headers()
            self.wfile.write(b"retry: 1000\n\n")   # browser reconnects 1 s after a drop
            seen_version, log_after = -1, 0
            last_frame = last_write = time.monotonic()
            while not stop.is_set():
                idle_left = KEEPALIVE_S - (time.monotonic() - last_write)
                version = hub.wait_for_change(seen_version, min(0.5, max(0.0, idle_left)))
                now = time.monotonic()
                if version != seen_version:
                    # Coalesce: wait out the frame gap, then send whatever is latest by then.
                    stop.wait(max(0.0, min_gap - (now - last_frame)))
                    frame = hub.frame(log_after=log_after)
                    seen_version = frame["version"]
                    if frame["log"]:
                        log_after = frame["log"][-1]["id"]
                    payload = json.dumps(frame, separators=(",", ":"))
                    self.wfile.write(("data: %s\n\n" % payload).encode())
                    last_frame = last_write = time.monotonic()
                elif now - last_write >= KEEPALIVE_S:
                    # A named event, not an SSE comment: comments never reach page script,
                    # and the page needs something to time out on when the server freezes.
                    self.wfile.write(b"event: ping\ndata: {}\n\n")
                    last_write = now

    return Handler


# ---------------------------------------------------------------------------
# Main
# ---------------------------------------------------------------------------

def tcp_port(text):
    """argparse type for --http-port: a bad number gets a usage message, not a traceback."""
    try:
        value = int(text)
    except ValueError:
        raise argparse.ArgumentTypeError("not a number: %r" % text) from None
    if not 0 <= value <= 65535:
        raise argparse.ArgumentTypeError("must be 0..65535, got %d" % value)
    return value


def parse_args(argv):
    parser = argparse.ArgumentParser(
        description="Live web dashboard for the Nerf turret controller (TurretTelemetry sketch).")
    parser.add_argument("--port", help="serial port (default: the one UNO R4 WiFi, USB 2341:1002)")
    parser.add_argument("--baud", type=int, default=115200, help="serial baud (default 115200)")
    parser.add_argument("--http-port", type=tcp_port, default=8765, help="web port (default 8765)")
    parser.add_argument("--host", default="127.0.0.1",
                        help="web address to listen on (default 127.0.0.1, this computer only; "
                             "0.0.0.0 lets a phone on the same network open it)")
    parser.add_argument("--demo", action="store_true",
                        help="no serial port: a synthetic controller, to preview the page")
    parser.add_argument("--no-browser", action="store_true", help="do not open a browser tab")
    parser.add_argument("--log", metavar="FILE",
                        help="append every XR4 and '#' line to FILE with a host timestamp")
    return parser.parse_args(argv)


def start_thread(target, *args):
    thread = threading.Thread(target=target, args=args, daemon=True)
    thread.start()
    return thread


def main(argv=None):
    args = parse_args(argv)
    if not PAGE_PATH.is_file():
        print("dashboard: page missing: %s" % PAGE_PATH, file=sys.stderr)
        return 2
    port = None
    if not args.demo:
        try:
            import serial  # noqa: F401  (checked now, so a missing pyserial is a clear message)
            port = args.port or detect_port()
        except PortError as error:
            print("dashboard: %s" % error, file=sys.stderr)
            return 2
        except ImportError:
            print("dashboard: pyserial missing; run it with the project venv python",
                  file=sys.stderr)
            return 2
    try:
        line_log = LineLog(args.log)   # before the server, so a bad path fails cleanly
    except OSError as error:
        print("dashboard: cannot open log %s: %s" % (args.log, error), file=sys.stderr)
        return 2
    hub = Hub("demo" if args.demo else port, demo=args.demo)
    stop = threading.Event()
    try:
        server = ThreadingHTTPServer((args.host, args.http_port), make_handler(hub, stop))
    except OSError as error:
        line_log.close()
        print("dashboard: cannot listen on %s:%d: %s (another dashboard running? try "
              "--http-port)" % (args.host, args.http_port, error), file=sys.stderr)
        return 2
    server.daemon_threads = True

    for signum in (signal.SIGINT, signal.SIGTERM):
        signal.signal(signum, lambda *_: stop.set())
    if args.demo:
        start_thread(demo_reader, hub, stop, line_log)
    else:
        start_thread(serial_reader, hub, port, args.baud, args.port is None, stop, line_log)
    start_thread(ticker, hub, stop)
    start_thread(server.serve_forever)

    shown_host = "127.0.0.1" if args.host in ("0.0.0.0", "") else args.host
    url = "http://%s:%d/" % (shown_host, server.server_address[1])
    print("dashboard: %s  (source: %s; Ctrl+C to stop)" % (url, hub.source), flush=True)
    if not args.no_browser:
        webbrowser.open(url)

    while not stop.wait(0.5):
        pass
    server.shutdown()
    server.server_close()
    line_log.close()
    print("dashboard: stopped", flush=True)
    return 0


if __name__ == "__main__":
    sys.exit(main())

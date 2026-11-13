# SPDX-License-Identifier: MIT
"""Dashboard keyboard control, server side (architecture.md 7). No I/O and an injectable clock,
so every rule is unit-testable.

Rules:
- Off at every relay start. A click in the page POSTs /keyboard/enable with the per-boot token in
  an X-DSentry-Token header and gets a session id; only that session is accepted afterwards (a
  second enable replaces the first). The token reaches the page only through /config, which a
  page from another origin cannot read; a cross-origin request that carries the custom header is
  stopped at the CORS preflight, which the server never grants. Host header and loopback-client
  checks (host_allowed, client_is_loopback) are applied by the web layer.
- The page POSTs the whole key state on every change and at KEYBOARD_POST_HZ: hold-type keys as
  levels, toggle-type keys (and Space/F too) as press counters that only go up.
- Deadman: no state POST for LINK_TIMEOUT_MS -> neutral output (levels and queued presses
  dropped). No POST for KEYBOARD_REVOKE_MS -> the session is revoked; the page must click again.
- Each new press is held for KEYBOARD_PRESS_MS and followed by a release of the same length, so
  the RA4M1 (BUTTON_DEBOUNCE_MS lockout) sees every tap as its own rising edge; at most
  KEYBOARD_PRESS_QUEUE_MAX presses per key wait in line.
- A connected pad locks the keyboard out: enabling is refused and a live session is revoked.
"""

import hmac
import ipaddress
import secrets
import threading
import time
from typing import Callable, Dict, NamedTuple, Optional

from . import protocol as P

TOKEN_HEADER = "X-DSentry-Token"
LOOPBACK_NAMES = ("127.0.0.1", "localhost", "[::1]")

# Level keys the page reports (true while the key is held).
LEVEL_KEYS = ("up", "down", "left", "right", "full", "rt", "lb")
# Press counters the page reports -> the output they press. "rt" presses the trigger fully.
PRESS_KEYS: Dict[str, int] = {
    "rt": 0,                      # trigger, not a button bit
    "lb": P.DS_BTN_LB,
    "a": P.DS_BTN_A,
    "b": P.DS_BTN_B,
    "x": P.DS_BTN_X,
    "dpadLeft": P.DS_BTN_DPAD_LEFT,
    "dpadDown": P.DS_BTN_DPAD_DOWN,
}
COUNTER_LIMIT = 1 << 31        # press counters are JSON integers 0 .. 2**31-1

# The key map the page shows (event.code values -> what they drive), in display order.
KEY_MAP = (
    (("KeyW", "ArrowUp"), "tilt up", "up"),
    (("KeyS", "ArrowDown"), "tilt down", "down"),
    (("KeyA", "ArrowLeft"), "pan left", "left"),
    (("KeyD", "ArrowRight"), "pan right", "right"),
    (("ShiftLeft", "ShiftRight"), "full stick deflection", "full"),
    (("Space",), "right trigger (fire)", "rt"),
    (("KeyF",), "left bumper (flywheels)", "lb"),
    (("Enter", "NumpadEnter"), "A: arm / disarm", "a"),
    (("KeyM",), "B: semi / full", "b"),
    (("KeyR",), "X: magazine reset", "x"),
    (("KeyH",), "d-pad left: set home", "dpadLeft"),
    (("KeyG",), "d-pad down: go home", "dpadDown"),
    (("Escape",), "panic: neutral and keyboard off", "panic"),
)


class KeyboardError(Exception):
    """A refused keyboard request; .status is the HTTP status to answer with."""

    def __init__(self, status: int, message: str) -> None:
        super().__init__(message)
        self.status = status


class KeyboardOutput(NamedTuple):
    active: bool          # a session is enabled (the keyboard is a live input source)
    rx: int = 0
    ry: int = 0           # up negative, as on the wire
    rt: int = 0
    buttons: int = 0
    deadman: bool = False  # enabled, but no POST for LINK_TIMEOUT_MS: output forced neutral


INACTIVE = KeyboardOutput(False)


def host_allowed(host_header: Optional[str], port: int) -> bool:
    """True when the Host header names this server on a loopback name with its own port, so a
    page served by a hostile domain that re-binds its DNS to 127.0.0.1 is refused."""
    if not host_header:
        return False
    host = host_header.strip().lower()
    return host in {"%s:%d" % (name, port) for name in LOOPBACK_NAMES}


def client_is_loopback(address: Optional[str]) -> bool:
    """True for a request from this computer (127.0.0.0/8, ::1, or ::ffff:127.x)."""
    if not address:
        return False
    try:
        ip = ipaddress.ip_address(address.split("%", 1)[0])
    except ValueError:
        return False
    mapped = getattr(ip, "ipv4_mapped", None)
    return bool(ip.is_loopback or (mapped is not None and mapped.is_loopback))


def token_matches(given: Optional[str], token: str) -> bool:
    return bool(given) and hmac.compare_digest(given.encode(), token.encode())


class _PressTrack:
    """Turns press-counter increases into presses of at least press_s, each followed by a
    release that at least one output carries (so two presses never merge into one)."""

    def __init__(self, press_s: float, queue_max: int) -> None:
        self.press_s = press_s
        self.queue_max = queue_max
        self.seen = 0
        self.pending = 0
        self.pressing = False
        self.until = 0.0

    def counter(self, value: int) -> int:
        """A counter from the page; returns the number of new presses accepted."""
        if value <= self.seen:
            return 0                       # repeat or an out-of-order POST
        new = value - self.seen
        self.seen = value
        room = self.queue_max - self.pending - (1 if self.pressing else 0)
        accepted = max(0, min(new, room))
        self.pending += accepted
        return accepted

    def cancel(self) -> None:
        self.pending = 0
        self.pressing = False
        self.until = 0.0

    def held(self, now: float) -> bool:
        if self.pressing:
            if now < self.until:
                return True
            self.pressing = False
            self.until = now + self.press_s      # the release starts with this output
            return False
        if now < self.until:
            return False
        if self.pending:
            self.pending -= 1
            self.pressing = True
            self.until = now + self.press_s
            return True
        return False


class KeyboardSession:
    """The one keyboard session (or none). Thread-safe: HTTP threads call enable/apply/disable,
    the sender thread calls output()."""

    def __init__(self, settings, clock: Callable[[], float] = time.monotonic,
                 on_event: Optional[Callable[[str, str], None]] = None,
                 token: Optional[str] = None) -> None:
        self.s = settings
        self._clock = clock
        self._on_event = on_event or (lambda kind, text: None)
        self.token = token or secrets.token_urlsafe(24)
        self._lock = threading.Lock()
        self._session: Optional[str] = None
        self._last_post = 0.0
        self._last_seq = -1
        self._levels = dict.fromkeys(LEVEL_KEYS, False)
        self._tracks = {k: _PressTrack(settings.keyboard_press_s, settings.keyboard_press_queue_max)
                        for k in PRESS_KEYS}
        self._pad_present = False
        self._deadman = False
        self.reason = "off (click Enable keyboard in the page)"
        self.posts = 0
        self.enables = 0
        self._events = []          # (kind, text) raised under the lock, reported after it

    def _locked(self, fn):
        """Runs fn under the lock, then reports its events outside it (so a callback that takes
        another lock, such as the hub's, can never deadlock against this one)."""
        with self._lock:
            try:
                result = fn()
            finally:
                events, self._events = self._events, []
        for kind, text in events:
            self._on_event(kind, text)
        return result

    def _emit(self, kind: str, text: str) -> None:
        self._events.append((kind, text))

    # -- state changes ----------------------------------------------------------------------
    def _now(self, now):
        return self._clock() if now is None else now

    def _clear(self) -> None:
        self._levels = dict.fromkeys(LEVEL_KEYS, False)
        for t in self._tracks.values():
            t.cancel()

    def _end(self, why: str) -> None:
        if self._session is not None:
            self._session = None
            self._clear()
            self._deadman = False
            self.reason = why
            self._emit("keyboard", "keyboard OFF: " + why)

    def enable(self, now: Optional[float] = None) -> str:
        now = self._now(now)
        return self._locked(lambda: self._enable(now))

    def _enable(self, now: float) -> str:
        if self._pad_present:
            raise KeyboardError(409, "a pad is connected; it locks the keyboard out")
        replaced = self._session is not None
        self._session = secrets.token_urlsafe(12)
        self._last_post = now
        self._last_seq = -1
        self._clear()
        for t in self._tracks.values():
            t.seen = 0
        self._deadman = False
        self.enables += 1
        self.reason = "live"
        self._emit("keyboard", "keyboard LIVE%s" % (" (replaced the previous tab)" if replaced else ""))
        return self._session

    def disable(self, session: Optional[str], why: str = "disabled in the page",
                now: Optional[float] = None) -> None:
        def run():
            self._check_session(session)
            self._end(why)
        self._locked(run)

    def _check_session(self, session: Optional[str]) -> None:
        if self._session is None or not session or not hmac.compare_digest(
                str(session).encode(), self._session.encode()):
            raise KeyboardError(409, "keyboard session not live (revoked, replaced or off)")

    def apply(self, session: Optional[str], body: dict, now: Optional[float] = None) -> dict:
        """One key-state POST. Raises KeyboardError(400) on a malformed body, (409) when the
        session is not the live one."""
        now = self._now(now)
        seq, levels, presses = _validate_body(body)
        return self._locked(lambda: self._apply(session, seq, levels, presses, now))

    def _apply(self, session, seq, levels, presses, now) -> dict:
        self._check_session(session)
        self._expire(now)
        if self._session is None:          # revoked just now by _expire
            raise KeyboardError(409, "keyboard session revoked: " + self.reason)
        self.posts += 1
        self._last_post = now
        if self._deadman:
            self._deadman = False
            self._emit("keyboard", "keyboard state arriving again (deadman released)")
        if seq > self._last_seq:
            self._last_seq = seq
            self._levels = levels
        for key, value in presses.items():
            self._tracks[key].counter(value)
        return {"ok": True, "seq": self._last_seq}

    def set_pad_present(self, present: bool) -> None:
        def run():
            self._pad_present = bool(present)
            if present:
                self._end("a pad connected; it locks the keyboard out")
        self._locked(run)

    def _expire(self, now: float) -> None:
        """Deadman and revoke (lock held)."""
        if self._session is None:
            return
        quiet = now - self._last_post
        if quiet >= self.s.keyboard_revoke_s:
            self._end("no key state from the page for %d ms" % round(self.s.keyboard_revoke_s * 1000))
        elif quiet >= self.s.link_timeout_s:
            if not self._deadman:
                self._deadman = True
                self._clear()
                self._emit("keyboard", "keyboard deadman: no key state for %d ms, input neutral"
                               % round(self.s.link_timeout_s * 1000))

    # -- output -------------------------------------------------------------------------------
    def output(self, now: Optional[float] = None) -> KeyboardOutput:
        now = self._now(now)
        return self._locked(lambda: self._output(now))

    def _output(self, now: float) -> KeyboardOutput:
        self._expire(now)
        if self._session is None:
            return INACTIVE
        if self._deadman:
            return KeyboardOutput(True, deadman=True)
        lv = self._levels
        pressed = {k: t.held(now) for k, t in self._tracks.items()}
        rx = self._axis(lv["left"], lv["right"], lv["full"])
        ry = -self._axis(lv["down"], lv["up"], lv["full"])
        rt = self.s.trigger_axis_range if (lv["rt"] or pressed["rt"]) else 0
        buttons = P.DS_BTN_LB if lv["lb"] else 0
        for key, mask in PRESS_KEYS.items():
            if mask and pressed[key]:
                buttons |= mask
        return KeyboardOutput(True, self._clamp(rx), self._clamp(ry), rt, buttons, False)

    def _axis(self, negative: bool, positive: bool, full: bool) -> int:
        deflection = 1.0 if full else self.s.keyboard_deflection
        return round((int(positive) - int(negative)) * deflection * self.s.joy_axis_range)

    def _clamp(self, v: int) -> int:
        r = self.s.joy_axis_range
        return max(-r, min(r - 1, v))

    @property
    def enabled(self) -> bool:
        return self._session is not None

    def snapshot(self, now: Optional[float] = None) -> dict:
        now = self._now(now)
        return self._locked(lambda: self._snapshot(now))

    def _snapshot(self, now: float) -> dict:
        self._expire(now)
        return {
            "enabled": self._session is not None,
            "deadman": self._deadman,
            "padLock": self._pad_present,
            "reason": self.reason,
            "levels": dict(self._levels) if self._session else {},
            "posts": self.posts,
            "quietMs": round((now - self._last_post) * 1000) if self._session else None,
        }


def _validate_body(body) -> tuple:
    if not isinstance(body, dict):
        raise KeyboardError(400, "body must be a JSON object")
    seq = body.get("seq", 0)
    if not isinstance(seq, int) or isinstance(seq, bool) or not 0 <= seq < COUNTER_LIMIT:
        raise KeyboardError(400, "seq must be an integer 0..2**31-1")
    raw_levels = body.get("levels", {})
    raw_presses = body.get("presses", {})
    if not isinstance(raw_levels, dict) or not isinstance(raw_presses, dict):
        raise KeyboardError(400, "levels and presses must be objects")
    levels = dict.fromkeys(LEVEL_KEYS, False)
    for key, value in raw_levels.items():
        if key not in levels:
            raise KeyboardError(400, "unknown level key %r" % key)
        if not isinstance(value, bool):
            raise KeyboardError(400, "level %r must be true or false" % key)
        levels[key] = value
    presses = {}
    for key, value in raw_presses.items():
        if key not in PRESS_KEYS:
            raise KeyboardError(400, "unknown press key %r" % key)
        if not isinstance(value, int) or isinstance(value, bool) or not 0 <= value < COUNTER_LIMIT:
            raise KeyboardError(400, "press counter %r must be an integer 0..2**31-1" % key)
        presses[key] = value
    return seq, levels, presses

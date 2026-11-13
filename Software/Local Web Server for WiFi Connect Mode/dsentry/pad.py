# SPDX-License-Identifier: MIT
"""The game pad on the PC (architecture.md 7): SDL GameController through pygame._sdl2.controller,
scaled to the ControlPacket ranges, behind a small PadSource interface so the relay and the tests
do not need a real pad. Besides what a ControlPacket carries (right stick, right trigger, button
bits 0..15) a reading holds the left stick, the left trigger and SDL's battery level, for the
dashboard's controller view.

SDL facts (SDL_gamecontroller.h 2.28): sticks -32768..32767, triggers 0..32767, stick Y is
positive downward (= "up is negative", the ControlPacket convention, so Y is sent unchanged and
the RA4M1 applies JOY_INVERT_Y). The shifts to the wire ranges are derived from config.h
(JOY_AXIS_RANGE, TRIGGER_AXIS_RANGE): 32768 / 512 = 2**6, 32768 / 1024 = 2**5.

Threading: SdlPad must be created and polled on the main thread (SDL and IOKit run on the run
loop of the thread that calls them on macOS). poll() pumps SDL's event queue, which is also what
makes a timed rumble stop, so the relay calls it at RELAY_PAD_POLL_HZ.
"""

import os
from typing import Callable, List, NamedTuple, Optional, Sequence, Tuple

from . import protocol as P

SDL_AXIS_MIN = -32768
SDL_AXIS_MAX = 32767
SDL_TRIGGER_MAX = 32767
SDL_AXIS_SPAN = -SDL_AXIS_MIN            # half the stick range: 32768
SDL_TRIGGER_SPAN = SDL_TRIGGER_MAX + 1   # 32768 trigger steps

# ControlPacket button bits 0..11 (spec 5) <- SDL GameController buttons, by pygame constant name.
BUTTON_MAP: Tuple[Tuple[int, str], ...] = (
    (P.DS_BTN_A, "CONTROLLER_BUTTON_A"),
    (P.DS_BTN_B, "CONTROLLER_BUTTON_B"),
    (P.DS_BTN_X, "CONTROLLER_BUTTON_X"),
    (P.DS_BTN_Y, "CONTROLLER_BUTTON_Y"),
    (P.DS_BTN_LB, "CONTROLLER_BUTTON_LEFTSHOULDER"),
    (P.DS_BTN_RB, "CONTROLLER_BUTTON_RIGHTSHOULDER"),
    (P.DS_BTN_DPAD_UP, "CONTROLLER_BUTTON_DPAD_UP"),
    (P.DS_BTN_DPAD_DOWN, "CONTROLLER_BUTTON_DPAD_DOWN"),
    (P.DS_BTN_DPAD_LEFT, "CONTROLLER_BUTTON_DPAD_LEFT"),
    (P.DS_BTN_DPAD_RIGHT, "CONTROLLER_BUTTON_DPAD_RIGHT"),
    (P.DS_BTN_START, "CONTROLLER_BUTTON_START"),
    (P.DS_BTN_BACK, "CONTROLLER_BUTTON_BACK"),
)
# Bits 12..15 (shown on the dashboard; the ControlPacket carries them and the RA4M1 ignores them).
# SDL 2.0.14+ calls the Series X|S Share button SDL_CONTROLLER_BUTTON_MISC1 = 15
# (SDL_gamecontroller.h); pygame 2.6.1 has no constant for it, hence the fallback id.
EXTRA_BUTTON_MAP: Tuple[Tuple[int, str], ...] = (
    (P.DS_BTN_LS_CLICK, "CONTROLLER_BUTTON_LEFTSTICK"),
    (P.DS_BTN_RS_CLICK, "CONTROLLER_BUTTON_RIGHTSTICK"),
    (P.DS_BTN_XBOX, "CONTROLLER_BUTTON_GUIDE"),
    (P.DS_BTN_SHARE, "CONTROLLER_BUTTON_MISC1"),
)
SDL_FALLBACK_IDS = {"CONTROLLER_BUTTON_MISC1": 15}
AXIS_RIGHTX = "CONTROLLER_AXIS_RIGHTX"
AXIS_RIGHTY = "CONTROLLER_AXIS_RIGHTY"
AXIS_TRIGGERRIGHT = "CONTROLLER_AXIS_TRIGGERRIGHT"
AXIS_LEFTX = "CONTROLLER_AXIS_LEFTX"
AXIS_LEFTY = "CONTROLLER_AXIS_LEFTY"
AXIS_TRIGGERLEFT = "CONTROLLER_AXIS_TRIGGERLEFT"
POWER_POLL_S = 1.0       # SDL battery level re-read interval (it changes over minutes)


class PadState(NamedTuple):
    """One reading of the pad in ControlPacket units. Only rx, ry, rt and buttons are sent; the
    rest is for the dashboard's controller view."""
    connected: bool = False
    rx: int = 0          # -JOY_AXIS_RANGE .. JOY_AXIS_RANGE-1, right positive
    ry: int = 0          # same, UP NEGATIVE
    rt: int = 0          # 0 .. TRIGGER_AXIS_RANGE
    buttons: int = 0     # DS_BTN_* bits, 12..15 included
    name: str = ""
    lx: int = 0          # left stick, same ranges and signs as rx / ry
    ly: int = 0
    lt: int = 0          # left trigger, 0 .. TRIGGER_AXIS_RANGE
    battery: Optional[int] = None   # percent, when the source knows it (the demo script)
    power: str = ""      # SDL power level: unknown empty low medium full wired max ("" = n/a)


NEUTRAL = PadState()


def shift_for(sdl_span: int, wire_span: int, what: str) -> int:
    """The right shift that maps an SDL span onto a wire span; the ratio must be a power of two
    so both ends map exactly (research: v >> 6 maps -32768/0/32767 to -512/0/511)."""
    if wire_span <= 0 or sdl_span % wire_span:
        raise ValueError("%s: SDL span %d is not a multiple of %d" % (what, sdl_span, wire_span))
    ratio = sdl_span // wire_span
    if ratio & (ratio - 1):
        raise ValueError("%s: SDL span / wire span = %d is not a power of two" % (what, ratio))
    return ratio.bit_length() - 1


class Scaler:
    """SDL axis values -> ControlPacket values, with the shifts derived from config.h."""

    def __init__(self, joy_axis_range: int, trigger_axis_range: int) -> None:
        self.stick_shift = shift_for(SDL_AXIS_SPAN, joy_axis_range, "JOY_AXIS_RANGE")
        self.trigger_shift = shift_for(SDL_TRIGGER_SPAN, trigger_axis_range + 1, "TRIGGER_AXIS_RANGE")

    @classmethod
    def from_settings(cls, settings) -> "Scaler":
        return cls(settings.joy_axis_range, settings.trigger_axis_range)

    def stick(self, value: int) -> int:
        v = max(SDL_AXIS_MIN, min(SDL_AXIS_MAX, int(value)))
        return v >> self.stick_shift          # arithmetic shift: -32768 -> -512, 32767 -> 511

    def trigger(self, value: int) -> int:
        v = max(0, min(SDL_TRIGGER_MAX, int(value)))
        return v >> self.trigger_shift        # 32767 -> 1023


def buttons_from(is_pressed: Callable[[int], bool], sdl_ids: Sequence[Optional[int]],
                 table: Sequence[Tuple[int, str]] = BUTTON_MAP) -> int:
    """ControlPacket button bits from an SDL get_button(id) callable; sdl_ids in `table` order
    (None = this SDL has no such button: never pressed)."""
    bits = 0
    for (mask, _), sdl_id in zip(table, sdl_ids):
        if sdl_id is not None and is_pressed(sdl_id):
            bits |= mask
    return bits


def sdl_button_ids(pg, table: Sequence[Tuple[int, str]]) -> List[Optional[int]]:
    return [getattr(pg, name, SDL_FALLBACK_IDS.get(name)) for _, name in table]


class PadSource:
    """What the relay needs from a pad. poll() is called on the main thread."""

    kind = "none"

    def poll(self, now: float) -> PadState:
        return NEUTRAL

    def rumble(self, strong: int, weak: int, ms: int) -> bool:
        """strong/weak 0..255 (StatusPacket units), ms > 0. False when not delivered."""
        return False

    def stop_rumble(self) -> None:
        pass

    def close(self) -> None:
        pass

    def describe(self) -> str:
        return "no pad"


class NullPad(PadSource):
    """--no-pad: never connected."""

    kind = "none"

    def describe(self) -> str:
        return "pad disabled (--no-pad)"


class FakePad(PadSource):
    """A scripted pad for tests: set .state, read .rumbles."""

    kind = "fake"

    def __init__(self, state: PadState = NEUTRAL) -> None:
        self.state = state
        self.rumbles: List[Tuple] = []
        self.polls = 0

    def poll(self, now: float) -> PadState:
        self.polls += 1
        return self.state

    def rumble(self, strong: int, weak: int, ms: int) -> bool:
        self.rumbles.append(("rumble", strong, weak, ms))
        return self.state.connected

    def stop_rumble(self) -> None:
        self.rumbles.append(("stop",))

    def describe(self) -> str:
        return "fake pad"


def _prepare_sdl_environment() -> None:
    # Before pygame is imported: no window (and no Dock icon), input while unfocused, no banner.
    os.environ.setdefault("SDL_VIDEODRIVER", "dummy")
    os.environ.setdefault("SDL_JOYSTICK_ALLOW_BACKGROUND_EVENTS", "1")
    os.environ.setdefault("PYGAME_HIDE_SUPPORT_PROMPT", "1")


def import_pygame():
    """(pygame, pygame._sdl2.controller), imported with the headless environment set."""
    _prepare_sdl_environment()
    import pygame  # noqa: WPS433 (deliberately lazy: the codec and the tests work without it)
    from pygame._sdl2 import controller
    return pygame, controller


class SdlPad(PadSource):
    """The first SDL game controller. Hot-plug: CONTROLLERDEVICEADDED opens a pad when none is
    open; CONTROLLERDEVICEREMOVED (matched by instance id) closes it. Any SDL error while reading
    closes the pad, so a half-gone pad reads as disconnected instead of as a stuck stick.
    Rumble: StatusPacket strong -> SDL low_frequency, weak -> high_frequency, value / 255."""

    kind = "sdl"
    RUMBLE_FULL = 255     # StatusPacket rumble strengths are 0..255

    def __init__(self, scaler: Scaler, modules=None, log: Optional[Callable[[str], None]] = None) -> None:
        self.scaler = scaler
        self._log = log or (lambda text: None)
        self.pygame, self.gc = modules if modules is not None else import_pygame()
        pg = self.pygame
        pg.display.init()                  # the event pump needs the (dummy) video subsystem
        pg.joystick.init()
        self.gc.init()
        self._ids = [getattr(pg, name) for _, name in BUTTON_MAP]
        self._extra_ids = sdl_button_ids(pg, EXTRA_BUTTON_MAP)
        self._ax = (getattr(pg, AXIS_RIGHTX), getattr(pg, AXIS_RIGHTY), getattr(pg, AXIS_TRIGGERRIGHT))
        self._lax = (getattr(pg, AXIS_LEFTX), getattr(pg, AXIS_LEFTY), getattr(pg, AXIS_TRIGGERLEFT))
        self._pad = None
        self._iid = None
        self._joy = None
        self._name = ""
        self._power = ""
        self._power_at: Optional[float] = None
        self._scan()

    # -- hot-plug --
    def _open(self, index: int) -> None:
        try:
            if not self.gc.is_controller(index):
                return
            pad = self.gc.Controller(index)
            joy = pad.as_joystick()
            iid = joy.get_instance_id()
        except Exception as error:  # noqa: BLE001 (SDL raises pygame.error and others)
            self._log("pad %d could not be opened: %s" % (index, error))
            return
        self._pad, self._iid, self._joy = pad, iid, joy
        self._power, self._power_at = "", None
        self._name = str(getattr(pad, "name", "") or "game controller")
        self._log("pad connected: %s" % self._name)

    def _close(self, why: str) -> None:
        if self._pad is None:
            return
        pad, self._pad, self._iid, self._joy = self._pad, None, None, None
        try:
            pad.quit()
        except Exception:  # noqa: BLE001
            pass
        self._log("pad disconnected (%s): %s" % (why, self._name))

    def _scan(self) -> None:
        if self._pad is not None:
            return
        try:
            count = self.gc.get_count()
        except Exception:  # noqa: BLE001
            count = 0
        for index in range(count):
            self._open(index)
            if self._pad is not None:
                return

    def poll(self, now: float) -> PadState:
        pg = self.pygame
        try:
            events = pg.event.get()
        except Exception as error:  # noqa: BLE001
            self._close("SDL event error: %s" % error)
            return NEUTRAL
        for event in events:
            if event.type == pg.CONTROLLERDEVICEADDED and self._pad is None:
                self._open(event.device_index)
            elif event.type == pg.CONTROLLERDEVICEREMOVED and event.instance_id == self._iid:
                self._close("removed")
        pad = self._pad
        if pad is None:
            return NEUTRAL
        try:
            if not pad.attached():
                self._close("detached")
                return NEUTRAL
            rx, ry, rt = (pad.get_axis(a) for a in self._ax)
            lx, ly, lt = (pad.get_axis(a) for a in self._lax)
            buttons = buttons_from(pad.get_button, self._ids)
            buttons |= buttons_from(pad.get_button, self._extra_ids, EXTRA_BUTTON_MAP)
        except Exception as error:  # noqa: BLE001
            self._close("read error: %s" % error)
            return NEUTRAL
        s = self.scaler
        return PadState(True, s.stick(rx), s.stick(ry), s.trigger(rt), buttons, self._name,
                        s.stick(lx), s.stick(ly), s.trigger(lt), None, self._power_level(now))

    def _power_level(self, now: float) -> str:
        """SDL's battery level for the dashboard, re-read every POWER_POLL_S. SDL reports a
        level, not a percent (SDL_joystick.h: empty <= 5 %, low <= 20 %, medium <= 70 %,
        full <= 100 %, wired, max); "" when this pygame / pad cannot say."""
        if self._power_at is not None and now - self._power_at < POWER_POLL_S:
            return self._power
        self._power_at = now
        try:
            self._power = str(self._joy.get_power_level()) if self._joy is not None else ""
        except Exception:  # noqa: BLE001 (older pygame: no get_power_level)
            self._power = ""
        return self._power

    def rumble(self, strong: int, weak: int, ms: int) -> bool:
        pad = self._pad
        if pad is None:
            return False
        try:
            return bool(pad.rumble(strong / self.RUMBLE_FULL, weak / self.RUMBLE_FULL, int(ms)))
        except Exception:  # noqa: BLE001
            return False

    def stop_rumble(self) -> None:
        if self._pad is not None:
            try:
                self._pad.stop_rumble()
            except Exception:  # noqa: BLE001
                pass

    def close(self) -> None:
        self._close("relay stopping")
        try:
            self.gc.quit()
            self.pygame.quit()
        except Exception:  # noqa: BLE001
            pass

    def describe(self) -> str:
        return ("pad: %s" % self._name) if self._pad is not None else "pad: none connected (SDL)"

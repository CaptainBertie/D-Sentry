# SPDX-License-Identifier: MIT
"""The dashboard's shared state (the pattern of tools/dashboard.py: a Condition plus a version
counter that rises on every change a page should see, and a per-boot token so a page notices a
relay restart). Transports feed it frames and text from their reader threads; HTTP threads read
JSON frames out of it.

The PC never re-computes turret logic: every turret value shown is the RA4M1's TelemetryPacket.
A packet is shown when its version byte matches the layout that arrived: DS_TELEMETRY_VERSION, or
version 1 from an older RA4M1 build (no controller block: the dashboard says "not reported by
this firmware"). Anything else is counted and never shown. With no packet for
DASHBOARD_STALE_MS the frame says "stale" and the page greys every turret value; before the
first packet it says "none".

The controller view (frame["controller"], the page's CONTROLLER panel) describes one source:
while the relay itself drives with its own pad, the keyboard or the demo script, that input's
full state as the relay holds it (the pad's left stick, left trigger and battery level included,
though only the right stick, right trigger and buttons are sent); otherwise (--monitor, or the
relay has no input) the controller block of the turret's TelemetryPacket, which in BT mode is
the pad paired with the turret.

The event log is derived here from consecutive TelemetryPackets (state changes, shots, magazine,
flywheel, home, toggles, ESP32 radio, reboots), plus relay notes (ports, keyboard, input source).
"""

import collections
import queue
import secrets
import threading
import time
from typing import Callable, List, Optional, Tuple

from . import protocol as P
from .relay import RumbleDedupe

_TF = P.TelemetryFlags

CONTROLLER_SINCE_VERSION = 2     # the TelemetryPacket version that appended padState..padButtons
PAD_FIELDS = ("padState", "padBattery", "padLx", "padLy", "padRx", "padRy", "padLt", "padRt", "padButtons")
BATTERY_MAX_PERCENT = 100

# What the dashboard says for each DS_PAD_STATE_* (protocol_spec.py PadState). SCANNING means
# "scanning" only for the BLE pad; for the relay's WiFi or USB packets it means "not connected".
PAD_STATE_TEXT = {
    P.DS_PAD_STATE_CONNECTED: "connected",
    P.DS_PAD_STATE_WAITING_FIRST_REPORT: "linked, waiting for the first report",
    P.DS_PAD_STATE_FOUND: "found, connecting",
    P.DS_PAD_STATE_SCANNING: "scanning, no pad connected",
    P.DS_PAD_STATE_OFF: "Xbox BLE client off",
    P.DS_PAD_STATE_BLOCKED_BY_HCI: "blocked: the HCI bridge owns the radio",
    P.DS_PAD_STATE_NONE: "no control source",
}
NOT_CONNECTED_TEXT = "not connected"

# Which source a controller view describes, as the page labels it.
SOURCE_LABELS = {
    "pad": "relay pad",
    "keyboard": "keyboard",
    "demo": "demo script (scripted pad)",
    P.DS_LINK_SRC_BLE: "pad on the turret (BLE)",
    P.DS_LINK_SRC_WIFI: "relay over WiFi (as the turret sees it)",
    P.DS_LINK_SRC_USB: "USB ControlPackets (as the turret sees it)",
    P.DS_LINK_SRC_NONE: "no control source",
}

# SDL's battery levels (SDL_joystick.h SDL_JoystickPowerLevel) as the page shows them.
SDL_POWER_TEXT = {
    "empty": "empty (<= 5 %)", "low": "low (<= 20 %)", "medium": "medium (<= 70 %)",
    "full": "full (<= 100 %)", "wired": "wired", "max": "max",
}


def enum_name(enum_cls, value: int) -> str:
    try:
        return enum_cls(value).name
    except ValueError:
        return "UNKNOWN(%d)" % value


def deg(cdeg: int) -> float:
    return cdeg / P.DS_ANGLE_SCALE


def lcd_rows(text: str) -> List[str]:
    cols = P.DS_TELEMETRY_LCD_COLS
    return [text[i * cols:(i + 1) * cols] for i in range(P.DS_TELEMETRY_LCD_ROWS)]


def telemetry_layout_ok(pkt: P.TelemetryPacket, payload_len: Optional[int] = None) -> bool:
    """The version byte names the layout that arrived (payload_len None = the current one):
    DS_TELEMETRY_VERSION, or an older version at its own exact length."""
    layout = P.DS_TELEMETRY_VERSION if payload_len is None else P.TelemetryPacket.layout_version(payload_len)
    return layout is not None and pkt.version == layout


def controller_reported(pkt: P.TelemetryPacket) -> bool:
    return pkt.version >= CONTROLLER_SINCE_VERSION


def telemetry_view(pkt: P.TelemetryPacket) -> dict:
    d = pkt.to_dict()
    reported = controller_reported(pkt)
    d.update({
        "stateName": enum_name(P.TurretState, pkt.state),
        "faultName": enum_name(P.Fault, pkt.fault),
        "fireModeName": enum_name(P.FireMode, pkt.fireMode),
        "linkSourceName": enum_name(P.LinkSource, pkt.linkSource),
        "espModeName": enum_name(P.EspMode, pkt.espMode),
        "flagBits": P.bits_to_dict("TelemetryFlags", pkt.flags),
        "panDeg": deg(pkt.panCdeg), "tiltDeg": deg(pkt.tiltCdeg),
        "panVelDegS": deg(pkt.panVelCdegS), "tiltVelDegS": deg(pkt.tiltVelCdegS),
        "homePanDeg": deg(pkt.homePanCdeg), "homeTiltDeg": deg(pkt.homeTiltCdeg),
        "lcdRows": lcd_rows(pkt.lcd),
        "controllerReported": reported,
        "padStateName": enum_name(P.PadState, pkt.padState) if reported else None,
        "padButtonBits": P.bits_to_dict("Buttons", pkt.padButtons) if reported else None,
    })
    if not reported:                 # an older build: the decoder's defaults are not facts
        for name in PAD_FIELDS:
            d[name] = None
    return d


def battery_text_percent(value: Optional[int]) -> str:
    if value is None or value == P.DS_PAD_BATTERY_UNKNOWN:
        return "unknown"
    if value > BATTERY_MAX_PERCENT:
        return "invalid (%d)" % value
    return "%d %%" % value


def pad_state_text(state: int, link_source: int) -> str:
    if state == P.DS_PAD_STATE_SCANNING and link_source != P.DS_LINK_SRC_BLE:
        return NOT_CONNECTED_TEXT
    return PAD_STATE_TEXT.get(state, "unknown state %d" % state)


def _sticks(lx=0, ly=0, rx=0, ry=0, lt=0, rt=0, buttons=0) -> dict:
    return {"lx": lx, "ly": ly, "rx": rx, "ry": ry, "lt": lt, "rt": rt, "buttons": buttons,
            "buttonBits": P.bits_to_dict("Buttons", buttons)}


def turret_controller_view(pkt: Optional[P.TelemetryPacket], telemetry_state: str) -> dict:
    """The controller block of the latest TelemetryPacket, for the page."""
    view = {"origin": "turret", "source": None, "label": "the turret's report", "name": "",
            "reported": False, "live": telemetry_state == "live", "connected": False,
            "state": None, "stateName": None, "stateText": "", "level": "bad", "battery": None,
            "batteryText": "unknown", "note": "", "sent": None}
    view.update(_sticks())
    if pkt is None or telemetry_state in ("none", "badVersion"):
        view["note"] = ("telemetry version mismatch" if telemetry_state == "badVersion"
                        else "no telemetry yet")
        return view
    view["source"] = pkt.linkSource
    view["label"] = SOURCE_LABELS.get(pkt.linkSource, "link source %d" % pkt.linkSource)
    if not controller_reported(pkt):
        view["note"] = "not reported by this firmware (TelemetryPacket v%d)" % pkt.version
        return view
    view.update(_sticks(pkt.padLx, pkt.padLy, pkt.padRx, pkt.padRy, pkt.padLt, pkt.padRt, pkt.padButtons))
    view.update({
        "reported": True,
        "connected": pkt.padState == P.DS_PAD_STATE_CONNECTED,
        "state": pkt.padState, "stateName": enum_name(P.PadState, pkt.padState),
        "stateText": pad_state_text(pkt.padState, pkt.linkSource),
        "level": ("ok" if pkt.padState == P.DS_PAD_STATE_CONNECTED else
                  "bad" if pkt.padState == P.DS_PAD_STATE_NONE else "warn"),
        "battery": None if pkt.padBattery == P.DS_PAD_BATTERY_UNKNOWN else pkt.padBattery,
        "batteryText": battery_text_percent(pkt.padBattery),
    })
    if pkt.linkSource != P.DS_LINK_SRC_BLE and pkt.padState != P.DS_PAD_STATE_NONE:
        view["note"] = "a ControlPacket has no left stick, left trigger or battery level"
    return view


def relay_controller_view(source: str, values, hold_ms: int = 0, deadman: bool = False) -> dict:
    """The relay's own input (pad, keyboard or demo script) with everything it knows."""
    if source == "keyboard":
        battery_text = "n/a (keyboard)"
    elif values.battery is not None:
        battery_text = battery_text_percent(values.battery)
    else:
        battery_text = SDL_POWER_TEXT.get(values.power, "unknown")
    if deadman:
        state_text = "enabled, but deadman: neutral (no key state from the page)"
    elif hold_ms:
        state_text = "connected; source change: sending connected=0 for %d ms" % hold_ms
    else:
        state_text = "connected, driving the turret" if source != "keyboard" else "driving the turret"
    view = {"origin": "relay", "source": source, "label": SOURCE_LABELS.get(source, source),
            "name": values.name, "reported": True, "live": True, "connected": True,
            "state": None, "stateName": None, "stateText": state_text,
            "level": "warn" if (deadman or hold_ms) else "ok",
            "battery": values.battery, "batteryText": battery_text,
            "note": "the turret receives the right stick, right trigger and buttons only",
            "sent": ["rx", "ry", "rt", "buttons"]}
    view.update(_sticks(values.lx, values.ly, values.rx, values.ry, values.lt, values.rt, values.buttons))
    return view


def status_view(pkt: P.StatusPacket) -> dict:
    d = pkt.to_dict()
    d["stateName"] = enum_name(P.TurretState, pkt.state)
    d["wantModeName"] = enum_name(P.RadioMode, pkt.wantMode)
    return d


def _flag(pkt, bit) -> bool:
    return bool(pkt.flags & bit)


def _onoff(value: bool, on: str, off: str) -> str:
    return on if value else off


def telemetry_events(prev: Optional[P.TelemetryPacket], cur: P.TelemetryPacket,
                     jitter_limit: Optional[int] = None) -> List[Tuple[str, str]]:
    """(kind, text) log entries for the change from prev to cur (kinds: turret, safety)."""
    ev: List[Tuple[str, str]] = []
    state = enum_name(P.TurretState, cur.state)
    if prev is None:
        return [("turret", "telemetry: first packet, state %s, uptime %.1f s"
                 % (state, cur.uptimeMs / 1000.0))]
    if cur.uptimeMs < prev.uptimeMs:
        ev.append(("safety", "RA4M1 restarted (uptime %.1f s -> %.1f s)"
                   % (prev.uptimeMs / 1000.0, cur.uptimeMs / 1000.0)))
    if cur.state != prev.state:
        text = "state %s -> %s" % (enum_name(P.TurretState, prev.state), state)
        if cur.state == P.DS_STATE_FAULT:
            text += " (%s)" % enum_name(P.Fault, cur.fault)
        ev.append(("safety" if cur.state in (P.DS_STATE_FAULT, P.DS_STATE_PAIRING, P.DS_STATE_OFF)
                   else "turret", text))
    elif cur.fault != prev.fault and cur.fault != P.DS_FAULT_NONE:
        ev.append(("safety", "fault %s" % enum_name(P.Fault, cur.fault)))
    if cur.fireMode != prev.fireMode:
        ev.append(("turret", "fire mode %s -> %s" % (enum_name(P.FireMode, prev.fireMode),
                                                     enum_name(P.FireMode, cur.fireMode))))
    if cur.linkSource != prev.linkSource:
        ev.append(("turret", "link source %s -> %s" % (enum_name(P.LinkSource, prev.linkSource),
                                                       enum_name(P.LinkSource, cur.linkSource))))
    if _flag(cur, _TF.LINK_CONNECTED) != _flag(prev, _TF.LINK_CONNECTED):
        up = _flag(cur, _TF.LINK_CONNECTED)
        ev.append(("turret" if up else "safety", "control link connected" if up else "control link LOST"))
    if _flag(cur, _TF.FLYWHEEL_ON) != _flag(prev, _TF.FLYWHEEL_ON):
        ev.append(("turret", _onoff(_flag(cur, _TF.FLYWHEEL_ON), "flywheel on (ramping)", "flywheel off")))
    if _flag(cur, _TF.FLYWHEEL_READY) and not _flag(prev, _TF.FLYWHEEL_READY):
        ev.append(("turret", "flywheel ready (spun up)"))
    shots = (cur.shotsTotal - prev.shotsTotal) & 0xFFFF
    if 0 < shots < 0x8000:
        first = (prev.shotsTotal + 1) & 0xFFFF
        if shots == 1:
            ev.append(("turret", "shot #%d, ammo %d" % (cur.shotsTotal, cur.ammo)))
        else:
            ev.append(("turret", "shots #%d-#%d, ammo %d" % (first, cur.shotsTotal, cur.ammo)))
    if cur.ammo > prev.ammo:
        ev.append(("turret", "magazine reset to %d" % cur.ammo))
    elif cur.ammo == 0 and prev.ammo > 0:
        ev.append(("turret", "magazine empty"))
    if _flag(cur, _TF.EMPTY_MAG_BLINK) and not _flag(prev, _TF.EMPTY_MAG_BLINK):
        ev.append(("turret", "empty magazine: LEDs flash purple until X"))
    if _flag(cur, _TF.HOMING) != _flag(prev, _TF.HOMING):
        ev.append(("turret", _onoff(_flag(cur, _TF.HOMING), "go home started", "go home ended")))
    if (cur.homePanCdeg, cur.homeTiltCdeg) != (prev.homePanCdeg, prev.homeTiltCdeg):
        ev.append(("turret", "home set to pan %.2f deg, tilt %.2f deg"
                   % (deg(cur.homePanCdeg), deg(cur.homeTiltCdeg))))
    if _flag(cur, _TF.CALIBRATED) != _flag(prev, _TF.CALIBRATED):
        ev.append(("turret", _onoff(_flag(cur, _TF.CALIBRATED), "tilt calibrated", "tilt calibration cleared")))
    if _flag(cur, _TF.POWER_TOGGLE_ON) != _flag(prev, _TF.POWER_TOGGLE_ON):
        ev.append(("safety", "power toggle " + _onoff(_flag(cur, _TF.POWER_TOGGLE_ON), "ON", "OFF")))
    if _flag(cur, _TF.MODE_TOGGLE_WIFI) != _flag(prev, _TF.MODE_TOGGLE_WIFI):
        ev.append(("turret", "mode toggle " + _onoff(_flag(cur, _TF.MODE_TOGGLE_WIFI), "WIFI", "BT")))
    if cur.espMode != prev.espMode:
        ev.append(("turret", "ESP32 radio %s -> %s" % (enum_name(P.EspMode, prev.espMode),
                                                      enum_name(P.EspMode, cur.espMode))))
    if _flag(cur, _TF.ESP_ALIVE) != _flag(prev, _TF.ESP_ALIVE):
        ev.append(("turret", _onoff(_flag(cur, _TF.ESP_ALIVE), "ESP32 answering", "ESP32 silent")))
    if _flag(cur, _TF.ESP_LEGACY_FIRMWARE) and not _flag(prev, _TF.ESP_LEGACY_FIRMWARE):
        ev.append(("turret", "ESP32 runs legacy firmware (BT mode only, no rumble or telemetry by WiFi)"))
    if _flag(cur, _TF.LCD_PRESENT) != _flag(prev, _TF.LCD_PRESENT):
        ev.append(("turret", _onoff(_flag(cur, _TF.LCD_PRESENT), "LCD found", "LCD missing")))
    if _flag(cur, _TF.STALL_ENABLED) != _flag(prev, _TF.STALL_ENABLED):
        ev.append(("turret", "stall detector " + _onoff(_flag(cur, _TF.STALL_ENABLED), "on", "off")))
    if _flag(cur, _TF.DRIVERS_ENABLED) != _flag(prev, _TF.DRIVERS_ENABLED):
        ev.append(("turret", "stepper drivers " + _onoff(_flag(cur, _TF.DRIVERS_ENABLED), "enabled", "disabled")))
    if jitter_limit is not None and cur.stepJitterPermille > jitter_limit >= prev.stepJitterPermille:
        ev.append(("safety", "step jitter %d permille, over the %d limit" % (cur.stepJitterPermille, jitter_limit)))
    return ev


class Hub:
    """Thread-safe latest state. `version` rises on every change a page should see."""

    def __init__(self, settings, mode: str, demo: bool = False,
                 clock: Callable[[], float] = time.monotonic, wall: Callable[[], float] = time.time) -> None:
        self.s = settings
        self.mode = mode
        self.demo = demo
        self._clock = clock
        self._wall = wall
        self._cond = threading.Condition()
        self.boot = secrets.token_hex(4)
        self.started = clock()
        self.version = 0
        self.telemetry: Optional[P.TelemetryPacket] = None
        self._telem_at: Optional[float] = None
        self.status: Optional[P.StatusPacket] = None
        self._status_at: Optional[float] = None
        self.bad_version = 0
        self._bad_at: Optional[float] = None
        self.frames_by_type = collections.Counter()
        self._events = collections.deque(maxlen=settings.event_log_keep)
        self._text = collections.deque(maxlen=settings.event_log_keep)
        self._next_id = 1
        self._last_tstate = "none"
        self._last_input = None
        self._last_kb = None
        self._last_ctl = None
        self._last_rates_at = clock()
        self._crc_logged: Tuple[int, int] = (0, 0)
        self._crc_logged_at = clock()
        self.jitter_limit = settings.value("STEP_JITTER_LIMIT_PERMILLE")
        self.rumble = RumbleDedupe()
        self.rumble_queue: "queue.SimpleQueue" = queue.SimpleQueue()
        # attached later by the relay (all optional, so tests can build a bare Hub)
        self.transport = None
        self.stats = None
        self.sender = None
        self.keyboard = None
        self.merger = None
        self.pad_info: Callable[[], dict] = lambda: {"present": False, "name": "", "enabled": False}

    def attach(self, **parts) -> None:
        for name, value in parts.items():
            if not hasattr(self, name):
                raise AttributeError(name)
            setattr(self, name, value)

    # -- sink (transport reader threads) ------------------------------------------------------
    def on_frame(self, frame: P.Frame, via: str = "", now: Optional[float] = None) -> None:
        now = self._clock() if now is None else now
        pkt = frame.packet
        with self._cond:
            self.frames_by_type[frame.type.name] += 1
            if frame.type == P.PacketType.TELEMETRY:
                # the layout that arrived; a Frame built in process without its bytes is current
                self._telemetry(pkt, now, len(frame.raw) - P.DS_FRAME_OVERHEAD if frame.raw else None)
            elif frame.type == P.PacketType.STATUS:
                self._status(pkt, now)
            else:
                return                        # Control/Beacon frames are not for the dashboard
            self._changed()

    def _telemetry(self, pkt: P.TelemetryPacket, now: float, payload_len: Optional[int] = None) -> None:
        if not telemetry_layout_ok(pkt, payload_len):
            self.bad_version += 1
            if self._bad_at is None:
                known = sorted(set(P.TelemetryPacket.LEGACY.values()) | {P.DS_TELEMETRY_VERSION})
                self._log("safety", "TelemetryPacket version %d (%d bytes), this relay understands %s: "
                          "values ignored" % (pkt.version, P.TelemetryPacket.LEN if payload_len is None
                                              else payload_len, ", ".join(map(str, known))))
            self._bad_at = now
            return
        if not controller_reported(pkt) and (self.telemetry is None or controller_reported(self.telemetry)):
            self._log("turret", "TelemetryPacket version %d: an older RA4M1 build, no controller block "
                      "(flash the current sketch to see the pad on the turret)" % pkt.version)
        if self.stats is not None:
            expected = self.transport.expected_link_source if (self.transport is not None and self.transport.sends) else None
            self.stats.on_telemetry(pkt, expected, now)
        for kind, text in telemetry_events(self.telemetry, pkt, self.jitter_limit):
            self._log(kind, text)
        self._log_crc(pkt, now)
        self.telemetry = pkt
        self._telem_at = now
        self._bad_at = None

    def _log_crc(self, pkt: P.TelemetryPacket, now: float) -> None:
        """Turret CRC errors and seq gaps: at most one log line per RELAY_STATS_MS."""
        counts = (pkt.crcErrors, pkt.seqGaps)
        if self.telemetry is None or self.telemetry.uptimeMs > pkt.uptimeMs:
            self._crc_logged = counts                         # baseline (first packet, reboot)
            return
        if counts != self._crc_logged and now - self._crc_logged_at >= self.s.stats_s:
            d_crc = (counts[0] - self._crc_logged[0]) & 0xFFFF
            d_gap = (counts[1] - self._crc_logged[1]) & 0xFFFF
            self._log("turret", "turret saw %d CRC errors, %d seq gaps (totals %d, %d)"
                      % (d_crc, d_gap, counts[0], counts[1]))
            self._crc_logged, self._crc_logged_at = counts, now

    def _status(self, pkt: P.StatusPacket, now: float) -> None:
        if self.stats is not None:
            self.stats.on_status(now)
        cmd = self.rumble.feed(pkt)
        if cmd is not None:
            self.rumble_queue.put(cmd)
        self.status = pkt
        self._status_at = now

    def on_text(self, text: str, via: str = "") -> None:
        with self._cond:
            self._text.append({"id": self._next_id, "t": round(self._wall(), 3), "via": via, "text": text})
            self._next_id += 1
            self._changed()

    def note(self, kind: str, text: str) -> None:
        with self._cond:
            self._log(kind, text)
            self._changed()

    def _log(self, kind: str, text: str) -> None:
        self._events.append({"id": self._next_id, "t": round(self._wall(), 3), "kind": kind, "text": text})
        self._next_id += 1

    def _changed(self) -> None:
        self.version += 1
        self._cond.notify_all()

    # -- clock-driven checks (ticker thread) ----------------------------------------------------
    def telemetry_state(self, now: float) -> str:
        if self._bad_at is not None:
            return "badVersion"
        if self._telem_at is None:
            return "none"
        return "stale" if now - self._telem_at > self.s.stale_s else "live"

    def tick(self, now: Optional[float] = None) -> None:
        now = self._clock() if now is None else now
        inp = None
        if self.sender is not None:
            pkt, snap = self.sender.latest()
            if pkt is not None:
                inp = (pkt.flags, pkt.rx, pkt.ry, pkt.rt, pkt.buttons, snap.source, snap.hold_s > 0)
        kb = None
        if self.keyboard is not None:
            snap = self.keyboard.snapshot(now)
            kb = (snap["enabled"], snap["deadman"], snap["padLock"])
        # the relay's own input in full (a pad's left stick moves without changing the packet)
        ctl = self.merger.last_input() if self.merger is not None else None
        with self._cond:
            changed = False
            state = self.telemetry_state(now)
            if state != self._last_tstate:
                if state == "stale":
                    self._log("safety", "telemetry STALE: nothing for %d ms" % round(self.s.stale_s * 1000))
                elif state == "live" and self._last_tstate == "stale":
                    self._log("turret", "telemetry live again")
                self._last_tstate = state
                changed = True
            if inp != self._last_input or kb != self._last_kb or ctl != self._last_ctl:
                self._last_input, self._last_kb, self._last_ctl = inp, kb, ctl
                changed = True
            if now - self._last_rates_at >= self.s.stats_s:
                self._last_rates_at = now
                changed = True
            if changed:
                self._changed()

    def wait_for_change(self, after_version: int, timeout: float) -> int:
        with self._cond:
            self._cond.wait_for(lambda: self.version != after_version, timeout)
            return self.version

    # -- output -------------------------------------------------------------------------------
    def frame(self, now: Optional[float] = None, log_after: int = 0, text_after: int = 0) -> dict:
        now = self._clock() if now is None else now
        # parts with their own locks first, never while holding the hub's
        stats = self.stats.snapshot(now) if self.stats is not None else {}
        transport = self.transport.status() if self.transport is not None else {"kind": self.mode, "up": False}
        kb = self.keyboard.snapshot(now) if self.keyboard is not None else {"enabled": False}
        inp = self._input_view()
        pad = self.pad_info()
        relay_ctl = self._relay_controller()
        with self._cond:
            state = self.telemetry_state(now)
            telem = self.telemetry
            controller = self._pick_controller(relay_ctl, telem, state)
            return {
                "version": self.version,
                "boot": self.boot,
                "t": round(self._wall(), 3),
                "demo": self.demo,
                "mode": self.mode,
                "uptimeS": round(now - self.started, 1),
                "telemetryState": state,
                "telemetryAgeMs": None if self._telem_at is None else round((now - self._telem_at) * 1000),
                "telemetry": telemetry_view(telem) if telem is not None else None,
                "badVersionCount": self.bad_version,
                "status": status_view(self.status) if self.status is not None else None,
                "statusAgeMs": None if self._status_at is None else round((now - self._status_at) * 1000),
                "relay": dict(stats, transport=transport, relayCrcErrors=transport.get("crcErrors", 0),
                              rumbleCommands=self.rumble.commands),
                "input": inp,
                "pad": pad,
                "controller": controller,
                "keyboard": kb,
                "log": [dict(e) for e in self._events if e["id"] > log_after],
                "text": [dict(e) for e in self._text if e["id"] > text_after],
            }

    def _pick_controller(self, relay_ctl: Optional[dict], telem, state: str) -> dict:
        """Which pad the CONTROLLER panel describes. The relay's own input only while the turret is
        actually obeying the relay (live telemetry whose linkSource is the source this transport
        drives); otherwise the turret's own report, e.g. a BLE pad that won over a relay whose USB
        control the turret ignores. With no live telemetry the relay's input is shown, marked as
        not confirmed by the turret."""
        if relay_ctl is None:
            return turret_controller_view(telem, state)
        expected = self.transport.expected_link_source if (self.transport is not None and self.transport.sends) else None
        if state == "live" and telem is not None:
            if expected is not None and telem.linkSource == expected:
                return relay_ctl
            view = turret_controller_view(telem, state)
            view["note"] = "the turret is following another source, not this relay's " + relay_ctl.get("label", "input")
            return view
        view = dict(relay_ctl)
        view["level"] = "warn"
        view["stateText"] = relay_ctl.get("stateText", "") + " (not confirmed: no live telemetry from the turret)"
        return view

    def _relay_controller(self) -> Optional[dict]:
        """The relay's own input when it drives (a pad, the keyboard or the demo script), else
        None: then the controller view is the turret's report."""
        if self.sender is None or self.merger is None:
            return None
        source, values = self.merger.last_input()
        if source is None:
            return None
        _pkt, snap = self.sender.latest()
        same = snap.source == source
        return relay_controller_view(source, values, round(snap.hold_s * 1000) if same else 0,
                                     snap.deadman if same else False)

    def _input_view(self) -> Optional[dict]:
        if self.sender is None:
            return None
        pkt, snap = self.sender.latest()
        if pkt is None:
            return None
        return {
            "source": snap.source, "holdMs": round(snap.hold_s * 1000), "deadman": snap.deadman,
            "flags": pkt.flags, "flagBits": P.bits_to_dict("ControlFlags", pkt.flags),
            "rx": pkt.rx, "ry": pkt.ry, "rt": pkt.rt, "buttons": pkt.buttons,
            "buttonBits": P.bits_to_dict("Buttons", pkt.buttons), "seq": pkt.seq,
        }


def console_line(hub: Hub, now: Optional[float] = None) -> str:
    """The once-a-second terminal line: tx rate, rx telemetry rate, turret CRC errors (from
    telemetry), relay-side deframer CRC errors, latency from lastCtrlSeq, input source, link."""
    f = hub.frame(now, log_after=1 << 62, text_after=1 << 62)
    r = f["relay"]
    t = f["telemetry"]
    lat = r.get("latencyMs")
    inp = f["input"] or {}
    src = inp.get("source") or "none"
    if inp.get("holdMs"):
        src += " (hold)"
    return ("relay: tx %5.1f/s | telemetry %4.1f/s %s lost %d | turret crc %s | relay crc %d | "
            "latency %s | input %s | %s" % (
                r.get("txHz", 0.0), r.get("telemetryHz", 0.0), f["telemetryState"], r.get("telemetryLost", 0),
                "-" if t is None else t["crcErrors"], r.get("relayCrcErrors", 0),
                "-" if lat is None else "%.1f ms" % lat, src, r["transport"].get("detail", f["mode"])))

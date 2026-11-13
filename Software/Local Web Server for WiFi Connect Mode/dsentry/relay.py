# SPDX-License-Identifier: MIT
"""The relay's core, without I/O (architecture.md 4.4, 7):

- InputMerger: one input source at a time, never blended: a connected pad, else the dashboard
  keyboard, else (demo only) the demo script. A pad locks the keyboard out. ANY change of source
  (including one appearing or going) sends connected=0 for RELAY_SOURCE_CHANGE_HOLD_MS
  (> LINK_TIMEOUT_MS), so the RA4M1 sees a link loss and comes back DISARMED, never ARMED.
- Sender: builds a ControlPacket from the merger at CONTROL_PACKET_HZ on a deadline schedule
  (no drift, no burst after a stall), seq increments and wraps at 256, always sent (connected=0
  when there is no source, so the ESP32 always knows where to reply).
- LinkStats: tx/rx rates, relay-side telemetry loss, round-trip latency from the echoed seq.
- RumbleDedupe: a StatusPacket only rumbles when its rumbleSeq is new (heartbeats repeat it).

Flag bits sent: CONNECTED when a source is live; SOURCE_WIFI always (the relay is the "WiFi" side
of spec 5's source bit, whatever cable it uses); KEYBOARD while the dashboard keyboard drives.
The ESP_PAIRING bit belongs to the ESP32 and is never set here.
"""

import collections
import statistics
import threading
import time
from typing import Callable, Deque, List, NamedTuple, Optional

from . import protocol as P
from .pad import NEUTRAL, PadState

SEQ_SPACE = 256                     # ControlPacket.seq is a u8


class InputSnapshot(NamedTuple):
    source: Optional[str]           # "pad" | "keyboard" | "demo" | None
    connected: bool
    keyboard: bool
    rx: int
    ry: int
    rt: int
    buttons: int
    hold_s: float                   # time left in the source-change hold (0 = none)
    deadman: bool = False


SILENT = InputSnapshot(None, False, False, 0, 0, 0, 0, 0.0)


class InputMerger:
    def __init__(self, settings, keyboard, fallback: Optional[Callable[[float], Optional[PadState]]] = None,
                 clock: Callable[[], float] = time.monotonic,
                 on_event: Optional[Callable[[str, str], None]] = None) -> None:
        self.s = settings
        self.keyboard = keyboard
        self.fallback = fallback
        self._clock = clock
        self._on_event = on_event or (lambda kind, text: None)
        self._lock = threading.Lock()
        self._pad: PadState = NEUTRAL
        self._pad_present = False
        self._source: Optional[str] = None
        self._hold_until = 0.0
        self._last_source: Optional[str] = None
        self._last: PadState = NEUTRAL        # the full values of the source picked last
        self.changes = 0

    def set_pad(self, state: PadState) -> None:
        """The latest pad reading (main thread)."""
        with self._lock:
            self._pad = state
        if state.connected != self._pad_present:
            self._pad_present = state.connected
            self.keyboard.set_pad_present(state.connected)     # a pad locks the keyboard out

    @property
    def pad(self) -> PadState:
        with self._lock:
            return self._pad

    def last_input(self):
        """(source, PadState): the input source current() picked last and its full values
        (a pad's left stick, left trigger and battery included; a keyboard as a PadState).
        For the dashboard's controller view; during a source-change hold these are the
        source's own values, not the neutral packet being sent."""
        with self._lock:
            return self._last_source, self._last

    def current(self, now: Optional[float] = None) -> InputSnapshot:
        now = self._clock() if now is None else now
        with self._lock:
            pad = self._pad
        kb = self.keyboard.output(now)
        source, values, deadman = None, NEUTRAL, False
        if pad.connected:
            source, values = "pad", pad
        elif kb.active:
            source, values, deadman = "keyboard", kb, kb.deadman
        elif self.fallback is not None:
            demo = self.fallback(now)
            if demo is not None and demo.connected:
                source, values = "demo", demo
        full = values if isinstance(values, PadState) else \
            PadState(True, values.rx, values.ry, values.rt, values.buttons, "dashboard keyboard")
        with self._lock:
            self._last_source, self._last = source, full
        if source != self._source:
            self.changes += 1
            self._on_event("input", "input source %s -> %s: connected=0 for %d ms (RA4M1 link-loss failsafe)"
                           % (self._source or "none", source or "none", round(self.s.source_change_hold_s * 1000)))
            self._source = source
            self._hold_until = now + self.s.source_change_hold_s
        hold = max(0.0, self._hold_until - now)
        if source is None or hold > 0.0:
            return InputSnapshot(source, False, False, 0, 0, 0, 0, hold, deadman)
        return InputSnapshot(source, True, source == "keyboard", values.rx, values.ry, values.rt,
                             values.buttons, 0.0, deadman)


def flags_for(snap: InputSnapshot) -> int:
    flags = P.DS_CTRL_FLAG_SOURCE_WIFI
    if snap.connected:
        flags |= P.DS_CTRL_FLAG_CONNECTED
        if snap.keyboard:
            flags |= P.DS_CTRL_FLAG_KEYBOARD
    return flags


def packet_for(snap: InputSnapshot, seq: int) -> P.ControlPacket:
    if not snap.connected:
        return P.ControlPacket(flags=flags_for(snap), seq=seq % SEQ_SPACE)
    return P.ControlPacket(flags=flags_for(snap), rx=snap.rx, ry=snap.ry, rt=snap.rt,
                           buttons=snap.buttons, seq=seq % SEQ_SPACE)


# ---------------------------------------------------------------------------------------------
class LinkStats:
    """Counters the console line and the dashboard show. Thread-safe."""

    def __init__(self, settings, clock: Callable[[], float] = time.monotonic) -> None:
        self.s = settings
        self._clock = clock
        self._lock = threading.Lock()
        self._tx: Deque[float] = collections.deque()
        self._telem: Deque[float] = collections.deque()
        self._status: Deque[float] = collections.deque()
        self._lat: Deque = collections.deque()
        self._sent_at: List[Optional[float]] = [None] * SEQ_SPACE
        self._last_telem_seq: Optional[int] = None
        self.tx_total = 0
        self.tx_failed = 0
        self.telemetry_total = 0
        self.telemetry_lost = 0
        self.status_total = 0
        self.latency_last_ms: Optional[float] = None

    def _trim(self, q: Deque, now: float, key=lambda x: x) -> None:
        while q and key(q[0]) <= now - self.s.stats_s:
            q.popleft()

    def on_send(self, seq: int, ok: bool, now: Optional[float] = None) -> None:
        now = self._clock() if now is None else now
        with self._lock:
            self._sent_at[seq % SEQ_SPACE] = now if ok else None
            if ok:
                self.tx_total += 1
                self._tx.append(now)
            else:
                self.tx_failed += 1

    def on_telemetry(self, pkt: P.TelemetryPacket, expected_source: Optional[int],
                     now: Optional[float] = None) -> Optional[float]:
        """Records one telemetry packet; returns the round trip in ms, or None when the echoed
        seq cannot be ours (another source is active, never sent, or ambiguous after a wrap).
        The figure includes up to one control period of waiting for the next packet."""
        now = self._clock() if now is None else now
        with self._lock:
            self.telemetry_total += 1
            self._telem.append(now)
            if self._last_telem_seq is not None:
                gap = (pkt.seq - self._last_telem_seq - 1) % SEQ_SPACE
                if gap < SEQ_SPACE // 2:
                    self.telemetry_lost += gap
            self._last_telem_seq = pkt.seq
            if expected_source is None or pkt.linkSource != expected_source:
                return None
            sent = self._sent_at[pkt.lastCtrlSeq % SEQ_SPACE]
            if sent is None:
                return None
            age = now - sent
            if age < 0 or age >= (SEQ_SPACE // 2) * self.s.control_period_s:
                return None
            ms = age * 1000.0
            self.latency_last_ms = ms
            self._lat.append((now, ms))
            return ms

    def on_status(self, now: Optional[float] = None) -> None:
        now = self._clock() if now is None else now
        with self._lock:
            self.status_total += 1
            self._status.append(now)

    def snapshot(self, now: Optional[float] = None) -> dict:
        now = self._clock() if now is None else now
        with self._lock:
            for q in (self._tx, self._telem, self._status):
                self._trim(q, now)
            self._trim(self._lat, now, key=lambda item: item[0])
            window = self.s.stats_s
            lat = [ms for _, ms in self._lat]
            return {
                "txHz": round(len(self._tx) / window, 1),
                "telemetryHz": round(len(self._telem) / window, 1),
                "statusHz": round(len(self._status) / window, 1),
                "txTotal": self.tx_total,
                "txFailed": self.tx_failed,
                "telemetryTotal": self.telemetry_total,
                "telemetryLost": self.telemetry_lost,
                "statusTotal": self.status_total,
                "latencyMs": round(statistics.median(lat), 1) if lat else None,
                "latencyMaxMs": round(max(lat), 1) if lat else None,
            }


# ---------------------------------------------------------------------------------------------
class Sender:
    """CONTROL_PACKET_HZ ControlPackets on a deadline schedule. run() is the thread body."""

    def __init__(self, settings, transport, build: Callable[[float], InputSnapshot],
                 stats: Optional[LinkStats] = None, clock: Callable[[], float] = time.monotonic,
                 first_seq: int = 0) -> None:
        self.s = settings
        self.transport = transport
        self.build = build
        self.stats = stats
        self._clock = clock
        self.seq = first_seq % SEQ_SPACE
        self.sent = 0
        self.skipped_slots = 0
        self.last_packet: Optional[P.ControlPacket] = None
        self.last_snapshot: InputSnapshot = SILENT
        self._lock = threading.Lock()

    def step(self, now: Optional[float] = None) -> P.ControlPacket:
        now = self._clock() if now is None else now
        snap = self.build(now)
        pkt = packet_for(snap, self.seq)
        ok = self.transport.send(pkt.encode())
        if self.stats is not None:
            self.stats.on_send(pkt.seq, ok, now)
        with self._lock:
            self.last_packet, self.last_snapshot = pkt, snap
        self.sent += 1
        self.seq = (self.seq + 1) % SEQ_SPACE
        return pkt

    def send_final(self) -> None:
        """One neutral connected=0 packet, so the turret fails safe now instead of after
        LINK_TIMEOUT_MS when the relay stops."""
        pkt = packet_for(SILENT, self.seq)
        try:
            self.transport.send(pkt.encode())
        except Exception:  # noqa: BLE001
            pass

    def latest(self):
        with self._lock:
            return self.last_packet, self.last_snapshot

    def run(self, stop: threading.Event) -> None:
        period = self.s.control_period_s
        due = self._clock()
        while not stop.is_set():
            self.step()
            due += period
            now = self._clock()
            if now - due > period:
                # Fell more than a period behind (laptop slept, GC pause): skip the missed slots
                # instead of bursting them out.
                self.skipped_slots += int((now - due) / period)
                due = now
            stop.wait(max(0.0, due - now))
        self.send_final()


class RumbleDedupe:
    """StatusPacket -> a rumble command only when rumbleSeq is new. The first StatusPacket seen is
    a baseline (a rumble that started before the relay was listening is not replayed)."""

    def __init__(self) -> None:
        self.last_seq: Optional[int] = None
        self.commands = 0

    def feed(self, status: P.StatusPacket):
        seq = status.rumbleSeq
        if self.last_seq is None:
            self.last_seq = seq
            return None
        if seq == self.last_seq:
            return None
        self.last_seq = seq
        self.commands += 1
        if status.rumbleMs == 0:
            return ("stop",)
        return ("rumble", status.rumbleStrong, status.rumbleWeak, status.rumbleMs)

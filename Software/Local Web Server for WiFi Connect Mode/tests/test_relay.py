# SPDX-License-Identifier: MIT
"""dsentry.relay: input source priority and the source-change hold, the deadline-scheduled
sender (rate, seq wrap, flags, final packet), link stats (latency from lastCtrlSeq) and rumble
de-duplication (architecture.md 4.4, 7).

Key safety property: test_keyboard_to_pad_sends_connected_zero_longer_than_link_timeout fails
if a source change could hand the turret straight from one input to another without the RA4M1
seeing a link loss (which is what forces it back to DISARMED, never ARMED)."""

import threading
import time
import unittest

import _support as S

from dsentry import keyboard as K
from dsentry import pad as PD
from dsentry import protocol as P
from dsentry import relay as R
from dsentry import settings as ST

SET = ST.Settings.load(S.CONFIG_H)
EPS = 1e-3
PERIOD = SET.control_period_s


class Clock:
    def __init__(self, t=500.0):
        self.t = t

    def __call__(self):
        return self.t

    def advance(self, s):
        self.t += s


class FakeTransport:
    def __init__(self, ok=True):
        self.ok = ok
        self.frames = []
        self.times = []

    def send(self, data):
        self.frames.append(bytes(data))
        self.times.append(time.monotonic())
        return self.ok


def decode_all(frames):
    return [P.ControlPacket.decode(f) for f in frames]


class MergerTests(unittest.TestCase):
    def setUp(self):
        self.clock = Clock()
        self.events = []
        self.kb = K.KeyboardSession(SET, clock=self.clock)
        self.m = R.InputMerger(SET, self.kb, clock=self.clock, on_event=lambda k, t: self.events.append(t))

    def keyboard_drive(self, **levels):
        sid = self.kb.enable()
        self.seq = 1
        self.kb.apply(sid, {"seq": 1, "levels": levels})
        return sid

    def repost(self, sid, **levels):
        """What the page does at KEYBOARD_POST_HZ: re-send the same key state."""
        def again():
            self.seq += 1
            self.kb.apply(sid, {"seq": self.seq, "levels": levels})
        return again

    def run_for(self, seconds, refresh=None):
        out = []
        t = 0.0
        while t < seconds:
            out.append((t, self.m.current()))
            if refresh:
                refresh()
            self.clock.advance(PERIOD)
            t += PERIOD
        return out

    def test_no_source_is_disconnected_neutral(self):
        snap = self.m.current()
        self.assertFalse(snap.connected)
        pkt = R.packet_for(snap, 0)
        self.assertEqual(pkt.flags, P.DS_CTRL_FLAG_SOURCE_WIFI)
        self.assertEqual((pkt.rx, pkt.ry, pkt.rt, pkt.buttons), (0, 0, 0, 0))

    def test_pad_appearing_is_held_then_passed_through(self):
        self.m.set_pad(PD.PadState(True, 100, -200, 300, P.DS_BTN_B, "pad"))
        first = self.m.current()
        self.assertEqual((first.source, first.connected), ("pad", False))
        self.clock.advance(SET.source_change_hold_s - EPS)
        self.assertFalse(self.m.current().connected)
        self.clock.advance(2 * EPS)
        snap = self.m.current()
        self.assertTrue(snap.connected)
        self.assertEqual((snap.rx, snap.ry, snap.rt, snap.buttons), (100, -200, 300, P.DS_BTN_B))
        self.assertFalse(snap.keyboard)

    def test_keyboard_to_pad_sends_connected_zero_longer_than_link_timeout(self):
        sid = self.keyboard_drive(rt=True, right=True)
        self.run_for(SET.source_change_hold_s + PERIOD, refresh=self.repost(sid, rt=True, right=True))
        live = self.m.current()
        self.assertEqual((live.source, live.connected, live.rt), ("keyboard", True, SET.trigger_axis_range))
        # a pad connects while the keyboard holds the trigger
        self.m.set_pad(PD.PadState(True, 0, 0, SET.trigger_axis_range, 0, "pad"))
        outs = self.run_for(SET.link_timeout_s + 2 * PERIOD)
        for t, snap in outs:
            self.assertFalse(snap.connected, "connected=1 at %.3f s after the source change" % t)
            pkt = R.packet_for(snap, 0)
            self.assertEqual((pkt.flags & P.DS_CTRL_FLAG_CONNECTED, pkt.rt, pkt.rx), (0, 0, 0))
        self.assertGreater(SET.source_change_hold_s, SET.link_timeout_s)
        self.assertFalse(self.kb.enabled, "a connected pad locks the keyboard out")
        with self.assertRaises(K.KeyboardError):
            self.kb.apply(sid, {"seq": 999, "levels": {"rt": True}})

    def test_pad_to_keyboard_is_held_too(self):
        self.m.set_pad(PD.PadState(True, 50, 0, 0, 0, "pad"))
        self.run_for(SET.source_change_hold_s + PERIOD)
        self.assertTrue(self.m.current().connected)
        self.m.set_pad(PD.NEUTRAL)                      # pad gone
        self.assertFalse(self.m.current().connected)
        sid = self.keyboard_drive(left=True)
        outs = self.run_for(SET.link_timeout_s + PERIOD, refresh=self.repost(sid, left=True))
        self.assertTrue(all(not s.connected for _, s in outs))
        self.run_for(SET.source_change_hold_s, refresh=self.repost(sid, left=True))
        snap = self.m.current()
        self.assertEqual((snap.source, snap.connected, snap.keyboard), ("keyboard", True, True))
        self.assertTrue(R.packet_for(snap, 0).flags & P.DS_CTRL_FLAG_KEYBOARD)
        self.assertEqual(R.packet_for(snap, 0).rx, -round(SET.keyboard_deflection * SET.joy_axis_range))

    def test_keyboard_flag_only_while_keyboard_drives(self):
        self.m.set_pad(PD.PadState(True, 0, 0, 0, 0, "pad"))
        self.run_for(SET.source_change_hold_s + PERIOD)
        snap = self.m.current()
        self.assertTrue(snap.connected)
        self.assertFalse(R.packet_for(snap, 0).flags & P.DS_CTRL_FLAG_KEYBOARD)

    def test_demo_script_is_lowest_priority(self):
        demo = PD.PadState(True, 7, 7, 0, 0, "demo")
        m = R.InputMerger(SET, self.kb, fallback=lambda now: demo, clock=self.clock)
        self.clock.advance(SET.source_change_hold_s + EPS)
        m.current()
        self.clock.advance(SET.source_change_hold_s + EPS)
        self.assertEqual(m.current().source, "demo")
        sid = self.kb.enable()
        self.kb.apply(sid, {"seq": 1, "levels": {}})
        self.assertEqual((m.current().source, m.current().connected), ("keyboard", False))
        m.set_pad(PD.PadState(True, 1, 1, 0, 0, "pad"))
        self.assertEqual(m.current().source, "pad")

    def test_last_input_keeps_the_full_values_of_the_source(self):
        self.assertEqual(self.m.last_input(), (None, PD.NEUTRAL))
        pad = PD.PadState(True, 1, 2, 3, P.DS_BTN_A | P.DS_BTN_SHARE, "pad", lx=-4, ly=5, lt=6, power="full")
        self.m.set_pad(pad)
        snap = self.m.current()
        self.assertFalse(snap.connected)                        # held: the packet is neutral ...
        self.assertEqual(self.m.last_input(), ("pad", pad))     # ... the controller view is not
        self.run_for(SET.source_change_hold_s + PERIOD)
        snap = self.m.current()
        self.assertEqual(R.packet_for(snap, 0).buttons, P.DS_BTN_A | P.DS_BTN_SHARE)   # bit 15 is sent
        self.m.set_pad(PD.NEUTRAL)
        sid = self.keyboard_drive(right=True, rt=True)
        self.m.current()
        source, values = self.m.last_input()
        self.assertEqual(source, "keyboard")
        self.assertEqual((values.rx, values.rt, values.lx, values.name),
                         (round(SET.keyboard_deflection * SET.joy_axis_range), SET.trigger_axis_range, 0,
                          "dashboard keyboard"))
        self.kb.disable(sid)
        self.m.current()
        self.assertEqual(self.m.last_input()[0], None)

    def test_every_change_is_logged(self):
        self.m.set_pad(PD.PadState(True, 0, 0, 0, 0, "pad"))
        self.m.current()
        self.m.set_pad(PD.NEUTRAL)
        self.m.current()
        self.assertEqual(len(self.events), 2)
        self.assertIn("none -> pad", self.events[0])
        self.assertIn("pad -> none", self.events[1])


class SenderTests(unittest.TestCase):
    def test_frames_are_valid_and_seq_wraps(self):
        tr = FakeTransport()
        snap = R.InputSnapshot("pad", True, False, -512, 511, 1023, 0x0811, 0.0)
        sender = R.Sender(SET, tr, lambda now: snap, first_seq=254)
        for _ in range(4):
            sender.step()
        pkts = decode_all(tr.frames)
        self.assertEqual([p.seq for p in pkts], [254, 255, 0, 1])
        self.assertEqual(pkts[0].flags, P.DS_CTRL_FLAG_CONNECTED | P.DS_CTRL_FLAG_SOURCE_WIFI)
        self.assertEqual((pkts[0].rx, pkts[0].ry, pkts[0].rt, pkts[0].buttons), (-512, 511, 1023, 0x0811))

    def test_rate_on_a_real_thread_and_final_packet(self):
        tr = FakeTransport()
        snap = R.InputSnapshot("pad", True, False, 0, 0, 0, 0, 0.0)
        sender = R.Sender(SET, tr, lambda now: snap)
        stop = threading.Event()
        th = threading.Thread(target=sender.run, args=(stop,))
        seconds = 0.6
        th.start()
        time.sleep(seconds)
        stop.set()
        th.join(2)
        n = len(tr.frames) - 1                       # the last one is the final neutral packet
        self.assertGreater(n, seconds * SET.control_hz * 0.85)
        self.assertLess(n, seconds * SET.control_hz * 1.15)
        gaps = sorted(b - a for a, b in zip(tr.times, tr.times[1:-1]))
        self.assertAlmostEqual(gaps[len(gaps) // 2], PERIOD, delta=PERIOD * 0.3)
        final = P.ControlPacket.decode(tr.frames[-1])
        self.assertFalse(final.flags & P.DS_CTRL_FLAG_CONNECTED)
        seqs = [p.seq for p in decode_all(tr.frames)]
        self.assertEqual(seqs, [(seqs[0] + i) % 256 for i in range(len(seqs))])

    def test_no_burst_after_a_stall(self):
        clock = Clock()
        tr = FakeTransport()
        sent_at = []
        snap = R.InputSnapshot(None, False, False, 0, 0, 0, 0, 0.0)

        def build(now):
            sent_at.append(clock())
            return snap

        class FakeStop:
            """wait() advances the fake clock; one wait stalls a whole second (laptop sleep)."""
            calls = 0

            def is_set(self):
                return self.calls >= 60

            def wait(self, timeout):
                self.calls += 1
                clock.advance(timeout + (1.0 if self.calls == 20 else 0.0))
                return self.is_set()

        sender = R.Sender(SET, tr, build, clock=clock)
        sender.run(FakeStop())
        same_instant = max(sent_at.count(t) for t in set(sent_at))
        self.assertLessEqual(same_instant, 2, "missed slots were burst out after the stall")
        self.assertGreater(sender.skipped_slots, 0)

    def test_failed_sends_are_counted_and_the_loop_goes_on(self):
        tr = FakeTransport(ok=False)
        stats = R.LinkStats(SET)
        sender = R.Sender(SET, tr, lambda now: R.SILENT, stats)
        for _ in range(5):
            sender.step()
        self.assertEqual((stats.tx_failed, stats.tx_total), (5, 0))


def telem(**kw):
    return P.TelemetryPacket(**kw)


class StatsTests(unittest.TestCase):
    def setUp(self):
        self.clock = Clock()
        self.st = R.LinkStats(SET, clock=self.clock)

    def test_latency_from_the_echoed_seq(self):
        self.st.on_send(7, True)
        self.clock.advance(0.012)
        ms = self.st.on_telemetry(telem(lastCtrlSeq=7, linkSource=P.DS_LINK_SRC_USB), P.DS_LINK_SRC_USB)
        self.assertAlmostEqual(ms, 12.0, places=3)
        self.assertAlmostEqual(self.st.snapshot()["latencyMs"], 12.0, places=1)

    def test_no_latency_when_another_source_is_obeyed(self):
        self.st.on_send(7, True)
        self.assertIsNone(self.st.on_telemetry(telem(lastCtrlSeq=7, linkSource=P.DS_LINK_SRC_BLE), P.DS_LINK_SRC_USB))
        self.assertIsNone(self.st.on_telemetry(telem(lastCtrlSeq=7, linkSource=P.DS_LINK_SRC_USB), None))

    def test_no_latency_for_unsent_failed_or_wrapped_seq(self):
        self.assertIsNone(self.st.on_telemetry(telem(lastCtrlSeq=9, linkSource=3), 3))      # never sent
        self.st.on_send(10, False)
        self.assertIsNone(self.st.on_telemetry(telem(lastCtrlSeq=10, linkSource=3), 3))     # send failed
        self.st.on_send(11, True)
        self.clock.advance(R.SEQ_SPACE // 2 * PERIOD + EPS)                                 # ambiguous
        self.assertIsNone(self.st.on_telemetry(telem(lastCtrlSeq=11, linkSource=3), 3))

    def test_telemetry_loss(self):
        for seq in (1, 2, 5, 6, 100, 255, 0, 1):
            self.st.on_telemetry(telem(seq=seq), None)
        # 2->5 loses 2, 6->100 loses 93; 100->255 jumps over half the seq space, which cannot be
        # told from reordering, so it is not counted; 255->0->1 is a clean wrap
        self.assertEqual(self.st.telemetry_lost, 2 + 93)
        st = R.LinkStats(SET, clock=self.clock)
        for seq in (10, 9, 11):               # a reordered datagram is not 254 lost packets
            st.on_telemetry(telem(seq=seq), None)
        self.assertEqual(st.telemetry_lost, 1)
        st = R.LinkStats(SET, clock=self.clock)
        for seq in (254, 255, 0, 1):
            st.on_telemetry(telem(seq=seq), None)
        self.assertEqual(st.telemetry_lost, 0)

    def test_rates_over_the_window(self):
        n = round(SET.control_hz * SET.stats_s)
        for i in range(n):
            self.st.on_send(i, True)
            self.clock.advance(SET.stats_s / n)
        self.assertAlmostEqual(self.st.snapshot()["txHz"], SET.control_hz, delta=SET.control_hz * 0.02)
        self.clock.advance(SET.stats_s + EPS)
        self.assertEqual(self.st.snapshot()["txHz"], 0.0)


class RumbleTests(unittest.TestCase):
    def status(self, seq, ms=80):
        return P.StatusPacket(wantMode=1, rumbleStrong=200, rumbleWeak=120, rumbleMs=ms, state=5, rumbleSeq=seq)

    def test_first_status_is_a_baseline(self):
        r = R.RumbleDedupe()
        self.assertIsNone(r.feed(self.status(4)))

    def test_heartbeat_repeat_is_not_a_new_shot(self):
        r = R.RumbleDedupe()
        r.feed(self.status(4))
        self.assertEqual(r.feed(self.status(5)), ("rumble", 200, 120, 80))
        for _ in range(3):
            self.assertIsNone(r.feed(self.status(5)))
        self.assertEqual(r.commands, 1)

    def test_zero_ms_stops(self):
        r = R.RumbleDedupe()
        r.feed(self.status(4))
        self.assertEqual(r.feed(self.status(5, ms=0)), ("stop",))

    def test_wrap_is_new(self):
        r = R.RumbleDedupe()
        r.feed(self.status(255))
        self.assertEqual(r.feed(self.status(0))[0], "rumble")


if __name__ == "__main__":
    unittest.main()

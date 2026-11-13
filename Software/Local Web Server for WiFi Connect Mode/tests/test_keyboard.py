# SPDX-License-Identifier: MIT
"""dsentry.keyboard: the dashboard keyboard rules of architecture.md 7 -- off at start, per-boot
token, Host and loopback checks, deadman to neutral after LINK_TIMEOUT_MS, revoke after
KEYBOARD_REVOKE_MS, press counters stretched to KEYBOARD_PRESS_MS, pad lock-out.

Safety properties (each test below fails if its property is broken):
  test_off_at_every_start, test_deadman_goes_neutral_after_link_timeout,
  test_deadman_drops_queued_presses, test_revoked_after_revoke_ms, test_pad_locks_keyboard_out,
  test_two_taps_are_two_separate_presses.
"""

import threading
import unittest

import _support as S

from dsentry import keyboard as K
from dsentry import protocol as P
from dsentry import settings as ST

SET = ST.Settings.load(S.CONFIG_H)
EPS = 1e-3
DEFL = round(SET.keyboard_deflection * SET.joy_axis_range)


class Clock:
    def __init__(self, t=1000.0):
        self.t = t

    def __call__(self):
        return self.t

    def advance(self, s):
        self.t += s


def body(seq, presses=None, **levels):
    return {"seq": seq, "levels": levels, "presses": presses or {}}


class RequestCheckTests(unittest.TestCase):
    def test_host_header(self):
        for host, ok in (("127.0.0.1:8766", True), ("localhost:8766", True), ("LOCALHOST:8766", True),
                         ("[::1]:8766", True), (" 127.0.0.1:8766 ", True), ("127.0.0.1:8767", False),
                         ("127.0.0.1", False), ("evil.example:8766", False), ("127.0.0.1:8766.evil.example", False),
                         ("", False), (None, False)):
            with self.subTest(host=host):
                self.assertEqual(K.host_allowed(host, 8766), ok)

    def test_loopback_client(self):
        for addr, ok in (("127.0.0.1", True), ("127.5.6.7", True), ("::1", True), ("::ffff:127.0.0.1", True),
                         ("192.168.1.5", False), ("10.0.0.1", False), ("::ffff:192.168.4.2", False),
                         ("", False), (None, False), ("not an ip", False)):
            with self.subTest(addr=addr):
                self.assertEqual(K.client_is_loopback(addr), ok)

    def test_token_compare(self):
        self.assertTrue(K.token_matches("abc", "abc"))
        for given in ("abd", "", None, "abc "):
            self.assertFalse(K.token_matches(given, "abc"))

    def test_tokens_are_per_boot(self):
        a, b = K.KeyboardSession(SET), K.KeyboardSession(SET)
        self.assertNotEqual(a.token, b.token)
        self.assertGreaterEqual(len(a.token), 24)


class SessionTests(unittest.TestCase):
    def setUp(self):
        self.clock = Clock()
        self.events = []
        self.kb = K.KeyboardSession(SET, clock=self.clock, on_event=lambda k, t: self.events.append((k, t)))

    def test_off_at_every_start(self):
        self.assertFalse(self.kb.enabled)
        self.assertEqual(self.kb.output(), K.INACTIVE)
        self.assertFalse(self.kb.snapshot()["enabled"])
        with self.assertRaises(K.KeyboardError) as cm:
            self.kb.apply("guess", body(1, right=True))
        self.assertEqual(cm.exception.status, 409)
        self.assertEqual(self.kb.output(), K.INACTIVE)

    def test_only_the_live_session_is_accepted(self):
        sid = self.kb.enable()
        for wrong in (None, "", "x" * len(sid), sid + "x"):
            with self.subTest(wrong=wrong), self.assertRaises(K.KeyboardError):
                self.kb.apply(wrong, body(1, right=True))
        self.kb.apply(sid, body(1, right=True))
        self.assertEqual(self.kb.output().rx, DEFL)

    def test_second_enable_replaces_the_first(self):
        first = self.kb.enable()
        second = self.kb.enable()
        with self.assertRaises(K.KeyboardError):
            self.kb.apply(first, body(1, right=True))
        self.kb.apply(second, body(1, left=True))
        self.assertEqual(self.kb.output().rx, -DEFL)
        self.assertTrue(any("replaced" in t for _, t in self.events))

    def test_disable(self):
        sid = self.kb.enable()
        self.kb.apply(sid, body(1, rt=True))
        self.kb.disable(sid, "test")
        self.assertEqual(self.kb.output(), K.INACTIVE)
        with self.assertRaises(K.KeyboardError):
            self.kb.disable(sid)

    def test_malformed_bodies(self):
        sid = self.kb.enable()
        for bad in ([], {"seq": "1"}, {"seq": -1}, {"seq": True}, {"levels": {"right": 1}},
                    {"levels": {"warp": True}}, {"presses": {"a": -1}}, {"presses": {"a": True}},
                    {"presses": {"z": 1}}, {"presses": []}, {"presses": {"a": 1 << 31}}):
            with self.subTest(bad=bad), self.assertRaises(K.KeyboardError) as cm:
                self.kb.apply(sid, bad)
            self.assertEqual(cm.exception.status, 400)

    def test_pad_locks_keyboard_out(self):
        sid = self.kb.enable()
        self.kb.apply(sid, body(1, rt=True, right=True))
        self.kb.set_pad_present(True)
        self.assertEqual(self.kb.output(), K.INACTIVE)          # a live session is revoked
        with self.assertRaises(K.KeyboardError) as cm:
            self.kb.enable()                                     # and a new one refused
        self.assertEqual(cm.exception.status, 409)
        self.kb.set_pad_present(False)
        self.assertTrue(self.kb.enable())

    def test_events_are_reported_outside_the_lock(self):
        # the hub's note() takes the hub lock; a callback that calls back into the session must
        # not deadlock (events are reported after the session lock is released)
        kb = K.KeyboardSession(SET, clock=self.clock, on_event=lambda k, t: kb.snapshot())
        done = threading.Event()

        def run():
            sid = kb.enable()
            kb.disable(sid)
            done.set()
        threading.Thread(target=run, daemon=True).start()
        self.assertTrue(done.wait(2.0), "deadlock: on_event called under the session lock")


class StickTests(unittest.TestCase):
    def setUp(self):
        self.clock = Clock()
        self.kb = K.KeyboardSession(SET, clock=self.clock)
        self.sid = self.kb.enable()
        self.seq = 0

    def out(self, **levels):
        self.seq += 1
        self.kb.apply(self.sid, body(self.seq, **levels))
        return self.kb.output()

    def test_directions_and_deflection(self):
        r = SET.joy_axis_range
        self.assertEqual(self.out(right=True).rx, DEFL)
        self.assertEqual(self.out(left=True).rx, -DEFL)
        self.assertEqual(self.out(up=True).ry, -DEFL)             # up is negative on the wire
        self.assertEqual(self.out(down=True).ry, DEFL)
        self.assertEqual(self.out(right=True, full=True).rx, r - 1)
        self.assertEqual(self.out(left=True, full=True).rx, -r)
        self.assertEqual(self.out(up=True, full=True).ry, -r)
        self.assertEqual(self.out(left=True, right=True).rx, 0)
        o = self.out()
        self.assertEqual((o.rx, o.ry, o.rt, o.buttons), (0, 0, 0, 0))

    def test_trigger_and_bumper_levels(self):
        o = self.out(rt=True, lb=True)
        self.assertEqual(o.rt, SET.trigger_axis_range)
        self.assertEqual(o.buttons, P.DS_BTN_LB)

    def test_out_of_order_levels_ignored(self):
        self.kb.apply(self.sid, body(5, right=True))
        self.kb.apply(self.sid, body(4))                          # an older POST arriving late
        self.assertEqual(self.kb.output().rx, DEFL)


class DeadmanTests(unittest.TestCase):
    def setUp(self):
        self.clock = Clock()
        self.events = []
        self.kb = K.KeyboardSession(SET, clock=self.clock, on_event=lambda k, t: self.events.append(t))
        self.sid = self.kb.enable()

    def test_deadman_goes_neutral_after_link_timeout(self):
        self.kb.apply(self.sid, body(1, right=True, rt=True))
        self.clock.advance(SET.link_timeout_s - EPS)
        o = self.kb.output()
        self.assertEqual((o.rx, o.rt, o.deadman), (DEFL, SET.trigger_axis_range, False))
        self.clock.advance(2 * EPS)
        o = self.kb.output()
        self.assertTrue(o.active and o.deadman)
        self.assertEqual((o.rx, o.ry, o.rt, o.buttons), (0, 0, 0, 0))
        self.assertTrue(any("deadman" in t for t in self.events))

    def test_a_post_releases_the_deadman(self):
        self.kb.apply(self.sid, body(1, right=True))
        self.clock.advance(SET.link_timeout_s + EPS)
        self.assertTrue(self.kb.output().deadman)
        self.kb.apply(self.sid, body(2, right=True))
        o = self.kb.output()
        self.assertEqual((o.deadman, o.rx), (False, DEFL))

    def test_deadman_drops_queued_presses(self):
        self.kb.apply(self.sid, body(1, presses={"a": 3}))
        self.assertTrue(self.kb.output().buttons & P.DS_BTN_A)
        self.clock.advance(SET.link_timeout_s + EPS)
        self.assertFalse(self.kb.output().buttons)
        self.kb.apply(self.sid, body(2, presses={"a": 3}))      # same counter: nothing new
        for _ in range(100):
            self.clock.advance(0.01)
            self.assertFalse(self.kb.output().buttons & P.DS_BTN_A)
            if _ % 10 == 0:
                self.kb.apply(self.sid, body(3 + _, presses={"a": 3}))

    def test_revoked_after_revoke_ms(self):
        self.kb.apply(self.sid, body(1, right=True))
        self.clock.advance(SET.keyboard_revoke_s + EPS)
        self.assertEqual(self.kb.output(), K.INACTIVE)
        self.assertFalse(self.kb.enabled)
        self.assertIn("no key state", self.kb.reason)
        with self.assertRaises(K.KeyboardError) as cm:
            self.kb.apply(self.sid, body(2, right=True))
        self.assertEqual(cm.exception.status, 409)

    def test_enable_without_posts_is_revoked_too(self):
        self.clock.advance(SET.keyboard_revoke_s + EPS)
        self.assertFalse(self.kb.snapshot()["enabled"])


class PressTests(unittest.TestCase):
    STEP = 0.005

    def setUp(self):
        self.clock = Clock()
        self.kb = K.KeyboardSession(SET, clock=self.clock)
        self.sid = self.kb.enable()
        self.seq = 0

    def post(self, presses=None, **levels):
        self.seq += 1
        self.kb.apply(self.sid, body(self.seq, presses, **levels))

    def sample(self, seconds, mask, presses=None, **levels):
        """Bit `mask` (or rt when mask is None) every STEP for `seconds`, re-posting the same
        state every KEYBOARD_POST_HZ period so the deadman never fires."""
        out, t, next_post = [], 0.0, 1.0 / SET.keyboard_post_hz
        while t < seconds:
            o = self.kb.output()
            out.append(bool(o.rt) if mask is None else bool(o.buttons & mask))
            self.clock.advance(self.STEP)
            t += self.STEP
            if t >= next_post:
                next_post += 1.0 / SET.keyboard_post_hz
                self.post(presses, **levels)
        return out

    @staticmethod
    def runs(bits, value):
        """Lengths of consecutive runs of `value`."""
        out, n = [], 0
        for b in bits + [not value]:
            if b == value:
                n += 1
            elif n:
                out.append(n)
                n = 0
        return out

    def test_a_tap_is_held_for_press_ms(self):
        self.post({"a": 1})
        bits = self.sample(0.5, P.DS_BTN_A, {"a": 1})
        self.assertEqual(len(self.runs(bits, True)), 1)
        held = self.runs(bits, True)[0] * self.STEP
        self.assertGreaterEqual(held, SET.keyboard_press_s - EPS)
        self.assertLess(held, SET.keyboard_press_s + 2 * self.STEP)

    def test_two_taps_are_two_separate_presses(self):
        self.post({"b": 2})                           # two taps between two POSTs
        bits = self.sample(1.0, P.DS_BTN_B, {"b": 2})
        presses = self.runs(bits, True)
        self.assertEqual(len(presses), 2, "two taps must reach the RA4M1 as two rising edges")
        first_end = bits.index(True) + presses[0]
        gap = self.runs(bits[first_end:], False)[0] * self.STEP
        self.assertGreaterEqual(gap, SET.keyboard_press_s - EPS)      # > BUTTON_DEBOUNCE_MS
        self.assertGreater(gap, SET.button_debounce_s)

    def test_queue_is_capped(self):
        self.post({"x": 50})
        bits = self.sample(3.0, P.DS_BTN_X, {"x": 50})
        self.assertEqual(len(self.runs(bits, True)), SET.keyboard_press_queue_max)

    def test_counters_only_go_up(self):
        self.post({"a": 1})
        self.sample(0.5, P.DS_BTN_A, {"a": 1})
        self.post({"a": 0})                          # out of order / page reload: ignored
        self.post({"a": 1})
        self.assertFalse(any(self.sample(0.5, P.DS_BTN_A, {"a": 1})))

    def test_trigger_tap_is_stretched(self):
        self.post({"rt": 1})                         # Space down and up between two sender ticks
        bits = self.sample(0.5, None, {"rt": 1})
        self.assertEqual(len(self.runs(bits, True)), 1)
        self.assertGreaterEqual(self.runs(bits, True)[0] * self.STEP, SET.keyboard_press_s - EPS)

    def test_bumper_level_and_press_merge(self):
        self.post({"lb": 1}, lb=True)                # F down: level + counter
        held = self.sample(0.3, P.DS_BTN_LB, {"lb": 1}, lb=True)
        self.assertTrue(all(held), "LB must stay down while F is held, across the press and its gap")
        self.post({"lb": 1})                         # F up
        self.assertFalse(any(self.sample(0.1, P.DS_BTN_LB, {"lb": 1})))

    def test_short_lb_tap_still_reaches_the_turret(self):
        self.post({"lb": 1})                         # F down and up within one POST
        bits = self.sample(0.3, P.DS_BTN_LB, {"lb": 1})
        self.assertGreaterEqual(sum(bits) * self.STEP, SET.keyboard_press_s - EPS)


if __name__ == "__main__":
    unittest.main()

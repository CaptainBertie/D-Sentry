# test_dashboard.py - unit and end-to-end tests of dashboard.py. No hardware: nothing here
# opens a real serial port. The serial reader is driven by a fake port object, and the
# end-to-end test runs the server in --demo mode.
#
# Run with the project venv:
#   "<repo>/.venv/bin/python" "<repo>/tools/tests/test_dashboard.py" -v
#
# SPDX-License-Identifier: MIT

import http.client
import json
import os
import socket
import subprocess
import sys
import termios
import threading
import time
import types
import unittest
import urllib.request
from http.server import ThreadingHTTPServer
from pathlib import Path

TOOLS_DIR = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(TOOLS_DIR))

import dashboard  # noqa: E402  (path set up above)
from dashboard import (DemoController, Hub, LineSplitter, PortError, TurretState,  # noqa: E402
                       aim_direction, axis, detect_port, dpad_word, parse_xr4)

GOOD = "XR4,123456,0,87,42,32768,32768,65535,0,0,1023,0815,3"


def xr4(state=0, rx=32768, ry=32768, rt=0, buttons=0, millis=1000):
    return "XR4,%d,%d,87,1,32768,32768,%d,%d,0,%d,%04x,0" % (millis, state, rx, ry, rt, buttons)


BIT = {name: 1 << i for i, name in enumerate(dashboard.BUTTON_NAMES)}


class ParseTests(unittest.TestCase):
    def test_good_line(self):
        r = parse_xr4(GOOD)
        self.assertEqual(r.millis, 123456)
        self.assertEqual((r.state, r.battery, r.seq), (0, 87, 42))
        self.assertEqual((r.lx, r.ly, r.rx, r.ry), (32768, 32768, 65535, 0))
        self.assertEqual((r.lt, r.rt), (0, 1023))
        self.assertEqual(r.buttons, 0x0815)
        self.assertEqual(r.dpad, 3)

    def test_trailing_cr_and_lf(self):
        self.assertEqual(parse_xr4(GOOD + "\r"), parse_xr4(GOOD))
        self.assertEqual(parse_xr4(GOOD + "\r\n"), parse_xr4(GOOD))

    def test_wrong_field_count(self):
        self.assertIsNone(parse_xr4(GOOD.rsplit(",", 1)[0]))
        self.assertIsNone(parse_xr4(GOOD + ",0"))
        self.assertIsNone(parse_xr4(""))
        self.assertIsNone(parse_xr4("XR4"))

    def test_wrong_tag(self):
        self.assertIsNone(parse_xr4("XR5" + GOOD[3:]))
        self.assertIsNone(parse_xr4("# begin ok: firmware 0.6.99 protocol 1"))

    def test_non_numeric(self):
        fields = GOOD.split(",")
        for index, bad in ((1, "12a"), (2, "-1"), (5, " 32768"), (5, "+1"), (9, "1.5"),
                           (11, "08g5"), (11, "815"), (11, "00815"), (12, ""), (5, "٣")):
            changed = list(fields)
            changed[index] = bad
            with self.subTest(index=index, value=bad):
                self.assertIsNone(parse_xr4(",".join(changed)))

    def test_out_of_range(self):
        fields = GOOD.split(",")
        for index, bad in ((2, "6"), (3, "101"), (3, "254"), (4, "256"), (5, "65536"),
                           (8, "70000"), (9, "1024"), (10, "1024"), (11, "1000"),
                           (12, "9"), (1, "4294967296")):
            changed = list(fields)
            changed[index] = bad
            with self.subTest(index=index, value=bad):
                self.assertIsNone(parse_xr4(",".join(changed)))

    def test_battery_unknown_is_valid(self):
        self.assertEqual(parse_xr4(GOOD.replace(",87,", ",255,")).battery, 255)

    def test_garbage(self):
        for junk in ("\x00\x00\xff", "XR4,��,1,2", "XR4" + "," * 12,
                     "�XR4,1,0,87,1,1,1,1,1,1,1,0000,0", None, 42, "XR4,1,0,87"):
            with self.subTest(junk=repr(junk)):
                self.assertIsNone(parse_xr4(junk))


class ConversionTests(unittest.TestCase):
    def test_axis_ends_and_centre(self):
        self.assertEqual(axis(0), -1.0)
        self.assertEqual(axis(65535), 1.0)
        self.assertAlmostEqual(axis(32768), 0.0, places=3)

    def test_y_inversion(self):
        self.assertEqual(axis(0, invert=True), 1.0)       # raw 0 is stick up
        self.assertEqual(axis(65535, invert=True), -1.0)  # raw 65535 is stick down
        self.assertAlmostEqual(axis(32768, invert=True), 0.0, places=3)

    def test_frame_y_is_up_positive(self):
        hub = Hub("test")
        hub.feed_line(xr4(rx=65535, ry=0), now=1.0)
        frame = hub.frame(now=1.0)
        self.assertEqual((frame["rx"], frame["ry"]), (1.0, 1.0))
        self.assertEqual(frame["aim"], "up-right")

    def test_dpad_words(self):
        expected = ["none", "up", "up-right", "right", "down-right", "down", "down-left",
                    "left", "up-left"]
        self.assertEqual([dpad_word(code) for code in range(9)], expected)
        self.assertEqual(dpad_word(9), "none")

    def test_aim_words(self):
        cases = {(0.0, 0.0): "centre", (0.1, 0.1): "centre", (0.9, 0.0): "right",
                 (-0.9, 0.0): "left", (0.0, 0.9): "up", (0.0, -0.9): "down",
                 (0.6, 0.6): "up-right", (-0.6, 0.6): "up-left", (-0.6, -0.6): "down-left",
                 (0.6, -0.6): "down-right",
                 # each axis is named once it passes the deadzone by itself (pan and tilt
                 # are separate motions, so a 20 % tilt must not hide behind "right")
                 (0.9, 0.2): "up-right", (0.93, 0.37): "up-right", (0.34, 0.94): "up-right",
                 (0.9, 0.1): "right", (-0.1, -0.9): "down",
                 # just outside the circle with neither axis at 15 %: the larger axis counts
                 (0.12, 0.1): "right", (0.11, 0.11): "up-right", (0.0, -0.15): "down"}
        for (x, y), word in cases.items():
            with self.subTest(x=x, y=y):
                self.assertEqual(aim_direction(x, y)[0], word)
        self.assertEqual(aim_direction(-0.6, 0.6), ("up-left", "left", "up"))
        self.assertEqual(aim_direction(0.9, 0.1), ("right", "right", ""))

    def test_deadzone_edge(self):
        self.assertEqual(aim_direction(0.149, 0.0)[0], "centre")
        self.assertEqual(aim_direction(0.15, 0.0)[0], "right")


class TurretStateTests(unittest.TestCase):
    def step(self, turret, a=False, x=False, lb=False, rt=0.0, live=True, now=None):
        """One sample 20 ms after the previous one unless now is given."""
        if now is None:
            self._t = getattr(self, "_t", -0.02) + 0.02
            now = self._t
        else:
            self._t = now
        return turret.update(a, x, lb, rt, live=live, now=now)

    def spin(self, turret, seconds, **held):
        """Holds the given inputs for `seconds` of virtual time, ticking every 20 ms."""
        t0 = getattr(self, "_t", 0.0)
        for i in range(1, int(seconds / 0.02) + 1):
            self.step(turret, now=t0 + 0.02 * i, **held)

    def test_a_held_toggles_once(self):
        t = TurretState()
        self.step(t)
        for _ in range(5):
            self.step(t, a=True)
        self.assertEqual(t.fire_mode, "full")

    def test_a_release_and_press_toggles_again(self):
        t = TurretState()
        self.step(t)
        self.step(t, a=True)
        self.step(t)
        self.step(t, a=True)
        self.assertEqual(t.fire_mode, "semi")
        self.step(t)
        events = self.step(t, a=True)
        self.assertEqual(t.fire_mode, "full")
        self.assertEqual(events, [("turret", "A: fire mode SEMI -> FULL AUTO")])

    def test_x_arms_and_disarms_on_edges(self):
        t = TurretState()
        self.step(t)
        self.step(t, x=True)
        self.step(t, x=True)
        self.assertTrue(t.armed)
        self.step(t)
        self.step(t, x=True)
        self.assertFalse(t.armed)

    def test_button_held_at_start_needs_release(self):
        t = TurretState()
        self.step(t, a=True, x=True)
        self.assertEqual((t.fire_mode, t.armed), ("semi", False))
        self.step(t)
        self.step(t, a=True, x=True)
        self.assertEqual((t.fire_mode, t.armed), ("full", True))

    def test_lb_is_momentary(self):
        t = TurretState()
        self.step(t, lb=True)
        self.assertTrue(t.flywheels)
        self.step(t, lb=True)
        self.assertTrue(t.flywheels)
        self.step(t)
        self.assertFalse(t.flywheels)

    def test_firing_needs_armed_ready_wheels_and_trigger_past_threshold(self):
        t = TurretState()
        self.spin(t, 1.2, lb=True, rt=1.0)
        self.assertFalse(t.firing)              # disarmed, even with the wheels ready
        self.step(t, lb=True)
        self.step(t, x=True, lb=True)           # arm
        self.step(t, lb=True, rt=0.10)
        self.assertFalse(t.firing)              # exactly 10 % is not past it
        self.step(t, lb=True, rt=0.11)
        self.assertTrue(t.firing)
        self.assertEqual(t.shots, 1)
        self.step(t, lb=True, rt=0.0)
        self.assertFalse(t.firing)
        self.step(t, rt=0.11)                   # LB released: wheels not ready
        self.assertFalse(t.firing)
        self.assertEqual(t.shots, 1)

    def test_wheels_need_one_second_of_spin(self):
        t = TurretState()
        self.step(t)
        self.step(t, x=True)
        self.spin(t, 0.98, lb=True, rt=1.0)
        self.assertTrue(t.flywheels)
        self.assertFalse(t.flywheels_ready)
        self.assertFalse(t.firing)
        self.assertEqual(t.shots, 0)
        self.spin(t, 0.04, lb=True, rt=1.0)     # crosses 1.0 s while the trigger is held
        self.assertTrue(t.flywheels_ready)
        self.assertTrue(t.firing)
        self.assertEqual(t.shots, 1)            # the pull that was waiting fires once

    def test_semi_fires_one_dart_per_pull(self):
        t = TurretState()
        self.step(t)
        self.step(t, x=True)
        self.spin(t, 1.1, lb=True)
        self.spin(t, 2.0, lb=True, rt=0.9)      # held for 2 s
        self.assertEqual(t.shots, 1)
        self.spin(t, 0.1, lb=True, rt=0.0)      # release
        self.spin(t, 0.1, lb=True, rt=0.9)      # pull again
        self.assertEqual(t.shots, 2)

    def test_semi_is_capped_at_three_per_second_and_queues_a_tap(self):
        t = TurretState()
        self.step(t)
        self.step(t, x=True)
        self.spin(t, 1.1, lb=True)
        for _ in range(25):                     # 25 quick taps in 1.0 s: 20 ms down, 20 ms up
            self.spin(t, 0.02, lb=True, rt=0.9)
            self.spin(t, 0.02, lb=True, rt=0.0)
        self.assertIn(t.shots, (3, 4))          # capped at one dart per 1/3 s
        # a tap during the cooldown is queued and fires when the cooldown ends, even if released
        self.spin(t, 0.5, lb=True)              # let the cooldown fully expire
        before = t.shots
        self.spin(t, 0.02, lb=True, rt=0.9)     # dart at once
        self.assertEqual(t.shots, before + 1)
        self.spin(t, 0.02, lb=True, rt=0.0)
        self.spin(t, 0.02, lb=True, rt=0.9)     # second tap, 40 ms later: inside the cooldown
        self.spin(t, 0.02, lb=True, rt=0.0)     # released again
        self.assertEqual(t.shots, before + 1)   # not yet
        self.spin(t, 0.30, lb=True)             # cooldown over
        self.assertEqual(t.shots, before + 2)   # the queued tap fired
        self.spin(t, 0.5, lb=True)
        self.assertEqual(t.shots, before + 2)   # and only once

    def test_semi_queued_tap_is_dropped_when_disarmed(self):
        t = TurretState()
        self.step(t)
        self.step(t, x=True)
        self.spin(t, 1.1, lb=True)
        self.spin(t, 0.02, lb=True, rt=0.9)
        self.spin(t, 0.02, lb=True, rt=0.0)
        self.spin(t, 0.02, lb=True, rt=0.9)     # queued
        self.spin(t, 0.02, lb=True, rt=0.0)
        self.step(t, lb=True)
        self.step(t, x=True, lb=True)           # disarm while the request is pending
        self.spin(t, 0.5, lb=True)
        self.assertEqual(t.shots, 1)

    def test_full_auto_fires_three_per_second(self):
        t = TurretState()
        self.step(t)
        self.step(t, a=True, x=True)
        self.spin(t, 1.1, lb=True)
        self.spin(t, 2.0, lb=True, rt=0.9)      # held for 2 s: first dart at once, then every 1/3 s
        self.assertIn(t.shots, (6, 7))
        self.spin(t, 0.2, lb=True, rt=0.0)
        shots = t.shots
        self.spin(t, 0.02, lb=True, rt=0.9)     # a new pull fires immediately again
        self.assertEqual(t.shots, shots + 1)

    def test_full_auto_cadence_advances_by_tick_without_reports(self):
        t = TurretState()
        self.step(t)
        self.step(t, a=True, x=True)
        self.spin(t, 1.1, lb=True)
        self.step(t, lb=True, rt=0.9, now=1.2)  # one sample, then the controller goes quiet
        self.assertEqual(t.shots, 1)
        for i in range(1, 51):
            t.tick(now=1.2 + 0.02 * i)          # 1.0 s of ticks
        self.assertIn(t.shots, (3, 4))

    def test_trigger_threshold_in_raw_units(self):
        hub = Hub("test")
        hub.feed_line(xr4(), now=0.0)
        hub.feed_line(xr4(buttons=BIT["x"] | BIT["lb"]), now=0.02)
        hub.feed_line(xr4(buttons=BIT["lb"], rt=102), now=1.10)        # wheels ready; 102/1023 = 9.97 %
        self.assertFalse(hub.frame(now=1.10)["firing"])
        hub.feed_line(xr4(buttons=BIT["lb"], rt=103), now=1.12)        # 103/1023 = 10.07 %
        frame = hub.frame(now=1.12)
        self.assertTrue(frame["firing"])
        self.assertEqual(frame["shots"], 1)
        self.assertTrue(frame["flywheels_ready"])

    def arm_full_spinning(self, t):
        self.step(t)
        self.step(t, a=True, x=True, lb=True)
        self.spin(t, 1.1, lb=True, rt=0.5)
        self.assertEqual((t.fire_mode, t.armed, t.flywheels, t.firing), ("full", True, True, True))

    def test_disconnect_resets_to_safe(self):
        t = TurretState()
        self.arm_full_spinning(t)
        events = t.update(False, False, False, 0.0, live=False, reason="controller scanning")
        self.assertEqual((t.fire_mode, t.armed, t.flywheels, t.firing), ("semi", False, False, False))
        self.assertEqual(events[0][0], "safety")
        self.assertIn("controller scanning", events[0][1])
        self.assertEqual(t.update(False, False, False, 0.0, live=False), [])  # logged once

    def test_held_through_dropout_needs_release(self):
        t = TurretState()
        self.arm_full_spinning(t)
        t.update(True, True, True, 0.5, live=False)
        self.step(t, a=True, x=True, lb=True, rt=0.5)        # still held after reconnect
        self.assertEqual((t.fire_mode, t.armed, t.firing), ("semi", False, False))
        self.assertTrue(t.flywheels)                         # LB is momentary, not a toggle

    def test_hub_resets_on_state_change(self):
        hub = Hub("test")
        hub.feed_line(xr4(), now=0.0)
        hub.feed_line(xr4(buttons=BIT["x"] | BIT["a"] | BIT["lb"], rt=900), now=0.02)
        frame = hub.frame(now=0.02)
        self.assertEqual((frame["armed"], frame["fire_mode"], frame["firing"]), (True, "full", False))
        for i in range(1, 12):                               # 100 ms heartbeats keep the link alive
            hub.feed_line(xr4(buttons=BIT["x"] | BIT["a"] | BIT["lb"], rt=900), now=0.02 + 0.1 * i)
        hub.tick(now=1.13)                                   # the wheels come up to speed by the clock
        frame = hub.frame(now=1.13)
        self.assertEqual((frame["flywheels_ready"], frame["firing"]), (True, True))
        self.assertGreaterEqual(frame["shots"], 1)
        hub.feed_line(xr4(state=3), now=1.15)
        frame = hub.frame(now=1.15)
        self.assertEqual((frame["armed"], frame["fire_mode"], frame["flywheels"], frame["firing"]),
                         (False, "semi", False, False))
        self.assertEqual(frame["state_word"], "scanning")

    def test_hub_resets_on_link_loss(self):
        hub = Hub("test")
        hub.feed_line(xr4(), now=0.0)
        hub.feed_line(xr4(buttons=BIT["x"] | BIT["lb"]), now=0.02)
        hub.tick(now=0.30)
        self.assertTrue(hub.frame(now=0.30)["armed"])        # 280 ms: still inside the window
        hub.tick(now=0.35)
        frame = hub.frame(now=0.35)
        self.assertFalse(frame["link_ok"])
        self.assertEqual((frame["armed"], frame["flywheels"]), (False, False))
        texts = [entry["text"] for entry in frame["log"]]
        self.assertIn("link lost: no XR4 line for 300 ms", texts)
        self.assertTrue(any(text.startswith("safety reset") for text in texts))

    def test_frame_never_says_link_lost_and_armed(self):
        hub = Hub("test")
        hub.feed_line(xr4(), now=0.0)
        hub.feed_line(xr4(buttons=BIT["x"], rt=900), now=0.02)
        frame = hub.frame(now=0.34)                          # no tick() in between
        self.assertFalse(frame["link_ok"])
        self.assertEqual((frame["armed"], frame["firing"], frame["flywheels"]),
                         (False, False, False))


class HubTests(unittest.TestCase):
    def test_bad_and_other_lines(self):
        hub = Hub("test")
        hub.feed_line("XR4,garbage", now=0.0)
        hub.feed_line("\x00\xffnoise", now=0.0)
        hub.feed_line("# begin ok: firmware 0.6.99 protocol 1\r", now=0.0)
        frame = hub.frame(now=0.0)
        self.assertEqual((frame["lines"], frame["bad_lines"]), (0, 1))
        self.assertFalse(frame["have_data"])
        self.assertIsNone(frame["state"])
        self.assertEqual(frame["log"][-1]["text"], "# begin ok: firmware 0.6.99 protocol 1")

    def test_rate_and_link(self):
        hub = Hub("test")
        for i in range(50):
            hub.feed_line(xr4(), now=i * 0.02)
        frame = hub.frame(now=0.99)
        self.assertEqual(frame["rate"], 50)
        self.assertTrue(frame["link_ok"])
        self.assertEqual(hub.frame(now=2.5)["rate"], 0)

    def test_log_after_and_cap(self):
        hub = Hub("test")
        for i in range(60):
            hub.feed_line("# line %d" % i, now=0.0)
        log = hub.frame(now=0.0)["log"]
        self.assertEqual(len(log), dashboard.LOG_KEEP)
        self.assertEqual(log[-1]["text"], "# line 59")
        self.assertEqual(hub.frame(now=0.0, log_after=log[-1]["id"])["log"], [])

    def test_control_characters_cleaned(self):
        hub = Hub("test")
        hub.feed_line("# scan: a\x1b[31mb\x07", now=0.0)
        self.assertEqual(hub.frame(now=0.0)["log"][-1]["text"], "# scan: a [31mb")

    def test_version_counts_changes(self):
        hub = Hub("test")
        before = hub.version
        hub.feed_line(xr4(), now=0.0)
        hub.feed_line("# hello", now=0.0)
        self.assertEqual(hub.version, before + 2)
        self.assertEqual(hub.wait_for_change(hub.version, 0.01), hub.version)


class LineSplitterTests(unittest.TestCase):
    def test_split_across_chunks_and_bad_bytes(self):
        splitter = LineSplitter()
        self.assertEqual(splitter.feed(b"XR4,1,"), [])
        lines = splitter.feed(b"2\r\n\xff\xfe# hi\n")
        self.assertEqual(lines, ["XR4,1,2\r", "��# hi"])

    def test_runaway_line_dropped_whole(self):
        splitter = LineSplitter(max_bytes=16)
        self.assertEqual(splitter.feed(b"x" * 40), [])
        self.assertEqual(splitter.feed(b"#tail of it\nok\n"), ["ok"])  # tail is not a line
        self.assertEqual(splitter.feed(b"y" * 20 + b"\nfine\n"), ["fine"])  # long, one chunk

    def test_runaway_line_in_small_chunks(self):
        splitter = LineSplitter()
        data = b"XR4," + b"7" * 10240 + b"\r\n" + GOOD.encode() + b"\r\n"
        lines = []
        for start in range(0, len(data), 64):
            lines += splitter.feed(data[start:start + 64])
        self.assertEqual(lines, [GOOD + "\r"])


class FakePort:
    """Stands in for serial.Serial: hands out chunks, then fails like an unplugged board."""

    def __init__(self, chunks, error):
        self._chunks = list(chunks)
        self._error = error
        self.in_waiting = 0

    def read(self, size):
        if not self._chunks:
            raise self._error
        return self._chunks.pop(0)

    def __enter__(self):
        return self

    def __exit__(self, *exc):
        return False


class SerialReaderTests(unittest.TestCase):
    def setUp(self):
        try:
            import serial
        except ImportError:
            self.skipTest("pyserial not installed (use the project venv)")
        self.serial = serial

    def test_garbage_then_unplug_then_reconnect(self):
        hub = Hub("fake")
        stop = threading.Event()
        opened = []
        error = self.serial.SerialException("device disconnected")

        def open_port(name):
            opened.append(name)
            if len(opened) == 1:
                return FakePort([b"\x00\xff\xfe garbage\n", ("XR4,1,0,87,1,1,1,1,1,1,1,0000,0\r\n"
                                 "XR4,bad\r\n").encode(), b"# hi\r\n"], error)
            if len(opened) == 2:
                raise self.serial.SerialException("port busy")
            stop.set()
            return FakePort([("XR4,2,0,87,2,1,1,1,1,1,1,0001,0\n").encode()], error)

        old_retry = dashboard.RETRY_S
        dashboard.RETRY_S = 0.01
        try:
            thread = threading.Thread(target=dashboard.serial_reader,
                                      args=(hub, "/dev/fake", 115200, False, stop,
                                            dashboard.LineLog(None), open_port))
            thread.start()
            thread.join(5)
        finally:
            dashboard.RETRY_S = old_retry
        self.assertFalse(thread.is_alive())
        self.assertEqual(opened, ["/dev/fake"] * 3)
        frame = hub.frame()
        self.assertEqual((frame["lines"], frame["bad_lines"]), (1, 1))
        texts = [entry["text"] for entry in frame["log"]]
        self.assertIn("# hi", texts)
        self.assertTrue(any("device disconnected" in text for text in texts))
        self.assertTrue(any("port busy" in text for text in texts))

    def test_unexpected_errors_are_retried(self):
        # termios.error is not an OSError, and pyserial lets it out of open() when a USB
        # device is half re-enumerated; a missing pyserial is an ImportError. Neither may
        # kill the reader thread.
        for error in (termios.error(6, "Device not configured"),
                      ImportError("No module named 'serial'")):
            with self.subTest(error=type(error).__name__):
                hub = Hub("fake")
                stop = threading.Event()
                opened = []

                def open_port(name, error=error, opened=opened, stop=stop):
                    opened.append(name)
                    if len(opened) == 1:
                        raise error                   # the reader must survive this ...
                    if len(opened) == 3:
                        stop.set()                    # ... and get here via a later retry
                    return FakePort([b"XR4,2,0,87,2,1,1,1,1,1,1,0001,0\n"] if len(opened) == 2
                                    else [], self.serial.SerialException("gone"))

                old_retry = dashboard.RETRY_S
                dashboard.RETRY_S = 0.01
                try:
                    thread = threading.Thread(target=dashboard.serial_reader,
                                              args=(hub, "/dev/fake", 115200, False, stop,
                                                    dashboard.LineLog(None), open_port))
                    thread.start()
                    thread.join(5)
                finally:
                    dashboard.RETRY_S = old_retry
                self.assertFalse(thread.is_alive())
                self.assertEqual(len(opened), 3)
                frame = hub.frame()
                self.assertEqual(frame["lines"], 1)
                texts = [entry["text"] for entry in frame["log"]]
                self.assertTrue(any(type(error).__name__ in text for text in texts), texts)


class DetectPortTests(unittest.TestCase):
    @staticmethod
    def port(device, vid, pid):
        return types.SimpleNamespace(device=device, vid=vid, pid=pid)

    def test_single_match(self):
        ports = [self.port("/dev/cu.Bluetooth", None, None),
                 self.port("/dev/cu.usbmodemA", 0x2341, 0x1002),
                 self.port("/dev/cu.usbmodemB", 0x303A, 0x1001)]
        self.assertEqual(detect_port(ports), "/dev/cu.usbmodemA")

    def test_none_or_several_refused(self):
        with self.assertRaisesRegex(PortError, "no UNO R4 WiFi"):
            detect_port([self.port("/dev/cu.x", 0x303A, 0x1001)])
        with self.assertRaisesRegex(PortError, "several"):
            detect_port([self.port("/dev/cu.a", 0x2341, 0x1002),
                         self.port("/dev/cu.b", 0x2341, 0x1002)])


class DemoTests(unittest.TestCase):
    def run_demo(self, seconds, step=0.02):
        demo = DemoController()
        hub = Hub("demo", demo=True)
        frames = []
        t = 0.0
        while t < seconds:
            for line in demo.lines(t):
                if line.startswith("XR4"):
                    self.assertIsNotNone(parse_xr4(line), line)
                hub.feed_line(line, now=t)
            hub.tick(now=t)
            frames.append((t, hub.frame(now=t, log_after=10 ** 9)))
            t = round(t + step, 6)
        return hub, frames

    def test_demo_frames_valid_and_exercise_everything(self):
        hub, frames = self.run_demo(85.0)
        self.assertEqual(hub.bad_lines, 0)
        for _, frame in frames[::50]:
            json.dumps(frame)                                # every frame is JSON-ready
        at = lambda when: min(frames, key=lambda tf: abs(tf[0] - when))[1]  # noqa: E731
        self.assertEqual(at(0.5)["state_word"], "scanning")
        self.assertEqual(at(3.0)["state_word"], "connected")
        aims = {frame["aim"] for _, frame in frames if frame["link_ok"]}
        for word in ("centre", "up-right", "up-left", "down-left", "down-right"):
            self.assertIn(word, aims)
        self.assertGreaterEqual(sum(1 for _, f in frames if f["fire_mode"] == "full"), 100)
        self.assertTrue(any(f["firing"] for _, f in frames))
        self.assertTrue(any(f["flywheels"] for _, f in frames))
        self.assertTrue(any(f["armed"] for _, f in frames))
        # 37..40 s: the controller drops (state scanning); 77..80 s: the board goes silent
        self.assertEqual(at(38.5)["state_word"], "scanning")
        self.assertFalse(at(38.5)["armed"])
        self.assertFalse(at(78.5)["link_ok"])
        self.assertTrue(at(81.0)["link_ok"])
        texts = [entry["text"] for entry in hub.frame(now=85.0)["log"]]
        self.assertIn("# demo: board back", texts)

    def test_stick_circle_pauses_in_quadrants(self):
        demo = DemoController()
        x, y = demo.right_stick(2.0 + 2.0 + 1.5)     # second quarter, holding still
        self.assertAlmostEqual(x, -0.8 * 0.7071, places=3)
        self.assertAlmostEqual(y, 0.8 * 0.7071, places=3)
        self.assertEqual(demo.right_stick(5.3), demo.right_stick(5.9))


def free_port():
    with socket.socket() as sock:
        sock.bind(("127.0.0.1", 0))
        return sock.getsockname()[1]


class EventStreamTests(unittest.TestCase):
    def test_idle_stream_sends_ping(self):
        """With nothing changing, the stream still carries an event the page can see, so a
        frozen server can be told from a quiet board."""
        hub = Hub("test")
        stop = threading.Event()
        server = ThreadingHTTPServer(("127.0.0.1", 0), dashboard.make_handler(hub, stop))
        server.daemon_threads = True
        threading.Thread(target=server.serve_forever, daemon=True).start()
        try:
            conn = http.client.HTTPConnection("127.0.0.1", server.server_address[1], timeout=5)
            conn.request("GET", "/events")
            reply = conn.getresponse()
            start = time.monotonic()
            frames, ping_at = 0, None
            while ping_at is None and time.monotonic() - start < 3:
                line = reply.fp.readline().decode()
                if line.startswith("data: {\"version\""):
                    frames += 1
                elif line == "event: ping\n":
                    ping_at = time.monotonic() - start
            conn.close()
        finally:
            stop.set()
            server.shutdown()
            server.server_close()
        self.assertEqual(frames, 1)                     # the first frame, then nothing changed
        self.assertIsNotNone(ping_at)
        self.assertLess(ping_at, 1.5)


def run_dashboard(*args):
    return subprocess.run([sys.executable, str(TOOLS_DIR / "dashboard.py"), *args],
                          capture_output=True, text=True, timeout=10,
                          env=dict(os.environ, PYTHONDONTWRITEBYTECODE="1"))


class CommandLineTests(unittest.TestCase):
    def test_unwritable_log_is_a_clean_error(self):
        result = run_dashboard("--demo", "--no-browser", "--http-port", str(free_port()),
                               "--log", "/nonexistent-dir/x.csv")
        self.assertEqual(result.returncode, 2)
        self.assertIn("dashboard: cannot open log /nonexistent-dir/x.csv", result.stderr)
        self.assertNotIn("Traceback", result.stderr)

    def test_bad_http_port_is_a_clean_error(self):
        for bad in ("99999", "-1", "web"):
            with self.subTest(port=bad):
                result = run_dashboard("--demo", "--no-browser", "--http-port", bad)
                self.assertEqual(result.returncode, 2)
                self.assertIn("--http-port", result.stderr)
                self.assertNotIn("Traceback", result.stderr)


class EndToEndTests(unittest.TestCase):
    def test_demo_server(self):
        port = free_port()
        proc = subprocess.Popen(
            [sys.executable, str(TOOLS_DIR / "dashboard.py"), "--demo", "--no-browser",
             "--http-port", str(port)],
            stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True,
            env=dict(os.environ, PYTHONUNBUFFERED="1", PYTHONDONTWRITEBYTECODE="1"))
        try:
            banner = proc.stdout.readline()
            self.assertIn("http://127.0.0.1:%d/" % port, banner)

            with urllib.request.urlopen("http://127.0.0.1:%d/state" % port, timeout=5) as reply:
                state = json.loads(reply.read())
            self.assertTrue(state["demo"])
            self.assertEqual(state["source"], "demo")

            with urllib.request.urlopen("http://127.0.0.1:%d/" % port, timeout=5) as reply:
                self.assertIn(b"EventSource", reply.read())

            conn = http.client.HTTPConnection("127.0.0.1", port, timeout=5)
            conn.request("GET", "/events")
            reply = conn.getresponse()
            self.assertEqual(reply.getheader("Content-Type"), "text/event-stream; charset=utf-8")
            frames = []
            deadline = time.monotonic() + 5
            while len(frames) < 5 and time.monotonic() < deadline:
                line = reply.fp.readline().decode()
                if line.startswith("data: "):
                    frames.append(json.loads(line[6:]))
            conn.close()
            self.assertGreaterEqual(len(frames), 5)
            self.assertEqual(len({f["version"] for f in frames}), len(frames))
        finally:
            proc.terminate()
            try:
                proc.wait(5)
            except subprocess.TimeoutExpired:
                proc.kill()
                proc.wait()
            out, err = proc.communicate()
        self.assertEqual(proc.returncode, 0, err)
        self.assertIn("dashboard: stopped", out)
        self.assertNotIn("Traceback", err)


if __name__ == "__main__":
    unittest.main()

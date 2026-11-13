# SPDX-License-Identifier: MIT
"""dsentry.pad: SDL ranges -> ControlPacket ranges (architecture.md 7), the button map, and the
SdlPad hot-plug/read/rumble logic against a fake SDL module (plus a headless real-pygame smoke)."""

import types
import unittest

import _support as S

from dsentry import pad as PD
from dsentry import protocol as P
from dsentry import settings as ST

PD._prepare_sdl_environment()          # headless, and no pygame banner in the test output
try:
    import pygame  # noqa: F401
    HAVE_PYGAME = True
except ImportError:      # pragma: no cover
    HAVE_PYGAME = False

SETTINGS = ST.Settings.load(S.CONFIG_H)
# architecture.md 7: spec bits 0..11 <- SDL [A, B, X, Y, LEFTSHOULDER, RIGHTSHOULDER, DPAD_UP,
# DPAD_DOWN, DPAD_LEFT, DPAD_RIGHT, START, BACK]; research measured those SDL ids with pygame 2.6.1.
CONTRACT_SDL_NAMES = ["A", "B", "X", "Y", "LEFTSHOULDER", "RIGHTSHOULDER", "DPAD_UP", "DPAD_DOWN",
                      "DPAD_LEFT", "DPAD_RIGHT", "START", "BACK"]
RESEARCH_SDL_IDS = [0, 1, 2, 3, 9, 10, 11, 12, 13, 14, 6, 4]
# protocol_spec.py bits 12..15 <- SDL LEFTSTICK, RIGHTSTICK, GUIDE, MISC1 (SDL_gamecontroller.h
# 2.28: 7, 8, 5, 15; pygame 2.6.1 has no constant for MISC1, the Series X|S Share button).
EXTRA_SDL_NAMES = ["LEFTSTICK", "RIGHTSTICK", "GUIDE", "MISC1"]
EXTRA_SDL_IDS = [7, 8, 5, 15]
SDL_AXES = {"CONTROLLER_AXIS_LEFTX": 0, "CONTROLLER_AXIS_LEFTY": 1, "CONTROLLER_AXIS_RIGHTX": 2,
            "CONTROLLER_AXIS_RIGHTY": 3, "CONTROLLER_AXIS_TRIGGERLEFT": 4, "CONTROLLER_AXIS_TRIGGERRIGHT": 5}


class ScalingTests(unittest.TestCase):
    def setUp(self):
        self.sc = PD.Scaler.from_settings(SETTINGS)

    def test_stick_ends_are_exact(self):
        # architecture.md 7: sticks -32768..32767 -> >> 6 -> -512..511
        self.assertEqual(self.sc.stick_shift, 6)
        self.assertEqual(self.sc.stick(-32768), -512)
        self.assertEqual(self.sc.stick(0), 0)
        self.assertEqual(self.sc.stick(32767), 511)
        self.assertEqual(self.sc.stick(-1), -1)           # arithmetic shift, not truncation

    def test_trigger_ends_are_exact(self):
        self.assertEqual(self.sc.trigger_shift, 5)
        self.assertEqual(self.sc.trigger(0), 0)
        self.assertEqual(self.sc.trigger(31), 0)
        self.assertEqual(self.sc.trigger(32), 1)
        self.assertEqual(self.sc.trigger(32767), SETTINGS.trigger_axis_range)

    def test_out_of_range_input_is_clamped(self):
        self.assertEqual(self.sc.stick(40000), 511)
        self.assertEqual(self.sc.stick(-40000), -512)
        self.assertEqual(self.sc.trigger(-5), 0)
        self.assertEqual(self.sc.trigger(99999), SETTINGS.trigger_axis_range)

    def test_sweep_is_monotonic_and_covers_the_wire_range(self):
        values = [self.sc.stick(v) for v in range(-32768, 32768, 37)] + [self.sc.stick(32767)]
        self.assertEqual(values, sorted(values))
        self.assertEqual((values[0], values[-1]), (-512, 511))

    def test_shift_follows_config(self):
        sc = PD.Scaler(256, 2047)
        self.assertEqual((sc.stick_shift, sc.trigger_shift), (7, 4))
        self.assertEqual(sc.stick(32767), 255)
        self.assertEqual(sc.trigger(32767), 2047)

    def test_non_power_of_two_ratio_refused(self):
        with self.assertRaises(ValueError):
            PD.Scaler(500, 1023)
        with self.assertRaises(ValueError):
            PD.Scaler(512, 999)


class ButtonMapTests(unittest.TestCase):
    def test_bits_in_contract_order(self):
        self.assertEqual([mask for mask, _ in PD.BUTTON_MAP], [1 << i for i in range(12)])
        self.assertEqual([name for _, name in PD.BUTTON_MAP],
                         ["CONTROLLER_BUTTON_" + n for n in CONTRACT_SDL_NAMES])
        self.assertEqual(PD.BUTTON_MAP[0][0], P.DS_BTN_A)
        self.assertEqual(PD.BUTTON_MAP[11][0], P.DS_BTN_BACK)

    @unittest.skipUnless(HAVE_PYGAME, "pygame not installed")
    def test_sdl_ids_match_research(self):
        import pygame
        self.assertEqual([getattr(pygame, name) for _, name in PD.BUTTON_MAP], RESEARCH_SDL_IDS)

    def test_buttons_from_pressed_ids(self):
        pressed = {RESEARCH_SDL_IDS[4], RESEARCH_SDL_IDS[10]}      # LEFTSHOULDER, START
        self.assertEqual(PD.buttons_from(lambda i: i in pressed, RESEARCH_SDL_IDS), P.DS_BTN_LB | P.DS_BTN_START)
        self.assertEqual(PD.buttons_from(lambda i: False, RESEARCH_SDL_IDS), 0)
        self.assertEqual(PD.buttons_from(lambda i: True, RESEARCH_SDL_IDS), (1 << 12) - 1)

    def test_extra_buttons_are_bits_12_to_15(self):
        self.assertEqual([mask for mask, _ in PD.EXTRA_BUTTON_MAP], [1 << i for i in range(12, 16)])
        self.assertEqual([mask for mask, _ in PD.EXTRA_BUTTON_MAP],
                         [P.DS_BTN_LS_CLICK, P.DS_BTN_RS_CLICK, P.DS_BTN_XBOX, P.DS_BTN_SHARE])
        self.assertEqual([name for _, name in PD.EXTRA_BUTTON_MAP],
                         ["CONTROLLER_BUTTON_" + n for n in EXTRA_SDL_NAMES])
        self.assertEqual(PD.buttons_from(lambda i: True, EXTRA_SDL_IDS, PD.EXTRA_BUTTON_MAP), 0xF000)
        self.assertEqual(PD.buttons_from(lambda i: i == 15, EXTRA_SDL_IDS, PD.EXTRA_BUTTON_MAP), P.DS_BTN_SHARE)
        # a button this SDL does not have (id None) is never pressed
        self.assertEqual(PD.buttons_from(lambda i: True, [None, 8, None, None], PD.EXTRA_BUTTON_MAP),
                         P.DS_BTN_RS_CLICK)

    @unittest.skipUnless(HAVE_PYGAME, "pygame not installed")
    def test_extra_sdl_ids_with_the_misc1_fallback(self):
        import pygame
        self.assertEqual(PD.sdl_button_ids(pygame, PD.EXTRA_BUTTON_MAP), EXTRA_SDL_IDS)
        self.assertLess(15, pygame.CONTROLLER_BUTTON_MAX)     # SDL accepts the MISC1 id
        self.assertEqual([getattr(pygame, n) for n in SDL_AXES], list(SDL_AXES.values()))


# ---- a fake pygame + pygame._sdl2.controller ------------------------------------------------
ADDED, REMOVED = 0x653, 0x654


class FakeController:
    def __init__(self, sdl, index):
        self.sdl = sdl
        self.index = index
        self.name = "Fake Xbox pad"
        self.iid = sdl.next_iid
        sdl.next_iid += 1
        self.closed = False

    def as_joystick(self):
        return types.SimpleNamespace(get_instance_id=lambda: self.iid, get_power_level=self._power)

    def _power(self):
        self.sdl.power_reads += 1
        return self.sdl.power

    def attached(self):
        return not self.closed and self.sdl.plugged

    def get_axis(self, axis):
        if self.sdl.fail:
            raise RuntimeError("SDL read failed")
        return self.sdl.axes.get(axis, 0)

    def get_button(self, button):
        return button in self.sdl.buttons

    def rumble(self, lo, hi, ms):
        self.sdl.rumbles.append((lo, hi, ms))
        return True

    def stop_rumble(self):
        self.sdl.rumbles.append("stop")

    def quit(self):
        self.closed = True


class FakeSdl:
    """Stands in for (pygame, pygame._sdl2.controller)."""

    def __init__(self, count=0):
        self.count = count
        self.plugged = count > 0
        self.events = []
        self.axes = {}
        self.buttons = set()
        self.rumbles = []
        self.fail = False
        self.next_iid = 7
        self.power = "medium"
        self.power_reads = 0
        names = dict(SDL_AXES)
        for (_, name), sdl_id in zip(PD.BUTTON_MAP, RESEARCH_SDL_IDS):
            names[name] = sdl_id
        for name, sdl_id in zip(EXTRA_SDL_NAMES[:3], EXTRA_SDL_IDS[:3]):   # like pygame: no MISC1
            names["CONTROLLER_BUTTON_" + name] = sdl_id
        self.pygame = types.SimpleNamespace(
            CONTROLLERDEVICEADDED=ADDED, CONTROLLERDEVICEREMOVED=REMOVED,
            display=types.SimpleNamespace(init=lambda: None),
            joystick=types.SimpleNamespace(init=lambda: None),
            event=types.SimpleNamespace(get=self._take_events),
            quit=lambda: None, **names)
        self.gc = types.SimpleNamespace(init=lambda: None, quit=lambda: None, get_count=lambda: self.count,
                                        is_controller=lambda i: i < self.count,
                                        Controller=lambda i: FakeController(self, i))

    def _take_events(self):
        out, self.events = self.events, []
        return out

    def plug(self):
        self.count, self.plugged = 1, True
        self.events.append(types.SimpleNamespace(type=ADDED, device_index=0))

    def unplug(self, iid):
        self.count, self.plugged = 0, False
        self.events.append(types.SimpleNamespace(type=REMOVED, instance_id=iid))


class SdlPadTests(unittest.TestCase):
    def make(self, count=0):
        sdl = FakeSdl(count)
        log = []
        pad = PD.SdlPad(PD.Scaler.from_settings(SETTINGS), modules=(sdl.pygame, sdl.gc), log=log.append)
        return sdl, pad, log

    def test_hot_plug_read_and_remove(self):
        sdl, pad, log = self.make()
        self.assertEqual(pad.poll(0.0), PD.NEUTRAL)
        sdl.plug()
        sdl.axes = {2: 32767, 3: -32768, 5: 32767}
        sdl.buttons = {RESEARCH_SDL_IDS[0], RESEARCH_SDL_IDS[8]}        # A, DPAD_LEFT
        st = pad.poll(0.01)
        self.assertTrue(st.connected)
        self.assertEqual((st.rx, st.ry, st.rt), (511, -512, SETTINGS.trigger_axis_range))   # Y sent raw: up negative
        self.assertEqual(st.buttons, P.DS_BTN_A | P.DS_BTN_DPAD_LEFT)
        sdl.events.append(types.SimpleNamespace(type=REMOVED, instance_id=999))            # some other device
        self.assertTrue(pad.poll(0.02).connected)
        sdl.unplug(7)
        self.assertEqual(pad.poll(0.03), PD.NEUTRAL)
        self.assertTrue(any("disconnected" in line for line in log))

    def test_full_state_for_the_controller_view(self):
        sdl, pad, _ = self.make(count=1)
        sdl.axes = {0: -32768, 1: 32767, 2: 0, 3: 0, 4: 32767, 5: 0}    # left stick, left trigger
        sdl.buttons = {7, 5, 15, RESEARCH_SDL_IDS[1]}                      # LS click, guide, MISC1, B
        st = pad.poll(0.0)
        self.assertEqual((st.lx, st.ly, st.lt), (-512, 511, SETTINGS.trigger_axis_range))
        self.assertEqual((st.rx, st.ry, st.rt), (0, 0, 0))
        self.assertEqual(st.buttons, P.DS_BTN_B | P.DS_BTN_LS_CLICK | P.DS_BTN_XBOX | P.DS_BTN_SHARE)
        self.assertEqual(st.power, "medium")
        self.assertIsNone(st.battery)                    # SDL gives a level, not a percent
        self.assertEqual(st.name, "Fake Xbox pad")

    def test_power_level_is_read_once_a_poll_interval(self):
        sdl, pad, _ = self.make(count=1)
        for i in range(10):
            pad.poll(i * 0.01)                           # 0.1 s of 100 Hz polls
        self.assertEqual(sdl.power_reads, 1)
        sdl.power = "low"
        self.assertEqual(pad.poll(PD.POWER_POLL_S + 0.001).power, "low")
        self.assertEqual(sdl.power_reads, 2)

    def test_pad_present_at_start_is_opened(self):
        sdl, pad, _ = self.make(count=1)
        self.assertTrue(pad.poll(0.0).connected)

    def test_read_error_reads_as_disconnected_not_stuck(self):
        sdl, pad, log = self.make(count=1)
        sdl.axes = {2: 32767}
        self.assertEqual(pad.poll(0.0).rx, 511)
        sdl.fail = True
        self.assertEqual(pad.poll(0.01), PD.NEUTRAL)       # never the last stick value
        sdl.fail = False
        self.assertEqual(pad.poll(0.02), PD.NEUTRAL)       # closed until SDL reports it again
        self.assertTrue(any("read error" in line for line in log))

    def test_detached_pad_is_closed(self):
        sdl, pad, _ = self.make(count=1)
        self.assertTrue(pad.poll(0.0).connected)
        sdl.plugged = False
        self.assertEqual(pad.poll(0.01), PD.NEUTRAL)

    def test_rumble_mapping(self):
        sdl, pad, _ = self.make()
        self.assertFalse(pad.rumble(200, 120, 80))         # no pad
        sdl.plug()
        pad.poll(0.0)
        self.assertTrue(pad.rumble(200, 120, 80))
        pad.stop_rumble()
        lo, hi, ms = sdl.rumbles[0]
        self.assertAlmostEqual(lo, 200 / 255)              # strong -> low frequency
        self.assertAlmostEqual(hi, 120 / 255)              # weak -> high frequency
        self.assertEqual(ms, 80)
        self.assertEqual(sdl.rumbles[1], "stop")

    @unittest.skipUnless(HAVE_PYGAME, "pygame not installed")
    def test_real_pygame_headless(self):
        pad = PD.SdlPad(PD.Scaler.from_settings(SETTINGS))
        try:
            for _ in range(5):
                state = pad.poll(0.0)
                self.assertIsInstance(state, PD.PadState)
                if state.connected:          # a pad is plugged into this Mac: values in range
                    self.assertTrue(-512 <= state.rx <= 511 and 0 <= state.rt <= SETTINGS.trigger_axis_range)
        finally:
            pad.close()


class FakePadTests(unittest.TestCase):
    def test_records_rumble(self):
        fp = PD.FakePad(PD.PadState(True, 1, 2, 3, 4))
        self.assertEqual(fp.poll(0.0).rx, 1)
        fp.rumble(1, 2, 3)
        fp.stop_rumble()
        self.assertEqual(fp.rumbles, [("rumble", 1, 2, 3), ("stop",)])


if __name__ == "__main__":
    unittest.main()

# SPDX-License-Identifier: MIT
"""dsentry.demo: the --demo stand-in turret behaves like the contract in the ways the dashboard
shows (it is not the firmware; these tests keep the preview honest): calibration then PAIRING,
DISARMED on connect and never ARMED after a link loss, soft tilt limit, semi-auto one shot per
pull with rumble per stroke, LED brightness never above LED_MAX_BRIGHTNESS."""

import random
import unittest

import _support as S

from dsentry import demo as D
from dsentry import protocol as P
from dsentry import settings as ST

SET = ST.Settings.load(S.CONFIG_H)
C = SET.cfg
DT = 1.0 / C["CONTROL_LOOP_HZ"]
CONNECTED = P.DS_CTRL_FLAG_CONNECTED | P.DS_CTRL_FLAG_SOURCE_WIFI


class Rig:
    def __init__(self):
        self.t = 0.0
        self.turret = D.DemoTurret(C, self.t, rng=random.Random(1))
        self.seq = 0
        self.telemetry = []
        self.status = []
        self.texts = []

    def run(self, seconds, flags=CONNECTED, **fields):
        n = round(seconds / DT)
        for _ in range(n):
            if flags is not None:
                self.seq = (self.seq + 1) % 256
                self.turret.receive(P.ControlPacket(flags=flags, seq=self.seq, **fields).encode(), self.t)
            for raw in self.turret.step(self.t):
                if raw[0] == P.DS_TELEMETRY_START:
                    self.telemetry.append(P.TelemetryPacket.decode(raw))
                elif raw[0] == P.DS_STATUS_START:
                    self.status.append(P.StatusPacket.decode(raw))
                else:
                    self.texts.append(raw)
            self.t += DT

    @property
    def last(self):
        return self.telemetry[-1]

    def tap(self, button):
        self.run(0.1, buttons=button)
        self.run(0.1)


def booted():
    rig = Rig()
    rig.run(6.0, flags=None)          # calibration, no controller yet
    return rig


class DemoTests(unittest.TestCase):
    def test_calibrates_then_pairs(self):
        rig = booted()
        states = [t.state for t in rig.telemetry]
        self.assertEqual(states[0], P.DS_STATE_CALIBRATING)
        self.assertEqual(rig.last.state, P.DS_STATE_PAIRING)
        self.assertEqual(rig.last.tiltCdeg, 0)
        self.assertTrue(rig.last.flags & P.DS_TELEM_FLAG_CALIBRATED)
        self.assertIn("Calibrating", rig.telemetry[0].lcd)
        self.assertTrue(any(b"BOOT" in t for t in rig.texts))

    def test_connect_arm_and_link_loss_returns_disarmed(self):
        rig = booted()
        rig.run(0.5)
        self.assertEqual(rig.last.state, P.DS_STATE_DISARMED)
        self.assertIn("Connected", rig.last.lcd)
        rig.tap(P.DS_BTN_A)
        self.assertEqual(rig.last.state, P.DS_STATE_ARMED)
        rig.run(SET.link_timeout_s + 0.1, flags=0)                  # connected=0 packets
        self.assertEqual(rig.last.state, P.DS_STATE_PAIRING)
        rig.run(0.5)
        self.assertEqual(rig.last.state, P.DS_STATE_DISARMED)      # never ARMED after a loss

    def test_tilt_soft_limit_and_pan_sign(self):
        rig = booted()
        rig.run(0.3)
        rig.run(3.0, ry=-C["JOY_AXIS_RANGE"], rx=C["JOY_AXIS_RANGE"] - 1)   # stick up and right
        top = C["TILT_RANGE_DEG"] * P.DS_ANGLE_SCALE
        self.assertTrue(all(t.tiltCdeg <= top for t in rig.telemetry))
        self.assertGreater(rig.last.tiltCdeg, top * 0.95)
        self.assertLess(rig.last.panCdeg, 0)                       # stick right = clockwise = negative
        speeds = [abs(t.panVelCdegS) for t in rig.telemetry]
        self.assertLessEqual(max(speeds), C["PAN_MAX_DEG_PER_S"] * P.DS_ANGLE_SCALE + 1)

    def test_semi_auto_one_shot_per_pull_with_rumble(self):
        rig = booted()
        rig.run(0.3)
        rig.tap(P.DS_BTN_A)
        rig.run(C["FLYWHEEL_SPINUP_MS"] / 1000.0 + 0.2, buttons=P.DS_BTN_LB)
        self.assertTrue(rig.last.flags & P.DS_TELEM_FLAG_FLYWHEEL_READY)
        seq0 = rig.status[-1].rumbleSeq
        rig.run(1.5, buttons=P.DS_BTN_LB, rt=C["TRIGGER_AXIS_RANGE"])   # held well past a stroke
        rig.run(0.3, buttons=P.DS_BTN_LB)
        self.assertEqual(rig.last.shotsTotal, 1)
        self.assertEqual(rig.last.ammo, C["MAG_CAPACITY"] - 1)
        rig.run(0.5, buttons=P.DS_BTN_LB, rt=C["TRIGGER_AXIS_RANGE"])
        rig.run(0.3, buttons=P.DS_BTN_LB)
        self.assertEqual(rig.last.shotsTotal, 2)
        self.assertEqual((rig.status[-1].rumbleSeq - seq0) % 256, 2)
        self.assertEqual(rig.status[-1].rumbleMs, C["RUMBLE_DURATION_MS"])

    def test_no_shot_while_disarmed(self):
        rig = booted()
        rig.run(0.3)
        rig.run(1.5, buttons=P.DS_BTN_LB, rt=C["TRIGGER_AXIS_RANGE"])
        self.assertEqual(rig.last.shotsTotal, 0)
        self.assertEqual(rig.last.flywheelDuty, 0)

    def test_led_never_above_max_brightness(self):
        rig = booted()
        rig.run(0.3)
        rig.tap(P.DS_BTN_A)
        rig.run(1.0, buttons=P.DS_BTN_LB, rt=C["TRIGGER_AXIS_RANGE"])
        rig.run(0.5, buttons=P.DS_BTN_LB)
        cap = C["LED_MAX_BRIGHTNESS"]
        for t in rig.telemetry:
            self.assertLessEqual(max(t.ledR, t.ledG, t.ledB), cap)
        # the owner's ARMED colours (2026-11-03): yellow while spinning up / mid stroke, red once
        # a pull would fire
        self.assertTrue(any(t.ledR == cap and t.ledG == cap and t.ledB == 0 for t in rig.telemetry))
        self.assertTrue(any(t.ledR == cap and t.ledG == 0 for t in rig.telemetry))

    def test_empty_magazine_keeps_firing_and_flashes_purple(self):
        rig = booted()
        rig.run(0.3)
        rig.tap(P.DS_BTN_A)
        rig.tap(P.DS_BTN_B)                                          # full auto
        rig.run(C["FLYWHEEL_SPINUP_MS"] / 1000.0 + 0.2, buttons=P.DS_BTN_LB)
        stroke_s = (C["TRIGGER_FORWARD_MS"] + C["TRIGGER_RETURN_MS"]) / 1000.0
        rig.run((C["MAG_CAPACITY"] + 3) * stroke_s, buttons=P.DS_BTN_LB, rt=C["TRIGGER_AXIS_RANGE"])
        self.assertEqual(rig.last.ammo, 0)                           # never negative
        self.assertGreater(rig.last.shotsTotal, C["MAG_CAPACITY"])  # FIRE_LOCKOUT_WHEN_EMPTY false
        n = len(rig.telemetry)
        rig.run(2 * C["LED_EMPTY_FLASH_PERIOD_MS"] / 1000.0, buttons=P.DS_BTN_LB)
        idle = rig.telemetry[n:]
        self.assertTrue(all(t.flags & P.DS_TELEM_FLAG_EMPTY_MAG_BLINK for t in idle))
        purple = tuple(round(v * C["LED_MAX_BRIGHTNESS"] / C["LED_CHANNEL_MAX"]) for v in C.rgb("LED_COLOR_EMPTY_MAG"))
        self.assertIn(purple, {(t.ledR, t.ledG, t.ledB) for t in idle})
        rig.tap(P.DS_BTN_X)
        self.assertEqual(rig.last.ammo, C["MAG_CAPACITY"])
        self.assertFalse(rig.last.flags & P.DS_TELEM_FLAG_EMPTY_MAG_BLINK)

    def test_script_values_in_range(self):
        script = D.DemoScript(C, 0.0)
        r = C["JOY_AXIS_RANGE"]
        for i in range(int(D.DEMO_SCRIPT_PERIOD_S * 2 / 0.05)):
            st = script(i * 0.05)
            self.assertTrue(-r <= st.rx < r and -r <= st.ry < r and 0 <= st.rt <= C["TRIGGER_AXIS_RANGE"])
            self.assertTrue(-r <= st.lx < r and -r <= st.ly < r and 0 <= st.lt <= C["TRIGGER_AXIS_RANGE"])

    def test_script_shows_every_button_both_sticks_and_both_triggers(self):
        script = D.DemoScript(C, 0.0)
        seen, lt_max, lx_span = 0, 0, set()
        step = 0.01
        for i in range(int((D.DEMO_SCRIPT_LEAD_S + D.DEMO_SCRIPT_PERIOD_S) / step)):
            st = script(i * step)
            if st.connected:
                seen |= st.buttons
                lt_max = max(lt_max, st.lt)
                lx_span.add(st.lx > 0)
                self.assertEqual(st.battery, D.DEMO_PAD_BATTERY_PERCENT)
        self.assertEqual(seen, sum(m for _k, m in P.BIT_KEYS["Buttons"]))   # all sixteen light up
        self.assertEqual(lt_max, C["TRIGGER_AXIS_RANGE"])
        self.assertEqual(lx_span, {True, False})

    def test_telemetry_carries_the_relays_packet_as_the_controller(self):
        rig = booted()
        self.assertEqual(rig.last.padState, P.DS_PAD_STATE_NONE)          # nothing heard yet
        rig.run(0.3, rx=100, ry=-50, rt=600, buttons=P.DS_BTN_Y | P.DS_BTN_XBOX)
        t = rig.last
        self.assertEqual((t.padState, t.padRx, t.padRy, t.padRt, t.padButtons),
                         (P.DS_PAD_STATE_CONNECTED, 100, -50, 600, P.DS_BTN_Y | P.DS_BTN_XBOX))
        self.assertEqual((t.padLx, t.padLy, t.padLt, t.padBattery), (0, 0, 0, P.DS_PAD_BATTERY_UNKNOWN))
        rig.run(0.1, flags=P.DS_CTRL_FLAG_SOURCE_WIFI, rx=100)               # connected = 0
        self.assertEqual((rig.last.padState, rig.last.padRx), (P.DS_PAD_STATE_SCANNING, 0))
        rig.run(SET.link_timeout_s + 0.1, flags=None)                        # silence
        self.assertEqual((rig.last.padState, rig.last.padButtons), (P.DS_PAD_STATE_NONE, 0))


if __name__ == "__main__":
    unittest.main()

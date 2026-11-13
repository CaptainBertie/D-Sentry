# SPDX-License-Identifier: MIT
"""dsentry.hub: the dashboard's state. Telemetry is the only source of turret values; a stale,
missing or wrong-version packet is never presented as live; the event log is derived from
telemetry changes; StatusPackets feed rumble once per new rumbleSeq; the console line carries
every figure the contract lists.

Key safety property: test_stale_and_none_are_never_live fails if old telemetry could be shown
as live."""

import json
import unittest

import _support as S

from dsentry import hub as H
from dsentry import pad as PD
from dsentry import protocol as P
from dsentry import relay as R
from dsentry import settings as ST

SET = ST.Settings.load(S.CONFIG_H)
EPS = 1e-3
TF = P.TelemetryFlags


class Clock:
    def __init__(self, t=100.0):
        self.t = t

    def __call__(self):
        return self.t

    def advance(self, s):
        self.t += s


def tpkt(**kw):
    base = dict(seq=1, uptimeMs=5000, state=P.DS_STATE_DISARMED, ammo=15, magCapacity=15,
                flags=TF.LINK_CONNECTED | TF.CALIBRATED, lcd="DISARMED AMMO:15BT     Connected")
    base.update(kw)
    return P.TelemetryPacket(**base)


def frame_of(pkt):
    raw = pkt.encode()
    return P.Frame(pkt.TYPE, raw, pkt)


class EventTests(unittest.TestCase):
    def texts(self, prev, cur):
        return [t for _, t in H.telemetry_events(prev, cur)]

    def test_first_packet(self):
        self.assertIn("first packet", self.texts(None, tpkt())[0])

    def test_state_and_fault(self):
        ev = H.telemetry_events(tpkt(), tpkt(state=P.DS_STATE_FAULT, fault=P.DS_FAULT_STALL))
        self.assertIn(("safety", "state DISARMED -> FAULT (STALL)"), ev)
        self.assertIn("state DISARMED -> ARMED", self.texts(tpkt(), tpkt(state=P.DS_STATE_ARMED)))

    def test_shots_and_magazine(self):
        self.assertIn("shot #1, ammo 14", self.texts(tpkt(shotsTotal=0), tpkt(shotsTotal=1, ammo=14)))
        self.assertIn("shots #2-#4, ammo 11", self.texts(tpkt(shotsTotal=1, ammo=14), tpkt(shotsTotal=4, ammo=11)))
        self.assertIn("shot #0, ammo 14", self.texts(tpkt(shotsTotal=0xFFFF), tpkt(shotsTotal=0, ammo=14)))   # wrap
        self.assertIn("magazine empty", self.texts(tpkt(ammo=1), tpkt(ammo=0)))
        self.assertIn("magazine reset to 15", self.texts(tpkt(ammo=0), tpkt(ammo=15)))

    def test_link_home_reboot_toggles(self):
        lost = H.telemetry_events(tpkt(), tpkt(flags=TF.CALIBRATED))
        self.assertIn(("safety", "control link LOST"), lost)
        self.assertIn("home set to pan 12.50 deg, tilt 3.00 deg",
                      self.texts(tpkt(), tpkt(homePanCdeg=1250, homeTiltCdeg=300)))
        self.assertIn("RA4M1 restarted", self.texts(tpkt(uptimeMs=9000), tpkt(uptimeMs=10))[0])
        self.assertIn("mode toggle WIFI", self.texts(tpkt(), tpkt(flags=tpkt().flags | TF.MODE_TOGGLE_WIFI)))
        self.assertIn("ESP32 radio BLE -> WIFI", self.texts(tpkt(espMode=0), tpkt(espMode=1)))
        self.assertIn("fire mode SEMI -> FULL", self.texts(tpkt(), tpkt(fireMode=1)))

    def test_no_change_no_event(self):
        self.assertEqual(H.telemetry_events(tpkt(), tpkt(seq=2, uptimeMs=5050)), [])

    def test_jitter_limit(self):
        limit = SET.value("STEP_JITTER_LIMIT_PERMILLE")
        ev = H.telemetry_events(tpkt(stepJitterPermille=limit), tpkt(stepJitterPermille=limit + 1), limit)
        self.assertEqual(ev[0][0], "safety")

    def test_unknown_enum_values_are_named_not_crashing(self):
        self.assertIn("UNKNOWN(9)", self.texts(tpkt(), tpkt(state=9))[0])


class HubTests(unittest.TestCase):
    def setUp(self):
        self.clock = Clock()
        self.hub = H.Hub(SET, "test", clock=self.clock, wall=lambda: 1.0e9)

    def test_stale_and_none_are_never_live(self):
        self.assertEqual(self.hub.frame()["telemetryState"], "none")
        self.assertIsNone(self.hub.frame()["telemetry"])
        self.hub.on_frame(frame_of(tpkt()))
        self.assertEqual(self.hub.frame()["telemetryState"], "live")
        self.clock.advance(SET.stale_s - EPS)
        self.assertEqual(self.hub.frame()["telemetryState"], "live")
        self.clock.advance(2 * EPS)
        f = self.hub.frame()
        self.assertEqual(f["telemetryState"], "stale")
        self.assertGreater(f["telemetryAgeMs"], SET.stale_s * 1000)
        self.hub.tick()
        self.assertTrue(any("STALE" in e["text"] for e in self.hub.frame()["log"]))
        self.hub.on_frame(frame_of(tpkt(seq=2)))
        self.hub.tick()
        self.assertEqual(self.hub.frame()["telemetryState"], "live")
        self.assertTrue(any("live again" in e["text"] for e in self.hub.frame()["log"]))

    def test_wrong_version_is_never_shown(self):
        self.hub.on_frame(frame_of(tpkt(version=P.DS_TELEMETRY_VERSION + 1, state=P.DS_STATE_ARMED)))
        f = self.hub.frame()
        self.assertEqual(f["telemetryState"], "badVersion")
        self.assertIsNone(f["telemetry"])
        self.assertEqual(f["badVersionCount"], 1)
        self.hub.on_frame(frame_of(tpkt()))
        self.assertEqual(self.hub.frame()["telemetryState"], "live")
        self.assertEqual(self.hub.frame()["telemetry"]["stateName"], "DISARMED")

    def test_view_fields(self):
        self.hub.on_frame(frame_of(tpkt(panCdeg=-4550, tiltCdeg=1234, fireMode=1, linkSource=3)))
        t = self.hub.frame()["telemetry"]
        self.assertEqual((t["panDeg"], t["tiltDeg"]), (-45.5, 12.34))
        self.assertEqual((t["stateName"], t["fireModeName"], t["linkSourceName"]), ("DISARMED", "FULL", "USB"))
        self.assertEqual(t["lcdRows"], ["DISARMED AMMO:15", "BT     Connected"])
        self.assertTrue(t["flagBits"]["calibrated"])

    def test_status_feeds_rumble_once_per_new_seq(self):
        def status(seq, ms=80):
            pkt = P.StatusPacket(wantMode=1, rumbleStrong=200, rumbleWeak=120, rumbleMs=ms, state=5, rumbleSeq=seq)
            return frame_of(pkt)
        for f in (status(3), status(3), status(4), status(4), status(5, 0)):
            self.hub.on_frame(f)
        cmds = []
        while not self.hub.rumble_queue.empty():
            cmds.append(self.hub.rumble_queue.get())
        self.assertEqual(cmds, [("rumble", 200, 120, 80), ("stop",)])
        self.assertEqual(self.hub.frame()["status"]["wantModeName"], "WIFI")

    def test_text_and_log_after(self):
        self.hub.on_text("[2] STATE BOOT -> CALIBRATING", "serial")
        self.hub.note("relay", "port open")
        f = self.hub.frame()
        self.assertEqual(f["text"][0]["text"], "[2] STATE BOOT -> CALIBRATING")
        self.assertEqual(f["log"][0]["text"], "port open")
        last = max(f["text"][-1]["id"], f["log"][-1]["id"])
        g = self.hub.frame(log_after=last, text_after=last)
        self.assertEqual((g["log"], g["text"]), ([], []))

    def test_log_is_capped(self):
        for i in range(SET.event_log_keep * 2):
            self.hub.note("relay", "n%d" % i)
        self.assertEqual(len(self.hub.frame()["log"]), SET.event_log_keep)

    def test_version_rises_and_wait_returns(self):
        v = self.hub.version
        self.hub.note("relay", "x")
        self.assertEqual(self.hub.wait_for_change(v, 0.01), v + 1)
        self.assertEqual(self.hub.wait_for_change(v + 1, 0.01), v + 1)     # timeout, no change

    def test_turret_crc_errors_logged_at_most_once_per_stats_window(self):
        self.hub.on_frame(frame_of(tpkt(crcErrors=0)))
        self.clock.advance(SET.stats_s + EPS)
        self.hub.on_frame(frame_of(tpkt(crcErrors=3, uptimeMs=6000)))
        self.hub.on_frame(frame_of(tpkt(crcErrors=5, uptimeMs=6050)))
        lines = [e["text"] for e in self.hub.frame()["log"] if "CRC" in e["text"]]
        self.assertEqual(len(lines), 1)
        self.assertIn("3 CRC errors", lines[0])

    def test_frame_is_json(self):
        self.hub.on_frame(frame_of(tpkt()))
        json.dumps(self.hub.frame())

    def test_console_line(self):
        stats = R.LinkStats(SET, clock=self.clock)

        class Tr:
            sends, expected_link_source, kind = True, P.DS_LINK_SRC_USB, "serial"

            def status(self):
                return {"kind": "serial", "up": True, "detail": "serial /dev/x @ 115200", "crcErrors": 4}
        self.hub.attach(stats=stats, transport=Tr())
        stats.on_send(9, True)
        self.clock.advance(0.02)
        self.hub.on_frame(frame_of(tpkt(crcErrors=2, lastCtrlSeq=9, linkSource=P.DS_LINK_SRC_USB)))
        line = H.console_line(self.hub)
        for part in ("tx ", "telemetry ", "turret crc 2", "relay crc 4", "latency 20.0 ms", "serial /dev/x"):
            self.assertIn(part, line)


# ---- telemetry v1 / v2 and the controller view ---------------------------------------------
class UsbTransport:
    """A sending serial transport: the turret obeying it reports linkSource USB."""
    sends, expected_link_source, kind = True, P.DS_LINK_SRC_USB, "serial"

    def status(self):
        return {"kind": "serial", "up": True, "detail": "serial (test)", "crcErrors": 0}


def v1_frame(**kw):
    """A version 1 TelemetryPacket (an older RA4M1 build) as the relay's deframer delivers it."""
    kw.setdefault("version", 1)
    raw = P._frame(P.DS_TELEMETRY_START, tpkt(**kw).payload()[:P.DS_TELEMETRY_V1_LEN])
    return P.Deframer(accept_legacy=True).feed(raw)[0]


BLE_PAD = dict(linkSource=P.DS_LINK_SRC_BLE, padState=P.DS_PAD_STATE_CONNECTED, padBattery=62,
               padLx=-100, padLy=200, padRx=300, padRy=-400, padLt=10, padRt=1023,
               padButtons=P.DS_BTN_A | P.DS_BTN_XBOX | P.DS_BTN_SHARE)


class FakeMerger:
    def __init__(self, source=None, values=None):
        self.source, self.values = source, values

    def last_input(self):
        return self.source, self.values


class FakeSender:
    def __init__(self, source=None, hold_s=0.0, deadman=False):
        self.snap = R.InputSnapshot(source, source is not None and not hold_s, source == "keyboard",
                                    0, 0, 0, 0, hold_s, deadman)

    def latest(self):
        return R.packet_for(self.snap, 1), self.snap


class ControllerViewTests(unittest.TestCase):
    def setUp(self):
        self.clock = Clock()
        self.hub = H.Hub(SET, "test", clock=self.clock, wall=lambda: 1.0e9)

    def test_v2_block_from_the_turret(self):
        self.hub.on_frame(frame_of(tpkt(**BLE_PAD)))
        f = self.hub.frame()
        t = f["telemetry"]
        self.assertTrue(t["controllerReported"])
        self.assertEqual((t["padStateName"], t["padBattery"], t["padLx"]), ("CONNECTED", 62, -100))
        self.assertTrue(t["padButtonBits"]["xbox"] and t["padButtonBits"]["share"])
        c = f["controller"]
        self.assertEqual((c["origin"], c["label"], c["reported"], c["live"]),
                         ("turret", "pad on the turret (BLE)", True, True))
        self.assertEqual((c["lx"], c["ly"], c["rx"], c["ry"], c["lt"], c["rt"]), (-100, 200, 300, -400, 10, 1023))
        self.assertEqual((c["stateName"], c["stateText"], c["batteryText"]), ("CONNECTED", "connected", "62 %"))
        self.assertEqual(c["buttons"], P.DS_BTN_A | P.DS_BTN_XBOX | P.DS_BTN_SHARE)
        self.assertTrue(c["buttonBits"]["a"] and not c["buttonBits"]["b"])
        json.dumps(f)

    def test_v1_is_shown_and_its_controller_is_not_reported(self):
        frame = v1_frame(state=P.DS_STATE_ARMED, ammo=7)
        self.assertEqual(len(frame.raw), P.DS_TELEMETRY_V1_FRAME_LEN)
        self.hub.on_frame(frame)
        f = self.hub.frame()
        self.assertEqual(f["telemetryState"], "live")              # not a version mismatch
        self.assertEqual((f["telemetry"]["stateName"], f["telemetry"]["ammo"]), ("ARMED", 7))
        self.assertFalse(f["telemetry"]["controllerReported"])
        self.assertIsNone(f["telemetry"]["padState"])               # defaults are not facts
        c = f["controller"]
        self.assertEqual((c["origin"], c["reported"]), ("turret", False))
        self.assertIn("not reported by this firmware", c["note"])
        self.assertEqual((c["rx"], c["buttons"]), (0, 0))
        self.assertTrue(any("older RA4M1 build" in e["text"] for e in f["log"]))
        self.assertEqual(f["badVersionCount"], 0)

    def test_version_byte_must_match_the_layout(self):
        self.hub.on_frame(v1_frame(version=2))                     # v1 length, says v2
        self.assertEqual(self.hub.frame()["telemetryState"], "badVersion")
        self.hub.on_frame(frame_of(tpkt(version=1)))                # v2 length, says v1
        self.assertEqual(self.hub.frame()["badVersionCount"], 2)
        self.assertIsNone(self.hub.frame()["telemetry"])

    def test_turret_states_and_battery_texts(self):
        cases = [
            (dict(BLE_PAD, padState=P.DS_PAD_STATE_SCANNING, padBattery=P.DS_PAD_BATTERY_UNKNOWN),
             "scanning, no pad connected", "unknown"),
            (dict(BLE_PAD, padState=P.DS_PAD_STATE_WAITING_FIRST_REPORT, padBattery=100),
             "linked, waiting for the first report", "100 %"),
            (dict(BLE_PAD, padState=P.DS_PAD_STATE_OFF, padBattery=101), "Xbox BLE client off", "invalid (101)"),
            (dict(linkSource=P.DS_LINK_SRC_USB, padState=P.DS_PAD_STATE_SCANNING), "not connected", "unknown"),
            (dict(linkSource=P.DS_LINK_SRC_WIFI, padState=P.DS_PAD_STATE_CONNECTED), "connected", "unknown"),
            (dict(linkSource=P.DS_LINK_SRC_NONE, padState=P.DS_PAD_STATE_NONE), "no control source", "unknown"),
        ]
        for fields, state_text, battery_text in cases:
            with self.subTest(fields=fields):
                c = H.turret_controller_view(tpkt(**fields), "live")
                self.assertEqual((c["stateText"], c["batteryText"]), (state_text, battery_text))
        self.assertEqual(H.turret_controller_view(tpkt(linkSource=P.DS_LINK_SRC_USB), "live")["label"],
                         "USB ControlPackets (as the turret sees it)")

    def test_turret_block_goes_grey_when_stale_and_says_why_when_missing(self):
        c = H.turret_controller_view(None, "none")
        self.assertEqual((c["reported"], c["note"], c["live"]), (False, "no telemetry yet", False))
        self.hub.on_frame(frame_of(tpkt(**BLE_PAD)))
        self.clock.advance(SET.stale_s + EPS)
        c = self.hub.frame()["controller"]
        self.assertEqual((c["origin"], c["live"], c["rx"]), ("turret", False, 300))   # last known, not live

    def test_the_relays_own_pad_wins_while_it_drives(self):
        pad = PD.PadState(True, 10, -20, 30, P.DS_BTN_B | P.DS_BTN_LS_CLICK, "Xbox Wireless Controller",
                          lx=-512, ly=511, lt=700, power="medium")
        self.hub.attach(merger=FakeMerger("pad", pad), sender=FakeSender("pad"), transport=UsbTransport())
        obeying = dict(BLE_PAD, linkSource=P.DS_LINK_SRC_USB)     # the turret follows this relay
        self.hub.on_frame(frame_of(tpkt(**obeying)))                # so its block is not shown
        c = self.hub.frame()["controller"]
        self.assertEqual((c["origin"], c["source"], c["label"], c["name"]),
                         ("relay", "pad", "relay pad", "Xbox Wireless Controller"))
        self.assertEqual((c["lx"], c["ly"], c["lt"], c["rx"], c["ry"], c["rt"]), (-512, 511, 700, 10, -20, 30))
        self.assertEqual(c["buttons"], P.DS_BTN_B | P.DS_BTN_LS_CLICK)
        self.assertEqual((c["batteryText"], c["battery"], c["live"]), ("medium (<= 70 %)", None, True))
        self.assertEqual(c["sent"], ["rx", "ry", "rt", "buttons"])

    def test_a_relay_the_turret_ignores_does_not_hide_the_turrets_pad(self):
        # Review finding: the turret drives from its BLE pad (e.g. USB control switched off) while
        # this relay also has a pad. The panel must show the pad that is really in control.
        pad = PD.PadState(True, 10, -20, 30, P.DS_BTN_B, "Xbox Wireless Controller")
        self.hub.attach(merger=FakeMerger("pad", pad), sender=FakeSender("pad"), transport=UsbTransport())
        self.hub.on_frame(frame_of(tpkt(**BLE_PAD)))                # linkSource BLE, not USB
        c = self.hub.frame()["controller"]
        self.assertEqual((c["origin"], c["label"]), ("turret", "pad on the turret (BLE)"))
        self.assertIn("another source", c["note"])
        # With no live telemetry the relay's own pad is shown, but flagged as unconfirmed.
        self.clock.advance(SET.stale_s + EPS)
        c = self.hub.frame()["controller"]
        self.assertEqual((c["origin"], c["level"]), ("relay", "warn"))
        self.assertIn("not confirmed", c["stateText"])

    def test_keyboard_demo_and_hold(self):
        kb = PD.PadState(True, 256, 0, 1023, P.DS_BTN_A, "dashboard keyboard")
        self.hub.attach(merger=FakeMerger("keyboard", kb), sender=FakeSender("keyboard", deadman=True))
        c = self.hub.frame()["controller"]
        self.assertEqual((c["label"], c["batteryText"]), ("keyboard", "n/a (keyboard)"))
        self.assertIn("deadman", c["stateText"])
        demo = PD.PadState(True, name="demo script", lx=5, battery=76)
        self.hub.attach(merger=FakeMerger("demo", demo), sender=FakeSender("demo", hold_s=0.3))
        c = self.hub.frame()["controller"]
        self.assertEqual((c["origin"], c["label"], c["batteryText"], c["lx"]),
                         ("relay", "demo script (scripted pad)", "76 %", 5))
        self.assertIn("connected=0 for 300 ms", c["stateText"])

    def test_no_relay_input_falls_back_to_the_turret(self):
        self.hub.attach(merger=FakeMerger(None, PD.NEUTRAL), sender=FakeSender(None))
        self.hub.on_frame(frame_of(tpkt(**BLE_PAD)))
        self.assertEqual(self.hub.frame()["controller"]["origin"], "turret")
        # --monitor: no sender at all
        mon = H.Hub(SET, "monitor", clock=self.clock)
        mon.on_frame(frame_of(tpkt(**BLE_PAD)))
        self.assertEqual(mon.frame()["controller"]["label"], "pad on the turret (BLE)")

    def test_a_moving_left_stick_is_a_change_for_the_page(self):
        merger = FakeMerger("pad", PD.PadState(True, name="pad", lx=0))
        self.hub.attach(merger=merger, sender=FakeSender("pad"))
        self.hub.tick()
        v = self.hub.version
        self.hub.tick()
        self.assertEqual(self.hub.version, v)                       # nothing changed
        merger.values = merger.values._replace(lx=300)              # not in the ControlPacket
        self.hub.tick()
        self.assertEqual(self.hub.version, v + 1)


if __name__ == "__main__":
    unittest.main()

# SPDX-License-Identifier: MIT
"""BT mode and the mode toggle (spec 3.1, 3.2, 11.3, 11.14, 11.17): the ESP32's restart into the
other radio (new bootId, radio-less boot plus the one-shot RTC request, the passthrough silent,
the ROM banner on the RA4M1's Serial), BLE with the sim's fake pad, and the loop/packet-loss stats
over 60 s with both steppers moving. The USB side is read through the sim's pty with the relay's
own SerialTransport, exactly as `relay --serial <pty>` would."""

import dataclasses
import threading
import unittest

import _sim as S
from _sim import P, cfg, flag, rows, lcd_row, state_is

AXIS = cfg("JOY_AXIS_RANGE")


class Usb:
    """The PC's end of the RA4M1's USB port: relay transports' SerialTransport on the pty."""

    def __init__(self, sim) -> None:
        self.sink = S.Collector()
        settings = dataclasses.replace(S.SET, control_hz=S.SET.control_hz * sim.speed)
        self.tr = S.T.SerialTransport(settings, self.sink, port=sim.pty_path, read_only=True)
        self.stop = threading.Event()
        self.tr.start(self.stop)

    def close(self) -> None:
        self.stop.set()
        self.tr.join(2)


class ModeSwitchTests(unittest.TestCase):
    SPEED = 5
    ESP_BOOT_MS = 400

    def test_mode_toggle_restarts_the_esp32_into_the_other_radio(self):
        sim = S.Sim("--pty", "--fake-pad", "--esp-boot-ms", str(self.ESP_BOOT_MS), speed=self.SPEED)
        self.addCleanup(lambda: self.assertEqual(sim.stop(), 0))
        drv = S.Driver(sim)
        self.addCleanup(drv.close)
        usb = Usb(sim)
        self.addCleanup(usb.close)

        S.boot_to_pairing(self, sim, drv)
        m = drv.sink.mark()
        drv.set(connected=True)
        drv.wait(state_is("DISARMED"), 1.0, m)
        usb.sink.wait(lambda t: t.state == P.DS_STATE_DISARMED and t.linkSource == P.DS_LINK_SRC_WIFI,
                      sim.wall(1.0), what="USB telemetry DISARMED over WiFi")
        st0 = sim.status()
        self.assertEqual((st0["espMode"], st0["sock"], st0["restarts"]), ("WIFI", "bound", "0"))
        boot0 = int(st0["bootId"])

        # Flip the toggle to BT: the link counts as lost, PAIRING, StatusPacket.wantMode = BLE.
        out = sim.out.mark()
        err = sim.err.mark()
        st_mark = len(usb.sink.status)
        um = usb.sink.mark()
        udp_mark = drv.sink.mark()
        sim.command("mode bt", r"mode toggle bt")
        lost_ms, _, i = sim.wait_log(r"link lost: mode toggle", 1.0, out)
        sim.wait_log(r"state PAIRING", 0.5, i)
        usb.sink.wait(lambda t: True, sim.wall(0.5), um)
        sim.sleep(0.2)
        want = [s.wantMode for s in usb.sink.status[st_mark:]]
        self.assertIn(P.DS_RADIO_BLE, want, "no USB StatusPacket with wantMode BLE")
        # The ESP32: restart requested, reply flushed, ESP_RESTART_DELAY_MS, esp_restart(), boot
        # radio-less with the request from RTC memory, and a new boot id.
        sim.err.wait(r"esp32: restart into BLE requested", sim.wall(1.0), err)
        m_rst, j = sim.err.wait(r"t=(\S+) esp32: esp_restart\(\)", sim.wall(1.0), err)
        m_boot, _ = sim.err.wait(r"t=(\S+) esp32: boot \(software reset\), boot id (\d+), radio BLE", sim.wall(2.0), j)
        boot1 = int(m_boot.group(2))
        self.assertNotEqual(boot1, boot0)
        self.assertNotEqual(boot1, 0)
        down_ms = (float(m_boot.group(1)) - float(m_rst.group(1))) * 1000.0
        self.assertAlmostEqual(down_ms, self.ESP_BOOT_MS, delta=2)
        # The RA4M1 sees it on the AT link: silence, pings, then a reply with the new boot id.
        sim.wait_log(r"link: ESP32 silent for LINK_TIMEOUT_MS: pinging", 2.0, out)
        sim.wait_log(r"link: ESP32 rebooted \(bootId %d -> %d\)" % (boot0, boot1), 2.0, out)
        sim.wait_log(r"link: ESP32 radio BLE", 2.0, out)
        # The ROM banner after a software reset reaches the RA4M1's shell (UART0 = its Serial RX);
        # its "rst:0xc (RTC_SW_CPU_RST),..." line is longer than SHELL_LINE_MAX, so it is dropped.
        sim.wait_log(r"unknown command: ESP-ROM:esp32s3-20210327", 2.0, out)
        # In BT mode with no pad: PAIRING, mode label BT, espMode BLE; WiFi is off.
        t = usb.sink.wait(lambda t: t.espMode == P.DS_ESP_MODE_BLE and rows(t)[1].startswith(cfg("LCD_TXT_MODE_BT") + " "),
                          sim.wall(3.0), um, "USB telemetry in BT mode")
        self.assertEqual(t.state, P.DS_STATE_PAIRING)
        self.assertFalse(flag(t, P.DS_TELEM_FLAG_MODE_TOGGLE_WIFI))
        st1 = sim.status()
        self.assertEqual((st1["espMode"], st1["sock"], st1["restarts"]), ("BLE", "closed", "1"))
        self.assertEqual(st1["wifi"], "IDLE")
        # UDP telemetry stopped with the WiFi radio.
        n_udp = len(drv.sink.since(udp_mark))
        sim.sleep(1.0)
        self.assertEqual(len(drv.sink.since(udp_mark)), n_udp)
        # The relay's packets cannot reach a turret in BT mode.
        self.assertEqual(st1["ctrlConnected"], "0")

        # BT mode: the pad pairs with the turret itself; reconnection lands in DISARMED.
        um = usb.sink.mark()
        sim.command("pad on", r"pad on ")
        t = usb.sink.wait(lambda t: t.state == P.DS_STATE_DISARMED, sim.wall(1.0), um, "DISARMED over BLE")
        self.assertEqual(t.linkSource, P.DS_LINK_SRC_BLE)
        t = usb.sink.wait(lambda t: rows(t)[1] == lcd_row(cfg("LCD_TXT_MODE_BT"), cfg("LCD_TXT_CONNECTED")),
                          sim.wall(1.0), um)
        # The pad's X loads the magazine and A arms; firing a stroke sends a rumble report to the pad
        # over BLE (rumble_report()).
        um = usb.sink.mark()
        for buttons in (P.DS_BTN_X, 0):
            sim.command("pad set 0 0 0 %d" % buttons, r"pad on ")
            sim.sleep(0.15)
        usb.sink.wait(lambda t: t.ammo == cfg("MAG_CAPACITY"), sim.wall(1.0), um, "magazine loaded over BLE")
        for buttons in (P.DS_BTN_A, 0):
            sim.command("pad set 0 0 0 %d" % buttons, r"pad on ")
            sim.sleep(0.15)
        usb.sink.wait(lambda t: t.state == P.DS_STATE_ARMED, sim.wall(1.0), um, "ARMED over BLE")
        err = sim.err.mark()
        sim.command("pad set 0 0 0 %d" % P.DS_BTN_LB, r"pad on ")
        sim.sleep((cfg("FLYWHEEL_SPINUP_MS") + 100) / 1000.0)
        sim.command("pad set 0 0 %d %d" % (cfg("TRIGGER_AXIS_RANGE"), P.DS_BTN_LB), r"pad on ")
        usb.sink.wait(lambda t: t.shotsTotal == 1, sim.wall(S.STROKE_S + 1.0), um, "a stroke over BLE")
        m_r, _ = sim.err.wait(r"esp32: pad rumble report ([0-9a-f]+)", sim.wall(1.0), err)
        report = bytes.fromhex(m_r.group(1))
        self.assertEqual(len(report), 8)
        self.assertEqual(report[0] & 0x03, 0x03)   # both grip motors enabled
        self.assertGreater(report[3], 0)            # strong motor power
        sim.command("pad set 0 0 0 0", r"pad on ")

        # And back to WiFi: a second restart, a new boot id, the relay drives it again.
        out = sim.out.mark()
        err = sim.err.mark()
        m = drv.sink.mark()
        sim.command("mode wifi", r"mode toggle wifi")
        sim.wait_log(r"link lost: mode toggle", 1.0, out)
        m_boot, _ = sim.err.wait(r"esp32: boot \(software reset\), boot id (\d+), radio WIFI", sim.wall(2.0), err)
        self.assertNotIn(int(m_boot.group(1)), (0, boot1))
        t = drv.wait(lambda t: t.state == P.DS_STATE_DISARMED and t.linkSource == P.DS_LINK_SRC_WIFI,
                     cfg("LINK_RECONNECT_GRACE_MS") / 1000.0 + 2.0, m, "DISARMED over WiFi again")
        self.assertEqual(t.espMode, P.DS_ESP_MODE_WIFI)
        self.assertNotIn(P.DS_STATE_ARMED, [x.state for x in drv.sink.since(m)])
        self.assertEqual(sim.status()["restarts"], "2")


class BtStatsTests(unittest.TestCase):
    SPEED = 25

    def test_bt_pairing_row_and_60s_stats_with_both_axes_moving(self):
        sim = S.Sim("--mode", "bt", "--fake-pad", speed=self.SPEED)
        self.addCleanup(lambda: self.assertEqual(sim.stop(), 0))
        S.boot_to_pairing(self, sim)
        sim.sleep(0.5)   # the LCD writes one character per update
        # Spec 11.3: "BT Pairing" with the dots cycling at LCD_PAIRING_DOT_MS.
        seen = set()
        for _ in range(8):
            seen.add(sim.status()["lcd1"])
            sim.sleep(cfg("LCD_PAIRING_DOT_MS") / 2000.0)
        for r in seen:
            self.assertTrue(r.startswith(cfg("LCD_TXT_MODE_BT") + " " + cfg("LCD_TXT_PAIRING")), r)
        self.assertGreaterEqual(len(seen), 3, seen)
        out = sim.out.mark()
        sim.command("pad on", r"pad on ")
        sim.wait_log(r"state DISARMED", 1.0, out)
        sim.sleep(0.5)
        self.assertEqual(sim.status()["lcd1"], lcd_row(cfg("LCD_TXT_MODE_BT"), cfg("LCD_TXT_CONNECTED")))
        # Spec 11.17: both steppers at max speed for 60 s (pan full right; tilt end to end), with
        # the LCD and LEDs running: no packet loss, no AT poll timeout. (Step jitter is 0 here: the
        # host model's timer is ideal; the board measures the real one.)
        window_s = 60.0
        half_travel_s = cfg("TILT_RANGE_DEG") / cfg("TILT_MAX_DEG_PER_S")
        out = sim.out.mark()
        t_end = sim.status()["t"] + window_s
        up = True
        while True:
            st = sim.command("pad set %d %d 0 0" % (AXIS - 1, -AXIS if up else AXIS - 1), r"pad on ")
            if float(st.group(0).split()[1][2:]) >= t_end:
                break
            up = not up
            sim.sleep(half_travel_s)
        stats = sim.out.all(r"stats loop=(\d+) rx=(\d+) to=(\d+) crc=(\d+) gap=\d+ loss=([\d.]+)%(!?) jit=(\d+)/(\d+)pm(!?)", out)
        self.assertGreaterEqual(len(stats), window_s - 2)
        for s in stats[1:]:
            self.assertEqual(s.group(6), "", s.group(0))      # loss under PACKET_LOSS_LIMIT_PERMILLE
            self.assertLess(float(s.group(5)) * 10, cfg("PACKET_LOSS_LIMIT_PERMILLE"), s.group(0))
            self.assertEqual(s.group(3), "0", s.group(0))     # no AT poll timed out
            self.assertEqual(s.group(9), "", s.group(0))      # jitter under STEP_JITTER_LIMIT_PERMILLE
            self.assertGreaterEqual(int(s.group(2)), cfg("CONTROL_PACKET_HZ") * 0.9, s.group(0))
        st = sim.status()
        self.assertGreater(abs(float(st["panDeg"])), 0.9 * window_s * cfg("PAN_MAX_DEG_PER_S") * 0.9)


if __name__ == "__main__":
    unittest.main()

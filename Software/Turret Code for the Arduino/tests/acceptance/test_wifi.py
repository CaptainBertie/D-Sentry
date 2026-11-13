# SPDX-License-Identifier: MIT
"""WiFi mode end to end: the real RA4M1 firmware in dsentry_sim, the ESP32's real AT handlers on
the real chAT server, and the PC relay's own UdpTransport + Sender on the other side of a real
UDP socket. Spec 11 items 2-13 as far as they are observable without hardware.

Each test boots its own simulator at SPEED times real time. Every time it judges is virtual (the
RA4M1's millis() in log lines and TelemetryPacket.uptimeMs, the sim's status fields)."""

import unittest

import _sim as S
from _sim import P, cfg, flag, rows, lcd_row, scaled, state_is

SPEED = 5
FULL_RT = cfg("TRIGGER_AXIS_RANGE")
AXIS = cfg("JOY_AXIS_RANGE")
STROKE_MS = S.STROKE_MS      # one shot's whole sequence (PRE-FIRE -> ... -> PRE-FIRE), from config.h
STROKE_S = S.STROKE_S
COOLDOWN_S = S.COOLDOWN_S    # semi auto, after each shot's sequence
BOOT_AMMO = cfg("TRIGGER_AMMO_AT_BOOT")
TEL_PERIOD_MS = 1000.0 / cfg("TELEMETRY_WIFI_HZ")
SCALE = P.DS_ANGLE_SCALE


def ammo_text(n):
    return "%s%d" % (cfg("LCD_TXT_AMMO_PREFIX"), n)


def shaped(x):
    """architecture 5 Inputs: deadzone rescaled (|x|-dz)/(1-dz), then expo (1-e)s + e s^3."""
    dz, e = cfg("JOY_DEADZONE"), cfg("JOY_EXPO")
    a = abs(x)
    if a <= dz:
        return 0.0
    s = min(1.0, (a - dz) / (1.0 - dz))
    return (1.0 - e) * s + e * s ** 3


class WifiCase(unittest.TestCase):
    sim_args = ()

    def setUp(self):
        self.sim = S.Sim(*self.sim_args, speed=SPEED)
        self.addCleanup(self._stop_sim)
        self.drv = self.make_driver()
        self.addCleanup(self.drv.close)

    def make_driver(self):
        return S.Driver(self.sim)

    def _stop_sim(self):
        self.assertEqual(self.sim.stop(), 0)
        # Spec 3.8 / 11.16: no LED channel ever above LED_MAX_BRIGHTNESS, whatever the test did.
        limit = cfg("LED_MAX_BRIGHTNESS")
        for t in self.drv.sink.since(0):
            self.assertLessEqual(max(t.ledR, t.ledG, t.ledB), limit, S.describe(t))

    def connect(self):
        S.boot_to_pairing(self, self.sim, self.drv)
        m = self.drv.sink.mark()
        self.drv.set(connected=True)
        return self.drv.wait(state_is("DISARMED"), 1.0, m, "DISARMED")

    def arm(self):
        m = self.drv.sink.mark()
        self.drv.tap(P.DS_BTN_A)
        return self.drv.wait(state_is("ARMED"), 1.0, m, "ARMED")

    def load(self):
        """X: the magazine to MAG_CAPACITY and the servo to PRE-FIRE, then its chambering time."""
        m = self.drv.sink.mark()
        self.drv.tap(P.DS_BTN_X)
        t = self.drv.wait(lambda t: t.ammo == cfg("MAG_CAPACITY"), 1.0, m, "magazine loaded")
        self.assertEqual(t.servoDeg, cfg("TRIGGER_PREFIRE_ANGLE"))
        self.sim.sleep(S.CHAMBER_S)
        return t

    def spin_up(self):
        m = self.drv.sink.mark()
        self.drv.hold(P.DS_BTN_LB)
        return self.drv.wait(lambda t: flag(t, P.DS_TELEM_FLAG_FLYWHEEL_READY),
                             (cfg("FLYWHEEL_SPINUP_MS") + 500) / 1000.0, m, "flywheel ready")

    def settle(self):
        """Long enough for the LCD (LCD_REFRESH_MS, one character per update) and LEDs to catch up."""
        self.sim.sleep(0.4)
        m = self.drv.sink.mark()
        return self.drv.wait(lambda t: True, 0.5, m)


class BootTests(WifiCase):
    def setUp(self):
        self.beacon_port = S.free_udp_port()
        self.sim_args = ("--beacon", "--beacon-port", str(self.beacon_port))
        super().setUp()

    def make_driver(self):
        # No address given: the relay's transport finds the turret by its BeaconPacket.
        return S.Driver(self.sim, discovery_port=self.beacon_port)

    def test_boot_calibrates_tilt_then_pairs_with_telemetry_over_udp(self):
        sim, drv = self.sim, self.drv
        pairing_ms = S.boot_to_pairing(self, sim, drv)

        # Discovery: the beacon names the sim's socket and this boot's id.
        st = sim.status()
        self.assertEqual(drv.udp.how, "beacon")
        self.assertEqual(drv.udp.target, (S.T.LOOPBACK, sim.udp_port))
        self.assertEqual(drv.udp.beacon["bootId"], int(st["bootId"]))
        self.assertEqual(st["relay"].split(":")[0], "127.0.0.1")
        self.assertEqual(st["espMode"], "WIFI")
        # WIFI_SSID empty (owner, 2026-11-03): never join a station, go straight to the own AP.
        self.assertEqual(st["wifi"], "STA_UP" if S.cfg("WIFI_SSID") else "AP_UP")

        # Spec 3.1 / 11.2: BOOT -> CALIBRATING -> PAIRING, in that order, nothing in between.
        states = sim.states()
        self.assertEqual([n for _, n in states[:3]], ["BOOT", "CALIBRATING", "PAIRING"])
        self.assertEqual(states[2][0], pairing_ms)
        # CAL_RAM (spec 3.5): drive down TILT_CAL_MAX_TRAVEL_DEG at TILT_CAL_SPEED_DEG_PER_S, wait
        # TILT_CAL_SETTLE_MS, then lift TILT_CAL_BACKOFF_DEG; PAIRING only after all of it.
        self.assertEqual(cfg("TILT_CAL_MODE"), cfg("CAL_RAM"))
        drive_ms = 1000.0 * cfg("TILT_CAL_MAX_TRAVEL_DEG") / cfg("TILT_CAL_SPEED_DEG_PER_S") + cfg("TILT_CAL_SETTLE_MS")
        lift_ms = 1000.0 * cfg("TILT_CAL_BACKOFF_DEG") / min(cfg("TILT_CAL_SPEED_DEG_PER_S"), cfg("HOME_SPEED_DEG_PER_S"))
        self.assertGreaterEqual(pairing_ms - states[1][0], drive_ms)
        self.assertLess(pairing_ms - states[1][0], drive_ms + lift_ms + 1000)

        # The barrel physically sits TILT_CAL_BACKOFF_DEG above the bottom stop it rammed, and the
        # firmware calls that tilt 0 (calibrated flag set).
        one_step = 1.0 / cfg("TILT_STEPS_PER_DEG")
        # Calibration done, the barrel then lifts TILT_BOOT_RAISE_DEG on its own (owner, 2026-11-04).
        raise_deg = cfg("TILT_BOOT_RAISE_DEG")
        self.assertGreaterEqual(float(st["tiltAboveStopDeg"]), cfg("TILT_CAL_BACKOFF_DEG") - 2 * one_step)
        self.assertLessEqual(float(st["tiltAboveStopDeg"]), cfg("TILT_CAL_BACKOFF_DEG") + raise_deg + 2 * one_step)
        p = drv.wait(lambda t: t.tiltVelCdegS == 0 and abs(t.tiltCdeg - raise_deg * P.DS_ANGLE_SCALE) <= one_step * P.DS_ANGLE_SCALE + 1,
                     raise_deg / cfg("TILT_MAX_DEG_PER_S") + 2, drv.sink.mark(), "boot raise")
        self.assertAlmostEqual(float(sim.status()["tiltAboveStopDeg"]), cfg("TILT_CAL_BACKOFF_DEG") + raise_deg,
                               delta=2 * one_step)
        self.assertTrue(flag(p, P.DS_TELEM_FLAG_CALIBRATED))
        self.assertTrue(flag(p, P.DS_TELEM_FLAG_ESP_ALIVE))
        self.assertEqual(p.espMode, P.DS_ESP_MODE_WIFI)
        # Our connected=0 packets make WiFi the live source, but not a connected one.
        self.assertEqual(p.linkSource, P.DS_LINK_SRC_WIFI)
        self.assertFalse(flag(p, P.DS_TELEM_FLAG_LINK_CONNECTED))

        # Telemetry came back over UDP from the sim's socket the whole time we were sending.
        tel = drv.sink.since(0)
        cal = [t for t in tel if t.state == P.DS_STATE_CALIBRATING]
        self.assertGreater(len(cal), 10)
        self.assertTrue(all(t.version == P.DS_TELEMETRY_VERSION for t in tel))
        ups = [t.uptimeMs for t in tel]
        self.assertEqual(ups, sorted(ups))
        for t in cal[3:]:   # (the LCD needs a moment after BOOT to write its rows)
            self.assertEqual(rows(t), (lcd_row(cfg("LCD_TXT_CALIBRATING")), lcd_row(cfg("LCD_TXT_MODE_WIFI"))))
        # Spec 3.8 / 11.2: BOOT and CALIBRATING pulse white; never above LED_MAX_BRIGHTNESS.
        whites = {t.ledR for t in cal}
        for t in cal:
            self.assertEqual(t.ledR, t.ledG)
            self.assertEqual(t.ledG, t.ledB)
            self.assertLessEqual(t.ledR, cfg("LED_MAX_BRIGHTNESS"))
        self.assertGreater(len(whites), 3, "the white LED does not pulse")

        # Spec 3.9 / 11.3: PAIRING row 1 = mode + "Pairing" with dots cycling every LCD_PAIRING_DOT_MS;
        # LEDs pulse blue <-> white.
        m = drv.sink.mark()
        sim.sleep(6 * cfg("LCD_PAIRING_DOT_MS") / 1000.0)
        pairing = [t for t in drv.sink.since(m) if t.state == P.DS_STATE_PAIRING]
        self.assertGreater(len(pairing), 10)
        row1 = {rows(t)[1] for t in pairing}
        for r in row1:
            self.assertTrue(r.startswith(cfg("LCD_TXT_MODE_WIFI") + " "), r)
            self.assertIn(cfg("LCD_TXT_PAIRING"), r)
        self.assertGreaterEqual(len(row1), 3, "the pairing dots do not cycle: %r" % row1)
        self.assertEqual({rows(t)[0] for t in pairing}, {lcd_row(cfg("LCD_TXT_DISARMED"), ammo_text(BOOT_AMMO))})
        for t in pairing:
            self.assertEqual(t.ledR, t.ledG)                 # blue (0,0,b) to white (w,w,w)
            self.assertGreaterEqual(t.ledB, t.ledR)
            self.assertLessEqual(max(t.ledR, t.ledG, t.ledB), cfg("LED_MAX_BRIGHTNESS"))
        self.assertGreater(len({(t.ledR, t.ledB) for t in pairing}), 3)


class ConnectAndFireTests(WifiCase):
    def test_connected_stream_lands_in_disarmed_and_shows_connected(self):
        t = self.connect()
        self.assertEqual(t.linkSource, P.DS_LINK_SRC_WIFI)
        self.assertTrue(flag(t, P.DS_TELEM_FLAG_LINK_CONNECTED))
        t = self.settle()
        self.assertEqual(rows(t), (lcd_row(cfg("LCD_TXT_DISARMED"), ammo_text(BOOT_AMMO)),
                                   lcd_row(cfg("LCD_TXT_MODE_WIFI"), cfg("LCD_TXT_CONNECTED"))))
        self.assertEqual((t.ledR, t.ledG, t.ledB), scaled(cfg("LED_COLOR_DISARMED")))
        self.assertEqual(t.flywheelDuty, 0)
        # The boot magazine (TRIGGER_AMMO_AT_BOOT) with the servo home: REST at 0, else PRE-FIRE.
        self.assertEqual(t.ammo, BOOT_AMMO)
        self.assertEqual(t.servoDeg, cfg("TRIGGER_PREFIRE_ANGLE") if BOOT_AMMO else cfg("TRIGGER_REST_ANGLE"))
        self.assertGreater(t.ctrlRxHz, 0)
        # lastCtrlSeq echoes a recent ControlPacket seq of ours (the relay's latency measurement).
        self.assertLess((self.drv.sender.seq - t.lastCtrlSeq) % 256, cfg("CONTROL_PACKET_HZ") // 2)
        # The ESP32 serves the relay's packets: the stick reaches the turret through AT+XBOXSTATE?.
        st = self.sim.status()
        self.assertEqual(st["ctrlConnected"], "1")
        self.assertGreater(int(st["udpIn"]), 0)
        self.assertEqual(st["udpBad"], "0")

    def test_arm_semi_fire_ammo_and_rumble(self):
        sim, drv = self.sim, self.drv
        self.connect()
        self.load()
        # Spec 3.2 / 11.6: arming is refused while the trigger is pulled (ARM_REQUIRES_TRIGGER_RELEASED).
        self.assertTrue(cfg("ARM_REQUIRES_TRIGGER_RELEASED"))
        drv.set(rt=FULL_RT)
        sim.sleep(0.1)
        mark = sim.out.mark()
        drv.tap(P.DS_BTN_A)
        sim.wait_log(r"A \(arm\): refused: trigger held", 1.0, mark)
        self.assertEqual(drv.sink.latest().state, P.DS_STATE_DISARMED)
        drv.set(rt=0)
        sim.sleep(0.1)
        self.arm()
        t = self.settle()
        self.assertEqual((t.ledR, t.ledG, t.ledB), scaled(cfg("LED_COLOR_ARMED_SEMI")))
        self.assertEqual(rows(t)[0], lcd_row(cfg("LCD_TXT_ARMED") + cfg("LCD_TXT_FIRE_SEMI"), ammo_text(cfg("MAG_CAPACITY"))))

        # FIRE_REQUIRES_FLYWHEEL: a pull with the flywheels off does nothing.
        self.assertTrue(cfg("FIRE_REQUIRES_FLYWHEEL"))
        drv.set(rt=FULL_RT)
        sim.sleep(2 * STROKE_S)
        t = drv.sink.latest()
        self.assertEqual((t.ammo, t.shotsTotal), (cfg("MAG_CAPACITY"), 0))
        drv.set(rt=0)
        sim.sleep(0.1)

        # Spec 3.7 / 11.8: LB ramps ENA 0 -> FLYWHEEL_DUTY_255 over FLYWHEEL_RAMP_MS; ready after SPINUP.
        m = drv.sink.mark()
        ready = self.spin_up()
        spin = [t for t in drv.sink.since(m) if flag(t, P.DS_TELEM_FLAG_FLYWHEEL_ON)]
        duties = [t.flywheelDuty for t in spin]
        self.assertEqual(duties, sorted(duties))
        self.assertTrue(any(0 < d < cfg("FLYWHEEL_DUTY_255") for d in duties), duties)
        self.assertEqual(ready.flywheelDuty, cfg("FLYWHEEL_DUTY_255"))
        on_at = spin[0].uptimeMs - TEL_PERIOD_MS
        full_at = next(t.uptimeMs for t in spin if t.flywheelDuty == cfg("FLYWHEEL_DUTY_255"))
        self.assertGreaterEqual(full_at - spin[0].uptimeMs, cfg("FLYWHEEL_RAMP_MS") - 2 * TEL_PERIOD_MS)
        self.assertLessEqual(full_at - on_at, cfg("FLYWHEEL_RAMP_MS") + 2 * TEL_PERIOD_MS)
        self.assertGreaterEqual(ready.uptimeMs - on_at, cfg("FLYWHEEL_SPINUP_MS"))

        # Spec 3.4 / 11.9: semi auto, one stroke per pull above TRIGGER_PRESS_THRESHOLD.
        s0 = len(drv.sink.status)
        m = drv.sink.mark()
        drv.set(rt=FULL_RT)
        t = drv.wait(lambda t: t.shotsTotal == 1 and not flag(t, P.DS_TELEM_FLAG_STROKE_ACTIVE), 2 * STROKE_S, m)
        self.assertEqual(t.ammo, cfg("MAG_CAPACITY") - 1)
        stroke = [t for t in drv.sink.since(m) if flag(t, P.DS_TELEM_FLAG_STROKE_ACTIVE)]
        self.assertTrue(stroke)
        # Spec 3.4 / 11.11: red during the stroke.
        self.assertIn(scaled(cfg("LED_COLOR_FIRING")), [(t.ledR, t.ledG, t.ledB) for t in stroke])
        self.assertIn(cfg("TRIGGER_FIRE_ANGLE"), [t.servoDeg for t in stroke])
        # Holding does not re-fire; between the thresholds the trigger still reads pulled.
        sim.sleep(3 * STROKE_S)
        self.assertEqual(drv.sink.latest().shotsTotal, 1)
        mid = (cfg("TRIGGER_PRESS_THRESHOLD") + cfg("TRIGGER_RELEASE_THRESHOLD")) / 2
        drv.set(rt=int(FULL_RT * mid))
        sim.sleep(STROKE_S)
        t = drv.sink.latest()
        self.assertEqual(t.shotsTotal, 1)
        self.assertTrue(flag(t, P.DS_TELEM_FLAG_TRIGGER_PULLED))
        # Below TRIGGER_RELEASE_THRESHOLD, then pulled again: the next stroke.
        drv.set(rt=int(FULL_RT * cfg("TRIGGER_RELEASE_THRESHOLD") * 0.5))
        m = drv.sink.mark()
        drv.wait(lambda t: not flag(t, P.DS_TELEM_FLAG_TRIGGER_PULLED), 0.5, m)
        drv.set(rt=FULL_RT)
        t = drv.wait(lambda t: t.shotsTotal == 2, COOLDOWN_S + 2 * STROKE_S, m)
        self.assertEqual(t.ammo, cfg("MAG_CAPACITY") - 2)
        sim.sleep(STROKE_S)
        self.assertEqual(drv.sink.latest().shotsTotal, 2)

        # Spec 11.11 (radio): a rumble command per stroke goes out on the link. In WiFi mode the
        # ESP32 turns each new AT+DSS rumbleSeq into one StatusPacket to the relay.
        status = drv.sink.status[s0:]
        seqs = []
        for s in status:
            if not seqs or s.rumbleSeq != seqs[-1]:
                seqs.append(s.rumbleSeq)
        self.assertEqual(len(seqs), 2, status)
        for s in status:
            self.assertEqual((s.rumbleStrong, s.rumbleWeak, s.rumbleMs),
                             (cfg("RUMBLE_STRONG_255"), cfg("RUMBLE_WEAK_255"), cfg("RUMBLE_DURATION_MS")))
            self.assertEqual(s.wantMode, P.DS_RADIO_WIFI)
            self.assertEqual(s.state, P.DS_STATE_ARMED)

    def test_full_auto_empty_magazine_lockout_and_reset(self):
        sim, drv = self.sim, self.drv
        self.connect()
        self.load()
        self.arm()
        self.spin_up()
        # Spec 3.2 / 11.7: B toggles semi/full; orange when armed in full auto.
        m = drv.sink.mark()
        drv.tap(P.DS_BTN_B)
        drv.wait(lambda t: t.fireMode == P.DS_FIRE_MODE_FULL, 1.0, m)
        t = self.settle()
        self.assertEqual((t.ledR, t.ledG, t.ledB), scaled(cfg("LED_COLOR_ARMED_FULL")))
        self.assertEqual(rows(t)[0], lcd_row(cfg("LCD_TXT_ARMED") + cfg("LCD_TXT_FIRE_FULL"), ammo_text(cfg("MAG_CAPACITY"))))

        # Spec 3.4 / 11.10: strokes back-to-back while held, no gap (after the first, straight from
        # REST: one dart per S.BURST_MS).
        cap = cfg("MAG_CAPACITY")
        s0 = len(drv.sink.status)
        m = drv.sink.mark()
        drv.set(rt=FULL_RT)
        drv.wait(lambda t: t.ammo == 0, cap * STROKE_S + 2.0, m, "empty magazine")
        frames = drv.sink.since(m)
        first = {}
        for t in frames:
            first.setdefault(t.ammo, t.uptimeMs)
        span = first[0] - first[cap - 1]
        self.assertAlmostEqual(span, (cap - 1) * S.BURST_MS, delta=2 * TEL_PERIOD_MS)   # REST -> FIRE -> REST
        # Spec 11.12: at 0 the trigger is ignored (FIRE_LOCKOUT_WHEN_EMPTY) and the LEDs blink
        # LED_COLOR_EMPTY_MAG while it is held.
        self.assertTrue(cfg("FIRE_LOCKOUT_WHEN_EMPTY"))
        m = drv.sink.mark()
        sim.sleep(1.0)
        held = drv.sink.since(m)
        self.assertTrue(all(t.ammo == 0 and t.shotsTotal == cap for t in held))
        self.assertTrue(all(t.servoDeg == cfg("TRIGGER_REST_ANGLE") for t in held))   # the last dart: at REST
        self.assertTrue(all(flag(t, P.DS_TELEM_FLAG_EMPTY_MAG_BLINK) for t in held))
        colours = {(t.ledR, t.ledG, t.ledB) for t in held}
        self.assertIn(scaled(cfg("LED_COLOR_EMPTY_MAG")), colours)
        self.assertGreaterEqual(len(colours), 2, "the empty-magazine colour does not blink")
        # Released: the blink stops.
        drv.set(rt=0)
        m = drv.sink.mark()
        t = drv.wait(lambda t: not flag(t, P.DS_TELEM_FLAG_EMPTY_MAG_BLINK), 0.5, m)
        t = self.settle()
        self.assertEqual((t.ledR, t.ledG, t.ledB), scaled(cfg("LED_COLOR_ARMED_FULL")))
        # X restores MAG_CAPACITY and firing resumes; releasing mid-stroke finishes that stroke only.
        m = drv.sink.mark()
        drv.tap(P.DS_BTN_X)
        t = drv.wait(lambda t: t.ammo == cap, 1.0, m)
        self.assertEqual(t.servoDeg, cfg("TRIGGER_PREFIRE_ANGLE"))   # X: to PRE-FIRE
        self.assertEqual(rows(self.settle())[0][-len(ammo_text(cap)):], ammo_text(cap))
        drv.set(rt=FULL_RT)
        sim.sleep(S.CHAMBER_S + (S.COUNT_MS + S.BURST_MS / 2) / cfg("MS_PER_S"))   # into the second shot
        drv.set(rt=0)
        sim.sleep(2 * STROKE_S)
        t = drv.sink.latest()
        self.assertEqual((t.shotsTotal, t.ammo), (cap + 2, cap - 2))
        # Rumble while cycling: with RUMBLE_FULL_AUTO_CONTINUOUS each command lasts until the next
        # stroke starts, else RUMBLE_DURATION_MS.
        status = [s for s in drv.sink.status[s0:] if s.rumbleMs]
        self.assertTrue(status)
        for s in status:
            if cfg("RUMBLE_FULL_AUTO_CONTINUOUS"):
                self.assertGreaterEqual(s.rumbleMs, STROKE_MS)
            else:
                self.assertEqual(s.rumbleMs, cfg("RUMBLE_DURATION_MS"))


class LinkLossTests(WifiCase):
    def test_link_loss_stops_everything_and_reconnect_is_disarmed(self):
        sim, drv = self.sim, self.drv
        self.connect()
        self.arm()
        self.spin_up()
        drv.set(rx=AXIS - 1)    # full right: the turret pans clockwise (negative), at speed
        m = drv.sink.mark()
        drv.wait(lambda t: t.panVelCdegS < -0.9 * cfg("PAN_MAX_DEG_PER_S") * SCALE * shaped((AXIS - 1) / AXIS), 2.0, m)

        # Spec 3.1 / 11.13: no packet for LINK_TIMEOUT_MS -> stop motion, flywheels, PAIRING.
        out = sim.out.mark()
        m = drv.sink.mark()
        drv.pause()
        lost_ms, _, i = sim.wait_log(r"link lost: no connected frame for LINK_TIMEOUT_MS", 2.0, out)
        pairing_ms, _, _ = sim.wait_log(r"state PAIRING", 0.5, i)
        st = sim.status()
        last_rx = int(st["ctrlRxMs"])
        # The loss is never declared early, and at the latest one RA4M1 poll period after the
        # ESP32's own LINK_TIMEOUT_MS check (a pending AT+DST line and the reply in between).
        poll_ms = 1000.0 / cfg("CONTROL_PACKET_HZ")
        line_ms = P.DS_AT_DST_LINE_LEN * cfg("UART_BITS_PER_BYTE") * 1000.0 / cfg("LINK_BAUD")
        self.assertGreaterEqual(lost_ms - last_rx, cfg("LINK_TIMEOUT_MS"))
        self.assertLessEqual(lost_ms - last_rx, cfg("LINK_TIMEOUT_MS") + poll_ms + line_ms + cfg("AT_REPLY_TIMEOUT_MS"))
        self.assertLessEqual(pairing_ms - lost_ms, 1)
        # Everything stops: flywheels cut (FAILSAFE_STOP_FLYWHEEL), servo home (REST: the boot
        # magazine, nothing loaded), pan decelerates at its accel limit to 0.
        self.assertTrue(cfg("FAILSAFE_STOP_FLYWHEEL"))
        stop_ms = 1000.0 * cfg("PAN_MAX_DEG_PER_S") / cfg("PAN_ACCEL_DEG_PER_S2")
        t = drv.wait(lambda t: t.state == P.DS_STATE_PAIRING and t.panVelCdegS == 0, (stop_ms + 500) / 1000.0, m)
        self.assertLessEqual(t.uptimeMs - lost_ms, stop_ms + 2 * TEL_PERIOD_MS)
        after = [t for t in drv.sink.since(m) if t.uptimeMs > lost_ms + 5]
        self.assertTrue(after)
        for t in after:
            self.assertEqual(t.flywheelDuty, 0)
            self.assertFalse(flag(t, P.DS_TELEM_FLAG_FLYWHEEL_ON))
            self.assertEqual(t.servoDeg, cfg("TRIGGER_REST_ANGLE"))
            self.assertNotEqual(t.state, P.DS_STATE_ARMED)
        # LCD: "Pairing" for LINK_RECONNECT_GRACE_MS, then "No Link".
        t = self.settle()
        self.assertTrue(rows(t)[1].startswith(cfg("LCD_TXT_MODE_WIFI") + " "))
        self.assertIn(cfg("LCD_TXT_PAIRING"), rows(t)[1])
        sim.sleep(cfg("LINK_RECONNECT_GRACE_MS") / 1000.0)
        t = self.settle()
        self.assertEqual(rows(t)[1], lcd_row(cfg("LCD_TXT_MODE_WIFI"), cfg("LCD_TXT_NO_CONNECTION")))
        self.assertEqual(t.state, P.DS_STATE_PAIRING)

        # Reconnection restores DISARMED, never ARMED (spec 3.1) - even though we were armed.
        drv.set(rx=0)
        m = drv.sink.mark()
        drv.resume()
        drv.wait(state_is("DISARMED"), 1.0, m)
        sim.sleep(1.0)
        self.assertNotIn(P.DS_STATE_ARMED, [t.state for t in drv.sink.since(m)])

        # A relay that stops cleanly sends one connected=0 packet (Sender.send_final): a loss at
        # once, not after LINK_TIMEOUT_MS.
        out = sim.out.mark()
        drv.stop_clean()
        lost_ms, _, _ = sim.wait_log(r"link lost: no connected frame", 1.0, out)
        st = sim.status()
        self.assertEqual(st["ctrlConnected"], "0")
        self.assertLess(lost_ms - int(st["ctrlRxMs"]), poll_ms + line_ms + cfg("AT_REPLY_TIMEOUT_MS"))


class MotionTests(WifiCase):
    def test_stick_speed_limits_home_and_link_stats(self):
        sim, drv = self.sim, self.drv
        self.connect()
        out0 = sim.out.mark()
        pan_max = cfg("PAN_MAX_DEG_PER_S") * SCALE

        # Spec 3.2 / 11.4: inside the deadzone nothing moves.
        x = int(0.9 * cfg("JOY_DEADZONE") * AXIS)
        drv.set(rx=x)
        sim.sleep(0.5)
        t = drv.sink.latest()
        self.assertEqual((t.panVelCdegS, t.panCdeg), (0, 0))
        # Speed proportional to the shaped deflection (deadzone + expo), stick right = clockwise.
        for raw in (AXIS // 2, AXIS - 1):
            drv.set(rx=raw)
            sim.sleep(0.5)
            m = drv.sink.mark()
            sim.sleep(0.3)
            v = [t.panVelCdegS for t in drv.sink.since(m)]
            want = -shaped(raw / AXIS) * pan_max
            for got in v:
                self.assertAlmostEqual(got, want, delta=abs(want) * 0.02 + 1)
        drv.set(rx=0)
        sim.sleep(0.5)

        # Tilt: stick up (negative, JOY_INVERT_Y) raises the barrel to TILT_RANGE_DEG and never past
        # it, braking before it; the mechanism really stops there (no lost steps, no crash).
        up_ry, down_ry = (-AXIS, AXIS - 1) if cfg("JOY_INVERT_Y") else (AXIS - 1, -AXIS)
        m = drv.sink.mark()
        drv.set(ry=up_ry)
        top = cfg("TILT_RANGE_DEG") * SCALE
        drv.wait(lambda t: t.tiltVelCdegS == 0 and t.tiltCdeg > top - SCALE, cfg("TILT_RANGE_DEG") / cfg("TILT_MAX_DEG_PER_S") + 2, m)
        up = drv.sink.since(m)
        self.assertLessEqual(max(t.tiltCdeg for t in up), top)
        self.assertLessEqual(max(t.tiltVelCdegS for t in up), cfg("TILT_MAX_DEG_PER_S") * SCALE)
        near = [t for t in up if t.tiltCdeg > top - cfg("SOFT_LIMIT_SLOWDOWN_DEG") * SCALE / 2 and t.tiltVelCdegS > 0]
        for t in near:   # inside half the slowdown zone it is already well below full speed
            self.assertLess(t.tiltVelCdegS, cfg("TILT_MAX_DEG_PER_S") * SCALE * 0.9)
        one_step = 1.0 / cfg("TILT_STEPS_PER_DEG")
        st = sim.status()
        self.assertAlmostEqual(float(st["tiltAboveStopDeg"]), cfg("TILT_RANGE_DEG") + cfg("TILT_CAL_BACKOFF_DEG"),
                               delta=0.5)
        # ... and down to 0, not into the panel.
        m = drv.sink.mark()
        drv.set(ry=down_ry)
        drv.wait(lambda t: t.tiltVelCdegS == 0 and t.tiltCdeg < SCALE, cfg("TILT_RANGE_DEG") / cfg("TILT_MAX_DEG_PER_S") + 2, m)
        self.assertGreaterEqual(min(t.tiltCdeg for t in drv.sink.since(m)), 0)
        self.assertAlmostEqual(float(sim.status()["tiltAboveStopDeg"]), cfg("TILT_CAL_BACKOFF_DEG"), delta=2 * one_step + 0.1)
        drv.set(ry=0)

        # Spec 3.2 / 11.5: d-pad left stores home, d-pad down returns there at HOME_SPEED_DEG_PER_S.
        drv.set(ry=up_ry // 2)
        sim.sleep(0.6)
        drv.set(ry=0, rx=-AXIS // 2)
        sim.sleep(0.5)
        drv.set(rx=0)
        sim.sleep(0.5)
        here = drv.sink.latest()
        m = drv.sink.mark()
        drv.tap(P.DS_BTN_DPAD_LEFT)
        t = drv.wait(lambda t: t.homePanCdeg != 0 or t.homeTiltCdeg != 0, 1.0, m)
        self.assertAlmostEqual(t.homePanCdeg, here.panCdeg, delta=2)
        self.assertAlmostEqual(t.homeTiltCdeg, here.tiltCdeg, delta=2)
        drv.set(rx=AXIS - 1)
        sim.sleep(0.6)
        drv.set(rx=0)
        sim.sleep(0.5)
        m = drv.sink.mark()
        drv.tap(P.DS_BTN_DPAD_DOWN)
        drv.wait(lambda t: flag(t, P.DS_TELEM_FLAG_HOMING), 1.0, m)
        m2 = drv.sink.mark()
        t = drv.wait(lambda t: not flag(t, P.DS_TELEM_FLAG_HOMING), 3.0, m2)
        self.assertAlmostEqual(t.panCdeg, here.panCdeg, delta=SCALE / 10)
        self.assertAlmostEqual(t.tiltCdeg, here.tiltCdeg, delta=SCALE / 10)
        homing = [t for t in drv.sink.since(m) if flag(t, P.DS_TELEM_FLAG_HOMING)]
        peak = max(abs(t.panVelCdegS) for t in homing)
        self.assertLessEqual(peak, cfg("HOME_SPEED_DEG_PER_S") * SCALE * 1.01)
        self.assertGreaterEqual(peak, cfg("HOME_SPEED_DEG_PER_S") * SCALE * 0.95)
        # A stick nudge mid-move cancels it.
        drv.set(rx=AXIS - 1)
        sim.sleep(0.6)
        drv.set(rx=0)
        sim.sleep(0.5)
        m = drv.sink.mark()
        drv.tap(P.DS_BTN_DPAD_DOWN, 50)
        drv.wait(lambda t: flag(t, P.DS_TELEM_FLAG_HOMING), 1.0, m)
        m = drv.sink.mark()
        drv.set(rx=-AXIS // 2)
        t = drv.wait(lambda t: not flag(t, P.DS_TELEM_FLAG_HOMING) and t.panVelCdegS > 0, 1.0, m)
        drv.set(rx=0)

        # Spec 11.17 (the part the sim can show): the stats lines while connected report packet
        # loss under PACKET_LOSS_LIMIT_PERMILLE (no '!') and no AT poll timeout.
        stats = sim.out.all(r"stats loop=(\d+) rx=(\d+) to=(\d+) crc=(\d+) gap=\d+ loss=([\d.]+)%(!?)", out0)
        self.assertGreaterEqual(len(stats), 4)
        for s in stats[1:]:
            self.assertEqual(s.group(6), "", s.group(0))
            self.assertLess(float(s.group(5)) * 10, cfg("PACKET_LOSS_LIMIT_PERMILLE"), s.group(0))
            self.assertEqual(s.group(3), "0", s.group(0))
            self.assertEqual(s.group(4), "0", s.group(0))


if __name__ == "__main__":
    unittest.main()

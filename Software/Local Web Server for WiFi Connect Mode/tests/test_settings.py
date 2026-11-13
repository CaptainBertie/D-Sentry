# SPDX-License-Identifier: MIT
"""dsentry.settings: every relay value comes from config.h (a missing one is refused by name),
and unsafe values are refused."""

import re
import unittest

import _support as S

from dsentry import settings as ST

CONFIG_TEXT = S.CONFIG_H.read_text(encoding="utf-8")


def with_define(name, value, text=CONFIG_TEXT):
    """config.h text with one #define replaced (or appended when absent)."""
    pattern = re.compile(r"^(\s*#\s*define\s+%s\s+)(\S+)" % re.escape(name), re.M)
    if pattern.search(text):
        return pattern.sub(lambda m: m.group(1) + str(value), text, count=1)
    return text + "\n#define %s %s\n" % (name, value)


class LoadTests(unittest.TestCase):
    def test_real_config_values(self):
        s = ST.Settings.load(S.CONFIG_H)
        cfg = s.cfg
        self.assertEqual(s.control_hz, cfg["CONTROL_PACKET_HZ"])
        self.assertAlmostEqual(s.control_period_s, 1.0 / cfg["CONTROL_PACKET_HZ"])
        self.assertAlmostEqual(s.link_timeout_s, cfg["LINK_TIMEOUT_MS"] / 1000.0)
        self.assertEqual(s.serial_baud, cfg["DEBUG_SERIAL_BAUD"])
        self.assertAlmostEqual(s.serial_write_timeout_s, cfg["RELAY_SERIAL_WRITE_TIMEOUT_MS"] / 1000.0)
        self.assertEqual(s.udp_port, cfg["WIFI_UDP_PORT"])
        self.assertEqual(s.discovery_port, cfg["WIFI_DISCOVERY_PORT"])
        self.assertEqual(s.ap_ip, cfg["WIFI_AP_IP"])
        self.assertEqual(s.http_port, cfg["DASHBOARD_HTTP_PORT"])
        self.assertEqual(s.joy_axis_range, cfg["JOY_AXIS_RANGE"])
        self.assertAlmostEqual(s.keyboard_press_s, cfg["KEYBOARD_PRESS_MS"] / 1000.0)

    def test_names_once_pending_now_come_from_config_h(self):
        # These three were CONFIG-PENDING defaults in settings.py until the integration stage
        # moved them into config.h section 13; now config.h is their only source.
        s = ST.Settings.load(S.CONFIG_H)
        cfg = s.cfg
        self.assertAlmostEqual(s.source_change_hold_s, cfg["RELAY_SOURCE_CHANGE_HOLD_MS"] / 1000.0)
        self.assertEqual(s.keyboard_press_queue_max, cfg["KEYBOARD_PRESS_QUEUE_MAX"])
        self.assertAlmostEqual(s.stale_s, cfg["DASHBOARD_STALE_MS"] / 1000.0)
        self.assertFalse(hasattr(ST, "pending_names"))
        s = ST.Settings.from_text(with_define("DASHBOARD_STALE_MS", 900))
        self.assertAlmostEqual(s.stale_s, 0.9)
        for name in ("RELAY_SOURCE_CHANGE_HOLD_MS", "KEYBOARD_PRESS_QUEUE_MAX", "DASHBOARD_STALE_MS"):
            text = re.sub(r"^#define %s\b.*$" % name, "", CONFIG_TEXT, flags=re.M)
            with self.subTest(name=name), self.assertRaisesRegex(ST.SettingsError, name):
                ST.Settings.from_text(text)

    def test_value_falls_back_only_to_the_callers_default(self):
        s = ST.Settings.load(S.CONFIG_H)
        self.assertEqual(s.value("STEP_JITTER_LIMIT_PERMILLE"), s.cfg["STEP_JITTER_LIMIT_PERMILLE"])
        self.assertIsNone(s.value("NO_SUCH_DEFINE"))
        self.assertEqual(s.value("NO_SUCH_DEFINE", 7), 7)

    def test_missing_define_is_named(self):
        text = re.sub(r"^#define WIFI_UDP_PORT.*$", "", CONFIG_TEXT, flags=re.M)
        with self.assertRaisesRegex(ST.SettingsError, "WIFI_UDP_PORT"):
            ST.Settings.from_text(text)


class ValidationTests(unittest.TestCase):
    def test_reset_bauds_refused(self):
        for baud in ST.BRIDGE_RESET_BAUDS:
            with self.subTest(baud=baud), self.assertRaisesRegex(ST.SettingsError, "resets the RA4M1"):
                ST.Settings.from_text(with_define("DEBUG_SERIAL_BAUD", baud))

    def test_source_change_hold_must_exceed_link_timeout(self):
        timeout = ST.Settings.load(S.CONFIG_H).cfg["LINK_TIMEOUT_MS"]
        with self.assertRaisesRegex(ST.SettingsError, "RELAY_SOURCE_CHANGE_HOLD_MS"):
            ST.Settings.from_text(with_define("RELAY_SOURCE_CHANGE_HOLD_MS", timeout))

    def test_press_must_outlast_debounce(self):
        debounce = ST.Settings.load(S.CONFIG_H).cfg["BUTTON_DEBOUNCE_MS"]
        with self.assertRaisesRegex(ST.SettingsError, "KEYBOARD_PRESS_MS"):
            ST.Settings.from_text(with_define("KEYBOARD_PRESS_MS", debounce))

    def test_revoke_after_deadman(self):
        timeout = ST.Settings.load(S.CONFIG_H).cfg["LINK_TIMEOUT_MS"]
        with self.assertRaisesRegex(ST.SettingsError, "KEYBOARD_REVOKE_MS"):
            ST.Settings.from_text(with_define("KEYBOARD_REVOKE_MS", timeout))

    def test_deflection_range(self):
        for bad in ("0.0f", "1.5f"):
            with self.subTest(bad=bad), self.assertRaisesRegex(ST.SettingsError, "KEYBOARD_DEFLECTION"):
                ST.Settings.from_text(with_define("KEYBOARD_DEFLECTION", bad))


if __name__ == "__main__":
    unittest.main()

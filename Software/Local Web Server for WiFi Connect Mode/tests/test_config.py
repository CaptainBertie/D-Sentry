# SPDX-License-Identifier: MIT
"""relay-pc/dsentry/config.py against the real D-SentryCode/config.h and tricky inputs."""

import os
import re
import tempfile
import unittest
from pathlib import Path

import _support as S
from dsentry import config as C


class TestRealConfig(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.cfg = C.load(S.CONFIG_H)

    def test_default_path_is_the_dsentry_config(self):
        self.assertEqual(C.DEFAULT_PATH.resolve(), S.CONFIG_H.resolve())
        self.assertEqual(len(C.load()), len(self.cfg))

    def test_steps_per_degree(self):
        cfg = self.cfg
        self.assertEqual(cfg["PAN_STEPS_PER_DEG"], 200 * 8 * 5 / 360)
        self.assertEqual(cfg.PAN_STEPS_PER_DEG,
                         cfg.MOTOR_FULL_STEPS_PER_REV * cfg.MICROSTEPS * cfg.PAN_GEAR_RATIO / 360)
        self.assertEqual(cfg.TILT_STEPS_PER_DEG,
                         cfg.MOTOR_FULL_STEPS_PER_REV * cfg.MICROSTEPS * cfg.TILT_GEAR_RATIO / 360)

    def test_types(self):
        cfg = self.cfg
        self.assertIs(type(cfg.CONTROL_PACKET_HZ), int)
        self.assertIs(type(cfg.JOY_DEADZONE), float)
        self.assertIs(cfg.TOGGLE_ACTIVE_LOW, True)
        self.assertIs(cfg.PAN_LIMITS_ENABLED, False)
        self.assertIs(type(cfg.WIFI_SSID), str)
        self.assertNotIsInstance(cfg.WIFI_SSID, C.RawToken)
        self.assertEqual(cfg.LCD_TXT_OFF_ROW1, "")
        self.assertEqual(cfg.LCD_I2C_ADDR, int(cfg.raw("LCD_I2C_ADDR"), 16))
        self.assertLess(cfg.PAN_MIN_DEG, 0)
        self.assertEqual(cfg.TILT_CAL_MODE, cfg.CAL_RAM)          # an alias of another define
        self.assertEqual(cfg.rgb("LED_COLOR_ARMED_FULL"), (0xFF, 0x60, 0x00))
        for name in ("PIN_TOGGLE_POWER", "LINK_SERIAL", "LED_PIXEL_ORDER"):
            self.assertIsInstance(cfg[name], C.RawToken, name)
        self.assertEqual(cfg.PIN_TOGGLE_POWER, "A0")
        self.assertEqual(cfg.LINK_SERIAL, "Serial2")
        self.assertEqual(cfg.LED_PIXEL_ORDER, "NEO_GRB")

    def test_every_define_resolves_or_is_one_arduino_name(self):
        # Robust to new defines: anything left raw must be a single identifier (A0, Serial2 ...),
        # never an expression the evaluator failed on.
        for name in self.cfg.unresolved:
            self.assertRegex(self.cfg.raw(name), r"^[A-Za-z_]\w*$", name)

    def test_sections_and_comments(self):
        cfg = self.cfg
        self.assertEqual(cfg.section("LINK_TIMEOUT_MS"), "10. CONTROLLER / LINK")
        self.assertIn("WIFI_UDP_PORT", cfg.names_in_section(11))
        self.assertIn("controller lost", cfg.comment("LINK_TIMEOUT_MS"))   # continuation joined
        self.assertTrue(cfg.comment("LINK_TIMEOUT_MS").startswith("[TUNE]"))
        self.assertEqual(cfg.define("CONTROL_PACKET_HZ").line,
                         next(i for i, l in enumerate(S.CONFIG_H.read_text().splitlines(), 1)
                              if re.match(r"#define\s+CONTROL_PACKET_HZ\b", l)))

    def test_mapping(self):
        cfg = self.cfg
        self.assertIn("LED_COUNT", cfg)
        self.assertEqual(list(cfg)[0], "PIN_PAN_STEP")
        with self.assertRaises(AttributeError):
            cfg.NOT_A_DEFINE
        with self.assertRaises(KeyError):
            cfg["NOT_A_DEFINE"]


class TestEvaluator(unittest.TestCase):
    def cfg(self, text):
        return C.from_text(text)

    def test_numbers(self):
        c = self.cfg("#define A 0x1F\n#define B 017\n#define C 0b101\n#define D 1'000'000\n"
                     "#define E 1000000UL\n#define F 2.5e-3f\n#define G .5\n#define H 3.\n"
                     "#define I 0x1p4\n#define J 'A'\n#define K -170.0f\n")
        self.assertEqual([c[k] for k in "ABCDE"], [31, 15, 5, 1000000, 1000000])
        self.assertEqual([c[k] for k in "FGHI"], [2.5e-3, 0.5, 3.0, 16.0])
        self.assertEqual((c.J, c.K), (65, -170.0))
        self.assertIs(type(c.E), int)
        self.assertIs(type(c.H), float)

    def test_c_division_and_modulo(self):
        c = self.cfg("#define HZ 100\n#define MS_PER_S 1000\n#define PERIOD (MS_PER_S / HZ)\n"
                     "#define ODD (7 / 2)\n#define NEG (-7 / 2)\n#define MOD (-7 % 3)\n"
                     "#define FL (7 / 2.0f)\n#define BITS ((1u << 4) | 0x3)\n"
                     "#define TERN (HZ > 50 ? 1 : 2)\n#define NOT !0\n")
        self.assertEqual((c.PERIOD, c.ODD, c.NEG, c.MOD), (10, 3, -3, -1))
        self.assertEqual(c.FL, 3.5)
        self.assertEqual((c.BITS, c.TERN, c.NOT), (19, 1, 1))

    def test_casts(self):
        c = self.cfg("#define A ((uint8_t)300)\n#define B ((int8_t)200)\n#define C ((float)3 / 2)\n"
                     "#define D ((uint32_t)-1)\n#define E ((unsigned long)5.9)\n")
        self.assertEqual((c.A, c.B, c.C, c.D, c.E), (44, -56, 1.5, 0xFFFFFFFF, 5))

    def test_strings(self):
        c = self.cfg('#define S "a\\"b" "c"\n#define T "http://x" // comment\n#define U "tab\\tx"\n')
        self.assertEqual((c.S, c.T, c.U), ('a"bc', "http://x", "tab\tx"))

    def test_unresolved_stays_raw(self):
        c = self.cfg("#define P A0\n#define Q (A0 + 1)\n#define F(x) ((x) * 2)\n#define R P\n"
                     "#define LOOP1 LOOP2\n#define LOOP2 LOOP1\n#define Z (1 / 0)\n#define S \"a\" + 1\n")
        for name in ("P", "Q", "F", "R", "LOOP1", "LOOP2", "Z", "S"):
            self.assertIsInstance(c[name], C.RawToken, name)
        self.assertEqual(c.P, "A0")
        self.assertEqual(c.Q, "(A0 + 1)")
        self.assertEqual(c.F, "((x) * 2)")
        self.assertEqual(c.R, "A0")             # an alias expands to its target's text
        self.assertEqual(c.define("F").params, "(x)")

    def test_forward_reference_comments_undef_duplicates(self):
        c = self.cfg("#define LATER (EARLY_USES * 2)\n#define EARLY_USES 21\n"
                     "#define BLOCK /* 99 */ 4 /* multi\nline */\n"
                     "#define CONT (1 + \\\n  2)\n#define GONE 1\n#undef GONE\n"
                     "#define DUP 1\n#define DUP 2\n#if 0\n#endif\n")
        self.assertEqual((c.LATER, c.BLOCK, c.CONT, c.DUP), (42, 4, 3, 2))
        self.assertNotIn("GONE", c)
        self.assertEqual(c.duplicates, ["DUP"])
        self.assertTrue(c.has_conditionals)

    def test_env_override(self):
        with tempfile.TemporaryDirectory() as d:
            p = Path(d) / "config.h"
            p.write_text("#define ONLY_ME 5\n")
            old = os.environ.get(C.ENV_VAR)
            os.environ[C.ENV_VAR] = str(p)
            try:
                self.assertEqual(dict(C.load()), {"ONLY_ME": 5})
            finally:
                if old is None:
                    del os.environ[C.ENV_VAR]
                else:
                    os.environ[C.ENV_VAR] = old


if __name__ == "__main__":
    unittest.main()

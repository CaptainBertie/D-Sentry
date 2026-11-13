# SPDX-License-Identifier: MIT
"""dsentry_sim as a program: the command line the relay uses, stdout carrying the RA4M1's text
and nothing else, the stdin commands, clean exits on SIGTERM / SIGINT, and the power toggle
(spec 11 item 1)."""

import re
import signal
import subprocess
import time
import unittest

import _sim as S

# A text line on stdout: printable ASCII and tabs only (the relay's clean_text() would keep it as is).
TEXT_ONLY = re.compile(rb"^[\x20-\x7e\t]*$")


class CommandLineTests(unittest.TestCase):
    def test_stdout_is_text_lines_only_while_frames_flow(self):
        # SAFETY / contract: the RA4M1's USB line carries Status and Telemetry frames between its
        # text; stdout must never leak a byte of them (architecture 4.3, sim contract).
        speed, seconds = 10, 3.0
        r = subprocess.run([S.sim_binary(), "--seconds", str(seconds), "--speed", str(speed)],
                           stdin=subprocess.DEVNULL, capture_output=True, timeout=seconds / speed + 20)
        self.assertEqual(r.returncode, 0, r.stderr.decode(errors="replace"))
        self.assertTrue(r.stdout.endswith(b"\n"))
        lines = r.stdout.split(b"\n")[:-1]
        self.assertGreater(len(lines), 5)
        for line in lines:
            self.assertRegex(line, TEXT_ONLY)
        text = r.stdout.decode()
        self.assertIn("D-Sentry RA4M1 boot", text)
        self.assertRegex(text, r"\n0 state BOOT\n")
        self.assertRegex(text, r"\n0 state CALIBRATING\n")
        # The frames were on the line (and cut): TELEMETRY_USB_HZ telemetry frames a second.
        m = re.search(r"stopped: \d+ loops, (\d+) text lines on stdout, (\d+) frames cut \((\d+) telemetry, "
                      r"(\d+) status\), (\d+) junk bytes", r.stderr.decode())
        self.assertIsNotNone(m, r.stderr.decode())
        self.assertEqual(int(m.group(1)), len(lines))
        expect = seconds * S.cfg("TELEMETRY_USB_HZ")
        self.assertGreaterEqual(int(m.group(3)), expect - 2)
        self.assertLessEqual(int(m.group(3)), expect + 1)
        self.assertGreater(int(m.group(4)), 0)
        self.assertEqual(int(m.group(5)), 0)
        # The ESP32-S3's ROM banner reaches the RA4M1's Serial RX at power-on (UART0 is wired to
        # it) and its shell refuses every line (a risk the core stage listed; visible here).
        self.assertIn("unknown command: ESP-ROM:esp32s3-20210327", text)
        # ... and with --no-esp-banner it does not.
        r2 = subprocess.run([S.sim_binary(), "--seconds", "0.5", "--speed", str(speed), "--no-esp-banner"],
                            stdin=subprocess.DEVNULL, capture_output=True, timeout=20)
        self.assertEqual(r2.returncode, 0)
        self.assertNotIn(b"ESP-ROM", r2.stdout)

    def test_relay_argv_and_signals_exit_cleanly(self):
        # The relay starts "<bin> --udp-port N [--sim-arg ...]" with stdin=/dev/null and stops it
        # with SIGTERM (relay-pc/dsentry/transports.py SimProcess).
        for sig in (signal.SIGTERM, signal.SIGINT):
            with self.subTest(sig=sig):
                port = S.free_udp_port()
                proc = subprocess.Popen([S.sim_binary(), "--udp-port", str(port), "--speed", "2"],
                                        stdin=subprocess.DEVNULL, stdout=subprocess.PIPE, stderr=subprocess.PIPE)
                try:
                    time.sleep(1.0)
                    self.assertIsNone(proc.poll(), "the sim exited on its own (closed stdin must not stop it)")
                    proc.send_signal(sig)
                    out, err = proc.communicate(timeout=5)
                finally:
                    if proc.poll() is None:
                        proc.kill()
                        proc.communicate()
                self.assertEqual(proc.returncode, 0, err.decode())
                self.assertIn(b"stopped by signal", err)
                self.assertIn(("udp 127.0.0.1:%d" % port).encode(), err)
                self.assertIn(b"D-Sentry RA4M1 boot", out)

    def test_bad_arguments_are_usage_errors(self):
        for args in (["--bogus"], ["--speed", "0"], ["--speed", "x"], ["--udp-port", "70000"], ["--udp-port"],
                     ["--mode", "usb"], ["--tilt-stop", "5"], ["--a2", "5000"], ["stray"]):
            with self.subTest(args=args):
                r = subprocess.run([S.sim_binary(), *args], stdin=subprocess.DEVNULL, capture_output=True,
                                   timeout=10)
                self.assertEqual(r.returncode, 2)
                self.assertIn(b"usage: dsentry_sim", r.stderr)
                self.assertEqual(r.stdout, b"")
        r = subprocess.run([S.sim_binary(), "--help"], capture_output=True, timeout=10)
        self.assertEqual(r.returncode, 0)
        self.assertIn(b"usage: dsentry_sim", r.stdout)


class StdinAndPowerTests(unittest.TestCase):
    def test_stdin_commands_and_the_power_toggle(self):
        # Spec 11 item 1: power toggle open -> everything idle, LCD SYSTEM OFF, LEDs dark;
        # closing it reruns BOOT -> CALIBRATING (-> PAIRING, see test_wifi).
        sim = S.Sim(speed=10)
        try:
            sim.wait_log(r"state CALIBRATING", 1.0)
            st = sim.status()
            for key in ("speed", "power", "mode", "lcd0", "lcd1", "led0", "esp", "espMode", "bootId", "wifi",
                        "tiltAboveStopDeg", "ctrlRxMs"):
                self.assertIn(key, st)
            self.assertEqual(st["power"], "on")
            self.assertEqual(st["mode"], "wifi")
            sim.command("bogus 1", r"unknown or malformed command: bogus 1")
            sim.command("help", r"commands: ")
            mark = sim.out.mark()
            sim.command("power off", r"power toggle off")
            ms_off, _, _ = sim.wait_log(r"state OFF", 1.0, mark)
            sim.sleep(0.5)   # LCD_REFRESH_MS and the LED frame have passed
            st = sim.status()
            self.assertEqual(st["lcd0"], S.lcd_row(S.cfg("LCD_TXT_OFF_ROW0")))
            self.assertEqual(st["lcd1"], S.lcd_row(S.cfg("LCD_TXT_OFF_ROW1")))
            self.assertEqual(st["led0"], "0,0,0")
            if S.cfg("LCD_BACKLIGHT_OFF_WHEN_SYSTEM_OFF"):
                self.assertEqual(st["backlight"], "0")
            # A shell action is refused in OFF; the line still gets its reply.
            sim.send("shell arm")
            sim.wait_log(r"arm: refused: not in this state", 1.0, mark)
            mark = sim.out.mark()
            ack = sim.command("power on", r"power toggle on")
            t_on_ms = float(ack.group(0).split()[1][2:]) * 1000.0
            ms_boot, _, i = sim.wait_log(r"state BOOT", 1.0, mark)
            ms_cal, _, _ = sim.wait_log(r"state CALIBRATING", 1.0, i)
            # debounced: the switch must read closed for TOGGLE_DEBOUNCE_MS before BOOT
            self.assertGreaterEqual(ms_boot - t_on_ms, S.cfg("TOGGLE_DEBOUNCE_MS") - 1)
            self.assertLess(ms_boot - t_on_ms, S.cfg("TOGGLE_DEBOUNCE_MS") + 20)
            self.assertLessEqual(ms_cal - ms_boot, 1)   # CALIBRATING on the next tick
            # A closed stdin only ends command input.
            sim.proc.stdin.close()
            sim.sleep(1.0)
            self.assertIsNone(sim.proc.poll())
        finally:
            rc = sim.stop()
        self.assertEqual(rc, 0)


if __name__ == "__main__":
    unittest.main()

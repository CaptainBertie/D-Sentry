# test_r4flash.py - unit tests of r4flash.py's safety guards. No hardware: every
# command that could reach a board runs with --dry-run, or with a --port that the
# port guard rejects before esptool is started, or in-process with esptool, the port
# and hidapi replaced by fakes (RealFlowTests).
#
# Run with the project venv:
#   "<repo>/.venv/bin/python" "<repo>/tools/tests/test_r4flash.py" -v
# Inputs (environment):
#   R4FLASH_TEST_BUILD_DIR    a known-good build dir (S3-ALL.bin, S3-BOOT-APP.bin, ...);
#                             default: the firmware build dir. Tests that need it skip
#                             when it is missing.
#   R4FLASH_TEST_OFFICIAL_ZIP Arduino's unor4wifi-update-macos.zip (0.6.0); default:
#                             tools/cache/unor4wifi-update-macos.zip. Optional.
#   R4FLASH_TEST_NETWORK=1    also run the one test that really downloads the 0.6.0 zip
#                             from GitHub (about 5 MB, into a temp folder). Off by default.
#
# SPDX-License-Identifier: MIT

import argparse
import io
import os
import shutil
import struct
import subprocess
import sys
import tempfile
import types
import unittest
from pathlib import Path
from unittest import mock

TESTS_DIR = Path(__file__).resolve().parent
TOOLS_DIR = TESTS_DIR.parent
TOOL = TOOLS_DIR / "r4flash.py"
sys.path.insert(0, str(TOOLS_DIR))

import r4flash  # noqa: E402  (needs the sys.path line above)

BUILD_DIR = Path(os.environ.get("R4FLASH_TEST_BUILD_DIR", str(r4flash.DEFAULT_BUILD_DIR)))
OFFICIAL_ZIP = Path(os.environ.get("R4FLASH_TEST_OFFICIAL_ZIP",
                                   str(r4flash.DEFAULT_CACHE_DIR / r4flash.OFFICIAL_ZIP_NAME)))
NEEDED_FILES = list(r4flash.BUILD_FILES.values())
REFUSED = r4flash.EXIT_REFUSED
# esptool 5.5.0 prints this on connecting: the label padded to 20 characters.
FAKE_MAC = "34:85:18:aa:bb:cc"
MAC_LINE = f"{'MAC:':<20}{FAKE_MAC}\n"


def have_build():
    return all((BUILD_DIR / name).is_file() for name in NEEDED_FILES)


def run_tool(*args):
    """Run r4flash.py in a child process; returns (exit code, stdout + stderr)."""
    cmd = [sys.executable, str(TOOL)] + [str(a) for a in args]
    proc = subprocess.run(cmd, capture_output=True, text=True, timeout=300)
    return proc.returncode, proc.stdout + proc.stderr


def write_with_sidecar(path, data):
    path.write_bytes(data)
    r4flash.write_sidecar(path, r4flash.sha256_bytes(data))
    return path


def board_attached():
    """True if any UNO R4 WiFi is visible, in which case 'no device found' is not expected."""
    if r4flash.ports_with(r4flash.DOWNLOAD_VID_PID) or r4flash.ports_with(r4flash.BRIDGE_VID_PID):
        return True
    return bool(r4flash.bridge_hid_devices())


class PureFunctionTests(unittest.TestCase):
    """Helpers that need no files at all."""

    def test_reboot_report_matches_unor4wifi_reboot(self):
        # main.go: 65 bytes, b[0] = 0 (report id), b[1] = 0xAA, rest zero.
        self.assertEqual(len(r4flash.REBOOT_REPORT), 65)
        self.assertEqual(r4flash.REBOOT_REPORT[:2], bytes([0x00, 0xAA]))
        self.assertEqual(set(r4flash.REBOOT_REPORT[2:]), {0})

    def test_parse_flash_size(self):
        self.assertEqual(r4flash.parse_flash_size("Detected flash size: 8MB\n"), 0x800000)
        self.assertEqual(r4flash.parse_flash_size("Detected flash size: 4MB"), 0x400000)
        self.assertIsNone(r4flash.parse_flash_size("Detected flash size: Unknown"))
        self.assertIsNone(r4flash.parse_flash_size("nothing here"))

    def test_size_label(self):
        self.assertEqual(r4flash.size_label(0x800000), "8MB")
        self.assertEqual(r4flash.size_label(0x1234), "0x1234")

    def test_image_info_verdict(self):
        good = ("Chip ID: 9 (ESP32-S3)\nChecksum: 0x6d (valid)\n"
                "Validation hash: 8414 (valid)\n")
        bad = ("Chip ID: 9 (ESP32-S3)\nChecksum: 0x6d (invalid - calculated 0x92)\n"
               "Validation hash: b6fc (invalid)\n")
        self.assertTrue(r4flash.image_info_verdict(good)[0])
        self.assertFalse(r4flash.image_info_verdict(bad)[0])

    def test_distinct_boards(self):
        one = [{"serial_number": "A", "path": b"1"}, {"serial_number": "A", "path": b"2"}]
        two = [{"serial_number": "A", "path": b"1"}, {"serial_number": "B", "path": b"2"}]
        self.assertEqual(r4flash.distinct_boards(one), 1)
        self.assertEqual(r4flash.distinct_boards(two), 2)

    def test_unknown_shape_refused(self):
        with self.assertRaisesRegex(r4flash.Refusal, "unknown image shape"):
            r4flash.check_known_shape(0x1000, 0x0)
        with self.assertRaisesRegex(r4flash.Refusal, "unknown image shape"):
            r4flash.check_known_shape(0x1000, 0x9000)

    def test_empty_image_refused(self):
        with self.assertRaisesRegex(r4flash.Refusal, "empty"):
            r4flash.check_image(b"", 0, False)

    def test_parse_mac(self):
        text = "Chip type:          ESP32-S3\n" + MAC_LINE + f"{'BASE MAC:':<20}11:22:33:44:55:66\n"
        self.assertEqual(r4flash.parse_mac(text), FAKE_MAC)
        self.assertEqual(r4flash.parse_mac("MAC:  34:85:18:AA:BB:CC\r\n"), FAKE_MAC)
        self.assertIsNone(r4flash.parse_mac("BASE MAC:           11:22:33:44:55:66\n"))
        self.assertIsNone(r4flash.parse_mac("no mac here"))

    def test_mac_round_trips_through_backup_name(self):
        name = f"r4-esp32s3-backup-8MB-{r4flash.mac_tag(FAKE_MAC)}20261011T050000Z.bin"
        self.assertEqual(name, "r4-esp32s3-backup-8MB-mac348518aabbcc-20261011T050000Z.bin")
        self.assertEqual(r4flash.mac_from_backup_name(name), FAKE_MAC)
        self.assertIsNone(r4flash.mac_from_backup_name("r4-esp32s3-backup-8MB-X.bin"))
        self.assertEqual(r4flash.mac_tag(None), "")

    @unittest.skipUnless(r4flash.SYSTEM_CA_FILE.is_file(), "no /etc/ssl/cert.pem here")
    def test_https_context_has_ca_certificates(self):
        # The venv's python.org Python has none of its own; the download needs some.
        context = r4flash.https_context()
        self.assertGreater(context.cert_store_stats()["x509_ca"], 0)
        self.assertEqual(context.verify_mode, r4flash.ssl.CERT_REQUIRED)

    def run_main_capturing(self, argv):
        with mock.patch("sys.stderr", new_callable=io.StringIO) as err, \
                mock.patch("sys.stdout", new_callable=io.StringIO):
            code = r4flash.main(argv)
        return code, err.getvalue()

    def test_main_turns_unexpected_errors_into_failed(self):
        with mock.patch.object(r4flash, "cmd_ports", side_effect=RuntimeError("boom")):
            code, err = self.run_main_capturing(["ports"])
        self.assertEqual(code, r4flash.EXIT_FAIL)
        self.assertTrue(err.startswith("FAILED: unexpected RuntimeError: boom"), err)
        with mock.patch.object(r4flash, "cmd_ports", side_effect=IsADirectoryError("x")):
            code, err = self.run_main_capturing(["ports"])
        self.assertEqual(code, r4flash.EXIT_FAIL)
        self.assertEqual(err, "FAILED: IsADirectoryError: x\n")

    def test_main_interrupted_device_run_gives_recovery_advice(self):
        # main() starts a log for a real device run; _log_path is patched too, so the
        # module-level path is put back afterwards and later tests log nowhere.
        with tempfile.TemporaryDirectory() as tmp, \
                mock.patch.object(r4flash, "LOG_DIR", Path(tmp)), \
                mock.patch.object(r4flash, "_log_path", None), \
                mock.patch.object(r4flash, "cmd_verify", side_effect=KeyboardInterrupt):
            code, err = self.run_main_capturing(["verify", "x.bin"])
            logged = "".join(p.read_text() for p in Path(tmp).glob("*.log"))
        self.assertEqual(code, r4flash.EXIT_FAIL)
        self.assertIn("Do not unplug", err)
        self.assertIn("FAILED: interrupted", logged)

    def test_refusal_is_logged_once_a_log_exists(self):
        with tempfile.TemporaryDirectory() as tmp:
            log_path = Path(tmp) / "run.log"
            with mock.patch.object(r4flash, "_log_path", log_path), \
                    mock.patch.object(r4flash, "cmd_ports",
                                      side_effect=r4flash.Refusal("too late")):
                code, _ = self.run_main_capturing(["ports"])
                self.assertFalse(log_path.exists(), "nothing was sent, so no log")
                r4flash.log("RUN: something")
                code, _ = self.run_main_capturing(["ports"])
            self.assertEqual(code, REFUSED)
            self.assertIn("REFUSED: too late", log_path.read_text())

    def test_esptool_lines_use_esptool5_spellings(self):
        cmd = r4flash.esptool_cmd("/dev/cu.x", "no-reset", "flash-id")
        self.assertEqual(cmd[1:3], ["-m", "esptool"])
        for flag, value in (("--chip", "esp32s3"), ("--baud", "115200"),
                            ("--before", "default-reset"), ("--after", "no-reset")):
            self.assertEqual(cmd[cmd.index(flag) + 1], value)
        self.assertNotIn("-e", cmd)
        self.assertNotIn("--erase-all", cmd)


@unittest.skipUnless(have_build(), f"no build in {BUILD_DIR} (set R4FLASH_TEST_BUILD_DIR)")
class ImageTests(unittest.TestCase):
    """Guards and checks against real build output, mostly through the command line."""

    @classmethod
    def setUpClass(cls):
        cls.tmp = Path(tempfile.mkdtemp(prefix="r4flash-test-"))
        cls.files = {name: (BUILD_DIR / name).read_bytes() for name in NEEDED_FILES}
        cls.boot_app = cls.files["S3-BOOT-APP.bin"]
        cls.all = cls.files["S3-ALL.bin"]
        cls.backup = write_with_sidecar(cls.tmp / "backup-4MB.bin", cls.all)

    @classmethod
    def tearDownClass(cls):
        shutil.rmtree(cls.tmp, ignore_errors=True)

    def image(self, name, data):
        path = self.tmp / name
        path.write_bytes(data)
        return path

    def flash_dry(self, image, *extra):
        return run_tool("flash", image, "--backup", self.backup, "--dry-run", *extra)

    def assertRefused(self, result, text):
        code, out = result
        self.assertEqual(code, REFUSED, out)
        self.assertIn("REFUSED:", out)
        self.assertIn(text, out)

    # --- check -------------------------------------------------------------

    def test_check_passes_on_good_build(self):
        code, out = run_tool("check", "--build-dir", BUILD_DIR)
        self.assertEqual(code, 0, out)
        self.assertIn("check: OK", out)
        self.assertNotIn("\nFAIL", out)
        self.assertIn("PASS  cert slice at 0xB000 == x509_crt_bundle", out)
        self.assertRegex(out, r"PASS  esptool image-info on bootloader slice +checksum valid=True")

    def corrupt_build(self, name, changes):
        """A copy of the build in which the files named in 'changes' are replaced."""
        build = self.tmp / name
        build.mkdir()
        for file_name, data in self.files.items():
            (build / file_name).write_bytes(changes.get(file_name, data))
        return build

    def test_check_fails_on_corrupted_bootloader(self):
        boot_app, full = bytearray(self.boot_app), bytearray(self.all)
        boot_app[0x100] ^= 0xFF
        full[0x100] ^= 0xFF  # keep S3-ALL consistent, so only the bootloader rows fail
        build = self.corrupt_build("bad-bootloader-build", {"S3-BOOT-APP.bin": boot_app,
                                                            "S3-ALL.bin": full})
        code, out = run_tool("check", "--build-dir", build)
        self.assertEqual(code, r4flash.EXIT_FAIL, out)
        self.assertIn("FAIL  esptool image-info on bootloader slice", out)
        self.assertIn("the bootloader at 0x0 is corrupt", out)

    def test_check_fails_on_empty_files(self):
        build = self.corrupt_build("empty-build", {name: b"" for name in NEEDED_FILES})
        code, out = run_tool("check", "--build-dir", build)
        self.assertEqual(code, r4flash.EXIT_FAIL, out)
        self.assertIn("S3-ALL.bin present", out)
        self.assertIn("the file is empty", out)
        self.assertNotIn("PASS  S3-BOOT-APP == S3-ALL", out)
        self.assertNotIn("PASS  app fits app0", out)

    def test_check_fails_on_corrupted_boot_app(self):
        bad = bytearray(self.boot_app)
        bad[r4flash.APP0_OFFSET + 0x1000] ^= 0xFF
        build = self.corrupt_build("corrupt-build", {"S3-BOOT-APP.bin": bad})
        code, out = run_tool("check", "--build-dir", build)
        self.assertEqual(code, r4flash.EXIT_FAIL, out)
        self.assertIn("FAIL  S3-BOOT-APP == S3-ALL[:0x1E0000]", out)
        self.assertIn("check: FAILED", out)

    # --- flash: guard (1), the backup ---------------------------------------

    def test_flash_refuses_without_backup(self):
        result = run_tool("flash", BUILD_DIR / "S3-BOOT-APP.bin",
                          "--backup", self.tmp / "no-such-backup.bin", "--dry-run")
        self.assertRefused(result, "does not exist")

    def test_flash_refuses_missing_sidecar(self):
        backup = self.image("no-sidecar.bin", self.all)
        result = run_tool("flash", BUILD_DIR / "S3-BOOT-APP.bin", "--backup", backup,
                          "--dry-run")
        self.assertRefused(result, "sidecar")

    def test_flash_refuses_bad_sidecar(self):
        backup = self.image("bad-sidecar.bin", self.all)
        r4flash.write_sidecar(backup, "0" * 64)
        result = run_tool("flash", BUILD_DIR / "S3-BOOT-APP.bin", "--backup", backup,
                          "--dry-run")
        self.assertRefused(result, "sha256 mismatch")

    def test_flash_refuses_backup_of_wrong_size(self):
        backup = write_with_sidecar(self.tmp / "short-backup.bin", self.all[:0x100000])
        result = run_tool("flash", BUILD_DIR / "S3-BOOT-APP.bin", "--backup", backup,
                          "--dry-run")
        self.assertRefused(result, "a full backup is 0x400000")

    # --- flash: guards (2)-(7), the image ----------------------------------

    def test_flash_refuses_s3_all_without_flag(self):
        result = self.flash_dry(BUILD_DIR / "S3-ALL.bin")
        self.assertRefused(result, "--i-know-this-wipes-nvs-and-spiffs")

    def test_flash_allows_s3_all_with_flag(self):
        code, out = self.flash_dry(BUILD_DIR / "S3-ALL.bin", "--i-know-this-wipes-nvs-and-spiffs")
        self.assertEqual(code, 0, out)

    def test_flash_refuses_s3_app_at_0x0(self):
        result = self.flash_dry(BUILD_DIR / "S3-APP.bin", "--offset", "0x0")
        self.assertRefused(result, "must be written at 0xe000")

    def test_flash_refuses_s3_boot_app_at_0xe000(self):
        result = self.flash_dry(BUILD_DIR / "S3-BOOT-APP.bin", "--offset", "0xe000")
        self.assertRefused(result, "must be written at 0x0")

    def test_flash_allows_s3_app_at_0xe000(self):
        code, out = self.flash_dry(BUILD_DIR / "S3-APP.bin", "--offset", "0xE000")
        self.assertEqual(code, 0, out)
        self.assertIn("write-flash --flash-mode keep --flash-freq keep --flash-size keep "
                      "0xe000", out)
        self.assertIn("does not reset otadata", out)
        self.assertNotIn("WARNING", out)  # the backup's otadata is boot_app0

    def test_flash_app_only_refused_when_backup_has_foreign_table(self):
        bad = bytearray(self.all)
        bad[r4flash.PT_OFFSET + 5] ^= 0x01
        backup = write_with_sidecar(self.tmp / "backup-foreign-table.bin", bytes(bad))
        for image, offset in (("S3-APP.bin", "0xe000"), ("UNOR4USBBridge.ino.bin", "0x50000")):
            with self.subTest(image=image):
                result = run_tool("flash", BUILD_DIR / image, "--backup", backup,
                                  "--offset", offset, "--dry-run")
                self.assertRefused(result, "the backup's partition table at 0x8000")

    def test_flash_refuses_image_without_magic(self):
        bad = bytearray(self.boot_app)
        bad[0] = 0xFF
        result = self.flash_dry(self.image("no-magic.bin", bad))
        self.assertRefused(result, "must start with the bootloader magic 0xe9")

    def test_flash_refuses_corrupt_bootloader(self):
        bad = bytearray(self.boot_app)
        bad[0x100] ^= 0xFF  # inside the bootloader's first segment
        result = self.flash_dry(self.image("corrupt-bootloader.bin", bad))
        self.assertRefused(result, "the bootloader at 0x0 is corrupt")

    def test_flash_refuses_bootloader_with_changed_header(self):
        bad = bytearray(self.boot_app)
        bad[0x18] ^= 0x01  # first segment's load address: the checksum misses it, SHA-256 not
        result = self.flash_dry(self.image("bootloader-header.bin", bad))
        self.assertRefused(result, "bootloader at 0x0")

    def test_flash_refuses_wrong_otadata(self):
        bad = bytearray(self.boot_app)
        start = r4flash.OTADATA_OFFSET
        bad[start:start + r4flash.OTADATA_LEN] = bytes(r4flash.OTADATA_LEN)
        result = self.flash_dry(self.image("zero-otadata.bin", bad))
        self.assertRefused(result, "otadata at 0x9000")

    # --- guard edge cases, in-process -----------------------------------------

    def test_bare_app_size_limits(self):
        app = self.files["UNOR4USBBridge.ino.bin"]
        exact = app + b"\xff" * (r4flash.APP0_LEN - len(app))
        r4flash.check_image(exact, r4flash.APP0_OFFSET, False)  # exactly 0x190000: allowed
        with self.assertRaisesRegex(r4flash.Refusal, r"\(1 bytes too big\)"):
            r4flash.check_image(exact + b"\xff", r4flash.APP0_OFFSET, False)

    def test_tiny_images_refused_at_every_offset(self):
        for size in (0, 19, 20, 21):
            data = b"\xe9" + bytes(size - 1) if size else b""
            for offset in (0, r4flash.S3_APP_OFFSET, r4flash.APP0_OFFSET):
                with self.subTest(size=size, offset=offset):
                    with self.assertRaises(r4flash.Refusal):
                        r4flash.check_image(data, offset, False)

    def test_app_without_appended_sha256_is_refused_not_a_crash(self):
        # Header byte 23 says whether a SHA-256 follows the image; esptool's parser then
        # leaves data_length as None, which used to raise TypeError.
        app = bytearray(self.files["UNOR4USBBridge.ino.bin"])
        app[23] = 0
        with self.assertRaisesRegex(r4flash.Refusal, "no appended SHA-256"):
            r4flash.check_image(bytes(app), r4flash.APP0_OFFSET, False)
        boot_app = bytearray(self.boot_app)
        boot_app[r4flash.APP0_OFFSET + 23] = 0
        with self.assertRaisesRegex(r4flash.Refusal, "no appended SHA-256"):
            r4flash.check_image(bytes(boot_app), 0, False)

    def test_flash_refuses_image_past_4mb(self):
        result = self.flash_dry(self.image("eight-mb.bin", self.all + b"\xff" * 0x400000))
        self.assertRefused(result, "past 0x400000")

    def test_flash_refuses_unknown_shape(self):
        result = self.flash_dry(self.image("bootloader-only.bin", self.boot_app[:0x8000]))
        self.assertRefused(result, "unknown image shape")

    def test_flash_refuses_foreign_partition_table(self):
        bad = bytearray(self.boot_app)
        bad[r4flash.PT_OFFSET + 5] ^= 0x01  # first entry's offset: otadata 0x9000 -> 0x9100
        result = self.flash_dry(self.image("foreign-table.bin", bad))
        self.assertRefused(result, "partition table at 0x8000")

    def test_flash_refuses_oversize_bare_app(self):
        app = self.files["UNOR4USBBridge.ino.bin"]
        big = app + b"\xff" * (0x198000 - len(app))  # boards.txt's limit, 32 KiB too big
        result = self.flash_dry(self.image("big-app.bin", big), "--offset", "0x50000")
        self.assertRefused(result, "app0 holds only 0x190000")

    def test_flash_refuses_truncated_app_in_boot_app(self):
        # What combine.py makes from an app longer than app0: the image is cut at
        # 0x1E0000, so the app's own header claims more bytes than the slot holds.
        bad = bytearray(self.boot_app)
        first_segment_len = r4flash.APP0_OFFSET + 24 + 4
        struct.pack_into("<I", bad, first_segment_len, 0x190000)
        result = self.flash_dry(self.image("truncated-app.bin", bad))
        self.assertRefused(result, "does not parse as a complete ESP32-S3 image")

    def test_flash_refuses_corrupt_app(self):
        bad = bytearray(self.boot_app)
        bad[r4flash.APP0_OFFSET + 0x1000] ^= 0xFF
        result = self.flash_dry(self.image("corrupt-app.bin", bad))
        self.assertRefused(result, "is corrupt")

    def test_flash_allows_bare_app_at_0x50000(self):
        code, out = self.flash_dry(BUILD_DIR / "UNOR4USBBridge.ino.bin", "--offset", "0x50000")
        self.assertEqual(code, 0, out)
        self.assertIn("does not reset otadata", out)

    # --- flash: guard (8), the port, without --dry-run ----------------------

    def test_flash_refuses_port_that_is_not_download_mode(self):
        # Not a dry run: guards (1)-(7) pass, then guard (8) refuses because this port is
        # not a 303A:1001 port, so esptool is never started.
        logs_before = set(r4flash.LOG_DIR.glob("*"))
        result = run_tool("flash", BUILD_DIR / "S3-BOOT-APP.bin", "--backup", self.backup,
                          "--port", "/dev/cu.r4flash-test-no-such-port")
        self.assertRefused(result, "is not an ESP32-S3 download-mode port")
        self.assertEqual(set(r4flash.LOG_DIR.glob("*")), logs_before,
                         "a run refused before anything was sent must not leave a log")

    # --- flash: the happy path ----------------------------------------------

    def test_flash_happy_path_dry_run(self):
        code, out = self.flash_dry(BUILD_DIR / "S3-BOOT-APP.bin")
        self.assertEqual(code, 0, out)
        dry = [line for line in out.splitlines() if line.startswith("DRY: ")]
        self.assertEqual(len(dry), 3, out)  # flash-id, write-flash, verify-flash
        self.assertNotIn("RUN: ", out)
        for line in dry:
            self.assertIn("-m esptool --chip esp32s3", line)
            self.assertIn("--baud 115200", line)
            self.assertIn("--before default-reset", line)
        self.assertTrue(dry[0].endswith("--after no-reset flash-id"))
        self.assertIn("--after no-reset write-flash --flash-mode keep --flash-freq keep "
                      "--flash-size keep 0x0 ", dry[1])
        self.assertIn("--after hard-reset verify-flash 0x0 ", dry[2])
        self.assertIn("Pressing RST alone keeps the ESP32-S3 in download mode", out)
        for forbidden in (" -e ", "--erase-all", "erase-flash", "erase-region"):
            self.assertNotIn(forbidden, out)

    # --- restore and verify ---------------------------------------------------

    def test_restore_backup_dry_run(self):
        code, out = run_tool("restore", "--backup", self.backup, "--dry-run")
        self.assertEqual(code, 0, out)
        self.assertIn("--flash-size keep 0x0 ", out)

    def test_restore_8mb_backup_dry_run(self):
        backup = write_with_sidecar(self.tmp / "backup-8MB.bin", self.all + b"\xff" * 0x400000)
        code, out = run_tool("restore", "--backup", backup, "--dry-run")
        self.assertEqual(code, 0, out)

    def test_restore_refuses_backup_without_magic(self):
        bad = bytearray(self.all)
        bad[0] = 0x00
        backup = write_with_sidecar(self.tmp / "backup-no-magic.bin", bytes(bad))
        result = run_tool("restore", "--backup", backup, "--dry-run")
        self.assertRefused(result, "magic 0xe9")

    def test_restore_official_with_safety_backup_checks_it(self):
        result = run_tool("restore", "--official", "--backup", self.tmp / "missing.bin",
                          "--dry-run", "--cache-dir", self.tmp / "unused-cache")
        self.assertRefused(result, "does not exist")

    def test_verify_dry_run(self):
        code, out = run_tool("verify", BUILD_DIR / "S3-BOOT-APP.bin", "--dry-run")
        self.assertEqual(code, 0, out)
        self.assertIn("--after no-reset verify-flash 0x0 ", out)


class FakeEsptool:
    """Stands in for r4flash.run() so the real (non-dry) flows can be exercised without a
    board. It records each command, answers like esptool 5.5.0 would (including the
    'MAC:' line printed on connecting), and keeps the bytes of each image it was given."""

    def __init__(self, flash_size=0x400000, read_size=None, mac=FAKE_MAC, on_flash_id=None):
        self.flash_size = flash_size
        self.read_size = flash_size if read_size is None else read_size
        self.mac = mac
        self.on_flash_id = on_flash_id
        self.commands = []
        self.received = {}  # 'write-flash' / 'verify-flash' -> (path, bytes at call time)

    def __call__(self, cmd, dry_run):
        args = [str(part) for part in cmd]
        self.commands.append(args)
        connect = f"{'MAC:':<20}{self.mac}\n" if self.mac else ""
        detected = connect + f"Detected flash size: {self.flash_size // r4flash.MIB}MB\n"
        if "read-flash" in args:
            Path(args[-1]).write_bytes(b"\xe9" + b"\xff" * (self.read_size - 1))
            return detected
        if "flash-id" in args:
            if self.on_flash_id:
                self.on_flash_id()
            return "Manufacturer: 20\n" + detected
        for verb in ("write-flash", "verify-flash"):
            if verb in args:
                path = Path(args[-1])
                self.received[verb] = (path, path.read_bytes() if path.is_file() else None)
        return connect + "Verification successful (digest matched).\n"

    def verbs(self):
        verbs = ("flash-id", "read-flash", "write-flash", "verify-flash")
        return [next(v for v in verbs if v in cmd) for cmd in self.commands]


class FakeHidDevice:
    calls = []

    def open_path(self, path):
        self.calls.append(("open_path", path))

    def send_feature_report(self, data):
        self.calls.append(("send_feature_report", bytes(data)))
        return len(data)

    def close(self):
        self.calls.append(("close",))


def real_run_args(**extra):
    """Arguments for a non-dry run; the port is patched, so nothing real is opened."""
    return argparse.Namespace(dry_run=False, port=None, **extra)


def backup_of(size, mac=None):
    return r4flash.Backup(Path("backup.bin"), size, mac, b"")


IMAGE = b"\xe9" + b"checked image bytes"


class RealFlowTests(unittest.TestCase):
    """The non-dry code paths, with esptool, the port and hidapi replaced by fakes."""

    def setUp(self):
        self.tmp = Path(tempfile.mkdtemp(prefix="r4flash-test-"))
        port = mock.patch.object(r4flash, "resolve_download_port", return_value="/dev/fake")
        port.start()
        self.addCleanup(port.stop)
        quiet = mock.patch("sys.stdout", new_callable=io.StringIO)  # keep test output clean
        self.out = quiet.start()
        self.addCleanup(quiet.stop)

    def tearDown(self):
        shutil.rmtree(self.tmp, ignore_errors=True)

    def write(self, fake, backup, data=IMAGE):
        with mock.patch.object(r4flash, "run", fake):
            r4flash.write_and_verify(real_run_args(), "/dev/fake", data, Path("img.bin"),
                                     0, backup)

    # --- backup ---------------------------------------------------------------

    def test_backup_names_checks_and_copies(self):
        fake = FakeEsptool(0x400000)
        with mock.patch.object(r4flash, "run", fake):
            code = r4flash.cmd_backup(real_run_args(out=str(self.tmp)))
        self.assertEqual(code, 0)
        self.assertEqual(fake.verbs(), ["flash-id", "read-flash", "verify-flash"])
        self.assertIn(["read-flash", "0", "ALL"], [c[-4:-1] for c in fake.commands])
        backups = sorted(self.tmp.glob("r4-esp32s3-backup-4MB-mac348518aabbcc-*Z.bin"))
        self.assertEqual(len(backups), 1)
        backup = r4flash.check_backup_file(backups[0])
        self.assertEqual((backup.size, backup.mac), (0x400000, FAKE_MAC))
        copy = self.tmp / "copy2" / backups[0].name
        self.assertEqual(r4flash.check_backup_file(copy).size, 0x400000)
        self.assertEqual(list(self.tmp.glob("*.partial")), [])

    def test_backup_without_a_mac_line_still_works(self):
        fake = FakeEsptool(0x400000, mac=None)
        with mock.patch.object(r4flash, "run", fake):
            r4flash.cmd_backup(real_run_args(out=str(self.tmp)))
        backups = list(self.tmp.glob("r4-esp32s3-backup-4MB-2*Z.bin"))
        self.assertEqual(len(backups), 1)
        self.assertIsNone(r4flash.check_backup_file(backups[0]).mac)

    def test_backup_refuses_short_read(self):
        fake = FakeEsptool(0x800000, read_size=0x400000)
        with mock.patch.object(r4flash, "run", fake):
            with self.assertRaisesRegex(r4flash.Refusal, "but the chip reports"):
                r4flash.cmd_backup(real_run_args(out=str(self.tmp)))
        self.assertEqual(fake.verbs(), ["flash-id", "read-flash"])
        self.assertEqual(list(self.tmp.glob("*.bin")), [])  # only the .partial is left

    # --- write and verify -------------------------------------------------------

    def test_write_refuses_backup_from_other_size_chip(self):
        fake = FakeEsptool(0x800000)
        with self.assertRaisesRegex(r4flash.Refusal, "not a backup of this board"):
            self.write(fake, backup_of(0x400000))
        self.assertEqual(fake.verbs(), ["flash-id"])

    def test_write_refuses_backup_from_other_chip_mac(self):
        fake = FakeEsptool(0x800000)
        with self.assertRaisesRegex(r4flash.Refusal, "another board's backup"):
            self.write(fake, backup_of(0x800000, "00:11:22:33:44:55"))
        self.assertEqual(fake.verbs(), ["flash-id"])

    def test_write_then_verify_with_hard_reset(self):
        fake = FakeEsptool(0x800000)
        self.write(fake, backup_of(0x800000, FAKE_MAC))
        self.assertEqual(fake.verbs(), ["flash-id", "write-flash", "verify-flash"])
        write, verify = fake.commands[1], fake.commands[2]
        self.assertEqual(write[write.index("--after") + 1], "no-reset")
        self.assertEqual(verify[verify.index("--after") + 1], "hard-reset")
        self.assertEqual(write[-6:-1], ["--flash-freq", "keep", "--flash-size", "keep", "0x0"])

    def test_esptool_gets_a_private_copy_of_the_checked_bytes(self):
        fake = FakeEsptool(0x800000)
        self.write(fake, backup_of(0x800000))
        for verb in ("write-flash", "verify-flash"):
            path, data = fake.received[verb]
            self.assertEqual(data, IMAGE, verb)
            self.assertEqual(path.name, "img.bin")
            self.assertNotEqual(path, Path("img.bin"))
        self.assertFalse(path.parent.exists(), "the staging folder is removed afterwards")

    @unittest.skipUnless(have_build(), f"no build in {BUILD_DIR}")
    def test_flash_writes_the_checked_bytes_even_if_the_file_changes(self):
        image = self.tmp / "S3-BOOT-APP.bin"
        checked = (BUILD_DIR / "S3-BOOT-APP.bin").read_bytes()
        image.write_bytes(checked)
        full = (BUILD_DIR / "S3-ALL.bin").read_bytes()
        backup = write_with_sidecar(self.tmp / "backup.bin", full)
        # A rebuild lands between the guards and write-flash.
        fake = FakeEsptool(0x400000, on_flash_id=lambda: image.write_bytes(bytes(len(checked))))
        args = real_run_args(image=str(image), backup=str(backup), offset=0,
                             i_know_this_wipes_nvs_and_spiffs=False)
        with mock.patch.object(r4flash, "run", fake):
            self.assertEqual(r4flash.cmd_flash(args), 0)
        self.assertEqual(fake.received["write-flash"][1], checked)
        self.assertEqual(fake.received["verify-flash"][1], checked)

    def test_restore_backup_writes_the_backup_on_its_own_chip(self):
        name = f"r4-esp32s3-backup-4MB-{r4flash.mac_tag(FAKE_MAC)}20261011T000000Z.bin"
        data = b"\xe9" + b"\xff" * (0x400000 - 1)
        backup = write_with_sidecar(self.tmp / name, data)
        fake = FakeEsptool(0x400000)
        with mock.patch.object(r4flash, "run", fake):
            code = r4flash.cmd_restore(real_run_args(official=False, backup=str(backup)))
        self.assertEqual(code, 0)
        self.assertEqual(fake.received["write-flash"][1], data)
        self.assertIn("WARNING: in this backup, the bootloader at 0x0", self.out.getvalue())

    def failing_write(self, failing_verb):
        fake = FakeEsptool(0x800000)

        def failing_run(cmd, dry_run):
            if failing_verb in [str(part) for part in cmd]:
                raise r4flash.Failure(f"exit code 2 from: {failing_verb}")
            return fake(cmd, dry_run)

        return failing_run

    def test_failed_write_says_not_to_unplug(self):
        with self.assertRaisesRegex(r4flash.Failure, "Do not unplug yet"):
            self.write(self.failing_write("write-flash"), backup_of(0x800000))

    def test_failed_verify_says_the_chip_was_reset(self):
        with self.assertRaises(r4flash.Failure) as caught:
            self.write(self.failing_write("verify-flash"), backup_of(0x800000))
        self.assertIn("probably left download mode", str(caught.exception))
        self.assertNotIn("should still be in download mode", str(caught.exception))

    # --- restore --official ------------------------------------------------------

    def test_restore_official_looks_for_the_port_before_downloading(self):
        no_port = mock.patch.object(r4flash, "resolve_download_port",
                                    side_effect=r4flash.Refusal("no port"))
        never = mock.patch.object(r4flash.urllib.request, "urlopen",
                                  side_effect=AssertionError("downloaded first"))
        args = real_run_args(official=True, backup=None, cache_dir=str(self.tmp))
        with no_port, never as urlopen:
            with self.assertRaisesRegex(r4flash.Refusal, "no port"):
                r4flash.cmd_restore(args)
        urlopen.assert_not_called()
        self.assertEqual(list(self.tmp.iterdir()), [])

    @unittest.skipUnless(OFFICIAL_ZIP.is_file(), "no official 0.6.0 zip to test with")
    def test_official_download_checks_and_extracts(self):
        # urlopen is replaced by the zip's bytes, so no network is used.
        zip_bytes = io.BytesIO(OFFICIAL_ZIP.read_bytes())
        with mock.patch.object(r4flash.urllib.request, "urlopen",
                               return_value=zip_bytes) as urlopen:
            path, data = r4flash.official_image(self.tmp, dry_run=False)
        self.assertIsInstance(urlopen.call_args.kwargs["context"], r4flash.ssl.SSLContext)
        self.assertEqual(path, self.tmp / r4flash.OFFICIAL_IMAGE_NAME)
        self.assertEqual(r4flash.sha256_file(path), r4flash.OFFICIAL_IMAGE_SHA256)
        self.assertEqual(len(data), r4flash.S3_BOOT_APP_LEN)
        r4flash.check_image(data, 0, allow_wipe=False)

    def test_official_download_with_wrong_sha_is_discarded(self):
        fake_download = io.BytesIO(b"this is not the release zip")
        with mock.patch.object(r4flash.urllib.request, "urlopen", return_value=fake_download):
            with self.assertRaisesRegex(r4flash.Failure, "downloaded zip has sha256"):
                r4flash.official_image(self.tmp, dry_run=False)
        self.assertEqual(list(self.tmp.iterdir()), [])

    @unittest.skipUnless(os.environ.get("R4FLASH_TEST_NETWORK") == "1",
                         "set R4FLASH_TEST_NETWORK=1 to download from GitHub")
    def test_real_download_of_the_official_zip(self):
        zip_path = self.tmp / r4flash.OFFICIAL_ZIP_NAME
        r4flash.download_official_zip(zip_path)
        self.assertEqual(r4flash.sha256_file(zip_path), r4flash.OFFICIAL_ZIP_SHA256)

    # --- enter-download -------------------------------------------------------

    def test_send_reboot_report_bytes(self):
        FakeHidDevice.calls = []
        fake_hid = types.SimpleNamespace(device=FakeHidDevice)
        with mock.patch.dict(sys.modules, {"hid": fake_hid}):
            self.assertTrue(r4flash.send_reboot_report(b"fake-path"))
        sent = FakeHidDevice.calls[1][1]
        self.assertEqual(FakeHidDevice.calls[0], ("open_path", b"fake-path"))
        self.assertEqual(sent, bytes([0x00, 0xAA]) + bytes(63))
        self.assertEqual(FakeHidDevice.calls[2], ("close",))

    def enter_download(self, port_found, existing=()):
        board = [{"path": b"fake-path", "serial_number": "X"}]
        found = [types.SimpleNamespace(device=device) for device in existing]
        patches = (mock.patch.object(r4flash, "ports_with", return_value=found),
                   mock.patch.object(r4flash, "bridge_hid_devices", return_value=board),
                   mock.patch.object(r4flash, "send_reboot_report", return_value=False),
                   mock.patch.object(r4flash, "wait_for_download_port",
                                     return_value=port_found))
        mocks = [patch.start() for patch in patches]
        for patch in patches:
            self.addCleanup(patch.stop)
        return r4flash.cmd_enter_download(real_run_args()), mocks

    def test_enter_download_waits_for_port_even_if_hidapi_errors(self):
        self.assertEqual(self.enter_download("/dev/cu.usbmodem-fake")[0], 0)

    def test_enter_download_fails_when_no_port_appears(self):
        self.assertEqual(self.enter_download(None)[0], r4flash.EXIT_FAIL)

    def test_enter_download_reboots_the_r4_despite_another_esp32_port(self):
        code, mocks = self.enter_download("/dev/cu.r4", existing=["/dev/cu.other-esp32"])
        self.assertEqual(code, 0)
        mocks[2].assert_called_once_with(b"fake-path")
        self.assertEqual(mocks[3].call_args.kwargs["ignore"], ["/dev/cu.other-esp32"])
        self.assertIn("pass --port /dev/cu.r4", self.out.getvalue())

    def test_wait_for_download_port_ignores_ports_seen_before(self):
        ports = [types.SimpleNamespace(device="/dev/cu.old"),
                 types.SimpleNamespace(device="/dev/cu.new")]
        with mock.patch.object(r4flash, "ports_with", return_value=ports):
            found = r4flash.wait_for_download_port(1.0, ignore=["/dev/cu.old"])
        self.assertEqual(found, "/dev/cu.new")


class NoBoardCommandTests(unittest.TestCase):
    """Commands that only list devices, or dry runs that need no build."""

    def setUp(self):
        self.tmp = Path(tempfile.mkdtemp(prefix="r4flash-test-"))

    def tearDown(self):
        shutil.rmtree(self.tmp, ignore_errors=True)

    def test_ports(self):
        code, out = run_tool("ports")
        self.assertEqual(code, 0, out)
        self.assertIn("HID 2341:1002", out)
        if not board_attached():
            self.assertIn("no device found", out)

    def test_enter_download_dry_run(self):
        code, out = run_tool("enter-download", "--dry-run")
        self.assertEqual(code, 0, out)
        if not board_attached():
            self.assertIn("no device found", out)

    def test_backup_dry_run_writes_nothing(self):
        out_dir = self.tmp / "backups"
        code, out = run_tool("backup", "--out", out_dir, "--dry-run")
        self.assertEqual(code, 0, out)
        self.assertIn("--after no-reset read-flash 0 ALL ", out)
        self.assertIn("--after no-reset verify-flash 0x0 ", out)
        self.assertFalse(out_dir.exists())

    def test_restore_official_dry_run_empty_cache_downloads_nothing(self):
        cache = self.tmp / "cache"
        cache.mkdir()
        code, out = run_tool("restore", "--official", "--dry-run", "--cache-dir", cache)
        self.assertEqual(code, 0, out)
        self.assertIn(f"DRY: download {r4flash.OFFICIAL_ZIP_URL}", out)
        self.assertIn("WARNING: no --backup given", out)
        self.assertIn("--flash-size keep 0x0 ", out)
        self.assertEqual(list(cache.iterdir()), [])

    @unittest.skipUnless(OFFICIAL_ZIP.is_file(), "no official 0.6.0 zip to test with")
    def test_restore_official_dry_run_with_cached_zip(self):
        cache = self.tmp / "cache"
        cache.mkdir()
        shutil.copyfile(OFFICIAL_ZIP, cache / r4flash.OFFICIAL_ZIP_NAME)
        code, out = run_tool("restore", "--official", "--dry-run", "--cache-dir", cache)
        self.assertEqual(code, 0, out)
        self.assertIn("guards passed: UNOR4-WIFI-S3-0.6.0.bin, 0x1e0000 bytes at 0x0", out)
        self.assertIn("--after hard-reset verify-flash 0x0 ", out)
        # A dry run reads the cached zip but writes nothing into the cache.
        self.assertEqual([p.name for p in cache.iterdir()], [r4flash.OFFICIAL_ZIP_NAME])

    def test_restore_official_refuses_corrupt_cached_zip(self):
        cache = self.tmp / "cache"
        cache.mkdir()
        (cache / r4flash.OFFICIAL_ZIP_NAME).write_bytes(b"not the real zip")
        result = run_tool("restore", "--official", "--dry-run", "--cache-dir", cache)
        self.assertEqual(result[0], REFUSED, result[1])
        self.assertIn("Delete it and run again", result[1])

    def test_flash_refuses_garbage_sidecar_without_a_traceback(self):
        backup = self.tmp / "backup.bin"
        backup.write_bytes(b"\xe9" + bytes(0x3FFFFF))
        r4flash.sidecar_path(backup).write_bytes(b"\xff\xfe\x00garbage")
        code, out = run_tool("flash", self.tmp / "img.bin", "--backup", backup, "--dry-run")
        self.assertEqual(code, REFUSED, out)
        self.assertIn("does not start with a sha256 hex digest", out)
        self.assertNotIn("Traceback", out)

    def test_check_dry_run_says_it_changes_nothing(self):
        code, out = run_tool("check", "--dry-run", "--build-dir", self.tmp / "no-build")
        self.assertEqual(code, r4flash.EXIT_FAIL, out)  # nothing to check there
        self.assertIn("--dry-run changes nothing here", out)

    def test_restore_needs_a_source(self):
        code, out = run_tool("restore", "--dry-run")
        self.assertEqual(code, REFUSED, out)
        self.assertIn("--official or --backup", out)


if __name__ == "__main__":
    unittest.main()

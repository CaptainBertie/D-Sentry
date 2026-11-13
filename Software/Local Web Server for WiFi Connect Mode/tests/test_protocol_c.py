# SPDX-License-Identifier: MIT
"""The generated C header against the generated Python module: compiles
tests/unit/test_protocol_c.cpp with clang++, runs its self-test, then compares its encoder and
deframer with protocol.py on the spec examples and on a random mixed byte stream. Also compiles
the header as C and with the two firmware cross compilers when they are installed."""

import subprocess
import unittest

import _support as S
from dsentry import protocol as P

SPEC = S.load_spec()
STRICT = ["-Wall", "-Wextra", "-Wpedantic", "-Wconversion", "-Wsign-conversion", "-Wshadow", "-Werror"]

PROBE = """#include "dsentry_protocol.h"
int ds_probe(void);
int ds_probe(void) {
  ds_deframer_t d;
  ds_control_packet_t p;
  uint8_t f[DS_CONTROL_FRAME_LEN];
  ds_deframer_init(&d);
  ds_control_init(&p);
  ds_control_encode(&p, f);
  return ds_deframer_feed(&d, f[0]);
}
"""


def run(cmd, **kw):
    return subprocess.run(cmd, capture_output=True, text=True, timeout=120, **kw)


@unittest.skipUnless(S.CLANGXX, "clang++ not found")
class TestCHeaderAgainstPython(unittest.TestCase):
    exe = None

    @classmethod
    def setUpClass(cls):
        out = S.build_dir() / "test_protocol_c"
        r = run([S.CLANGXX, "-std=gnu++17", "-O1", *STRICT, "-I", str(S.HEADER.parent),
                 str(S.CTEST), "-o", str(out)])
        if r.returncode != 0:
            raise AssertionError("test_protocol_c.cpp did not compile:\n" + r.stderr[-3000:])
        cls.exe = str(out)

    def test_self_test(self):
        r = run([self.exe])
        self.assertEqual(r.returncode, 0, r.stdout + r.stderr)
        self.assertIn("PASS  test_protocol_c", r.stdout)

    def test_examples_encode_identically(self):
        r = run([self.exe, "--dump"])
        self.assertEqual(r.returncode, 0)
        lines = dict(line.split(" ", 1) for line in r.stdout.splitlines() if line.startswith("CRC"))
        self.assertEqual(lines["CRC"], "%02x" % P.crc8(b"123456789"))
        enc = dict(line.split(" ")[1:3] for line in r.stdout.splitlines() if line.startswith("ENC "))
        self.assertEqual(sorted(enc), sorted(c.__name__ for c in P.PACKET_CLASSES))
        for cls in P.PACKET_CLASSES:
            self.assertEqual(enc[cls.__name__], cls(**P.EXAMPLES[cls.__name__]).encode().hex(), cls.__name__)

    def test_deframer_matches_python(self):
        rng = S.seeded(424242)
        stream = S.mixed_stream(P, SPEC, rng, pieces=600)
        r = run([self.exe, "--deframe"], input=stream.hex())
        self.assertEqual(r.returncode, 0, r.stderr)
        py = P.Deframer()
        expected = py.feed(stream)
        got = [line.split() for line in r.stdout.splitlines() if line.startswith("FRAME ")]
        self.assertEqual(len(got), len(expected))
        self.assertGreater(len(expected), 200)
        for words, frame in zip(got, expected):
            self.assertEqual(words[1], type(frame.packet).__name__)
            self.assertEqual(words[2], frame.raw.hex())
            self.assertEqual(words[-1], "roundtrip=ok")
            fields = dict(w.split("=", 1) for w in words[3:-1])
            for name, value in frame.packet.to_dict().items():
                if isinstance(value, str):
                    value = value.encode("latin-1").hex()
                elif isinstance(value, tuple):
                    value = bytes(value).hex()
                self.assertEqual(fields[name], str(value), "%s.%s" % (words[1], name))
        stats = dict(w.split("=") for w in r.stdout.splitlines()[-1].split()[1:])
        self.assertEqual({k: int(v) for k, v in stats.items()},
                         {"frames": py.frames, "crc": py.crc_errors, "len": py.len_errors,
                          "junk": py.junk_bytes, "held": py.held})
        self.assertGreater(py.crc_errors, 0)
        self.assertGreater(py.len_errors, 0)


class TestHeaderCompilers(unittest.TestCase):
    """dsentry_protocol.h must build as C and for both firmware targets (syntax only)."""

    def probe(self, suffix):
        p = S.build_dir() / ("header_probe" + suffix)
        p.write_text(PROBE)
        return str(p)

    def compile_ok(self, cmd):
        r = run(cmd)
        self.assertEqual(r.returncode, 0, " ".join(cmd) + "\n" + r.stderr[-3000:])

    @unittest.skipUnless(S.CLANG, "clang not found")
    def test_c11_and_gnu99(self):
        for std in ("c11", "gnu99"):
            self.compile_ok([S.CLANG, "-std=" + std, *STRICT, "-I", str(S.HEADER.parent),
                             "-fsyntax-only", self.probe(".c")])

    @unittest.skipUnless(S.ARM_GXX, "arm-none-eabi-g++ (Arduino renesas toolchain) not found")
    def test_ra4m1_toolchain(self):
        # the flags arduino:renesas_uno 1.6.0 compiles sketches with, plus -Werror
        self.compile_ok([S.ARM_GXX, "-std=gnu++17", "-mcpu=cortex-m4", "-mthumb", "-mfloat-abi=hard",
                         "-mfpu=fpv4-sp-d16", "-fsigned-char", "-fno-rtti", "-fno-exceptions",
                         "-Wall", "-Wextra", "-Werror", "-I", str(S.HEADER.parent), "-fsyntax-only",
                         self.probe(".cpp")])

    @unittest.skipUnless(S.XTENSA_GXX, "xtensa-esp32s3-elf-g++ (vendored ESP32 core) not found")
    def test_esp32_toolchain(self):
        # the ESP32 copy, with the flags arduino-esp32 2.0.9 uses for esp32s3, plus -Werror
        self.compile_ok([S.XTENSA_GXX, "-std=gnu++17", "-mlongcalls", "-fno-rtti", "-Wall", "-Wextra",
                         "-Werror", "-I", str(S.ESP32_HEADER.parent), "-fsyntax-only", self.probe(".cpp")])


if __name__ == "__main__":
    unittest.main()

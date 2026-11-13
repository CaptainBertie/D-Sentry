# SPDX-License-Identifier: MIT
"""The RA4M1's Telemetry encoder (firmware-ra4m1/dsentry/src/Telemetry.cpp) against the
generated Python codec (dsentry/protocol.py), byte for byte.

Builds tests/unit/test_telemetry.cpp with every portable RA4M1 module and the host HAL (the same
way tests/run.sh does), runs its unit tests, then runs `test_telemetry --dump`, which prints one
encoded TelemetryPacket per named snapshot. Every frame must decode with protocol.py (start, len,
CRC), equal the values below field by field, and equal protocol.py's own encoding of those values
byte for byte. The expected values are derived here from the contract (docs/architecture.md 4.1,
protocol_spec.py): centidegrees rounded half away from zero and saturated to the field, NaN as 0,
rates saturated, event counters wrapped to 16 bits, LCD NULs sent as spaces.
"""

import concurrent.futures
import os
import subprocess
import unittest
from pathlib import Path

import _support as S
from dsentry import protocol as P

SPEC = S.load_spec()
SRC = S.DSC / "firmware-ra4m1" / "dsentry" / "src"
SIM = S.DSC / "sim"
UNIT = S.DSC / "tests" / "unit"
FLAGS = ["-std=gnu++17", "-O1", "-Wall", "-Wextra", "-Werror"]
INCLUDES = ["-I", str(SRC), "-I", str(SIM), "-I", str(SIM / "host_include"), "-I", str(UNIT),
            "-I", str(S.DSC / "protocol")]

I16_MIN, I16_MAX = -0x8000, 0x7FFF
I32_MIN, I32_MAX = -0x80000000, 0x7FFFFFFF


def run(cmd, **kw):
    return subprocess.run(cmd, capture_output=True, text=True, timeout=300, **kw)


def spec_flag_masks():
    """camelCase key -> mask, from protocol_spec.py itself (not from the generated module)."""
    group = next(b for b in SPEC.BITS if b["name"] == "TelemetryFlags")
    return {key: 1 << bit for key, bit, _doc in group["bits"]}


def expected_cases():
    base = P.TelemetryPacket().to_dict()
    base["espMode"] = int(P.EspMode.UNKNOWN)   # TelemetrySnapshot's default: no DSS reply yet
    cases = {
        "example": dict(P.EXAMPLES["TelemetryPacket"]),
        "rounding": dict(base, seq=1, panCdeg=13, tiltCdeg=-13, panVelCdegS=38, tiltVelCdegS=-38,
                         homePanCdeg=12, homeTiltCdeg=-13, servoDeg=180),
        "saturate_hi": dict(base, seq=255, uptimeMs=0xFFFFFFFF, panCdeg=I32_MAX, tiltCdeg=I32_MAX,
                            panVelCdegS=I16_MAX, tiltVelCdegS=I16_MAX, homePanCdeg=1_000_000_000,
                            homeTiltCdeg=I32_MAX, ammo=255, magCapacity=255, shotsTotal=0xFFFF,
                            flywheelDuty=255, servoDeg=255, stallSample=1023, stallBaseline=1023,
                            stallPeak=1023, loopHz=0xFFFF, ctrlRxHz=0xFFFF,
                            crcErrors=70000 % 0x10000, seqGaps=0xFFFF, stepJitterPermille=0xFFFF,
                            lastCtrlSeq=255, espMode=int(P.EspMode.UNKNOWN), ledR=127, ledG=127,
                            ledB=127),
        "saturate_lo": dict(base, seq=0, panCdeg=I32_MIN, tiltCdeg=I32_MIN, panVelCdegS=I16_MIN,
                            tiltVelCdegS=I16_MIN, homePanCdeg=0, homeTiltCdeg=-1_000_000_000,
                            servoDeg=0, crcErrors=0),
        "nan": dict(base, seq=2),
        "lcd_nul": dict(base, seq=4, lcd="SYSTEM OFF".ljust(P.DS_TELEMETRY_LCD_LEN)),
        # the controller block at its extremes (JOY_AXIS_RANGE 512, TRIGGER_AXIS_RANGE 1023)
        "pad_extremes": dict(base, seq=5, padState=int(P.PadState.BLOCKED_BY_HCI), padBattery=100,
                             padLx=-512, padLy=511, padRx=511, padRy=-512, padLt=1023, padRt=0,
                             padButtons=0xFFFF),
    }
    for key, mask in spec_flag_masks().items():
        cases["flag_" + key] = dict(base, seq=3, flags=mask)
    return cases


def out_dir():
    """$DSENTRY_TEST_BUILD (the relay tests' own override), else tests/run.sh's
    $DSENTRY_TESTS_BUILD_DIR/relay-pc (so parallel runs with their own build dirs never share
    objects), else the default tests/build/relay-pc."""
    if not os.environ.get("DSENTRY_TEST_BUILD") and os.environ.get("DSENTRY_TESTS_BUILD_DIR"):
        d = Path(os.environ["DSENTRY_TESTS_BUILD_DIR"]) / "relay-pc"
        d.mkdir(parents=True, exist_ok=True)
        return d
    return S.build_dir()


def compile_all(out):
    """Every portable module + hal_host + test_telemetry.cpp -> one host binary (parallel)."""
    obj_dir = out.parent / (out.name + "_obj")
    obj_dir.mkdir(parents=True, exist_ok=True)
    sources = [p for p in sorted(SRC.glob("*.cpp")) if p.name != "hal_arduino.cpp"]
    sources += [SIM / "hal_host.cpp", UNIT / "test_telemetry.cpp"]

    def one(src):
        obj = obj_dir / (src.stem + ".o")
        r = run([S.CLANGXX, *FLAGS, *INCLUDES, "-c", str(src), "-o", str(obj)])
        return src, obj, r

    objects = []
    with concurrent.futures.ThreadPoolExecutor(max_workers=os.cpu_count() or 4) as pool:
        for src, obj, r in pool.map(one, sources):
            if r.returncode != 0:
                raise AssertionError("%s did not compile:\n%s" % (src.name, r.stderr[-3000:]))
            objects.append(str(obj))
    r = run([S.CLANGXX, *FLAGS, *objects, "-o", str(out)])
    if r.returncode != 0:
        raise AssertionError("test_telemetry did not link:\n" + r.stderr[-3000:])


@unittest.skipUnless(S.CLANGXX, "clang++ not found")
class TestTelemetryEncoderAgainstPython(unittest.TestCase):
    exe = None
    frames = None

    @classmethod
    def setUpClass(cls):
        out = out_dir() / "test_telemetry"
        compile_all(out)
        cls.exe = str(out)
        r = run([cls.exe, "--dump"])
        if r.returncode != 0:
            raise AssertionError("test_telemetry --dump failed:\n" + r.stdout + r.stderr)
        cls.frames = {}
        for line in r.stdout.splitlines():
            if line.startswith("CASE "):
                _tag, name, hexframe = line.split()
                cls.frames[name] = bytes.fromhex(hexframe)

    def test_unit_tests_pass(self):
        r = run([self.exe])
        self.assertEqual(r.returncode, 0, r.stdout + r.stderr)
        self.assertRegex(r.stdout, r"info test_telemetry: \d+ tests, \d+ checks, 0 failures")

    def test_every_case_is_dumped_and_expected(self):
        # the C++ cases and the expectations here must stay in step
        self.assertEqual(sorted(self.frames), sorted(expected_cases()))

    def test_frames_decode_to_the_contract_values(self):
        for name, want in expected_cases().items():
            with self.subTest(case=name):
                frame = self.frames[name]
                self.assertEqual(len(frame), P.TelemetryPacket.FRAME_LEN)
                got = P.TelemetryPacket.decode(frame)   # start, len and CRC checked here
                self.assertEqual(got.to_dict(), want)

    def test_frames_are_byte_identical_to_the_python_encoder(self):
        for name, want in expected_cases().items():
            with self.subTest(case=name):
                self.assertEqual(self.frames[name].hex(), P.TelemetryPacket(**want).encode().hex())

    def test_example_frame_is_the_spec_example(self):
        spec_example = next(p for p in SPEC.PACKETS if p["name"] == "TelemetryPacket")["example"]
        self.assertEqual(P.TelemetryPacket.decode(self.frames["example"]).to_dict(), spec_example)

    def test_flag_cases_cover_every_bit_once(self):
        masks = spec_flag_masks()
        seen = 0
        for key, mask in masks.items():
            flags = P.TelemetryPacket.decode(self.frames["flag_" + key]).flags
            self.assertEqual(flags, mask, key)
            self.assertEqual(P.bits_to_dict("TelemetryFlags", flags)[key], True)
            seen |= flags
        self.assertEqual(seen, sum(masks.values()))
        self.assertEqual(seen, sum(mask for _key, mask in P.BIT_KEYS["TelemetryFlags"]))

    def test_frames_survive_the_usb_deframer_between_debug_text(self):
        names = sorted(self.frames)
        stream = bytearray()
        for name in names:
            stream += b"12345 LCD row 1: \"WIFI\" + \"Pairing . . .\" = 18 > 16 columns\r\n"
            stream += self.frames[name]
        frames = P.Deframer().feed(bytes(stream))
        self.assertEqual([f.raw for f in frames], [self.frames[n] for n in names])
        self.assertTrue(all(isinstance(f.packet, P.TelemetryPacket) for f in frames))


if __name__ == "__main__":
    unittest.main()

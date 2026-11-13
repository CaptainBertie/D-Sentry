# SPDX-License-Identifier: MIT
"""Shared paths and helpers for the relay-pc tests (stdlib unittest).

Run all of them with the project venv:
    "<repo>/.venv/bin/python" -m unittest discover -s D-SentryCode/relay-pc/tests -v
Compiled test programs go to D-SentryCode/tests/build/relay-pc (gitignored), or
$DSENTRY_TEST_BUILD when set.
"""

import importlib.util
import os
import random
import shutil
import sys
from pathlib import Path

TESTS = Path(__file__).resolve().parent
RELAY = TESTS.parent                 # D-SentryCode/relay-pc
DSC = RELAY.parent                   # D-SentryCode
REPO = DSC.parent
TOOLS = DSC / "tools"
SPEC_PATH = DSC / "protocol" / "protocol_spec.py"
HEADER = DSC / "protocol" / "dsentry_protocol.h"
ESP32_HEADER = REPO / "firmware" / "UNOR4USBBridge" / "dsentry_protocol.h"
DOC = DSC / "docs" / "protocol.md"
CTEST = DSC / "tests" / "unit" / "test_protocol_c.cpp"
CONFIG_H = DSC / "config.h"
PYTHON = sys.executable

if str(RELAY) not in sys.path:
    sys.path.insert(0, str(RELAY))
if str(TESTS) not in sys.path:
    sys.path.insert(0, str(TESTS))


def build_dir():
    d = Path(os.environ.get("DSENTRY_TEST_BUILD", DSC / "tests" / "build" / "relay-pc"))
    d.mkdir(parents=True, exist_ok=True)
    return d


def load_module(name, path):
    spec = importlib.util.spec_from_file_location(name, str(path))
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def load_spec():
    return load_module("protocol_spec_under_test", SPEC_PATH)


def find_tool(name, *candidates):
    """A compiler on PATH or at one of the candidate paths, else None."""
    found = shutil.which(name)
    if found:
        return found
    for c in candidates:
        if Path(c).is_file():
            return str(c)
    return None


ARM_GXX = find_tool("arm-none-eabi-g++", *sorted(
    Path.home().glob("Library/Arduino15/packages/arduino/tools/arm-none-eabi-gcc/*/bin/arm-none-eabi-g++")))
XTENSA_GXX = find_tool("xtensa-esp32s3-elf-g++", REPO / "firmware" / "hardware" / "esp32-patched" /
                       "esp32" / "tools" / "xtensa-esp32s3-elf" / "bin" / "xtensa-esp32s3-elf-g++")
CLANGXX = find_tool("clang++")
CLANG = find_tool("clang")

SCALAR_RANGES = {
    "u8": (0, 0xFF), "i8": (-0x80, 0x7F), "u16": (0, 0xFFFF), "i16": (-0x8000, 0x7FFF),
    "u32": (0, 0xFFFFFFFF), "i32": (-0x80000000, 0x7FFFFFFF),
}


def field_specs(spec, packet_name):
    """[(name, kind, lo, hi, count)] from the spec, computed here rather than by the generator."""
    p = next(p for p in spec.PACKETS if p["name"] == packet_name)
    out = []
    for f in p["fields"]:
        t = f["type"]
        if t in SCALAR_RANGES:
            lo, hi = SCALAR_RANGES[t]
            out.append((f["name"], "int", lo, hi, None))
        else:
            base, count = t.rstrip("]").split("[")
            out.append((f["name"], "text" if base == "char" else "u8s", 0, 0xFF, int(count)))
    return out


def random_values(spec, packet_name, rng, extreme=None):
    """Field values for one packet: random, or all minimum (extreme='lo') / maximum ('hi')."""
    values = {}
    for name, kind, lo, hi, count in field_specs(spec, packet_name):
        if kind == "int":
            values[name] = lo if extreme == "lo" else hi if extreme == "hi" else rng.randint(lo, hi)
        elif kind == "u8s":
            values[name] = tuple((lo if extreme == "lo" else hi if extreme == "hi" else rng.randint(0, 255))
                                 for _ in range(count))
        else:
            # latin-1 text, including bytes a deframer must not care about (0x5A, 0xA5, 0xD5...)
            pool = [chr(b) for b in range(256)]
            values[name] = "".join(rng.choice(pool) for _ in range(count))
    return values


def mixed_stream(protocol, spec, rng, pieces=200):
    """A byte stream like the USB port's: debug text (with 'Z'), valid frames of every type,
    frames with a bad CRC or a wrong len byte, truncated frames, random bytes."""
    classes = protocol.PACKET_CLASSES
    out = bytearray()
    texts = [b"[2] STATE PAIRING -> DISARMED\r\n", b"Zz ZAP Z\x07\r\n", b"stats loop=9876 Hz\r\n",
             b"Z", b"\xa5\x0a", b"\xd5\x5b", b"\x5a\x07", b"\xd6\x08"]
    for _ in range(pieces):
        r = rng.random()
        cls = rng.choice(classes)
        frame = bytearray(cls(**random_values(spec, cls.__name__, rng)).encode())
        if r < 0.45:
            out += frame
        elif r < 0.55:
            frame[rng.randrange(2, len(frame))] ^= 1 << rng.randrange(8)      # bad CRC
            out += frame
        elif r < 0.62:
            del frame[rng.randrange(1, len(frame))]                         # truncated
            out += frame
        elif r < 0.68:
            frame[1] ^= 0x01                                                # wrong len byte
            out += frame
        elif r < 0.9:
            out += rng.choice(texts)
        else:
            out += bytes(rng.randrange(256) for _ in range(rng.randrange(1, 20)))
    return bytes(out)


def seeded(seed=20261103):
    return random.Random(seed)

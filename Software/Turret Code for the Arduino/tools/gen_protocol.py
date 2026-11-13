#!/usr/bin/env python3
# gen_protocol.py -- turns D-SentryCode/protocol/protocol_spec.py (the single source of truth for
# the D-Sentry wire format) into every file that speaks it, so the C, Python and docs copies
# cannot drift:
#   D-SentryCode/protocol/dsentry_protocol.h        C/C++ header: constants, packed structs with
#                                                   size/offset static_asserts, CRC-8/SMBUS,
#                                                   encoders, decoders, one streaming deframer
#   firmware/UNOR4USBBridge/dsentry_protocol.h      byte-identical copy for the ESP32-S3 fork
#   D-SentryCode/relay-pc/dsentry/protocol.py       the same for the PC side (stdlib only)
#   D-SentryCode/docs/protocol.md                   human docs with offsets and example frames
#   D-SentryCode/tests/unit/test_protocol_c.cpp     C++ check of the header against the examples
#
# The example frames in the docs and the C++ test are computed here by a small reference codec
# that is independent of the generated code, so the tests compare three implementations.
#
# Usage (stdlib only, any Python 3.8+):
#   "<repo>/.venv/bin/python" D-SentryCode/tools/gen_protocol.py           write what changed
#   "<repo>/.venv/bin/python" D-SentryCode/tools/gen_protocol.py --check   exit 1 if any is stale
# Exit status: 0 ok, 1 stale output (--check) or an invalid spec, 2 usage error.
#
# SPDX-License-Identifier: MIT

import argparse
import importlib.util
import re
import struct
import sys
from pathlib import Path

HERE = Path(__file__).resolve().parent
DSC = HERE.parent                      # D-SentryCode/
REPO = DSC.parent
SPEC_PATH = DSC / "protocol" / "protocol_spec.py"
OUT_HEADER = DSC / "protocol" / "dsentry_protocol.h"
OUT_ESP32_HEADER = REPO / "firmware" / "UNOR4USBBridge" / "dsentry_protocol.h"
OUT_PY = DSC / "relay-pc" / "dsentry" / "protocol.py"
OUT_DOC = DSC / "docs" / "protocol.md"
OUT_CTEST = DSC / "tests" / "unit" / "test_protocol_c.cpp"

GEN_BY = "D-SentryCode/tools/gen_protocol.py"
GEN_FROM = "D-SentryCode/protocol/protocol_spec.py"

FRAME_HEADER_LEN = 2      # start + len
FRAME_OVERHEAD = 3        # start + len + crc
DOC_HEX_BYTES_PER_LINE = 16

# name -> (size, struct code, C type, min, max)
SCALARS = {
    "u8": (1, "B", "uint8_t", 0, 0xFF),
    "i8": (1, "b", "int8_t", -0x80, 0x7F),
    "u16": (2, "H", "uint16_t", 0, 0xFFFF),
    "i16": (2, "h", "int16_t", -0x8000, 0x7FFF),
    "u32": (4, "I", "uint32_t", 0, 0xFFFFFFFF),
    "i32": (4, "i", "int32_t", -0x80000000, 0x7FFFFFFF),
}
ARRAY_RE = re.compile(r"^(u8|char)\[(\d+)\]$")
IDENT_RE = re.compile(r"^[A-Za-z_][A-Za-z0-9_]*$")


class SpecError(Exception):
    pass


def upper_snake(name):
    """camelCase / UPPER / lower -> UPPER_SNAKE (dpadUp -> DPAD_UP, lb -> LB, espMode -> ESP_MODE)."""
    s = re.sub(r"([a-z0-9])([A-Z])", r"\1_\2", name)
    return s.upper()


def lower_snake(name):
    return upper_snake(name).lower()


# ---------------------------------------------------------------------------------------------
# Model: the spec, validated and with offsets worked out
# ---------------------------------------------------------------------------------------------
class Field:
    def __init__(self, raw, packet_name, consts):
        self.name = raw["name"]
        self.type = raw["type"]
        self.doc = raw["doc"]
        self.ref = raw.get("ref")
        self.default = raw.get("default")
        self.since = raw.get("since", 1)       # the packet version that appended this field
        where = "%s.%s" % (packet_name, self.name)
        if not IDENT_RE.match(self.name):
            raise SpecError("%s: bad field name" % where)
        if not isinstance(self.since, int) or isinstance(self.since, bool) or self.since < 1:
            raise SpecError("%s: since must be a version number >= 1" % where)
        m = ARRAY_RE.match(self.type)
        if self.type in SCALARS:
            self.size, self.code, self.ctype, self.lo, self.hi = SCALARS[self.type]
            self.count = None
            self.kind = "int"
        elif m:
            self.count = int(m.group(2))
            if self.count < 1:
                raise SpecError("%s: array length must be >= 1" % where)
            self.size = self.count
            self.lo, self.hi = 0, 0xFF
            if m.group(1) == "u8":
                self.kind, self.ctype, self.code = "u8s", "uint8_t", "%dB" % self.count
            else:
                self.kind, self.ctype, self.code = "text", "char", "%ds" % self.count
        else:
            raise SpecError("%s: unknown type %r" % (where, self.type))
        if self.kind == "int" and isinstance(self.default, str):
            if self.default not in consts:
                raise SpecError("%s: default %r is not a CONSTANTS name or enum macro"
                                % (where, self.default))
        if self.kind == "text" and self.default is not None:
            if not (isinstance(self.default, str) and len(self.default) == 1):
                raise SpecError("%s: a char array default is one fill character" % where)
        self.offset = None  # payload offset, set by Packet

    def default_value(self, consts):
        """The Python default of this field."""
        if self.kind == "int":
            if self.default is None:
                return 0
            return consts[self.default] if isinstance(self.default, str) else int(self.default)
        if self.kind == "u8s":
            return tuple([0] * self.count)
        return (self.default or " ") * self.count

    def check_value(self, value, where):
        if self.kind == "int":
            if not isinstance(value, int) or not self.lo <= value <= self.hi:
                raise SpecError("%s: %r does not fit %s" % (where, value, self.type))
        elif self.kind == "u8s":
            if len(value) != self.count or any(not 0 <= v <= 0xFF for v in value):
                raise SpecError("%s: needs %d bytes 0..255" % (where, self.count))
        else:
            if len(value) != self.count:
                raise SpecError("%s: needs exactly %d characters" % (where, self.count))
            value.encode("latin-1")

    def pack(self, value):
        """Reference encoder for one field (independent of the generated code)."""
        if self.kind == "int":
            return struct.pack("<" + self.code, value)
        if self.kind == "u8s":
            return bytes(value)
        return value.encode("latin-1")


class Packet:
    def __init__(self, raw, consts, refs):
        self.name = raw["name"]
        self.id = raw["id"]
        self.start = raw["start"]
        self.doc = raw["doc"]
        self.carried = raw.get("carried", [])
        self.fields = [Field(f, self.name, consts) for f in raw["fields"]]
        self.example = raw["example"]
        self.UP = upper_snake(self.id)                    # CONTROL
        self.macro = "DS_%s_" % self.UP                   # DS_CONTROL_
        self.ptype = "DS_PKT_%s" % self.UP                # DS_PKT_CONTROL
        self.ctype = "ds_%s_packet_t" % self.id           # ds_control_packet_t
        self.cstruct = "ds_%s_packet" % self.id
        names = set()
        offset = 0
        for f in self.fields:
            if f.name in names:
                raise SpecError("%s: duplicate field %s" % (self.name, f.name))
            names.add(f.name)
            if f.ref is not None and f.ref not in refs:
                raise SpecError("%s.%s: ref %r is not an ENUMS/BITS name" % (self.name, f.name, f.ref))
            f.offset = offset
            offset += f.size
        self.payload_len = offset
        self.frame_len = offset + FRAME_OVERHEAD
        if not 1 <= self.payload_len <= 0xFF:
            raise SpecError("%s: payload of %d bytes does not fit the len byte" % (self.name, offset))
        if not 0 <= self.start <= 0xFF:
            raise SpecError("%s: start byte out of range" % self.name)
        if set(self.example) != names:
            raise SpecError("%s: example must give exactly the fields %s" % (self.name, sorted(names)))
        for f in self.fields:
            f.check_value(self.example[f.name], "%s example %s" % (self.name, f.name))
        # Versioned layouts: fields with "since" > 1 were appended by a later packet version.
        # Every older version is an exact (shorter) length the PC deframer may also accept.
        self.version_const = raw.get("version")
        self.version = None
        self.legacy = []          # (version, payload_len, fields of that version), oldest first
        if self.version_const is not None:
            if self.version_const not in consts:
                raise SpecError("%s: version %r is not a CONSTANTS name" % (self.name, self.version_const))
            self.version = consts[self.version_const]
        newest = max(f.since for f in self.fields)
        if newest > 1:
            if self.version is None:
                raise SpecError("%s: fields with since > 1 need the packet's version constant" % self.name)
            if newest > self.version:
                raise SpecError("%s: a field is newer than version %d" % (self.name, self.version))
            for a, b in zip(self.fields, self.fields[1:]):
                if b.since < a.since:
                    raise SpecError("%s.%s: since must not fall: newer fields go at the end"
                                    % (self.name, b.name))
            for f in self.fields:
                if f.since > 1 and f.kind != "int":
                    raise SpecError("%s.%s: only integer fields can be appended (since > 1)"
                                    % (self.name, f.name))
            seen = {self.payload_len}
            for v in range(1, self.version):
                fields_v = [f for f in self.fields if f.since <= v]
                plen = sum(f.size for f in fields_v)
                if plen in seen:
                    continue
                seen.add(plen)
                self.legacy.append((v, plen, fields_v))
        # Text on the USB port must never look like a frame: a printable start byte needs a
        # len byte that text never contains (architecture.md 4.1).
        for plen in [self.payload_len] + [plen for _v, plen, _f in self.legacy]:
            if 0x20 <= self.start < 0x7F and 0x20 <= plen < 0x7F:
                raise SpecError("%s: start %r and len %r are both printable ASCII; debug text could "
                                "fake this frame" % (self.name, chr(self.start), chr(plen)))

    def legacy_macro(self, version):
        """DS_TELEMETRY_V1_ (prefix of an older layout's LEN / FRAME_LEN constants)."""
        return "%sV%d_" % (self.macro, version)


class Model:
    def __init__(self, spec):
        self.spec = spec
        self.consts = {}
        for name, value, _doc in spec.CONSTANTS:
            if not name.startswith("DS_") or name in self.consts:
                raise SpecError("constant %s: needs a DS_ prefix and a unique name" % name)
            self.consts[name] = value
        self.enums = spec.ENUMS
        self.bits = spec.BITS
        refs = set()
        named = dict(self.consts)          # what a field default may name: constants, enum macros
        for e in self.enums:
            seen = set()
            for vname, value, _doc in e["values"]:
                if value in seen or not 0 <= value <= 0xFF:
                    raise SpecError("%s.%s: duplicate or out-of-range value" % (e["name"], vname))
                seen.add(value)
                macro = e["prefix"] + vname
                if macro in named:
                    raise SpecError("%s: enum macro clashes with a constant" % macro)
                named[macro] = value
            refs.add(e["name"])
        self.named = named
        for b in self.bits:
            seen = set()
            for key, bit, _doc in b["bits"]:
                if bit in seen or not 0 <= bit < b["width"]:
                    raise SpecError("%s.%s: duplicate or out-of-range bit" % (b["name"], key))
                seen.add(bit)
            refs.add(b["name"])
        self.packets = [Packet(p, self.named, refs) for p in spec.PACKETS]
        starts = [p.start for p in self.packets]
        if len(set(starts)) != len(starts):
            raise SpecError("two packets share a start byte")
        for p in self.packets:
            for f in p.fields:
                if f.ref in self.enum_names():
                    width = f.hi
                    for vname, value, _doc in self.enum(f.ref)["values"]:
                        if value > width:
                            raise SpecError("%s.%s: %s.%s does not fit" % (p.name, f.name, f.ref, vname))
        self.max_frame = max(p.frame_len for p in self.packets)
        self.max_payload = max(p.payload_len for p in self.packets)
        self.at = spec.AT_COMMANDS
        self.at_existing = spec.AT_COMMANDS_EXISTING
        self.crc = spec.CRC
        self.framing = spec.FRAMING
        if self.crc["width"] != 8 or self.crc["refin"] or self.crc["refout"]:
            raise SpecError("the generator implements only non-reflected CRC-8 variants")
        if ref_crc8(b"123456789", self.crc) != self.crc["check"]:
            raise SpecError("CRC parameters do not give the check value %#04x" % self.crc["check"])

    def enum_names(self):
        return {e["name"] for e in self.enums}

    def enum(self, name):
        return next(e for e in self.enums if e["name"] == name)

    def bitgroup(self, name):
        return next(b for b in self.bits if b["name"] == name)

    def packet(self, name):
        return next(p for p in self.packets if p.name == name)


def load_spec(path=SPEC_PATH):
    loader_spec = importlib.util.spec_from_file_location("protocol_spec", str(path))
    module = importlib.util.module_from_spec(loader_spec)
    loader_spec.loader.exec_module(module)
    return module


# ---------------------------------------------------------------------------------------------
# Reference codec (used for the docs' example frames and the C++ test's expectations)
# ---------------------------------------------------------------------------------------------
def ref_crc8(data, crc_def):
    crc = crc_def["init"]
    for byte in data:
        crc ^= byte
        for _ in range(8):
            crc = ((crc << 1) ^ crc_def["poly"]) & 0xFF if crc & 0x80 else (crc << 1) & 0xFF
    return crc ^ crc_def["xorout"]


def ref_encode(model, packet, values):
    payload = b"".join(f.pack(values[f.name]) for f in packet.fields)
    assert len(payload) == packet.payload_len
    body = bytes([packet.payload_len]) + payload
    return bytes([packet.start]) + body + bytes([ref_crc8(body, model.crc)])


def ref_deframe(model, data):
    """The acceptance rules of architecture.md 4.1, written as plainly as possible.
    Returns (frames as (packet, bytes), crc_errors, len_errors, junk_bytes, held)."""
    by_start = {p.start: p for p in model.packets}
    buf = bytearray(data)
    frames, crc_errors, len_errors, junk = [], 0, 0, 0
    while buf:
        p = by_start.get(buf[0])
        if p is None:
            del buf[0]
            junk += 1
            continue
        if len(buf) < FRAME_HEADER_LEN:
            break
        if buf[1] != p.payload_len:
            del buf[0]
            junk += 1
            len_errors += 1
            continue
        if len(buf) < p.frame_len:
            break
        if ref_crc8(buf[1:p.frame_len - 1], model.crc) != buf[p.frame_len - 1]:
            del buf[0]
            junk += 1
            crc_errors += 1
            continue
        frames.append((p, bytes(buf[:p.frame_len])))
        del buf[:p.frame_len]
    return frames, crc_errors, len_errors, junk, len(buf)


def example_frame(model, packet):
    return ref_encode(model, packet, packet.example)


def hex_spaced(data):
    return " ".join("%02x" % b for b in data)


def c_string(text):
    out = []
    for ch in text:
        o = ord(ch)
        if ch in "\\\"":
            out.append("\\" + ch)
        elif 0x20 <= o < 0x7F:
            out.append(ch)
        else:
            out.append("\\x%02x\"\"" % o)
    return '"' + "".join(out) + '"'


def wrap(text, width, indent):
    """Greedy word wrap; returns lines (without newline) each prefixed by indent."""
    words = text.split()
    lines, cur = [], ""
    for w in words:
        if cur and len(indent) + len(cur) + 1 + len(w) > width:
            lines.append(indent + cur)
            cur = w
        else:
            cur = w if not cur else cur + " " + w
    if cur:
        lines.append(indent + cur)
    return lines


MAX_LINE = 100


def block_comment(text, indent=""):
    """A /* */ comment wrapped to MAX_LINE."""
    if len(indent) + len(text) + 6 <= MAX_LINE:
        return ["%s/* %s */" % (indent, text)]
    lines = wrap(text, MAX_LINE, indent + " * ")
    lines[0] = indent + "/* " + lines[0][len(indent) + 3:]
    lines[-1] += " */"
    return lines


def define_block(rows, indent=""):
    """rows: (name, value_text, comment or None) -> aligned #define lines. A comment that would
    run past MAX_LINE goes on its own lines above its define."""
    width_n = max(len(r[0]) for r in rows)
    width_v = max(len(r[1]) for r in rows)
    out = []
    for name, value, comment in rows:
        line = "%s#define %-*s %-*s" % (indent, width_n, name, width_v, value)
        if comment and len(line) + len(comment) + 8 <= MAX_LINE:
            line += "  /* %s */" % comment
        elif comment:
            out.extend(block_comment(comment, indent))
        out.append(line.rstrip())
    return out


# ---------------------------------------------------------------------------------------------
# C header
# ---------------------------------------------------------------------------------------------
def at_format(cmd):
    if cmd["mode"] == "read":
        return "AT%s?" % cmd["name"]
    if cmd["mode"] == "run":
        return "AT%s" % cmd["name"]
    return "AT%s=%s" % (cmd["name"], ",".join("<%s>" % a[0] for a in cmd["args"]))


def at_reply_format(cmd):
    return "%s: %s" % (cmd["name"], ",".join("<%s>" % r[0] for r in cmd["reply"]))


def gen_header(model):
    L = []
    w = L.append
    w("/*")
    w(" * dsentry_protocol.h -- the D-Sentry wire format: the four framed packets (ControlPacket,")
    w(" * StatusPacket, TelemetryPacket, BeaconPacket), their enums and bit masks, the D-Sentry AT")
    w(" * command names, CRC-8/SMBUS, encoders, decoders and one streaming deframer for all four.")
    w(" *")
    w(" * GENERATED by %s from %s." % (GEN_BY, GEN_FROM))
    w(" * Do not edit: change protocol_spec.py, run tools/gen_protocol.py and commit every output.")
    w(" * Byte-identical copies (gen_protocol.py --check fails while either is stale):")
    w(" *   D-SentryCode/protocol/dsentry_protocol.h     canonical; the RA4M1 sketch symlinks to it")
    w(" *   firmware/UNOR4USBBridge/dsentry_protocol.h   the ESP32-S3 fork's copy")
    w(" * Offsets, enums, AT commands and example frames: D-SentryCode/docs/protocol.md.")
    w(" *")
    w(" * Frame: start | len | payload[len] | crc. len counts the payload only; crc is %s" % model.crc["name"])
    w(" * (poly 0x%02X, init 0x%02X, no reflection, no xor-out) over len and the payload."
      % (model.crc["poly"], model.crc["init"]))
    w(" * A frame is accepted only with one of the four start bytes, that type's exact len and a")
    w(" * matching CRC, so ASCII debug text on the same USB port can never be taken for one.")
    w(" *")
    w(" * Byte order: little-endian. Each payload struct is packed and laid out exactly as on the")
    w(" * wire (static_asserts below pin every size and offset), so the encoders and decoders memcpy")
    w(" * it. That is correct only on a little-endian CPU: the RA4M1 (Cortex-M4), the ESP32-S3")
    w(" * (Xtensa LX7) and the Mac all are, and a static_assert refuses to compile anywhere else.")
    w(" * Access packed fields by value (p.rx = v); do not take their address.")
    w(" * Plain C99 (gnu99/gnu11) and C++11..17; no dynamic allocation, no globals, no Arduino headers.")
    w(" *")
    w(" * SPDX-License-Identifier: MIT")
    w(" * Copyright (c) 2026 the D-Sentry project")
    w(" */")
    w("#pragma once")
    w("#include <stdint.h>")
    w("#include <stddef.h>")
    w("#include <string.h>")
    w("#include <stdbool.h>")
    w("")
    w("/* static_assert for C++11, C11 and plain C99 (where a negative array size stops the build) */")
    w("#if defined(__cplusplus)")
    w("#define DS_STATIC_ASSERT(cond, msg) static_assert(cond, msg)")
    w("#elif defined(__STDC_VERSION__) && __STDC_VERSION__ >= 201112L")
    w("#define DS_STATIC_ASSERT(cond, msg) _Static_assert(cond, msg)")
    w("#else")
    w("#define DS_STATIC_ASSERT_JOIN2_(a, b) a##b")
    w("#define DS_STATIC_ASSERT_JOIN_(a, b) DS_STATIC_ASSERT_JOIN2_(a, b)")
    w("#define DS_STATIC_ASSERT(cond, msg) \\")
    w("  typedef char DS_STATIC_ASSERT_JOIN_(ds_static_assert_line_, __LINE__)[(cond) ? 1 : -1]")
    w("#endif")
    w("")
    w("#if !defined(__BYTE_ORDER__) || !defined(__ORDER_LITTLE_ENDIAN__)")
    w("#error \"dsentry_protocol.h: cannot tell the byte order; the codecs memcpy little-endian structs\"")
    w("#endif")
    w("DS_STATIC_ASSERT(__BYTE_ORDER__ == __ORDER_LITTLE_ENDIAN__,")
    w("                 \"dsentry_protocol.h memcpys packed structs: little-endian CPUs only\");")
    w("")
    w("/* ---- versions and scales ---- */")
    L.extend(define_block([(n, str(v), d) for n, v, d in model.spec.CONSTANTS]))
    w("")
    w("/* ---- framing ---- */")
    rows = [
        ("DS_FRAME_HEADER_LEN", str(FRAME_HEADER_LEN), "start + len"),
        ("DS_FRAME_OVERHEAD", str(FRAME_OVERHEAD), "start + len + crc"),
        ("DS_MAX_PAYLOAD_LEN", str(model.max_payload), "largest len of any packet"),
        ("DS_MAX_FRAME_LEN", str(model.max_frame), "largest frame of any packet"),
        ("DS_CRC8_POLY", "0x%02Xu" % model.crc["poly"], model.crc["name"]),
        ("DS_CRC8_INIT", "0x%02Xu" % model.crc["init"], None),
        ("DS_CRC8_CHECK", "0x%02Xu" % model.crc["check"], "CRC of the ASCII bytes \"123456789\""),
    ]
    L.extend(define_block(rows))
    w("")
    w("/* Packet type ids: what ds_deframer_feed() reports. 0 = no frame completed. */")
    rows = [("DS_PKT_NONE", "0", None)]
    for i, p in enumerate(model.packets, 1):
        rows.append((p.ptype, str(i), p.name))
    rows.append(("DS_PKT_COUNT", str(len(model.packets) + 1), "including DS_PKT_NONE"))
    L.extend(define_block(rows))
    w("")
    for p in model.packets:
        rows = [
            (p.macro + "START", "0x%02X" % p.start, "%s start byte" % p.name),
            (p.macro + "LEN", str(p.payload_len), "payload bytes = the len byte"),
            (p.macro + "FRAME_LEN", str(p.frame_len), "start + len + payload + crc"),
        ]
        for f in p.fields:
            if f.count is not None:
                rows.append((p.macro + upper_snake(f.name) + "_LEN", str(f.count),
                             "%s.%s elements" % (p.name, f.name)))
        L.extend(define_block(rows))
        w("")
    w("/* ---- enums (one byte on the wire) ---- */")
    for e in model.enums:
        L.extend(block_comment("%s: %s" % (e["name"], e["doc"])))
        rows = [(e["prefix"] + vn, str(v), d) for vn, v, d in e["values"]]
        vals = [v for _n, v, _d in e["values"]]
        if vals == list(range(len(vals))):
            rows.append((e["prefix"] + "COUNT", str(len(vals)), "values 0..COUNT-1"))
        L.extend(define_block(rows))
        w("")
    w("/* ---- bit masks ---- */")
    for b in model.bits:
        L.extend(block_comment("%s (%d-bit): %s" % (b["name"], b["width"], b["doc"])))
        digits = b["width"] // 4
        rows = [(b["prefix"] + upper_snake(k), "0x%0*Xu" % (digits, 1 << bit), "bit %d: %s" % (bit, d))
                for k, bit, d in b["bits"]]
        mask = 0
        for _k, bit, _d in b["bits"]:
            mask |= 1 << bit
        rows.append((b["prefix"] + "ALL", "0x%0*Xu" % (digits, mask), "every defined bit"))
        L.extend(define_block(rows))
        w("")
    w("/* ---- D-Sentry AT commands (Serial2, chAT server on the ESP32). Every command answers with")
    w(" * its prompt and OK; failures go in the payload. Arguments and reply fields are decimal")
    w(" * unless marked hex. The RA4M1 sends \"AT\" NAME (\"?\" | \"=\" args) \"\\r\\n\". ---- */")
    for cmd in model.at:
        cid = cmd["id"]
        L.extend(block_comment("%s  ->  %s" % (at_format(cmd), at_reply_format(cmd))))
        rows = [("DS_AT_%s" % cid, '"%s"' % cmd["name"], None)]
        rows.append(("DS_AT_%s_ARGC" % cid, str(len(cmd["args"])), "arguments"))
        for i, (an, at, ad) in enumerate(cmd["args"]):
            rows.append(("DS_AT_%s_ARG_%s" % (cid, upper_snake(an)), str(i), "%s: %s" % (at, ad)))
        rows.append(("DS_AT_%s_REPLY_COUNT" % cid, str(len(cmd["reply"])), "reply fields"))
        for i, (rn, rt, rd) in enumerate(cmd["reply"]):
            rows.append(("DS_AT_%s_REPLY_%s" % (cid, upper_snake(rn)), str(i), "%s: %s" % (rt, rd)))
        L.extend(define_block(rows))
        w("")
    tel = model.packet("TelemetryPacket")
    rows = [
        ("DS_AT_DST_HEX_LEN", "(2 * DS_%s_FRAME_LEN)" % tel.UP,
         "hex digits of one TelemetryPacket in AT+DST"),
        ("DS_AT_DST_LINE_LEN", "(sizeof(\"AT\" DS_AT_DST \"=\") - 1 + DS_AT_DST_HEX_LEN + 2)",
         "\"AT+DST=\" + hex + \"\\r\\n\" (%d bytes)" % (len("AT+DST=") + 2 * tel.frame_len + 2)),
    ]
    L.extend(define_block(rows))
    w("")

    # structs
    w("/* ---- payload structs: exactly the wire layout of the bytes between len and crc ---- */")
    for p in model.packets:
        w("")
        w("/* %s 0x%02X, payload %d bytes, frame %d bytes." % (p.name, p.start, p.payload_len, p.frame_len))
        for line in wrap(p.doc, 98, " * "):
            w(line)
        if p.legacy:
            older = ", ".join("version %d = len %d" % (v, plen) for v, plen, _f in p.legacy)
            for line in wrap("Version %d (%s). Fields marked vN were appended by version N; older "
                             "layouts (%s) end before them. This header encodes, decodes and "
                             "deframes the current layout only; the PC relay also reads the older "
                             "ones." % (p.version, p.version_const, older), 98, " * "):
                w(line)
        w(" */")
        w("typedef struct __attribute__((packed)) %s {" % p.cstruct)
        decls = []
        tw = max(len(f.ctype) for f in p.fields)
        for f in p.fields:
            if f.count is None:
                decl = "%-*s %s;" % (tw, f.ctype, f.name)
            else:
                decl = "%-*s %s[%s];" % (tw, f.ctype, f.name, p.macro + upper_snake(f.name) + "_LEN")
            doc = f.doc if f.since == 1 else "v%d: %s" % (f.since, f.doc)
            decls.append((decl, "%2d: %s" % (f.offset, doc), "%d: %s" % (f.offset, doc)))
        dw = max(len(d[0]) for d in decls)
        for decl, comment, comment_above in decls:
            line = "  %-*s  /* %s */" % (dw, decl, comment)
            if len(line) <= MAX_LINE:
                w(line)
            else:
                L.extend(block_comment(comment_above, "  "))
                w("  %s" % decl)
        w("} %s;" % p.ctype)
        w("DS_STATIC_ASSERT(sizeof(%s) == %sLEN, \"%s payload size\");" % (p.ctype, p.macro, p.name))
        for f in p.fields:
            cond = "offsetof(%s, %s) == %d" % (p.ctype, f.name, f.offset)
            msg = '"%s.%s offset"' % (p.name, f.name)
            line = "DS_STATIC_ASSERT(%s, %s);" % (cond, msg)
            if len(line) <= MAX_LINE:
                w(line)
            else:
                w("DS_STATIC_ASSERT(%s," % cond)
                w("                 %s);" % msg)
    w("")

    # CRC and per-type lookups
    w("/* ---- CRC-8/SMBUS ---- */")
    w("static inline uint8_t ds_crc8_update(uint8_t crc, uint8_t byte) {")
    w("  crc = (uint8_t)(crc ^ byte);")
    w("  for (int bit = 0; bit < 8; bit++) {")
    w("    crc = (uint8_t)((crc & 0x80u) ? ((unsigned)(crc << 1) ^ DS_CRC8_POLY) : (unsigned)(crc << 1));")
    w("  }")
    w("  return crc;")
    w("}")
    w("")
    w("static inline uint8_t ds_crc8(const uint8_t* data, size_t len) {")
    w("  uint8_t crc = DS_CRC8_INIT;")
    w("  for (size_t i = 0; i < len; i++) crc = ds_crc8_update(crc, data[i]);")
    w("  return crc;")
    w("}")
    w("")
    w("/* ---- per-type lookups ---- */")
    w("/* DS_PKT_* for a start byte, or DS_PKT_NONE. */")
    w("static inline uint8_t ds_packet_type_for_start(uint8_t start) {")
    w("  switch (start) {")
    for p in model.packets:
        w("    case %sSTART: return %s;" % (p.macro, p.ptype))
    w("    default: return DS_PKT_NONE;")
    w("  }")
    w("}")
    w("")
    w("/* Start byte of a DS_PKT_* type, or 0 for DS_PKT_NONE / unknown. */")
    w("static inline uint8_t ds_start_for_type(uint8_t type) {")
    w("  switch (type) {")
    for p in model.packets:
        w("    case %s: return %sSTART;" % (p.ptype, p.macro))
    w("    default: return 0;")
    w("  }")
    w("}")
    w("")
    w("/* Exact payload length (the len byte) of a DS_PKT_* type, or 0. */")
    w("static inline uint8_t ds_payload_len_for_type(uint8_t type) {")
    w("  switch (type) {")
    for p in model.packets:
        w("    case %s: return %sLEN;" % (p.ptype, p.macro))
    w("    default: return 0;")
    w("  }")
    w("}")
    w("")
    w("/* Whole frame length of a DS_PKT_* type, or 0. */")
    w("static inline uint8_t ds_frame_len_for_type(uint8_t type) {")
    w("  uint8_t len = ds_payload_len_for_type(type);")
    w("  return len ? (uint8_t)(len + DS_FRAME_OVERHEAD) : (uint8_t)0;")
    w("}")
    w("")
    w("static inline const char* ds_packet_type_name(uint8_t type) {")
    w("  switch (type) {")
    w("    case DS_PKT_NONE: return \"none\";")
    for p in model.packets:
        w("    case %s: return \"%s\";" % (p.ptype, p.name))
    w("    default: return \"?\";")
    w("  }")
    w("}")
    w("")
    w("/* True when frame[0..len) is one complete, valid frame of the given type: exact length,")
    w(" * start byte, len byte and CRC. */")
    w("static inline bool ds_frame_check(const uint8_t* frame, size_t len, uint8_t type) {")
    w("  uint8_t payload = ds_payload_len_for_type(type);")
    w("  if (payload == 0 || frame == NULL) return false;")
    w("  if (len != (size_t)payload + DS_FRAME_OVERHEAD) return false;")
    w("  if (frame[0] != ds_start_for_type(type) || frame[1] != payload) return false;")
    w("  return ds_crc8(frame + 1, (size_t)payload + 1) == frame[len - 1];")
    w("}")
    w("")
    w("/* Writes start, len and crc around a payload already copied to out + DS_FRAME_HEADER_LEN. */")
    w("static inline size_t ds_frame_finish(uint8_t* out, uint8_t type) {")
    w("  uint8_t payload = ds_payload_len_for_type(type);")
    w("  out[0] = ds_start_for_type(type);")
    w("  out[1] = payload;")
    w("  out[DS_FRAME_HEADER_LEN + payload] = ds_crc8(out + 1, (size_t)payload + 1);")
    w("  return (size_t)payload + DS_FRAME_OVERHEAD;")
    w("}")
    w("")
    w("/* ---- encoders, decoders, initialisers ----")
    w(" * ds_<pkt>_init(p)               zeroes p and applies the spec's defaults")
    w(" * ds_<pkt>_encode(p, out)        writes the whole frame to out, returns its length")
    w(" * ds_<pkt>_decode(frame, len, p) true and fills p only for one exact, valid frame */")
    consts = model.consts
    for p in model.packets:
        w("")
        w("static inline void ds_%s_init(%s* p) {" % (p.id, p.ctype))
        w("  memset(p, 0, sizeof *p);")
        for f in p.fields:
            if f.default is None:
                continue
            if f.kind == "int":
                w("  p->%s = %s;" % (f.name, f.default if isinstance(f.default, str) else str(f.default)))
            elif f.kind == "text":
                w("  memset(p->%s, '%s', sizeof p->%s);" % (f.name, f.default, f.name))
        w("}")
        w("")
        w("static inline size_t ds_%s_encode(const %s* p," % (p.id, p.ctype))
        indent = " " * len("static inline size_t ds_%s_encode(" % p.id)
        w("%suint8_t out[%sFRAME_LEN]) {" % (indent, p.macro))
        w("  memcpy(out + DS_FRAME_HEADER_LEN, p, %sLEN);" % p.macro)
        w("  return ds_frame_finish(out, %s);" % p.ptype)
        w("}")
        w("")
        w("static inline bool ds_%s_decode(const uint8_t* frame, size_t len, %s* p) {"
          % (p.id, p.ctype))
        w("  if (!ds_frame_check(frame, len, %s)) return false;" % p.ptype)
        w("  memcpy(p, frame + DS_FRAME_HEADER_LEN, %sLEN);" % p.macro)
        w("  return true;")
        w("}")
    w("")

    # deframer
    w("/* ---- streaming deframer (all four packet types on one byte stream) ----")
    w(" * Feed every received byte; when a frame completes, feed() returns its DS_PKT_* type and")
    w(" * ds_deframer_frame() / ds_deframer_frame_len() give its bytes (start..crc) until the next")
    w(" * feed() or poll(). Acceptance (architecture.md 4.1): one of the four start bytes, that")
    w(" * type's exact len, matching CRC. On any mismatch only the first held byte is dropped (counted")
    w(" * as junk) and the search restarts at the next one, so a frame that began inside a rejected")
    w(" * candidate is still found. In that rare case one fed byte can expose more than one frame:")
    w(" * the later ones are reported by the following calls, so after draining the input call")
    w(" * ds_deframer_poll() until it returns DS_PKT_NONE. Every byte not in an accepted frame is")
    w(" * counted in junk_bytes (debug text included). At most DS_MAX_FRAME_LEN bytes are held.")
    w(" * Cost: a memmove of the held bytes per dropped byte; text bytes cost almost nothing. */")
    w("typedef struct ds_deframer {")
    w("  uint8_t  buf[DS_MAX_FRAME_LEN];")
    w("  uint8_t  held;        /* bytes in buf, including a frame reported by the last call */")
    w("  uint8_t  ready_len;   /* >0: buf[0..ready_len) is the frame the last call reported */")
    w("  uint8_t  ready_type;  /* its DS_PKT_* type */")
    w("  uint32_t frames;      /* frames accepted */")
    w("  uint32_t crc_errors;  /* candidates with a start byte and the right len but a bad CRC */")
    w("  uint32_t len_errors;  /* start bytes followed by the wrong len (text 'Z' lands here) */")
    w("  uint32_t junk_bytes;  /* bytes that were not part of an accepted frame */")
    w("} ds_deframer_t;")
    w("")
    w("static inline void ds_deframer_init(ds_deframer_t* d) {")
    w("  memset(d, 0, sizeof *d);")
    w("}")
    w("")
    w("/* internal: forget the first count held bytes */")
    w("static inline void ds_deframer_drop_(ds_deframer_t* d, uint8_t count) {")
    w("  memmove(d->buf, d->buf + count, (size_t)(d->held - count));")
    w("  d->held = (uint8_t)(d->held - count);")
    w("}")
    w("")
    w("/* internal: resolve held bytes until a frame completes or more bytes are needed */")
    w("static inline uint8_t ds_deframer_settle_(ds_deframer_t* d) {")
    w("  while (d->held > 0) {")
    w("    uint8_t type = ds_packet_type_for_start(d->buf[0]);")
    w("    if (type == DS_PKT_NONE) {")
    w("      d->junk_bytes++;")
    w("      ds_deframer_drop_(d, 1);")
    w("      continue;")
    w("    }")
    w("    if (d->held < DS_FRAME_HEADER_LEN) return DS_PKT_NONE;")
    w("    if (d->buf[1] != ds_payload_len_for_type(type)) {")
    w("      d->len_errors++;")
    w("      d->junk_bytes++;")
    w("      ds_deframer_drop_(d, 1);")
    w("      continue;")
    w("    }")
    w("    uint8_t n = ds_frame_len_for_type(type);")
    w("    if (d->held < n) return DS_PKT_NONE;")
    w("    if (ds_crc8(d->buf + 1, (size_t)n - 2) != d->buf[n - 1]) {")
    w("      d->crc_errors++;")
    w("      d->junk_bytes++;")
    w("      ds_deframer_drop_(d, 1);")
    w("      continue;")
    w("    }")
    w("    d->frames++;")
    w("    d->ready_len = n;")
    w("    d->ready_type = type;")
    w("    return type;")
    w("  }")
    w("  return DS_PKT_NONE;")
    w("}")
    w("")
    w("/* internal: release the frame the previous call reported */")
    w("static inline void ds_deframer_release_(ds_deframer_t* d) {")
    w("  if (d->ready_len) {")
    w("    ds_deframer_drop_(d, d->ready_len);")
    w("    d->ready_len = 0;")
    w("    d->ready_type = DS_PKT_NONE;")
    w("  }")
    w("}")
    w("")
    w("/* Feed one received byte. Returns the DS_PKT_* type of a frame that completed, or")
    w(" * DS_PKT_NONE. */")
    w("static inline uint8_t ds_deframer_feed(ds_deframer_t* d, uint8_t byte) {")
    w("  ds_deframer_release_(d);")
    w("  /* Never true: after settle_() the held bytes are a frame or a strict prefix of one. */")
    w("  if (d->held >= DS_MAX_FRAME_LEN) {")
    w("    d->junk_bytes++;")
    w("    ds_deframer_drop_(d, 1);")
    w("  }")
    w("  d->buf[d->held++] = byte;")
    w("  return ds_deframer_settle_(d);")
    w("}")
    w("")
    w("/* Report a further frame already held (see above), without new input. */")
    w("static inline uint8_t ds_deframer_poll(ds_deframer_t* d) {")
    w("  ds_deframer_release_(d);")
    w("  return ds_deframer_settle_(d);")
    w("}")
    w("")
    w("/* The frame the last feed()/poll() reported (start..crc), or NULL. */")
    w("static inline const uint8_t* ds_deframer_frame(const ds_deframer_t* d) {")
    w("  return d->ready_len ? d->buf : NULL;")
    w("}")
    w("")
    w("static inline uint8_t ds_deframer_frame_len(const ds_deframer_t* d) {")
    w("  return d->ready_len;")
    w("}")
    w("")
    w("/* ---- hex for AT+DST ---- */")
    w("/* Writes 2*len lowercase hex digits and a NUL to out (2*len + 1 chars). Returns 2*len. */")
    w("static inline size_t ds_hex_encode(const uint8_t* data, size_t len, char* out) {")
    w("  static const char digits[] = \"0123456789abcdef\";")
    w("  for (size_t i = 0; i < len; i++) {")
    w("    out[2 * i] = digits[data[i] >> 4];")
    w("    out[2 * i + 1] = digits[data[i] & 0x0Fu];")
    w("  }")
    w("  out[2 * len] = '\\0';")
    w("  return 2 * len;")
    w("}")
    w("")
    w("/* Value of one hex digit (either case), or -1. */")
    w("static inline int ds_hex_digit(char c) {")
    w("  if (c >= '0' && c <= '9') return c - '0';")
    w("  if (c >= 'a' && c <= 'f') return c - 'a' + 10;")
    w("  if (c >= 'A' && c <= 'F') return c - 'A' + 10;")
    w("  return -1;")
    w("}")
    w("")
    w("/* Parses hex_len hex digits (either case) into out. Returns the byte count, or 0 when")
    w(" * hex_len is odd or zero, a character is not a hex digit, or the bytes do not fit in cap. */")
    w("static inline size_t ds_hex_decode(const char* hex, size_t hex_len, uint8_t* out, size_t cap) {")
    w("  if (hex_len == 0 || (hex_len & 1u) || hex_len / 2 > cap) return 0;")
    w("  for (size_t i = 0; i < hex_len / 2; i++) {")
    w("    int hi = ds_hex_digit(hex[2 * i]);")
    w("    int lo = ds_hex_digit(hex[2 * i + 1]);")
    w("    if (hi < 0 || lo < 0) return 0;")
    w("    out[i] = (uint8_t)((hi << 4) | lo);")
    w("  }")
    w("  return hex_len / 2;")
    w("}")
    w("")
    w("/* ---- enum names for logs (\"?\" for an unknown value) ---- */")
    for e in model.enums:
        w("static inline const char* %s(uint8_t v) {" % e["c_name_func"])
        w("  switch (v) {")
        for vn, _v, _d in e["values"]:
            w("    case %s%s: return \"%s\";" % (e["prefix"], vn, vn))
        w("    default: return \"?\";")
        w("  }")
        w("}")
        w("")
    return "\n".join(L).rstrip("\n") + "\n"


# ---------------------------------------------------------------------------------------------
# Python module
# ---------------------------------------------------------------------------------------------
def gen_python(model):
    L = []
    w = L.append
    w("# GENERATED by %s from %s -- do not edit." % (GEN_BY, GEN_FROM))
    w("# Change protocol_spec.py, run tools/gen_protocol.py and commit every output;")
    w("# `gen_protocol.py --check` fails while this file is stale.")
    w("#")
    w("# SPDX-License-Identifier: MIT")
    w('"""D-Sentry wire format for the PC side: constants, enums, one dataclass per packet with')
    w("encode()/decode(), CRC-8/SMBUS and a streaming Deframer with the same acceptance rules as")
    w("dsentry_protocol.h (architecture.md 4.1). Standard library only. Field names are the")
    w("contract's camelCase names, identical in C, Python and docs/protocol.md.")
    w("")
    w("Frame: start | len | payload[len] | crc, little-endian; len counts the payload; the CRC covers")
    w("len and the payload. decode() raises ValueError unless the bytes are exactly one valid frame.")
    w('"""')
    w("")
    w("import dataclasses")
    w("import enum")
    w("import re")
    w("import struct")
    w("from typing import ClassVar, Dict, List, NamedTuple, Tuple, Union")
    w("")
    w("# ---- versions and scales ----")
    for n, v, d in model.spec.CONSTANTS:
        w("%s = %d  # %s" % (n, v, d))
    w("")
    w("# ---- framing ----")
    w("DS_FRAME_HEADER_LEN = %d  # start + len" % FRAME_HEADER_LEN)
    w("DS_FRAME_OVERHEAD = %d  # start + len + crc" % FRAME_OVERHEAD)
    w("DS_MAX_PAYLOAD_LEN = %d" % model.max_payload)
    w("DS_MAX_FRAME_LEN = %d" % model.max_frame)
    w("DS_CRC8_POLY = 0x%02X  # %s" % (model.crc["poly"], model.crc["name"]))
    w("DS_CRC8_INIT = 0x%02X" % model.crc["init"])
    w("DS_CRC8_CHECK = 0x%02X  # crc8(b\"123456789\")" % model.crc["check"])
    w("")
    w("DS_PKT_NONE = 0")
    for i, p in enumerate(model.packets, 1):
        w("%s = %d" % (p.ptype, i))
    for p in model.packets:
        w("%sSTART = 0x%02X" % (p.macro, p.start))
        w("%sLEN = %d" % (p.macro, p.payload_len))
        w("%sFRAME_LEN = %d" % (p.macro, p.frame_len))
        for v, plen, _fields in p.legacy:
            w("%sLEN = %d  # version %d layout (older firmware), read with accept_legacy=True"
              % (p.legacy_macro(v), plen, v))
            w("%sFRAME_LEN = %d" % (p.legacy_macro(v), plen + FRAME_OVERHEAD))
        for f in p.fields:
            if f.count is not None:
                w("%s%s_LEN = %d" % (p.macro, upper_snake(f.name), f.count))
    w("")
    w("")
    w("class PacketType(enum.IntEnum):")
    w("    NONE = 0")
    for i, p in enumerate(model.packets, 1):
        w("    %s = %d" % (p.UP, i))
    w("")
    w("")
    w("# ---- enums ----")
    for e in model.enums:
        for vn, v, d in e["values"]:
            w("%s%s = %d  # %s" % (e["prefix"], vn, v, d))
        w("")
        w("")
        w("class %s(enum.IntEnum):" % e["name"])
        w('    """%s"""' % e["doc"])
        for vn, v, _d in e["values"]:
            w("    %s = %d" % (vn, v))
        w("")
        w("")
    w("# ---- bit masks ----")
    for b in model.bits:
        for k, bit, d in b["bits"]:
            w("%s%s = 0x%X  # bit %d: %s" % (b["prefix"], upper_snake(k), 1 << bit, bit, d))
        w("")
        w("")
        w("class %s(enum.IntFlag):" % b["name"])
        w('    """%s"""' % b["doc"])
        for k, bit, _d in b["bits"]:
            w("    %s = 0x%X" % (upper_snake(k), 1 << bit))
        w("")
        w("")
    w("# camelCase key -> mask, in bit order, per bit group (for JSON and the dashboard)")
    w("BIT_KEYS: Dict[str, Tuple[Tuple[str, int], ...]] = {")
    for b in model.bits:
        items = ", ".join('("%s", 0x%X)' % (k, 1 << bit) for k, bit, _d in b["bits"])
        w('    "%s": (%s,),' % (b["name"], items))
    w("}")
    w("")
    w("")
    w("def bits_to_dict(group: str, value: int) -> Dict[str, bool]:")
    w('    """{camelCaseKey: bool} for every defined bit of a BIT_KEYS group."""')
    w("    return {key: bool(value & mask) for key, mask in BIT_KEYS[group]}")
    w("")
    w("")
    w("# ---- D-Sentry AT commands (architecture.md 4.2) ----")
    for cmd in model.at:
        cid = cmd["id"]
        w('DS_AT_%s = "%s"  # %s -> %s' % (cid, cmd["name"], at_format(cmd), at_reply_format(cmd)))
        w("DS_AT_%s_ARGC = %d" % (cid, len(cmd["args"])))
        for i, (an, _t, _d) in enumerate(cmd["args"]):
            w("DS_AT_%s_ARG_%s = %d" % (cid, upper_snake(an), i))
        w("DS_AT_%s_REPLY_COUNT = %d" % (cid, len(cmd["reply"])))
        for i, (rn, _t, _d) in enumerate(cmd["reply"]):
            w("DS_AT_%s_REPLY_%s = %d" % (cid, upper_snake(rn), i))
    tel = model.packet("TelemetryPacket")
    w("DS_AT_DST_HEX_LEN = 2 * %sFRAME_LEN" % tel.macro)
    w("DS_AT_DST_LINE_LEN = len(\"AT\" + DS_AT_DST + \"=\") + DS_AT_DST_HEX_LEN + 2  # \"AT+DST=\" + hex + \"\\r\\n\"")
    w("")
    w("AT_COMMANDS = (")
    for cmd in model.at:
        w("    {")
        w('        "id": %r,' % cmd["id"])
        w('        "name": %r,' % cmd["name"])
        w('        "mode": %r,' % cmd["mode"])
        w('        "request": %r,' % at_format(cmd))
        w('        "reply": %r,' % at_reply_format(cmd))
        w('        "args": %r,' % (tuple(tuple(a) for a in cmd["args"]),))
        w('        "reply_fields": %r,' % (tuple(tuple(r) for r in cmd["reply"]),))
        w("    },")
    w(")")
    w("")
    w("")
    w("# ---- CRC ----")
    w("def crc8(data) -> int:")
    w('    """%s over data (poly 0x%02X, init 0x%02X, no reflection, no xor-out)."""'
      % (model.crc["name"], model.crc["poly"], model.crc["init"]))
    w("    crc = DS_CRC8_INIT")
    w("    for byte in data:")
    w("        crc ^= byte")
    w("        for _ in range(8):")
    w("            crc = ((crc << 1) ^ DS_CRC8_POLY) & 0xFF if crc & 0x80 else (crc << 1) & 0xFF")
    w("    return crc")
    w("")
    w("")
    w("def _frame(start: int, payload: bytes) -> bytes:")
    w("    body = bytes((len(payload),)) + payload")
    w("    return bytes((start,)) + body + bytes((crc8(body),))")
    w("")
    w("")
    w("def _check_frame(frame, start: int, length: int, name: str, legacy=()) -> None:")
    w("    if len(frame) >= DS_FRAME_HEADER_LEN and frame[1] in legacy:")
    w("        length = frame[1]                    # an older layout's exact length")
    w("    if len(frame) != length + DS_FRAME_OVERHEAD:")
    w('        raise ValueError("%s: %d bytes, a frame is %d" % (name, len(frame), length + DS_FRAME_OVERHEAD))')
    w("    if frame[0] != start:")
    w('        raise ValueError("%s: start byte 0x%02x, expected 0x%02x" % (name, frame[0], start))')
    w("    if frame[1] != length:")
    w('        raise ValueError("%s: len byte %d, expected %d" % (name, frame[1], length))')
    w("    if crc8(frame[1:-1]) != frame[-1]:")
    w('        raise ValueError("%s: bad CRC" % name)')
    w("")
    w("")
    w("def _text(value: str, size: int, name: str) -> bytes:")
    w('    """A char[size] field: latin-1, space padded (the LCD convention); too long is an error."""')
    w('    raw = value.encode("latin-1")')
    w("    if len(raw) > size:")
    w('        raise ValueError("%s: %d characters, the field holds %d" % (name, len(raw), size))')
    w('    return raw.ljust(size, b" ")')
    w("")
    w("")
    w("def _u8s(value, size: int, name: str) -> Tuple[int, ...]:")
    w("    items = tuple(value)")
    w("    if len(items) != size:")
    w('        raise ValueError("%s: %d bytes, the field holds %d" % (name, len(items), size))')
    w("    return items")
    w("")
    w("")
    w("# ---- packets ----")
    for p in model.packets:
        fmt = "<" + "".join(f.code for f in p.fields)
        assert struct.calcsize(fmt) == p.payload_len
        w("@dataclasses.dataclass")
        w("class %s:" % p.name)
        w('    """%s 0x%02X: %s' % (p.name, p.start, p.doc))
        w("")
        w("    Payload %d bytes, frame %d bytes. Fields in wire order (payload offset):" % (p.payload_len, p.frame_len))
        for f in p.fields:
            doc = f.doc if f.since == 1 else "v%d: %s" % (f.since, f.doc)
            w("     %3d %-19s %-8s %s" % (f.offset, f.name, f.type, doc))
        if p.legacy:
            w("")
            w("    Version %d (%s). Older layouts end before the fields of later versions:" % (p.version, p.version_const))
            for v, plen, _fields in p.legacy:
                w("      version %d: payload %d bytes, frame %d bytes" % (v, plen, plen + FRAME_OVERHEAD))
            w("    decode(accept_legacy=True) and Deframer(accept_legacy=True) read them; the fields")
            w("    they lack keep their defaults, and `version` says which layout it was.")
        w('    """')
        w("")
        w("    START: ClassVar[int] = %sSTART" % p.macro)
        w("    LEN: ClassVar[int] = %sLEN" % p.macro)
        w("    FRAME_LEN: ClassVar[int] = %sFRAME_LEN" % p.macro)
        w("    TYPE: ClassVar[PacketType] = PacketType.%s" % p.UP)
        w("    _STRUCT: ClassVar[struct.Struct] = struct.Struct(%r)" % fmt)
        # payload len of an older layout -> its version (empty: one layout only)
        items = ", ".join("%sLEN: %d" % (p.legacy_macro(v), v) for v, _plen, _f in p.legacy)
        w("    LEGACY: ClassVar[Dict[int, int]] = {%s}" % items)
        if p.legacy:
            w("    _LEGACY_STRUCTS: ClassVar[Dict[int, struct.Struct]] = {")
            for v, _plen, fields_v in p.legacy:
                w("        %sLEN: struct.Struct(%r)," % (p.legacy_macro(v), "<" + "".join(f.code for f in fields_v)))
            w("    }")
            w("    _LEGACY_TAILS: ClassVar[Dict[int, Tuple[int, ...]]] = {   # defaults of the fields they lack")
            for v, _plen, fields_v in p.legacy:
                tail = [f.default if isinstance(f.default, str) else str(f.default_value(model.named))
                        for f in p.fields[len(fields_v):]]
                w("        %sLEN: (%s,)," % (p.legacy_macro(v), ", ".join(tail)))
            w("    }")
        w("")
        for f in p.fields:
            default = f.default_value(model.named)
            if f.kind == "int":
                if isinstance(f.default, str):
                    w("    %s: int = %s" % (f.name, f.default))
                else:
                    w("    %s: int = %d" % (f.name, default))
            elif f.kind == "u8s":
                w("    %s: Tuple[int, ...] = %r" % (f.name, default))
            else:
                w("    %s: str = %r" % (f.name, default))
        w("")
        w("    def payload(self) -> bytes:")
        args = []
        for f in p.fields:
            if f.kind == "int":
                args.append("self.%s" % f.name)
            elif f.kind == "u8s":
                args.append("*_u8s(self.%s, %s%s_LEN, %r)" % (f.name, p.macro, upper_snake(f.name), f.name))
            else:
                args.append("_text(self.%s, %s%s_LEN, %r)" % (f.name, p.macro, upper_snake(f.name), f.name))
        w("        return self._STRUCT.pack(")
        for a in args:
            w("            %s," % a)
        w("        )")
        w("")
        w("    def encode(self) -> bytes:")
        w('        """The whole frame: start, len, payload, crc."""')
        w("        return _frame(self.START, self.payload())")
        w("")
        w("    @classmethod")
        w("    def from_payload(cls, payload) -> \"%s\":" % p.name)
        if p.legacy:
            w("        data = bytes(payload)")
            w("        old = cls._LEGACY_STRUCTS.get(len(data))")
            w("        v = old.unpack(data) + cls._LEGACY_TAILS[len(data)] if old is not None else cls._STRUCT.unpack(data)")
        else:
            w("        v = cls._STRUCT.unpack(bytes(payload))")
        w("        return cls(")
        idx = 0
        for f in p.fields:
            if f.kind == "int":
                w("            %s=v[%d]," % (f.name, idx))
                idx += 1
            elif f.kind == "u8s":
                w("            %s=tuple(v[%d:%d])," % (f.name, idx, idx + f.count))
                idx += f.count
            else:
                w("            %s=v[%d].decode(\"latin-1\")," % (f.name, idx))
                idx += 1
        w("        )")
        w("")
        w("    @classmethod")
        w("    def decode(cls, frame, accept_legacy: bool = False) -> \"%s\":" % p.name)
        w('        """Decode exactly one frame; ValueError on wrong length, start, len or CRC.')
        w('        accept_legacy: an older layout (LEGACY) is accepted too."""')
        w("        _check_frame(frame, cls.START, cls.LEN, %r, cls.LEGACY if accept_legacy else ())" % p.name)
        w("        return cls.from_payload(frame[DS_FRAME_HEADER_LEN:-1])")
        w("")
        if p.version is not None:
            w("    @classmethod")
            w("    def layout_version(cls, payload_len: int):")
            w('        """The packet version whose layout has this payload length, or None."""')
            w("        return %s if payload_len == cls.LEN else cls.LEGACY.get(payload_len)" % p.version_const)
            w("")
        w("    def to_dict(self) -> dict:")
        w("        return dataclasses.asdict(self)")
        w("")
        w("")
    w("PACKET_CLASSES = (%s,)" % ", ".join(p.name for p in model.packets))
    w("_BY_START = {cls.START: cls for cls in PACKET_CLASSES}")
    w("_BY_TYPE = {cls.TYPE: cls for cls in PACKET_CLASSES}")
    w("_START_RE = re.compile(b\"[%s]\")" % "".join("\\x%02x" % p.start for p in model.packets))
    w("")
    w("Packet = Union[%s]" % ", ".join(p.name for p in model.packets))
    w("")
    w("")
    w("def class_for_start(start: int):")
    w('    """The packet class for a start byte, or None."""')
    w("    return _BY_START.get(start)")
    w("")
    w("")
    w("def class_for_type(ptype: int):")
    w("    return _BY_TYPE.get(ptype)")
    w("")
    w("")
    w("def decode_frame(frame) -> Packet:")
    w('    """Decode one frame of any type; ValueError unless it is exactly one valid frame."""')
    w("    if not frame:")
    w('        raise ValueError("empty frame")')
    w("    cls = _BY_START.get(frame[0])")
    w("    if cls is None:")
    w('        raise ValueError("0x%02x is not a D-Sentry start byte" % frame[0])')
    w("    return cls.decode(frame)")
    w("")
    w("")
    w("# Spec example values per packet (also the example frames in docs/protocol.md)")
    w("EXAMPLES: Dict[str, dict] = {")
    for p in model.packets:
        w("    %r: {" % p.name)
        for f in p.fields:
            w("        %r: %r," % (f.name, tuple(p.example[f.name]) if f.kind == "u8s" else p.example[f.name]))
        w("    },")
    w("}")
    w("")
    w("")
    w("class Frame(NamedTuple):")
    w('    """One accepted frame: its type, its raw bytes (start..crc) and the decoded packet."""')
    w("    type: PacketType")
    w("    raw: bytes")
    w("    packet: Packet")
    w("")
    w("")
    w("class Deframer:")
    w('    """Streaming deframer for all four packet types, same rules as ds_deframer_feed():')
    w("    a frame is accepted only with one of the four start bytes, that type's exact len and a")
    w("    matching CRC; on any mismatch only the first held byte is dropped and the search restarts")
    w("    at the next byte. Bytes outside accepted frames are junk (on the USB port: debug text).")
    w("    Results do not depend on how the stream is split into feed() calls.")
    w("    accept_legacy=True also accepts each packet's older layouts at their exact lengths")
    w("    (LEGACY: an older RA4M1 build's TelemetryPacket); ds_deframer_feed() never does.")
    w('    """')
    w("")
    w("    def __init__(self, accept_legacy: bool = False) -> None:")
    w("        self.accept_legacy = accept_legacy")
    w("        self._buf = bytearray()")
    w("        self.frames = 0")
    w("        self.crc_errors = 0")
    w("        self.len_errors = 0")
    w("        self.junk_bytes = 0")
    w("")
    w("    @property")
    w("    def held(self) -> int:")
    w('        """Bytes held: a strict prefix of a possible frame, waiting for more input."""')
    w("        return len(self._buf)")
    w("")
    w("    def reset(self) -> None:")
    w("        self._buf.clear()")
    w("")
    w("    def feed_ordered(self, data) -> List[Union[Frame, bytes]]:")
    w('        """Frames and junk runs (bytes) in stream order. Junk is reported once resolved:')
    w("        a held prefix of a possible frame is reported by a later call.\"\"\"")
    w("        buf = self._buf")
    w("        buf += data")
    w("        items: List[Union[Frame, bytes]] = []")
    w("        i = 0          # first unresolved byte")
    w("        junk_from = 0  # [junk_from, i) is resolved junk not yet reported")
    w("        n = len(buf)")
    w("        while i < n:")
    w("            cls = _BY_START.get(buf[i])")
    w("            if cls is None:")
    w("                m = _START_RE.search(buf, i + 1)")
    w("                i = m.start() if m else n")
    w("                continue")
    w("            if n - i < DS_FRAME_HEADER_LEN:")
    w("                break")
    w("            length = buf[i + 1]")
    w("            if length != cls.LEN and not (self.accept_legacy and length in cls.LEGACY):")
    w("                self.len_errors += 1")
    w("                i += 1")
    w("                continue")
    w("            end = i + length + DS_FRAME_OVERHEAD")
    w("            if end > n:")
    w("                break")
    w("            if crc8(buf[i + 1:end - 1]) != buf[end - 1]:")
    w("                self.crc_errors += 1")
    w("                i += 1")
    w("                continue")
    w("            if junk_from < i:")
    w("                items.append(bytes(buf[junk_from:i]))")
    w("                self.junk_bytes += i - junk_from")
    w("            raw = bytes(buf[i:end])")
    w("            items.append(Frame(cls.TYPE, raw, cls.from_payload(raw[DS_FRAME_HEADER_LEN:-1])))")
    w("            self.frames += 1")
    w("            i = junk_from = end")
    w("        if junk_from < i:")
    w("            items.append(bytes(buf[junk_from:i]))")
    w("            self.junk_bytes += i - junk_from")
    w("        del buf[:i]")
    w("        return items")
    w("")
    w("    def feed(self, data) -> List[Frame]:")
    w('        """Only the frames completed by data, in order."""')
    w("        return [item for item in self.feed_ordered(data) if isinstance(item, Frame)]")
    return "\n".join(L).rstrip("\n") + "\n"


# ---------------------------------------------------------------------------------------------
# docs/protocol.md
# ---------------------------------------------------------------------------------------------
AT_IN_TEXT_RE = re.compile(r"AT\+[A-Z]+(?:\?|=<\w+>(?:,<\w+>)*)?")


def md_escape(text):
    """Free text for a Markdown table or paragraph: AT commands become code spans (so a
    renderer cannot take <hex> for an HTML tag) and | is escaped."""
    text = AT_IN_TEXT_RE.sub(lambda m: "`%s`" % m.group(), text)
    return text.replace("|", "\\|")


def field_value_text(model, f, value):
    if f.kind == "u8s":
        return ".".join(str(v) for v in value) if f.count == 4 else " ".join("%d" % v for v in value)
    if f.kind == "text":
        return '"%s"' % value
    if f.ref:
        if f.ref in model.enum_names():
            names = [vn for vn, v, _d in model.enum(f.ref)["values"] if v == value]
            if names:
                return "%d (%s)" % (value, names[0])
        else:
            keys = [k for k, bit, _d in model.bitgroup(f.ref)["bits"] if value & (1 << bit)]
            return "0x%0*X (%s)" % (f.size * 2, value, ", ".join(keys) if keys else "none")
    return str(value)


def gen_doc(model):
    L = []
    w = L.append
    w("<!-- GENERATED by %s from %s -- do not edit; run the generator. -->" % (GEN_BY, GEN_FROM))
    w("# D-Sentry wire protocol")
    w("")
    w("> Generated from `%s` by `%s`. The same file produces" % (GEN_FROM, GEN_BY))
    w("> `protocol/dsentry_protocol.h` (and its byte-identical ESP32 copy")
    w("> `firmware/UNOR4USBBridge/dsentry_protocol.h`) and `relay-pc/dsentry/protocol.py`, so the")
    w("> C, Python and this page cannot drift. `gen_protocol.py --check` fails while any is stale.")
    w("> The contract behind it is `docs/architecture.md` section 4.")
    w("")
    w("## Framing")
    w("")
    w("Every packet travels as `%s`. `len` is the %s. All multi-byte fields are"
      % (model.framing["layout"], model.framing["len"]))
    w("%s; field names are identical in C (`dsentry_protocol.h`), Python (`protocol.py`) and here."
      % model.framing["byte_order"])
    w("")
    w("A receiver accepts a frame only when:")
    w("")
    for i, rule in enumerate(model.framing["accept"], 1):
        w("%d. %s;" % (i, rule))
    w("")
    w("and %s." % model.framing["resync"])
    w("")
    w("Why the exact length: %s" % model.framing["why_exact_len"])
    w("")
    c = model.crc
    w("**CRC:** %s, width %d, poly `0x%02X`, init `0x%02X`, no input or output reflection, xor-out"
      % (c["name"], c["width"], c["poly"], c["init"]))
    w("`0x%02X`; check value over ASCII `123456789` = `0x%02X`. It covers %s." % (c["xorout"], c["check"], c["covers"]))
    w("")
    w("| Packet | Start | len | Frame bytes | Purpose |")
    w("|---|---|---|---|---|")
    for p in model.packets:
        w("| [%s](#%s) | `0x%02X` | %d | %d | %s |"
          % (p.name, p.name.lower(), p.start, p.payload_len, p.frame_len,
             md_escape(p.doc.split(". ")[0].rstrip("."))))
    w("")
    w("Constants:")
    w("")
    w("| Name | Value | Meaning |")
    w("|---|---|---|")
    for n, v, d in model.spec.CONSTANTS:
        w("| `%s` | %d | %s |" % (n, v, md_escape(d)))
    w("| `DS_MAX_FRAME_LEN` | %d | largest frame of any packet (deframer buffer size) |" % model.max_frame)
    w("")
    for p in model.packets:
        w("## %s" % p.name)
        w("")
        w("Start `0x%02X`, len %d, frame %d bytes. %s" % (p.start, p.payload_len, p.frame_len, md_escape(p.doc)))
        w("")
        for line in p.carried:
            w("- %s." % md_escape(line))
        w("")
        w("| Frame offset | Field | Type | Meaning |")
        w("|---|---|---|---|")
        w("| 0 | start | u8 | `0x%02X` |" % p.start)
        w("| 1 | len | u8 | %d |" % p.payload_len)
        for f in p.fields:
            off = FRAME_HEADER_LEN + f.offset
            span = str(off) if f.size == 1 else "%d-%d" % (off, off + f.size - 1)
            ref = " (see %s)" % f.ref if f.ref else ""
            since = "" if f.since == 1 else "**v%d:** " % f.since
            w("| %s | `%s` | %s | %s%s%s |" % (span, f.name, f.type, since, md_escape(f.doc), ref))
        w("| %d | crc | u8 | CRC-8 over bytes 1-%d |" % (p.frame_len - 1, p.frame_len - 2))
        w("")
        if p.legacy:
            w("**Versions.** This is version %d (`%s`); fields marked **vN** were appended by "
              "version N." % (p.version, p.version_const))
            for v, plen, fields_v in p.legacy:
                w("A version %d frame ends after `%s`: len %d, frame %d bytes."
                  % (v, fields_v[-1].name, plen, plen + FRAME_OVERHEAD))
            w("The PC side reads the older layouts too (`Deframer(accept_legacy=True)`, which the relay "
              "and the bench console use; the missing fields keep their defaults and `version` says "
              "which layout arrived), so a relay keeps working with an older RA4M1 build. The C "
              "deframer, the RA4M1 and the ESP32's `AT+DST` check accept the current layout only.")
            w("")
        frame = example_frame(model, p)
        w("**Worked example.** These values:")
        w("")
        w("| Frame bytes | Field | Value |")
        w("|---|---|---|")
        w("| `%02x` | start | |" % frame[0])
        w("| `%02x` | len | %d |" % (frame[1], frame[1]))
        for f in p.fields:
            off = FRAME_HEADER_LEN + f.offset
            chunk = frame[off:off + f.size]
            if f.kind == "text":
                shown = hex_spaced(chunk[:8]) + " ..."
            else:
                shown = hex_spaced(chunk)
            w("| `%s` | `%s` | %s |" % (shown, f.name, md_escape(field_value_text(model, f, p.example[f.name]))))
        w("| `%02x` | crc | |" % frame[-1])
        w("")
        w("encode to this %d-byte frame:" % len(frame))
        w("")
        w("<!-- example %s -->" % p.name)
        w("```text")
        for i in range(0, len(frame), DOC_HEX_BYTES_PER_LINE):
            w(hex_spaced(frame[i:i + DOC_HEX_BYTES_PER_LINE]))
        w("```")
        w("")
    w("## Enums")
    w("")
    w("Each is one byte on the wire. C: `#define` with the prefix shown and `%s()`-style name functions;"
      % model.enums[0]["c_name_func"])
    w("Python: an `IntEnum` of the same name.")
    w("")
    for e in model.enums:
        w("### %s" % e["name"])
        w("")
        w("%s C prefix `%s`." % (md_escape(e["doc"]), e["prefix"]))
        w("")
        w("| Value | Name | Meaning |")
        w("|---|---|---|")
        for vn, v, d in e["values"]:
            w("| %d | `%s` | %s |" % (v, vn, md_escape(d)))
        w("")
    w("## Bit masks")
    w("")
    w("C: `#define <prefix><KEY>` = `1 << bit` (plus `<prefix>ALL`). Python: an `IntFlag` of the same")
    w("name, and `BIT_KEYS` / `bits_to_dict()` for the camelCase keys below.")
    w("")
    for b in model.bits:
        w("### %s" % b["name"])
        w("")
        w("%s %d bits, C prefix `%s`." % (md_escape(b["doc"]), b["width"], b["prefix"]))
        w("")
        w("| Bit | Mask | Key | C macro | Meaning |")
        w("|---|---|---|---|---|")
        for k, bit, d in b["bits"]:
            w("| %d | `0x%0*X` | `%s` | `%s%s` | %s |"
              % (bit, b["width"] // 4, 1 << bit, k, b["prefix"], upper_snake(k), md_escape(d)))
        w("")
    w("## AT commands (RA4M1 <-> ESP32, Serial2 at LINK_BAUD)")
    w("")
    w("Served by the ESP32 fork's chAT server next to the existing Xbox commands. Every command")
    w("answers with its prompt and `OK` (`+NAME: ...\\r\\nOK\\r\\n`); failures go in the payload, never")
    w("a bare `ERROR`, because the RA4M1 would only notice that at its timeout. Arguments and reply")
    w("fields are decimal unless the type says hex. C names: `DS_AT_<ID>` (the `+NAME` string),")
    w("`DS_AT_<ID>_ARGC`, `DS_AT_<ID>_ARG_<NAME>` (argument index), `DS_AT_<ID>_REPLY_COUNT`,")
    w("`DS_AT_<ID>_REPLY_<NAME>` (reply field index).")
    w("")
    w("| Command | Reply | Meaning |")
    w("|---|---|---|")
    for cmd in model.at:
        w("| `%s` | `%s` | %s |" % (at_format(cmd), at_reply_format(cmd), md_escape(cmd["doc"])))
    w("")
    for cmd in model.at:
        w("### `%s`" % at_format(cmd))
        w("")
        if cmd["args"]:
            w("| Arg | Name | Type | Meaning |")
            w("|---|---|---|---|")
            for i, (an, at, ad) in enumerate(cmd["args"]):
                w("| %d | `%s` | %s | %s |" % (i, an, at, md_escape(ad)))
            w("")
        w("| Reply field | Name | Type | Meaning |")
        w("|---|---|---|---|")
        for i, (rn, rt, rd) in enumerate(cmd["reply"]):
            w("| %d | `%s` | %s | %s |" % (i, rn, rt, md_escape(rd)))
        w("")
        ex = cmd["example"]
        args = []
        for (an, at, _ad), value in zip(cmd["args"], ex["args"]):
            if at == "hex":
                args.append(example_frame(model, model.packet(value)).hex())
            else:
                args.append(str(value))
        if cmd["mode"] == "read":
            req = "AT%s?" % cmd["name"]
        elif cmd["mode"] == "run":
            req = "AT%s" % cmd["name"]
        else:
            req = "AT%s=%s" % (cmd["name"], ",".join(args))
        rep = "%s: %s" % (cmd["name"], ",".join(str(v) for v in ex["reply"]))
        w("Example (%d request bytes on the wire):" % (len(req) + 2))
        w("")
        w("```text")
        w("-> %s\\r\\n" % req)
        w("<- %s\\r\\nOK\\r\\n" % rep)
        w("```")
        w("")
    w("### Existing fork commands D-Sentry uses")
    w("")
    w("Wire format in `protocol/xbox_r4_protocol.h` and the repo root's `docs/protocol.md`.")
    w("")
    w("| Command | Reply | D-Sentry use |")
    w("|---|---|---|")
    for req, rep, doc in model.at_existing:
        w("| `%s` | `%s` | %s |" % (req, md_escape(rep), md_escape(doc)))
    w("")
    return "\n".join(L).rstrip("\n") + "\n"


# ---------------------------------------------------------------------------------------------
# tests/unit/test_protocol_c.cpp
# ---------------------------------------------------------------------------------------------
def ctest_stream(model):
    """A deterministic byte stream exercising the deframer: debug text with 'Z', split frames,
    a truncated frame, bad CRCs, wrong len bytes, and a frame hidden inside a rejected one."""
    ex = {p.name: example_frame(model, p) for p in model.packets}
    ctrl, stat, tel, bea = (ex["ControlPacket"], ex["StatusPacket"], ex["TelemetryPacket"],
                            ex["BeaconPacket"])
    bad_ctrl = bytearray(ctrl)
    bad_ctrl[5] ^= 0x40
    s = bytearray()
    s += b"[2] STATE PAIRING -> DISARMED Zap Zz\r\n"
    s += ctrl
    s += b"Z\x07junk\r\n"            # 'Z' + StatusPacket's len byte, then text: a CRC reject
    s += stat
    s += tel[:40] + tel[41:]         # telemetry missing one byte swallows the next frame's start
    s += stat                        # ... and this StatusPacket is still recovered
    s += bytes(bad_ctrl)             # bad CRC
    s += bea + tel + b"Z" + bytes([0xA5, 0x0B]) + b"ok\n"
    s += ctrl[:6]                    # held at the end: a strict prefix
    return bytes(s)


def gen_ctest(model):
    stream = ctest_stream(model)
    frames, crc_errors, len_errors, junk, held = ref_deframe(model, stream)
    L = []
    w = L.append
    w("// test_protocol_c.cpp -- checks dsentry_protocol.h against the spec's example frames and")
    w("// the reference deframer, and gives the Python tests a C side to compare with.")
    w("// GENERATED by %s from %s -- do not edit." % (GEN_BY, GEN_FROM))
    w("//")
    w("// Build (host): clang++ -std=gnu++17 -Wall -Wextra -I D-SentryCode/protocol test_protocol_c.cpp")
    w("// Run:  test_protocol_c            self-test; exit 0 when every check passes")
    w("//       test_protocol_c --dump     print \"ENC <Packet> <hex>\" per example and \"CRC <hex>\"")
    w("//       test_protocol_c --deframe  read hex from stdin, print \"FRAME <type> <hex> <field=value>...\"")
    w("//                                  per frame, then \"STATS frames=.. crc=.. len=.. junk=.. held=..\"")
    w("// relay-pc/tests/test_protocol_c.py compiles it and compares both modes with protocol.py.")
    w("//")
    w("// SPDX-License-Identifier: MIT")
    w("#include \"dsentry_protocol.h\"")
    w("")
    w("#include <stdio.h>")
    w("#include <string.h>")
    w("")
    w("static int g_failures = 0;")
    w("static int g_checks = 0;")
    w("")
    w("static void check(bool ok, const char* what) {")
    w("  g_checks++;")
    w("  if (!ok) {")
    w("    g_failures++;")
    w("    printf(\"FAIL  %s\\n\", what);")
    w("  }")
    w("}")
    w("")
    w("static void print_hex(const uint8_t* data, size_t len) {")
    w("  for (size_t i = 0; i < len; i++) printf(\"%02x\", data[i]);")
    w("}")
    w("")
    # example builders
    for p in model.packets:
        w("static void example_%s(%s* p) {" % (p.id, p.ctype))
        w("  ds_%s_init(p);" % p.id)
        for f in p.fields:
            v = p.example[f.name]
            if f.kind == "int":
                lit = str(v) + ("u" if f.type == "u32" else "")
                if f.type == "i32" and v == -0x80000000:
                    lit = "INT32_MIN"
                w("  p->%s = %s;" % (f.name, lit))
            elif f.kind == "u8s":
                for i, b in enumerate(v):
                    w("  p->%s[%d] = %d;" % (f.name, i, b))
            else:
                w("  memcpy(p->%s, %s, %sLEN);" % (f.name, c_string(v), p.macro + upper_snake(f.name) + "_"))
        w("}")
        w("")
    # field printers
    for p in model.packets:
        w("static void print_%s_fields(const %s* p) {" % (p.id, p.ctype))
        for f in p.fields:
            if f.kind == "int":
                if f.type in ("u32",):
                    w("  printf(\" %s=%%lu\", (unsigned long)p->%s);" % (f.name, f.name))
                elif f.type in ("i32",):
                    w("  printf(\" %s=%%ld\", (long)p->%s);" % (f.name, f.name))
                elif f.type.startswith("u"):
                    w("  printf(\" %s=%%u\", (unsigned)p->%s);" % (f.name, f.name))
                else:
                    w("  printf(\" %s=%%d\", (int)p->%s);" % (f.name, f.name))
            else:
                w("  printf(\" %s=\");" % f.name)
                w("  print_hex((const uint8_t*)p->%s, sizeof p->%s);" % (f.name, f.name))
        w("}")
        w("")
    w("static void print_frame(uint8_t type, const uint8_t* frame, size_t len) {")
    w("  printf(\"FRAME %s \", ds_packet_type_name(type));")
    w("  print_hex(frame, len);")
    w("  switch (type) {")
    for p in model.packets:
        w("    case %s: {" % p.ptype)
        w("      %s p;" % p.ctype)
        w("      ds_%s_init(&p);" % p.id)
        w("      uint8_t again[%sFRAME_LEN];" % p.macro)
        w("      bool ok = ds_%s_decode(frame, len, &p);" % p.id)
        w("      ok = ok && ds_%s_encode(&p, again) == len && memcmp(again, frame, len) == 0;" % p.id)
        w("      print_%s_fields(&p);" % p.id)
        w("      printf(\" roundtrip=%s\", ok ? \"ok\" : \"BAD\");")
        w("      break;")
        w("    }")
    w("    default: break;")
    w("  }")
    w("  printf(\"\\n\");")
    w("}")
    w("")
    # expected data
    w("// Expected example frames, computed by gen_protocol.py's reference encoder.")
    for p in model.packets:
        fr = example_frame(model, p)
        w("static const uint8_t k_expect_%s[%sFRAME_LEN] = {" % (p.id, p.macro))
        for i in range(0, len(fr), 12):
            w("  " + ", ".join("0x%02x" % b for b in fr[i:i + 12]) + ",")
        w("};")
    w("")
    w("// Deframer stream and its expected result (reference deframer in gen_protocol.py).")
    w("static const uint8_t k_stream[] = {")
    for i in range(0, len(stream), 12):
        w("  " + ", ".join("0x%02x" % b for b in stream[i:i + 12]) + ",")
    w("};")
    w("static const uint8_t k_stream_types[] = {%s};" % ", ".join(p.ptype for p, _f in frames))
    w("static const size_t k_stream_frames = %d;" % len(frames))
    w("static const uint32_t k_stream_crc_errors = %d;" % crc_errors)
    w("static const uint32_t k_stream_len_errors = %d;" % len_errors)
    w("static const uint32_t k_stream_junk = %d;" % junk)
    w("static const uint8_t k_stream_held = %d;" % held)
    w("")
    w("static int self_test(void) {")
    w("  static const uint8_t check_input[] = {'1', '2', '3', '4', '5', '6', '7', '8', '9'};")
    w("  check(ds_crc8(check_input, sizeof check_input) == DS_CRC8_CHECK, \"crc8 check value 0xF4\");")
    for p in model.packets:
        w("  {")
        w("    %s p;" % p.ctype)
        w("    %s q;" % p.ctype)
        w("    uint8_t out[%sFRAME_LEN];" % p.macro)
        w("    example_%s(&p);" % p.id)
        w("    check(ds_%s_encode(&p, out) == %sFRAME_LEN, \"%s encode length\");" % (p.id, p.macro, p.name))
        w("    check(memcmp(out, k_expect_%s, sizeof out) == 0, \"%s example frame matches the spec\");"
          % (p.id, p.name))
        w("    check(ds_%s_decode(out, sizeof out, &q) && memcmp(&p, &q, sizeof p) == 0, \"%s decode round trip\");"
          % (p.id, p.name))
        w("    check(!ds_%s_decode(out, sizeof out - 1, &q), \"%s rejects a short frame\");" % (p.id, p.name))
        w("    out[%sFRAME_LEN - 1] = (uint8_t)(out[%sFRAME_LEN - 1] ^ 1u);" % (p.macro, p.macro))
        w("    check(!ds_%s_decode(out, sizeof out, &q), \"%s rejects a bad CRC\");" % (p.id, p.name))
        w("    out[%sFRAME_LEN - 1] = (uint8_t)(out[%sFRAME_LEN - 1] ^ 1u);" % (p.macro, p.macro))
        w("    out[1] = (uint8_t)(out[1] ^ 1u);")
        w("    check(!ds_frame_check(out, sizeof out, %s), \"%s rejects a wrong len byte\");" % (p.ptype, p.name))
        w("  }")
    w("  {")
    w("    ds_deframer_t d;")
    w("    ds_deframer_init(&d);")
    w("    size_t found = 0;")
    w("    bool order_ok = true;")
    w("    for (size_t i = 0; i < sizeof k_stream; i++) {")
    w("      uint8_t t = ds_deframer_feed(&d, k_stream[i]);")
    w("      while (t != DS_PKT_NONE) {")
    w("        if (found >= k_stream_frames || k_stream_types[found] != t ||")
    w("            !ds_frame_check(ds_deframer_frame(&d), ds_deframer_frame_len(&d), t)) {")
    w("          order_ok = false;")
    w("        }")
    w("        found++;")
    w("        t = ds_deframer_poll(&d);")
    w("      }")
    w("    }")
    w("    check(order_ok && found == k_stream_frames, \"deframer finds the expected frames in order\");")
    w("    check(d.crc_errors == k_stream_crc_errors, \"deframer CRC error count\");")
    w("    check(d.len_errors == k_stream_len_errors, \"deframer len error count\");")
    w("    check(d.junk_bytes == k_stream_junk, \"deframer junk byte count\");")
    w("    check(d.held == k_stream_held, \"deframer holds the trailing prefix\");")
    w("  }")
    w("  {")
    w("    uint8_t frame[DS_TELEMETRY_FRAME_LEN];")
    w("    char hex[DS_AT_DST_HEX_LEN + 1];")
    w("    uint8_t back[DS_TELEMETRY_FRAME_LEN];")
    w("    ds_telemetry_packet_t p;")
    w("    example_telemetry(&p);")
    w("    ds_telemetry_encode(&p, frame);")
    w("    check(ds_hex_encode(frame, sizeof frame, hex) == DS_AT_DST_HEX_LEN, \"hex encode length\");")
    w("    check(ds_hex_decode(hex, DS_AT_DST_HEX_LEN, back, sizeof back) == sizeof back &&")
    w("          memcmp(back, frame, sizeof frame) == 0, \"hex round trip\");")
    w("    check(ds_hex_decode(hex, DS_AT_DST_HEX_LEN - 1, back, sizeof back) == 0, \"hex rejects odd length\");")
    w("    check(ds_hex_decode(\"0G\", 2, back, sizeof back) == 0, \"hex rejects a non-digit\");")
    w("    check(ds_hex_decode(\"A5ff\", 4, back, 1) == 0, \"hex rejects overflow\");")
    w("    check(ds_hex_decode(\"A5ff\", 4, back, 2) == 2 && back[0] == 0xA5 && back[1] == 0xFF, \"hex accepts both cases\");")
    w("  }")
    w("  check(strcmp(ds_turret_state_name(DS_STATE_ARMED), \"ARMED\") == 0, \"enum names\");")
    w("  check(strcmp(ds_esp_mode_name(DS_ESP_MODE_UNKNOWN), \"UNKNOWN\") == 0, \"enum names (255)\");")
    w("  check(strcmp(ds_fault_name(200), \"?\") == 0, \"unknown enum value\");")
    w("  printf(\"%s  test_protocol_c: %d checks, %d failures\\n\", g_failures ? \"FAIL\" : \"PASS\", g_checks, g_failures);")
    w("  return g_failures ? 1 : 0;")
    w("}")
    w("")
    w("static void dump(void) {")
    w("  static const uint8_t check_input[] = {'1', '2', '3', '4', '5', '6', '7', '8', '9'};")
    w("  printf(\"CRC %02x\\n\", ds_crc8(check_input, sizeof check_input));")
    for p in model.packets:
        w("  {")
        w("    %s p;" % p.ctype)
        w("    uint8_t out[%sFRAME_LEN];" % p.macro)
        w("    example_%s(&p);" % p.id)
        w("    size_t n = ds_%s_encode(&p, out);" % p.id)
        w("    printf(\"ENC %s \");" % p.name)
        w("    print_hex(out, n);")
        w("    printf(\"\\n\");")
        w("  }")
    w("}")
    w("")
    w("static void deframe_stdin(void) {")
    w("  ds_deframer_t d;")
    w("  ds_deframer_init(&d);")
    w("  int hi = -1;")
    w("  int c;")
    w("  while ((c = getchar()) != EOF) {")
    w("    int v = ds_hex_digit((char)c);")
    w("    if (v < 0) continue;")
    w("    if (hi < 0) {")
    w("      hi = v;")
    w("      continue;")
    w("    }")
    w("    uint8_t t = ds_deframer_feed(&d, (uint8_t)((hi << 4) | v));")
    w("    hi = -1;")
    w("    while (t != DS_PKT_NONE) {")
    w("      print_frame(t, ds_deframer_frame(&d), ds_deframer_frame_len(&d));")
    w("      t = ds_deframer_poll(&d);")
    w("    }")
    w("  }")
    w("  printf(\"STATS frames=%lu crc=%lu len=%lu junk=%lu held=%u\\n\", (unsigned long)d.frames,")
    w("         (unsigned long)d.crc_errors, (unsigned long)d.len_errors, (unsigned long)d.junk_bytes,")
    w("         (unsigned)d.held);")
    w("}")
    w("")
    w("int main(int argc, char** argv) {")
    w("  if (argc > 1 && strcmp(argv[1], \"--dump\") == 0) {")
    w("    dump();")
    w("    return 0;")
    w("  }")
    w("  if (argc > 1 && strcmp(argv[1], \"--deframe\") == 0) {")
    w("    deframe_stdin();")
    w("    return 0;")
    w("  }")
    w("  return self_test();")
    w("}")
    return "\n".join(L).rstrip("\n") + "\n"


# ---------------------------------------------------------------------------------------------
def outputs(model):
    header = gen_header(model)
    return [
        (OUT_HEADER, header),
        (OUT_ESP32_HEADER, header),       # byte-identical copy
        (OUT_PY, gen_python(model)),
        (OUT_DOC, gen_doc(model)),
        (OUT_CTEST, gen_ctest(model)),
    ]


def rel(path):
    try:
        return str(path.relative_to(REPO))
    except ValueError:
        return str(path)


def main(argv=None):
    ap = argparse.ArgumentParser(description="Generate the D-Sentry protocol files from protocol_spec.py.")
    ap.add_argument("--check", action="store_true",
                    help="write nothing; exit 1 if any generated file is missing or stale")
    args = ap.parse_args(argv)
    try:
        model = Model(load_spec())
    except SpecError as e:
        print("gen_protocol: invalid spec: %s" % e, file=sys.stderr)
        return 1
    stale = []
    for path, text in outputs(model):
        current = path.read_bytes() if path.exists() else None
        wanted = text.encode("utf-8")
        if current == wanted:
            if not args.check:
                print("unchanged %s" % rel(path))
            continue
        if args.check:
            stale.append(path)
            print("STALE %s%s" % (rel(path), "" if current is not None else " (missing)"))
            continue
        path.parent.mkdir(parents=True, exist_ok=True)
        path.write_bytes(wanted)
        print("wrote %s" % rel(path))
    if args.check:
        if stale:
            print("gen_protocol --check: %d stale file(s); run tools/gen_protocol.py" % len(stale), file=sys.stderr)
            return 1
        print("gen_protocol --check: all generated files are current")
    return 0


if __name__ == "__main__":
    sys.exit(main())

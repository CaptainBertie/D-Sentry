#!/usr/bin/env python3
# sync_config.py -- mirrors the ESP32-relevant part of D-SentryCode/config.h (the only tunables
# file) into the ESP32-S3 fork as firmware/UNOR4USBBridge/dsentry_config.h, so both builds share
# one source of truth (spec 6 step 5, architecture.md section 1).
#
# What is copied:
#   1. sections "10. CONTROLLER / LINK" and "11. ESP32-S3 RADIO FIRMWARE" verbatim, found by
#      their "// N. TITLE" header comments (each directly under a "// ----" rule line);
#   2. every other #define whose name starts with one of NAME_PREFIXES or is listed in
#      ESP32_EXTRA (the selection is by name, so defines added later, e.g. in sections 13/14,
#      are picked up without editing this script);
#   3. any config.h define that something already selected refers to (e.g. US_PER_MS inside an
#      expression), so the copy always compiles on its own.
# Refused, loudly: a missing or renamed section header, and any selected define that is a pin
# (PIN_*) or refers to a name config.h does not define (A0, Serial2, NEO_GRB: those need
# <Arduino.h>, which the fork's copy must not include). Also refused: a copied name that the fork
# or its ESP32 core already #defines (e.g. a new ESP_* tunable clashing with ESP-IDF), because
# the redefinition would silently change one side.
#
# Usage (stdlib only):
#   "<repo>/.venv/bin/python" D-SentryCode/tools/sync_config.py           write if changed
#   "<repo>/.venv/bin/python" D-SentryCode/tools/sync_config.py --check   exit 1 if stale
# Exit status: 0 ok, 1 stale (--check) or a refused config.h, 2 usage error.
#
# SPDX-License-Identifier: MIT

import argparse
import re
import sys
from pathlib import Path

HERE = Path(__file__).resolve().parent
DSC = HERE.parent
REPO = DSC.parent
CONFIG_H = DSC / "config.h"
FORK = REPO / "firmware" / "UNOR4USBBridge"
OUT = FORK / "dsentry_config.h"
ESP32_CORE = REPO / "firmware" / "hardware" / "esp32-patched" / "esp32"
CORE_HEADER_DIRS = ("cores", "libraries", "tools/sdk/esp32s3/include")

# (section number, a word its title must contain): copied verbatim, in this order
VERBATIM_SECTIONS = ((10, "CONTROLLER"), (11, "ESP32"))
# Any #define whose name starts with one of these is ESP32 business. XSTATE_ is the 20-byte
# AT+XBOXSTATE? layout (config.h section 14), which the ESP32 builds from relay ControlPackets
# in WiFi mode.
NAME_PREFIXES = ("WIFI_", "ESP_", "BLE_", "XSTATE_")
# Further names the ESP32 additions need (names not in config.h yet are reported, not fatal:
# sections 13 and 14 are added by a later stage).
ESP32_EXTRA = (
    "LINK_BAUD",              # section 1: the AT link baud, must match Serial1 on the ESP32
    "LINK_TIMEOUT_MS",        # relay freshness (wifiStatus 4, XBOXSTATE state 0 vs 3)
    "CONTROL_PACKET_HZ",      # relay send rate, XBOXSTATE poll rate
    "JOY_AXIS_RANGE",         # ControlPacket -> 20-byte XBOXSTATE scaling
    "TRIGGER_AXIS_RANGE",
    "STATUS_HEARTBEAT_MS",    # how often AT+DSS repeats
    "TELEMETRY_WIFI_HZ",      # how often AT+DST arrives
    "US_PER_S",               # unit constants (section 14)
    "MS_PER_S",
    "US_PER_MS",
    "UART_BITS_PER_BYTE",
    "BITS_PER_BYTE",          # byte assembly and hex/decimal formatting (section 14)
    "BYTE_MASK",
    "DEC_BASE",
    "HEX_BASE",
)
# Never copied: these need the Arduino core.
PIN_PREFIX = "PIN_"
# Identifiers a copied value may use that are not config.h defines.
ALLOWED_WORDS = {
    "true", "false", "sizeof", "unsigned", "signed", "int", "long", "short", "char", "float",
    "double", "bool", "const", "size_t", "uint8_t", "int8_t", "uint16_t", "int16_t", "uint32_t",
    "int32_t", "uint64_t", "int64_t",
}

RULE_RE = re.compile(r"^\s*//\s*-{10,}\s*$")
HEADER_RE = re.compile(r"^\s*//\s*(\d+)\.\s+(\S.*?)\s*$")
DEFINE_RE = re.compile(r"^\s*#\s*define\s+([A-Za-z_]\w*)(\([^)]*\))?(.*)$")
CONTINUATION_COMMENT_RE = re.compile(r"^\s+//")
TOKEN_RE = re.compile(r"[A-Za-z_]\w*|[0-9.][\w.']*")
STRING_RE = re.compile(r'"(?:[^"\\\n]|\\.)*"|\'(?:[^\'\\\n]|\\.)+\'')
CORE_DEFINE_RE = re.compile(rb"^[ \t]*#[ \t]*define[ \t]+([A-Za-z_]\w*)", re.M)


class Refused(Exception):
    pass


def strip_comment(value):
    """Value text of a #define line without its // or /* */ comment (strings respected)."""
    out, i = [], 0
    while i < len(value):
        c = value[i]
        if c == '"':
            m = STRING_RE.match(value, i)
            j = m.end() if m else len(value)
            out.append(value[i:j])
            i = j
            continue
        if value.startswith("//", i):
            break
        if value.startswith("/*", i):
            j = value.find("*/", i + 2)
            i = len(value) if j < 0 else j + 2
            out.append(" ")
            continue
        out.append(c)
        i += 1
    return "".join(out).strip()


def identifiers(value):
    """Names a value refers to (numbers with suffixes such as 0x27 or 5.0f are skipped)."""
    return [t for t in TOKEN_RE.findall(STRING_RE.sub(" ", value)) if not t[0].isdigit() and t[0] != "."]


class Define:
    def __init__(self, name, value, first, last, section):
        self.name = name
        self.value = value
        self.first = first      # 0-based index of the #define line
        self.last = last        # last line of its continuation comment
        self.section = section  # (number, title) or None


def parse(lines):
    """-> (sections {number: (title, first, last)}, defines {name: Define} in file order)."""
    headers = []
    for i, line in enumerate(lines):
        m = HEADER_RE.match(line)
        if m and i > 0 and RULE_RE.match(lines[i - 1]):
            headers.append((int(m.group(1)), m.group(2), i - 1))
    sections = {}
    for k, (num, title, start) in enumerate(headers):
        end = headers[k + 1][2] - 1 if k + 1 < len(headers) else len(lines) - 1
        while end > start and not lines[end].strip():
            end -= 1
        if num in sections:
            raise Refused("config.h has two sections numbered %d" % num)
        sections[num] = (title, start, end)
    defines = {}
    current = None
    k = 0
    i = 0
    while i < len(lines):
        while k < len(headers) and headers[k][2] <= i:
            current = (headers[k][0], headers[k][1])
            k += 1
        m = DEFINE_RE.match(lines[i])
        if not m:
            i += 1
            continue
        first = i
        value = m.group(3)
        while value.rstrip().endswith("\\") and i + 1 < len(lines):
            i += 1
            value = value.rstrip()[:-1] + " " + lines[i]
        while i + 1 < len(lines) and CONTINUATION_COMMENT_RE.match(lines[i + 1]):
            i += 1
        name = m.group(1)
        if m.group(2) is not None:
            value = m.group(2) + " " + value
        defines.pop(name, None)
        defines[name] = Define(name, strip_comment(value), first, i, current)
        i += 1
    return sections, defines


def core_defines(skip):
    """Names #defined by the fork's own sources and its ESP32 core (examples excluded)."""
    found = {}
    roots = [(FORK, False)] + [(ESP32_CORE / d, True) for d in CORE_HEADER_DIRS]
    for root, recursive in roots:
        if not root.is_dir():
            continue
        files = root.rglob("*") if recursive else root.iterdir()
        for p in files:
            if p.suffix not in (".h", ".hpp", ".c", ".cpp", ".ino") or not p.is_file():
                continue
            if p.resolve() in skip or "examples" in p.parts or "build" in p.relative_to(root).parts:
                continue
            for m in CORE_DEFINE_RE.finditer(p.read_bytes()):
                found.setdefault(m.group(1).decode(), p)
    return found


def build(config_text, check_collisions=True, notes=None):
    """-> the dsentry_config.h text. Raises Refused."""
    notes = notes if notes is not None else []
    lines = config_text.split("\n")
    sections, defines = parse(lines)
    blocks = []
    verbatim = set()
    for num, word in VERBATIM_SECTIONS:
        if num not in sections:
            raise Refused('config.h has no "// %d. ..." section header (under a "// ----" rule); '
                          "sync_config.py copies that section verbatim" % num)
        title, start, end = sections[num]
        if word not in title.upper():
            raise Refused('config.h section %d is "%s", expected a title containing %s; update '
                          "VERBATIM_SECTIONS in sync_config.py if the sections were renumbered"
                          % (num, title, word))
        blocks.append("\n".join(lines[start:end + 1]))
        verbatim.update(d.name for d in defines.values() if start <= d.first <= end)

    reasons = {}
    for d in defines.values():
        if d.name in verbatim:
            continue
        if d.name.startswith(NAME_PREFIXES):
            reasons[d.name] = "name starts with %s" % next(p for p in NAME_PREFIXES if d.name.startswith(p))
        elif d.name in ESP32_EXTRA:
            reasons[d.name] = "listed in ESP32_EXTRA"
    missing = [name for name in ESP32_EXTRA if name not in defines]
    if missing:
        notes.append("ESP32_EXTRA names not in config.h yet (skipped): %s" % ", ".join(missing))

    # dependency closure, then refuse anything Arduino-only
    selected = list(verbatim) + list(reasons)
    todo = list(selected)
    problems = []
    while todo:
        name = todo.pop()
        d = defines[name]
        if name.startswith(PIN_PREFIX):
            problems.append("%s (config.h:%d) is a pin; pins need <Arduino.h>" % (name, d.first + 1))
        for ident in identifiers(d.value):
            if ident in ALLOWED_WORDS:
                continue
            if ident not in defines:
                problems.append("%s (config.h:%d) uses %s, which config.h does not define "
                                "(an Arduino name?)" % (name, d.first + 1, ident))
                continue
            if ident not in verbatim and ident not in reasons:
                reasons[ident] = "needed by %s" % name
                todo.append(ident)
    if problems:
        raise Refused("these cannot go to the ESP32 build:\n  " + "\n  ".join(sorted(set(problems))))

    extras = [d for d in defines.values() if d.name in reasons]
    if extras:
        out = ["// -----------------------------------------------------------------------------",
               "// Other ESP32-relevant defines from config.h (selected by name; see sync_config.py)",
               "// -----------------------------------------------------------------------------"]
        for d in extras:
            # "1. PIN MAP  (final assignment ...)" -> "1. PIN MAP"
            where = "%d. %s" % (d.section[0], d.section[1].split("  ")[0]) if d.section else "before section 1"
            out.append("// config.h:%d (%s): %s" % (d.first + 1, where, reasons[d.name]))
            out.extend(lines[d.first:d.last + 1])
        blocks.append("\n".join(out))

    if check_collisions:
        names = list(verbatim) + [d.name for d in extras]
        clash = core_defines({OUT.resolve(), (FORK / "dsentry_protocol.h").resolve()})
        hits = sorted("%s (also #defined in %s)" % (n, rel(clash[n])) for n in names if n in clash)
        if hits:
            raise Refused("these names already exist in the ESP32 build; rename them in config.h:\n  "
                          + "\n  ".join(hits))

    head = [
        "// =============================================================================",
        "//  dsentry_config.h  --  the ESP32-S3 fork's slice of D-SentryCode/config.h",
        "//  GENERATED by D-SentryCode/tools/sync_config.py from D-SentryCode/config.h.",
        "//  Do not edit: change config.h, run tools/sync_config.py and commit both files;",
        "//  `sync_config.py --check` fails while this copy is stale.",
        "//",
        "//  Contents: config.h sections 10 and 11 verbatim, then every other #define whose",
        "//  name starts with %s or is listed in ESP32_EXTRA, plus what those refer to." % ", ".join(NAME_PREFIXES),
        "//  No pin and no Arduino object is copied, so this header needs no <Arduino.h>.",
        "//",
        "//  SPDX-License-Identifier: MIT",
        "// =============================================================================",
        "#pragma once",
    ]
    return "\n".join(head) + "\n\n" + "\n\n".join(blocks) + "\n"


def rel(path):
    try:
        return str(Path(path).relative_to(REPO))
    except ValueError:
        return str(path)


def main(argv=None):
    ap = argparse.ArgumentParser(description="Mirror config.h's ESP32 part into the fork.")
    ap.add_argument("--check", action="store_true", help="write nothing; exit 1 if the copy is stale")
    ap.add_argument("--config", type=Path, default=CONFIG_H, help="config.h to read (default: %(default)s)")
    ap.add_argument("--out", type=Path, default=OUT, help="header to write (default: %(default)s)")
    ap.add_argument("--no-collision-check", action="store_true",
                    help="skip the scan of the fork and its ESP32 core for clashing #defines")
    args = ap.parse_args(argv)
    notes = []
    try:
        text = build(args.config.read_text(encoding="utf-8"), not args.no_collision_check, notes)
    except (Refused, OSError) as e:
        print("sync_config: %s" % e, file=sys.stderr)
        return 1
    for n in notes:
        print("sync_config: note: %s" % n, file=sys.stderr)
    current = args.out.read_bytes() if args.out.exists() else None
    if current == text.encode("utf-8"):
        print("sync_config: %s is current" % rel(args.out))
        return 0
    if args.check:
        print("sync_config --check: %s is %s; run tools/sync_config.py"
              % (rel(args.out), "missing" if current is None else "stale"), file=sys.stderr)
        return 1
    args.out.write_bytes(text.encode("utf-8"))
    print("sync_config: wrote %s" % rel(args.out))
    return 0


if __name__ == "__main__":
    sys.exit(main())

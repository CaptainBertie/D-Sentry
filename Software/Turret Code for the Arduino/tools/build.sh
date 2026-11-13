#!/bin/bash
# build.sh -- builds and tests everything in D-SentryCode that can be built without a board
# (docs/architecture.md section 8). Never uploads or flashes anything.
#   1. generators: gen_protocol.py --check, sync_config.py --check (generated files current)
#   2. tests/run.sh: host unit tests, dsentry_sim, Python tests, the acceptance tests against
#      dsentry_sim (tests/acceptance), check_literals, the chAT and ESP32 host suites
#   3. arduino-cli: clean compile of firmware-ra4m1/dsentry with --warnings all; fails on any
#      warning in D-Sentry code and unless the core and libraries it used match
#      firmware-ra4m1/libraries.txt; reports flash and RAM
#   4. PlatformIO: pio run in firmware-ra4m1, same version check
#
# arduino-cli runs WITHOUT --profile: a profile build installs its pinned copies into
# arduino-cli's profile cache, which can need the network. Instead the versions of the copies
# actually compiled are checked against libraries.txt (the same pins as dsentry/sketch.yaml).
#
# Usage: tools/build.sh [-v] [--skip-tests] [--skip-arduino] [--skip-pio]
# Env:   DSENTRY_BUILD_DIR  where every build output goes (default D-SentryCode/build, which git
#                           ignores; nothing is ever written inside the sketch folder)
#        PYTHON             default <repo>/.venv/bin/python, else python3
#        ARDUINO_CLI        default arduino-cli
#        PIO                default pio on PATH, else ~/.platformio/penv/bin/pio
# Exit status: 0 when every step passes, 1 otherwise, 2 on a usage error.
#
# SPDX-License-Identifier: MIT
set -u

usage() {
  printf 'Usage: %s [-v] [--skip-tests] [--skip-arduino] [--skip-pio]\n' "$0"
}

VERBOSE=0
DO_TESTS=1
DO_ARDUINO=1
DO_PIO=1
for arg in "$@"; do
  case "$arg" in
    -v) VERBOSE=1 ;;
    --skip-tests) DO_TESTS=0 ;;
    --skip-arduino) DO_ARDUINO=0 ;;
    --skip-pio) DO_PIO=0 ;;
    -h|--help) usage; exit 0 ;;
    *) printf 'build.sh: unknown argument: %s\n' "$arg" >&2; usage >&2; exit 2 ;;
  esac
done

HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
DSC="$(cd "$HERE/.." && pwd)"
REPO="$(cd "$DSC/.." && pwd)"
FW="$DSC/firmware-ra4m1"
SKETCH="$FW/dsentry"
PINS="$FW/libraries.txt"
FQBN="arduino:renesas_uno:unor4wifi"
if [ -n "${PYTHON:-}" ]; then
  PY="$PYTHON"
elif [ -x "$REPO/.venv/bin/python" ]; then
  PY="$REPO/.venv/bin/python"
else
  PY="python3"
fi
ARDUINO_CLI="${ARDUINO_CLI:-arduino-cli}"
if [ -z "${PIO:-}" ]; then
  if command -v pio > /dev/null 2>&1; then
    PIO="pio"
  else
    PIO="$HOME/.platformio/penv/bin/pio"
  fi
fi
if ! mkdir -p "${DSENTRY_BUILD_DIR:-$DSC/build}"; then
  printf 'build.sh: cannot create %s\n' "${DSENTRY_BUILD_DIR:-$DSC/build}" >&2
  exit 1
fi
BUILD="$(cd "${DSENTRY_BUILD_DIR:-$DSC/build}" && pwd)"

npass=0
nfail=0
say() { printf '%s\n' "$*"; }
pass() { say "PASS  $1"; npass=$((npass + 1)); }
fail() { say "FAIL  $1"; nfail=$((nfail + 1)); }

say "D-Sentry build: $DSC"
say "      info build directory: $BUILD"

# ---- 1. generated files ----
for gen in "gen_protocol.py" "sync_config.py"; do
  log="$BUILD/${gen%.py}-check.log"
  if "$PY" "$HERE/$gen" --check > "$log" 2>&1; then
    pass "$gen --check"
  else
    fail "$gen --check (log: $log)"
    tail -5 "$log" | sed 's/^/      /'
  fi
done

# ---- 2. host tests ----
if [ $DO_TESTS = 1 ]; then
  log="$BUILD/tests-run.log"
  targs=()
  [ $VERBOSE = 1 ] && targs=(-v)
  DSENTRY_TESTS_BUILD_DIR="$BUILD/tests" PYTHON="$PY" bash "$DSC/tests/run.sh" ${targs[@]+"${targs[@]}"} > "$log" 2>&1
  rc=$?
  sed 's/^/    | /' "$log"
  if [ $rc -eq 0 ]; then
    pass "tests/run.sh"
  else
    fail "tests/run.sh (log: $log)"
  fi
fi

# ---- 3. arduino-cli ----
# check_used <kind> <pins file> <report>: compares what a build used with libraries.txt.
# report lines: "core <id> <version>" and "lib <name> <version-or-path>" (a path means: read the
# version from that folder's library.properties).
check_used() {
  "$PY" - "$@" <<'PYEOF'
import re, sys
from pathlib import Path
kind, pins_file, report = sys.argv[1], Path(sys.argv[2]), Path(sys.argv[3])
pins = {}
for line in pins_file.read_text().splitlines():
    line = line.strip()
    if not line or line.startswith("#"):
        continue
    what, rest = line.split(" ", 1)
    name, version = rest.rsplit("@", 1)
    pins[(what, name.strip())] = version.strip()
used = {}
for line in report.read_text().splitlines():
    what, rest = line.split(" ", 1)
    name, value = rest.rsplit(" ", 1) if what == "core" else rest.split("\t", 1)
    if value.startswith("/"):
        props = Path(value) / "library.properties"
        m = re.search(r"^version=(.*)$", props.read_text(), re.M) if props.is_file() else None
        value = m.group(1).strip() if m else "unknown (%s)" % value
    used[(what, name.strip())] = value
bad = 0
for key, want in pins.items():
    got = used.get(key)
    if got is None:
        print("%s: %s %s is pinned but was not used" % (kind, key[0], key[1]))
        bad += 1
    elif got != want:
        print("%s: %s %s is %s, libraries.txt pins %s" % (kind, key[0], key[1], got, want))
        bad += 1
    else:
        print("%s: %s %s %s (pinned)" % (kind, key[0], key[1], got))
sys.exit(1 if bad else 0)
PYEOF
}

if [ $DO_ARDUINO = 1 ]; then
  if ! command -v "$ARDUINO_CLI" > /dev/null 2>&1; then
    fail "arduino-cli not found (set ARDUINO_CLI)"
  else
    out="$BUILD/arduino-dsentry"
    json="$BUILD/arduino-compile.json"
    log="$BUILD/arduino-compile.log"
    # --clean: every file is compiled again, so every warning is seen.
    "$ARDUINO_CLI" compile --fqbn "$FQBN" --warnings all --clean --build-path "$out" \
      --format json "$SKETCH" > "$json" 2> "$log"
    rc=$?
    report="$BUILD/arduino-used.txt"
    "$PY" - "$json" "$SKETCH" "$out" "$report" "$log" <<'PYEOF'
import json, re, sys
from pathlib import Path
json_path, sketch, out, report, log = sys.argv[1:6]
try:
    d = json.loads(Path(json_path).read_text())
except ValueError as e:
    print("      arduino-cli: no JSON result (%s); see %s" % (e, log))
    sys.exit(1)
text = (d.get("compiler_out") or "") + "\n" + (d.get("compiler_err") or "")
Path(log).write_text(Path(log).read_text() + text)
ours = [l for l in text.splitlines() if "warning:" in l and (sketch in l or (out + "/sketch") in l)]
for l in ours:
    print("      FAIL arduino-cli: warning in D-Sentry code: " + l)
total = sum(1 for l in text.splitlines() if "warning:" in l)
br = d.get("builder_result") or {}
lines = []
plat = br.get("build_platform") or br.get("board_platform") or {}
if plat:
    lines.append("core %s %s" % (plat.get("id"), plat.get("version")))
for lib in br.get("used_libraries") or []:
    lines.append("lib %s\t%s" % (lib.get("name"), lib.get("version") or lib.get("install_dir")))
Path(report).write_text("\n".join(lines) + "\n")
for s in br.get("executable_sections_size") or []:
    print("      info arduino-cli: %s %d of %d bytes (%.0f%%)" % (
        "flash" if s["name"] == "text" else "RAM  " if s["name"] == "data" else s["name"],
        s["size"], s["max_size"], 100.0 * s["size"] / s["max_size"] if s["max_size"] else 0))
print("      info arduino-cli: %d warning(s) in total, %d in D-Sentry code" % (total, len(ours)))
sys.exit(0 if d.get("success") and not ours else 1)
PYEOF
    prc=$?
    if [ $rc -eq 0 ] && [ $prc -eq 0 ]; then
      pass "arduino-cli compile $FQBN, no warnings in D-Sentry code"
    else
      fail "arduino-cli compile (exit $rc; log: $log)"
      grep -E "error:" "$log" | head -10 | sed 's/^/      /'
    fi
    if [ -s "$report" ] && check_used "arduino-cli" "$PINS" "$report" > "$BUILD/arduino-pins.log" 2>&1; then
      sed 's/^/      info /' "$BUILD/arduino-pins.log"
      pass "arduino-cli used the pinned core and libraries"
    else
      sed 's/^/      /' "$BUILD/arduino-pins.log" 2> /dev/null
      fail "arduino-cli: core/library versions differ from libraries.txt"
    fi
  fi
fi

# ---- 4. PlatformIO ----
if [ $DO_PIO = 1 ]; then
  if ! command -v "$PIO" > /dev/null 2>&1 && [ ! -x "$PIO" ]; then
    fail "PlatformIO not found (set PIO)"
  else
    log="$BUILD/pio-run.log"
    # -v prints the library paths of the dependency graph, which the version check reads.
    (cd "$FW" && PLATFORMIO_BUILD_DIR="$BUILD/pio" "$PIO" run -v) > "$log" 2>&1
    rc=$?
    grep -E '^(RAM|Flash):' "$log" | sed 's/^/      info pio /'
    if [ $rc -eq 0 ]; then
      pass "pio run (firmware-ra4m1, env uno_r4_wifi)"
    else
      fail "pio run (exit $rc; log: $log)"
      grep -E "error:|Error:" "$log" | head -10 | sed 's/^/      /'
    fi
    report="$BUILD/pio-used.txt"
    {
      sed -nE 's/.*framework-arduinorenesas-uno @ ([0-9.]+).*/core arduino:renesas_uno \1/p' "$log" | head -1
      # "|-- LiquidCrystal_I2C (License: ..., Path: /x/y)": the version is read from the folder
      sed -nE 's/^\|-- ([^(@]*[^ (@]) .*Path: ([^)]*)\).*/\1|\2/p' "$log" | while IFS='|' read -r name path; do
        props="$path/library.properties"
        pname="$(sed -nE 's/^name=(.*)$/\1/p' "$props" 2>/dev/null | head -1)"
        printf 'lib %s\t%s\n' "${pname:-$name}" "$path"
      done
    } > "$report"
    if check_used "pio" "$PINS" "$report" > "$BUILD/pio-pins.log" 2>&1; then
      sed 's/^/      info /' "$BUILD/pio-pins.log"
      pass "pio used the pinned framework and libraries"
    else
      sed 's/^/      /' "$BUILD/pio-pins.log"
      fail "pio: framework/library versions differ from libraries.txt"
    fi
  fi
fi

say "Summary: $npass passed, $nfail failed"
[ $nfail -eq 0 ] || exit 1
exit 0

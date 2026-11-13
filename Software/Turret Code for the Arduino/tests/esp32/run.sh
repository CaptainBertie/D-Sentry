#!/bin/bash
# run.sh -- host checks of the D-Sentry additions to the ESP32-S3 bridge fork
# (firmware/UNOR4USBBridge/dsentry_*, cmds_dsentry.h). No board, no ESP32 toolchain:
#   1. test_dsentry_wire  dsentry_wire.cpp (decisions, byte formats) with -Wall -Wextra -Werror;
#   2. test_dsentry_at    dsentry_at.cpp + dsentry_wire.cpp on the real chAT parser.cpp and
#                         server.cpp, replies parsed by the RA4M1's real AsyncAt;
#   3. check_literals --esp32 (the D-Sentry fork files; lit-ok only for BLE/HID protocol facts;
#                         every tunable and size comes from config.h through dsentry_config.h).
# The firmware build itself is tests/esp32/build_firmware.sh (about a minute; needs the
# vendored ESP32 core under firmware/hardware).
#
# Usage: tests/esp32/run.sh [-v]
# Env:   CXX                      compiler (default clang++)
#        PYTHON                   interpreter (default <repo>/.venv/bin/python, else python3)
#        DSENTRY_TESTS_BUILD_DIR  output folder (default D-SentryCode/tests/build); this script
#                                 uses its esp32/ subfolder
# Exit status: 0 when everything passes, 1 otherwise, 2 on a usage error.
#
# SPDX-License-Identifier: MIT
set -u

VERBOSE=0
for arg in "$@"; do
  case "$arg" in
    -v) VERBOSE=1 ;;
    -h|--help) printf 'Usage: %s [-v]\n' "$0"; exit 0 ;;
    *) printf 'run.sh: unknown argument: %s\n' "$arg" >&2; exit 2 ;;
  esac
done

HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
DSC="$(cd "$HERE/../.." && pwd)"
REPO="$(cd "$DSC/.." && pwd)"
FW="$REPO/firmware/UNOR4USBBridge"
SRC="$DSC/firmware-ra4m1/dsentry/src"
CXX="${CXX:-clang++}"
if [ -n "${PYTHON:-}" ]; then
  PY="$PYTHON"
elif [ -x "$REPO/.venv/bin/python" ]; then
  PY="$REPO/.venv/bin/python"
else
  PY="python3"
fi
mkdir -p "${DSENTRY_TESTS_BUILD_DIR:-$DSC/tests/build}/esp32" || exit 1
BUILD="$(cd "${DSENTRY_TESTS_BUILD_DIR:-$DSC/tests/build}/esp32" && pwd)"

CXXFLAGS=(-std=gnu++17 -O1 -g)
OURS=(-Wall -Wextra -Werror)
npass=0
nfail=0
say() { printf '%s\n' "$*"; }
pass() { say "PASS  $1"; npass=$((npass + 1)); }
fail() { say "FAIL  $1"; nfail=$((nfail + 1)); }

compile() {   # compile <label> <args...>
  label="$1"; shift
  if "$CXX" "${CXXFLAGS[@]}" "$@" > "$BUILD/$label.build.log" 2>&1; then
    return 0
  fi
  fail "$label: compile failed (log: $BUILD/$label.build.log)"
  head -20 "$BUILD/$label.build.log" | sed 's/^/      /'
  return 1
}

run_test() {   # run_test <name> <binary>
  name="$1"; bin="$2"
  if [ "$VERBOSE" = 1 ]; then "$bin" -v > "$BUILD/$name.log" 2>&1; else "$bin" > "$BUILD/$name.log" 2>&1; fi
  rc=$?
  if [ "$VERBOSE" = 1 ]; then sed 's/^/      /' "$BUILD/$name.log"; else grep -E '^(info|FAIL|      )' "$BUILD/$name.log" | sed 's/^/      /'; fi
  if [ $rc -eq 0 ]; then pass "$name"; else fail "$name (exit $rc, log: $BUILD/$name.log)"; fi
}

say "D-Sentry ESP32 fork host checks: $FW"
say "      info build directory: $BUILD"

# ---- 1. the pure logic ----
if compile test_dsentry_wire "${OURS[@]}" -I "$DSC/tests/unit" -I "$FW" \
     "$HERE/test_dsentry_wire.cpp" "$FW/dsentry_wire.cpp" -o "$BUILD/test_dsentry_wire"; then
  run_test test_dsentry_wire "$BUILD/test_dsentry_wire"
fi

# ---- 2. the AT handlers on the real chAT server, parsed by the real AsyncAt ----
if compile chat_parser.o -w -c "$FW/parser.cpp" -o "$BUILD/chat_parser.o" &&
   compile chat_server.o -w -c "$FW/server.cpp" -o "$BUILD/chat_server.o" &&
   compile dsentry_at.o "${OURS[@]}" -I "$FW" -c "$FW/dsentry_at.cpp" -o "$BUILD/dsentry_at.o" &&
   compile dsentry_wire.o "${OURS[@]}" -I "$FW" -c "$FW/dsentry_wire.cpp" -o "$BUILD/dsentry_wire.o" &&
   compile AsyncAt.o "${OURS[@]}" -I "$SRC" -I "$DSC/sim/host_include" -c "$SRC/AsyncAt.cpp" -o "$BUILD/AsyncAt.o" &&
   compile ra_side.o "${OURS[@]}" -I "$SRC" -I "$DSC/sim/host_include" -c "$HERE/ra_side.cpp" -o "$BUILD/ra_side.o" &&
   compile test_dsentry_at "${OURS[@]}" -I "$DSC/tests/unit" -I "$FW" "-DREPO_ROOT=\"$REPO\"" \
     "$HERE/test_dsentry_at.cpp" "$BUILD/dsentry_at.o" "$BUILD/dsentry_wire.o" "$BUILD/chat_parser.o" \
     "$BUILD/chat_server.o" "$BUILD/AsyncAt.o" "$BUILD/ra_side.o" -o "$BUILD/test_dsentry_at"; then
  run_test test_dsentry_at "$BUILD/test_dsentry_at"
fi

# ---- 3. no numeric literals in the D-Sentry fork files ----
log="$BUILD/check_literals_esp32.log"
if "$PY" "$DSC/tools/check_literals.py" --esp32 > "$log" 2>&1; then
  say "      info $(tail -1 "$log")"
  pass "check_literals --esp32"
else
  fail "check_literals --esp32 (log: $log)"
  head -20 "$log" | sed 's/^/      /'
fi

say "Summary: $npass passed, $nfail failed"
[ $nfail -eq 0 ] || exit 1
exit 0

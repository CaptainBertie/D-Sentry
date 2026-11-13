#!/bin/bash
# run.sh -- build and run the host tests on this Mac. No hardware, no arduino-cli, no serial
# ports: everything is compiled with clang++ from this repo and the installed Renesas core.
#
# Usage: tests/run.sh [-v]
#   -v  also print every passing check, not just the verdicts, info lines and failures.
# Env:   CXX              compiler (default clang++)
#        WIFIS3_SRC       folder holding the WiFiS3 Modem.cpp to test (default: the installed
#                         arduino:renesas_uno 1.6.0 core, which is what the project targets)
#        TESTS_BUILD_DIR  where objects, binaries and logs go (default: tests/build)
# Exit status: 0 when every test passes (SKIPs allowed), 1 otherwise, 2 on a usage error.
#
# SPDX-License-Identifier: MIT
set -u

usage() {
  printf 'Usage: %s [-v]\n  -v  also print every passing check\n' "$0"
}

VERBOSE=0
for arg in "$@"; do
  case "$arg" in
    -v) VERBOSE=1 ;;
    -h|--help) usage; exit 0 ;;
    *) printf 'run.sh: unknown argument: %s\n' "$arg" >&2; usage >&2; exit 2 ;;
  esac
done

HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ROOT="$(cd "$HERE/.." && pwd)"
CXX="${CXX:-clang++}"
if ! mkdir -p "${TESTS_BUILD_DIR:-$HERE/build}"; then
  printf 'run.sh: cannot create the build directory %s\n' "${TESTS_BUILD_DIR:-$HERE/build}" >&2
  exit 1
fi
BUILD="$(cd "${TESTS_BUILD_DIR:-$HERE/build}" && pwd)"

CXXFLAGS=(-std=gnu++17 -O1 -g)
OUR_WARNINGS=(-Wall -Wextra)
THIRD_PARTY_WARNINGS=(-w)   # not our code; its warnings are not ours to fix
STUBS="$HERE/stubs"
FIRMWARE_SKETCH="$ROOT/firmware/UNOR4USBBridge"
PARSER_SRC="$ROOT/firmware/libraries/XboxControllerNotificationParser/src"
RA_LIB="$ROOT/ra4m1/libraries/XboxControllerR4/src"

npass=0
nfail=0
nskip=0

say() { printf '%s\n' "$*"; }
pass() { say "PASS  $1"; npass=$((npass + 1)); }
fail() { say "FAIL  $1"; nfail=$((nfail + 1)); }
skip() { say "SKIP  $1"; nskip=$((nskip + 1)); }

# The RA4M1 side must be the real Modem.cpp from the core the project targets (1.6.0); the
# tests pin its quirks (RESULT_ERROR == "OK\r\n"), so another version is not substituted.
find_wifis3() {
  if [ -n "${WIFIS3_SRC:-}" ]; then
    printf '%s\n' "$WIFIS3_SRC"
    return
  fi
  for base in "$HOME/Library/Arduino15" "$HOME/.arduino15"; do
    d="$base/packages/arduino/hardware/renesas_uno/1.6.0/libraries/WiFiS3/src"
    if [ -f "$d/Modem.cpp" ]; then
      printf '%s\n' "$d"
      return
    fi
  done
}

# compile <label> <output> <compiler args...>; prints the tail of the log on failure.
compile() {
  label="$1"; out="$2"; shift 2
  log="$BUILD/$label.build.log"
  if "$CXX" "${CXXFLAGS[@]}" "$@" -o "$out" > "$log" 2>&1; then
    return 0
  fi
  fail "$label: compile failed (log: $log)"
  tail -15 "$log" | sed 's/^/      /'
  return 1
}

# run_test <name> <command...>: shows info/FAIL lines (all lines with -v), then the verdict.
run_test() {
  name="$1"; shift
  log="$BUILD/$name.log"
  "$@" > "$log" 2>&1
  rc=$?
  if [ "$VERBOSE" = 1 ]; then
    sed 's/^/      /' "$log"
  else
    grep -E '^(info|FAIL)' "$log" | sed 's/^/      /'
  fi
  if [ $rc -eq 0 ]; then
    pass "$name"
  else
    fail "$name (exit $rc, log: $log)"
  fi
}

say "host tests: repo $ROOT"
say "      info build directory: $BUILD"

if ! command -v "$CXX" > /dev/null 2>&1; then
  fail "compiler '$CXX' not found; install the Xcode command line tools or set CXX"
  say "Summary: $npass passed, $nfail failed, $nskip skipped"
  exit 1
fi

# ---- wire-format header ----
if compile test_protocol "$BUILD/test_protocol" "${OUR_WARNINGS[@]}" "$HERE/test_protocol.cpp"; then
  run_test test_protocol "$BUILD/test_protocol" "$ROOT"
fi

# ---- XboxControllerNotificationParser replay ----
if compile test_parser_replay "$BUILD/test_parser_replay" "${OUR_WARNINGS[@]}" -I "$STUBS" \
     "$HERE/test_parser_replay.cpp" \
     "$PARSER_SRC/XboxControllerNotificationParser.cpp" \
     "$PARSER_SRC/XboxControllerElite2NotificationParser.cpp"; then
  run_test test_parser_replay "$BUILD/test_parser_replay"
fi

# ---- the next three need the real WiFiS3 Modem.cpp ----
WIFIS3="$(find_wifis3)"
if [ -z "$WIFIS3" ] || [ ! -f "$WIFIS3/Modem.cpp" ]; then
  if [ -n "${WIFIS3_SRC:-}" ]; then
    msg="WIFIS3_SRC=$WIFIS3_SRC has no Modem.cpp"
  else
    msg="WiFiS3 Modem.cpp not found under ~/Library/Arduino15 or ~/.arduino15 at packages/arduino/hardware/renesas_uno/1.6.0/libraries/WiFiS3/src. Install the core with: arduino-cli core install arduino:renesas_uno@1.6.0 (or set WIFIS3_SRC)"
  fi
  fail "test_modem_e2e: $msg"
  fail "test_ra_library: $msg"
  fail "test_chat_e2e: $msg"
else
  say "      info RA4M1 Modem.cpp under test: $WIFIS3/Modem.cpp"
  # stubs first: tests/stubs/WiFiS3.h must shadow the real one (see that file)
  RA_FLAGS=(-DARDUINO_UNOWIFIR4 -I "$STUBS" -I "$WIFIS3")
  modem_ok=0
  if compile Modem.o "$BUILD/Modem.o" "${THIRD_PARTY_WARNINGS[@]}" "${RA_FLAGS[@]}" -c "$WIFIS3/Modem.cpp"; then
    modem_ok=1
  fi

  # ---- real Modem.cpp against canned ESP32 replies ----
  if [ $modem_ok = 1 ] &&
     compile test_modem_e2e "$BUILD/test_modem_e2e" "${OUR_WARNINGS[@]}" "${RA_FLAGS[@]}" \
       "$HERE/test_modem_e2e.cpp" "$BUILD/Modem.o"; then
    run_test test_modem_e2e "$BUILD/test_modem_e2e"
  fi

  # ---- the RA4M1 library's own poll() on the real Modem.cpp ----
  if [ ! -f "$RA_LIB/XboxControllerR4.cpp" ]; then
    skip "test_ra_library: $RA_LIB/XboxControllerR4.cpp not present yet"
  elif [ $modem_ok = 1 ]; then
    LIB_FLAGS=(-DARDUINO_UNOR4_WIFI "${RA_FLAGS[@]}" -I "$RA_LIB")
    if compile XboxControllerR4.o "$BUILD/XboxControllerR4.o" "${OUR_WARNINGS[@]}" "${LIB_FLAGS[@]}" \
         -c "$RA_LIB/XboxControllerR4.cpp" &&
       compile ra_parser.o "$BUILD/ra_parser.o" "${THIRD_PARTY_WARNINGS[@]}" -I "$STUBS" \
         -c "$RA_LIB/vendor/XboxControllerNotificationParser.cpp" &&
       compile ra_elite2_parser.o "$BUILD/ra_elite2_parser.o" "${THIRD_PARTY_WARNINGS[@]}" -I "$STUBS" \
         -c "$RA_LIB/vendor/XboxControllerElite2NotificationParser.cpp" &&
       compile test_ra_library "$BUILD/test_ra_library" "${OUR_WARNINGS[@]}" "${LIB_FLAGS[@]}" \
         "$HERE/test_ra_library.cpp" "$BUILD/XboxControllerR4.o" "$BUILD/ra_parser.o" \
         "$BUILD/ra_elite2_parser.o" "$BUILD/Modem.o"; then
      run_test test_ra_library "$BUILD/test_ra_library"
    fi
  fi

  # ---- real chAT server (firmware tree) joined to the real Modem.cpp ----
  if [ $modem_ok = 1 ] &&
     compile chat_parser.o "$BUILD/chat_parser.o" "${THIRD_PARTY_WARNINGS[@]}" -c "$FIRMWARE_SKETCH/parser.cpp" &&
     compile chat_server.o "$BUILD/chat_server.o" "${THIRD_PARTY_WARNINGS[@]}" -c "$FIRMWARE_SKETCH/server.cpp" &&
     compile test_chat_e2e "$BUILD/test_chat_e2e" "${OUR_WARNINGS[@]}" "${RA_FLAGS[@]}" \
       "$HERE/test_chat_e2e.cpp" "$BUILD/Modem.o" "$BUILD/chat_parser.o" "$BUILD/chat_server.o"; then
    run_test test_chat_e2e "$BUILD/test_chat_e2e" "$ROOT"
  fi
fi

# ---- files kept in two places: the protocol header copies and the vendored parser ----
# Run through bash so a lost exec bit (zip download, core.fileMode=false) does not matter.
copies_out="$(bash "$HERE/test_protocol_copies.sh" 2>&1)"
copies_rc=$?
compared=0
while IFS= read -r line; do
  case "$line" in
    SKIP*) skip "test_protocol_copies ${line#SKIP }" ;;
    PASS*) compared=$((compared + 1)); say "      ${line}" ;;
    *) say "      ${line}" ;;
  esac
done <<< "$copies_out"
if [ $copies_rc -ne 0 ]; then
  fail test_protocol_copies
elif [ $compared -gt 0 ]; then
  pass test_protocol_copies
else
  say "      info test_protocol_copies compared nothing: every copy is missing (see the SKIP lines)"
fi

say "Summary: $npass passed, $nfail failed, $nskip skipped"
if [ $nfail -ne 0 ]; then
  exit 1
fi
exit 0

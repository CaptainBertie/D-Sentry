#!/bin/bash
# run.sh -- builds and runs tests/chat/test_controllink_chat: the RA4M1's AT client (AsyncAt +
# ControlLink + every portable module, on the host HAL) against the REAL chAT server compiled
# from the ESP32 fork (firmware/UNOR4USBBridge/parser.cpp and server.cpp, read-only, AGPL-3.0,
# never copied into D-SentryCode). docs/architecture.md 8.2.
#
# Usage: tests/chat/run.sh [-v]
#   -v  print every line of the test's output, not just info / ok / FAIL lines
# Env:   CXX                      compiler (default clang++)
#        DSENTRY_TESTS_BUILD_DIR  objects, binary and log go to <dir>/chat (default
#                                 D-SentryCode/tests/build/chat, which git ignores)
# Output: the PASS / FAIL / SKIP verdict lines tests/run.sh prints. Exit 0 pass (or skip: the
# fork is absent), 1 fail.
#
# SPDX-License-Identifier: MIT
set -u

HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
DSC="$(cd "$HERE/../.." && pwd)"
REPO="$(cd "$DSC/.." && pwd)"
SRC="$DSC/firmware-ra4m1/dsentry/src"
SIM="$DSC/sim"
FORK="$REPO/firmware/UNOR4USBBridge"
CXX="${CXX:-clang++}"
VERBOSE=0
[ "${1:-}" = "-v" ] && VERBOSE=1
BUILD="${DSENTRY_TESTS_BUILD_DIR:-$DSC/tests/build}/chat"
mkdir -p "$BUILD/obj" || exit 1

if [ ! -f "$FORK/parser.cpp" ] || [ ! -f "$FORK/server.cpp" ]; then
  printf 'SKIP  test_controllink_chat: %s has no chAT parser.cpp / server.cpp\n' "$FORK"
  exit 0
fi

CXXFLAGS=(-std=gnu++17 -O2 -g)
OURS=(-Wall -Wextra -Werror)
THIRD=(-w)   # chAT is not our code; its warnings are not ours to fix
INC=(-I "$SRC" -I "$SIM" -I "$SIM/host_include" -I "$DSC/protocol")

fail() { printf 'FAIL  %s\n' "$1"; exit 1; }

"$CXX" "${CXXFLAGS[@]}" "${THIRD[@]}" -c "$FORK/parser.cpp" -o "$BUILD/obj/chat_parser.o" > "$BUILD/chat_parser.log" 2>&1 \
  || fail "test_controllink_chat: chAT parser.cpp did not compile (log: $BUILD/chat_parser.log)"
"$CXX" "${CXXFLAGS[@]}" "${THIRD[@]}" -c "$FORK/server.cpp" -o "$BUILD/obj/chat_server.o" > "$BUILD/chat_server.log" 2>&1 \
  || fail "test_controllink_chat: chAT server.cpp did not compile (log: $BUILD/chat_server.log)"

objects=()
for f in "$SRC"/*.cpp "$SIM/hal_host.cpp"; do
  [ "$(basename "$f")" = "hal_arduino.cpp" ] && continue   # board only
  o="$BUILD/obj/$(basename "${f%.cpp}").o"
  "$CXX" "${CXXFLAGS[@]}" "${OURS[@]}" "${INC[@]}" -c "$f" -o "$o" > "$o.log" 2>&1 \
    || fail "test_controllink_chat: $(basename "$f") did not compile (log: $o.log)"
  objects+=("$o")
done
# The test includes chAT.hpp from the fork; -I "$FORK" comes after the sketch so the sketch's
# xbox_r4_protocol.h symlink, not the fork's byte-identical copy, is the one found.
"$CXX" "${CXXFLAGS[@]}" "${OURS[@]}" -Wno-missing-field-initializers "${INC[@]}" -I "$FORK" \
  "$HERE/test_controllink_chat.cpp" "${objects[@]}" "$BUILD/obj/chat_parser.o" "$BUILD/obj/chat_server.o" \
  -o "$BUILD/test_controllink_chat" > "$BUILD/test_controllink_chat.build.log" 2>&1 \
  || { head -20 "$BUILD/test_controllink_chat.build.log"; fail "test_controllink_chat: compile failed (log: $BUILD/test_controllink_chat.build.log)"; }

log="$BUILD/test_controllink_chat.log"
"$BUILD/test_controllink_chat" "$REPO" > "$log" 2>&1
rc=$?
if [ $VERBOSE = 1 ]; then
  sed 's/^/      /' "$log"
else
  grep -E '^(info|ok|FAIL)' "$log" | sed 's/^/      /'
fi
if [ $rc -eq 0 ]; then
  printf 'PASS  test_controllink_chat (real chAT server)\n'
  exit 0
fi
printf 'FAIL  test_controllink_chat (exit %s, log: %s)\n' "$rc" "$log"
exit 1

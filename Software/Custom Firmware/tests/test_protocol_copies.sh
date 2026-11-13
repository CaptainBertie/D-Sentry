#!/bin/bash
# test_protocol_copies.sh -- checks the files this project keeps in two places:
#   - protocol/xbox_r4_protocol.h, copied into the firmware and the RA4M1 library;
#     docs/protocol.md requires the copies to be byte-identical.
#   - the XboxControllerNotificationParser sources (asukiaaa, MIT), vendored into the RA4M1
#     library from firmware/libraries/XboxControllerNotificationParser/src. test_parser_replay
#     tests the firmware copy and docs/protocol.md says both sides decode with that parser, so
#     the RA4M1 copy must stay the same bytes (THIRD_PARTY.md: copied verbatim).
#
# Prints one line per copy: PASS (identical), FAIL (differs, with the first diff lines) or
# SKIP (a file of the pair does not exist yet). Exit status 1 if any copy differs or the
# canonical protocol header is missing, otherwise 0.
#
# SPDX-License-Identifier: MIT

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
CANON="protocol/xbox_r4_protocol.h"
PARSER="firmware/libraries/XboxControllerNotificationParser/src"
VENDOR="ra4m1/libraries/XboxControllerR4/src/vendor"

if [ ! -f "$ROOT/$CANON" ]; then
  echo "FAIL $CANON: canonical header missing"
  exit 1
fi

status=0

# check_pair <original> <copy>, both relative to the repo root
check_pair() {
  original="$1"
  copy="$2"
  if [ ! -f "$ROOT/$copy" ]; then
    echo "SKIP $copy: not present yet"
  elif [ ! -f "$ROOT/$original" ]; then
    echo "SKIP $copy: $original not present yet"
  elif cmp -s "$ROOT/$original" "$ROOT/$copy"; then
    echo "PASS $copy: identical to $original"
  else
    echo "FAIL $copy: differs from $original"
    diff -u "$ROOT/$original" "$ROOT/$copy" | head -20 | sed 's/^/     /'
    status=1
  fi
}

check_pair "$CANON" firmware/UNOR4USBBridge/xbox_r4_protocol.h
check_pair "$CANON" ra4m1/libraries/XboxControllerR4/src/xbox_r4_protocol.h
for f in XboxControllerNotificationParser.h XboxControllerNotificationParser.cpp \
         XboxControllerElite2NotificationParser.h XboxControllerElite2NotificationParser.cpp; do
  check_pair "$PARSER/$f" "$VENDOR/$f"
done
exit $status

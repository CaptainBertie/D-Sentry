#!/bin/bash
# build_firmware.sh -- compile the ESP32-S3 bridge fork (firmware/UNOR4USBBridge, D-Sentry files
# included) in a scratch copy, with every compiler warning on, and check the app against the
# app0 partition. It never runs firmware/export.sh or build.sh (export.sh deletes
# UNOR4USBBridge/build in place) and never writes inside firmware/: the copy gets symlinks to
# firmware/hardware and firmware/libraries, its own arduino-cli.yaml and its own build path.
#
# Usage: tests/esp32/build_firmware.sh
# Env:   DSENTRY_BUILD_DIR  where the copy and the build go (default D-SentryCode/build, which git
#                           ignores); the work is done in $DSENTRY_BUILD_DIR/esp32fw
# Output: compile log, "app N bytes, app0 M bytes, headroom K bytes", then a warning pass: each
#         D-Sentry-touched translation unit (dsentry_*.cpp, xbox_client.cpp, at_handler.cpp, the
#         .ino) is compiled again with the build's own command and -Wall -Wextra, and every
#         warning located in dsentry_*, cmds_dsentry.h or xbox_client.* is printed.
# Exit status: 0 built, fits app0 and no warning in those files; 1 otherwise.
#
# SPDX-License-Identifier: MIT
set -euo pipefail

HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
DSC="$(cd "$HERE/../.." && pwd)"
REPO="$(cd "$DSC/.." && pwd)"
FW="$REPO/firmware"
OUT="${DSENTRY_BUILD_DIR:-$DSC/build}/esp32fw"
WORK="$OUT/firmware"
BUILD="$OUT/build"
LOG="$OUT/compile.log"

mkdir -p "$WORK" "$BUILD"
ln -sfn "$FW/hardware" "$WORK/hardware"
ln -sfn "$FW/libraries" "$WORK/libraries"
rsync -a --delete --exclude build "$FW/UNOR4USBBridge/" "$WORK/UNOR4USBBridge/"
sed "s#PWD#$WORK#g" "$FW/arduino-cli.yaml.orig" > "$WORK/arduino-cli.yaml"
FQBN="$(grep -o -- '--fqbn=[^ ]*' "$FW/compile.sh" | head -1 | cut -d= -f2-)"
if [ -z "$FQBN" ]; then
  echo "build_firmware: cannot read the FQBN from $FW/compile.sh" >&2
  exit 1
fi

echo "== compile (scratch copy $WORK, build path $BUILD)"
start=$(date +%s)
# Stock flags (as compile.sh: -w). The core's --warnings more/all add -Werror=all, which the
# upstream sources do not survive; the D-Sentry files get their warning pass below instead.
if ! (cd "$WORK" && arduino-cli compile --config-file arduino-cli.yaml --fqbn="$FQBN" \
        --warnings none -v --build-path "$BUILD" UNOR4USBBridge) > "$LOG" 2>&1; then
  echo "FAIL compile (log: $LOG)"
  grep -E "error" "$LOG" | head -20
  exit 1
fi
echo "   $(( $(date +%s) - start )) s; $(grep -E '^Sketch uses' "$LOG")"
echo "   $(grep -E '^Global variables use' "$LOG")"

status=0
CSV="$FW/hardware/esp32-patched/esp32/tools/partitions/unor4wifi.csv"
APP0=$(( $(awk -F, '/^app0/ { gsub(/ /, "", $5); print $5 }' "$CSV") ))
APP_BIN="$BUILD/UNOR4USBBridge.ino.bin"
APP=$(stat -f %z "$APP_BIN" 2>/dev/null || stat -c %s "$APP_BIN")
echo "== size gate: app $APP bytes, app0 $APP0 bytes, headroom $((APP0 - APP)) bytes"
if [ "$APP" -gt "$APP0" ]; then
  echo "FAIL the app does not fit app0"
  status=1
fi

# Warning pass: recompile each D-Sentry-touched translation unit with the exact command the
# build used, -w replaced by -Wall -Wextra, and keep the warnings located in D-Sentry files.
OURS='(dsentry_[a-z_]+\.(h|cpp)|cmds_dsentry\.h|xbox_client\.(h|cpp))'
WARN_DIR="$OUT/warnings"
rm -rf "$WARN_DIR"
mkdir -p "$WARN_DIR"
: > "$WARN_DIR/all.log"
for unit in dsentry_wire.cpp dsentry_at.cpp dsentry_radio.cpp xbox_client.cpp at_handler.cpp UNOR4USBBridge.ino.cpp; do
  # In a fresh build directory the log also holds arduino-cli's library-discovery runs of each
  # file (`-E ... -o /dev/null`, preprocess only); skip them and take the real compile.
  cmd="$(grep -E "xtensa-esp32s3-elf-g\+\+.* -c .*sketch/$unit\"? " "$LOG" | grep -v -- ' -E ' | head -1 || true)"
  if [ -z "$cmd" ]; then
    echo "FAIL warning pass: no compile command for $unit in $LOG"
    status=1
    continue
  fi
  cmd="${cmd/ -w / -Wall -Wextra }"
  # arduino-cli prints the core's resolved paths unquoted; the repo path has a space in it, so
  # point them at the copy's symlink instead (the build directory must not contain a space).
  cmd="${cmd//$FW\/hardware\//$WORK/hardware/}"
  cmd="$(printf '%s' "$cmd" | sed -E "s# -o \"?[^ \"]+\"?# -o \"$WARN_DIR/$unit.o\"#")"
  if ! (cd "$BUILD" && bash -c "$cmd") >> "$WARN_DIR/all.log" 2>&1; then   # paths are relative to it
    echo "FAIL warning pass: $unit did not compile (log: $WARN_DIR/all.log)"
    status=1
  fi
done
warn="$(grep -E "/$OURS:[0-9]+:[0-9]+: warning:" "$WARN_DIR/all.log" | sed -E 's#^.*/##' | sort -u || true)"
if [ -n "$warn" ]; then
  echo "== -Wall -Wextra warnings in D-Sentry files:"
  printf '%s\n' "$warn"
  status=1
else
  echo "== -Wall -Wextra warnings in D-Sentry files: none ($(grep -c ': warning:' "$WARN_DIR/all.log" || true) in upstream headers they include; log $WARN_DIR/all.log)"
fi
exit $status

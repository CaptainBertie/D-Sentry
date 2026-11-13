#!/bin/bash
# build.sh -- build the forked bridge firmware and the flashable images, with the size gate
# that upstream lacks. Runs compile.sh, checks the app fits the app0 partition, then export.sh.
#
# Why the gate: boards.txt lets arduino-cli accept apps up to 1,671,168 bytes, but the app0
# partition (tools/partitions/unor4wifi.csv) is 0x190000 = 1,638,400 bytes, and combine.py
# would silently write the overflow into app1. We check the real limit here.
#
# SPDX-License-Identifier: MIT
set -euo pipefail
cd "$(dirname "$0")"

VENV_BIN="$(cd .. && pwd)/.venv/bin"
if [ -x "$VENV_BIN/python" ]; then
  export PATH="$VENV_BIN:$PATH"   # export.sh calls plain "python" and needs the cryptography package
fi

BUILD_DIR=UNOR4USBBridge/build/esp32-patched.esp32.arduino_unor4wifi_usb_bridge
APP_BIN="$BUILD_DIR/UNOR4USBBridge.ino.bin"
CSV=hardware/esp32-patched/esp32/tools/partitions/unor4wifi.csv

echo "== compile"
./compile.sh

APP0_SIZE=$(awk -F, '/^app0/ { gsub(/ /, "", $5); print $5 }' "$CSV")
APP0_SIZE_DEC=$((APP0_SIZE))
APP_SIZE=$(stat -f %z "$APP_BIN" 2>/dev/null || stat -c %s "$APP_BIN")
echo "== size gate: app $APP_SIZE bytes, app0 partition $APP0_SIZE_DEC bytes ($APP0_SIZE)"
if [ "$APP_SIZE" -gt "$APP0_SIZE_DEC" ]; then
  echo "FAIL: app is $((APP_SIZE - APP0_SIZE_DEC)) bytes too big for app0. Not exporting." >&2
  exit 1
fi
echo "   headroom: $((APP0_SIZE_DEC - APP_SIZE)) bytes"

echo "== export (certificate bundle + combined images)"
./export.sh

echo "== images"
ls -l "$BUILD_DIR"/S3-ALL.bin "$BUILD_DIR"/S3-BOOT-APP.bin "$BUILD_DIR"/S3-APP.bin | awk '{print "   " $5 "  " $9}'
shasum -a 256 "$BUILD_DIR"/S3-BOOT-APP.bin "$BUILD_DIR"/UNOR4USBBridge.ino.bin | sed 's/^/   /'
echo "== done. Flash S3-BOOT-APP.bin at 0x0 (see tools/README.md)."

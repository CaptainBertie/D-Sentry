#!/bin/bash
# build.sh -- builds dsentry_sim (docs/architecture.md 8.4) from two halves that never share an
# include path, because config.h (RA4M1) and dsentry_config.h (the ESP32's synced copy) define
# the same names:
#   RA4M1 half: every portable module (firmware-ra4m1/dsentry/src/*.cpp except the board-only
#               hal_arduino.cpp), the host HAL (sim/hal_host.cpp) and sim/sim_main.cpp;
#               -I src -I sim -I sim/host_include
#   ESP32 half: sim/sim_esp32.cpp and the fork's own dsentry_at.cpp + dsentry_wire.cpp;
#               -I firmware/UNOR4USBBridge -I sim
#   chAT:       the fork's parser.cpp + server.cpp (AGPL-3.0, compiled from the fork, never copied;
#               not our code, so its warnings are not ours: -w)
# Our code is compiled with -Wall -Wextra -Werror. The linked binary contains AGPL-3.0 code.
#
# Usage: sim/build.sh
# Env:   CXX                    compiler (default clang++)
#        DSENTRY_SIM_BUILD_DIR  output folder (default sim/build, which git ignores)
# Output: <build dir>/dsentry_sim. Exit status: 0 built, 1 compile error or the fork is missing.
#
# SPDX-License-Identifier: MIT
set -u

HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
DSC="$(cd "$HERE/.." && pwd)"
REPO="$(cd "$DSC/.." && pwd)"
SRC="$DSC/firmware-ra4m1/dsentry/src"
FORK="$REPO/firmware/UNOR4USBBridge"
CXX="${CXX:-clang++}"
OUT="${DSENTRY_SIM_BUILD_DIR:-$HERE/build}"
OBJ="$OUT/obj"
mkdir -p "$OBJ" || exit 1

for f in parser.cpp server.cpp dsentry_at.cpp dsentry_wire.cpp dsentry_config.h dsentry_protocol.h chAT.hpp; do
  if [ ! -f "$FORK/$f" ]; then
    printf 'sim/build.sh: %s is missing: the ESP32 half needs the fork in %s\n' "$f" "$FORK" >&2
    exit 1
  fi
done

FLAGS=(-std=gnu++17 -O1 -g)
OURS=(-Wall -Wextra -Werror)
RA_INC=(-I "$SRC" -I "$HERE" -I "$HERE/host_include")
ESP_INC=(-I "$FORK" -I "$HERE")

pids=()
labels=()
objects=()
# compile <label> <object> <args...>: in the background; the log is <object>.log
compile() {
  label="$1"; o="$2"; shift 2
  "$CXX" "${FLAGS[@]}" "$@" -c -o "$o" > "$o.log" 2>&1 &
  pids+=($!)
  labels+=("$label")
  objects+=("$o")
}

for f in "$SRC"/*.cpp; do
  [ "$(basename "$f")" = "hal_arduino.cpp" ] && continue
  compile "$(basename "$f")" "$OBJ/ra_$(basename "${f%.cpp}").o" "${OURS[@]}" "${RA_INC[@]}" "$f"
done
compile hal_host.cpp "$OBJ/ra_hal_host.o" "${OURS[@]}" "${RA_INC[@]}" "$HERE/hal_host.cpp"
compile sim_main.cpp "$OBJ/ra_sim_main.o" "${OURS[@]}" "${RA_INC[@]}" "$HERE/sim_main.cpp"
compile sim_esp32.cpp "$OBJ/esp_sim_esp32.o" "${OURS[@]}" "${ESP_INC[@]}" "$HERE/sim_esp32.cpp"
compile dsentry_at.cpp "$OBJ/esp_dsentry_at.o" "${OURS[@]}" "${ESP_INC[@]}" "$FORK/dsentry_at.cpp"
compile dsentry_wire.cpp "$OBJ/esp_dsentry_wire.o" "${OURS[@]}" "${ESP_INC[@]}" "$FORK/dsentry_wire.cpp"
compile "chAT parser.cpp" "$OBJ/chat_parser.o" -w -I "$FORK" "$FORK/parser.cpp"
compile "chAT server.cpp" "$OBJ/chat_server.o" -w -I "$FORK" "$FORK/server.cpp"

ok=1
for i in "${!pids[@]}"; do
  if ! wait "${pids[$i]}"; then
    ok=0
    printf 'sim/build.sh: %s did not compile (log: %s.log)\n' "${labels[$i]}" "${objects[$i]}" >&2
    head -20 "${objects[$i]}.log" >&2
  fi
done
[ $ok = 1 ] || exit 1

"$CXX" "${FLAGS[@]}" "${objects[@]}" -o "$OUT/dsentry_sim" > "$OBJ/link.log" 2>&1 || {
  printf 'sim/build.sh: link failed (log: %s)\n' "$OBJ/link.log" >&2
  head -20 "$OBJ/link.log" >&2
  exit 1
}
printf 'sim/build.sh: built %s\n' "$OUT/dsentry_sim"

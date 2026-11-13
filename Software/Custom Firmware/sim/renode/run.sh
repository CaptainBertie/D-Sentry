#!/bin/bash
# run.sh -- runs an UNO R4 WiFi sketch ELF on Renode's RA4M1 with a scripted ESP32-S3 on the AT link.
#
# Usage: run.sh <sketch.elf> <seconds> <out_dir> [fork|stock|silent] [hook|socket]
#   fork|stock|silent  what the stand-in ESP32-S3 does (see fake_esp32.py); default fork
#   hook    (default) the stand-in runs inside Renode as a UART line hook, in virtual time
#   socket  Renode exposes Serial2 (sci1) on TCP port ${PORT:-3456}; the stand-in runs as a host
#           python3 process connected to it. The same port is where a second emulator's UART
#           (for example QEMU's ESP32-S3 UART1) would plug in instead.
# Env: RENODE (path to the renode binary), FAKE_ESP32_LOG_ALL=1 (log every poll), PORT.
# Writes <out_dir>/serial.txt (the sketch's Serial output), at_tx.txt (bytes the RA4M1 sent on
# the AT link), at_exchanges.txt (one line per command with the reply), renode.log, run.resc.
set -eu
HERE="$(cd "$(dirname "$0")" && pwd)"
RENODE="${RENODE:-$HERE/../Renode.app/Contents/MacOS/renode}"
ELF="$(cd "$(dirname "$1")" && pwd)/$(basename "$1")"
SECS="$2"
OUT="$(mkdir -p "$3" && cd "$3" && pwd)"
export FAKE_ESP32_MODE="${4:-fork}"
LINK="${5:-hook}"
PORT="${PORT:-3456}"
export FAKE_ESP32_LOG="$OUT/at_exchanges.txt"
rm -f "$OUT/serial.txt" "$OUT/at_tx.txt" "$FAKE_ESP32_LOG"

if [ "$LINK" = "socket" ]; then
  ATTACH="emulation CreateServerSocketTerminal $PORT \"esp_link\" false
connector Connect sysbus.sci1 esp_link"
else
  ATTACH="sysbus.sci1 AddLineHook \"AT\" \"import sys; sys.path.append(r'$HERE') if r'$HERE' not in sys.path else None; import fake_esp32; fake_esp32.on_line(machine, uart, line)\""
fi

cat > "$OUT/run.resc" <<RESC
\$elf=@$ELF
\$serial_out=@$OUT/serial.txt
\$at_tx_out=@$OUT/at_tx.txt
\$sci_model=@$HERE/UnoR4_SCI.cs
\$platform=@$HERE/uno_r4_wifi.repl
include @$HERE/uno_r4_wifi.resc
$ATTACH
emulation RunFor "$(printf '00:00:%02d' "$SECS")"
sysbus.cpu PC
quit
RESC

"$RENODE" --disable-gui --console --plain "$OUT/run.resc" > "$OUT/renode.log" 2>&1 < /dev/null &
PID=$!
if [ "$LINK" = "socket" ]; then
  python3 "$HERE/fake_esp32.py" "$PORT" > "$OUT/fake_esp32.out" 2>&1 &
  FAKE=$!
fi
LIMIT=$(( SECS * 3 + 60 ))
for _ in $(seq 1 "$LIMIT"); do kill -0 "$PID" 2>/dev/null || break; sleep 1; done
if kill -0 "$PID" 2>/dev/null; then kill "$PID"; echo "run.sh: Renode did not quit within ${LIMIT}s (see renode.log); killed" >&2; fi
if [ "$LINK" = "socket" ]; then kill "$FAKE" 2>/dev/null || true; fi
echo "serial: $OUT/serial.txt  AT tx: $OUT/at_tx.txt  exchanges: $FAKE_ESP32_LOG  log: $OUT/renode.log"

# UNO R4 WiFi RA4M1 in Renode

Status 2026-10-12: run on this Mac with Renode 1.17.0; evidence (Serial output and AT exchanges)
is in `evidence/`. The Renode download itself is not in the repo.

This folder runs the XboxControllerR4 sketches on Renode's model of the RA4M1, the UNO R4 WiFi's
main chip. The real sketch ELF runs unmodified: Arduino core, FSP, WiFiS3 `Modem` and the library.
A Python stand-in plays the ESP32-S3 side of the AT link (`docs/protocol.md`).

Nothing here touches a board. It needs no account and no token.

## What is simulated and what is not

| Part | Simulated? |
|---|---|
| Cortex-M4 core, flash, SRAM, ICU interrupt routing, AGT (`millis()`), GPT, GPIO | Yes (Renode's RA4M1 models) |
| `Serial` (SCI9) | Yes. The output is written to `serial.txt`. |
| `Serial2` (SCI1, the AT link to the ESP32-S3) | Yes, with the patched model `UnoR4_SCI.cs` |
| The ESP32-S3 | No. `fake_esp32.py` answers the AT commands with a scripted controller. |
| The 12x8 LED matrix | No. The pin-function writes it uses are logged as unhandled, so nothing is visible. |
| ADC, USB, flash writes (EEPROM), WiFi, BLE | No |

## Files

- `uno_r4_wifi.repl` is Renode's `renesas-r7fa4m1a.repl` with these changes:
  - every SCI uses `UnoR4_SCI`;
  - the clocks are the ones the Arduino core sets;
  - a stateful `SCKDIVCR` register is added, because `millis()` ran at half speed without it;
  - an option-setting memory window is added.
- `UnoR4_SCI.cs` is Renode's `RenesasRA6M5_SCI` model (MIT) with three changes:
  - TXI no longer stalls in FIFO mode;
  - RXI fires for bytes below the FIFO trigger;
  - the TXI start delay is one character time instead of 1 ms.

  Renode compiles the file at load time.
- `uno_r4_wifi.resc` loads the platform and the ELF and sets the vector table to 0x4000.
- `fake_esp32.py` is the ESP32-S3 stand-in. It has three modes:
  - `fork` is firmware 0.6.99 with a controller that connects 2 s after `AT+XBOXBEGIN`. The left stick then turns a circle every 4 s, and A is held for half of every second.
  - `stock` answers `ERROR` to every `+XBOX*` command.
  - `silent` sends no answers at all.
- `run.sh` is the runner.

## Run it

1. Install Renode 1.17.0 for macOS arm64. Download
   `https://github.com/renode/renode/releases/download/v1.17.0/renode-1.17.0.osx-arm64-portable.dmg`.
   Open it and copy `Renode.app` somewhere. Set `RENODE` to `.../Renode.app/Contents/MacOS/renode`.
2. Build the sketch:

   ```sh
   A4="$(cd "$(dirname "$0")/../.." && pwd)"   # or the absolute path of this repo
   arduino-cli compile --fqbn arduino:renesas_uno:unor4wifi \
     --libraries "$A4/ra4m1/libraries" --output-dir /tmp/pai \
     "$A4/ra4m1/libraries/XboxControllerR4/examples/PrintAllInputs"
   ```

3. Run it for 8 s of virtual time. Renode paces virtual time to host time, so this takes about 12 s:

   ```sh
   ./run.sh /tmp/pai/PrintAllInputs.ino.elf 8 /tmp/pai-run fork
   cat /tmp/pai-run/serial.txt          # what the Serial Monitor would show
   cat /tmp/pai-run/at_exchanges.txt    # every AT command and the bytes sent back
   ```

   - With `stock` or `silent` as the 4th argument, the sketch prints `lastError() = 1` or `4`.
   - With `socket` as the 5th argument, Serial2 goes out on TCP port 3456 and `fake_esp32.py` runs as a host process. That port is also where a second emulator's UART would connect.

## Evidence from the 2026-10-12 runs (in `evidence/`)

- `run-fork20/serial.txt`: PrintAllInputs on the RA4M1 model against the `fork` stand-in for 20 s
  of virtual time: `begin()` reports firmware 0.6.99 protocol 1, then scanning, found, waiting,
  connected; the left stick turns a circle and A toggles. `at_exchanges-first-6KB.txt`: the first
  commands and replies on the AT link (`AT+SOFTRESETWIFI`, `AT+XBOXVERSION?`, `AT+XBOXBEGIN`, then
  `AT+XBOXSTATE?` every 20.0 ms; 976 polls in the full run, none lost).
- `run-stock/serial.txt`: against a stock-firmware stand-in: `lastError() = 1` after the 200 ms
  `AT+XBOXVERSION?` timeout and a successful `AT+FWVERSION?` probe.
- `run-silent/serial.txt`: against a silent stand-in: `lastError() = 4`.
- `run-stick/serial.txt`: StickToLED, the brightness value following the stick.

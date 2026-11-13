# The forked ESP32-S3 firmware in Espressif's QEMU

Status 2026-10-12: run on this Mac with Espressif QEMU 9.2.2 (`esp-develop-9.2.2-20260417`,
macOS arm64 prebuilt). Evidence from the runs is in `evidence/`. The QEMU download, the flash
images and the full logs are not in the repo (`.gitignore`); `reproduce.sh` recreates them.

## What it proves and what it cannot

QEMU's ESP32-S3 machine has no Bluetooth, no WiFi and no USB. So this run proves:

- the image boots: ROM, second-stage bootloader, partition table, otadata selecting app0, and
  the app at 0x50000 start exactly as on the stock image;
- `setup()` survives the USB and WiFi initialisation calls (the AT server, created at the end of
  `setup()`, answers), with no watchdog reset in 30 s of idle;
- every `AT+XBOX*` reply before `AT+XBOXBEGIN` matches `docs/protocol.md` byte for byte on the
  real compiled firmware, including the 20-byte `AT+XBOXSTATE?` frame with a valid CRC;
- the fork's HCI guard works: `AT+HCIAVAILABLE?` answers `+HCIAVAILABLE: ERROR` where the
  stock firmware panics on a NULL stream buffer (the stock crash is real stock behaviour when
  `AT+HCIBEGIN` was never sent, not something the fork introduced).

It cannot prove anything about Bluetooth. `AT+XBOXBEGIN` reaches ESP-IDF's
`esp_bt_controller_init`, which asserts (`bt.c:1257`, the Bluetooth low-power clock setup)
because the emulator has no Bluetooth clock, and the chip reboots. The stock firmware's
`AT+HCIBEGIN` dies at the same assert, so this is the emulator, not the fork. The
`+XBOXBEGIN: 2` (init failed) path is therefore unexercised too. NVS (flash key-value
storage) also does not work in this emulator, for stock as well: a `Preferences` write cannot
be read back, so `AT+XBOXFORGET`'s effect and the remembered-controller address stay untested.

## Two emulator quirks you have to know

1. **The 4 MB image boot-loops in QEMU.** For a 4 MB (or 8 MB) flash file QEMU picks a GigaDevice
   flash model; our app is built from the `qio_qspi` SDK and `esp_flash_init_main` fails with
   `ESP_ERR_FLASH_NO_RESPONSE`, so `startup.c:326` asserts and the chip reboots every 0.5 s.
   Stock does exactly the same. Padding the file to 16 MB with 0xFF (QEMU then uses an ISSI model)
   makes both boot. The first 4 MB stay byte-identical. This says nothing about the real board,
   whose flash is a real 8 MB GigaDevice part that the stock firmware already runs on.
2. **UART mapping.** The machine wires the first `-serial` to UART0 (the IDF console, which is the
   RA4M1's Serial RX on the real board) and the second to UART1 (the AT link). The firmware's
   own prints go to the USB CDC port, which does not exist here, so UART0 only shows ROM and
   bootloader output and panics.

## Files

- `reproduce.sh`: downloads QEMU, prepares the 16 MB images and an all-zero eFuse file, runs the
  stock control and the fork through `run_sim.py`, and summarises with `analyze.py`.
- `run_sim.py <label> <flash.bin> <efuse.bin> [--cmds stock|fork] [--idle S] [--probe-ms N]`:
  starts QEMU paused with UART0, UART1 and QMP on TCP, sends `AT` every 20 ms until the first
  reply, then every command in turn with 2 s capture windows, an idle period, then the commands
  that crash (`AT+XBOXBEGIN`, or `AT+HCIBEGIN` for the control). Writes `logs/<label>/` with
  `uart0-timed.log`, `uart1.raw`, `at_results.json` (every reply as hex and text with first-byte
  latency), `command.txt` (the exact QEMU command) and `qemu-debug.log` (`-d unimp,guest_errors`).
- `analyze.py`, `mapsym.py`: compare runs; resolve panic addresses with the linker map.
- `evidence/fork-B`, `evidence/stock-B`: the two reference runs (results, UART0 log, command).
  `evidence/fork-HCI`, `evidence/stock-HCI`: the `AT+HCIBEGIN` control that shows the identical
  Bluetooth-clock assert on both images.

## Results table (fork-B)

| Command | Reply | Spec |
|---|---|---|
| `AT` | `OK` | pass |
| `AT+FWVERSION?` | `+FWVERSION: 0.6.99` (stock: `0.6.0`) | pass |
| `AT+XBOXVERSION?` | `+XBOXVERSION: 1,0.6.99` (stock: `ERROR`) | pass |
| `AT+XBOXSTATE?` | `+XBOXSTATE: 20|` FF7F FF7F FF7F FF7F 0000 0000 0000 0000 04 FF 00 51, CRLF, `OK` (41 bytes) | pass: state 4 (off), battery unknown, seq 0, CRC 0x51 verified |
| `AT+XBOXADDR?` | `+XBOXADDR: 00:00:00:00:00:00` | pass |
| `AT+XBOXLOG=1` / `AT+XBOXLOG?` | `+XBOXLOG: 1` / `+XBOXLOG: 1` | pass |
| `AT+HCIAVAILABLE?` | `+HCIAVAILABLE: ERROR` (stock: panic and reboot) | pass |
| `AT+XBOXFORGET` (before BEGIN) | `+XBOXFORGET: 0` | pass (NVS effect untestable here) |
| `AT+XBOXBEGIN` | no reply; assert in `esp_bt_controller_init`, reboot; AT answers again 0.4 s later | emulator limit (stock `AT+HCIBEGIN` identical) |

First-byte latency inside the emulator was 2 to 5 ms per command (not real timing).

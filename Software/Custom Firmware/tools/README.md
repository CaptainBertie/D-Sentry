<!-- SPDX-License-Identifier: MIT -->
# tools: flashing the UNO R4 WiFi's ESP32-S3

`r4flash.py` backs up, writes, checks and restores the USB-bridge firmware on the
ESP32-S3 of an Arduino UNO R4 WiFi. It is glue: esptool 5.5.0 does every flash read and
write, pyserial lists ports (it never opens them), and hidapi sends the same "restart into
download mode" request as Arduino's own `unor4wifi-reboot` tool. Each step prints the exact
command it runs.

It never erases the chip (no `-e`, no `erase-flash`, no `erase-region`). It refuses to write
an image that would break the board's layout. See "What `flash` refuses" below.

Words used here:

- **Download mode**: the ESP32-S3's built-in ROM loader. The S3 shows up as USB
  `303A:1001` ("USB JTAG/serial debug unit") and esptool can read and write its flash.
  A bad flash cannot damage it.
- **Bridge mode**: normal operation. The bridge firmware runs and the board shows up as
  `2341:1002` (a serial port plus a CMSIS-DAP HID interface).
- **NVS** (0x3F0000, 64 KB): the S3's key-value store. It holds Preferences and the BLE
  bonds with the Xbox controller. **SPIFFS** (0x370000, 512 KB): the S3's small file system.
  The normal image (`S3-BOOT-APP.bin`) leaves both untouched.

## Setup

Use the project venv's python for everything. Its console scripts do not work because the
repo path contains a space (DECISIONS.md D03), so `esptool` is always run as `python -m esptool`.

```sh
cd "/Users/jwilson/Downloads/University/Meche Projects/A4"
PY="$PWD/.venv/bin/python"
"$PY" tools/r4flash.py --help
```

The options come after the command name, for example `r4flash.py flash IMAGE --dry-run`.
Every command accepts `--dry-run` (print each external command as `DRY: ...` and run nothing
that touches a device) and `--port PORT`. Without `--port`, the tool picks the one port with
VID:PID `303A:1001`, and refuses if there are none or several. `ports`, `check` and
`enter-download` ignore `--port`.

## Before you start

- **Close Arduino IDE, Arduino Cloud Agent and Arduino App Lab** while flashing, or make sure
  the `serial-discovery` tool they use is version 1.5.1 or newer. Older versions probe every
  serial port in the background and corrupt the transfer. The symptom is "Bad data checksum"
  or "MemData command" errors (forum thread 1425480; fixed in serial-discovery PR #130).
- Plug in only one board: one UNO R4 WiFi, with a data USB-C cable, and no other ESP32
  board. An ESP32-S3 or ESP32-C3 board on its built-in USB port can also show up as
  `303A:1001`, the same as the R4 in download mode.

## Morning procedure

1. **Build.**

   ```sh
   (cd firmware && ./build.sh)
   ```

   `build.sh` compiles, stops if the app is bigger than the 0x190000-byte app0 partition,
   then runs `export.sh` (certificate bundle and `combine.py`) with the venv on `PATH`. It
   writes the images to
   `firmware/UNOR4USBBridge/build/esp32-patched.esp32.arduino_unor4wifi_usb_bridge/`. Do
   not run `export.sh` on its own: it only works from inside `firmware/`, needs a plain
   `python` with the `cryptography` package, and has no size gate.

2. **Check the build (no board needed).**

   ```sh
   "$PY" tools/r4flash.py check
   ```

   Every row must say `PASS` and the last line must say `check: OK`. The check covers the
   tool versions, the image sizes, how the three images relate to each other, the 0xE9
   magic bytes, the app size against the real 0x190000 limit (arduino-cli's limit is 32 KiB
   too high), the certificate bundle at 0xB000, the partition table against
   `unor4wifi.csv`, otadata against `boot/boot_app0.bin`, `esptool image-info` on the
   bootloader and on the app, and the `flash` guards. Use `--build-dir DIR` to check another
   build. `check` never touches a device, so `--dry-run` changes nothing for it; it still
   runs its offline commands (`esptool version`, `image-info`, `gen_esp32part.py`).

3. **Plug the board in**, then:

   ```sh
   "$PY" tools/r4flash.py ports
   ```

   You should see a port labelled `2341:1002 bridge mode` and `HID 2341:1002 ...: present`.

4. **Enter download mode.**

   ```sh
   "$PY" tools/r4flash.py enter-download
   ```

   This sends the bridge's HID feature report (`0xAA`) and waits up to 8 s for a new
   `303A:1001` port. If some other ESP32 device already has a `303A:1001` port, the tool
   still sends the report, ignores that old port, and tells you to pass `--port` with the
   new one to the later commands. It only works while a bridge firmware is running. If it
   fails, use the
   jumper: unplug the board, short **GND** to **DOWNLOAD** on the 6-pin ESP header next to
   the USB-C socket, plug the board in, then remove the short. Run `ports` again and you
   should see `303A:1001 download mode`.

5. **Back up the whole flash.**

   ```sh
   "$PY" tools/r4flash.py backup --out ~/r4-backups
   ```

   This runs `flash-id` to get the chip size (8MB expected) and its MAC address, then
   `read-flash 0 ALL` into a `.partial` file. It checks that the file is exactly the
   detected size (if not, it refuses with exit code 3), then runs `verify-flash` of the file
   against the chip. Only after all three pass does the file get its final name,
   `r4-esp32s3-backup-8MB-mac<12 hex digits>-<UTC time>.bin`, plus a `.sha256` sidecar and a
   second copy in `copy2/`. The MAC in the name ties the backup to this board: `flash` and
   `restore --backup` refuse it on a chip with a different MAC, so keep the name. Look for
   the `backup OK: <path> (<bytes>, 8MB, MAC ...)` line. Copy the backup somewhere off this
   machine as well.

6. **Flash.** Do a dry run first, then the real run:

   ```sh
   IMG="$PWD/firmware/UNOR4USBBridge/build/esp32-patched.esp32.arduino_unor4wifi_usb_bridge/S3-BOOT-APP.bin"
   BK=~/r4-backups/r4-esp32s3-backup-8MB-macXXXXXXXXXXXX-YYYYMMDDTHHMMSSZ.bin
   "$PY" tools/r4flash.py flash "$IMG" --backup "$BK" --dry-run
   "$PY" tools/r4flash.py flash "$IMG" --backup "$BK"
   ```

   `S3-BOOT-APP.bin` at 0x0 is what Arduino's own updater writes. It covers 0x0-0x1E0000
   (bootloader, partition table, otadata, certificates, app0) and keeps NVS and SPIFFS.
   The tool runs these steps:
   - all guards, on one read of the image file
   - `flash-id`; the backup must be the size of this chip's flash, and its MAC must match
   - it writes the checked bytes to a new private temp folder and gives esptool that copy,
     so a rebuild that lands during the run cannot be written by mistake. The `RUN:` lines
     therefore show a temp path; the folder is deleted afterwards.
   - `write-flash --flash-mode keep --flash-freq keep --flash-size keep 0x0 IMAGE` with
     `--after no-reset`; esptool checks the MD5 of what it wrote
   - `verify-flash 0x0 IMAGE` with `--after hard-reset`

   If a step fails, read the advice under the `FAILED:` line:
   - **write-flash failed**: the chip stays in download mode (`--after no-reset`). Do not
     unplug. Run the same command again, or `restore --backup "$BK"`.
   - **verify-flash failed**: esptool resets the chip even when the command fails, so the
     chip has probably left download mode and may be running the image that did not
     verify. Enter download mode again (`enter-download`, or the jumper if the bridge does
     not start), then `restore --backup "$BK"`.
   - **Ctrl-C**: the flash may be half written. Do not unplug; run `ports` and follow the
     printed advice.

7. **Verify.** The `verify-flash` in step 6 is the verify step; look for `flash OK: ...
   written at 0x0 and verified`. Its `--after hard-reset` also clears the S3's
   force-download flag, so the new firmware starts. To check again later, enter download
   mode again and run `"$PY" tools/r4flash.py verify "$IMG"`. That command uses
   `--after no-reset`, so the board stays in download mode until step 8.

8. **Unplug and replug.** If the GND-DOWNLOAD jumper is fitted, remove it first. Pressing
   RST alone keeps the ESP32-S3 in download mode, so it does not replace the replug.
   Afterwards `ports` should show `2341:1002 bridge mode` again.

9. **Blink test in the Arduino IDE.** Select board "Arduino UNO R4 WiFi" and its
   `2341:1002` port. Open File > Examples > 01.Basics > Blink and click Upload. If the
   built-in LED blinks, the bridge firmware is passing sketch uploads through to the
   RA4M1 correctly. After that, try the Serial Monitor, then the Xbox examples.

If the board does not come back in step 8, put it in download mode with the jumper and
restore it (next section).

## Restoring

Both restore paths need download mode. Use `enter-download`, or the jumper if the bridge
firmware no longer starts.

- **From your backup** (puts back exactly what the board had, including NVS and SPIFFS):

  ```sh
  "$PY" tools/r4flash.py restore --backup "$BK"
  ```

  The backup must be 0x400000 or 0x800000 bytes. Its `.sha256` sidecar must match it, its
  size must equal the chip's detected flash size, and the MAC in its file name must match
  the chip's MAC. A backup puts back whatever the board had, so a different partition
  table or a damaged bootloader in it only prints a warning. It is written at 0x0 and
  verified.

- **Arduino's official 0.6.0 firmware**:

  ```sh
  "$PY" tools/r4flash.py restore --official --backup "$BK"
  ```

  The first real run looks for the download-mode port, then downloads
  `unor4wifi-update-macos.zip` (release 0.6.0) into `tools/cache/` and checks it against
  sha256 `7d527455...fe632`. The venv's Python (python.org 3.13) has no CA certificates of
  its own, so the tool loads macOS's `/etc/ssl/cert.pem` for the TLS check; the sha256 pins
  decide what is used. A copy of the zip, checked against that sha256, was put in
  `tools/cache/` on 2026-10-11, so the first run needs no network. It then extracts
  `UNOR4-WIFI-S3-0.6.0.bin` (1,966,080 bytes) and checks it against sha256
  `e4a0e6f9...2fd3c`. The image goes through the same guards as `flash` and is written at
  0x0; NVS and SPIFFS are kept. Here `--backup` names the safety backup, which is checked
  as in `flash`. Without it the command prints a loud warning and continues. With
  `--dry-run` and an empty cache it downloads nothing; it prints what it would download and
  write.

  Arduino's manual method does the same thing: use the jumper, then run
  `./bin/espflash write-bin -b 115200 0x0 firmware/UNOR4-WIFI-S3-0.6.0.bin` from inside
  the extracted zip. That command uses the espflash 2.0.0 bundled in the zip (see the next
  section).

## The espflash `-b` / `-B` trap

The Homebrew `espflash` here is 4.6.0. **From espflash 3.0 onward `-b` means `--before`
(the reset mode) and the baud flag is `-B` / `--baud`.** Arduino's documented
`espflash write-bin -b 115200 0x0 ...` only works with the espflash **2.0.0** bundled in
their updater zip. With 4.6.0 it fails with
`error: invalid value '115200' for '--before <BEFORE>'`. With 4.x, write
`espflash write-bin -S -c esp32s3 -p PORT -B 115200 0x0 IMAGE`. Also, espflash
`read-flash` needs an explicit size (`0x800000`, not `ALL`), and espflash 2.0.0 has no
`read-flash` at all. `r4flash.py` does not use espflash.

## What `flash` refuses

Each refusal prints `REFUSED: <reason>` and exits with code 3.

1. The `--backup` file is missing, its `.sha256` sidecar is missing or does not match, or
   it is not 0x400000/0x800000 bytes. Outside a dry run, it is also refused when its size
   differs from the chip's detected flash size, or when the MAC in its name differs from
   the chip's MAC.
2. An image written at 0x0 does not start with 0xE9 (the bootloader magic byte), or its
   bootloader (0x0-0x8000) fails esptool's own image parser: bad checksum, bad SHA-256,
   no SHA-256, or not for an ESP32-S3. A bad bootloader stops the bridge from starting, so
   only the jumper could bring the board back.
3. The image reaches SPIFFS/NVS (for example `S3-ALL.bin`). This is refused unless you pass
   `--i-know-this-wipes-nvs-and-spiffs`, because it erases the BLE bonds and Preferences.
4. An image the size of `S3-APP.bin` (0x1D2000) at any offset but 0xE000, or one the size
   of `S3-BOOT-APP.bin` (0x1E0000) at any offset but 0x0.
5. The image would extend past 0x400000. A full backup goes through `restore --backup`
   instead.
6. The image covers 0x8000 but its partition table is not the one from `unor4wifi.csv`.
   This stops anyone flashing an image built with a default partition layout. An image that
   covers otadata (0x9000-0xB000) must also carry `boot_app0.bin` there, which selects app0.
7. The app that lands at 0x50000 is missing (no 0xE9), is longer than the 0x190000-byte
   app0 slot, is truncated, has a bad checksum or SHA-256, has no SHA-256, or is not for
   an ESP32-S3. esptool's own image parser does this check.
8. The port is not a `303A:1001` download-mode port (skipped in a dry run).

The only shapes accepted are `S3-BOOT-APP.bin` at 0x0, `S3-APP.bin` at 0xE000, `S3-ALL.bin`
at 0x0 (with the flag), and a bare app (`UNOR4USBBridge.ino.bin`) at 0x50000.
`S3-APP.bin` and a bare app keep the board's own partition table and otadata:
- The tool refuses them unless the backup's partition table at 0x8000 is the
  `unor4wifi.csv` table, because otherwise app0 may not be at 0x50000 on this board.
- Neither resets otadata. If the board last booted app1 (after an ESP over-the-air
  update), it keeps booting app1. The tool prints a note, and a warning when the backup's
  otadata is not `boot_app0.bin`. Prefer `S3-BOOT-APP.bin`.

The expected partition table is pinned in the code as the sha256 of the 3072-byte table
that the core's `gen_esp32part.py` makes from `unor4wifi.csv`:
`b41d2bef...0a3973`. It equals the stock build's `UNOR4USBBridge.ino.partitions.bin` and the
table inside Arduino's 0.6.0 image. `check` rebuilds the table from the csv and compares.

## Commands

| Command | What it does |
|---|---|
| `ports` | Lists serial ports with VID:PID and a label (download mode, bridge mode, other), and whether the bridge HID device is present. Opens nothing. |
| `check [--build-dir DIR]` | The offline build check above. Exit 1 if any row fails. `--dry-run` changes nothing. |
| `enter-download` | Sends the HID report and waits for `303A:1001`. |
| `backup --out DIR` | Full verified backup, sidecar and second copy. |
| `flash IMAGE [--offset 0x0] --backup FILE [--i-know-this-wipes-nvs-and-spiffs]` | Guarded write and verify. |
| `restore --official [--backup FILE] [--cache-dir DIR]` or `restore --backup FILE` | The two restore paths. |
| `verify IMAGE [--offset 0x0]` | `verify-flash` only, `--after no-reset`. |

Exit codes: 0 ok, 1 failed, 2 usage error (argparse), 3 refused by a guard (including a
backup read that is not the chip's full size). An unexpected error prints one `FAILED:`
line first; a bug in the tool also prints its traceback after that line.

The tool does not pass `ESPTOOL_*` environment variables on to esptool. It runs esptool
from `tools/`, so an `esptool.cfg` in your current directory cannot change the command
lines it printed.

## Files it writes

- Backups: in the `--out` directory, each with a `.sha256` sidecar, plus copies in `copy2/`.
- `tools/cache/`: the official zip and the extracted image.
- A private temp folder (`r4flash-*` under `$TMPDIR`) holding the checked copy of the image
  while esptool writes and verifies it. It is deleted when the step ends.
- `tools/logs/r4flash-<UTC time>-<command>.log`: written only by real device runs. It records
  every command, its output and its exit code, and the `REFUSED:` or `FAILED:` reason. A run
  refused before anything was sent leaves no log.

`tools/.gitignore` keeps `cache/` and `logs/` out of git.

## Tests

```sh
R4FLASH_TEST_BUILD_DIR=/path/to/a/known-good/build \
R4FLASH_TEST_OFFICIAL_ZIP=/path/to/unor4wifi-update-macos.zip \
"$PY" tools/tests/test_r4flash.py -v
```

Both variables are optional. The build defaults to the firmware build directory, and the
tests that need it are skipped if it is missing. The zip defaults to
`tools/cache/unor4wifi-update-macos.zip`. Set `R4FLASH_TEST_NETWORK=1` to also run the one
test that really downloads the zip from GitHub (into a temp folder) and checks its sha256.
The tests never touch hardware. Device commands
run with `--dry-run`, or with a `--port` that guard 8 rejects before esptool starts. The
real (non-dry) backup, write and enter-download steps are tested in-process, with esptool,
the port and hidapi replaced by fakes.

## Not verified yet (no board was attached when this was written)

- The HID step. The hidapi 0.15 API (`hid.enumerate`, `device.open_path`,
  `send_feature_report` returning a byte count) was checked against the installed module's
  help, and the report bytes match `firmware/unor4wifi-reboot/main.go`. Nobody has sent the
  report to a real board yet. The board may restart before it acknowledges the report, so
  the tool waits for the `303A:1001` port even when hidapi reports an error. It is also not
  known whether macOS asks for permission first.
- The `/dev/cu.usbmodem*` names in each mode. The tool goes by VID:PID only.
- The exact `Detected flash size` line from this board (8MB is expected for the
  ESP32-S3-MINI-1-N8), and how long a full backup takes.
- The `MAC:` line. Its format (`MAC:` padded to 20 characters, then six lower-case hex
  pairs) comes from esptool 5.5.0's source (`cmds.py`, `read_mac`), not from a board. If no
  `MAC:` line is found, the backup name has no MAC part and the tool says that only the
  flash size ties a backup to the board.

Verified without a board on 2026-10-11: the real download of the 0.6.0 zip through the
tool's TLS context (4,998,365 bytes, sha256 matched). The same download with Python's
default context fails with `CERTIFICATE_VERIFY_FAILED`.

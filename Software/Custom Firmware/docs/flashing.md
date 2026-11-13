<!-- SPDX-License-Identifier: MIT -->
# Flashing D-Sentry, and getting back to stock

> **Rule 1: back up the whole flash before writing anything (step 3).** The backup is your way
> back to exactly what the board has today. `tools/r4flash.py flash` refuses to run without a
> verified backup of this board, but make it a habit anyway, and copy the backup off this Mac.
>
> **Rule 2: if anything goes wrong, go to "Restore" below.** Download mode is in the ESP32's
> read-only ROM, so a bad image cannot damage the board: one of the three restore commands always
> brings it back.

The board has two chips, and each gets its own firmware:

- **RA4M1** (the main microcontroller): the D-Sentry sketch in `../firmware-ra4m1/dsentry`. You
  upload it like any Arduino sketch, over USB-C (`../README.md`). Nothing on this page is needed
  for that.
- **ESP32-S3** (the radio and USB chip): the project's fork of Arduino's USB-bridge firmware with
  the D-Sentry additions (`../firmware-esp32s3/README.md`). It is written with `tools/r4flash.py`
  in the repo root, which wraps esptool, checks every image before writing it and refuses to write
  without a verified backup (`../../tools/README.md` has every detail; this page is the D-Sentry
  route through it).

The bridge firmware is also what passes sketch uploads from USB-C to the RA4M1, so the order
below always ends with "does Blink still upload".

Words used here:

- **Download mode**: the ESP32-S3's built-in ROM loader. The board then shows up as USB
  `303A:1001`, and esptool can read and write the flash.
- **Bridge mode**: normal running, USB `2341:1002`.
- **NVS**: a small key-value store in the ESP32's flash. It holds the Xbox pad's pairing keys and
  the RA4M1 sketches' saved Preferences. **SPIFFS**: a small file system next to it.
  `S3-BOOT-APP.bin`, the image you flash, leaves both alone.

## Before you start

- Close Arduino IDE, Arduino Cloud Agent and Arduino App Lab while flashing (their port
  scanner corrupts transfers; `../../tools/README.md`, "Before you start").
- One board plugged in, with a data USB-C cable.
- **Decline every offer to "update the WiFi / connectivity module firmware"**: the Arduino IDE
  Firmware Updater (Tools menu), Arduino Cloud and App Lab can all write Arduino's stock 0.6.0
  bridge. That removes D-Sentry's radio modes and the Xbox client. Nothing breaks for good (you
  flash the fork again with this page), but the turret then only sees "legacy firmware".

Set these once per terminal (the repo path contains a space, so keep the quotes):

```sh
cd "/Users/jwilson/Downloads/University/Meche Projects/A4"
PY="$PWD/.venv/bin/python"
OUT=firmware/UNOR4USBBridge/build/esp32-patched.esp32.arduino_unor4wifi_usb_bridge
```

## 1. Pick the image

- **The prebuilt image** (no build needed): `artifacts/dsentry-esp32/S3-BOOT-APP.bin`, built from
  commit `96af008` and already checked (`../../artifacts/dsentry-esp32/README.md`). Its WiFi
  credentials are the shipped `CHANGE_ME`, so in WiFi mode it hosts its own access point
  `D-SENTRY` (password `dsentry123`) after a failed 10 s join. That is enough for every bench
  test. Use it as `IMG` below and skip to step 3:

  ```sh
  IMG="$PWD/artifacts/dsentry-esp32/S3-BOOT-APP.bin"
  (cd artifacts/dsentry-esp32 && shasum -a 256 -c SHA256SUMS)   # both lines must say OK
  ```

- **Your own build**, needed when you change anything the ESP32 reads (your WiFi network, its
  password, `LINK_TIMEOUT_MS`, any `WIFI_*`, `ESP_*` or `BLE_*` value; `tuning.md` marks them
  **ESP32**). First set the values in `../config.h`, then
  `"$PY" D-SentryCode/tools/sync_config.py`, then step 2.

## 2. Build and check (own build only)

`firmware/build.sh` deletes and rewrites `$OUT`, which today holds the fork image of 2026-10-12.
That image is also kept, byte-identical, in `artifacts/xbox-0.6.99/` (sha256 `ae7dc0ef...9033`),
so nothing is lost; copy `$OUT` anyway if you have flashed a build of your own since:

```sh
mkdir -p ~/r4-backups && cp -R "$OUT" ~/r4-backups/fork-build-$(date +%Y%m%d)
(cd firmware && ./build.sh)
"$PY" tools/r4flash.py check
IMG="$PWD/$OUT/S3-BOOT-APP.bin"
```

`build.sh` compiles (about 1.5 min), refuses an app larger than the app0 partition and writes
`S3-BOOT-APP.bin` into `$OUT`. Every `check` row must say `PASS` and the last line `check: OK`.
(`D-SentryCode/tests/esp32/run.sh` and `build_firmware.sh` are the D-Sentry host tests and a
scratch compile that leaves `firmware/` alone; they do not make a flashable image.)

## 3. Back up the whole flash (always, before every flash)

```sh
"$PY" tools/r4flash.py ports            # expect "2341:1002 bridge mode"
"$PY" tools/r4flash.py enter-download   # waits for 303A:1001
"$PY" tools/r4flash.py backup --out ~/r4-backups
```

Look for `backup OK: <path> ...`. Keep the file name as it is: it carries the chip's MAC, and
`flash` / `restore` refuse a backup from another board. Copy it off this Mac too.

If `enter-download` fails (the bridge firmware is not running), use the jumper: unplug the
board, short **GND** to **DOWNLOAD** on the 6-pin ESP header next to the USB-C socket, plug
in, remove the short; `ports` should then show `303A:1001 download mode`.

## 4. Flash S3-BOOT-APP

```sh
BK=~/r4-backups/r4-esp32s3-backup-8MB-macXXXXXXXXXXXX-YYYYMMDDTHHMMSSZ.bin   # from step 3
"$PY" tools/r4flash.py flash "$IMG" --backup "$BK" --dry-run
"$PY" tools/r4flash.py flash "$IMG" --backup "$BK"
```

The dry run must print `guards passed: S3-BOOT-APP.bin, 0x1e0000 bytes at 0x0`. Then wait for
`flash OK: ... written at 0x0 and verified`. If a step fails, follow the advice the tool prints
under `FAILED:`; do not unplug while the chip is in download mode after a failed write.

Then **unplug and replug** the board (remove the jumper first if you used it). `ports` must
show `2341:1002 bridge mode` again.

## 5. Regression test: uploads still work

1. Arduino IDE, board "Arduino UNO R4 WiFi", its `2341:1002` port, File > Examples > 01.Basics >
   Blink, Upload. The LED must blink. This proves the bridge still passes sketch uploads to the
   RA4M1 (spec acceptance 18).
2. Serial Monitor at 115200 with a sketch that prints: text arrives.
3. Optional: an `XboxControllerR4` example still pairs (protocol 1 is unchanged).
4. Upload the D-Sentry sketch. In the bench console, `!status` must show `esp=alive`, not
   `esp=legacy` (the dashboard's `espLegacyFirmware` flag clear): the ESP32 answered
   `AT+DSCAPS?`.

## What to expect from then on

- **Every BT <-> WiFi switch restarts the ESP32.** Flipping the A1 toggle makes the RA4M1 ask
  for the other radio; the ESP32 answers "restarting" and reboots about `ESP_RESTART_DELAY_MS`
  (50 ms) later. Its USB device (`2341:1002`) disappears and comes back about a second later, so
  an open Serial Monitor or the PC relay loses the port and must reopen it (the relay retries
  every `RELAY_REOPEN_MS`). The RA4M1 should keep running: the fork no longer pulses its reset
  line LOW at ESP32 boot (`UNOR4USBBridge.ino`), but the line floats for a few hundred ms while
  the ESP32 reboots, so check it once on the bench (`bench-checklist.md` C3).
- A few bytes of the ESP32's boot banner reach the RA4M1's USB input at each restart. The packet
  parser ignores them, but text lines among them reach the bench shell, which answers
  `unknown command: ...` and runs nothing.
- After a **power cycle** the ESP32 starts with no radio (as stock does) and the RA4M1 asks for
  the toggle's mode again; with no radio running it starts in place, without a restart. An
  `AT+RESET` or a crash also comes back radio-less, and the RA4M1 asks again.
- **WiFi mode:** the ESP32 joins `WIFI_SSID`; if that fails for `WIFI_STA_CONNECT_TIMEOUT_MS`
  (10 s) and `WIFI_MODE_AP_FALLBACK` is true, it hosts `WIFI_AP_SSID` (password
  `WIFI_AP_PASSWORD`) at `WIFI_AP_IP`, until its next restart. The relay finds the turret by its
  beacon on `WIFI_DISCOVERY_PORT`, or use `--udp <ip>`. A sketch upload's 1200-baud touch drops
  the station; the ESP32 rejoins at once.
- **Running a non-D-Sentry sketch afterwards:** the ESP32 stays in its D-Sentry radio mode across
  RA4M1 uploads, and ignores `AT+SOFTRESETWIFI` (what WiFiS3's `modem.begin()` sends) while it
  does. Unplug and replug the board once (or have a sketch send `AT+RESET`) and it is the
  stock-behaving fork again. In D-Sentry WiFi mode `AT+XBOXBEGIN` and ArduinoBLE's `AT+HCIBEGIN`
  are refused, so BLE never starts next to WiFi.

## Restore

All three restore paths need download mode (`enter-download`, or the jumper). `BK` is your
backup from step 3.

- **Exactly what the board had** (the whole flash from step 3, NVS and SPIFFS included):

  ```sh
  "$PY" tools/r4flash.py restore --backup "$BK"
  ```

- **The fork without D-Sentry**, as flashed on 2026-10-12 (NVS and SPIFFS kept):

  ```sh
  "$PY" tools/r4flash.py flash artifacts/xbox-0.6.99/S3-BOOT-APP.bin --backup "$BK"
  ```

- **Arduino's official 0.6.0 bridge** (stock; no D-Sentry, no Xbox client):

  ```sh
  "$PY" tools/r4flash.py restore --official --backup "$BK"
  ```

  The official image is pinned by sha256 and cached in `tools/cache/`. This is also what the
  Arduino IDE Firmware Updater would write.

Unplug and replug after any restore, then repeat the Blink test.

## Fallback: an external ESP32 on Serial1 (documented, not built)

If the on-board ESP32-S3 route ever proves unworkable, the spec's de-risk path (spec section 6)
still applies: an original ESP32 dev board (dual-mode Bluetooth, so Bluetooth Classic pads work
too) running the radio code without the USB-bridge parts, wired to the RA4M1's `Serial1` on
D0/D1 (TX to RX, RX to TX, GND to GND, 3.3 V logic) and powered from its own USB or the 5 V pin.
The UNO R4 keeps Arduino's stock bridge, so uploads are never at risk, and on the RA4M1 only the
link's serial object changes (`LINK_SERIAL` in `config.h`). The external board would serve the
same AT commands (`AT+DSCAPS?`, `AT+DSS`, `AT+DST`, `AT+XBOXSTATE?`) on that UART, so
`ControlLink` stays as it is. Not built: the owner chose the on-board fork (architecture.md 0).

## Troubleshooting

| Symptom | Likely cause | What to do |
|---|---|---|
| `ports` shows nothing after flashing | the ESP32 is still in download mode, or the image does not start | remove the jumper, unplug/replug; else restore (above) |
| Blink no longer uploads | bridge broken | restore the backup or the 2026-10-12 fork image, then report |
| `!status` says `esp=legacy` | the ESP32 runs stock or the pre-D-Sentry fork (or an IDE "firmware update" wrote stock) | flash step 4 again |
| `!status` says `esp=silent` | the ESP32 does not answer on the AT link | replug; if it stays, restore |
| WiFi mode never shows a station | `CHANGE_ME` or wrong `WIFI_SSID`/`WIFI_PASSWORD` compiled in, or a 5 GHz-only network | fix config.h, `sync_config.py`, rebuild, reflash; or join the turret's access point |
| Relay sees no beacon | campus WiFi isolates clients | use the turret's own access point or a phone hotspot, or `--udp <ip>` |
| Serial Monitor drops when the toggle flips | expected: each BT <-> WiFi switch re-enumerates USB | reopen it |

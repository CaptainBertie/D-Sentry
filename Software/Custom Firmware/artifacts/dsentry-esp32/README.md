# D-Sentry ESP32-S3 image (flashed to the owner's board 2026-11-03, MAC d8:85:ac:a7:b6:6c)

The project's fork of Arduino's UNO R4 WiFi USB-bridge firmware with the D-Sentry radio modes
(BT or WiFi, `D-SentryCode/firmware-esp32s3/README.md`). Same bridge as before for everything
else: sketch uploads, the Serial Monitor, the Xbox client (`AT+XBOX*`, protocol 1),
`ESP_FW_VERSION` 0.6.99.

| File | Bytes | sha256 | What |
|---|---|---|---|
| `S3-BOOT-APP.bin` | 1,966,080 | `311c14e28fd795403851f0b5b7a2052f739183cfad15691da5e6037be9504e66` | **the image to flash, at 0x0.** Bootloader, partition table, otadata, certificates and the app; it ends at 0x1E0000, so NVS (the pad's pairing, Preferences) and SPIFFS are kept. |
| `UNOR4USBBridge.ino.bin` | 1,315,088 | `89f035be5ae9f01b9bae647427acc8214ae452cc003c0b1f38588a6fcdb41556` | the app alone (for `esptool image-info`); 323,312 bytes free in the 1,638,400-byte app0 slot |

`SHA256SUMS` holds the same two lines: `shasum -a 256 -c SHA256SUMS` checks them.

## Source

- Built and flashed 2026-11-03 by the coordinating session itself (owner's instruction), from a
  frozen copy of `firmware/` taken from the working tree on top of commit `c74a705` (TelemetryPacket
  v2, 110-byte frames; DS_PROTO_VERSION 1). `firmware/build.sh` in that copy: exit 0, app 1,315,088
  bytes. `tools/r4flash.py check`: 29 of 29. `r4flash.py flash ... --backup
  ~/r4-backups/r4-esp32s3-backup-8MB-macd885aca7b66c-20261012T020152Z.bin`: written at 0x0 and
  verified; the board came back in bridge mode by itself, the RA4M1 sketch kept running, the turret
  reported "D-Sentry firmware" (not legacy), and the BLE pad reconnected (its bond is in NVS, kept).
- Settings compiled in (from `D-SentryCode/config.h` via `dsentry_config.h`): **`WIFI_SSID` is empty**,
  so WiFi mode never joins a network and goes straight to its own access point **`D-SENTRY`,
  password `brockpurdy`** (owner, 2026-11-03), at 192.168.4.1. `strings` on the image finds
  `D-SENTRY` and `brockpurdy`, and no `CHANGE_ME` or `dsentry123`.
- How it was built: `firmware/build.sh` (compile, size gate, certificate bundle, `combine.py`) run
  inside the frozen copy, with `hardware/` and `.venv` symlinked to the repo's; log kept in the
  session scratchpad.

## Checks run (no board touched)

- `tools/r4flash.py check --build-dir <the build folder>`: **29 of 29 PASS**, `check: OK`. It
  covers the sizes, the three images against each other, the bootloader and app checksums and
  hashes (`esptool image-info`), the certificate bundle at 0xB000, the partition table against
  `unor4wifi.csv` (sha256 `b41d2bef...0a3973`), otadata against `boot_app0.bin`, the app against
  the real app0 limit, and the `flash` guards.
- `tools/r4flash.py flash artifacts/dsentry-esp32/S3-BOOT-APP.bin --backup <backup> --dry-run` on
  this exact file: `guards passed: S3-BOOT-APP.bin, 0x1e0000 bytes at 0x0`; nothing was written.
- The app contains the D-Sentry replies (`+DSCAPS`, `+DSS`, `+DST`) and `0.6.99`.
- Compile: 1,314,789 bytes of program, 70,972 bytes of RAM. The six files D-Sentry touches
  (`dsentry_wire/at/radio.cpp`, `xbox_client.cpp`, `at_handler.cpp`, the `.ino`) compiled again
  with the build's own commands plus `-Wall -Wextra`: 32 warnings, all in upstream headers, none
  in D-Sentry files.

## Flash it

**Back up the whole flash first.** `r4flash.py` refuses to write without a verified backup of
this exact board, and the backup is your way back. The full procedure, with the restore paths, is
`D-SentryCode/docs/flashing.md`; in short:

```sh
cd "/Users/jwilson/Downloads/University/Meche Projects/A4"
PY="$PWD/.venv/bin/python"
"$PY" tools/r4flash.py ports                      # expect 2341:1002 bridge mode
"$PY" tools/r4flash.py enter-download             # waits for 303A:1001
"$PY" tools/r4flash.py backup --out ~/r4-backups  # "backup OK: <path> ..."
BK=~/r4-backups/r4-esp32s3-backup-8MB-mac<your MAC>-<time>.bin
"$PY" tools/r4flash.py flash artifacts/dsentry-esp32/S3-BOOT-APP.bin --backup "$BK" --dry-run
"$PY" tools/r4flash.py flash artifacts/dsentry-esp32/S3-BOOT-APP.bin --backup "$BK"
```

Then unplug and replug, check `ports` shows `2341:1002 bridge mode`, and upload Blink: it must
still blink. To go back: `"$PY" tools/r4flash.py restore --backup "$BK"` (exactly what the board
had), or flash `artifacts/xbox-0.6.99/S3-BOOT-APP.bin` (the fork without D-Sentry, as flashed on
2026-10-12), or `restore --official --backup "$BK"` (Arduino's stock 0.6.0).

## Licence

This image is a combined work under the **AGPL-3.0**, because it links the bridge's AT server
chAT (AGPL-3.0-or-later). The D-Sentry and Xbox glue files are MIT, NimBLE-Arduino is
Apache-2.0, and the other bridge parts are GPL/LGPL/BSD/MPL (`THIRD_PARTY.md`). For use on your
own board there is no obligation. If you give this file to anyone else, the AGPL requires you to
offer them the complete corresponding source: commit `96af008` of this repo, with its
submodules (`DECISIONS.md` D04).

# Decisions

Each entry: what was decided, why, and what else was considered. Newest at the bottom.
Written during an unattended overnight build on 2026-10-11; nobody was available to ask.

## D01 Repo layout: one repo, bridge firmware vendored as a pristine import, its submodules re-declared

**What.** This directory is one git repo. `firmware/` is a verbatim copy of
`arduino/uno-r4-wifi-usb-bridge` at commit 94d5bb2 (the latest `main`, tag 0.6.0),
committed unmodified and tagged `vendor/uno-r4-wifi-usb-bridge`. The bridge repo's
seven git submodules are declared as submodules of this repo at the same relative paths
and the same pinned commits. Our changes to upstream files are ordinary commits on top;
`git diff vendor/uno-r4-wifi-usb-bridge..HEAD -- firmware` shows every one of them.

**Why.** The handoff asks for a fork with changes kept as clearly marked patches or
fork commits, buildable from a clean checkout. A nested fork repo cannot be pinned by
the parent without pushing it somewhere public, and creating a GitHub fork under the
developer's account is an outward-facing act I did not want to take unattended.

**Alternatives.** (a) `firmware/` as a submodule of upstream plus a patch directory
applied at build time: upstream files stay byte-identical, but it is harder to read,
develop and debug, and the recursive submodule dance makes the clean-checkout build
fragile. (b) A GitHub fork pushed to the developer's account: rejected as above.

## D02 compile.sh: quote `$PWD`

**What.** One-line change to `firmware/compile.sh`: `sed "s#PWD#$PWD#g"` instead of
`sed s#PWD#$PWD#g`.

**Why.** This checkout lives under `.../Meche Projects/A4`, and the space splits the
unquoted substitute expression, so `arduino-cli` is never told where the platform is
("Platform 'esp32-patched:esp32' not found"). The fix is correct for upstream too.

**Alternatives.** Build through a space-free symlink. Kept in reserve if the ESP32
toolchain itself chokes on the space further in.

## D03 Python tools: call `python -m esptool`, never the venv's console scripts

**What.** The project venv is `.venv/`. Scripts invoke `"$PY" -m esptool ...`.

**Why.** The venv's console-script shebangs embed the venv path, and a path with a space
makes a shebang unusable ("bad interpreter: .../Meche: no such file or directory").
`python -m esptool` sidesteps it.

## D04 License of the combined firmware: AGPL-3.0, because of chAT

**What.** The upstream bridge repo has no top-level LICENSE file. Its sources carry
these notices (see THIRD_PARTY.md for the full table):

- `chAT.hpp`, `parser.cpp`, `server.cpp` (SudoMaker chAT, the AT command server): **AGPL-3.0-or-later**
- `OTA.cpp/.h`: GPL-3.0; `Arduino_DebugUtils`, `Arduino_ESP32_OTA`: GPL (Arduino SA, with a commercial-license offer)
- `BossaUnoR4WiFi.*`, `ArduinoBLE`, `arduino-esp32`: LGPL-2.1
- `freedap.*`, `dap_config.h`, BOSSA: BSD-3-Clause; `SSE.*`: MPL-2.0

A binary that links all of these is governed by the strongest copyleft present, the
AGPL-3.0. The code we add is MIT (asukiaaa libraries), Apache-2.0 (NimBLE-Arduino) and
our own glue; all three are one-way compatible with AGPL-3.0. Our glue files carry an
MIT header with a note that the combined work is AGPL-3.0.

**Flag for the developer.** If this firmware binary is ever distributed (even to one
other person), the AGPL requires offering the complete corresponding source. For
personal use on your own board there is no obligation. The RA4M1 library is a separate
program (it does not include chAT) and is released under MIT.

**Why MIT for the glue.** It is the most reusable choice and stays compatible with every
licence in the mix. Choosing AGPL for the glue would add nothing (the combined work is
AGPL anyway) and would stop the RA4M1 library from being reused permissively.

## D05 Stack: NimBLE-Arduino 2.5.1 on the ESP32-S3, not ArduinoBLE on the RA4M1, not Bluepad32

**What.** The Xbox client runs on the ESP32-S3 with h2zero/NimBLE-Arduino 2.5.1 (commit
5ae2c43f; the tag lives on branch `release/2.5`, not `master`) and asukiaaa's
XboxSeriesXControllerESP32 1.1.3 with its two helper libraries.

**Why.** Recon (docs/recon.md) closed the alternatives. Stock ArduinoBLE on the RA4M1 only
pairs as the responding peripheral: its pairing-response handler is empty and it never issues
LE Start Encryption, so it cannot bond with the controller, and HID-over-GATT requires an
encrypted bonded link. Writing a central-side pairing engine is not glue. Bluepad32 needs
BTstack and its own board package, which conflicts with the vendored 2.0.9 core. NimBLE
2.5.1 was the newest release that both compiles on the core's ESP-IDF 4.4.4 and offers the
2.x API the Xbox library needs: 2.3.0/2.3.1 fail on IDF 4.4, master (3.0.0-dev) removed a
macro the library uses, 1.x has the old callback API. A scratch link test with the exact
build FQBN passed before anything was vendored.

## D06 Command set: six AT commands, and no known command ever answers ERROR

**What.** `+XBOXVERSION?`, `+XBOXBEGIN`, `+XBOXSTATE?`, `+XBOXADDR?`, `+XBOXFORGET`,
`+XBOXLOG=`. All answer `+NAME: <payload>\r\nOK\r\n`; failures are result codes in the payload.
State is a fixed 20-byte binary payload in the `len|bytes` form that `+PREFGET` already uses.

**Why.** The RA4M1's WiFiS3 `Modem.cpp` (core 1.6.0) defines `RESULT_ERROR` as `"OK\r\n"`, so an
`ERROR` reply is only noticed after the full timeout (10 s by default), and a reply with no
prompt also times out. A fixed length lets the RA4M1 reject stale or truncated replies by
size; the CRC-8 catches corruption; the seq byte tells a new report from a repeated poll.
Host harnesses that compile the real chAT and Modem sources confirmed the byte format.

**Alternatives.** The fallback "framed binary stream" design from the handoff was not
needed: the AT channel carries 20 bytes at 50 Hz using about 25% of the line.

## D07 BLE controller ownership: Xbox mode and the stock HCI bridge are mutually exclusive, sticky until reset

**What.** `AT+XBOXBEGIN` answers code 1 if `AT+HCIBEGIN` has ever succeeded since boot;
`AT+HCIBEGIN` answers `ERROR` once the Xbox client has started. Nothing tries to hand the
controller over at runtime. `AT+RESET` (an ESP restart) switches sides.

**Why.** Both stacks register the single VHCI host callback and both call
`esp_bt_controller_init`. A runtime handover looks possible in source (both deinit paths
exist) but cannot be tested without hardware tonight, and a wrong guess would brick BLE
until reboot anyway. Sticky modes are the safe default; stock behaviour is untouched when
Xbox mode is never requested.

## D08 The client task runs on core 1 at priority 1 and starts only on AT+XBOXBEGIN

**What.** `onLoop()` runs in a dedicated FreeRTOS task (8 KB stack, core 1, priority 1,
`vTaskDelay(10 ms)` each iteration). Nothing BLE-related runs at boot.

**Why.** The library's `onLoop()` blocks for seconds while connecting; in `loop()` it would
stall the USB passthrough, in the AT task it would stall every reply. Core 1 matches the
upstream example; the task always yields so neither the USB loop nor the idle task starves.
Starting only on request means a board that never sends `AT+XBOXBEGIN` runs exactly the stock
code paths (the review in D14 moved the one boot-time change, the log sink, into BEGIN), which
is the top regression requirement (sketch upload must keep working).

## D09 Remembered controller, kept bonds, and FORGET

**What.** The first controller that connects is remembered (full `ble_addr_t` in the ESP32's
NVS, Preferences namespace `xboxr4`). From then on the scanner accepts only that controller
or any controller in pairing mode. The vendored library's "delete the bond after a failed
connect" is switched off (patch 3). `AT+XBOXFORGET` deletes all bonds and the remembered
address.

**Why.** Rooms have more than one Xbox controller; without a remembered address the library
connects to the first one it sees. Deleting the bond on every transient failure would force
a re-pair after a single radio hiccup; NimBLE already replaces a genuinely stale bond on its
own (PINKEY_MISSING handling). The bond table itself is not a reliable record, so the address
lives in our own namespace. All of this is untested on hardware tonight.

## D10 Logging: esp_log goes to USB; the client is silent unless AT+XBOXLOG=1

**What.** `AT+XBOXBEGIN` installs an `esp_log` sink that writes to the USB CDC port without
ever blocking (lines are dropped if the port is full). The Xbox client prints nothing unless
`AT+XBOXLOG=1`, and then only short `[xbox] ...` lines on USB; the flag is cleared by the
1200/2400-baud upload touch so a sketch upload can never be disturbed.

**Why.** Recon found that the stock `stdout` redirect only rebinds `loopTask`; any other
task's `printf`/`ESP_LOGE` would go to UART0, which is the RA4M1's Serial RX and the upload
path. Routing `esp_log` to USB matches what stock already does for `stdout` and ROM printf.
Keeping the client silent by default means nothing new can disturb a sketch upload.

## D11 No build_opt.h; NimBLE at its defaults

**What.** NimBLE-Arduino is compiled with its stock `nimconfig.h` (3 connections, all roles).

**Why.** Disabling the peripheral and broadcaster roles would save about 14 KB, but there is
339 KB of headroom and every deviation from the configuration the Xbox library is normally
used with is an untestable risk tonight. `CONFIG_BT_NIMBLE_MAX_CONNECTIONS=1` in particular
would break the library's "create a new client" path after a controller change.

## D12 Firmware version 0.6.100

**What.** `ESP_FW_VERSION "0.6.99"`, `FIRMWARE_PATCH 99`.

**Why.** The USB HID GetFeature report and `WiFi.firmwareVersion()` on the RA4M1 then show
which firmware is running without entering download mode, and the number is unlikely to collide
with an upstream 0.6.x release. It fits the RA4M1's 12-byte version buffer. 99 rather than 100:
the WiFiS3 examples compare versions with `strcmp`, where "0.6.100" would sort below a future
"0.6.2" and print an "upgrade your firmware" warning; and `FWVERSION_U32` reads the patch
through a signed char, so it must stay below 128.

## D13 Xbox library vendored as a plain copy with three marked patches

**What.** `firmware/libraries/XboxSeriesXControllerESP32_asukiaaa/` is a copy of 1.1.3
(commit 0023432), not a submodule. Patches: a null check before `disconnect()`, a reset of
the dangling pointer in `clearTargetAddress()`, and a report counter plus a bond-deletion
switch. Listed in its `UPSTREAM.md`.

**Why.** A submodule cannot carry local patches without a public fork. The library is one
header, so a copy is cheap to diff (`git diff b1e79fb -- firmware/libraries/XboxSeries...`).
The two bug fixes stop a crash and a use-after-free that would take the USB bridge down; the
counter is needed for the seq byte. The other libraries are unmodified submodules.

## D14 What the adversarial firmware review changed

Four reviewers (concurrency, wire protocol, NimBLE semantics, stock-mode regression) and six
verifiers went over the first version of the glue. Confirmed and fixed:

- **FORGET raced the client task.** `AT+XBOXFORGET` cleared the remembered address in the AT
  task while the client task, still seeing the link as up, wrote it straight back, and the
  two tasks touched the scanner's target pointer without a lock. Now the AT handler only
  clears NVS and sets a request flag; the client task does the scan stop, bond deletion and
  target change between `onLoop()` calls, and remembering is suppressed until the link has
  dropped. Everything that touches the library's target, bonds or clients runs in one task.
- **The state snapshot could pair an old report with a new seq.** The NimBLE host task
  (same core, higher priority) can deliver a report between the report copy and the counter
  read; the RA4M1 would then skip the correct report because seq had not changed. The
  snapshot now re-reads until the counter is unchanged. The RA4M1 library also applies every
  report while connected and uses seq only for "is this new".
- **Only `+HCIBEGIN` was guarded.** An ArduinoBLE sketch that ignores `BLE.begin()`'s result
  and polls would have hit NULL stream buffers in the other five HCI handlers and rebooted
  the ESP32. All six now refuse (prompt + `ERROR`, which the RA4M1 parser sees at once) until
  a `+HCIBEGIN` has succeeded on this boot.
- **The pairing-mode filter accepted any Swift Pair device.** The manufacturer data the
  library keys on is Microsoft's generic "pairing over LE" beacon; a keyboard or mouse in
  pairing mode would have been bonded and remembered. Patch 4 adds the gamepad appearance and
  HID service checks to that branch.
- **Policy: pairing mode only, until a controller is remembered.** Without a target the
  library also accepted a controller advertising its normal reconnect data, i.e. a
  neighbour's controller looking for its own console, and a forgotten controller came
  straight back. The scanner now carries an impossible target address until the first
  controller connects (and again after FORGET), so only pairing-mode adverts match.
- **Smaller fixes.** `NimBLEDevice::init()` is called by the glue first, because
  `Core::begin()` ignores a failed init and would assert; a failed task creation no longer
  reports success forever; the log sink moved from boot into BEGIN and became non-blocking;
  `AT+XBOXLOG` is cleared by the upload touch; the stock AT loop now yields to the idle task
  at 1200/2400 baud (an upstream watchdog risk during long upload windows); disconnected
  NimBLE clients are freed so three controller changes do not exhaust the three slots;
  the battery level read during discovery is reported until the first notification; a
  reconnect sends a neutral report until the first HID report of the new link; `AT+XBOXADDR?`
  works before BEGIN; FORGET before BEGIN erases NimBLE's bond store directly.

A second review round (two lenses) found no blocker or major and five small holes, all
closed: the client task now re-clears NVS, drops a pending scan match (`advDevice`) and
re-asserts the remember block inside FORGET, so it has the last word; the library's
`pConnectedClient` is nulled before a dead client is freed; clients are freed only after
Scanning has been seen on two consecutive iterations and with the scan stopped; the battery
cache lookup never touches the radio and runs once per link; the saved-peer probe opens NVS
read-only. FORGET latency is documented as "up to about two minutes while a connection attempt
is failing" (four 30 s connect timeouts in the library).

Not changed, by choice: the library still bonds with "Just Works" (no MITM), which is what the
controller supports; the 50 Hz poll rate; the sticky HCI/Xbox exclusion.

## D15 Flash tooling: esptool only, S3-BOOT-APP by default, refuse without a verified backup

**What.** `tools/r4flash.py` drives esptool 5.5.0 (`python -m esptool` from the venv) at
115200 baud with `--flash-size keep`. `flash` refuses unless it is given a full backup whose
sha256 sidecar matches, whose size equals the chip's detected flash size, and whose file name
carries the same chip MAC that esptool reports; the image must be one of the known shapes
(S3-BOOT-APP at 0x0, S3-APP at 0xE000, bare app at 0x50000, or S3-ALL at 0x0 with an explicit
wipe flag), carry a valid bootloader and app (esptool `image-info` checksum and hash), the
project's partition table and `boot_app0` otadata, and fit app0. The checked bytes are staged
to a private temp copy that esptool writes, so the file cannot change between check and write.
`enter-download` replays Arduino's own `unor4wifi-reboot` (a 0xAA HID feature report), with
the GND+DOWNLOAD jumper as the documented fallback. Backups are read with `read-flash 0 ALL`
(detected size, 8 MB on this module) and verified against the chip before they count.

**Why.** The handoff's rules: never flash without a verified backup, read the real flash size
first, flash at Arduino's 115200. esptool rather than espflash because it verifies MD5 on
write, has `verify-flash`, and its ESP32-S3 `hard-reset` clears the force-download bit; and
because espflash 4.x silently changed `-b` from baud to `--before`, which is exactly the kind
of trap a sleepy morning does not need. S3-BOOT-APP is what Arduino's updater writes; it keeps
NVS (bonds, RA4M1 Preferences, WiFi) and SPIFFS. The venv's python.org Python has no CA bundle,
so the official-image download uses the system `/etc/ssl/cert.pem`.

## D16 RA4M1 library behaviour beyond the protocol spec

**What.** `XboxControllerR4` uses 1000 ms for `BEGIN`/`FORGET` (they may start NimBLE or
write flash), 200 ms for the other text commands and 50 ms for `STATE`. `begin()` tells a
board whose ESP32 does not answer at all (error 4, "bridge silent") apart from a board on stock
firmware (error 1) by probing the stock `AT+FWVERSION?`. A connected-to-disconnected change
loads a neutral report and flags it as a new report; a failed poll changes nothing but counts
in `pollErrors()`, and `lastPollOkMillis()` lets a motor sketch stop on a dead link. The header
refuses to compile for anything but the UNO R4 WiFi.

**Why.** Each came out of the implement/review/fix cycle with evidence: the reviewer measured a
simulated 900 ms BEGIN passing and 1100 ms failing; a just-flashed board would otherwise be
reported as "stock firmware"; the README's quick-start pattern (act on new reports) would have
kept a motor running after a disconnect. A neutral report after N failed polls was considered
and left to the sketch, because it would make `isConnected()` disagree with the firmware.

## D17 Simulation: QEMU for the ESP32-S3, Renode for the RA4M1, no Wokwi

**What.** `sim/qemu/` boots the real forked image in Espressif's QEMU and drives the AT link;
`sim/renode/` runs the real RA4M1 sketch ELF in Renode 1.17.0 with a Python stand-in for the
ESP32-S3. Both are committed with their evidence; the emulator downloads are not.

**Why.** The developer asked for a simulated flash. No emulator covers the whole board. QEMU
has no Bluetooth, WiFi or USB, so it can only prove boot, layout and the AT handlers; Renode
has an RA4M1 model (Renesas R7FA4M1A) that runs the unmodified Arduino core once its SCI UART
model gets three small fixes (FIFO TXI/RXI behaviour and a stateful SCKDIVCR clock divider). Wokwi
has no UNO R4 or RA4M1 board at all, so a custom-chip stand-in there was impossible.

**What was learned.** The 4 MB image boot-loops in QEMU for stock and fork alike (QEMU's
GigaDevice flash model rejects the QIO init of the `qio_qspi` SDK build; padding the file to
16 MB selects a different model and both boot). NVS does not persist in that emulator. The
`+XBOXBEGIN: 2` path is unreachable there because IDF asserts inside `esp_bt_controller_init`
before returning; on real hardware the same code path is what stock `AT+HCIBEGIN` runs every
day. Everything the RA4M1 library does was exercised end to end on an emulated Cortex-M4,
including the two `begin()` failure modes. Upstreaming the Renode model fixes would be an
outward-facing act and was not done.

## D18 First hardware session (2026-10-12): what the board taught us

**Facts observed.** The board came from the factory on stock 0.6.0 with the expected partition
table; the backup read 8 MB (GigaDevice flash) and verified. `enter-download` over the HID
report worked without a jumper; `flash` wrote and verified `S3-BOOT-APP.bin` and esptool's
hard reset brought the board straight back in bridge mode (no replug needed). Blink uploaded
through the new bridge first time. `AT+XBOXBEGIN` answered 0 and NimBLE scanned; the
`AT+XBOXSCAN?` diagnostic listed about twenty Low Energy devices around the bench every 5 s.

**Controllers.** A model 1697 pad (2015, firmware 2.3.2385.0) has no Bluetooth radio at all
and never appeared. A model 1708 pad on its old firmware appeared on the Mac (Bluetooth
Classic) but never in the board's Low Energy scan; after the owner updated it to 5.23.6.0 it
advertised: public address, appearance 964, manufacturer data `0600030080`, **no service UUID
in the advertising packet**.

**Change forced by that.** Patch 4 of the vendored Xbox header, and upstream's own default
branch, required the HID UUID 0x1812 in the advert. The 1708 keeps its service list for the
scan response, which the library's passive scan never requests, so the filter rejected the pad
while the scanner was plainly seeing it. Patch 4 now checks the UUID only when the advert
carries one; appearance 964 plus Microsoft's pairing beacon remains mandatory, which still
excludes Swift Pair keyboards and mice. Active scanning was the alternative; it was not chosen
because it changes radio timing for every device around and the relaxed check is sufficient.

**Also added.** `AT+XBOXSCAN?` (ring of the last eight adverts) and `scanReport()` in the
RA4M1 library: without them this would have been guesswork. Worth keeping as the first thing
to run when a pad does not pair.

## D19 D-Sentry radio modes in the fork: BT or WiFi per boot, switched by a stored restart

**What.** The fork gains the D-Sentry turret's two radio modes (D-SentryCode/docs/architecture.md
4.2 and 4.4; file map in D-SentryCode/firmware-esp32s3/README.md). Three new AT commands:
`AT+DSCAPS?` (capability probe, `1,15`), `AT+DSS=<wantMode>,<rumbleSeq>,<strong>,<weak>,<rumbleMs>,<state>`
(status in, `<espMode>,<bootId>,<wifiStatus>` out) and `AT+DST=<hex>` (telemetry to the PC relay).
BT mode is the existing Xbox client started exactly as `AT+XBOXBEGIN` does, plus rumble: the AT
task leaves the newest 8-byte output report in a latest-wins slot and the client task writes it
without response to the pad's 0x2A4D characteristic right after `onLoop()`. WiFi mode is a new
FreeRTOS task (core 0, priority 1) that joins `WIFI_SSID` (AP fallback at `WIFI_AP_IP`), takes
ControlPackets from the relay on UDP, answers `AT+XBOXSTATE?` from the latest one (state 0 only
while it says connected and is younger than `LINK_TIMEOUT_MS`, else 3), sends StatusPackets and the
AT+DST bytes back to the sender of the last valid packet and broadcasts a BeaconPacket. Also:
`AT+SOFTRESETWIFI` is a no-op (same reply bytes) while a D-Sentry mode runs; `AT+XBOXBEGIN` and
`AT+HCIBEGIN` are refused while WiFi mode runs; `setup()` sets GPIO_BOOT/GPIO_RST HIGH before
enabling them as outputs. `XBOX_R4_PROTOCOL_VERSION` (1), `cmds_xbox.h` and `ESP_FW_VERSION`
are unchanged.

**Why a restart and not a runtime handover.** The owner's rule is that BLE and WiFi never run
together. `NimBLEDevice::deinit()` at run time has a double free (one shared callbacks object
deleted with every client), leaves the library's static `advDevice`/`pConnectedClient` dangling,
and can only happen between `onLoop()` calls, which block up to about two minutes while
connecting; it is also untested (D07 already avoided it). So a switch replies `espMode 3`, writes
the wanted mode into four words of RTC_NOINIT memory (magic plus check word, accepted only after
`ESP_RST_SW`), restarts after `ESP_RESTART_DELAY_MS` and starts that radio at boot. The request is
one-shot: `AT+RESET`, a crash or a power cycle come back radio-less, so the stock behaviour is
always one replug away and a board that never sees a D-Sentry command runs the fork's code paths
unchanged (D08's regression rule). A radio is started in place only when no other radio user is
active, including the stock WiFiS3 station that `modem.begin()`'s `AT+SOFTRESETWIFI` leaves running
and the HCI bridge; anything else is a restart. Cost: USB re-enumerates at every switch, and the ROM
banner reaches the RA4M1's Serial RX (CRC framing drops it).

**Smaller choices.** `bootId` is random but never 0 and never the previous boot's (kept in the same
RTC words), so a reboot is always visible. The first `AT+DSS` after a boot only records
`rumbleSeq`, so a heartbeat repeating an old shot never rumbles. The WiFi task is the only owner of
the WiFi state and the only sender on its UDP socket (the AT task hands it frames), auto-reconnect
is off and the supervisor re-`begin()`s at once after a drop (the 1200/2400-baud upload touch calls
`WiFi.disconnect()`, reason ASSOC_LEAVE, which never reconnects), modem sleep is off, and
`WiFi.persistent(false)` keeps the stock credentials in NVS untouched. The rumble characteristic is
found from NimBLE's cache only and forgotten whenever the link is not Connected and before clients
are deleted. Rumble power 0..255 maps to the pad's 0..100; durations over 2.55 s become equal repeats.
The handler bodies and every byte format live in `dsentry_wire.cpp` / `dsentry_at.cpp`, which have no
ESP32 dependencies, so D-SentryCode/tests/esp32 runs the real code on the Mac against the real chAT
server and the RA4M1's real AsyncAt parser.

**Not verified without hardware.** That the pad accepts the stop report (both grip motors at power
0); whether RES# stays high while the ESP32 reboots (GPIO4 floats then); the WiFi timings. These are
on the bench list.

## D20-D25: reconstructed from a separate Claude.ai Project, 2026-11-13

Entries D01-D19 above were written live, during the work. D20 onward were not: a separate
Claude.ai Project (not this repo, not this machine) held chat history about hardware changes
made after `D-SENTRY_SPEC.md` was written, and that history was searched and handed back as
three Claude Artifacts on 2026-11-13. These entries are that handoff's findings, organized
into this log's format. Where the source chats never reached a firm conclusion, that is
stated plainly rather than resolved here.

## D20 5 V rail: a UBEC, not the board's own 5 V, superseding the spec's "no buck converters" rule

**What.** `D-SENTRY_SPEC.md` §13 rules out buck converters and has LEDs/servo/LCD run off the
board's own 5 V. The build moved past that: a 9 V battery browned out the Arduino under load,
so a Durlo 5 V 5 A UBEC was added, fed from the 6S LiPo after the main fuse, into a breadboard
+5V/GND rail that now carries the servo, the LED module, the LCD, and the L298N's logic
supply. The Arduino's own 5 V pin is left unconnected; the Arduino's GND is tied into the
rail; the 9 V battery stays on the barrel jack for the board's logic only. The CNC Shield's
stepper-driver logic still runs from the Arduino's 5 V, unchanged.

**Why.** Estimated loads that drove the change: board ~100 mA (350 mA peak), servo ~200 mA
(stall 0.7-2.5 A), LED module up to 240 mA, LCD ~25 mA, L298N logic ~35 mA — more than the
board's onboard regulator was ever meant to carry once the servo and LEDs were added.

**Open.** Whether `D-SENTRY_SPEC.md` should be edited to reflect this is unresolved — the
question was raised in that project's chat and no answer was recorded.

## D21 Flywheel driver: L298N, after a MOSFET design was built and set aside

**What.** An IRLZ44N MOSFET design (SB560/1N5822 flyback diode, 100 Ω gate resistor, 10 kΩ
pulldown, driven from D9/D10) was designed first. The L298N board actually went in instead —
same part already referenced in `D-SENTRY_SPEC.md`'s hardware table and `THIRD_PARTY.md`'s
CAD vendor list.

**Why not the MOSFET.** Not a correctness problem — the chats note the MOSFET is the more
efficient option (the L298N drops about 2 V and runs warm at 2-3 A combined) — the owner
chose the L298N board anyway. A simple resistor/voltage-divider motor speed control was also
considered and rejected (~54% efficient).

**Also rejected:** driving the servo from the L298N's spare channel (OUT3/OUT4) with PWM —
those outputs swing the full battery voltage, which would damage a hobby servo.

## D22 Servo and logic power: UBEC, after Arduino-5V and L298N-PWM were both rejected

**What.** Before settling on the UBEC (D20), two other ways to power the trigger servo were
tried and dropped: straight off the Arduino's 5 V pin (rejected once the servo is under real
load — the regulator can't carry stall current on top of everything else), and off the L298N's
spare channel with PWM (rejected per D21, full battery voltage would reach the servo). Both
the servo's own smoothing capacitor and the CNC Shield's input capacitor were skipped by the
owner's own choice.

**Also considered earlier, before the LiPo was chosen as the main rail:** an ESP32 camera
(dropped), and running the Arduino from a 9V-only supply with no buck conversion at all (an
XL4015 buck module was on hand but not used; the UBEC was chosen once a buck converter became
acceptable after all).

## D23 Tilt gear: iterated from 16T/80T to 8T/40T, module 2 — and the spec's "4:1" label is wrong

**What.** The tilt reduction went through three gear-tooth revisions at the same pitch
diameters: 16T/80T at module 1 (5:1) → 10T/50T at module 2 → 8T/40T at module 2, +0.6/-0.6 mm
profile shift, 48.2 mm centre distance, 12 mm face width, a 5 mm bore with a D-flat, held by an
M3 set screw. A 10T/50T fallback at 60.2 mm centre distance was kept in reserve.

**Why.** Module 1 teeth were too fine to print reliably; module 2 fixed it. A thin 4.2 mm
pinion wall was flagged as a risk, with a printed hub suggested as the fix.

**Flag.** 40 ÷ 8 = 5:1, not the 4:1 the tilt reduction is called everywhere else in this repo
(`D-SENTRY_SPEC.md`'s hardware table, `README.md`). Nothing recovered in this handoff resolves
which is right — check the actual printed/installed pair before trusting either number.

## D24 Hall-sensor tilt limit and the A2 battery-voltage divider: both removed

**What.** Two sensing approaches that are not in `D-SENTRY_SPEC.md` were built, then taken
back out. A hall sensor + magnet tilt limit switch was removed; tilt homing instead drives
into the mechanical bottom stop (`CAL_RAM` in the spec, §3.5). A 47 kΩ/10 kΩ resistor divider
on A2 (reading 4.42 V at a full 25.2 V pack) for battery-voltage sensing was also removed —
consistent with `D-SENTRY_SPEC.md` §13's "no battery-voltage monitoring" rule, which this
divider would otherwise have violated. The spec's own stall-sense option (a) (D-SENTRY_SPEC.md
§8.1) is a similar divider on A2; nothing in this handoff says one was ever wired for that
purpose, and `STALL_DETECT_ENABLED` is still `false`.

## D25 Open items this handoff could not close

Listed here rather than guessed at:

- **Flywheel duty is voltage-dependent; config has one fixed number.** A wiring-diagram note
  gives duty = 9 ÷ (V_batt − 2): about 99/255 on a full pack (25.2 V), about 129/255 near
  empty (19.8 V). `config.h`'s `FLYWHEEL_DUTY_255` (113, assuming ~22.2 V) is a single point in
  that range, not a formula. A ~0.5 s ramp was suggested.
- **CNC Shield microstep jumpers:** never read off the board. `MICROSTEPS = 8` in config is
  stated as a placeholder to verify, not a confirmed setting.
- **Stepper driver Vref:** never set. Formula on hand: `Vref = Imax × 8 × Rsense` for an
  A4988; at 1.7 A/phase and a guessed Rsense of 0.068 Ω that's roughly 0.92 V, with 70-80% of
  rated current (~0.65-0.75 V) suggested as the safer target. The Rsense resistor's own marking
  (R050 = 0.05 Ω vs R100 = 0.1 Ω) was never read. Set this before motors are ever connected.
- **LCD is blank with the backlight on**, after the UBEC change (D20). Candidate causes, none
  confirmed: the contrast trimmer; the LCD rail only being powered once the LiPo is connected,
  so `lcd.init()` runs before the LCD has power (possible fix: re-run it on POWER-on); a wrong
  I2C address (0x27 vs 0x3F, never scanned); an SDA/SCL swap; a missing common ground; an
  AVR-only LCD library running on the Renesas core.
- **L298N channel:** moved from A to B and back to A during the build. What's in
  `D-SENTRY_SPEC.md` now (channel A, IN1 D9, ENA D10, IN2 D11) is the final state, per this
  handoff and existing memory.

## D26-D31: the owner's own recap, 2026-11-13 — and a branch split that matters

Told directly by the project owner, not reconstructed from a handoff. This is the most
authoritative source in this file for anything it covers, and it corrects the branch this
whole repo was built from.

## D26 `turret-live` is what's actually on the turret; `d-sentry` is not

**What.** `d-sentry` (20 commits) is where everything up through D19 above was built. After
it, the owner rolled the turret back to before its LED-scheme and servo-flash-save changes —
those two are still sitting on `d-sentry`, merged in, but were **never flashed**. Every change
since went on a new branch, `turret-live`, which branched from just before the rollback point
and is what has actually been running on the turret through 13 further commits of bench
tuning (D27-D30 below). This repo's `Software/Turret Code for the Arduino/`,
`Software/System Block Diagram/architecture.md`, `Software/Custom Firmware/protocol/`, and
the PC relay under `Software/Local Web Server for WiFi Connect Mode/` were re-synced from
`turret-live` on 2026-11-13; they had originally been built from `d-sentry` and were stale.

**What this removed.** `ServoStore.cpp/.h` (the data-flash save for trigger angles) does not
exist on `turret-live` — it was part of the rolled-back work. Don't look for it; it's gone,
not missing.

**Also found and fixed during this re-sync:** `turret-live`'s `config.h` and the ESP32 fork's
synced copy both hard-code a real WiFi access-point password (`WIFI_AP_PASSWORD`) that had
already been pushed to this public repo in its very first commit (it was present on
`d-sentry` too, set 2026-11-03). Both copies have been redacted to a placeholder going
forward. **That password is still recoverable from this repo's git history** — redacting the
current file doesn't remove it from past commits. The turret's own AP password should be
treated as burned; change it on the real device to something not in any commit.

## D27 Firing rework: a three-position servo sequence, not two

**What.** The trigger servo now moves through three named positions instead of REST/FIRE:
**REST** (125), **FIRE** (9), **PRE-FIRE** (65), slewing at 600°/s between them. The turret
boots with ammo at 0 and the servo at REST. X loads 15 rounds and moves the servo to
PRE-FIRE; X again unloads (ammo back to 0, servo back to REST). A semi-auto shot is
PRE-FIRE → FIRE → REST → PRE-FIRE. Full auto is REST → FIRE → REST repeated for as long as
the trigger is held, dropping to PRE-FIRE only once it's released. A `TRIGGER_CLEAR_ENABLED`
stop (a fourth, in-between position) exists in code but is switched off.

**Why.** Superseded by the owner's own bench feel; no single REST/FIRE pair covered both "sits
ready to fire" and "stays clear while not loaded." Values arrived at in stages: REST 180/FIRE
55 and later REST 180/FIRE 64 (still two-position, pre-dating this rework) → REST 125/FIRE 10
→ REST 125/FIRE 9/PRE-FIRE 65 (current).

## D28 Pan: direction flipped, speed raised in three steps, one still a trial

**What.** `PAN_DIR_INVERT` turned on (it ran the wrong way). Full-stick pan speed went
90°/s → 135°/s → 270°/s; the last is explicitly a trial, not yet confirmed safe. Raising it
needed the step-pulse ceiling raised (`STEP_MAX_RATE_HZ` 3,000 → 6,000/s) and acceleration
raised (400 → 800°/s²).

**Open.** Test: swing pan hard both directions, then command home. Landing exactly on home
means the motor kept up at 270°/s and 360°/s can be tried; missing home means steps were
skipped and the speed should drop to 180°/s instead. Not yet run as of this recap.

## D29 Tilt: calibration sped up, barrel auto-lifts, stick direction settled

**What.** Tilt calibration now takes under 5 seconds (was about 9), and the barrel lifts to
20° on its own right after. `JOY_INVERT_Y` was flipped, then flipped back — the final state
has the stick working the intuitive way (stick up = barrel up).

## D30 Flywheels: a d-pad toggle, not the bumper the spec describes — and a known overdrive, unfixed

**What.** D-pad up toggles the flywheels on/off while ARMED. This replaces
`D-SENTRY_SPEC.md` §3.2's "left bumper, hold or toggle" mapping — the spec is stale here.

**Open, by choice.** The flywheels still run at a fixed PWM duty of 113/255 in the mode that
over-drives the motors (consistent with `DECISIONS.md` D25's note that `FLYWHEEL_DUTY_255` is
one fixed point inside a voltage-dependent range). A software fix was written and offered;
the owner has not approved it, so it is not applied. Don't apply it unasked.

## D31 Bugs hit and fixed during turret-live bench sessions

- **Pad wouldn't pair.** The turret still remembered an old pairing. A small one-off sketch
  made it forget; pairing then worked once the owner's phone had Bluetooth switched off (a
  phone in range will out-compete the turret for a pad in pairing mode).
- **Board froze after a sketch upload.** Fixed by unplugging and replugging the board. No
  root cause recorded beyond that.
- **LCD froze.** Traced to wiring; errors went to zero after a reflash and reseating the
  connections. Possibly the same issue as the "LCD blank, backlight on" problem in D25 (that
  one came from a separate chat-history handoff and was never confirmed fixed there) — the
  symptom described here is "froze" rather than "blank," so treat this as likely related, not
  confirmed identical, until someone checks both against the current wiring.

**Parked, still on `d-sentry` only, never flashed:** the LED scheme and the servo-flash-save
feature (D26). If either is wanted on the real turret, it needs to be ported onto
`turret-live`, not assumed to already be there.

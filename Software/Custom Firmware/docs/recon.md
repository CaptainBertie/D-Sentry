# Stage 0 recon: Xbox Series X|S BLE controller on the UNO R4 WiFi USB bridge

Date: 2026-10-11. This is the Stage 0 deliverable of the overnight build.

Scope: what five recon readers and their adversarial verifiers found about two additions. The first is an Xbox BLE client in the ESP32-S3 bridge firmware. The second is an RA4M1 library that reads the controller over the existing AT link. Nothing was run on hardware, and no board was connected.

How to read this file:
- Confidence comes from the readers. **High** means seen in source or tool output. **Medium** means reasoned from source or computed. **Low** means recalled. Anything low or recalled is marked *unverified*.
- Where a verifier corrected a claim, this file states the corrected version.
- Paths are relative to the repo root unless they start with one of these prefixes:
  - `$CORE` = `firmware/hardware/esp32-patched/esp32` (the patched arduino-esp32 2.0.9 core).
  - `$RA` = `~/Library/Arduino15/packages/arduino/hardware/renesas_uno/1.6.0` (byte-identical to the PlatformIO copy).
  - `$SCR` = `/private/tmp/claude-501/-Users-jwilson-Downloads-University-Meche-Projects-A4/8d962a3f-21ef-4dd6-a11c-7b809952b608/scratchpad`. This is session scratch and may be gone.
  - `$SRC` = `$SCR/src` (scratch clones of upstream repos).
  - `hpp` = `$SRC/arduino-XboxSeriesXControllerESP32/src/XboxSeriesXControllerESP32_asukiaaa.hpp` (Xbox library v1.1.3).
  - NimBLE paths are at tag 2.5.1 of NimBLE-Arduino unless a version is named.

## 1. Summary

1. Nothing reusable exists. None of the 19 forks of `arduino/uno-r4-wifi-usb-bridge` adds BLE central, NimBLE, Bluepad32 or Xbox support, so the glue must be written.
2. The RA4M1 cannot do it alone. Stock ArduinoBLE 2.1.0 pairs only as a peripheral, and HID over GATT needs a bonded, encrypted link. The BLE host must run on the ESP32-S3.
3. NimBLE-Arduino 2.5.1 and the asukiaaa Xbox library 1.1.3 link into the patched 2.0.9 core with no core or sdkconfig change. They add 124,176 B; the projected app is about 1,304,272 B of a 1,638,400 B slot.
4. The RA4M1 polls one binary-safe command: `AT+XBOXSTATE?` returns `+XBOXSTATE: 20|<20 bytes>\r\nOK\r\n` (41 B) at 115200 baud. Polling at 50 Hz blocks about 25–31 % of RA4M1 CPU (computed).
5. The Xbox client and the existing `AT+HCI*` ArduinoBLE bridge both want the one BT controller. They must be mutually exclusive.
6. The library's `onLoop()` blocks for seconds, so it needs its own FreeRTOS task. The ESP handler must always send the prompt and `OK`, never `ERROR`.
7. Three RA-side parser bugs force the RA library to use a short timeout and drain Serial2 before each poll. It must also check length, sequence number and checksum.
8. New tasks must print nothing. Their stdout and IDF logs go out on UART0, into the RA4M1's Serial and upload path.
9. The build needs its own size gate at 1,638,400 B, because arduino-cli allows 1,671,168 B and `combine.py` checks nothing. Keep the HID 0xAA reboot handler, USB VID/PID and 1200/2400-baud logic untouched.
10. The combined firmware is AGPL-3.0-or-later, because chAT is linked in.

## 2. How the ESP32-S3 and the RA4M1 talk

### 2.1 Physical links

| Link | ESP32-S3 side | RA4M1 side | Speed | ESP32 owner |
|---|---|---|---|---|
| AT command link | `Serial1` = UART1, RX GPIO6, TX GPIO5, 8192 B RX and TX buffers | `Serial2` = `_UART3_`, D24 = P501 "TX WIFI", D25 = P502 "RX WIFI", 512 B RX ring | fixed 115200 8N1 | task `Task1` (`atLoop`), core 0 |
| USB passthrough | `Serial` = UART0, RX GPIO44, TX GPIO43 | `Serial` = `_UART1_`, D22/D23 = P109/P110 "TX"/"RX" | follows the host's CDC line coding | `loopTask`, core 1 |
| RA4M1 control | `GPIO_RST` = 4, `GPIO_BOOT` = 9 | reset and boot-mode pins (*unverified*, recalled from the schematic) | — | 1200/2400-baud handler |

Evidence:
- `firmware/UNOR4USBBridge/UNOR4USBBridge.ino:27` `#define SERIAL_AT Serial1`.
- `UNOR4USBBridge.ino:157` `SERIAL_USER_INTERNAL.begin(115200, SERIAL_8N1, 44, 43);`.
- `UNOR4USBBridge.ino:158-160` `SERIAL_AT.setRxBufferSize(8192); ... SERIAL_AT.begin(115200, SERIAL_8N1, 6, 5);`.
- `$RA/cores/arduino/Arduino.h:113-115` (under `NO_USB`) `#define Serial _UART1_`, `#define Serial1 _UART2_`, `#define Serial2 _UART3_`.
- `$RA/variants/UNOWIFIR4/pins_arduino.h:111-112` `UART3_TX_PIN 24`, `UART3_RX_PIN 25`. `variant.cpp:51-54` labels D22 "TX", D23 "RX", D24 "TX WIFI", D25 "RX WIFI".
- `$RA/cores/arduino/Serial.h:41` `#define SERIAL_BUFFER_SIZE 512`.
- `firmware/UNOR4USBBridge/at_handler.h:25-26` `#define GPIO_BOOT 9`, `#define GPIO_RST 4`.

Board wiring (medium, from the text of the ABX00087 schematic):
- ESP_IO5 and ESP_IO6 pass through a TXB0108 level shifter to `P502_SCI1_RXD` and `P501_SCI1_TXD`.
- ESP_TXD0 (GPIO43) goes to `P110_SCI9_RXD`. ESP_RXD0 (GPIO44) comes from `P109_SCI9_TXD`.
- Which shifter channel pairs with which line was read from the layout of extracted text.
- UART0 TX and RX also appear on ESP header J6 (datasheet p11: `ESP_TXD0 (Pin 3)`, `ESP_RXD0 (Pin 5)`).

The header comment at `UNOR4USBBridge.ino:5-6` names GPIO 20/21 and 2/3. It is stale from the ESP32-C3 era.

The RA4M1 build defines `ARDUINO_UNOWIFIR4`, `ARDUINO_UNOR4_WIFI` and `NO_USB` in both the Arduino IDE and PlatformIO (`$RA/boards.txt:121-122`, `$RA/platform.txt:74`, PlatformIO `uno_r4_wifi.json`).

### 2.2 The chAT protocol on the ESP32 side

chAT (from SudoMaker) is the AT server in the bridge: `firmware/UNOR4USBBridge/parser.cpp`, `server.cpp` and `chAT.hpp`.

Line framing:
- A command starts with uppercase `AT` (`parser.cpp:79`). Lowercase is malformed and gets `ERROR`.
- A line ends at the first `\n` (`parser.cpp:84-91`). `CHAT_STRICT_CRLF` is defined nowhere, so LF-only input also parses.
- `\r` is ignored only after the `AT` keyword, in the Command and Argument states (`parser.cpp:135-137`, `:171-173`). A `\r` before `AT` makes the line malformed (`parser.cpp:101-107`). A stray leading `\r\n` therefore produces an extra `ERROR\r\n`. The RA library must send exactly `AT+XBOXSTATE?\r\n` with no prefix. (Verifier correction; harness `$SCR/refute_wire/t.cpp`.)

Command modes (`chAT.hpp:35-37`, `parser.cpp:119-131`):

| Form | Mode | Enum value |
|---|---|---|
| `AT+X` | Run | 0 |
| `AT+X=a,b` | Write | 1 |
| `AT+X?` | Read | 2 |
| `AT+X=?` | Test | 3 |

- The command name keeps the `+` and drops the `?` (`parser.cpp:139-140`).
- Lookup is an exact match in an `unordered_map` (`at_handler.h:81`, `at_handler.cpp:131-134`). An unknown name returns `ERROR` (`at_handler.cpp:134-135`, `server.cpp:212`).
- A name must not contain `?`, `=`, `:`, CR or LF. The RA parser ends the prompt at `:` or `=` (`Modem.cpp:268`).
- Arguments follow `=`. They split on `,` unless inside `"`; the quote toggles and is not stored. `\` escapes the next byte (`parser.cpp:151-179`).
- The input buffer is 1024 B (`server.cpp:105`). The write queue is capped at 16384 B and drops the oldest bytes (`server.cpp:108`, `:153-161`).
- Binary input after a length argument works in three steps (`server.cpp:168-187`, `cmds_ble_bridge.h:127-139`). `inhibit_read(n)` returns the bytes already buffered. The handler then busy-reads until n bytes arrive, with no yield. `continue_read()` resumes parsing.

Response framing:
- `write_response_prompt()` writes the command name and `": "` (`server.cpp:221-225`).
- `write_line_end()` writes `\r\n` (`server.cpp:232-234`).
- The server appends the trailer from the handler's return value (`server.cpp:132-137`, `:212`, `:216-218`). `OK` appends `OK\r\n`, `ERROR` appends `ERROR\r\n`, and `CUSTOM` appends nothing.

### 2.3 The binary-safe `len|bytes` reply

Request, 15 B (verified by harness). The RA macro `CMD_READ(x)` produces exactly this (`commands.h:160` `#define CMD_READ(x) _AT x "?" _ENDL`):
```
41 54 2B 58 42 4F 58 53 54 41 54 45 3F 0D 0A      "AT+XBOXSTATE?\r\n"
```

Reply, 41 B:
```
2B 58 42 4F 58 53 54 41 54 45 3A 20 32 30 7C      "+XBOXSTATE: 20|"   15 B
<20 raw payload bytes>                                                 20 B
0D 0A                                                                   2 B
4F 4B 0D 0A                                        "OK\r\n"             4 B
```

Proposed handler. It does not exist in the firmware yet:
```cpp
srv.write_response_prompt();
srv.write_str("20");
srv.write_str("|");
srv.write_vec8(state);            // exactly 20 bytes
srv.write_line_end();
return chAT::CommandStatus::OK;
```

Existing commands with the same shape:

| Command | Wire form | Usable as a template |
|---|---|---|
| `+PREFGET` with `PT_BLOB` (`cmds_preferences.h:321-331`) | `+PREFGET: <len>\|<bytes>\r\nOK\r\n` | yes |
| `_SOFTSE_SERIAL` (`cmds_se.h`) | `<len>\|<data>\r\n` then `OK\r\n` | yes |
| `+HCIREAD` (`cmds_ble_bridge.h:90-95`) | `+HCIREAD: <n>\|<bytes>OK\r\n`, no CRLF before OK | yes |
| `+CLIENTRECEIVE` (`cmds_wifi_netif.h:764-768`) | no CRLF before OK | yes |
| `+FWVERSION_U32` (`cmds_esp_generic.h:88-90`) | 4 raw bytes, no length | no: the RA side drops 0x20 and stops at CR, LF or `\|` |
| `+GMR` (`cmds_esp_generic.h:47-49`) | prompt commented out | no: times out against any prompt |

- Both template forms parse correctly on the RA side for any byte values.
- A payload containing CR, LF, `|`, `O`, `K`, `ERROR`, NUL and 0xFF came back intact. The harnesses compiled the real chAT `parser.cpp` and `server.cpp` and the real RA `Modem.cpp` (`$SCR/chatsim/e2e_esp.cpp`, `$SCR/modemsim/e2e_ra.cpp`, `$SCR/verify_wire/`, `$SCR/refute_link/h.cpp`).
- The CRLF before `OK` is a design choice, not a framework rule.

### 2.4 The RA4M1 side: `ModemClass`

- `modem` is a public global bound to `Serial2` (`$RA/libraries/WiFiS3/src/Modem.h:169`, `Modem.cpp:423-424`).
- WiFiS3 uses `modem`. So do Preferences (`Preferences.h:18`, `Preferences.cpp:344-353`) and ArduinoBLE on the R4 (`HCIVirtualTransportAT.cpp:20`, `:24`, `:43`, `:86-92`).
- The first `modem.begin()` wins. It opens Serial2 at 115200 and sends `AT+SOFTRESETWIFI` with a 500 ms timeout and 3 tries (`Modem.cpp:33-45`). Every stock caller uses 115200, so changing the baud breaks them.
- `modem.write(prompt, res, fmt, ...)` formats into a 128 B buffer (`Modem.h:14`, `Modem.cpp:105-118`). It does a blocking UART write (`$RA/cores/arduino/Serial.cpp:127-133`), then busy-polls until it sees OK or times out. There is no RTOS and no yield.
- The default timeout `MODEM_TIMEOUT` is 10000 ms (`Modem.h:12`). The idiom is `modem.timeout(x)`, the call, then `modem.timeout(MODEM_TIMEOUT)` (`OTAUpdate.cpp:60`, `:69`). There is no getter (`Modem.h:149`).
- `WiFiS3/src/WiFiCommands.h` is a symlink to the bridge's `commands.h`. It brings the `CMD`, `CMD_READ`, `CMD_WRITE` and `PROMPT` macros. A new command string must be `#define`d in our own library.
- Sized read: in the Data state, a `|` runs `atoi()` on the bytes since `:`. A non-zero value enters the Sized state, which copies exactly N bytes without looking at them. A zero skips to the Res state (`Modem.cpp:296-303`, `:315-325`). Res skips an optional CRLF before `OK` (`Modem.cpp:337-356`).
- `read_using_size()` is an empty no-op (`Modem.h:111-113`). `avoid_trim_results()` changes nothing for sized data and resets after one call (`Modem.cpp:413`).
- The length must be the leading decimal token. `20x|` and `20 |` still parse as 20.

### 2.5 RA4M1 parser quirks the design must route around

| # | Quirk | Cause | Effect | Mitigation |
|---|---|---|---|---|
| 1 | ERROR surfaces only after the timeout | `Modem.cpp:7` `#define RESULT_ERROR OK_TOKEN TERM_TOKEN` (copy bug) | `ERROR\r\n` is never matched. `modem.write()` returns false only after the full timeout, 10 s by default. `passthrough()` reports a plain ERROR as success. | The ESP handler never returns ERROR. The RA library probes once in `begin()` with a ~100 ms timeout to detect stock firmware. |
| 2 | A reply without the prompt times out | `Modem.cpp:243-247`, `:400-402` | A bare `OK\r\n` with no `+XBOXSTATE:` line is discarded and the call times out. The same latent bug exists upstream at `cmds_wifi_station.h:16-17`. | Always send the prompt, even with no controller. Prefer the fixed payload with a "connected" flag; `+XBOXSTATE: 0\|\r\nOK\r\n` also parses. |
| 3 | A stale sized reply truncates the real one | `sized_read_count` is set once per call (`Modem.cpp:170-171`) and never reset on restart paths (`:389-403`) | The real payload is cut short and the call still returns true. | Drain Serial2 before each poll. Require `res.size() == N`. |
| 4 | Late reply lag (medium, reasoned) | only the prompt is compared (`Modem.cpp:271-276`) | After a timeout, the next poll consumes the late reply as if fresh, and the lag repeats. | Put a sequence counter in the payload. Treat a timeout as "drop the next reply". |
| 5 | Length not the only token | `atoi` | `1,20\|` keeps 1 byte. It returns true with a 1-byte payload if no `O` follows, and fails otherwise. | The length is the only token before `\|`. |
| 6 | Payload equal to "OK" or "ERROR" | the Res state compares the whole collected string | Only a 2- or 5-byte payload equal to those words completes early. | Moot for a fixed 20 B payload. |

Harness results (`$SCR/modemsim`, `$SCR/refute_sized`):
- Plain `ERROR` with a 300 ms timeout: ok=0 after 301 ms. The real chAT reply to `AT+NOPE?` with a 200 ms timeout: ok=0 after 201 ms.
- `OK\r\n` alone: ok=0 after 201 ms. `+XBOXSTATE: \r\nOK\r\n`: ok=1, len=0.
- Stale `+HCIAVAILABLE: 5|vwxyzOK` ahead of `+HCIREAD: 20|<20 B>OK`: ok=1, len=15.
- `1,20|` with no `O` in the rest: ok=1, len=1. With `OKa` in the rest: ok=0.
- `0|`: ok=1, len=0.

The Arduino15 and PlatformIO copies of `Modem.cpp` and `Modem.h` are byte-identical to the ArduinoCore-renesas clone at `424e86e`.

Proposed RA poll (link reader). Include `<WiFiS3.h>`, as ArduinoBLE does, to get `Modem.h`, `extern ModemClass modem` and the macros:
```cpp
modem.begin();
while (Serial2.available()) Serial2.read();      // drop stale bytes
std::string res;
modem.timeout(50);
bool ok = modem.write(std::string(XBOX_AT_STATE ":"), res, "%s", "AT" XBOX_AT_STATE "?\r\n");
modem.timeout(MODEM_TIMEOUT);
if (!ok || res.size() != 20) return false;       // then check sequence and checksum
memcpy(out, res.data(), 20);
```

### 2.6 Bandwidth and latency (computed, not measured)

At 115200 8N1 the line carries 11,520 B/s, or 86.8 µs per byte. One poll is 15 B out and 41 B back: 56 B, 4.86 ms on the wire. The ESP adds turnaround time:
- the UART RX idle timeout of 2 symbols, about 0.17 ms (`$CORE/cores/esp32/HardwareSerial.cpp:143` `_rxTimeout(2)`);
- up to 1 ms for `vTaskDelay(1)` in `atLoop` (`at_handler.cpp:86`; sdkconfig:1737 `CONFIG_FREERTOS_HZ=1000`);
- preemption by higher-priority core-0 tasks.

The total is about 5.0–6.1 ms per poll. The RA4M1 busy-waits through all of it (`Modem.cpp:194-196`).

| Poll rate | ESP→RA | RA→ESP | RA4M1 CPU blocked |
|---|---|---|---|
| 50 Hz | 2,050 B/s (17.8 % of line) | 750 B/s (6.5 %) | ~25–31 % |
| 100 Hz | 4,100 B/s (35.6 %) | 1,500 B/s (13.0 %) | ~50–61 % |
| ceiling | ~205 Hz on the wire, ~160–200 Hz in practice | — | ~100 % |

- Recommended rate: 50 Hz.
- A shorter name such as `+XBS` saves 12 B per poll (44 B, 3.8 ms). Do that before touching the baud.
- While the host's CDC baud is 1200 or 2400, `atLoop` skips `atHandler.run()` (`UNOR4USBBridge.ino:114-116`). RA polls time out during an RA4M1 upload. The upload itself is unaffected.

## 3. The bridge firmware's tasks and what must not change

### 3.1 Tasks

| Task | Core | Priority | Stack | Job | Evidence |
|---|---|---|---|---|---|
| `loopTask` | 1 | 1 | core default | USB CDC ↔ UART0 passthrough in `loop()` | `$CORE/cores/esp32/main.cpp:71`; `UNOR4USBBridge.ino:199-215` |
| `Task1` (`atLoop`) | 0 | 1 | 10000 B | chAT server on Serial1, drains the `addTask()` queue, then `vTaskDelay(1)` | `UNOR4USBBridge.ino:111-119`, `:172-179`; `at_handler.cpp:69-87` |
| `usbd` (TinyUSB) | unpinned | 24 | 4096 B | USB device stack | `esp32-hal-tinyusb.c:718` |
| BT controller | 0 | — | — | IDF BLE controller | sdkconfig:835 |
| WiFi, lwIP `tcpip` | 0 | — | — | network | sdkconfig:1625, :1918 |
| `bleTask` (only after `AT+HCIBEGIN`) | 0 | 5 | 2048 B | ArduinoBLE VHCI pump | `firmware/libraries/ArduinoBLE/src/utility/HCIVirtualTransport.cpp:87-89` |
| `nimble_host` (new) | 0 | 21 | 4096 B | NimBLE host; runs notification callbacks | `nimconfig.h:212-218`; `nimble_port_freertos.c:40`, `:56-57` (2.3.9) |
| Xbox client (new) | 0 or 1, see below | 1 | 6–8 KB suggested | runs `xbox.onLoop()` | — |

- `configMAX_PRIORITIES` is 25 (`FreeRTOSConfig.h:81`).
- The task watchdog checks only the CPU0 idle task, with a 5 s timeout (sdkconfig:1587-1589). Whether `CONFIG_ESP_TASK_WDT_PANIC` is set was not checked. If it is, a spinning core-0 task reboots the bridge. A watchdog message would also go to UART0.
- The NimBLE host defaults are fine. The host shares core 0 with the controller and preempts the AT task only briefly.

The Xbox task's core is undecided. The readers disagree:
- Core 1, priority 1 (xbox reader; the upstream example uses core 1 with 4096 B). A spinning task time-slices with `loopTask`, which carries the USB passthrough and the RA4M1 upload.
- Core 0, priority 1, 6144 B (link reader). A spinning task starves IDLE0, and the watchdog fires after 5 s.
- Either way, the task must block for at least one tick per iteration. The compiled example uses `vTaskDelay(pdMS_TO_TICKS(10))`.

### 3.2 The USB passthrough path

- `loop()` moves bytes only between `USBSerial` (the TinyUSB CDC port) and `Serial` (UART0, pins 44/43) (`UNOR4USBBridge.ino:20-22`, `:199-215`).
- Only UART0 follows the host's baud (`UNOR4USBBridge.ino:244-245` `SERIAL_USER_INTERNAL.updateBaudRate(baud)`).
- The AT server only touches Serial1 (`at_handler.cpp:118-128`). Adding a `command_table` entry touches neither UART0 nor `loop()`.

The fork can break upload or passthrough in four ways only:
1. Printing to the wrong stream (3.3).
2. Blocking or starving `loopTask` (core 1, priority 1).
3. Changing the USB VID/PID or the 1200/2400-baud handling.
4. `AT+HCIBEGIN` fighting NimBLE for the BT controller (3.5).

### 3.3 Where output goes: the stdout and esp_log caveat

- `setup()` redirects `stdout` to `USBSerial` with `funopen(write_fn)` (`UNOR4USBBridge.ino:41-46`, `:126`). It redirects ROM `ets_printf` with `ets_install_putc1` (`:131`). `USBSerial` is the CDC stream that bossac and the Serial Monitor use, so any print there can corrupt an RA4M1 upload.
- The `stdout` redirect covers `loopTask` only (critic, medium). Newlib keeps stdout per task: `FreeRTOSConfig.h:202` `#define configUSE_NEWLIB_REENTRANT 1`, toolchain `stdio.h:161` `#define stdout (_REENT->_stdout)`, `sys/reent.h:832` `# define _REENT (__getreent())`.
- Other tasks (Task1, `nimble_host`, the new Xbox task) write to the IDF console, which is UART0 (sdkconfig:1571 `CONFIG_ESP_CONSOLE_UART_DEFAULT=y`, :1580 `CONFIG_ESP_CONSOLE_UART_NUM=0`). UART0 TX is GPIO43, the line into the RA4M1's Serial RX and its bossac upload stream.
- That `esp_reent_init` copies the global stdout into new tasks is *unverified* (recalled). The per-task stdout macro is verified.
- The precompiled IDF libraries log at ERROR level (sdkconfig:1807 `CONFIG_LOG_DEFAULT_LEVEL_ERROR=y`). A controller init failure or a coexistence error would therefore inject bytes into the RA4M1. Neither the bridge nor the core calls `esp_log_level_set` or `esp_log_set_vprintf`.

Rules for the fork:
- At the top of `setup()`, call `esp_log_level_set("*", ESP_LOG_NONE)` or `esp_log_set_vprintf` with a sink that discards or writes to `USBSerial`.
- Put no `printf` or `ESP_LOG*` in the Xbox task or the glue.
- Never define `XBOX_SERIES_X_CONTROLLER_DEBUG_SERIAL`. Pointed at `Serial` it feeds the RA4M1; pointed at `Serial1` it corrupts AT replies.
- Keep `DebugLevel=none` and NimBLE's default log level, which is none (`nimconfig.h:267-268` in 2.3.9).

### 3.4 The HID 0xAA reboot handler, VID/PID and version

- `DAP.h:71-74` answers a HID SET_REPORT(Feature) whose first byte is 0xAA by calling `usb_persist_restart(RESTART_BOOTLOADER)`. This is the jumper-free path into ESP32-S3 download mode (section 8).
- USB identity: `USB.VID(0x2341); USB.PID(0x1002);`, manufacturer "Arduino", product "UNO WiFi R4 CMSIS-DAP" (`UNOR4USBBridge.ino:143-146`).
- `DAP.begin()` (`:149`) and `USB.begin()` (`:161`) run in `setup()`. Start the BLE code after `USB.begin()`, so a BLE crash cannot stop the HID handler from enumerating.
- `SERIAL_USER.enableReboot(false)` (`:151`) disables the core's CDC reboot. Keep it.
- The 1200/2400-baud handler (`:226-243`) resets the RA4M1 or puts it in its bootloader. Keep it.
- The version is `FIRMWARE_MAJOR/MINOR/PATCH` = 0/6/0 (`at_handler.h:19-21`). HID GET_REPORT(Feature) returns these 3 bytes (`DAP.h:76-80`), and bcdDevice carries major and minor (`UNOR4USBBridge.ino:147`). Bump the version so the fork can be told apart without entering download mode.
- If `ESP_FW_VERSION` gets a suffix, it must still fit the RA4M1's 12-byte buffer (`$RA/libraries/WiFiS3/src/WiFi.cpp:20-21`).

### 3.5 BT controller ownership: `AT+HCI*` versus NimBLE

- The stock bridge exposes the BLE controller to the RA4M1 through VHCI, the IDF's virtual host-controller interface. The RA4M1 runs the ArduinoBLE host and sends raw HCI over AT.
- `AT+HCIBEGIN` calls `HCIVirtualTransport.begin()` (`cmds_ble_bridge.h:18`). That calls `esp_bt_controller_init` and `esp_bt_controller_enable(ESP_BT_MODE_BLE)`, registers its VHCI callback and starts `bleTask`, with no return-value checks (`HCIVirtualTransport.cpp:46`, `:87-89`).
- `NimBLEDevice::init` also calls `esp_bt_controller_init` (`NimBLEDevice.cpp:945`) and `esp_vhci_host_register_callback` (`esp_nimble_hci.c:300`).
- The controller holds one VHCI callback, and a second init returns an error. NimBLE's init then returns false, and the Xbox library's `Core::begin()` ignores that.
- Both stacks link into one image (tested). Only one can own the controller at runtime.
- Both deinit paths disable and deinit the controller without releasing its BLE memory (`nimble_port.c:164`, `:231`, `:237`; `HCIVirtualTransport.cpp:97-99`). A handover without a reboot is plausible in source and untested.

Recommended (critic): sticky modes.
- A new `AT+XBOXBEGIN` starts the Xbox client. It refuses while HCI is active.
- All six `_HCI_*` commands (BEGIN, END, WAIT, AVAILABLE, READ, WRITE) refuse while Xbox mode is active. Because of quirk 1, the RA side sees that refusal only after its timeout; document it.
- Modes switch through the existing `AT+RESET` (`cmds_esp_generic.h:21-25`, `ESP.restart()`).
- With Xbox mode unused, the bridge behaves exactly like stock.

### 3.6 What must not change

- USB VID/PID 0x2341/0x1002, the product string, the DAP HID device and its `_onSetFeature` 0xAA handler.
- The 1200/2400-baud handler and `enableReboot(false)`.
- `loop()`'s passthrough and the UART0 pin setup.
- The AT link baud (115200) and pins (GPIO6/5).
- No blocking work in `loop()`, `atLoop` or `CAtHandler::addTask()` (`at_handler.h:100-102`).
- No `partitions.csv` or `bootloader.bin` in the sketch folder (5.8).
- The partition table and the 4 MB layout (section 4).

## 4. Partition table and flash

### 4.1 Layout

Rows come from `$CORE/tools/partitions/unor4wifi.csv:2-7`, confirmed by decoding the built `UNOR4USBBridge.ino.partitions.bin`. The bootloader and table rows come from `firmware/combine.py:12-20`.

| Region | Type / subtype | Offset | Size | Bytes | Contents and users |
|---|---|---|---|---|---|
| bootloader | — | 0x000000 | 0x8000 | 32,768 | second-stage bootloader |
| partition table | — | 0x008000 | 0x1000 | 4,096 | this table (3,072 B image) |
| otadata | data / ota | 0x009000 | 0x2000 | 8,192 | `boot_app0.bin`, which selects app0 |
| cert | data / 0x06 | 0x00B000 | 0x45000 | 282,624 | x509 bundle (0.6.0: 0x109F8 = 68,088 B, rest 0xFF); found by name (`cmds_wifi_SSL.h:108`) |
| app0 | app / ota_0 | 0x050000 | 0x190000 | 1,638,400 | the bridge app |
| app1 | app / ota_1 | 0x1E0000 | 0x190000 | 1,638,400 | ESP32 OTA target (`OTA.cpp:21` includes `Update.h`; that OTA writes app1 is recalled) |
| spiffs | data / spiffs | 0x370000 | 0x80000 | 524,288 | OTA and BOSSA staging files (`BossaUnoR4WiFI.cpp:52`) |
| nvs | data / nvs | 0x3F0000 | 0x10000 | 65,536 (16 × 4 KB) | see 4.4 |

- The partitions sit end to end and end exactly at 0x400000 (4 MB).
- There is no factory, coredump or phy_init partition. The prebuilt SDK enables core dump to flash (`dio_qspi/include/sdkconfig.h:813` `CONFIG_ESP_COREDUMP_ENABLE_TO_FLASH 1`), so a core dump has nowhere to go.

### 4.2 The three images

`firmware/combine.py` builds a 4 MB buffer filled with 0xFF and slices it (`:23`, `:26`, `:33-45`, `:60`, `:68`).

| Image | Size | Write at | Covers | Leaves untouched |
|---|---|---|---|---|
| `S3-ALL.bin` | 0x400000 = 4,194,304 | 0x0 | everything; app1 and NVS become 0xFF; SPIFFS becomes an empty formatted image | nothing: wipes NVS (Preferences, BLE bonds) and SPIFFS |
| `S3-BOOT-APP.bin`, shipped as `UNOR4-WIFI-S3-<ver>.bin` | 0x1E0000 = 1,966,080 | 0x0 | bootloader, table, otadata, the whole cert region, app0 | app1, SPIFFS, NVS |
| `S3-APP.bin` | 0x1D2000 = 1,908,736 | **0xE000 only** | cert from 0xE000, app0 | bootloader, table, otadata, cert 0xB000–0xE000, app1, SPIFFS, NVS |

- The default is `S3-BOOT-APP.bin` at 0x0. Arduino's updater ships it. It resets otadata to app0, so the board boots the new app even if an earlier ESP32 OTA left it on app1. It keeps NVS and SPIFFS.
- `S3-APP.bin` does not reset otadata, and it keeps whatever cert bytes sit at 0xB000–0xE000. On a board running official 0.6.0 those bytes are byte-identical to ours (verified).
- Use `S3-ALL.bin` only for a deliberate factory wipe. Use a full backup to restore.
- Bridge issue #49: an update from 0.3.0 via S3-BOOT-APP broke SPIFFS. The fix was S3-ALL from 0.4.1.

### 4.3 The real app size limit

| Limit | Bytes | Source |
|---|---|---|
| app0 slot (real) | 1,638,400 (0x190000) | `unor4wifi.csv:4` |
| arduino-cli limit with `PartitionScheme=unor4wifi` | 1,671,168 (0x198000) | `$CORE/boards.txt:1251` |
| board default, overridden by the menu | 1,310,720 | `$CORE/boards.txt:1085` |

- arduino-cli allows 32,768 B too much.
- arduino-cli sums the ELF sections matched by `$CORE/platform.txt:237`, not the .bin. The .bin is about 359 B larger (1,180,096 vs 1,179,737). A .bin up to about 1,671,500 B therefore passes, and lowering `maximum_size` alone does not fix it.
- `combine.py:41-42` copies the app to 0x50000 with no length check. An oversized app spills into app1 in `S3-ALL.bin`. It is silently cut at 0x1E0000 in `S3-BOOT-APP.bin` and `S3-APP.bin`. Every output is corrupt, and nothing reports it.
- That the IDF bootloader refuses an app longer than its partition is *unverified* (recalled; the bootloader's log strings are stripped).
- Required: a post-build gate that fails when `UNOR4USBBridge.ino.bin` exceeds 1,638,400 B. It must run before `combine.py`.

Budget:

| Item | Bytes |
|---|---|
| Stock `UNOR4USBBridge.ino.bin`, in-tree build at 04:24 | 1,180,096 |
| Official 0.6.0 app0 image (`esptool image-info`) | 1,180,064 |
| NimBLE 2.5.1 + Xbox 1.1.3, over a control that already links WiFi, Preferences and ArduinoBLE VHCI | +124,176 flash, +5,536 static RAM |
| Projected bridge with the Xbox client (±1 KB, glue code excluded) | ~1,304,272 |
| Spare in app0 | ~334,128 |
| Saved by disabling NimBLE's peripheral and broadcaster roles | 14,336 |
| Saved in addition by `MAX_CONNECTIONS=1` | 48 |

- The baseline varies by about ±512 B with the build path. A clean clone gives 1,180,608 B.
- The xbox reader's +234,464 B was measured against an empty sketch, so it double-counts WiFi and the controller. Use the core reader's figure. Even the worst case, 1,180,096 + 234,464 = 1,414,560 B, fits.
- Static RAM: the baseline is `.dram0.data` 24,916 + `.dram0.bss` 39,712 = 64,628 B. With NimBLE it is about 70 KB.
- NimBLE pools in 2.5.1 (`nimconfig.h:253-261`, `:289-307`): 12 × 256 B message buffers, 12 × 255 B ACL buffers, 30 + 8 HCI event buffers of 70 B. Runtime heap was not measured. The reader's estimate of "a few tens of KB" for controller plus host is *unverified*.

### 4.4 NVS users

The 64 KB `nvs` partition is shared by six users:
1. `initArduino()` runs `nvs_flash_init()`. It erases the whole partition on NO_FREE_PAGES or NEW_VERSION (`$CORE/cores/esp32/esp32-hal-misc.c:244`, `:248`).
2. The WiFi driver stores its config there (`CONFIG_ESP32_WIFI_NVS_ENABLED=1`).
3. PHY calibration data goes there (`CONFIG_ESP_PHY_CALIBRATION_AND_DATA_STORAGE=1`; items 2 and 3 are `qio_qspi/include/sdkconfig.h:756`, `:803`).
4. The bridge's Preferences objects `pref` (`cmds_preferences.h:7`, `:31`) and `sse` (`SSE.cpp:381`, `cmds_se.h:31`). The RA4M1 picks their namespace names over AT.
5. NimBLE's bond store, namespace `nimble_bond`, keys `peer_sec_N`, `our_sec_N`, `cccd_sec_N` and `p_dev_rec_N` (`ble_store_nvs.c:40-45`).
   - It is always on: `CONFIG_BT_NIMBLE_NVS_PERSIST 1` is hard-coded in the section marked not for editing (`nimconfig.h:276-277`).
   - The limits are 3 bonds and 8 CCCD records (`nimconfig.h:233-239`). When the store is full, the oldest entry is deleted (`NimBLEDevice.cpp:1393-1395`).
   - An API for a custom namespace exists only on NimBLE master.
6. `HCIVirtualTransport::begin` and `NimBLEDevice::init` both call `nvs_flash_init` with an `nvs_flash_erase` fallback (`NimBLEDevice.cpp:904-909`). An erase wipes the RA4M1's Preferences and SSE keys.

How full NVS gets at runtime is unknown. Keep bonds few (1–3) and avoid code paths that call `nvs_flash_erase`.

### 4.5 Physical flash: 8 MB module, 4 MB layout

- The module is the ESP32-S3-MINI-1-N8 (ABX00087 datasheet §1, §6 and BOM row "M1"). Espressif's ordering table lists N8 as 8 MB Quad-SPI flash with no PSRAM (ESP32-S3-MINI-1 datasheet v1.7). espflash prints `Flash size: 8MB` on real boards (Arduino support article 16379769332892, forum thread 1231834, bridge issue #78).
- The module has 512 kB SRAM and 384 kB ROM (datasheet p10).
- Everything in the build targets 4 MB:
  - `$CORE/boards.txt:1110` `build.flash_size=4MB` and the FQBN's `FlashSize=4M`.
  - Header byte 3 is 0x2f in both images: bootloader `e9 03 02 2f`, app `e9 05 02 2f`.
  - `combine.py:23` `outputSize = 0x400000`.
  - sdkconfig:108 `CONFIG_ESPTOOLPY_FLASHSIZE_4MB=y`, with :115 `CONFIG_ESPTOOLPY_FLASHSIZE_DETECT=y`.
  - The official 0.6.0 app header reads `Flash size: 4MB`, `Flash mode: DIO`.
- The upper 4 MB is unused. Do not repartition: there is enough headroom, and the updater and OTA assume the 4 MB layout.
- A full backup should read the detected size, 8 MB expected, not a hard-coded 0x400000.
- PSRAM: sdkconfig:1358 sets `CONFIG_SPIRAM=y`, but :1359 leaves `SPIRAM_BOOT_INIT` off, and the FQBN uses `PSRAM=disabled`. That matches the N8 hardware. `initArduino()` still calls `psramInit()` (`esp32-hal-misc.c:240-241`).
- The flash reader's low-confidence recall of "N8" is superseded by the datasheet evidence above.

## 5. The ESP32 core and its BLE options

### 5.1 Fork versus upstream 2.0.9

The core at `$CORE` is `arduino/arduino-esp32` branch `unor4wifi-2.0.9`, HEAD `a2b82168` "partition table: fix build issue on windows" (2024-11-25), a shallow clone. It targets IDF v4.4.4 (`platform.txt:55` `-DIDF_VER="v4.4.4"`).

Compared with a clean clone of `espressif/arduino-esp32` tag 2.0.9 (`3670e2bf2aca822f2e1225fdb0e0796e490005a8`):

| File | Change |
|---|---|
| `platform.txt` | `-std=gnu++11` becomes `-std=gnu++17` for esp32/s3/s2/c3 (lines 43, 58, 73, 88). New keys `build.boot_app0_addr=0xe000` and `build.app0_addr=0x10000` (145-146) replace hard-coded addresses in the upload and program patterns (270, 278). |
| `boards.txt` | New board `arduino_unor4wifi_usb_bridge` (lines 1073–1300): `bootloader_addr=0x0`, `boot_app0_addr=0x9000`, `app0_addr=0x50000`, menu `PartitionScheme.unor4wifi` (1250-1251). |
| `libraries/USB/src/USBHID.cpp:203` | String descriptor "TinyUSB HID" becomes "TinyUSB CMSIS-DAP". |
| `tools/partitions/unor4wifi.csv` | added |

- Everything else is byte-identical: `cores/esp32`, every other library, and all 8,984 tracked files under `tools/sdk`. `sdkconfig`, `lib/libbt.a` and `ld/libbtdm_app.a` compare equal.
- The 11 extra directories (`dist`, `esptool`, toolchains and so on) are untracked downloads from `get.py`.
- C compiles as `-std=gnu99`, and C++ as `-std=gnu++17 -fexceptions -fno-rtti` (`platform.txt:57-58`). NimBLE-Arduino 2.x compiles under these flags.

### 5.2 Bluetooth facts in sdkconfig

All lines are in `$CORE/tools/sdk/esp32s3/sdkconfig`. The compiled `qio_qspi/include/sdkconfig.h:550-558` agrees.

| Line | Setting | Meaning |
|---|---|---|
| 823 | `CONFIG_BT_ENABLED=y` | Bluetooth on |
| 829 | `CONFIG_BT_CTRL_MODE_EFF=1` | controller in BLE-only mode |
| 830 | `CONFIG_BT_CTRL_BLE_MAX_ACT=10` | up to 10 BLE activities |
| 835 | `CONFIG_BT_CTRL_PINNED_TO_CORE=0` | controller task on core 0 |
| 836 | `CONFIG_BT_CTRL_HCI_MODE_VHCI=y` | host reaches the controller over VHCI |
| 864 | `CONFIG_BT_CTRL_DFT_TX_POWER_LEVEL_P9=y` | default TX power +9 dBm |
| 872 | `CONFIG_BT_CTRL_BLE_SCAN_DUPL=y` | duplicate scan filtering on, by device |
| 880 | `CONFIG_BT_CTRL_COEX_PHY_CODED_TX_RX_TLIM_DIS=y` | coded-PHY coexistence time limit off |
| 886 | `# CONFIG_BT_CTRL_MODEM_SLEEP is not set` | controller modem sleep off |
| 896 | `CONFIG_BT_BLUEDROID_ENABLED=y` | Bluedroid host selected |
| 897 | `# CONFIG_BT_NIMBLE_ENABLED is not set` | IDF's NimBLE host off |
| 898 | `# CONFIG_BT_CONTROLLER_ONLY is not set` | not controller-only |
| 1114 | `CONFIG_BLE_MESH=y` | BLE mesh built |
| 1611, 2778 | `CONFIG_ESP32_WIFI_SW_COEXIST_ENABLE=y`, `CONFIG_SW_COEXIST_ENABLE=y` | WiFi/BT software coexistence on |

The S3 sdkconfig has no `CONFIG_BT_NIMBLE_*` or `CONFIG_BTDM_*` lines. Line 2560 reads `# CONFIG_NIMBLE_ENABLED is not set`.

### 5.3 Bluedroid is present but not linked

- `tools/sdk/esp32s3/lib/libbt.a` (21 MB, 353 members) holds the Bluedroid host (`bta_*`, `btc_*`, `esp_bt_main`, `esp_gattc_api`), ESP-BLE-MESH, tinycrypt and `bt.c.obj`.
- `bt.c.obj` is the controller glue: `esp_bt_controller_*` and `esp_vhci_host_register_callback`, `_send_packet` and `_check_send_available`.
- `libbt.a` defines zero NimBLE host symbols, and there is no `libnimble*.a`. The controller binary is `ld/libbtdm_app.a`, next to `ld/libbtbb.a`.
- In both the stock bridge build and the NimBLE test, the only member linked is `libbt.a(bt.c.obj)`. In the bridge, `HCIVirtualTransport.cpp.o` pulls it in through `esp_vhci_host_check_send_available` (map line 587).
- So the Bluedroid-side copies of tinycrypt, `uECC_*` and `net_buf_*` never clash with NimBLE's own.
- The link line has `-lbt -lbtdm_app -lbtbb -lcoexist -lmesh` and no `-lnimble`, all inside one `--start-group` (`platform.txt:56`, `:216`). Linker scripts: `memory.ld`, `sections.ld`, `esp32s3.rom*.ld`, `esp32s3.peripherals.ld` (`platform.txt:60`).

### 5.4 VHCI

- `$CORE/tools/sdk/esp32s3/include/bt/include/esp32s3/include/esp_bt.h` declares `esp_vhci_host_check_send_available` (`:394`), `esp_vhci_host_send_packet` (`:405`), `esp_vhci_host_register_callback` (`:412`) and `BT_CONTROLLER_INIT_CONFIG_DEFAULT()`.
- The SDK ships no `esp_nimble_hci.h`, `nimble_port*.h` or `ble_hs.h`. NimBLE-Arduino bundles its own (`src/nimble/esp_port/esp-hci/src/esp_nimble_hci.c`). It sets `CONFIG_BT_NIMBLE_LEGACY_VHCI_ENABLE` for the S3 (`nimconfig.h:352-355`).
- The IDF 4.4 controller config struct has no `dup_list_refresh_period` field (`esp_bt.h:249-259`). That is why NimBLE 2.3.0 and 2.3.1 fail to build (6.2).

### 5.5 Why NimBLE-Arduino can be dropped in

1. NimBLE-Arduino defines `CONFIG_BT_NIMBLE_ENABLED` itself (`nimconfig.h:319-321`). Nothing in 2.5.1 errors on `CONFIG_BT_BLUEDROID_ENABLED`.
2. It brings its own host and HCI glue. From the SDK it needs only the controller and VHCI in `bt.c.obj`.
3. The controller's memory survives boot. `initArduino()` frees it unless the strong `btInUse()` is linked (`esp32-hal-misc.c:261-264`, `esp32-hal-bt.c:19`).
   - NimBLE 2.5.1 calls `btStarted()` when `CONFIG_ENABLE_ARDUINO_DEPENDS && SOC_BT_SUPPORTED` (`NimBLEDevice.cpp:899-901`). Both are 1 here (`sdkconfig.h:83`, `soc_caps.h:26`).
   - The stock bridge already links `esp32-hal-bt.c.o` through `HCIVirtualTransport` (map lines 27-28).
4. NimBLE sets `ble_max_act = MAX_CONNECTIONS + BROADCASTER + OBSERVER`. That is 5 with the defaults, under the controller's 10.
5. 2.5.1 guards every IDF-5-only API (`CHANGELOG.md:119` "Build errors with arduino cores using IDF v4.x").
6. The preferred ATT MTU is 255. The sdkconfig is a Bluedroid build and leaves `CONFIG_BT_NIMBLE_ATT_PREFERRED_MTU` undefined, so NimBLE's default applies (`nimconfig.h:245-247`).
7. The NimBLE host defaults to core 0, priority 21, 4096 B stack (`nimconfig.h:212-218`).

### 5.6 Coexistence

- Software coexistence is on, and `libcoexist.a` is already linked in the bridge.
- IDF 4.4 has no `CONFIG_ESP_COEX_*` keys; that naming is IDF 5. The runtime knob is `esp_coex_preference_set(ESP_COEX_PREFER_WIFI | _BT | _BALANCE)` (`esp_coexist.h:98`).
- Arduino WiFi defaults to `WIFI_PS_MIN_MODEM` (`WiFiGeneric.cpp:764`). The bridge never calls `setSleep`, `esp_wifi_set_ps` or `esp_coex_*`.
- *Unverified* (low, recalled): with BT enabled, WiFi must stay in modem sleep, and `WiFi.setSleep(false)` is rejected. Do not add it. If BLE latency suffers, try `esp_coex_preference_set(ESP_COEX_PREFER_BT)` or `BALANCE` at runtime.
- While no controller is connected, the Xbox library scans at 100 % radio duty (6.4). That competes with WiFi. The effect on throughput was not measured.
- The S3 controller supports BLE 5 (`dio_qspi/include/sdkconfig.h:551` `CONFIG_BT_SOC_SUPPORT_5_0 1`).

### 5.7 Setting NimBLE options: `build_opt.h`

Three routes work:
1. **Sketch-folder `build_opt.h` (verified).** A prebuild hook copies `UNOR4USBBridge/build_opt.h` into the build directory (`platform.txt:185`). `@{build.opt.path}` is passed to every .c, .cpp and .S compile, libraries included (`:204`, `:207`, `:210`). The tested contents were:
   ```
   -DCONFIG_BT_NIMBLE_ROLE_PERIPHERAL_DISABLED
   -DCONFIG_BT_NIMBLE_ROLE_BROADCASTER_DISABLED
   -DCONFIG_BT_NIMBLE_MAX_CONNECTIONS=1
   ```
   They saved 14,384 B of flash and 696 B of static RAM. `build_opt.h` appears 278 times in the test's `compile_commands.json`.
2. Edit the vendored `nimconfig.h`. Its options are `#ifndef`-guarded (`:196-210`).
3. Pass `--build-property` for both `compiler.c.extra_flags` and `compiler.cpp.extra_flags` (empty at `platform.txt:129`, `:132`). NimBLE has both C and C++ files.

- Do not override `build.extra_flags`. It carries `-DESP32`, `CORE_DEBUG_LEVEL` and the `ARDUINO_USB_*` defines (`platform.txt:153`).
- A `#define` in the .ino does not reach library compile units (standard Arduino behaviour, recalled).
- Some `nimconfig.h` values are hard-coded: `NVS_PERSIST`, `SM_LEGACY`, `SM_SC`, `TRANSPORT_EVT_COUNT`. A `-D` with another value gives only redefinition warnings.
- The `nimconfig.h` comments estimate ~16 kB for the peripheral role and ~5 kB for the broadcaster role (`:116-124`). The measured saving for both was 14,336 B.

### 5.8 Build hygiene

Always use the full FQBN from `firmware/compile.sh:2` with `--config-file arduino-cli.yaml`. That file's `directories.user` is the firmware directory, so `firmware/hardware` and `firmware/libraries` are found.
```
arduino-cli compile -e --config-file arduino-cli.yaml \
  --fqbn=esp32-patched:esp32:arduino_unor4wifi_usb_bridge:JTAGAdapter=default,PSRAM=disabled,FlashMode=qio,FlashSize=4M,LoopCore=1,EventsCore=1,USBMode=default,CDCOnBoot=default,MSCOnBoot=default,DFUOnBoot=default,UploadMode=default,PartitionScheme=unor4wifi,CPUFreq=240,UploadSpeed=921600,DebugLevel=none,EraseFlash=none \
  -v UNOR4USBBridge
```
- An omitted menu option falls back to the first menu entry. That gives `PartitionScheme=default`, a 1.2 MB app with the wrong table (`boards.txt:1213`). It also gives `USBMode=hwcdc` (`boards.txt:1186`), but the bridge needs TinyUSB (`USBMode=default`, `usb_mode=0`).
- A `partitions.csv` or `bootloader.bin` in the sketch folder silently overrides the table or the bootloader (`platform.txt:170`, `:180`). Neither exists today. Do not add one.
- Build into a scratch build path, not the running in-tree build.
- The installed arduino-cli is 1.5.1 (Homebrew).

### 5.9 Link test (verified)

The core reader built these in `$SCR/xb_linktest` with the exact FQBN against the symlinked core. The fork's `git status` stayed clean.

| Sketch | Contents | .bin bytes | Static RAM | Result |
|---|---|---|---|---|
| XbControl | WiFi + Preferences + ArduinoBLE 1.3.6 `HCIVirtualTransport` | 747,776 | 45,128 | exit 0 |
| XbLinkTest | control + NimBLE 2.5.1 + Xbox 1.1.3 + parser + report builder | 871,952 | 50,664 | exit 0, no warnings, no multiple definitions |
| XbRoles | XbLinkTest + the two role-disable flags | 857,616 | 50,424 | exit 0 |
| XbOpt | XbRoles + `MAX_CONNECTIONS=1` | 857,568 | 49,968 | exit 0 |

- arduino-cli printed `Sketch uses 871585 bytes (52%)` for XbLinkTest.
- The NimBLE copy is byte-identical to tag 2.5.1. The ArduinoBLE copy is byte-identical to `firmware/libraries/ArduinoBLE` (1.3.6).
- Of the 1,296 symbols XbLinkTest adds, only 24 (975 B) already exist in the bridge ELF. So the delta adds to the bridge within about 1 KB.

The xbox reader's separate harness (`$SCR/ctest`) gave these results, all exit 0:

| Sketch | .bin bytes | Static RAM |
|---|---|---|
| empty sketch | 255,885 | 19,480 |
| Xbox client (NimBLE 2.5.1) | 490,349 | 30,004 |
| Xbox client + ArduinoBLE VHCI | 534,681 | 31,652 |
| Xbox client on NimBLE 2.3.9 | 486,933 | — |
| Xbox client with the debug macro defined | 498,673 | — |

## 6. The Xbox library and NimBLE-Arduino

### 6.1 Pins

| Library | Version | Commit | Licence |
|---|---|---|---|
| NimBLE-Arduino | 2.5.1 | `5ae2c43f62c5a588a25c1de68f7d17b6f3b75adb` | Apache-2.0 |
| NimBLE-Arduino (fallback) | 2.3.9 | `5ffd4f12905518b17236fe2c3b401bb126745eb8` | Apache-2.0 |
| XboxSeriesXControllerESP32_asukiaaa | 1.1.3 | `00234325be391543e7df914fc85a82bcb274c045` | MIT |
| XboxControllerNotificationParser | 1.0.5 | `77a449d77970166a01c25ef7c26c084a42c3a5d6` | MIT |
| XboxSeriesXHIDReportBuilder_asukiaaa | 1.0.1 | `4afd097b198097d2b66f892529488b7c9f8a2898` | MIT |

- The 2.5.1 tag is on `origin/release/2.5`, not on master. Fetch it explicitly with `git clone --depth 1 --branch 2.5.1`.
- The Xbox library's `library.properties` says `depends=XboxControllerNotificationParser,NimBLE-Arduino,XboxSeriesXHIDReportBuilder_asukiaaa`. It pins no NimBLE version.

### 6.2 NimBLE API generation and which releases work

Xbox 1.1.3 uses only the NimBLE 2.x API, with no version guards. Examples from hpp:
- `NimBLEScanCallbacks` with `onResult(const NimBLEAdvertisedDevice*)` (`:84`, `:114`) and `onScanEnd(const NimBLEScanResults&, int)` (`:156`).
- `onDisconnect(NimBLEClient*, int)` (`:53`), `onPassKeyEntry(NimBLEConnInfo&)` (`:63`), `onConfirmPasskey(NimBLEConnInfo&, uint32_t)` (`:67`), `onAuthenticationComplete(NimBLEConnInfo&)` (`:73`).
- `setScanCallbacks` (`:270`), `NimBLEUtils::dataToHexString` (`:128`), `getCreatedClientCount` (`:344`), `NIMBLE_MAX_CONNECTIONS` (`:353`).

The README says: "For NimBLE 2.1.0 or more, use version 1.1.0 or more of this library. For NimBLE less than 2.1.0 (1.4.3 or less), use version 1.0.9 of this library" (`README.md:14-15`). There is no NimBLE 2.0.x release; the tags jump from 1.4.3 to 2.1.0 (2024-12-14).

| NimBLE-Arduino | Status with Xbox 1.1.3 on this core | Reason |
|---|---|---|
| 1.x (1.4.3 tested) | fails | pre-2.x callback API. `compile_1.4.3.log`: `onDisconnect(NimBLEClient*, int)' marked 'override', but does not override`, `'injectPassKey' is not a member of 'NimBLEDevice'` |
| 2.2.0–2.2.2 | avoid | bug in `connect(NimBLEAdvertisedDevice*)`, the overload the library uses; fixed in 2.2.3 |
| 2.2.3 | valid by the reader's analysis, not compile-tested | — |
| 2.3.0, 2.3.1 | fails | `NimBLEDevice.cpp:920: 'struct esp_bt_controller_config_t' has no member named 'dup_list_refresh_period'` (IDF 4.4). Commit `971e5df5` "Fix build with idf versions < 5.x" fixed it, first released in 2.3.2. |
| 2.3.2–2.5.1 | valid; 2.3.9 and 2.5.1 compile-tested | 2.5.1 guards IDF-5-only APIs |
| master 3.0.0-dev | fails | commit `ceeb21aa` (2025-09-05) removed `NIMBLE_MAX_CONNECTIONS`: `hpp:353:52: error: 'NIMBLE_MAX_CONNECTIONS' was not declared in this scope` |

Pin rule: NimBLE 2.x (≥ 2.1.0, < 3.0), specifically 2.5.1. The reader's valid range is 2.2.3 and 2.3.2–2.5.1.

### 6.3 What `begin()` does

`Core::begin()` (hpp:186-192) makes exactly these calls:
```cpp
NimBLEDevice::init("");
NimBLEDevice::setOwnAddrType(BLE_OWN_ADDR_PUBLIC);
NimBLEDevice::setSecurityAuth(true, false, false);   // bonding on, MITM off, Secure Connections off
NimBLEDevice::setPower(ESP_PWR_LVL_P9);              // the source comment says +9 dB; the result is +12 dBm
```
- It never calls `setSecurityIOCap`, so the IO capability stays at NimBLE's default, NoInputNoOutput (`NimBLEDevice.cpp:996-999`). The result is legacy "Just Works" pairing with bonding. NimBLE 2.1.0 defaults to `sm_sc = 1`, but `begin()` overrides it.
- It never calls `deleteAllBonds`, `setMTU` or `secureConnection`.
- `setOwnAddrType(PUBLIC)` changes nothing in practice, because public is already NimBLE's default (`NimBLEDevice.cpp:102`). The ESP32's identity therefore stays stable across reboots. That the controller's stored bond depends on this is inferred, not verified.
- **TX power is +12 dBm, not +9.** NimBLE 2.x `setPower(int8_t dbm)` takes dBm (`NimBLEDevice.h:162`). `ESP_PWR_LVL_P9` is the enum value 11 on the S3 (`esp_bt.h:312`). NimBLE rounds 11 up to 12 and selects `ESP_PWR_LVL_P12` for advertising, scanning and connections (`NimBLEDevice.cpp:512-521`). The library meant `setPowerLevel()`.
- If the controller is already initialised, for example by `AT+HCIBEGIN`, `NimBLEDevice::init` returns false and `begin()` ignores it.

### 6.4 What `onLoop()` does, and why it blocks

Scanning returns quickly (hpp:269-277):
- `setDuplicateFilter(false)`, then `setScanCallbacks(cb)` with `wantDuplicates=false`, so `onResult` fires once per device per scan.
- `setInterval(97)` and `setWindow(97)` give 100 % radio duty. `start(4000)` is asynchronous.
- The scan is passive. `setActiveScan(true)` is commented out (hpp:271), and NimBLE's default is passive (`NimBLEScan.cpp:73`).
- Scans repeat back to back until a controller connects.

Connecting (`connectToServer`, hpp:340-418) is synchronous:
1. The library reuses an existing client with `getClientByPeerAddress(addr)->connect()` (hpp:344-348). Otherwise it calls `createClient()`, `setClientCallbacks(cb, true)` and `connect(advDevice, true)` (hpp:361-377). It returns false if the client count already equals `NIMBLE_MAX_CONNECTIONS` (hpp:353).
2. The first `connect()` return value is ignored on both paths.
3. It retries up to 3 times, each with `delay(100)` then `connect(true)` (hpp:299-300, :381-397).
4. Inside each `connect()`, NimBLE 2.5.1 retries a 0x3E connection-establishment failure up to 2 more times (`connectFailRetries` default 2; `NimBLEClient.h:121-129`, `NimBLEClient.cpp:1062-1075`).
5. The connect timeout is 30 s (`NimBLEClient.cpp:69`). The wait is `(30000 + 7 × itvl_max) × (connectFailRetries + 1)` ms.
6. The MTU exchange is requested on connect but not awaited (`NimBLEClient.cpp:1128-1130`). The MTU used is the smaller of 255 and the controller's value.
7. In 2.5.1, `onConnect` runs before `connect()` returns, so the library's `isConnected()` check right after `connectToServer` is valid.

After connecting (`afterConnect`, hpp:420-439):
- It copies the peer MAC into the public `deviceAddressArr` (hpp:421-422).
- It calls `getServices(true)` and keeps only HID 0x1812 and Battery 0x180F (hpp:423-425).
- For each characteristic, it calls `readValue()` if readable, with the comment "Reading value is required for subscribe" and a second read if the value is empty (hpp:466-475). It then calls `subscribe(true, notifyCB, true)` if the characteristic has the NOTIFY bit (hpp:483-493). That writes the CCCD (descriptor 0x2902) with a response.
- Indicate-only characteristics are skipped. A failed subscribe is ignored, and `afterConnect` always returns true. If there is no CCCD, the callback is set without a write.
- No handle is hard-coded. Any notifiable characteristic in the HID service feeds the parser, with no UUID or report-ID check.
- Every GATT read and subscribe waits with `BLE_NPL_TIME_FOREVER` (`NimBLERemoteValueAttribute.cpp`). That the host times GATT procedures out is recalled.

On any failure the library deletes the bond: `NimBLEDevice::deleteBond(advDevice->getAddress()); ++countFailedConnection;` (hpp:238-244; commit `77dc3f4` "Delete bond if failed connecting", 2022-06-11).

Consequence: `onLoop()` can block for seconds to tens of seconds. It must run in its own FreeRTOS task, never in `loop()`, `atLoop` or `addTask()`.
- The upstream receiver example runs it in a task pinned to core 1, priority 1, 4096 B stack, with `delay(10)` (`$SRC/arduino-ControllerAsI2c/examples/slave_target/esp32/wireless-xbox-series-x/wireless-xbox-series-x.ino:88-89`, `:154`, `:197-198`).
- Use 6–8 KB of stack and check `uxTaskGetStackHighWaterMark`. ESP-IDF stack sizes are in bytes.

Minimal code that compiled (`$SCR/ctest/sketch/XboxTest/XboxTest.ino`):
```cpp
#include <XboxSeriesXControllerESP32_asukiaaa.hpp>  // in exactly ONE .cpp
static XboxSeriesXControllerESP32_asukiaaa::Core xbox;  // or Core("aa:bb:cc:dd:ee:ff")
static void xboxTask(void*){ for(;;){ xbox.onLoop(); vTaskDelay(pdMS_TO_TICKS(10)); } }
void xboxBegin(){ xbox.begin(); xTaskCreatePinnedToCore(xboxTask,"xbox",8192,nullptr,1,nullptr,1); }
bool xboxRead(uint8_t out[16]){
  if(!xbox.isConnected()||xbox.isWaitingForFirstNotification()) return false;
  xbox.xboxNotif.toArr(out,16); return true; }
// also: xbox.battery (percent), xbox.getReceiveNotificationAt() (millis), xbox.getCountFailedConnection()
```

### 6.5 Notifications and thread safety

- `notifyCB` runs in the `nimble_host` task (core 0, priority 21).
- Any notification, battery included, first sets the state to Connected (hpp:509-515). `onConnect` had set WaitingForFirstNotification (hpp:45-49).
- HID-service data goes to `xboxNotif.update(pData, length); receivedNotificationAt = millis();` (hpp:540-541). Battery data sets `battery = pData[0]` with no length check (hpp:548-550).
- There is no lock. Each 16-bit field is written whole, so only tearing across fields is possible, which is benign for a gamepad.
- For consistent snapshots, patch the vendored header. Wrap the update and the reader's `toArr` copy in the same `portMUX` critical section. `NimBLEClient` callbacks and `notifyCB` are private and cannot be overridden.

### 6.6 Pairing, bonding and reconnection

- The library never starts security itself. NimBLE does:
  - When a read or the CCCD write returns Insufficient Authentication or Insufficient Encryption, NimBLE calls the blocking `secureConnection()` and retries (`NimBLERemoteValueAttribute.cpp:76-79`, `:155-158`). The read in `afterConnect` is what triggers pairing.
  - On `PINKEY_MISSING`, NimBLE deletes the stale peer record and pairs again (`NimBLEClient.cpp:1282-1287`).
- `onAuthenticationComplete` disconnects if the link is not encrypted (hpp:73-79).
- Whether the controller sends a Security Request on connect is *unverified*.
- Key distribution is ENC, plus ID when `BLE_LL_CFG_FEAT_LL_PRIVACY` is 1, which is the Arduino default (NimBLE master `NimBLEDevice.cpp:1000-1004`, `syscfg.h:1895-1896`).
- Reconnection (medium; not tested on hardware):
  - `onDisconnect` sets the state to Scanning and clears the client pointer (hpp:53-61). `onLoop()` then restarts a 4 s scan.
  - A bonded controller that wakes advertises `060000`. The library reuses the client, reconnects, rediscovers and resubscribes on every connection.
  - Bond keys stay in NVS on both sides. No re-pair should be needed after controller sleep, controller power-cycle or ESP32 reboot, as long as the address stays public.
- Caveat: every failed connect deletes the bond. Whether the controller then accepts a fresh pairing while not in pairing mode is unknown.
- The examples recover by calling `ESP.restart()` after more than 2 failures (`examples/connect/connect.ino:35-37`). The bridge must not copy this. Use `getCountFailedConnection()` for status reporting only.
- The bond table is not a reliable record. Store the controller's full `ble_addr_t` (type and value) in the bridge's own Preferences namespace.
- Verifier design notes: treat a subscription as valid only after `onAuthenticationComplete` reports `isEncrypted()`. Do not delete bonds on transient failures.

### 6.7 How the controller is identified

The constants are at hpp:27-29: appearance 964 (0x03C4, gamepad), manufacturer data `060000` (normal) and `0600030080` (pairing mode), HID service UUID 0x1812. The match is in `ScanCallbacks::onResult` (hpp:131-142). There are three cases:

| Case | When | What is accepted |
|---|---|---|
| (a) | no target set (`targetDeviceAddress == nullptr`) | appearance 964, **and** first manufacturer-data field as lowercase hex exactly `060000` or `0600030080` (company ID 0x0006 included), **and** first advertised service UUID 0x1812 |
| (b) | a target is set | the advertised address equals the target, address type included (NimBLE ≥ 2.1.0 `equals()`; `NimBLEAddress.cpp:213-218`, `CHANGELOG.md:285`) |
| (c) | a target is set and `setIgnoreTargetAddressToConnectToUnkownSearchingDevice(true)` | also any device whose first manufacturer data is exactly `0600030080`, with no appearance or UUID check |

- Case (a) needs all three fields in the advertising packet itself, because the scan is passive.
- `Core("aa:bb:cc:dd:ee:ff")` builds the target as type 0, public (hpp:89-90). `setTargetAddress(ble_addr_t)` keeps the caller's type (hpp:97-102).
- A string that is not 17 characters gives `00:00:00:00:00:00` and logs "Invalid address" (`NimBLEAddress.cpp:79-80`). The pointer is still non-null, so only case (c) can ever match.
- The library keeps the peer address in RAM (hpp:421-422, public via `buildDeviceAddressStr`, hpp:256-262). It never saves or reloads a target.
- ControllerAsI2c saves the address in EEPROM (`wireless-xbox-series-x.ino:159-171`, `:222-246`). Its line 224 feeds LSB-first bytes to a byte-reversing constructor, so its comparison likely never matches (*unverified* on hardware).
- A target must be the controller's BLE address, not the string printed by `buildDeviceAddressStr` (asukiaaa issues #15, #16).
- In default mode the library connects to the first matching controller it sees.

### 6.8 Report layout

16 bytes; 16-bit fields are little-endian; there is no report-ID byte. Source: `XboxControllerNotificationParser.cpp:3-6`, `:23-51`, `.h:28-30`.

| Byte(s) | Field | Range or encoding |
|---|---|---|
| 0–1 | `joyLHori` | 0 = left, 65535 = right |
| 2–3 | `joyLVert` | 0 = up, 65535 = down |
| 4–5 | `joyRHori` | 0 = left, 65535 = right |
| 6–7 | `joyRVert` | 0 = up, 65535 = down |
| 8–9 | `trigLT` | 0–1023 (`maxTrig` 0x3FF) |
| 10–11 | `trigRT` | 0–1023 |
| 12 | d-pad | 0 none, 1 N, 2 NE, 3 E, 4 SE, 5 S, 6 SW, 7 W, 8 NW |
| 13 | buttons | bit0 A, bit1 B, bit3 X, bit4 Y, bit6 LB, bit7 RB (bits 2 and 5 unused) |
| 14 | buttons | bit2 View/Select, bit3 Menu/Start, bit4 Xbox, bit5 LS click, bit6 RS click |
| 15 | buttons | bit0 Share |
| 16 (Elite 2 only) | profile | — |
| 18 (Elite 2 only) | paddles | bits 0–3 = P1–P4 |

- The d-pad helpers are: Up = value in {1, 2, 8}; Right = 2–4; Down = 4–6; Left = 6–8.
- Sticks start at 32767 (0xFFFF / 2), so centre is nominally 32767 or 32768. Triggers start at 0 and buttons at false.
- Parser API (`.h:24-26`):
  - `uint8_t update(uint8_t* data, size_t length)` returns 0. If `length < 16` it returns `XBOX_CONTROLLER_ERROR_INVALID_LENGTH` (1) and leaves the fields untouched (`.cpp:17-20`).
  - `uint8_t toArr(uint8_t* data, size_t length)` writes the same 16-byte layout back, re-encoding the d-pad.
  - `String toString()`. There is no `parse()`.
- Parser 1.0.4 rejected the Elite 2's 19-byte reports silently (`length != 16`). Version 1.0.5 accepts any length of 16 or more.

### 6.9 Payload for the RA4M1 (open design choice)

The readers disagree:
- Link reader: a fixed 20 B payload with a "connected" flag and a sequence counter.
- Xbox reader: ControllerAsI2c-style. The 16 B `toArr` report, connection state, battery %, an 8-bit sequence counter and CRC16 make 21 B, or 27 B with the MAC.

The design must pick one fixed length N other than 2 or 5. The RA library must assert `res.size() == N`. A workable 20 B layout (critic): the 16 B report, state, battery, sequence, and an XOR or CRC8 checksum.

ControllerAsI2c, for reference (`$SRC/arduino-ControllerAsI2c`, MIT):
- I2C address 0x30 (`ControllerAsI2c_asukiaaa.hpp:6`).
- A 7-byte header at register 0: [0] controller type (1 = Xbox Series X), [1] receiver type (1 = esp32), [2] read-only length, [3] writable length, [4] unused, [5–6] CRC16 big-endian.
- A 27-byte read-only block at register 7: [0–5] MAC, [6–21] report in `toArr` format, [22] connection state (0 Connected, 1 WaitingForFirstNotification, 2 FoundButNotConnected, 3 NotFound, 9 Unknown), [23] battery %, [24] request counter, [25–26] CRC16 big-endian.
- An 11-byte writable block at register 34: 8 B rumble report, 1 sequence byte, CRC16.
- The CRC is `crcx::crc16` from the CRCx library; its variant was not inspected (`Common.hpp:23-27`).
- Its `fromArr` never copies the address (`XboxSeriesX.hpp:44` `memcpy(dataArr, dataArr, lenAddress)`). Do not copy that bug.
- The example converts sticks to −1..+1 with vertical inverted (`XboxSeriesX.hpp:86-89`). Do no float conversion on the ESP32 side.

### 6.10 Library bugs to avoid or patch

| # | Bug | Where | Action |
|---|---|---|---|
| 1 | `clearTargetAddress()` deletes the target but does not null the pointer. Case (a) never returns, `onResult` reads freed memory, and a later `setTargetAddress` writes through it. | hpp:105-109 | do not call it, or patch it to set `nullptr` |
| 2 | `onAuthenticationComplete` dereferences `getClientByHandle(...)` without a null check | hpp:77 | patch |
| 3 | The Elite 2 parser's `update()` always returns 0, even on a bad length | `XboxControllerElite2NotificationParser.cpp:7`, `:28` | do not trust its return value |
| 4 | The debug macro prints to its target and drops HID notifications that arrive within 100 ms of a print | hpp:8, :10, :520 | never define it |
| 5 | `deleteBond` after every failed connect | hpp:238-244 | persist the address in Preferences; consider a patch |
| 6 | `setPower(ESP_PWR_LVL_P9)` gives +12 dBm | hpp:191 | note it; patch optional |
| 7 | The header defines `static` namespace-scope globals (UUIDs, found-device and client pointers) | hpp:13-25 | include the header in exactly one .cpp and expose wrapper functions |
| 8 | `begin()` ignores a `NimBLEDevice::init` failure | hpp:188 | the glue checks controller state first |
| 9 | `battery = pData[0]` without a length check | hpp:548-550 | minor |
| 10 | The examples call `ESP.restart()` after repeated failures | `examples/connect/connect.ino:35-37` | do not copy |

Record every patch to the vendored copy in `firmware/UPSTREAM.md`.

### 6.11 History and other references

- Xbox library releases:
  - 1.1.0 (2025-01-11) ported the library to NimBLE 2.1.3.
  - 1.1.1 (2025-05-10) added `getReceiveBatteryAt()` and zeroes the timestamps while disconnected.
  - 1.1.2 (2026-08-01) fixed the stored MAC. `getBase()` returns a pointer to `ble_addr_t {type; val[6]}`, so 1.1.1 had copied the type byte and 5 address bytes.
  - 1.1.3 (2026-09-10) added `setTargetAddress(ble_addr_t)`, `clearTargetAddress()` and `setIgnoreTargetAddressToConnectToUnkownSearchingDevice(bool)`.
  - No release after 1.1.0 changed the security settings or the NimBLE generation.
- Security history: `1d79fbe` (2022-01-29) used a random address and `setSecurityAuth(true,true,true)`. `27b71ee` (2022-06-11) switched to a public address. `869221f` (2022-07-18) "Use lower securily authentication" switched to `(true,false,false)`.
- The upstream receiver example (last changed 2026-09-20) calls `esp_task_wdt_init(WDT_TIMEOUT, true)`, the IDF 4.x signature (`wireless-xbox-series-x.ino:177`). This suggests the author also builds on arduino-esp32 2.x with NimBLE 2.x. That IDF 5 changed this signature is recalled.
- `asukiaaa/esp32-client-for-xbox-controller-with-nim-ble` is a 2021 PlatformIO proof of concept:
  - unpinned NimBLE (`platformio.ini:15-16`), the 1.x API, first connected with NimBLE 1.1.0;
  - random own address and `setSecurityAuth(true,true,true)` (`src/main.cpp:268-271`);
  - connection parameters 12, 12, 0, 51 (15 ms interval, 510 ms supervision timeout) and a 5 s connect timeout (`main.cpp:226-227`);
  - a vendor GATT service `00000001-5f60-4c4f-9c83-a7953298d40d` that the library ignores (`main.cpp:16`).
  - It has no LICENSE file: reference only.

## 7. Existing projects and alternatives

### 7.1 No reusable fork

- `arduino/uno-r4-wifi-usb-bridge` has 19 forks (GitHub API, 2026-10-11). On their main branches only two are ahead of upstream:
  - Virus-V, 3 commits: a BL616CL port, RA4 SWD/UART handling, and holding the RA4M1 in reset. Its only change to `cmds_ble_bridge.h` is a trailing newline.
  - zmanu04/uno-r4-wifi-usb-bridge-ESP-NOW, 3 commits: "try 1", "it worked", "Added UNO_R4_ESP_NOW wrapper library".
- Some non-default branches are ahead of upstream. None is BLE work:
  - Chaxiraxi `copilot/add-esp-now-support-implementation-plan`: a second ESP-NOW fork (`cmds_espnow.h`, `espnow_manager.h`, "radio-mode mutual exclusion").
  - dvanbler `increased-serial-rate` (921600 baud) and matthieuvigne `at_set_baudrate`.
  - paulvha `Ping` and `gettime`.
  - andreagilardoni `baud-autodetect`, `file_download`, `map-out`, `prerelease-0.4.2`, `tls-certs`.
  - pennam `certificates`, `esp_3.x.x`, `OTA`, `SSE` and partition branches.
- Two non-fork copies (`sh2o-1/Update-firmware`, `frassix/GLACIER_muon_telescope`) contain no NimBLE, Bluepad32 or Xbox files.
- Repo and code searches combining the bridge with NimBLE, Bluepad32, Xbox, gamepad or BLE controller returned nothing.
- Limits: GitHub code search indexes default branches and mostly skips forks, and later queries hit the rate limit. The search evidence supports the conclusion. The fork and branch checks are exhaustive.

### 7.2 The zmanu04 ESP-NOW precedent

It has the same shape as our design: a new AT command family in the bridge plus an RA4M1 wrapper library.
- Bridge side: `UNOR4USBBridge/cmds_esp_now.cpp` (+116 lines), `cmds_esp_now.h` (+6), and 2-line hooks in `at_handler.cpp` and `at_handler.h`.
- RA4M1 side: `UNO_R4_ESP_NOW/UNO_R4_ESP_NOW.cpp` and `.h`.
- It also commits build binaries and edits `compile.sh`.
- It has no licence file, so it is a pattern to follow, not code to copy.

### 7.3 Maintainer statements

- Bluepad32 (`ricardoquesada/bluepad32-arduino#10`, open): R4 support is "low priority", and "a new platform should be created".
- asukiaaa (`arduino-XboxSeriesXControllerESP32#10`, 2024-08-30): "NimBLE arduino does not support Arduino Uno R4 WiFi so this library also does not support the Uno R4 WiFi". Issue #11 suggests a separate ESP32 running ControllerAsI2c.
- h2zero (`NimBLE-Arduino#621`, closed 2026-03-17): "I do not have time to implement the communication layer on this board. You could flash the esp32 on it directly". This fork does that while keeping the bridge.

### 7.4 Forum attempts with stock ArduinoBLE

- Users connected an R4 WiFi to an Xbox controller with ArduinoBLE. They discovered HID 0x1812 and report characteristics 0x2A4D, but none got input.
- KurtE (2023-09-04): "Still need to figure out how to logically pair the XBox controller to the UNO." jjoravec (2024-02) connected, but attribute discovery failed. No thread has a resolution.
- ArduinoBLE issue #276 (open): `discoverAttributes` fails or loops on an Xbox controller. A 2024-01-16 comment confirms it on the UNO R4 and says KurtE's fork fixes it. It was not re-tested on 2.1.0.

### 7.5 Why stock ArduinoBLE on the RA4M1 cannot work

Source: ArduinoBLE 2.1.0 at `281377b` (tag and master; "Publish 2.1.0" dated 2026-06-22), cloned to `$SRC/ArduinoBLE`. Its Security Manager (SMP, the BLE pairing protocol) only answers as a peripheral:
- `src/utility/L2CAPSignaling.cpp:136` `if (code == CONNECTION_PAIRING_REQUEST) {` is the responder path. Every SMP send in the library is a responder reply.
- `L2CAPSignaling.cpp:264-266` `else if (code == CONNECTION_PAIRING_RESPONSE) { }` is empty.
- `L2CAPSignaling.h:39` `#define CONNECTION_SECURITY_REQUEST 0x0B` is defined and never handled.
- `HCI.h:34-42` lists the LE commands: `ENCRYPT 0x0017`, `RANDOM 0x0018`, `LONG_TERM_KEY_REPLY 0x001A`, `LONG_TERM_KEY_NEGATIVE_REPLY 0x001B`, `READ_LOCAL_P256 0x0025` and `GENERATE_DH_KEY`. LE Start Encryption (0x0019, opcode 0x2019) appears nowhere.
- `HCI.cpp:1179` handles only the LTK Request event, which is peripheral-side.
- `ATT.cpp:672-689` records an ATT error such as 0x05 Insufficient Authentication but never starts pairing.
- `src/local/BLELocalDevice.h:90-109` offers only `setPairable`, `pairable`, `paired` and `setBinaryConfirmPairing`. `BLEDevice.h` has no pair API. Bond storage goes only through app callbacks (`setStoreIRK`, `setGetIRKs`, `setStoreLTK`, `setGetLTK`).
- PR #156 (merged 2022-05-13) says: "The branch supports encryption instigated by the central when the arduino is the peripheral."
- Release notes 1.3.0–2.1.0 add no central pairing. None of the 21 open PRs adds it, and neither does the `multi-connection` branch.

Why encryption matters:
- HOGP (HID over GATT profile) v1.2 §8.1: "HID Service characteristics shall require an encrypted link for reading, writing, and notification", and the HID Device "shall be in the Bondable mode".
- §8.2: the HID Host "shall perform the Bonding procedure with the HID Device" and "should encrypt the link as early as possible after reconnection".
- Battery, Device Information and Scan Parameters characteristics only "should" use the same security level.
- That the Xbox pad itself enforces this is inferred from the spec and library behaviour (*unverified*; no primary test was found).

Verifier corrections:
- The gap is in the RA4M1's host stack, not in the ESP32. The bridge passes raw HCI to the controller unfiltered (`cmds_ble_bridge.h:103-155`; `HCIVirtualTransport.cpp:53` `esp_vhci_host_send_packet`). In principle, a patched ArduinoBLE could work with no ESP32 change.
- That patch would need an SMP initiator written from scratch, LE Start Encryption, LTK storage and key-distribution handling.
- The crypto is the small part. `btct.h:11-17` already declares `AES_CMAC`, `f5`, `f6`, `g2` and `ah`; `f4` is computed inline; `AES_128` runs through HCI LE Encrypt (`btct.cpp:267-271`). Legacy `c1` and `s1` would be a few lines each.
- This is substantial new code, not glue, so it is rejected.
- Novel Bits' phrase "encryption is mandatory for any kind of Characteristics interaction" paraphrases HOGP 1.0 and overstates it; GATT discovery is not covered.
- NimBLE-Arduino issue #172 is not evidence for the encryption requirement. Its fix was identity-key distribution and own-address handling.

### 7.6 Why Bluepad32 is out

- Bluepad32 needs the BTstack host. It ships either as its own board package or as an ESP-IDF template with the controller only (`CONFIG_BT_CONTROLLER_ONLY=y`, template `sdkconfig.defaults:15-16`). It is not a plain library for stock arduino-esp32 2.0.x.
- Board package `esp32_bluepad32` 4.1.0 = Bluepad32 4.1.0 + BTstack 1.6.1 + arduino-esp32 2.0.17. Version 4.0.4 was built on 2.0.15.
- Our core has Bluedroid enabled and `CONFIG_BT_CONTROLLER_ONLY` off (sdkconfig:823, :896, :898). Switching cores would replace the bridge's whole build base.
- Licence: Bluepad32 is Apache-2.0, but BTstack is free only for open source. Closed-source ESP32 use requires contacting BlueKitchen.
- Bluepad32 5.0.0-beta0 (2026-10-05) dropped the NINA platform and turned LE Secure Connections off by default.
- Bluepad32 remains a useful reference for Xbox security behaviour (section 9).

## 8. Flashing facts

No device was connected or opened during recon. The agents must never flash. These facts feed the human's runbook.

### 8.1 Entering ESP32-S3 download mode

**Path 1: HID feature report 0xAA, from the PC, no jumper.**
- Host side: `firmware/unor4wifi-reboot/main.go` enumerates or opens HID 0x2341:0x1002 (`:40`, `:47`) through the vendored `pennam/hid@931f677`, a karalabe/hid fork over hidapi (`go.mod:5`).
- It sends a 65-byte feature report `[0x00, 0xAA, 0, ...]` (`main.go:37`, `:57-59`). hidapi strips the 0x00 report ID, so the wire carries SET_REPORT(Feature, ID 0) with 64 bytes starting 0xAA. No serial or baud code is involved.
- Device side: `USBHID.cpp:266-276` routes the SET_REPORT to `tinyusb_on_set_feature`. The DAP descriptor declares no report ID, so ID 0 matches the DAP device (`USBHID.cpp:87-94`, `:120`, `:127`).
- `DAP.h:71-75` checks `buffer[0] == 0xAA` and calls `usb_persist_restart(RESTART_BOOTLOADER)`.
- `usb_persist_restart` (`esp32-hal-tinyusb.c:488-498`) registers a shutdown handler, calls `usb_switch_to_cdc_jtag()` on the S3, then calls `esp_restart()`.
- `usb_switch_to_cdc_jtag()` (`:402-452`) resets and disables the USB-OTG module. It hands the PHY back to the built-in USB-Serial/JTAG controller and drives D− and D+ low to force a bus reset. It waits up to 1000 ms; on timeout it logs `reset_sem timeout` and carries on.
- The shutdown handler writes `RTC_CNTL_OPTION1_REG = RTC_CNTL_FORCE_DOWNLOAD_BOOT` (`:461-471`). `usb_persist_enabled` is false (`:383`; its setter at `:566` is commented out), so no OTG persist flag is set. The ROM boots into download mode on USB-Serial/JTAG.
- The updater waits 1 s on macOS and Linux, 5 s on Windows, after the reboot (`update_mac.command:15`, `update_windows.bat:15`).

**Path 2: `AT+RESTARTBOOTLOADER` from the RA4M1.** It reaches the same function (`cmds_esp_generic.h:33-37`, registered through `at_handler.cpp:143`; `commands.h:25`). It arrives on Serial1, so only an RA4M1 sketch can send it. Stock WiFiS3 never does.

**Not paths into S3 download mode:**
- The 1200-baud touch sets `GPIO_BOOT` (9) high and pulses `GPIO_RST` (4) twice. That is an RA4M1 double reset, the same as `BossaUnoR4WiFi::setProgramMode` (`UNOR4USBBridge.ino:227-235`; `BossaUnoR4WiFI.cpp:84-93`).
- The 2400-baud touch sets `GPIO_BOOT` low and pulses reset once. BOOT stays low until a later 1200 touch (`UNOR4USBBridge.ino:237-243`).
- Both touches also call `WiFi.disconnect()`, drain UART0 RX, skip forwarding the baud to UART0, and pause `atHandler.run()` (`:114-116`, `:247-249`). Neither restarts the S3.
- The core's CDC reboot, by 1200 baud or by the DTR/RTS sequence, is disabled by `enableReboot(false)` (`:151`; `USBCDC.cpp:203-233`, `:257`). esptool's DTR/RTS reset on 0x2341:0x1002 therefore cannot reach S3 download mode.
- DFU detach is unreachable, because `USB.enableDFU()` is commented out (`:148`).

**Precondition for path 1.** A bridge firmware must be running with TinyUSB, the DAP HID device and the 0xAA handler.
- A `DEBUG_AT` build lacks `DAP.begin()`, which sits in the `#else` branch (`:149`).
- If the fork crashes before `USB.begin()`, or boot-loops too fast to enumerate, recovery needs the jumper.
- Mask-ROM download mode itself cannot be bricked by a bad flash.
- If `Open` fails after `Enumerate` succeeds, `unor4wifi-reboot` panics with exit 2 instead of exiting 1 (`main.go:53`).
- Whether the host sees success depends on TinyUSB finishing the status stage before the reset (*unverified*).

**Jumper path.** Short GND to Download on the 6-pin header next to the USB-C connector, then plug the board in. The short can be released once the board is connected.

### 8.2 USB identities

| Mode | VID:PID | Identity |
|---|---|---|
| Normal bridge | 0x2341:0x1002 | "Arduino" / "UNO WiFi R4 CMSIS-DAP". A TinyUSB composite of one CDC port and one vendor HID interface (CMSIS-DAP v1 plus the 0xAA report). Serial number is the MAC (`USB.cpp:38`). |
| ROM download | 0x303A:0x1001 | Espressif built-in USB-Serial/JTAG (`$CORE/boards.txt:1075-1076`; esptool `USB_JTAG_SERIAL_PID = 0x1001`) |

- The name "Espressif USB JTAG/serial debug unit" comes from ArduinoCore-renesas `bootloaders/UNO_R4/README.md:54-55`. The updater README says that on Windows espflash shows only a generic COMx port (`unor4wifi-updater/README.md:17`).
- The S3 ROM can also download over USB-OTG at 0x303A:0x0009. Neither of this board's entry paths uses it.
- That both modes appear as `/dev/cu.usbmodem*` on macOS is *unverified*, and the suffix is unknown. Find the port by VID/PID.
- With `--before default-reset`, esptool switches to the USB-JTAG reset sequence when the port's PID is 0x1001 (`loader.py:850`).
- Non-invasive check of the running firmware: HID GET_REPORT(Feature) returns 0, 6, 0 on stock. `ioreg -p IOUSB -l -w0` and `hidutil list --matching '{"VendorID":0x2341,"ProductID":0x1002}'` ran without error here, with no board attached.

### 8.3 Arduino's official restore procedure

Support article 16379769332892, updated 2026-10-08. It was read through the Help Center JSON API because the page returns 403 to WebFetch. Condensed, with quotes verbatim:
1. Unplug the board and any non-essential USB devices.
2. Find GND and Download on the 6-pin header next to the USB-C connector.
3. "Connect the board to your computer while shorting the GND and Download pins." "You can release the short after you've connected the board to your computer."
4. The ESP32 is now in Download mode.
5. Download `unor4wifi-update-{windows,macos,linux}.zip` from releases/latest.
6. Open a terminal in the extracted folder.
7. Run "macOS/Linux: `./bin/espflash write-bin -b 115200 0x0 firmware/UNOR4-WIFI-S3-*.bin`" or "Windows: `bin\espflash write-bin -b 115200 0x0 (Get-Item .\firmware\UNOR4-WIFI-S3-*.bin).FullName`".
8. Select the port: on macOS "USB JTAG_serial debug unit", the `/dev/cu.usbmodem*` one; on Windows "USB Serial Device".
9. Answer N to "Remember this serial port".
10. Wait for the progress bar to finish.
11. Close the terminal, disconnect, "Ensure that the Download and GND pins are no longer shorted", and reconnect.

Related facts:
- After flashing, unplug, remove the jumper and plug in again. Pressing RST alone leaves the board in ESP download mode (pennam, bridge issue #9).
- Support article 9670986058780: "If you upload a sketch while the board is in ESP Mode, it will erase the special firmware that lets the ESP32 chip function as an USB bridge."
- The updater scripts run `unor4wifi-reboot` first (`update_mac.command:8`), so with a working bridge no jumper is needed.
- releases/latest is 0.6.0 (2025-05-08). Before 2025-07-15 it served 0.5.2, because 0.6.0 lacked the "latest" flag (bridge issue #75).
- The official image `UNOR4-WIFI-S3-0.6.0.bin` is 1,966,080 B, sha256 `e4a0e6f9451188b5714c2ec53a6552d2ad59b59f468cd47b695bd19ba752fd3c`, md5 `1dd25628a5550b6045d5194aafb56038`. A copy is in `$SCR/upd/unor4wifi-update-macos/firmware/`. Its partition table, otadata and cert bundle match a local build byte for byte.

Arduino's esp32-upload tutorial (docs.arduino.cc, by Hannes Siebeneicher, last commit `dab66ecb`, 2026-09-22):
- "In order to flash custom firmware to the ESP32-S3 we need to put the chip in download mode by shorting the download pin and GND." "At this point, the board has to be powered off. Once the pins are shorted you can connect the board to your PC and remove the jumper wire. If you check the device name it should have changed to: USB JTAG/serial debug unit."
- Its commands use esptool v4 underscore names: `esptool.py --chip esp32s3 --port <yourPort> erase_flash`, then `esptool.py --chip esp32s3 --port <your port> write_flash -z 0 <yourCustomFirmware.bin>`. Do not run `erase_flash`; it wipes NVS.
- "Any direct programming of the ESP32 will override that firmware and the communication between the chips may be disrupted until the default firmware is restored."

### 8.4 Tools and syntax traps

| Tool | Version | Where | Notes |
|---|---|---|---|
| espflash, bundled by Arduino | 2.0.0, x86_64 | updater zip `bin/espflash` | only board-info, completions, flash, monitor, partition-table, save-image and write-bin; **no read-flash, no erase-flash** |
| espflash, installed | 4.6.0 | `/opt/homebrew/bin/espflash` | `-b` means `--before`; baud is `-B` |
| esptool | 5.5.0 | `.venv`, run as `python -m esptool` | hyphenated commands; `-b` is still baud |
| esptool, bundled with the core | esptool.py 4.5.1, x86_64 (runs under Rosetta) | `$CORE/tools/esptool/esptool` | underscore syntax; used by the arduino-cli upload recipe |
| unor4wifi-reboot-macos | prebuilt x86_64 | `$SCR/upd/unor4wifi-update-macos/bin/` | not executed; Go is not installed, so it cannot be built from source |

The espflash `-b` trap: from espflash 3.0.0 on, `-b` means `--before` and the baud flag is `-B` or `--baud` (`espflash/src/cli/mod.rs` at v2.0.0 versus v3.0.0 and later). Arduino's `write-bin -b 115200` works only with the bundled v2.0.0. The installed 4.6.0 rejects it:
```
error: invalid value '115200' for '--before <BEFORE>' [possible values: default-reset, no-reset, no-reset-no-sync, usb-reset]
```

esptool v5 names:
- The canonical spellings use hyphens: `flash-id`, `read-flash`, `write-flash`, `verify-flash`, `erase-region`, `erase-flash`, `image-info`, `merge-bin`, `--flash-size`, `--flash-mode`, `--flash-freq`, `default-reset`, `hard-reset`.
- Underscore forms still run with a "Deprecated" warning, and will be removed in the next major release. `esptool.py` becomes `esptool`.
- `read-flash ADDRESS SIZE FILE` accepts `ALL`, meaning the detected size (since v4.6), and k/M suffixes.
- `write-flash` verifies the MD5 by default.
- `-e` / `--erase-all` wipes the whole chip. Never use it.
- On the ESP32-S3, `hard_reset` clears `FORCE_DOWNLOAD_BOOT` first (`esptool/targets/esp32s3.py:347-354`), so `--after hard-reset` boots the new app.
- The latest release is v5.5.0 (2026-10-08). The v4 line is still maintained (v4.12.0, 2026-07-14).

espflash 4.6.0:
- Subcommands: `board-info`; `read-flash <ADDRESS> <SIZE> <FILE>` (SIZE in decimal or 0x hex, no `ALL`, no `M` suffix; added in 3.0.0); `write-bin <ADDRESS> <FILE>` (no verify or flash-size option in help); `checksum-md5 <ADDRESS> <SIZE>`; `erase-region` (4 KiB aligned); `erase-flash` (never: it wipes NVS); `partition-table` (works offline); `list-ports`.
- Shared options: `-c/--chip`, `-p/--port`, `-B/--baud`, `-b/--before {default-reset,no-reset,no-reset-no-sync,usb-reset}`, `-a/--after {hard-reset,no-reset,no-reset-no-stub,watchdog-reset}`, `--non-interactive`, `-S/--skip-update-check`, `--no-stub`.
- *Unverified*: whether `write-bin` verifies by itself, and whether its hard reset clears `FORCE_DOWNLOAD_BOOT`. After any espflash write, power-cycle the board.

### 8.5 Exact command lines

Every line below was parse-tested against a nonexistent port and reached the connection stage. None was run against a board.

esptool 5.5.0:
```sh
PY="/Users/jwilson/Downloads/University/Meche Projects/A4/.venv/bin/python"

# 0. Enter download mode: run unor4wifi-reboot-macos (or use the jumper), then wait 1-5 s.

# Find the port by VID/PID without opening it.
PORT=$("$PY" -c "from serial.tools import list_ports as l; print(next(p.device for p in l.comports() if (p.vid,p.pid)==(0x303a,0x1001)))")
# Fallbacks: PORT=$(ls /dev/cu.usbmodem* | head -n1)   or pass --port-filter vid=0x303a pid=0x1001

# 1. Identify. Prints "Detected flash size: ...".
"$PY" -m esptool --chip esp32s3 --port "$PORT" --baud 115200 --before default-reset --after no-reset flash-id

# 2. Back up the whole flash, then check the backup.
"$PY" -m esptool --chip esp32s3 --port "$PORT" --baud 115200 --before default-reset --after no-reset read-flash 0 ALL backup_full.bin
"$PY" -m esptool --chip esp32s3 --port "$PORT" --baud 115200 --before default-reset --after no-reset verify-flash 0x0 backup_full.bin

# 3. Write. Verifies MD5 by itself. S3-APP.bin would go at 0xE000. Never pass -e.
"$PY" -m esptool --chip esp32s3 --port "$PORT" --baud 115200 --before default-reset --after no-reset write-flash --flash-mode keep --flash-freq keep --flash-size keep 0x0 S3-BOOT-APP.bin

# 4. Verify and boot.
"$PY" -m esptool --chip esp32s3 --port "$PORT" --baud 115200 --before default-reset --after hard-reset verify-flash 0x0 S3-BOOT-APP.bin

# Restore from the backup.
"$PY" -m esptool --chip esp32s3 --port "$PORT" --baud 115200 --before default-reset --after hard-reset write-flash --flash-size keep 0x0 backup_full.bin
```

espflash 4.6.0. Note the capital `-B`:
```sh
E=/opt/homebrew/bin/espflash
SIZE=0x800000   # take it from board-info: 8 MB expected on the N8 module, 0x400000 if it reports 4 MB

# 1. Identify.
$E board-info -S --non-interactive -c esp32s3 -p "$PORT" -B 115200 --before usb-reset --after no-reset

# 2. Back up.
$E read-flash -S --non-interactive -c esp32s3 -p "$PORT" -B 115200 --before usb-reset --after no-reset 0x0 "$SIZE" backup_full.bin

# 3. Write. S3-APP.bin would go at 0xE000.
$E write-bin -S --non-interactive -c esp32s3 -p "$PORT" -B 115200 --before usb-reset --after no-reset 0x0 S3-BOOT-APP.bin

# 4. Verify: compare with `md5 -q S3-BOOT-APP.bin` (for S3-APP use 0xE000 0x1D2000), then power-cycle the board.
$E checksum-md5 -S --non-interactive -c esp32s3 -p "$PORT" -B 115200 --before usb-reset --after hard-reset 0x0 0x1E0000
```
The exact output of `checksum-md5` and the flash-size line of `board-info` were not seen.

Recommendation: prefer esptool 5.5.0 for the final write and reset. It verifies MD5 by default, has `verify-flash`, and clears `FORCE_DOWNLOAD_BOOT` on reset.

### 8.6 Other flashing facts

- `firmware/README.md:31` uses the default arduino-esp32 offsets: boot_app0 at 0xe000 and the app at 0x10000. Both fall inside the cert partition, so following it corrupts the cert bundle and leaves app0 unchanged. `README.md:47` uses an obsolete file name (`S3.bin`) and the obsolete `-b`. Do not follow either line.
- The arduino-cli upload recipe, used by `download.sh`, writes the bootloader at 0x0, the table at 0x8000, boot_app0 at 0x9000 and the app at 0x50000. It passes `--flash_size 4MB` at 921600 baud with `--before default_reset --after hard_reset` (`$CORE/platform.txt:270`). It does not write the cert partition and does not touch app1, SPIFFS or NVS.
- That recipe needs the board already in download mode (`upload.use_1200bps_touch=false`, `boards.txt:1089`). `download.sh:3` hard-codes the Linux port `/dev/ttyACM0`.
- On macOS use `/dev/cu.*`, not `/dev/tty.*`.
- The venv has esptool 5.5.0, pyserial 3.5, esp-pylib 1.2.0 and cryptography, but no Python hidapi. Homebrew has espflash and libusb. An alternative to the reboot binary is to install hidapi and send `[0x00, 0xAA] + [0]*63` to 0x2341:0x1002. The hidapi API details are recalled, so check them first. Whether macOS asks for permission for this feature report is untested.

### 8.7 The retracted 0.5.2 workaround

- Forum thread 1355018 (Feb 2025): the user needed a 0.5.x pre-release. `update.sh` looped "Unable to connect, retrying", then `unor4wifi-reboot` said "No board connected".
  - van_der_decken's manual fix: short the pins before power-on, remove the jumper after power is applied, run `esptool.py --chip esp32s3 --port <port> erase_flash`, then `write_flash -z 0 <bin>`, plus a PC power-cycle.
- Forum thread 1425480 and bridge issue #78 (Jul 2026): `espflash write-bin` of 0.6.0 failed with "Error while running FlashDeflateData command ... Bad data checksum".
  - ptillisch first suggested flashing 0.5.2, then retracted it (post #18): "the correlation I thought I identified between the firmware version and the error was not actually real ... the cause of the problem as a tool named serial-discovery, which is used by Arduino IDE, Arduino Cloud Agent, and Arduino App Lab ... You don't need to worry about the modified procedure for flashing the 0.5.2 firmware".
  - The cause was Arduino's serial-discovery probing the port. serial-discovery PR #130 fixed it (merged 2026-07-15; releases v1.5.1 and v1.5.2).
  - jacob-b22 also hit "MemData command" errors, tried 57600 and 921600 baud, and succeeded only with every Arduino tool uninstalled.
- Runbook rule: close Arduino IDE, Cloud Agent and App Lab, or confirm serial-discovery is 1.5.1 or newer. Restore to 0.6.0, not 0.5.2.

### 8.8 Checks a dry run can do without a board

1. Tool versions: espflash 4.6.0, esptool 5.5.0, the reboot binary present, Rosetta available.
2. Every planned command against `-p /dev/cu.<nonexistent>`. A syntax error is a usage error (espflash exit 2). A valid command fails later with "Error while connecting to device" or "Could not open ... port".
3. Image sizes: S3-ALL 0x400000, S3-BOOT-APP 0x1E0000, S3-APP 0x1D2000. Also S3-BOOT-APP == S3-ALL[:0x1E0000] and S3-APP == S3-ALL[0xE000:0x1E0000].
4. Magic byte 0xE9 at 0x0 and at 0x50000. Offline `esptool image-info` on the bootloader and app0 slices: checksum valid, validation hash valid, Chip ID 9, flash size 4MB.
5. `len(UNOR4USBBridge.ino.bin) <= 0x190000`.
6. The 0x8000 slice equals `espflash partition-table --to-binary unor4wifi.csv`, which equals `gen_esp32part.py` output byte for byte (verified: `$SCR/pt_espflash.bin` and `pt_gen.bin`, 3072 B each).
7. The 0x9000 slice equals `boot/boot_app0.bin` (verified identical to the core's copy).
8. The cert slice equals the official 0.6.0 cert region, and the bundle is at most 0x45000.
9. The bootloader slice compared with the official 0.6.0 bootloader.
10. Port discovery by VID/PID through pyserial `list_ports`, which does not open ports, plus `ioreg` or `hidutil` for 0x2341:0x1002.
11. Print the planned command lines and the image MD5s.
12. After a backup exists: its size equals the detected flash size; `backup[0x8000:]` parses as the expected table; extract NVS with `dd bs=4096 skip=$((0x3F0000/4096)) count=16`.

## 9. Xbox controller facts

- **Models.** The Series X|S controller (model 1914, "3 buttons") shipped with Bluetooth LE only. A September 2021 firmware update added BLE (HID over GATT) to model 1708 ("2 buttons"), the Elite Series 2 and the Adaptive Controller. The ESP32-S3 has no Bluetooth Classic, so only BLE controllers can work.
- **Firmware.** Bluepad32 requires v5.15 or newer for model 1914 and for BLE on the 1708 (`docs/supported_gamepads.md:265`, `:294`). asukiaaa tested 5.13.3143.0 and 5.17.3203.0. No Microsoft document gives a minimum. One user cured "waiting for first notification" by updating the controller firmware (asukiaaa issue #7). Update through the Xbox Accessories app or a console before testing.
- **Pairing mode.** Microsoft: "Press and hold the Pair button on your controller for three seconds (the XBOX button will start flashing rapidly)." In pairing mode the controller advertises manufacturer data `0600030080`; otherwise `060000` (library constants).
- **One remembered Bluetooth host.**
  - The controller remembers one Bluetooth host and one Xbox Wireless host. A double press of Pair switches between them.
  - Microsoft: "press and hold the Pair button to start pairing mode. The controller will automatically connect to the Bluetooth device that you were just using while it also searches for new devices."
  - Pairing the controller to a phone or PC later replaces the ESP32 as its Bluetooth host, and the ESP32 must then re-pair. The ESP32's bond must survive reboots in NVS.
- **Security settings.** Bluepad32 configures NoInputNoOutput ("Just Works"), turns Secure-Connections-only mode off, allows key sizes 7–16 and requests bonding (`uni_bt_le.c:978-1015`). Its source comments record these results:

| Setting (BTstack on the original ESP32) | Model 1914 (3 buttons) | Model 1708 (2 buttons) |
|---|---|---|
| Legacy pairing, no bonding | flaky | flaky |
| Secure Connections, no bonding | OK | fails to connect |
| Secure Connections + bonding | fails to connect | fails to connect |

  - Bluepad32 ships legacy pairing with bonding. The asukiaaa library uses the same: bonding on, MITM off, Secure Connections off. It used `(true,true,true)` from January to July 2022, then switched.
  - So legacy pairing with bonding is the known-working configuration, not a proven requirement.
  - The Bluepad32 comments are undated and come from BTstack on the original ESP32, not NimBLE on the S3. Whether a 1914 on current firmware negotiates Secure Connections is not verified.
- **Reports.**
  - Notifications on 0x1812/0x2A4D carry 16 bytes (layout in 6.8). Decoding Bluepad32's Xbox HID descriptor gives a 128-bit input report with ID 1 (`uni_hid_parser_xboxone.c:31-49`).
  - The Elite Series 2 sends 19 bytes: the same first 16, then the profile at [16] and the paddles at [18] (asukiaaa issue #18, raw dumps "LENGTH = 19"; issue #14).
  - Accept reports of 16 bytes or more and parse only bytes 0–15.
- **Known quirks.**
  - The original ESP32 logs "lld_pdu_get_tx_flush_nb HCI packet count mismatch (1, 2)". The C3 does not, and the S3 likely does not (asukiaaa issue #3; `README.md:44`).
  - No input arrives until the link is encrypted and bonded: the LED keeps blinking and nothing is notified (NimBLE-Arduino #172; see 7.5 for what that fix actually was).
  - finger563 PR #22 (merged 2026-09-15) applied these fixes: wait for authentication before discovery and subscription, retrying about every 500 ms up to 10 times; never delete the bond on a transient failure; subscribe to every input report, keyed by its Report Reference descriptor; on disconnect, release all buttons and centre the sticks. The PR is AI-generated and says "Not exercised on hardware", so its root-cause story is *unverified*.
  - arduino-esp32 #12169: an Xbox pad on NimBLE gave no data until security was configured. It was fixed by a `Client_Gamepad` example in core 3.x only.
  - esp-idf #18893 reported an encrypted-link stall on an S3 NimBLE central, ending in reason 0x208 (supervision timeout) after about 16 s, on IDF 5.5. Espressif could not reproduce it and closed it. This is weak evidence, and whether IDF 4.4 is affected is unknown.
  - That the example address prefix 44:16:22 is a Microsoft OUI is *unverified* (recalled). Whether the controller's address is always public (type 0) is unknown.

## 10. Licences

The bridge repo has no top-level LICENSE. GitHub reports `"license": null`, and `/license` returns 404.

| Component | Files | Licence | Evidence |
|---|---|---|---|
| chAT | `parser.cpp`, `server.cpp`, `chAT.hpp` | AGPL-3.0-or-later | `chAT.hpp:1-8`, same header in `parser.cpp:6`, `server.cpp:6`; upstream SudoMaker/chAT spdx AGPL-3.0 |
| OTA | `OTA.cpp`, `OTA.h` | GPL-3.0 (no "or later"), with an Arduino commercial-licence note | `OTA.h:6`, `OTA.cpp:6` |
| SSE | `SSE.cpp`, `SSE.h` | MPL-2.0 | `SSE.cpp` header |
| BOSSA glue | `BossaUnoR4WiFi.h`, `BossaUnoR4WiFI.cpp` | LGPL-2.1-or-later | `BossaUnoR4WiFi.h:5-7` |
| freedap | `freedap.c`, `freedap.h`, `dap_config.h` | BSD-3-Clause | `dap_config.h:1` |
| Arduino glue | `UNOR4USBBridge.ino`, `at_handler.*`, `commands.h`, `cmds_*.h`, `DAP.*`, `ping.*` | no header and no grant | — |
| arduino-esp32 core | `$CORE` | LGPL-2.1 | `$CORE/LICENSE.md:1` |
| ArduinoBLE, bridge copy | `firmware/libraries/ArduinoBLE` | LGPL-2.1 | critic |
| BOSSA | — | BSD-style | critic |
| NimBLE-Arduino 2.5.1 | whole library | Apache-2.0. `NOTICE` credits Ryan Powell (2020-2026) and Neil Kolban's esp32-snippets (2017). | `LICENSE:1-2`, `NOTICE` |
| Apache Mynewt NimBLE, bundled | `src/nimble` | Apache-2.0. NOTICE: ASF 2015-2022, Espressif modifications 2017-2022, Runtime Inc 2015. The LICENSE appendix lists queue.h 8.5 and os_mbuf.c (3-clause BSD), Gary S. Brown CRC32 (unrestricted), tinycrypt (3-clause BSD), Nordic nRF52 SDK files (BSD-style) and CMSIS-CORE files. | `src/nimble/NOTICE`, `src/nimble/LICENSE:203-226` |
| asukiaaa libraries | Xbox client, parser, HID report builder, ControllerAsI2c | MIT, Copyright (c) 2022 Asuki Kono | each `LICENSE:1` |
| esp32-client-for-xbox-controller-with-nim-ble | — | none | reference only |
| zmanu04 ESP-NOW fork | — | none | pattern only |
| Bluepad32 / BTstack | not used | Apache-2.0 / free only for open source | Bluepad32 README |

Consequences:
- chAT is linked into the image, so the fork's binary must ship as AGPL-3.0-or-later with complete source.
- GPLv3 and AGPLv3 section 13 allow combining OTA's GPL-3.0-only code with AGPL code.
- MIT is compatible. That Apache-2.0 is GPLv3-compatible was recalled by the web reader and is *unverified* in recon.
- The Arduino glue files carry no explicit grant, and no Arduino-wide default was found. This residual ambiguity is a question for the project owner, or for Arduino.
- Ship these inside `firmware/libraries`: NimBLE's `LICENSE`, `NOTICE`, `src/nimble/LICENSE`, `src/nimble/NOTICE` and `src/nimble/ext/tinycrypt/LICENSE`, plus each asukiaaa MIT `LICENSE`. Keep every upstream file header.
- License the RA4M1 library separately, for example MIT or LGPL-2.1. It must define its own command string rather than copy `commands.h`.
- Do not copy code from the two unlicensed repos.

## 11. Open questions that need hardware

1. **Controller handover.** Can `NimBLEDevice::deinit(true)` followed by `HCIVirtualTransport.begin()`, or the reverse, work without a reboot? Source says it is plausible. Until it is tested, use sticky modes and `AT+RESET`.
2. **Re-pairing after `deleteBond`.** Does the controller accept a fresh Just Works pairing when it is not in pairing mode (advertising `060000`)?
3. **Address type.** Is the controller's address always public (type 0)? The String constructor and NimBLE's type-aware `equals()` depend on it.
4. **Encryption trigger.** Does the controller send a Security Request on connect, or return Insufficient Encryption on the first read?
5. **Secure Connections.** Must Secure Connections stay off with NimBLE on the S3? Bluepad32's notes come from BTstack on the original ESP32.
6. **Minimum controller firmware.** Bluepad32 says 5.15; asukiaaa tested 5.13 and 5.17.
7. **GATT layout.** Which handle carries the 16-byte input report? Does the controller expose more than one notifiable 0x2A4D report?
8. **Heap.** How much heap is free once WiFi, the BT controller and NimBLE are up, with no PSRAM? Static DRAM is about 70 KB of the module's 512 KB SRAM.
9. **NVS fill.** How full does the 64 KB NVS get with WiFi config, PHY data, RA4M1 Preferences, SSE keys and bonds?
10. **AT latency.** What are the real per-poll latency and jitter, especially while `nimble_host` handles a 100 %-duty scan with duplicate filtering off? The 5–6 ms figure is computed.
11. **Coexistence.** What is WiFi throughput during continuous scanning, and BLE latency while WiFi is active?
12. **esp-idf #18893.** Does the encrypted-link stall occur on the IDF 4.4 controller library?
13. **Flash size.** Does the board report 8 MB, as the N8 datasheet says, or 4 MB? This sets `$SIZE` for espflash backups.
14. **Port names.** Which `/dev/cu.usbmodem*` names does macOS assign in download mode (0x303A:0x1001) and in normal mode (0x2341:0x1002)?
15. **espflash behaviour.** Does `write-bin` verify by itself? Does its hard reset clear `FORCE_DOWNLOAD_BOOT` on the S3?
16. **Board state.** Is the board on official 0.6.0? Has it ever done an ESP32 OTA, so that it now boots app1? Both matter only for `S3-APP.bin`.
17. **HID reboot.** Does `unor4wifi-reboot` report success, which depends on TinyUSB finishing the status stage before the reset? Does macOS hidapi need a permission prompt?

Also open, but answerable without hardware:
- Is `CONFIG_ESP_TASK_WDT_PANIC` set in the vendored sdkconfig?
- Does arduino-cli reuse a cached `core.a` after `build_opt.h` changes? This does not matter for NimBLE macros.
- Does espflash `write-bin` rewrite the flash-size field of the bootloader header? esptool's `write-flash` keeps it with `--flash-size keep`.
- Design decisions the readers left open: the payload layout (20 B versus 21 or 27 B), the Xbox task's core (0 or 1), and the policy for rooms with several controllers. One option is to save the first controller's address in Preferences, use `setTargetAddress`, and add a re-pair command that sets the ignore flag.
- Licensing: is AGPL-3.0 for the fork acceptable to the project owner?

## 12. Stage 1 baseline results

Source: `$SCR/notes.md`. Stock firmware, commit `94d5bb2` plus a `compile.sh` quoting fix, 2026-10-11.

Build:

| Item | Value |
|---|---|
| `compile.sh` wall time | 44 s |
| arduino-cli "Sketch uses" | 1,179,737 B (70 %) of 1,671,168 (arduino-cli's maximum) |
| `UNOR4USBBridge.ino.bin` | 1,180,096 B |
| Real app0 partition | 1,638,400 B (0x190000) |
| Headroom to the partition | 458,304 B |
| Global variables | 64,628 B (19 %) of 327,680 |

Export, via `export.sh`. It needs `python` on PATH (use `.venv/bin`) and the `cryptography` package:

| Output | Bytes |
|---|---|
| `x509_crt_bundle` (151 certs) | 68,088 |
| `S3-ALL.bin` | 4,194,304 |
| `S3-BOOT-APP.bin` (0x0–0x1E0000) | 1,966,080 |
| `S3-APP.bin` (0xE000–0x1E0000) | 1,908,736 |

- Stock outputs are copied to `$SCR/stock-build` with `SHA256SUMS`.
- Installed tools: arduino-cli 1.5.1, espflash 4.6.0, esptool 5.5.0 (in `.venv`), and the `renesas_uno` core 1.6.0 in `~/Library/Arduino15`.

Clean-checkout check, repo at `4fd7bb4`, about 04:35:
- `git clone --recurse-submodules`, `python3 get.py` and `./compile.sh` took 2 min 41 s in total. All 7 submodules were at their pins.
- The sketch was 1,180,249 B (70 %). Its sha differs from the in-tree build, because path strings are embedded. The layout is reproducible; the bytes are not bit-identical.

Comparison with the official release:
- `UNOR4-WIFI-S3-0.6.0.bin` (1,966,080 B) has the same layout as `S3-BOOT-APP.bin`.
- It is identical to our stock build up to byte 0x5001C, the app header. 256,268 bytes differ inside the app.
- The official updater runs `unor4wifi-reboot` (software download mode), then `espflash write-bin -b 115200 0x0 <bin>`.

Serial topology confirmed from `UNOR4USBBridge.ino`: UART0 (RX 44, TX 43) is the USB CDC passthrough to the RA4M1's Serial; 1200 baud resets the RA4M1 and 2400 puts it in its bootloader. UART1 (RX 6, TX 5, 115200 8N1) is the AT link. `GPIO_BOOT` 9 and `GPIO_RST` 4 drive the RA4M1. Stock BLE is ArduinoBLE `HCIVirtualTransport` over VHCI, exposed as `AT+HCIBEGIN/END/WAIT/READ/WRITE/AVAILABLE`, with the RA4M1 running the ArduinoBLE host.

## 13. Sources

### Bridge firmware (`firmware/UNOR4USBBridge/`)
- `UNOR4USBBridge.ino`: 5-6, 20-22, 27, 34, 41-46, 111-119, 114-116, 126, 131, 143-147, 148, 149, 151, 157, 158-160, 161, 172-179, 190-215, 226-243, 244-245, 247-249
- `parser.cpp`: 6, 79, 84-91, 101-107, 119-131, 135-137, 139-140, 151-179
- `server.cpp`: 6, 105, 108, 125-139, 153-161, 168-187, 212, 216-218, 221-225, 232-234
- `chAT.hpp`: 1-8, 35-37
- `at_handler.cpp`: 69-87, 118-128, 131-135, 143
- `at_handler.h`: 19-21, 25-26, 81, 100-102
- `commands.h`: 19-20, 25, 110, 160
- `cmds_preferences.h`: 7, 31, 321-331, 346
- `cmds_se.h`: 31 and `_SOFTSE_SERIAL`
- `cmds_ble_bridge.h`: 5-6, 13-18, 89-95, 103-155, 127-139
- `cmds_esp_generic.h`: 11-16, 21-25, 33-37, 47-49, 88-90
- `cmds_wifi_netif.h`: 764-768; `cmds_wifi_station.h`: 16-17; `cmds_wifi_SSL.h`: 108
- `DAP.h`: 13-45, 47-53, 71-75, 76-80; `DAP.cpp`: 4
- `OTA.cpp`: 6, 21, 147; `OTA.h`: 6; `SSE.cpp`: header, 381
- `BossaUnoR4WiFI.cpp`: 52, 84-93; `BossaUnoR4WiFi.h`: 5-7; `dap_config.h`: 1

### Build and tooling (`firmware/`)
- `compile.sh`: 2; `download.sh`: 3; `export.sh`: 9
- `combine.py`: 12-23, 26, 33-45, 41-42, 47-52, 55-60, 68
- `README.md`: 31, 47
- `unor4wifi-reboot/main.go`: 37, 40, 47, 53, 57-59, 66-79; `go.mod`: 5; `hid/hid_enabled.go`: 222-237; `compile_darwin.sh`: 1
- `unor4wifi-updater/package.sh`: 8, 15, 26, 33, 45, 48, 52; `update_mac.command`: 8, 15, 16; `update_windows.bat`: 15-16; `README.md`: 15, 17, 33, 42, 57-66
- `libraries/ArduinoBLE/src/utility/HCIVirtualTransport.cpp`: 35, 46, 53, 86-89, 97-99

### Patched core (`$CORE`)
- `boards.txt`: 1073-1300, 1075-1076, 1081, 1085, 1089-1090, 1096-1098, 1110, 1116, 1177, 1186, 1189, 1192, 1202, 1213, 1250-1251, 1267, 1295
- `platform.txt`: 15-16, 43, 55-58, 60, 73, 88, 129, 132, 145-146, 153, 170-172, 180, 185, 204, 207, 210, 216, 236-237, 270, 278
- `tools/partitions/unor4wifi.csv`: 1-7
- `tools/sdk/esp32s3/sdkconfig`: 108, 115, 220, 823, 829-830, 835-836, 864, 872, 880, 886, 896-898, 1114, 1358-1359, 1571, 1580, 1587-1589, 1611, 1625, 1731, 1737, 1807, 1918, 2560, 2778
- `tools/sdk/esp32s3/qio_qspi/include/sdkconfig.h`: 83, 112, 550-558, 756, 791, 803
- `tools/sdk/esp32s3/dio_qspi/include/sdkconfig.h`: 551, 813
- `tools/sdk/esp32s3/include/bt/include/esp32s3/include/esp_bt.h`: 249-259, 309, 312-313, 394, 405, 412
- `tools/sdk/esp32s3/include/esp_wifi/include/esp_coexist.h`: 98
- `tools/sdk/esp32s3/include/freertos/include/esp_additions/freertos/FreeRTOSConfig.h`: 81, 202
- `soc_caps.h`: 26
- `tools/sdk/esp32s3/lib/libbt.a`, `ld/libbtdm_app.a`, `ld/libbtbb.a`
- `cores/esp32/main.cpp`: 71; `esp32-hal-misc.c`: 240-248, 261-264; `esp32-hal-bt.c`: 19; `esp32-hal-tinyusb.c`: 383, 402-452, 461-471, 488-498, 566, 718; `USBCDC.cpp`: 94, 203-233, 257-258; `HardwareSerial.cpp`: 143
- `libraries/USB/src/USBHID.cpp`: 87-94, 120, 127, 203, 266-276; `USB.cpp`: 38, 65-67
- `libraries/WiFi/src/WiFiGeneric.cpp`: 764
- `tools/esptool/esptool` (v4.5.1); `package/package_esp32_index.template.json`: 521-522; `tools/get.py`: 39, 218; `LICENSE.md`: 1
- Toolchain `xtensa-esp32s3-elf/include/stdio.h`: 161; `sys/reent.h`: 832
- Upstream comparison clone: `$SRC/arduino-esp32-upstream-2.0.9` (tag 2.0.9, `3670e2bf`)

### RA4M1 core (`$RA`)
- `libraries/WiFiS3/src/Modem.cpp`: 7, 33-45, 105-118, 151, 170-171, 189-196, 243-247, 268, 271-276, 296-325, 337-356, 389-403, 413, 423-424
- `libraries/WiFiS3/src/Modem.h`: 12, 14, 54, 111-113, 149, 169
- `libraries/WiFiS3/src/WiFiCommands.h` (symlink to the bridge's `commands.h`); `WiFi.cpp`: 20-21, 32; `OTAUpdate.cpp`: 60, 69
- `libraries/Preferences/src/Preferences.h`: 18; `Preferences.cpp`: 344-353
- `cores/arduino/Arduino.h`: 113-115; `Serial.cpp`: 127-133; `Serial.h`: 41
- `variants/UNOWIFIR4/pins_arduino.h`: 107-112; `variant.cpp`: 51-54
- `boards.txt`: 121-124, 142-148; `platform.txt`: 74
- `~/.platformio/platforms/renesas-ra/boards/uno_r4_wifi.json`
- ArduinoCore-renesas clone `424e86e`: `bootloaders/UNO_R4/README.md`: 54-55

### NimBLE-Arduino (tag 2.5.1 unless stated)
- `src/nimconfig.h`: 116-124, 196-218, 233-239, 245-247, 253-261, 267-268 (2.3.9), 276-277, 289-307, 319-321, 352-355, 428
- `src/NimBLEDevice.cpp`: 102, 507-521, 652, 855-857, 899-909, 920 (2.3.1), 945, 996-1004, 1130-1146, 1189-1194, 1393-1395; `src/NimBLEDevice.h`: 114, 162
- `src/NimBLEClient.cpp`: 69, 954-987, 1062-1075, 1128-1130, 1282-1287; `src/NimBLEClient.h`: 52-58, 111-129
- `src/NimBLERemoteValueAttribute.cpp`: 76-79, 155-158; `src/NimBLERemoteCharacteristic.cpp`: 241-253, 263-264, 349-351; `.h`: 54
- `src/NimBLEScan.cpp`: 73; `src/NimBLEScan.h`: 73-76
- `src/NimBLEAddress.cpp`: 48, 79-80, 213-218; `src/NimBLEUtils.cpp`: 551; `src/NimBLEAdvertisedDevice.h`: 57, 67; `.cpp`: 137-146; `src/NimBLEBondMigration.h`: 37-40
- `src/nimble/esp_port/esp-hci/src/esp_nimble_hci.c`: 300, 337; `src/nimble/esp_port/port/include/esp_nimble_cfg.h`: 1001-1007
- `src/nimble/nimble/host/store/config/src/ble_store_nvs.c`: 40-45; `nimble_port.c`: 164, 231, 237; `nimble_port_freertos.c` (2.3.9): 40, 56-57
- `CHANGELOG.md`: 119, 227, 285
- `LICENSE`, `NOTICE`, `src/nimble/LICENSE` (203-226), `src/nimble/NOTICE`, `src/nimble/ext/tinycrypt/LICENSE`
- Commits `971e5df5` (IDF < 5 build fix, first in 2.3.2) and `ceeb21aa` (master config refactor)

### Xbox libraries (`$SRC`)
- `arduino-XboxSeriesXControllerESP32` v1.1.3 (`0023432`): hpp 8, 10, 13-25, 27-29, 38, 45-49, 53-61, 63-79, 84, 89-90, 97-109, 114, 128, 131-142, 156, 174, 186-192, 234-254, 256-262, 269-277, 297-300, 340-418, 420-439, 466-475, 483-493, 506-550; `README.md`: 14-15, 44, 51; `examples/connect/connect.ino`: 5, 35-37; `library.properties`
- `arduino-XboxControllerNotificationParser` 1.0.5: `src/XboxControllerNotificationParser.h`: 6, 24-30; `.cpp`: 3-6, 17-20, 23-51; `src/XboxControllerElite2NotificationParser.cpp`: 3-4, 7, 28; `README.md`: 68-72
- `arduino-ControllerAsI2c`: `src/ControllerAsI2c_asukiaaa/Common.hpp`: 10-16, 23-27, 36; `XboxSeriesX.hpp`: 12-14, 27-33, 44, 86-94, 132-136; `ControllerAsI2c_asukiaaa.hpp`: 6; `examples/slave_target/esp32/wireless-xbox-series-x/wireless-xbox-series-x.ino`: 88-89, 154, 159-171, 177, 197-198, 222-246
- `esp32-client-for-xbox-controller-with-nim-ble`: `platformio.ini`: 15-16; `src/main.cpp`: 16, 226-227, 268-271

### ArduinoBLE 2.1.0 (`$SRC/ArduinoBLE`, `281377b`)
- `src/utility/L2CAPSignaling.cpp`: 136, 185-187, 264-266; `L2CAPSignaling.h`: 39
- `src/utility/HCI.h`: 34-42; `HCI.cpp`: 1179, 1355; `src/utility/ATT.cpp`: 672-689
- `src/utility/btct.h`: 11-17; `btct.cpp`: 267-271
- `src/utility/HCIVirtualTransportAT.cpp`: 20, 24, 43, 86-92, 101; `HCIVirtualTransportAT.h`: 25
- `src/local/BLELocalDevice.h`: 90-109

### Bluepad32 (`$SRC/bluepad32`, `80d6314`)
- `src/components/bluepad32/bt/uni_bt_le.c`: 978-1015
- `src/components/bluepad32/parser/uni_hid_parser_xboxone.c`: 31-49
- `docs/supported_gamepads.md`: 252, 265, 282, 294

### Scratch harnesses and artefacts (`$SCR`)
- AT round-trip harnesses: `chatsim/sim.cpp`, `chatsim/e2e_esp.cpp`, `modemsim/e2e_ra.cpp`
- Verifier harnesses: `refute_wire/t.cpp`, `verify_wire/`, `refute_link/h.cpp`, `refute_sized/test.cpp`, `refute_sized/test2.cpp`
- Link tests: `xb_linktest/` (XbControl, XbLinkTest, XbOpt, `compile.log`), `refute_budget/XbRoles`
- Compatibility tests: `ctest/sketch/XboxTest/XboxTest.ino`, `ctest/compile_1.4.3.log`, `compile_2.3.1.log`, `compile_2.3.9.log`, `compile_master.log`, `compile_debug.log`
- Official 0.6.0 updater: `upd/unor4wifi-update-macos/`
- Stock outputs: `stock-build/` with `SHA256SUMS`
- Other: `pt_espflash.bin`, `pt_gen.bin`, `notes.md`, `cleancheck.log`
- Reader outputs: `recon_link.txt`, `recon_core.txt`, `recon_xbox.txt`, `recon_web.txt`, `recon_flash.txt`, `recon_critic.txt`; the full verdict texts are in `recon_result.json`

### Datasheets, schematics and specifications
- ABX00087 datasheet: https://docs.arduino.cc/resources/datasheets/ABX00087-datasheet.pdf (§1, §6, p10, p11, BOM)
- ABX00087 schematics: https://docs.arduino.cc/resources/schematics/ABX00087-schematics.pdf (pdftotext lines 371, 430-439, 516, 525-535)
- ESP32-S3-MINI-1/1U datasheet v1.7: https://documentation.espressif.com/esp32-s3-mini-1_mini-1u_datasheet_en.pdf
- HOGP: https://www.bluetooth.com/specifications/specs/hid-over-gatt-profile-hogp/ and the v1.2 HTML at bluetooth.com/wp-content/uploads/Files/Specification/HTML/HOGP_v1.2/out/en/index-en.html (§8.1, §8.2)

### Arduino documentation and support
- Restore with espflash: https://support.arduino.cc/hc/en-us/articles/16379769332892-Restore-the-USB-connectivity-firmware-on-UNO-R4-WiFi-with-espflash (body via https://support.arduino.cc/api/v2/help_center/en-us/articles/16379769332892.json)
- ESP download-mode warning: https://support.arduino.cc/hc/en-us/articles/9670986058780
- esp32-upload tutorial: https://docs.arduino.cc/tutorials/uno-r4-wifi/esp32-upload; source https://github.com/arduino/docs-content/blob/main/content/hardware/uno/boards/uno-r4-wifi/tutorials/esp32-upload/esp32-upload.md (lines 20, 32, 36, 51-67, 71-73)
- Forum: https://forum.arduino.cc/t/trying-to-connect-xbox-one-controller-to-uno-r4-wifi-and-others/1165216
- Forum: https://forum.arduino.cc/t/arduinoble-and-talking-to-hid-devices/1148903
- Forum: https://forum.arduino.cc/t/arduinoble-trying-to-connect-to-xbox-one-controller-and-other-ble-devices/1054844
- Forum: https://forum.arduino.cc/t/arduino-uno-r4-wifi-firmware/1355018
- Forum: https://forum.arduino.cc/t/arduino-uno-r4-wifi-firmware/1425480
- Forum: https://forum.arduino.cc/t/arduino-uno-r4-wifi-is-recognized-as-wifiduino32s3-in-arduino-ide/1231834

### GitHub
- Bridge: https://github.com/arduino/uno-r4-wifi-usb-bridge (forks at `/forks`; issues #9, #49, #75, #78)
- Release zip: https://github.com/arduino/uno-r4-wifi-usb-bridge/releases/download/0.6.0/unor4wifi-update-macos.zip
- Reboot tool zip: https://github.com/arduino/uno-r4-wifi-usb-bridge/releases/download/0.1.0/unor4wifi-reboot-macos.zip
- HID fork: https://github.com/pennam/hid.git (`931f677`)
- ESP-NOW precedent: https://github.com/zmanu04/uno-r4-wifi-usb-bridge-ESP-NOW
- https://github.com/ricardoquesada/bluepad32-arduino/issues/10
- https://github.com/asukiaaa/arduino-XboxSeriesXControllerESP32 (issues #3, #7, #10, #11, #14, #15, #16, #18)
- https://github.com/h2zero/NimBLE-Arduino/issues/621 and /issues/172
- https://github.com/arduino-libraries/ArduinoBLE/pull/156, /issues/276, /releases
- https://github.com/finger563/esp-usb-ble-hid/pull/22
- https://github.com/espressif/arduino-esp32/issues/12169
- https://github.com/espressif/esp-idf/issues/18893
- https://github.com/arduino/serial-discovery/pull/130
- https://github.com/ricardoquesada/bluepad32 (README; https://github.com/ricardoquesada/bluepad32/releases/tag/5.0.0-beta0; https://github.com/ricardoquesada/bluepad32/blob/main/src/components/bluepad32/bt/uni_bt_le.c)
- https://github.com/ricardoquesada/esp32-arduino-lib-builder/releases/tag/4.1.0
- https://github.com/ricardoquesada/esp-idf-arduino-bluepad32-template/blob/main/sdkconfig.defaults (lines 15-16)
- https://github.com/esp-rs/espflash/blob/v4.0.0/espflash/src/cli/mod.rs; https://github.com/esp-rs/espflash/blob/v4.6.0/espflash/src/cli/mod.rs (205-225, 397-406); https://github.com/esp-rs/espflash/blob/main/CHANGELOG.md
- https://github.com/espressif/esptool/releases/tag/v5.0.0 and /releases/tag/v4.6

### Other web sources
- https://bluepad32.readthedocs.io/en/latest/plat_arduino/
- https://bluepad32.readthedocs.io/en/latest/supported_gamepads/
- https://docs.espressif.com/projects/esptool/en/latest/esp32s3/migration-guide.html
- https://docs.espressif.com/projects/esptool/en/latest/esp32s3/esptool/basic-commands.html
- https://novelbits.io/bluetooth-hid-devices-an-intro/
- https://en.wikipedia.org/wiki/Xbox_Wireless_Controller
- https://news.xbox.com/en-us/2021/09/08/xbox-controller-firmware-update-rolling-out-to-insiders-starting-today/
- https://support.xbox.com/en-US/help/hardware-network/controller/connect-xbox-wireless-controller-to-pc (JS-rendered; read with Playwright)

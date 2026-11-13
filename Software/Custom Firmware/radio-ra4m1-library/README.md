<!-- SPDX-License-Identifier: MIT -->
# XboxControllerR4

Reads an Xbox Series X|S controller from an Arduino UNO R4 WiFi sketch.

The controller pairs over Bluetooth LE with the board's ESP32-S3 module. The ESP32-S3 keeps
the latest controller state, and this library, running on the RA4M1 (the chip your sketch
runs on), asks for it with `AT+XBOXSTATE?` over the board's internal AT link. That is the same
UART and the same `modem` object the WiFiS3 library uses. Each answer is 20 bytes: the 16-byte
controller report, the connection state, the battery level, a report counter and a CRC.

## Requirements

- Arduino UNO R4 WiFi (`arduino:renesas_uno:unor4wifi`, core 1.6.0; compile-tested and tested
  against a simulated ESP32-S3, not yet run on hardware). The library refuses to compile for
  other boards.
- **The forked ESP32-S3 bridge firmware** from this project (`firmware/`). The stock firmware
  does not know the Xbox commands; `begin()` then returns false with
  `lastError() == XBOX_R4_ERR_NO_FIRMWARE_SUPPORT`. The fork keeps sketch upload, the Serial
  Monitor and WiFi working as before.
- Nothing else: WiFiS3 ships with the board core, and the report parser is vendored in
  `src/vendor/` (MIT, see `src/vendor/VENDORED.md`).

## Install

Either copy the whole `XboxControllerR4` folder into your sketchbook's `libraries` folder
(`~/Documents/Arduino/libraries/` on macOS and Windows, `~/Arduino/libraries/` on Linux) and
restart the Arduino IDE, or point arduino-cli at the folder that contains it:

```sh
arduino-cli compile --fqbn arduino:renesas_uno:unor4wifi \
  --libraries "path/to/ra4m1/libraries" path/to/YourSketch
```

## Quick start

```cpp
#include <XboxControllerR4.h>

XboxControllerR4 xbox;

void setup() {
  Serial.begin(115200);
  if (!xbox.begin()) {
    Serial.print("begin failed, lastError() = ");
    Serial.println(xbox.lastError());  // 1 = stock ESP32-S3 firmware
  }
}

void loop() {
  static unsigned long last = 0;
  if (millis() - last >= 20) {         // 50 Hz
    last = millis();
    if (xbox.poll() && xbox.hasNewReport()) {
      Serial.println(xbox.leftY());     // -1.0 (down) .. +1.0 (up)
      if (xbox.xboxNotif.btnA) Serial.println("A");
    }
  }
}
```

**Pairing:** after `begin()` the ESP32-S3 scans. Hold the controller's Pair button (the small
button on its top edge) for 3 s until the Xbox logo flashes fast. Once bonded, switching the
controller on with its Xbox button should reconnect it without pairing again. `forget()`
deletes the bond.

## API

| Call | Returns |
|---|---|
| `bool begin()` | Checks the ESP32-S3 firmware (`AT+XBOXVERSION?`) and starts its Xbox client (`AT+XBOXBEGIN`). False on failure, with the reason in `lastError()`. Safe to call again. |
| `bool poll()` | One `AT+XBOXSTATE?` exchange. True when a valid 20-byte state arrived (length and CRC checked). Call it every 20 ms. |
| `bool hasNewReport()` | True when the last `poll()` changed `xboxNotif`: a controller report not seen before, or the neutral report after a disconnect. |
| `uint8_t connectionState()` | `XBOX_R4_CONNECTED` (0), `XBOX_R4_WAITING_FIRST_REPORT` (1), `XBOX_R4_FOUND` (2), `XBOX_R4_SCANNING` (3), `XBOX_R4_OFF` (4, `begin()` not sent or the ESP32-S3 restarted), `XBOX_R4_BLOCKED_BY_HCI` (5). |
| `bool isConnected()` | `connectionState() == XBOX_R4_CONNECTED`. |
| `uint8_t battery()` | Battery percent 0..100, or `XBOX_R4_BATTERY_UNKNOWN` (255). |
| `uint8_t seq()` | Low byte of the ESP32-S3's report counter. |
| `unsigned long lastReportMillis()` | `millis()` when the last new controller report arrived. An idle controller sends none, so this alone cannot tell a quiet controller from a dead link. |
| `unsigned long lastPollOkMillis()` | `millis()` of the last `poll()` that returned true (0 before the first). Use it to detect that the ESP32-S3 stopped answering (see Limits). |
| `uint32_t pollErrors()` | Polls that timed out or failed the length or CRC check. |
| `int lastError()` | `XBOX_R4_ERR_*` from the last `begin()` (table below). |
| `int beginResult()` | The `AT+XBOXBEGIN` result code: 0 ok, 1 blocked by ArduinoBLE, 2 Bluetooth init failed, -1 no answer. |
| `const char* firmwareVersion()` | ESP32-S3 firmware version text, for example `0.6.99`. |
| `int protocolVersion()` | Protocol version the firmware reported. |
| `float leftX()`, `leftY()`, `rightX()`, `rightY()` | Sticks, -1.0 .. +1.0. Right and up are positive. |
| `float leftTrigger()`, `rightTrigger()` | Triggers, 0.0 .. 1.0. |
| `int16_t leftXRaw()`, `leftYRaw()`, `rightXRaw()`, `rightYRaw()` | Sticks as -32768 .. 32767, same directions as the floats: X raw = `joyLHori` - 32768, Y raw = 32767 - `joyLVert`. A centred stick reads -1 on X and 0 on Y. |
| `bool peerAddress(char out[18])` | Controller address `aa:bb:cc:dd:ee:ff` (all zeros if none). |
| `bool forget()` | Deletes all bonds on the ESP32-S3; the controller must pair again (pairing mode only). Works before or after `begin()`; after `begin()` the ESP32-S3 finishes the bond work within a few seconds of answering, or up to about two minutes if a connection attempt is failing at that moment. |
| `bool setLogging(bool on)` | ESP32-S3 Xbox diagnostics on the board's USB serial port (off by default). |
| `xboxNotif` | The decoded report, with the same fields as on a bare ESP32: `btnA`, `btnB`, `btnX`, `btnY`, `btnLB`, `btnRB`, `btnSelect` (View), `btnStart` (Menu), `btnXbox`, `btnLS`, `btnRS`, `btnShare`, `btnDirUp`/`Down`/`Left`/`Right`, `joyLHori`, `joyLVert`, `joyRHori`, `joyRVert` (0..65535; vertical 0 = up), `trigLT`, `trigRT` (0..1023). |

`xboxNotif` and the stick and trigger helpers change only inside `poll()`. When the
controller disconnects they read as neutral (sticks centred, nothing pressed) and
`hasNewReport()` is true for that poll, so a sketch that drives motors from the sticks stops
instead of holding the last position, also when it acts only on new reports.

### `lastError()` values

| Value | Name | Meaning |
|---|---|---|
| 0 | `XBOX_R4_ERR_NONE` | OK |
| 1 | `XBOX_R4_ERR_NO_FIRMWARE_SUPPORT` | The ESP32-S3 answers the stock `AT+FWVERSION?` but gives no usable answer to `AT+XBOXVERSION?`: it runs the stock bridge firmware. `firmwareVersion()` then holds the stock version. |
| 2 | `XBOX_R4_ERR_PROTOCOL_MISMATCH` | Firmware and library speak different protocol versions; update one of them. |
| 3 | `XBOX_R4_ERR_BEGIN_FAILED` | `AT+XBOXBEGIN` failed; see `beginResult()`. Code 1 means an ArduinoBLE sketch took the Bluetooth radio since the last ESP32-S3 restart: power-cycle the board. |
| 4 | `XBOX_R4_ERR_BRIDGE_SILENT` | The ESP32-S3 answered neither `AT+XBOXVERSION?` nor `AT+FWVERSION?` (still booting, crashed, or left in its bootloader after flashing). Power-cycle the board. |

## Poll rate

One poll moves 56 bytes at 115200 baud and takes about 5-6 ms, during which the RA4M1
busy-waits (there is no RTOS). At 50 Hz that blocks the RA4M1 about 25-30% of the time; at
100 Hz about 50-60%. The examples poll at 50 Hz, and every 500 ms while the ESP32-S3 is not
answering. A poll that gets no answer gives up after 50 ms. With the stock firmware `begin()`
fails after about 0.2 s; when the ESP32-S3 is silent it takes about 0.4 s, or about 1.9 s on
the first call after reset (the core's own `AT+SOFTRESETWIFI` tries come first).

The timeouts are the `XBOX_R4_*_TIMEOUT_MS` macros in `XboxControllerR4.h`. A `#define` in
the sketch does not reach the library's `.cpp`; change them with a build flag, for example
`--build-property "compiler.cpp.extra_flags=-DXBOX_R4_SLOW_CMD_TIMEOUT_MS=200"`.

## Examples

| Example | Shows |
|---|---|
| `PrintAllInputs` | Every input on the Serial Monitor (115200 baud), ten lines a second: connection state, battery, report counter, sticks, triggers, held buttons and the d-pad. Explains a failed `begin()` in plain words and prints the pairing hint while scanning. |
| `StickToLED` | Left stick up dims an LED on D11 (PWM) brighter; the right trigger speeds up the built-in LED's blinking. Turns the LED off when the ESP32-S3 stops answering (the pattern to copy for motors). Wiring: D11, 220 ohm resistor, LED, GND. |
| `MatrixDot` | No wiring: a dot on the built-in 12x8 LED matrix follows the left stick, and the whole matrix lights while A is held. A blinking centre dot means no controller yet; an X means `begin()` failed. |

## Limits

- The Xbox client and ArduinoBLE cannot run at the same time: both need the ESP32-S3's one
  Bluetooth controller. Switching from one to the other needs an ESP32-S3 restart.
- WiFiS3 goes through the same `modem` object; one request runs at a time, so a slow WiFi
  call delays the next poll. Using both in one sketch is untested on hardware.
- If the ESP32-S3 stops answering, `poll()` returns false and `pollErrors()` rises, but the
  last controller values and `isConnected()` stay in place. A sketch that drives motors
  should also stop when `millis() - xbox.lastPollOkMillis()` exceeds about 200 ms, as
  `StickToLED` does.
- Rumble is not supported.

## Licence

MIT. The vendored XboxControllerNotificationParser is MIT, Copyright (c) 2022 Asuki Kono.

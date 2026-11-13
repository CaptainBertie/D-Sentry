/*
 * XboxControllerR4.h -- reads an Xbox Series X|S controller that the UNO R4 WiFi's ESP32-S3
 * has paired over Bluetooth LE. The RA4M1 asks the ESP32-S3 for the latest controller state
 * with AT+XBOXSTATE? over the board's internal AT link, the same UART and `modem` object the
 * WiFiS3 library uses. Needs the forked bridge firmware on the ESP32-S3.
 *
 * The wire contract is docs/protocol.md plus xbox_r4_protocol.h (a byte-identical copy of the
 * project's protocol/xbox_r4_protocol.h).
 *
 * SPDX-License-Identifier: MIT
 * Copyright (c) 2026 the UNO R4 WiFi Xbox controller project
 */
#pragma once

#include <Arduino.h>
#include <string>

#include "xbox_r4_protocol.h"
#include "vendor/XboxControllerNotificationParser.h"

#if !defined(ARDUINO_UNOR4_WIFI)
#error "XboxControllerR4 needs the Arduino UNO R4 WiFi: the controller is paired to its ESP32-S3 module."
#endif

/* ---- lastError() values ---- */
#define XBOX_R4_ERR_NONE                0  /* begin() succeeded, or has not run yet */
#define XBOX_R4_ERR_NO_FIRMWARE_SUPPORT 1  /* ESP32-S3 answers AT+FWVERSION? but gives no usable AT+XBOXVERSION? answer: stock bridge firmware */
#define XBOX_R4_ERR_PROTOCOL_MISMATCH   2  /* firmware speaks another protocol version; see protocolVersion() */
#define XBOX_R4_ERR_BEGIN_FAILED        3  /* AT+XBOXBEGIN failed or got no answer; see beginResult() */
#define XBOX_R4_ERR_BRIDGE_SILENT       4  /* ESP32-S3 answered neither AT+XBOXVERSION? nor AT+FWVERSION? (not booted, crashed, or held in its ROM bootloader) */

/* ---- Timeouts in milliseconds (docs/protocol.md rule 3) ----
 * Only XboxControllerR4.cpp uses these, and Arduino compiles it without the sketch's #defines,
 * so a #define in the .ino has no effect. Override with a build flag instead, for example
 *   arduino-cli compile --build-property "compiler.cpp.extra_flags=-DXBOX_R4_SLOW_CMD_TIMEOUT_MS=200" ...
 */
#ifndef XBOX_R4_POLL_TIMEOUT_MS
#define XBOX_R4_POLL_TIMEOUT_MS 50     /* AT+XBOXSTATE?: one poll is about 5-6 ms on the wire */
#endif
#ifndef XBOX_R4_CMD_TIMEOUT_MS
#define XBOX_R4_CMD_TIMEOUT_MS 200     /* AT+XBOXVERSION?, AT+XBOXADDR?, AT+XBOXLOG=, AT+FWVERSION? probe */
#endif
#ifndef XBOX_R4_SLOW_CMD_TIMEOUT_MS
#define XBOX_R4_SLOW_CMD_TIMEOUT_MS 1000  /* AT+XBOXBEGIN (starts NimBLE), AT+XBOXFORGET (erases bonds in flash) */
#endif

/* beginResult() before AT+XBOXBEGIN got an answer. */
#define XBOX_R4_RESULT_NO_ANSWER (-1)

class XboxControllerR4 {
 public:
  /* Asks the ESP32-S3 for its protocol version, then starts its Xbox client (AT+XBOXBEGIN).
   * Safe to call again, for example after the ESP32-S3 restarted (connectionState() == XBOX_R4_OFF).
   * Returns false and sets lastError() on failure. */
  bool begin();

  /* The hot path: one AT+XBOXSTATE? exchange, about 5-6 ms during which the RA4M1 busy-waits.
   * Returns true when a valid 20-byte state arrived. Poll at 50 Hz (every 20 ms) or slower. */
  bool poll();

  /* True when the last poll() changed xboxNotif: a controller report the library had not seen
   * yet, or the neutral report loaded because the controller disconnected. */
  bool hasNewReport() const { return newReport_; }

  uint8_t connectionState() const { return state_; }  /* enum xbox_r4_conn_state */
  bool isConnected() const { return state_ == XBOX_R4_CONNECTED; }
  uint8_t battery() const { return battery_; }         /* 0..100 %, XBOX_R4_BATTERY_UNKNOWN (0xFF) if unknown */
  uint8_t seq() const { return seq_; }                 /* low byte of the ESP32-S3's report counter */
  unsigned long lastReportMillis() const { return lastReportMillis_; }  /* millis() of the last new controller report */
  /* millis() of the last poll() that returned true (0 before the first). An idle controller
   * sends no new reports but still polls fine, so this, not lastReportMillis(), tells a healthy
   * link from an ESP32-S3 that stopped answering. */
  unsigned long lastPollOkMillis() const { return lastPollOkMillis_; }
  uint32_t pollErrors() const { return pollErrors_; }  /* polls that timed out or carried a bad length/CRC */
  int lastError() const { return lastError_; }         /* XBOX_R4_ERR_* from the last begin() */
  int beginResult() const { return beginResult_; }     /* AT+XBOXBEGIN code (XBOX_R4_RC_*), or XBOX_R4_RESULT_NO_ANSWER */
  const char* firmwareVersion() const { return firmwareVersion_; }  /* ESP32-S3 firmware version text, "" if unknown */
  int protocolVersion() const { return protocolVersion_; }          /* 0 until AT+XBOXVERSION? answered */

  /* Sticks in -1.0 .. +1.0. X: right is positive. Y: up is positive (the raw axis is inverted). */
  float leftX() const { return axisToFloat(xboxNotif.joyLHori); }
  float leftY() const { return -axisToFloat(xboxNotif.joyLVert); }
  float rightX() const { return axisToFloat(xboxNotif.joyRHori); }
  float rightY() const { return -axisToFloat(xboxNotif.joyRVert); }

  /* Triggers in 0.0 (released) .. 1.0 (fully pulled). */
  float leftTrigger() const { return triggerToFloat(xboxNotif.trigLT); }
  float rightTrigger() const { return triggerToFloat(xboxNotif.trigRT); }

  /* Sticks as signed integers -32768 .. 32767, same directions as the floats.
   * X raw = joyLHori - 32768. Y raw = 32767 - joyLVert, mirrored so up is positive.
   * The centre (32767 on the wire) therefore reads -1 on X and 0 on Y. */
  int16_t leftXRaw() const { return axisToRawX(xboxNotif.joyLHori); }
  int16_t leftYRaw() const { return axisToRawY(xboxNotif.joyLVert); }
  int16_t rightXRaw() const { return axisToRawX(xboxNotif.joyRHori); }
  int16_t rightYRaw() const { return axisToRawY(xboxNotif.joyRVert); }

  /* Address of the connected (or last connected) controller as "aa:bb:cc:dd:ee:ff";
   * all zeros if none. Returns false if the ESP32-S3 did not answer. */
  bool peerAddress(char out[18]);
  /* Diagnostic: the last Bluetooth adverts the ESP32-S3 saw while scanning, as text
   * ("addr/type,appearance,rssi,mfgdata_hex,first_service_uuid,name,count;" per device, "none",
   * or "off" before begin()). Copies at most cap-1 chars into out. False if the bridge did not
   * answer. An Xbox controller in pairing mode shows appearance 964, mfgdata 0600030080, uuid 1812. */
  bool scanReport(char* out, size_t cap);

  /* Deletes every BLE bond and the remembered controller on the ESP32-S3; the controller
   * must be paired again. Returns true when the ESP32-S3 reports success. Firmware 0.6.99
   * accepts it before or after begin(); after begin() its client task finishes the bond
   * work a few seconds after the reply. */
  bool forget();

  /* Turns the ESP32-S3's Xbox diagnostics on the board's USB serial port on or off. */
  bool setLogging(bool on);

  /* The decoded controller report. Same field names as the parser on a bare ESP32:
   * btnA, btnB, btnX, btnY, btnLB, btnRB, btnSelect, btnStart, btnXbox, btnLS, btnRS, btnShare,
   * btnDirUp/Down/Left/Right, joyLHori, joyLVert, joyRHori, joyRVert, trigLT, trigRT.
   * Updated only by poll(). Reads as neutral (sticks centred, nothing pressed) while the
   * controller is not connected. */
  XboxControllerNotificationParser xboxNotif;

 private:
  static float axisToFloat(uint16_t raw);
  static float triggerToFloat(uint16_t raw);
  static int16_t axisToRawX(uint16_t raw);
  static int16_t axisToRawY(uint16_t raw);
  static int parseResultCode(const std::string& text);
  void storeFirmwareVersion(const std::string& text, size_t start);

  bool queryVersion();
  bool probeBridge();
  bool sendBegin();
  void loadNeutralReport();

  /* Reused for every poll so the 50 Hz hot path does not allocate on the heap. */
  std::string reply_;

  uint8_t state_ = XBOX_R4_OFF;
  uint8_t battery_ = XBOX_R4_BATTERY_UNKNOWN;
  uint8_t seq_ = 0;
  bool newReport_ = false;
  unsigned long lastReportMillis_ = 0;
  unsigned long lastPollOkMillis_ = 0;
  uint32_t pollErrors_ = 0;
  int lastError_ = XBOX_R4_ERR_NONE;
  int beginResult_ = XBOX_R4_RESULT_NO_ANSWER;
  int protocolVersion_ = 0;
  char firmwareVersion_[16] = {0};
};

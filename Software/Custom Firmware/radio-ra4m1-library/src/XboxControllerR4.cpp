/*
 * XboxControllerR4.cpp -- AT exchanges with the ESP32-S3 bridge firmware and decoding of the
 * 20-byte AT+XBOXSTATE? payload. Talks through the WiFiS3 `modem` global the same way
 * ArduinoBLE's HCIVirtualTransportAT.cpp does on this board.
 *
 * SPDX-License-Identifier: MIT
 * Copyright (c) 2026 the UNO R4 WiFi Xbox controller project
 */
#include "XboxControllerR4.h"

/* WiFiS3.h brings Modem.h, which declares `extern ModemClass modem;` (bound to Serial2),
 * MODEM_TIMEOUT, and the CMD / CMD_READ / CMD_WRITE / PROMPT macros from WiFiCommands.h. */
#include <WiFiS3.h>

#include <ctype.h>
#include <stdlib.h>
#include <string.h>

namespace {

/* Sets the modem timeout for one exchange and puts MODEM_TIMEOUT back on every return path.
 * This is the OTAUpdate.cpp idiom; Modem.h has no getter, so the core default is restored. */
class ModemTimeout {
 public:
  explicit ModemTimeout(size_t ms) { modem.timeout(ms); }
  ~ModemTimeout() { modem.timeout(MODEM_TIMEOUT); }
};

/* Throws away whatever is already waiting on the AT UART. A reply that arrived after an
 * earlier exchange timed out carries the same prompt, so the WiFiS3 parser would take it as
 * the answer to the next request (docs/protocol.md rule 4). */
void drainModemUart() {
  while (Serial2.available()) {
    Serial2.read();
  }
}

/* One fixed-text AT command ("AT+NAME?\r\n" or "AT+NAME\r\n") and its "+NAME: <payload>" reply.
 * Returns false on timeout. Stock firmware ends up here: it answers "ERROR", which the WiFiS3
 * parser (core 1.6.0, Modem.cpp:7) never recognises, so only the timeout ends the wait. */
bool runCommand(const char* prompt, const char* request, size_t timeoutMs, std::string& payload) {
  drainModemUart();
  ModemTimeout timeout(timeoutMs);
  return modem.write(std::string(prompt), payload, "%s", request);
}

}  // namespace

bool XboxControllerR4::begin() {
  lastError_ = XBOX_R4_ERR_NONE;
  beginResult_ = XBOX_R4_RESULT_NO_ANSWER;
  protocolVersion_ = 0;
  firmwareVersion_[0] = '\0';

  /* Opens Serial2 and sends AT+SOFTRESETWIFI (3 tries, 500 ms each) the first time only;
   * later calls, including the ones WiFiS3 makes, return at once. */
  modem.begin();

  if (!queryVersion()) {
    /* A stock command tells "firmware without Xbox support" apart from "ESP32-S3 silent".
     * modem.beginned cannot: it stays true after the first answered AT+SOFTRESETWIFI, even
     * when the ESP32-S3 has since crashed, rebooted or been reflashed. */
    lastError_ = probeBridge() ? XBOX_R4_ERR_NO_FIRMWARE_SUPPORT : XBOX_R4_ERR_BRIDGE_SILENT;
    return false;
  }
  if (protocolVersion_ != XBOX_R4_PROTOCOL_VERSION) {
    lastError_ = XBOX_R4_ERR_PROTOCOL_MISMATCH;
    return false;
  }
  if (!sendBegin()) {
    lastError_ = XBOX_R4_ERR_BEGIN_FAILED;
    return false;
  }
  return true;
}

/* AT+XBOXVERSION? -> "+XBOXVERSION: <proto>,<fw>". The parser drops spaces from plain-text
 * replies, so the payload arrives as e.g. "1,0.6.99". A payload that does not start with a
 * number followed by ',' or its end is not an answer this library understands: returns false. */
bool XboxControllerR4::queryVersion() {
  std::string text;
  if (!runCommand(PROMPT(XBOX_R4_AT_VERSION), CMD_READ(XBOX_R4_AT_VERSION), XBOX_R4_CMD_TIMEOUT_MS, text)) {
    return false;
  }
  size_t digits = 0;
  while (digits < text.size() && isdigit(static_cast<unsigned char>(text[digits]))) {
    digits++;
  }
  if (digits == 0 || (digits < text.size() && text[digits] != ',')) {
    return false;
  }
  protocolVersion_ = atoi(text.c_str());
  if (digits < text.size()) {
    storeFirmwareVersion(text, digits + 1);
  }
  return true;
}

/* AT+FWVERSION? -> "+FWVERSION: <version>". Stock and forked bridge firmware both answer it
 * (UNOR4USBBridge/cmds_esp_generic.h), and unlike AT+SOFTRESETWIFI it leaves WiFi sockets open.
 * Returns true when the ESP32-S3 answered; keeps the version for firmwareVersion(). */
bool XboxControllerR4::probeBridge() {
  std::string text;
  if (!runCommand(PROMPT(_FWVERSION), CMD_READ(_FWVERSION), XBOX_R4_CMD_TIMEOUT_MS, text)) {
    return false;
  }
  storeFirmwareVersion(text, 0);
  return true;
}

/* Copies text from `start` into firmwareVersion_, cut to fit its 15 characters. */
void XboxControllerR4::storeFirmwareVersion(const std::string& text, size_t start) {
  size_t n = 0;
  for (size_t i = start; i < text.size() && n < sizeof(firmwareVersion_) - 1; i++) {
    firmwareVersion_[n++] = text[i];
  }
  firmwareVersion_[n] = '\0';
}

/* AT+XBOXBEGIN -> "+XBOXBEGIN: <code>". Code 0 means started or already running. */
bool XboxControllerR4::sendBegin() {
  std::string text;
  if (!runCommand(PROMPT(XBOX_R4_AT_BEGIN), CMD(XBOX_R4_AT_BEGIN), XBOX_R4_SLOW_CMD_TIMEOUT_MS, text)) {
    return false;
  }
  beginResult_ = parseResultCode(text);
  return beginResult_ == XBOX_R4_RC_OK;
}

bool XboxControllerR4::poll() {
  newReport_ = false;

  bool ok;
  {
    drainModemUart();
    ModemTimeout timeout(XBOX_R4_POLL_TIMEOUT_MS);
    /* Trimming only touches the text before '|' (Modem.cpp:296-325), so it cannot alter the
     * sized payload. Turned off anyway, as ArduinoBLE and Preferences do before sized reads.
     * It is one-shot: the parser turns trimming back on after this exchange. */
    modem.avoid_trim_results();
    ok = modem.write(std::string(PROMPT(XBOX_R4_AT_STATE)), reply_, "%s", CMD_READ(XBOX_R4_AT_STATE));
  }
  if (!ok) {
    pollErrors_++;
    return false;
  }

  /* reply_ holds the raw payload bytes, NULs included: use size() and data(), never c_str(). */
  xbox_r4_state_t st;
  if (reply_.size() != XBOX_R4_STATE_LEN ||
      !xbox_r4_decode(reinterpret_cast<const uint8_t*>(reply_.data()), reply_.size(), &st)) {
    pollErrors_++;
    return false;
  }
  lastPollOkMillis_ = millis();

  bool wasConnected = (state_ == XBOX_R4_CONNECTED);
  state_ = st.state;
  battery_ = st.battery;

  if (st.state == XBOX_R4_CONNECTED) {
    /* Apply every valid connected frame. The ESP32-S3 snapshots report and seq together, but
     * applying unconditionally costs nothing and can never leave a report behind. seq only
     * answers "is this new": an unchanged seq means no report arrived since the last poll,
     * and the first poll after connecting always counts. */
    xboxNotif.update(st.report, XBOX_R4_REPORT_LEN);
    if (!wasConnected || st.seq != seq_) {
      newReport_ = true;
      lastReportMillis_ = millis();
    }
  } else if (wasConnected) {
    /* The controller dropped: read as sticks centred and nothing pressed rather than freezing
     * the last stick position, so a sketch driving motors stops. hasNewReport() reports the
     * change, so a sketch that acts only on new reports also sees the neutral sticks. */
    loadNeutralReport();
    newReport_ = true;
  }
  seq_ = st.seq;
  return true;
}

void XboxControllerR4::loadNeutralReport() {
  uint8_t report[XBOX_R4_REPORT_LEN];
  xbox_r4_neutral_report(report);
  xboxNotif.update(report, XBOX_R4_REPORT_LEN);
}

bool XboxControllerR4::scanReport(char* out, size_t cap) {
  if (cap == 0) return false;
  out[0] = '\0';
  std::string text;
  if (!runCommand(PROMPT(XBOX_R4_AT_SCAN), CMD_READ(XBOX_R4_AT_SCAN), XBOX_R4_SLOW_CMD_TIMEOUT_MS, text)) {
    return false;
  }
  size_t n = text.size() < cap - 1 ? text.size() : cap - 1;
  memcpy(out, text.data(), n);
  out[n] = '\0';
  return true;
}

bool XboxControllerR4::peerAddress(char out[18]) {
  out[0] = '\0';
  std::string text;
  if (!runCommand(PROMPT(XBOX_R4_AT_ADDR), CMD_READ(XBOX_R4_AT_ADDR), XBOX_R4_CMD_TIMEOUT_MS, text)) {
    return false;
  }
  const size_t addressLen = 17;  /* "aa:bb:cc:dd:ee:ff" */
  if (text.size() != addressLen) {
    return false;
  }
  memcpy(out, text.data(), addressLen);
  out[addressLen] = '\0';
  return true;
}

bool XboxControllerR4::forget() {
  std::string text;
  if (!runCommand(PROMPT(XBOX_R4_AT_FORGET), CMD(XBOX_R4_AT_FORGET), XBOX_R4_SLOW_CMD_TIMEOUT_MS, text)) {
    return false;
  }
  return parseResultCode(text) == XBOX_R4_RC_OK;
}

/* AT+XBOXLOG=<0|1> -> "+XBOXLOG: <0|1>". Uses the Write form directly because the value is
 * formatted into the request. */
bool XboxControllerR4::setLogging(bool on) {
  int value = on ? 1 : 0;
  std::string text;
  drainModemUart();
  ModemTimeout timeout(XBOX_R4_CMD_TIMEOUT_MS);
  if (!modem.write(std::string(PROMPT(XBOX_R4_AT_LOG)), text, "%s%d\r\n", CMD_WRITE(XBOX_R4_AT_LOG), value)) {
    return false;
  }
  return parseResultCode(text) == value;
}

/* A result code is a bare decimal number. An empty or non-numeric payload is not "0". */
int XboxControllerR4::parseResultCode(const std::string& text) {
  if (text.empty() || !isdigit(static_cast<unsigned char>(text[0]))) {
    return XBOX_R4_RESULT_NO_ANSWER;
  }
  return atoi(text.c_str());
}

/* 0 = full left/up, 65535 = full right/down, centre about 32768. */
float XboxControllerR4::axisToFloat(uint16_t raw) {
  return (static_cast<float>(raw) - 32768.0f) / 32768.0f;
}

float XboxControllerR4::triggerToFloat(uint16_t raw) {
  float v = static_cast<float>(raw) / static_cast<float>(XboxControllerNotificationParser::maxTrig);
  return v > 1.0f ? 1.0f : v;
}

int16_t XboxControllerR4::axisToRawX(uint16_t raw) {
  return static_cast<int16_t>(static_cast<int32_t>(raw) - 32768);
}

/* Mirrored so up is positive: 0 (full up) -> 32767, 65535 (full down) -> -32768. */
int16_t XboxControllerR4::axisToRawY(uint16_t raw) {
  return static_cast<int16_t>(32767 - static_cast<int32_t>(raw));
}

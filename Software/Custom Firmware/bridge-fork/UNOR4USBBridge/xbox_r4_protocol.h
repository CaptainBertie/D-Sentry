/*
 * xbox_r4_protocol.h -- wire format shared by the UNO R4 WiFi ESP32-S3 bridge fork
 * (firmware/UNOR4USBBridge/xbox_r4_protocol.h) and the RA4M1 library
 * (ra4m1/libraries/XboxControllerR4/src/xbox_r4_protocol.h).
 *
 * The canonical copy is protocol/xbox_r4_protocol.h. The other two must be byte-identical;
 * tests/test_protocol_copies.sh checks that.
 *
 * SPDX-License-Identifier: MIT
 * Copyright (c) 2026 the UNO R4 WiFi Xbox controller project
 */
#pragma once
#include <stdint.h>
#include <stddef.h>
#include <string.h>
#include <stdbool.h>

/* ---- AT command names (the "+NAME" part; the RA4M1 side sends "AT" "+NAME" ... "\r\n") ---- */
#define XBOX_R4_AT_VERSION  "+XBOXVERSION"   /* Read:  AT+XBOXVERSION?  -> "+XBOXVERSION: <proto>,<fw>\r\nOK\r\n" */
#define XBOX_R4_AT_BEGIN    "+XBOXBEGIN"     /* Run:   AT+XBOXBEGIN     -> "+XBOXBEGIN: <code>\r\nOK\r\n"          */
#define XBOX_R4_AT_STATE    "+XBOXSTATE"     /* Read:  AT+XBOXSTATE?    -> "+XBOXSTATE: 20|<20 raw bytes>\r\nOK\r\n" */
#define XBOX_R4_AT_ADDR     "+XBOXADDR"      /* Read:  AT+XBOXADDR?     -> "+XBOXADDR: aa:bb:cc:dd:ee:ff\r\nOK\r\n" */
#define XBOX_R4_AT_FORGET   "+XBOXFORGET"    /* Run:   AT+XBOXFORGET    -> "+XBOXFORGET: <code>\r\nOK\r\n"         */
#define XBOX_R4_AT_LOG      "+XBOXLOG"       /* Write: AT+XBOXLOG=<0|1> -> "+XBOXLOG: <0|1>\r\nOK\r\n"             */
#define XBOX_R4_AT_SCAN     "+XBOXSCAN"      /* Read:  AT+XBOXSCAN?     -> "+XBOXSCAN: <len>|<text>\r\nOK\r\n": diagnostic, the last
                                              * BLE adverts the ESP32-S3 saw while scanning, one per ';':
                                              * addr,appearance,rssi,mfgdata_hex,first_service_uuid,name,count          */

/* Every known command always answers with its prompt and OK, even on failure, and puts the
 * failure in the payload. Reason: the RA4M1's WiFiS3 Modem parser (core 1.6.0) never
 * recognises "ERROR\r\n" and would only report it after its full timeout. */

#define XBOX_R4_PROTOCOL_VERSION 1

/* ---- AT+XBOXBEGIN / AT+XBOXFORGET result codes ---- */
#define XBOX_R4_RC_OK             0   /* started, or already running */
#define XBOX_R4_RC_BLOCKED_BY_HCI 1   /* AT+HCIBEGIN (ArduinoBLE on the RA4M1) owns the BLE controller; AT+RESET first */
#define XBOX_R4_RC_INIT_FAILED    2   /* NimBLE init or task creation failed */
#define XBOX_R4_RC_NOT_RUNNING    3   /* reserved; firmware 0.6.99 answers FORGET with 0 before and after BEGIN */

/* ---- AT+XBOXSTATE payload: exactly 20 bytes, little-endian where multi-byte ---- */
#define XBOX_R4_STATE_LEN   20
#define XBOX_R4_REPORT_LEN  16

/* Bytes 0..15: the controller's 16-byte HID input report exactly as
 * XboxControllerNotificationParser::toArr() writes it:
 *   0-1  joyLHori  uint16 LE  0 = full left,  65535 = full right (centre ~32768)
 *   2-3  joyLVert  uint16 LE  0 = full up,    65535 = full down
 *   4-5  joyRHori  uint16 LE
 *   6-7  joyRVert  uint16 LE
 *   8-9  trigLT    uint16 LE  0..1023
 *  10-11 trigRT    uint16 LE  0..1023
 *  12    d-pad     0 none, 1 N, 2 NE, 3 E, 4 SE, 5 S, 6 SW, 7 W, 8 NW
 *  13    buttons   bit0 A, bit1 B, bit3 X, bit4 Y, bit6 LB, bit7 RB
 *  14    buttons   bit2 View(Select), bit3 Menu(Start), bit4 Xbox, bit5 LS click, bit6 RS click
 *  15    buttons   bit0 Share
 * Byte 16: connection state (xbox_r4_conn_state below)
 * Byte 17: battery percent 0..100, or XBOX_R4_BATTERY_UNKNOWN
 * Byte 18: seq, low byte of the count of HID reports received since AT+XBOXBEGIN
 * Byte 19: CRC-8 over bytes 0..18 (poly 0x07, init 0x00, no reflection, no xor-out) */

enum xbox_r4_conn_state {
  XBOX_R4_CONNECTED            = 0,  /* reports flowing; bytes 0..15 are live */
  XBOX_R4_WAITING_FIRST_REPORT = 1,  /* link up, encryption/subscription in progress */
  XBOX_R4_FOUND                = 2,  /* controller seen in a scan, connecting */
  XBOX_R4_SCANNING             = 3,  /* no controller connected, scanning */
  XBOX_R4_OFF                  = 4,  /* AT+XBOXBEGIN has not been sent */
  XBOX_R4_BLOCKED_BY_HCI       = 5   /* ArduinoBLE HCI bridge active; Xbox mode unavailable until AT+RESET */
};

#define XBOX_R4_BATTERY_UNKNOWN 0xFF

typedef struct {
  uint8_t report[XBOX_R4_REPORT_LEN];
  uint8_t state;    /* enum xbox_r4_conn_state */
  uint8_t battery;  /* 0..100 or XBOX_R4_BATTERY_UNKNOWN */
  uint8_t seq;
} xbox_r4_state_t;

static inline uint8_t xbox_r4_crc8(const uint8_t* data, size_t len) {
  uint8_t crc = 0x00;
  for (size_t i = 0; i < len; i++) {
    crc ^= data[i];
    for (int b = 0; b < 8; b++) {
      crc = (uint8_t)((crc & 0x80) ? ((crc << 1) ^ 0x07) : (crc << 1));
    }
  }
  return crc;
}

static inline void xbox_r4_encode(const xbox_r4_state_t* s, uint8_t out[XBOX_R4_STATE_LEN]) {
  memcpy(out, s->report, XBOX_R4_REPORT_LEN);
  out[16] = s->state;
  out[17] = s->battery;
  out[18] = s->seq;
  out[19] = xbox_r4_crc8(out, XBOX_R4_STATE_LEN - 1);
}

/* Returns false on wrong length or bad CRC; *s is untouched in that case. */
static inline bool xbox_r4_decode(const uint8_t* in, size_t len, xbox_r4_state_t* s) {
  if (len != XBOX_R4_STATE_LEN) return false;
  if (xbox_r4_crc8(in, XBOX_R4_STATE_LEN - 1) != in[19]) return false;
  memcpy(s->report, in, XBOX_R4_REPORT_LEN);
  s->state   = in[16];
  s->battery = in[17];
  s->seq     = in[18];
  return true;
}

/* A neutral report: sticks centred, triggers released, nothing pressed. */
static inline void xbox_r4_neutral_report(uint8_t report[XBOX_R4_REPORT_LEN]) {
  memset(report, 0, XBOX_R4_REPORT_LEN);
  report[0] = 0xFF; report[1] = 0x7F;  /* 32767 */
  report[2] = 0xFF; report[3] = 0x7F;
  report[4] = 0xFF; report[5] = 0x7F;
  report[6] = 0xFF; report[7] = 0x7F;
}

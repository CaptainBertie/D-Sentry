/*
 * xbox_client.h -- glue between the UNO R4 WiFi bridge firmware and the
 * XboxSeriesXControllerESP32_asukiaaa BLE client (NimBLE-Arduino).
 *
 * This is the only header the rest of the bridge includes. It deliberately exposes a small
 * C-style API so that the NimBLE and Xbox library headers (which define per-translation-unit
 * static objects) are included in exactly one file, xbox_client.cpp.
 *
 * SPDX-License-Identifier: MIT
 * Copyright (c) 2026 the UNO R4 WiFi Xbox controller project
 * The combined firmware is governed by the AGPL-3.0 (see DECISIONS.md D04).
 */
#pragma once
#include <stdint.h>
#include <stdbool.h>
#include "xbox_r4_protocol.h"

#ifdef __cplusplus
extern "C" {
#endif

/* AT+XBOXBEGIN: start NimBLE and the controller client task. Idempotent.
 * Returns an XBOX_R4_RC_* code. */
int xbox_begin(void);

/* AT+XBOXSTATE?: fill the 20-byte payload. Always succeeds; before BEGIN the state is
 * XBOX_R4_OFF (or XBOX_R4_BLOCKED_BY_HCI) with a neutral report. */
void xbox_fill_state(uint8_t out[XBOX_R4_STATE_LEN]);

/* AT+XBOXADDR?: "aa:bb:cc:dd:ee:ff" of the connected or last connected controller,
 * "00:00:00:00:00:00" if none. out must hold 18 bytes. */
void xbox_peer_address(char out[18]);

/* AT+XBOXFORGET: delete every BLE bond and the remembered controller address, then accept
 * only a controller in pairing mode. Works before or after BEGIN. After BEGIN the bond work
 * happens in the client task within a few seconds, or up to about two minutes if a connection
 * attempt is failing (four 30 s connect timeouts). Returns XBOX_R4_RC_OK. */
int xbox_forget(void);

/* AT+XBOXSCAN?: diagnostic. Writes a text report of the last adverts the scanner saw
 * ("addr/type,appearance,rssi,mfgdata_hex,first_service_uuid,name,count;" per device, "none"
 * if nothing yet, "off" before BEGIN). Returns the number of characters written. */
size_t xbox_scan_report(char* out, size_t cap);

/* AT+XBOXLOG=<0|1>: diagnostics on the USB CDC port (the Serial Monitor). Default off. */
void xbox_set_log(bool on);
bool xbox_log_enabled(void);

/* Mutual exclusion with the stock ArduinoBLE HCI bridge (AT+HCIBEGIN). The BLE controller
 * has one VHCI host interface; whichever side starts first keeps it until AT+RESET. */
bool xbox_is_active(void);      /* true once AT+XBOXBEGIN has initialised NimBLE (D-Sentry: or
                                 * while D-Sentry WiFi mode owns the radio, so +HCIBEGIN is
                                 * refused then too; BLE and WiFi never run together) */
void xbox_note_hci_used(void);  /* the +HCIBEGIN handler calls this on success */
bool xbox_hci_used(void);

/* ---- D-Sentry additions (docs: D-SentryCode/docs/architecture.md 4.2, DECISIONS.md D19) ---- */

/* D-Sentry: true once NimBLE has been initialised by xbox_begin() (what xbox_is_active() meant
 * before D-Sentry). */
bool xbox_ble_running(void);

/* D-Sentry: the Xbox pad's BLE HID output report (report ID 3, characteristic 0x2A4D) is
 * 8 bytes; dsentry_wire.h rumble_report() builds it. */
#define XBOX_RUMBLE_REPORT_LEN 8

/* D-Sentry: hand one rumble report to the client task, which writes it (without response) to
 * the pad's 0x2A4D output characteristic right after its next onLoop(), within about 10 ms
 * while connected. Latest request wins; a request made while no pad is connected is dropped.
 * Returns XBOX_R4_RC_OK, or XBOX_R4_RC_NOT_RUNNING when xbox_begin() has not run. AT task. */
int xbox_request_rumble_report(const uint8_t report[XBOX_RUMBLE_REPORT_LEN]);

/* D-Sentry: send ESP-IDF logs from every task to USB instead of UART0 (the RA4M1's Serial RX),
 * as xbox_begin() does. D-Sentry WiFi mode calls it when it starts. Idempotent. */
void xbox_route_idf_log_to_usb(void);

/* D-Sentry: one "[tag] ..." diagnostic line on USB, only while AT+XBOXLOG=1 (the same rule
 * and sink as the Xbox client's own log). */
void xbox_logf_tag(const char* tag, const char* fmt, ...) __attribute__((format(printf, 2, 3)));

#ifdef __cplusplus
}
#endif

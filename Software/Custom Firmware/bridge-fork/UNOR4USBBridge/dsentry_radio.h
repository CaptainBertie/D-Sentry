/*
 * dsentry_radio.h -- the D-Sentry radio modes on the ESP32-S3 bridge (ESP32 build only).
 *
 * BLE mode: the stock Xbox client (xbox_begin(), same as AT+XBOXBEGIN) plus rumble through the
 * client task. WiFi mode: a FreeRTOS task that joins WIFI_SSID (or hosts WIFI_AP_SSID), takes
 * ControlPackets from the PC relay on UDP WIFI_UDP_PORT, serves AT+XBOXSTATE? from the latest
 * one, sends StatusPackets and AT+DST telemetry back to the relay and broadcasts a BeaconPacket.
 * The two never run in the same boot: switching stores the wanted mode in RTC_NOINIT memory and
 * restarts the ESP32 (docs/architecture.md 4.2, DECISIONS.md D19). With no D-Sentry command and
 * no stored request the firmware boots radio-less and behaves exactly like the fork before.
 *
 * SPDX-License-Identifier: MIT
 * Copyright (c) 2026 the D-Sentry project
 * The combined firmware is governed by the AGPL-3.0 (see DECISIONS.md D04).
 */
#pragma once
#include <stdbool.h>
#include <stdint.h>

#include "xbox_r4_protocol.h"

#ifdef __cplusplus
extern "C" {
#endif

/* setup(), before the AT task starts: choose this boot's bootId and, only after a software
 * reset that left a valid mode request in RTC_NOINIT memory, start that radio (one shot). */
void dsentry_boot(void);

/* A D-Sentry radio mode (BLE or WiFi) was started or adopted this boot. AT+SOFTRESETWIFI is a
 * no-op while this is true. */
bool dsentry_radio_mode_active(void);

/* The D-Sentry WiFi task was started this boot. */
bool dsentry_wifi_mode_active(void);

/* AT+XBOXSTATE? in WiFi mode: fills out from the relay's latest valid ControlPacket and
 * returns true; returns false (out untouched) when WiFi mode is not active. */
bool dsentry_fill_xbox_state(uint8_t out[XBOX_R4_STATE_LEN]);

/* Wait ESP_RESTART_DELAY_MS, then esp_restart(). Runs as an AT-handler task, after the
 * "+DSS: 3,..." reply has been written and flushed. Does not return. */
void dsentry_restart_now(void);

#ifdef __cplusplus
}
#endif

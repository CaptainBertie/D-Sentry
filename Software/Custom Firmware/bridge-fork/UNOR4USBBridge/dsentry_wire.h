/*
 * dsentry_wire.h -- the logic of the D-Sentry additions to the bridge, with no Arduino,
 * FreeRTOS, NimBLE or WiFi in it, so that D-SentryCode/tests/esp32 compiles exactly this code
 * on the Mac (clang++ -std=gnu++17 -Wall -Wextra -Werror) and proves the bytes it produces.
 *
 * What lives here (every function is pure: inputs in, bytes or decisions out):
 *   - AT argument parsing and reply formatting for AT+DSCAPS?, AT+DSS=..., AT+DST=<hex>
 *     (D-SentryCode/protocol/protocol_spec.py, AT_COMMANDS);
 *   - the radio decision: start in place, already running, or restart into the other radio
 *     (BLE and WiFi never run together; docs/architecture.md 4.2);
 *   - the RTC_NOINIT mode request that survives esp_restart(), and the per-boot bootId;
 *   - WiFi mode: which UDP datagrams are accepted, the 20-byte AT+XBOXSTATE? payload built from
 *     the relay's latest ControlPacket, the StatusPacket and BeaconPacket frames, and the
 *     station/access-point supervisor;
 *   - the Xbox pad's 8-byte BLE rumble output report.
 * dsentry_radio.cpp (ESP32 only) wires these to the real radios; dsentry_at.cpp wires them to
 * the chAT server.
 *
 * SPDX-License-Identifier: MIT
 * Copyright (c) 2026 the D-Sentry project
 * The combined firmware is governed by the AGPL-3.0 (see DECISIONS.md D04).
 */
#pragma once
#include <stddef.h>
#include <stdint.h>

#include <string>
#include <vector>

#include "dsentry_config.h"
#include "dsentry_protocol.h"
#include "xbox_client.h"
#include "xbox_r4_protocol.h"

namespace dsentry {

/* ---- AT arguments and replies --------------------------------------------------------- */

/* Strict decimal: one or more '0'..'9' and nothing else (no sign, no spaces), value <= max.
 * Returns false (and leaves *out alone) otherwise, including on overflow. */
bool parse_uint(const std::string& text, uint32_t max, uint32_t* out);

/* AT+DSS=<wantMode>,<rumbleSeq>,<strong>,<weak>,<rumbleMs>,<state>: the StatusPacket fields. */
struct DssArgs {
  uint8_t wantMode;    /* DS_RADIO_*; anything >= DS_RADIO_COUNT is rejected */
  uint8_t rumbleSeq;
  uint8_t strong;      /* 0..255 */
  uint8_t weak;        /* 0..255 */
  uint16_t rumbleMs;   /* 0 = stop */
  uint8_t state;       /* DS_STATE_*; any u8 is accepted (it is only logged and forwarded) */
};

/* True only for exactly DS_AT_DSS_ARGC strict-decimal arguments in range. */
bool parse_dss(const std::vector<std::string>& args, DssArgs* out);

/* AT+DSCAPS? capBits: everything this firmware implements. */
uint8_t caps_bits();

/* Reply payloads (what follows "+NAME: "): "<dsProto>,<capBits>", "<espMode>,<bootId>,<wifiStatus>",
 * "<sent>". Decimal, no padding, comma separated, exactly as docs/protocol.md shows. */
std::string format_caps_reply();
std::string format_dss_reply(uint8_t espMode, uint8_t bootId, uint8_t wifiStatus);
std::string format_dst_reply(uint8_t sent);

/* AT+DST=<hex>: returns DS_TELEMETRY_FRAME_LEN and fills out only when hex is exactly
 * DS_AT_DST_HEX_LEN hex digits (either case) that decode to one valid TelemetryPacket frame
 * (start byte, len byte and CRC); 0 otherwise. The version byte is not judged: the bytes are
 * forwarded unchanged and the relay checks it. */
size_t parse_dst(const std::string& hex, uint8_t out[DS_TELEMETRY_FRAME_LEN]);

/* ---- which radio runs ----------------------------------------------------------------- */

/* What is running right now, as the AT task sees it. */
struct Radios {
  bool bleRunning;        /* NimBLE started by the Xbox client (AT+XBOXBEGIN or D-Sentry BLE) */
  bool hciUsed;           /* the stock ArduinoBLE HCI bridge took the BLE controller this boot */
  bool dsWifiRunning;     /* the D-Sentry WiFi task was started this boot */
  bool otherWifiRunning;  /* the WiFi driver is on but not ours (stock AT+BEGINSTA, SOFTRESETWIFI ...) */
  bool restartPending;    /* a mode-switch restart is already queued */
};

enum class ModeAction : uint8_t {
  INVALID,     /* wantMode is not a DS_RADIO_* value: change nothing */
  ALREADY,     /* exactly the wanted radio is running: nothing to start */
  START_BLE,   /* nothing radio-wise is running: start the BLE client in place */
  START_WIFI,  /* nothing radio-wise is running: start the WiFi task in place */
  RESTART      /* something else holds a radio: store the wanted mode and esp_restart() */
};

/* The rule that keeps BLE and WiFi apart: a radio is started in place only when no other
 * radio user (NimBLE, the HCI bridge, any WiFi) is active; otherwise the ESP32 restarts and the
 * next boot starts the wanted radio alone. While a restart is pending every request is RESTART
 * (the latest wanted mode is stored, no second restart is queued). */
ModeAction decide_mode(uint8_t wantMode, const Radios& radios);

/* espMode for the AT+DSS reply: RESTARTING, WIFI, BLE or NONE, in that priority. */
uint8_t esp_mode_of(const Radios& radios);

/* ---- rumble ----------------------------------------------------------------------------- */

/* A rumble is played only when AT+DSS carries a rumbleSeq different from the previous one.
 * The first AT+DSS after an ESP32 boot only sets the baseline, so a heartbeat that repeats an
 * old shot after a reboot or mode switch never rumbles again. */
class RumbleSeqTracker {
 public:
  bool is_new(uint8_t seq);
  void reset() { have_ = false; }

 private:
  bool have_ = false;
  uint8_t last_ = 0;
};

/* The Xbox pad's BLE HID output report (report ID 3 behind characteristic 0x2A4D), 8 bytes:
 *   [0] enable bits: 0x01 weak motor, 0x02 strong motor, 0x04 right trigger, 0x08 left trigger
 *   [1] left trigger power  [2] right trigger power  [3] strong power  [4] weak power  (0..100)
 *   [5] on time, 10 ms units  [6] off time between repeats, 10 ms  [7] repeat count
 * (Linux drivers/hid/hid-microsoft.c struct xb1s_ff_report minus the report ID byte, and
 * XboxSeriesXHIDReportBuilder_asukiaaa ReportBase.) Both grip motors are always enabled, as
 * Linux does; strengths 0..255 are scaled to 0..100 with rounding. ms > 0: the time is rounded
 * up to 10 ms units, and anything longer than 255 units is split into equal repeats, so the
 * total stays within one unit per repeat of ms (65535 ms at most). ms == 0: both grip motors
 * to 0, which stops a running rumble. The trigger motors are never driven. */
void rumble_report(uint8_t strong, uint8_t weak, uint16_t ms, uint8_t out[XBOX_RUMBLE_REPORT_LEN]);

/* ---- WiFi mode: what comes in from the relay ------------------------------------------- */

/* A UDP datagram is a ControlPacket only when it is exactly DS_CONTROL_FRAME_LEN bytes with the
 * ControlPacket start byte, len byte and a matching CRC (architecture.md 4.1). */
bool accept_control(const uint8_t* datagram, size_t len, ds_control_packet_t* out);

/* The relay counts as connected only while its latest ControlPacket says connected and arrived
 * less than LINK_TIMEOUT_MS ago (wrap-safe). This is the WiFi-mode failsafe: a relay that goes
 * silent reads as "scanning" on the very next AT+XBOXSTATE? after LINK_TIMEOUT_MS. */
bool control_is_live(bool have, const ds_control_packet_t& packet, uint32_t rxMs, uint32_t nowMs);

/* Any valid ControlPacket (connected or not) within LINK_TIMEOUT_MS: wifiStatus RELAY_HEARD. */
bool relay_heard(bool have, uint32_t rxMs, uint32_t nowMs);

/* The d-pad code (XSTATE_DPAD_*) for the four DS_BTN_DPAD_* bits; opposite directions cancel. */
uint8_t dpad_code(uint16_t buttons);

/* The 20-byte AT+XBOXSTATE? payload in WiFi mode (protocol/xbox_r4_protocol.h), the inverse of
 * what the RA4M1 does with it (architecture.md 4.2): right stick (rx, ry clamped to
 * -JOY_AXIS_RANGE..JOY_AXIS_RANGE-1) to 0..65535 with centre XSTATE_AXIS_CENTRE, so
 * (raw - centre) >> XSTATE_AXIS_SHIFT gives rx back exactly; trigger clamped to
 * TRIGGER_AXIS_RANGE; A B X Y LB RB to byte 13, start to Menu and back to View in byte 14;
 * the d-pad bits to the d-pad code; seq = the packet's seq; battery unknown. State is
 * XBOX_R4_CONNECTED when live, else XBOX_R4_SCANNING with the neutral report (packet may be
 * null: nothing received yet, seq 0). */
void control_to_xstate(const ds_control_packet_t* packet, bool live, uint8_t out[XBOX_R4_STATE_LEN]);

/* ---- WiFi mode: what goes out to the relay --------------------------------------------- */

/* The StatusPacket the relay gets on a new rumbleSeq: the AT+DSS fields unchanged. */
size_t build_status_frame(const DssArgs& args, uint8_t out[DS_STATUS_FRAME_LEN]);

/* The BeaconPacket broadcast every WIFI_BEACON_MS: ip first octet first, udpPort WIFI_UDP_PORT. */
size_t build_beacon_frame(const uint8_t ip[DS_BEACON_IP_LEN], uint8_t bootId, uint8_t wifiStatus,
                          uint8_t out[DS_BEACON_FRAME_LEN]);

/* ---- mode request kept across esp_restart() (RTC_NOINIT memory) ------------------------ */

/* Four words in RTC slow memory. After a power-on they hold garbage, which store_valid()
 * rejects (magic and check word); esp_restart() leaves them as written. */
struct ModeStore {
  uint32_t magic;
  uint32_t request;     /* DS_ESP_MODE_BLE / _WIFI: start this radio at the next software boot;
                         * DS_ESP_MODE_NONE: no request */
  uint32_t lastBootId;  /* the bootId the last boot chose */
  uint32_t check;
};

bool store_valid(const ModeStore& store);
void store_write(ModeStore* store, uint8_t request, uint8_t lastBootId);

/* Never 0 (0 means "no reply yet" on the RA4M1) and never the previous boot's id, so a reboot
 * always changes bootId even when the random byte repeats. */
uint8_t pick_boot_id(uint32_t random, uint8_t previous);

struct BootPlan {
  uint8_t startMode;   /* DS_ESP_MODE_BLE, DS_ESP_MODE_WIFI or DS_ESP_MODE_NONE */
  uint8_t bootId;
};

/* At every boot: a radio is started only when this is a software reset (ESP_RST_SW) and the
 * store holds a valid request. The request is consumed (one shot), so AT+RESET, a crash or a
 * power cycle all come back radio-less, exactly like the stock firmware. The new bootId is
 * recorded in the store. */
BootPlan plan_boot(ModeStore* store, bool softwareReset, uint32_t random);

/* ---- WiFi station / access point supervisor -------------------------------------------- */

class WifiSupervisor {
 public:
  enum Phase : uint8_t {
    IDLE,        /* not started */
    JOINING,     /* station join in progress (first attempt, or retries without AP fallback) */
    STA_UP,      /* joined WIFI_SSID */
    STA_LOST,    /* was joined, dropped (router, or the 1200/2400-baud upload touch), retrying */
    AP_UP,       /* own access point WIFI_AP_SSID (fallback; stays until the next restart) */
    NO_NETWORK   /* no SSID configured and no AP fallback: nothing to do */
  };
  enum Action : uint8_t {
    NONE,
    STA_BEGIN,   /* WiFi.mode(STA) + WiFi.begin(WIFI_SSID, WIFI_PASSWORD) */
    STA_REBEGIN, /* WiFi.begin(WIFI_SSID, WIFI_PASSWORD) again */
    AP_START     /* drop the station, WiFi.mode(AP), softAPConfig(WIFI_AP_IP ...), softAP(...) */
  };

  /* haveSsid false (WIFI_SSID empty) counts as a station that failed at once. */
  Action start(uint32_t nowMs, bool haveSsid, bool apFallback);
  /* Call often. Station joined -> STA_UP. With AP fallback, a first join that has not
   * succeeded after WIFI_STA_CONNECT_TIMEOUT_MS -> AP_START. Without it, a new attempt every
   * WIFI_STA_RETRY_MS. A drop after joining -> STA_REBEGIN at once (the upload touch's
   * WiFi.disconnect() never reconnects by itself), then every WIFI_STA_RETRY_MS. */
  Action step(uint32_t nowMs, bool staConnected);
  Phase phase() const { return phase_; }
  bool network_up() const { return phase_ == STA_UP || phase_ == AP_UP; }

 private:
  Phase phase_ = IDLE;
  uint32_t t0_ = 0;
  bool apFallback_ = false;
};

/* wifiStatus (DS_WIFI_STATUS_*) for the AT+DSS reply and the beacon. */
uint8_t wifi_status(bool wifiStarted, WifiSupervisor::Phase phase, bool relayHeard);

}  /* namespace dsentry */

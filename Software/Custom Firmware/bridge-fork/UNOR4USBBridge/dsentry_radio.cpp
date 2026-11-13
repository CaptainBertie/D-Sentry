/*
 * dsentry_radio.cpp -- the D-Sentry radio modes on the ESP32-S3 (see dsentry_radio.h).
 * The decisions and byte formats are in dsentry_wire.cpp (host-tested); this file only
 * connects them to NimBLE (through xbox_client.h), the WiFi driver, lwIP sockets, RTC memory
 * and esp_restart().
 *
 * Threads:
 *   - setup() (loopTask, core 1): dsentry_boot(), before the AT task exists.
 *   - AT task (Task1, core 0): the DsentryBackend methods, dsentry_fill_xbox_state().
 *   - "dswifi" task (core 0): everything that calls WiFi.* or the UDP socket in WiFi mode.
 *     It is the only owner of the WiFi state and the only sender on the socket; the AT task
 *     hands it StatusPackets and telemetry through latest-wins slots under g_mux.
 *
 * SPDX-License-Identifier: MIT
 * Copyright (c) 2026 the D-Sentry project
 * The combined firmware is governed by the AGPL-3.0 (see DECISIONS.md D04).
 */
#include "dsentry_radio.h"

#include <Arduino.h>
#include <WiFi.h>
#include <esp_attr.h>
#include <esp_system.h>
#include <lwip/sockets.h>
#include <string.h>

#include "dsentry_at.h"
#include "dsentry_wire.h"
#include "xbox_client.h"

namespace {

const char* const kLogTag = "dsentry";

/* ---- RTC_NOINIT mode request ------------------------------------------------------------ */

RTC_NOINIT_ATTR dsentry::ModeStore g_rtcStore;   /* garbage after power-on; survives esp_restart() */

dsentry::ModeStore rtc_load() {
  volatile dsentry::ModeStore* p = &g_rtcStore;
  dsentry::ModeStore s;
  s.magic = p->magic;
  s.request = p->request;
  s.lastBootId = p->lastBootId;
  s.check = p->check;
  return s;
}

void rtc_save(const dsentry::ModeStore& s) {
  volatile dsentry::ModeStore* p = &g_rtcStore;
  p->magic = s.magic;
  p->request = s.request;
  p->lastBootId = s.lastBootId;
  p->check = s.check;
}

/* ---- state shared between the AT task and the WiFi task -------------------------------- */

volatile uint8_t g_dsMode = DS_ESP_MODE_NONE;   /* radio a D-Sentry command or request started */
volatile uint8_t g_bootId = 0;
volatile bool g_wifiStarted = false;
volatile uint8_t g_wifiPhase = dsentry::WifiSupervisor::IDLE;   /* written by the WiFi task */
volatile uint8_t g_turretState = DS_STATE_OFF;
TaskHandle_t g_wifiTask = nullptr;

portMUX_TYPE g_mux = portMUX_INITIALIZER_UNLOCKED;
struct Shared {
  ds_control_packet_t ctrl;     /* latest valid ControlPacket (latest wins) */
  bool haveCtrl;
  uint32_t rxMs;                /* millis() when it arrived */
  uint32_t relayAddr;           /* its source, network byte order: where replies go */
  uint16_t relayPort;           /* network byte order */
  bool haveRelay;
  uint8_t status[DS_STATUS_FRAME_LEN];
  bool statusPending;
  uint8_t telem[DS_TELEMETRY_FRAME_LEN];
  bool telemPending;
};
Shared g_sh;   /* zero-initialised */

bool wifi_network_up() {
  const uint8_t ph = g_wifiPhase;
  return ph == dsentry::WifiSupervisor::STA_UP || ph == dsentry::WifiSupervisor::AP_UP;
}

/* ---- WiFi task -------------------------------------------------------------------------- */

void apply(dsentry::WifiSupervisor::Action action) {
  switch (action) {
    case dsentry::WifiSupervisor::STA_BEGIN:
      WiFi.mode(WIFI_STA);
      WiFi.setSleep(false);   /* no modem sleep: received UDP is not held until the next DTIM */
      WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
      xbox_logf_tag(kLogTag, "wifi: joining %s", WIFI_SSID);
      break;
    case dsentry::WifiSupervisor::STA_REBEGIN:
      WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
      xbox_logf_tag(kLogTag, "wifi: joining %s again", WIFI_SSID);
      break;
    case dsentry::WifiSupervisor::AP_START: {
      WiFi.disconnect();      /* stop a station join still in progress (no-op if none) */
      WiFi.mode(WIFI_AP);
      IPAddress ip;
      IPAddress mask;
      ip.fromString(WIFI_AP_IP);
      mask.fromString(WIFI_AP_NETMASK);
      WiFi.softAPConfig(ip, ip, mask);
      bool ok = WiFi.softAP(WIFI_AP_SSID, WIFI_AP_PASSWORD, WIFI_AP_CHANNEL);
      xbox_logf_tag(kLogTag, "wifi: own access point %s at %s %s", WIFI_AP_SSID,
                    WiFi.softAPIP().toString().c_str(), ok ? "up" : "FAILED");
      break;
    }
    case dsentry::WifiSupervisor::NONE:
    default:
      break;
  }
}

int open_socket() {
  int s = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
  if (s < 0) return -1;
  int on = 1;
  setsockopt(s, SOL_SOCKET, SO_REUSEADDR, &on, sizeof on);
  setsockopt(s, SOL_SOCKET, SO_BROADCAST, &on, sizeof on);
  struct timeval tv;
  tv.tv_sec = 0;
  tv.tv_usec = (suseconds_t)(WIFI_TASK_POLL_MS * US_PER_MS);
  setsockopt(s, SOL_SOCKET, SO_RCVTIMEO, &tv, sizeof tv);
  struct sockaddr_in local;
  memset(&local, 0, sizeof local);
  local.sin_family = AF_INET;
  local.sin_port = htons(WIFI_UDP_PORT);
  local.sin_addr.s_addr = htonl(INADDR_ANY);
  if (bind(s, (struct sockaddr*)&local, sizeof local) != 0) {
    close(s);
    return -1;
  }
  return s;
}

void send_to(int sock, uint32_t addr, uint16_t port, const uint8_t* data, size_t len) {
  struct sockaddr_in to;
  memset(&to, 0, sizeof to);
  to.sin_family = AF_INET;
  to.sin_port = port;
  to.sin_addr.s_addr = addr;
  sendto(sock, data, len, 0, (struct sockaddr*)&to, sizeof to);   /* best effort, like UDP */
}

/* One datagram per call; blocks at most WIFI_TASK_POLL_MS (SO_RCVTIMEO). */
void receive_one(int sock) {
  uint8_t buf[DS_MAX_FRAME_LEN];   /* longer datagrams are truncated, then fail the length check */
  struct sockaddr_in from;
  socklen_t fromLen = sizeof from;
  int n = recvfrom(sock, buf, sizeof buf, 0, (struct sockaddr*)&from, &fromLen);
  if (n <= 0) return;
  ds_control_packet_t p;
  if (!dsentry::accept_control(buf, (size_t)n, &p)) return;
  const uint32_t now = millis();
  bool newRelay;
  taskENTER_CRITICAL(&g_mux);
  newRelay = !g_sh.haveRelay || g_sh.relayAddr != from.sin_addr.s_addr || g_sh.relayPort != from.sin_port;
  g_sh.ctrl = p;
  g_sh.haveCtrl = true;
  g_sh.rxMs = now;
  g_sh.relayAddr = from.sin_addr.s_addr;
  g_sh.relayPort = from.sin_port;
  g_sh.haveRelay = true;
  taskEXIT_CRITICAL(&g_mux);
  if (newRelay) {
    xbox_logf_tag(kLogTag, "wifi: relay is %s:%u", IPAddress(from.sin_addr.s_addr).toString().c_str(),
                  (unsigned)ntohs(from.sin_port));
  }
}

void send_outbox(int sock) {
  uint8_t status[DS_STATUS_FRAME_LEN];
  uint8_t telem[DS_TELEMETRY_FRAME_LEN];
  bool haveStatus = false;
  bool haveTelem = false;
  bool haveRelay;
  uint32_t addr;
  uint16_t port;
  taskENTER_CRITICAL(&g_mux);
  haveRelay = g_sh.haveRelay;
  addr = g_sh.relayAddr;
  port = g_sh.relayPort;
  if (g_sh.statusPending) {
    memcpy(status, g_sh.status, sizeof status);
    g_sh.statusPending = false;
    haveStatus = true;
  }
  if (g_sh.telemPending) {
    memcpy(telem, g_sh.telem, sizeof telem);
    g_sh.telemPending = false;
    haveTelem = true;
  }
  taskEXIT_CRITICAL(&g_mux);
  if (!haveRelay) return;   /* nobody to tell yet: a StatusPacket is dropped (rumble is only now) */
  if (haveStatus) send_to(sock, addr, port, status, sizeof status);
  if (haveTelem) send_to(sock, addr, port, telem, sizeof telem);
}

uint8_t wifi_status_now() {
  bool have;
  uint32_t rx;
  taskENTER_CRITICAL(&g_mux);
  have = g_sh.haveCtrl;
  rx = g_sh.rxMs;
  taskEXIT_CRITICAL(&g_mux);
  const bool heard = dsentry::relay_heard(have, rx, millis());
  return dsentry::wifi_status(g_wifiStarted, (dsentry::WifiSupervisor::Phase)g_wifiPhase, heard);
}

void send_beacon(int sock, bool ap) {
  IPAddress ip = ap ? WiFi.softAPIP() : WiFi.localIP();
  IPAddress bcast = ap ? WiFi.softAPBroadcastIP() : WiFi.broadcastIP();
  if ((uint32_t)ip == 0 || (uint32_t)bcast == 0) return;   /* no address yet */
  uint8_t octets[DS_BEACON_IP_LEN];
  for (size_t i = 0; i < DS_BEACON_IP_LEN; i++) octets[i] = ip[(int)i];
  uint8_t frame[DS_BEACON_FRAME_LEN];
  size_t n = dsentry::build_beacon_frame(octets, g_bootId, wifi_status_now(), frame);
  send_to(sock, (uint32_t)bcast, htons(WIFI_DISCOVERY_PORT), frame, n);
}

void wifi_task(void*) {
  WiFi.persistent(false);         /* never write our credentials over the stock ones in NVS */
  WiFi.setAutoReconnect(false);   /* the supervisor below is the only one that reconnects */
  dsentry::WifiSupervisor sup;
  apply(sup.start(millis(), WIFI_SSID[0] != '\0', WIFI_MODE_AP_FALLBACK));
  g_wifiPhase = sup.phase();
  int sock = -1;
  bool beaconSent = false;
  uint32_t lastBeacon = 0;
  for (;;) {
    const uint32_t now = millis();
    const dsentry::WifiSupervisor::Phase before = sup.phase();
    apply(sup.step(now, WiFi.status() == WL_CONNECTED));
    g_wifiPhase = sup.phase();
    if (sup.phase() != before && sup.phase() == dsentry::WifiSupervisor::STA_UP) {
      xbox_logf_tag(kLogTag, "wifi: station up at %s", WiFi.localIP().toString().c_str());
    } else if (sup.phase() != before && sup.phase() == dsentry::WifiSupervisor::STA_LOST) {
      xbox_logf_tag(kLogTag, "wifi: station dropped");
    }
    if (sock < 0 && sup.network_up()) sock = open_socket();
    if (sock < 0) {
      vTaskDelay(pdMS_TO_TICKS(WIFI_TASK_POLL_MS));
      continue;
    }
    receive_one(sock);
    send_outbox(sock);
    if (sup.network_up() && (!beaconSent || (uint32_t)(now - lastBeacon) >= (uint32_t)WIFI_BEACON_MS)) {
      send_beacon(sock, sup.phase() == dsentry::WifiSupervisor::AP_UP);
      beaconSent = true;
      lastBeacon = now;
    }
  }
}

/* ---- the backend the AT handlers use ---------------------------------------------------- */

class RadioBackend final : public DsentryBackend {
 public:
  dsentry::Radios radios() override {
    dsentry::Radios r;
    r.bleRunning = xbox_ble_running();
    r.hciUsed = xbox_hci_used();
    r.dsWifiRunning = g_wifiStarted;
    r.otherWifiRunning = !g_wifiStarted && WiFi.getMode() != WIFI_MODE_NULL;
    r.restartPending = false;
    return r;
  }

  uint8_t bootId() override { return g_bootId; }

  uint8_t wifiStatus() override { return wifi_status_now(); }

  bool dsModeActive() override { return g_dsMode != DS_ESP_MODE_NONE; }

  bool startBle() override {
    if (BLE_FORGET_PAIRINGS_ON_BOOT) xbox_forget();   /* before NimBLE starts: erases NVS bonds */
    int rc = xbox_begin();
    xbox_logf_tag(kLogTag, "BLE mode: xbox_begin() = %d", rc);
    if (rc != XBOX_R4_RC_OK) return false;
    g_dsMode = DS_ESP_MODE_BLE;
    return true;
  }

  bool startWifi() override {
    if (g_wifiStarted) return true;
    xbox_route_idf_log_to_usb();
    g_wifiPhase = dsentry::WifiSupervisor::IDLE;
    g_wifiStarted = true;   /* before the task runs, so xbox_begin()/+HCIBEGIN refuse at once */
    if (xTaskCreatePinnedToCore(wifi_task, "dswifi", ESP_WIFI_TASK_STACK_BYTES, nullptr,
                                ESP_WIFI_TASK_PRIORITY, &g_wifiTask, ESP_WIFI_TASK_CORE) != pdPASS) {
      g_wifiTask = nullptr;
      g_wifiStarted = false;
      xbox_logf_tag(kLogTag, "WiFi mode: task create failed");
      return false;
    }
    g_dsMode = DS_ESP_MODE_WIFI;
    xbox_logf_tag(kLogTag, "WiFi mode started");
    return true;
  }

  void adoptMode(uint8_t espMode) override {
    if (g_dsMode == DS_ESP_MODE_NONE) g_dsMode = espMode;
  }

  void storeRestartRequest(uint8_t espMode) override {
    dsentry::ModeStore s;
    dsentry::store_write(&s, espMode, g_bootId);
    rtc_save(s);
    xbox_logf_tag(kLogTag, "restart into %s requested", ds_esp_mode_name(espMode));
  }

  void rumbleBle(const uint8_t report[XBOX_RUMBLE_REPORT_LEN]) override {
    xbox_request_rumble_report(report);
  }

  void statusToRelay(const uint8_t* frame, size_t len) override {
    if (len != DS_STATUS_FRAME_LEN) return;
    taskENTER_CRITICAL(&g_mux);
    memcpy(g_sh.status, frame, DS_STATUS_FRAME_LEN);
    g_sh.statusPending = true;
    taskEXIT_CRITICAL(&g_mux);
  }

  bool telemetryToRelay(const uint8_t* frame, size_t len) override {
    if (!g_wifiStarted || !wifi_network_up() || len != DS_TELEMETRY_FRAME_LEN) return false;
    bool ok;
    taskENTER_CRITICAL(&g_mux);
    ok = g_sh.haveRelay;
    if (ok) {
      memcpy(g_sh.telem, frame, DS_TELEMETRY_FRAME_LEN);
      g_sh.telemPending = true;
    }
    taskEXIT_CRITICAL(&g_mux);
    return ok;
  }

  void noteTurretState(uint8_t state) override {
    if (state == g_turretState) return;
    g_turretState = state;
    xbox_logf_tag(kLogTag, "turret state %s", ds_turret_state_name(state));
  }
};

RadioBackend g_backend;
DsentryAt g_at(g_backend);

}  /* namespace */

DsentryAt& dsentry_at() { return g_at; }

extern "C" {

void dsentry_boot(void) {
  dsentry::ModeStore s = rtc_load();
  dsentry::BootPlan plan = dsentry::plan_boot(&s, esp_reset_reason() == ESP_RST_SW, esp_random());
  rtc_save(s);
  g_bootId = plan.bootId;
  if (plan.startMode == DS_ESP_MODE_BLE) {
    g_backend.startBle();
  } else if (plan.startMode == DS_ESP_MODE_WIFI) {
    g_backend.startWifi();
  }
}

bool dsentry_radio_mode_active(void) { return g_dsMode != DS_ESP_MODE_NONE; }

bool dsentry_wifi_mode_active(void) { return g_wifiStarted; }

bool dsentry_fill_xbox_state(uint8_t out[XBOX_R4_STATE_LEN]) {
  if (!g_wifiStarted) return false;
  ds_control_packet_t p;
  bool have;
  uint32_t rx;
  taskENTER_CRITICAL(&g_mux);
  p = g_sh.ctrl;
  have = g_sh.haveCtrl;
  rx = g_sh.rxMs;
  taskEXIT_CRITICAL(&g_mux);
  const bool live = dsentry::control_is_live(have, p, rx, millis());   /* millis() after the copy */
  dsentry::control_to_xstate(have ? &p : nullptr, live, out);
  return true;
}

void dsentry_restart_now(void) {
  vTaskDelay(pdMS_TO_TICKS(ESP_RESTART_DELAY_MS));
  esp_restart();
}

}  /* extern "C" */

/*
 * xbox_client.cpp -- the one translation unit that includes NimBLE-Arduino and
 * XboxSeriesXControllerESP32_asukiaaa (both define per-file static objects).
 *
 * What it does:
 *  - AT+XBOXBEGIN starts NimBLE and a small FreeRTOS task that runs the library's onLoop()
 *    (which blocks for seconds while connecting, so it must not run in loop() or in the AT task).
 *  - AT+XBOXSTATE? snapshots the parsed controller state into the 20-byte protocol payload.
 *  - The scanner only accepts a controller in pairing mode until one has connected. That
 *    controller is then remembered (its BLE address, in NVS through Preferences) and from then
 *    on only it, or another controller in pairing mode, is accepted. AT+XBOXFORGET clears it.
 *  - Mutual exclusion with the stock AT+HCIBEGIN bridge (same BLE controller, one VHCI host).
 *
 * D-Sentry additions (marked "D-Sentry:", DECISIONS.md D19): rumble through the pad's 0x2A4D
 * output report, written in the client task right after onLoop(); AT+XBOXSTATE? served from the
 * PC relay's packets while D-Sentry WiFi mode runs (dsentry_radio.cpp); no NimBLE start while
 * that WiFi mode runs.
 *
 * Threading rule: everything that touches the library's target address, bonds or clients runs
 * in the client task. AT handlers (which run in the AT task on core 0) only read snapshots and
 * set request flags. The review in DECISIONS.md D14 explains why.
 *
 * SPDX-License-Identifier: MIT
 * Copyright (c) 2026 the UNO R4 WiFi Xbox controller project
 * The combined firmware is governed by the AGPL-3.0 (see DECISIONS.md D04).
 */
#include "xbox_client.h"

#include <Arduino.h>
#include <Preferences.h>
#include <esp_log.h>
#include <stdarg.h>
#include <string.h>

#include "USB.h"
#include "USBCDC.h"
#include <XboxSeriesXControllerESP32_asukiaaa.hpp>

#include "dsentry_radio.h"   /* D-Sentry: WiFi-mode state source and radio exclusion */

extern USBCDC USBSerial;  /* defined in UNOR4USBBridge.ino */

namespace {

using XboxSeriesXControllerESP32_asukiaaa::Core;
using XboxSeriesXControllerESP32_asukiaaa::ConnectionState;

Core* g_core = nullptr;              /* created on the first AT+XBOXBEGIN */
TaskHandle_t g_task = nullptr;
volatile bool g_active = false;      /* NimBLE initialised by us (sticky until reset) */
volatile bool g_hci_used = false;    /* AT+HCIBEGIN succeeded at least once (sticky until reset) */
volatile bool g_log = false;

/* Requests from the AT task to the client task. */
volatile bool g_forget_req = false;      /* AT+XBOXFORGET: do the bond/target work in the client task */
volatile bool g_block_remember = false;  /* set by FORGET until the link has actually dropped */

ble_addr_t g_saved_peer;             /* type + 6 address bytes, LSB first (NimBLE order) */
bool g_have_saved_peer = false;
bool g_saved_peer_loaded = false;    /* load_saved_peer() has run at least once */

volatile int g_battery_cached = -1;  /* battery level read during discovery, before any notification */
bool g_battery_tried = false;        /* one cache lookup per link */
bool g_scanning_seen = false;        /* Scanning observed on the previous iteration too */

/* ---- AT+XBOXSCAN? diagnostic: the last adverts the scanner saw (filled in the NimBLE host task) ---- */
struct AdvertSeen {
  uint8_t  addr[6];
  uint8_t  addrType;
  uint16_t appearance;
  int8_t   rssi;
  uint8_t  mfgLen;
  uint8_t  mfg[8];
  char     svc[40];
  char     name[16];
  uint16_t count;
  bool     used;
};
const int kAdvertRing = 8;
AdvertSeen g_adverts[kAdvertRing];
int g_advertNext = 0;
portMUX_TYPE g_advertMux = portMUX_INITIALIZER_UNLOCKED;

void advert_hook(const NimBLEAdvertisedDevice* d) {
  if (d == nullptr) return;
  const ble_addr_t* base = d->getAddress().getBase();
  AdvertSeen e;
  memset(&e, 0, sizeof(e));
  memcpy(e.addr, base->val, 6);
  e.addrType = base->type;
  e.appearance = d->haveAppearance() ? d->getAppearance() : 0;
  e.rssi = (int8_t)d->getRSSI();
  std::string m = d->getManufacturerData();
  e.mfgLen = m.size() > sizeof(e.mfg) ? sizeof(e.mfg) : m.size();
  memcpy(e.mfg, m.data(), e.mfgLen);
  if (d->haveServiceUUID()) strncpy(e.svc, d->getServiceUUID().toString().c_str(), sizeof(e.svc) - 1);
  if (d->haveName()) strncpy(e.name, d->getName().c_str(), sizeof(e.name) - 1);
  e.count = 1;
  e.used = true;
  taskENTER_CRITICAL(&g_advertMux);
  int slot = -1;
  for (int i = 0; i < kAdvertRing; i++) {
    if (g_adverts[i].used && memcmp(g_adverts[i].addr, e.addr, 6) == 0) { slot = i; break; }
  }
  if (slot >= 0) {
    e.count = g_adverts[slot].count + 1;
  } else {
    slot = g_advertNext;
    g_advertNext = (g_advertNext + 1) % kAdvertRing;
  }
  g_adverts[slot] = e;
  taskEXIT_CRITICAL(&g_advertMux);
}

/* ---- D-Sentry: rumble hand-off (AT task -> client task) ------------------------------------
 * The AT task stores one 8-byte output report and bumps a generation counter under the mux
 * (latest request wins). The client task, right after onLoop(), takes the newest report and
 * writes it without response to the HID service's 0x2A4D characteristic that accepts writes,
 * found once per link from NimBLE's cache only (no GATT traffic, no blocking). The cached
 * pointer is dropped whenever the link is not in Connected state and before
 * free_disconnected_clients() deletes clients, so it can never dangle. */
const uint16_t kUuidServiceHid = 0x1812;   /* BLE HID service */
const uint16_t kUuidHidReport = 0x2A4D;    /* BLE HID Report characteristic */
uint8_t g_rumble_report[XBOX_RUMBLE_REPORT_LEN];
uint32_t g_rumble_gen = 0;                 /* under g_rumbleMux */
uint32_t g_rumble_done = 0;                /* client task, read and written under g_rumbleMux */
portMUX_TYPE g_rumbleMux = portMUX_INITIALIZER_UNLOCKED;
NimBLERemoteCharacteristic* g_out_report = nullptr;   /* client task only */
NimBLEClient* g_out_client = nullptr;                 /* the client g_out_report belongs to */
bool g_out_looked = false;                            /* lookup done for g_out_client */

const char* const kPrefsNamespace = "xboxr4";
const char* const kPeerKey = "peer";
const char* const kNimbleBondNamespace = "nimble_bond";   /* NimBLE's own bond store (ble_store_nvs.c) */

/* ---- output ---------------------------------------------------------------------------- */

/* Non-blocking write to the USB CDC port: drop the line rather than stall a task. */
void usb_write_nonblocking(const uint8_t* buf, size_t len) {
  int room = USBSerial.availableForWrite();
  if (room <= 0) return;
  if ((size_t)room < len) len = room;
  USBSerial.write(buf, len);
}

/* Diagnostics to the USB CDC port (the Serial Monitor), only when AT+XBOXLOG=1.
 * D-Sentry: the body moved into vlogf_tag() so dsentry_radio.cpp can log as "[dsentry]". */
void vlogf_tag(const char* tag, const char* fmt, va_list ap) {
  if (!g_log) return;
  char buf[176];
  int n = snprintf(buf, sizeof(buf), "[%s] ", tag);
  if (n < 0 || n >= (int)sizeof(buf) - 3) return;
  int m = vsnprintf(buf + n, sizeof(buf) - n - 2, fmt, ap);
  if (m < 0) return;
  n += m;
  if (n > (int)sizeof(buf) - 3) n = sizeof(buf) - 3;
  buf[n++] = '\r';
  buf[n++] = '\n';
  usb_write_nonblocking((const uint8_t*)buf, n);
}

void logf(const char* fmt, ...) __attribute__((format(printf, 1, 2)));
void logf(const char* fmt, ...) {
  va_list ap;
  va_start(ap, fmt);
  vlogf_tag("xbox", fmt, ap);
  va_end(ap);
}

/* esp_log sink, installed by AT+XBOXBEGIN: ESP-IDF logs from any task go to USB, never to
 * UART0 (the RA4M1's Serial RX and upload path). Stock mode keeps the stock console. */
int idf_log_to_usb(const char* fmt, va_list ap) {
  char buf[256];
  int n = vsnprintf(buf, sizeof(buf), fmt, ap);
  if (n > 0) {
    if (n > (int)sizeof(buf) - 1) n = sizeof(buf) - 1;
    usb_write_nonblocking((const uint8_t*)buf, n);
  }
  return n;
}

void format_address(const uint8_t val[6], char out[18]) {
  /* NimBLE stores the address least-significant byte first; print it the usual way round. */
  snprintf(out, 18, "%02x:%02x:%02x:%02x:%02x:%02x",
           val[5], val[4], val[3], val[2], val[1], val[0]);
}

/* ---- remembered controller (NVS) ------------------------------------------------------- */

void load_saved_peer() {
  Preferences prefs;
  g_have_saved_peer = false;
  /* read-only: a missing namespace then fails begin() (= no saved peer) instead of being created */
  if (prefs.begin(kPrefsNamespace, true)) {
    g_have_saved_peer = prefs.isKey(kPeerKey) &&
                        prefs.getBytesLength(kPeerKey) == sizeof(ble_addr_t) &&
                        prefs.getBytes(kPeerKey, &g_saved_peer, sizeof(g_saved_peer)) == sizeof(g_saved_peer);
    prefs.end();
  }
  g_saved_peer_loaded = true;
}

void save_peer(const ble_addr_t& addr) {
  Preferences prefs;
  if (!prefs.begin(kPrefsNamespace, false)) return;
  prefs.putBytes(kPeerKey, &addr, sizeof(addr));
  prefs.end();
}

void clear_saved_peer() {
  Preferences prefs;
  if (prefs.begin(kPrefsNamespace, false)) {
    prefs.remove(kPeerKey);
    prefs.end();
  }
  g_have_saved_peer = false;
}

/* ---- scanner policy (client task only) ------------------------------------------------- */

/* "Pairing mode only": a target address no real device can have, plus the library's
 * pairing-mode branch (which, with fork patch 4, also checks appearance and the HID UUID).
 * Without a target the library would also accept any controller advertising its normal
 * reconnect data, including a neighbour's controller looking for its own console. */
void accept_pairing_mode_only() {
  ble_addr_t none;
  memset(&none, 0, sizeof(none));
  none.type = 0xFF;
  g_core->setTargetAddress(none);
  g_core->setIgnoreTargetAddressToConnectToUnkownSearchingDevice(true);
}

void accept_saved_peer_or_pairing_mode() {
  g_core->setTargetAddress(g_saved_peer);
  g_core->setIgnoreTargetAddressToConnectToUnkownSearchingDevice(true);
}

/* D-Sentry: forget the cached output characteristic (client task only). */
void reset_out_report_cache() {
  g_out_report = nullptr;
  g_out_client = nullptr;
  g_out_looked = false;
}

/* The library never frees NimBLE client objects and there are only three slots; after three
 * different peers a new controller could never connect. Called in the client task only after
 * Scanning has been observed on two consecutive iterations (so the host task has long finished
 * the disconnect event), with the scan stopped (the scan handler walks the client list). */
void free_disconnected_clients() {
  reset_out_report_cache();          /* D-Sentry: the clients (and their characteristics) go away */
  NimBLEDevice::getScan()->stop();   /* onLoop() restarts it on the next pass */
  while (NimBLEDevice::getCreatedClientCount() > 0) {
    NimBLEClient* c = NimBLEDevice::getDisconnectedClient();
    if (c == nullptr) break;
    /* The library can leave this pointing at a client whose link dropped mid-discovery. */
    if (c == XboxSeriesXControllerESP32_asukiaaa::pConnectedClient) {
      XboxSeriesXControllerESP32_asukiaaa::pConnectedClient = nullptr;
    }
    /* The library hands the one shared callbacks object to every client with deleteCallbacks
     * = true; detach it first or deleting the client would free it under the others. */
    c->setClientCallbacks(g_core->clientCBs, false);
    if (!NimBLEDevice::deleteClient(c)) break;
  }
}

/* While connected: remember a controller we have not seen before (client task only). */
void remember_peer_if_new() {
  if (g_block_remember) return;
  NimBLEClient* client = XboxSeriesXControllerESP32_asukiaaa::pConnectedClient;
  if (client == nullptr) return;
  NimBLEAddress peer = client->getPeerAddress();
  const ble_addr_t* base = peer.getBase();
  if (base == nullptr) return;
  if (g_have_saved_peer && memcmp(base, &g_saved_peer, sizeof(ble_addr_t)) == 0) return;
  g_saved_peer = *base;
  g_have_saved_peer = true;
  save_peer(g_saved_peer);
  accept_saved_peer_or_pairing_mode();
  logf("remembered controller %s", peer.toString().c_str());
}

/* The battery level was read during discovery; use it until the first battery notification.
 * Cache only: getServices(false)/getCharacteristics(false) never touch the radio, so a peer
 * without a battery service costs nothing. One attempt per link. */
void read_cached_battery() {
  if (g_battery_cached >= 0 || g_battery_tried) return;
  g_battery_tried = true;
  NimBLEClient* client = XboxSeriesXControllerESP32_asukiaaa::pConnectedClient;
  if (client == nullptr) return;
  const NimBLEUUID uuidBattery((uint16_t)0x180F);
  const NimBLEUUID uuidLevel((uint16_t)0x2A19);
  for (NimBLERemoteService* svc : client->getServices(false)) {
    if (svc == nullptr || !svc->getUUID().equals(uuidBattery)) continue;
    for (NimBLERemoteCharacteristic* chr : svc->getCharacteristics(false)) {
      if (chr == nullptr || !chr->getUUID().equals(uuidLevel)) continue;
      NimBLEAttValue v = chr->getValue();
      if (v.size() >= 1) g_battery_cached = v.data()[0];
      return;
    }
  }
}

/* AT+XBOXFORGET, executed here in the client task so nothing races the scanner. The AT task
 * already cleared NVS so AT+XBOXADDR? answers zeros at once; this task has the last word. */
void do_forget() {
  NimBLEDevice::getScan()->stop();
  XboxSeriesXControllerESP32_asukiaaa::advDevice = nullptr;   /* drop a match made under the old policy */
  g_block_remember = true;
  clear_saved_peer();
  accept_pairing_mode_only();
  bool ok = NimBLEDevice::deleteAllBonds();   /* also terminates a live link to a bonded peer */
  for (NimBLEClient* c : NimBLEDevice::getConnectedClients()) {
    if (c != nullptr) c->disconnect();
  }
  g_battery_cached = -1;
  g_battery_tried = false;
  logf("forgot bonds (%s) and the remembered controller; pairing mode needed now", ok ? "ok" : "some remained");
}

/* D-Sentry: the pad's rumble output characteristic, from NimBLE's cache only. Prefers the
 * first 0x2A4D that canWrite() (what the library's own writeHIDReport() writes to, proven on
 * the pad), else one that canWriteNoResponse(). */
NimBLERemoteCharacteristic* find_out_report(NimBLEClient* client) {
  const NimBLEUUID uuidHid(kUuidServiceHid);
  const NimBLEUUID uuidReport(kUuidHidReport);
  NimBLERemoteCharacteristic* noRsp = nullptr;
  for (NimBLERemoteService* svc : client->getServices(false)) {
    if (svc == nullptr || !svc->getUUID().equals(uuidHid)) continue;
    for (NimBLERemoteCharacteristic* chr : svc->getCharacteristics(false)) {
      if (chr == nullptr || !chr->getUUID().equals(uuidReport)) continue;
      if (chr->canWrite()) return chr;
      if (noRsp == nullptr && chr->canWriteNoResponse()) noRsp = chr;
    }
  }
  return noRsp;
}

/* D-Sentry: write the newest rumble request, if any (client task, right after onLoop()). A
 * request that finds no connected pad is dropped: a late rumble means nothing. */
void service_rumble() {
  uint8_t report[XBOX_RUMBLE_REPORT_LEN];
  bool fresh;
  taskENTER_CRITICAL(&g_rumbleMux);
  fresh = g_rumble_gen != g_rumble_done;
  if (fresh) {
    memcpy(report, g_rumble_report, sizeof(report));
    g_rumble_done = g_rumble_gen;   /* taken, whether or not a pad is there to receive it */
  }
  taskEXIT_CRITICAL(&g_rumbleMux);
  if (!fresh) return;
  if (g_core->connectionStateValue() != (uint8_t)ConnectionState::Connected) return;
  NimBLEClient* client = XboxSeriesXControllerESP32_asukiaaa::pConnectedClient;
  if (client == nullptr || !client->isConnected()) return;
  if (client != g_out_client) {
    reset_out_report_cache();
    g_out_client = client;
  }
  if (!g_out_looked) {
    g_out_report = find_out_report(client);
    g_out_looked = true;
    logf("rumble output report %s", g_out_report != nullptr ? "found" : "NOT found");
  }
  if (g_out_report == nullptr) return;
  g_out_report->writeValue(report, sizeof(report), false);   /* write without response: no wait */
}

const char* state_name(uint8_t st) {
  switch (st) {
    case XBOX_R4_CONNECTED:            return "connected";
    case XBOX_R4_WAITING_FIRST_REPORT: return "waiting-first-report";
    case XBOX_R4_FOUND:                return "found";
    case XBOX_R4_SCANNING:             return "scanning";
    default:                           return "?";
  }
}

void xbox_task(void*) {
  uint8_t prev = 0xFF;
  for (;;) {
    if (g_forget_req) {
      g_forget_req = false;
      do_forget();
    }

    g_core->onLoop();   /* may block for seconds while connecting; that is why it has its own task */
    service_rumble();   /* D-Sentry: at most one 8-byte write per iteration while connected */

    uint8_t st = g_core->connectionStateValue();
    if (st != prev) {
      logf("state: %s (failed connects so far: %u)", state_name(st), g_core->getCountFailedConnection());
      prev = st;
    }
    if (st == (uint8_t)ConnectionState::Connected) {
      remember_peer_if_new();
      read_cached_battery();
    }
    if (st != (uint8_t)ConnectionState::Connected) {
      reset_out_report_cache();     /* D-Sentry: a new link re-discovers its characteristics */
    }
    if (!g_core->isConnected()) {   /* Found or Scanning: the link is down */
      g_block_remember = false;     /* a FORGET has taken effect (do_forget re-asserts it if pending) */
      g_battery_cached = -1;
      g_battery_tried = false;
    }
    if (st == (uint8_t)ConnectionState::Scanning) {
      if (g_scanning_seen && NimBLEDevice::getCreatedClientCount() >= 2) free_disconnected_clients();
      g_scanning_seen = true;
    } else {
      g_scanning_seen = false;
    }
    vTaskDelay(pdMS_TO_TICKS(10));   /* always yield: core 1 is shared with the USB passthrough loop */
  }
}

}  /* namespace */

extern "C" {

int xbox_begin(void) {
  if (g_active && g_task != nullptr) return XBOX_R4_RC_OK;
  if (g_hci_used) return XBOX_R4_RC_BLOCKED_BY_HCI;
  /* D-Sentry: BLE and WiFi never run together. While D-Sentry WiFi mode owns the radio this
   * is refused like the HCI case, and the remedy is the same: AT+RESET boots radio-less. */
  if (dsentry_wifi_mode_active()) return XBOX_R4_RC_BLOCKED_BY_HCI;

  if (!g_active) {
    /* From here on, logs from any task go to USB instead of the RA4M1's UART. */
    esp_log_set_vprintf(&idf_log_to_usb);

    if (g_core == nullptr) {
      g_core = new Core();
      memset(g_core->deviceAddressArr, 0, sizeof(g_core->deviceAddressArr));
    }
    g_core->deleteBondOnFailedConnection = false;
    g_core->scanCBs->onAdvertHook = &advert_hook;   /* for AT+XBOXSCAN? */

    load_saved_peer();
    if (g_have_saved_peer) {
      accept_saved_peer_or_pairing_mode();
      char addr[18];
      format_address(g_saved_peer.val, addr);
      logf("looking for remembered controller %s (or any controller in pairing mode)", addr);
    } else {
      accept_pairing_mode_only();
      logf("no remembered controller: hold the controller's Pair button until the logo flashes fast");
    }

    /* Init here first: Core::begin() ignores init()'s result and would then call into an
     * uninitialised host and assert. A second init() inside begin() is a no-op. */
    if (!NimBLEDevice::init("")) {
      logf("NimBLE init failed");
      return XBOX_R4_RC_INIT_FAILED;
    }
    g_core->begin();   /* public own address, bonding on, MITM off, SC off, +power */
    g_active = true;
  }

  BaseType_t ok = xTaskCreatePinnedToCore(xbox_task, "xbox", 8192, nullptr, 1, &g_task, 1);
  if (ok != pdPASS) {
    g_task = nullptr;   /* g_active stays true: NimBLE is up, so the HCI bridge stays excluded */
    logf("task create failed");
    return XBOX_R4_RC_INIT_FAILED;
  }
  logf("started");
  return XBOX_R4_RC_OK;
}

void xbox_fill_state(uint8_t out[XBOX_R4_STATE_LEN]) {
  /* D-Sentry: in D-Sentry WiFi mode the state comes from the PC relay's ControlPackets. */
  if (dsentry_fill_xbox_state(out)) return;
  xbox_r4_state_t s;
  memset(&s, 0, sizeof(s));
  if (!g_active || g_core == nullptr) {
    xbox_r4_neutral_report(s.report);
    s.state = g_hci_used ? XBOX_R4_BLOCKED_BY_HCI : XBOX_R4_OFF;
    s.battery = XBOX_R4_BATTERY_UNKNOWN;
    s.seq = 0;
  } else {
    /* ConnectionState numbering (0..3) is the protocol's numbering. */
    s.state = g_core->connectionStateValue();
    /* Consistent snapshot: the NimBLE host task (same core, higher priority) may deliver a
     * report between any two reads here. Re-read until the count did not move. */
    uint32_t count;
    do {
      count = g_core->notificationCount;
      g_core->xboxNotif.toArr(s.report, XBOX_R4_REPORT_LEN);
    } while (count != g_core->notificationCount);
    s.seq = (uint8_t)count;
    /* No HID report yet on this link: send neutral bytes, not the previous session's last input. */
    if (s.state != XBOX_R4_CONNECTED || g_core->getReceiveNotificationAt() == 0) {
      xbox_r4_neutral_report(s.report);
    }
    if (g_core->getReceiveBatteryAt() != 0) {
      s.battery = g_core->battery;
    } else if (g_battery_cached >= 0 && s.state == XBOX_R4_CONNECTED) {
      s.battery = (uint8_t)g_battery_cached;
    } else {
      s.battery = XBOX_R4_BATTERY_UNKNOWN;
    }
  }
  xbox_r4_encode(&s, out);
}

void xbox_peer_address(char out[18]) {
  if (!g_saved_peer_loaded) load_saved_peer();
  if (g_active && g_core != nullptr && g_core->connectionStateValue() == XBOX_R4_CONNECTED) {
    format_address(g_core->deviceAddressArr, out);
  } else if (g_have_saved_peer) {
    format_address(g_saved_peer.val, out);
  } else {
    snprintf(out, 18, "00:00:00:00:00:00");
  }
}

int xbox_forget(void) {
  g_block_remember = true;     /* before clearing: the client task must not write it back */
  clear_saved_peer();
  if (!g_active || g_core == nullptr) {
    /* NimBLE is not running, so erase its bond store directly. Safe: NimBLE only reads the
     * store during init. */
    Preferences bonds;
    if (bonds.begin(kNimbleBondNamespace, false)) {
      bonds.clear();
      bonds.end();
    }
    g_block_remember = false;
    return XBOX_R4_RC_OK;
  }
  g_forget_req = true;         /* the client task does the rest between onLoop() calls */
  return XBOX_R4_RC_OK;
}

size_t xbox_scan_report(char* out, size_t cap) {
  AdvertSeen copy[kAdvertRing];
  taskENTER_CRITICAL(&g_advertMux);
  memcpy(copy, g_adverts, sizeof(copy));
  taskEXIT_CRITICAL(&g_advertMux);
  size_t n = 0;
  if (!g_active) {
    n = snprintf(out, cap, "off");
    return n < cap ? n : cap - 1;
  }
  bool any = false;
  for (int i = 0; i < kAdvertRing; i++) {
    const AdvertSeen& e = copy[i];
    if (!e.used) continue;
    any = true;
    char mfg[2 * 8 + 1] = "";
    for (int k = 0; k < e.mfgLen; k++) snprintf(mfg + 2 * k, 3, "%02x", e.mfg[k]);
    int w = snprintf(out + n, cap - n, "%02x:%02x:%02x:%02x:%02x:%02x/%u,%u,%d,%s,%s,%s,%u;",
                     e.addr[5], e.addr[4], e.addr[3], e.addr[2], e.addr[1], e.addr[0], e.addrType,
                     e.appearance, e.rssi, mfg, e.svc[0] ? e.svc : "-", e.name[0] ? e.name : "-", e.count);
    if (w < 0 || (size_t)w >= cap - n) { out[cap - 1] = 0; return cap - 1; }
    n += w;
  }
  if (!any) n = snprintf(out, cap, "none");
  return n < cap ? n : cap - 1;
}

void xbox_set_log(bool on) { g_log = on; }
bool xbox_log_enabled(void) { return g_log; }

/* D-Sentry: also true while D-Sentry WiFi mode runs, so +HCIBEGIN (its only caller) is refused. */
bool xbox_is_active(void) { return g_active || dsentry_wifi_mode_active(); }
bool xbox_ble_running(void) { return g_active; }   /* D-Sentry */
void xbox_note_hci_used(void) { g_hci_used = true; }
bool xbox_hci_used(void) { return g_hci_used; }

/* ---- D-Sentry additions ---- */

int xbox_request_rumble_report(const uint8_t report[XBOX_RUMBLE_REPORT_LEN]) {
  if (!g_active || g_task == nullptr) return XBOX_R4_RC_NOT_RUNNING;
  taskENTER_CRITICAL(&g_rumbleMux);
  memcpy(g_rumble_report, report, XBOX_RUMBLE_REPORT_LEN);
  g_rumble_gen++;
  taskEXIT_CRITICAL(&g_rumbleMux);
  return XBOX_R4_RC_OK;
}

void xbox_route_idf_log_to_usb(void) { esp_log_set_vprintf(&idf_log_to_usb); }

void xbox_logf_tag(const char* tag, const char* fmt, ...) {
  va_list ap;
  va_start(ap, fmt);
  vlogf_tag(tag, fmt, ap);
  va_end(ap);
}

}  /* extern "C" */

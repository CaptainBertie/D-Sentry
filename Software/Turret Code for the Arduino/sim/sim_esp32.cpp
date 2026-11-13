// sim_esp32.cpp -- the ESP32-S3 of dsentry_sim. See sim_esp32.h for what is real and what is a
// stand-in. Compiled with -I firmware/UNOR4USBBridge -I sim (never the RA4M1's src): the names
// it uses come from the fork's dsentry_config.h (generated from config.h by tools/sync_config.py)
// and dsentry_protocol.h, never from the RA4M1's config.h.
//
// Where each piece mirrors the firmware:
//   CAtHandler::run() (at_handler.cpp)        atTask(): at_srv.run(), then the queued tasks; every
//                                             kAtTickUs (vTaskDelay(1) at the 1 kHz FreeRTOS tick)
//   CAtHandler's command callback             unknown command -> ERROR
//   cmds_dsentry.h                            +DSCAPS / +DSS / +DST -> DsentryAt; the DSS restart
//                                             task = serial->flush() + dsentry_restart_now()
//   cmds_xbox.h +XBOXSTATE / +XBOXBEGIN       handler bodies copied (std::to_string for String())
//   cmds_esp_generic.h +SOFTRESETWIFI         the D-Sentry guard, then the stock reply
//   xbox_fill_state() + dsentry_fill_xbox_state()   fillXboxState()
//   xbox_begin() (xbox_client.cpp)            simXboxBegin(): refused while WiFi mode runs
//   dsentry_radio.cpp RadioBackend            SimBackend
//   dsentry_radio.cpp wifi_task()             wifiTask(): supervisor, socket, receive, outbox, beacon
//   dsentry_boot()                            boot(): plan_boot() on the RTC store, then the radio
// tests/acceptance/test_sim_drift.py fails when those firmware files change shape.
//
// SPDX-License-Identifier: MIT (the combined dsentry_sim binary: AGPL-3.0, see sim_esp32.h)
#include "sim_esp32.h"

#include <arpa/inet.h>
#include <errno.h>
#include <netinet/in.h>
#include <stdarg.h>
#include <stdio.h>
#include <string.h>
#include <sys/socket.h>
#include <unistd.h>

#include <functional>
#include <map>
#include <memory>
#include <queue>
#include <random>
#include <string>

#include "chAT.hpp"
#include "dsentry_at.h"
#include "dsentry_wire.h"
#include "sim_wire.h"
#include "xbox_r4_protocol.h"

using namespace SudoMaker;

namespace {

// ---- model constants (not firmware tunables: facts about the stand-in hardware) -------------
// CAtHandler::run() ends with vTaskDelay(1): one tick of the 1 kHz FreeRTOS clock.
constexpr uint64_t kAtTickUs = US_PER_MS;
// The ROM banner starts this long after a reset (QEMU, sim/qemu/evidence/fork-B/uart0-timed.log:
// "Rebooting..." at 55.805 s, "ESP-ROM:" at 55.821 s; 0.013 s after power-on).
constexpr uint64_t kRomBannerAfterUs = 16 * US_PER_MS;
// Line time of one byte on the AT UART (8N1 at LINK_BAUD), in ns.
constexpr uint64_t kAtByteNs = (uint64_t)UART_BITS_PER_BYTE * US_PER_S * 1000u / LINK_BAUD;
// The ESP32's UART RX buffer for the AT link: far more than one request ever needs.
constexpr size_t kAtRxBytes = 4096;
const uint8_t kLoopbackIp[DS_BEACON_IP_LEN] = {127, 0, 0, 1};

// The ESP32-S3 ROM banner as QEMU printed it on UART0 (same evidence file; power-on at 0.013 s,
// software reset at 55.821 s). Only the ROM part: the app's own early log lines are an emulator
// observation (research-esp32-fork.md), so they are left out.
const char kRomHead[] = "ESP-ROM:esp32s3-20210327\r\nBuild:Mar 27 2021\r\n";
const char kRomRstPowerOn[] = "rst:0x1 (POWERON),boot:0x4 (SPI_FLASH_BOOT)\r\n";
const char kRomRstSoftware[] = "rst:0xc (RTC_SW_CPU_RST),boot:0x4 (SPI_FLASH_BOOT)\r\n";
const char kRomTail[] =
    "SPIWP:0xee\r\nmode:DIO, clock div:1\r\nload:0x3fce3808,len:0x44c\r\n"
    "load:0x403c9700,len:0xbe4\r\nload:0x403cc700,len:0x2a38\r\nentry 0x403c98d4\r\n";

typedef std::function<chAT::CommandStatus(chAT::Server&, chAT::ATParser&)> handler_t;

const char* phaseName(dsentry::WifiSupervisor::Phase p) {
  switch (p) {
    case dsentry::WifiSupervisor::IDLE: return "IDLE";
    case dsentry::WifiSupervisor::JOINING: return "JOINING";
    case dsentry::WifiSupervisor::STA_UP: return "STA_UP";
    case dsentry::WifiSupervisor::STA_LOST: return "STA_LOST";
    case dsentry::WifiSupervisor::AP_UP: return "AP_UP";
    case dsentry::WifiSupervisor::NO_NETWORK: return "NO_NETWORK";
  }
  return "?";
}

class SimBackend;

struct Esp {
  simesp::Options opt;
  simesp::LogFn log = nullptr;
  std::mt19937 rng;
  uint64_t nowUs = 0;

  // power and boot
  bool up = false;               // booted: the AT server and the USB passthrough run
  uint64_t bootStartUs = 0;      // the last reset (power-on or esp_restart())
  bool softwareReset = false;
  bool bannerDue = false;
  std::string uart0;             // banner text not yet taken
  enum class Restart { NONE, FLUSH, DELAY } restart = Restart::NONE;
  uint64_t restartAtUs = 0;
  dsentry::ModeStore rtc{};      // RTC_NOINIT: survives esp_restart(), garbage at power-on
  uint8_t bootId = 0;
  uint32_t boots = 0;

  // the AT task
  std::unique_ptr<chAT::Server> srv;
  std::unique_ptr<DsentryAt> at;
  std::map<std::string, handler_t> table;
  std::queue<std::function<void()>> tasks;
  std::string rx;                // UART RX bytes the AT server has not read yet
  SimWire toEsp{kAtByteNs, 0};
  SimWire fromEsp{kAtByteNs, 0};
  uint64_t nextTickUs = 0;
  uint64_t lostWhileDown = 0;   // AT bytes that arrived while the chip was resetting
  uint64_t rxOverflow = 0;      // AT bytes beyond kAtRxBytes (never expected)

  // radios
  bool bleActive = false;        // xbox_begin() ran (NimBLE + the client task)
  uint8_t dsMode = DS_ESP_MODE_NONE;
  bool wifiStarted = false;
  dsentry::WifiSupervisor sup;
  uint64_t joinStartUs = 0;
  bool joining = false;
  int sock = -1;
  bool bindWarned = false;
  uint64_t lastBindTryUs = 0;
  ds_control_packet_t ctrl{};
  bool haveCtrl = false;
  uint32_t rxMs = 0;             // ESP32 millis() of the latest valid ControlPacket
  uint64_t rxSimUs = 0;          // the same, as simulator time
  sockaddr_in relay{};
  bool haveRelay = false;
  uint8_t status[DS_STATUS_FRAME_LEN] = {};
  bool statusPending = false;
  uint8_t telem[DS_TELEMETRY_FRAME_LEN] = {};
  bool telemPending = false;
  bool beaconSent = false;
  uint64_t lastBeaconUs = 0;
  uint8_t turretState = DS_STATE_OFF;

  // the fake pad (BLE mode)
  bool padOn = false;
  ds_control_packet_t pad{};

  // counters
  uint64_t udpIn = 0, udpBad = 0, statusOut = 0, telemOut = 0, beacons = 0, rumblesBle = 0;
  uint64_t xstateReads = 0;
  uint32_t restarts = 0;

  uint32_t millis() const { return (uint32_t)((nowUs - bootStartUs) / US_PER_MS); }
};

Esp g;

void logf(const char* fmt, ...) __attribute__((format(printf, 1, 2)));
void logf(const char* fmt, ...) {
  if (!g.log) return;
  char line[256];
  va_list ap;
  va_start(ap, fmt);
  vsnprintf(line, sizeof line, fmt, ap);
  va_end(ap);
  g.log(line);
}

std::string addrText(const sockaddr_in& a) {
  char ip[INET_ADDRSTRLEN] = "?";
  inet_ntop(AF_INET, &a.sin_addr, ip, sizeof ip);
  return std::string(ip) + ":" + std::to_string(ntohs(a.sin_port));
}

uint16_t theUdpPort() { return g.opt.udpPort ? g.opt.udpPort : (uint16_t)WIFI_UDP_PORT; }
uint16_t theBeaconPort() { return g.opt.beaconPort ? g.opt.beaconPort : (uint16_t)WIFI_DISCOVERY_PORT; }

bool wifiNetworkUp() { return g.sup.network_up(); }

uint8_t wifiStatusNow() {
  const bool heard = dsentry::relay_heard(g.haveCtrl, g.rxMs, g.millis());
  return dsentry::wifi_status(g.wifiStarted, g.sup.phase(), heard);
}

// xbox_begin() of xbox_client.cpp, minus NimBLE: idempotent, refused while D-Sentry WiFi runs.
int simXboxBegin() {
  if (g.bleActive) return XBOX_R4_RC_OK;
  if (g.wifiStarted) return XBOX_R4_RC_BLOCKED_BY_HCI;
  g.bleActive = true;
  return XBOX_R4_RC_OK;
}

// WiFi supervisor actions (dsentry_radio.cpp apply()): the station join is a timer here.
void apply(dsentry::WifiSupervisor::Action action) {
  switch (action) {
    case dsentry::WifiSupervisor::STA_BEGIN:
    case dsentry::WifiSupervisor::STA_REBEGIN:
      g.joining = true;
      g.joinStartUs = g.nowUs;
      logf("esp32: wifi: joining %s%s", WIFI_SSID, g.opt.wifiJoinMs < 0 ? " (the sim never lets it join)" : "");
      break;
    case dsentry::WifiSupervisor::AP_START:
      g.joining = false;
      logf("esp32: wifi: own access point %s up (the sim answers on 127.0.0.1)", WIFI_AP_SSID);
      break;
    case dsentry::WifiSupervisor::NONE:
    default:
      break;
  }
}

bool staConnected() {
  return g.joining && g.opt.wifiJoinMs >= 0 &&
         g.nowUs - g.joinStartUs >= (uint64_t)g.opt.wifiJoinMs * US_PER_MS;
}

// ---- the backend the AT handlers use (dsentry_radio.cpp RadioBackend) ------------------------
class SimBackend final : public DsentryBackend {
 public:
  dsentry::Radios radios() override {
    dsentry::Radios r;
    r.bleRunning = g.bleActive;
    r.hciUsed = false;               // the sim has no ArduinoBLE HCI bridge
    r.dsWifiRunning = g.wifiStarted;
    r.otherWifiRunning = false;      // no stock WiFi commands are served
    r.restartPending = false;
    return r;
  }
  uint8_t bootId() override { return g.bootId; }
  uint8_t wifiStatus() override { return wifiStatusNow(); }
  bool dsModeActive() override { return g.dsMode != DS_ESP_MODE_NONE; }
  bool startBle() override {
    // BLE_FORGET_PAIRINGS_ON_BOOT -> xbox_forget(): the sim keeps no bonds.
    const int rc = simXboxBegin();
    logf("esp32: BLE mode: xbox_begin() = %d%s", rc, g.opt.fakePad ? " (fake pad: stdin 'pad on')" : " (no pad)");
    if (rc != XBOX_R4_RC_OK) return false;
    g.dsMode = DS_ESP_MODE_BLE;
    return true;
  }
  bool startWifi() override {
    if (g.wifiStarted) return true;
    g.wifiStarted = true;
    g.sup = dsentry::WifiSupervisor();
    apply(g.sup.start(g.millis(), WIFI_SSID[0] != '\0', WIFI_MODE_AP_FALLBACK));
    g.dsMode = DS_ESP_MODE_WIFI;
    logf("esp32: WiFi mode started");
    return true;
  }
  void adoptMode(uint8_t espMode) override {
    if (g.dsMode == DS_ESP_MODE_NONE) g.dsMode = espMode;
  }
  void storeRestartRequest(uint8_t espMode) override {
    dsentry::store_write(&g.rtc, espMode, g.bootId);
    logf("esp32: restart into %s requested", ds_esp_mode_name(espMode));
  }
  void rumbleBle(const uint8_t report[XBOX_RUMBLE_REPORT_LEN]) override {
    g.rumblesBle++;
    if (!g.padOn) return;   // xbox_request_rumble_report(): dropped while no pad is connected
    char hex[2 * XBOX_RUMBLE_REPORT_LEN + 1];
    ds_hex_encode(report, XBOX_RUMBLE_REPORT_LEN, hex);
    logf("esp32: pad rumble report %s", hex);
  }
  void statusToRelay(const uint8_t* frame, size_t len) override {
    if (len != DS_STATUS_FRAME_LEN) return;
    memcpy(g.status, frame, DS_STATUS_FRAME_LEN);
    g.statusPending = true;
  }
  bool telemetryToRelay(const uint8_t* frame, size_t len) override {
    if (!g.wifiStarted || !wifiNetworkUp() || len != DS_TELEMETRY_FRAME_LEN) return false;
    if (!g.haveRelay) return false;
    memcpy(g.telem, frame, DS_TELEMETRY_FRAME_LEN);
    g.telemPending = true;
    return true;
  }
  void noteTurretState(uint8_t state) override { g.turretState = state; }
};

SimBackend g_backend;

// AT+XBOXSTATE?'s 20 bytes: dsentry_fill_xbox_state() in WiFi mode, else xbox_fill_state().
void fillXboxState(uint8_t out[XBOX_R4_STATE_LEN]) {
  g.xstateReads++;
  if (g.wifiStarted) {
    const bool live = dsentry::control_is_live(g.haveCtrl, g.ctrl, g.rxMs, g.millis());
    dsentry::control_to_xstate(g.haveCtrl ? &g.ctrl : nullptr, live, out);
    return;
  }
  if (!g.bleActive) {   // before AT+XBOXBEGIN: neutral, OFF
    xbox_r4_state_t s;
    memset(&s, 0, sizeof(s));
    xbox_r4_neutral_report(s.report);
    s.state = XBOX_R4_OFF;
    s.battery = XBOX_R4_BATTERY_UNKNOWN;
    s.seq = 0;
    xbox_r4_encode(&s, out);
    return;
  }
  // The client task: scanning with no pad, connected with the fake pad's latest report.
  dsentry::control_to_xstate(g.padOn ? &g.pad : nullptr, g.padOn, out);
}

void addCommands() {
  g.table.clear();
  // cmds_xbox.h
  g.table[XBOX_R4_AT_STATE] = [](chAT::Server& srv, chAT::ATParser& parser) {
    switch (parser.cmd_mode) {
      case chAT::CommandMode::Read: {
        std::vector<uint8_t> state(XBOX_R4_STATE_LEN);
        fillXboxState(state.data());
        srv.write_response_prompt();
        srv.write_str(std::to_string((int)XBOX_R4_STATE_LEN).c_str());
        srv.write_str("|");
        srv.write_vec8(state);
        srv.write_line_end();
        return chAT::CommandStatus::OK;
      }
      default:
        return chAT::CommandStatus::ERROR;
    }
  };
  g.table[XBOX_R4_AT_BEGIN] = [](chAT::Server& srv, chAT::ATParser& parser) {
    switch (parser.cmd_mode) {
      case chAT::CommandMode::Run: {
        int rc = simXboxBegin();
        srv.write_response_prompt();
        srv.write_str(std::to_string(rc).c_str());
        srv.write_line_end();
        return chAT::CommandStatus::OK;
      }
      default:
        return chAT::CommandStatus::ERROR;
    }
  };
  // cmds_esp_generic.h +SOFTRESETWIFI: the D-Sentry guard, then the stock reply (the stock body's
  // socket and WiFi teardown has nothing to tear down in the sim).
  g.table["+SOFTRESETWIFI"] = [](chAT::Server& srv, chAT::ATParser& parser) {
    switch (parser.cmd_mode) {
      case chAT::CommandMode::Run: {
        if (g.at->softResetWifi(srv)) return chAT::CommandStatus::OK;
        srv.write_response_prompt();
        srv.write_line_end();
        return chAT::CommandStatus::OK;
      }
      default:
        return chAT::CommandStatus::ERROR;
    }
  };
  // cmds_dsentry.h
  g.table[DS_AT_DSCAPS] = [](chAT::Server& srv, chAT::ATParser& parser) { return g.at->caps(srv, parser); };
  g.table[DS_AT_DSS] = [](chAT::Server& srv, chAT::ATParser& parser) {
    return g.at->dss(srv, parser, []() {
      g.tasks.push([]() {          // serial->flush(); dsentry_restart_now();
        g.restart = Esp::Restart::FLUSH;
      });
    });
  };
  g.table[DS_AT_DST] = [](chAT::Server& srv, chAT::ATParser& parser) { return g.at->dst(srv, parser); };
}

void startServer() {
  g.srv.reset(new chAT::Server());
  g.at.reset(new DsentryAt(g_backend));
  g.srv->set_io_callback({
      .callback_io_read = [](uint8_t* buf, size_t len) -> ssize_t {
        const size_t n = len < g.rx.size() ? len : g.rx.size();
        memcpy(buf, g.rx.data(), n);
        g.rx.erase(0, n);
        return (ssize_t)n;
      },
      .callback_io_write = [](uint8_t* buf, size_t len) -> ssize_t {
        g.fromEsp.send(g.nowUs, buf, len);
        return (ssize_t)len;
      },
  });
  g.srv->set_command_callback([](chAT::Server& s, const std::string& command) {
    auto it = g.table.find(command);
    if (it == g.table.end()) return chAT::CommandStatus::ERROR;
    return it->second(s, s.parser());
  });
  addCommands();
}

void closeSocket() {
  if (g.sock >= 0) close(g.sock);
  g.sock = -1;
}

// What a reset leaves: no radio, no socket, an empty UART, no AT server until boot().
void resetChip(bool software) {
  closeSocket();
  g.up = false;
  g.bootStartUs = g.nowUs;
  g.softwareReset = software;
  g.bannerDue = g.opt.romBanner;
  g.restart = Esp::Restart::NONE;
  g.srv.reset();
  g.at.reset();
  std::queue<std::function<void()>>().swap(g.tasks);
  g.rx.clear();
  g.fromEsp.clear();
  g.bleActive = false;
  g.dsMode = DS_ESP_MODE_NONE;
  g.wifiStarted = false;
  g.sup = dsentry::WifiSupervisor();
  g.joining = false;
  g.haveCtrl = false;
  g.haveRelay = false;
  g.statusPending = false;
  g.telemPending = false;
  g.beaconSent = false;
  g.bindWarned = false;
}

// setup(): dsentry_boot() before the AT task starts.
void boot() {
  dsentry::BootPlan plan = dsentry::plan_boot(&g.rtc, g.softwareReset, g.rng());
  g.bootId = plan.bootId;
  g.boots++;
  startServer();
  g.up = true;
  g.nextTickUs = g.nowUs;
  logf("esp32: boot (%s), boot id %u, radio %s", g.softwareReset ? "software reset" : "power-on",
       (unsigned)g.bootId, plan.startMode == DS_ESP_MODE_NONE ? "none (waits for AT+DSS)" : ds_esp_mode_name(plan.startMode));
  if (plan.startMode == DS_ESP_MODE_BLE)
    g_backend.startBle();
  else if (plan.startMode == DS_ESP_MODE_WIFI)
    g_backend.startWifi();
}

// ---- the WiFi task (dsentry_radio.cpp wifi_task) ----------------------------------------------
bool openSocket() {
  int s = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
  if (s < 0) return false;
  int on = 1;
  setsockopt(s, SOL_SOCKET, SO_REUSEADDR, &on, sizeof on);
  sockaddr_in local;
  memset(&local, 0, sizeof local);
  local.sin_family = AF_INET;
  local.sin_port = htons(theUdpPort());
  local.sin_addr.s_addr = htonl(INADDR_LOOPBACK);
  if (bind(s, (sockaddr*)&local, sizeof local) != 0) {
    if (!g.bindWarned) logf("esp32: wifi: cannot bind 127.0.0.1:%u: %s (retrying)", (unsigned)theUdpPort(), strerror(errno));
    g.bindWarned = true;
    close(s);
    return false;
  }
  g.sock = s;
  logf("esp32: wifi: UDP socket on 127.0.0.1:%u", (unsigned)theUdpPort());
  return true;
}

void sendTo(const sockaddr_in& to, const uint8_t* data, size_t len) {
  sendto(g.sock, data, len, 0, (const sockaddr*)&to, sizeof to);   // best effort, like UDP
}

void receiveAll() {
  for (;;) {
    uint8_t buf[DS_MAX_FRAME_LEN];   // longer datagrams are truncated, then fail the length check
    sockaddr_in from;
    socklen_t fromLen = sizeof from;
    const ssize_t n = recvfrom(g.sock, buf, sizeof buf, MSG_DONTWAIT, (sockaddr*)&from, &fromLen);
    if (n <= 0) return;
    ds_control_packet_t p;
    if (!dsentry::accept_control(buf, (size_t)n, &p)) {
      g.udpBad++;
      continue;
    }
    g.udpIn++;
    const bool newRelay = !g.haveRelay || g.relay.sin_addr.s_addr != from.sin_addr.s_addr ||
                          g.relay.sin_port != from.sin_port;
    g.ctrl = p;
    g.haveCtrl = true;
    g.rxMs = g.millis();
    g.rxSimUs = g.nowUs;
    g.relay = from;
    g.haveRelay = true;
    if (newRelay) logf("esp32: wifi: relay is %s", addrText(from).c_str());
  }
}

void sendOutbox() {
  if (!g.haveRelay) {   // nobody to tell yet: a StatusPacket is dropped (rumble is only now)
    g.statusPending = g.telemPending = false;
    return;
  }
  if (g.statusPending) {
    sendTo(g.relay, g.status, sizeof g.status);
    g.statusOut++;
  }
  if (g.telemPending) {
    sendTo(g.relay, g.telem, sizeof g.telem);
    g.telemOut++;
  }
  g.statusPending = g.telemPending = false;
}

void sendBeacon() {
  uint8_t frame[DS_BEACON_FRAME_LEN];
  dsentry::build_beacon_frame(kLoopbackIp, g.bootId, wifiStatusNow(), frame);
  if (theUdpPort() != WIFI_UDP_PORT) {   // the sim's socket is not on WIFI_UDP_PORT: say where it is
    ds_beacon_packet_t b;
    if (ds_beacon_decode(frame, sizeof frame, &b)) {
      b.udpPort = theUdpPort();
      ds_beacon_encode(&b, frame);
    }
  }
  sockaddr_in to;
  memset(&to, 0, sizeof to);
  to.sin_family = AF_INET;
  to.sin_port = htons(theBeaconPort());
  to.sin_addr.s_addr = htonl(INADDR_LOOPBACK);
  sendTo(to, frame, sizeof frame);
  g.beacons++;
}

void wifiTask() {
  if (!g.wifiStarted) return;
  const dsentry::WifiSupervisor::Phase before = g.sup.phase();
  apply(g.sup.step(g.millis(), staConnected()));
  if (g.sup.phase() != before && g.sup.phase() == dsentry::WifiSupervisor::STA_UP)
    logf("esp32: wifi: station up (the sim answers on 127.0.0.1)");
  if (g.sock < 0 && wifiNetworkUp() &&
      (g.lastBindTryUs == 0 || g.nowUs - g.lastBindTryUs >= (uint64_t)WIFI_TASK_POLL_MS * US_PER_MS)) {
    g.lastBindTryUs = g.nowUs;
    openSocket();
  }
  if (g.sock < 0) return;
  receiveAll();
  sendOutbox();
  if (g.opt.beacon && wifiNetworkUp() &&
      (!g.beaconSent || g.nowUs - g.lastBeaconUs >= (uint64_t)WIFI_BEACON_MS * US_PER_MS)) {
    sendBeacon();
    g.beaconSent = true;
    g.lastBeaconUs = g.nowUs;
  }
}

// CAtHandler::run(): at_srv.run(), then the tasks queued so far.
void atTask() {
  g.srv->run();
  const size_t n = g.tasks.size();
  for (size_t i = 0; i < n; i++) {
    auto task = g.tasks.front();
    g.tasks.pop();
    if (task) task();
  }
}

}  // namespace

// =================================================================================================
namespace simesp {

void begin(const Options& opt, LogFn log) {
  g.opt = opt;
  g.log = log;
  g.rng.seed(opt.seed);
  g.nowUs = 0;
  // RTC_NOINIT after power-on: whatever the cells hold (store_valid() rejects it).
  g.rtc.magic = g.rng();
  g.rtc.request = g.rng();
  g.rtc.lastBootId = g.rng();
  g.rtc.check = g.rng();
  resetChip(false);
}

void end() { closeSocket(); }

void fromRa(uint64_t nowUs, const std::string& bytes) {
  if (!bytes.empty()) g.toEsp.send(nowUs, bytes);
}

void run(uint64_t nowUs) {
  g.nowUs = nowUs;
  const std::string in = g.toEsp.take(nowUs);
  if (!g.up) {
    g.lostWhileDown += in.size();   // nobody is listening on the UART while the chip resets
    if (g.bannerDue && nowUs - g.bootStartUs >= kRomBannerAfterUs) {
      g.bannerDue = false;
      g.uart0 += kRomHead;
      g.uart0 += g.softwareReset ? kRomRstSoftware : kRomRstPowerOn;
      g.uart0 += kRomTail;
    }
    if (nowUs - g.bootStartUs >= (uint64_t)g.opt.bootMs * US_PER_MS) boot();
    return;
  }
  if (g.rx.size() + in.size() <= kAtRxBytes)
    g.rx += in;
  else
    g.rxOverflow += in.size();
  // The restart task: serial->flush() (the reply is fully on the wire), then
  // dsentry_restart_now(): vTaskDelay(ESP_RESTART_DELAY_MS) and esp_restart(). The AT task is
  // blocked in that task meanwhile; the WiFi task keeps running.
  if (g.restart == Esp::Restart::FLUSH && g.fromEsp.empty()) {
    g.restart = Esp::Restart::DELAY;
    g.restartAtUs = nowUs + (uint64_t)ESP_RESTART_DELAY_MS * US_PER_MS;
  }
  if (g.restart == Esp::Restart::DELAY && nowUs >= g.restartAtUs) {
    g.restarts++;
    logf("esp32: esp_restart() (boot id %u)", (unsigned)g.bootId);
    resetChip(true);
    return;
  }
  if (nowUs < g.nextTickUs) return;
  g.nextTickUs += kAtTickUs;
  if (g.nextTickUs <= nowUs) g.nextTickUs = nowUs + kAtTickUs;
  if (g.restart == Esp::Restart::NONE) atTask();
  wifiTask();
}

std::string toRa(uint64_t nowUs) { return g.fromEsp.take(nowUs); }

std::string takeUart0(uint64_t /*nowUs*/) {
  std::string out;
  out.swap(g.uart0);
  return out;
}

bool bridgeUp() { return g.up; }

bool padCommand(const std::string& args, std::string* reply) {
  if (!g.opt.fakePad) {
    *reply = "no fake pad (start the sim with --fake-pad)";
    return false;
  }
  char word[16] = {};
  int rx = 0, ry = 0, rt = 0, buttons = 0;
  if (sscanf(args.c_str(), "%15s", word) != 1) word[0] = '\0';
  if (strcmp(word, "on") == 0 || strcmp(word, "off") == 0) {
    g.padOn = strcmp(word, "on") == 0;
    g.pad.flags = g.padOn ? DS_CTRL_FLAG_CONNECTED : 0;
    g.pad.seq++;
  } else if (strcmp(word, "set") == 0 &&
             sscanf(args.c_str(), "%*s %d %d %d %i", &rx, &ry, &rt, &buttons) == 4) {
    if (rx < -JOY_AXIS_RANGE || rx >= JOY_AXIS_RANGE || ry < -JOY_AXIS_RANGE || ry >= JOY_AXIS_RANGE ||
        rt < 0 || rt > TRIGGER_AXIS_RANGE || buttons < 0 || buttons > UINT16_MAX) {
      *reply = "pad set: rx, ry -JOY_AXIS_RANGE..JOY_AXIS_RANGE-1, rt 0..TRIGGER_AXIS_RANGE, buttons 0..0xffff";
      return false;
    }
    g.pad.rx = (int16_t)rx;
    g.pad.ry = (int16_t)ry;
    g.pad.rt = (uint16_t)rt;
    g.pad.buttons = (uint16_t)buttons;
    g.pad.seq++;   // one HID notification
  } else if (word[0] != '\0') {
    *reply = "usage: pad on|off | pad set <rx> <ry> <rt> <buttons>   (ControlPacket units, stick up negative)";
    return false;
  }
  char text[160];
  snprintf(text, sizeof text, "pad %s rx=%d ry=%d rt=%u buttons=0x%03x%s", g.padOn ? "on" : "off", g.pad.rx,
           g.pad.ry, (unsigned)g.pad.rt, (unsigned)g.pad.buttons,
           g.bleActive ? "" : " (BLE is not running: the turret sees no pad)");
  *reply = text;
  return true;
}

std::string status() {
  char text[1024];
  const uint8_t espMode = dsentry::esp_mode_of(g_backend.radios());
  snprintf(text, sizeof text,
           "esp=%s espMode=%s bootId=%u boots=%u restarts=%u wifi=%s wifiStatus=%s udp=127.0.0.1:%u sock=%s "
           "relay=%s ctrlRxMs=%llu ctrlSeq=%u ctrlConnected=%d udpIn=%llu udpBad=%llu statusOut=%llu "
           "telemOut=%llu beacons=%llu xstate=%llu bleRumbles=%llu atLostWhileDown=%llu atRxOverflow=%llu pad=%s "
           "espTurretState=%s",
           g.up ? (g.restart == Esp::Restart::NONE ? "up" : "restarting") : "down",
           g.up && g.restart != Esp::Restart::NONE ? "RESTARTING" : ds_esp_mode_name(espMode),
           (unsigned)g.bootId, (unsigned)g.boots, (unsigned)g.restarts, phaseName(g.sup.phase()),
           ds_wifi_status_name(wifiStatusNow()), (unsigned)theUdpPort(), g.sock >= 0 ? "bound" : "closed",
           g.haveRelay ? addrText(g.relay).c_str() : "none",
           (unsigned long long)(g.haveCtrl ? g.rxSimUs / US_PER_MS : 0), (unsigned)g.ctrl.seq,
           g.haveCtrl && (g.ctrl.flags & DS_CTRL_FLAG_CONNECTED) ? 1 : 0, (unsigned long long)g.udpIn,
           (unsigned long long)g.udpBad, (unsigned long long)g.statusOut, (unsigned long long)g.telemOut,
           (unsigned long long)g.beacons, (unsigned long long)g.xstateReads, (unsigned long long)g.rumblesBle,
           (unsigned long long)g.lostWhileDown, (unsigned long long)g.rxOverflow, g.opt.fakePad ? (g.padOn ? "on" : "off") : "none", ds_turret_state_name(g.turretState));
  return text;
}

uint16_t udpPort() { return theUdpPort(); }

}  // namespace simesp

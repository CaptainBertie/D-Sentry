// test_controllink_chat.cpp -- the RA4M1's AT client (AsyncAt + ControlLink, on the host HAL)
// against the REAL chAT server the ESP32-S3 bridge runs: firmware/UNOR4USBBridge/parser.cpp
// and server.cpp, compiled from the fork (read-only, AGPL-3.0; never copied), with fake
// command handlers. docs/architecture.md 8.2. Ported from the research harness
// scratchpad/research/ra4m1_link/async_at/test_async_chat.cpp.
//
// Handlers: +XBOXSTATE / +XBOXBEGIN as firmware/UNOR4USBBridge/cmds_xbox.h writes them (a drift
// guard reads that file), and +DSCAPS / +DSS / +DST following protocol_spec.py AT_COMMANDS
// (a guard checks cmds_dsentry.h registers the same three). The research shapes (+XBOXVERSION,
// +XBOXSCAN, +SOFTRESETWIFI, +HCIBEGIN, +HCIREAD) keep AsyncAt honest about every chAT form.
//
// Part A  every reply shape from the real server, fed to AsyncAt whole, byte by byte and in
//         200 random splits; ControlLink's request lines parsed by the real chAT parser.
// Part B  virtual time: two 115200 8N1 wires (86.8 us/byte), the ESP32 AT task running
//         at_srv.run() every 1.0-1.2 ms, the RA4M1 loop() every 20..2000 us with a 6 ms stall
//         1 time in 400. B1 60 s BLE at 100 Hz; B2 the switch to WiFi (ESP32 restart) and 20 s
//         of AT+DST at 10 Hz next to the polls; B3 rumble commands each reach the ESP32 once.
// Part C  faults: C1 garbage on the RA4M1's RX; C2 ESP32 restart mid-request with 400 ms of
//         silence; C3 AT task stalls 30 ms (replies late but in order); C4 a stale 231-byte
//         +XBOXSCAN reply full of CR/LF/'+'/'|'; C5 legacy firmware (no D-Sentry commands).
//
// Key safety property (B1, C1, C3, C4): the turret is never handed a controller frame older
// than one it already accepted, and the link never drops while the ESP32 answers in time.
//
// Usage: test_controllink_chat <repo root>   (the root is needed for the drift guards)
//
// SPDX-License-Identifier: MIT
#include "chAT.hpp"

#include <cstdio>
#include <cstring>
#include <deque>
#include <fstream>
#include <functional>
#include <map>
#include <memory>
#include <random>
#include <sstream>
#include <string>
#include <vector>

#include "../xbox_r4_protocol.h"   // the sketch's symlink, through -I src
#include "AsyncAt.h"
#include "ControlLink.h"
#include "Debug.h"
#include "hal.h"
#include "hal_host.h"

using namespace SudoMaker;

static int g_fail = 0;
#define EXPECT(c, ...)                                         \
  do {                                                         \
    if (!(c)) {                                                \
      g_fail++;                                                \
      std::printf("FAIL %s:%d %s: ", __FILE__, __LINE__, #c);  \
      std::printf(__VA_ARGS__);                                \
      std::printf("\n");                                       \
    }                                                          \
  } while (0)
static void section(const char* name, int before) {
  if (g_fail == before) std::printf("ok   %s\n", name);
}

typedef std::function<chAT::CommandStatus(chAT::Server&, chAT::ATParser&)> handler_t;
static const double kByteUs = 1e6 * UART_BITS_PER_BYTE / LINK_BAUD;   // 86.806 us

// ---- what the ESP32 knows (survives a "restart" only where RTC_NOINIT would) -----------------
struct EspState {
  bool dsFirmware = true;
  uint8_t espMode = DS_ESP_MODE_NONE;
  uint8_t bootId = 0x5C;
  uint8_t rtcWanted = DS_ESP_MODE_NONE;   // RTC_NOINIT: the mode to boot into
  bool restartRequested = false;
  int lastRumbleSeq = -1;
  std::vector<int> rumbleSeqs;            // each new rumbleSeq, in order
  std::vector<std::vector<uint8_t>> dst;  // every frame AT+DST delivered
  std::vector<std::vector<std::string>> dssArgs;
  xbox_r4_state_t st{};
  bool beginConnects = true;
  std::string scanText;
};
static EspState g_esp;

// ---- one chAT server; input bytes from `rx`, output to `tx` ----------------------------------
struct Esp {
  chAT::Server srv;
  std::map<std::string, handler_t> table;
  std::deque<uint8_t> rx;
  std::string tx;

  Esp() {
    srv.set_io_callback({
        .callback_io_read = [this](uint8_t* buf, size_t len) -> ssize_t {
          size_t n = 0;
          while (n < len && !rx.empty()) {
            buf[n++] = rx.front();
            rx.pop_front();
          }
          return (ssize_t)n;
        },
        .callback_io_write = [this](uint8_t* buf, size_t len) -> ssize_t {
          tx.append((const char*)buf, len);
          return (ssize_t)len;
        },
    });
    // same dispatch as CAtHandler's constructor in at_handler.cpp (unknown -> ERROR)
    srv.set_command_callback([this](chAT::Server& s, const std::string& cmd) {
      auto it = table.find(cmd);
      if (it == table.end()) return chAT::CommandStatus::ERROR;
      return it->second(s, s.parser());
    });
    addXbox();
    if (g_esp.dsFirmware) addDsentry();
  }

  // cmds_xbox.h (std::to_string stands in for Arduino String(int)); kFirmwareStateCalls below.
  void addXbox() {
    table[XBOX_R4_AT_STATE] = [](chAT::Server& srv, chAT::ATParser& parser) {
      switch (parser.cmd_mode) {
        case chAT::CommandMode::Read: {
          xbox_r4_state_t s = g_esp.st;
          if (g_esp.dsFirmware && g_esp.espMode != DS_ESP_MODE_BLE && g_esp.espMode != DS_ESP_MODE_WIFI) {
            xbox_r4_neutral_report(s.report);
            s.state = XBOX_R4_OFF;
          }
          std::vector<uint8_t> state(XBOX_R4_STATE_LEN);
          xbox_r4_encode(&s, state.data());
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
    table[XBOX_R4_AT_BEGIN] = [](chAT::Server& srv, chAT::ATParser& p) {
      if (p.cmd_mode != chAT::CommandMode::Run) return chAT::CommandStatus::ERROR;
      if (g_esp.st.state == XBOX_R4_OFF) g_esp.st.state = g_esp.beginConnects ? XBOX_R4_CONNECTED : XBOX_R4_SCANNING;
      srv.write_response_prompt();
      srv.write_str("0");
      srv.write_line_end();
      return chAT::CommandStatus::OK;
    };
    table[XBOX_R4_AT_VERSION] = [](chAT::Server& srv, chAT::ATParser& p) {
      if (p.cmd_mode != chAT::CommandMode::Read) return chAT::CommandStatus::ERROR;
      srv.write_response_prompt();
      srv.write_str("1");
      srv.write_str(",");
      srv.write_str("0.6.99");
      srv.write_line_end();
      return chAT::CommandStatus::OK;
    };
    table[XBOX_R4_AT_SCAN] = [](chAT::Server& srv, chAT::ATParser& p) {
      if (p.cmd_mode != chAT::CommandMode::Read) return chAT::CommandStatus::ERROR;
      std::vector<uint8_t> bytes(g_esp.scanText.begin(), g_esp.scanText.end());
      srv.write_response_prompt();
      srv.write_str(std::to_string((int)bytes.size()).c_str());
      srv.write_str("|");
      srv.write_vec8(bytes);
      srv.write_line_end();
      return chAT::CommandStatus::OK;
    };
    // cmds_esp_generic.h +SOFTRESETWIFI: prompt + line end only
    table["+SOFTRESETWIFI"] = [](chAT::Server& srv, chAT::ATParser& p) {
      if (p.cmd_mode != chAT::CommandMode::Run) return chAT::CommandStatus::ERROR;
      srv.write_response_prompt();
      srv.write_line_end();
      return chAT::CommandStatus::OK;
    };
    // cmds_ble_bridge.h +HCIBEGIN while Xbox mode is active: prompt, then ERROR
    table["+HCIBEGIN"] = [](chAT::Server& srv, chAT::ATParser&) {
      srv.write_response_prompt();
      return chAT::CommandStatus::ERROR;
    };
    // cmds_ble_bridge.h +HCIREAD: sized payload with no CRLF before OK
    table["+HCIREAD"] = [](chAT::Server& srv, chAT::ATParser&) {
      std::vector<uint8_t> d = {0x04, 0x0E, 0x0D, 0x0A};
      srv.write_response_prompt();
      srv.write_str("4");
      srv.write_str("|");
      srv.write_vec8(d);
      return chAT::CommandStatus::OK;
    };
  }

  static bool decimal(const std::string& s, uint32_t max, uint32_t& out) {
    if (s.empty() || s.size() > 5) return false;
    uint32_t v = 0;
    for (char c : s) {
      if (c < '0' || c > '9') return false;
      v = v * 10 + (uint32_t)(c - '0');
    }
    if (v > max) return false;
    out = v;
    return true;
  }

  // protocol_spec.py AT_COMMANDS: every command answers with its prompt and OK.
  void addDsentry() {
    table[DS_AT_DSCAPS] = [](chAT::Server& srv, chAT::ATParser& p) {
      if (p.cmd_mode != chAT::CommandMode::Read) return chAT::CommandStatus::ERROR;
      srv.write_response_prompt();
      srv.write_str((std::to_string(DS_PROTO_VERSION) + "," + std::to_string(DS_CAP_ALL)).c_str());
      srv.write_line_end();
      return chAT::CommandStatus::OK;
    };
    table[DS_AT_DSS] = [](chAT::Server& srv, chAT::ATParser& p) {
      if (p.cmd_mode != chAT::CommandMode::Write || p.args.size() != DS_AT_DSS_ARGC)
        return chAT::CommandStatus::ERROR;
      uint32_t v[DS_AT_DSS_ARGC];
      for (size_t i = 0; i < DS_AT_DSS_ARGC; i++) {
        const uint32_t max = i == DS_AT_DSS_ARG_RUMBLE_MS ? UINT16_MAX : UINT8_MAX;
        if (!decimal(p.args[i], max, v[i])) return chAT::CommandStatus::ERROR;
      }
      g_esp.dssArgs.push_back(p.args);
      const uint8_t want = v[DS_AT_DSS_ARG_WANT_MODE] == DS_RADIO_WIFI ? DS_ESP_MODE_WIFI : DS_ESP_MODE_BLE;
      uint8_t reply = g_esp.espMode;
      if (g_esp.espMode == DS_ESP_MODE_NONE) {
        g_esp.espMode = reply = want;
      } else if (g_esp.espMode != want) {
        g_esp.rtcWanted = want;
        g_esp.restartRequested = true;
        reply = DS_ESP_MODE_RESTARTING;
      }
      if (!g_esp.restartRequested && (int)v[DS_AT_DSS_ARG_RUMBLE_SEQ] != g_esp.lastRumbleSeq) {
        if (g_esp.lastRumbleSeq >= 0) g_esp.rumbleSeqs.push_back((int)v[DS_AT_DSS_ARG_RUMBLE_SEQ]);
        g_esp.lastRumbleSeq = (int)v[DS_AT_DSS_ARG_RUMBLE_SEQ];
      }
      const int wifi = g_esp.espMode == DS_ESP_MODE_WIFI ? DS_WIFI_STATUS_RELAY_HEARD : DS_WIFI_STATUS_OFF;
      srv.write_response_prompt();
      srv.write_str((std::to_string(reply) + "," + std::to_string(g_esp.bootId) + "," + std::to_string(wifi)).c_str());
      srv.write_line_end();
      return chAT::CommandStatus::OK;
    };
    table[DS_AT_DST] = [](chAT::Server& srv, chAT::ATParser& p) {
      if (p.cmd_mode != chAT::CommandMode::Write || p.args.size() != DS_AT_DST_ARGC)
        return chAT::CommandStatus::ERROR;
      const std::string& hex = p.args[DS_AT_DST_ARG_HEX];
      std::vector<uint8_t> f(hex.size() / 2 + 1);
      const size_t n = ds_hex_decode(hex.data(), hex.size(), f.data(), f.size());
      f.resize(n);
      const bool ok = n == DS_TELEMETRY_FRAME_LEN && ds_frame_check(f.data(), n, DS_PKT_TELEMETRY) &&
                      g_esp.espMode == DS_ESP_MODE_WIFI;
      if (ok) g_esp.dst.push_back(f);
      srv.write_response_prompt();
      srv.write_str(ok ? "1" : "0");
      srv.write_line_end();
      return chAT::CommandStatus::OK;
    };
  }

  std::string transact(const std::string& req) {
    for (unsigned char c : req) rx.push_back(c);
    tx.clear();
    for (size_t i = 0; i < req.size() + 4; i++) srv.run();
    return tx;
  }
};

static std::string hex(const std::string& s) {
  std::string o;
  char t[4];
  for (unsigned char c : s) {
    std::snprintf(t, sizeof t, "%02X ", c);
    o += t;
  }
  return o;
}

// =============================================================================================
// Part A: reply shapes, and ControlLink's request lines through the real parser
// =============================================================================================
static void partA() {
  int f0 = g_fail;
  std::mt19937 rng(20261103);
  const uint8_t rep[16] = {0x0D, 0x0A, 0x7C, 'O', 'K', 0x00, 0xFF, 0x0D, 0x0A, 0x7C, 'O', 'K', 0x0D, 0x0A, 0x00, 0xFF};
  g_esp = EspState();
  memcpy(g_esp.st.report, rep, sizeof rep);
  g_esp.st.state = XBOX_R4_CONNECTED;
  g_esp.st.battery = 64;
  g_esp.st.seq = 0x7C;
  g_esp.espMode = DS_ESP_MODE_BLE;
  uint8_t enc[XBOX_R4_STATE_LEN];
  xbox_r4_encode(&g_esp.st, enc);
  g_esp.scanText = "aa:bb:cc:dd:ee:ff,964,-60,0600030080,,Xbox Wireless Controller,3;\r\nOK\r\n+XBOXSTATE: 20|";
  ds_telemetry_packet_t tp;
  ds_telemetry_init(&tp);
  tp.seq = 9;
  uint8_t tf[DS_TELEMETRY_FRAME_LEN];
  ds_telemetry_encode(&tp, tf);
  char tfHex[DS_AT_DST_HEX_LEN + 1];
  ds_hex_encode(tf, sizeof tf, tfHex);
  struct Case {
    std::string req;
    const char* prompt;
    AsyncAt::Result want;
    std::string payload;
  };
  const std::vector<Case> cases = {
      {"AT+XBOXSTATE?\r\n", XBOX_R4_AT_STATE, AsyncAt::OK, std::string((const char*)enc, sizeof enc)},
      {"AT+XBOXVERSION?\r\n", XBOX_R4_AT_VERSION, AsyncAt::OK, "1,0.6.99"},
      {"AT+XBOXBEGIN\r\n", XBOX_R4_AT_BEGIN, AsyncAt::OK, "0"},
      {"AT+XBOXSCAN?\r\n", XBOX_R4_AT_SCAN, AsyncAt::OK, g_esp.scanText},
      {"AT+SOFTRESETWIFI\r\n", "+SOFTRESETWIFI", AsyncAt::OK, ""},
      {"AT+HCIBEGIN\r\n", "+HCIBEGIN", AsyncAt::ERR_PROMPT, ""},
      {"AT+HCIREAD\r\n", "+HCIREAD", AsyncAt::OK, std::string("\x04\x0E\x0D\x0A", 4)},
      {"AT+XBOXSTATE\r\n", XBOX_R4_AT_STATE, AsyncAt::ERR_BARE, ""},   // wrong mode
      {"AT+NOPE?\r\n", "+NOPE", AsyncAt::ERR_BARE, ""},                // unknown
      {"AT\r\n", nullptr, AsyncAt::OK, ""},                             // the ping
      {"\r\n", nullptr, AsyncAt::ERR_BARE, ""},                         // blank line
      {"XYZ\r\n", nullptr, AsyncAt::ERR_BARE, ""},                      // no AT keyword
      {"AT+DSCAPS?\r\n", DS_AT_DSCAPS, AsyncAt::OK,
       std::to_string(DS_PROTO_VERSION) + "," + std::to_string(DS_CAP_ALL)},
      {"AT+DSS=0,0,0,0,0,0\r\n", DS_AT_DSS, AsyncAt::OK, "0,92,0"},
      {"AT+DSS=1,42,200,120,80,5\r\n", DS_AT_DSS, AsyncAt::OK, "3,92,0"},   // BLE running -> restart
      {"AT+DSS=9\r\n", DS_AT_DSS, AsyncAt::ERR_BARE, ""},                    // wrong argc
      {"AT+DST=" + std::string(tfHex) + "\r\n", DS_AT_DST, AsyncAt::OK, "0"},   // not in WiFi
      {"AT+DST=zz\r\n", DS_AT_DST, AsyncAt::OK, "0"},
  };
  for (const Case& c : cases) {
    g_esp.espMode = DS_ESP_MODE_BLE;
    g_esp.restartRequested = false;
    Esp esp;
    const std::string wire = esp.transact(c.req);
    std::printf("info reply to %-24.24s %3zu B: %s\n", c.req.substr(0, c.req.size() - 2).c_str(), wire.size(),
                wire.size() <= 40 ? hex(wire).c_str() : (hex(wire.substr(0, 20)) + "...").c_str());
    for (int mode = 0; mode < 3; mode++) {
      for (int r = 0; r < (mode == 2 ? 200 : 1); r++) {
        uint8_t buf[1024];
        AsyncAt at(buf, sizeof buf);
        at.start(c.prompt, 0, 20000);
        size_t i = 0;
        AsyncAt::Result res = AsyncAt::PENDING;
        while (i < wire.size()) {
          size_t n = mode == 0 ? wire.size() : mode == 1 ? 1 : 1 + rng() % 9;
          for (size_t k = 0; k < n && i < wire.size(); k++) at.feed((uint8_t)wire[i++]);
          AsyncAt::Result rr = at.check(1);
          if (rr != AsyncAt::PENDING) {
            res = rr;
            EXPECT(i == wire.size(), "%s completed with %zu bytes unread", c.req.c_str(), wire.size() - i);
            break;
          }
        }
        std::string got((const char*)at.data(), at.length());
        EXPECT(res == c.want, "%s mode %d: got %s want %s", c.req.c_str(), mode, AsyncAt::resultName(res),
               AsyncAt::resultName(c.want));
        if (c.want == AsyncAt::OK)
          EXPECT(got == c.payload, "%s mode %d payload %s", c.req.c_str(), mode, hex(got).c_str());
      }
    }
  }
  section("A1: every reply shape from the real chAT server, whole / 1-byte / 200 random splits", f0);

  // The real parser splits ControlLink's DSS line into exactly the six decimal arguments, and
  // takes the 188-hex-digit DST argument whole.
  f0 = g_fail;
  g_esp = EspState();
  g_esp.espMode = DS_ESP_MODE_WIFI;
  {
    Esp esp;
    esp.transact("AT+DSS=1,42,200,120,80,5\r\n");
    EXPECT(g_esp.dssArgs.size() == 1, "DSS not parsed");
    if (!g_esp.dssArgs.empty()) {
      const std::vector<std::string> want = {"1", "42", "200", "120", "80", "5"};
      EXPECT(g_esp.dssArgs[0] == want, "DSS args differ");
    }
    const std::string reply = esp.transact("AT+DST=" + std::string(tfHex) + "\r\n");
    EXPECT(reply == "+DST: 1\r\nOK\r\n", "DST reply %s", hex(reply).c_str());
    EXPECT(g_esp.dst.size() == 1 && g_esp.dst[0] == std::vector<uint8_t>(tf, tf + sizeof tf), "DST bytes differ");
    EXPECT(strlen("AT+DST=") + strlen(tfHex) + 2 == DS_AT_DST_LINE_LEN, "DST line length");
  }
  section("A2: the real chAT parser reads AT+DSS's six arguments and AT+DST's 188 hex digits exactly", f0);
}

// =============================================================================================
// Part B / C: virtual time
// =============================================================================================
struct Wire {
  std::deque<std::pair<double, uint8_t>> q;
  double last = 0;
  void send(double now, const std::string& s) {
    for (unsigned char c : s) {
      last = std::max(now, last) + kByteUs;
      q.push_back({last, c});
    }
  }
  template <class F>
  void deliver(double now, F f) {
    while (!q.empty() && q.front().first <= now) {
      f(q.front().second);
      q.pop_front();
    }
  }
  void clear() { q.clear(); }
};

struct Sim {
  // configuration
  int fault = 0;           // 0 none, 1 garbage, 2 restart, 3 late replies, 4 stale scan
  uint32_t seed = 1;
  bool wantWifiAt5s = false;
  bool rumbleEvery300ms = false;
  double lastRumbleUs = 0;
  bool offerTelemetry = false;
  // results
  int accepted = 0, older = 0, losses = 0, offSeen = 0;
  double connectedUs = 0;
  uint32_t restarts = 0;
  std::vector<double> lossAt;
};

struct Run {
  Sim& s;
  std::mt19937 rng;
  std::unique_ptr<Esp> esp;
  Debug dbg;
  ControlLink link;
  Wire ra2esp, esp2ra;
  bool espAlive = true;
  double espDeadUntil = 0;
  double now = 0, nextEsp = 0, nextRa = 0, nextReport = 0;
  double lateUntil = -1;
  std::string lateReply;
  int lastSeq = -1;
  uint8_t padSeq = 0;
  bool wasConnected = false;

  explicit Run(Sim& sim) : s(sim), rng(sim.seed) {
    hal_host::reset();
    hal::begin();
    dbg.begin(DebugSources());
    link.begin(dbg, hal::nowMs(), hal::nowUs());
    esp.reset(new Esp());
  }

  void restartEsp(double silenceUs) {
    espAlive = false;
    espDeadUntil = now + silenceUs;
    ra2esp.clear();
    esp2ra.clear();
    lateReply.clear();
    lateUntil = -1;
    g_esp.bootId++;
    g_esp.espMode = g_esp.rtcWanted;   // boots straight into the stored mode (or NONE)
    g_esp.rtcWanted = DS_ESP_MODE_NONE;
    g_esp.restartRequested = false;
    g_esp.lastRumbleSeq = -1;
    esp.reset(new Esp());              // a fresh chAT server, as after a reboot
    s.restarts++;
  }

  void step() {
    // the pad: HID reports at ~125 Hz; seq and a stick follow the report counter
    if (now >= nextReport) {
      nextReport += 8000;
      padSeq++;
      g_esp.st.seq = padSeq;
      g_esp.st.report[XSTATE_OFS_JOY_RH] = padSeq;
    }
    if (g_esp.restartRequested && espAlive) restartEsp(ESP_RESTART_DELAY_MS * 1000.0 + 300000.0);
    if (s.fault == 2 && s.restarts == 0 && now >= 5e6) {
      g_esp.rtcWanted = g_esp.espMode;   // a crash: comes back in the same mode
      restartEsp(400e3);
    }
    if (!espAlive && now >= espDeadUntil) espAlive = true;
    // wires
    ra2esp.deliver(now, [&](uint8_t c) {
      if (espAlive) esp->rx.push_back(c);
    });
    esp2ra.deliver(now, [&](uint8_t c) { hal_host::linkInject(&c, 1); });
    if (s.fault == 1 && rng() % 5000 == 0) {   // ~40 garbage bytes/s at 5 us ticks
      const uint8_t g = (uint8_t)rng();
      hal_host::linkInject(&g, 1);
    }
    // ESP32 AT task: at_srv.run() then vTaskDelay(1)
    if (espAlive && now >= nextEsp) {
      nextEsp = now + 1000 + rng() % 200;
      esp->srv.run();
      if (!esp->tx.empty()) {
        if (s.fault == 3 && lateUntil < 0 && rng() % 500 == 0) lateUntil = now + 30000;
        if (lateUntil >= 0)
          lateReply += esp->tx;   // the AT task stalls: later replies queue behind
        else
          esp2ra.send(now, esp->tx);
        esp->tx.clear();
      }
    }
    if (lateUntil >= 0 && now >= lateUntil) {
      esp2ra.send(now, lateReply);
      lateReply.clear();
      lateUntil = -1;
    }
    if (s.fault == 4 && now >= 3e6 && now < 3e6 + 5) {
      g_esp.scanText = std::string("x\r\nOK\r\n+XBOXSTATE: 20|") + std::string(200, '|') + "\r\nERROR\r\n";
      Esp tmp;
      esp2ra.send(now, tmp.transact("AT+XBOXSCAN?\r\n"));
    }
    // RA4M1 loop()
    if (now >= nextRa) {
      double period = 20 + rng() % 1980;
      if (rng() % 400 == 0) period = 6000;   // an LCD/LED burst
      nextRa = now + period;
      hal_host::setTimeUs((uint64_t)now);
      hal::service(hal::nowUs());
      if (s.wantWifiAt5s && now >= 5e6) link.setWantMode(DS_RADIO_WIFI);
      if (s.rumbleEvery300ms && now >= 2e6 && now < 8e6 && now - s.lastRumbleUs >= 300000) {
        s.lastRumbleUs = now;
        link.requestRumble(RUMBLE_STRONG_255, RUMBLE_WEAK_255, RUMBLE_DURATION_MS);
      }
      link.update(hal::nowMs(), hal::nowUs());
      if (s.offerTelemetry && (uint64_t)now / 50000 != (uint64_t)(now - period) / 50000) {
        ds_telemetry_packet_t tp;
        ds_telemetry_init(&tp);
        tp.uptimeMs = hal::nowMs();
        uint8_t f[DS_TELEMETRY_FRAME_LEN];
        ds_telemetry_encode(&tp, f);
        link.offerTelemetry(f, sizeof f);
      }
      hal::service(hal::nowUs());
      for (unsigned char c : hal_host::linkTakeTx()) ra2esp.send(now, std::string(1, (char)c));
      // what the turret sees
      const bool c = link.connected();
      if (c) {
        const int sq = link.frame().seq;
        if (lastSeq >= 0 && (uint8_t)(sq - lastSeq) > 128) s.older++;
        if (sq != lastSeq) s.accepted++;
        lastSeq = sq;
      }
      if (wasConnected && !c) {
        s.losses++;
        s.lossAt.push_back(now);
      }
      if (c) s.connectedUs += period;
      wasConnected = c;
      (void)hal_host::usbTakeTx();
    }
    now += 5;
  }
};

static void report(const char* name, const Sim& s, const ControlLink& link, double seconds) {
  const LinkStats& st = link.stats();
  std::printf("info %s %.0f s: polls %u timeouts %u errors %u crc %u stale %u junk %u frames %u (%.1f/s) "
              "losses %d older %d restarts %u dss %u dst %u/%u blockingWrites %u/%u\n",
              name, seconds, st.polls, st.pollTimeouts, st.pollErrors, st.crcErrors, st.staleReplies,
              st.junkBytes, st.radioFrames, st.radioFrames / seconds, s.losses, s.older, s.restarts, st.dssSent,
              st.telemetrySent, st.telemetryNotSent, hal_host::linkTxProbe().blockingWrites,
              hal_host::usbTxProbe().blockingWrites);
}

static void runFor(Run& r, double seconds) {
  const double end = r.now + seconds * 1e6;
  while (r.now < end) r.step();
}

static void initPad() {
  xbox_r4_neutral_report(g_esp.st.report);
  g_esp.st.state = XBOX_R4_CONNECTED;
  g_esp.st.battery = 80;
}

static void partB() {
  // B1: 60 s of BLE at CONTROL_PACKET_HZ.
  int f0 = g_fail;
  {
    g_esp = EspState();
    initPad();
    Sim s;
    s.seed = 1;
    Run r(s);
    runFor(r, 60);
    report("B1 BLE", s, r.link, 60);
    const LinkStats& st = r.link.stats();
    EXPECT(st.pollTimeouts == 0 && st.pollErrors == 0 && st.crcErrors == 0, "clean link had failures");
    EXPECT(st.radioFrames >= (uint32_t)(CONTROL_PACKET_HZ * 60 * 98 / 100), "only %u frames in 60 s", st.radioFrames);
    EXPECT(s.losses == 0, "%d link losses", s.losses);
    EXPECT(s.older == 0, "%d older frames", s.older);
    EXPECT(r.link.status().espMode == DS_ESP_MODE_BLE, "espMode %u", r.link.status().espMode);
    EXPECT(hal_host::linkTxProbe().blockingWrites == 0, "blocking link writes");
  }
  section("B1: 60 s BLE at CONTROL_PACKET_HZ against the real chAT: every poll answered, never older", f0);

  // B2: switch to WiFi at 5 s (the ESP32 restarts into it), then AT+DST at TELEMETRY_WIFI_HZ.
  f0 = g_fail;
  {
    g_esp = EspState();
    initPad();
    Sim s;
    s.seed = 2;
    s.wantWifiAt5s = true;
    s.offerTelemetry = true;
    Run r(s);
    runFor(r, 8);
    EXPECT(r.link.status().espMode == DS_ESP_MODE_WIFI, "not in WiFi after the switch (espMode %u)",
           r.link.status().espMode);
    EXPECT(r.link.connected() && r.link.status().activeSource == DS_LINK_SRC_WIFI, "WiFi source not connected");
    EXPECT(s.restarts == 1, "%u restarts", s.restarts);
    const uint32_t to0 = r.link.stats().pollTimeouts;
    const uint32_t frames0 = r.link.stats().radioFrames;
    const size_t dst0 = g_esp.dst.size();
    const int losses0 = s.losses;
    runFor(r, 20);
    report("B2 WiFi+DST", s, r.link, 28);
    const size_t dst = g_esp.dst.size() - dst0;
    std::printf("info B2 DST window 20 s: %u control frames (%.1f/s of %d) beside %zu AT+DST lines of %u B\n",
                r.link.stats().radioFrames - frames0, (r.link.stats().radioFrames - frames0) / 20.0,
                CONTROL_PACKET_HZ, dst, (unsigned)DS_AT_DST_LINE_LEN);
    EXPECT(dst >= (size_t)(TELEMETRY_WIFI_HZ * 20 - 2) && dst <= (size_t)(TELEMETRY_WIFI_HZ * 20 + 1),
           "%zu DST frames in 20 s", dst);
    EXPECT(r.link.stats().pollTimeouts == to0, "%u timeouts with DST running", r.link.stats().pollTimeouts - to0);
    // Each 229-byte line holds the link ~35 ms here (15 FIFO bursts, each waiting for the next
    // loop() after the FIFO empties, with this harness's pessimistic 20..2000 us loop): the
    // polls it displaces cost ~28 % of CONTROL_PACKET_HZ while WiFi telemetry runs (71.7/s
    // measured; ~24 %, 76.0/s, with the 197-byte version 1 line).
    EXPECT(r.link.stats().radioFrames - frames0 >= (uint32_t)(CONTROL_PACKET_HZ * 20 * 7 / 10),
           "polls starved by DST: %u frames in 20 s", r.link.stats().radioFrames - frames0);
    EXPECT(s.losses == losses0, "link lost during DST");
    for (const auto& f : g_esp.dst) EXPECT(ds_frame_check(f.data(), f.size(), DS_PKT_TELEMETRY), "bad DST frame");
    EXPECT(hal_host::linkTxProbe().blockingWrites == 0, "%u blocking link writes", hal_host::linkTxProbe().blockingWrites);
    EXPECT(hal_host::linkTxProbe().largestWrite <= (size_t)SCI_FIFO_DEPTH, "write of %zu bytes",
           hal_host::linkTxProbe().largestWrite);
  }
  section("B2: BLE -> WiFi switch through an ESP32 restart, then AT+DST at TELEMETRY_WIFI_HZ beside the polls", f0);

  // B3: rumble commands every 300 ms each reach the ESP32 exactly once, in order.
  f0 = g_fail;
  {
    g_esp = EspState();
    initPad();
    Sim s;
    s.seed = 3;
    s.rumbleEvery300ms = true;
    Run r(s);
    runFor(r, 10);
    const int sent = r.link.rumbleSeq();
    EXPECT(sent == 20, "%d rumbles requested", sent);
    EXPECT((int)g_esp.rumbleSeqs.size() == sent, "ESP32 saw %zu new rumbleSeqs for %d requests",
           g_esp.rumbleSeqs.size(), sent);
    for (size_t i = 0; i < g_esp.rumbleSeqs.size(); i++)
      EXPECT(g_esp.rumbleSeqs[i] == (int)i + 1, "rumbleSeq %zu is %d", i, g_esp.rumbleSeqs[i]);
  }
  section("B3: every rumble command reaches the ESP32 once, in order (heartbeats repeat, never re-fire)", f0);
}

static void partC() {
  int f0 = g_fail;
  {
    g_esp = EspState();
    initPad();
    Sim s;
    s.seed = 11;
    s.fault = 1;
    Run r(s);
    runFor(r, 60);
    report("C1 garbage", s, r.link, 60);
    EXPECT(s.older == 0, "%d older frames accepted", s.older);
    EXPECT(s.losses == 0, "%d link losses from ~40 garbage bytes/s", s.losses);
    // Each garbage byte costs about a third of a poll (the research harness measured 87 % of
    // polls OK at this rate); what matters is that no failure lasts LINK_TIMEOUT_MS.
    EXPECT(r.link.stats().radioFrames >= (uint32_t)(CONTROL_PACKET_HZ * 60 * 8 / 10), "frames %u",
           r.link.stats().radioFrames);
  }
  section("C1: garbage on the RA4M1's RX: failures stay local, no link loss, no older frame", f0);

  f0 = g_fail;
  {
    g_esp = EspState();
    initPad();
    Sim s;
    s.seed = 12;
    s.fault = 2;
    Run r(s);
    runFor(r, 10);
    report("C2 restart", s, r.link, 10);
    EXPECT(s.restarts == 1, "%u restarts", s.restarts);
    EXPECT(s.losses == 1, "%d losses (one for the restart)", s.losses);
    EXPECT(r.link.connected(), "not reconnected after the restart");
    EXPECT(r.link.status().bootId == g_esp.bootId, "bootId %u, ESP32 says %u", r.link.status().bootId, g_esp.bootId);
    EXPECT(r.link.stats().espSilences == 1, "%u silences", r.link.stats().espSilences);
    EXPECT(s.older == 0, "%d older frames", s.older);
    // back within the silence + LINK_TIMEOUT_MS + a ping cycle + CAPS + DSS
    if (!s.lossAt.empty()) {
      EXPECT(s.lossAt[0] >= 5e6 && s.lossAt[0] <= 5e6 + LINK_TIMEOUT_MS * 1000.0 + 20000, "lost at %.0f us",
             s.lossAt[0]);
    }
  }
  section("C2: ESP32 restart mid-request: link lost once, ping -> caps -> DSS, reconnected with the new bootId", f0);

  f0 = g_fail;
  {
    g_esp = EspState();
    initPad();
    Sim s;
    s.seed = 13;
    s.fault = 3;
    Run r(s);
    runFor(r, 60);
    report("C3 late", s, r.link, 60);
    EXPECT(s.older == 0, "%d older frames", s.older);
    EXPECT(s.losses == 0, "%d losses", s.losses);
    EXPECT(r.link.stats().pollTimeouts > 0, "the stalls never caused a timeout (fault not injected?)");
  }
  section("C3: AT task stalls 30 ms past the deadline: never an older frame, no link loss", f0);

  f0 = g_fail;
  {
    g_esp = EspState();
    initPad();
    Sim s;
    s.seed = 14;
    s.fault = 4;
    Run r(s);
    runFor(r, 6);
    report("C4 stale scan", s, r.link, 6);
    EXPECT(s.older == 0, "%d older frames", s.older);
    EXPECT(s.losses == 0, "%d losses", s.losses);
  }
  section("C4: a stale 231-byte +XBOXSCAN full of CR/LF/'+'/'|': no frame taken from it, no link loss", f0);

  f0 = g_fail;
  {
    g_esp = EspState();
    g_esp.dsFirmware = false;
    initPad();
    g_esp.st.state = XBOX_R4_OFF;
    Sim s;
    s.seed = 15;
    Run r(s);
    runFor(r, 5);
    report("C5 legacy", s, r.link, 5);
    EXPECT(r.link.atPhase() == ControlLink::AtPhase::RUN_LEGACY, "phase %s", ControlLink::phaseName(r.link.atPhase()));
    EXPECT(r.link.status().espLegacy, "not legacy");
    EXPECT(r.link.connected(), "XBOXBEGIN did not bring the pad up");
    EXPECT(r.link.stats().beginsSent == 1, "%u XBOXBEGIN", r.link.stats().beginsSent);
    EXPECT(r.link.stats().dssSent == 0, "DSS sent to legacy firmware");
    EXPECT(g_esp.dssArgs.empty() && g_esp.dst.empty(), "the ESP32 saw D-Sentry commands");
  }
  section("C5: legacy firmware (bare ERROR to AT+DSCAPS?): BT only, AT+XBOXBEGIN when OFF, no DSS/DST", f0);
}

// ---- drift guards ----------------------------------------------------------------------------
static const char* const kFirmwareStateCalls[] = {
    "srv.write_response_prompt();",
    "srv.write_str(String((int)XBOX_R4_STATE_LEN).c_str());",
    "srv.write_str(\"|\");",
    "srv.write_vec8(state);",
    "srv.write_line_end();",
};

static std::string squeeze(const std::string& code) {
  std::string out;
  bool inString = false;
  for (size_t i = 0; i < code.size(); i++) {
    char c = code[i];
    if (c == '"' && (i == 0 || code[i - 1] != '\\')) inString = !inString;
    if (!inString && (c == ' ' || c == '\t' || c == '\r' || c == '\n')) continue;
    out += c;
  }
  return out;
}

static std::string readFile(const std::string& path) {
  std::ifstream f(path);
  if (!f.good()) return "";
  std::stringstream ss;
  ss << f.rdbuf();
  return ss.str();
}

static void driftGuards(const std::string& root) {
  int f0 = g_fail;
  const std::string xbox = readFile(root + "/firmware/UNOR4USBBridge/cmds_xbox.h");
  if (xbox.empty()) {
    std::printf("info cmds_xbox.h not found: +XBOXSTATE guard skipped\n");
  } else {
    size_t start = xbox.find("command_table[XBOX_R4_AT_STATE]");
    EXPECT(start != std::string::npos, "no command_table[XBOX_R4_AT_STATE] in cmds_xbox.h");
    if (start != std::string::npos) {
      size_t end = xbox.find("command_table[", start + 1);
      const std::string h = xbox.substr(start, end == std::string::npos ? std::string::npos : end - start);
      size_t readAt = h.find("case chAT::CommandMode::Read:");
      size_t defAt = h.find("default:", readAt == std::string::npos ? 0 : readAt);
      EXPECT(readAt != std::string::npos && defAt != std::string::npos, "no Read branch");
      if (readAt != std::string::npos && defAt != std::string::npos) {
        const std::string branch = h.substr(readAt, defAt - readAt);
        std::vector<std::string> calls;
        for (size_t p = branch.find("srv."); p != std::string::npos; p = branch.find("srv.", p)) {
          size_t semi = branch.find(';', p);
          if (semi == std::string::npos) break;
          calls.push_back(squeeze(branch.substr(p, semi - p + 1)));
          p = semi + 1;
        }
        const size_t n = sizeof kFirmwareStateCalls / sizeof kFirmwareStateCalls[0];
        EXPECT(calls.size() == n, "+XBOXSTATE makes %zu srv calls, this test copies %zu", calls.size(), n);
        for (size_t i = 0; i < calls.size() && i < n; i++)
          EXPECT(calls[i] == squeeze(kFirmwareStateCalls[i]), "srv call %zu is now %s", i, calls[i].c_str());
      }
    }
  }
  const std::string ds = readFile(root + "/firmware/UNOR4USBBridge/cmds_dsentry.h");
  if (ds.empty()) {
    std::printf("info cmds_dsentry.h not found: D-Sentry command guard skipped\n");
  } else {
    for (const char* name : {"command_table[DS_AT_DSCAPS]", "command_table[DS_AT_DSS]", "command_table[DS_AT_DST]"})
      EXPECT(ds.find(name) != std::string::npos, "cmds_dsentry.h no longer registers %s", name);
  }
  section("drift: cmds_xbox.h +XBOXSTATE makes the srv calls copied here; cmds_dsentry.h registers DSCAPS/DSS/DST", f0);
}

int main(int argc, char** argv) {
  const std::string root = argc > 1 ? argv[1] : "../..";
  driftGuards(root);
  partA();
  partB();
  partC();
  std::printf("info test_controllink_chat: %s (%d failures)\n", g_fail ? "FAIL" : "PASS", g_fail);
  return g_fail ? 1 : 0;
}

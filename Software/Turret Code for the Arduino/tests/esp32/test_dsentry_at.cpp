// test_dsentry_at.cpp -- the D-Sentry AT commands end to end on the host, with real code at
// every step that can run on a Mac:
//   ESP32 side: chAT parser.cpp + server.cpp from firmware/UNOR4USBBridge (AGPL, compiled from
//               the firmware tree), and the firmware's own handler bodies DsentryAt::caps / dss /
//               dst / softResetWifi (dsentry_at.cpp) and dsentry_wire.cpp, unchanged;
//   RA4M1 side: the real AsyncAt reply parser (firmware-ra4m1/dsentry/src, via ra_side.cpp).
// Only the radios are faked (FakeBackend). The command table and the "tasks after run()" loop
// copy CAtHandler (at_handler.cpp); drift_guard_* tests read the firmware files that cannot be
// compiled on the host and fail if the wiring they rely on changed.
//
// Expected bytes come from docs/protocol.md (generated from protocol_spec.py) and from the stock
// AT+SOFTRESETWIFI handler (cmds_esp_generic.h: prompt, line end, OK).
//
// Safety properties pinned here:
//   - restart_is_queued_once_and_only_after_the_reply: a mode switch never restarts before its
//     "+DSS: 3,..." reply is on the wire, and a burst of requests restarts once;
//   - softresetwifi_is_a_byte_identical_noop_in_dsentry_mode: the RA4M1 library's
//     AT+SOFTRESETWIFI cannot tear down a D-Sentry radio, and its reply bytes do not change.
//
// Usage: test_dsentry_at [-v] [filters]   (REPO_ROOT is compiled in by run.sh)
//
// SPDX-License-Identifier: MIT
#include "testkit.h"

#include <fstream>
#include <functional>
#include <map>
#include <queue>
#include <sstream>
#include <string>
#include <vector>

#include "chAT.hpp"
#include "dsentry_at.h"

#ifndef REPO_ROOT
#error "build with -DREPO_ROOT=\"<repo root>\" (tests/esp32/run.sh does)"
#endif

int ra_parse_reply(const char* prompt, const std::string& bytes, std::string* payload);
int ra_result_ok();
int ra_result_err_bare();

using namespace SudoMaker;

namespace {

const char* const kDstExampleLine =   // docs/protocol.md, AT+DST example (229 bytes)
    "AT+DST=d56b02c840e2010005000101c7683aeeffffd2040000d8dc0000000000002c0100000c0f030071140002fe01"
    "07009426640002000100230007007f2f0041524d454420462020414d4d4f3a313242542020202020436f6e6e"
    "656374656400508000d4fe00feff010002ff031160ca\r\n";
const char* const kStatusExample = "5a 07 01 c8 78 50 00 05 2a 9f";   // docs/protocol.md

std::vector<uint8_t> unhex(const std::string& text) {
  std::vector<uint8_t> out;
  int hi = -1;
  for (char c : text) {
    int v = ds_hex_digit(c);
    if (v < 0) continue;
    if (hi < 0) {
      hi = v;
    } else {
      out.push_back((uint8_t)(hi * 16 + v));
      hi = -1;
    }
  }
  return out;
}

std::string printable(const std::string& s) {
  std::string o;
  for (unsigned char c : s) {
    if (c == '\r') o += "\\r";
    else if (c == '\n') o += "\\n";
    else if (c < 32 || c > 126) { char t[8]; snprintf(t, sizeof t, "\\x%02x", c); o += t; }
    else o += (char)c;
  }
  return o;
}

std::string read_file(const std::string& rel) {
  std::ifstream f(std::string(REPO_ROOT) + "/" + rel, std::ios::binary);
  std::stringstream ss;
  ss << f.rdbuf();
  return ss.str();
}

// The radios, recorded.
struct FakeBackend final : DsentryBackend {
  dsentry::Radios r{};
  uint8_t id = 92;
  uint8_t wstat = DS_WIFI_STATUS_OFF;
  bool active = false;
  bool bleOk = true, wifiOk = true, relayKnown = true;
  int bleStarts = 0, wifiStarts = 0, adopts = 0;
  std::vector<uint8_t> stored, states;
  std::vector<std::vector<uint8_t>> rumbles, statuses, telems;

  dsentry::Radios radios() override { return r; }
  uint8_t bootId() override { return id; }
  uint8_t wifiStatus() override { return wstat; }
  bool dsModeActive() override { return active; }
  bool startBle() override {
    bleStarts++;
    if (!bleOk) return false;
    r.bleRunning = true;
    active = true;
    return true;
  }
  bool startWifi() override {
    wifiStarts++;
    if (!wifiOk) return false;
    r.dsWifiRunning = true;
    active = true;
    wstat = DS_WIFI_STATUS_CONNECTING;
    return true;
  }
  void adoptMode(uint8_t) override {
    adopts++;
    active = true;
  }
  void storeRestartRequest(uint8_t m) override { stored.push_back(m); }
  void rumbleBle(const uint8_t report[XBOX_RUMBLE_REPORT_LEN]) override {
    rumbles.emplace_back(report, report + XBOX_RUMBLE_REPORT_LEN);
  }
  void statusToRelay(const uint8_t* f, size_t n) override { statuses.emplace_back(f, f + n); }
  bool telemetryToRelay(const uint8_t* f, size_t n) override {
    if (!r.dsWifiRunning || !relayKnown) return false;
    telems.emplace_back(f, f + n);
    return true;
  }
  void noteTurretState(uint8_t s) override { states.push_back(s); }
};

typedef std::function<chAT::CommandStatus(chAT::Server&, chAT::ATParser&)> handler_t;

// One chAT server wired like CAtHandler: command table, tasks run after each at_srv.run().
struct Bridge {
  FakeBackend be;
  DsentryAt at{be};
  chAT::Server srv;
  std::map<std::string, handler_t> table;
  std::queue<std::function<void()>> tasks;
  std::string in, out;
  size_t pos = 0, chunk = 1024;
  int restarts = 0, stockSoftReset = 0;
  std::string outWhenRestarted;

  Bridge() {
    srv.set_io_callback({
        .callback_io_read = [this](uint8_t* buf, size_t len) -> ssize_t {
          size_t n = std::min(std::min(len, chunk), in.size() - pos);
          memcpy(buf, in.data() + pos, n);
          pos += n;
          return (ssize_t)n;
        },
        .callback_io_write = [this](uint8_t* buf, size_t len) -> ssize_t {
          out.append((const char*)buf, len);
          return (ssize_t)len;
        },
    });
    srv.set_command_callback([this](chAT::Server& s, const std::string& command) {
      auto it = table.find(command);
      if (it == table.end()) return chAT::CommandStatus::ERROR;
      return it->second(s, s.parser());
    });
    // As cmds_dsentry.h registers them.
    table[DS_AT_DSCAPS] = [this](chAT::Server& s, chAT::ATParser& p) { return at.caps(s, p); };
    table[DS_AT_DSS] = [this](chAT::Server& s, chAT::ATParser& p) {
      return at.dss(s, p, [this]() {
        tasks.push([this]() {   // stands in for serial->flush(); dsentry_restart_now();
          restarts++;
          outWhenRestarted = out;
        });
      });
    };
    table[DS_AT_DST] = [this](chAT::Server& s, chAT::ATParser& p) { return at.dst(s, p); };
    // The stock AT+SOFTRESETWIFI Run branch with the D-Sentry guard in front (cmds_esp_generic.h).
    table["+SOFTRESETWIFI"] = [this](chAT::Server& s, chAT::ATParser& p) {
      switch (p.cmd_mode) {
        case chAT::CommandMode::Run: {
          if (at.softResetWifi(s)) return chAT::CommandStatus::OK;
          stockSoftReset++;   // the stock body: tear down sockets and WiFi, then
          s.write_response_prompt();
          s.write_line_end();
          return chAT::CommandStatus::OK;
        }
        default:
          return chAT::CommandStatus::ERROR;
      }
    };
  }

  // CAtHandler::run(): at_srv.run(), then the queued tasks.
  void run_once() {
    srv.run();
    size_t n = tasks.size();
    for (size_t i = 0; i < n; i++) {
      auto t = tasks.front();
      tasks.pop();
      t();
    }
  }

  std::string transact(const std::string& request, size_t readChunk = 1024) {
    in = request;
    pos = 0;
    out.clear();
    chunk = readChunk;
    for (size_t i = 0; i < request.size() + 4; i++) run_once();
    return out;
  }
};

bool expect_bytes(const std::string& got, const std::string& want) {
  bool ok = CHECK(got == want);
  if (!ok) std::printf("      got  %s\n      want %s\n", printable(got).c_str(), printable(want).c_str());
  return ok;
}

}  // namespace

// ---------------------------------------------------------------------------------------------

TEST(dscaps_answers_the_documented_bytes) {
  Bridge b;
  expect_bytes(b.transact("AT+DSCAPS?\r\n"), "+DSCAPS: 1,15\r\nOK\r\n");
  expect_bytes(b.transact("AT+DSCAPS?\r\n", 1), "+DSCAPS: 1,15\r\nOK\r\n");   // byte by byte
}

TEST(dss_docs_example) {
  Bridge b;
  b.be.r.dsWifiRunning = true;
  b.be.active = true;
  b.be.wstat = DS_WIFI_STATUS_RELAY_HEARD;
  expect_bytes(b.transact("AT+DSS=1,42,200,120,80,5\r\n"), "+DSS: 1,92,4\r\nOK\r\n");
  CHECK_EQ(b.be.states.size(), 1u);
  CHECK_EQ(b.be.states.at(0), 5);
  CHECK_EQ(b.be.wifiStarts, 0);   // already running: nothing restarted
  CHECK_EQ(b.be.adopts, 1);
  CHECK(b.be.statuses.empty());   // the first AT+DSS only sets the rumble baseline
  CHECK_EQ(b.restarts, 0);
}

TEST(dst_docs_example_forwards_the_exact_frame) {
  for (size_t chunk : {(size_t)1024, (size_t)64, (size_t)1}) {
    Bridge b;
    b.be.r.dsWifiRunning = true;
    CHECK_EQ(strlen(kDstExampleLine), (size_t)DS_AT_DST_LINE_LEN);
    expect_bytes(b.transact(kDstExampleLine, chunk), "+DST: 1\r\nOK\r\n");
    CHECK_EQ(b.be.telems.size(), 1u);
    std::string line = kDstExampleLine;
    const std::vector<uint8_t> want = unhex(line.substr(strlen("AT+DST=")));
    if (CHECK_EQ(b.be.telems.size(), 1u)) CHECK(b.be.telems[0] == want);
    CHECK_EQ(want.size(), (size_t)DS_TELEMETRY_FRAME_LEN);
  }
}

TEST(dst_not_forwarded_says_0) {
  Bridge b;   // BLE mode: no WiFi
  b.be.r.bleRunning = true;
  expect_bytes(b.transact(kDstExampleLine), "+DST: 0\r\nOK\r\n");
  b.be.r = dsentry::Radios{};
  b.be.r.dsWifiRunning = true;
  b.be.relayKnown = false;        // WiFi up, relay not heard yet
  expect_bytes(b.transact(kDstExampleLine), "+DST: 0\r\nOK\r\n");
  b.be.relayKnown = true;
  std::string bad = kDstExampleLine;
  bad[20] = 'x';                  // not hex
  expect_bytes(b.transact(bad), "+DST: 0\r\nOK\r\n");
  std::string flipped = kDstExampleLine;
  flipped[flipped.size() - 4] = flipped[flipped.size() - 4] == '0' ? '1' : '0';   // bad CRC
  expect_bytes(b.transact(flipped), "+DST: 0\r\nOK\r\n");
  expect_bytes(b.transact("AT+DST=d5,5b\r\n"), "+DST: 0\r\nOK\r\n");          // two args
  expect_bytes(b.transact("AT+DST=\r\n"), "+DST: 0\r\nOK\r\n");
  CHECK(b.be.telems.empty());
  CHECK_EQ(b.at.malformedDst(), 4u);
}

TEST(other_command_modes_get_the_bare_error) {
  Bridge b;
  for (const char* req : {"AT+DSCAPS\r\n", "AT+DSCAPS=1\r\n", "AT+DSCAPS=?\r\n", "AT+DSS?\r\n", "AT+DSS\r\n",
                          "AT+DSS=?\r\n", "AT+DST?\r\n", "AT+DST\r\n", "AT+DSX?\r\n"}) {
    if (!expect_bytes(b.transact(req), "ERROR\r\n")) std::printf("      for %s\n", printable(req).c_str());
  }
  CHECK_EQ(b.be.bleStarts + b.be.wifiStarts, 0);
}

TEST(dss_starts_the_wanted_radio_in_place_once) {
  Bridge b;
  expect_bytes(b.transact("AT+DSS=0,0,0,0,0,3\r\n"), "+DSS: 0,92,0\r\nOK\r\n");
  CHECK_EQ(b.be.bleStarts, 1);
  expect_bytes(b.transact("AT+DSS=0,0,0,0,0,3\r\n"), "+DSS: 0,92,0\r\nOK\r\n");   // heartbeat
  CHECK_EQ(b.be.bleStarts, 1);
  CHECK_EQ(b.restarts, 0);
  Bridge w;
  expect_bytes(w.transact("AT+DSS=1,0,0,0,0,3\r\n"), "+DSS: 1,92,1\r\nOK\r\n");   // connecting
  CHECK_EQ(w.be.wifiStarts, 1);
  w.transact("AT+DSS=1,0,0,0,0,3\r\n");
  CHECK_EQ(w.be.wifiStarts, 1);
  CHECK_EQ(w.restarts, 0);
}

TEST(restart_is_queued_once_and_only_after_the_reply) {
  // SAFETY PROPERTY: switching radios restarts the ESP32 exactly once, after the reply is out.
  Bridge b;
  b.be.r.bleRunning = true;
  b.be.active = true;
  const std::string reply = b.transact("AT+DSS=1,0,0,0,0,4\r\n");
  expect_bytes(reply, "+DSS: 3,92,0\r\nOK\r\n");
  CHECK_EQ(b.restarts, 1);
  expect_bytes(b.outWhenRestarted, "+DSS: 3,92,0\r\nOK\r\n");   // complete before the restart ran
  CHECK(b.be.stored == std::vector<uint8_t>({DS_ESP_MODE_WIFI}));
  CHECK_EQ(b.be.wifiStarts, 0);   // never started next to BLE
  // More requests before the restart lands: the latest wanted mode is stored, no second restart.
  expect_bytes(b.transact("AT+DSS=0,1,200,120,80,4\r\n"), "+DSS: 3,92,0\r\nOK\r\n");
  expect_bytes(b.transact("AT+DSS=1,2,200,120,80,4\r\n"), "+DSS: 3,92,0\r\nOK\r\n");
  CHECK_EQ(b.restarts, 1);
  CHECK(b.be.stored == std::vector<uint8_t>({DS_ESP_MODE_WIFI, DS_ESP_MODE_BLE, DS_ESP_MODE_WIFI}));
  CHECK(b.be.rumbles.empty());    // nothing is played while the radio is going away
  CHECK(b.be.statuses.empty());
  CHECK_EQ(b.be.bleStarts + b.be.wifiStarts, 0);
  // WiFi -> BLE the same way.
  Bridge w;
  w.be.r.dsWifiRunning = true;
  expect_bytes(w.transact("AT+DSS=0,0,0,0,0,4\r\n"), "+DSS: 3,92,0\r\nOK\r\n");
  CHECK(w.be.stored == std::vector<uint8_t>({DS_ESP_MODE_BLE}));
  CHECK_EQ(w.restarts, 1);
  CHECK_EQ(w.be.bleStarts, 0);
}

TEST(stock_radio_users_force_a_restart) {
  Bridge h;
  h.be.r.hciUsed = true;           // a previous ArduinoBLE sketch took the controller
  expect_bytes(h.transact("AT+DSS=0,0,0,0,0,3\r\n"), "+DSS: 3,92,0\r\nOK\r\n");
  CHECK_EQ(h.be.bleStarts, 0);
  Bridge s;
  s.be.r.otherWifiRunning = true;  // the stock WiFiS3 STA (modem.begin()'s SOFTRESETWIFI)
  expect_bytes(s.transact("AT+DSS=0,0,0,0,0,3\r\n"), "+DSS: 3,92,0\r\nOK\r\n");
  Bridge t;
  t.be.r.otherWifiRunning = true;
  expect_bytes(t.transact("AT+DSS=1,0,0,0,0,3\r\n"), "+DSS: 3,92,0\r\nOK\r\n");
  CHECK_EQ(t.be.wifiStarts, 0);
}

TEST(failed_start_reports_none_and_retries) {
  Bridge b;
  b.be.bleOk = false;
  expect_bytes(b.transact("AT+DSS=0,7,0,0,0,3\r\n"), "+DSS: 2,92,0\r\nOK\r\n");
  expect_bytes(b.transact("AT+DSS=0,8,200,120,80,5\r\n"), "+DSS: 2,92,0\r\nOK\r\n");
  CHECK_EQ(b.be.bleStarts, 2);     // every AT+DSS tries again
  CHECK(b.be.rumbles.empty());     // no radio, no rumble
  CHECK(b.be.statuses.empty());
}

TEST(ble_rumble_only_on_a_new_rumble_seq) {
  Bridge b;
  b.be.r.bleRunning = true;
  b.transact("AT+DSS=0,5,200,120,80,5\r\n");   // baseline (old shot repeated after a reboot)
  b.transact("AT+DSS=0,5,200,120,80,5\r\n");   // heartbeats
  b.transact("AT+DSS=0,5,200,120,80,5\r\n");
  CHECK(b.be.rumbles.empty());
  b.transact("AT+DSS=0,6,200,120,80,5\r\n");
  if (CHECK_EQ(b.be.rumbles.size(), 1u)) {
    CHECK(b.be.rumbles[0] == std::vector<uint8_t>({0x03, 0, 0, 78, 47, 8, 0, 0}));
  }
  b.transact("AT+DSS=0,6,200,120,80,5\r\n");
  b.transact("AT+DSS=0,7,0,0,0,5\r\n");        // rumbleMs 0: stop
  if (CHECK_EQ(b.be.rumbles.size(), 2u)) {
    CHECK(b.be.rumbles[1] == std::vector<uint8_t>({0x03, 0, 0, 0, 0, 0, 0, 0}));
  }
  CHECK(b.be.statuses.empty());
}

TEST(wifi_rumble_goes_to_the_relay_as_the_docs_status_packet) {
  Bridge b;
  b.be.r.dsWifiRunning = true;
  b.transact("AT+DSS=1,41,0,0,0,5\r\n");       // baseline
  CHECK(b.be.statuses.empty());
  b.transact("AT+DSS=1,42,200,120,80,5\r\n");
  if (CHECK_EQ(b.be.statuses.size(), 1u)) CHECK(b.be.statuses[0] == unhex(kStatusExample));
  b.transact("AT+DSS=1,42,200,120,80,5\r\n");  // heartbeat repeat: not a new shot
  CHECK_EQ(b.be.statuses.size(), 1u);
  CHECK(b.be.rumbles.empty());
}

TEST(malformed_dss_changes_nothing_and_reports_the_state) {
  Bridge b;
  for (const char* req : {"AT+DSS=1,2,3\r\n", "AT+DSS=1,42,256,0,0,0\r\n", "AT+DSS=2,0,0,0,0,0\r\n",
                          "AT+DSS=1,0,0,0,65536,0\r\n", "AT+DSS=1,,0,0,0,0\r\n", "AT+DSS=1,0,0,0,0,0,9\r\n",
                          "AT+DSS=x,0,0,0,0,0\r\n"}) {
    if (!expect_bytes(b.transact(req), "+DSS: 2,92,0\r\nOK\r\n")) std::printf("      for %s\n", printable(req).c_str());
  }
  CHECK_EQ(b.at.malformedDss(), 7u);
  CHECK_EQ(b.be.bleStarts + b.be.wifiStarts + b.restarts, 0);
  CHECK(b.be.states.empty());
  // chAT strips quotes around arguments, so a quoted request is the same request.
  expect_bytes(b.transact("AT+DSS=\"0\",\"1\",\"2\",\"3\",\"4\",\"5\"\r\n"), "+DSS: 0,92,0\r\nOK\r\n");
  CHECK_EQ(b.be.bleStarts, 1);
}

TEST(softresetwifi_is_a_byte_identical_noop_in_dsentry_mode) {
  // SAFETY PROPERTY: modem.begin() on the RA4M1 sends AT+SOFTRESETWIFI; in a D-Sentry mode it
  // must not run, and the RA4M1 must see the stock reply.
  Bridge stock;
  const std::string stockReply = stock.transact("AT+SOFTRESETWIFI\r\n");
  expect_bytes(stockReply, "+SOFTRESETWIFI: \r\nOK\r\n");
  CHECK_EQ(stock.stockSoftReset, 1);
  Bridge ds;
  ds.transact("AT+DSS=1,0,0,0,0,3\r\n");          // WiFi mode started
  CHECK(ds.be.active);
  expect_bytes(ds.transact("AT+SOFTRESETWIFI\r\n"), stockReply);
  CHECK_EQ(ds.stockSoftReset, 0);
  Bridge adopted;
  adopted.be.r.bleRunning = true;                  // started by AT+XBOXBEGIN, then adopted by DSS
  adopted.transact("AT+SOFTRESETWIFI\r\n");
  CHECK_EQ(adopted.stockSoftReset, 1);            // no D-Sentry command yet: stock behaviour
  adopted.transact("AT+DSS=0,0,0,0,0,3\r\n");
  adopted.transact("AT+SOFTRESETWIFI\r\n");
  CHECK_EQ(adopted.stockSoftReset, 1);
}

TEST(pipelined_requests_answer_in_order) {
  Bridge b;
  b.be.r.dsWifiRunning = true;
  const std::string all = std::string("AT+DSCAPS?\r\nAT+DSS=1,0,0,0,0,4\r\n") + kDstExampleLine + "AT\r\n";
  expect_bytes(b.transact(all, 7), "+DSCAPS: 1,15\r\nOK\r\n+DSS: 1,92,0\r\nOK\r\n+DST: 1\r\nOK\r\nOK\r\n");
}

TEST(ra4m1_asyncat_parses_every_reply) {
  Bridge b;
  b.be.r.dsWifiRunning = true;
  b.be.wstat = DS_WIFI_STATUS_RELAY_HEARD;
  std::string payload;
  CHECK_EQ(ra_parse_reply(DS_AT_DSCAPS, b.transact("AT+DSCAPS?\r\n"), &payload), ra_result_ok());
  CHECK_EQ(payload, std::string("1,15"));
  CHECK_EQ(ra_parse_reply(DS_AT_DSS, b.transact("AT+DSS=1,42,200,120,80,5\r\n"), &payload), ra_result_ok());
  CHECK_EQ(payload, std::string("1,92,4"));
  CHECK_EQ(ra_parse_reply(DS_AT_DST, b.transact(kDstExampleLine), &payload), ra_result_ok());
  CHECK_EQ(payload, std::string("1"));
  // A stock bridge's bare ERROR is what marks the ESP32 as legacy firmware.
  CHECK_EQ(ra_parse_reply(DS_AT_DSCAPS, b.transact("AT+DSCAPS\r\n"), &payload), ra_result_err_bare());
}

// ---- drift guards on the firmware files the host cannot compile ----------------------------

namespace {
size_t find_after(const std::string& text, const std::string& what, size_t from) {
  size_t p = text.find(what, from);
  return p;
}
}  // namespace

TEST(drift_guard_cmds_dsentry_registers_these_methods) {
  const std::string s = read_file("firmware/UNOR4USBBridge/cmds_dsentry.h");
  CHECK(!s.empty());
  size_t p = s.find("command_table[DS_AT_DSCAPS]");
  CHECK(p != std::string::npos && s.find("dsentry_at().caps(srv, parser)", p) != std::string::npos);
  p = s.find("command_table[DS_AT_DSS]");
  CHECK(p != std::string::npos && s.find("dsentry_at().dss(srv, parser,", p) != std::string::npos);
  size_t task = s.find("addTask(", p);
  size_t flush = s.find("serial->flush();", task);
  size_t restart = s.find("dsentry_restart_now();", flush);
  CHECK(task != std::string::npos && flush != std::string::npos && restart != std::string::npos);
  p = s.find("command_table[DS_AT_DST]");
  CHECK(p != std::string::npos && s.find("dsentry_at().dst(srv, parser)", p) != std::string::npos);
  const std::string h = read_file("firmware/UNOR4USBBridge/at_handler.cpp");
  CHECK(h.find("#include \"cmds_dsentry.h\"") != std::string::npos);
  CHECK(h.find("add_cmds_dsentry();") != std::string::npos);
}

TEST(drift_guard_softresetwifi_guard_comes_first) {
  const std::string s = read_file("firmware/UNOR4USBBridge/cmds_esp_generic.h");
  size_t cmd = s.find("command_table[_SOFTRESETWIFI]");
  size_t run = find_after(s, "case chAT::CommandMode::Run:", cmd);
  size_t guard = find_after(s, "if (dsentry_at().softResetWifi(srv)) {", run);
  size_t firstStock = find_after(s, "serverClients[i].client.stop();", run);
  size_t tail = find_after(s, "WiFi.softAPdisconnect();", run);
  CHECK(cmd != std::string::npos && run != std::string::npos && guard != std::string::npos);
  CHECK(guard < firstStock);
  // The stock tail still writes exactly what the test's copy writes.
  size_t prompt = find_after(s, "srv.write_response_prompt();", tail);
  size_t end = find_after(s, "srv.write_line_end();", prompt);
  size_t ok = find_after(s, "return chAT::CommandStatus::OK;", end);
  CHECK(tail != std::string::npos && prompt != std::string::npos && end != std::string::npos &&
        ok != std::string::npos);
}

TEST(drift_guard_client_task_and_state_source) {
  const std::string s = read_file("firmware/UNOR4USBBridge/xbox_client.cpp");
  size_t onloop = s.find("g_core->onLoop();");
  size_t service = s.find("service_rumble();", onloop);
  size_t nextStmt = s.find("uint8_t st = g_core->connectionStateValue();", onloop);
  CHECK(onloop != std::string::npos && service != std::string::npos && service < nextStmt);
  size_t fill = s.find("void xbox_fill_state(uint8_t out[XBOX_R4_STATE_LEN]) {");
  size_t sw = s.find("if (dsentry_fill_xbox_state(out)) return;", fill);
  size_t firstLocal = s.find("xbox_r4_state_t s;", fill);
  CHECK(fill != std::string::npos && sw != std::string::npos && sw < firstLocal);
  size_t begin = s.find("int xbox_begin(void) {");
  size_t guard = s.find("if (dsentry_wifi_mode_active()) return XBOX_R4_RC_BLOCKED_BY_HCI;", begin);
  size_t init = s.find("NimBLEDevice::init(", begin);
  CHECK(begin != std::string::npos && guard != std::string::npos && guard < init);
  size_t freeFn = s.find("void free_disconnected_clients() {");
  CHECK(s.find("reset_out_report_cache();", freeFn) < s.find("NimBLEDevice::deleteClient(c)", freeFn));
}

TEST(drift_guard_boot_order_in_setup) {
  const std::string s = read_file("firmware/UNOR4USBBridge/UNOR4USBBridge.ino");
  size_t w = s.find("digitalWrite(GPIO_RST, HIGH);");
  size_t m = s.find("pinMode(GPIO_RST, OUTPUT);");
  CHECK(w != std::string::npos && m != std::string::npos && w < m);   // no LOW glitch on RES#
  size_t boot = s.find("dsentry_boot();");
  size_t atTask = s.find("xTaskCreatePinnedToCore(");
  CHECK(boot != std::string::npos && atTask != std::string::npos && boot < atTask);
}

TESTKIT_MAIN()

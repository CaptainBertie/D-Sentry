/*
 * test_chat_e2e.cpp -- end-to-end host test of the AT link with the REAL code at both ends:
 *   ESP32 side: chAT parser.cpp + server.cpp from firmware/UNOR4USBBridge (SudoMaker chAT,
 *               AGPL-3.0-or-later; compiled from the firmware tree, not copied);
 *   RA4M1 side: WiFiS3 Modem.cpp from the installed core arduino:renesas_uno 1.6.0.
 * The two are joined by the stub UART: whatever modem.write() sends is run through the chAT
 * server, and the server's output becomes the modem's rx.
 *
 * The +XBOXSTATE handler below is a copy of the Read branch in firmware/UNOR4USBBridge/
 * cmds_xbox.h (modelled on +PREFGET in cmds_preferences.h:321-331). The real handler cannot be
 * compiled on the host (it needs the ESP32 build), so test_firmware_handler_unchanged() reads
 * cmds_xbox.h and fails if its srv calls no longer match the ones copied here. The command
 * callback is a copy of the dispatch in at_handler.cpp (unknown command -> ERROR).
 *
 * Usage: test_chat_e2e <repo root>   (the root is needed to read cmds_xbox.h)
 *
 * Adapted from the recon harnesses scratchpad/chatsim/sim.cpp and e2e_esp.cpp.
 *
 * SPDX-License-Identifier: MIT
 */
#include "../firmware/UNOR4USBBridge/chAT.hpp"
#include "Modem.h"
#include "../protocol/xbox_r4_protocol.h"

#include <chrono>
#include <cstdio>
#include <cstring>
#include <fstream>
#include <map>
#include <sstream>
#include <string>
#include <vector>

/* ---- tiny assert macro: count failures, print each one, keep going ---- */
static int g_failures = 0;
#define EXPECT(cond, ...)                                              \
  do {                                                                 \
    if (!(cond)) {                                                     \
      g_failures++;                                                    \
      std::printf("FAIL %s:%d: %s: ", __FILE__, __LINE__, #cond);      \
      std::printf(__VA_ARGS__);                                        \
      std::printf("\n");                                               \
    }                                                                  \
  } while (0)
static void section_done(const char* name, int failures_before) {
  if (g_failures == failures_before) std::printf("ok   %s\n", name);
}

using namespace SudoMaker;

/* The real board's modem global is bound to Serial2 (Modem.cpp:423-424). */
UART Serial2;

/* The state the ESP32 would publish; the handler encodes it on every poll. */
static xbox_r4_state_t g_state;

typedef std::function<chAT::CommandStatus(chAT::Server&, chAT::ATParser&)> handler_t;

/* One chAT server with a scripted input stream and a captured output stream. */
struct Bridge {
  chAT::Server srv;
  std::map<std::string, handler_t> command_table;
  std::string in, out;
  size_t pos = 0;
  size_t chunk = 1024;  /* how many bytes one io_read may return */

  explicit Bridge(bool with_xbox) {
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
    /* same dispatch as CAtHandler's constructor in at_handler.cpp */
    srv.set_command_callback([this](chAT::Server& s, const std::string& command) {
      auto it = command_table.find(command);
      if (it == command_table.end()) {
        return chAT::CommandStatus::ERROR;
      }
      return it->second(s, s.parser());
    });
    if (with_xbox) add_xbox_state();
  }

  /* What the firmware's +XBOXSTATE handler does (cmds_xbox.h; kFirmwareStateCalls below).
   * std::to_string stands in for Arduino's String(int); both give the decimal digits. */
  void add_xbox_state() {
    command_table[XBOX_R4_AT_STATE] = [](chAT::Server& srv, chAT::ATParser& parser) {
      switch (parser.cmd_mode) {
        case chAT::CommandMode::Read: {
          std::vector<uint8_t> state(XBOX_R4_STATE_LEN);
          xbox_r4_encode(&g_state, state.data());
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
  }

  /* Feed one request, run the server until it has consumed it, return what it wrote. */
  std::string transact(const std::string& request, size_t read_chunk = 1024) {
    in = request;
    pos = 0;
    out.clear();
    chunk = read_chunk;
    for (size_t i = 0; i < request.size() + 4; i++) srv.run();
    return out;
  }
};

static std::string hexdump(const std::string& b) {
  std::string out;
  char tmp[4];
  for (unsigned char c : b) {
    snprintf(tmp, sizeof tmp, "%02X ", c);
    out += tmp;
  }
  if (!out.empty()) out.pop_back();
  return out;
}

static std::string payload_of(const xbox_r4_state_t& s) {
  uint8_t out[XBOX_R4_STATE_LEN];
  xbox_r4_encode(&s, out);
  return std::string((const char*)out, sizeof out);
}

/* The 41 bytes docs/protocol.md shows, built from its hex dump. */
static std::string documented_reply(const std::string& payload) {
  const std::string head("\x2B\x58\x42\x4F\x58\x53\x54\x41\x54\x45\x3A\x20\x32\x30\x7C", 15);
  const std::string tail("\x0D\x0A\x4F\x4B\x0D\x0A", 6);
  return head + payload + tail;
}

static void set_tricky_state() {
  const uint8_t rep[16] = {0x0D, 0x0A, 0x7C, 'O', 'K', 0x00, 0xFF, 0x0D,
                           0x0A, 0x7C, 'O', 'K', 0x0D, 0x0A, 0x00, 0xFF};
  memcpy(g_state.report, rep, 16);
  g_state.state = XBOX_R4_CONNECTED;
  g_state.battery = 64;
  g_state.seq = 0x7C;  /* '|' again, in the seq byte */
}

static void test_server_output() {
  int f0 = g_failures;
  set_tricky_state();
  Bridge b(true);
  const std::string payload = payload_of(g_state);
  const std::string want = documented_reply(payload);
  std::string got = b.transact("AT+XBOXSTATE?\r\n");
  EXPECT(got.size() == 41, "server wrote %zu bytes, docs say 41", got.size());
  EXPECT(got == want, "server wrote %s", hexdump(got).c_str());
  std::printf("info ESP32 reply to AT+XBOXSTATE?: %s\n", hexdump(got).c_str());
  section_done("chAT answers AT+XBOXSTATE? with exactly the 41 bytes of docs/protocol.md", f0);

  /* the ESP32's UART can hand the request to chAT in pieces; one byte at a time is the worst */
  f0 = g_failures;
  Bridge b1(true);
  got = b1.transact("AT+XBOXSTATE?\r\n", 1);
  EXPECT(got == want, "request fed byte by byte gave %s", hexdump(got).c_str());
  section_done("same 41 bytes when the request arrives one byte per read", f0);
}

static void test_errors_from_server() {
  int f0 = g_failures;
  Bridge b(true);
  std::string got = b.transact("AT+XBOXSTATE\r\n");
  EXPECT(got == "ERROR\r\n", "Run form gave %s", hexdump(got).c_str());
  got = b.transact("AT+XBOXSTATE=?\r\n");
  EXPECT(got == "ERROR\r\n", "Test form gave %s", hexdump(got).c_str());
  got = b.transact("AT+NOPE?\r\n");
  EXPECT(got == "ERROR\r\n", "unknown command gave %s", hexdump(got).c_str());
  section_done("Run form AT+XBOXSTATE, Test form and unknown AT+NOPE? all get ERROR\\r\\n", f0);

  /* A stock bridge has no +XBOXSTATE at all. */
  f0 = g_failures;
  Bridge stock(false);
  got = stock.transact("AT+XBOXSTATE?\r\n");
  EXPECT(got == "ERROR\r\n", "stock bridge gave %s", hexdump(got).c_str());
  section_done("a stock bridge (no Xbox commands) answers AT+XBOXSTATE? with ERROR\\r\\n", f0);
}

struct PollResult {
  bool ok;
  std::string res;
  double ms;
};

/* The RA4M1 poll, wired to a bridge through the stub UART. */
static PollResult poll_through(Bridge& bridge) {
  Serial2.reset();
  Serial2.on_write = [&bridge](const uint8_t* b, size_t n) {
    Serial2.feed(bridge.transact(std::string((const char*)b, n)));
  };
  while (Serial2.available()) Serial2.read();  /* drain, docs/protocol.md rule 4 */
  PollResult r;
  modem.timeout(50);
  modem.avoid_trim_results();  /* as XboxControllerR4::poll() does */
  auto t0 = std::chrono::steady_clock::now();
  r.ok = modem.write(std::string(XBOX_R4_AT_STATE ":"), r.res, "%s", "AT" XBOX_R4_AT_STATE "?\r\n");
  auto t1 = std::chrono::steady_clock::now();
  modem.timeout(MODEM_TIMEOUT);
  r.ms = std::chrono::duration<double, std::milli>(t1 - t0).count();
  return r;
}

static void test_round_trip_through_both() {
  int f0 = g_failures;
  set_tricky_state();
  Bridge b(true);
  PollResult p = poll_through(b);
  xbox_r4_state_t got;
  memset(&got, 0, sizeof got);
  bool decoded = p.res.size() == XBOX_R4_STATE_LEN &&
                 xbox_r4_decode((const uint8_t*)p.res.data(), p.res.size(), &got);
  EXPECT(p.ok, "modem.write() returned false");
  EXPECT(p.res == payload_of(g_state), "RA got %s", hexdump(p.res).c_str());
  EXPECT(decoded, "RA could not decode the payload");
  EXPECT(memcmp(got.report, g_state.report, 16) == 0 && got.state == g_state.state &&
             got.battery == g_state.battery && got.seq == g_state.seq,
         "decoded state differs from what the ESP32 published");
  EXPECT(Serial2.tx == "AT+XBOXSTATE?\r\n", "RA sent %s", hexdump(Serial2.tx).c_str());
  std::printf("info RA4M1 Modem.cpp <- chAT: ok=%d len=%zu in %.2f ms\n", p.ok, p.res.size(), p.ms);
  section_done("real Modem.cpp -> real chAT -> real Modem.cpp: same 20 bytes, CRC good, fields equal", f0);

  /* a run of polls with a changing state, as the RA4M1 would see at 50 Hz */
  f0 = g_failures;
  for (int n = 0; n < 64; n++) {
    xbox_r4_neutral_report(g_state.report);
    g_state.report[0] = (uint8_t)(n * 4);
    g_state.report[12] = (uint8_t)(n % 9);
    g_state.report[13] = (uint8_t)(n & 0xDB);
    g_state.state = (uint8_t)(n % 6);
    g_state.battery = (uint8_t)n;
    g_state.seq = (uint8_t)(n * 5);
    PollResult q = poll_through(b);
    xbox_r4_state_t s;
    bool good = q.ok && q.res.size() == 20 && xbox_r4_decode((const uint8_t*)q.res.data(), 20, &s) &&
                s.seq == g_state.seq && memcmp(s.report, g_state.report, 16) == 0;
    EXPECT(good, "poll %d: ok=%d len=%zu", n, q.ok, q.res.size());
  }
  section_done("64 consecutive polls through both real parsers all decode to the published state", f0);
}

static void test_stock_bridge_from_ra_side() {
  int f0 = g_failures;
  Bridge stock(false);
  double ms[3];
  for (int i = 0; i < 3; i++) {
    PollResult p = poll_through(stock);
    ms[i] = p.ms;
    EXPECT(!p.ok, "RA reported success against a stock bridge");
    EXPECT(p.ms >= 50.0, "returned after %.2f ms, before the timeout: has Modem.cpp's result parsing changed?", p.ms);
    EXPECT(p.ms < 1000.0, "took %.2f ms", p.ms);
  }
  double fastest = ms[0];
  for (double m : ms) if (m < fastest) fastest = m;
  EXPECT(fastest <= 60.0, "fastest of 3 took %.2f ms", fastest);
  std::printf("info stock bridge: ESP32 answers ERROR\\r\\n, RA4M1 Modem.cpp sees only a timeout: false after "
              "%.2f / %.2f / %.2f ms (timeout 50 ms). Two causes in Modem.cpp 1.6.0: line 7 defines RESULT_ERROR as "
              "\"OK\\r\\n\", and a bare result code with no prompt sets restart (lines 243-255), so even a "
              "corrected line 7 would still wait for the timeout here\n",
              ms[0], ms[1], ms[2]);
  section_done("stock bridge: the RA side sees ERROR only as the 50 ms timeout (50..60 ms)", f0);
}

/* The srv calls the Read branch of the real +XBOXSTATE handler makes, as they are written in
 * firmware/UNOR4USBBridge/cmds_xbox.h. add_xbox_state() above must do the same. If the
 * firmware changes, update add_xbox_state() first, then this list. */
static const char* const kFirmwareStateCalls[] = {
    "srv.write_response_prompt();",
    "srv.write_str(String((int)XBOX_R4_STATE_LEN).c_str());",
    "srv.write_str(\"|\");",
    "srv.write_vec8(state);",
    "srv.write_line_end();",
};

/* Drops whitespace outside string literals, so a reformatted line still compares equal but a
 * changed literal does not. */
static std::string squeeze(const std::string& code) {
  std::string out;
  bool in_string = false;
  for (size_t i = 0; i < code.size(); i++) {
    char c = code[i];
    if (c == '"' && (i == 0 || code[i - 1] != '\\')) in_string = !in_string;
    if (!in_string && (c == ' ' || c == '\t' || c == '\r' || c == '\n')) continue;
    out += c;
  }
  return out;
}

/* Drift guard for the hand copy in add_xbox_state(): reads the real handler's source. */
static void test_firmware_handler_unchanged(const std::string& root) {
  int f0 = g_failures;
  const std::string path = root + "/firmware/UNOR4USBBridge/cmds_xbox.h";
  std::ifstream f(path);
  if (!f.good()) {
    std::printf("info %s not present yet: firmware handler check skipped\n", path.c_str());
    return;
  }
  std::stringstream ss;
  ss << f.rdbuf();
  const std::string src = ss.str();

  /* the +XBOXSTATE handler runs from its command_table line to the next one */
  size_t start = src.find("command_table[XBOX_R4_AT_STATE]");
  EXPECT(start != std::string::npos, "no command_table[XBOX_R4_AT_STATE] in %s", path.c_str());
  if (start == std::string::npos) return;
  size_t end = src.find("command_table[", start + 1);
  const std::string handler = src.substr(start, end == std::string::npos ? std::string::npos : end - start);

  size_t read_at = handler.find("case chAT::CommandMode::Read:");
  size_t default_at = handler.find("default:", read_at == std::string::npos ? 0 : read_at);
  EXPECT(read_at != std::string::npos && default_at != std::string::npos,
         "cannot find the Read branch of the +XBOXSTATE handler in %s", path.c_str());
  if (read_at == std::string::npos || default_at == std::string::npos) return;
  const std::string branch = handler.substr(read_at, default_at - read_at);

  std::vector<std::string> calls;
  for (size_t p = branch.find("srv."); p != std::string::npos; p = branch.find("srv.", p)) {
    size_t semi = branch.find(';', p);
    if (semi == std::string::npos) break;
    calls.push_back(squeeze(branch.substr(p, semi - p + 1)));
    p = semi + 1;
  }
  const size_t want_n = sizeof kFirmwareStateCalls / sizeof kFirmwareStateCalls[0];
  EXPECT(calls.size() == want_n, "cmds_xbox.h +XBOXSTATE Read branch makes %zu srv calls, this test copies %zu",
         calls.size(), want_n);
  for (size_t i = 0; i < calls.size() && i < want_n; i++) {
    EXPECT(calls[i] == squeeze(kFirmwareStateCalls[i]), "srv call %zu in cmds_xbox.h is now %s, test copies %s", i,
           calls[i].c_str(), kFirmwareStateCalls[i]);
  }
  EXPECT(squeeze(branch).find("returnchAT::CommandStatus::OK;") != std::string::npos,
         "the Read branch no longer returns chAT::CommandStatus::OK");
  section_done("firmware cmds_xbox.h +XBOXSTATE Read branch still makes the srv calls this test copies", f0);
}

int main(int argc, char** argv) {
  if (argc != 2) {
    std::printf("FAIL usage: %s <repo root>\n", argv[0]);
    return 2;
  }
  test_firmware_handler_unchanged(argv[1]);
  test_server_output();
  test_errors_from_server();
  test_round_trip_through_both();
  test_stock_bridge_from_ra_side();
  return g_failures == 0 ? 0 : 1;
}

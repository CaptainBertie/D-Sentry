/*
 * test_protocol.cpp -- host test of the canonical wire-format header
 * protocol/xbox_r4_protocol.h against docs/protocol.md.
 *
 * Proves: the CRC-8 is the documented one (check value 0xF4), encode/decode round-trip,
 * decode rejects wrong lengths and every single-bit error, the neutral report, the constants,
 * and that the 20-byte layout sits at the byte offsets the docs table states.
 *
 * Usage: test_protocol <repo root>   (the root is needed to read docs/protocol.md)
 *
 * SPDX-License-Identifier: MIT
 */
#include "../protocol/xbox_r4_protocol.h"

#include <cstdio>
#include <cstring>
#include <fstream>
#include <random>
#include <sstream>
#include <string>

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

/* Byte offsets from the table in docs/protocol.md, typed in by hand on purpose so that a
 * change to either the header or the doc shows up here. */
enum {
  OFF_LX = 0, OFF_LY = 2, OFF_RX = 4, OFF_RY = 6, OFF_LT = 8, OFF_RT = 10,
  OFF_DPAD = 12, OFF_BTN_MAIN = 13, OFF_BTN_CENTER = 14, OFF_BTN_SHARE = 15,
  OFF_STATE = 16, OFF_BATTERY = 17, OFF_SEQ = 18, OFF_CRC = 19
};

static void put_u16_le(uint8_t* p, int off, uint16_t v) {
  p[off] = (uint8_t)(v & 0xFF);
  p[off + 1] = (uint8_t)(v >> 8);
}

static bool states_equal(const xbox_r4_state_t& a, const xbox_r4_state_t& b) {
  return memcmp(a.report, b.report, XBOX_R4_REPORT_LEN) == 0 && a.state == b.state &&
         a.battery == b.battery && a.seq == b.seq;
}

static void test_crc() {
  int f0 = g_failures;
  const char* check = "123456789";
  uint8_t crc = xbox_r4_crc8((const uint8_t*)check, strlen(check));
  EXPECT(crc == 0xF4, "crc8(\"123456789\") = 0x%02X, want 0xF4", crc);
  EXPECT(xbox_r4_crc8((const uint8_t*)"", 0) == 0x00, "crc8 of nothing must be the init value 0");
  section_done("crc8 check value 0xF4 (CRC-8/0x07, init 0, no reflection, no xor-out)", f0);
}

static void test_constants() {
  int f0 = g_failures;
  EXPECT(XBOX_R4_STATE_LEN == 20, "state length %d", XBOX_R4_STATE_LEN);
  EXPECT(XBOX_R4_REPORT_LEN == 16, "report length %d", XBOX_R4_REPORT_LEN);
  EXPECT(XBOX_R4_PROTOCOL_VERSION == 1, "protocol version %d", XBOX_R4_PROTOCOL_VERSION);
  /* connection states, docs table row "16" */
  EXPECT(XBOX_R4_CONNECTED == 0 && XBOX_R4_WAITING_FIRST_REPORT == 1 && XBOX_R4_FOUND == 2 &&
             XBOX_R4_SCANNING == 3 && XBOX_R4_OFF == 4 && XBOX_R4_BLOCKED_BY_HCI == 5,
         "connection state enum differs from docs/protocol.md");
  EXPECT(XBOX_R4_BATTERY_UNKNOWN == 0xFF, "battery unknown 0x%X", XBOX_R4_BATTERY_UNKNOWN);
  /* result codes, docs Commands table */
  EXPECT(XBOX_R4_RC_OK == 0 && XBOX_R4_RC_BLOCKED_BY_HCI == 1 && XBOX_R4_RC_INIT_FAILED == 2 &&
             XBOX_R4_RC_NOT_RUNNING == 3,
         "result codes differ from docs/protocol.md");
  /* the request the RA4M1 sends is "AT" + name + "?\r\n" (docs wire example, 15 bytes) */
  const char* req = "AT" XBOX_R4_AT_STATE "?\r\n";
  const uint8_t want[15] = {0x41, 0x54, 0x2B, 0x58, 0x42, 0x4F, 0x58, 0x53,
                            0x54, 0x41, 0x54, 0x45, 0x3F, 0x0D, 0x0A};
  EXPECT(strlen(req) == 15 && memcmp(req, want, 15) == 0, "AT+XBOXSTATE? request bytes");
  EXPECT(strcmp(XBOX_R4_AT_VERSION, "+XBOXVERSION") == 0 && strcmp(XBOX_R4_AT_BEGIN, "+XBOXBEGIN") == 0 &&
             strcmp(XBOX_R4_AT_ADDR, "+XBOXADDR") == 0 && strcmp(XBOX_R4_AT_FORGET, "+XBOXFORGET") == 0 &&
             strcmp(XBOX_R4_AT_LOG, "+XBOXLOG") == 0,
         "AT command names differ from docs/protocol.md");
  section_done("constants match docs/protocol.md (lengths, states, codes, command names)", f0);
}

static void test_neutral() {
  int f0 = g_failures;
  uint8_t r[XBOX_R4_REPORT_LEN];
  memset(r, 0xAA, sizeof r);
  xbox_r4_neutral_report(r);
  const int sticks[4] = {OFF_LX, OFF_LY, OFF_RX, OFF_RY};
  for (int off : sticks) {
    uint16_t v = (uint16_t)(r[off] | (r[off + 1] << 8));
    EXPECT(v == 32767, "stick at offset %d = %u, want 32767", off, v);
  }
  for (int i = OFF_LT; i < XBOX_R4_REPORT_LEN; i++) {
    EXPECT(r[i] == 0, "neutral byte %d = 0x%02X, want 0 (triggers, d-pad, buttons)", i, r[i]);
  }
  section_done("neutral report: sticks 32767, triggers 0, d-pad 0, no buttons", f0);
}

/* Fills a state with a distinct, recognisable value in every field. */
static xbox_r4_state_t make_marked_state() {
  xbox_r4_state_t s;
  memset(&s, 0, sizeof s);
  put_u16_le(s.report, OFF_LX, 0x1101);
  put_u16_le(s.report, OFF_LY, 0x2202);
  put_u16_le(s.report, OFF_RX, 0x3303);
  put_u16_le(s.report, OFF_RY, 0x4404);
  put_u16_le(s.report, OFF_LT, 0x0305);  /* 773, inside 0..1023 */
  put_u16_le(s.report, OFF_RT, 0x0106);  /* 262 */
  s.report[OFF_DPAD] = 7;                 /* W */
  s.report[OFF_BTN_MAIN] = 0x01 | 0x80;   /* A, RB */
  s.report[OFF_BTN_CENTER] = 0x10;        /* Xbox */
  s.report[OFF_BTN_SHARE] = 0x01;         /* Share */
  s.state = XBOX_R4_CONNECTED;
  s.battery = 87;
  s.seq = 0xC9;
  return s;
}

static void test_layout() {
  int f0 = g_failures;
  xbox_r4_state_t s = make_marked_state();
  uint8_t out[XBOX_R4_STATE_LEN];
  xbox_r4_encode(&s, out);
  /* every multi-byte field little-endian at the docs offset */
  EXPECT(out[OFF_LX] == 0x01 && out[OFF_LX + 1] == 0x11, "left stick X at bytes 0-1 LE");
  EXPECT(out[OFF_LY] == 0x02 && out[OFF_LY + 1] == 0x22, "left stick Y at bytes 2-3 LE");
  EXPECT(out[OFF_RX] == 0x03 && out[OFF_RX + 1] == 0x33, "right stick X at bytes 4-5 LE");
  EXPECT(out[OFF_RY] == 0x04 && out[OFF_RY + 1] == 0x44, "right stick Y at bytes 6-7 LE");
  EXPECT(out[OFF_LT] == 0x05 && out[OFF_LT + 1] == 0x03, "left trigger at bytes 8-9 LE");
  EXPECT(out[OFF_RT] == 0x06 && out[OFF_RT + 1] == 0x01, "right trigger at bytes 10-11 LE");
  EXPECT(out[OFF_DPAD] == 7, "d-pad at byte 12");
  EXPECT(out[OFF_BTN_MAIN] == 0x81, "A|RB at byte 13");
  EXPECT(out[OFF_BTN_CENTER] == 0x10, "Xbox at byte 14");
  EXPECT(out[OFF_BTN_SHARE] == 0x01, "Share at byte 15");
  EXPECT(out[OFF_STATE] == XBOX_R4_CONNECTED, "connection state at byte 16");
  EXPECT(out[OFF_BATTERY] == 87, "battery at byte 17");
  EXPECT(out[OFF_SEQ] == 0xC9, "seq at byte 18");
  uint8_t crc = xbox_r4_crc8(out, 19);
  EXPECT(out[OFF_CRC] == crc, "CRC over bytes 0..18 at byte 19: got 0x%02X want 0x%02X", out[OFF_CRC], crc);
  section_done("20-byte layout at the docs/protocol.md offsets", f0);
}

/* The docs table rows the offsets above were copied from; fail if the doc text moves. */
static void test_docs_table(const std::string& root) {
  int f0 = g_failures;
  std::ifstream f(root + "/docs/protocol.md");
  EXPECT(f.good(), "cannot open %s/docs/protocol.md", root.c_str());
  std::stringstream ss;
  ss << f.rdbuf();
  const std::string doc = ss.str();
  const char* rows[] = {
      "| 0-1 | left stick X |", "| 2-3 | left stick Y |", "| 4-5 | right stick X |",
      "| 6-7 | right stick Y |", "| 8-9 | left trigger |", "| 10-11 | right trigger |",
      /* full rows where the encoding column is the contract: the d-pad map that
       * test_parser_replay hard-codes, the state list the enum hard-codes, the battery range */
      "| 12 | d-pad | 0 none, 1 N, 2 NE, 3 E, 4 SE, 5 S, 6 SW, 7 W, 8 NW |",
      "| 13 | buttons | bit0 A, bit1 B, bit3 X, bit4 Y, bit6 LB, bit7 RB |",
      "| 14 | buttons | bit2 View, bit3 Menu, bit4 Xbox, bit5 left stick click, bit6 right stick click |",
      "| 15 | buttons | bit0 Share |",
      ("| 16 | connection state | 0 connected, 1 waiting for first report, 2 found/connecting, 3 scanning, "
       "4 off (no BEGIN yet), 5 blocked by HCI bridge |"),
      "| 17 | battery | percent 0..100, 0xFF unknown |",
      "| 18 | seq |", "| 19 | CRC-8 | over bytes 0..18, polynomial 0x07, init 0x00",
      "`123456789` gives `0xF4`"};
  for (const char* row : rows) {
    EXPECT(doc.find(row) != std::string::npos, "docs/protocol.md no longer contains: %s", row);
  }
  section_done("docs/protocol.md table rows match the hard-coded offsets", f0);
}

static void check_round_trip(const xbox_r4_state_t& s, const char* label) {
  uint8_t wire[XBOX_R4_STATE_LEN];
  xbox_r4_encode(&s, wire);
  xbox_r4_state_t back;
  memset(&back, 0x5A, sizeof back);
  bool ok = xbox_r4_decode(wire, sizeof wire, &back);
  EXPECT(ok, "decode failed for %s", label);
  EXPECT(states_equal(s, back), "round trip changed a field for %s", label);
}

static void test_round_trip() {
  int f0 = g_failures;
  xbox_r4_state_t s;

  /* before BEGIN: neutral, off, battery unknown */
  memset(&s, 0, sizeof s);
  xbox_r4_neutral_report(s.report);
  s.state = XBOX_R4_OFF;
  s.battery = XBOX_R4_BATTERY_UNKNOWN;
  s.seq = 0;
  check_round_trip(s, "neutral/off");

  s = make_marked_state();
  check_round_trip(s, "marked state");

  memset(&s, 0xFF, sizeof s);  /* every bit set */
  check_round_trip(s, "all 0xFF");

  memset(&s, 0x00, sizeof s);  /* every bit clear */
  check_round_trip(s, "all 0x00");

  /* bytes that are special to the AT parsers: CR, LF, '|', 'O', 'K', NUL, 0xFF */
  memset(&s, 0, sizeof s);
  const uint8_t tricky[16] = {0x0D, 0x0A, 0x7C, 'O', 'K', 0x00, 0xFF, 0x0D,
                              0x0A, 'O', 'K', 0x0D, 0x0A, 0x7C, 0x00, 0xFF};
  memcpy(s.report, tricky, 16);
  s.state = XBOX_R4_BLOCKED_BY_HCI;
  s.battery = 100;
  s.seq = 0x0D;
  check_round_trip(s, "AT-special bytes");

  /* 200 random states, fixed seed so a failure is reproducible */
  std::mt19937 rng(20261011u);
  for (int n = 0; n < 200; n++) {
    for (int i = 0; i < XBOX_R4_REPORT_LEN; i++) s.report[i] = (uint8_t)(rng() & 0xFF);
    s.state = (uint8_t)(rng() % 6);
    s.battery = (uint8_t)(rng() % 102);
    if (s.battery == 101) s.battery = XBOX_R4_BATTERY_UNKNOWN;
    s.seq = (uint8_t)(rng() & 0xFF);
    char label[32];
    snprintf(label, sizeof label, "random #%d", n);
    check_round_trip(s, label);
  }
  section_done("encode/decode round trip (5 fixed + 200 random states)", f0);
}

static void test_reject_length() {
  int f0 = g_failures;
  xbox_r4_state_t s = make_marked_state();
  uint8_t wire[XBOX_R4_STATE_LEN + 1];
  xbox_r4_encode(&s, wire);
  wire[20] = 0;
  const size_t bad_lengths[] = {0, 1, 15, 16, 19, 21};
  for (size_t len : bad_lengths) {
    xbox_r4_state_t out;
    memset(&out, 0x5A, sizeof out);
    xbox_r4_state_t before = out;
    EXPECT(!xbox_r4_decode(wire, len, &out), "decode accepted length %zu", len);
    EXPECT(states_equal(out, before), "decode wrote to *s on rejected length %zu", len);
  }
  section_done("decode rejects lengths 0, 1, 15, 16, 19, 21 and leaves *s untouched", f0);
}

static void test_single_bit_errors() {
  int f0 = g_failures;
  xbox_r4_state_t s = make_marked_state();
  uint8_t good[XBOX_R4_STATE_LEN];
  xbox_r4_encode(&s, good);
  int caught = 0;
  for (int pos = 0; pos < XBOX_R4_STATE_LEN; pos++) {
    for (int bit = 0; bit < 8; bit++) {
      uint8_t bad[XBOX_R4_STATE_LEN];
      memcpy(bad, good, sizeof bad);
      bad[pos] ^= (uint8_t)(1u << bit);
      xbox_r4_state_t out;
      memset(&out, 0x5A, sizeof out);
      xbox_r4_state_t before = out;
      bool ok = xbox_r4_decode(bad, sizeof bad, &out);
      EXPECT(!ok, "flipped bit %d of byte %d was not caught", bit, pos);
      EXPECT(states_equal(out, before), "decode wrote to *s on CRC error (byte %d bit %d)", pos, bit);
      if (!ok) caught++;
    }
  }
  std::printf("info single-bit flips caught by the CRC: %d of 160 (8 bits x 20 positions)\n", caught);
  section_done("decode rejects every single-bit flip in all 20 bytes", f0);
}

int main(int argc, char** argv) {
  if (argc < 2) {
    std::printf("FAIL usage: %s <repo root>\n", argv[0]);
    return 2;
  }
  test_crc();
  test_constants();
  test_neutral();
  test_layout();
  test_docs_table(argv[1]);
  test_round_trip();
  test_reject_length();
  test_single_bit_errors();
  return g_failures == 0 ? 0 : 1;
}

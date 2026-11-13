/*
 * test_parser_replay.cpp -- host test of the real XboxControllerNotificationParser
 * (firmware/libraries/XboxControllerNotificationParser, asukiaaa, MIT, 1.0.5 @ 77a449d)
 * compiled against tests/stubs/Arduino.h.
 *
 * Proves: each button bit maps to exactly one flag, the d-pad values 0..8 map to the four
 * direction flags as docs/protocol.md says, stick and trigger extremes decode little-endian,
 * long (Elite) reports are accepted and short ones rejected, and that update() then toArr()
 * gives back the original 16 bytes on the documented value domain. Where toArr() is lossy
 * (bits and values the docs do not define) the test states exactly which bytes change.
 *
 * SPDX-License-Identifier: MIT
 */
#include "../firmware/libraries/XboxControllerNotificationParser/src/XboxControllerNotificationParser.h"
#include "../firmware/libraries/XboxControllerNotificationParser/src/XboxControllerElite2NotificationParser.h"

#include <cstdio>
#include <cstring>
#include <random>

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

typedef XboxControllerNotificationParser Parser;

/* A neutral 16-byte report: sticks 32767 LE (FF 7F), everything else 0. */
static void neutral(uint8_t r[16]) {
  memset(r, 0, 16);
  for (int i = 0; i < 8; i += 2) { r[i] = 0xFF; r[i + 1] = 0x7F; }
}

/* The 16 boolean flags of the parser, in a fixed order, so tests can compare them as a set. */
enum Flag {
  F_A, F_B, F_X, F_Y, F_LB, F_RB, F_SELECT, F_START, F_XBOX, F_LS, F_RS, F_SHARE,
  F_UP, F_RIGHT, F_DOWN, F_LEFT, F_COUNT
};
static const char* kFlagNames[F_COUNT] = {"A", "B", "X", "Y", "LB", "RB", "View", "Menu",
                                          "Xbox", "LS", "RS", "Share", "Up", "Right", "Down", "Left"};
static void read_flags(const Parser& p, bool f[F_COUNT]) {
  f[F_A] = p.btnA; f[F_B] = p.btnB; f[F_X] = p.btnX; f[F_Y] = p.btnY;
  f[F_LB] = p.btnLB; f[F_RB] = p.btnRB; f[F_SELECT] = p.btnSelect; f[F_START] = p.btnStart;
  f[F_XBOX] = p.btnXbox; f[F_LS] = p.btnLS; f[F_RS] = p.btnRS; f[F_SHARE] = p.btnShare;
  f[F_UP] = p.btnDirUp; f[F_RIGHT] = p.btnDirRight; f[F_DOWN] = p.btnDirDown; f[F_LEFT] = p.btnDirLeft;
}

/* All parser fields, for "fields untouched" checks. */
struct Snapshot {
  bool f[F_COUNT];
  uint16_t lx, ly, rx, ry, lt, rt;
};
static Snapshot snap(const Parser& p) {
  Snapshot s;
  read_flags(p, s.f);
  s.lx = p.joyLHori; s.ly = p.joyLVert; s.rx = p.joyRHori; s.ry = p.joyRVert;
  s.lt = p.trigLT; s.rt = p.trigRT;
  return s;
}
static bool same(const Snapshot& a, const Snapshot& b) {
  return memcmp(a.f, b.f, sizeof a.f) == 0 && a.lx == b.lx && a.ly == b.ly && a.rx == b.rx &&
         a.ry == b.ry && a.lt == b.lt && a.rt == b.rt;
}

/* Buttons per docs/protocol.md rows 13, 14, 15: (byte, bit, flag). */
struct ButtonBit { int byte; int bit; Flag flag; };
static const ButtonBit kButtons[] = {
    {13, 0, F_A}, {13, 1, F_B}, {13, 3, F_X}, {13, 4, F_Y}, {13, 6, F_LB}, {13, 7, F_RB},
    {14, 2, F_SELECT}, {14, 3, F_START}, {14, 4, F_XBOX}, {14, 5, F_LS}, {14, 6, F_RS},
    {15, 0, F_SHARE}};

static void test_each_button() {
  int f0 = g_failures;
  for (const ButtonBit& b : kButtons) {
    uint8_t r[16];
    neutral(r);
    r[b.byte] = (uint8_t)(1u << b.bit);
    Parser p;
    EXPECT(p.update(r, 16) == 0, "update rejected a 16-byte report");
    bool f[F_COUNT];
    read_flags(p, f);
    for (int i = 0; i < F_COUNT; i++) {
      bool want = (i == b.flag);
      EXPECT(f[i] == want, "byte %d bit %d: flag %s is %d, want %d", b.byte, b.bit, kFlagNames[i], f[i], want);
    }
    EXPECT(p.joyLHori == 32767 && p.joyLVert == 32767 && p.joyRHori == 32767 && p.joyRVert == 32767 &&
               p.trigLT == 0 && p.trigRT == 0,
           "a button bit moved a stick or trigger (byte %d bit %d)", b.byte, b.bit);
  }
  section_done("each documented button bit alone sets only its own flag (12 buttons)", f0);

  /* Undefined bits: docs leave byte 13 bits 2 and 5, byte 14 bits 0, 1 and 7, byte 15 bits 1..7
   * unassigned. They must not light any flag. */
  f0 = g_failures;
  const int undefined[][2] = {{13, 2}, {13, 5}, {14, 0}, {14, 1}, {14, 7},
                              {15, 1}, {15, 2}, {15, 3}, {15, 4}, {15, 5}, {15, 6}, {15, 7}};
  for (const auto& u : undefined) {
    uint8_t r[16];
    neutral(r);
    r[u[0]] = (uint8_t)(1u << u[1]);
    Parser p;
    p.update(r, 16);
    bool f[F_COUNT];
    read_flags(p, f);
    for (int i = 0; i < F_COUNT; i++) {
      EXPECT(!f[i], "undefined byte %d bit %d set flag %s", u[0], u[1], kFlagNames[i]);
    }
  }
  section_done("the 12 undocumented button bits set no flag", f0);
}

static void test_dpad() {
  int f0 = g_failures;
  /* docs/protocol.md row 12: 0 none, 1 N, 2 NE, 3 E, 4 SE, 5 S, 6 SW, 7 W, 8 NW.
   * Columns: up, right, down, left. */
  const bool want[9][4] = {
      {0, 0, 0, 0}, {1, 0, 0, 0}, {1, 1, 0, 0}, {0, 1, 0, 0}, {0, 1, 1, 0},
      {0, 0, 1, 0}, {0, 0, 1, 1}, {0, 0, 0, 1}, {1, 0, 0, 1}};
  for (int v = 0; v <= 8; v++) {
    uint8_t r[16];
    neutral(r);
    r[12] = (uint8_t)v;
    Parser p;
    p.update(r, 16);
    EXPECT(p.btnDirUp == want[v][0] && p.btnDirRight == want[v][1] && p.btnDirDown == want[v][2] &&
               p.btnDirLeft == want[v][3],
           "d-pad %d gave up=%d right=%d down=%d left=%d", v, p.btnDirUp, p.btnDirRight, p.btnDirDown,
           p.btnDirLeft);
    EXPECT(!p.btnA && !p.btnB && !p.btnX && !p.btnY && !p.btnShare, "d-pad %d set a face button", v);
  }
  section_done("d-pad values 0..8 map to up/right/down/left exactly as docs/protocol.md", f0);

  f0 = g_failures;
  for (int v = 9; v <= 255; v++) {
    uint8_t r[16];
    neutral(r);
    r[12] = (uint8_t)v;
    Parser p;
    p.update(r, 16);
    EXPECT(!p.btnDirUp && !p.btnDirRight && !p.btnDirDown && !p.btnDirLeft, "d-pad %d set a direction", v);
  }
  section_done("d-pad values 9..255 (undocumented) set no direction", f0);
}

static void test_sticks_and_triggers() {
  int f0 = g_failures;
  const uint16_t extremes[] = {0, 32767, 65535, 0x1234};
  for (int axis = 0; axis < 4; axis++) {
    for (uint16_t v : extremes) {
      uint8_t r[16];
      neutral(r);
      r[axis * 2] = (uint8_t)(v & 0xFF);
      r[axis * 2 + 1] = (uint8_t)(v >> 8);
      Parser p;
      p.update(r, 16);
      const uint16_t got[4] = {p.joyLHori, p.joyLVert, p.joyRHori, p.joyRVert};
      for (int other = 0; other < 4; other++) {
        uint16_t want = (other == axis) ? v : 32767;
        EXPECT(got[other] == want, "axis %d = %u: axis %d read %u, want %u", axis, v, other, got[other], want);
      }
    }
  }
  section_done("stick extremes 0/32767/65535 (and 0x1234 for byte order) on all four axes, LE", f0);

  f0 = g_failures;
  const uint16_t trig[] = {0, 1023, 0x0201};
  for (int t = 0; t < 2; t++) {
    for (uint16_t v : trig) {
      uint8_t r[16];
      neutral(r);
      r[8 + t * 2] = (uint8_t)(v & 0xFF);
      r[8 + t * 2 + 1] = (uint8_t)(v >> 8);
      Parser p;
      p.update(r, 16);
      uint16_t mine = t == 0 ? p.trigLT : p.trigRT;
      uint16_t other = t == 0 ? p.trigRT : p.trigLT;
      EXPECT(mine == v && other == 0, "trigger %d = %u read %u (other %u)", t, v, mine, other);
    }
  }
  section_done("triggers 0/1023 at bytes 8-9 (LT) and 10-11 (RT), LE", f0);
}

static void test_lengths() {
  int f0 = g_failures;
  /* An Elite Series 2 report is longer (profile at 16, paddles at 18). The base parser must
   * accept it and read only the first 16 bytes. */
  uint8_t r19[19];
  neutral(r19);
  r19[0] = 0x00; r19[1] = 0x00;  /* LX = 0 */
  r19[13] = 0x01;                /* A */
  r19[12] = 3;                   /* E */
  r19[16] = 0xFF; r19[17] = 0xFF; r19[18] = 0xFF;
  Parser p;
  EXPECT(p.update(r19, 19) == 0, "base parser rejected a 19-byte report");
  EXPECT(p.joyLHori == 0 && p.btnA && p.btnDirRight && !p.btnB && !p.btnShare,
         "19-byte report not parsed from its first 16 bytes");
  uint8_t back[16];
  p.toArr(back, 16);
  EXPECT(memcmp(back, r19, 16) == 0, "19-byte report: first 16 bytes did not round-trip");
  section_done("19-byte Elite report accepted by the base parser; first 16 bytes parsed", f0);

  f0 = g_failures;
  Parser q;
  uint8_t r16[16];
  neutral(r16);
  r16[0] = 0x34; r16[1] = 0x12; r16[9] = 0x03; r16[13] = 0x02; r16[12] = 5;
  q.update(r16, 16);
  Snapshot before = snap(q);
  uint8_t r15[15];
  memset(r15, 0xFF, sizeof r15);
  uint8_t rc = q.update(r15, 15);
  EXPECT(rc == XBOX_CONTROLLER_ERROR_INVALID_LENGTH, "15-byte report returned %u, want %d", rc,
         XBOX_CONTROLLER_ERROR_INVALID_LENGTH);
  EXPECT(same(before, snap(q)), "15-byte report changed parser fields");
  EXPECT(q.update(r15, 0) == XBOX_CONTROLLER_ERROR_INVALID_LENGTH, "0-byte report accepted");
  uint8_t small[15];
  EXPECT(q.toArr(small, 15) == XBOX_CONTROLLER_ERROR_INVALID_LENGTH, "toArr accepted a 15-byte buffer");
  section_done("15-byte report rejected with XBOX_CONTROLLER_ERROR_INVALID_LENGTH, fields untouched", f0);

  /* The Elite2 subclass is not used by the bridge, but it lives in the same library. Record
   * its behaviour: it reads profile and paddles, and it ignores the base class's length error. */
  f0 = g_failures;
  XboxControllerElite2NotificationParser e;
  r19[16] = 2;      /* profile */
  r19[18] = 0x05;   /* P1, P3 */
  EXPECT(e.update(r19, 19) == 0, "Elite2 parser rejected 19 bytes");
  EXPECT(e.profile == 2 && e.btnP1 && !e.btnP2 && e.btnP3 && !e.btnP4 && e.btnA, "Elite2 fields wrong");
  Snapshot e_before = snap(e);
  uint8_t er = e.update(r15, 15);
  EXPECT(er == 0, "Elite2 update(15 bytes) returned %u", er);
  EXPECT(same(e_before, snap(e)), "Elite2 update(15 bytes) changed base fields");
  EXPECT(e.profile == 0 && !e.btnP1 && !e.btnP3, "Elite2 update(15 bytes) kept profile/paddles");
  std::printf("info Elite2 parser quirk (upstream): update() on a 15-byte report returns %u (not "
              "XBOX_CONTROLLER_ERROR_INVALID_LENGTH); base fields stay untouched, profile and paddles reset to 0. "
              "The bridge uses the base parser, so this does not reach the wire.\n",
              er);
  section_done("Elite2 subclass: profile byte 16, paddles byte 18 (documented quirk on short reports)", f0);
}

static void test_to_string() {
  int f0 = g_failures;
  uint8_t r[16];
  neutral(r);
  r[13] = 0x01;
  Parser p;
  p.update(r, 16);
  String s = p.toString();
  EXPECT(s.indexOf("btnA: 1") >= 0, "toString lacks 'btnA: 1': %s", s.c_str());
  EXPECT(s.indexOf("joyLHori: 32767") >= 0, "toString lacks 'joyLHori: 32767'");
  section_done("toString() builds against the String stub and reports fields", f0);
}

/* What toArr() can reproduce: only the documented bits of bytes 13..15 and d-pad 0..8. */
static void expected_after_round_trip(const uint8_t in[16], uint8_t out[16]) {
  memcpy(out, in, 16);
  if (out[12] > 8) out[12] = 0;
  out[13] &= 0xDB;  /* keep bits 0,1,3,4,6,7 */
  out[14] &= 0x7C;  /* keep bits 2..6 */
  out[15] &= 0x01;  /* keep bit 0 */
}

static bool round_trip(const uint8_t in[16], uint8_t out[16]) {
  Parser p;
  uint8_t copy[16];
  memcpy(copy, in, 16);  /* update() takes a non-const pointer */
  if (p.update(copy, 16) != 0) return false;
  memset(out, 0xEE, 16);
  return p.toArr(out, 16) == 0;
}

static void test_round_trip() {
  int f0 = g_failures;
  for (int v = 0; v <= 8; v++) {
    uint8_t in[16], out[16];
    neutral(in);
    in[12] = (uint8_t)v;
    EXPECT(round_trip(in, out), "round trip call failed");
    EXPECT(memcmp(in, out, 16) == 0, "d-pad %d did not round-trip (got %u)", v, out[12]);
  }
  section_done("update()+toArr() round trip is exact for every d-pad value 0..8", f0);

  /* 200 random reports inside the documented domain: any stick value, triggers 0..1023,
   * d-pad 0..8, only documented button bits. These must come back byte for byte. */
  f0 = g_failures;
  std::mt19937 rng(16u);
  for (int n = 0; n < 200; n++) {
    uint8_t in[16], out[16];
    for (int i = 0; i < 16; i++) in[i] = (uint8_t)(rng() & 0xFF);
    in[9] &= 0x03;   /* LT <= 1023 */
    in[11] &= 0x03;  /* RT <= 1023 */
    in[12] = (uint8_t)(rng() % 9);
    in[13] &= 0xDB;
    in[14] &= 0x7C;
    in[15] &= 0x01;
    EXPECT(round_trip(in, out), "round trip call failed");
    for (int i = 0; i < 16; i++) {
      EXPECT(in[i] == out[i], "documented-domain report #%d byte %d: 0x%02X came back as 0x%02X", n, i, in[i], out[i]);
    }
  }
  section_done("update()+toArr() round trip is exact on 200 random documented-domain reports (seed 16)", f0);

  /* 200 fully random reports: toArr() is not lossless here. Check that the loss is exactly
   * the undocumented part, and say which bytes changed how often. */
  f0 = g_failures;
  std::mt19937 rng2(20261011u);
  int changed_reports = 0;
  int changed_per_byte[16] = {0};
  for (int n = 0; n < 200; n++) {
    uint8_t in[16], out[16], want[16];
    for (int i = 0; i < 16; i++) in[i] = (uint8_t)(rng2() & 0xFF);
    EXPECT(round_trip(in, out), "round trip call failed");
    expected_after_round_trip(in, want);
    bool changed = false;
    for (int i = 0; i < 16; i++) {
      EXPECT(out[i] == want[i], "random report #%d byte %d: in 0x%02X out 0x%02X, predicted 0x%02X", n, i,
             in[i], out[i], want[i]);
      if (out[i] != in[i]) { changed = true; changed_per_byte[i]++; }
    }
    if (changed) changed_reports++;
  }
  std::printf("info toArr() is lossless on bytes 0-11 (sticks and triggers, no 0..1023 clamp) and on the documented "
              "values of bytes 12-15.\n");
  std::printf("info toArr() is lossy outside the documented domain: byte 12 values 9..255 -> 0; byte 13 bits 2,5 "
              "dropped (mask 0xDB); byte 14 bits 0,1,7 dropped (mask 0x7C); byte 15 bits 1..7 dropped (mask 0x01).\n");
  std::printf("info fully random reports (seed 20261011): %d of 200 changed; changes per byte 0..15:", changed_reports);
  for (int i = 0; i < 16; i++) std::printf(" %d", changed_per_byte[i]);
  std::printf("\n");
  for (int i = 0; i < 12; i++) {
    EXPECT(changed_per_byte[i] == 0, "byte %d changed in a round trip", i);
  }
  section_done("fully random reports lose exactly the undocumented bits/values and nothing else", f0);
}

int main() {
  test_each_button();
  test_dpad();
  test_sticks_and_triggers();
  test_lengths();
  test_to_string();
  test_round_trip();
  return g_failures == 0 ? 0 : 1;
}

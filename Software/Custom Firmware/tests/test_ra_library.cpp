/*
 * test_ra_library.cpp -- host test of the RA4M1 library's own poll():
 *   ra4m1/libraries/XboxControllerR4/src/XboxControllerR4.cpp and its vendored parser (this
 *   project's code and asukiaaa's MIT parser, compiled in place), linked with the REAL WiFiS3
 *   Modem.cpp (core arduino:renesas_uno 1.6.0) and the stub UART in tests/stubs/Arduino.h.
 *
 * test_modem_e2e.cpp shows what Modem.cpp returns for each kind of reply. This test shows what
 * the library makes of it: poll()'s result, pollErrors(), hasNewReport(), seq(),
 * connectionState(), battery() and the decoded fields. So a regression in poll() itself (a
 * dropped drain, a dropped length or CRC check, or a timeout left at 50 ms) fails here.
 * (poll() checks the length twice, itself and inside xbox_r4_decode(); either one is enough.)
 *
 * The library includes <WiFiS3.h>; tests/stubs/WiFiS3.h stands in for it and brings only the
 * real Modem.h.
 *
 * SPDX-License-Identifier: MIT
 */
#include "XboxControllerR4.h"
#include "Modem.h"

#include <chrono>
#include <cstdio>
#include <cstring>
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

/* The real board's modem global is bound to Serial2 (Modem.cpp:423-424). */
UART Serial2;

static const char kRequest[] = "AT+XBOXSTATE?\r\n";

static std::string encode(const xbox_r4_state_t& s) {
  uint8_t out[XBOX_R4_STATE_LEN];
  xbox_r4_encode(&s, out);
  return std::string((const char*)out, sizeof out);
}

static std::string reply(const std::string& payload) {
  return std::string("+XBOXSTATE: 20|") + payload + "\r\nOK\r\n";
}

static void put_u16(uint8_t* p, uint16_t v) {
  p[0] = (uint8_t)(v & 0xFF);
  p[1] = (uint8_t)(v >> 8);
}

/* A report inside the documented domain whose bytes still include CR, LF, '|', 'O', 'K', NUL
 * and 0xFF: LX 0x0A0D, LY 0x4F7C, RX 0x004B, RY 0xFFFF, LT 0x030D, RT 512, d-pad 3 (E),
 * A + RB, Xbox, Share. */
static xbox_r4_state_t make_state(uint8_t conn, uint8_t battery, uint8_t seq) {
  xbox_r4_state_t s;
  memset(&s, 0, sizeof s);
  put_u16(&s.report[0], 0x0A0D);
  put_u16(&s.report[2], 0x4F7C);
  put_u16(&s.report[4], 0x004B);
  put_u16(&s.report[6], 0xFFFF);
  put_u16(&s.report[8], 0x030D);
  put_u16(&s.report[10], 512);
  s.report[12] = 3;
  s.report[13] = 0x81;
  s.report[14] = 0x10;
  s.report[15] = 0x01;
  s.state = conn;
  s.battery = battery;
  s.seq = seq;
  return s;
}

/* The ESP32 answers the next request with `wire`, after the request has gone out. */
static void answer_with(const std::string& wire) {
  Serial2.on_write = [wire](const uint8_t*, size_t) { Serial2.feed(wire); };
}

struct TimedPoll {
  bool ok;
  double ms;
};

static TimedPoll timed_poll(XboxControllerR4& x) {
  auto t0 = std::chrono::steady_clock::now();
  bool ok = x.poll();
  auto t1 = std::chrono::steady_clock::now();
  return {ok, std::chrono::duration<double, std::milli>(t1 - t0).count()};
}

/* True when the fields poll() decoded are the ones make_state() put in the report. */
static bool fields_match_make_state(const XboxControllerNotificationParser& n) {
  return n.joyLHori == 0x0A0D && n.joyLVert == 0x4F7C && n.joyRHori == 0x004B && n.joyRVert == 0xFFFF &&
         n.trigLT == 0x030D && n.trigRT == 512 && n.btnDirRight && !n.btnDirUp && !n.btnDirDown &&
         !n.btnDirLeft && n.btnA && n.btnRB && !n.btnB && !n.btnX && !n.btnY && !n.btnLB && n.btnXbox &&
         !n.btnSelect && !n.btnStart && !n.btnLS && !n.btnRS && n.btnShare;
}

static bool is_neutral(const XboxControllerNotificationParser& n) {
  return n.joyLHori == 32767 && n.joyLVert == 32767 && n.joyRHori == 32767 && n.joyRVert == 32767 &&
         n.trigLT == 0 && n.trigRT == 0 && !n.btnA && !n.btnRB && !n.btnXbox && !n.btnShare && !n.btnDirRight;
}

/* (a) the documented reply, then seq gating */
static void case_correct_reply() {
  int f0 = g_failures;
  Serial2.reset();
  XboxControllerR4 x;
  EXPECT(x.connectionState() == XBOX_R4_OFF && x.battery() == XBOX_R4_BATTERY_UNKNOWN && x.pollErrors() == 0,
         "fresh object: state %u battery %u errors %u", x.connectionState(), x.battery(), (unsigned)x.pollErrors());
  answer_with(reply(encode(make_state(XBOX_R4_CONNECTED, 87, 0x2A))));
  TimedPoll p = timed_poll(x);
  EXPECT(p.ok, "poll() returned false on the documented reply");
  EXPECT(Serial2.tx == kRequest, "poll() sent %zu bytes, not AT+XBOXSTATE?\\r\\n", Serial2.tx.size());
  EXPECT(x.hasNewReport() && x.seq() == 0x2A && x.connectionState() == XBOX_R4_CONNECTED && x.isConnected() &&
             x.battery() == 87 && x.pollErrors() == 0,
         "new=%d seq=%u state=%u battery=%u errors=%u", x.hasNewReport(), x.seq(), x.connectionState(), x.battery(),
         (unsigned)x.pollErrors());
  EXPECT(fields_match_make_state(x.xboxNotif), "decoded fields differ from the report");
  std::printf("info library poll() on the 41-byte reply: ok=%d in %.2f ms\n", p.ok, p.ms);
  section_done("(a) poll() accepts the documented reply: new report, seq, state, battery and every field", f0);

  /* the same seq again means no new HID report (hasNewReport false), but every valid connected
   * frame is applied: the firmware snapshots report and seq together, so a frame with the same
   * seq and different bytes cannot happen on the wire; the library simply follows the frame */
  f0 = g_failures;
  Serial2.reset();
  xbox_r4_state_t same_seq = make_state(XBOX_R4_CONNECTED, 86, 0x2A);
  put_u16(&same_seq.report[0], 0x1111);
  answer_with(reply(encode(same_seq)));
  p = timed_poll(x);
  EXPECT(p.ok && !x.hasNewReport() && x.xboxNotif.joyLHori == 0x1111 && x.battery() == 86,
         "ok=%d new=%d LX=0x%04X battery=%u", p.ok, x.hasNewReport(), x.xboxNotif.joyLHori, x.battery());
  section_done("(a) an unchanged seq is not a new report; the frame is still applied (fields and battery follow it)", f0);

  f0 = g_failures;
  Serial2.reset();
  xbox_r4_state_t next = same_seq;
  next.seq = 0x2B;
  answer_with(reply(encode(next)));
  p = timed_poll(x);
  EXPECT(p.ok && x.hasNewReport() && x.seq() == 0x2B && x.xboxNotif.joyLHori == 0x1111,
         "ok=%d new=%d seq=%u LX=0x%04X", p.ok, x.hasNewReport(), x.seq(), x.xboxNotif.joyLHori);
  EXPECT(x.pollErrors() == 0, "errors=%u", (unsigned)x.pollErrors());
  section_done("(a) a new seq is a new report and updates the fields", f0);
}

/* A connected object with seq 0x10, for the cases that check nothing changes on a bad poll. */
static void connect(XboxControllerR4& x) {
  Serial2.reset();
  answer_with(reply(encode(make_state(XBOX_R4_CONNECTED, 50, 0x10))));
  bool ok = x.poll();
  EXPECT(ok && x.seq() == 0x10 && x.isConnected(), "setup poll failed");
}

/* A rejected poll must count an error and leave every reading as it was. */
static void expect_rejected(XboxControllerR4& x, bool ok, uint32_t errors_before, const char* label) {
  EXPECT(!ok, "%s: poll() returned true", label);
  EXPECT(x.pollErrors() == errors_before + 1, "%s: pollErrors %u, want %u", label, (unsigned)x.pollErrors(),
         (unsigned)(errors_before + 1));
  EXPECT(!x.hasNewReport() && x.seq() == 0x10 && x.isConnected() && x.battery() == 50 &&
             fields_match_make_state(x.xboxNotif),
         "%s: a rejected poll changed the readings (new=%d seq=%u state=%u)", label, x.hasNewReport(), x.seq(),
         x.connectionState());
}

/* (c) what a stock bridge sends (bare ERROR), and a prompt followed by ERROR */
static void case_timeouts() {
  int f0 = g_failures;
  XboxControllerR4 x;
  connect(x);
  double ms[3];
  for (int i = 0; i < 3; i++) {
    Serial2.reset();
    answer_with("ERROR\r\n");
    uint32_t e = x.pollErrors();
    TimedPoll p = timed_poll(x);
    ms[i] = p.ms;
    expect_rejected(x, p.ok, e, "bare ERROR");
    EXPECT(p.ms >= XBOX_R4_POLL_TIMEOUT_MS, "bare ERROR: returned after %.2f ms, before the timeout", p.ms);
    EXPECT(p.ms < 1000.0, "bare ERROR: took %.2f ms", p.ms);
  }
  double fastest = ms[0];
  for (double m : ms) if (m < fastest) fastest = m;
  EXPECT(fastest <= XBOX_R4_POLL_TIMEOUT_MS + 10, "fastest of 3 took %.2f ms, want <= %d", fastest,
         XBOX_R4_POLL_TIMEOUT_MS + 10);
  std::printf("info library poll() against a bare ERROR: false after %.2f / %.2f / %.2f ms\n", ms[0], ms[1], ms[2]);

  Serial2.reset();
  answer_with("+XBOXSTATE: \r\nERROR\r\n");
  uint32_t e = x.pollErrors();
  TimedPoll p = timed_poll(x);
  expect_rejected(x, p.ok, e, "prompt then ERROR");
  EXPECT(p.ms >= XBOX_R4_POLL_TIMEOUT_MS, "prompt then ERROR: returned after %.2f ms", p.ms);
  section_done("(c) bare ERROR and prompt-then-ERROR: poll() false at the 50 ms timeout, one error each, "
               "readings unchanged",
               f0);
}

/* After poll() the modem timeout must be back to MODEM_TIMEOUT, or every later WiFiS3 call
 * would give up after 50 ms. Modem.h has no getter, so this sends one more command whose
 * reply lands 100 ms later: with the timeout restored it succeeds, at 50 ms it would return
 * false at about 51 ms. (millis() counts whole milliseconds, so "100 ms later" can measure as
 * 99.x ms on the steady clock; the bound below only needs to be well past 51.) */
static void expect_timeout_restored(const char* after) {
  const unsigned long kLateMs = 100;
  Serial2.reset();
  const std::string late = reply(encode(make_state(XBOX_R4_CONNECTED, 50, 0x77)));
  Serial2.on_write = [late](const uint8_t*, size_t) { Serial2.feed_after(kLateMs, late); };
  std::string res;
  auto t0 = std::chrono::steady_clock::now();
  bool ok = modem.write(std::string("+XBOXSTATE:"), res, "%s", kRequest);
  double ms = std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - t0).count();
  EXPECT(ok && res.size() == XBOX_R4_STATE_LEN && ms > 75.0,
         "after %s: a reply %lu ms late gave ok=%d len=%zu after %.2f ms; the 50 ms poll timeout was left in place",
         after, kLateMs, ok, res.size(), ms);
}

static void case_timeout_restored() {
  int f0 = g_failures;
  XboxControllerR4 x;
  connect(x);
  expect_timeout_restored("a good poll");
  Serial2.reset();
  answer_with("ERROR\r\n");
  bool ok = x.poll();
  EXPECT(!ok, "ERROR poll returned true");
  expect_timeout_restored("a timed-out poll");
  section_done("poll() puts the modem timeout back after a good poll and after a timeout", f0);
}

/* (e1) the late reply to an earlier poll is waiting in rx; (e3) it lands after the drain */
static void case_stale() {
  int f0 = g_failures;
  XboxControllerR4 x;
  connect(x);
  Serial2.reset();
  Serial2.feed(reply(encode(make_state(XBOX_R4_CONNECTED, 50, 7))));
  answer_with(reply(encode(make_state(XBOX_R4_CONNECTED, 50, 8))));
  bool ok = x.poll();
  EXPECT(ok && x.seq() == 8 && x.hasNewReport(), "ok=%d seq=%u", ok, x.seq());
  EXPECT(Serial2.available() == 0, "%d bytes left in rx", Serial2.available());
  section_done("(e1) a stale reply already in rx is drained; poll() returns the fresh frame", f0);

  /* e3: last accepted seq 8; poll N's late reply (seq 9) lands after poll N+1's drain, ahead of
   * the fresh reply (seq 10). poll N+1 returns seq 9: one poll late, but newer than seq 8.
   * poll N+2 drains seq 10 and returns its own reply (seq 11). Seq never goes backwards. */
  f0 = g_failures;
  Serial2.reset();
  const std::string late = reply(encode(make_state(XBOX_R4_CONNECTED, 50, 9)));
  const std::string fresh = reply(encode(make_state(XBOX_R4_CONNECTED, 50, 10)));
  Serial2.on_write = [late, fresh](const uint8_t*, size_t) {
    Serial2.feed(late);
    Serial2.feed(fresh);
  };
  ok = x.poll();
  EXPECT(ok && x.seq() == 9 && x.hasNewReport(), "late-after-drain: ok=%d seq=%u", ok, x.seq());
  Serial2.on_write = nullptr;
  answer_with(reply(encode(make_state(XBOX_R4_CONNECTED, 50, 11))));
  ok = x.poll();
  EXPECT(ok && x.seq() == 11, "next poll: ok=%d seq=%u", ok, x.seq());
  EXPECT(x.pollErrors() == 0, "errors=%u", (unsigned)x.pollErrors());
  std::printf("info library poll() with a late reply landing after the drain: returned seq 9 (one poll late, newer "
              "than the last accepted seq 8), then seq 11\n");
  section_done("(e3) a late reply after the drain is returned one poll late, never older; the next poll catches up",
               f0);
}

/* (f2) truncated payload + trailer; (f3) a stale sized reply from another command */
static void case_truncated() {
  int f0 = g_failures;
  XboxControllerR4 x;
  connect(x);
  const std::string payload = encode(make_state(XBOX_R4_CONNECTED, 50, 0x20));
  Serial2.reset();
  answer_with("+XBOXSTATE: 20|" + payload.substr(0, 15) + "\r\nOK\r\n");
  uint32_t e = x.pollErrors();
  bool ok = x.poll();
  expect_rejected(x, ok, e, "20| with 15 bytes then the trailer");
  section_done("(f2) 20| with 15 bytes then \\r\\nOK\\r\\n: poll() false, one error, readings unchanged", f0);

  /* f3 drained: the stale +PREFGET reply is in rx before the poll */
  f0 = g_failures;
  Serial2.reset();
  Serial2.feed("+PREFGET: 5|abcde\r\nOK\r\n");
  answer_with(reply(payload));
  ok = x.poll();
  EXPECT(ok && x.seq() == 0x20, "drained: ok=%d seq=%u", ok, x.seq());
  section_done("(f3) a stale +PREFGET: 5|abcde in rx is drained; poll() gets all 20 bytes", f0);

  /* f3 undrained: the stale sized reply lands after the drain, ahead of ours. With no 'O' in
   * payload bytes 15..19, Modem.cpp returns true with 15 bytes, so only the size check is left. */
  f0 = g_failures;
  xbox_r4_state_t s = make_state(XBOX_R4_CONNECTED, 50, 0x21);
  std::string pl = encode(s);
  for (int seq = 0x21; seq < 0x21 + 256 && pl.substr(15).find('O') != std::string::npos; seq++) {
    s.seq = (uint8_t)seq;
    pl = encode(s);
  }
  EXPECT(pl.substr(15).find('O') == std::string::npos, "test precondition: no 'O' in payload bytes 15..19");
  Serial2.reset();
  Serial2.on_write = [pl](const uint8_t*, size_t) {
    Serial2.feed("+PREFGET: 5|abcde\r\nOK\r\n");
    Serial2.feed(reply(pl));
  };
  e = x.pollErrors();
  ok = x.poll();
  EXPECT(!ok && x.pollErrors() == e + 1 && x.seq() == 0x20, "ok=%d errors=%u seq=%u", ok, (unsigned)x.pollErrors(),
         x.seq());
  section_done("(f3) a stale sized reply landing after the drain truncates ours; the size check rejects it", f0);
}

/* Modem.cpp returns ok for each of these; only poll()'s size and CRC check rejects them. */
static void case_wrong_size_and_crc() {
  int f0 = g_failures;
  XboxControllerR4 x;
  connect(x);
  const std::string payload = encode(make_state(XBOX_R4_CONNECTED, 50, 0x30));
  std::string bad_crc = payload;
  bad_crc[19] = (char)(bad_crc[19] ^ 0x80);
  struct Edge {
    const char* label;
    std::string wire;
  };
  const Edge edges[] = {
      {"0|", "+XBOXSTATE: 0|\r\nOK\r\n"},
      {"19|", "+XBOXSTATE: 19|" + payload.substr(0, 19) + "\r\nOK\r\n"},
      {"21|", "+XBOXSTATE: 21|" + payload + "X\r\nOK\r\n"},
      {"20| bad CRC", reply(bad_crc)},
  };
  for (const Edge& edge : edges) {
    Serial2.reset();
    answer_with(edge.wire);
    uint32_t e = x.pollErrors();
    TimedPoll p = timed_poll(x);
    expect_rejected(x, p.ok, e, edge.label);
    EXPECT(p.ms < XBOX_R4_POLL_TIMEOUT_MS, "%s: took %.2f ms; Modem.cpp should have returned ok at once", edge.label,
           p.ms);
  }
  section_done("0|, 19|, 21| and a bad CRC: Modem.cpp says ok, poll() rejects each, readings unchanged", f0);
}

/* State changes: dropping to scanning loads the neutral report, and hasNewReport() says so
 * (XboxControllerR4.h: "a controller report the library had not seen yet, or the neutral
 * report loaded because the controller disconnected"); reconnecting counts as a new report. */
static void case_disconnect_reconnect() {
  int f0 = g_failures;
  XboxControllerR4 x;
  connect(x);
  Serial2.reset();
  answer_with(reply(encode(make_state(XBOX_R4_SCANNING, XBOX_R4_BATTERY_UNKNOWN, 0x10))));
  bool ok = x.poll();
  EXPECT(ok && x.connectionState() == XBOX_R4_SCANNING && !x.isConnected() && x.hasNewReport() &&
             x.battery() == XBOX_R4_BATTERY_UNKNOWN,
         "ok=%d state=%u new=%d", ok, x.connectionState(), x.hasNewReport());
  EXPECT(is_neutral(x.xboxNotif), "fields not neutral after the controller dropped (LX=%u, A=%d)",
         x.xboxNotif.joyLHori, x.xboxNotif.btnA);
  section_done("dropping to scanning: poll() true, not connected, fields neutral, hasNewReport() flags the change",
               f0);

  /* still scanning: nothing changed, so no new report */
  f0 = g_failures;
  Serial2.reset();
  answer_with(reply(encode(make_state(XBOX_R4_SCANNING, XBOX_R4_BATTERY_UNKNOWN, 0x10))));
  ok = x.poll();
  EXPECT(ok && !x.hasNewReport() && is_neutral(x.xboxNotif), "ok=%d new=%d", ok, x.hasNewReport());
  section_done("still scanning: no new report, fields stay neutral", f0);

  /* back to connected with the same seq the old connection had: still a new report */
  f0 = g_failures;
  Serial2.reset();
  answer_with(reply(encode(make_state(XBOX_R4_CONNECTED, 49, 0x10))));
  ok = x.poll();
  EXPECT(ok && x.isConnected() && x.hasNewReport() && fields_match_make_state(x.xboxNotif),
         "ok=%d connected=%d new=%d", ok, x.isConnected(), x.hasNewReport());
  section_done("reconnecting with an unchanged seq still counts as a new report", f0);
}

int main() {
  case_correct_reply();
  case_timeouts();
  case_timeout_restored();
  case_stale();
  case_truncated();
  case_wrong_size_and_crc();
  case_disconnect_reconnect();
  return g_failures == 0 ? 0 : 1;
}

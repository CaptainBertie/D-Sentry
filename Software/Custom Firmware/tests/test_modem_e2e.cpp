/*
 * test_modem_e2e.cpp -- host test of the REAL RA4M1 WiFiS3 Modem.cpp (core
 * arduino:renesas_uno 1.6.0, LGPL-2.1, compiled from the installed core, not copied) against
 * the stub UART in tests/stubs/Arduino.h.
 *
 * It replays canned ESP32 replies to "AT+XBOXSTATE?" with the same modem calls the RA4M1
 * library's poll() makes:
 *   modem.timeout(50); modem.avoid_trim_results();
 *   modem.write(std::string("+XBOXSTATE:"), res, "%s", "AT+XBOXSTATE?\r\n");
 * and checks what modem.write() returns, how long it takes, and what bytes it sent. The
 * acceptance rule below is a copy of the library's; test_ra_library.cpp runs the library's own
 * poll() against the same kinds of reply.
 *
 * Adapted from the recon harnesses scratchpad/modemsim/sim.cpp and e2e_ra.cpp.
 *
 * SPDX-License-Identifier: MIT
 */
#include "Modem.h"
#include "../protocol/xbox_r4_protocol.h"

#include <chrono>
#include <cstdio>
#include <cstring>
#include <random>
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

static const char kPrompt[] = XBOX_R4_AT_STATE ":";  /* "+XBOXSTATE:" */
static const char kRequest[] = "AT" XBOX_R4_AT_STATE "?\r\n";
static const int kTimeoutMs = 50;                      /* docs/protocol.md rule 3 */

struct PollResult {
  bool ok;
  std::string res;
  double ms;
};

/* One poll with the modem calls XboxControllerR4::poll() makes (minus the drain, which some
 * cases add). avoid_trim_results() is one-shot: Modem.cpp turns trimming back on itself. */
static PollResult poll_once(int timeout_ms = kTimeoutMs) {
  PollResult r;
  modem.timeout(timeout_ms);
  modem.avoid_trim_results();
  auto t0 = std::chrono::steady_clock::now();
  r.ok = modem.write(std::string(kPrompt), r.res, "%s", kRequest);
  auto t1 = std::chrono::steady_clock::now();
  modem.timeout(MODEM_TIMEOUT);
  r.ms = std::chrono::duration<double, std::milli>(t1 - t0).count();
  return r;
}

/* The drain the RA4M1 library does before each poll (docs/protocol.md rule 4). */
static void drain_serial2() {
  while (Serial2.available()) Serial2.read();
}

/* A copy of the library's acceptance rule (XboxControllerR4.cpp poll()): write() ok,
 * exactly 20 bytes, CRC good. */
static bool accept(const PollResult& r, xbox_r4_state_t* s) {
  return r.ok && r.res.size() == XBOX_R4_STATE_LEN &&
         xbox_r4_decode((const uint8_t*)r.res.data(), r.res.size(), s);
}

static std::string encode(const xbox_r4_state_t& s) {
  uint8_t out[XBOX_R4_STATE_LEN];
  xbox_r4_encode(&s, out);
  return std::string((const char*)out, sizeof out);
}

static std::string reply(const std::string& payload) {
  return std::string("+XBOXSTATE: 20|") + payload + "\r\nOK\r\n";
}

/* A connected state whose 20 encoded bytes contain CR, LF, '|', "OK", NUL and 0xFF. */
static xbox_r4_state_t tricky_state(uint8_t seq) {
  xbox_r4_state_t s;
  const uint8_t rep[16] = {0x0D, 0x0A, 0x7C, 'O', 'K', 0x00, 0xFF, 0x0D,
                           0x0A, 0x7C, 0x0D, 0x0A, 'O', 'K', 0x00, 0xFF};
  memcpy(s.report, rep, 16);
  s.state = XBOX_R4_CONNECTED;
  s.battery = 0xFF;
  s.seq = seq;
  return s;
}

static std::string hex(const std::string& b) {
  std::string out;
  char tmp[4];
  for (unsigned char c : b) {
    snprintf(tmp, sizeof tmp, "%02X ", c);
    out += tmp;
  }
  if (!out.empty()) out.pop_back();
  return out;
}

static void start_case() { Serial2.reset(); }

/* The seq byte of a 20-byte frame, or -1 if res is not a full frame. */
static int seq_of(const std::string& res) {
  return res.size() == XBOX_R4_STATE_LEN ? (uint8_t)res[18] : -1;
}

/* (a) the documented 41-byte reply, and the exact 15 request bytes */
static void case_a() {
  int f0 = g_failures;
  start_case();
  xbox_r4_state_t want = tricky_state(0x2A);
  std::string payload = encode(want);
  std::string r = reply(payload);
  EXPECT(r.size() == 41, "reply is %zu bytes, docs say 41", r.size());
  Serial2.on_write = [&](const uint8_t*, size_t) { Serial2.feed(r); };
  PollResult p = poll_once();
  xbox_r4_state_t got;
  EXPECT(p.ok, "write() returned false on a correct reply");
  EXPECT(p.res.size() == 20, "res.size() = %zu", p.res.size());
  EXPECT(p.res == payload, "payload changed: %s", hex(p.res).c_str());
  EXPECT(accept(p, &got) && got.seq == 0x2A && memcmp(got.report, want.report, 16) == 0, "decode failed");
  EXPECT(Serial2.available() == 0, "%d bytes left in rx", Serial2.available());
  std::printf("info (a) correct reply: ok=%d len=%zu in %.2f ms; request sent: %s\n", p.ok, p.res.size(), p.ms,
              hex(Serial2.tx).c_str());
  section_done("(a) 41-byte reply with CR/LF/'|'/OK/NUL/0xFF in the payload: ok, 20 bytes, identical, CRC good", f0);

  /* the bytes the real Modem.cpp put on the wire, against the docs/protocol.md wire example */
  f0 = g_failures;
  const std::string want_tx("\x41\x54\x2B\x58\x42\x4F\x58\x53\x54\x41\x54\x45\x3F\x0D\x0A", 15);
  EXPECT(Serial2.tx == want_tx, "request bytes were %s", hex(Serial2.tx).c_str());
  section_done("request bytes are exactly 41 54 2B 58 42 4F 58 53 54 41 54 45 3F 0D 0A", f0);

  /* the same path over 200 random states, so every byte value appears in the payload */
  f0 = g_failures;
  std::mt19937 rng(41u);
  for (int n = 0; n < 200; n++) {
    start_case();
    xbox_r4_state_t s;
    for (int i = 0; i < 16; i++) s.report[i] = (uint8_t)(rng() & 0xFF);
    s.state = (uint8_t)(rng() % 6);
    s.battery = (uint8_t)(rng() & 0xFF);
    s.seq = (uint8_t)n;
    std::string pl = encode(s);
    Serial2.feed(reply(pl));
    PollResult q = poll_once();
    xbox_r4_state_t back;
    EXPECT(accept(q, &back) && q.res == pl, "random #%d: ok=%d len=%zu", n, q.ok, q.res.size());
  }
  section_done("(a) 200 random payloads (seed 41) all come back byte-identical", f0);
}

/* (b) the +HCIREAD form: no CRLF between the payload and OK */
static void case_b() {
  int f0 = g_failures;
  start_case();
  std::string payload = encode(tricky_state(0x2B));
  Serial2.feed(std::string("+XBOXSTATE: 20|") + payload + "OK\r\n");
  PollResult p = poll_once();
  xbox_r4_state_t got;
  EXPECT(p.ok && p.res == payload && accept(p, &got), "ok=%d len=%zu", p.ok, p.res.size());
  section_done("(b) HCIREAD form (payload then OK, no CRLF) also accepted", f0);
}

/* Runs a timeout case three times and checks every run fails no earlier than the timeout.
 * The fastest of the three must land within 50..60 ms; the others only get a loose bound so a
 * busy machine cannot make the test flaky. */
static void timeout_case(const char* label, const std::string& canned) {
  int f0 = g_failures;
  double ms[3];
  for (int i = 0; i < 3; i++) {
    start_case();
    Serial2.feed(canned);
    PollResult p = poll_once();
    ms[i] = p.ms;
    EXPECT(!p.ok, "%s: write() returned true", label);
    EXPECT(p.ms >= kTimeoutMs, "%s: returned after %.2f ms, before the %d ms timeout", label, p.ms, kTimeoutMs);
    EXPECT(p.ms < 1000.0, "%s: took %.2f ms", label, p.ms);
  }
  double fastest = ms[0];
  for (double m : ms) if (m < fastest) fastest = m;
  EXPECT(fastest <= 60.0, "%s: fastest of 3 runs took %.2f ms, want <= 60", label, fastest);
  std::printf("info %s: false after %.2f / %.2f / %.2f ms (timeout %d ms)\n", label, ms[0], ms[1], ms[2], kTimeoutMs);
  char name[160];
  snprintf(name, sizeof name, "%s -> false only when the 50 ms timeout expires (50..60 ms)", label);
  section_done(name, f0);
}

/* (e) stale replies */
static void case_e() {
  xbox_r4_state_t stale_s = tricky_state(7);
  xbox_r4_state_t fresh_s = tricky_state(8);
  xbox_r4_state_t next_s = tricky_state(9);
  stale_s.report[0] = 0x11;
  fresh_s.report[0] = 0x22;
  next_s.report[0] = 0x33;
  const std::string stale = encode(stale_s), fresh = encode(fresh_s), next = encode(next_s);

  /* e1: the late reply to a timed-out poll is waiting in rx when the next poll is sent. */
  int f0 = g_failures;
  start_case();
  Serial2.feed(reply(stale));
  std::string answer = reply(fresh);
  Serial2.on_write = [&](const uint8_t*, size_t) { Serial2.feed(answer); };
  PollResult p1 = poll_once();
  EXPECT(p1.ok && p1.res == stale, "first poll without drain: ok=%d seq=%d", p1.ok, seq_of(p1.res));
  EXPECT(Serial2.available() == 41, "fresh reply should still be queued, rx has %d bytes", Serial2.available());
  answer = reply(next);
  PollResult p2 = poll_once();
  EXPECT(p2.ok && p2.res == fresh, "second poll without drain should return the previous poll's reply");
  std::printf("info (e) without drain: poll 1 returned the STALE reply (seq %d, ok=%d); poll 2 returned poll 1's "
              "reply (seq %d). The lag of one poll persists.\n",
              seq_of(p1.res), p1.ok, seq_of(p2.res));
  section_done("(e) without drain, write() returns the stale reply (same prompt) and lags one poll", f0);

  /* e2: drain first, as docs/protocol.md rule 4 says. */
  f0 = g_failures;
  start_case();
  Serial2.feed(reply(stale));
  answer = reply(fresh);
  Serial2.on_write = [&](const uint8_t*, size_t) { Serial2.feed(answer); };
  drain_serial2();
  PollResult p3 = poll_once();
  xbox_r4_state_t got;
  EXPECT(accept(p3, &got) && p3.res == fresh && got.seq == 8, "drained poll: ok=%d len=%zu", p3.ok, p3.res.size());
  EXPECT(Serial2.available() == 0, "rx not empty after drained poll");
  section_done("(e) drain-before-poll removes the stale reply; the poll gets the fresh one", f0);

  /* e3: a late reply that lands AFTER the drain is still matched first. Drain narrows the
   * window, it does not close it. The UART delivers bytes in order and chAT answers requests
   * in order, so the frame returned is one poll late but never older than a frame accepted
   * before it; the next drained poll catches up. */
  f0 = g_failures;
  start_case();
  Serial2.on_write = [&](const uint8_t*, size_t) {
    Serial2.feed(reply(stale));
    Serial2.feed(reply(fresh));
  };
  drain_serial2();
  PollResult p4 = poll_once();
  EXPECT(p4.ok && p4.res == stale, "late-after-drain: expected the stale frame");
  std::printf("info (e) residual: a late reply that lands after the drain is returned one poll late (seq %d). "
              "It is never older than a frame accepted earlier; the next drained poll catches up.\n",
              seq_of(p4.res));
  section_done("(e) residual risk recorded: a late reply arriving after the drain is still matched", f0);
}

/* e4: only the TAIL of a late frame lands after the drain. At 115200 baud a byte takes about
 * 87 us, so the library's drain loop usually stops part-way through a frame that is still
 * arriving. Every split point of a stale frame is tried. Half the stale payloads are plain
 * random bytes; the other half also get '+', ':', 'O', 'K', '|', CR, LF and space planted in
 * them, the bytes that steer Modem.cpp's parser. The rule under test is the library's: such a
 * poll may fail, but it must never accept 20 bytes other than the fresh frame.
 * A failed poll costs the full timeout (50 ms in the library). This loop uses a 5 ms timeout
 * to stay fast: every byte is queued before the parse starts, so the timeout only decides how
 * long a failing poll waits, not whether a poll succeeds. */
static void case_e4() {
  int f0 = g_failures;
  const int kFrames = 16;
  const int kShortTimeoutMs = 5;
  const char specials[] = {'+', ':', 'O', 'K', '|', '\r', '\n', ' '};
  std::mt19937 rng(44u);
  int total = 0, fresh_ok = 0, failed = 0, wrong = 0;
  for (int n = 0; n < kFrames; n++) {
    xbox_r4_state_t a, b;
    for (int i = 0; i < 16; i++) {
      a.report[i] = (uint8_t)(rng() & 0xFF);
      b.report[i] = (uint8_t)(rng() & 0xFF);
    }
    if (n % 2 == 1) {
      for (int j = 0; j < 6; j++) a.report[rng() % 16] = (uint8_t)specials[rng() % sizeof specials];
    }
    a.state = b.state = XBOX_R4_CONNECTED;
    a.battery = b.battery = 50;
    a.seq = (uint8_t)(2 * n);
    b.seq = (uint8_t)(2 * n + 1);
    const std::string stale_frame = reply(encode(a));
    const std::string fresh = encode(b);
    const std::string fresh_frame = reply(fresh);
    for (size_t k = 1; k < stale_frame.size(); k++) {
      start_case();
      const std::string tail = stale_frame.substr(k);
      Serial2.on_write = [&](const uint8_t*, size_t) {
        Serial2.feed(tail);
        Serial2.feed(fresh_frame);
      };
      drain_serial2();
      PollResult p = poll_once(kShortTimeoutMs);
      xbox_r4_state_t got;
      total++;
      if (!accept(p, &got)) {
        failed++;
      } else if (p.res == fresh) {
        fresh_ok++;
      } else {
        wrong++;
        EXPECT(false, "frame %d split at %zu: accepted bytes that are not the fresh frame: %s", n, k,
               hex(p.res).c_str());
      }
    }
  }
  EXPECT(wrong == 0, "%d polls accepted a wrong frame", wrong);
  EXPECT(fresh_ok > 0, "no poll returned the fresh frame; the case is not exercising anything");
  std::printf("info (e4) tail of a late frame after the drain, %d split points over %d stale frames (half seeded "
              "with parser-steering bytes): %d returned the fresh frame, %d failed (each such poll costs the full "
              "timeout, 50 ms in the library), %d accepted any other frame\n",
              total, kFrames, fresh_ok, failed, wrong);
  section_done("(e4) a late frame's tail before the fresh reply never makes the poll accept a wrong frame", f0);
}

/* (f) a sized reply that does not deliver 20 good bytes is never accepted */
static void case_f() {
  xbox_r4_state_t s = tricky_state(0x30);
  std::string payload = encode(s);

  /* f1: 20| announced, 15 bytes arrive, then nothing */
  int f0 = g_failures;
  start_case();
  Serial2.feed(std::string("+XBOXSTATE: 20|") + payload.substr(0, 15));
  PollResult p = poll_once();
  xbox_r4_state_t got;
  EXPECT(!p.ok, "truncated reply returned ok");
  EXPECT(!accept(p, &got), "truncated reply accepted");
  std::printf("info (f1) 20| then 15 bytes then silence: ok=%d len=%zu after %.2f ms\n", p.ok, p.res.size(), p.ms);
  section_done("(f1) 20| with only 15 bytes: write() false at timeout, size check also fails", f0);

  /* f2: 15 bytes lost-in-the-middle, then the normal trailer: the sized read swallows
   * "\r\nOK\r" as payload, so res has 20 bytes but write() still returns false. */
  f0 = g_failures;
  start_case();
  Serial2.feed(std::string("+XBOXSTATE: 20|") + payload.substr(0, 15) + "\r\nOK\r\n");
  p = poll_once();
  EXPECT(!p.ok, "truncated reply with trailer returned ok");
  EXPECT(p.res.size() == 20, "expected the trailer to be swallowed into a 20-byte res, got %zu", p.res.size());
  EXPECT(!xbox_r4_decode((const uint8_t*)p.res.data(), p.res.size(), &got), "CRC accepted the swallowed trailer");
  EXPECT(!accept(p, &got), "accepted");
  std::printf("info (f2) 20| then 15 bytes then \\r\\nOK\\r\\n: ok=%d len=%zu after %.2f ms -- res.size()==20 here, "
              "so the library must check write()'s result and the CRC, not the size alone\n",
              p.ok, p.res.size(), p.ms);
  section_done("(f2) truncated payload + trailer: res is 20 bytes but write() false and CRC bad", f0);

  /* f3: a stale sized reply to ANOTHER command leaves Modem.cpp's sized_read_count at 5, so
   * the next sized read stops 5 bytes early (buf_read initialises the counter once per call). */
  f0 = g_failures;
  start_case();
  Serial2.feed(std::string("+PREFGET: 5|abcde\r\nOK\r\n"));
  /* choose a payload whose last five bytes contain no 'O', so the parser stays in Res */
  xbox_r4_state_t s3 = s;
  s3.report[15] = 0x00;
  for (int seq = 0; seq < 256; seq++) {
    s3.seq = (uint8_t)seq;
    std::string pl = encode(s3);
    if (pl.substr(15).find('O') == std::string::npos) break;
  }
  std::string pl3 = encode(s3);
  EXPECT(pl3.substr(15).find('O') == std::string::npos, "test precondition: no 'O' in payload bytes 15..19");
  Serial2.feed(reply(pl3));
  p = poll_once();
  EXPECT(!accept(p, &got), "a truncated frame was accepted");
  std::printf("info (f3) stale '+PREFGET: 5|abcde' before our reply: ok=%d len=%zu after %.2f ms -- Modem.cpp "
              "stops the sized read 5 bytes early. With no 'O' in payload bytes 15..19 write() returns true "
              "with 15 bytes, otherwise false at the timeout; the size check rejects the first, the result "
              "check the second\n",
              p.ok, p.res.size(), p.ms);
  EXPECT(p.ok && p.res.size() == 15, "documented behaviour changed: ok=%d len=%zu", p.ok, p.res.size());
  section_done("(f3) stale sized reply from another command truncates ours to 15 bytes; size check rejects", f0);

  f0 = g_failures;
  start_case();
  Serial2.feed(std::string("+PREFGET: 5|abcde\r\nOK\r\n"));
  Serial2.on_write = [&](const uint8_t*, size_t) { Serial2.feed(reply(pl3)); };
  drain_serial2();
  p = poll_once();
  EXPECT(accept(p, &got) && p.res == pl3, "drained poll after stale sized reply: ok=%d len=%zu", p.ok, p.res.size());
  section_done("(f3) with drain-before-poll the same sequence yields the full 20 bytes", f0);

  /* f4: wrong announced sizes and a bad CRC. Modem.cpp copies whatever size is announced and
   * returns ok; only the library's size and CRC check stands between these and the state. */
  std::string bad_crc = payload;
  bad_crc[19] = (char)(bad_crc[19] ^ 0x01);
  struct Edge {
    const char* label;
    std::string wire;
  };
  const Edge edges[] = {
      {"0|", "+XBOXSTATE: 0|\r\nOK\r\n"},
      {"19| + 19 bytes", "+XBOXSTATE: 19|" + payload.substr(0, 19) + "\r\nOK\r\n"},
      {"21| + 21 bytes", "+XBOXSTATE: 21|" + payload + "X\r\nOK\r\n"},
      {"20| + bad CRC", reply(bad_crc)},
  };
  f0 = g_failures;
  for (const Edge& e : edges) {
    start_case();
    Serial2.feed(e.wire);
    p = poll_once();
    EXPECT(!accept(p, &got), "%s was accepted", e.label);
    std::printf("info (f4) %s: ok=%d len=%zu after %.2f ms\n", e.label, p.ok, p.res.size(), p.ms);
  }
  section_done("(f4) 0|, 19|, 21| and a bad CRC are all rejected by the size/CRC check", f0);

  /* f5: a stale sized reply LONGER than ours leaves sized_read_count past 20, so the counter
   * never equals 20 again and our read runs on to the timeout. Draining first avoids it. */
  f0 = g_failures;
  const std::string long_stale = "+PREFGET: 30|" + std::string(30, 'p') + "\r\nOK\r\n";
  start_case();
  Serial2.feed(long_stale);
  Serial2.feed(reply(payload));
  p = poll_once();
  EXPECT(!accept(p, &got), "accepted after a 30-byte stale sized reply");
  EXPECT(p.ms >= kTimeoutMs, "expected the read to run to the timeout, returned after %.2f ms", p.ms);
  std::printf("info (f5) stale '+PREFGET: 30|<30 B>' before our reply, no drain: ok=%d len=%zu after %.2f ms -- "
              "costs one full timeout\n",
              p.ok, p.res.size(), p.ms);
  start_case();
  Serial2.feed(long_stale);
  Serial2.on_write = [&](const uint8_t*, size_t) { Serial2.feed(reply(payload)); };
  drain_serial2();
  p = poll_once();
  EXPECT(accept(p, &got) && p.res == payload, "drained poll after a 30-byte stale reply: ok=%d len=%zu", p.ok,
         p.res.size());
  section_done("(f5) a longer stale sized reply makes the undrained poll time out; the drained poll is fine", f0);
}

int main() {
  case_a();
  case_b();
  timeout_case("(c) bare OK\\r\\n", "OK\r\n");
  /* A bare ERROR times out for two reasons: RESULT_ERROR is "OK\r\n" (Modem.cpp:7), and a
   * bare result code with no prompt sets restart (Modem.cpp:243-255), so fixing line 7 alone
   * would not change this case. "Prompt then ERROR" times out only because of line 7. */
  timeout_case("(d) ERROR\\r\\n", "ERROR\r\n");
  timeout_case("(d) prompt then ERROR", "+XBOXSTATE: \r\nERROR\r\n");
  case_e();
  case_e4();
  case_f();
  return g_failures == 0 ? 0 : 1;
}

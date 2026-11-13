// test_asyncat.cpp -- AsyncAt, the byte-fed AT reply parser (docs/architecture.md 4.2).
//
// The reply bytes below are the exact shapes the real chAT server emits (captured in
// research-ra4m1-link.md section 1, and re-proven against the real parser.cpp/server.cpp by
// tests/chat/test_controllink_chat.cpp). Every shape is fed whole, one byte at a time and in
// seeded random splits.
//
// Key safety property (test stale_line_never_completes_a_newer_request): a reply, or part of
// one, that belongs to an older request never completes the current request, so an old
// controller frame can never be taken for a fresh one.
//
// SPDX-License-Identifier: MIT
#include <random>
#include <string>
#include <vector>

#include "AsyncAt.h"
#include "testkit.h"
#include "../xbox_r4_protocol.h"   // the sketch's symlink, found through -I src

namespace {

constexpr uint32_t kTimeout = 20000;

std::string xboxPayload() {
  // CR, LF, '|', "OK", NUL and 0xFF inside the payload: a sized read must not care.
  const uint8_t rep[XBOX_R4_REPORT_LEN] = {0x0D, 0x0A, 0x7C, 'O', 'K', 0x00, 0xFF, 0x0D,
                                           0x0A, 0x7C, 'O', 'K', 0x0D, 0x0A, 0x00, 0xFF};
  xbox_r4_state_t s{};
  memcpy(s.report, rep, sizeof rep);
  s.state = XBOX_R4_CONNECTED;
  s.battery = 64;
  s.seq = 0x7C;
  uint8_t out[XBOX_R4_STATE_LEN];
  xbox_r4_encode(&s, out);
  return std::string((const char*)out, sizeof out);
}

struct Shape {
  const char* name;
  const char* prompt;
  std::string wire;
  AsyncAt::Result want;
  std::string payload;
};

std::vector<Shape> shapes() {
  const std::string x = xboxPayload();
  const std::string scan = "aa:bb:cc:dd:ee:ff,964,-60,0600030080,,Xbox Wireless Controller,3;\r\nOK\r\n+XBOXSTATE: 20|";
  return {
      {"xboxstate", "+XBOXSTATE", "+XBOXSTATE: 20|" + x + "\r\nOK\r\n", AsyncAt::OK, x},
      {"version", "+XBOXVERSION", "+XBOXVERSION: 1,0.6.99\r\nOK\r\n", AsyncAt::OK, "1,0.6.99"},
      {"begin", "+XBOXBEGIN", "+XBOXBEGIN: 0\r\nOK\r\n", AsyncAt::OK, "0"},
      {"scan", "+XBOXSCAN", "+XBOXSCAN: " + std::to_string(scan.size()) + "|" + scan + "\r\nOK\r\n",
       AsyncAt::OK, scan},
      {"softreset", "+SOFTRESETWIFI", "+SOFTRESETWIFI: \r\nOK\r\n", AsyncAt::OK, ""},
      {"prompt-error", "+HCIBEGIN", "+HCIBEGIN: ERROR\r\n", AsyncAt::ERR_PROMPT, ""},
      {"sized-no-crlf", "+HCIREAD", std::string("+HCIREAD: 4|\x04\x0E\x0D\x0A", 16) + "OK\r\n",
       AsyncAt::OK, std::string("\x04\x0E\x0D\x0A", 4)},
      {"wrong-mode", "+XBOXSTATE", "ERROR\r\n", AsyncAt::ERR_BARE, ""},
      {"bare-ok-for-prompt", "+DSCAPS", "OK\r\n", AsyncAt::ERR_BARE, ""},
      {"ping-ok", nullptr, "OK\r\n", AsyncAt::OK, ""},
      {"ping-error", nullptr, "ERROR\r\n", AsyncAt::ERR_BARE, ""},
      {"dscaps", "+DSCAPS", "+DSCAPS: 1,15\r\nOK\r\n", AsyncAt::OK, "1,15"},
      {"dscaps-legacy", "+DSCAPS", "ERROR\r\n", AsyncAt::ERR_BARE, ""},
      {"dss", "+DSS", "+DSS: 1,92,4\r\nOK\r\n", AsyncAt::OK, "1,92,4"},
      {"dst", "+DST", "+DST: 1\r\nOK\r\n", AsyncAt::OK, "1"},
      {"no-space", "+DST", "+DST:0\r\nOK\r\n", AsyncAt::OK, "0"},
      {"prompt-then-error", "+DSS", "+DSS: 9\r\nERROR\r\n", AsyncAt::ERR_FINAL, "9"},
  };
}

// Feeds `wire` in chunks given by `next`, checking after each chunk like loop() does.
template <typename F>
AsyncAt::Result feedAll(AsyncAt& at, const std::string& wire, F next, size_t& unread) {
  size_t i = 0;
  while (i < wire.size()) {
    size_t n = next();
    for (size_t k = 0; k < n && i < wire.size(); k++) at.feed((uint8_t)wire[i++]);
    AsyncAt::Result r = at.check(1);
    if (r != AsyncAt::PENDING) {
      unread = wire.size() - i;
      return r;
    }
  }
  unread = 0;
  return at.check(1);
}

}  // namespace

TEST(every_reply_shape_whole_byte_by_byte_and_random_splits) {
  std::mt19937 rng(20261103);
  for (const Shape& s : shapes()) {
    for (int mode = 0; mode < 3; mode++) {
      const int reps = mode == 2 ? 200 : 1;
      for (int rep = 0; rep < reps; rep++) {
        uint8_t buf[256];
        AsyncAt at(buf, sizeof buf);
        CHECK(at.start(s.prompt, 0, kTimeout));
        size_t unread = 0;
        AsyncAt::Result r = feedAll(
            at, s.wire,
            [&]() -> size_t { return mode == 0 ? s.wire.size() : mode == 1 ? 1 : 1 + rng() % 9; },
            unread);
        if (!CHECK_EQ(r, s.want)) std::printf("  shape %s mode %d\n", s.name, mode);
        CHECK_EQ(unread, (size_t)0);   // completes on the last byte, not before
        if (s.want == AsyncAt::OK || s.want == AsyncAt::ERR_FINAL)
          CHECK_EQ(std::string((const char*)at.data(), at.length()), s.payload);
        CHECK_EQ(at.check(2), AsyncAt::IDLE);   // the outcome is reported exactly once
      }
    }
  }
}

TEST(timeout_fires_at_the_deadline_not_before) {
  uint8_t buf[64];
  AsyncAt at(buf, sizeof buf);
  CHECK(at.start("+XBOXSTATE", 1000, kTimeout));
  CHECK(!at.start("+DSS", 1000, kTimeout));   // one request at a time
  CHECK_EQ(at.check(1000 + kTimeout - 1), AsyncAt::PENDING);
  CHECK_EQ(at.check(1000 + kTimeout), AsyncAt::TIMEOUT);
  CHECK_EQ(at.timeouts(), 1u);
  CHECK(!at.busy());
  CHECK_EQ(at.check(1000 + kTimeout + 1), AsyncAt::IDLE);
}

TEST(timeout_is_wrap_safe) {
  uint8_t buf[64];
  AsyncAt at(buf, sizeof buf);
  const uint32_t t0 = UINT32_MAX - 100;
  CHECK(at.start(nullptr, t0, 1000));
  CHECK_EQ(at.check(t0 + 50), AsyncAt::PENDING);
  CHECK_EQ(at.check((uint32_t)(t0 + 999)), AsyncAt::PENDING);   // wrapped to 898
  CHECK_EQ(at.check((uint32_t)(t0 + 1000)), AsyncAt::TIMEOUT);
}

TEST(extend_moves_the_deadline_later_never_earlier) {
  uint8_t buf[64];
  AsyncAt at(buf, sizeof buf);
  CHECK(at.start("+DST", 0, 1000));
  at.extend(100, 100);                    // would be earlier (200 < 1000): ignored
  CHECK_EQ(at.check(999), AsyncAt::PENDING);
  at.extend(900, 1000);                   // now 1900
  CHECK_EQ(at.check(1000), AsyncAt::PENDING);
  CHECK_EQ(at.check(1899), AsyncAt::PENDING);
  CHECK_EQ(at.check(1900), AsyncAt::TIMEOUT);
  at.extend(2000, 1000);                  // idle: no effect, nothing revived
  CHECK_EQ(at.check(2001), AsyncAt::IDLE);
}

TEST(extend_does_not_undo_a_decided_result) {
  uint8_t buf[64];
  AsyncAt at(buf, sizeof buf);
  CHECK(at.start("+DST", 0, 1000));
  for (char c : std::string("+DST: 1\r\nOK\r\n")) at.feed((uint8_t)c);
  at.extend(5000, 1000);
  CHECK_EQ(at.check(5000), AsyncAt::OK);
}

// KEY SAFETY PROPERTY: an older request's reply (half received when the new request starts, or
// arriving late) never completes the new one.
TEST(stale_line_never_completes_a_newer_request) {
  uint8_t buf[64];
  AsyncAt at(buf, sizeof buf);
  const std::string old = "+XBOXSTATE: 20|" + xboxPayload() + "\r\nOK\r\n";
  // An older poll timed out; its reply starts arriving...
  CHECK(at.start("+XBOXSTATE", 0, 10));
  CHECK_EQ(at.check(10), AsyncAt::TIMEOUT);
  for (size_t i = 0; i < 20; i++) at.feed((uint8_t)old[i]);
  // ...the next poll starts in the middle of it.
  CHECK(at.start("+XBOXSTATE", 20, kTimeout));
  for (size_t i = 20; i < old.size(); i++) at.feed((uint8_t)old[i]);
  CHECK_EQ(at.check(30), AsyncAt::PENDING);   // the old line was skipped as a whole
  CHECK_EQ(at.staleReplies(), 1u);
  // The new request's own reply completes it.
  std::string fresh = "+XBOXSTATE: 20|" + xboxPayload() + "\r\nOK\r\n";
  for (char c : fresh) at.feed((uint8_t)c);
  CHECK_EQ(at.check(40), AsyncAt::OK);
  CHECK_EQ(at.length(), (size_t)XBOX_R4_STATE_LEN);
}

TEST(a_name_half_received_at_start_cannot_match) {
  uint8_t buf[64];
  AsyncAt at(buf, sizeof buf);
  CHECK(at.start("+DSS", 0, 10));
  for (char c : std::string("+DS")) at.feed((uint8_t)c);   // request A's reply begins (matching)...
  CHECK_EQ(at.check(10), AsyncAt::TIMEOUT);                 // ...A gives up...
  CHECK(at.start("+DSS", 20, kTimeout));                    // ...and B, same prompt, starts
  for (char c : std::string("S: 0,1,2\r\nOK\r\n")) at.feed((uint8_t)c);
  CHECK_EQ(at.check(21), AsyncAt::PENDING);                 // A's tail must not answer B
  CHECK_EQ(at.staleReplies(), 1u);
  for (char c : std::string("+DSS: 3,4,5\r\nOK\r\n")) at.feed((uint8_t)c);
  CHECK_EQ(at.check(22), AsyncAt::OK);
  CHECK_EQ(std::string((const char*)at.data(), at.length()), std::string("3,4,5"));
}

TEST(foreign_prompts_are_stale_and_prefixes_do_not_match) {
  uint8_t buf[64];
  AsyncAt at(buf, sizeof buf);
  CHECK(at.start("+XBOXSTATE", 0, kTimeout));
  const std::string foreign = "+XBOXSTATEX: 5\r\nOK\r\n+XBOX: 1\r\nOK\r\n+DSS: 1,2,3\r\nOK\r\n";
  for (char c : foreign) at.feed((uint8_t)c);
  CHECK_EQ(at.check(1), AsyncAt::PENDING);
  CHECK_EQ(at.staleReplies(), 3u);
}

// No fixed-size name buffer any more: a stale reply with a very long name and a sized payload
// that contains a whole fake "+XBOXSTATE" reply is still skipped as one structure.
TEST(long_prompt_names_are_parsed_structurally) {
  uint8_t buf[64];
  AsyncAt at(buf, sizeof buf);
  CHECK(at.start("+XBOXSTATE", 0, kTimeout));
  const std::string fake = "\r\nOK\r\n+XBOXSTATE: 20|" + xboxPayload() + "\r\nOK\r\n";
  const std::string longName = "+ABCDEFGHIJKLMNOPQRSTUVWXYZ_0123456789_LONGER_THAN_ANY_BUFFER";
  const std::string stale = longName + ": " + std::to_string(fake.size()) + "|" + fake + "\r\nOK\r\n";
  for (char c : stale) at.feed((uint8_t)c);
  CHECK_EQ(at.check(1), AsyncAt::PENDING);
  CHECK_EQ(at.staleReplies(), 1u);
}

TEST(garbage_before_a_reply_is_dropped) {
  uint8_t buf[64];
  AsyncAt at(buf, sizeof buf);
  CHECK(at.start("+DSS", 0, kTimeout));
  const std::string junk("xyz\r\n\x00\xff\x7f|||: ", 15);
  for (char c : junk) at.feed((uint8_t)c);
  for (char c : std::string("+DSS: 0,7,2\r\nOK\r\n")) at.feed((uint8_t)c);
  CHECK_EQ(at.check(1), AsyncAt::OK);
  CHECK_EQ(std::string((const char*)at.data(), at.length()), std::string("0,7,2"));
  CHECK(at.junkLines() > 0u);
}

TEST(garbage_inside_a_line_rehunts_from_the_next_line_start) {
  uint8_t buf[64];
  AsyncAt at(buf, sizeof buf);
  CHECK(at.start("+DST", 0, kTimeout));
  // A corrupted name ('\x01'), then a corrupted final ("OX"), then the real reply.
  const std::string wire("+DS\x01T: 1\r\nOX\r\n+DST: 1\r\nOK\r\n");
  for (char c : wire) at.feed((uint8_t)c);
  CHECK_EQ(at.check(1), AsyncAt::OK);
  CHECK_EQ(std::string((const char*)at.data(), at.length()), std::string("1"));
}

TEST(payload_larger_than_the_buffer_fails) {
  uint8_t buf[8];
  AsyncAt at(buf, sizeof buf);
  // A sized payload larger than the buffer is not read as a count (see the next test); its
  // final then arrives as a bare final: the request fails, fast.
  CHECK(at.start("+XBOXSTATE", 0, kTimeout));
  for (char c : "+XBOXSTATE: 20|" + xboxPayload() + "\r\nOK\r\n") at.feed((uint8_t)c);
  CHECK_EQ(at.check(1), AsyncAt::ERR_BARE);
  // A text payload that does not fit is OVERFLOW_.
  CHECK(at.start("+XBOXVERSION", 2, kTimeout));
  for (char c : std::string("+XBOXVERSION: 123456789\r\nOK\r\n")) at.feed((uint8_t)c);
  CHECK_EQ(at.check(3), AsyncAt::OVERFLOW_);
}

// A garbage digit inside a real header ("+XBOXSTATE: 920|") must not make the parser read 920
// bytes: the replies after it would all be swallowed and the link would look dead.
TEST(a_corrupted_size_never_swallows_the_replies_after_it) {
  uint8_t buf[LINK_RX_PAYLOAD_BYTES];
  AsyncAt at(buf, sizeof buf);
  CHECK(at.start("+XBOXSTATE", 0, kTimeout));
  for (char c : "+XBOXSTATE: 920|" + xboxPayload() + "\r\nOK\r\n") at.feed((uint8_t)c);
  CHECK(at.check(1) != AsyncAt::OK);        // the corrupted reply is not accepted
  CHECK(!at.busy());
  // The very next request and its reply work.
  CHECK(at.start(nullptr, 2, kTimeout));
  for (char c : std::string("OK\r\n")) at.feed((uint8_t)c);
  CHECK_EQ(at.check(3), AsyncAt::OK);
  CHECK(at.start("+XBOXSTATE", 4, kTimeout));
  for (char c : "+XBOXSTATE: 20|" + xboxPayload() + "\r\nOK\r\n") at.feed((uint8_t)c);
  CHECK_EQ(at.check(5), AsyncAt::OK);
  CHECK_EQ(at.length(), (size_t)XBOX_R4_STATE_LEN);
}

TEST(error_word_must_be_the_whole_payload) {
  uint8_t buf[64];
  AsyncAt at(buf, sizeof buf);
  CHECK(at.start("+X", 0, kTimeout));
  for (char c : std::string("+X: ERRORS\r\n")) at.feed((uint8_t)c);
  CHECK_EQ(at.check(1), AsyncAt::PENDING);   // text "ERRORS" is a payload, the final follows
  for (char c : std::string("OK\r\n")) at.feed((uint8_t)c);
  CHECK_EQ(at.check(2), AsyncAt::OK);
  CHECK_EQ(std::string((const char*)at.data(), at.length()), std::string("ERRORS"));
}

TEST(a_bare_final_with_no_request_is_stale) {
  uint8_t buf[64];
  AsyncAt at(buf, sizeof buf);
  for (char c : std::string("OK\r\nERROR\r\n")) at.feed((uint8_t)c);
  CHECK_EQ(at.staleReplies(), 2u);
  CHECK_EQ(at.check(0), AsyncAt::IDLE);
}

TEST(reset_forgets_everything) {
  uint8_t buf[64];
  AsyncAt at(buf, sizeof buf);
  CHECK(at.start("+DSS", 0, kTimeout));
  for (char c : std::string("+DSS: 1,")) at.feed((uint8_t)c);
  at.reset();
  CHECK(!at.busy());
  CHECK_EQ(at.bytes(), 0u);
  CHECK(at.start("+DSS", 0, kTimeout));
  for (char c : std::string("2,3\r\n")) at.feed((uint8_t)c);   // tail of the forgotten line
  CHECK_EQ(at.check(1), AsyncAt::PENDING);                       // is noise, not a reply
  for (char c : std::string("+DSS: 1,2,3\r\nOK\r\n")) at.feed((uint8_t)c);
  CHECK_EQ(at.check(2), AsyncAt::OK);
  CHECK_EQ(std::string((const char*)at.data(), at.length()), std::string("1,2,3"));
  // A bare final while a prompt is expected fails fast instead of waiting for the timeout.
  CHECK(at.start("+DSS", 3, kTimeout));
  for (char c : std::string("OK\r\n")) at.feed((uint8_t)c);
  CHECK_EQ(at.check(4), AsyncAt::ERR_BARE);
}

TEST(result_names) {
  CHECK_EQ(std::string(AsyncAt::resultName(AsyncAt::OK)), std::string("ok"));
  CHECK_EQ(std::string(AsyncAt::resultName(AsyncAt::TIMEOUT)), std::string("timeout"));
}

TESTKIT_MAIN()

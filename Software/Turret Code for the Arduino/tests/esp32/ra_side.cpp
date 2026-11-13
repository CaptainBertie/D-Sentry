// ra_side.cpp -- the RA4M1's side of the AT link for test_dsentry_at: the real AsyncAt reply
// parser from firmware-ra4m1/dsentry/src, in its own translation unit so the RA4M1's config.h
// and the ESP32's dsentry_config.h (two copies of the same names) never meet.
//
// SPDX-License-Identifier: MIT
#include <string>

#include "AsyncAt.h"

// Feeds the bytes the ESP32 sent to a fresh AsyncAt that is waiting for `prompt`, then asks for
// the outcome. Returns the AsyncAt::Result as an int and the kept payload in *payload.
int ra_parse_reply(const char* prompt, const std::string& bytes, std::string* payload) {
  static uint8_t buf[LINK_RX_PAYLOAD_BYTES];
  AsyncAt at(buf, sizeof buf);
  const uint32_t t0 = 1000;
  const uint32_t timeout = AT_REPLY_TIMEOUT_MS * 1000u;
  if (!at.start(prompt, t0, timeout)) return -1;
  for (char c : bytes) at.feed((uint8_t)c);
  AsyncAt::Result r = at.check(t0 + 1);
  if (r == AsyncAt::PENDING) r = at.check(t0 + timeout + 1);
  payload->assign((const char*)at.data(), at.length());
  return (int)r;
}

int ra_result_ok() { return (int)AsyncAt::OK; }
int ra_result_err_bare() { return (int)AsyncAt::ERR_BARE; }

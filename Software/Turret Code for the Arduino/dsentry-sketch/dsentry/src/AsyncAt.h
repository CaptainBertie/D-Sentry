// AsyncAt.h -- non-blocking client parser for the chAT AT server that the ESP32-S3 bridge runs
// on the RA4M1's Serial2 (docs/architecture.md 4.2). No I/O of its own: ControlLink writes the
// request bytes through hal::linkWrite, hands every received byte to feed(), and asks check()
// once per loop(). One request is outstanding at a time.
//
// Ported from the research prototype (scratchpad research/ra4m1_link/async_at/AsyncAt.h),
// which passed every reply shape the real chAT emits (whole, byte by byte, 200 random splits),
// 6000 virtual-time polls without a timeout, and garbage / restart / stall / stale-reply fault
// injection. Changes since: numbers from config.h; the prompt name is matched as it arrives
// against the caller's string instead of being copied into a fixed buffer (no name length
// limit, no array sized by a bare number); extend() for a deadline counted from the
// request's last byte on the wire. tests/unit/test_asyncat.cpp and tests/chat/ re-run every
// research case against this code.
//
// Grammar it accepts (what chAT's server.cpp + the cmds_*.h handlers emit):
//   reply      := info? final | prompt_err
//   info       := '+' NAME ':' ' '? ( text | DIGITS '|' <DIGITS raw bytes> ) [ CRLF ]
//   final      := "OK" CRLF | "ERROR" CRLF
//   prompt_err := '+' NAME ": ERROR" CRLF       (a handler wrote the prompt, then ERROR)
//   text       := any bytes except CR/LF ('|' after a leading decimal makes it a sized read)
// The CRLF after a sized payload is optional (the +HCIREAD form goes straight to "OK").
// A sized header declaring more bytes than the caller's buffer holds is noise too: it can
// only come from a corrupted header, and reading it as a count would swallow the replies
// that follow (the request then ends ERR_BARE at its final, or TIMEOUT).
// Anything else is noise, dropped byte by byte until a '+', 'O' or 'E' can start a line.
//
// SPDX-License-Identifier: MIT
#pragma once
#include <stddef.h>
#include <stdint.h>
#include "../config.h"

class AsyncAt {
 public:
  enum Result : uint8_t {
    PENDING = 0,  // still waiting
    OK,           // the matching prompt line (if one was expected), then OK
    ERR_PROMPT,   // the matching "+NAME: ERROR"
    ERR_BARE,     // a bare ERROR (unknown command, wrong mode, stock firmware) or a bare final
                  // where a prompt was expected
    ERR_FINAL,    // the matching prompt line, then ERROR (payload kept)
    TIMEOUT,      // no completing final before the deadline
    OVERFLOW_,    // a text payload was larger than the caller's buffer
    IDLE          // nothing outstanding
  };

  // buf/cap: where the matching info payload is kept (LINK_RX_PAYLOAD_BYTES in ControlLink).
  AsyncAt(uint8_t* buf, size_t cap);

  // Begin waiting for the reply to a request the caller has just queued.
  // prompt: "+XBOXSTATE"-style name (with the '+'), or nullptr for a command answered by a bare
  // final ("AT"). The pointer is kept, not copied: pass a string that outlives the request
  // (the DS_AT_* / XBOX_R4_AT_* literals). timeout_us: AT_REPLY_TIMEOUT_MS plus the request's
  // own wire time. Returns false (and changes nothing) while another request is outstanding.
  bool start(const char* prompt, uint32_t now_us, uint32_t timeout_us);

  // Move the outstanding request's deadline to at least now_us + timeout_us (never earlier).
  // ControlLink calls it once the request's last byte has left the UART, so the reply
  // deadline counts from there however long the paced writer took. No effect when idle or
  // when the result is already decided.
  void extend(uint32_t now_us, uint32_t timeout_us);

  // Forget everything (outstanding request, half-parsed line, counters): ControlLink::begin.
  void reset();

  bool busy() const { return busy_; }
  size_t length() const { return len_; }        // payload bytes of the last OK/ERR_FINAL
  const uint8_t* data() const { return buf_; }

  // Call once per loop() after feeding. Returns the outcome exactly once, then IDLE.
  Result check(uint32_t now_us);

  // Every byte received on the link, in order.
  void feed(uint8_t c);

  uint32_t bytes() const { return bytes_; }          // bytes fed since construction
  uint32_t junkLines() const { return junk_; }       // noise / malformed bytes dropped
  uint32_t staleReplies() const { return stale_; }   // finals that were not ours
  uint32_t timeouts() const { return timeouts_; }

  static const char* resultName(Result r);

 private:
  enum State : uint8_t { LS, NAME, SPACE, HEAD, TEXT_CR, SIZED, AFTER_SIZED, AFTER_SIZED_CR, FINAL };

  void rehunt(uint8_t c);
  void keep(uint8_t c);
  void endTextLine();
  void finish(Result r, bool promptErr);
  void resetLine();

  static constexpr char kErrorWord[] = "ERROR";   // a text payload that is exactly this ends
                                                  // the reply ("+NAME: ERROR")

  uint8_t* buf_;
  size_t cap_;
  size_t len_ = 0;
  const char* want_ = nullptr;   // expected prompt, '+' included (caller's string)
  size_t widx_ = 0;              // characters of want_ matched by the name being received
  bool nameMatch_ = false;       // the name so far equals the start of want_
  char ehead_[sizeof kErrorWord - 1] = {};   // first characters of a text payload
  size_t hlen_ = 0;
  uint32_t digits_ = 0;
  bool allDigits_ = true;
  uint32_t sized_ = 0;       // raw bytes still to read in a sized payload
  size_t fidx_ = 0;          // index into the final being matched
  bool fok_ = false;         // matching "OK" (true) or "ERROR" (false)
  bool infoSeen_ = false;
  bool infoMatch_ = false;
  bool overflow_ = false;
  bool busy_ = false;
  Result result_ = PENDING;
  uint32_t t0_ = 0;
  uint32_t timeout_ = 0;
  State st_ = LS;
  uint32_t bytes_ = 0;
  uint32_t junk_ = 0;
  uint32_t stale_ = 0;
  uint32_t timeouts_ = 0;
};

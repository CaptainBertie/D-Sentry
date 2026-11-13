// AsyncAt.cpp -- see AsyncAt.h. The state machine is the research prototype's; only the name
// match (incremental, against the caller's string) and extend()/reset() are new.
//
// SPDX-License-Identifier: MIT
#include "AsyncAt.h"

#include <string.h>

namespace {
const char kOk[] = "OK\r\n";
const char kError[] = "ERROR\r\n";
}  // namespace

AsyncAt::AsyncAt(uint8_t* buf, size_t cap) : buf_(buf), cap_(cap) { resetLine(); }

void AsyncAt::reset() {
  len_ = 0;
  want_ = nullptr;
  widx_ = 0;
  nameMatch_ = false;
  hlen_ = 0;
  digits_ = 0;
  allDigits_ = true;
  fidx_ = 0;
  fok_ = false;
  busy_ = false;
  result_ = PENDING;
  t0_ = timeout_ = 0;
  st_ = LS;
  bytes_ = junk_ = stale_ = timeouts_ = 0;
  resetLine();
}

bool AsyncAt::start(const char* prompt, uint32_t now_us, uint32_t timeout_us) {
  if (busy_) return false;
  busy_ = true;
  want_ = (prompt && prompt[0]) ? prompt : nullptr;
  t0_ = now_us;
  timeout_ = timeout_us;
  len_ = 0;
  result_ = PENDING;
  infoMatch_ = false;  // a reply already arriving belongs to an older request
  nameMatch_ = false;  // and so does a name half received
  return true;
}

void AsyncAt::extend(uint32_t now_us, uint32_t timeout_us) {
  if (!busy_ || result_ != PENDING) return;
  const uint32_t oldDeadline = t0_ + timeout_;
  const uint32_t newDeadline = now_us + timeout_us;
  if ((int32_t)(newDeadline - oldDeadline) > 0) {
    t0_ = now_us;
    timeout_ = timeout_us;
  }
}

AsyncAt::Result AsyncAt::check(uint32_t now_us) {
  if (!busy_) return IDLE;
  if (result_ == PENDING && (uint32_t)(now_us - t0_) >= timeout_) {
    result_ = TIMEOUT;
    infoMatch_ = false;
    nameMatch_ = false;
  }
  if (result_ == PENDING) return PENDING;
  busy_ = false;
  Result r = result_;
  result_ = PENDING;
  if (r == TIMEOUT) timeouts_++;
  return r;
}

void AsyncAt::feed(uint8_t c) {
  bytes_++;
  switch (st_) {
    case LS:
      if (c == '+') {
        // The '+' is part of the prompt: compare the rest as it arrives.
        nameMatch_ = busy_ && result_ == PENDING && want_ && want_[0] == '+';
        widx_ = 1;
        st_ = NAME;
      } else if (c == 'O') {
        fidx_ = 1;
        fok_ = true;
        st_ = FINAL;
      } else if (c == 'E') {
        fidx_ = 1;
        fok_ = false;
        st_ = FINAL;
      } else if (c == '\r' || c == '\n') {
        // blank
      } else {
        junk_++;  // hunt: noise at a line start is dropped byte by byte
      }
      break;
    case NAME:
      if (c == ':') {
        infoSeen_ = true;
        infoMatch_ = nameMatch_ && busy_ && result_ == PENDING && want_[widx_] == 0;
        nameMatch_ = false;
        if (infoMatch_) len_ = 0;
        st_ = SPACE;
      } else if ((c >= 'A' && c <= 'Z') || (c >= '0' && c <= '9') || c == '_') {
        if (nameMatch_) {
          if (want_[widx_] == (char)c)
            widx_++;
          else
            nameMatch_ = false;   // includes a name longer than want_ (want_[widx_] == 0)
        }
      } else {
        rehunt(c);
      }
      break;
    case SPACE:
      st_ = HEAD;
      hlen_ = 0;
      digits_ = 0;
      allDigits_ = true;
      if (c == ' ') break;
      // no space after ':'
      [[fallthrough]];
    case HEAD:  // text so far; may still turn into "<len>|"
      if (c == '|' && allDigits_ && hlen_ > 0) {
        if (digits_ > cap_) {
          // Larger than any reply this client keeps, so never a reply to anything it sent: a
          // garbage digit in a real header ("+XBOXSTATE: 920|"). Consuming 920 bytes would
          // swallow every reply after it (the link would look dead for seconds); hunt instead.
          rehunt(c);
          break;
        }
        sized_ = digits_;
        if (infoMatch_) len_ = 0;  // never touch a finished result's buffer for a stale line
        st_ = sized_ ? SIZED : AFTER_SIZED;
      } else if (c == '\r') {
        st_ = TEXT_CR;
      } else if (c == '\n') {
        endTextLine();
      } else {
        if (c >= '0' && c <= '9' && allDigits_ && hlen_ < AT_SIZED_MAX_DIGITS)
          digits_ = digits_ * DEC_BASE + (uint32_t)(c - '0');
        else
          allDigits_ = false;
        hlen_++;
        keep(c);
        if (hlen_ <= sizeof ehead_) ehead_[hlen_ - 1] = (char)c;
      }
      break;
    case TEXT_CR:
      if (c == '\n') {
        endTextLine();
      } else {
        keep('\r');
        keep(c);
        hlen_++;
        hlen_++;
        allDigits_ = false;
        st_ = HEAD;
      }
      break;
    case SIZED:
      keep(c);
      if (--sized_ == 0) st_ = AFTER_SIZED;
      break;
    case AFTER_SIZED:  // CRLF, or straight into the final (+HCIREAD form)
      if (c == '\r') {
        st_ = AFTER_SIZED_CR;
      } else if (c == 'O') {
        fidx_ = 1;
        fok_ = true;
        st_ = FINAL;
      } else if (c == 'E') {
        fidx_ = 1;
        fok_ = false;
        st_ = FINAL;
      } else {
        rehunt(c);
      }
      break;
    case AFTER_SIZED_CR:
      if (c == '\n')
        st_ = LS;  // infoSeen_ stays set until the final
      else
        rehunt(c);
      break;
    case FINAL: {
      const char* tok = fok_ ? kOk : kError;
      size_t tlen = fok_ ? sizeof kOk - 1 : sizeof kError - 1;
      if ((char)c == tok[fidx_]) {
        if (++fidx_ == tlen) {
          finish(fok_ ? OK : ERR_FINAL, false);
          st_ = LS;
        }
      } else {
        rehunt(c);
      }
      break;
    }
  }
}

const char* AsyncAt::resultName(Result r) {
  switch (r) {
    case PENDING: return "pending";
    case OK: return "ok";
    case ERR_PROMPT: return "err-prompt";
    case ERR_BARE: return "err-bare";
    case ERR_FINAL: return "err-final";
    case TIMEOUT: return "timeout";
    case OVERFLOW_: return "overflow";
    case IDLE: return "idle";
  }
  return "?";
}

// Malformed: drop the line so far and look at this byte again as a possible line start.
void AsyncAt::rehunt(uint8_t c) {
  junk_++;
  resetLine();
  st_ = LS;
  if (c == '+' || c == 'O' || c == 'E') feed(c);
}

void AsyncAt::keep(uint8_t c) {
  if (!infoMatch_) return;
  if (len_ < cap_)
    buf_[len_++] = c;
  else
    overflow_ = true;
}

// A text info line ended. "+NAME: ERROR" is a final in its own right (chAT writes the prompt,
// then write_error()). Any other text line waits for its final.
void AsyncAt::endTextLine() {
  if (hlen_ == sizeof ehead_ && memcmp(ehead_, kErrorWord, sizeof ehead_) == 0) {
    finish(ERR_PROMPT, true);
  }
  st_ = LS;
}

// A final arrived (or a prompt_err). Decide whether it completes the outstanding request.
void AsyncAt::finish(Result r, bool promptErr) {
  bool pending = busy_ && result_ == PENDING;
  if (pending) {
    if (want_) {
      if (infoSeen_ && infoMatch_) {
        result_ = overflow_ ? OVERFLOW_ : r;
        if (promptErr) len_ = 0;
      } else if (!infoSeen_) {
        result_ = ERR_BARE;  // bare OK or ERROR while a prompt was expected
        len_ = 0;
      } else {
        stale_++;            // another command's reply: not ours
      }
    } else {
      if (!infoSeen_)
        result_ = (r == OK) ? OK : ERR_BARE;
      else
        stale_++;
    }
  } else {
    stale_++;
  }
  resetLine();
}

void AsyncAt::resetLine() {
  infoSeen_ = false;
  infoMatch_ = false;
  nameMatch_ = false;
  overflow_ = false;
  sized_ = 0;
}

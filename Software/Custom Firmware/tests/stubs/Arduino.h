/*
 * Arduino.h -- minimal host stand-in for the Arduino core, used only by the host tests.
 *
 * It provides just what the code under test touches:
 *   - millis(), Stream and UART for the RA4M1 WiFiS3 Modem.cpp (core arduino:renesas_uno 1.6.0);
 *   - a String class for XboxControllerNotificationParser (its toString() needs
 *     String(int), String(unsigned char), operator+ and +=).
 *
 * Adapted from the recon harness scratchpad/modemsim/stub/Arduino.h (UART with an rx queue
 * and a tx capture string). Added here: an on_write hook so a test can answer a request
 * only after the RA side has sent it (closer to a real UART than pre-filling rx), a delayed
 * feed (bytes that arrive only after some milliseconds), and the String class.
 *
 * SPDX-License-Identifier: MIT
 */
#pragma once
#include <cstdint>
#include <cstddef>
#include <cstring>
#include <cstdarg>
#include <cstdio>
#include <cstdlib>
#include <string>
#include <deque>
#include <chrono>
#include <functional>

/* Milliseconds since the first call, from a monotonic clock, like the Arduino millis(). */
inline unsigned long millis() {
  using namespace std::chrono;
  static const auto t0 = steady_clock::now();
  return (unsigned long)duration_cast<milliseconds>(steady_clock::now() - t0).count();
}

/* Modem.cpp only prints to a Stream when debugging is switched on; the tests never do. */
struct Stream {
  void print(const char* s) { fputs(s, stderr); }
  void print(char c) { fputc(c, stderr); }
  void print(unsigned v) { fprintf(stderr, "%u", v); }
  void print(int v) { fprintf(stderr, "%d", v); }
  void println(const char* s = "") { fprintf(stderr, "%s\n", s); }
  void println(int v) { fprintf(stderr, "%d\n", v); }
  size_t write(const uint8_t* b, size_t n) { return fwrite(b, 1, n, stderr); }
};

/* A fake serial port. rx holds bytes "arriving" from the ESP32; tx collects what the RA4M1
 * sent. If on_write is set, it is called after every write so a test can queue a reply that
 * only exists once the request has gone out. Bytes given to feed_after() join rx only once
 * millis() reaches their due time, the way a late reply lands while the RA4M1 is waiting. */
struct UART {
  std::deque<uint8_t> rx;
  std::string tx;
  std::function<void(const uint8_t*, size_t)> on_write;
  std::string later;              /* bytes not arrived yet */
  unsigned long later_due = 0;    /* millis() at which `later` arrives */

  UART() {}
  UART(int, int) {}
  void begin(int) {}
  void end() {}
  void flush() {}
  int available() {
    release_due();
    return (int)rx.size();
  }
  int read() {
    release_due();
    if (rx.empty()) return -1;
    int c = rx.front();
    rx.pop_front();
    return c;
  }
  size_t write(const uint8_t* b, size_t n) {
    tx.append((const char*)b, n);
    if (on_write) on_write(b, n);
    return n;
  }
  /* Test helper: make bytes arrive on rx. */
  void feed(const std::string& s) {
    for (unsigned char c : s) rx.push_back(c);
  }
  /* Test helper: make bytes arrive on rx delay_ms from now (one pending batch at a time). */
  void feed_after(unsigned long delay_ms, const std::string& s) {
    later += s;
    later_due = millis() + delay_ms;
  }
  /* Test helper: reset between cases. */
  void reset() {
    rx.clear();
    tx.clear();
    later.clear();
    on_write = nullptr;
  }

 private:
  void release_due() {
    if (!later.empty() && millis() >= later_due) {
      feed(later);
      later.clear();
    }
  }
};

#define D24 24
#define D25 25
/* Modem.cpp binds the global modem to &Serial2 when ARDUINO_UNOWIFIR4 is defined, as on the
 * real board. Each modem test defines this object. */
extern UART Serial2;

/* Just enough of Arduino's String for XboxControllerNotificationParser::toString(). */
class String {
 public:
  String() {}
  String(const char* s) : s_(s ? s : "") {}
  String(const std::string& s) : s_(s) {}
  explicit String(int v) : s_(std::to_string(v)) {}
  explicit String(unsigned int v) : s_(std::to_string(v)) {}
  explicit String(long v) : s_(std::to_string(v)) {}
  explicit String(unsigned long v) : s_(std::to_string(v)) {}
  /* Arduino prints an unsigned char as a number, not as a character. */
  explicit String(unsigned char v) : s_(std::to_string((unsigned)v)) {}

  String& operator+=(const String& o) { s_ += o.s_; return *this; }
  String& operator+=(const char* o) { s_ += o; return *this; }
  const char* c_str() const { return s_.c_str(); }
  size_t length() const { return s_.size(); }
  int indexOf(const char* needle) const {
    size_t p = s_.find(needle);
    return p == std::string::npos ? -1 : (int)p;
  }

 private:
  std::string s_;
};

inline String operator+(const String& a, const String& b) { String r(a); r += b; return r; }
inline String operator+(const String& a, const char* b) { String r(a); r += b; return r; }
inline String operator+(const char* a, const String& b) { String r(a); r += b; return r; }

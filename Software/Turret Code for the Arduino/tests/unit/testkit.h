// testkit.h -- the tiny assert kit for the host unit tests (docs/architecture.md 8.1).
//
//   #include "testkit.h"
//   TEST(stepper_counts_up) {
//     hal_host::reset();
//     CHECK(hal::begin());
//     CHECK_EQ(hal::stepPosition(AXIS_PAN), 0);
//     CHECK_NEAR(Motion::stepsToDeg(AXIS_PAN, 2222), 100.0f, 0.01f);
//   }
//   TESTKIT_MAIN()
//
// A failed check prints "FAIL <test>: <file>:<line>: <expression> ..." and the test carries
// on (so one run shows every failure). The binary prints "info <name>: N tests, M checks,
// K failures" and exits 1 if anything failed. tests/run.sh shows the info and FAIL lines.
// Arguments: substrings; only tests whose names contain one of them run. -v lists each test.
//
// SPDX-License-Identifier: MIT
#pragma once
#include <cmath>
#include <cstdio>
#include <cstring>
#include <sstream>
#include <string>
#include <type_traits>
#include <vector>

namespace testkit {

struct Case {
  const char* name;
  void (*fn)();
};

inline std::vector<Case>& cases() {
  static std::vector<Case> all;
  return all;
}

struct Counters {
  int checks = 0;
  int failures = 0;
  int testFailures = 0;
  const char* current = "";
};

inline Counters& counters() {
  static Counters c;
  return c;
}

struct Reg {
  Reg(const char* name, void (*fn)()) { cases().push_back(Case{name, fn}); }
};

// Printable form of a value: integers as numbers (also uint8_t / char-sized ones), enums by
// their underlying value, strings quoted.
template <typename T>
std::string show(const T& v) {
  std::ostringstream os;
  if constexpr (std::is_enum_v<T>) {
    os << +static_cast<std::underlying_type_t<T>>(v);
  } else if constexpr (std::is_same_v<T, bool>) {
    os << (v ? "true" : "false");
  } else if constexpr (std::is_integral_v<T>) {
    os << +v;
  } else if constexpr (std::is_convertible_v<T, std::string>) {
    os << '"' << std::string(v) << '"';
  } else {
    os << v;
  }
  return os.str();
}

inline void fail(const char* file, int line, const std::string& what) {
  Counters& c = counters();
  c.failures++;
  std::printf("FAIL %s: %s:%d: %s\n", c.current, file, line, what.c_str());
}

inline bool check(bool ok, const char* file, int line, const char* expr) {
  counters().checks++;
  if (!ok) fail(file, line, expr);
  return ok;
}

template <typename T>
constexpr bool isNegative(T v) {
  if constexpr (std::is_signed_v<T>)
    return v < 0;
  else
    return false;
}

template <typename A, typename B>
bool checkEq(const A& a, const B& b, const char* file, int line, const char* ea, const char* eb) {
  counters().checks++;
  bool ok;
  if constexpr (std::is_integral_v<A> && std::is_integral_v<B>) {
    // Compare integers by value whatever their signedness (CHECK_EQ(u32, 5) must work).
    ok = isNegative(a) == isNegative(b) &&
         static_cast<unsigned long long>(a) == static_cast<unsigned long long>(b);
  } else {
    ok = a == b;
  }
  if (!ok) fail(file, line, std::string(ea) + " == " + eb + "  (" + show(a) + " vs " + show(b) + ")");
  return ok;
}

inline bool checkNear(double a, double b, double tol, const char* file, int line, const char* ea,
                      const char* eb) {
  counters().checks++;
  bool ok = std::fabs(a - b) <= tol;
  if (!ok) {
    std::ostringstream os;
    os << ea << " ~= " << eb << "  (" << a << " vs " << b << ", tolerance " << tol << ")";
    fail(file, line, os.str());
  }
  return ok;
}

inline int runAll(int argc, char** argv) {
  bool verbose = false;
  std::vector<const char*> filters;
  for (int i = 1; i < argc; i++) {
    if (std::strcmp(argv[i], "-v") == 0)
      verbose = true;
    else
      filters.push_back(argv[i]);
  }
  Counters& c = counters();
  int ran = 0;
  for (const Case& t : cases()) {
    if (!filters.empty()) {
      bool hit = false;
      for (const char* f : filters) hit = hit || std::strstr(t.name, f) != nullptr;
      if (!hit) continue;
    }
    c.current = t.name;
    const int before = c.failures;
    t.fn();
    ran++;
    if (c.failures != before)
      c.testFailures++;
    else if (verbose)
      std::printf("ok   %s\n", t.name);
  }
  const char* self = argc > 0 ? argv[0] : "test";
  const char* slash = std::strrchr(self, '/');
  std::printf("info %s: %d tests, %d checks, %d failures\n", slash ? slash + 1 : self, ran, c.checks,
              c.failures);
  return c.failures ? 1 : 0;
}

}  // namespace testkit

#define TESTKIT_CAT2_(a, b) a##b
#define TESTKIT_CAT_(a, b) TESTKIT_CAT2_(a, b)
#define TEST(name)                                                   \
  static void name();                                                \
  static ::testkit::Reg TESTKIT_CAT_(name, _registration_)(#name, &name); \
  static void name()
#define CHECK(cond) ::testkit::check(static_cast<bool>(cond), __FILE__, __LINE__, #cond)
#define CHECK_EQ(a, b) ::testkit::checkEq((a), (b), __FILE__, __LINE__, #a, #b)
#define CHECK_NEAR(a, b, tol) \
  ::testkit::checkNear(static_cast<double>(a), static_cast<double>(b), static_cast<double>(tol), __FILE__, __LINE__, #a, #b)
#define TESTKIT_MAIN() \
  int main(int argc, char** argv) { return ::testkit::runAll(argc, argv); }

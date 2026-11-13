#!/bin/bash
# run.sh -- every D-Sentry check that runs on this Mac without a board:
#   1. the RA4M1 modules (firmware-ra4m1/dsentry/src/*.cpp except hal_arduino.cpp) and the host
#      HAL (sim/hal_host.cpp) compiled with clang++ -std=gnu++17 -Wall -Wextra -Werror;
#   2. each tests/unit/test_*.cpp linked against them into its own binary, and run;
#   3. dsentry_sim built (sim/build.sh) and run for a moment;
#   4. the Python tests in relay-pc/tests, then the acceptance tests in tests/acceptance (dsentry_sim
#      driven by the relay's own code over UDP and its pty, the relay process, bench_console);
#   5. the generators' --check modes (protocol and the ESP32 config copy);
#   6. tools/check_literals.py: its self-test, then the RA4M1 sketch and the ESP32 D-Sentry files;
#   7. tests/chat/run.sh: ControlLink + AsyncAt against the real chAT server of the ESP32 fork;
#   8. tests/esp32/run.sh: the D-Sentry ESP32 additions on the host (wire logic, AT handlers on the
#      real chAT, their literals).
#
# Usage: tests/run.sh [-v]
#   -v  also print every passing check line, not just verdicts, info lines and failures.
# Env:   CXX                      compiler (default clang++)
#        PYTHON                   interpreter (default <repo>/.venv/bin/python, else python3)
#        DSENTRY_TESTS_BUILD_DIR  objects, binaries and logs (default D-SentryCode/tests/build,
#                                 which git ignores)
# Exit status: 0 when everything passes (SKIPs allowed), 1 otherwise, 2 on a usage error.
#
# SPDX-License-Identifier: MIT
set -u

usage() {
  printf 'Usage: %s [-v]\n  -v  also print every passing check\n' "$0"
}

VERBOSE=0
for arg in "$@"; do
  case "$arg" in
    -v) VERBOSE=1 ;;
    -h|--help) usage; exit 0 ;;
    *) printf 'run.sh: unknown argument: %s\n' "$arg" >&2; usage >&2; exit 2 ;;
  esac
done

HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
DSC="$(cd "$HERE/.." && pwd)"
REPO="$(cd "$DSC/.." && pwd)"
SRC="$DSC/firmware-ra4m1/dsentry/src"
SIM="$DSC/sim"
UNIT="$HERE/unit"
CXX="${CXX:-clang++}"
if [ -n "${PYTHON:-}" ]; then
  PY="$PYTHON"
elif [ -x "$REPO/.venv/bin/python" ]; then
  PY="$REPO/.venv/bin/python"
else
  PY="python3"
fi
if ! mkdir -p "${DSENTRY_TESTS_BUILD_DIR:-$HERE/build}/obj"; then
  printf 'run.sh: cannot create the build directory %s\n' "${DSENTRY_TESTS_BUILD_DIR:-$HERE/build}" >&2
  exit 1
fi
BUILD="$(cd "${DSENTRY_TESTS_BUILD_DIR:-$HERE/build}" && pwd)"

CXXFLAGS=(-std=gnu++17 -O1 -g)
OUR_WARNINGS=(-Wall -Wextra -Werror)
INCLUDES=(-I "$SRC" -I "$SIM" -I "$SIM/host_include" -I "$UNIT" -I "$DSC/protocol")

npass=0
nfail=0
nskip=0

say() { printf '%s\n' "$*"; }
pass() { say "PASS  $1"; npass=$((npass + 1)); }
fail() { say "FAIL  $1"; nfail=$((nfail + 1)); }
skip() { say "SKIP  $1"; nskip=$((nskip + 1)); }

# run_test <name> <command...>: shows info/FAIL lines (all lines with -v), then the verdict.
run_test() {
  name="$1"; shift
  log="$BUILD/$name.log"
  "$@" > "$log" 2>&1
  rc=$?
  if [ "$VERBOSE" = 1 ]; then
    sed 's/^/      /' "$log"
  else
    grep -E '^(info|FAIL)' "$log" | sed 's/^/      /'
  fi
  if [ $rc -eq 0 ]; then
    pass "$name"
  else
    fail "$name (exit $rc, log: $log)"
    [ "$VERBOSE" = 1 ] || tail -5 "$log" | sed 's/^/      /'
  fi
}

say "D-Sentry host checks: $DSC"
say "      info build directory: $BUILD"

if ! command -v "$CXX" > /dev/null 2>&1; then
  fail "compiler '$CXX' not found; install the Xcode command line tools or set CXX"
  say "Summary: $npass passed, $nfail failed, $nskip skipped"
  exit 1
fi

# ---- 1. objects: every portable module plus the host HAL, compiled in parallel ----
sources=()
for f in "$SRC"/*.cpp; do
  [ "$(basename "$f")" = "hal_arduino.cpp" ] && continue   # board only (#ifdef ARDUINO)
  sources+=("$f")
done
sources+=("$SIM/hal_host.cpp")
objects=()
pids=()
for f in "${sources[@]}"; do
  o="$BUILD/obj/$(basename "${f%.cpp}").o"
  objects+=("$o")
  "$CXX" "${CXXFLAGS[@]}" "${OUR_WARNINGS[@]}" "${INCLUDES[@]}" -c "$f" -o "$o" > "$o.log" 2>&1 &
  pids+=($!)
done
objects_ok=1
for i in "${!pids[@]}"; do
  if ! wait "${pids[$i]}"; then
    objects_ok=0
    fail "compile $(basename "${sources[$i]}") (log: ${objects[$i]}.log)"
    head -20 "${objects[$i]}.log" | sed 's/^/      /'
  fi
done
if [ $objects_ok = 1 ]; then
  pass "compile ${#sources[@]} sources (modules + hal_host) with -Wall -Wextra -Werror"
fi

# ---- 2. unit tests, one binary each ----
if [ $objects_ok = 1 ]; then
  for t in "$UNIT"/test_*.cpp; do
    [ -e "$t" ] || continue
    name="$(basename "${t%.cpp}")"
    log="$BUILD/$name.build.log"
    if "$CXX" "${CXXFLAGS[@]}" "${OUR_WARNINGS[@]}" "${INCLUDES[@]}" "$t" "${objects[@]}" \
         -o "$BUILD/$name" > "$log" 2>&1; then
      if [ "$VERBOSE" = 1 ]; then
        run_test "$name" "$BUILD/$name" -v
      else
        run_test "$name" "$BUILD/$name"
      fi
    else
      fail "$name: compile failed (log: $log)"
      head -20 "$log" | sed 's/^/      /'
    fi
  done
else
  skip "unit tests: the sources did not compile"
fi

# ---- 3. dsentry_sim builds and runs ----
sim_ok=0
if DSENTRY_SIM_BUILD_DIR="$BUILD/sim" bash "$SIM/build.sh" > "$BUILD/sim.build.log" 2>&1; then
  sim_out="$("$BUILD/sim/dsentry_sim" --seconds 0.3 --udp-port 0 < /dev/null 2>&1)"
  if printf '%s\n' "$sim_out" | grep -q "D-Sentry RA4M1 boot" && printf '%s\n' "$sim_out" | grep -q "stopped"; then
    say "      info $(printf '%s\n' "$sim_out" | tail -1)"
    sim_ok=1
    pass "dsentry_sim builds and boots"
  else
    fail "dsentry_sim ran but did not print its boot line"
    printf '%s\n' "$sim_out" | tail -5 | sed 's/^/      /'
  fi
else
  fail "dsentry_sim: build failed (log: $BUILD/sim.build.log)"
  tail -15 "$BUILD/sim.build.log" | sed 's/^/      /'
fi

# ---- 4. Python: relay, codec, config parser, tools ----
if "$PY" -c 'import sys; sys.exit(0 if sys.version_info >= (3, 8) else 1)' > /dev/null 2>&1; then
  log="$BUILD/relay-pc-tests.log"
  if "$PY" -m unittest discover -s "$DSC/relay-pc/tests" > "$log" 2>&1; then
    say "      info $(grep -E '^Ran [0-9]+ tests' "$log")$(grep -oE 'skipped=[0-9]+' "$log" | sed 's/^/, /')"
    pass "relay-pc Python tests"
  else
    fail "relay-pc Python tests (log: $log)"
    grep -E '^(FAIL|ERROR):' "$log" | head -10 | sed 's/^/      /'
    tail -3 "$log" | sed 's/^/      /'
  fi
  # The acceptance tests: dsentry_sim (the real RA4M1 firmware, the ESP32's real AT handlers on
  # chAT) driven by the relay's own code. They need the simulator from step 3.
  if [ $sim_ok = 1 ]; then
    log="$BUILD/acceptance.log"
    uargs=()
    [ "$VERBOSE" = 1 ] && uargs=(-v)
    if DSENTRY_SIM_BIN="$BUILD/sim/dsentry_sim" DSENTRY_TESTS_BUILD_DIR="$BUILD" \
         "$PY" -m unittest discover -s "$HERE/acceptance" -t "$HERE/acceptance" ${uargs[@]+"${uargs[@]}"} \
         > "$log" 2>&1; then
      [ "$VERBOSE" = 1 ] && sed 's/^/      /' "$log"
      say "      info $(grep -E '^Ran [0-9]+ tests' "$log")$(grep -oE 'skipped=[0-9]+' "$log" | sed 's/^/, /')"
      pass "acceptance tests against dsentry_sim (tests/acceptance)"
    else
      fail "acceptance tests against dsentry_sim (log: $log)"
      grep -E '^(FAIL|ERROR):' "$log" | head -10 | sed 's/^/      /'
      tail -3 "$log" | sed 's/^/      /'
    fi
  else
    fail "acceptance tests: dsentry_sim did not build or boot (step 3)"
  fi
else
  fail "Python 3.8+ not found at '$PY'; set PYTHON"
fi

# ---- 5. generated files are current ----
gen_check() {
  name="$1"; shift
  log="$BUILD/$name.log"
  if "$PY" "$@" > "$log" 2>&1; then
    pass "$name"
  else
    fail "$name (log: $log)"
    sed 's/^/      /' "$log" | tail -5
  fi
}
gen_check "gen_protocol --check" "$DSC/tools/gen_protocol.py" --check
gen_check "sync_config --check" "$DSC/tools/sync_config.py" --check

# ---- 6. acceptance item 20: no numeric literals ----
gen_check "check_literals --selftest" "$DSC/tools/check_literals.py" --selftest
log="$BUILD/check_literals.log"
if "$PY" "$DSC/tools/check_literals.py" --ra4m1 > "$log" 2>&1; then
  say "      info $(tail -1 "$log")"
  pass "check_literals --ra4m1"
else
  fail "check_literals --ra4m1 (log: $log)"
  head -20 "$log" | sed 's/^/      /'
fi
# The ESP32 fork's D-Sentry files: an empty scope (files moved or renamed) is a failure.
log="$BUILD/check_literals_esp32.log"
if "$PY" "$DSC/tools/check_literals.py" --esp32 > "$log" 2>&1; then
  say "      info $(tail -1 "$log")"
  pass "check_literals --esp32"
else
  fail "check_literals --esp32 (log: $log)"
  head -20 "$log" | sed 's/^/      /'
fi

# ---- 7, 8. the suites with their own scripts: their PASS / FAIL / SKIP lines count here ----
# sub_suite <name> <script>: runs it with this run's build directory and -v, shows its lines
# indented (all of them with -v; info / FAIL / verdicts otherwise) and adds its verdicts to the
# totals. A script that exits non-zero without printing a FAIL line counts as one failure.
sub_suite() {
  name="$1"; script="$2"
  log="$BUILD/$name.log"
  if [ "$VERBOSE" = 1 ]; then
    DSENTRY_TESTS_BUILD_DIR="$BUILD" PYTHON="$PY" CXX="$CXX" bash "$script" -v > "$log" 2>&1
  else
    DSENTRY_TESTS_BUILD_DIR="$BUILD" PYTHON="$PY" CXX="$CXX" bash "$script" > "$log" 2>&1
  fi
  rc=$?
  if [ "$VERBOSE" = 1 ]; then
    grep -vE '^(PASS|FAIL|SKIP)  ' "$log" | sed 's/^/      /'
  else
    grep -E '^ *info |^ *FAIL ' "$log" | grep -vE '^(PASS|FAIL|SKIP)  ' | sed 's/^ */      /'
  fi
  sp=$(grep -cE '^PASS  ' "$log")
  sf=$(grep -cE '^FAIL  ' "$log")
  ss=$(grep -cE '^SKIP  ' "$log")
  grep -E '^(PASS|FAIL|SKIP)  ' "$log" | sed "s/^\(....\)  /\1  $name: /"
  npass=$((npass + sp)); nfail=$((nfail + sf)); nskip=$((nskip + ss))
  if [ $rc -ne 0 ] && [ "$sf" -eq 0 ]; then
    fail "$name (exit $rc without a FAIL line, log: $log)"
  fi
}
sub_suite "chat" "$HERE/chat/run.sh"
sub_suite "esp32" "$HERE/esp32/run.sh"

say "Summary: $npass passed, $nfail failed, $nskip skipped"
if [ $nfail -ne 0 ]; then
  exit 1
fi
exit 0

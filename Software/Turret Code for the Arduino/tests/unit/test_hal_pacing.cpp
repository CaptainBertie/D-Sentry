// test_hal_pacing.cpp -- the hardware rules both HALs share or mirror (link-hal stage):
//   - TxQueue (firmware-ra4m1/dsentry/src/TxQueue.h), checked against this file's own model
//     of the RA4M1 transmitters: Serial (SCI9, no FIFO: a write blocks while the shift register
//     is busy) and Serial2 (SCI1, 16-byte FIFO opened with FCR.TTRG = 0: a write blocks unless
//     the FIFO is empty). Random message sizes, random service() intervals, stale timestamps
//     and the micros() wrap; bytes must arrive whole, in order, and no write may ever block.
//   - the same through hal::usbWrite / hal::linkWrite / hal::service, with sim/hal_host.cpp's
//     own transmitter probe;
//   - stepper edge cases the board's ISR handles: a restart after a stop uses the new period
//     (never a stale long one), bounds are checked before the pulse, the quiet window ignores
//     stopped axes and follows the axis whose edge is nearest.
//
// Key safety property (test tx_queue_never_makes_a_write_block, for both UARTs): no UART write
// ever blocks loop(); it fails if the pacer offers bytes the transmitter cannot take at once.
//
// SPDX-License-Identifier: MIT
#include <random>
#include <string>
#include <vector>

#include "TxQueue.h"
#include "hal.h"
#include "hal_host.h"
#include "testkit.h"

namespace {

// Independent model of one transmitter, in nanoseconds with the exact byte time.
struct Uart {
  bool fifo;
  size_t depth;
  uint64_t byteNs;
  uint64_t shiftFreeNs = 0;
  size_t fifoCount = 0;
  uint32_t blocking = 0;
  std::string wire;

  Uart(bool f, size_t d, uint32_t baud)
      : fifo(f), depth(d), byteNs((uint64_t)UART_BITS_PER_BYTE * US_PER_S * US_PER_MS / baud) {}
  void advance(uint64_t now) {
    while (fifoCount > 0 && shiftFreeNs <= now) {
      shiftFreeNs += byteNs;
      fifoCount--;
    }
  }
  void write(uint64_t now, const uint8_t* p, size_t n) {
    advance(now);
    const bool busy = fifo ? (fifoCount > 0 || n > depth) : (shiftFreeNs > now || n > 1);
    if (busy) blocking++;
    wire.append((const char*)p, n);
    size_t k = n;
    if (shiftFreeNs <= now) {
      shiftFreeNs = now + byteNs;
      k--;
    }
    if (fifo)
      fifoCount += k;
    else
      shiftFreeNs += k * byteNs;
  }
  bool idle(uint64_t now) {
    advance(now);
    return fifoCount == 0 && shiftFreeNs <= now;
  }
};

constexpr size_t kQueue = 512;

// Drives one TxQueue + Uart for `seconds` of virtual time with random traffic. Returns the
// bytes the queue accepted, in order, so the caller can compare them with the wire.
std::string soak(bool fifo, uint32_t baud, size_t burst, uint32_t seed, uint64_t startUs, Uart& uart,
                 uint32_t& drops) {
  std::mt19937 rng(seed);
  TxQueue<kQueue> q;
  uint64_t now = startUs;   // true time, us (64-bit); the queue sees (uint32_t)now
  q.begin(baud, burst, (uint32_t)now);
  std::string accepted;
  uint8_t counter = 0;
  uint32_t lastStamp = (uint32_t)now;
  for (int i = 0; i < 40000; i++) {
    // loop() periods from 5 us to 3 ms, occasionally a 6 ms stall
    uint32_t dt = 5 + rng() % 400;
    if (rng() % 50 == 0) dt = 3000;
    if (rng() % 400 == 0) dt = 6000;
    now += dt;
    if (rng() % 8 == 0) {   // a message of 1..120 bytes
      std::string m(1 + rng() % 120, 0);
      for (char& c : m) c = (char)counter++;
      if (q.push((const uint8_t*)m.data(), m.size()))
        accepted += m;
      else
        counter = (uint8_t)(counter - m.size());   // keep the sequence gap-free
    }
    // The HAL reads micros() right before the write: whole microseconds, so the write really
    // happens up to ~1 us after the timestamp. Sometimes an idle() poll saw the clock first.
    if (rng() % 10 == 0) q.idle((uint32_t)now);
    const uint32_t stamp = (uint32_t)now;
    const uint64_t writeNs = now * US_PER_MS + rng() % US_PER_MS;   // 0..999 ns later
    uint8_t chunk[SCI_FIFO_DEPTH];
    const size_t n = q.take(stamp, chunk, sizeof chunk);
    if (n) uart.write(writeNs, chunk, n);
    (void)lastStamp;
  }
  // drain the rest
  for (int i = 0; i < 100000 && q.used(); i++) {
    now += 50;
    uint8_t chunk[SCI_FIFO_DEPTH];
    const size_t n = q.take((uint32_t)now, chunk, sizeof chunk);
    if (n) uart.write(now * US_PER_MS, chunk, n);
  }
  drops = q.dropped();
  (void)fifo;
  return accepted;
}

}  // namespace

// KEY SAFETY PROPERTY: no write ever blocks, on either UART, whatever the timing.
TEST(tx_queue_never_makes_a_write_block) {
  struct Case { bool fifo; size_t burst; uint32_t baud; uint64_t start; };
  const Case cases[] = {
      {false, SERIAL_TX_SLACK_BYTES, DEBUG_SERIAL_BAUD, 0},
      {true, SCI_FIFO_DEPTH, LINK_BAUD, 0},
      {false, SERIAL_TX_SLACK_BYTES, DEBUG_SERIAL_BAUD, (uint64_t)UINT32_MAX - 2000000u},   // wraps
      {true, SCI_FIFO_DEPTH, LINK_BAUD, (uint64_t)UINT32_MAX - 2000000u},
  };
  uint32_t seed = 1;
  for (const Case& c : cases) {
    for (int rep = 0; rep < 3; rep++) {
      Uart uart(c.fifo, c.fifo ? SCI_FIFO_DEPTH : SERIAL_TX_SLACK_BYTES, c.baud);
      uint32_t drops = 0;
      const std::string accepted = soak(c.fifo, c.baud, c.burst, seed++, c.start, uart, drops);
      CHECK_EQ(uart.blocking, 0u);
      CHECK(uart.wire == accepted);   // whole messages, in order, nothing lost or invented
      CHECK(accepted.size() > 10000u);
      CHECK(drops > 0u);              // the load really exceeded the line rate at times
    }
  }
}

TEST(tx_queue_runs_close_to_line_rate) {
  // A busy link, service() every 5 us: the pacer must not waste the line (it lags the
  // hardware by the round-up of the byte time plus 1 us, and a FIFO burst waits for the FIFO
  // to empty).
  for (int fifo = 0; fifo < 2; fifo++) {
    const uint32_t baud = fifo ? LINK_BAUD : DEBUG_SERIAL_BAUD;
    Uart uart(fifo != 0, fifo ? SCI_FIFO_DEPTH : SERIAL_TX_SLACK_BYTES, baud);
    TxQueue<kQueue> q;
    q.begin(baud, fifo ? SCI_FIFO_DEPTH : SERIAL_TX_SLACK_BYTES, 0);
    const std::string msg(64, 'm');
    uint64_t now = 0;
    const uint64_t endUs = US_PER_S;
    while (now < endUs) {
      while (q.freeBytes() >= msg.size()) q.push((const uint8_t*)msg.data(), msg.size());
      uint8_t chunk[SCI_FIFO_DEPTH];
      const size_t n = q.take((uint32_t)now, chunk, sizeof chunk);
      if (n) uart.write(now * US_PER_MS, chunk, n);
      now += 5;
    }
    const double lineRate = (double)baud / UART_BITS_PER_BYTE;   // bytes/s
    CHECK(uart.wire.size() >= (size_t)(lineRate * 0.95));
    CHECK(uart.wire.size() <= (size_t)(lineRate + 1));
    CHECK_EQ(uart.blocking, 0u);
  }
}

TEST(tx_queue_idle_means_the_last_byte_has_left) {
  TxQueue<kQueue> q;
  q.begin(LINK_BAUD, SCI_FIFO_DEPTH, 1000);
  Uart uart(true, SCI_FIFO_DEPTH, LINK_BAUD);
  CHECK(q.idle(1000));
  const std::string m(20, 'x');
  CHECK(q.push((const uint8_t*)m.data(), m.size()));
  CHECK(!q.idle(1000));
  uint8_t chunk[SCI_FIFO_DEPTH];
  CHECK_EQ(q.take(1000, chunk, sizeof chunk), (size_t)SCI_FIFO_DEPTH);
  CHECK_EQ(q.take(1000, chunk, sizeof chunk), (size_t)0);    // one write per empty FIFO
  const uint32_t burstUs = (uint32_t)SCI_FIFO_DEPTH * q.byteUs();
  CHECK_EQ(q.take(1000 + burstUs - 1, chunk, sizeof chunk), (size_t)0);
  CHECK_EQ(q.take(1000 + burstUs, chunk, sizeof chunk), m.size() - SCI_FIFO_DEPTH);
  const uint32_t end = 1000 + burstUs + (uint32_t)(m.size() - SCI_FIFO_DEPTH) * q.byteUs();
  CHECK(!q.idle(end - 1));
  CHECK(q.idle(end));
  CHECK(uart.idle((uint64_t)end * US_PER_MS));   // the model agrees: the line is quiet
}

TEST(tx_queue_take_respects_the_callers_buffer_and_wraps_the_ring) {
  TxQueue<SCI_FIFO_DEPTH * 2> q;
  q.begin(LINK_BAUD, SCI_FIFO_DEPTH, 0);
  std::string all;
  uint32_t t = 0;
  uint8_t next = 0;
  for (int round = 0; round < 20; round++) {
    std::string m(SCI_FIFO_DEPTH + 3, 0);
    for (char& c : m) c = (char)next++;
    CHECK(q.push((const uint8_t*)m.data(), m.size()));
    while (q.used()) {
      uint8_t chunk[SCI_FIFO_DEPTH / 2];   // smaller than a burst
      t += SCI_FIFO_DEPTH * q.byteUs();
      const size_t n = q.take(t, chunk, sizeof chunk);
      CHECK(n <= sizeof chunk);
      all.append((const char*)chunk, n);
    }
  }
  CHECK_EQ(all.size(), (size_t)(20 * (SCI_FIFO_DEPTH + 3)));
  bool ordered = true;
  for (size_t i = 0; i < all.size(); i++) ordered = ordered && (uint8_t)all[i] == (uint8_t)i;
  CHECK(ordered);
}

TEST(hal_serial_under_random_load_never_blocks) {
  hal_host::reset();
  hal::begin();
  std::mt19937 rng(42);
  std::string usbSent, linkSent, usbWire, linkWire;
  for (int i = 0; i < 30000; i++) {
    hal_host::advanceUs(5 + rng() % 600);
    if (rng() % 6 == 0) {
      std::string m(1 + rng() % 100, (char)('a' + rng() % 26));
      if (hal::usbWrite((const uint8_t*)m.data(), m.size())) usbSent += m;
    }
    if (rng() % 9 == 0) {
      std::string m(1 + rng() % (DS_AT_DST_LINE_LEN), (char)('A' + rng() % 26));
      if (hal::linkWrite((const uint8_t*)m.data(), m.size())) linkSent += m;
    }
    const uint32_t stamp = hal::nowUs() - (uint32_t)(rng() % 40);
    if (rng() % 5 == 0) (void)hal::linkTxIdle();   // reads a fresher clock than `stamp`
    hal::service(stamp);
    usbWire += hal_host::usbTakeTx();
    linkWire += hal_host::linkTakeTx();
  }
  for (int i = 0; i < 20000; i++) {
    hal_host::advanceUs(50);
    hal::service(hal::nowUs());
  }
  usbWire += hal_host::usbTakeTx();
  linkWire += hal_host::linkTakeTx();
  CHECK(usbWire == usbSent);
  CHECK(linkWire == linkSent);
  CHECK_EQ(hal_host::usbTxProbe().blockingWrites, 0u);
  CHECK_EQ(hal_host::linkTxProbe().blockingWrites, 0u);
  CHECK_EQ(hal_host::usbTxProbe().largestWrite, (size_t)SERIAL_TX_SLACK_BYTES);
  CHECK_EQ(hal_host::linkTxProbe().largestWrite, (size_t)SCI_FIFO_DEPTH);
  CHECK(hal::linkTxIdle());
}

// ---- steppers -----------------------------------------------------------------------------------

TEST(restart_after_a_stop_waits_one_new_period_not_the_old_one) {
  hal_host::reset();
  hal::begin();
  hal::stepperEnable(true);
  const float slow = STEP_MIN_RATE_HZ + 1.0f;               // ~38 ms period
  hal::stepSetRate(AXIS_PAN, slow);
  hal_host::advanceUs(US_PER_S / 100);                      // 10 ms into the long period
  hal::stepSetRate(AXIS_PAN, 0.0f);                         // below the minimum: stopped
  CHECK(!hal_host::running(AXIS_PAN));
  CHECK_EQ(hal::stepQuietUs(), UINT32_MAX);                 // a stopped axis has no edge due
  const uint32_t steps0 = hal_host::totalSteps(AXIS_PAN);
  const float fast = STEP_MAX_RATE_HZ / 2;
  hal::stepSetRate(AXIS_PAN, fast);
  const uint32_t periodUs = (uint32_t)((float)US_PER_S / fast);
  CHECK(hal::stepQuietUs() <= periodUs);
  hal_host::advanceUs(periodUs + 1);
  CHECK_EQ(hal_host::totalSteps(AXIS_PAN), steps0 + 1);     // one new period, not 28 ms more
}

TEST(a_refused_step_still_applies_the_pending_direction) {
  hal_host::reset();
  hal::begin();
  hal::stepperEnable(true);
  hal::stepSetBounds(AXIS_TILT, true, 0, 0);
  hal::stepSetRate(AXIS_TILT, STEP_MAX_RATE_HZ);            // up, into the bound at 0
  hal_host::advanceUs(US_PER_S / 100);
  CHECK_EQ(hal::stepPosition(AXIS_TILT), 0);
  CHECK(hal_host::totalRefusals(AXIS_TILT) > 0u);
  CHECK_EQ(hal_host::trueSteps(AXIS_TILT), (int64_t)0);    // no pulse left the pin
  hal::stepSetBounds(AXIS_TILT, true, -5, 0);
  hal::stepSetRate(AXIS_TILT, -STEP_MAX_RATE_HZ);           // reverse while running
  hal_host::advanceUs(US_PER_S / 100);
  CHECK_EQ(hal::stepPosition(AXIS_TILT), -5);               // down to the other bound, no further
  CHECK_EQ(hal_host::trueSteps(AXIS_TILT), (int64_t)-5);
}

TEST(quiet_window_follows_the_nearest_edge_of_running_axes) {
  hal_host::reset();
  hal::begin();
  hal::stepperEnable(true);
  hal::stepSetRate(AXIS_PAN, 1000.0f);    // 1000 us period
  hal_host::advanceUs(300);
  hal::stepSetRate(AXIS_TILT, 500.0f);    // 2000 us period, started 300 us later
  CHECK_EQ(hal::stepQuietUs(), 700u);     // pan's edge is nearer
  hal_host::advanceUs(700);               // pan steps
  CHECK_EQ(hal::stepQuietUs(), 1000u);    // pan again at +1000, tilt at +1300
  hal::stepSetRate(AXIS_PAN, 0.0f);
  CHECK_EQ(hal::stepQuietUs(), 1300u);    // a stopped axis is ignored
  hal::stepSetRate(AXIS_TILT, 0.0f);
  CHECK_EQ(hal::stepQuietUs(), UINT32_MAX);
}

TESTKIT_MAIN()

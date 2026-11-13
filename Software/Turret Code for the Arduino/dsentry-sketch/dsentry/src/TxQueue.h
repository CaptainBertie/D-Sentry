// TxQueue.h -- a byte ring plus a pacer that says how many bytes a UART can take right now
// without UART::write() blocking. Used by both HALs (hal_arduino.cpp for Serial and Serial2,
// sim/hal_host.cpp for the virtual wires), so the pacing the board runs is the pacing the
// host tests check (and sim/hal_host.cpp checks it against its own model of the transmitter).
//
// Why: on arduino:renesas_uno 1.6.0 UART::write() spins until the FSP driver reports
// UART_EVENT_TX_DATA_EMPTY (Serial.cpp:113-138). When that happens depends on the channel
// (disassembly of libfsp.a r_sci_uart.o, RA4M1 hardware manual section 28):
//   - Serial = SCI9, no FIFO. R_SCI_UART_Write stores the first byte straight into TDR and the
//     TXI interrupt that ends the write fires when TDR moves to the shift register. One byte
//     written while the previous one is still shifting out therefore waits up to one byte time
//     (86.8 us at 115200); a second byte in the same call always waits. Burst = 1.
//   - Serial2 = SCI1, 16-byte FIFO. R_SCI_UART_Write only sets SCR.TIE (Write +0x14..+0x46,
//     +0xa4); sci_uart_txi_isr then fills the FIFO up to its depth from FDR.T. R_SCI_UART_Open
//     programs FCR = FM | RFRST | TFRST | RTRG | RSTRG with TTRG = 0 (Open +0x212..+0x22c and
//     +0x41c..+0x42c), and TDFE, which raises TXI, is set only while the FIFO holds no more than
//     TTRG bytes (hardware manual "TDFE flag", hwm.txt:43503-43514, 44847-44850). So a write
//     returns at once only when the FIFO is EMPTY, and then takes up to 16 bytes in one call;
//     a write while even one byte is still in the FIFO waits until the FIFO has drained, and a
//     second write() in the same burst always waits. Burst = SCI_FIFO_DEPTH.
// The pacer models both with one rule: nothing is handed to the UART while any byte of the
// previous burst is still in flight, then at most `burst` bytes go in ONE write() call. A byte
// is retired one byte time after the burst started, the byte time rounded up and then one
// microsecond added, so the model always lags the hardware and a write never finds the
// transmitter busy.
//
// Timestamps: take() must get the time read IMMEDIATELY before the write it feeds (the HALs'
// service() reads its own clock for this). A timestamp taken earlier in loop() would make the
// burst look older than it is and the next write could block: measured by
// tests/unit/test_hal_pacing.cpp against an independent transmitter model. The extra
// microsecond covers micros()' whole-microsecond resolution and the instructions between the
// clock read and the write.
//
// Messages are queued whole or not at all (push() returns false and counts the drop), so
// a binary frame and a text line can never interleave or arrive cut.
//
// SPDX-License-Identifier: MIT
#pragma once
#include <stddef.h>
#include <stdint.h>
#include <string.h>
#include "../config.h"

template <size_t N>
class TxQueue {
 public:
  // baud: line rate; burst: bytes one write() may carry into an idle transmitter
  // (SERIAL_TX_SLACK_BYTES for a UART without FIFO, SCI_FIFO_DEPTH with one).
  void begin(uint32_t baud, size_t burst, uint32_t nowUs) {
    // Round the byte time up (writing early by a fraction of a microsecond would block), plus
    // one microsecond for the clock's resolution (see the file comment).
    byteUs_ = (uint32_t)((UART_BITS_PER_BYTE * US_PER_S + baud - 1) / baud) + 1u;
    burst_ = burst;
    inFlight_ = 0;
    lastUs_ = nowUs;
    seenUs_ = nowUs;
    head_ = tail_ = count_ = 0;
  }

  // Queue all `len` bytes, or none of them (counted in dropped()).
  bool push(const uint8_t* data, size_t len) {
    if (len > N - count_) {
      dropped_++;
      droppedBytes_ += (uint32_t)len;
      return false;
    }
    for (size_t i = 0; i < len; i++) {
      buf_[head_] = data[i];
      head_ = (head_ + 1) % N;
    }
    count_ += len;
    return true;
  }

  // Copies the bytes the UART can take now (none while the previous burst is still in flight,
  // else up to the burst size and `cap`) into `out`, in order, and counts them as in flight.
  // nowUs: the clock read just now. The caller must hand exactly these bytes to ONE
  // UART::write() call, immediately.
  size_t take(uint32_t nowUs, uint8_t* out, size_t cap) {
    drain(nowUs);
    if (inFlight_ != 0 || count_ == 0) return 0;
    size_t n = count_ < burst_ ? count_ : burst_;
    if (n > cap) n = cap;
    for (size_t i = 0; i < n; i++) {
      out[i] = buf_[tail_];
      tail_ = (tail_ + 1) % N;
    }
    count_ -= n;
    inFlight_ = n;
    // The burst cannot have started before any time already seen: a stale caller timestamp
    // would make the model retire these bytes early and the next write would block.
    lastUs_ = seenUs_;
    return n;
  }

  // Nothing queued and the model says the last byte has left the wire.
  bool idle(uint32_t nowUs) {
    drain(nowUs);
    return count_ == 0 && inFlight_ == 0;
  }

  size_t used() const { return count_; }
  size_t freeBytes() const { return N - count_; }
  size_t inFlight() const { return inFlight_; }
  uint32_t dropped() const { return dropped_; }
  uint32_t droppedBytes() const { return droppedBytes_; }
  uint32_t byteUs() const { return byteUs_; }
  size_t burst() const { return burst_; }
  static constexpr size_t capacity() { return N; }

 private:
  // Retire one in-flight byte per byte time since lastUs_. While bytes are in flight lastUs_
  // advances in whole byte times, so no fraction of a byte time is ever lost or invented.
  void drain(uint32_t nowUs) {
    // A caller's timestamp older than one already seen (taken before an idle() call that used
    // a fresher micros()) would wrap to a huge elapsed time and retire bytes still in the
    // UART. Treat it as no time passed. Real gaps are far below 2^31 us (35 min).
    if ((int32_t)(nowUs - seenUs_) > 0) seenUs_ = nowUs;
    if (inFlight_ == 0 || byteUs_ == 0) return;
    const uint32_t elapsed = seenUs_ - lastUs_;
    const uint32_t done = elapsed / byteUs_;
    if (done >= inFlight_) {
      lastUs_ += (uint32_t)inFlight_ * byteUs_;
      inFlight_ = 0;
    } else {
      inFlight_ -= done;
      lastUs_ += done * byteUs_;
    }
  }

  uint8_t buf_[N];
  size_t head_ = 0;           // next write index
  size_t tail_ = 0;           // next byte to send
  size_t count_ = 0;          // bytes queued
  size_t burst_ = 0;
  size_t inFlight_ = 0;       // bytes handed to the UART and not yet on the wire
  uint32_t byteUs_ = 0;
  uint32_t lastUs_ = 0;       // when the oldest in-flight byte started
  uint32_t seenUs_ = 0;       // the newest timestamp any call has passed
  uint32_t dropped_ = 0;
  uint32_t droppedBytes_ = 0;
};

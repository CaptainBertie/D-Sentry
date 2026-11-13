// sim_wire.h -- one direction of a serial line inside dsentry_sim: bytes put on it at a virtual
// time arrive one byte time apart, in order, like an 8N1 UART wire (or a bridge buffer that
// forwards at the line rate). Header-only and free of config.h / dsentry_config.h, so both
// halves of the simulator (sim_main.cpp: RA4M1, sim_esp32.cpp: ESP32) can use it.
//
// SPDX-License-Identifier: MIT
#pragma once
#include <stddef.h>
#include <stdint.h>

#include <deque>
#include <string>
#include <utility>

class SimWire {
 public:
  // byteNs: time one byte occupies the line. capacity: bytes that may wait (0 = unbounded);
  // bytes beyond it are dropped and counted, as a full buffer drops them.
  SimWire(uint64_t byteNs, size_t capacity) : byteNs_(byteNs), capacity_(capacity) {}

  // Bytes that start leaving at nowUs (after anything already queued). Returns bytes accepted.
  size_t send(uint64_t nowUs, const uint8_t* data, size_t len) {
    const uint64_t nowNs = nowUs * kNsPerUs;
    size_t n = 0;
    for (size_t i = 0; i < len; i++) {
      if (capacity_ && q_.size() >= capacity_) {
        dropped_ += len - i;
        break;
      }
      lastNs_ = (lastNs_ > nowNs ? lastNs_ : nowNs) + byteNs_;
      q_.push_back(std::make_pair(lastNs_, data[i]));
      n++;
    }
    sent_ += n;
    return n;
  }
  size_t send(uint64_t nowUs, const std::string& s) {
    return send(nowUs, reinterpret_cast<const uint8_t*>(s.data()), s.size());
  }

  // Every byte whose stop bit has arrived by nowUs, in order.
  std::string take(uint64_t nowUs) {
    const uint64_t nowNs = nowUs * kNsPerUs;
    std::string out;
    while (!q_.empty() && q_.front().first <= nowNs) {
      out.push_back((char)q_.front().second);
      q_.pop_front();
    }
    return out;
  }

  bool empty() const { return q_.empty(); }
  void clear() {
    dropped_ += q_.size();
    q_.clear();
  }
  uint64_t sent() const { return sent_; }
  uint64_t dropped() const { return dropped_; }

 private:
  static constexpr uint64_t kNsPerUs = 1000;
  std::deque<std::pair<uint64_t, uint8_t>> q_;   // (arrival time ns, byte)
  uint64_t lastNs_ = 0;
  uint64_t byteNs_;
  size_t capacity_;
  uint64_t sent_ = 0;
  uint64_t dropped_ = 0;
};

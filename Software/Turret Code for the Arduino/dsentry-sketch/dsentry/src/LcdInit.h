// LcdInit.h -- the HD44780 re-initialisation both HALs run after a failed LCD transaction
// (hal.h LcdWrite::FAILED; hal_arduino.cpp on the board, sim/hal_host.cpp on the Mac).
//
// Why: a failed I2C transaction means the display can no longer be trusted. If the LCD was
// unplugged, its controller lost power and came back in its power-on state (8-bit interface,
// display off), where no character write ever shows. Re-running the datasheet's "initializing
// by instruction" sequence works from any state, so the HAL runs it after every failure:
//   probe   one transaction with only the backlight bit: is the backpack powered again?
//           then wait HD44780_POWER_ON_MS (the controller's power-on time)
//   reset   HD44780_RESET_NIBBLE as one nibble, HD44780_RESET_REPEATS times
//           (HD44780_RESET_FIRST_US after the first, HD44780_RESET_NEXT_US after the others)
//   4-bit   HD44780_4BIT_NIBBLE as one nibble
//   setup   function set (2 lines), display on, clear (HD44780_CLEAR_US), entry mode
// One step per call, and a call that comes before its step's wait has passed only returns
// AGAIN, so the sequence never blocks loop(): it spreads over the next ~55 ms of Lcd updates.
// A step that fails starts the sequence over (from the probe) on the following call. At the
// end the display is clear with the cursor at (0,0), the state LiquidCrystal_I2C::init() leaves.
//
// SPDX-License-Identifier: MIT
#pragma once
#include <stddef.h>
#include <stdint.h>
#include "hal.h"

namespace lcdinit {

enum Kind : uint8_t {
  PROBE,     // one transaction carrying only the backlight bit
  NIBBLE,    // one 4-bit write: the high nibble of `value` (E pulse high, then low)
  COMMAND    // one command byte as two nibbles (RS = 0)
};

struct Step {
  Kind kind;
  uint8_t value;
  uint32_t waitUs;   // how long after this step the next one may run
};

// The steps after the reset nibbles.
constexpr Step kTail[] = {
    {NIBBLE, HD44780_4BIT_NIBBLE, HD44780_CMD_US},
    {COMMAND, HD44780_CMD_FUNCTION_2LINE, HD44780_CMD_US},
    {COMMAND, HD44780_CMD_DISPLAY_ON, HD44780_CMD_US},
    {COMMAND, HD44780_CMD_CLEAR, HD44780_CLEAR_US},
    {COMMAND, HD44780_CMD_ENTRY_INC, HD44780_CMD_US},
};
constexpr size_t kTailSteps = sizeof kTail / sizeof kTail[0];
// probe + the reset nibbles + the tail
constexpr size_t kSteps = 1 + HD44780_RESET_REPEATS + kTailSteps;
static_assert(HD44780_RESET_REPEATS >= 1, "the reset needs at least one 8-bit function set");

// Step i of the sequence, 0 <= i < kSteps.
inline Step stepAt(size_t i) {
  if (i == 0) return Step{PROBE, 0, (uint32_t)(HD44780_POWER_ON_MS * US_PER_MS)};
  if (i <= HD44780_RESET_REPEATS)
    return Step{NIBBLE, HD44780_RESET_NIBBLE,
                i == 1 ? (uint32_t)HD44780_RESET_FIRST_US : (uint32_t)HD44780_RESET_NEXT_US};
  return kTail[i - 1 - HD44780_RESET_REPEATS];
}

class Sequencer {
 public:
  void start() {
    active_ = true;
    index_ = 0;
    waiting_ = false;
  }
  bool active() const { return active_; }
  size_t index() const { return index_; }   // the next step to run

  // One call of the HAL's lcdWriteAt while active(). send(kind, value) makes the step's single
  // transaction and returns whether the backpack acknowledged it. Returns AGAIN while waiting
  // or after a step succeeded (active() turns false after the last one), FAILED when a step
  // was refused (the next call starts again from the probe).
  template <typename Send>
  hal::LcdWrite run(uint32_t nowUs, Send send) {
    if (!active_) return hal::LcdWrite::AGAIN;
    if (waiting_ && (int32_t)(nowUs - dueUs_) < 0) return hal::LcdWrite::AGAIN;
    const Step s = stepAt(index_);
    if (!send(s.kind, s.value)) {
      index_ = 0;
      waiting_ = false;
      return hal::LcdWrite::FAILED;
    }
    index_++;
    waiting_ = true;
    dueUs_ = nowUs + s.waitUs;
    if (index_ >= kSteps) active_ = false;   // the clear's wait is covered by the last step's
    return hal::LcdWrite::AGAIN;
  }

 private:
  bool active_ = false;
  size_t index_ = 0;
  bool waiting_ = false;
  uint32_t dueUs_ = 0;
};

}  // namespace lcdinit

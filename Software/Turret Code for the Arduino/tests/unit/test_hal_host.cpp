// test_hal_host.cpp -- proves the host HAL fake (sim/hal_host.cpp) behaves like the board's
// hal_arduino.cpp where the module tests will rely on it: virtual time, the stepper model
// (exact periods, buffered speed changes, direction after the next step, bounds, lost steps
// against the tilt stop and with the drivers off), the step-quiet window, the paced serial
// pipes, the LCD cursor contract, LEDs, servo, PWM, injected inputs and the settings store.
//
// SPDX-License-Identifier: MIT
#include <string>

#include "hal.h"
#include "LcdInit.h"
#include "hal_host.h"
#include "testkit.h"

namespace {

// The pacer's byte time (TxQueue.h): 10 bits per byte at 115200 baud, rounded up (87 us), plus
// 1 us for micros()' resolution = 88 us.
constexpr uint32_t kByteUs = (UART_BITS_PER_BYTE * US_PER_S + DEBUG_SERIAL_BAUD - 1) / DEBUG_SERIAL_BAUD + 1u;

void boot() {
  hal_host::reset();
  hal::begin();
}

}  // namespace

TEST(virtual_time_moves_only_when_told) {
  boot();
  CHECK_EQ(hal::nowUs(), 0u);
  hal_host::advanceUs(1500);
  CHECK_EQ(hal::nowUs(), 1500u);
  CHECK_EQ(hal::nowMs(), 1u);
  hal_host::advanceMs(2);
  CHECK_EQ(hal::nowUs(), 3500u);
  CHECK_EQ(hal::nowMs(), 3u);
  hal_host::setTimeUs(1000);   // never backwards
  CHECK_EQ(hal_host::timeUs(), 3500u);
}

TEST(begin_leaves_everything_safe) {
  boot();
  const hal::BeginReport& r = hal::beginReport();
  CHECK(r.panTimer);
  CHECK(r.tiltTimer);
  CHECK(r.servo);
  CHECK(r.lcd);
  CHECK_EQ(r.lcdAddress, LCD_I2C_ADDR);
  CHECK(!hal::steppersEnabled());
  CHECK(hal_host::pinLevel(PIN_STEPPER_EN));   // EN high = disabled
  CHECK(!hal::servoEnabled());
  CHECK_EQ(hal_host::servoPulseUs(), 0u);
  CHECK_EQ(hal_host::pwmDuty(PIN_FLY_ENA), 0);
  CHECK(!hal_host::running(AXIS_PAN));
  CHECK(!hal_host::running(AXIS_TILT));
  CHECK_EQ(hal::stepQuietUs(), UINT32_MAX);
}

TEST(stepper_steps_at_exact_period) {
  boot();
  hal::stepperEnable(true);
  hal::stepSetRate(AXIS_PAN, 1000.0f);   // 48000 counts: first edge 1 ms after the call
  CHECK_EQ(hal_host::periodCounts(AXIS_PAN), GPT_CLOCK_HZ / 1000u);
  hal_host::advanceUs(999);
  CHECK_EQ(hal::stepPosition(AXIS_PAN), 0);
  hal_host::advanceUs(1);
  CHECK_EQ(hal::stepPosition(AXIS_PAN), 1);
  hal_host::setTimeUs(1000000);
  CHECK_EQ(hal::stepPosition(AXIS_PAN), 1000);
  CHECK_EQ(hal_host::trueSteps(AXIS_PAN), 1000);
  CHECK_EQ(hal_host::lostSteps(AXIS_PAN), 0u);
  CHECK_NEAR(hal::stepRate(AXIS_PAN), 1000.0f, 0.0f);
}

TEST(rate_clamps_and_stops) {
  boot();
  hal::stepSetRate(AXIS_TILT, STEP_MAX_RATE_HZ * 2.0f);
  CHECK_NEAR(hal::stepRate(AXIS_TILT), STEP_MAX_RATE_HZ, 0.0f);
  hal::stepSetRate(AXIS_TILT, -STEP_MAX_RATE_HZ * 2.0f);
  CHECK_NEAR(hal::stepRate(AXIS_TILT), -STEP_MAX_RATE_HZ, 0.0f);
  hal::stepSetRate(AXIS_TILT, STEP_MIN_RATE_HZ * 0.5f);   // too slow: stop
  CHECK(!hal_host::running(AXIS_TILT));
  CHECK_NEAR(hal::stepRate(AXIS_TILT), 0.0f, 0.0f);
  hal::stepSetRate(AXIS_TILT, STEP_MIN_RATE_HZ);
  CHECK(hal_host::running(AXIS_TILT));
  hal::stepSetRate(AXIS_TILT, 0.0f);
  CHECK(!hal_host::running(AXIS_TILT));
}

TEST(speed_change_lands_at_the_next_step) {
  boot();
  hal::stepperEnable(true);
  hal::stepSetRate(AXIS_PAN, 100.0f);   // 10 ms period: first edge at 10 ms
  hal_host::advanceUs(1000);
  hal::stepSetRate(AXIS_PAN, 1000.0f);  // buffered: the running 10 ms cycle completes first
  hal_host::setTimeUs(9999);
  CHECK_EQ(hal::stepPosition(AXIS_PAN), 0);
  hal_host::setTimeUs(15000);           // edges at 10, 11, 12, 13, 14, 15 ms
  CHECK_EQ(hal::stepPosition(AXIS_PAN), 6);
}

TEST(direction_change_lands_after_the_next_step) {
  boot();
  hal::stepperEnable(true);
  hal::stepSetRate(AXIS_PAN, 1000.0f);
  hal_host::setTimeUs(10500);
  CHECK_EQ(hal::stepPosition(AXIS_PAN), 10);
  hal::stepSetRate(AXIS_PAN, -1000.0f);
  CHECK_EQ(hal_host::dirPin(AXIS_PAN), 1);
  hal_host::setTimeUs(11000);           // this step still goes +1, then DIR flips
  CHECK_EQ(hal::stepPosition(AXIS_PAN), 11);
  CHECK_EQ(hal_host::dirPin(AXIS_PAN), -1);
  hal_host::setTimeUs(13000);
  CHECK_EQ(hal::stepPosition(AXIS_PAN), 9);
}

TEST(bounds_refuse_steps_and_count_them) {
  boot();
  hal::stepperEnable(true);
  StepStats s;
  hal::stepTakeStats(s);
  hal::stepSetBounds(AXIS_PAN, true, 0, 100);
  hal::stepSetRate(AXIS_PAN, 1000.0f);
  hal_host::setTimeUs(1000000);
  CHECK_EQ(hal::stepPosition(AXIS_PAN), 100);
  CHECK_EQ(hal_host::trueSteps(AXIS_PAN), 100);
  hal::stepTakeStats(s);
  CHECK_EQ(s.steps[AXIS_PAN], 100u);
  CHECK_EQ(s.refusals[AXIS_PAN], 900u);
  CHECK_EQ(s.samples[AXIS_PAN], 1000u);
  CHECK_EQ(s.windowUs, 1000000u);
  CHECK_EQ(s.jitterMaxPermille[AXIS_PAN], 0);
  // Reverse: the next edge is refused (still +1 at the bound), then the axis comes back.
  hal::stepSetRate(AXIS_PAN, -1000.0f);
  hal_host::advanceMs(11);
  CHECK_EQ(hal::stepPosition(AXIS_PAN), 90);
  hal::stepTakeStats(s);
  CHECK_EQ(s.refusals[AXIS_PAN], 1u);
  CHECK_EQ(s.steps[AXIS_PAN], 10u);
  // Bounds off: free again.
  hal::stepSetBounds(AXIS_PAN, false, 0, 0);
  hal::stepSetPosition(AXIS_PAN, -5);
  hal_host::advanceMs(5);
  CHECK_EQ(hal::stepPosition(AXIS_PAN), -10);
}

TEST(tilt_bottom_stop_loses_steps_but_the_counter_counts) {
  boot();
  hal::stepperEnable(true);
  const int8_t down = TILT_DIR_INVERT ? 1 : -1;
  hal_host::setTrueSteps(AXIS_TILT, -down * 100);   // 100 steps above the stop
  hal_host::setTiltBottomStop(true, 0);
  hal::stepSetRate(AXIS_TILT, down * 1000.0f);
  hal_host::setTimeUs(200000);
  CHECK_EQ(hal::stepPosition(AXIS_TILT), down * 200);   // what the firmware believes
  CHECK_EQ(hal_host::trueSteps(AXIS_TILT), 0);          // where the barrel is
  CHECK_EQ(hal_host::lostSteps(AXIS_TILT), 100u);
  // Upward motion is not blocked.
  hal::stepSetRate(AXIS_TILT, -down * 1000.0f);
  hal_host::advanceMs(51);   // the first edge after the change is still downward (lost)
  CHECK_EQ(hal_host::trueSteps(AXIS_TILT), -down * 50);
}

TEST(drivers_off_loses_every_step) {
  boot();
  hal::stepSetRate(AXIS_PAN, 1000.0f);
  hal_host::setTimeUs(10000);
  CHECK_EQ(hal::stepPosition(AXIS_PAN), 10);
  CHECK_EQ(hal_host::trueSteps(AXIS_PAN), 0);
  CHECK_EQ(hal_host::lostSteps(AXIS_PAN), 10u);
}

TEST(quiet_window_is_time_to_the_next_edge) {
  boot();
  hal::stepSetRate(AXIS_PAN, 1000.0f);
  CHECK_EQ(hal::stepQuietUs(), 1000u);
  hal_host::advanceUs(400);
  CHECK_EQ(hal::stepQuietUs(), 600u);
  hal::stepSetRate(AXIS_TILT, 500.0f);   // first tilt edge at 400 + 2000 us
  CHECK_EQ(hal::stepQuietUs(), 600u);
  hal_host::advanceUs(700);              // pan edge at 1000 passed; next at 2000
  CHECK_EQ(hal::stepQuietUs(), 900u);
  hal::stepSetRate(AXIS_PAN, 0.0f);
  CHECK_EQ(hal::stepQuietUs(), 1300u);   // tilt only
  hal::stepSetRate(AXIS_TILT, 0.0f);
  CHECK_EQ(hal::stepQuietUs(), UINT32_MAX);
}

TEST(timer_failure_keeps_the_steppers_still) {
  hal_host::reset();
  hal_host::setBeginTimerFailure(true);
  CHECK(!hal::begin());
  CHECK(!hal::beginReport().panTimer);
  hal::stepSetRate(AXIS_PAN, 1000.0f);
  hal_host::advanceMs(100);
  CHECK_EQ(hal::stepPosition(AXIS_PAN), 0);
  CHECK(!hal_host::running(AXIS_PAN));
}

TEST(usb_rx_pipe_and_overflow) {
  boot();
  CHECK_EQ(hal_host::usbInject(std::string("hello")), 5u);
  uint8_t buf[8];
  CHECK_EQ(hal::usbRead(buf, 3), 3u);
  CHECK_EQ(std::string((const char*)buf, 3), std::string("hel"));
  CHECK_EQ(hal::usbRead(buf, sizeof buf), 2u);
  CHECK_EQ(std::string((const char*)buf, 2), std::string("lo"));
  CHECK_EQ(hal::usbRead(buf, sizeof buf), 0u);
  std::string big(SERIAL_RX_BUFFER_BYTES + 10, 'x');
  CHECK_EQ(hal_host::usbInject(big), (size_t)SERIAL_RX_BUFFER_BYTES);
  CHECK_EQ(hal_host::usbRxDropped(), 10u);
  CHECK_EQ(hal_host::linkInject(std::string("OK\r\n")), 4u);
  CHECK_EQ(hal::linkRead(buf, sizeof buf), 4u);
}

TEST(usb_tx_is_paced_one_byte_per_byte_time) {
  boot();
  const std::string msg = "abcdef";
  CHECK(hal::usbWrite((const uint8_t*)msg.data(), msg.size()));
  CHECK_EQ(hal::usbTxFree(), (size_t)DEBUG_TX_BUFFER_BYTES - msg.size());
  CHECK_EQ(hal_host::usbTakeTx(), std::string());   // nothing leaves before service()
  std::string wire;
  int calls = 0;
  while (wire.size() < msg.size() && calls < 100) {
    hal::service(hal::nowUs());
    wire += hal_host::usbTakeTx();
    calls++;
    hal_host::advanceUs(kByteUs);
  }
  CHECK_EQ(wire, msg);
  CHECK_EQ(calls, 6);   // SERIAL_TX_SLACK_BYTES = 1: one byte per byte time, never a burst
  // Calls closer together than a byte time release nothing new.
  CHECK(hal::usbWrite((const uint8_t*)"xy", 2));
  hal::service(hal::nowUs());
  CHECK_EQ(hal_host::usbTakeTx(), std::string("x"));
  hal_host::advanceUs(kByteUs - 1);
  hal::service(hal::nowUs());
  CHECK_EQ(hal_host::usbTakeTx(), std::string());
  hal_host::advanceUs(1);
  hal::service(hal::nowUs());
  CHECK_EQ(hal_host::usbTakeTx(), std::string("y"));
}

TEST(usb_tx_drops_whole_messages_that_do_not_fit) {
  boot();
  std::string big(DEBUG_TX_BUFFER_BYTES + 1, 'z');
  CHECK(!hal::usbWrite((const uint8_t*)big.data(), big.size()));
  CHECK_EQ(hal::usbTxDropped(), 1u);
  CHECK_EQ(hal::usbTxFree(), (size_t)DEBUG_TX_BUFFER_BYTES);   // nothing partial was queued
  std::string fits(DEBUG_TX_BUFFER_BYTES, 'q');
  CHECK(hal::usbWrite((const uint8_t*)fits.data(), fits.size()));
  CHECK(!hal::usbWrite((const uint8_t*)"!", 1));
  CHECK_EQ(hal::usbTxDropped(), 2u);
}

TEST(link_tx_fills_the_fifo_then_paces) {
  boot();
  std::string req(20, 'a');
  CHECK(hal::linkTxIdle());
  CHECK(hal::linkWrite((const uint8_t*)req.data(), req.size()));
  CHECK(!hal::linkTxIdle());
  hal::service(hal::nowUs());
  CHECK_EQ(hal_host::linkTakeTx().size(), (size_t)SCI_FIFO_DEPTH);   // one FIFO-full burst
  hal_host::advanceUs((uint64_t)SCI_FIFO_DEPTH * kByteUs);
  hal::service(hal::nowUs());
  CHECK_EQ(hal_host::linkTakeTx().size(), req.size() - SCI_FIFO_DEPTH);
  CHECK(!hal::linkTxIdle());   // the last bytes are still shifting out
  hal_host::advanceUs((uint64_t)(req.size() - SCI_FIFO_DEPTH) * kByteUs);
  CHECK(hal::linkTxIdle());
  CHECK_EQ(hal::linkTxFree(), (size_t)LINK_TX_BUFFER_BYTES);
}

// A service() timestamp older than one the queue has already seen (linkTxIdle() reads a
// fresher clock) must not count as a huge elapsed time: on the board that would push bytes
// into a FIFO that still holds bytes, and UART::write would block. (Corrected by the link-hal
// stage: the core opens SCI1 with FCR.TTRG = 0, so a write is non-blocking only into an EMPTY
// FIFO; half free is not enough. Evidence in TxQueue.h.)
TEST(link_tx_ignores_a_stale_timestamp) {
  boot();
  const uint32_t t0 = hal::nowUs();
  std::string req((size_t)SCI_FIFO_DEPTH * 2, 'a');
  CHECK(hal::linkWrite((const uint8_t*)req.data(), req.size()));
  hal::service(t0);
  CHECK_EQ(hal_host::linkTakeTx().size(), (size_t)SCI_FIFO_DEPTH);
  hal_host::advanceUs((uint64_t)(SCI_FIFO_DEPTH / 2) * kByteUs);
  CHECK(!hal::linkTxIdle());   // retires the half that has left
  hal::service(t0);            // stale, and the FIFO still holds bytes: nothing
  CHECK_EQ(hal_host::linkTakeTx().size(), (size_t)0);
  hal_host::advanceUs((uint64_t)(SCI_FIFO_DEPTH / 2) * kByteUs);
  hal::service(t0);            // the HAL paces by its own clock: the FIFO has emptied
  CHECK_EQ(hal_host::linkTakeTx().size(), (size_t)SCI_FIFO_DEPTH);
  hal::service(hal::nowUs());  // the FIFO is full again: nothing more until it empties
  CHECK_EQ(hal_host::linkTakeTx().size(), (size_t)0);
  CHECK_EQ(hal_host::linkTxProbe().blockingWrites, 0u);
}

TEST(lcd_cursor_contract) {
  boot();
  CHECK(hal::lcdPresent());
  CHECK(hal::lcdWriteAt(0, 0, 'A') == hal::LcdWrite::WROTE);   // cursor starts at (0,0) after init
  CHECK(hal::lcdWriteAt(1, 0, 'B') == hal::LcdWrite::WROTE);   // auto-increment: no cursor command
  CHECK(hal::lcdWriteAt(5, 1, 'C') == hal::LcdWrite::AGAIN);   // cursor command only
  CHECK(hal::lcdWriteAt(5, 1, 'C') == hal::LcdWrite::WROTE);
  CHECK_EQ(hal_host::lcdTransactions(), 4u);
  CHECK_EQ(hal_host::lcdCharsWritten(), 3u);
  CHECK_EQ(hal_host::lcdRow(0), std::string("AB              "));
  CHECK_EQ(hal_host::lcdRow(1), std::string("     C          "));
  CHECK(hal::lcdWriteAt(LCD_COLS, 0, 'X') == hal::LcdWrite::FAILED);   // off-screen: nothing sent
  CHECK_EQ(hal_host::lcdTransactions(), 4u);
  CHECK(hal::lcdBacklight(false));
  CHECK(!hal_host::lcdBacklightOn());
  CHECK_EQ(hal_host::lcdTransactions(), 5u);
  CHECK_EQ(hal::lcdErrors(), 0u);
}

// A refused transaction is never reported as written, and the following calls run the
// re-initialisation (LcdInit.h) one step at a time, respecting its waits, before any
// character goes out again.
TEST(lcd_failure_is_reported_and_reinitialises) {
  boot();
  hal_host::failLcdTransactions(1);
  CHECK(hal::lcdWriteAt(0, 0, 'A') == hal::LcdWrite::FAILED);
  CHECK_EQ(hal::lcdErrors(), 1u);
  CHECK_EQ(hal_host::lcdCharsWritten(), 0u);
  // Re-init: every call returns AGAIN; calls before a step's wait has passed send nothing.
  const uint64_t t0 = hal_host::timeUs();
  uint32_t calls = 0;
  const uint32_t tx0 = hal_host::lcdTransactions();
  while (hal::lcdReinits() == 0 && calls < 100000) {
    CHECK(hal::lcdWriteAt(0, 0, 'A') == hal::LcdWrite::AGAIN);
    hal_host::advanceUs(10);
    calls++;
  }
  CHECK_EQ(hal::lcdReinits(), 1u);
  CHECK_EQ(hal_host::lcdTransactions() - tx0, (uint32_t)lcdinit::kSteps);   // one per step
  CHECK(hal_host::timeUs() - t0 >= (uint64_t)HD44780_POWER_ON_MS * US_PER_MS + HD44780_RESET_FIRST_US +
                                       HD44780_CLEAR_US);
  CHECK(hal_host::lcdControllerReady());
  CHECK(hal::lcdWriteAt(0, 0, 'A') == hal::LcdWrite::WROTE);   // the clear left the cursor home
  CHECK_EQ(hal_host::lcdRow(0), std::string("A               "));
}

TEST(lcd_unplugged_controller_needs_the_whole_reinit) {
  boot();
  CHECK(hal::lcdWriteAt(0, 0, 'A') == hal::LcdWrite::WROTE);
  hal_host::setLcdAttached(false);
  CHECK(hal::lcdWriteAt(1, 0, 'B') == hal::LcdWrite::FAILED);
  CHECK(!hal_host::lcdControllerReady());
  CHECK_EQ(hal_host::lcdRow(0), std::string(LCD_COLS, ' '));   // dark
  // Still unplugged: the probe fails and the sequence starts over each time.
  hal_host::advanceUs(US_PER_S);
  CHECK(hal::lcdWriteAt(1, 0, 'B') == hal::LcdWrite::FAILED);
  CHECK(hal::lcdBacklight(true));   // during a re-init: no transaction, the bit is kept
  hal_host::setLcdAttached(true);
  uint32_t guard = 0;
  while (hal::lcdReinits() == 0 && guard++ < 100000) {
    hal::lcdWriteAt(1, 0, 'B');
    hal_host::advanceUs(10);
  }
  CHECK(hal_host::lcdControllerReady());
  CHECK_EQ(hal_host::lcdRow(0), std::string(LCD_COLS, ' '));   // cleared by the re-init
  CHECK(hal::lcdWriteAt(0, 0, 'Z') == hal::LcdWrite::WROTE);
  CHECK_EQ(hal_host::lcdRow(0), std::string("Z               "));
}

// Unplugged again in the middle of a re-initialisation: the sequence starts over from the
// probe (and its power-on wait), because the controller lost the steps it had already seen.
TEST(lcd_reinit_interrupted_starts_over_from_the_probe) {
  boot();
  hal_host::failLcdTransactions(1);
  CHECK(hal::lcdWriteAt(0, 0, 'A') == hal::LcdWrite::FAILED);
  // Probe and the first reset nibble go through.
  uint32_t guard = 0;
  while (hal_host::lcdTransactions() < 3 && guard++ < 100000) {
    hal::lcdWriteAt(0, 0, 'A');
    hal_host::advanceUs(10);
  }
  hal_host::setLcdAttached(false);
  guard = 0;
  while (hal::lcdWriteAt(0, 0, 'A') != hal::LcdWrite::FAILED && guard++ < 100000) hal_host::advanceUs(10);
  hal_host::setLcdAttached(true);
  guard = 0;
  while (hal::lcdReinits() == 0 && guard++ < 100000) {
    hal::lcdWriteAt(0, 0, 'A');
    hal_host::advanceUs(10);
  }
  CHECK_EQ(hal::lcdReinits(), 1u);
  CHECK(hal_host::lcdControllerReady());   // the full reset reached the controller
  CHECK(hal::lcdWriteAt(0, 0, 'Q') == hal::LcdWrite::WROTE);
  CHECK_EQ(hal_host::lcdRow(0), std::string("Q               "));
}

TEST(lcd_absent) {
  hal_host::reset();
  hal_host::setLcdPresent(false, 0);
  hal::begin();
  CHECK(!hal::lcdPresent());
  CHECK(!hal::beginReport().lcd);
  CHECK_EQ(hal::lcdAddress(), 0);
  CHECK(hal::lcdWriteAt(0, 0, 'A') == hal::LcdWrite::FAILED);
  CHECK(!hal::lcdBacklight(true));
  CHECK_EQ(hal_host::lcdTransactions(), 0u);
}

TEST(leds_guard_and_unsafe_show_count) {
  boot();
  CHECK_EQ(hal_host::ledShowCount(), 1u);   // begin() blanks them
  hal::ledsSetPixel(0, 200, 10, 0);         // 200 is above LED_MAX_BRIGHTNESS
  CHECK_EQ(hal_host::ledMaxChannel(), 200);
  CHECK_EQ(hal_host::ledOverLimit(), 1u);
  CHECK_EQ(hal::ledsOverLimit(), 1u);
  CHECK_EQ(hal_host::ledSet(0).r, LED_MAX_BRIGHTNESS);
  CHECK_EQ(hal_host::ledShown(0).r, 0);
  hal::ledsShow();
  CHECK_EQ(hal_host::ledShown(0).r, LED_MAX_BRIGHTNESS);
  CHECK_EQ(hal_host::ledShown(0).g, 10);
  CHECK_EQ(hal_host::ledUnsafeShows(), 0u);
  hal::stepSetRate(AXIS_PAN, 1000.0f);
  hal_host::advanceUs(900);                 // an edge is due in 100 us < LED_SHOW_GUARD_US
  hal::ledsShow();
  CHECK_EQ(hal_host::ledUnsafeShows(), 1u);
  hal::ledsSetPixel(LED_COUNT, 1, 1, 1);    // out of range: ignored
  CHECK_EQ(hal_host::ledShowCount(), 3u);
}

TEST(servo_pwm_and_inputs) {
  boot();
  hal::servoEnable(true);
  hal::servoWriteDeg(90.0f);
  CHECK_EQ(hal_host::servoPulseUs(),
           (uint32_t)(TRIGGER_SERVO_MIN_US + (TRIGGER_SERVO_MAX_US - TRIGGER_SERVO_MIN_US) / 2.0f + 0.5f));
  hal::servoWriteDeg(500.0f);
  CHECK_EQ(hal_host::servoPulseUs(), (uint32_t)TRIGGER_SERVO_MAX_US);
  hal::servoWriteDeg(-5.0f);
  CHECK_EQ(hal_host::servoPulseUs(), (uint32_t)TRIGGER_SERVO_MIN_US);
  hal::servoEnable(false);
  CHECK_EQ(hal_host::servoPulseUs(), 0u);
  CHECK_NEAR(hal::servoDeg(), 0.0f, 0.0f);

  CHECK(hal::pwmWrite(PIN_FLY_ENA, FLYWHEEL_DUTY_255));
  CHECK_EQ(hal_host::pwmDuty(PIN_FLY_ENA), FLYWHEEL_DUTY_255);
  CHECK(!hal::pwmWrite(PIN_TILT_STEP, 10));   // GPT1: a step timer
  CHECK(!hal::pwmWrite(PIN_FLY_IN2, 10));     // GPT6: the servo's timer
  CHECK_EQ(hal_host::pwmRefused(), 2u);

  CHECK_EQ(hal::readPin(PIN_TOGGLE_POWER), TOGGLE_ACTIVE_LOW);   // open toggle
  hal_host::setDigitalInput(PIN_TOGGLE_POWER, !TOGGLE_ACTIVE_LOW);
  CHECK_EQ(hal::readPin(PIN_TOGGLE_POWER), !TOGGLE_ACTIVE_LOW);
  hal_host::setAnalog(PIN_STALL_SENSE, 700);
  CHECK_EQ(hal::readAnalog(PIN_STALL_SENSE), 700);
  hal_host::setAnalog(PIN_STALL_SENSE, 5000);
  CHECK_EQ(hal::readAnalog(PIN_STALL_SENSE), ADC_MAX_COUNTS);

  hal::writePin(PIN_FLY_IN1, true);
  CHECK(hal_host::pinLevel(PIN_FLY_IN1));
  CHECK_EQ(hal_host::pinWrites(PIN_FLY_IN1), 1u);
}

namespace {
// A scripted run touching the stepper model and both pipes; returns everything observable.
std::string scriptedRun() {
  boot();
  hal::stepperEnable(true);
  hal::stepSetBounds(AXIS_TILT, true, -50, 50);
  hal::stepSetRate(AXIS_PAN, 1234.5f);
  hal::stepSetRate(AXIS_TILT, -777.0f);
  std::string out;
  for (int i = 0; i < 2000; i++) {
    if (i % 100 == 0) hal::usbWrite((const uint8_t*)"tick\n", 5);
    if (i == 700) hal::stepSetRate(AXIS_PAN, -2500.0f);
    if (i == 1300) hal::stepSetRate(AXIS_TILT, 300.0f);
    hal::service(hal::nowUs());
    hal_host::advanceUs(37);
  }
  out += hal_host::usbTakeTx();
  out += "|" + std::to_string(hal::stepPosition(AXIS_PAN)) + "|" + std::to_string(hal::stepPosition(AXIS_TILT));
  out += "|" + std::to_string(hal_host::totalRefusals(AXIS_TILT)) + "|" + std::to_string(hal::stepQuietUs());
  return out;
}
}  // namespace

TEST(deterministic) {
  const std::string a = scriptedRun();
  const std::string b = scriptedRun();
  CHECK_EQ(a, b);
  CHECK(a.find("tick\ntick\n") != std::string::npos);
}

// The settings store (SERVO ADJUST's data flash): blank after reset(), kept by reboot(), a save
// blocks for storeSaveUs() of virtual time while the steppers keep stepping, sizes past
// SETTINGS_STORE_MAX_BYTES are refused, a failing flash returns false and writes nothing.
TEST(settings_store_is_blank_after_reset_kept_by_reboot_and_a_save_blocks) {
  boot();
  uint8_t buf[SETTINGS_STORE_MAX_BYTES];
  CHECK(hal::storeLoad(buf, sizeof buf));
  for (uint8_t b : buf) CHECK_EQ(b, (uint8_t)BYTE_MASK);
  CHECK(!hal::storeLoad(buf, sizeof buf + 1));
  CHECK(!hal::storeSave(buf, sizeof buf + 1));
  CHECK(!hal::storeSave(buf, 0));
  const uint8_t rec[] = {7, 8, 9};
  hal::stepperEnable(true);
  hal::stepSetRate(AXIS_PAN, 1000.0f);   // one step a ms
  const uint64_t t0 = hal_host::timeUs();
  const uint32_t steps0 = hal_host::totalSteps(AXIS_PAN);
  CHECK(hal::storeSave(rec, sizeof rec));
  CHECK_EQ(hal_host::timeUs() - t0, hal_host::kStoreSaveTypicalUs);   // it blocked ...
  const uint32_t stepped = hal_host::totalSteps(AXIS_PAN) - steps0;    // ... and the steps went on
  CHECK(stepped + 1 >= (uint32_t)(hal_host::kStoreSaveTypicalUs / US_PER_MS));
  CHECK_EQ(hal_host::storeSaves(), 1u);
  CHECK_EQ(hal_host::storeByte(1), (uint8_t)8);
  CHECK_EQ(hal_host::storeByte(sizeof rec), (uint8_t)BYTE_MASK);        // the rest untouched
  hal_host::reboot();
  hal::begin();
  CHECK_EQ(hal_host::timeUs(), 0u);
  CHECK_EQ(hal_host::storeSaves(), 0u);
  CHECK(hal::storeLoad(buf, sizeof rec));
  CHECK(buf[0] == 7 && buf[1] == 8 && buf[2] == 9);
  hal_host::storeSetByte(2, 0);           // corrupt one byte
  CHECK(hal::storeLoad(buf, sizeof rec));
  CHECK_EQ(buf[2], (uint8_t)0);
  hal_host::setStoreSaveUs(0);
  hal_host::failStoreSaves(true);
  const uint8_t other[] = {1, 2, 3};
  CHECK(!hal::storeSave(other, sizeof other));
  CHECK_EQ(hal_host::storeByte(0), (uint8_t)7);
  hal_host::failStoreSaves(false);
  hal_host::reset();
  CHECK_EQ(hal_host::storeByte(0), (uint8_t)BYTE_MASK);
  CHECK_EQ(hal_host::storeSaveUs(), hal_host::kStoreSaveTypicalUs);
}

TESTKIT_MAIN()

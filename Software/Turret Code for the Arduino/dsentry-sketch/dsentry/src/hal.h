// hal.h -- the only door between D-Sentry's portable logic and the hardware.
//
// Two implementations, same API:
//   hal_arduino.cpp   the UNO R4 WiFi (compiled only when ARDUINO is defined)
//   sim/hal_host.cpp  the Mac: virtual time, a stepper physics model, captured outputs and
//                     byte pipes, for the unit tests and dsentry_sim
// Every other module includes this header and nothing from Arduino (architecture.md rule 4).
//
// Rules the implementations keep, so callers never have to think about them:
//   - nothing here blocks except begin() and storeSave(); every other call from loop() returns
//     in microseconds (lcdWriteAt ~0.5 ms for its one I2C transaction, readAnalog ~10 us).
//     storeSave() is the one documented blocking call (about 50 ms, see below), made only by
//     SERVO ADJUST's save in DISARMED with no stroke running and no axis moving;
//   - serial output is queued and paced by service(); a message that does not fit its queue
//     is dropped whole and counted, never split and never waited for;
//   - steppers run from timer interrupts; the API sets rates, bounds and positions only.
//
// SPDX-License-Identifier: MIT
#pragma once
#include <stddef.h>
#include <stdint.h>
#include "Types.h"

namespace hal {

// What begin() managed to bring up, for the boot log and the self-test.
struct BeginReport {
  bool panTimer = false;    // GPT STEP_TIMER_PAN_CHANNEL claimed, interrupt at priority
  bool tiltTimer = false;   //   STEP_TIMER_IRQ_PRIORITY, opened (stopped until a rate is set)
  bool servo = false;       // PwmOut on PIN_TRIGGER_SERVO began (parked LOW = detached)
  bool lcd = false;         // an LCD answered at LCD_I2C_ADDR or LCD_I2C_ADDR_FALLBACK
  uint8_t lcdAddress = 0;   // the address that answered, 0 when none
};

// ---- lifecycle -------------------------------------------------------------------------------
// setup() only, before any module: claims the step timers first (GPT0/GPT1 by force, before
// anything else can allocate a timer), then the servo, flywheel PWM, pins, UARTs, LEDs (all
// off) and the LCD (init blocks about 1 s). Leaves every output safe: drivers disabled (EN
// high), flywheel coasting at duty 0, servo parked, LEDs dark. Returns false when a step timer
// could not be claimed; the steppers then never move (stepSetRate does nothing).
bool begin();
const BeginReport& beginReport();

// Call often from loop() (App calls it between modules): moves queued serial bytes to the
// UARTs at no more than the line rate, so UART::write never blocks. Does nothing else.
// Each call makes at most one UART::write per port: one byte on Serial (no FIFO), up to
// SCI_FIFO_DEPTH bytes on LINK_SERIAL and only once its FIFO is empty (the core opens SCI1
// with FCR.TTRG = 0; TxQueue.h). So the USB rate is also bounded by how often this runs.
// nowUs is not used for pacing: the HAL reads its own clock right before each write, because
// a timestamp taken earlier in loop() would let the next byte in while the UART is still busy.
void service(uint32_t nowUs);

// ---- time -----------------------------------------------------------------------------------
uint32_t nowMs();   // milliseconds since boot (millis()), wraps after 49.7 days
uint32_t nowUs();   // microseconds since boot (micros()), wraps after 71.6 minutes

// ---- GPIO, ADC, PWM ---------------------------------------------------------------------------
// begin() already configures every pin config.h names; these are for the modules' writes and
// reads. Pins are config.h PIN_* numbers.
void writePin(uint8_t pin, bool high);
bool readPin(uint8_t pin);           // true = HIGH
uint16_t readAnalog(uint8_t pin);    // 0..ADC_MAX_COUNTS; ~10 us busy-wait, loop() only, never
                                     // from an interrupt
// 8-bit PWM (0..PWM_MAX). Only PIN_FLY_ENA is accepted: every other PWM-capable pin shares a
// GPT channel with a step timer or the servo, and analogWrite on it would reprogram that
// timer. Returns false (and does nothing) for any other pin.
bool pwmWrite(uint8_t pin, uint8_t duty);

// ---- trigger servo (hardware PWM, immune to the NeoPixel interrupt mask) -----------------------
void servoEnable(bool on);          // false = parked LOW (detached, no holding torque)
bool servoEnabled();
void servoWriteDeg(float deg);      // 0..SERVO_SPAN_DEG, clamped; kept while disabled and
                                    // applied on the next enable
float servoDeg();                   // last commanded angle

// ---- steppers ----------------------------------------------------------------------------------
// Coordinates are motor steps: + = DIR pin high. Motion owns degrees and *_DIR_INVERT.
void stepperEnable(bool on);        // shared EN line (PIN_STEPPER_EN, LOW = enabled)
bool steppersEnabled();
// Signed steps per second. |rate| < STEP_MIN_RATE_HZ stops the axis; |rate| is capped at
// STEP_MAX_RATE_HZ. A running axis changes speed at its next step (the timer's buffered
// period), so a change never glitches a pulse. A direction change takes effect after the
// next step; from a stop, the first step comes one full period after the call.
void stepSetRate(Axis axis, float stepsPerSec);
float stepRate(Axis axis);          // the rate in force after clamping (0 = stopped)
int32_t stepPosition(Axis axis);    // steps counted by the interrupt (what was commanded)
void stepSetPosition(Axis axis, int32_t steps);
// Hard bounds in the interrupt: a step that would leave [minSteps, maxSteps] is not emitted
// and is counted in StepStats.refusals. Motion brakes before a bound; this is the backstop.
void stepSetBounds(Axis axis, bool enabled, int32_t minSteps, int32_t maxSteps);
// Microseconds until the next STEP edge of any running axis (UINT32_MAX when both are
// stopped). Leds calls ledsShow() only when this is at least LED_SHOW_GUARD_US.
uint32_t stepQuietUs();
// Copies the statistics since the previous call into `out` and starts a new window.
void stepTakeStats(StepStats& out);

// ---- LEDs (WS2812 on PIN_LED_DATA) ---------------------------------------------------------------
// Bytes as they go to the LEDs: Leds has already applied the brightness clamp. As a last
// line of defence a channel above LED_MAX_BRIGHTNESS is cut to it here and counted.
void ledsSetPixel(uint8_t index, uint8_t r, uint8_t g, uint8_t b);
// Sends all LED_COUNT pixels. Masks every interrupt for ~241 us: call it only when
// stepQuietUs() >= LED_SHOW_GUARD_US (or when Leds has deferred LED_SHOW_MAX_DEFER_MS).
void ledsShow();
uint32_t ledsOverLimit();           // channels cut by the defence above (should stay 0)

// ---- LCD (16x2 behind a PCF8574 I2C backpack) -------------------------------------------------------
// What one lcdWriteAt call did.
enum class LcdWrite : uint8_t {
  WROTE,    // `c` is on the display and the cursor advanced one column
  AGAIN,    // nothing written yet: the call only moved the cursor, or ran (or waited for) one
            // step of the controller's re-initialisation; call again to write `c`
  FAILED    // the I2C transaction failed (LCD unplugged, NAK, bus timeout): nothing on the
            // display can be trusted. The following calls re-initialise the controller
            // (LcdInit.h: power-on wait, reset, 4-bit mode, clear), so it ends blank. Also
            // returned, without any transaction, when there is no LCD or (col, row) is off-screen
};
bool lcdPresent();                  // an LCD answered at begin(); stays true while unplugged
uint8_t lcdAddress();               // 0 when no LCD answered
// One I2C transaction (none during a re-initialisation: the bit rides on its transactions).
// Returns false when the transaction failed, with the same consequence as LcdWrite::FAILED.
bool lcdBacklight(bool on);
// At most one I2C transaction per call. If the display's cursor is not at (col, row) this
// call only moves it and returns AGAIN: call again (next Lcd::update) to write `c`.
// Otherwise it writes `c`, the cursor advances one column, and it returns WROTE. The HAL checks
// every transaction's result (Wire.endTransmission), so a failure is never reported as WROTE.
LcdWrite lcdWriteAt(uint8_t col, uint8_t row, char c);
uint32_t lcdErrors();               // failed LCD transactions since begin()
uint32_t lcdReinits();              // re-initialisations completed since begin()

// ---- USB serial (Serial, DEBUG_SERIAL_BAUD, through the ESP32 bridge) ------------------------------
size_t usbRead(uint8_t* buf, size_t cap);        // received bytes, up to cap, never waits
bool usbWrite(const uint8_t* data, size_t len);  // queue all of it, or nothing (+1 dropped)
size_t usbTxFree();                              // bytes the queue can take now
uint32_t usbTxDropped();                         // messages refused since boot

// ---- AT link to the ESP32 (LINK_SERIAL, LINK_BAUD) -----------------------------------------------------
size_t linkRead(uint8_t* buf, size_t cap);
bool linkWrite(const uint8_t* data, size_t len); // all or nothing, like usbWrite
bool linkTxIdle();                               // queue empty and the UART has sent it all
size_t linkTxFree();
uint32_t linkTxDropped();

// ---- settings store (the RA4M1's data flash through the core's EEPROM library) ------------------------
// n bytes at SETTINGS_STORE_ADDR, n at most SETTINGS_STORE_MAX_BYTES (more: false, nothing done).
// Survives power off and resets. Erased (blank) flash reads 0xFF.
// storeLoad: copies the bytes into dst; a few microseconds a byte (setup() or a rare command).
bool storeLoad(void* dst, size_t n);
// storeSave: writes the bytes and reads them back; true only when the read-back matches.
// BLOCKS: the core erases the store's 1 KB block (when a stored byte changes) and programs all
// 1 KB of it, about 50 ms typical at the UNO R4's 24 MHz flash clock (RA4M1 datasheet table 2.73:
// 1 KB erase 6.4-9.0 ms, 1 byte 42-52 us x 1024; no erase on a blank block); the datasheet maximum
// is 0.6-0.75 s, more than LINK_TIMEOUT_MS. Interrupts stay enabled (code flash stays readable while the
// data flash programs), so the step timers, the servo PWM and the UART receive rings keep running;
// loop() does not (no planner, no hal::service(), no link polls). Callers: SERVO ADJUST only.
bool storeSave(const void* src, size_t n);

}  // namespace hal

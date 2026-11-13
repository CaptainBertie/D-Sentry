// hal_host.h -- controls and probes for the host implementation of hal.h (sim/hal_host.cpp).
//
// The fake is deterministic: nothing in it reads a real clock or a random source. Time moves
// only when a test (or dsentry_sim) calls setTimeUs / advanceUs, and moving time runs the
// stepper model exactly the way the GPT timers would: each axis steps at its period (whole
// counts of 1/GPT_CLOCK_HZ, as the hardware), a speed change while running lands at the next
// step (the buffered period), a direction change after the next step, bounds refuse steps.
//
// Two positions per axis:
//   hal::stepPosition()     what the step interrupt counted (what the firmware believes)
//   hal_host::trueSteps()   where the mechanism is. They differ when steps are lost: with the
//                           drivers disabled, or against the tilt bottom stop (the CAL_RAM
//                           calibration rams it on purpose).
//
// Serial: bytes a test injects are what the RA4M1 receives (RX rings of
// SERIAL_RX_BUFFER_BYTES, excess dropped and counted, like the core). Bytes the firmware
// writes go into the same TxQueue the board uses and reach the "wire" only through
// hal::service(), at the paced rate; tests read the wire with usbTakeTx() / linkTakeTx().
// Every write service() makes is also checked against an independent model of the RA4M1
// transmitter (Serial: no FIFO; Serial2: 16-byte FIFO loaded only when empty, FCR.TTRG = 0;
// see TxQueue.h): usbTxProbe() / linkTxProbe() count the writes UART::write() would have
// blocked on. Every test that sends serial output should expect blockingWrites == 0.
//
// SPDX-License-Identifier: MIT
#pragma once
#include <stddef.h>
#include <stdint.h>
#include <string>

#include "hal.h"

namespace hal_host {

// Power-on state: time 0, pins low, inputs at their idle level (toggles open: HIGH with the
// pull-ups when TOGGLE_ACTIVE_LOW), analog 0, steppers stopped at 0, LCD present at
// LCD_I2C_ADDR, empty pipes. Call before hal::begin() in every test.
void reset();

// ---- time ----
void setTimeUs(uint64_t us);   // moves forward (running the model); never backwards
void advanceUs(uint64_t us);
void advanceMs(uint64_t ms);
uint64_t timeUs();

// ---- steppers ----
int64_t trueSteps(Axis axis);
void setTrueSteps(Axis axis, int64_t steps);
uint32_t lostSteps(Axis axis);           // pulses that did not move the mechanism
uint32_t totalSteps(Axis axis);          // pulses emitted since reset
uint32_t totalRefusals(Axis axis);       // pulses refused at a bound since reset
bool running(Axis axis);
uint32_t periodCounts(Axis axis);        // timer period in force, GPT counts
int8_t dirPin(Axis axis);                // +1 = DIR high, -1 = low
// The tilt mechanism's bottom stop, in true motor steps: the barrel cannot go further down
// than this. "Down" is the direction that lowers the barrel: -1 in motor steps, or +1 when
// TILT_DIR_INVERT. Pulses into the stop are counted by the firmware but lost.
void setTiltBottomStop(bool enabled, int64_t trueStopSteps);

// ---- captured outputs ----
bool pinLevel(uint8_t pin);              // last hal::writePin level
uint32_t pinWrites(uint8_t pin);
uint8_t pwmDuty(uint8_t pin);            // last accepted hal::pwmWrite
uint32_t pwmRefused();                   // hal::pwmWrite calls refused (pin not PIN_FLY_ENA)
bool servoOn();
float servoAngle();
uint32_t servoPulseUs();                 // what the PWM would output now (0 when parked)
Rgb ledSet(uint8_t index);               // set but not yet shown
Rgb ledShown(uint8_t index);             // latched by the last hal::ledsShow()
uint32_t ledShowCount();
// Shows made while a step edge was due within LED_SHOW_GUARD_US (each one would have delayed
// a step on the board; Leds may only do this after LED_SHOW_MAX_DEFER_MS).
uint32_t ledUnsafeShows();
uint8_t ledMaxChannel();                 // largest channel byte ever passed to ledsSetPixel
uint32_t ledOverLimit();                 // channels the HAL's last-line guard cut
uint64_t lastShowUs();
std::string lcdRow(uint8_t row);         // what the display shows, LCD_COLS chars (blank while
                                         // unplugged or not initialised)
bool lcdBacklightOn();
uint32_t lcdTransactions();              // I2C transactions (each lcdWriteAt / backlight = 1)
uint32_t lcdCharsWritten();
void setLcdPresent(bool present, uint8_t address);   // before hal::begin()
// The LCD after begin(): unplugged (false) every transaction is refused (NAK) and the
// controller loses power, so when it is plugged back in it shows nothing (lcdRow() is blank)
// and ignores writes until the HAL has run the whole re-initialisation (LcdInit.h).
void setLcdAttached(bool attached);
void failLcdTransactions(uint32_t n);   // the next n transactions NAK (a bus glitch); the
                                        // controller keeps its state
void lcdGlitch(uint8_t row, uint8_t col, char c);   // a cell silently shows `c` (ACKed noise)
uint32_t lcdFailedTransactions();       // transactions the model refused
bool lcdControllerReady();              // attached and initialised

// ---- injected inputs ----
void setDigitalInput(uint8_t pin, bool high);
void setAnalog(uint8_t pin, uint16_t counts);
void setBeginTimerFailure(bool fail);    // hal::begin() reports the step timers as failed

// ---- serial pipes ----
// What the transmitter model saw since hal::begin().
struct UartTxProbe {
  uint32_t writes = 0;          // UART::write() calls service() made
  uint32_t blockingWrites = 0;  // of which the real transmitter would have made wait
  size_t largestWrite = 0;      // most bytes in one call
  uint64_t bytes = 0;           // bytes written in total
};
UartTxProbe usbTxProbe();
UartTxProbe linkTxProbe();
size_t usbInject(const uint8_t* data, size_t len);     // returns bytes accepted (RX ring)
size_t usbInject(const std::string& data);
std::string usbTakeTx();                               // wire bytes so far, then cleared
uint32_t usbRxDropped();
size_t linkInject(const uint8_t* data, size_t len);
size_t linkInject(const std::string& data);
std::string linkTakeTx();
uint32_t linkRxDropped();

}  // namespace hal_host

// hal_host.cpp -- hal.h on the Mac, for the unit tests and dsentry_sim. See hal_host.h.
// It mirrors hal_arduino.cpp rule for rule (rate clamping, buffered periods, bounds, the
// lcdWriteAt cursor contract, the paced TxQueues) so the logic tested here is the logic that
// runs on the board.
//
// SPDX-License-Identifier: MIT
#include "hal_host.h"

#include <deque>
#include <string.h>

#include "LcdInit.h"
#include "TxQueue.h"

namespace {

constexpr uint64_t kCountsPerUs = GPT_CLOCK_HZ / US_PER_S;
constexpr size_t kPins = 64;   // more than the UNO R4's D0..D21 + A0..A5
constexpr uint64_t kNsPerUs = US_PER_MS;             // 1000 ns per us, as 1000 us per ms
constexpr uint64_t kNsPerS = US_PER_S * kNsPerUs;

struct FakeAxis {
  bool running = false;
  int32_t pos = 0;
  int8_t dir = 1;
  int8_t pendingDir = 1;
  bool boundsOn = false;
  int32_t boundMin = 0;
  int32_t boundMax = 0;
  float rate = 0.0f;
  uint32_t periodInForce = 0;    // GPT counts of the cycle now running
  uint32_t periodBuffered = 0;   // GTPBR: becomes periodInForce at the next edge
  uint64_t nextEdge = 0;         // absolute time of the next edge, GPT counts since reset
  uint32_t winSteps = 0;
  uint32_t winRefusals = 0;
  uint32_t winSamples = 0;
  uint32_t totalSteps = 0;
  uint32_t totalRefusals = 0;
  int64_t truePos = 0;
  uint32_t lost = 0;
};

// The RA4M1 transmitter as arduino:renesas_uno 1.6.0 drives it, modelled independently of
// TxQueue so the tests check the pacer against the hardware rule, not against itself (the
// evidence is in TxQueue.h). Time in nanoseconds, the byte time exact (not rounded up).
//   fifo == false (SCI9, Serial): R_SCI_UART_Write puts the first byte in TDR; UART::write
//     returns when TDR has moved to the shift register. It blocks when the shift register is
//     still busy, or when one call carries more than one byte.
//   fifo == true (SCI1, Serial2, FCR.TTRG = 0): the write returns when every byte is in the
//     FIFO, and the TXI that loads the FIFO fires only while it is empty. It blocks when the
//     FIFO still holds a byte, or when one call carries more than the depth.
struct UartModel {
  bool fifo = false;
  size_t depth = 0;
  uint64_t byteNs = 0;
  uint64_t shiftFreeNs = 0;   // when the shift register finishes its current byte
  size_t fifoCount = 0;       // bytes waiting in the FIFO behind the shift register
  hal_host::UartTxProbe probe;

  void begin(bool hasFifo, size_t fifoDepth, uint32_t baud) {
    fifo = hasFifo;
    depth = fifoDepth;
    byteNs = (uint64_t)UART_BITS_PER_BYTE * kNsPerS / baud;
    shiftFreeNs = 0;
    fifoCount = 0;
    probe = hal_host::UartTxProbe();
  }
  // Moves FIFO bytes into the shift register as it frees, up to `now`.
  void advance(uint64_t now) {
    while (fifoCount > 0 && shiftFreeNs <= now) {
      shiftFreeNs += byteNs;
      fifoCount--;
    }
  }
  void write(uint64_t now, size_t n) {
    advance(now);
    probe.writes++;
    probe.bytes += n;
    if (n > probe.largestWrite) probe.largestWrite = n;
    const bool busy = fifo ? (fifoCount > 0 || n > depth) : (shiftFreeNs > now || n > 1);
    if (busy) probe.blockingWrites++;
    if (shiftFreeNs <= now) {   // idle: the first byte starts shifting now
      shiftFreeNs = now + byteNs;
      n--;
    }
    if (fifo) {
      fifoCount += n;
    } else {
      shiftFreeNs += n * byteNs;
    }
  }
};

struct State {
  uint64_t us = 0;
  FakeAxis ax[AXIS_COUNT];
  bool driversOn = false;
  bool timersOk = false;
  bool failTimers = false;
  bool tiltStopOn = false;
  int64_t tiltStop = 0;
  uint64_t statsStartUs = 0;
  bool pin[kPins] = {};
  uint32_t pinWrites[kPins] = {};
  uint8_t pwm[kPins] = {};
  uint32_t pwmRefused = 0;
  bool digitalIn[kPins] = {};
  uint16_t analog[kPins] = {};
  bool servoOk = false;
  bool servoOn = false;
  float servoDeg = TRIGGER_REST_ANGLE;
  Rgb ledSet[LED_COUNT];
  Rgb ledShown[LED_COUNT];
  uint32_t showCount = 0;
  uint32_t unsafeShows = 0;
  uint8_t maxChannel = 0;
  uint32_t ledsOver = 0;
  uint64_t lastShowUs = 0;
  bool lcdPresentCfg = true;
  uint8_t lcdAddrCfg = LCD_I2C_ADDR;
  bool lcdOk = false;
  uint8_t lcdAddr = 0;
  char lcd[LCD_ROWS][LCD_COLS] = {};
  uint8_t curRow = 0;
  uint8_t curCol = 0;
  bool backlight = false;
  uint32_t lcdTx = 0;
  uint32_t lcdChars = 0;
  // The LCD's own state (the model) ...
  bool lcdAttached = true;       // plugged in and powered
  bool lcdCtrlReady = true;      // controller initialised (4-bit, display on); false after a
                                 // power loss until the HAL's re-initialisation has run
  uint32_t lcdFailNext = 0;      // transient NAKs still to come
  uint32_t lcdResetNibbles = 0;  // reset nibbles seen since the last probe
  bool lcdFourBit = false;       // the 4-bit nibble followed enough reset nibbles
  uint32_t lcdFailed = 0;        // transactions that were not acknowledged
  // ... and the HAL's (as hal_arduino.cpp)
  uint32_t lcdErrors = 0;
  uint32_t lcdReinits = 0;
  lcdinit::Sequencer lcdInit;
  hal::BeginReport report;
  TxQueue<DEBUG_TX_BUFFER_BYTES> usbTx;
  TxQueue<LINK_TX_BUFFER_BYTES> linkTx;
  std::deque<uint8_t> usbRx;
  std::deque<uint8_t> linkRx;
  uint32_t usbRxDropped = 0;
  uint32_t linkRxDropped = 0;
  std::string usbWire;
  std::string linkWire;
  UartModel usbUart;
  UartModel linkUart;
};

State g;

uint64_t nowCounts() { return g.us * kCountsPerUs; }

// One timer overflow on `axis`: exactly what stepIsr() in hal_arduino.cpp does.
void stepEdge(Axis axis) {
  FakeAxis& a = g.ax[axis];
  a.winSamples++;
  const int32_t next = a.pos + a.dir;
  if (a.boundsOn && (next < a.boundMin || next > a.boundMax)) {
    a.winRefusals++;
    a.totalRefusals++;
  } else {
    a.pos = next;
    a.winSteps++;
    a.totalSteps++;
    // The mechanism moves only with the drivers on and not into the tilt bottom stop.
    const int8_t down = TILT_DIR_INVERT ? 1 : -1;
    bool blocked = !g.driversOn;
    if (axis == AXIS_TILT && g.tiltStopOn && a.dir == down) {
      const int64_t to = a.truePos + a.dir;
      blocked = blocked || (down < 0 ? to < g.tiltStop : to > g.tiltStop);
    }
    if (blocked)
      a.lost++;
    else
      a.truePos += a.dir;
  }
  a.dir = a.pendingDir;   // a pending direction change lands after the step
}

void runAxesTo(uint64_t us) {
  const uint64_t target = us * kCountsPerUs;
  for (uint8_t i = 0; i < AXIS_COUNT; i++) {
    FakeAxis& a = g.ax[i];
    while (a.running && a.nextEdge <= target) {
      stepEdge((Axis)i);
      a.periodInForce = a.periodBuffered;   // GTPBR -> GTPR at the overflow
      a.nextEdge += a.periodInForce;
    }
  }
}

size_t inject(std::deque<uint8_t>& q, uint32_t& dropped, const uint8_t* data, size_t len) {
  size_t n = 0;
  for (size_t i = 0; i < len; i++) {
    if (q.size() >= SERIAL_RX_BUFFER_BYTES) {
      dropped++;
      continue;
    }
    q.push_back(data[i]);
    n++;
  }
  return n;
}

size_t take(std::deque<uint8_t>& q, uint8_t* buf, size_t cap) {
  size_t n = 0;
  while (n < cap && !q.empty()) {
    buf[n++] = q.front();
    q.pop_front();
  }
  return n;
}

// The same single write per service() as hal_arduino.cpp; the model sees it at the true time.
template <size_t N>
void drainTo(TxQueue<N>& q, UartModel& uart, std::string& wire, uint32_t nowUs) {
  uint8_t chunk[SCI_FIFO_DEPTH];
  const size_t n = q.take(nowUs, chunk, sizeof chunk);
  if (n == 0) return;
  uart.write(g.us * kNsPerUs, n);
  wire.append(reinterpret_cast<const char*>(chunk), n);
}

uint32_t servoPulseFor(float deg) {
  return (uint32_t)(TRIGGER_SERVO_MIN_US +
                    deg / SERVO_SPAN_DEG * (TRIGGER_SERVO_MAX_US - TRIGGER_SERVO_MIN_US) + MATH_HALF);
}

uint8_t ledGuard(uint8_t v) {
  if (v > g.maxChannel) g.maxChannel = v;
  if (v <= LED_MAX_BRIGHTNESS) return v;
  g.ledsOver++;
  return LED_MAX_BRIGHTNESS;
}

bool validPin(uint8_t pin) { return pin < kPins; }

// One LCD transaction on the model's bus: false (and counted on both sides) when the backpack
// does not acknowledge it (unplugged, or a transient NAK injected by failLcdTransactions).
bool lcdAck() {
  g.lcdTx++;
  if (g.lcdAttached && g.lcdFailNext == 0) return true;
  if (g.lcdFailNext) g.lcdFailNext--;
  g.lcdFailed++;
  g.lcdErrors++;
  return false;
}

// One re-initialisation step (LcdInit.h) as the model's controller sees it: it becomes ready
// only after the probe, the reset nibbles, the 4-bit nibble and the clear, in that order.
bool lcdInitStep(lcdinit::Kind kind, uint8_t value) {
  if (!lcdAck()) return false;
  if (kind == lcdinit::PROBE) {
    g.lcdResetNibbles = 0;
    g.lcdFourBit = false;
  } else if (kind == lcdinit::NIBBLE) {
    if (value == HD44780_RESET_NIBBLE) {
      g.lcdResetNibbles++;
      g.lcdFourBit = false;
    } else if (value == HD44780_4BIT_NIBBLE) {
      g.lcdFourBit = g.lcdResetNibbles >= HD44780_RESET_REPEATS;
    }
  } else if (value == HD44780_CMD_CLEAR && g.lcdFourBit) {
    for (uint8_t r = 0; r < LCD_ROWS; r++) memset(g.lcd[r], ' ', LCD_COLS);
    g.lcdCtrlReady = true;
  }
  return true;
}

}  // namespace

// =================================================================================================
namespace hal_host {

void reset() {
  g = State();
  for (bool& in : g.digitalIn) in = false;
  // Open toggles read HIGH through the pull-ups (TOGGLE_ACTIVE_LOW), LOW otherwise.
  g.digitalIn[PIN_TOGGLE_POWER] = TOGGLE_ACTIVE_LOW;
  g.digitalIn[PIN_TOGGLE_MODE] = TOGGLE_ACTIVE_LOW;
  for (uint8_t r = 0; r < LCD_ROWS; r++) memset(g.lcd[r], ' ', LCD_COLS);
}

void setTimeUs(uint64_t us) {
  if (us <= g.us) return;
  runAxesTo(us);
  g.us = us;
}
void advanceUs(uint64_t us) { setTimeUs(g.us + us); }
void advanceMs(uint64_t ms) { setTimeUs(g.us + ms * US_PER_MS); }
uint64_t timeUs() { return g.us; }

int64_t trueSteps(Axis axis) { return g.ax[axis].truePos; }
void setTrueSteps(Axis axis, int64_t steps) { g.ax[axis].truePos = steps; }
uint32_t lostSteps(Axis axis) { return g.ax[axis].lost; }
uint32_t totalSteps(Axis axis) { return g.ax[axis].totalSteps; }
uint32_t totalRefusals(Axis axis) { return g.ax[axis].totalRefusals; }
bool running(Axis axis) { return g.ax[axis].running; }
uint32_t periodCounts(Axis axis) { return g.ax[axis].running ? g.ax[axis].periodInForce : 0; }
int8_t dirPin(Axis axis) { return g.ax[axis].dir; }
void setTiltBottomStop(bool enabled, int64_t trueStopSteps) {
  g.tiltStopOn = enabled;
  g.tiltStop = trueStopSteps;
}

bool pinLevel(uint8_t pin) { return validPin(pin) && g.pin[pin]; }
uint32_t pinWrites(uint8_t pin) { return validPin(pin) ? g.pinWrites[pin] : 0; }
uint8_t pwmDuty(uint8_t pin) { return validPin(pin) ? g.pwm[pin] : 0; }
uint32_t pwmRefused() { return g.pwmRefused; }
bool servoOn() { return g.servoOn; }
float servoAngle() { return g.servoDeg; }
uint32_t servoPulseUs() { return g.servoOn ? servoPulseFor(g.servoDeg) : 0; }
Rgb ledSet(uint8_t index) { return index < LED_COUNT ? g.ledSet[index] : Rgb(); }
Rgb ledShown(uint8_t index) { return index < LED_COUNT ? g.ledShown[index] : Rgb(); }
uint32_t ledShowCount() { return g.showCount; }
uint32_t ledUnsafeShows() { return g.unsafeShows; }
uint8_t ledMaxChannel() { return g.maxChannel; }
uint32_t ledOverLimit() { return g.ledsOver; }
uint64_t lastShowUs() { return g.lastShowUs; }
std::string lcdRow(uint8_t row) {
  if (row >= LCD_ROWS) return std::string();
  if (!g.lcdAttached || !g.lcdCtrlReady) return std::string(LCD_COLS, ' ');   // dark
  return std::string(g.lcd[row], LCD_COLS);
}
bool lcdBacklightOn() { return g.lcdOk && g.lcdAttached && g.backlight; }
uint32_t lcdTransactions() { return g.lcdTx; }
uint32_t lcdCharsWritten() { return g.lcdChars; }
void setLcdPresent(bool present, uint8_t address) {
  g.lcdPresentCfg = present;
  g.lcdAddrCfg = address;
}
void setLcdAttached(bool attached) {
  if (!attached && g.lcdAttached) {   // power lost: the controller forgets everything
    g.lcdCtrlReady = false;
    g.lcdFourBit = false;
    g.lcdResetNibbles = 0;
    for (uint8_t r = 0; r < LCD_ROWS; r++) memset(g.lcd[r], '?', LCD_COLS);
  }
  g.lcdAttached = attached;
}
void failLcdTransactions(uint32_t n) { g.lcdFailNext = n; }
void lcdGlitch(uint8_t row, uint8_t col, char c) {
  if (row < LCD_ROWS && col < LCD_COLS) g.lcd[row][col] = c;
}
uint32_t lcdFailedTransactions() { return g.lcdFailed; }
bool lcdControllerReady() { return g.lcdAttached && g.lcdCtrlReady; }

void setDigitalInput(uint8_t pin, bool high) {
  if (validPin(pin)) g.digitalIn[pin] = high;
}
void setAnalog(uint8_t pin, uint16_t counts) {
  if (validPin(pin)) g.analog[pin] = counts > ADC_MAX_COUNTS ? (uint16_t)ADC_MAX_COUNTS : counts;
}
void setBeginTimerFailure(bool fail) { g.failTimers = fail; }

size_t usbInject(const uint8_t* data, size_t len) { return inject(g.usbRx, g.usbRxDropped, data, len); }
size_t usbInject(const std::string& data) {
  return usbInject(reinterpret_cast<const uint8_t*>(data.data()), data.size());
}
std::string usbTakeTx() {
  std::string out;
  out.swap(g.usbWire);
  return out;
}
uint32_t usbRxDropped() { return g.usbRxDropped; }
size_t linkInject(const uint8_t* data, size_t len) { return inject(g.linkRx, g.linkRxDropped, data, len); }
size_t linkInject(const std::string& data) {
  return linkInject(reinterpret_cast<const uint8_t*>(data.data()), data.size());
}
std::string linkTakeTx() {
  std::string out;
  out.swap(g.linkWire);
  return out;
}
uint32_t linkRxDropped() { return g.linkRxDropped; }
UartTxProbe usbTxProbe() { return g.usbUart.probe; }
UartTxProbe linkTxProbe() { return g.linkUart.probe; }

}  // namespace hal_host

// =================================================================================================
namespace hal {

bool begin() {
  g.driversOn = false;
  g.pin[PIN_STEPPER_EN] = true;   // EN high = drivers disabled
  g.timersOk = !g.failTimers;
  g.report = BeginReport();
  g.report.panTimer = g.timersOk;
  g.report.tiltTimer = g.timersOk;
  g.servoOk = true;
  g.servoOn = false;
  g.servoDeg = TRIGGER_REST_ANGLE;
  g.report.servo = true;
  g.pin[PIN_FLY_IN1] = false;
  g.pin[PIN_FLY_IN2] = false;
  g.pwm[PIN_FLY_ENA] = 0;
  g.usbTx.begin(DEBUG_SERIAL_BAUD, SERIAL_TX_SLACK_BYTES, (uint32_t)g.us);
  g.linkTx.begin(LINK_BAUD, SCI_FIFO_DEPTH, (uint32_t)g.us);
  g.usbUart.begin(false, SERIAL_TX_SLACK_BYTES, DEBUG_SERIAL_BAUD);
  g.linkUart.begin(true, SCI_FIFO_DEPTH, LINK_BAUD);
  for (uint8_t i = 0; i < LED_COUNT; i++) g.ledSet[i] = g.ledShown[i] = Rgb();
  g.showCount++;
  g.lastShowUs = g.us;
  g.lcdOk = g.lcdPresentCfg;
  g.lcdAddr = g.lcdOk ? g.lcdAddrCfg : 0;
  for (uint8_t r = 0; r < LCD_ROWS; r++) memset(g.lcd[r], ' ', LCD_COLS);
  g.curRow = g.curCol = 0;
  g.backlight = g.lcdOk;
  g.report.lcd = g.lcdOk;
  g.report.lcdAddress = g.lcdAddr;
  g.statsStartUs = g.us;
  return g.timersOk;
}

const BeginReport& beginReport() { return g.report; }

// Like hal_arduino.cpp: paced by the HAL's own clock, never by the caller's timestamp.
void service(uint32_t /*nowUs*/) {
  const uint32_t now = (uint32_t)g.us;
  drainTo(g.usbTx, g.usbUart, g.usbWire, now);
  drainTo(g.linkTx, g.linkUart, g.linkWire, now);
}

uint32_t nowMs() { return (uint32_t)(g.us / US_PER_MS); }
uint32_t nowUs() { return (uint32_t)g.us; }

void writePin(uint8_t pin, bool high) {
  if (!validPin(pin)) return;
  g.pin[pin] = high;
  g.pinWrites[pin]++;
}
bool readPin(uint8_t pin) { return validPin(pin) && g.digitalIn[pin]; }
uint16_t readAnalog(uint8_t pin) { return validPin(pin) ? g.analog[pin] : 0; }

bool pwmWrite(uint8_t pin, uint8_t duty) {
  if (pin != PIN_FLY_ENA) {
    g.pwmRefused++;
    return false;
  }
  g.pwm[pin] = duty;
  return true;
}

void servoEnable(bool on) {
  if (g.servoOk) g.servoOn = on;
}
bool servoEnabled() { return g.servoOn; }
void servoWriteDeg(float deg) {
  if (!(deg >= 0.0f)) deg = 0.0f;
  if (deg > SERVO_SPAN_DEG) deg = SERVO_SPAN_DEG;
  g.servoDeg = deg;
}
float servoDeg() { return g.servoDeg; }

void stepperEnable(bool on) {
  g.driversOn = on;
  g.pin[PIN_STEPPER_EN] = !on;
  g.pinWrites[PIN_STEPPER_EN]++;
}
bool steppersEnabled() { return g.driversOn; }

void stepSetRate(Axis axis, float stepsPerSec) {
  if (axis >= AXIS_COUNT || !g.timersOk) return;
  FakeAxis& a = g.ax[axis];
  const int8_t want = stepsPerSec < 0.0f ? -1 : 1;
  float mag = stepsPerSec < 0.0f ? -stepsPerSec : stepsPerSec;
  if (!(mag >= STEP_MIN_RATE_HZ)) {
    a.running = false;
    a.rate = 0.0f;
    return;
  }
  if (mag > STEP_MAX_RATE_HZ) mag = STEP_MAX_RATE_HZ;
  const uint32_t counts = (uint32_t)((float)GPT_CLOCK_HZ / mag);
  if (!a.running) {
    a.pendingDir = a.dir = want;
    a.periodInForce = a.periodBuffered = counts;
    a.nextEdge = nowCounts() + counts;
    a.running = true;
  } else {
    a.pendingDir = want;
    a.periodBuffered = counts;
  }
  a.rate = want > 0 ? mag : -mag;
}

float stepRate(Axis axis) { return axis < AXIS_COUNT ? g.ax[axis].rate : 0.0f; }
int32_t stepPosition(Axis axis) { return axis < AXIS_COUNT ? g.ax[axis].pos : 0; }
void stepSetPosition(Axis axis, int32_t steps) {
  if (axis < AXIS_COUNT) g.ax[axis].pos = steps;
}
void stepSetBounds(Axis axis, bool enabled, int32_t minSteps, int32_t maxSteps) {
  if (axis >= AXIS_COUNT) return;
  g.ax[axis].boundsOn = enabled;
  g.ax[axis].boundMin = minSteps;
  g.ax[axis].boundMax = maxSteps;
}

uint32_t stepQuietUs() {
  uint64_t best = UINT32_MAX;
  const uint64_t now = nowCounts();
  for (const FakeAxis& a : g.ax) {
    if (!a.running) continue;
    const uint64_t remain = a.nextEdge > now ? (a.nextEdge - now) / kCountsPerUs : 0;
    if (remain < best) best = remain;
  }
  return (uint32_t)best;
}

void stepTakeStats(StepStats& out) {
  out = StepStats();
  out.windowUs = (uint32_t)(g.us - g.statsStartUs);
  g.statsStartUs = g.us;
  for (uint8_t i = 0; i < AXIS_COUNT; i++) {
    FakeAxis& a = g.ax[i];
    out.steps[i] = a.winSteps;
    out.refusals[i] = a.winRefusals;
    out.samples[i] = a.winSamples;   // the fake's timing is ideal: jitter stays 0
    a.winSteps = a.winRefusals = a.winSamples = 0;
  }
}

void ledsSetPixel(uint8_t index, uint8_t r, uint8_t g8, uint8_t b) {
  if (index >= LED_COUNT) return;
  Rgb c;
  c.r = ledGuard(r);
  c.g = ledGuard(g8);
  c.b = ledGuard(b);
  g.ledSet[index] = c;
}

void ledsShow() {
  if (stepQuietUs() < LED_SHOW_GUARD_US) g.unsafeShows++;
  for (uint8_t i = 0; i < LED_COUNT; i++) g.ledShown[i] = g.ledSet[i];
  g.showCount++;
  g.lastShowUs = g.us;
}

uint32_t ledsOverLimit() { return g.ledsOver; }

bool lcdPresent() { return g.lcdOk; }
uint8_t lcdAddress() { return g.lcdAddr; }
bool lcdBacklight(bool on) {
  if (!g.lcdOk) return false;
  g.backlight = on;
  if (g.lcdInit.active()) return true;   // the re-initialisation's transactions carry the bit
  if (lcdAck()) return true;
  g.lcdInit.start();
  return false;
}

LcdWrite lcdWriteAt(uint8_t col, uint8_t row, char c) {
  if (!g.lcdOk || row >= LCD_ROWS || col >= LCD_COLS) return LcdWrite::FAILED;
  if (g.lcdInit.active()) {
    const LcdWrite r = g.lcdInit.run((uint32_t)g.us, lcdInitStep);
    if (!g.lcdInit.active()) {   // done: the clear left the cursor at (0,0)
      g.curRow = 0;
      g.curCol = 0;
      g.lcdReinits++;
    }
    return r;
  }
  // either the cursor command or the character: one transaction
  if (!lcdAck()) {
    g.lcdInit.start();
    return LcdWrite::FAILED;
  }
  if (row != g.curRow || col != g.curCol) {
    g.curRow = row;
    g.curCol = col;
    return LcdWrite::AGAIN;
  }
  // A controller that lost power ignores writes until it is re-initialised.
  if (g.lcdCtrlReady && g.curCol < LCD_COLS) g.lcd[g.curRow][g.curCol] = c;
  g.curCol++;
  g.lcdChars++;
  return LcdWrite::WROTE;
}

uint32_t lcdErrors() { return g.lcdErrors; }
uint32_t lcdReinits() { return g.lcdReinits; }

size_t usbRead(uint8_t* buf, size_t cap) { return take(g.usbRx, buf, cap); }
bool usbWrite(const uint8_t* data, size_t len) { return g.usbTx.push(data, len); }
size_t usbTxFree() { return g.usbTx.freeBytes(); }
uint32_t usbTxDropped() { return g.usbTx.dropped(); }

size_t linkRead(uint8_t* buf, size_t cap) { return take(g.linkRx, buf, cap); }
bool linkWrite(const uint8_t* data, size_t len) { return g.linkTx.push(data, len); }
bool linkTxIdle() { return g.linkTx.idle((uint32_t)g.us); }
size_t linkTxFree() { return g.linkTx.freeBytes(); }
uint32_t linkTxDropped() { return g.linkTx.dropped(); }

}  // namespace hal

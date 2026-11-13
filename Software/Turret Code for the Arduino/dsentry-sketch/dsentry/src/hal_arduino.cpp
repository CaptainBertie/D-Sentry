// hal_arduino.cpp -- hal.h on the UNO R4 WiFi (arduino:renesas_uno 1.6.0). The only file that
// includes Arduino library headers (architecture.md rule 4); compiled only when ARDUINO is
// defined, so the host build skips it.
//
// Hardware facts this relies on (file:line evidence in the research reports
// research-ra4m1-timers.md and research-ra4m1-link.md):
//   - Steppers: GPT0 = pan, GPT1 = tilt, the two 32-bit channels, clocked by PCLKD at 48 MHz
//     with prescaler /1. initVariant() marks them PWM-reserved (for D5 / D3), so they are taken
//     with FspTimer::force_use_of_pwm_reserved_timer() and explicit channel numbers, in setup(),
//     before anything else can allocate a timer. Speed changes use the buffered set_period()
//     (R_GPT_PeriodSet writes GTPBR, which the hardware copies to GTPR at the next overflow:
//     glitch-free). set_frequency() / set_period_us() close and reopen the timer and are never
//     used at run time. Stop = stop(); restart = set_period() while stopped (writes GTPR and
//     clears the counter) then start(). The interrupt reads GTPR itself (one load) rather
//     than FspTimer::get_period_raw(), which goes through R_GPT_InfoGet.
//   - One interrupt per step: STEP high, busy-wait STEP_PULSE_US on the DWT cycle counter, STEP
//     low, count, then apply a pending DIR change, so DIR settles a whole period before the next
//     rising edge (A4988/DRV8825 need 650 ns). Pins are written with one 32-bit store to the
//     port's PCNTR3 (bits 15:0 set, 31:16 clear): atomic, no read-modify-write race with the
//     NeoPixel driver's POSR/PORR writes on the same port.
//   - Servo: hardware PWM (PwmOut) on D12 = GPT6 B, 50 Hz; suspend() stops the counter and the
//     pin rests LOW (= detached). GPT6 also serves D11 (L298N IN2): never analogWrite D11.
//   - Flywheel ENA: analogWrite on D10 = GPT2 A (490 Hz, 8 bit).
//   - NeoPixel show() masks interrupts ~241 us; Leds only calls it in a step-quiet window.
//   - Serial (SCI9) has no TX FIFO; Serial2 (SCI1) has 16 bytes, but the core opens it with
//     FCR.TTRG = 0, so a write returns at once only into an EMPTY FIFO (TxQueue.h has the
//     disassembly evidence). UART::write() spins until its bytes are in the hardware, so all
//     output goes through TxQueue and service(): one write() call per port per service(),
//     never while the previous burst is still in the transmitter.
//   - LiquidCrystal_I2C 1.1.2 is used for init() and backlight() in setup() only. init() calls
//     Wire.begin(), which resets the clock to 100 kHz, so setClock / setWireTimeout come after.
//     Every run-time LCD transaction checks Wire.endTransmission(); a failure (unplugged LCD,
//     NAK, bus timeout) is reported as LcdWrite::FAILED and the following calls re-initialise
//     the controller one non-blocking step at a time (LcdInit.h).
//
// SPDX-License-Identifier: MIT
#ifdef ARDUINO

#include <Arduino.h>
#include <Adafruit_NeoPixel.h>
#include <FspTimer.h>
#include <LiquidCrystal_I2C.h>
#include <Wire.h>
#include <pwm.h>

#include "LcdInit.h"
#include "TxQueue.h"
#include "hal.h"

static_assert(CPU_HZ % GPT_CLOCK_HZ == 0, "cycle-counter ticks per GPT count must be whole");
static_assert(CPU_HZ % US_PER_S == 0, "cycle-counter ticks per microsecond must be whole");
static_assert(STEP_TIMER_PAN_CHANNEL != STEP_TIMER_TILT_CHANNEL, "one GPT channel per axis");
static_assert(STEP_TIMER_PAN_CHANNEL < GTP32_HOWMANY && STEP_TIMER_TILT_CHANNEL < GTP32_HOWMANY,
              "the step timers must be the 32-bit GPT channels (slow rates need > 16 bits)");
static_assert(STEP_MIN_RATE_HZ > 0.0f && STEP_MIN_RATE_HZ < STEP_MAX_RATE_HZ, "step rate range");
// Jitter permille is diff * PERMILLE / period with diff < period: fits 32 bits at the slowest rate.
static_assert((float)CPU_HZ / STEP_MIN_RATE_HZ * PERMILLE < (float)UINT32_MAX,
              "STEP_MIN_RATE_HZ too low for the 32-bit jitter arithmetic");
static_assert(LINK_TX_BUFFER_BYTES >= DS_AT_DST_LINE_LEN, "the link queue must hold one AT+DST line");
static_assert(SERIAL_TX_SLACK_BYTES <= SCI_FIFO_DEPTH, "drainTo's burst buffer holds one SCI_FIFO_DEPTH burst");
static_assert(GPT_CLOCK_HZ % US_PER_S == 0, "GPT counts per microsecond must be whole");
static_assert(LED_MAX_BRIGHTNESS <= LED_BRIGHTNESS_LIMIT, "spec 3.8: LED brightness ceiling");

namespace {

constexpr uint32_t kCyclesPerUs = CPU_HZ / US_PER_S;
constexpr uint32_t kCyclesPerCount = CPU_HZ / GPT_CLOCK_HZ;
constexpr uint32_t kCountsPerUs = GPT_CLOCK_HZ / US_PER_S;
constexpr uint32_t kPulseCycles = STEP_PULSE_US * kCyclesPerUs;

// Masks interrupts for a handful of instructions and restores the previous PRIMASK (nests).
class IrqLock {
 public:
  IrqLock() : primask_(__get_PRIMASK()) { __disable_irq(); }
  ~IrqLock() { __set_PRIMASK(primask_); }
  IrqLock(const IrqLock&) = delete;
  IrqLock& operator=(const IrqLock&) = delete;

 private:
  uint32_t primask_;
};

// PCNTR3 of the port that carries an Arduino pin, and the pin's bit in it.
volatile uint32_t* pcntr3For(uint8_t pin, uint32_t& mask) {
  const uint32_t io = (uint32_t)g_pin_cfg[pin].pin;   // bsp_io_port_pin_t: port << 8 | bit
  mask = 1u << (io & BSP_IO_PRV_8BIT_MASK);
  R_PORT0_Type* port = R_PORT0 + (R_PORT1 - R_PORT0) * (io >> RA_IO_PORT_SHIFT);
  return &port->PCNTR3;
}

// Registers of GPT channel n (the channels are evenly spaced).
R_GPT0_Type* gptFor(uint8_t channel) {
  return reinterpret_cast<R_GPT0_Type*>(R_GPT0_BASE + (R_GPT1_BASE - R_GPT0_BASE) * channel);
}

// ---- steppers ----------------------------------------------------------------------------------
struct StepAxis {
  FspTimer timer;                  // constructor stores defaults only; begin() is in setup()
  R_GPT0_Type* gpt = nullptr;
  volatile uint32_t* stepReg = nullptr;
  uint32_t stepMask = 0;
  volatile uint32_t* dirReg = nullptr;
  uint32_t dirMask = 0;
  bool timerOk = false;
  // loop side
  bool running = false;
  float rate = 0.0f;               // signed steps/s in force
  uint32_t periodCounts = 0;       // last value given to set_period()
  // shared with the interrupt
  volatile int32_t position = 0;
  volatile int8_t dir = 1;         // direction the DIR pin shows now: +1 high, -1 low
  volatile int8_t pendingDir = 1;  // applied by the interrupt after its next step
  volatile bool boundsOn = false;
  volatile int32_t boundMin = 0;
  volatile int32_t boundMax = 0;
  volatile uint32_t steps = 0;
  volatile uint32_t refusals = 0;
  volatile uint32_t lastEdgeCyc = 0;   // DWT at the last interrupt (or at start)
  volatile uint32_t periodCyc = 0;     // cycles of the period now running
  volatile uint32_t jitMaxPm = 0;
  volatile uint64_t jitSumPm = 0;
  volatile uint32_t jitN = 0;
};

StepAxis g_axes[AXIS_COUNT];
bool g_driversOn = false;
uint32_t g_statsStartUs = 0;

void writeDir(StepAxis& a, int8_t dir) {
  *a.dirReg = dir > 0 ? a.dirMask : a.dirMask << RA_PCNTR3_RESET_SHIFT;
}

// The step interrupt (FSP overflow callback, priority STEP_TIMER_IRQ_PRIORITY).
void stepIsr(timer_callback_args_t* args) {
  if (args->event != TIMER_EVENT_CYCLE_END) return;
  StepAxis& a = *static_cast<StepAxis*>(const_cast<void*>(args->p_context));
  const uint32_t now = DWT->CYCCNT;

  // Jitter: the time since the previous interrupt against the period that was in force.
  const uint32_t actual = now - a.lastEdgeCyc;
  const uint32_t expected = a.periodCyc;
  if (expected) {
    const uint32_t diff = actual > expected ? actual - expected : expected - actual;
    const uint32_t pm = diff >= expected ? (uint32_t)PERMILLE : diff * PERMILLE / expected;
    if (pm > a.jitMaxPm) a.jitMaxPm = pm;
    a.jitSumPm = a.jitSumPm + pm;
    a.jitN = a.jitN + 1;
  }
  a.lastEdgeCyc = now;
  // GTPR now holds the period of the cycle that has just begun (copied from GTPBR at overflow).
  a.periodCyc = (a.gpt->GTPR + 1u) * kCyclesPerCount;

  const int32_t next = a.position + a.dir;
  if (a.boundsOn && (next < a.boundMin || next > a.boundMax)) {
    a.refusals = a.refusals + 1;
  } else {
    *a.stepReg = a.stepMask;                                   // STEP high
    // Pulse width on the cycle counter. The spin is also bounded by an iteration count (each
    // pass takes at least one cycle, so the bound never cuts a pulse short): if anything ever
    // stopped DWT->CYCCNT, the interrupt still ends within a few microseconds instead of hanging.
    const uint32_t t0 = DWT->CYCCNT;
    for (uint32_t spin = 0; spin < kPulseCycles && (uint32_t)(DWT->CYCCNT - t0) < kPulseCycles; spin++) {
    }
    *a.stepReg = a.stepMask << RA_PCNTR3_RESET_SHIFT;          // STEP low
    a.position = next;
    a.steps = a.steps + 1;
  }
  if (a.pendingDir != a.dir) {   // a whole period before the next rising edge
    writeDir(a, a.pendingDir);
    a.dir = a.pendingDir;
  }
}

bool beginAxis(Axis axis, uint8_t channel, uint8_t stepPin, uint8_t dirPin) {
  StepAxis& a = g_axes[axis];
  pinMode(stepPin, OUTPUT);
  pinMode(dirPin, OUTPUT);
  a.stepReg = pcntr3For(stepPin, a.stepMask);
  a.dirReg = pcntr3For(dirPin, a.dirMask);
  *a.stepReg = a.stepMask << RA_PCNTR3_RESET_SHIFT;   // STEP low
  writeDir(a, a.dir);
  a.gpt = gptFor(channel);
  // The channel is PWM-reserved by initVariant(): claim it by force, by number. The begin
  // frequency only fixes the prescaler: any rate up to STEP_MAX_RATE_HZ gives /1 on a 32-bit
  // channel. Duty is irrelevant (the GTIOC outputs stay disabled: the pins remain GPIO).
  FspTimer::force_use_of_pwm_reserved_timer();
  if (!a.timer.begin(TIMER_MODE_PERIODIC, GPT_TIMER, channel, STEP_MAX_RATE_HZ, 0.0f, stepIsr, &a))
    return false;
  if (!a.timer.setup_overflow_irq(STEP_TIMER_IRQ_PRIORITY)) return false;   // before open()
  if (!a.timer.open()) return false;   // opened, not counting until the first stepSetRate()
  a.timerOk = true;
  return true;
}

// ---- servo, LEDs, LCD -------------------------------------------------------------------------------
PwmOut g_servoPwm(PIN_TRIGGER_SERVO);   // constructor stores the pin only
bool g_servoOn = false;
float g_servoDeg = TRIGGER_REST_ANGLE;

int servoPulseUs(float deg) {
  return (int)(TRIGGER_SERVO_MIN_US +
               deg / SERVO_SPAN_DEG * (TRIGGER_SERVO_MAX_US - TRIGGER_SERVO_MIN_US) + MATH_HALF);
}

Adafruit_NeoPixel g_strip(LED_COUNT, PIN_LED_DATA, LED_PIXEL_ORDER + NEO_KHZ800);
uint32_t g_ledsOver = 0;

uint8_t ledGuard(uint8_t v) {
  if (v <= LED_MAX_BRIGHTNESS) return v;
  g_ledsOver++;
  return LED_MAX_BRIGHTNESS;
}

LiquidCrystal_I2C g_lcdPrimary(LCD_I2C_ADDR, LCD_COLS, LCD_ROWS);
LiquidCrystal_I2C g_lcdFallback(LCD_I2C_ADDR_FALLBACK, LCD_COLS, LCD_ROWS);
bool g_lcdOk = false;
uint8_t g_lcdAddr = 0;
uint8_t g_lcdBl = 0;            // LCD_PCF_BL or 0, sent with every transaction
uint8_t g_lcdRow = 0;           // where the display's cursor is
uint8_t g_lcdCol = 0;
uint32_t g_lcdErrors = 0;
uint32_t g_lcdReinits = 0;
lcdinit::Sequencer g_lcdInit;   // runs after a failed transaction (LcdInit.h)

bool i2cProbe(uint8_t addr) {
  Wire.beginTransmission(addr);
  return Wire.endTransmission() == 0;
}

// Ends the transaction begun by the caller; false (counted) unless the backpack acknowledged
// every byte (Wire.endTransmission() == 0; 2/3 = NAK, 5 = timeout).
bool lcdEnd() {
  if (Wire.endTransmission() == 0) return true;
  g_lcdErrors++;
  return false;
}

// One transaction: both nibbles of `value`, each strobed with EN (PCF8574: RS=P0 RW=P1 EN=P2
// BL=P3 D4-D7=P4-P7). ~0.5 ms at 100 kHz; the HD44780 executes a write in ~40 us, well inside
// the gap before the next transaction.
bool lcdSend(uint8_t value, bool data) {
  const uint8_t mode = (uint8_t)((data ? LCD_PCF_RS : 0) | g_lcdBl);
  const uint8_t hi = (uint8_t)((value & LCD_NIBBLE_MASK) | mode);
  const uint8_t lo = (uint8_t)(((value << LCD_NIBBLE_SHIFT) & LCD_NIBBLE_MASK) | mode);
  Wire.beginTransmission(g_lcdAddr);
  Wire.write((uint8_t)(hi | LCD_PCF_EN));
  Wire.write(hi);
  Wire.write((uint8_t)(lo | LCD_PCF_EN));
  Wire.write(lo);
  return lcdEnd();
}

// One re-initialisation step (LcdInit.h) as one transaction.
bool lcdInitStep(lcdinit::Kind kind, uint8_t value) {
  if (kind == lcdinit::COMMAND) return lcdSend(value, false);
  Wire.beginTransmission(g_lcdAddr);
  if (kind == lcdinit::NIBBLE) {   // 8-bit-mode function set: the high nibble alone
    const uint8_t nib = (uint8_t)((value & LCD_NIBBLE_MASK) | g_lcdBl);
    Wire.write((uint8_t)(nib | LCD_PCF_EN));
    Wire.write(nib);
  } else {                         // PROBE: the backlight bit only, EN low (no strobe)
    Wire.write(g_lcdBl);
  }
  return lcdEnd();
}

void beginLcd(hal::BeginReport& report) {
  Wire.begin();
  Wire.setWireTimeout(LCD_I2C_TIMEOUT_US, true);   // a stuck bus must not hang setup() either
  LiquidCrystal_I2C* lcd = nullptr;
  if (i2cProbe(LCD_I2C_ADDR)) {
    lcd = &g_lcdPrimary;
    g_lcdAddr = LCD_I2C_ADDR;
  } else if (i2cProbe(LCD_I2C_ADDR_FALLBACK)) {
    lcd = &g_lcdFallback;
    g_lcdAddr = LCD_I2C_ADDR_FALLBACK;
  }
  if (!lcd) {   // run without an LCD; App logs it from the report
    g_lcdAddr = 0;
    return;
  }
  lcd->init();                          // ~1 s of delays: setup() only; ends cleared at (0,0)
  Wire.setClock(LCD_I2C_CLOCK_HZ);      // init() -> Wire.begin() reset the clock
  Wire.setWireTimeout(LCD_I2C_TIMEOUT_US, true);
  lcd->backlight();
  g_lcdBl = LCD_PCF_BL;
  g_lcdRow = 0;
  g_lcdCol = 0;
  g_lcdOk = true;
  report.lcd = true;
  report.lcdAddress = g_lcdAddr;
}

// ---- serial ---------------------------------------------------------------------------------------
TxQueue<DEBUG_TX_BUFFER_BYTES> g_usbTx;
TxQueue<LINK_TX_BUFFER_BYTES> g_linkTx;

// At most one UART::write() per port per call: TxQueue hands out a burst only when the
// transmitter has finished the previous one, so the write returns at once (see TxQueue.h).
template <size_t N>
void drainTo(TxQueue<N>& q, UART& port, uint32_t nowUs) {
  uint8_t chunk[SCI_FIFO_DEPTH];
  const size_t n = q.take(nowUs, chunk, sizeof chunk);
  if (n) port.write(chunk, n);   // non-const overload: one FSP call for all n bytes
}

size_t readFrom(UART& port, uint8_t* buf, size_t cap) {
  size_t n = 0;
  while (n < cap && port.available() > 0) {
    const int c = port.read();
    if (c < 0) break;
    buf[n++] = (uint8_t)c;
  }
  return n;
}

hal::BeginReport g_report;

}  // namespace

// ================================================================================================
namespace hal {

bool begin() {
  // Cycle counter for the step pulse width and the jitter statistics.
  CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
  DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;

  // 1. Steppers: drivers disabled first (EN high), then the step timers, before anything else
  //    can allocate a timer.
  pinMode(PIN_STEPPER_EN, OUTPUT);
  digitalWrite(PIN_STEPPER_EN, HIGH);
  g_driversOn = false;
  g_report.panTimer = beginAxis(AXIS_PAN, STEP_TIMER_PAN_CHANNEL, PIN_PAN_STEP, PIN_PAN_DIR);
  g_report.tiltTimer = beginAxis(AXIS_TILT, STEP_TIMER_TILT_CHANNEL, PIN_TILT_STEP, PIN_TILT_DIR);

  // 2. Trigger servo: hardware PWM at the rest angle, then parked (LOW) until enabled.
  g_report.servo = g_servoPwm.begin((uint32_t)SERVO_PERIOD_US, (uint32_t)servoPulseUs(g_servoDeg), false);
  if (g_report.servo) g_servoPwm.suspend();
  g_servoOn = false;

  // 3. Flywheel: coasting, duty 0.
  pinMode(PIN_FLY_IN1, OUTPUT);
  pinMode(PIN_FLY_IN2, OUTPUT);
  digitalWrite(PIN_FLY_IN1, LOW);
  digitalWrite(PIN_FLY_IN2, LOW);
  analogWrite(PIN_FLY_ENA, 0);

  // 4. Inputs.
  pinMode(PIN_TOGGLE_POWER, TOGGLE_ACTIVE_LOW ? INPUT_PULLUP : INPUT);
  pinMode(PIN_TOGGLE_MODE, TOGGLE_ACTIVE_LOW ? INPUT_PULLUP : INPUT);
  pinMode(PIN_STALL_SENSE, INPUT);
  analogReadResolution(ADC_RESOLUTION_BITS);

  // 5. UARTs and their paced queues.
  Serial.begin(DEBUG_SERIAL_BAUD);
  LINK_SERIAL.begin(LINK_BAUD);
  g_usbTx.begin(DEBUG_SERIAL_BAUD, SERIAL_TX_SLACK_BYTES, micros());
  g_linkTx.begin(LINK_BAUD, SCI_FIFO_DEPTH, micros());

  // 6. LEDs dark (no step is running yet, so show()'s interrupt mask costs nothing).
  g_strip.begin();
  g_strip.clear();
  g_strip.show();

  // 7. LCD (blocks ~1 s in init()).
  beginLcd(g_report);

  g_statsStartUs = micros();
  return g_report.panTimer && g_report.tiltTimer;
}

const BeginReport& beginReport() { return g_report; }

// The pacer needs the time of the write itself, not the caller's (taken earlier in loop()):
// a burst written later than its timestamp says would let the next one in too early.
void service(uint32_t /*nowUs*/) {
  const uint32_t now = micros();
  drainTo(g_usbTx, Serial, now);
  drainTo(g_linkTx, LINK_SERIAL, now);
}

uint32_t nowMs() { return millis(); }
uint32_t nowUs() { return micros(); }

// ---- GPIO, ADC, PWM ---------------------------------------------------------------------------------
void writePin(uint8_t pin, bool high) { digitalWrite(pin, high ? HIGH : LOW); }
bool readPin(uint8_t pin) { return digitalRead(pin) == HIGH; }
uint16_t readAnalog(uint8_t pin) { return (uint16_t)analogRead(pin); }

bool pwmWrite(uint8_t pin, uint8_t duty) {
  if (pin != PIN_FLY_ENA) return false;   // see hal.h: other PWM pins share our timers
  analogWrite(pin, duty);
  return true;
}

// ---- servo ---------------------------------------------------------------------------------------------
void servoEnable(bool on) {
  if (!g_report.servo || on == g_servoOn) return;
  if (on) {
    g_servoPwm.pulseWidth_us(servoPulseUs(g_servoDeg));
    g_servoPwm.resume();
  } else {
    g_servoPwm.suspend();   // counter stopped, pin LOW
  }
  g_servoOn = on;
}

bool servoEnabled() { return g_servoOn; }

void servoWriteDeg(float deg) {
  if (!(deg >= 0.0f)) deg = 0.0f;   // also catches NaN
  if (deg > SERVO_SPAN_DEG) deg = SERVO_SPAN_DEG;
  g_servoDeg = deg;
  if (g_report.servo) g_servoPwm.pulseWidth_us(servoPulseUs(deg));   // buffered compare
}

float servoDeg() { return g_servoDeg; }

// ---- steppers ---------------------------------------------------------------------------------------------
void stepperEnable(bool on) {
  digitalWrite(PIN_STEPPER_EN, on ? LOW : HIGH);   // EN is active LOW
  g_driversOn = on;
}

bool steppersEnabled() { return g_driversOn; }

void stepSetRate(Axis axis, float stepsPerSec) {
  if (axis >= AXIS_COUNT) return;
  StepAxis& a = g_axes[axis];
  if (!a.timerOk) return;
  const int8_t want = stepsPerSec < 0.0f ? -1 : 1;
  float mag = stepsPerSec < 0.0f ? -stepsPerSec : stepsPerSec;
  if (!(mag >= STEP_MIN_RATE_HZ)) {   // too slow, zero or NaN: stop
    if (a.running) {
      a.timer.stop();
      a.running = false;
    }
    a.rate = 0.0f;
    return;
  }
  if (mag > STEP_MAX_RATE_HZ) mag = STEP_MAX_RATE_HZ;
  const uint32_t counts = (uint32_t)((float)GPT_CLOCK_HZ / mag);
  if (!a.running) {
    {
      // The timer is stopped, so no step interrupt can run; the lock only keeps the fields the
      // interrupt reads consistent with each other.
      IrqLock lock;
      a.pendingDir = want;
      a.dir = want;
      writeDir(a, want);   // DIR settles a whole period before the first edge
      a.periodCyc = counts * kCyclesPerCount;
    }
    a.timer.set_period(counts);   // stopped: writes GTPR and clears the counter
    a.timer.start();
    a.lastEdgeCyc = DWT->CYCCNT;  // the first edge is one full period from here
    a.running = true;
  } else {
    a.pendingDir = want;          // the interrupt switches DIR after its next step
    if (counts != a.periodCounts) a.timer.set_period(counts);   // takes effect at next overflow
  }
  a.periodCounts = counts;
  a.rate = want > 0 ? mag : -mag;
}

float stepRate(Axis axis) { return axis < AXIS_COUNT ? g_axes[axis].rate : 0.0f; }

int32_t stepPosition(Axis axis) { return axis < AXIS_COUNT ? g_axes[axis].position : 0; }

void stepSetPosition(Axis axis, int32_t steps) {
  if (axis >= AXIS_COUNT) return;
  IrqLock lock;
  g_axes[axis].position = steps;
}

void stepSetBounds(Axis axis, bool enabled, int32_t minSteps, int32_t maxSteps) {
  if (axis >= AXIS_COUNT) return;
  IrqLock lock;
  g_axes[axis].boundMin = minSteps;
  g_axes[axis].boundMax = maxSteps;
  g_axes[axis].boundsOn = enabled;
}

// Read from the timer itself, not from the interrupt's timestamps: the next STEP edge is the
// next overflow, GTPR + 1 - GTCNT counts away (up-counting saw wave), whatever delayed the
// last interrupt. An overflow between the reads (GTCNT went down) means GTPR was just reloaded
// from GTPBR: read it again so count and period belong to the same cycle.
uint32_t stepQuietUs() {
  uint32_t best = UINT32_MAX;
  for (uint8_t i = 0; i < AXIS_COUNT; i++) {
    StepAxis& a = g_axes[i];
    if (!a.running) continue;   // stopped axes have no edge due
    uint32_t cnt = a.gpt->GTCNT;
    uint32_t pr = a.gpt->GTPR;
    const uint32_t cnt2 = a.gpt->GTCNT;
    if (cnt2 < cnt) {
      pr = a.gpt->GTPR;
      cnt = cnt2;
    }
    const uint32_t remainCounts = pr >= cnt ? pr - cnt + 1u : 0;
    const uint32_t remainUs = remainCounts / kCountsPerUs;
    if (remainUs < best) best = remainUs;
  }
  return best;
}

void stepTakeStats(StepStats& out) {
  out = StepStats();
  const uint32_t now = micros();
  out.windowUs = now - g_statsStartUs;
  g_statsStartUs = now;
  for (uint8_t i = 0; i < AXIS_COUNT; i++) {
    StepAxis& a = g_axes[i];
    uint32_t steps, refusals, n, maxPm;
    uint64_t sum;
    {
      IrqLock lock;
      steps = a.steps;
      refusals = a.refusals;
      n = a.jitN;
      maxPm = a.jitMaxPm;
      sum = a.jitSumPm;
      a.steps = 0;
      a.refusals = 0;
      a.jitN = 0;
      a.jitMaxPm = 0;
      a.jitSumPm = 0;
    }
    const uint32_t mean = n ? (uint32_t)(sum / n) : 0;
    out.steps[i] = steps;
    out.refusals[i] = refusals;
    out.samples[i] = n;
    out.jitterMaxPermille[i] = maxPm > UINT16_MAX ? UINT16_MAX : (uint16_t)maxPm;
    out.jitterMeanPermille[i] = mean > UINT16_MAX ? UINT16_MAX : (uint16_t)mean;
  }
}

// ---- LEDs --------------------------------------------------------------------------------------------------
void ledsSetPixel(uint8_t index, uint8_t r, uint8_t g, uint8_t b) {
  if (index >= LED_COUNT) return;
  g_strip.setPixelColor(index, ledGuard(r), ledGuard(g), ledGuard(b));
}

void ledsShow() { g_strip.show(); }

uint32_t ledsOverLimit() { return g_ledsOver; }

// ---- LCD ---------------------------------------------------------------------------------------------------
bool lcdPresent() { return g_lcdOk; }
uint8_t lcdAddress() { return g_lcdAddr; }

bool lcdBacklight(bool on) {
  if (!g_lcdOk) return false;
  g_lcdBl = on ? LCD_PCF_BL : 0;
  if (g_lcdInit.active()) return true;   // the re-initialisation's transactions carry the bit
  Wire.beginTransmission(g_lcdAddr);
  Wire.write(g_lcdBl);
  if (lcdEnd()) return true;
  g_lcdInit.start();
  return false;
}

LcdWrite lcdWriteAt(uint8_t col, uint8_t row, char c) {
  if (!g_lcdOk || row >= LCD_ROWS || col >= LCD_COLS) return LcdWrite::FAILED;
  if (g_lcdInit.active()) {
    const LcdWrite r = g_lcdInit.run(micros(), lcdInitStep);
    if (!g_lcdInit.active()) {   // done: the clear left the cursor at (0,0)
      g_lcdRow = 0;
      g_lcdCol = 0;
      g_lcdReinits++;
    }
    return r;
  }
  if (row != g_lcdRow || col != g_lcdCol) {
    const uint8_t ddram = (uint8_t)((row ? HD44780_ROW1_DDRAM : 0) + col);
    if (!lcdSend((uint8_t)(HD44780_CMD_SET_DDRAM | ddram), false)) {
      g_lcdInit.start();
      return LcdWrite::FAILED;
    }
    g_lcdRow = row;
    g_lcdCol = col;
    return LcdWrite::AGAIN;   // the character goes in the next call
  }
  if (!lcdSend((uint8_t)c, true)) {
    g_lcdInit.start();
    return LcdWrite::FAILED;
  }
  g_lcdCol++;       // the HD44780 auto-increments; past the last column a cursor move follows
  return LcdWrite::WROTE;
}

uint32_t lcdErrors() { return g_lcdErrors; }
uint32_t lcdReinits() { return g_lcdReinits; }

// ---- USB serial ------------------------------------------------------------------------------------------------
size_t usbRead(uint8_t* buf, size_t cap) { return readFrom(Serial, buf, cap); }
bool usbWrite(const uint8_t* data, size_t len) { return g_usbTx.push(data, len); }
size_t usbTxFree() { return g_usbTx.freeBytes(); }
uint32_t usbTxDropped() { return g_usbTx.dropped(); }

// ---- AT link ------------------------------------------------------------------------------------------------------
size_t linkRead(uint8_t* buf, size_t cap) { return readFrom(LINK_SERIAL, buf, cap); }
bool linkWrite(const uint8_t* data, size_t len) { return g_linkTx.push(data, len); }
bool linkTxIdle() { return g_linkTx.idle(micros()); }
size_t linkTxFree() { return g_linkTx.freeBytes(); }
uint32_t linkTxDropped() { return g_linkTx.dropped(); }

}  // namespace hal

#endif  // ARDUINO

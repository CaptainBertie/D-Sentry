// Hall sensor (KY-024) + LED + 16x2 I2C LCD bring-up test
// Board: Arduino Uno R4 WiFi under a CNC Shield V3 (steppers not used)
#include <Arduino.h>
#include <Wire.h>
#include <LiquidCrystal_I2C.h>
#include <Adafruit_NeoPixel.h>

// ---- Constants ----
const int THRESHOLD = 560;      // A0 counts; LED/state flips here (before hysteresis)
const int HYST = 10;            // +/-10 count hysteresis band around THRESHOLD
const uint8_t LCD_ADDR = 0x27;  // preferred I2C address; falls back to 0x3F if absent

const uint8_t PIN_HALL_A0 = A0;
const uint8_t PIN_HALL_D0 = 12;
const uint8_t LED_OUT_PIN = 9; // named to avoid colliding with the core's PIN_LED macro (built-in LED)

// Freenove 8 RGB LED Module: 8 WS2812 chips chained on one data pin (S/V/G),
// not a plain LED, so it needs the NeoPixel data protocol, and all 8 need
// their own pixel written each frame or only the first one updates.
const uint16_t NUM_PIXELS = 8;
Adafruit_NeoPixel rgb(NUM_PIXELS, LED_OUT_PIN, NEO_GRB + NEO_KHZ800);
const uint32_t LED_ON_COLOR = Adafruit_NeoPixel::Color(255, 255, 255);
const uint32_t LED_OFF_COLOR = 0;

const unsigned long LED_SLOW_MS = 500; // on/off time, BELOW threshold
const unsigned long LED_FAST_MS = 100; // on/off time, ABOVE threshold
const unsigned long LCD_REFRESH_MS = 100;
const unsigned long SERIAL_PRINT_MS = 200;

LiquidCrystal_I2C *lcd = nullptr;

bool aboveThreshold = false;
unsigned long lastLcdUpdate = 0;
unsigned long lastSerialPrint = 0;

void lcdPrintPadded(uint8_t row, const char *text) {
  char buf[17];
  uint8_t i = 0;
  for (; i < 16 && text[i] != '\0'; i++) buf[i] = text[i];
  for (; i < 16; i++) buf[i] = ' ';
  buf[16] = '\0';
  lcd->setCursor(0, row);
  lcd->print(buf);
}

// Scans the I2C bus, prints every address found, and returns the LCD
// address to use: LCD_ADDR if present, else 0x3F if that is present instead.
uint8_t scanI2CForLcdAddr() {
  Serial.println(F("I2C scan:"));
  bool found27 = false;
  bool found3F = false;
  for (uint8_t addr = 1; addr < 127; addr++) {
    Wire.beginTransmission(addr);
    if (Wire.endTransmission() == 0) {
      Serial.print(F("  found 0x"));
      Serial.println(addr, HEX);
      if (addr == 0x27) found27 = true;
      if (addr == 0x3F) found3F = true;
    }
  }
  if (found27) return 0x27;
  if (found3F) {
    Serial.println(F("0x27 absent, 0x3F present -> using 0x3F"));
    return 0x3F;
  }
  Serial.println(F("Neither 0x27 nor 0x3F found; defaulting to 0x27"));
  return LCD_ADDR;
}

void setup() {
  Serial.begin(115200);
  pinMode(PIN_HALL_D0, INPUT);
  rgb.begin();
  rgb.show(); // off

  Wire.begin();
  uint8_t effectiveAddr = scanI2CForLcdAddr();
  lcd = new LiquidCrystal_I2C(effectiveAddr, 16, 2);
  lcd->init();
  lcd->backlight();

  int baseline = analogRead(PIN_HALL_A0);
  float baselineV = baseline * 5.0 / 1023.0;
  Serial.print(F("Baseline (no field) A0 = "));
  Serial.print(baseline);
  Serial.print(F(" ("));
  Serial.print(baselineV, 2);
  Serial.println(F(" V)"));

  aboveThreshold = (baseline >= THRESHOLD);
}

void loop() {
  unsigned long now = millis();

  int raw = analogRead(PIN_HALL_A0);
  int d0 = digitalRead(PIN_HALL_D0);

  // Hysteresis: only flip state once the reading clears the +/-HYST band,
  // so the LED/LCD do not flicker when the reading sits near THRESHOLD.
  if (!aboveThreshold && raw >= THRESHOLD + HYST) {
    aboveThreshold = true;
  } else if (aboveThreshold && raw < THRESHOLD - HYST) {
    aboveThreshold = false;
  }

  // Re-sent every loop (not just on change) so a WS2812 bit glitch on any one
  // pixel self-heals on the next pass instead of leaving a LED stuck wrong.
  unsigned long blinkInterval = aboveThreshold ? LED_FAST_MS : LED_SLOW_MS;
  bool ledPhysicalOn = (now % (2 * blinkInterval)) < blinkInterval;
  uint32_t color = ledPhysicalOn ? LED_ON_COLOR : LED_OFF_COLOR;
  for (uint16_t i = 0; i < NUM_PIXELS; i++) {
    rgb.setPixelColor(i, color);
  }
  rgb.show();

  if (now - lastLcdUpdate >= LCD_REFRESH_MS) {
    lastLcdUpdate = now;
    float volts = raw * 5.0 / 1023.0;
    char voltStr[6];
    dtostrf(volts, 4, 2, voltStr);

    char line0[17];
    snprintf(line0, sizeof(line0), "Hall: %4d %sV", raw, voltStr);
    lcdPrintPadded(0, line0);

    lcdPrintPadded(1, aboveThreshold ? "State: ABOVE" : "State: BELOW");
  }

  if (now - lastSerialPrint >= SERIAL_PRINT_MS) {
    lastSerialPrint = now;
    Serial.print(F("A0="));
    Serial.print(raw);
    Serial.print(F(" D0="));
    Serial.print(d0 == LOW ? F("LOW(active)") : F("HIGH(inactive)"));
    Serial.print(F(" LED="));
    Serial.println(aboveThreshold ? F("FAST") : F("SLOW"));
  }
}

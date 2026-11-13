// i2c_scan.ino -- bench tool: which address does the LCD backpack answer on? (spec 9 item 3)
//
// Upload this sketch on its own (not D-Sentry) to the UNO R4 WiFi with the LCD wired to A4 (SDA)
// and A5 (SCL), open the serial monitor at SCAN_BAUD, and read the report. It repeats every
// RESCAN_MS, so you can re-seat a wire and watch the result change. It prints:
//   - the idle level of SDA and SCL (both must read HIGH: the backpack's pull-ups);
//   - every address 0x08..0x77 that ACKs, with the common LCD backpacks marked;
//   - the line to put in D-SentryCode/config.h:  #define LCD_I2C_ADDR 0x27  (or 0x3F ...).
// D-Sentry itself probes LCD_I2C_ADDR, then LCD_I2C_ADDR_FALLBACK (0x3F), and runs without an LCD
// if neither answers, so this tool is for when the LCD stays blank.
//
// Standalone bench sketch: it may use delay(); it does not include config.h, so the values it
// shares with config.h are repeated below under the same meaning.
//
// SPDX-License-Identifier: MIT
#include <Arduino.h>
#include <Wire.h>

// ---- settings (match config.h where noted) --------------------------------------------------
const unsigned long SCAN_BAUD = 115200;      // = DEBUG_SERIAL_BAUD; never 1200/2400 (those reset the
                                             //   RA4M1 through the ESP32 USB bridge)
const uint32_t SCAN_I2C_CLOCK_HZ = 100000;   // = LCD_I2C_CLOCK_HZ (PCF8574 is rated 100 kHz)
const unsigned int SCAN_TIMEOUT_US = 3000;   // = LCD_I2C_TIMEOUT_US: a stuck bus costs this, not forever
const unsigned long RESCAN_MS = 5000;        // pause between scans
const unsigned long SERIAL_WAIT_MS = 2000;   // how long setup() waits for a serial monitor

// ---- I2C facts ----------------------------------------------------------------------------------
const uint8_t FIRST_ADDRESS = 0x08;          // 0x00-0x07 and 0x78-0x7F are reserved by the I2C spec
const uint8_t LAST_ADDRESS = 0x77;
const uint8_t LCD_PCF8574 = 0x27;            // PCF8574T backpack, A0-A2 jumpers open (most common)
const uint8_t LCD_PCF8574A = 0x3F;           // PCF8574AT backpack, A0-A2 jumpers open
const uint8_t PCF8574_BASE = 0x20;           // PCF8574T: 0x20..0x27 as the A0-A2 jumpers are bridged
const uint8_t PCF8574A_BASE = 0x38;          // PCF8574AT: 0x38..0x3F
const uint8_t PCF_ADDRESSES = 8;             // three address jumpers
const uint8_t HEX_TWO_DIGITS = 0x10;         // below this a byte needs a leading zero in hex
const uint8_t MAX_FOUND = LAST_ADDRESS - FIRST_ADDRESS + 1;

uint8_t found[MAX_FOUND];
uint8_t foundCount = 0;
unsigned long scans = 0;

void printHex(uint8_t v) {
  Serial.print("0x");
  if (v < HEX_TWO_DIGITS) Serial.print('0');
  Serial.print(v, HEX);
}

bool isPcf8574(uint8_t a) { return a >= PCF8574_BASE && a < PCF8574_BASE + PCF_ADDRESSES; }
bool isPcf8574A(uint8_t a) { return a >= PCF8574A_BASE && a < PCF8574A_BASE + PCF_ADDRESSES; }
bool isLcdBackpack(uint8_t a) { return isPcf8574(a) || isPcf8574A(a); }

const char* describe(uint8_t a) {
  if (a == LCD_PCF8574) return "  <-- LCD backpack (PCF8574T, default jumpers)";
  if (a == LCD_PCF8574A) return "  <-- LCD backpack (PCF8574AT, default jumpers)";
  if (isPcf8574(a)) return "  <-- probably an LCD backpack (PCF8574T, address jumpers bridged)";
  if (isPcf8574A(a)) return "  <-- probably an LCD backpack (PCF8574AT, address jumpers bridged)";
  return "  (not an LCD backpack address)";
}

// The bus must idle HIGH on both lines; a LOW line means no pull-ups, a short, or a device
// holding the bus. Read before Wire takes the pins.
void reportIdleLevels() {
  pinMode(SDA, INPUT);
  pinMode(SCL, INPUT);
  const bool sda = digitalRead(SDA) == HIGH;
  const bool scl = digitalRead(SCL) == HIGH;
  Serial.print("Bus idle: SDA (A4) ");
  Serial.print(sda ? "HIGH" : "LOW");
  Serial.print(", SCL (A5) ");
  Serial.println(scl ? "HIGH" : "LOW");
  if (!sda || !scl) {
    Serial.println("  A line reads LOW with nothing driving it: check the 4.7k pull-ups on the backpack,");
    Serial.println("  GND/5V to the backpack, and that SDA and SCL are not swapped or shorted.");
  }
}

void suggest() {
  uint8_t lcd = 0;
  uint8_t lcdCandidates = 0;
  for (uint8_t i = 0; i < foundCount; i++) {
    if (!isLcdBackpack(found[i])) continue;
    lcdCandidates++;
    // prefer the two stock addresses, in the order D-Sentry probes them
    if (lcd == 0 || found[i] == LCD_PCF8574 || (found[i] == LCD_PCF8574A && lcd != LCD_PCF8574))
      lcd = found[i];
  }
  if (foundCount == 0) {
    Serial.println("No device answered. Check: backpack VCC to 5V, GND to GND, SDA to A4, SCL to A5,");
    Serial.println("and that the four wires are seated. D-Sentry will run without an LCD.");
    return;
  }
  if (lcdCandidates == 0) {
    Serial.println("Devices answered, but none at an LCD backpack address (0x20-0x27, 0x38-0x3F).");
    return;
  }
  Serial.print("Put this in D-SentryCode/config.h:   #define LCD_I2C_ADDR ");
  printHex(lcd);
  Serial.println();
  if (lcd == LCD_PCF8574A)
    Serial.println("(D-Sentry also finds 0x3F by itself as LCD_I2C_ADDR_FALLBACK, but setting it is clearer.)");
  if (lcdCandidates > 1)
    Serial.println("More than one backpack address answered: unplug the extra device, or pick by its jumpers.");
}

void scan() {
  foundCount = 0;
  uint8_t timeouts = 0;
  scans++;
  Serial.print("Scan ");
  Serial.print(scans);
  Serial.print(" of ");
  printHex(FIRST_ADDRESS);
  Serial.print("..");
  printHex(LAST_ADDRESS);
  Serial.println(":");
  for (uint8_t a = FIRST_ADDRESS; a <= LAST_ADDRESS; a++) {
    Wire.beginTransmission(a);
    const uint8_t rc = Wire.endTransmission();
    if (rc == END_TX_OK) {
      found[foundCount++] = a;
      Serial.print("  found ");
      printHex(a);
      Serial.println(describe(a));
    } else if (rc == END_TX_TIMEOUT) {
      timeouts++;
    }
  }
  if (timeouts) {
    Serial.print("  ");
    Serial.print(timeouts);
    Serial.println(" addresses timed out: the bus is stuck (a line held LOW?)");
  }
  Serial.print(foundCount);
  Serial.println(foundCount == 1 ? " device." : " devices.");
  suggest();
  Serial.println();
}

void setup() {
  Serial.begin(SCAN_BAUD);
  const unsigned long start = millis();
  while (!Serial && millis() - start < SERIAL_WAIT_MS) {
  }
  Serial.println();
  Serial.println("D-Sentry I2C scan (UNO R4 WiFi, Wire on A4/A5)");
  reportIdleLevels();
  Wire.begin();
  Wire.setClock(SCAN_I2C_CLOCK_HZ);
  Wire.setWireTimeout(SCAN_TIMEOUT_US, true);
}

void loop() {
  scan();
  delay(RESCAN_MS);
}

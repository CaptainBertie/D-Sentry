/*
 * StickToLED -- the left stick dims an LED on D11 and the right trigger sets how fast the
 * built-in LED blinks. Prints the brightness ten times a second (Serial Monitor, 115200 baud).
 *
 * Wiring: D11 -> 220 ohm resistor -> LED long leg (anode); LED short leg (cathode) -> GND.
 * D11 is one of the header's PWM pins (marked ~), so analogWrite() can dim it. The built-in
 * LED (LED_BUILTIN, D13) is not a marked PWM pin, so it only blinks here.
 * No LED to hand? The MatrixDot example uses the board's built-in 12x8 LED matrix instead.
 *
 * Needs the forked bridge firmware on the ESP32-S3 (see the library README).
 *
 * SPDX-License-Identifier: MIT
 */
#include <XboxControllerR4.h>

XboxControllerR4 xbox;

const int DIM_LED_PIN = 11;
const unsigned long POLL_INTERVAL_MS = 20;     // 50 Hz
const unsigned long PRINT_INTERVAL_MS = 100;   // 10 Hz
const unsigned long RETRY_INTERVAL_MS = 2000;  // how often a failed begin() is tried again
const float STICK_DEADZONE = 0.10f;            // a resting stick reads a few percent off centre
const unsigned long LINK_LOST_MS = 200;           // no good poll for this long: ESP32-S3 not answering
const unsigned long SLOW_POLL_INTERVAL_MS = 500;  // poll rate while the ESP32-S3 is not answering

bool ready = false;
int brightness = 0;
bool builtinOn = false;
unsigned long lastPollMs = 0;
unsigned long lastPrintMs = 0;
unsigned long lastRetryMs = 0;
unsigned long lastToggleMs = 0;

// True when no poll() has succeeded for LINK_LOST_MS: the ESP32-S3 has stopped answering.
// The library then keeps the last controller values, so the sketch must notice this itself.
bool linkLost() {
  return millis() - xbox.lastPollOkMillis() > LINK_LOST_MS;
}

// An unanswered poll blocks for its whole 50 ms timeout, so poll less often while the link
// is lost; at 50 Hz the RA4M1 would otherwise be blocked almost all the time.
unsigned long pollInterval() {
  return linkLost() ? SLOW_POLL_INTERVAL_MS : POLL_INTERVAL_MS;
}

void startClient() {
  ready = xbox.begin();
  lastRetryMs = millis();  // a failed begin() is retried RETRY_INTERVAL_MS from now, whoever called it
  if (!ready) {
    Serial.print("XboxControllerR4 begin() failed, lastError() = ");
    Serial.print(xbox.lastError());
    Serial.println(" (1 = stock ESP32-S3 firmware, see PrintAllInputs for details)");
  }
}

// Pushing the stick up brightens the LED; centre or down is off.
int brightnessFromStick(float y) {
  if (y < STICK_DEADZONE) {
    return 0;
  }
  float fraction = (y - STICK_DEADZONE) / (1.0f - STICK_DEADZONE);
  if (fraction > 1.0f) {
    fraction = 1.0f;
  }
  return (int)(fraction * 255.0f + 0.5f);
}

// Released trigger: blink once a second (500 ms on, 500 ms off). Fully pulled: 10 times a second.
unsigned long blinkHalfPeriodMs(float trigger) {
  return 500UL - (unsigned long)(trigger * 450.0f);
}

void setup() {
  Serial.begin(115200);
  pinMode(DIM_LED_PIN, OUTPUT);
  pinMode(LED_BUILTIN, OUTPUT);
  delay(500);  // give the Serial Monitor a moment to attach
  startClient();
}

void loop() {
  unsigned long now = millis();

  if (!ready) {
    if (now - lastRetryMs >= RETRY_INTERVAL_MS) {
      startClient();
    }
    return;
  }

  if (now - lastPollMs >= pollInterval()) {
    lastPollMs = now;
    // A good poll that says OFF means the ESP32-S3 restarted: start its Xbox client again.
    if (xbox.poll() && xbox.connectionState() == XBOX_R4_OFF) {
      startClient();
    }
    // While no controller is connected the library reads as neutral, so the LED goes dark.
    // While the ESP32-S3 is not answering the library keeps the last stick value, so the
    // sketch turns the LED off itself: the pattern a motor sketch should copy.
    if (linkLost()) {
      brightness = 0;
    } else {
      brightness = brightnessFromStick(xbox.leftY());
    }
    analogWrite(DIM_LED_PIN, brightness);
  }

  if (now - lastToggleMs >= blinkHalfPeriodMs(xbox.rightTrigger())) {
    lastToggleMs = now;
    builtinOn = !builtinOn;
    digitalWrite(LED_BUILTIN, builtinOn ? HIGH : LOW);
  }

  if (now - lastPrintMs >= PRINT_INTERVAL_MS) {
    lastPrintMs = now;
    if (linkLost()) {
      Serial.print("ESP32-S3 not answering");
    } else {
      Serial.print(xbox.isConnected() ? "connected" : "not connected");
    }
    Serial.print("  brightness ");
    Serial.println(brightness);
  }
}

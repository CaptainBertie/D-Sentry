// Diagnostic: drives STEP/DIR with raw digitalWrite + delayMicroseconds,
// no stepper library at all. If this still twitches or falls short of a
// full 360, that clears AccelStepper/the code as the cause and points at
// the driver module, the CNC shield, or the motor power supply instead.
#include <Arduino.h>

const uint8_t STEP_PIN = 2;
const uint8_t DIR_PIN = 5;
const uint8_t ENABLE_PIN = 8; // shared by all axes, active LOW

const long STEPS_PER_REV = 200;          // full step, 1.8 deg/step
// Slowed way down from 3000us (~166 steps/sec) to test whether the missing
// ~30 degrees is lost steps from starting at full speed with no ramp-up.
const unsigned int STEP_DELAY_US = 10000; // ~50 steps/sec

void pulseSteps(long count) {
  for (long i = 0; i < count; i++) {
    digitalWrite(STEP_PIN, HIGH);
    delayMicroseconds(STEP_DELAY_US);
    digitalWrite(STEP_PIN, LOW);
    delayMicroseconds(STEP_DELAY_US);
  }
}

void setup() {
  Serial.begin(115200);

  pinMode(STEP_PIN, OUTPUT);
  pinMode(DIR_PIN, OUTPUT);
  pinMode(ENABLE_PIN, OUTPUT);
  digitalWrite(ENABLE_PIN, LOW); // LOW enables the A4988 driver
}

void loop() {
  digitalWrite(DIR_PIN, HIGH);
  Serial.println(F("Forward 360, no library"));
  pulseSteps(STEPS_PER_REV);

  digitalWrite(DIR_PIN, LOW);
  Serial.println(F("Back 360, no library"));
  pulseSteps(STEPS_PER_REV);
}

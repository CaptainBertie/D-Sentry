// Holds the A4988 enabled and energized on one phase without ever pulsing
// STEP, so the motor stays still while you measure/adjust Vref safely.
// Vref is measured between the driver's trimpot screw and a nearby GND pin,
// with motor power connected. A4988 current limit = Vref x 2.5.
#include <Arduino.h>

const uint8_t STEP_PIN = 2;
const uint8_t DIR_PIN = 5;
const uint8_t ENABLE_PIN = 8; // shared by all axes, active LOW

void setup() {
  Serial.begin(115200);

  pinMode(STEP_PIN, OUTPUT);
  digitalWrite(STEP_PIN, LOW); // never toggled -- motor does not move

  pinMode(DIR_PIN, OUTPUT);
  digitalWrite(DIR_PIN, LOW);

  pinMode(ENABLE_PIN, OUTPUT);
  digitalWrite(ENABLE_PIN, LOW); // LOW enables the driver, holding the coil

  Serial.println(F("Driver enabled, holding position. Safe to measure Vref."));
}

void loop() {
  // Nothing to do -- staying still is the point.
}

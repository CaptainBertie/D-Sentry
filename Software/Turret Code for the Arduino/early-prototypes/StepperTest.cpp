// NEMA 17 bring-up test: continuous spin in one direction, ramped up to
// max speed. Board: Arduino Uno R4 WiFi under a CNC Shield V3, X-axis
// A4988 driver socket (Akozon CNC Shield + A4988 kit). STEP/DIR interface,
// not the 4-wire unipolar Stepper.h library.
#include <Arduino.h>
#include <AccelStepper.h>

// CNC Shield V3 standard GRBL pinout, X axis.
const uint8_t STEP_PIN = 2;
const uint8_t DIR_PIN = 5;
const uint8_t ENABLE_PIN = 8; // shared by all axes, active LOW

// NEMA 17 standard full step is 1.8 degrees = 200 steps/rev. If the A4988's
// MS1/MS2/MS3 microstepping jumpers are populated, multiply by that factor
// (e.g. 1/8 microstepping = 1600) or the motor will only turn 1/8 of a turn.
const long STEPS_PER_REV = 200;

// Probing for the real ceiling: how fast this motor/driver/supply combo can
// actually go before it loses sync. Still ramped, not an instant jump.
const float MAX_SPEED = 3000.0;    // steps/sec (~15 rev/sec at 200 steps/rev)
const float ACCELERATION = 1500.0; // steps/sec^2 -- ~2s to reach max speed

AccelStepper stepper(AccelStepper::DRIVER, STEP_PIN, DIR_PIN);

void setup() {
  Serial.begin(115200);

  pinMode(ENABLE_PIN, OUTPUT);
  digitalWrite(ENABLE_PIN, LOW); // LOW enables the A4988 driver

  stepper.setMaxSpeed(MAX_SPEED);
  stepper.setAcceleration(ACCELERATION);
  // Target far beyond any real move so it ramps up and holds max speed,
  // spinning one direction continuously instead of oscillating.
  stepper.moveTo(2000000000L);

  Serial.println(F("Ramping up to max speed, one direction, continuous"));
}

void loop() {
  stepper.run();
}

// Flywheel.h -- the two flywheel motors on L298N channel A (spec 3.7, architecture.md 5).
//
// On: IN1/IN2 set the direction per FLYWHEEL_FORWARD_IN1_HIGH (IN1 = that, IN2 = its opposite)
// while ENA is 0, then ENA (PIN_FLY_ENA, the only pin hal::pwmWrite accepts) ramps linearly
// 0 -> FLYWHEEL_DUTY_255 over FLYWHEEL_RAMP_MS: duty = target * elapsed / ramp, rounded down,
// exactly the target from elapsed >= ramp (soft start limits the 3-4 A inrush).
// ready() once on for FLYWHEEL_SPINUP_MS (the firing gate when FIRE_REQUIRES_FLYWHEEL).
//
// Stop (cut(), immediate, no update() needed): ENA 0 first, then
//   coast (FLYWHEEL_BRAKE_ON_STOP false): IN1 = IN2 = LOW, ENA 0. The wheels spin down freely.
//   brake (FLYWHEEL_BRAKE_ON_STOP true):  IN1 = IN2 = HIGH and ENA = PWM_MAX for
//     FLYWHEEL_BRAKE_MS, then coast. The L298N only brakes ("fast motor stop") while its enable
//     is HIGH; with ENA low it lets the motor run free whatever IN1/IN2 say, so IN1 = IN2 = HIGH
//     with ENA 0 would only coast. Braking starts only when the motors were being driven;
//     begin() and a cut() of stopped wheels go straight to coast.
// Flywheel only executes; TurretState decides when it may run (armed, link, faults, LB mode).
//
// SPDX-License-Identifier: MIT
#pragma once
#include <stdint.h>
#include "Types.h"

class Flywheel {
 public:
  // Tunables and wiring, from config.h by default; tests change them.
  struct Config {
    uint8_t duty;          // FLYWHEEL_DUTY_255: ramp target, 0..PWM_MAX
    uint32_t rampMs;       // FLYWHEEL_RAMP_MS: 0 = full duty at once
    uint32_t spinupMs;     // FLYWHEEL_SPINUP_MS
    uint32_t brakeMs;      // FLYWHEEL_BRAKE_MS: how long the brake holds before coasting
    bool brakeOnStop;      // FLYWHEEL_BRAKE_ON_STOP
    bool forwardIn1High;   // FLYWHEEL_FORWARD_IN1_HIGH
  };
  static Config defaults();

  Flywheel();                           // defaults()
  explicit Flywheel(const Config& cfg);
  const Config& config() const { return cfg_; }

  void begin();                         // stopped and coasting (never brakes at boot)
  // Every loop(): advances the ramp (writes ENA only when the duty changes) and ends a brake
  // after FLYWHEEL_BRAKE_MS (timed from the cut, or from the first update() after a bare cut()).
  void update(uint32_t nowMs);

  void setOn(bool on, uint32_t nowMs);  // on: start the ramp (no-op if already on); off: cut()
  void cut();                           // immediate stop, coast or brake per config

  bool on() const { return on_; }
  uint32_t onSinceMs() const { return onSinceMs_; }   // valid while on()
  // On for at least FLYWHEEL_SPINUP_MS.
  bool ready(uint32_t nowMs) const;
  // Drive duty now, 0..duty target: what the motors are being pushed with. 0 while off,
  // coasting or braking (ENA itself is PWM_MAX during a brake; see enaDuty()).
  uint8_t duty() const { return duty_; }
  uint8_t enaDuty() const { return ena_; }            // last value written to ENA
  bool braking() const { return braking_; }

 private:
  uint8_t rampDuty(uint32_t elapsedMs) const;
  void writeEna(uint8_t duty);
  void coast();

  Config cfg_;
  bool on_ = false;
  uint32_t onSinceMs_ = 0;
  uint8_t duty_ = 0;
  uint8_t ena_ = 0;
  bool rampDone_ = false;       // duty_ reached the target; the ramp is not recomputed
  bool spunUp_ = false;         // ready() latched (wrap-proof after 24 days on)
  bool braking_ = false;
  bool brakeTimed_ = false;     // brakeStartMs_ is set
  uint32_t brakeStartMs_ = 0;
};

// Telemetry.h -- TelemetryPackets (docs/architecture.md 4.1) for the PC dashboard.
//
// Three steps, so the encoding is testable without any other module:
//   collect()  reads the modules through const pointers (TelemetrySources) into a
//              TelemetrySnapshot: plain facts in module units (degrees, deg/s, bools, counters);
//   build()    snapshot -> ds_telemetry_packet_t (pure): angles to centidegrees (DS_ANGLE_SCALE)
//              rounded half away from zero and saturated to the field's range (NaN -> 0),
//              rate-like values saturated (loopHz, ctrlRxHz, stepJitterPermille), event
//              counters wrapped to 16 bits (crcErrors, seqGaps), flags packed as DS_TELEM_FLAG_*,
//              LCD text space padded;
//   update()   every 1000/TELEMETRY_USB_HZ ms: collect + build + encode one frame, queue it on
//              USB with hal::usbWrite (the whole frame or nothing; a refusal is counted, never
//              split), and offer the same frame to ControlLink::offerTelemetry(). ControlLink
//              alone decides whether and when it becomes an AT+DST line (TELEMETRY_WIFI_HZ,
//              WiFi mode, D-Sentry ESP32 firmware); Telemetry never times AT+DST.
// `seq` increments once per frame built (USB and the offer carry the same frame and seq).
//
// SPDX-License-Identifier: MIT
#pragma once
#include <stddef.h>
#include <stdint.h>
#include "Types.h"

class TurretState;
class Inputs;
class ControlLink;
class Motion;
class StallDetector;
class Flywheel;
class Trigger;
class Leds;
class Lcd;
class Debug;

// The modules collect() reads. Any may be null: its fields keep the snapshot defaults.
struct TelemetrySources {
  const TurretState* turret = nullptr;     // state, fault, emptyMagBlink (display model)
  const Inputs* inputs = nullptr;          // triggerPulled, power / mode toggles
  const ControlLink* link = nullptr;       // link source, connected, ESP32 status, link stats
  const Motion* motion = nullptr;          // positions, velocities, home, homing, calibrated
  const StallDetector* stall = nullptr;    // sample, baseline, peak, active
  const Flywheel* flywheel = nullptr;      // on, ready, duty
  const Trigger* trigger = nullptr;        // fire mode, stroke, ammo, shots, servo angle
  const Leds* leds = nullptr;              // LED 0 as shown
  const Lcd* lcd = nullptr;                // the 32-character mirror, present
  const Debug* debug = nullptr;            // RuntimeStats: loopHz, step jitter
};

// One TelemetryPacket's facts, in module units. Defaults are what a packet says when the
// module behind a field is absent.
struct TelemetrySnapshot {
  TelemetrySnapshot();   // lcd = all spaces

  uint32_t uptimeMs = 0;                      // hal::nowMs() when collected
  uint8_t state = DS_STATE_OFF;               // DS_STATE_*
  uint8_t fault = DS_FAULT_NONE;              // DS_FAULT_*
  uint8_t fireMode = DS_FIRE_MODE_SEMI;       // DS_FIRE_MODE_*
  uint8_t linkSource = DS_LINK_SRC_NONE;      // DS_LINK_SRC_*
  // TelemetryPacket.flags, one bool per DS_TELEM_FLAG_* bit (protocol_spec.py order)
  bool linkConnected = false;
  bool flywheelOn = false;
  bool flywheelReady = false;
  bool strokeActive = false;
  bool triggerPulled = false;
  bool homing = false;
  bool calibrated = false;
  bool powerToggleOn = false;
  bool modeToggleWifi = false;
  bool stallEnabled = false;
  bool emptyMagBlink = false;
  bool driversEnabled = false;
  bool espLegacyFirmware = false;
  bool lcdPresent = false;
  bool espAlive = false;
  float panDeg = 0.0f;                        // + = counter-clockwise from above
  float tiltDeg = 0.0f;                       // + = up, 0 = calibrated bottom + backoff
  float panVelDegS = 0.0f;
  float tiltVelDegS = 0.0f;
  float homePanDeg = 0.0f;
  float homeTiltDeg = 0.0f;
  uint8_t ammo = 0;
  uint8_t magCapacity = 0;
  uint16_t shotsTotal = 0;                    // wraps
  uint8_t flywheelDuty = 0;                   // 0..PWM_MAX
  float servoDeg = 0.0f;                      // commanded angle; rounded, 0..UINT8_MAX on the wire
  uint16_t stallSample = 0;                   // ADC counts
  uint16_t stallBaseline = 0;
  uint16_t stallPeak = 0;
  uint32_t loopHz = 0;                        // saturates at UINT16_MAX on the wire
  uint32_t ctrlRxHz = 0;                      // saturates
  uint32_t crcErrors = 0;                     // since boot; wire carries the low 16 bits
  uint32_t seqGaps = 0;                       // since boot; low 16 bits
  uint32_t stepJitterPermille = 0;            // saturates
  uint8_t lastCtrlSeq = 0;
  uint8_t espMode = DS_ESP_MODE_UNKNOWN;      // DS_ESP_MODE_*
  Rgb led0;                                   // LED 0 as shown, after the brightness clamp
  char lcd[DS_TELEMETRY_LCD_LEN];             // row 0 then row 1; NULs go out as spaces
  // The controller block (TelemetryPacket version 2), see Telemetry::padBlock(). Wire units:
  // sticks -JOY_AXIS_RANGE..JOY_AXIS_RANGE-1 up negative, triggers 0..TRIGGER_AXIS_RANGE.
  uint8_t padState = DS_PAD_STATE_NONE;       // DS_PAD_STATE_*
  uint8_t padBattery = DS_PAD_BATTERY_UNKNOWN;  // percent, or DS_PAD_BATTERY_UNKNOWN
  int16_t padLx = 0;
  int16_t padLy = 0;
  int16_t padRx = 0;
  int16_t padRy = 0;
  uint16_t padLt = 0;
  uint16_t padRt = 0;
  uint16_t padButtons = 0;                    // DS_BTN_* levels, bits 12-15 included
};

class Telemetry {
 public:
  // App: `link` receives every frame through offerTelemetry().
  void begin(const TelemetrySources& sources, ControlLink& link);
  // Tests: `link` may be null (USB only).
  void begin(const TelemetrySources& sources, ControlLink* link);
  // Once per loop(); does nothing until 1000/TELEMETRY_USB_HZ ms have passed since the last
  // frame (deadline schedule: exactly TELEMETRY_USB_HZ frames a second on average, no
  // catch-up burst after a stall).
  void update(uint32_t nowMs);

  // Read the modules (null sources keep the defaults). nowMs feeds uptimeMs and
  // Flywheel::ready().
  void collect(uint32_t nowMs, TelemetrySnapshot& out) const;
  // Pure: snapshot + sequence number -> packet (see the header comment for the rules).
  static void build(const TelemetrySnapshot& s, uint8_t seq, ds_telemetry_packet_t& out);
  // build() + ds_telemetry_encode(): the whole frame, returns its length.
  static size_t encode(const TelemetrySnapshot& s, uint8_t seq,
                       uint8_t frame[DS_TELEMETRY_FRAME_LEN]);
  // Degrees (or deg/s) -> centidegrees, rounded half away from zero, saturated, NaN -> 0.
  static int32_t centi32(float deg);
  static int16_t centi16(float deg);
  // The controller block from the active source's latest frame (ControlLink::frame()):
  //   no active source (or no frame yet)  padState NONE, battery unknown, everything neutral;
  //   a frame that is not connected        its padState and battery, sticks / triggers /
  //                                         buttons neutral (they are not live input);
  //   a connected frame                    its padState, battery, sticks, triggers, buttons.
  // Pure; collect() calls it, the tests call it directly.
  static void padBlock(const ControlFrame& f, uint8_t activeSource, TelemetrySnapshot& out);

  uint8_t seq() const { return seq_; }                 // the seq the next frame will carry
  uint32_t framesBuilt() const { return built_; }
  uint32_t usbSent() const { return usbSent_; }
  uint32_t usbDropped() const { return usbDropped_; }  // hal::usbWrite refused the frame
  uint32_t linkOffered() const { return offered_; }    // frames handed to ControlLink
  uint32_t linkAccepted() const { return accepted_; }  // of which it took (WiFi telemetry on)

 private:
  TelemetrySources src_;
  ControlLink* link_ = nullptr;
  uint8_t seq_ = 0;
  uint32_t nextUsbMs_ = 0;
  uint32_t built_ = 0;
  uint32_t usbSent_ = 0;
  uint32_t usbDropped_ = 0;
  uint32_t offered_ = 0;
  uint32_t accepted_ = 0;
};

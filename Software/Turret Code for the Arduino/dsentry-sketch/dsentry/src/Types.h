// Types.h -- plain data shared between the RA4M1 modules (docs/architecture.md section 5).
// No behaviour here: the structs every module passes to the next, so that two modules never
// have to include each other just to agree on a shape.
//
// Units used throughout (architecture.md rule 7): angles in degrees (float) inside modules,
// centidegrees (DS_ANGLE_SCALE) only on the wire; time as uint32_t milliseconds (hal::nowMs) for
// logic and microseconds (hal::nowUs) for sampling and scheduling; every comparison of two
// times is wrap-safe: (uint32_t)(now - then) >= period.
//
// SPDX-License-Identifier: MIT
#pragma once
#include <stddef.h>
#include <stdint.h>
#include "../config.h"
#include "../dsentry_protocol.h"

// The two stepper axes. AXIS_COUNT sizes every per-axis array.
enum Axis : uint8_t {
  AXIS_PAN = 0,   // CNC shield X: turret rotation, + = counter-clockwise seen from above
  AXIS_TILT,      // CNC shield Y: barrel elevation, + = up, 0 = calibrated bottom + backoff
  AXIS_COUNT
};

// Bit for one axis in a mask such as Motion::movingAxes() or StallDetector::stallAxes().
inline uint8_t axisBit(Axis a) { return (uint8_t)(1u << a); }

// One input frame from the active control source, as ControlLink publishes it (4.2, 4.3).
// Values are the ControlPacket's own: sticks -JOY_AXIS_RANGE..JOY_AXIS_RANGE-1 with stick UP
// NEGATIVE (Inputs applies JOY_INVERT_Y), triggers 0..TRIGGER_AXIS_RANGE, DS_BTN_* bits.
// lx, ly, lt, battery and padState only feed the TelemetryPacket's controller block (the
// dashboard); no module acts on them, nor on DS_BTN_LS_CLICK..DS_BTN_SHARE.
struct ControlFrame {
  bool valid = false;        // at least one frame arrived from `source` since the link came up
  bool connected = false;    // flags.connected, and the frame is fresher than LINK_TIMEOUT_MS
  bool espPairing = false;   // flags bit 2: the ESP32 is in pairing mode
  bool keyboard = false;     // flags bit 3: the relay's dashboard keyboard is driving
  uint8_t source = DS_LINK_SRC_NONE;  // DS_LINK_SRC_* this frame came from
  int16_t rx = 0;            // right stick X, + = right
  int16_t ry = 0;            // right stick Y, + = DOWN (wire convention)
  uint16_t rt = 0;           // right trigger
  uint16_t buttons = 0;      // DS_BTN_* levels (not edges), bits 12-15 included
  uint8_t seq = 0;           // the packet's seq (echoed in TelemetryPacket.lastCtrlSeq)
  uint32_t rxMs = 0;         // hal::nowMs() when it was accepted
  int16_t lx = 0;            // left stick X, + = right (BLE only; USB / WiFi frames carry 0)
  int16_t ly = 0;            // left stick Y, + = DOWN (BLE only)
  uint16_t lt = 0;           // left trigger (BLE only)
  uint8_t battery = DS_PAD_BATTERY_UNKNOWN;  // percent 0..100 (AT+XBOXSTATE? byte 17)
  uint8_t padState = DS_PAD_STATE_NONE;      // DS_PAD_STATE_* (TelemetryPacket.padState)
};

// What ControlLink knows about the ESP32 and the active source (architecture.md 4.2).
struct LinkStatus {
  bool espAlive = false;     // the ESP32 answered any AT request within LINK_TIMEOUT_MS
  bool espCapsKnown = false; // AT+DSCAPS? has been answered (either way)
  bool espLegacy = false;    // AT+DSCAPS? got bare ERROR: stock or pre-D-Sentry fork (BT only)
  uint8_t espProto = 0;      // DSCAPS dsProto (DS_PROTO_VERSION of the ESP32 build)
  uint8_t espCaps = 0;       // DS_CAP_* bits
  uint8_t espMode = DS_ESP_MODE_UNKNOWN;  // DS_ESP_MODE_* from the last AT+DSS reply
  uint8_t bootId = 0;        // ESP32 boot nonce from AT+DSS (0 = not seen yet)
  uint8_t wifiStatus = DS_WIFI_STATUS_OFF;  // DS_WIFI_STATUS_*
  uint8_t wantMode = DS_RADIO_BLE;          // DS_RADIO_* the RA4M1 is asking for
  uint8_t activeSource = DS_LINK_SRC_NONE;  // DS_LINK_SRC_* currently driving the turret
  bool connected = false;    // the active source's latest frame is connected and fresh
  uint32_t lastFrameMs = 0;  // hal::nowMs() of the last valid frame from any source
  // Added by the ControlLink stage:
  bool wifiUnavailable = false;  // WiFi wanted but the ESP32 cannot run it (legacy firmware, a
                                 // dsProto mismatch, or no DS_CAP_WIFI): the radio never connects
  bool usbControlEnabled = DEBUG_ACCEPT_USB_CONTROL;  // USB ControlPackets are obeyed
  uint32_t lastEspReplyMs = 0;   // hal::nowMs() of the ESP32's last AT final (any result)
};

// Counters for Debug's stats line and the TelemetryPacket. All wrap; never reset.
struct LinkStats {
  uint32_t frames = 0;        // valid control frames accepted, all sources
  uint32_t usbFrames = 0;     // of which over USB
  uint16_t ctrlRxHz = 0;      // valid frames/s from the active source, last second
  uint32_t crcErrors = 0;     // USB deframer CRC errors + XBOXSTATE payloads with a bad CRC
  uint32_t lenErrors = 0;     // USB deframer start bytes with the wrong len
  uint32_t seqGaps = 0;       // missing seq values between consecutive accepted frames (USB
                              // ControlPackets only: XBOXSTATE seq is a snapshot counter)
  uint32_t polls = 0;         // AT requests completed (any result)
  uint32_t pollTimeouts = 0;  // AT requests that got no final in time
  uint32_t pollErrors = 0;    // AT requests that ended in ERROR (any form)
  uint32_t staleReplies = 0;  // AT replies that belonged to an older request
  uint32_t junkBytes = 0;     // AT link bytes outside any reply + USB bytes outside frames/lines
  uint32_t sourceChanges = 0; // active source switched while connected (each = a link loss)
  uint32_t shellLines = 0;    // shell lines handed to Debug
  uint32_t shellOverflows = 0;  // shell lines longer than SHELL_LINE_MAX, dropped
  uint32_t telemetrySent = 0;   // AT+DST lines accepted by the ESP32 (+DST: 1)
  // Added by the ControlLink stage:
  uint32_t radioFrames = 0;      // valid XBOXSTATE frames accepted (good CRC, radio path open)
  uint32_t usbIgnored = 0;       // valid USB ControlPackets dropped: USB control disabled
  uint32_t dssSent = 0;          // AT+DSS requests sent
  uint32_t beginsSent = 0;       // AT+XBOXBEGIN requests sent (legacy firmware)
  uint32_t espRestarts = 0;      // AT+DSS bootId changes seen (each = an ESP32 reboot)
  uint32_t espSilences = 0;      // times the ESP32 went silent for LINK_TIMEOUT_MS (-> pinging)
  uint32_t telemetryOffered = 0; // valid TelemetryPacket frames offered by Telemetry
  uint32_t telemetryNotSent = 0; // AT+DST answered +DST: 0 (the ESP32 had no relay address)
  uint32_t shellQueueFull = 0;   // complete shell lines dropped: SHELL_QUEUE_LINES waiting
  uint32_t shellJunkLines = 0;   // USB lines dropped for a non-text byte (binary residue)
  uint32_t statusSent = 0;       // StatusPackets queued on USB
};

// How PAIRING is shown (architecture.md 6, "PAIRING row 1").
enum class PairingPhase : uint8_t {
  FROM_BOOT,   // never connected since boot: "<mode> Pairing<dots>" indefinitely
  GRACE,       // link lost less than LINK_RECONNECT_GRACE_MS ago: "Pairing<dots>"
  NO_LINK      // link lost longer ago: "No Link" until a source connects
};

// Everything Leds and Lcd render, produced by TurretState once per update. Leds and Lcd
// derive their own animation phases (pulses, blinks, dots) from their own clock and
// `stateSinceMs`, so this stays a snapshot of facts.
struct DisplayModel {
  uint8_t state = DS_STATE_OFF;           // DS_STATE_*
  uint8_t fault = DS_FAULT_NONE;          // DS_FAULT_* (meaningful in DS_STATE_FAULT)
  uint8_t fireMode = DS_FIRE_MODE_SEMI;   // DS_FIRE_MODE_*
  uint8_t radioMode = DS_RADIO_BLE;       // DS_RADIO_* from the A1 toggle (LCD mode label)
  uint8_t ammo = 0;                       // darts left
  bool linkConnected = false;             // a control source is connected
  bool firing = false;                    // FIRING red: a stroke is running or its
                                          // LED_FIRE_FLASH_MIN_MS hold has not ended
  bool emptyMagBlink = false;             // ARMED, ammo 0, lockout on, trigger pulled
  PairingPhase pairing = PairingPhase::FROM_BOOT;  // meaningful in DS_STATE_PAIRING
  uint32_t stateSinceMs = 0;              // hal::nowMs() when `state` was entered
  // SERVO TEACH (TurretState.h; only ever true in DS_STATE_DISARMED):
  bool servoTeach = false;                // teach rows on the LCD, LED_COLOR_SERVO_TEACH
  int16_t servoDeg = 0;                   // the angle the trigger servo is commanded to, whole deg
  bool restRecorded = false;              // REST recorded this power-on ("R<n>", else "R-")
  int16_t restDeg = 0;                    // the REST angle strokes use now, whole degrees
  bool fireRecorded = false;              // FIRE recorded this power-on
  int16_t fireDeg = 0;                    // the FIRE angle strokes use now, whole degrees
  bool prefireRecorded = false;           // PRE-FIRE recorded this power-on
  int16_t prefireDeg = 0;                 // the PRE-FIRE angle strokes use now, whole degrees
  bool clearRecorded = false;             // CLEAR recorded this power-on
  int16_t clearDeg = 0;                   // the CLEAR angle strokes use now, whole degrees
};

// One LED colour as the LED hardware receives it (after Leds' brightness clamp).
struct Rgb {
  uint8_t r = 0;
  uint8_t g = 0;
  uint8_t b = 0;
};

// Step generator statistics for one window, from hal::stepTakeStats() (which resets them).
// Jitter: in each step interrupt the HAL compares the time since the previous interrupt with
// the timer period in force for that cycle; |actual - expected| / expected, in permille.
struct StepStats {
  uint32_t windowUs = 0;                      // time since the previous take
  uint32_t steps[AXIS_COUNT] = {};            // STEP pulses emitted
  uint32_t refusals[AXIS_COUNT] = {};         // pulses withheld at a bound (stepSetBounds)
  uint32_t samples[AXIS_COUNT] = {};          // intervals measured for jitter
  uint16_t jitterMaxPermille[AXIS_COUNT] = {};
  uint16_t jitterMeanPermille[AXIS_COUNT] = {};
};

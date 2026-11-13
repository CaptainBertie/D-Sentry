// Telemetry.cpp -- see Telemetry.h.
//
// SPDX-License-Identifier: MIT
#include "Telemetry.h"

#include <math.h>
#include <string.h>
#include "ControlLink.h"
#include "Debug.h"
#include "Flywheel.h"
#include "Inputs.h"
#include "Lcd.h"
#include "Leds.h"
#include "Motion.h"
#include "StallDetector.h"
#include "Trigger.h"
#include "TurretState.h"
#include "hal.h"

static_assert(TELEMETRY_USB_HZ > 0 && TELEMETRY_USB_HZ <= MS_PER_S,
              "TelemetryPacket period on USB is a whole number of ms");

namespace {

const uint32_t kUsbPeriodMs = MS_PER_S / TELEMETRY_USB_HZ;

uint16_t saturate16(uint32_t v) { return v > UINT16_MAX ? (uint16_t)UINT16_MAX : (uint16_t)v; }
uint16_t wrap16(uint32_t v) { return (uint16_t)v; }   // the low 16 bits of a wrapping counter

// Rounded half away from zero (roundf), saturated to [lo, hi], NaN -> 0. The limits are
// compared as floats, where (float)INT32_MAX is 2^31: anything at or above it saturates.
int32_t roundSaturate(float v, int32_t lo, int32_t hi) {
  if (v != v) return 0;
  if (v >= (float)hi) return hi;
  if (v <= (float)lo) return lo;
  const float r = roundf(v);
  if (r >= (float)hi) return hi;
  if (r <= (float)lo) return lo;
  return (int32_t)r;
}

}  // namespace

TelemetrySnapshot::TelemetrySnapshot() { memset(lcd, ' ', sizeof lcd); }

int32_t Telemetry::centi32(float deg) {
  return roundSaturate(deg * (float)DS_ANGLE_SCALE, INT32_MIN, INT32_MAX);
}

int16_t Telemetry::centi16(float deg) {
  return (int16_t)roundSaturate(deg * (float)DS_ANGLE_SCALE, INT16_MIN, INT16_MAX);
}

void Telemetry::build(const TelemetrySnapshot& s, uint8_t seq, ds_telemetry_packet_t& out) {
  ds_telemetry_init(&out);
  out.version = DS_TELEMETRY_VERSION;
  out.seq = seq;
  out.uptimeMs = s.uptimeMs;
  out.state = s.state;
  out.fault = s.fault;
  out.fireMode = s.fireMode;
  out.linkSource = s.linkSource;

  uint16_t f = 0;
  if (s.linkConnected) f |= DS_TELEM_FLAG_LINK_CONNECTED;
  if (s.flywheelOn) f |= DS_TELEM_FLAG_FLYWHEEL_ON;
  if (s.flywheelReady) f |= DS_TELEM_FLAG_FLYWHEEL_READY;
  if (s.strokeActive) f |= DS_TELEM_FLAG_STROKE_ACTIVE;
  if (s.triggerPulled) f |= DS_TELEM_FLAG_TRIGGER_PULLED;
  if (s.homing) f |= DS_TELEM_FLAG_HOMING;
  if (s.calibrated) f |= DS_TELEM_FLAG_CALIBRATED;
  if (s.powerToggleOn) f |= DS_TELEM_FLAG_POWER_TOGGLE_ON;
  if (s.modeToggleWifi) f |= DS_TELEM_FLAG_MODE_TOGGLE_WIFI;
  if (s.stallEnabled) f |= DS_TELEM_FLAG_STALL_ENABLED;
  if (s.emptyMagBlink) f |= DS_TELEM_FLAG_EMPTY_MAG_BLINK;
  if (s.driversEnabled) f |= DS_TELEM_FLAG_DRIVERS_ENABLED;
  if (s.espLegacyFirmware) f |= DS_TELEM_FLAG_ESP_LEGACY_FIRMWARE;
  if (s.lcdPresent) f |= DS_TELEM_FLAG_LCD_PRESENT;
  if (s.espAlive) f |= DS_TELEM_FLAG_ESP_ALIVE;
  out.flags = f;

  out.panCdeg = centi32(s.panDeg);
  out.tiltCdeg = centi32(s.tiltDeg);
  out.panVelCdegS = centi16(s.panVelDegS);
  out.tiltVelCdegS = centi16(s.tiltVelDegS);
  out.homePanCdeg = centi32(s.homePanDeg);
  out.homeTiltCdeg = centi32(s.homeTiltDeg);
  out.ammo = s.ammo;
  out.magCapacity = s.magCapacity;
  out.shotsTotal = s.shotsTotal;
  out.flywheelDuty = s.flywheelDuty;
  out.servoDeg = (uint8_t)roundSaturate(s.servoDeg, 0, UINT8_MAX);
  out.stallSample = s.stallSample;
  out.stallBaseline = s.stallBaseline;
  out.stallPeak = s.stallPeak;
  out.loopHz = saturate16(s.loopHz);
  out.ctrlRxHz = saturate16(s.ctrlRxHz);
  out.crcErrors = wrap16(s.crcErrors);
  out.seqGaps = wrap16(s.seqGaps);
  out.stepJitterPermille = saturate16(s.stepJitterPermille);
  out.lastCtrlSeq = s.lastCtrlSeq;
  out.espMode = s.espMode;
  out.ledR = s.led0.r;
  out.ledG = s.led0.g;
  out.ledB = s.led0.b;
  for (size_t i = 0; i < DS_TELEMETRY_LCD_LEN; i++) out.lcd[i] = s.lcd[i] ? s.lcd[i] : ' ';
  out.padState = s.padState;
  out.padBattery = s.padBattery;
  out.padLx = s.padLx;
  out.padLy = s.padLy;
  out.padRx = s.padRx;
  out.padRy = s.padRy;
  out.padLt = s.padLt;
  out.padRt = s.padRt;
  out.padButtons = s.padButtons;
}

void Telemetry::padBlock(const ControlFrame& f, uint8_t activeSource, TelemetrySnapshot& out) {
  out.padState = DS_PAD_STATE_NONE;
  out.padBattery = DS_PAD_BATTERY_UNKNOWN;
  out.padLx = out.padLy = out.padRx = out.padRy = 0;
  out.padLt = out.padRt = 0;
  out.padButtons = 0;
  if (activeSource == DS_LINK_SRC_NONE || !f.valid) return;
  out.padState = f.padState;
  out.padBattery = f.battery;
  if (!f.connected) return;
  out.padLx = f.lx;
  out.padLy = f.ly;
  out.padRx = f.rx;
  out.padRy = f.ry;
  out.padLt = f.lt;
  out.padRt = f.rt;
  out.padButtons = f.buttons;
}

size_t Telemetry::encode(const TelemetrySnapshot& s, uint8_t seq,
                         uint8_t frame[DS_TELEMETRY_FRAME_LEN]) {
  ds_telemetry_packet_t p;
  build(s, seq, p);
  return ds_telemetry_encode(&p, frame);
}

void Telemetry::collect(uint32_t nowMs, TelemetrySnapshot& out) const {
  out = TelemetrySnapshot();
  out.uptimeMs = nowMs;
  out.magCapacity = MAG_CAPACITY;
  if (src_.turret) {
    out.state = src_.turret->state();
    out.fault = src_.turret->fault();
    out.emptyMagBlink = src_.turret->display().emptyMagBlink;   // what the LEDs show
  }
  if (src_.trigger) {
    out.fireMode = src_.trigger->fireMode();
    out.strokeActive = src_.trigger->strokeActive();
    out.ammo = src_.trigger->ammo();
    out.magCapacity = src_.trigger->capacity();
    out.shotsTotal = src_.trigger->shotsTotal();
    out.servoDeg = src_.trigger->servoDeg();
  }
  if (src_.link) {
    const LinkStatus& st = src_.link->status();
    const LinkStats& ls = src_.link->stats();
    out.linkSource = st.activeSource;
    out.linkConnected = src_.link->connected();
    out.espAlive = st.espAlive;
    out.espLegacyFirmware = st.espLegacy;
    out.espMode = st.espMode;
    out.ctrlRxHz = ls.ctrlRxHz;
    out.crcErrors = ls.crcErrors;
    out.seqGaps = ls.seqGaps;
    out.lastCtrlSeq = src_.link->frame().seq;
    padBlock(src_.link->frame(), st.activeSource, out);
  }
  if (src_.inputs) {
    out.triggerPulled = src_.inputs->triggerPulled();
    out.powerToggleOn = src_.inputs->powerOn();
    out.modeToggleWifi = src_.inputs->radioMode() == DS_RADIO_WIFI;
  }
  if (src_.flywheel) {
    out.flywheelOn = src_.flywheel->on();
    out.flywheelReady = src_.flywheel->ready(nowMs);
    out.flywheelDuty = src_.flywheel->duty();
  }
  if (src_.motion) {
    out.homing = src_.motion->homing();
    out.calibrated = src_.motion->calibrated();
    out.driversEnabled = src_.motion->driversEnabled();
    out.panDeg = src_.motion->positionDeg(AXIS_PAN);
    out.tiltDeg = src_.motion->positionDeg(AXIS_TILT);
    out.panVelDegS = src_.motion->velocityDeg(AXIS_PAN);
    out.tiltVelDegS = src_.motion->velocityDeg(AXIS_TILT);
    out.homePanDeg = src_.motion->homeDeg(AXIS_PAN);
    out.homeTiltDeg = src_.motion->homeDeg(AXIS_TILT);
  }
  if (src_.stall) {
    out.stallEnabled = src_.stall->active();
    out.stallSample = src_.stall->sample();
    out.stallBaseline = src_.stall->baseline();
    out.stallPeak = src_.stall->peakDelta();
  }
  if (src_.debug) {
    out.loopHz = src_.debug->stats().loopHz;
    out.stepJitterPermille = src_.debug->stats().jitterMaxPermille;
  }
  if (src_.leds) out.led0 = src_.leds->pixel0();
  if (src_.lcd) {
    src_.lcd->mirror(out.lcd);
    out.lcdPresent = src_.lcd->present();
  }
}

void Telemetry::begin(const TelemetrySources& sources, ControlLink& link) { begin(sources, &link); }

void Telemetry::begin(const TelemetrySources& sources, ControlLink* link) {
  src_ = sources;
  link_ = link;
  seq_ = 0;
  nextUsbMs_ = hal::nowMs() + kUsbPeriodMs;
  built_ = usbSent_ = usbDropped_ = offered_ = accepted_ = 0;
}

void Telemetry::update(uint32_t nowMs) {
  if ((int32_t)(nowMs - nextUsbMs_) < 0) return;
  nextUsbMs_ += kUsbPeriodMs;
  // After a stall longer than a period, restart the schedule instead of bursting frames.
  if ((int32_t)(nowMs - nextUsbMs_) >= 0) nextUsbMs_ = nowMs + kUsbPeriodMs;

  TelemetrySnapshot s;
  collect(nowMs, s);
  uint8_t frame[DS_TELEMETRY_FRAME_LEN];
  const size_t n = encode(s, seq_, frame);
  seq_++;
  built_++;
  if (hal::usbWrite(frame, n)) {
    usbSent_++;
  } else {
    usbDropped_++;   // the whole frame or nothing: a PC never sees half a packet
  }
  if (link_) {
    offered_++;
    if (link_->offerTelemetry(frame, n)) accepted_++;
  }
}

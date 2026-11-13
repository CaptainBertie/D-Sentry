/*
 * dsentry_at.h -- the bodies of the D-Sentry AT commands (AT+DSCAPS?, AT+DSS=..., AT+DST=<hex>)
 * and of the AT+SOFTRESETWIFI guard, written against the real chAT server types.
 *
 * cmds_dsentry.h registers these methods in CAtHandler's command table; the host tests in
 * D-SentryCode/tests/esp32 run the very same methods on the real chAT parser.cpp/server.cpp
 * with a fake backend, so the bytes on the wire are proven, not copied. Everything that touches
 * a radio goes through DsentryBackend (dsentry_radio.cpp on the ESP32).
 *
 * Wire format: D-SentryCode/protocol/protocol_spec.py (AT_COMMANDS) and docs/protocol.md.
 * Every known command answers "+NAME: <payload>\r\nOK\r\n" in its documented mode and puts
 * failures in the payload; another mode (AT+DSS?, AT+DSCAPS, ...) gets the bare ERROR every
 * other fork handler gives (cmds_xbox.h).
 *
 * Threading: all methods run in the AT task (Task1, core 0) only.
 *
 * SPDX-License-Identifier: MIT
 * Copyright (c) 2026 the D-Sentry project
 * The combined firmware is governed by the AGPL-3.0 (see DECISIONS.md D04).
 */
#pragma once
#include <stddef.h>
#include <stdint.h>

#include <functional>

#include "chAT.hpp"
#include "dsentry_wire.h"

/* What the AT handlers need from the radios. dsentry_radio.cpp implements it on the ESP32;
 * the host tests implement it with a recording fake. */
class DsentryBackend {
 public:
  virtual dsentry::Radios radios() = 0;           /* restartPending is filled by DsentryAt */
  virtual uint8_t bootId() = 0;
  virtual uint8_t wifiStatus() = 0;               /* DS_WIFI_STATUS_* */
  virtual bool dsModeActive() = 0;                /* a D-Sentry radio mode was started or adopted */
  virtual bool startBle() = 0;                    /* false: the BLE client did not start */
  virtual bool startWifi() = 0;                   /* false: the WiFi task did not start */
  virtual void adoptMode(uint8_t espMode) = 0;    /* the wanted radio was already running */
  virtual void storeRestartRequest(uint8_t espMode) = 0;   /* RTC_NOINIT request for the next boot */
  virtual void rumbleBle(const uint8_t report[XBOX_RUMBLE_REPORT_LEN]) = 0;
  virtual void statusToRelay(const uint8_t* frame, size_t len) = 0;
  virtual bool telemetryToRelay(const uint8_t* frame, size_t len) = 0;  /* false: not forwarded */
  virtual void noteTurretState(uint8_t state) = 0;

 protected:
  ~DsentryBackend() = default;
};

class DsentryAt {
 public:
  explicit DsentryAt(DsentryBackend& backend) : be_(backend) {}

  /* AT+DSCAPS? -> "+DSCAPS: <DS_PROTO_VERSION>,<capBits>" */
  SudoMaker::chAT::CommandStatus caps(SudoMaker::chAT::Server& srv, SudoMaker::chAT::ATParser& parser);

  /* AT+DSS=<wantMode>,<rumbleSeq>,<strong>,<weak>,<rumbleMs>,<state> -> "+DSS: <espMode>,<bootId>,<wifiStatus>".
   * Starts the wanted radio in place, or stores it and calls scheduleRestart() once (the caller
   * queues the restart to run after the reply has been written; espMode is then RESTARTING).
   * A new rumbleSeq rumbles the pad (BLE) or sends a StatusPacket to the relay (WiFi).
   * Malformed arguments change nothing; the reply then reports the current state. */
  SudoMaker::chAT::CommandStatus dss(SudoMaker::chAT::Server& srv, SudoMaker::chAT::ATParser& parser,
                                     const std::function<void()>& scheduleRestart);

  /* AT+DST=<hex> -> "+DST: <1 forwarded | 0 not>" */
  SudoMaker::chAT::CommandStatus dst(SudoMaker::chAT::Server& srv, SudoMaker::chAT::ATParser& parser);

  /* AT+SOFTRESETWIFI (Run): while a D-Sentry radio mode is active, write the stock reply
   * ("+SOFTRESETWIFI: \r\n", the handler then returns OK) and do nothing else; returns true.
   * Otherwise returns false and writes nothing, and the stock handler runs unchanged. */
  bool softResetWifi(SudoMaker::chAT::Server& srv);

  bool restartPending() const { return restartPending_; }
  uint32_t malformedDss() const { return malformedDss_; }
  uint32_t malformedDst() const { return malformedDst_; }

 private:
  dsentry::Radios radiosNow();

  DsentryBackend& be_;
  dsentry::RumbleSeqTracker rumble_;
  bool restartPending_ = false;
  uint32_t malformedDss_ = 0;
  uint32_t malformedDst_ = 0;
};

/* The one instance the firmware uses (dsentry_radio.cpp). */
DsentryAt& dsentry_at();

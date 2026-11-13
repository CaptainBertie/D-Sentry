/*
 * dsentry_at.cpp -- see dsentry_at.h. Portable C++17 against chAT.hpp: compiled into the
 * ESP32-S3 sketch and, unchanged, into the host tests in D-SentryCode/tests/esp32.
 *
 * SPDX-License-Identifier: MIT
 * Copyright (c) 2026 the D-Sentry project
 * The combined firmware is governed by the AGPL-3.0 (see DECISIONS.md D04).
 */
#include "dsentry_at.h"

using SudoMaker::chAT::ATParser;
using SudoMaker::chAT::CommandMode;
using SudoMaker::chAT::CommandStatus;
using SudoMaker::chAT::Server;

dsentry::Radios DsentryAt::radiosNow() {
  dsentry::Radios r = be_.radios();
  r.restartPending = restartPending_;
  return r;
}

CommandStatus DsentryAt::caps(Server& srv, ATParser& parser) {
  if (parser.cmd_mode != CommandMode::Read) return CommandStatus::ERROR;
  srv.write_response_prompt();
  srv.write_str(dsentry::format_caps_reply());
  srv.write_line_end();
  return CommandStatus::OK;
}

CommandStatus DsentryAt::dss(Server& srv, ATParser& parser, const std::function<void()>& scheduleRestart) {
  if (parser.cmd_mode != CommandMode::Write) return CommandStatus::ERROR;
  dsentry::DssArgs a;
  if (dsentry::parse_dss(parser.args, &a)) {
    be_.noteTurretState(a.state);
    const uint8_t wantEsp = a.wantMode == DS_RADIO_WIFI ? (uint8_t)DS_ESP_MODE_WIFI : (uint8_t)DS_ESP_MODE_BLE;
    switch (dsentry::decide_mode(a.wantMode, radiosNow())) {
      case dsentry::ModeAction::ALREADY:
        be_.adoptMode(wantEsp);
        break;
      case dsentry::ModeAction::START_BLE:
        be_.startBle();     /* on failure espMode stays NONE; the RA4M1 retries at its heartbeat */
        break;
      case dsentry::ModeAction::START_WIFI:
        be_.startWifi();
        break;
      case dsentry::ModeAction::RESTART:
        be_.storeRestartRequest(wantEsp);   /* the latest wanted mode wins */
        if (!restartPending_) {
          restartPending_ = true;
          scheduleRestart();
        }
        break;
      case dsentry::ModeAction::INVALID:    /* parse_dss() already rejects it */
      default:
        break;
    }
    if (!restartPending_ && rumble_.is_new(a.rumbleSeq)) {
      const uint8_t mode = dsentry::esp_mode_of(radiosNow());
      if (mode == DS_ESP_MODE_BLE) {
        uint8_t report[XBOX_RUMBLE_REPORT_LEN];
        dsentry::rumble_report(a.strong, a.weak, a.rumbleMs, report);
        be_.rumbleBle(report);
      } else if (mode == DS_ESP_MODE_WIFI) {
        uint8_t frame[DS_STATUS_FRAME_LEN];
        size_t n = dsentry::build_status_frame(a, frame);
        be_.statusToRelay(frame, n);
      }
    }
  } else {
    malformedDss_++;
  }
  srv.write_response_prompt();
  srv.write_str(dsentry::format_dss_reply(dsentry::esp_mode_of(radiosNow()), be_.bootId(), be_.wifiStatus()));
  srv.write_line_end();
  return CommandStatus::OK;
}

CommandStatus DsentryAt::dst(Server& srv, ATParser& parser) {
  if (parser.cmd_mode != CommandMode::Write) return CommandStatus::ERROR;
  uint8_t sent = DS_DST_NOT_SENT;
  uint8_t frame[DS_TELEMETRY_FRAME_LEN];
  size_t n = parser.args.size() == DS_AT_DST_ARGC ? dsentry::parse_dst(parser.args[DS_AT_DST_ARG_HEX], frame) : 0;
  if (n == 0) {
    malformedDst_++;
  } else if (be_.telemetryToRelay(frame, n)) {
    sent = DS_DST_SENT;
  }
  srv.write_response_prompt();
  srv.write_str(dsentry::format_dst_reply(sent));
  srv.write_line_end();
  return CommandStatus::OK;
}

bool DsentryAt::softResetWifi(Server& srv) {
  if (!be_.dsModeActive()) return false;
  srv.write_response_prompt();
  srv.write_line_end();
  return true;
}

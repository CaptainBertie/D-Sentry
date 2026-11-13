/*
 * cmds_dsentry.h -- registers the D-Sentry AT commands (AT+DSCAPS?, AT+DSS=..., AT+DST=<hex>)
 * from CAtHandler::CAtHandler(), like every other cmds_*.h file. The handler bodies are
 * DsentryAt's methods (dsentry_at.cpp), which the host tests in D-SentryCode/tests/esp32 run on
 * the real chAT server; this file only connects them to the command table and to addTask().
 * Wire format: D-SentryCode/protocol/protocol_spec.py and D-SentryCode/docs/protocol.md.
 *
 * SPDX-License-Identifier: MIT
 * Copyright (c) 2026 the D-Sentry project
 * The combined firmware is governed by the AGPL-3.0 (see DECISIONS.md D04).
 */
#ifndef CMDS_DSENTRY_H
#define CMDS_DSENTRY_H

#include "at_handler.h"
#include "dsentry_at.h"
#include "dsentry_radio.h"

void CAtHandler::add_cmds_dsentry() {

   /* AT+DSCAPS? -> "+DSCAPS: <dsProto>,<capBits>" */
   command_table[DS_AT_DSCAPS] = [](auto & srv, auto & parser) {
      return dsentry_at().caps(srv, parser);
   };

   /* AT+DSS=<wantMode>,<rumbleSeq>,<strong>,<weak>,<rumbleMs>,<state> -> "+DSS: <espMode>,<bootId>,<wifiStatus>".
    * A mode switch queues the restart as a task: CAtHandler::run() executes tasks after
    * at_srv.run() has written the reply, the flush waits until the UART has sent it, and
    * dsentry_restart_now() then waits ESP_RESTART_DELAY_MS before esp_restart(). */
   command_table[DS_AT_DSS] = [this](auto & srv, auto & parser) {
      return dsentry_at().dss(srv, parser, [this]() {
         addTask([this]() {
            serial->flush();
            dsentry_restart_now();
         });
      });
   };

   /* AT+DST=<hex> -> "+DST: <0|1>" */
   command_table[DS_AT_DST] = [](auto & srv, auto & parser) {
      return dsentry_at().dst(srv, parser);
   };
}

#endif

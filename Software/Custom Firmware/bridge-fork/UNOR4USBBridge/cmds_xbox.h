/*
 * cmds_xbox.h -- AT commands for the Xbox controller client. Registered from
 * CAtHandler::CAtHandler() like every other cmds_*.h file. Wire format: docs/protocol.md.
 *
 * Every handler answers with its prompt and OK and puts failures in the payload, because the
 * RA4M1's WiFiS3 Modem parser only notices "ERROR" after its full timeout (Modem.cpp:7).
 * The binary reply copies the +PREFGET PT_BLOB pattern (cmds_preferences.h): "<len>|<bytes>".
 *
 * SPDX-License-Identifier: MIT
 * Copyright (c) 2026 the UNO R4 WiFi Xbox controller project
 */
#ifndef CMDS_XBOX_H
#define CMDS_XBOX_H

#include "at_handler.h"
#include "xbox_client.h"

void CAtHandler::add_cmds_xbox() {

   /* ....................................................................... */
   command_table[XBOX_R4_AT_VERSION] = [this](auto & srv, auto & parser) {
   /* ....................................................................... */
      switch (parser.cmd_mode) {
         case chAT::CommandMode::Read: {
            srv.write_response_prompt();
            srv.write_str(String(XBOX_R4_PROTOCOL_VERSION).c_str());
            srv.write_str(",");
            srv.write_str(ESP_FW_VERSION);
            srv.write_line_end();
            return chAT::CommandStatus::OK;
         }
         default:
            return chAT::CommandStatus::ERROR;
      }
   };

   /* ....................................................................... */
   command_table[XBOX_R4_AT_BEGIN] = [this](auto & srv, auto & parser) {
   /* ....................................................................... */
      switch (parser.cmd_mode) {
         case chAT::CommandMode::Run: {
            int rc = xbox_begin();
            srv.write_response_prompt();
            srv.write_str(String(rc).c_str());
            srv.write_line_end();
            return chAT::CommandStatus::OK;
         }
         default:
            return chAT::CommandStatus::ERROR;
      }
   };

   /* ....................................................................... */
   command_table[XBOX_R4_AT_STATE] = [this](auto & srv, auto & parser) {
   /* ....................................................................... */
      switch (parser.cmd_mode) {
         case chAT::CommandMode::Read: {
            std::vector<uint8_t> state(XBOX_R4_STATE_LEN);
            xbox_fill_state(state.data());
            srv.write_response_prompt();
            srv.write_str(String((int)XBOX_R4_STATE_LEN).c_str());
            srv.write_str("|");
            srv.write_vec8(state);
            srv.write_line_end();
            return chAT::CommandStatus::OK;
         }
         default:
            return chAT::CommandStatus::ERROR;
      }
   };

   /* ....................................................................... */
   command_table[XBOX_R4_AT_ADDR] = [this](auto & srv, auto & parser) {
   /* ....................................................................... */
      switch (parser.cmd_mode) {
         case chAT::CommandMode::Read: {
            char addr[18];
            xbox_peer_address(addr);
            srv.write_response_prompt();
            srv.write_str(addr);
            srv.write_line_end();
            return chAT::CommandStatus::OK;
         }
         default:
            return chAT::CommandStatus::ERROR;
      }
   };

   /* ....................................................................... */
   command_table[XBOX_R4_AT_FORGET] = [this](auto & srv, auto & parser) {
   /* ....................................................................... */
      switch (parser.cmd_mode) {
         case chAT::CommandMode::Run: {
            int rc = xbox_forget();
            srv.write_response_prompt();
            srv.write_str(String(rc).c_str());
            srv.write_line_end();
            return chAT::CommandStatus::OK;
         }
         default:
            return chAT::CommandStatus::ERROR;
      }
   };

   /* ....................................................................... */
   command_table[XBOX_R4_AT_SCAN] = [this](auto & srv, auto & parser) {
   /* ....................................................................... */
      switch (parser.cmd_mode) {
         case chAT::CommandMode::Read: {
            char text[800];
            size_t n = xbox_scan_report(text, sizeof(text));
            std::vector<uint8_t> bytes(text, text + n);
            srv.write_response_prompt();
            srv.write_str(String((int)n).c_str());
            srv.write_str("|");
            srv.write_vec8(bytes);
            srv.write_line_end();
            return chAT::CommandStatus::OK;
         }
         default:
            return chAT::CommandStatus::ERROR;
      }
   };

   /* ....................................................................... */
   command_table[XBOX_R4_AT_LOG] = [this](auto & srv, auto & parser) {
   /* ....................................................................... */
      switch (parser.cmd_mode) {
         case chAT::CommandMode::Write: {
            if (parser.args.size() != 1 || parser.args[0].empty()) {
               return chAT::CommandStatus::ERROR;
            }
            bool on = atoi(parser.args[0].c_str()) != 0;
            xbox_set_log(on);
            srv.write_response_prompt();
            srv.write_str(on ? "1" : "0");
            srv.write_line_end();
            return chAT::CommandStatus::OK;
         }
         case chAT::CommandMode::Read: {
            srv.write_response_prompt();
            srv.write_str(xbox_log_enabled() ? "1" : "0");
            srv.write_line_end();
            return chAT::CommandStatus::OK;
         }
         default:
            return chAT::CommandStatus::ERROR;
      }
   };
}

#endif

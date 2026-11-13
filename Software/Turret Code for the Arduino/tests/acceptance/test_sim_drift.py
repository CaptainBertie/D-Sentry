# SPDX-License-Identifier: MIT
"""Drift guards for sim/sim_esp32.cpp. Most of the simulated ESP32 is the fork's own code
(chAT, dsentry_at.cpp, dsentry_wire.cpp, compiled in). The parts that cannot run on a Mac are
mirrored, and each mirror is only as good as its match with the firmware: these tests read the
firmware files and fail when the code the sim copies changes shape, so the sim is revisited
instead of silently drifting. Whitespace is ignored; nothing is compiled."""

import re
import unittest

import _sim as S


def squash(text: str) -> str:
    return re.sub(r"\s+", " ", text)


def read(name: str) -> str:
    return squash((S.FORK / name).read_text(encoding="utf-8", errors="replace"))


def block(text: str, start: str, end: str) -> str:
    i = text.index(start)
    return text[i:text.index(end, i + len(start))]


class FirmwareShapeTests(unittest.TestCase):
    def assertHas(self, text: str, snippet: str, where: str) -> None:
        self.assertIn(squash(snippet), text, "%s changed: the sim mirrors %r (update sim/sim_esp32.cpp)" % (where, snippet))

    def test_xboxstate_and_xboxbegin_reply_bytes(self):
        # sim_esp32.cpp addCommands(): the cmds_xbox.h bodies, std::to_string for String().
        t = read("cmds_xbox.h")
        state = block(t, "command_table[XBOX_R4_AT_STATE]", "command_table[XBOX_R4_AT_ADDR]")
        for s in ("case chAT::CommandMode::Read: {", "std::vector<uint8_t> state(XBOX_R4_STATE_LEN);",
                  "xbox_fill_state(state.data());", "srv.write_response_prompt();",
                  'srv.write_str(String((int)XBOX_R4_STATE_LEN).c_str());', 'srv.write_str("|");',
                  "srv.write_vec8(state);", "srv.write_line_end();", "return chAT::CommandStatus::OK;",
                  "default: return chAT::CommandStatus::ERROR;"):
            self.assertHas(state, s, "cmds_xbox.h +XBOXSTATE")
        begin = block(t, "command_table[XBOX_R4_AT_BEGIN]", "command_table[XBOX_R4_AT_STATE]")
        for s in ("case chAT::CommandMode::Run: {", "int rc = xbox_begin();", "srv.write_response_prompt();",
                  "srv.write_str(String(rc).c_str());", "srv.write_line_end();"):
            self.assertHas(begin, s, "cmds_xbox.h +XBOXBEGIN")

    def test_dsentry_registration_and_restart_task(self):
        # sim_esp32.cpp addCommands() and run(): the DSS restart is a task (flush, then restart).
        t = read("cmds_dsentry.h")
        for s in ("command_table[DS_AT_DSCAPS] = [](auto & srv, auto & parser) { return dsentry_at().caps(srv, parser); };",
                  "return dsentry_at().dss(srv, parser, [this]() { addTask([this]() { serial->flush(); dsentry_restart_now(); }); });",
                  "command_table[DS_AT_DST] = [](auto & srv, auto & parser) { return dsentry_at().dst(srv, parser); };"):
            self.assertHas(t, s, "cmds_dsentry.h")
        r = read("dsentry_radio.cpp")
        self.assertHas(r, "void dsentry_restart_now(void) { vTaskDelay(pdMS_TO_TICKS(ESP_RESTART_DELAY_MS)); esp_restart(); }",
                       "dsentry_radio.cpp dsentry_restart_now()")

    def test_at_task_loop_and_dispatch(self):
        # sim_esp32.cpp atTask() / startServer(): CAtHandler::run() and its command callback.
        t = read("at_handler.cpp")
        run = block(t, "void CAtHandler::run() {", "vTaskDelay(1);")
        self.assertHas(run, "at_srv.run();", "at_handler.cpp run()")
        self.assertHas(run, "auto size = tasks.size(); for(int i=0; i<size; i++) { auto task = tasks.front(); tasks.pop(); if(task) { task(); } }",
                       "at_handler.cpp run()")
        self.assertHas(t, "auto it = command_table.find(command); if (it == command_table.end()) { return chAT::CommandStatus::ERROR; }",
                       "at_handler.cpp command callback")
        g = read("cmds_esp_generic.h")
        self.assertHas(g, "if (dsentry_at().softResetWifi(srv)) { return chAT::CommandStatus::OK; }",
                       "cmds_esp_generic.h +SOFTRESETWIFI guard")

    def test_xbox_state_sources(self):
        # sim_esp32.cpp fillXboxState(): xbox_fill_state() + dsentry_fill_xbox_state().
        x = read("xbox_client.cpp")
        fill = block(x, "void xbox_fill_state(uint8_t out[XBOX_R4_STATE_LEN]) {", "xbox_r4_encode(&s, out);")
        self.assertTrue(fill.index("if (dsentry_fill_xbox_state(out)) return;") < fill.index("if (!g_active"),
                        "xbox_fill_state(): WiFi mode must answer first")
        self.assertHas(fill, "if (!g_active || g_core == nullptr) { xbox_r4_neutral_report(s.report); "
                             "s.state = g_hci_used ? XBOX_R4_BLOCKED_BY_HCI : XBOX_R4_OFF; "
                             "s.battery = XBOX_R4_BATTERY_UNKNOWN; s.seq = 0;", "xbox_client.cpp xbox_fill_state()")
        self.assertHas(x, "if (dsentry_wifi_mode_active()) return XBOX_R4_RC_BLOCKED_BY_HCI;", "xbox_client.cpp xbox_begin()")
        r = read("dsentry_radio.cpp")
        self.assertHas(r, "const bool live = dsentry::control_is_live(have, p, rx, millis());",
                       "dsentry_radio.cpp dsentry_fill_xbox_state()")
        self.assertHas(r, "dsentry::control_to_xstate(have ? &p : nullptr, live, out);",
                       "dsentry_radio.cpp dsentry_fill_xbox_state()")

    def test_radio_backend_and_wifi_task(self):
        # sim_esp32.cpp SimBackend, boot(), wifiTask().
        r = read("dsentry_radio.cpp")
        for s, where in (
                ("dsentry::BootPlan plan = dsentry::plan_boot(&s, esp_reset_reason() == ESP_RST_SW, esp_random());", "dsentry_boot()"),
                ("if (plan.startMode == DS_ESP_MODE_BLE) { g_backend.startBle(); } else if (plan.startMode == DS_ESP_MODE_WIFI) { g_backend.startWifi(); }", "dsentry_boot()"),
                ("if (!g_wifiStarted || !wifi_network_up() || len != DS_TELEMETRY_FRAME_LEN) return false;", "telemetryToRelay()"),
                ("ok = g_sh.haveRelay;", "telemetryToRelay()"),
                ("if (g_dsMode == DS_ESP_MODE_NONE) g_dsMode = espMode;", "adoptMode()"),
                ("dsentry::store_write(&s, espMode, g_bootId);", "storeRestartRequest()"),
                ("apply(sup.start(millis(), WIFI_SSID[0] != '\\0', WIFI_MODE_AP_FALLBACK));", "wifi_task()"),
                ("apply(sup.step(now, WiFi.status() == WL_CONNECTED));", "wifi_task()"),
                ("if (sock < 0 && sup.network_up()) sock = open_socket();", "wifi_task()"),
                ("receive_one(sock); send_outbox(sock);", "wifi_task()"),
                ("if (!dsentry::accept_control(buf, (size_t)n, &p)) return;", "receive_one()"),
                ("if (!haveRelay) return;", "send_outbox()"),
                ("dsentry::build_beacon_frame(octets, g_bootId, wifi_status_now(), frame);", "send_beacon()"),
                ("return dsentry::wifi_status(g_wifiStarted, (dsentry::WifiSupervisor::Phase)g_wifiPhase, heard);", "wifi_status_now()")):
            self.assertHas(r, s, "dsentry_radio.cpp " + where)

    def test_the_sim_compiles_the_real_handlers(self):
        # sim/build.sh links the fork's dsentry_at.cpp, dsentry_wire.cpp, parser.cpp and server.cpp
        # themselves, not copies.
        build = (S.DSC / "sim" / "build.sh").read_text()
        for f in ("parser.cpp", "server.cpp", "dsentry_at.cpp", "dsentry_wire.cpp"):
            self.assertIn('"$FORK/%s"' % f, build)
        self.assertFalse(any((S.DSC / "sim" / f).exists() for f in ("parser.cpp", "server.cpp", "dsentry_at.cpp")))


if __name__ == "__main__":
    unittest.main()

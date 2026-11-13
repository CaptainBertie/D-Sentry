// sim_esp32.h -- the ESP32-S3 inside dsentry_sim (sim_esp32.cpp).
//
// What runs is the bridge fork's own code wherever it can run on a Mac:
//   - the real chAT AT server (firmware/UNOR4USBBridge/parser.cpp, server.cpp), dispatching
//     exactly like CAtHandler (unknown command -> ERROR, tasks run after at_srv.run());
//   - the real D-Sentry handlers DsentryAt::caps / dss / dst / softResetWifi (dsentry_at.cpp)
//     and every decision and byte format of dsentry_wire.cpp (mode switch, restart request in
//     RTC memory, boot plan and bootId, the WiFi supervisor, accept_control, control_is_live,
//     control_to_xstate, the StatusPacket and BeaconPacket frames, the rumble report);
//   - AT+XBOXSTATE? / AT+XBOXBEGIN / AT+SOFTRESETWIFI with the reply bytes of cmds_xbox.h and
//     cmds_esp_generic.h, the state built by the same functions xbox_fill_state() and
//     dsentry_fill_xbox_state() use.
// Only the radios are stand-ins (a DsentryBackend like dsentry_radio.cpp's RadioBackend):
//   WiFi: the station "joins" after Options::wifiJoinMs; the UDP socket is a real one bound to
//         127.0.0.1:<udpPort>; ControlPackets in, StatusPackets and AT+DST telemetry out to the
//         source of the last valid ControlPacket; optional BeaconPacket to 127.0.0.1.
//   BLE:  no pad, or a fake pad set from the sim's stdin (padCommand).
// A mode switch restarts the ESP32 as esp_restart() would: the reply is flushed, the AT task
// waits ESP_RESTART_DELAY_MS, then the chip is silent for Options::bootMs (the USB passthrough
// too), prints the ESP32-S3 ROM banner on UART0 (the RA4M1's Serial RX), and boots radio-less
// except for the one-shot request kept in RTC memory, with a new bootId.
//
// This header names no config.h / dsentry_config.h value, so the RA4M1 half (sim_main.cpp,
// config.h) and this half (dsentry_config.h, the ESP32's copy of the same names) never meet.
//
// SPDX-License-Identifier: MIT
// The dsentry_sim binary links the fork's chAT server (AGPL-3.0), so the combined binary is
// governed by the AGPL-3.0, like the ESP32 firmware (DECISIONS.md D04). It is a local build
// artefact (sim/build, tests/build), never distributed.
#pragma once
#include <stdint.h>

#include <string>

namespace simesp {

struct Options {
  uint16_t udpPort = 0;      // 127.0.0.1:<udpPort>; 0 = WIFI_UDP_PORT
  bool beacon = false;       // send BeaconPackets in WiFi mode
  uint16_t beaconPort = 0;   // to 127.0.0.1:<beaconPort>; 0 = WIFI_DISCOVERY_PORT
  uint32_t bootMs = 0;       // power-on or esp_restart() until the AT server answers
  int32_t wifiJoinMs = 0;    // station join time; < 0: the station never joins (AP fallback)
  bool romBanner = true;     // the ROM banner on UART0 at every boot
  bool fakePad = false;      // BLE mode: a pad the stdin "pad" commands drive
  uint32_t seed = 0;         // esp_random() stand-in (bootId, RTC garbage at power-on)
};

// One line of the simulator's own log (no newline); the caller adds the time stamp.
typedef void (*LogFn)(const char* text);

void begin(const Options& opt, LogFn log);   // power-on at virtual time 0
void end();                                  // closes the socket

// The AT link (Serial2 <-> the ESP32's AT UART), both directions paced at LINK_BAUD.
void fromRa(uint64_t nowUs, const std::string& bytes);   // what the RA4M1 transmitted now
void run(uint64_t nowUs);     // the AT task (every 1 ms), the WiFi task, restart and boot
std::string toRa(uint64_t nowUs);                        // what reaches the RA4M1 by now

// UART0 output of the ESP32 itself (the ROM banner), for the RA4M1's Serial RX; once each.
std::string takeUart0(uint64_t nowUs);
// The USB passthrough runs (the ESP32 is booted and not restarting).
bool bridgeUp();

// stdin "pad ..." (needs Options::fakePad). Returns false with a reason for a bad command.
bool padCommand(const std::string& args, std::string* reply);

// key=value summary for the sim's "status" command.
std::string status();
uint16_t udpPort();

}  // namespace simesp

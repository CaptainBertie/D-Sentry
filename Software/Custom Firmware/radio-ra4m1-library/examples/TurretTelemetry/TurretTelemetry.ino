/*
 * TurretTelemetry -- streams the controller as one CSV line per report for the Nerf turret
 * dashboard (tools/dashboard.py). Serial at 115200 baud. The Serial Monitor shows the same text.
 *
 * Line format (one per new controller report, plus a heartbeat every 100 ms):
 *   XR4,<millis>,<state>,<battery>,<seq>,<lx>,<ly>,<rx>,<ry>,<lt>,<rt>,<buttons_hex4>,<dpad>
 *   millis   the board's millis() when the line was printed
 *   state    connectionState(): 0 connected, 1 waiting for first report, 2 found, 3 scanning,
 *            4 off, 5 blocked by ArduinoBLE
 *   battery  0..100 percent, or 255 unknown
 *   seq      0..255, low byte of the ESP32-S3's report counter
 *   lx..ry   raw sticks 0..65535 (joyLHori, joyLVert, joyRHori, joyRVert); centre ~32768,
 *            vertical 0 = up
 *   lt, rt   raw triggers 0..1023
 *   buttons  4 lowercase hex digits: bit0 A, bit1 B, bit2 X, bit3 Y, bit4 LB, bit5 RB,
 *            bit6 View, bit7 Menu, bit8 Xbox, bit9 LS click, bit10 RS click, bit11 Share
 *   dpad     0 none, 1 N, 2 NE, 3 E, 4 SE, 5 S, 6 SW, 7 W, 8 NW
 * Lines that start with '#' are for people (begin result, link lost, state changes).
 * Send the line "scan" to get "# scan: <the ESP32-S3's last Bluetooth scan>".
 *
 * No XR4 lines go out while the ESP32-S3 is not answering: the library would only repeat
 * stale values, and a silent board is what makes the dashboard drop to SAFE.
 *
 * Needs the forked bridge firmware on the ESP32-S3 (see the library README).
 *
 * SPDX-License-Identifier: MIT
 */
#include <XboxControllerR4.h>

XboxControllerR4 xbox;

const unsigned long POLL_INTERVAL_MS = 20;        // 50 Hz; each poll blocks the RA4M1 about 5-6 ms
const unsigned long SLOW_POLL_INTERVAL_MS = 500;  // while the ESP32-S3 is not answering
const unsigned long HEARTBEAT_MS = 100;           // well under the dashboard's 300 ms link timeout
const unsigned long RETRY_INTERVAL_MS = 2000;     // how often a failed begin() is tried again
const unsigned long LINK_LOST_MS = 200;           // no good poll for this long: ESP32-S3 silent
const size_t COMMAND_MAX = 16;                    // longest command line kept ("scan" + room)

bool ready = false;             // begin() succeeded
bool polledSinceBegin = false;  // the ESP32-S3 has answered a poll since the last begin()
int8_t bridgeState = -1;        // 1 answering, 0 silent, -1 not yet decided since begin()
unsigned long beginMs = 0;      // millis() of the last begin() attempt (also times the retry)
unsigned long lastPollMs = 0;
unsigned long lastLineMs = 0;
uint8_t lastState = 0xFF;
char command[COMMAND_MAX + 1];
size_t commandLen = 0;
char scanText[512];      // static: the RA4M1 stack is small and a scan report can be long

// 1 = the ESP32-S3 answered a poll in the last LINK_LOST_MS, 0 = it did not, -1 = too soon
// after begin() to tell. The library keeps its last values when the ESP32-S3 stops answering,
// so the sketch has to check this itself.
int8_t bridgeVerdict() {
  unsigned long now = millis();
  if (polledSinceBegin) {
    return now - xbox.lastPollOkMillis() <= LINK_LOST_MS ? 1 : 0;
  }
  return now - beginMs <= LINK_LOST_MS ? -1 : 0;
}

// An unanswered poll blocks for its whole 50 ms timeout, so poll slowly while the link is lost.
unsigned long pollInterval() {
  return bridgeState == 0 ? SLOW_POLL_INTERVAL_MS : POLL_INTERVAL_MS;
}

const char* errorWord(int error) {
  switch (error) {
    case XBOX_R4_ERR_NO_FIRMWARE_SUPPORT: return "stock bridge firmware";
    case XBOX_R4_ERR_PROTOCOL_MISMATCH:   return "protocol mismatch";
    case XBOX_R4_ERR_BEGIN_FAILED:        return "XBOXBEGIN failed";
    case XBOX_R4_ERR_BRIDGE_SILENT:       return "ESP32-S3 silent";
    default:                              return "unknown";
  }
}

const char* stateWord(uint8_t state) {
  switch (state) {
    case XBOX_R4_CONNECTED:            return "connected";
    case XBOX_R4_WAITING_FIRST_REPORT: return "waiting";
    case XBOX_R4_FOUND:                return "connecting";
    case XBOX_R4_SCANNING:             return "scanning";
    case XBOX_R4_OFF:                  return "off";
    case XBOX_R4_BLOCKED_BY_HCI:       return "blocked";
    default:                           return "unknown";
  }
}

void startClient() {
  ready = xbox.begin();
  beginMs = millis();      // a failed begin() is retried RETRY_INTERVAL_MS from now
  polledSinceBegin = false;
  bridgeState = -1;        // so the first verdict after every begin() is printed
  char line[96];
  if (ready) {
    snprintf(line, sizeof(line), "# begin ok: firmware %s protocol %d",
             xbox.firmwareVersion(), xbox.protocolVersion());
  } else if (xbox.lastError() == XBOX_R4_ERR_BEGIN_FAILED) {
    // result 1 = ArduinoBLE took the radio (power-cycle), 2 = Bluetooth init failed, -1 = no answer
    snprintf(line, sizeof(line), "# begin failed: lastError %d (%s, result %d)",
             xbox.lastError(), errorWord(xbox.lastError()), xbox.beginResult());
  } else {
    snprintf(line, sizeof(line), "# begin failed: lastError %d (%s)",
             xbox.lastError(), errorWord(xbox.lastError()));
  }
  Serial.println(line);
}

// bit layout documented at the top of the file
uint16_t buttonMask(const XboxControllerNotificationParser& n) {
  uint16_t mask = 0;
  if (n.btnA)      mask |= 1u << 0;
  if (n.btnB)      mask |= 1u << 1;
  if (n.btnX)      mask |= 1u << 2;
  if (n.btnY)      mask |= 1u << 3;
  if (n.btnLB)     mask |= 1u << 4;
  if (n.btnRB)     mask |= 1u << 5;
  if (n.btnSelect) mask |= 1u << 6;
  if (n.btnStart)  mask |= 1u << 7;
  if (n.btnXbox)   mask |= 1u << 8;
  if (n.btnLS)     mask |= 1u << 9;
  if (n.btnRS)     mask |= 1u << 10;
  if (n.btnShare)  mask |= 1u << 11;
  return mask;
}

// 0 none, 1 N, 2 NE, 3 E, 4 SE, 5 S, 6 SW, 7 W, 8 NW (diagonals first, they hold two bits)
uint8_t dpadCode(const XboxControllerNotificationParser& n) {
  if (n.btnDirUp && n.btnDirRight)   return 2;
  if (n.btnDirDown && n.btnDirRight) return 4;
  if (n.btnDirDown && n.btnDirLeft)  return 6;
  if (n.btnDirUp && n.btnDirLeft)    return 8;
  if (n.btnDirUp)    return 1;
  if (n.btnDirRight) return 3;
  if (n.btnDirDown)  return 5;
  if (n.btnDirLeft)  return 7;
  return 0;
}

void printTelemetry() {
  const XboxControllerNotificationParser& n = xbox.xboxNotif;
  char line[96];
  snprintf(line, sizeof(line), "XR4,%lu,%u,%u,%u,%u,%u,%u,%u,%u,%u,%04x,%u",
           millis(), (unsigned)xbox.connectionState(), (unsigned)xbox.battery(),
           (unsigned)xbox.seq(), (unsigned)n.joyLHori, (unsigned)n.joyLVert,
           (unsigned)n.joyRHori, (unsigned)n.joyRVert, (unsigned)n.trigLT,
           (unsigned)n.trigRT, (unsigned)buttonMask(n), (unsigned)dpadCode(n));
  Serial.println(line);
  lastLineMs = millis();
}

void runScan() {
  if (xbox.scanReport(scanText, sizeof(scanText))) {
    Serial.print("# scan: ");
    Serial.println(scanText);
  } else {
    Serial.println("# scan: no answer");
  }
}

// Collects characters into a line; "scan" is the only command. Anything else is ignored.
void handleSerialInput() {
  while (Serial.available() > 0) {
    char c = (char)Serial.read();
    if (c == '\n' || c == '\r') {
      command[commandLen] = '\0';
      if (strcmp(command, "scan") == 0) {
        runScan();
      }
      commandLen = 0;
    } else if (commandLen < COMMAND_MAX) {
      command[commandLen++] = c;
    }
  }
}

// Announces when the ESP32-S3 starts or stops answering; the XR4 stream follows this state.
// Only called once begin() has succeeded: a failed begin() prints its own reason.
void updateBridgeState() {
  int8_t verdict = bridgeVerdict();
  if (verdict < 0 || verdict == bridgeState) {
    return;
  }
  bridgeState = verdict;
  if (verdict == 1) {
    Serial.println("# link ok");
  } else {
    char line[64];
    snprintf(line, sizeof(line), "# link lost: no good poll for %lu ms", LINK_LOST_MS);
    Serial.println(line);
  }
}

void announceStateChange() {
  uint8_t state = xbox.connectionState();
  if (state == lastState) {
    return;
  }
  lastState = state;
  Serial.print("# state: ");
  Serial.println(stateWord(state));
  if (state == XBOX_R4_SCANNING) {
    Serial.println("# pairing: hold the controller's Pair button 3 s until the Xbox logo flashes fast");
  }
}

void setup() {
  Serial.begin(115200);
  delay(500);  // give the dashboard or Serial Monitor a moment to attach
  Serial.println("# TurretTelemetry: starting the Xbox client on the ESP32-S3");
  startClient();
}

void loop() {
  unsigned long now = millis();
  handleSerialInput();

  if (!ready) {
    if (now - beginMs >= RETRY_INTERVAL_MS) {
      startClient();
    }
    return;
  }

  bool newReport = false;
  if (now - lastPollMs >= pollInterval()) {
    lastPollMs = now;
    if (xbox.poll()) {
      polledSinceBegin = true;
      newReport = xbox.hasNewReport();  // read now: the next poll() overwrites it
      // A good poll that says OFF after a successful begin() means the ESP32-S3 restarted.
      if (xbox.connectionState() == XBOX_R4_OFF) {
        Serial.println("# ESP32-S3 restarted: sending begin again");
        startClient();
        return;
      }
    }
  }

  updateBridgeState();
  if (bridgeState != 1) {
    return;  // stale values never go out as telemetry
  }
  announceStateChange();
  if (newReport || millis() - lastLineMs >= HEARTBEAT_MS) {
    printTelemetry();
  }
}

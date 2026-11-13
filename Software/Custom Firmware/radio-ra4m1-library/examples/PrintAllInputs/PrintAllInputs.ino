/*
 * PrintAllInputs -- prints every input of an Xbox Series X|S controller paired to the
 * UNO R4 WiFi's ESP32-S3: one line ten times a second on the Serial Monitor (115200 baud).
 *
 * Needs the forked bridge firmware on the ESP32-S3 (see the library README). With the stock
 * firmware, begin() fails and this sketch prints why every two seconds.
 *
 * SPDX-License-Identifier: MIT
 */
#include <XboxControllerR4.h>

XboxControllerR4 xbox;

const unsigned long POLL_INTERVAL_MS = 20;        // 50 Hz; each poll blocks the RA4M1 about 5-6 ms
const unsigned long PRINT_INTERVAL_MS = 100;      // 10 lines a second stays readable
const unsigned long RETRY_INTERVAL_MS = 2000;     // how often a failed begin() is tried again
const unsigned long PAIR_HINT_INTERVAL_MS = 5000; // repeat the pairing hint while scanning
const unsigned long LINK_LOST_MS = 200;           // no good poll for this long: ESP32-S3 not answering
const unsigned long SLOW_POLL_INTERVAL_MS = 500;  // poll rate while the ESP32-S3 is not answering

bool ready = false;
unsigned long lastPollMs = 0;
unsigned long lastPrintMs = 0;
unsigned long lastRetryMs = 0;
unsigned long lastHintMs = 0;
uint8_t lastState = 0xFF;

// True when no poll() has succeeded for LINK_LOST_MS: the ESP32-S3 has stopped answering.
// The library then keeps the last controller values, so the sketch must notice this itself.
bool linkLost() {
  return millis() - xbox.lastPollOkMillis() > LINK_LOST_MS;
}

// An unanswered poll blocks for its whole 50 ms timeout, so poll less often while the link
// is lost; at 50 Hz the RA4M1 would otherwise be blocked almost all the time.
unsigned long pollInterval() {
  return linkLost() ? SLOW_POLL_INTERVAL_MS : POLL_INTERVAL_MS;
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

void printBeginFailure() {
  Serial.print("XboxControllerR4 begin() failed, lastError() = ");
  Serial.println(xbox.lastError());
  switch (xbox.lastError()) {
    case XBOX_R4_ERR_NO_FIRMWARE_SUPPORT:
      Serial.print("  The ESP32-S3 answers (bridge firmware ");
      Serial.print(xbox.firmwareVersion());
      Serial.println(") but does not know AT+XBOXVERSION?:");
      Serial.println("  it is most likely the stock firmware. Flash the forked firmware (library README).");
      break;
    case XBOX_R4_ERR_BRIDGE_SILENT:
      Serial.println("  The ESP32-S3 did not answer at all. Unplug and replug the board; if it was");
      Serial.println("  just flashed, it may still be in its bootloader.");
      break;
    case XBOX_R4_ERR_PROTOCOL_MISMATCH:
      Serial.print("  Firmware protocol ");
      Serial.print(xbox.protocolVersion());
      Serial.print(" (firmware ");
      Serial.print(xbox.firmwareVersion());
      Serial.print("), library expects ");
      Serial.print(XBOX_R4_PROTOCOL_VERSION);
      Serial.println(". Update the firmware or the library so they match.");
      break;
    case XBOX_R4_ERR_BEGIN_FAILED:
      Serial.print("  AT+XBOXBEGIN result ");
      Serial.print(xbox.beginResult());
      if (xbox.beginResult() == XBOX_R4_RC_BLOCKED_BY_HCI) {
        Serial.println(": ArduinoBLE owns the Bluetooth radio. Power-cycle the board.");
      } else if (xbox.beginResult() == XBOX_R4_RC_INIT_FAILED) {
        Serial.println(": the ESP32-S3 could not start Bluetooth.");
      } else {
        Serial.println(": no answer from the ESP32-S3.");
      }
      break;
    default:
      break;
  }
}

void printSigned(float v) {
  if (v >= 0.0f) {
    Serial.print('+');
  }
  Serial.print(v, 2);
}

// Prints label when the button is held; returns whether it was.
bool printIfPressed(bool pressed, const char* label) {
  if (pressed) {
    Serial.print(label);
    Serial.print(' ');
  }
  return pressed;
}

void printButtons() {
  XboxControllerNotificationParser& n = xbox.xboxNotif;
  bool any = false;
  any |= printIfPressed(n.btnA, "A");
  any |= printIfPressed(n.btnB, "B");
  any |= printIfPressed(n.btnX, "X");
  any |= printIfPressed(n.btnY, "Y");
  any |= printIfPressed(n.btnLB, "LB");
  any |= printIfPressed(n.btnRB, "RB");
  any |= printIfPressed(n.btnLS, "LS");
  any |= printIfPressed(n.btnRS, "RS");
  any |= printIfPressed(n.btnSelect, "View");
  any |= printIfPressed(n.btnStart, "Menu");
  any |= printIfPressed(n.btnXbox, "Xbox");
  any |= printIfPressed(n.btnShare, "Share");
  if (!any) {
    Serial.print("- ");
  }
}

const char* dpadWord() {
  XboxControllerNotificationParser& n = xbox.xboxNotif;
  if (n.btnDirUp && n.btnDirRight) return "up-right";
  if (n.btnDirUp && n.btnDirLeft) return "up-left";
  if (n.btnDirDown && n.btnDirRight) return "down-right";
  if (n.btnDirDown && n.btnDirLeft) return "down-left";
  if (n.btnDirUp) return "up";
  if (n.btnDirDown) return "down";
  if (n.btnDirLeft) return "left";
  if (n.btnDirRight) return "right";
  return "none";
}

void printStatusLine() {
  if (linkLost()) {
    Serial.print("[ESP32-S3 not answering, values are stale] ");
  }
  Serial.print(stateWord(xbox.connectionState()));
  Serial.print("  bat ");
  if (xbox.battery() == XBOX_R4_BATTERY_UNKNOWN) {
    Serial.print('?');
  } else {
    Serial.print(xbox.battery());
    Serial.print('%');
  }
  Serial.print("  seq ");
  Serial.print(xbox.seq());
  Serial.print("  L(");
  printSigned(xbox.leftX());
  Serial.print(',');
  printSigned(xbox.leftY());
  Serial.print(")  R(");
  printSigned(xbox.rightX());
  Serial.print(',');
  printSigned(xbox.rightY());
  Serial.print(")  LT ");
  Serial.print(xbox.leftTrigger(), 2);
  Serial.print("  RT ");
  Serial.print(xbox.rightTrigger(), 2);
  Serial.print("  buttons ");
  printButtons();
  Serial.print(" dpad ");
  Serial.println(dpadWord());
}

void startClient() {
  ready = xbox.begin();
  lastRetryMs = millis();  // a failed begin() is retried RETRY_INTERVAL_MS from now, whoever called it
  if (ready) {
    Serial.print("Xbox client running. ESP32-S3 firmware ");
    Serial.print(xbox.firmwareVersion());
    Serial.print(", protocol ");
    Serial.println(xbox.protocolVersion());
  } else {
    printBeginFailure();
  }
}

void setup() {
  Serial.begin(115200);
  delay(500);  // give the Serial Monitor a moment to attach
  Serial.println("PrintAllInputs: starting the Xbox client on the ESP32-S3");
  startClient();
}

void loop() {
  unsigned long now = millis();

  if (!ready) {
    if (now - lastRetryMs >= RETRY_INTERVAL_MS) {
      startClient();
    }
    return;
  }

  if (now - lastPollMs >= pollInterval()) {
    lastPollMs = now;
    // A good poll that says OFF after a successful begin() means the ESP32-S3 restarted
    // (its Xbox client is no longer running): start the client again.
    if (xbox.poll() && xbox.connectionState() == XBOX_R4_OFF) {
      Serial.println("The ESP32-S3 restarted; sending begin again");
      startClient();
    }
  }

  uint8_t state = xbox.connectionState();
  if (state == XBOX_R4_SCANNING &&
      (lastState != XBOX_R4_SCANNING || now - lastHintMs >= PAIR_HINT_INTERVAL_MS)) {
    lastHintMs = now;
    Serial.println("pairing: hold the controller's Pair button 3 s until the Xbox logo flashes fast");
  }
  lastState = state;

  if (now - lastPrintMs >= PRINT_INTERVAL_MS) {
    lastPrintMs = now;
    printStatusLine();
  }
}

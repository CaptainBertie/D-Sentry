/*
 * MatrixDot -- a lit dot on the board's built-in 12x8 LED matrix follows the left stick, and
 * the whole matrix lights while A is held. No wiring and no Serial Monitor needed: this is
 * the quick visual check that the controller, the ESP32-S3 firmware and the library all work.
 *
 * What the matrix shows:
 *   steady dot that moves with the stick  connected
 *   dot blinking in the centre            no controller yet (pair it, or press its Xbox button)
 *   an X across the matrix                begin() failed (most likely stock ESP32-S3 firmware;
 *                                         run PrintAllInputs to see why)
 *
 * Needs the forked bridge firmware on the ESP32-S3 (see the library README).
 *
 * SPDX-License-Identifier: MIT
 */
#include <Arduino_LED_Matrix.h>
#include <XboxControllerR4.h>

const int ROWS = 8;
const int COLS = 12;
const unsigned long POLL_INTERVAL_MS = 20;     // 50 Hz
const unsigned long RETRY_INTERVAL_MS = 2000;  // how often a failed begin() is tried again
const unsigned long BLINK_HALF_PERIOD_MS = 250;
const unsigned long LINK_LOST_MS = 200;           // no good poll for this long: ESP32-S3 not answering
const unsigned long SLOW_POLL_INTERVAL_MS = 500;  // poll rate while the ESP32-S3 is not answering

ArduinoLEDMatrix matrix;
XboxControllerR4 xbox;
uint8_t frame[ROWS][COLS];

bool ready = false;
unsigned long lastPollMs = 0;
unsigned long lastRetryMs = 0;

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

void clearFrame() {
  memset(frame, 0, sizeof(frame));
}

void fillFrame() {
  memset(frame, 1, sizeof(frame));
}

// Two diagonals: the "begin() failed" picture.
void drawCross() {
  clearFrame();
  for (int col = 0; col < COLS; col++) {
    int row = col * (ROWS - 1) / (COLS - 1);
    frame[row][col] = 1;
    frame[ROWS - 1 - row][col] = 1;
  }
}

// Maps -1.0..+1.0 to 0..count-1, clamped.
int scaleToIndex(float v, int count) {
  int i = (int)((v + 1.0f) * 0.5f * (count - 1) + 0.5f);
  if (i < 0) {
    return 0;
  }
  if (i > count - 1) {
    return count - 1;
  }
  return i;
}

void drawDot(bool visible) {
  clearFrame();
  if (!visible) {
    return;
  }
  int col = scaleToIndex(xbox.leftX(), COLS);
  int row = scaleToIndex(-xbox.leftY(), ROWS);  // stick up (+Y) is the top row (row 0)
  frame[row][col] = 1;
}

void startClient() {
  ready = xbox.begin();
  lastRetryMs = millis();  // a failed begin() is retried RETRY_INTERVAL_MS from now, whoever called it
  if (!ready) {
    drawCross();
    matrix.renderBitmap(frame, ROWS, COLS);
  }
}

void setup() {
  matrix.begin();
  clearFrame();
  matrix.renderBitmap(frame, ROWS, COLS);
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

  if (now - lastPollMs < pollInterval()) {
    return;
  }
  lastPollMs = now;
  // A good poll that says OFF means the ESP32-S3 restarted: start its Xbox client again.
  if (xbox.poll() && xbox.connectionState() == XBOX_R4_OFF) {
    startClient();
    return;
  }

  if (xbox.isConnected() && xbox.xboxNotif.btnA) {
    fillFrame();
  } else if (xbox.isConnected()) {
    drawDot(true);
  } else {
    drawDot((now / BLINK_HALF_PERIOD_MS) % 2 == 0);  // neutral stick: the dot sits in the centre
  }
  matrix.renderBitmap(frame, ROWS, COLS);
}

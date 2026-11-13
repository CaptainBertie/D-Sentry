// Lcd.h -- the 16x2 status display (spec 3.9, docs/architecture.md 3, 5 and 6).
//
// What each state shows (every row exactly LCD_COLS characters, space padded):
//   OFF               LCD_TXT_OFF_ROW0 / LCD_TXT_OFF_ROW1; backlight off when
//                     LCD_BACKLIGHT_OFF_WHEN_SYSTEM_OFF (on in every other state)
//   BOOT, CALIBRATING LCD_TXT_CALIBRATING / mode label (LCD_TXT_MODE_BT or _WIFI)
//   PAIRING           arm row (below, DISARMED) / mode + status: FROM_BOOT and GRACE
//                     "Pairing<dots>", NO_LINK "No Link" (architecture.md 6; TurretState picks
//                     the phase, LINK_RECONNECT_GRACE_MS)
//   DISARMED, ARMED   arm row / mode + "Connected" ("No Link" if the model says not connected)
//   SERVO TEACH       (DISARMED with DisplayModel::servoTeach) LCD_TXT_SERVO_TEACH + the commanded
//                     angle right-justified / LCD_TXT_SERVO_REST<rest>LCD_TXT_SERVO_FIRE<fire>
//                     LCD_TXT_SERVO_PREFIRE<prefire>LCD_TXT_SERVO_CLEAR<clear> with no spaces
//                     (four 3-digit angles fill 16 columns), LCD_TXT_SERVO_UNSET for an angle
//                     not recorded ("SERVO TEACH 118" / "R180F64P118C150", "R-F-P-C-")
//   FAULT             LCD_TXT_FAULT_STALL / _CAL (LCD_TXT_FAULT_OTHER for any other code) /
//                     mode + "Connected" or "No Link"
// Arm row: LCD_TXT_ARMED or LCD_TXT_DISARMED left; after ARMED, with LCD_TXT_SHOW_FIRE_MODE,
// the fire-mode suffix (LCD_TXT_FIRE_SEMI " S" / LCD_TXT_FIRE_FULL " F") when it fits beside the
// ammo text, else none; LCD_TXT_AMMO_PREFIX + the ammo count right-justified ("AMMO:15").
// Pairing status: LCD_TXT_PAIRING, one space, then the dots "", ".", ". .", ". . ." advancing
// every LCD_PAIRING_DOT_MS from the moment PAIRING was entered, padded with spaces to the
// longest pattern so the word "Pairing" stands still while the dots cycle ("BT Pairing . .").
//
// Row layout (layoutRow): left text from column 0, right text right-justified, at least one
// space between. Text that does not fit is cut: a left text longer than the row is cut at
// LCD_COLS, and the right text loses characters from its end. begin() logs (LOG_ERROR) every
// configured combination that will be cut, once ("WIFI" + "Pairing . . ." = 18 > 16 with the
// shipped config.h).
//
// Writing: the wanted rows are recomputed at most every LCD_REFRESH_MS. Every update() then
// makes at most ONE I2C transaction: a backlight change if one is due, else the first
// character where the wanted rows differ from what the display holds (shown_, this module's
// copy of the display RAM), preferring the character under the display's cursor so a run of
// changes needs no cursor moves. hal::lcdWriteAt returns AGAIN when it only moved the cursor;
// the same character is written by the next update(). The display is never cleared (it
// flickers). Without an LCD the rows are still computed (telemetry mirror) and nothing is
// written.
// Self-repair: the display RAM can go wrong without this module hearing of it (electrical
// noise on the I2C wires, an LCD unplugged and plugged back in). So while the display is
// believed to be right, one cell per LCD_REFRESH_MS is marked unknown and written again,
// round robin (a static screen costs one transaction per LCD_REFRESH_MS, a whole sweep takes
// LCD_ROWS * LCD_COLS * LCD_REFRESH_MS). A failed transaction (hal::LcdWrite::FAILED, or
// lcdBacklight false) marks every cell and the backlight unknown and leaves the LCD alone for
// LCD_RETRY_MS; the HAL then re-initialises the controller over the next updates (it ends
// clear) and the whole screen is redrawn.
//
// SPDX-License-Identifier: MIT
#pragma once
#include <stdint.h>
#include "Types.h"

class Debug;

class Lcd {
 public:
  void begin(Debug& debug);   // display assumed clear (LiquidCrystal_I2C::init), backlight on;
                              // logs the configured strings that will not fit
  void update(uint32_t nowMs, const DisplayModel& model);

  // ---- pure layout (unit tests, boot check) ----
  // `left` and `right` into one LCD_COLS row, space padded, NUL terminated. nullptr = "".
  // Returns false when anything had to be cut.
  static bool layoutRow(const char* left, const char* right, char out[LCD_COLS + 1]);
  // Both rows the model asks for at nowMs, and whether the backlight should be on. Returns
  // the number of rows that had to be cut.
  static uint8_t compose(const DisplayModel& model, uint32_t nowMs,
                         char rows[LCD_ROWS][LCD_COLS + 1], bool& backlight);
  // The pairing status text for a time in PAIRING ("Pairing" + dots, padded), NUL terminated,
  // cut to LCD_COLS.
  static void pairingText(uint32_t elapsedMs, char out[LCD_COLS + 1]);
  // How many configured text combinations do not fit (what begin() logs).
  static uint8_t overflowCount();

  // ---- state ----
  const char* row(uint8_t r) const { return want_[r]; }   // wanted text, LCD_COLS chars
  const char* shownRow(uint8_t r) const { return shown_[r]; }   // what the display holds
  // Both wanted rows, row 0 then row 1, space padded, no NUL (TelemetryPacket.lcd).
  void mirror(char out[DS_TELEMETRY_LCD_LEN]) const;
  bool present() const;
  bool inSync() const { return inSync_; }                 // display holds the wanted rows
  bool backlightWanted() const { return backlightWant_; }
  uint32_t charsWritten() const { return charsWritten_; }   // characters written
  // lcdWriteAt + lcdBacklight calls (a call that only waits for the HAL's re-initialisation
  // makes no I2C transaction)
  uint32_t transactions() const { return transactions_; }
  uint32_t cursorMoves() const { return cursorMoves_; }     // lcdWriteAt calls that returned AGAIN
  uint32_t refreshWrites() const { return refreshWrites_; } // characters rewritten by the round robin
  uint32_t failures() const { return failures_; }           // failed transactions seen
  uint32_t layoutCuts() const { return layoutCuts_; }       // composed rows that were cut
  uint8_t overflowWarnings() const { return overflowWarnings_; }   // lines begin() logged

 private:
  bool nextDiff(uint8_t& row, uint8_t& col) const;
  bool anyDiff() const;
  void logOverflows();
  void markRefreshCell(uint32_t nowMs);
  void onFailure(uint32_t nowMs);

  Debug* debug_ = nullptr;
  char want_[LCD_ROWS][LCD_COLS + 1] = {};
  char shown_[LCD_ROWS][LCD_COLS + 1] = {};   // what the display is believed to hold
  bool backlightWant_ = true;
  bool backlightShown_ = true;
  bool composed_ = false;          // want_ has been computed at least once
  uint32_t lastLayoutMs_ = 0;
  uint8_t cursorRow_ = 0;          // where the display's cursor is believed to be
  uint8_t cursorCol_ = 0;
  bool inSync_ = false;
  uint32_t charsWritten_ = 0;
  uint32_t transactions_ = 0;
  uint32_t cursorMoves_ = 0;
  uint32_t layoutCuts_ = 0;
  uint8_t overflowWarnings_ = 0;
  // self-repair
  uint32_t lastRefreshMs_ = 0;     // when the round robin last marked a cell
  uint8_t refreshRow_ = 0;         // the cell it marks next
  uint8_t refreshCol_ = 0;
  bool refreshPending_ = false;    // the marked cell has not been rewritten yet
  uint8_t pendingRow_ = 0;
  uint8_t pendingCol_ = 0;
  uint32_t refreshWrites_ = 0;
  bool waitRetry_ = false;         // a transaction failed at failMs_: wait LCD_RETRY_MS
  uint32_t failMs_ = 0;
  uint32_t failures_ = 0;
};

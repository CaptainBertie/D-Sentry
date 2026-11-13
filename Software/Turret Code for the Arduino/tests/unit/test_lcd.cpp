// test_lcd.cpp -- Lcd against spec 3.9, acceptance 1-3, 6, 7, 12 and docs/architecture.md 3, 5, 6.
//
// The expected rows are built here by an independent std::string layout (left text, spaces,
// right text), from the config.h texts, so the tests hold when the owner edits the texts. A few
// golden strings from the spec ("BT Pairing", "AMMO:15", the protocol example row) are also
// checked while config.h still has the shipped texts.
//
// KEY SAFETY TEST (I2C load / stepper smoothness, spec 3.3): at_most_one_transaction_per_update
// and static_screen_costs_only_the_slow_refresh fail if update() ever sends more than one I2C
// transaction or rewrites an unchanged screen faster than one cell per LCD_REFRESH_MS.
// Self-repair (a glitched or replugged LCD heals): a_glitched_cell_heals_within_one_sweep,
// an_unplugged_lcd_is_left_alone_then_redrawn_when_it_returns, a_nak_mid_write_redraws_everything.
//
// SPDX-License-Identifier: MIT
#include <algorithm>
#include <cstring>
#include <set>
#include <string>
#include <vector>

#include "Debug.h"
#include "Lcd.h"
#include "hal.h"
#include "hal_host.h"
#include "testkit.h"

namespace {

const bool kShippedTexts =
    std::strcmp(LCD_TXT_MODE_BT, "BT") == 0 && std::strcmp(LCD_TXT_MODE_WIFI, "WIFI") == 0 &&
    std::strcmp(LCD_TXT_PAIRING, "Pairing") == 0 && std::strcmp(LCD_TXT_ARMED, "ARMED") == 0 &&
    std::strcmp(LCD_TXT_DISARMED, "DISARMED") == 0 && std::strcmp(LCD_TXT_AMMO_PREFIX, "AMMO:") == 0 &&
    std::strcmp(LCD_TXT_CONNECTED, "Connected") == 0 && std::strcmp(LCD_TXT_NO_CONNECTION, "No Link") == 0 &&
    std::strcmp(LCD_TXT_FIRE_SEMI, " S") == 0 && std::strcmp(LCD_TXT_FIRE_FULL, " F") == 0 &&
    LCD_COLS == 16 && MAG_CAPACITY == 15 && LCD_TXT_SHOW_FIRE_MODE;

// Independent layout: left, then spaces, then right; the right text cut from its end.
std::string row(const std::string& left, const std::string& right) {
  std::string l = left.substr(0, LCD_COLS);
  const size_t gap = (!l.empty() && !right.empty()) ? 1 : 0;
  const size_t room = l.size() + gap <= (size_t)LCD_COLS ? LCD_COLS - l.size() - gap : 0;
  const std::string r = right.substr(0, room);
  return l + std::string(LCD_COLS - l.size() - r.size(), ' ') + r;
}
std::string ammoText(unsigned ammo) { return std::string(LCD_TXT_AMMO_PREFIX) + std::to_string(ammo); }
const std::vector<std::string> kDots = {"", ".", ". .", ". . ."};
std::string pairingText(size_t phase) {
  std::string t = std::string(LCD_TXT_PAIRING) + " " + kDots[phase % kDots.size()];
  const size_t width = std::strlen(LCD_TXT_PAIRING) + 1 + kDots.back().size();
  return t + std::string(width - t.size(), ' ');
}
std::string mode(uint8_t radio) { return radio == DS_RADIO_WIFI ? LCD_TXT_MODE_WIFI : LCD_TXT_MODE_BT; }

DisplayModel model(uint8_t state, uint32_t sinceMs = 0) {
  DisplayModel m;
  m.state = state;
  m.stateSinceMs = sinceMs;
  m.linkConnected = state == DS_STATE_DISARMED || state == DS_STATE_ARMED;
  m.ammo = MAG_CAPACITY;
  return m;
}

struct Rows {
  char r[LCD_ROWS][LCD_COLS + 1];
  bool backlight = true;
  uint8_t cuts = 0;
  Rows(const DisplayModel& m, uint32_t nowMs) { cuts = Lcd::compose(m, nowMs, r, backlight); }
  std::string top() const { return r[0]; }
  std::string bottom() const { return r[1]; }
};

struct Rig {
  Debug debug;
  Lcd lcd;
  explicit Rig(bool lcdPresent = true) {
    hal_host::reset();
    if (!lcdPresent) hal_host::setLcdPresent(false, 0);
    hal::begin();
    DebugSources none;
    debug.begin(none);
    lcd.begin(debug);
  }
  // One loop() worth of LCD work, 1 ms apart; returns the I2C transactions it made.
  uint32_t step(const DisplayModel& m, uint64_t us = US_PER_MS) {
    hal_host::advanceUs(us);
    const uint32_t before = hal_host::lcdTransactions();
    lcd.update(hal::nowMs(), m);
    return hal_host::lcdTransactions() - before;
  }
  // Run past the next recompute (LCD_REFRESH_MS), then until the display holds the wanted
  // rows (or give up). Returns the updates it ran.
  uint32_t settle(const DisplayModel& m, uint32_t maxUpdates = 1000) {
    uint32_t n = 0;
    while (n < maxUpdates && (n <= LCD_REFRESH_MS || !lcd.inSync())) {
      step(m);
      n++;
    }
    return n;
  }
  std::string display(uint8_t r) const { return hal_host::lcdRow(r); }
};

std::string drainUsb() {
  std::string out;
  for (int i = 0; i < 2000; i++) {
    hal_host::advanceUs(100);
    hal::service(hal::nowUs());
    out += hal_host::usbTakeTx();
  }
  return out;
}

}  // namespace

// ---- layoutRow ----------------------------------------------------------------------------------------
TEST(layout_left_right_justified_with_a_space) {
  char out[LCD_COLS + 1];
  CHECK(Lcd::layoutRow("ARMED F", "AMMO:12", out));
  CHECK_EQ(std::string(out), row("ARMED F", "AMMO:12"));
  if (LCD_COLS == 16) {
    CHECK_EQ(std::string(out), "ARMED F  AMMO:12");   // protocol_spec.py TelemetryPacket example
    Lcd::layoutRow("BT", "Connected", out);
    CHECK_EQ(std::string(out), "BT     Connected");
    CHECK(Lcd::layoutRow("DISARMED", "AMMO:15", out));   // exactly 16: fits, one space
    CHECK_EQ(std::string(out), "DISARMED AMMO:15");
    CHECK(!Lcd::layoutRow("WIFI", "Pairing . . .", out));   // 18 > 16: right text cut at its end
    CHECK_EQ(std::string(out), "WIFI Pairing . .");
  }
  CHECK(Lcd::layoutRow("", "", out));
  CHECK_EQ(std::string(out), std::string(LCD_COLS, ' '));
  CHECK(Lcd::layoutRow(nullptr, nullptr, out));
  CHECK_EQ(std::string(out), std::string(LCD_COLS, ' '));
  CHECK(Lcd::layoutRow("", "x", out));   // no left text: no separating space needed
  CHECK_EQ(std::string(out), std::string(LCD_COLS - 1, ' ') + "x");
  const std::string tooLong(LCD_COLS + 4, 'L');
  CHECK(!Lcd::layoutRow(tooLong.c_str(), "R", out));   // left cut at LCD_COLS, right dropped
  CHECK_EQ(std::string(out), std::string(LCD_COLS, 'L'));
  const std::string exact(LCD_COLS, 'L');
  CHECK(Lcd::layoutRow(exact.c_str(), "", out));
  CHECK(!Lcd::layoutRow(exact.c_str(), "R", out));   // no room for even the space
  CHECK_EQ(std::string(out), exact);
  const std::string almost(LCD_COLS - 2, 'L');
  CHECK(Lcd::layoutRow(almost.c_str(), "R", out));   // L...L R: one space, fits exactly
  CHECK_EQ(std::string(out), almost + " R");
  CHECK(!Lcd::layoutRow(almost.c_str(), "RS", out));
  CHECK_EQ(std::string(out), almost + " R");          // cut from its end: "RS" -> "R"
}

// ---- every state's text (spec 3.9, architecture.md 6) -----------------------------------------------------
TEST(off_shows_system_off_and_backlight_per_config) {
  Rows r(model(DS_STATE_OFF), 0);
  CHECK_EQ(r.top(), row(LCD_TXT_OFF_ROW0, ""));
  CHECK_EQ(r.bottom(), row(LCD_TXT_OFF_ROW1, ""));
  CHECK_EQ(r.backlight, !LCD_BACKLIGHT_OFF_WHEN_SYSTEM_OFF);
  if (kShippedTexts) CHECK_EQ(r.top(), "SYSTEM OFF      ");
}

TEST(boot_and_calibrating_show_calibrating_and_the_mode) {
  for (uint8_t state : {DS_STATE_BOOT, DS_STATE_CALIBRATING}) {
    for (uint8_t radio : {DS_RADIO_BLE, DS_RADIO_WIFI}) {
      DisplayModel m = model(state);
      m.radioMode = radio;
      Rows r(m, 0);
      CHECK_EQ(r.top(), row(LCD_TXT_CALIBRATING, ""));
      CHECK_EQ(r.bottom(), row(mode(radio), ""));
      CHECK(r.backlight);
    }
  }
}

TEST(pairing_rows_for_each_phase) {
  DisplayModel m = model(DS_STATE_PAIRING, 1000);
  for (PairingPhase phase : {PairingPhase::FROM_BOOT, PairingPhase::GRACE}) {
    m.pairing = phase;
    for (size_t k = 0; k < 2 * kDots.size(); k++) {
      Rows r(m, 1000 + (uint32_t)k * LCD_PAIRING_DOT_MS);
      CHECK_EQ(r.top(), row(LCD_TXT_DISARMED, ammoText(MAG_CAPACITY)));
      CHECK_EQ(r.bottom(), row(mode(DS_RADIO_BLE), pairingText(k)));
    }
  }
  m.pairing = PairingPhase::NO_LINK;
  Rows r(m, 5000);
  CHECK_EQ(r.bottom(), row(mode(DS_RADIO_BLE), LCD_TXT_NO_CONNECTION));
  if (kShippedTexts) {
    // acceptance 3: "BT Pairing" with the dots cycling
    m.pairing = PairingPhase::FROM_BOOT;
    CHECK_EQ(Rows(m, 1000).bottom(), "BT Pairing      ");
    CHECK_EQ(Rows(m, 1000 + LCD_PAIRING_DOT_MS).bottom(), "BT Pairing .    ");
    CHECK_EQ(Rows(m, 1000 + 2 * LCD_PAIRING_DOT_MS).bottom(), "BT Pairing . .  ");
    CHECK_EQ(Rows(m, 1000 + 3 * LCD_PAIRING_DOT_MS).bottom(), "BT Pairing . . .");
    CHECK_EQ(Rows(m, 1000 + 4 * LCD_PAIRING_DOT_MS).bottom(), "BT Pairing      ");
    CHECK_EQ(Rows(m, 1000).top(), "DISARMED AMMO:15");
    m.pairing = PairingPhase::NO_LINK;
    CHECK_EQ(Rows(m, 1000).bottom(), "BT       No Link");
    // WIFI + "Pairing . . ." is 18 > 16: the third dot is cut, the word stands still
    m.pairing = PairingPhase::GRACE;
    m.radioMode = DS_RADIO_WIFI;
    CHECK_EQ(Rows(m, 1000).bottom(), "WIFI Pairing    ");
    CHECK_EQ(Rows(m, 1000 + 2 * LCD_PAIRING_DOT_MS).bottom(), "WIFI Pairing . .");
    Rows cut(m, 1000 + 3 * LCD_PAIRING_DOT_MS);
    CHECK_EQ(cut.bottom(), "WIFI Pairing . .");
    CHECK_EQ(cut.cuts, 1);
  }
}

TEST(disarmed_and_armed_rows) {
  DisplayModel m = model(DS_STATE_DISARMED);
  CHECK_EQ(Rows(m, 0).top(), row(LCD_TXT_DISARMED, ammoText(MAG_CAPACITY)));
  CHECK_EQ(Rows(m, 0).bottom(), row(mode(DS_RADIO_BLE), LCD_TXT_CONNECTED));
  m.radioMode = DS_RADIO_WIFI;
  CHECK_EQ(Rows(m, 0).bottom(), row(mode(DS_RADIO_WIFI), LCD_TXT_CONNECTED));
  m.state = DS_STATE_ARMED;
  for (uint8_t fire : {DS_FIRE_MODE_SEMI, DS_FIRE_MODE_FULL}) {
    m.fireMode = fire;
    for (unsigned ammo : {0u, 7u, (unsigned)MAG_CAPACITY, 255u}) {
      m.ammo = (uint8_t)ammo;
      const std::string suffix = fire == DS_FIRE_MODE_FULL ? LCD_TXT_FIRE_FULL : LCD_TXT_FIRE_SEMI;
      const std::string right = ammoText(ammo);
      const bool fits = std::strlen(LCD_TXT_ARMED) + suffix.size() + 1 + right.size() <= (size_t)LCD_COLS;
      const std::string left = std::string(LCD_TXT_ARMED) + (LCD_TXT_SHOW_FIRE_MODE && fits ? suffix : "");
      CHECK_EQ(Rows(m, 0).top(), row(left, right));
    }
  }
  if (kShippedTexts) {
    m.ammo = 15;
    m.fireMode = DS_FIRE_MODE_SEMI;
    CHECK_EQ(Rows(m, 0).top(), "ARMED S  AMMO:15");
    m.fireMode = DS_FIRE_MODE_FULL;
    m.ammo = 12;
    CHECK_EQ(Rows(m, 0).top(), "ARMED F  AMMO:12");
    m.ammo = 0;
    CHECK_EQ(Rows(m, 0).top(), "ARMED F   AMMO:0");
    m.state = DS_STATE_DISARMED;   // the suffix is ARMED's only (config.h comment)
    CHECK_EQ(Rows(m, 0).top(), "DISARMED  AMMO:0");
    m.linkConnected = false;       // a model that says not connected is shown truthfully
    CHECK_EQ(Rows(m, 0).bottom(), "WIFI     No Link");
  }
}

TEST(fault_rows_name_the_fault) {
  DisplayModel m = model(DS_STATE_FAULT);
  m.fault = DS_FAULT_STALL;
  m.linkConnected = true;
  CHECK_EQ(Rows(m, 0).top(), row(LCD_TXT_FAULT_STALL, ""));
  CHECK_EQ(Rows(m, 0).bottom(), row(mode(DS_RADIO_BLE), LCD_TXT_CONNECTED));
  m.fault = DS_FAULT_CAL;
  m.linkConnected = false;
  CHECK_EQ(Rows(m, 0).top(), row(LCD_TXT_FAULT_CAL, ""));
  CHECK_EQ(Rows(m, 0).bottom(), row(mode(DS_RADIO_BLE), LCD_TXT_NO_CONNECTION));
  m.fault = DS_FAULT_NONE;   // FAULT with no code still says FAULT, never a blank row
  CHECK(Rows(m, 0).top() != std::string(LCD_COLS, ' '));
  CHECK(Rows(m, 0).backlight);
}

TEST(every_row_ever_composed_is_exactly_lcd_cols_printable) {
  uint32_t rows = 0;
  std::set<std::string> distinct;
  for (uint8_t state = 0; state <= DS_STATE_COUNT; state++) {
    for (uint8_t radio : {DS_RADIO_BLE, DS_RADIO_WIFI}) {
      for (uint8_t fire = 0; fire < DS_FIRE_MODE_COUNT; fire++) {
        for (PairingPhase phase : {PairingPhase::FROM_BOOT, PairingPhase::GRACE, PairingPhase::NO_LINK}) {
          for (uint8_t fault : {DS_FAULT_NONE, DS_FAULT_CAL, DS_FAULT_STALL}) {
            for (unsigned ammo = 0; ammo <= 255; ammo += 17) {
              for (uint32_t t = 0; t < 3 * kDots.size() * LCD_PAIRING_DOT_MS; t += LCD_PAIRING_DOT_MS / 3) {
                DisplayModel m = model(state, UINT32_MAX - 500);   // across the millis() wrap
                m.radioMode = radio;
                m.fireMode = fire;
                m.pairing = phase;
                m.fault = fault;
                m.ammo = (uint8_t)ammo;
                m.linkConnected = (t / LCD_PAIRING_DOT_MS) & 1;
                Rows r(m, (UINT32_MAX - 500) + t);
                for (uint8_t i = 0; i < LCD_ROWS; i++) {
                  rows++;
                  if (!CHECK_EQ(std::strlen(r.r[i]), (size_t)LCD_COLS)) return;
                  for (uint8_t c = 0; c < LCD_COLS; c++)
                    if (!CHECK(r.r[i][c] >= ' ' && r.r[i][c] <= '~')) return;
                  distinct.insert(r.r[i]);
                }
              }
            }
          }
        }
      }
    }
  }
  CHECK(rows > 100000u);
  CHECK(distinct.size() > 50u);
}

// ---- writing: diff, one transaction per update, convergence -----------------------------------------------
TEST(converges_to_the_wanted_rows_through_cursor_moves) {
  Rig rig;
  DisplayModel m = model(DS_STATE_DISARMED);
  uint32_t updates = 0;   // a fresh Lcd composes on its first update, then drips
  do {
    rig.step(m);
    updates++;
  } while (!rig.lcd.inSync() && updates < 1000);
  CHECK(rig.lcd.inSync());
  for (uint8_t r = 0; r < LCD_ROWS; r++) {
    CHECK_EQ(rig.display(r), std::string(rig.lcd.row(r)));
    CHECK_EQ(std::string(rig.lcd.shownRow(r)), std::string(rig.lcd.row(r)));
  }
  // Every changed character costs one write; the host HAL's lcdWriteAt returns AGAIN for each
  // cursor move, and the same character is written by the next update.
  CHECK(rig.lcd.cursorMoves() > 0u);
  CHECK_EQ(hal_host::lcdCharsWritten(), rig.lcd.charsWritten());
  CHECK_EQ(hal_host::lcdTransactions(), rig.lcd.transactions());
  // one transaction in every update until in sync, none wasted
  CHECK_EQ(updates, rig.lcd.charsWritten() + rig.lcd.cursorMoves());
  // characters that were already right (spaces) are never written
  uint32_t nonBlank = 0;
  for (uint8_t r = 0; r < LCD_ROWS; r++)
    for (uint8_t c = 0; c < LCD_COLS; c++) nonBlank += rig.lcd.row(r)[c] != ' ';
  CHECK_EQ(rig.lcd.charsWritten(), nonBlank);
}

TEST(static_screen_costs_only_the_slow_refresh) {
  Rig rig;
  DisplayModel m = model(DS_STATE_ARMED);
  rig.settle(m);
  const uint32_t before = hal_host::lcdTransactions();
  const uint32_t chars = hal_host::lcdCharsWritten();
  const uint32_t refresh = rig.lcd.refreshWrites();
  const std::string top = rig.display(0), bottom = rig.display(1);
  uint32_t worst = 0;
  for (uint32_t i = 0; i < 5 * MS_PER_S; i++) worst = std::max(worst, rig.step(m));   // 5 s
  const uint32_t refreshes = 5 * MS_PER_S / LCD_REFRESH_MS;
  // One cell per LCD_REFRESH_MS, each a character (plus a cursor move when the sweep wraps to
  // the next row): nothing else is written while nothing changes.
  CHECK(hal_host::lcdTransactions() - before <= 2 * (refreshes + 1));
  CHECK(hal_host::lcdTransactions() - before >= refreshes - 1);
  CHECK_EQ(hal_host::lcdCharsWritten() - chars, rig.lcd.refreshWrites() - refresh);
  CHECK(rig.lcd.refreshWrites() - refresh >= refreshes - 1);
  CHECK_EQ(worst, 1u);
  CHECK_EQ(rig.display(0), top);
  CHECK_EQ(rig.display(1), bottom);
}

// Electrical noise changes a cell behind the module's back: the round robin rewrites every cell
// within one sweep, LCD_ROWS * LCD_COLS * LCD_REFRESH_MS.
TEST(a_glitched_cell_heals_within_one_sweep) {
  Rig rig;
  DisplayModel m = model(DS_STATE_DISARMED);
  rig.settle(m);
  const std::string want0 = rig.lcd.row(0), want1 = rig.lcd.row(1);
  hal_host::lcdGlitch(1, LCD_COLS - 1, '#');
  hal_host::lcdGlitch(0, 0, '%');
  CHECK(rig.display(0) != want0);
  const uint32_t sweepMs = LCD_ROWS * LCD_COLS * LCD_REFRESH_MS;
  uint32_t healedAt = 0;
  for (uint32_t i = 0; i < sweepMs + 2 * LCD_REFRESH_MS; i++) {
    rig.step(m);
    if (!healedAt && rig.display(0) == want0 && rig.display(1) == want1) healedAt = i;
  }
  CHECK(healedAt > 0u);
  CHECK(healedAt <= sweepMs + LCD_REFRESH_MS);
  CHECK_EQ(rig.display(0), want0);
  CHECK_EQ(rig.display(1), want1);
}

// Unplugged: transactions fail, the module tries again only every LCD_RETRY_MS (each try costs
// one refused transaction). Plugged back in: the HAL re-initialises the controller and every
// cell is redrawn within a second, never more than one transaction per update.
TEST(an_unplugged_lcd_is_left_alone_then_redrawn_when_it_returns) {
  Rig rig;
  DisplayModel m = model(DS_STATE_ARMED);
  rig.settle(m);
  const std::string want0 = rig.lcd.row(0), want1 = rig.lcd.row(1);
  hal_host::setLcdAttached(false);
  uint32_t worst = 0;
  const uint32_t failed0 = hal_host::lcdFailedTransactions();
  for (uint32_t i = 0; i < 5 * MS_PER_S; i++) worst = std::max(worst, rig.step(m));
  const uint32_t failedTries = hal_host::lcdFailedTransactions() - failed0;
  CHECK(failedTries >= 1u);
  CHECK(failedTries <= 5 * MS_PER_S / LCD_RETRY_MS + 2);
  CHECK(rig.lcd.failures() >= 1u);
  CHECK(!rig.lcd.inSync());
  hal_host::setLcdAttached(true);
  CHECK_EQ(rig.display(0), std::string(LCD_COLS, ' '));   // powered up dark: needs a re-init
  uint32_t healedAt = 0;
  for (uint32_t i = 0; i < 2 * MS_PER_S; i++) {
    worst = std::max(worst, rig.step(m));
    if (!healedAt && rig.display(0) == want0 && rig.display(1) == want1) healedAt = i;
  }
  CHECK(hal_host::lcdControllerReady());
  CHECK(hal::lcdReinits() >= 1u);
  CHECK(healedAt > 0u);
  CHECK(healedAt <= LCD_RETRY_MS + MS_PER_S / 2);
  CHECK_EQ(rig.display(0), want0);
  CHECK_EQ(rig.display(1), want1);
  CHECK(hal_host::lcdBacklightOn());
  CHECK_EQ(worst, 1u);
}

// A single refused transaction (bus noise): the module distrusts the whole display, waits
// LCD_RETRY_MS, lets the HAL re-initialise it and redraws all of it.
TEST(a_nak_mid_write_redraws_everything) {
  Rig rig;
  DisplayModel m = model(DS_STATE_DISARMED);
  rig.step(m);   // compose
  hal_host::failLcdTransactions(1);
  uint32_t i = 0;
  while (rig.lcd.failures() == 0 && i++ < 100) rig.step(m);
  CHECK_EQ(rig.lcd.failures(), 1u);
  const uint32_t tx = hal_host::lcdTransactions();
  for (uint32_t k = 0; k + 2 < LCD_RETRY_MS; k++) rig.step(m);   // the retry wait: no traffic
  CHECK_EQ(hal_host::lcdTransactions(), tx);
  rig.settle(m, 5 * MS_PER_S);
  CHECK(rig.lcd.inSync());
  CHECK_EQ(hal::lcdReinits(), 1u);
  for (uint8_t r = 0; r < LCD_ROWS; r++) CHECK_EQ(rig.display(r), std::string(rig.lcd.row(r)));
}

TEST(one_character_change_is_one_write) {
  Rig rig;
  DisplayModel m = model(DS_STATE_ARMED);
  m.ammo = 15;
  rig.settle(m);
  const uint32_t chars = hal_host::lcdCharsWritten();
  const uint32_t tx = hal_host::lcdTransactions();
  const uint32_t refresh = rig.lcd.refreshWrites();
  m.ammo = 14;   // "AMMO:15" -> "AMMO:14": one column differs
  for (uint32_t i = 0; i < 2 * LCD_REFRESH_MS; i++) rig.step(m);
  CHECK(rig.lcd.inSync());
  const uint32_t refreshed = rig.lcd.refreshWrites() - refresh;   // the round robin's own
  CHECK_EQ(hal_host::lcdCharsWritten() - chars - refreshed, 1u);
  // at most one cursor move + the character, beside the round robin's (char + move each)
  CHECK(hal_host::lcdTransactions() - tx <= 2u + 2 * refreshed);
  CHECK_EQ(rig.display(0), row(LCD_TXT_ARMED + std::string(LCD_TXT_SHOW_FIRE_MODE ? LCD_TXT_FIRE_SEMI : ""), ammoText(14)));
}

TEST(at_most_one_transaction_per_update_while_everything_changes) {
  Rig rig;
  // Cycle through states faster than the screen can be redrawn; also across OFF (backlight).
  const uint8_t states[] = {DS_STATE_OFF, DS_STATE_BOOT, DS_STATE_PAIRING, DS_STATE_DISARMED,
                            DS_STATE_ARMED, DS_STATE_FAULT, DS_STATE_OFF};
  uint32_t worst = 0;
  for (uint32_t i = 0; i < 20000; i++) {
    DisplayModel m = model(states[(i / 37) % (sizeof states)], 0);
    m.radioMode = (i / 101) & 1 ? DS_RADIO_WIFI : DS_RADIO_BLE;
    m.ammo = (uint8_t)(i / 53);
    m.fault = DS_FAULT_STALL;
    worst = std::max(worst, rig.step(m, 300));
  }
  CHECK_EQ(worst, 1u);
  // and it still converges once the model stops changing
  DisplayModel last = model(DS_STATE_PAIRING, hal::nowMs());
  last.pairing = PairingPhase::NO_LINK;
  rig.settle(last);
  CHECK(rig.lcd.inSync());
  for (uint8_t r = 0; r < LCD_ROWS; r++) CHECK_EQ(rig.display(r), std::string(rig.lcd.row(r)));
  CHECK(hal_host::lcdBacklightOn());
}

TEST(rows_are_recomputed_at_most_every_refresh) {
  Rig rig;
  DisplayModel m = model(DS_STATE_ARMED);
  m.ammo = 15;
  rig.settle(m);
  const uint32_t t0 = hal::nowMs();
  m.ammo = 9;
  std::string before = rig.lcd.row(0);
  uint32_t changedAt = 0;
  for (uint32_t i = 0; i < 3 * LCD_REFRESH_MS; i++) {
    rig.step(m);
    if (!changedAt && std::string(rig.lcd.row(0)) != before) changedAt = hal::nowMs();
  }
  CHECK(changedAt > 0u);
  CHECK(changedAt - t0 <= LCD_REFRESH_MS);   // picked up at the next recompute, not later
  // two recomputes are never closer than LCD_REFRESH_MS: flip the model every update
  Rig rig2;
  std::vector<uint32_t> changes;
  std::string prev = rig2.lcd.row(0);
  for (uint32_t i = 0; i < 1000; i++) {
    DisplayModel x = model(DS_STATE_ARMED);
    x.ammo = (uint8_t)(i % 7);   // a new ammo count every update
    rig2.step(x);
    if (std::string(rig2.lcd.row(0)) != prev) {
      prev = rig2.lcd.row(0);
      changes.push_back(hal::nowMs());
    }
  }
  CHECK(changes.size() >= 2u);
  for (size_t i = 1; i < changes.size(); i++) CHECK(changes[i] - changes[i - 1] >= LCD_REFRESH_MS);
}

TEST(pairing_dots_advance_every_dot_period) {
  Rig rig;
  const uint32_t since = hal::nowMs();
  DisplayModel m = model(DS_STATE_PAIRING, since);
  m.pairing = PairingPhase::FROM_BOOT;
  std::string prev;
  std::vector<std::pair<uint32_t, std::string>> seen;   // (ms since PAIRING, wanted row 1)
  for (uint32_t i = 0; i < 4 * kDots.size() * LCD_PAIRING_DOT_MS; i++) {
    rig.step(m);
    const std::string now = rig.lcd.row(1);
    if (now != prev) {
      seen.push_back({hal::nowMs() - since, now});
      prev = now;
    }
  }
  CHECK(seen.size() >= 4 * kDots.size());
  for (size_t i = 0; i < seen.size(); i++) {
    const uint32_t t = seen[i].first;
    const size_t phase = (t / LCD_PAIRING_DOT_MS) % kDots.size();
    CHECK_EQ(seen[i].second, row(mode(DS_RADIO_BLE), pairingText(phase)));
    // a change lands at most one refresh after the dot boundary it belongs to
    if (i > 0) CHECK(t % LCD_PAIRING_DOT_MS < LCD_REFRESH_MS);
  }
  // and the display followed within a few updates of the last change
  CHECK_EQ(rig.display(1), std::string(rig.lcd.row(1)));
}

TEST(backlight_follows_off_state_as_one_transaction) {
  Rig rig;
  rig.settle(model(DS_STATE_DISARMED));
  CHECK(hal_host::lcdBacklightOn());
  const DisplayModel off = model(DS_STATE_OFF);
  for (uint32_t i = 0; i < 3 * LCD_REFRESH_MS; i++) CHECK(rig.step(off) <= 1u);
  rig.settle(off);
  CHECK_EQ(hal_host::lcdBacklightOn(), !LCD_BACKLIGHT_OFF_WHEN_SYSTEM_OFF);
  CHECK_EQ(rig.display(0), row(LCD_TXT_OFF_ROW0, ""));
  rig.settle(model(DS_STATE_BOOT));
  CHECK(hal_host::lcdBacklightOn());
  CHECK_EQ(rig.display(0), row(LCD_TXT_CALIBRATING, ""));
}

TEST(without_an_lcd_rows_still_render_and_nothing_is_sent) {
  Rig rig(false);
  CHECK(!rig.lcd.present());
  DisplayModel m = model(DS_STATE_ARMED);
  for (uint32_t i = 0; i < MS_PER_S; i++) rig.step(m);
  CHECK_EQ(hal_host::lcdTransactions(), 0u);
  CHECK(rig.lcd.inSync());
  CHECK_EQ(std::string(rig.lcd.row(0)), Rows(m, 0).top());
  char mirror[DS_TELEMETRY_LCD_LEN];
  rig.lcd.mirror(mirror);
  CHECK_EQ(std::string(mirror, DS_TELEMETRY_LCD_LEN), Rows(m, 0).top() + Rows(m, 0).bottom());
}

TEST(mirror_is_both_rows_space_padded) {
  Rig rig;
  DisplayModel m = model(DS_STATE_ARMED);
  m.fireMode = DS_FIRE_MODE_FULL;
  m.ammo = 12;
  rig.settle(m);
  char mirror[DS_TELEMETRY_LCD_LEN];
  rig.lcd.mirror(mirror);
  const std::string s(mirror, DS_TELEMETRY_LCD_LEN);
  CHECK_EQ(s, std::string(rig.lcd.row(0)) + rig.lcd.row(1));
  if (kShippedTexts) CHECK_EQ(s, "ARMED F  AMMO:12BT     Connected");   // protocol_spec.py example
}

// ---- the boot warning ---------------------------------------------------------------------------------
TEST(boot_logs_each_configured_overflow_once) {
  // Independent count of the combinations that cannot fit.
  auto need = [](size_t l, size_t r) { return l + r + (l && r ? 1 : 0); };
  unsigned expected = 0;
  for (const char* t : {LCD_TXT_OFF_ROW0, LCD_TXT_OFF_ROW1, LCD_TXT_CALIBRATING, LCD_TXT_FAULT_STALL,
                        LCD_TXT_FAULT_CAL, LCD_TXT_MODE_BT, LCD_TXT_MODE_WIFI})
    expected += std::strlen(t) > (size_t)LCD_COLS;
  const size_t ammo = ammoText(MAG_CAPACITY).size();
  expected += need(std::strlen(LCD_TXT_DISARMED), ammo) > (size_t)LCD_COLS;
  expected += need(std::strlen(LCD_TXT_ARMED), ammo) > (size_t)LCD_COLS;
  if (LCD_TXT_SHOW_FIRE_MODE)
    for (const char* suffix : {LCD_TXT_FIRE_SEMI, LCD_TXT_FIRE_FULL})
      expected += need(std::strlen(LCD_TXT_ARMED) + std::strlen(suffix), ammo) > (size_t)LCD_COLS;
  for (const char* md : {LCD_TXT_MODE_BT, LCD_TXT_MODE_WIFI})
    for (size_t st : {std::strlen(LCD_TXT_CONNECTED), pairingText(kDots.size() - 1).size(),
                      std::strlen(LCD_TXT_NO_CONNECTION)})
      expected += need(std::strlen(md), st) > (size_t)LCD_COLS;
  // SERVO ADJUST: its text alone, beside the widest angle, and "R:<n> F:<n>" at its widest.
  const size_t deg = std::to_string(SERVO_SPAN_DEG).size();
  expected += std::strlen(LCD_TXT_SERVO_ADJUST) > (size_t)LCD_COLS;
  expected += need(std::strlen(LCD_TXT_SERVO_ADJUST), deg) > (size_t)LCD_COLS;
  expected += std::strlen(LCD_TXT_SERVO_REST) + deg + 1 + std::strlen(LCD_TXT_SERVO_FIRE) + deg > (size_t)LCD_COLS;
  CHECK_EQ((unsigned)Lcd::overflowCount(), expected);

  Rig rig;
  CHECK_EQ((unsigned)rig.lcd.overflowWarnings(), expected);
  const std::string log = drainUsb();
  size_t lines = 0;
  for (size_t p = log.find("LCD"); p != std::string::npos; p = log.find("LCD", p + 1)) lines++;
  CHECK_EQ(lines, (size_t)expected);
  if (kShippedTexts) {
    CHECK_EQ(expected, 1u);   // the one the spec work found: WIFI + "Pairing . . ." = 18 > 16
    CHECK(log.find("\"WIFI\" + \"Pairing . . .\" = 18 > 16") != std::string::npos);
  }
  // once: updates never log it again
  for (uint32_t i = 0; i < MS_PER_S; i++) rig.step(model(DS_STATE_PAIRING));
  CHECK(drainUsb().find("LCD") == std::string::npos);
}

// ---- SERVO ADJUST rows (Lcd.h) ------------------------------------------------------------------------
// Row 0: the adjust text and the commanded angle; row 1: the angles in force (saved, config.h's or
// just recorded: Lcd shows what TurretState puts in the model), always both numbers.
TEST(servo_adjust_rows_show_the_angle_and_the_angles_in_force) {
  DisplayModel m = model(DS_STATE_DISARMED);
  m.servoTeach = true;
  m.servoDeg = 23;
  m.restDeg = 20;
  m.fireDeg = 110;
  CHECK_EQ(Rows(m, 0).top(), row(LCD_TXT_SERVO_ADJUST, "23"));
  CHECK_EQ(Rows(m, 0).bottom(), row(std::string(LCD_TXT_SERVO_REST) + "20 " + LCD_TXT_SERVO_FIRE + "110", ""));
  m.restDeg = SERVO_SPAN_DEG;   // the widest rows: three digits everywhere, nothing cut
  m.fireDeg = SERVO_SPAN_DEG;
  m.servoDeg = SERVO_SPAN_DEG;
  const std::string span = std::to_string(SERVO_SPAN_DEG);
  CHECK_EQ(Rows(m, 0).top(), row(LCD_TXT_SERVO_ADJUST, span));
  CHECK_EQ(Rows(m, 0).bottom(), row(std::string(LCD_TXT_SERVO_REST) + span + " " + LCD_TXT_SERVO_FIRE + span, ""));
  CHECK_EQ(Rows(m, 0).cuts, (uint8_t)0);
  m.restDeg = 23;
  m.fireDeg = 108;
  m.servoDeg = 108;
  CHECK_EQ(Rows(m, 0).top(), row(LCD_TXT_SERVO_ADJUST, "108"));
  CHECK_EQ(Rows(m, 0).bottom(), row(std::string(LCD_TXT_SERVO_REST) + "23 " + LCD_TXT_SERVO_FIRE + "108", ""));
  CHECK_EQ(Rows(m, 0).cuts, (uint8_t)0);
  if (std::strcmp(LCD_TXT_SERVO_ADJUST, "SERVO ADJUST") == 0 && std::strcmp(LCD_TXT_SERVO_REST, "R:") == 0 &&
      std::strcmp(LCD_TXT_SERVO_FIRE, "F:") == 0 && LCD_COLS == 16) {
    CHECK_EQ(Rows(m, 0).top(), "SERVO ADJUST 108");
    CHECK_EQ(Rows(m, 0).bottom(), "R:23 F:108      ");
  }
  // The flag means nothing outside DISARMED (TurretState never sets it there).
  m.state = DS_STATE_ARMED;
  CHECK(Rows(m, 0).top().compare(0, std::strlen(LCD_TXT_ARMED), LCD_TXT_ARMED) == 0);
  m.state = DS_STATE_FAULT;
  m.fault = DS_FAULT_STALL;
  CHECK_EQ(Rows(m, 0).top(), row(LCD_TXT_FAULT_STALL, ""));
}

TESTKIT_MAIN()

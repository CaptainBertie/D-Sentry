// Lcd.cpp -- see Lcd.h.
//
// SPDX-License-Identifier: MIT
#include "Lcd.h"

#include <string.h>
#include "Debug.h"
#include "hal.h"

static_assert(LCD_ROWS * LCD_COLS == DS_TELEMETRY_LCD_LEN,
              "TelemetryPacket.lcd mirrors exactly the LCD's rows");
static_assert(LCD_ROWS == DS_TELEMETRY_LCD_ROWS && LCD_COLS == DS_TELEMETRY_LCD_COLS,
              "compose() fills a top and a bottom row, as TelemetryPacket.lcd carries");
static_assert(LCD_REFRESH_MS > 0 && LCD_PAIRING_DOT_MS > 0, "LCD periods must be positive");
static_assert(LCD_ROWS * LCD_COLS <= UINT8_MAX, "the round robin counts cells in a uint8_t");

namespace {
// A value shown_ holds for a cell whose content is unknown: never a wanted character (the rows
// are built from NUL-terminated texts and space padding), so the cell is rewritten.
constexpr char kUnknownCell = '\0';
}  // namespace

namespace {

enum : uint8_t { ROW_TOP = 0, ROW_BOTTOM = 1 };

// The pairing dots (spec 3.9), shortest first; the cycle restarts after the last one.
const char* const kDots[] = {"", ".", ". .", ". . ."};
const uint8_t kDotPatterns = (uint8_t)(sizeof kDots / sizeof kDots[0]);

const char* orEmpty(const char* s) { return s ? s : ""; }

size_t dotsWidth() {
  size_t w = 0;
  for (uint8_t i = 0; i < kDotPatterns; i++) {
    const size_t n = strlen(kDots[i]);
    if (n > w) w = n;
  }
  return w;
}

// The width of the pairing status text: "Pairing", one space, the longest dots pattern.
size_t pairingWidth() { return strlen(LCD_TXT_PAIRING) + 1 + dotsWidth(); }

size_t decimalDigits(uint32_t v) {
  size_t n = 1;
  while (v >= DEC_BASE) {
    v /= DEC_BASE;
    n++;
  }
  return n;
}

// A row-sized text builder: never longer than LCD_COLS, always NUL terminated.
class Text {
 public:
  Text() { buf_[0] = 0; }
  Text& add(char c) {
    if (len_ < LCD_COLS)
      buf_[len_++] = c;
    else
      cut_ = true;
    buf_[len_] = 0;
    return *this;
  }
  Text& add(const char* s) {
    s = orEmpty(s);
    while (*s) add(*s++);
    return *this;
  }
  Text& addU(uint32_t v) {
    char digits[sizeof(uint32_t) * BITS_PER_BYTE];   // more than the 10 a uint32_t needs
    size_t n = 0;
    do {
      digits[n++] = (char)('0' + v % DEC_BASE);
      v /= DEC_BASE;
    } while (v && n < sizeof digits);
    while (n) add(digits[--n]);
    return *this;
  }
  Text& addI(int32_t v) {
    if (v < 0) {
      add('-');
      return addU((uint32_t)(-(v + 1)) + 1u);   // -(v + 1) + 1 cannot overflow on INT32_MIN
    }
    return addU((uint32_t)v);
  }
  Text& padTo(size_t width) {
    while (len_ < width && len_ < LCD_COLS) add(' ');
    return *this;
  }
  const char* c_str() const { return buf_; }
  size_t length() const { return len_; }
  bool cut() const { return cut_; }   // something did not fit LCD_COLS

 private:
  char buf_[LCD_COLS + 1];
  size_t len_ = 0;
  bool cut_ = false;
};

const char* modeLabel(uint8_t radioMode) {
  return radioMode == DS_RADIO_WIFI ? LCD_TXT_MODE_WIFI : LCD_TXT_MODE_BT;
}

const char* fireSuffix(uint8_t fireMode) {
  return fireMode == DS_FIRE_MODE_FULL ? LCD_TXT_FIRE_FULL : LCD_TXT_FIRE_SEMI;
}

const char* faultText(uint8_t fault) {
  switch (fault) {
    case DS_FAULT_STALL: return LCD_TXT_FAULT_STALL;
    case DS_FAULT_CAL: return LCD_TXT_FAULT_CAL;
    default: return LCD_TXT_FAULT_OTHER;
  }
}

// Columns `left` and `right` need side by side (layoutRow's rule: one space between, none
// when either is empty).
size_t needed(size_t left, size_t right) { return left + right + (left && right ? 1 : 0); }

// Row 0 in PAIRING / DISARMED / ARMED: arm state left, ammo right.
bool armRow(const DisplayModel& m, char out[LCD_COLS + 1]) {
  Text ammo;
  ammo.add(LCD_TXT_AMMO_PREFIX).addU(m.ammo);
  if (m.state != DS_STATE_ARMED) return Lcd::layoutRow(LCD_TXT_DISARMED, ammo.c_str(), out);
  const char* suffix = fireSuffix(m.fireMode);
  if (LCD_TXT_SHOW_FIRE_MODE &&
      needed(strlen(LCD_TXT_ARMED) + strlen(suffix), ammo.length()) <= LCD_COLS) {
    Text left;   // fits, so the Text never truncates it
    left.add(LCD_TXT_ARMED).add(suffix);
    return Lcd::layoutRow(left.c_str(), ammo.c_str(), out);
  }
  return Lcd::layoutRow(LCD_TXT_ARMED, ammo.c_str(), out);
}

// SERVO ADJUST rows: "SERVO ADJUST" left, the commanded angle right-justified; then
// "R:<rest> F:<fire>", the angles in force.
bool teachTopRow(const DisplayModel& m, char out[LCD_COLS + 1]) {
  Text deg;
  deg.addI(m.servoDeg);
  return Lcd::layoutRow(LCD_TXT_SERVO_ADJUST, deg.c_str(), out);
}

bool teachBottomRow(const DisplayModel& m, char out[LCD_COLS + 1]) {
  Text t;
  t.add(LCD_TXT_SERVO_REST).addI(m.restDeg).add(' ').add(LCD_TXT_SERVO_FIRE).addI(m.fireDeg);
  return Lcd::layoutRow(t.c_str(), "", out) && !t.cut();
}

// ---- the boot check: which configured strings will be cut ----
uint8_t warnCombo(Debug* debug, uint8_t row, const char* left, size_t leftLen, const char* right,
                  size_t rightLen, const char* consequence) {
  const size_t need = needed(leftLen, rightLen);
  if (need <= LCD_COLS) return 0;
  if (debug) {
    Debug::Line l;
    l.add("LCD row ").addU(row).add(": \"").add(left).add("\" + \"").add(right).add("\" = ")
        .addU((uint32_t)need).add(" > ").addU(LCD_COLS).add(" columns, ").add(consequence);
    debug->log(LOG_ERROR, l);
  }
  return 1;
}

uint8_t warnAlone(Debug* debug, const char* what, const char* text) {
  const size_t n = strlen(text);
  if (n <= LCD_COLS) return 0;
  if (debug) {
    Debug::Line l;
    l.add("LCD ").add(what).add(": \"").add(text).add("\" = ").addU((uint32_t)n).add(" > ")
        .addU(LCD_COLS).add(" columns, cut");
    debug->log(LOG_ERROR, l);
  }
  return 1;
}

uint8_t checkOverflows(Debug* debug) {
  uint8_t n = 0;
  n += warnAlone(debug, "LCD_TXT_OFF_ROW0", LCD_TXT_OFF_ROW0);
  n += warnAlone(debug, "LCD_TXT_OFF_ROW1", LCD_TXT_OFF_ROW1);
  n += warnAlone(debug, "LCD_TXT_CALIBRATING", LCD_TXT_CALIBRATING);
  n += warnAlone(debug, "LCD_TXT_FAULT_STALL", LCD_TXT_FAULT_STALL);
  n += warnAlone(debug, "LCD_TXT_FAULT_CAL", LCD_TXT_FAULT_CAL);
  n += warnAlone(debug, "LCD_TXT_FAULT_OTHER", LCD_TXT_FAULT_OTHER);
  n += warnAlone(debug, "LCD_TXT_MODE_BT", LCD_TXT_MODE_BT);
  n += warnAlone(debug, "LCD_TXT_MODE_WIFI", LCD_TXT_MODE_WIFI);
  n += warnAlone(debug, "LCD_TXT_SERVO_ADJUST", LCD_TXT_SERVO_ADJUST);

  // Row 0: the arm texts beside the widest ammo text a full magazine gives.
  Text ammo;
  ammo.add(LCD_TXT_AMMO_PREFIX).addU(MAG_CAPACITY);
  const size_t ammoLen = strlen(LCD_TXT_AMMO_PREFIX) + decimalDigits(MAG_CAPACITY);
  n += warnCombo(debug, ROW_TOP, LCD_TXT_DISARMED, strlen(LCD_TXT_DISARMED), ammo.c_str(), ammoLen,
                 "ammo text cut");
  n += warnCombo(debug, ROW_TOP, LCD_TXT_ARMED, strlen(LCD_TXT_ARMED), ammo.c_str(), ammoLen,
                 "ammo text cut");
  if (LCD_TXT_SHOW_FIRE_MODE) {
    const char* suffixes[] = {LCD_TXT_FIRE_SEMI, LCD_TXT_FIRE_FULL};
    for (const char* suffix : suffixes) {
      Text armed;
      armed.add(LCD_TXT_ARMED).add(suffix);
      n += warnCombo(debug, ROW_TOP, armed.c_str(), strlen(LCD_TXT_ARMED) + strlen(suffix),
                     ammo.c_str(), ammoLen, "fire-mode suffix not shown");
    }
  }

  // SERVO ADJUST row 0: the adjust text beside the widest angle (every angle is 0..SERVO_SPAN_DEG).
  Text widest;
  widest.addU(SERVO_SPAN_DEG);
  const size_t degLen = decimalDigits(SERVO_SPAN_DEG);
  n += warnCombo(debug, ROW_TOP, LCD_TXT_SERVO_ADJUST, strlen(LCD_TXT_SERVO_ADJUST), widest.c_str(), degLen,
                 "angle cut");
  // SERVO ADJUST row 1: "R:<n> F:<n>" at its widest.
  const size_t teachRow1 = strlen(LCD_TXT_SERVO_REST) + degLen + 1 + strlen(LCD_TXT_SERVO_FIRE) + degLen;
  if (teachRow1 > LCD_COLS) {
    if (debug) {
      Debug::Line l;
      l.add("LCD row 1: \"").add(LCD_TXT_SERVO_REST).add(widest.c_str()).add(' ').add(LCD_TXT_SERVO_FIRE)
          .add(widest.c_str()).add("\" = ").addU((uint32_t)teachRow1).add(" > ").addU(LCD_COLS)
          .add(" columns, servo adjust angles cut");
      debug->log(LOG_ERROR, l);
    }
    n++;
  }

  // Row 1: each mode label beside each status text (the pairing text at its widest).
  Text pairing;
  pairing.add(LCD_TXT_PAIRING).add(' ').add(kDots[kDotPatterns - 1]);
  const char* statuses[] = {LCD_TXT_CONNECTED, pairing.c_str(), LCD_TXT_NO_CONNECTION};
  const size_t statusLens[] = {strlen(LCD_TXT_CONNECTED), pairingWidth(),
                               strlen(LCD_TXT_NO_CONNECTION)};
  const char* modes[] = {LCD_TXT_MODE_BT, LCD_TXT_MODE_WIFI};
  for (const char* mode : modes) {
    for (size_t i = 0; i < sizeof statuses / sizeof statuses[0]; i++) {
      n += warnCombo(debug, ROW_BOTTOM, mode, strlen(mode), statuses[i], statusLens[i],
                     "right text cut");
    }
  }
  return n;
}

}  // namespace

// ---- pure layout ----------------------------------------------------------------------------------
bool Lcd::layoutRow(const char* left, const char* right, char out[LCD_COLS + 1]) {
  left = orEmpty(left);
  right = orEmpty(right);
  memset(out, ' ', LCD_COLS);
  out[LCD_COLS] = 0;
  bool fits = true;
  size_t l = strlen(left);
  size_t r = strlen(right);
  if (l > LCD_COLS) {
    l = LCD_COLS;
    fits = false;
  }
  memcpy(out, left, l);
  if (r == 0) return fits;
  const size_t gap = l ? 1 : 0;
  const size_t room = l + gap < LCD_COLS ? LCD_COLS - l - gap : 0;
  if (r > room) {
    r = room;   // keep the first `room` characters: the right text is cut from its end
    fits = false;
  }
  memcpy(out + LCD_COLS - r, right, r);
  return fits;
}

void Lcd::pairingText(uint32_t elapsedMs, char out[LCD_COLS + 1]) {
  const uint8_t phase = (uint8_t)((elapsedMs / LCD_PAIRING_DOT_MS) % kDotPatterns);
  Text t;
  t.add(LCD_TXT_PAIRING).add(' ').add(kDots[phase]).padTo(pairingWidth());
  memcpy(out, t.c_str(), t.length() + 1);
}

uint8_t Lcd::compose(const DisplayModel& m, uint32_t nowMs, char rows[LCD_ROWS][LCD_COLS + 1],
                     bool& backlight) {
  for (uint8_t r = 0; r < LCD_ROWS; r++) layoutRow("", "", rows[r]);
  backlight = true;
  const char* mode = modeLabel(m.radioMode);
  const char* link = m.linkConnected ? LCD_TXT_CONNECTED : LCD_TXT_NO_CONNECTION;
  bool top = true;
  bool bottom = true;
  switch (m.state) {
    case DS_STATE_OFF:
      backlight = !LCD_BACKLIGHT_OFF_WHEN_SYSTEM_OFF;
      top = layoutRow(LCD_TXT_OFF_ROW0, "", rows[ROW_TOP]);
      bottom = layoutRow(LCD_TXT_OFF_ROW1, "", rows[ROW_BOTTOM]);
      break;
    case DS_STATE_BOOT:
    case DS_STATE_CALIBRATING:
      top = layoutRow(LCD_TXT_CALIBRATING, "", rows[ROW_TOP]);
      bottom = layoutRow(mode, "", rows[ROW_BOTTOM]);
      break;
    case DS_STATE_PAIRING:
      top = armRow(m, rows[ROW_TOP]);
      if (m.pairing == PairingPhase::NO_LINK) {
        bottom = layoutRow(mode, LCD_TXT_NO_CONNECTION, rows[ROW_BOTTOM]);
      } else {   // FROM_BOOT and GRACE: dots from the moment PAIRING was entered
        char status[LCD_COLS + 1];
        pairingText(nowMs - m.stateSinceMs, status);
        bottom = layoutRow(mode, status, rows[ROW_BOTTOM]);
      }
      break;
    case DS_STATE_DISARMED:
    case DS_STATE_ARMED:
      if (m.servoTeach && m.state == DS_STATE_DISARMED) {
        top = teachTopRow(m, rows[ROW_TOP]);
        bottom = teachBottomRow(m, rows[ROW_BOTTOM]);
        break;
      }
      top = armRow(m, rows[ROW_TOP]);
      bottom = layoutRow(mode, link, rows[ROW_BOTTOM]);
      break;
    case DS_STATE_FAULT:
      top = layoutRow(faultText(m.fault), "", rows[ROW_TOP]);
      bottom = layoutRow(mode, link, rows[ROW_BOTTOM]);
      break;
    default:
      break;   // not a state: blank rows
  }
  return (uint8_t)((top ? 0 : 1) + (bottom ? 0 : 1));
}

uint8_t Lcd::overflowCount() { return checkOverflows(nullptr); }

// ---- the module ---------------------------------------------------------------------------------------
void Lcd::begin(Debug& debug) {
  debug_ = &debug;
  for (uint8_t r = 0; r < LCD_ROWS; r++) {
    layoutRow("", "", want_[r]);
    layoutRow("", "", shown_[r]);   // LiquidCrystal_I2C::init() leaves the display clear
  }
  backlightWant_ = true;
  backlightShown_ = true;   // hal::begin() switched it on with the LCD
  composed_ = false;
  lastLayoutMs_ = 0;
  cursorRow_ = 0;           // init() ends with the cursor home
  cursorCol_ = 0;
  inSync_ = true;
  charsWritten_ = 0;
  transactions_ = 0;
  cursorMoves_ = 0;
  layoutCuts_ = 0;
  lastRefreshMs_ = hal::nowMs();
  refreshRow_ = 0;
  refreshCol_ = 0;
  refreshPending_ = false;
  pendingRow_ = 0;
  pendingCol_ = 0;
  refreshWrites_ = 0;
  waitRetry_ = false;
  failMs_ = 0;
  failures_ = 0;
  overflowWarnings_ = checkOverflows(debug_);
}

void Lcd::markRefreshCell(uint32_t nowMs) {
  shown_[refreshRow_][refreshCol_] = kUnknownCell;
  refreshPending_ = true;
  pendingRow_ = refreshRow_;
  pendingCol_ = refreshCol_;
  if (++refreshCol_ >= LCD_COLS) {
    refreshCol_ = 0;
    if (++refreshRow_ >= LCD_ROWS) refreshRow_ = 0;
  }
  lastRefreshMs_ = nowMs;
  inSync_ = false;
}

void Lcd::onFailure(uint32_t nowMs) {
  failures_++;
  for (uint8_t r = 0; r < LCD_ROWS; r++) memset(shown_[r], kUnknownCell, LCD_COLS);
  backlightShown_ = !backlightWant_;   // send it again too
  refreshPending_ = false;
  cursorRow_ = 0;
  cursorCol_ = 0;
  inSync_ = false;
  waitRetry_ = true;
  failMs_ = nowMs;
  if (debug_ && failures_ == 1) debug_->log(LOG_ERROR, "lcd: I2C transaction failed: redrawing after a re-init");
}

bool Lcd::anyDiff() const {
  for (uint8_t r = 0; r < LCD_ROWS; r++) {
    if (memcmp(want_[r], shown_[r], LCD_COLS) != 0) return true;
  }
  return false;
}

bool Lcd::nextDiff(uint8_t& row, uint8_t& col) const {
  // The character under the cursor first: writing it needs no cursor command.
  if (cursorRow_ < LCD_ROWS && cursorCol_ < LCD_COLS &&
      want_[cursorRow_][cursorCol_] != shown_[cursorRow_][cursorCol_]) {
    row = cursorRow_;
    col = cursorCol_;
    return true;
  }
  for (uint8_t r = 0; r < LCD_ROWS; r++) {
    for (uint8_t c = 0; c < LCD_COLS; c++) {
      if (want_[r][c] != shown_[r][c]) {
        row = r;
        col = c;
        return true;
      }
    }
  }
  return false;
}

void Lcd::update(uint32_t nowMs, const DisplayModel& model) {
  bool recomposed = false;
  if (!composed_ || (uint32_t)(nowMs - lastLayoutMs_) >= LCD_REFRESH_MS) {
    layoutCuts_ += compose(model, nowMs, want_, backlightWant_);
    lastLayoutMs_ = nowMs;
    composed_ = true;
    recomposed = true;
  }

  if (!hal::lcdPresent()) {   // nothing to write to: the rows still feed the telemetry mirror
    memcpy(shown_, want_, sizeof shown_);
    backlightShown_ = backlightWant_;
    inSync_ = true;
    return;
  }
  if (waitRetry_) {           // a transaction failed: leave the LCD alone for a while
    if ((uint32_t)(nowMs - failMs_) < LCD_RETRY_MS) return;
    waitRetry_ = false;
  }
  if (!recomposed && inSync_) {
    // The wanted rows only change in compose(), so nothing new is due, except the slow
    // round-robin rewrite of one cell that heals a display gone wrong behind our back.
    if ((uint32_t)(nowMs - lastRefreshMs_) < LCD_REFRESH_MS) return;
    markRefreshCell(nowMs);
  }

  // At most one I2C transaction from here on.
  if (backlightShown_ != backlightWant_) {
    transactions_++;
    if (!hal::lcdBacklight(backlightWant_)) {
      onFailure(nowMs);
      return;
    }
    backlightShown_ = backlightWant_;
    inSync_ = !anyDiff();
    return;
  }
  uint8_t r = 0;
  uint8_t c = 0;
  if (!nextDiff(r, c)) {
    inSync_ = true;
    return;
  }
  transactions_++;
  const hal::LcdWrite w = hal::lcdWriteAt(c, r, want_[r][c]);
  if (w == hal::LcdWrite::WROTE) {
    shown_[r][c] = want_[r][c];
    charsWritten_++;
    if (refreshPending_ && r == pendingRow_ && c == pendingCol_) {
      refreshPending_ = false;
      refreshWrites_++;
    }
    cursorRow_ = r;
    cursorCol_ = (uint8_t)(c + 1);   // the display advanced its cursor
  } else if (w == hal::LcdWrite::AGAIN) {
    cursorRow_ = r;                  // the call only moved the cursor; write it next time
    cursorCol_ = c;
    cursorMoves_++;
  } else {
    onFailure(nowMs);
    return;
  }
  inSync_ = !anyDiff();
}

void Lcd::mirror(char out[DS_TELEMETRY_LCD_LEN]) const {
  for (uint8_t r = 0; r < LCD_ROWS; r++) memcpy(out + r * LCD_COLS, want_[r], LCD_COLS);
}

bool Lcd::present() const { return hal::lcdPresent(); }

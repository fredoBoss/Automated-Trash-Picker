

/*
 * 16x2 status LCD on a PCF8574 I2C backpack, SDA 20 / SCL 21.
 *
 * Row 0 is what the Mega is doing, set where it happens: the start-up
 * checks, the countdown, a bench command, an avoidance turn, a pickup.
 * Row 1 is the Pi, rewritten every loop by lcdService() from when it was
 * last heard and what it last reported.
 *
 * The screen is optional. If no backpack answers at start-up, every
 * function here returns at once and the rover runs exactly as without it.
 */

const uint8_t LCD_COLS = 16;
const uint8_t LCD_ROWS = 2;

// serial_link.py sends P once a second, so three missed in a row mean the
// Pi has stopped talking - crashed, unplugged, or vision.py has exited.
const uint32_t PI_TIMEOUT_MS = 3000;

const uint16_t CHECK_SHOW_MS = 400;    // a passing start-up check stays up this long
const uint16_t CHECK_FAIL_MS = 2000;   // a failing one, long enough to read

uint8_t lcdAddress = 0;                // 0 = no screen found
LiquidCrystal_I2C *lcd = nullptr;
char lcdShown[LCD_ROWS][LCD_COLS + 1]; // what each row shows right now

/*
 * Find the backpack and start it. A PCF8574 answers somewhere in 0x20-0x27
 * and a PCF8574A in 0x38-0x3F, depending on its solder jumpers, so the
 * address is looked for rather than configured. The Wire timeout keeps a
 * stuck bus - a loose SDA wire - from hanging loop(), and with it the
 * obstacle avoidance; a write that times out is just lost.
 */
void lcdSetup() {
  Wire.begin();
  Wire.setWireTimeout(25000, true);    // 25 ms, then reset the bus

  for (uint8_t addr = 0x20; addr <= 0x3F && lcdAddress == 0; addr++) {
    if (addr >= 0x28 && addr < 0x38) continue;   // between the two ranges
    Wire.beginTransmission(addr);
    if (Wire.endTransmission() == 0) lcdAddress = addr;
  }
  if (lcdAddress == 0) return;

  static LiquidCrystal_I2C screen(lcdAddress, LCD_COLS, LCD_ROWS);
  lcd = &screen;
  lcd->init();                         // leaves the screen blank
  lcd->backlight();
  for (uint8_t r = 0; r < LCD_ROWS; r++) {
    memset(lcdShown[r], ' ', LCD_COLS);
    lcdShown[r][LCD_COLS] = '\0';
  }
  lcdRow(0, F("RoboSort"));
  lcdRow(1, F("starting..."));
}

// Start-up report, with the other INIT lines in setup().
void printLcdCheck() {
  Serial.print(F("INIT lcd    - "));
  if (!lcd) {
    Serial.println(F("none found on SDA 20 / SCL 21, running without it"));
    return;
  }
  Serial.print(F("16x2 at 0x"));
  Serial.println(lcdAddress, HEX);
}

/*
 * Show text on one row, padded to the full width so nothing of the old
 * text is left behind. Only the characters that changed are sent: each
 * one is several I2C writes, about a millisecond, and loop() calls this
 * every pass with mostly the same text.
 */
void lcdRow(uint8_t row, const char *text) {
  if (!lcd) return;

  char next[LCD_COLS + 1];
  uint8_t n = 0;
  for (; n < LCD_COLS && text[n]; n++) next[n] = text[n];
  for (; n < LCD_COLS; n++) next[n] = ' ';
  next[LCD_COLS] = '\0';

  char *shown = lcdShown[row];
  uint8_t first = 0, last = LCD_COLS;
  while (first < LCD_COLS && next[first] == shown[first]) first++;
  if (first == LCD_COLS) return;       // already showing exactly this
  while (next[last - 1] == shown[last - 1]) last--;

  lcd->setCursor(first, row);
  for (uint8_t i = first; i < last; i++) lcd->write(next[i]);
  memcpy(shown, next, LCD_COLS);
}

void lcdRow(uint8_t row, const __FlashStringHelper *text) {
  if (!lcd) return;
  char buf[LCD_COLS + 1];
  strncpy_P(buf, (PGM_P)text, LCD_COLS);
  buf[LCD_COLS] = '\0';
  lcdRow(row, buf);
}

// One start-up check on row 1: the part on the left, the result on the
// right, held long enough to read. A failure is held longer.
void lcdCheck(const __FlashStringHelper *part, const __FlashStringHelper *result, bool ok) {
  if (!lcd) return;
  char row[LCD_COLS + 1];
  snprintf_P(row, sizeof(row), PSTR("%-9S%7S"), (PGM_P)part, (PGM_P)result);
  lcdRow(1, row);
  delay(ok ? CHECK_SHOW_MS : CHECK_FAIL_MS);
}

void lcdCountdown(uint8_t seconds) {
  if (!lcd) return;
  char row[LCD_COLS + 1];
  snprintf_P(row, sizeof(row), PSTR("INIT %2us to go"), seconds);
  lcdRow(0, row);
}

// Row 0 during a pickup: which bin the item is headed for, and the step.
void lcdPickup(char command, const __FlashStringHelper *step) {
  if (!lcd) return;
  char row[LCD_COLS + 1];
  snprintf_P(row, sizeof(row), PSTR("%-7S %S"), (PGM_P)classLabel(command), (PGM_P)step);
  lcdRow(0, row);
}

const __FlashStringHelper *classLabel(char command) {
  switch (command) {
    case CMD_BIO:    return F("BIO");
    case CMD_NONBIO: return F("NON-BIO");
    default:         return F("nothing");
  }
}

/*
 * Row 1, every loop: is the Pi talking, and what did it last report.
 * Opening the port resets the Mega, so a running vision.py is heard within
 * a second of boot. Still silent when the countdown ends means no Pi;
 * silent for PI_TIMEOUT_MS after being heard means it went away.
 */
void lcdService() {
  if (!lcd) return;
  char row[LCD_COLS + 1];
  if (!piHeard) {
    strcpy_P(row, autoStartPending ? PSTR("Pi: checking...") : PSTR("Pi: not found"));
  } else if (millis() - piHeardAt > PI_TIMEOUT_MS) {
    strcpy_P(row, PSTR("Pi: lost"));
  } else {
    snprintf_P(row, sizeof(row), PSTR("Pi OK  %S"), (PGM_P)classLabel(lastSeen));
  }
  lcdRow(1, row);
}

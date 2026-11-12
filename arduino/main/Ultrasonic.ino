

const uint8_t IDX_LEFT  = 0;
const uint8_t IDX_RIGHT = 1;

const uint32_t ECHO_TIMEOUT_US = 12000;   // ~2 m of round trip
const uint16_t MAX_RANGE_CM    = 200;

// From one ping to the next, whichever sensor sends it. pulseIn() gives up
// at 12 ms, but the echo off a wall 2-4 m away is still in the air until
// ~23 ms, and a ping sent into it reads that stray echo as something close.
// Alternating, each sensor then pings every 60 ms - the HC-SR04's own
// minimum cycle.
const uint8_t  PING_GAP_MS     = 30;
static uint32_t lastPingAt     = 0;       // millis() of the last ping, either sensor

void ultrasonicSetup() {
  for (uint8_t i = 0; i < sizeof(PIN_TRIG); i++) {
    pinMode(PIN_TRIG[i], OUTPUT);
    pinMode(PIN_ECHO[i], INPUT);
    digitalWrite(PIN_TRIG[i], LOW);
  }
  delay(50);
}

/*
 * Distance in cm, or MAX_RANGE_CM when nothing echoes back. Out of range
 * is reported as far, never as 0 - a 0 from a timeout would read as an
 * imminent collision and lock the rover up.
 */
uint16_t readDistance(uint8_t index) {
  while (millis() - lastPingAt < PING_GAP_MS) {}
  lastPingAt = millis();

  digitalWrite(PIN_TRIG[index], LOW);
  delayMicroseconds(2);
  digitalWrite(PIN_TRIG[index], HIGH);
  delayMicroseconds(10);
  digitalWrite(PIN_TRIG[index], LOW);

  uint32_t duration = pulseIn(PIN_ECHO[index], HIGH, ECHO_TIMEOUT_US);
  if (duration == 0) return MAX_RANGE_CM;

  uint16_t cm = duration / 58;            // 343 m/s, there and back
  if (cm == 0 || cm > MAX_RANGE_CM) return MAX_RANGE_CM;
  return cm;
}

// readDistance() spaces the pings, so the right sensor's echo has died out
// before the next call's left ping too - not only between left and right.
void readAllDistances() {
  distLeft  = readDistance(IDX_LEFT);
  distRight = readDistance(IDX_RIGHT);
}

/*
 * Is a sensor wired to this pair at all? readDistance() cannot say - an
 * unplugged sensor and an empty room both come back as MAX_RANGE_CM, and
 * a side that always reads far is a side the rover drives into. A wired
 * HC-SR04 holds ECHO low between pings and raises it within a millisecond
 * or two of a trigger, whatever is in front of it. The pull-up makes an
 * unplugged ECHO pin read high instead of floating, so it cannot pass the
 * first test by chance.
 */
bool sonarPresent(uint8_t index) {
  uint8_t echo = PIN_ECHO[index];
  pinMode(echo, INPUT_PULLUP);
  delayMicroseconds(100);
  bool present = digitalRead(echo) == LOW;

  if (present) {
    digitalWrite(PIN_TRIG[index], HIGH);
    delayMicroseconds(10);
    digitalWrite(PIN_TRIG[index], LOW);

    uint32_t start = micros();
    while (digitalRead(echo) == LOW) {
      if (micros() - start > 5000) {
        present = false;
        break;
      }
    }
    // Let this ping run out - ~40 ms with nothing in range - so the next
    // reading does not start on its tail.
    start = millis();
    while (digitalRead(echo) == HIGH && millis() - start < 250) {}
  }

  pinMode(echo, INPUT);
  return present;
}

static void printSonar(bool present, uint16_t cm) {
  if (!present) {
    Serial.print(F("NOT CONNECTED"));
    return;
  }
  Serial.print(cm);
  Serial.print(cm == MAX_RANGE_CM ? F(" cm (no echo - nothing within 2 m)") : F(" cm"));
}

// Start-up report: is each sensor there, and what does it read. One serial
// line for both, one LCD line each.
void printSensorCheck() {
  bool okLeft  = sonarPresent(IDX_LEFT);
  bool okRight = sonarPresent(IDX_RIGHT);
  readAllDistances();

  Serial.print(F("INIT sensors - left "));
  printSonar(okLeft, distLeft);
  Serial.print(F(", right "));
  printSonar(okRight, distRight);
  Serial.println();

  lcdCheck(F("Sonar L"), okLeft  ? F("OK") : F("MISSING"), okLeft);
  lcdCheck(F("Sonar R"), okRight ? F("OK") : F("MISSING"), okRight);
}


const uint16_t REVERSE_PAUSE_MS = 200;

static int8_t   lastDirL = 0, lastDirR = 0;   // way each side last ran, 0 = never
static bool     powered = false;              // a motor is running right now
static uint32_t lastPoweredMs = 0;            // last moment a motor was running

// Direction helpers ------------------------------------------------
static void relaySet(uint8_t pin, bool on) {
  if (DRIVE_RELAY_ACTIVE_LOW) on = !on;
  digitalWrite(pin, on ? HIGH : LOW);
}

// One side. dir is +1 forward, -1 backward, 0 stop.
static void sideWrite(bool left, int8_t dir, uint8_t speed) {
  if (DRIVE_BY_RELAY) {
    relaySet(left ? PIN_L_IN1 : PIN_R_IN1, dir > 0);
    relaySet(left ? PIN_L_IN2 : PIN_R_IN2, dir < 0);
  } else {
    // PWM on one input, the other held low. Both low is the stop: the
    // BTS7960 then grounds both motor wires, which brakes the wheel.
    analogWrite(left ? PIN_L_RPWM : PIN_R_RPWM, dir > 0 ? speed : 0);
    analogWrite(left ? PIN_L_LPWM : PIN_R_LPWM, dir < 0 ? speed : 0);
  }
}

// Both sides at once. Everything below goes through here, so the
// reversal pause cannot be skipped by a new caller.
static void drive(int8_t dirL, uint8_t speedL, int8_t dirR, uint8_t speedR) {
  if (speedL == 0) dirL = 0;
  if (speedR == 0) dirR = 0;

  if (powered) lastPoweredMs = millis();
  bool flip = (dirL != 0 && dirL == -lastDirL) ||
              (dirR != 0 && dirR == -lastDirR);
  uint32_t idle = millis() - lastPoweredMs;
  if (flip && idle < REVERSE_PAUSE_MS) {
    sideWrite(true,  0, 0);
    sideWrite(false, 0, 0);
    delay(REVERSE_PAUSE_MS - idle);
  }

  sideWrite(true,  dirL, speedL);
  sideWrite(false, dirR, speedR);

  if (dirL != 0) lastDirL = dirL;
  if (dirR != 0) lastDirR = dirR;
  powered = (dirL != 0 || dirR != 0);
}

// Public API -------------------------------------------------------
/*
 * Pins are written "stopped" before they become outputs, so no relay
 * clicks on at boot - the same trick as the conveyor relay. With the
 * BTS7960, analogWrite() makes the PWM pins outputs itself, and the
 * boards are enabled only once those are held low.
 */
void motorSetup() {
  driveStop();
  if (DRIVE_BY_RELAY) {
    const uint8_t pins[] = {PIN_L_IN1, PIN_L_IN2, PIN_R_IN1, PIN_R_IN2};
    for (uint8_t i = 0; i < sizeof(pins); i++) pinMode(pins[i], OUTPUT);
  } else {
    pinMode(PIN_L_EN, OUTPUT);
    pinMode(PIN_R_EN, OUTPUT);
    digitalWrite(PIN_L_EN, HIGH);
    digitalWrite(PIN_R_EN, HIGH);
  }
}

void driveForward(uint8_t speed) {
  drive(1, speed, 1, speed);
}

void driveBackward(uint8_t speed) {
  drive(-1, speed, -1, speed);
}

// Spin left: left motor (back-left) backward, right motor (front-right)
// forward - counter-clockwise seen from above.
void driveTurnLeft(uint8_t speed) {
  drive(-1, speed, 1, speed);
}

// Spin right: left motor forward, right motor backward - clockwise.
void driveTurnRight(uint8_t speed) {
  drive(1, speed, -1, speed);
}

void driveStop() {
  drive(0, 0, 0, 0);
}

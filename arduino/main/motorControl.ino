
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
static void sideWrite(bool left, int8_t dir) {
  relaySet(left ? PIN_L_IN1 : PIN_R_IN1, dir > 0);
  relaySet(left ? PIN_L_IN2 : PIN_R_IN2, dir < 0);
}

// Both sides at once. Everything below goes through here, so the
// reversal pause cannot be skipped by a new caller.
static void drive(int8_t dirL, int8_t dirR) {
  if (powered) lastPoweredMs = millis();
  bool flip = (dirL != 0 && dirL == -lastDirL) ||
              (dirR != 0 && dirR == -lastDirR);
  uint32_t idle = millis() - lastPoweredMs;
  if (flip && idle < REVERSE_PAUSE_MS) {
    sideWrite(true,  0);
    sideWrite(false, 0);
    delay(REVERSE_PAUSE_MS - idle);
  }

  sideWrite(true,  dirL);
  sideWrite(false, dirR);

  if (dirL != 0) lastDirL = dirL;
  if (dirR != 0) lastDirR = dirR;
  powered = (dirL != 0 || dirR != 0);
}

// Public API -------------------------------------------------------
/*
 * Pins are written "stopped" before they become outputs, so no relay
 * clicks on at boot - the same trick as the conveyor relay.
 */
void motorSetup() {
  driveStop();
  const uint8_t pins[] = {PIN_L_IN1, PIN_L_IN2, PIN_R_IN1, PIN_R_IN2};
  for (uint8_t i = 0; i < sizeof(pins); i++) pinMode(pins[i], OUTPUT);
}

void driveForward() {
  drive(1, 1);
}

void driveBackward() {
  drive(-1, -1);
}

// Spin left: left motor (back-left) backward, right motor (front-right)
// forward - counter-clockwise seen from above.
void driveTurnLeft() {
  drive(-1, 1);
}

/*
 * Turn right while moving forward: left motor (back-left) forward, right
 * motor (front-right) stopped - its relays off short the motor, so the
 * wheel brakes and the rover pivots clockwise around it.
 * Not a spin: with the right motor backward the rover crept backward on
 * the floor. The front-right wheel grips better than the back-left (the
 * arm, scoop and belt load the front), so its backward push won. Turn left
 * does not have this: there the front-right wheel drives forward.
 */
void driveTurnRight() {
  drive(1, 0);
}

void driveStop() {
  drive(0, 0);
}

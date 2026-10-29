

Servo servoLiftL;
Servo servoLiftR;
Servo servoSort;
Servo servoPan;

/*
 * MG996R covers its full 180 deg over roughly 500-2500 us. The library's
 * default range is 544-2400, which clips both ends, so the range is given
 * to attach() explicitly - without it write(0) and write(180) fall short
 * of the real travel.
 */
const uint16_t SERVO_MIN_US = 500;
const uint16_t SERVO_MAX_US = 2500;

// ------------------------------------------------------------- angles
/*
 * The one reset angle, mid-travel on a 180 deg servo. It is also where the
 * library parks a channel on attach(), so nothing snaps to an unknown
 * position at boot, and it is the fixed point of the lift mirroring below
 * - 180 - 90 is 90, so the pair agrees on home whichever way it is wired.
 */
const uint8_t SERVO_HOME = 90;

/*
 * The lift only goes up and down, and down is home: the arm rests at 90,
 * where attach() parks it. Lower angle is up on this linkage, so up is
 * below 90 - 60 deg of travel, the same swing as the old 120 down / 60 up.
 */
const uint8_t LIFT_DOWN   = SERVO_HOME;   // 90, arm down - the rest position
const uint8_t LIFT_UP     = 30;           // arm raised, item scooped up to the belt
const uint8_t SORT_BIO    = 40;
const uint8_t SORT_NONBIO = 140;

/*
 * Camera pan: home is straight ahead, the only direction the arm can
 * reach, and it turns 45 deg either way from there. On most servos seen
 * from the horn side, a larger angle turns anticlockwise - to the left.
 * If LOOKLEFT turns the camera right, swap PAN_LEFT and PAN_RIGHT.
 */
const uint8_t PAN_HOME  = SERVO_HOME;   // 90, looking straight ahead
const uint8_t PAN_LEFT  = 135;          // home + 45
const uint8_t PAN_RIGHT = 45;           // home - 45


const int8_t LIFT_R_TRIM = 0;   // degrees, applied to the right lift servo


const uint16_t SERVO_SETTLE_MS = 600;


const uint8_t  LIFT_STEP_DEG  = 2;
const uint8_t  LIFT_STEP_MS   = 15;
const uint16_t LIFT_HOLD_MS   = 150;    // after the ramp, let the arm stop swinging

// Last angle sent to the lift pair. attach() parks it at 90, so it starts there.
uint8_t liftAngle = SERVO_HOME;

/*
 * An SG90 is about 0.1 s per 60 deg, so the widest pan move (left to
 * right, 90 deg) takes ~0.15 s. 300 ms adds time for the camera on the
 * mast to stop wobbling before the next frame is worth classifying.
 */
const uint16_t PAN_SETTLE_MS = 300;
uint8_t panAngle = PAN_HOME;          // last angle sent to the pan servo
const uint16_t CONVEYOR_RUN_MS = 5000;  // belt time from the arm to the bin

void servoSetup() {
  // attach() starts a channel at 90, and 90 is every servo's rest angle -
  // lift down, gate and camera centred - so none needs a pulse loaded first.
  servoLiftL.attach(PIN_SERVO_LIFT_L, SERVO_MIN_US, SERVO_MAX_US);
  servoLiftR.attach(PIN_SERVO_LIFT_R, SERVO_MIN_US, SERVO_MAX_US);
  servoSort.attach(PIN_SERVO_SORT,    SERVO_MIN_US, SERVO_MAX_US);
  servoPan.attach(PIN_SERVO_PAN,      SERVO_MIN_US, SERVO_MAX_US);

  // Relay pin written to "off" before it becomes an output, so the belt
  // cannot twitch on for an instant at boot.
  conveyorStop();
  pinMode(PIN_CONV_RELAY, OUTPUT);

  servoResetAll();
}

/*
 * Drive the lift pair from one logical angle. Everything else in this file
 * talks to the joint through here, so the pairing lives in exactly one
 * place. The trim is clamped to the servo's 0-180 range so a large offset
 * cannot wrap a uint8_t and fling the arm to the far end.
 */
void liftWrite(uint8_t angle) {
  servoLiftL.write(angle);
  servoLiftR.write(constrain((int)angle + LIFT_R_TRIM, 0, 180));
  liftAngle = angle;
}

// Walk the lift pair to target a few degrees at a time, then hold briefly.
// Blocks until the arm is there, the same contract as the settle delays.
void liftMoveTo(uint8_t target) {
  while (liftAngle != target) {
    uint8_t next;
    if (liftAngle < target) {
      next = (target - liftAngle > LIFT_STEP_DEG) ? liftAngle + LIFT_STEP_DEG : target;
    } else {
      next = (liftAngle - target > LIFT_STEP_DEG) ? liftAngle - LIFT_STEP_DEG : target;
    }
    liftWrite(next);
    delay(LIFT_STEP_MS);
  }
  delay(LIFT_HOLD_MS);
}

// Park every servo, gate and camera included, at its rest angle: 90 for
// all of them, which leaves the arm down.
void servoResetAll() {
  servoSort.write(SERVO_HOME);
  servoPan.write(PAN_HOME);
  panAngle = PAN_HOME;
  liftMoveTo(LIFT_DOWN);
  delay(SERVO_SETTLE_MS);
}

/*
 * The arm rests down, so a pickup is one lift: raise it to scoop the item
 * up to the belt, belt it to the bin area, then lower the arm back to
 * rest. The gate is sequenced separately by sortTo(). Blocking on purpose
 * - the rover is stationary for this and nothing should interleave with a
 * raised arm.
 */
void runPickupSequence() {
  pickupBusy = true;

  liftMoveTo(LIFT_UP);              // scoop

  conveyorPulse();

  liftMoveTo(LIFT_DOWN);            // back to rest
  pickupBusy = false;
}

// Set the gate from a protocol byte. Anything other than B/N leaves it at
// the reset angle rather than picking a bin at random.
void sortTo(char command) {
  if (command == CMD_BIO) {
    servoSort.write(SORT_BIO);
  } else if (command == CMD_NONBIO) {
    servoSort.write(SORT_NONBIO);
  } else {
    servoSort.write(SERVO_HOME);
  }
  delay(SERVO_SETTLE_MS);

  servoSort.write(SERVO_HOME);      // let the item fall, then back to reset
  delay(SERVO_SETTLE_MS);
}


void armLiftUp() {
  liftMoveTo(LIFT_UP);
}

void armLiftDown() {
  liftMoveTo(LIFT_DOWN);
}

// ------------------------------------------------------- camera pan
// Same contract: each returns once the camera has stopped moving. A move
// to where the camera already points returns at once.
void panTo(uint8_t angle) {
  if (angle == panAngle) return;
  servoPan.write(angle);
  panAngle = angle;
  delay(PAN_SETTLE_MS);
}

void cameraLookLeft()  { panTo(PAN_LEFT);  }
void cameraLookRight() { panTo(PAN_RIGHT); }
void cameraLookAhead() { panTo(PAN_HOME);  }

// For the STATUS line - main.ino cannot read panAngle itself.
uint8_t cameraAngle() {
  return panAngle;
}

/*
 * Bench check for the lift pair, called by hand from setup() - never from
 * loop(). Both horns should travel together and sit level at 90 (down).
 * If they strain against each other while stopped, cut power and adjust
 * LIFT_R_TRIM; if one travels the wrong way, it is mounted backwards.
 */
void servoSweepTest() {
  liftMoveTo(LIFT_DOWN);
  liftMoveTo(LIFT_UP);
  liftMoveTo(LIFT_DOWN);
}

// --------------------------------------------------------- conveyor
// On/off only - a relay cannot set a speed.
void conveyorRun() {
  digitalWrite(PIN_CONV_RELAY, CONV_RELAY_ACTIVE_LOW ? LOW : HIGH);
}

void conveyorStop() {
  digitalWrite(PIN_CONV_RELAY, CONV_RELAY_ACTIVE_LOW ? HIGH : LOW);
}

// Reads back the level the relay pin is driven to.
bool conveyorIsOn() {
  return digitalRead(PIN_CONV_RELAY) == (CONV_RELAY_ACTIVE_LOW ? LOW : HIGH);
}

// Run the belt long enough to carry one item to the bin, then stop.
void conveyorPulse() {
  conveyorRun();
  delay(CONVEYOR_RUN_MS);
  conveyorStop();
}

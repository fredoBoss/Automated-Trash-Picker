

Servo servoLiftL;
Servo servoLiftR;
Servo servoSort;

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
 * The gate's reset angle, mid-travel on a 180 deg servo. It is also where the library parks a channel on attach(), so
 * nothing snaps to an unknown position at boot.
 */
const uint8_t SERVO_HOME = 90;

/*
 * The lift only goes up and down, and down is home. Higher angle is up on
 * this linkage: the arm rests at 30 and lifts to 90 - 60 deg of travel,
 * the same two angles as the old 90 down / 30 up, swapped.
 */
const uint8_t LIFT_DOWN   = 30;           // arm down - the rest position
const uint8_t LIFT_UP     = 90;           // arm raised, item scooped up to the belt
// Sort gate: where it swings to send an item to each bin. In MANUAL, 7 / 9
// put it there and leave it, to check these against the bins; 0 or H
// brings it home.
const uint8_t SORT_BIO    = 70;
const uint8_t SORT_NONBIO = 110;


const int8_t LIFT_R_TRIM = 0;   // degrees, applied to the right lift servo


const uint16_t SERVO_SETTLE_MS = 600;


const uint8_t  LIFT_STEP_DEG  = 2;
const uint8_t  LIFT_STEP_MS   = 15;
const uint16_t LIFT_HOLD_MS   = 150;    // after the ramp, let the arm stop swinging

// Last angle sent to the lift pair. servoSetup() loads LIFT_DOWN before
// attach(), so it starts there.
uint8_t liftAngle = LIFT_DOWN;

uint8_t gateAngle = SERVO_HOME;         // last angle sent to the sort gate
const uint16_t CONVEYOR_RUN_MS = 10000;  // belt time from the arm to the bin

void servoSetup() {
  // attach() starts a channel at the pulse already loaded, 90 if none.
  // That is the gate's rest angle but the lift's up, so the
  // lift is loaded with LIFT_DOWN first and the arm stays down at
  // power-on instead of jumping up. Before attach() the library maps
  // angles over its default 544-2400 us, a degree or two off, so
  // liftWrite() sends the exact pulse once attached.
  servoLiftL.write(LIFT_DOWN);
  servoLiftR.write(LIFT_DOWN);
  servoLiftL.attach(PIN_SERVO_LIFT_L, SERVO_MIN_US, SERVO_MAX_US);
  servoLiftR.attach(PIN_SERVO_LIFT_R, SERVO_MIN_US, SERVO_MAX_US);
  servoSort.attach(PIN_SERVO_SORT,    SERVO_MIN_US, SERVO_MAX_US);
  liftWrite(LIFT_DOWN);

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

// Park every servo at its rest angle: the arm down, the gate at 90.
void servoResetAll() {
  servoSort.write(SERVO_HOME);
  gateAngle = SERVO_HOME;
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
    gateBio();
  } else if (command == CMD_NONBIO) {
    gateNonBio();
  }
  gateHome();                       // let the item fall, then back to reset
}

// ------------------------------------------------------- sort gate
// Each returns once the gate has stopped moving. A move to where the gate
// already is returns at once. sortTo() uses them, and so do the MANUAL
// test commands 7 / 9 / 0, which leave the gate where they put it.
void gateTo(uint8_t angle) {
  if (angle == gateAngle) return;
  servoSort.write(angle);
  gateAngle = angle;
  delay(SERVO_SETTLE_MS);
}

void gateBio()    { gateTo(SORT_BIO);    }
void gateNonBio() { gateTo(SORT_NONBIO); }
void gateHome()   { gateTo(SERVO_HOME);  }

// For the STATUS line - main.ino cannot read gateAngle itself.
uint8_t gatePosition() {
  return gateAngle;
}

const __FlashStringHelper *gateName() {
  if (gateAngle == SORT_BIO)    return F("bio");
  if (gateAngle == SORT_NONBIO) return F("non-bio");
  if (gateAngle == SERVO_HOME)  return F("home");
  return F("?");
}


void armLiftUp() {
  liftMoveTo(LIFT_UP);
}

void armLiftDown() {
  liftMoveTo(LIFT_DOWN);
}

/*
 * Bench check for the lift pair, called by hand from setup() - never from
 * loop(). Both horns should travel together and sit level at 30 (down).
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



#include <Servo.h>


const bool DRIVE_BY_RELAY = true;

const bool DRIVE_RELAY_ACTIVE_LOW = true;

const uint8_t PIN_L_IN1  = 22;  // IN1 - forward    board 1 IN1
const uint8_t PIN_L_IN2  = 23;  // IN2 - backward   board 1 IN2
const uint8_t PIN_R_IN1  = 24;  // IN3 - forward    board 2 IN1
const uint8_t PIN_R_IN2  = 25;  // IN4 - backward   board 2 IN2
const uint8_t PIN_L_EN   = 2;   // unused           board 1 ENA (PWM)
const uint8_t PIN_R_EN   = 3;   // unused           board 2 ENA (PWM)


const uint8_t PIN_CONV_RELAY        = 26;
const bool    CONV_RELAY_ACTIVE_LOW = true;


const uint8_t PIN_SERVO_LIFT_L  = 9;
const uint8_t PIN_SERVO_STRETCH = 10;
const uint8_t PIN_SERVO_SORT    = 8;
const uint8_t PIN_SERVO_LIFT_R  = 12;


const uint8_t PIN_SERVO_PAN     = 11;

const uint8_t PIN_TRIG[2] = {36, 34};
const uint8_t PIN_ECHO[2] = {37, 35};

// ------------------------------------------------------------- tuning
// Speeds are PWM for the L298N. Relays have no speed: 0 is stop and
// anything else is full battery voltage.
const uint8_t  DRIVE_SPEED   = 160;   // 0-255 cruise
const uint8_t  TURN_SPEED    = 180;
const uint16_t AVOID_CM      = 60;    // AUTO: a side closer than this -> turn away from it
const uint16_t STOP_CM       = 20;    // MANUAL: forward refused / cut short closer than this
const uint16_t BACKUP_MS     = 300;   // both sides blocked: back out first - blind, no rear sensor
const uint16_t TURN_MS       = 3000;  // how long an avoidance turn runs
const uint16_t MANUAL_MOVE_MS = 1000; // how long one manual drive command runs
const uint16_t INIT_MS       = 10000; // after setup: motors held stopped, then auto start
const uint32_t CMD_TIMEOUT_MS = 5000; // Pi silence before we assume no target

// ------------------------------------------------------------- state
enum Command { CMD_NONE = 'X', CMD_BIO = 'B', CMD_NONBIO = 'N' };

char     lastCommand   = CMD_NONE;
uint32_t lastCommandAt = 0;
bool     pickupBusy    = false;   // true while the arm sequence is running
bool     manualMode    = true;    // stopped until the init countdown ends or 'A'
char     manualMove    = 0;       // drive command running: '8' '2' '4' '6', 0 = none
uint32_t manualMoveAt  = 0;       // when it was last sent
bool     autoStartPending = true; // init countdown running
uint32_t initStartedAt    = 0;

uint16_t distLeft, distRight;

void setup() {
  Serial.begin(9600);             // to the Orange Pi (USB / UART0)

  pinMode(LED_BUILTIN, OUTPUT);   // heartbeat, see heartbeat()

  motorSetup();
  servoSetup();
  ultrasonicSetup();

  Serial.println(F("RoboSort Mega ready"));
  printHelp();
  Serial.println(F("INIT drive  - relays off, wheels stopped"));
  Serial.println(F("INIT arm    - lift and sort servos at 90, stretch pulled in"));
  Serial.println(F("INIT camera - pan servo at 90, looking straight ahead"));
  Serial.println(F("INIT belt   - conveyor off"));
  printSensorCheck();
  Serial.print(F("INIT - driving starts in "));
  Serial.print(INIT_MS / 1000);
  Serial.println(F(" s. Any drive/arm/camera command cancels it; A starts now."));
  initStartedAt = millis();
}

void loop() {
  heartbeat();
  readSerialCommand();
  serviceAutoStart();

  // Manual: nothing below runs, so a manual move is not undone by the
  // navigation logic. Only a drive command in progress needs watching.
  if (manualMode) {
    serviceManualMove();
    return;
  }

  readAllDistances();

  // Safety first, and without asking the Pi: anything within AVOID_CM on
  // either side stops the wheels and turns the rover away from it,
  // regardless of what the vision pipeline is reporting.
  if (distLeft < AVOID_CM || distRight < AVOID_CM) {
    driveStop();
    avoidObstacle();
    return;
  }

  // A fresh B/N means the camera is looking at trash - collect it.
  if (isCommandFresh() && (lastCommand == CMD_BIO || lastCommand == CMD_NONBIO)) {
    driveStop();
    runPickupSequence();
    sortTo(lastCommand);
    // Consume the command so one detection triggers one pickup. The Pi
    // sends on change, so it will re-send B/N for the next item.
    lastCommand = CMD_NONE;
    return;
  }

  driveForward(DRIVE_SPEED);
}

/*
 * Blink the on-board L LED (pin 13) twice a second while loop() is alive.
 * When the serial monitor goes quiet this tells the two failures apart:
 * still blinking - the Mega is fine and the USB-serial link dropped;
 * frozen on or off - the Mega itself hung, almost always a supply sag.
 */
void heartbeat() {
  static uint32_t lastToggle = 0;
  if (millis() - lastToggle >= 250) {
    lastToggle = millis();
    digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN));
  }
}


 
const uint8_t  LINE_MAX     = 12;    // longest commands are "LOOKRIGHT" / "LOOKAHEAD"
const uint16_t LINE_IDLE_MS = 100;

char     rxLine[LINE_MAX + 1];
uint8_t  rxLen     = 0;
bool     rxTooLong = false;
uint32_t rxLastAt  = 0;

void readSerialCommand() {
  while (Serial.available() > 0) {
    char c = Serial.read();
    rxLastAt = millis();

    if (c == '\n' || c == '\r') {
      endLine();
    } else if (c == ' ' || c == '\t') {
      // dropped
    } else if (rxLen < LINE_MAX) {
      if (c >= 'a' && c <= 'z') c -= 32;   // typed by hand, accept lowercase
      rxLine[rxLen++] = c;
    } else {
      rxTooLong = true;
    }
  }

  if ((rxLen > 0 || rxTooLong) && millis() - rxLastAt >= LINE_IDLE_MS) {
    endLine();
  }
}


void endLine() {
  rxLine[rxLen] = '\0';
  if (rxLen > 0 || rxTooLong) {
    Serial.print(F("RX "));
    Serial.print(rxLine);
    if (rxTooLong) Serial.print(F("..."));
    Serial.println();
    Serial.flush();
  }

  if (rxTooLong) {
    Serial.println(F("FAIL - too long, not a command (send ? for the list)"));
  } else if (rxLen > 0) {
    runCommand(rxLine);
  }
  rxLen     = 0;
  rxTooLong = false;
}

char wordToCommand(const char *word) {
  if (strcmp_P(word, PSTR("FORWARD"))   == 0) return '8';
  if (strcmp_P(word, PSTR("BACKWARD"))  == 0) return '2';
  if (strcmp_P(word, PSTR("BACK"))      == 0) return '2';
  if (strcmp_P(word, PSTR("LEFT"))      == 0) return '4';
  if (strcmp_P(word, PSTR("RIGHT"))     == 0) return '6';
  if (strcmp_P(word, PSTR("STOP"))      == 0) return '5';
  if (strcmp_P(word, PSTR("LOOKLEFT"))  == 0) return '7';
  if (strcmp_P(word, PSTR("LOOKRIGHT")) == 0) return '9';
  if (strcmp_P(word, PSTR("LOOKAHEAD")) == 0) return '0';
  return 0;
}

void runCommand(const char *line) {
  char c = (line[1] == '\0') ? line[0] : wordToCommand(line);

  switch (c) {
    // From the Pi: latch it, loop() decides what to do about it.
    case CMD_BIO:
    case CMD_NONBIO:
    case CMD_NONE:
      lastCommand   = c;
      lastCommandAt = millis();
      Serial.print(F("OK "));
      if (c == CMD_BIO)         Serial.print(F("bio"));
      else if (c == CMD_NONBIO) Serial.print(F("non-bio"));
      else                      Serial.print(F("nothing detected"));
      if (manualMode && c != CMD_NONE) Serial.print(F(" - no pickup in MANUAL, send A"));
      Serial.println();
      break;

    // Manual driving - each command runs MANUAL_MOVE_MS, then stops.
    case '8':
    case '2':
    case '4':
    case '6':
      manualDrive(c);
      break;
    case '5': enterManual(); conveyorStop(); ok(F("stop - wheels and conveyor stopped")); break;

    // Manual movement tests. Each blocks until the joint has settled.
    case 'U': enterManual(); armLiftUp();     ok(F("lift up"));                 break;
    case 'D': enterManual(); armLiftDown();   ok(F("lift down"));               break;
    case 'S': enterManual(); armStretchOut(); ok(F("stretch out"));             break;
    case 'R': enterManual(); armRetract();    ok(F("retract"));                 break;
    case 'H': enterManual(); servoResetAll(); ok(F("home - servos at 90, stretch pulled in")); break;
    case 'C': enterManual(); conveyorRun();   ok(F("conveyor on"));             break;
    case 'O': enterManual(); conveyorStop();  ok(F("conveyor off"));            break;

    // Camera pan, 45 deg either side of straight ahead.
    case '7': enterManual(); cameraLookLeft();  ok(F("look left - camera 45 deg left"));   break;
    case '9': enterManual(); cameraLookRight(); ok(F("look right - camera 45 deg right")); break;
    case '0': enterManual(); cameraLookAhead(); ok(F("look ahead - camera at 90"));        break;

    case 'A': resumeAuto(); ok(F("autonomous - driving on its own, send 5 to stop")); break;
    case '?': printHelp(); printStatus(); break;

    default:
      Serial.print(F("FAIL "));
      Serial.print(line);
      Serial.println(F(" - unknown command (send ? for the list)"));
  }
}

void ok(const __FlashStringHelper *what) {
  Serial.print(F("OK "));
  Serial.println(what);
}

void enterManual() {
  cancelAutoStart();
  driveStop();
  manualMove = 0;
  if (manualMode) return;
  manualMode = true;
  Serial.println(F("MANUAL - wheels stopped, send A to resume"));
}

/*
 * Start a manual drive move, or keep one going. Repeating the same move
 * only pushes its deadline out, so a program that re-sends '8' every few
 * hundred ms drives smoothly - no stop in between to click the relays.
 */
void manualDrive(char move) {
  cancelAutoStart();
  if (!manualMode) enterManual();   // taking over from autonomous driving

  if (move == '8' && sideBlocked()) {
    driveStop();
    manualMove = 0;
    Serial.print(F("FAIL forward - "));
    printBlocked();
    return;
  }

  switch (move) {
    case '8': driveForward(DRIVE_SPEED);  break;
    case '2': driveBackward(DRIVE_SPEED); break;
    case '4': driveTurnLeft(TURN_SPEED);  break;
    case '6': driveTurnRight(TURN_SPEED); break;
  }
  manualMove   = move;
  manualMoveAt = millis();   // after drive(), which may pause before a reversal

  Serial.print(F("OK "));
  Serial.print(moveName(move));
  Serial.print(F(" - running "));
  Serial.print(MANUAL_MOVE_MS);
  Serial.println(F(" ms"));
}

const __FlashStringHelper *moveName(char move) {
  switch (move) {
    case '8': return F("forward");
    case '2': return F("backward");
    case '4': return F("turn left");
    default:  return F("turn right");
  }
}

// Every loop in MANUAL: end a drive move when its time is up, and cut a
// forward move short if either side gets too close.
void serviceManualMove() {
  if (manualMove == 0) return;

  if (millis() - manualMoveAt >= MANUAL_MOVE_MS) {
    driveStop();
    Serial.print(F("DONE "));
    Serial.print(moveName(manualMove));
    Serial.println(F(" - time up, send it again to keep going"));
    manualMove = 0;
  } else if (manualMove == '8' && sideBlocked()) {
    driveStop();
    manualMove = 0;
    Serial.print(F("STOP forward - "));
    printBlocked();
  }
}

bool sideBlocked() {
  readAllDistances();
  return distLeft < STOP_CM || distRight < STOP_CM;
}

void printBlocked() {
  Serial.print(F("blocked: left "));
  Serial.print(distLeft);
  Serial.print(F(" cm, right "));
  Serial.print(distRight);
  Serial.print(F(" cm (limit "));
  Serial.print(STOP_CM);
  Serial.println(F(" cm)"));
}

/*
 * One line of state. After a silence it tells the two failures apart: a
 * small uptime means the Mega reset, a large one means it kept running
 * and only the USB link dropped.
 */
void printStatus() {
  readAllDistances();
  Serial.print(F("STATUS "));
  Serial.print(manualMode ? F("MANUAL") : F("AUTO"));
  Serial.print(F(", manual move: "));
  Serial.print(manualMove ? moveName(manualMove) : F("none"));
  Serial.print(F(", conveyor "));
  Serial.print(conveyorIsOn() ? F("on") : F("off"));
  Serial.print(F(", camera "));
  Serial.print(cameraAngle());
  Serial.print(F(" deg"));
  Serial.print(F(", left "));
  Serial.print(distLeft);
  Serial.print(F(" cm, right "));
  Serial.print(distRight);
  Serial.print(F(" cm, up "));
  Serial.print(millis() / 1000);
  Serial.print(F(" s"));
  if (autoStartPending) {
    uint32_t elapsed = millis() - initStartedAt;
    Serial.print(F(", auto start in "));
    Serial.print(elapsed >= INIT_MS ? 0 : (INIT_MS - elapsed + 999) / 1000);
    Serial.print(F(" s"));
  }
  Serial.println();
}

/*
 * Back to autonomous. The latched command is dropped on the way out: a B
 * or N that arrived during the bench session would otherwise fire a
 * pickup the moment the rover starts driving again. A belt left running
 * by 'C' is stopped too - in AUTO the pickup sequence owns it. And the
 * camera is turned back to face ahead: the arm only reaches straight
 * ahead, so trash seen with the camera turned aside is not where the arm
 * would grab.
 */
void resumeAuto() {
  autoStartPending = false;
  manualMode  = false;
  manualMove  = 0;
  lastCommand = CMD_NONE;
  conveyorStop();
  cameraLookAhead();
}

// Every loop: count the init period down once a second, then start
// driving. Nothing happens once it has fired or been cancelled.
void serviceAutoStart() {
  static uint8_t shown = 0;   // last whole second printed
  if (!autoStartPending) return;

  uint32_t elapsed = millis() - initStartedAt;
  if (elapsed >= INIT_MS) {
    resumeAuto();
    Serial.print(F("AUTO - init done, driving forward; turns away from anything within "));
    Serial.print(AVOID_CM);
    Serial.println(F(" cm. Send 5 to stop."));
    return;
  }

  uint8_t left = (INIT_MS - elapsed + 999) / 1000;   // whole seconds, rounded up
  if (left != shown && left < INIT_MS / 1000) {
    shown = left;
    Serial.print(F("INIT - driving in "));
    Serial.print(left);
    Serial.println(F(" s"));
  }
}

// A drive or arm command during the countdown means a bench test: stay in
// MANUAL instead of driving off when the countdown ends.
void cancelAutoStart() {
  if (!autoStartPending) return;
  autoStartPending = false;
  Serial.println(F("INIT - auto start cancelled, staying in MANUAL (send A to drive)"));
}

void printHelp() {
  Serial.println(F("commands: 8 or FORWARD  2 or BACKWARD  4 or LEFT  6 or RIGHT  5 or STOP"));
  Serial.println(F("          U lift up  D lift down  S stretch out  R retract"));
  Serial.println(F("          7 or LOOKLEFT  9 or LOOKRIGHT  0 or LOOKAHEAD - camera"));
  Serial.println(F("          H home     C conveyor on  O conveyor off"));
  Serial.println(F("          A autonomous  ? this list + status"));
  Serial.println(F("          B bio      N non-bio    X none"));
  Serial.println(F("replies:  RX received, OK done, FAIL refused, DONE/STOP move ended"));
}

bool isCommandFresh() {
  return (millis() - lastCommandAt) < CMD_TIMEOUT_MS;
}

// ------------------------------------------------------- avoidance
/*
 * Called with the wheels already stopped, when either side reads closer
 * than AVOID_CM. Turn away from the blocked side:
 *     right blocked  -> turn left
 *     left blocked   -> turn right
 *     both blocked   -> back out first, then turn toward whichever side
 *                       has more room
 * A turn is a spin in place for TURN_MS. The next loop reads the sensors
 * again, so a turn that was not enough is simply followed by another.
 * The back-up blocks, but the turn is long, so it keeps the heartbeat and
 * the serial commands going: a 5 sent mid-turn stops the rover at once,
 * not TURN_MS later.
 */
void avoidObstacle() {
  bool turnLeft;

  if (distLeft < AVOID_CM && distRight < AVOID_CM) {
    driveBackward(DRIVE_SPEED);
    delay(BACKUP_MS);
    driveStop();
    readAllDistances();
    turnLeft = distLeft >= distRight;
  } else {
    turnLeft = distRight < AVOID_CM;
  }

  Serial.print(F("AVOID - left "));
  Serial.print(distLeft);
  Serial.print(F(" cm, right "));
  Serial.print(distRight);
  Serial.println(turnLeft ? F(" cm -> turn left") : F(" cm -> turn right"));

  if (turnLeft) {
    driveTurnLeft(TURN_SPEED);
  } else {
    driveTurnRight(TURN_SPEED);
  }

  uint32_t start = millis();
  while (millis() - start < TURN_MS) {
    heartbeat();
    readSerialCommand();
    if (manualMode) return;   // a command took over and has already stopped the wheels
    delay(1);
  }
  driveStop();
}


#include <Servo.h>
#include <Wire.h>
#include <LiquidCrystal_I2C.h>


// Drive motors: a 4-ch relay module, two relays per motor, on/off only.
const bool DRIVE_RELAY_ACTIVE_LOW = true;

const uint8_t PIN_L_IN1  = 22;  // IN1 - left forward
const uint8_t PIN_L_IN2  = 23;  // IN2 - left backward
const uint8_t PIN_R_IN1  = 24;  // IN3 - right forward
const uint8_t PIN_R_IN2  = 25;  // IN4 - right backward


const uint8_t PIN_CONV_RELAY        = 26;
const bool    CONV_RELAY_ACTIVE_LOW = true;


const uint8_t PIN_SERVO_LIFT_L  = 9;
const uint8_t PIN_SERVO_SORT    = 8;
const uint8_t PIN_SERVO_LIFT_R  = 12;

const uint8_t PIN_TRIG[2] = {36, 34};
const uint8_t PIN_ECHO[2] = {37, 35};

// Pi link: Serial1 - TX1 D18, RX1 D19 - to the Orange Pi's UART3. D18 goes
// through a 10k/20k divider: the Pi's pins take 3.3 V, not 5 V.

// Status LCD: I2C on the Mega's fixed SDA 20 / SCL 21 - see Lcd.ino.

// ------------------------------------------------------------- tuning
// Relays have no speed control: every move is full battery voltage.
const uint16_t AVOID_CM      = 30;    // AUTO: a side closer than this -> turn away from it
const uint16_t STOP_CM       = 10;    // MANUAL: forward refused / cut short closer than this
const uint16_t BACKUP_MS     = 300;   // both sides blocked: back out first - blind, no rear sensor
const uint16_t TURN_MS       = 3000;  // how long an avoidance turn runs
const uint16_t MANUAL_MOVE_MS = 5000; // how long one manual drive command runs
const uint16_t INIT_MS       = 10000; // after setup: motors held stopped, then auto start
const uint32_t CMD_TIMEOUT_MS = 5000; // Pi silence before we assume no target
// Approach: one F/L/R from the Pi is one short move, then the rover waits
// stopped for the Pi's next look. 250 / 150 ms barely moved the rover on
// the floor (2026-11-16): the relay pulse ended while the motors were still
// spinning up, and the Pi gave up after 40 steps. Keep a step under ~700 ms:
// the Pi takes its next look 800 ms after sending (vision.py STEP_SETTLE_S),
// and the step has to be over by then.
const uint16_t APPROACH_STEP_MS = 500;  // forward step toward trash
const uint16_t APPROACH_TURN_MS = 250;  // turn step to centre trash left/right
const uint16_t APPROACH_HOLD_MS = 3000; // trash lost mid-approach: stay stopped this long
// Pi silent mid-approach: stay stopped this long for its next look. A hot
// Pi skips frames for a while (vision.py's heat guard) - with the old 5 s
// the rover gave up and drove past the item.
const uint16_t APPROACH_WAIT_MS = 15000;

// ------------------------------------------------------------- state
enum Command { CMD_NONE = 'X', CMD_BIO = 'B', CMD_NONBIO = 'N', CMD_PING = 'P',
               // trash in the pickup zone, not at the grab spot yet
               CMD_AHEAD = 'F', CMD_LEFT = 'L', CMD_RIGHT = 'R' };

char     lastCommand   = CMD_NONE;
uint32_t lastCommandAt = 0;
char     lastSeen      = CMD_NONE; // last B/N/X received, never consumed - for the LCD
// The vision pipeline's lines - P, B, N, X - as last heard on each port: the
// Pi's wire (Serial1), and USB, where a laptop can run vision.py instead.
bool     piHeard       = false;    // any on the Pi's wire since boot
uint32_t piHeardAt     = 0;        // when the latest one arrived
bool     usbHeard      = false;    // the same, for USB
uint32_t usbHeardAt    = 0;
// A B/N/X/F/L/R has arrived since boot: vision.py has run its model on a
// camera frame. P does not count - the Pi sends it from the moment the port
// is open, seconds before the model's first frame.
bool     visionSeen    = false;
// From the Pi's T<deg C> lines (every 2 s) and W<seconds> line: its CPU
// temperature for the LCD, and a cool-down - its model is off, so the
// wheels stay stopped until it is over.
uint16_t piTempC       = 0;
uint32_t piTempAt      = 0;
bool     piTempKnown   = false;
uint32_t coolStartedAt = 0;
uint32_t coolMs        = 0;       // 0 = no cool-down
// The cool-down stopped AUTO: MANUAL, as a 5 would, but the belt keeps
// running, and A goes out by itself when it is over. Any command typed
// meanwhile (5, 8, ...) cancels that, and the rover stays in MANUAL.
bool     coolHold      = false;
bool     pickupBusy    = false;   // true while the arm sequence is running
bool     approaching   = false;   // AUTO stopped for trash ahead, stepping up to it
bool     stepPending   = false;   // an F/L/R arrived and its step has not run yet
bool     manualMode    = true;    // stopped until the init countdown ends or 'A'
char     manualMove    = 0;       // drive command running: '8' '2' '4' '6', 0 = none
uint32_t manualMoveAt  = 0;       // when it was last sent
bool     autoStartPending = true; // init countdown running
uint32_t initStartedAt    = 0;

uint16_t distLeft, distRight;

/*
 * Serial input, one line buffer per port, so junk on the Pi's wire - noise,
 * an echo - cannot run into a command typed on USB, or the other way round.
 * Read in readSerialCommand(). Declared up here, not with it, because the
 * prototypes the Arduino IDE generates for functions taking a LineIn are
 * placed above the first function in the file.
 */
const uint8_t LINE_MAX = 12;           // longest command is "SORTNONBIO"

struct LineIn {
  char     text[LINE_MAX + 1];
  uint8_t  len;
  bool     tooLong;
  uint32_t lastAt;       // last character received
  bool     quiet;        // junk was reported at junkAt; more is ignored for now
  uint32_t junkAt;
};
LineIn fromPi, fromUsb;

/*
 * Every line goes out on both ports, and commands are read from both: the
 * Pi on Serial1, and USB for a laptop running mega.py or the Serial
 * Monitor, so bench tests still work with the Pi wired in.
 */
class BothPorts : public Print {
public:
  size_t write(uint8_t c) override {
    Serial1.write(c);
    Serial.write(c);
    return 1;
  }
  void flush() override {
    Serial1.flush();
    Serial.flush();
  }
};
BothPorts Link;

void setup() {
  Serial.begin(9600);             // USB: a laptop, for uploads and mega.py
  Serial1.begin(9600);            // the Orange Pi's UART3, D18 TX1 / D19 RX1
  // Hold RX1 high when nothing drives it. A loose Pi wire, or the Pi still
  // booting, leaves D19 floating, and it reads noise as a stream of junk lines.
  // ~50 uA into the Pi's 3.3 V TX pin when wired; harmless.
  pinMode(19, INPUT_PULLUP);

  pinMode(LED_BUILTIN, OUTPUT);   // heartbeat, see heartbeat()

  motorSetup();
  lcdSetup();                     // early, so the screen is up while the servos home
  servoSetup();
  ultrasonicSetup();

  Link.println(F("RoboSort Mega ready"));
  printHelp();
  Link.println(F("INIT drive  - relays off, wheels stopped"));
  Link.println(F("INIT arm    - lift at 30, arm down; sort gate at 90"));
  Link.println(F("INIT belt   - conveyor off"));
  printLcdCheck();

  // The same checks on the LCD, a line each. Only the sonars can answer
  // back - relays and servos have no feedback - so the other lines say
  // what was set, not what was confirmed.
  lcdRow(0, F("Checking parts"));
  lcdCheck(F("Drive"),  F("off"),   true);
  lcdCheck(F("Belt"),   F("off"),   true);
  lcdCheck(F("Servos"), F("home"),  true);
  printSensorCheck();             // pings both sonars, adds their LCD lines
  Link.print(F("INIT - driving starts in "));
  Link.print(INIT_MS / 1000);
  Link.println(F(" s, camera or not. Any drive/arm/gate command cancels it; A starts now."));
  initStartedAt = millis();
}

void loop() {
  heartbeat();
  readSerialCommand();
  serviceAutoStart();
  lcdService();

  // Manual: nothing below runs, so a manual move is not undone by the
  // navigation logic. Only a drive command in progress needs watching.
  if (manualMode) {
    if (coolHold) {
      serviceCoolHold();
      return;
    }
    serviceManualMove();
    return;
  }

  readAllDistances();

  // Safety first, and without asking the Pi: anything within AVOID_CM on
  // either side stops the wheels and turns the rover away from it,
  // regardless of what the vision pipeline is reporting. While the Pi
  // reports trash in reach, only STOP_CM counts: the rover is creeping up
  // to it in short steps, and the sonars look sideways, so a wall or a
  // chair 50 cm to the side is no danger - it only kept turning the rover
  // away from the trash.
  uint16_t limit = trashInReach() ? STOP_CM : AVOID_CM;
  if (distLeft < limit || distRight < limit) {
    driveStop();
    avoidObstacle();
    // The rover has turned away, so whatever the Pi reported before or
    // during the turn - a B/N, a step - was about where the item used to
    // be. Acting on it now would lift at empty floor or step the wrong way.
    // The Pi's next frame, taken after the turn, starts afresh.
    lastCommand = CMD_NONE;
    approaching = false;
    stepPending = false;
    return;
  }

  // A fresh B/N means the camera is looking at trash - collect it.
  if (isCommandFresh() && (lastCommand == CMD_BIO || lastCommand == CMD_NONBIO)) {
    char item = lastCommand;
    driveStop();
    // PICKUP and SORTED bracket the sequence. The Pi sends nothing in
    // between, so the item on the arm or a flicker cannot start another.
    Link.print(F("PICKUP "));
    Link.print(classLabel(item));
    Link.println(F(" - arm up, belt, gate"));
    lcdPickup(item, F("pick up"));
    runPickupSequence();
    lcdPickup(item, F("sorting"));
    sortTo(item);
    // Serial is not read during the sequence, so anything sent in it is
    // still queued and would read as fresh now. Read it - a 5 still stops
    // the rover - then drop whatever B/N it latched, so one detection is
    // one pickup. Before SORTED, so anything after SORTED is new.
    readSerialCommand();
    lastCommand = CMD_NONE;
    stepPending = false;
    // Stay stopped for the Pi's first look after the pickup - the hold in
    // serviceApproach(). A missed item is still at the scoop: the Pi steps
    // up to it again (vision.py tries twice), where driving on at once
    // would run onto it before the Pi had looked.
    approaching   = true;
    lastCommandAt = millis();
    Link.print(F("SORTED "));
    Link.print(classLabel(item));
    Link.println(F(" - arm down, gate home"));
    return;
  }

  // F/L/R: trash in the pickup zone, not yet at the grab spot - step to it.
  if (serviceApproach()) return;

  lcdRow(0, F("AUTO driving"));
  driveForward();
}

bool isApproachCommand(char c) {
  return c == CMD_AHEAD || c == CMD_LEFT || c == CMD_RIGHT;
}

// Stepping up to trash, or the Pi has just reported trash in reach.
bool trashInReach() {
  return approaching || (lastCommand != CMD_NONE && isCommandFresh());
}

/*
 * Every AUTO loop: drive up to trash the Pi sees in the pickup zone. The Pi
 * sends F (straight ahead), L or R (off to that side) for each frame while
 * the item is in the zone and short of the grab spot, then B/N once it is
 * there - and loop() picks it up. Returns true while the wheels belong to
 * the approach, false to let the rover cruise.
 *
 *  - The first F/L/R only stops the rover. It came from a frame taken while
 *    cruising, and the rover has moved on since; the Pi looks again.
 *  - After that, each F/L/R is one short step, then the rover waits stopped
 *    for the next one. The Pi waits for the step to end before it looks.
 *  - X (lost the item): stay stopped APPROACH_HOLD_MS, so one missed frame
 *    does not send the rover driving over the item, then cruise. A silent
 *    Pi: stay stopped until the last F/L/R is APPROACH_WAIT_MS old, then cruise.
 *  - After a pickup the same hold runs, so the Pi gets one look first.
 */
bool serviceApproach() {
  if (isApproachCommand(lastCommand) && millis() - lastCommandAt < APPROACH_WAIT_MS) {
    if (!approaching) {
      approaching = true;
      stepPending = false;
      driveStop();
      lcdRow(0, F("APPROACH look"));
      Link.println(F("APPROACH - trash ahead, stopped to look again"));
    } else if (stepPending) {
      stepPending = false;
      approachStep(lastCommand);
    }
    return true;
  }

  if (!approaching) return false;
  if (millis() - lastCommandAt < APPROACH_HOLD_MS) {
    driveStop();
    lcdRow(0, F("APPROACH wait"));
    return true;
  }
  approaching = false;
  Link.println(F("APPROACH - nothing to pick up, driving on"));
  return false;
}

// One approach move, then stop. Keeps reading serial like avoidObstacle(),
// so a 5 stops it at once.
void approachStep(char c) {
  uint16_t ms = APPROACH_TURN_MS;
  switch (c) {
    case CMD_AHEAD: lcdRow(0, F("APPROACH forward")); driveForward();   ms = APPROACH_STEP_MS; break;
    case CMD_LEFT:  lcdRow(0, F("APPROACH left"));    driveTurnLeft();  break;
    default:        lcdRow(0, F("APPROACH right"));   driveTurnRight(); break;
  }
  uint32_t start = millis();
  while (millis() - start < ms) {
    heartbeat();
    readSerialCommand();
    lcdService();
    if (manualMode) return;   // a command took over and has already stopped the wheels
    delay(1);
  }
  driveStop();
  lcdRow(0, F("APPROACH look"));
}

const __FlashStringHelper *approachName(char c) {
  switch (c) {
    case CMD_AHEAD: return F("step forward");
    case CMD_LEFT:  return F("turn left");
    default:        return F("turn right");
  }
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


 
const uint16_t LINE_IDLE_MS  = 100;
const uint16_t JUNK_QUIET_MS = 2000;  // after junk is reported, more is ignored this long

// Every letter runCommand() acts on, plus the P heartbeat. A line that is
// not one of these, or a word from wordToCommand(), is junk. A new command
// letter goes here as well as into runCommand()'s switch.
const char COMMAND_CHARS[] PROGMEM = "BNXPFLR82465UDHCO790A?";

void readSerialCommand() {
  while (Serial1.available() > 0) rxChar(fromPi,  Serial1.read());
  while (Serial.available() > 0)  rxChar(fromUsb, Serial.read());
  endIdleLine(fromPi);
  endIdleLine(fromUsb);
}

// A line with no newline ends when its port has been quiet LINE_IDLE_MS.
void endIdleLine(LineIn &in) {
  if ((in.len > 0 || in.tooLong) && millis() - in.lastAt >= LINE_IDLE_MS) endLine(in);
}

void rxChar(LineIn &in, char c) {
  in.lastAt = millis();

  if (c == '\n' || c == '\r') {
    endLine(in);
  } else if (c == ' ' || c == '\t') {
    // dropped
  } else if (in.len < LINE_MAX) {
    if (c >= 'a' && c <= 'z') c -= 32;   // typed by hand, accept lowercase
    in.text[in.len++] = c;
  } else {
    in.tooLong = true;
  }
}

// The command a line stands for: a known letter or word. 0 = not a command.
char commandFor(const char *line) {
  if (line[0] == '\0') return 0;
  if (line[1] == '\0') return strchr_P(COMMAND_CHARS, line[0]) ? line[0] : 0;
  return wordToCommand(line);
}

// The vision pipeline is talking on this port. Only on the Pi's wire is it
// the Pi.
void markHeard(LineIn &in) {
  if (&in == &fromPi) { piHeard  = true; piHeardAt  = millis(); }
  else                { usbHeard = true; usbHeardAt = millis(); }
}

// "T87" or "W30": the letter, then 1-3 digits. value gets the number.
bool numberLine(const char *text, char letter, uint16_t &value) {
  if (text[0] != letter || text[1] == '\0' || strlen(text) > 4) return false;
  uint16_t v = 0;
  for (const char *p = text + 1; *p; p++) {
    if (*p < '0' || *p > '9') return false;
    v = v * 10 + (*p - '0');
  }
  value = v;
  return true;
}

bool piCoolingNow() {
  return coolMs != 0 && millis() - coolStartedAt < coolMs;
}

void endLine(LineIn &in) {
  in.text[in.len] = '\0';
  bool any = in.len > 0 || in.tooLong;
  uint16_t num;

  // T<deg C> every 2 s: the Pi's temperature, for the LCD. Not echoed -
  // like P, it would bury every other line.
  if (!in.tooLong && numberLine(in.text, 'T', num)) {
    piTempC = num;
    piTempAt = millis();
    piTempKnown = true;
    markHeard(in);
    in.len = 0;
    return;
  }
  // W<seconds>: the Pi turned its model off to cool. Driving on with the
  // camera off ran the rover past trash, so in AUTO it stops as a 5 would -
  // MANUAL, belt still running - shows COOLDOWN counting down, and sends
  // itself A when it is over (serviceCoolHold). Not driving - the start-up
  // countdown, or a bench test in MANUAL - nothing changes.
  if (!in.tooLong && numberLine(in.text, 'W', num)) {
    coolStartedAt = millis();
    coolMs = num * 1000UL;
    markHeard(in);
    if (!manualMode) {
      enterManual();              // wheels stopped; the belt is left alone
      lastCommand = CMD_NONE;     // stale by the time it drives again
      coolHold = true;
    }
    Link.print(F("COOLDOWN - Pi at "));
    Link.print(piTempC);
    Link.print(F(" C, camera off for "));
    Link.print(num);
    Link.println(coolHold ? F(" s - MANUAL stop, A after the countdown") : F(" s"));
    in.len = 0;
    return;
  }

  char c   = in.tooLong ? 0 : commandFor(in.text);

  if (c) {
    // Only what vision.py sends - P, B, N, X, F, L, R - shows the vision
    // pipeline is alive, and only on the Pi's wire is it the Pi. A 5 or ?
    // typed on the Serial Monitor is neither: it used to read "Pi OK", then
    // "Pi: lost", with no Pi wired at all.
    if (c == CMD_PING || c == CMD_BIO || c == CMD_NONBIO || c == CMD_NONE ||
        isApproachCommand(c)) {
      markHeard(in);
      if (c != CMD_PING) visionSeen = true;
    }
    // P is the Pi's once-a-second heartbeat. It only marks the Pi as alive,
    // so it is not echoed - RX P every second would bury every other line.
    if (c != CMD_PING) {
      Link.print(F("RX "));
      Link.println(in.text);
      Link.flush();
      runCommand(c);
    }
  } else if (any) {
    reportJunk(in, &in == &fromPi ? F("Pi") : F("USB"));
  }
  in.len     = 0;
  in.tooLong = false;
}

/*
 * A line that is not a command: noise on a loose wire, a typo, or the
 * Mega's own output coming back - a wiring loop from D18 to D19, or a Pi
 * program holding the port open with echo on. Answering every one feeds an
 * echo forever, each answer returning as more junk, so junk is not echoed
 * as RX and is reported once, then ignored for JUNK_QUIET_MS. An echo of
 * the report lands inside that quiet time and dies there.
 */
void reportJunk(LineIn &in, const __FlashStringHelper *port) {
  uint32_t now = millis();
  if (in.quiet && now - in.junkAt < JUNK_QUIET_MS) return;
  in.quiet  = true;
  in.junkAt = now;
  Link.print(F("FAIL from "));
  Link.print(port);
  Link.print(F(": "));
  Link.print(in.text);
  if (in.tooLong) Link.print(F("..."));
  Link.println(F(" - not a command, ignored (send ? for the list)"));
}

char wordToCommand(const char *word) {
  if (strcmp_P(word, PSTR("FORWARD"))   == 0) return '8';
  if (strcmp_P(word, PSTR("BACKWARD"))  == 0) return '2';
  if (strcmp_P(word, PSTR("BACK"))      == 0) return '2';
  if (strcmp_P(word, PSTR("LEFT"))      == 0) return '4';
  if (strcmp_P(word, PSTR("RIGHT"))     == 0) return '6';
  if (strcmp_P(word, PSTR("STOP"))      == 0) return '5';
  if (strcmp_P(word, PSTR("SORTBIO"))    == 0) return '7';
  if (strcmp_P(word, PSTR("SORTNONBIO")) == 0) return '9';
  if (strcmp_P(word, PSTR("SORTHOME"))   == 0) return '0';
  return 0;
}

// c comes from commandFor(), so it is always one of COMMAND_CHARS.
void runCommand(char c) {
  switch (c) {
    // From the Pi: latch it, loop() decides what to do about it.
    case CMD_BIO:
    case CMD_NONBIO:
    case CMD_NONE:
      lastCommand   = c;
      lastCommandAt = millis();
      lastSeen      = c;
      Link.print(F("OK "));
      if (c == CMD_BIO)         Link.print(F("bio"));
      else if (c == CMD_NONBIO) Link.print(F("non-bio"));
      else                      Link.print(F("nothing detected"));
      if (manualMode && c != CMD_NONE) Link.print(F(" - no pickup in MANUAL, send A"));
      Link.println();
      break;

    // From the Pi: trash in the pickup zone, short of the grab spot. Each
    // one is one step - serviceApproach() runs it.
    case CMD_AHEAD:
    case CMD_LEFT:
    case CMD_RIGHT:
      lastCommand   = c;
      lastCommandAt = millis();
      lastSeen      = c;
      stepPending   = true;
      Link.print(F("OK trash ahead - "));
      Link.print(approachName(c));
      if (manualMode) Link.print(F(" - not in MANUAL, send A"));
      Link.println();
      break;

    // Manual driving - each command runs MANUAL_MOVE_MS, then stops.
    case '8':
    case '2':
    case '4':
    case '6':
      manualDrive(c);
      break;
    case '5': enterManual(); conveyorStop(); lcdRow(0, F("MANUAL stopped")); ok(F("stop - wheels and conveyor stopped")); break;

    // Manual movement tests. Each blocks until the joint has settled, so
    // the LCD is set first and shows the move while it runs.
    case 'U': enterManual(); lcdRow(0, F("MANUAL lift up"));   armLiftUp();     ok(F("lift up"));      break;
    case 'D': enterManual(); lcdRow(0, F("MANUAL lift down")); armLiftDown();   ok(F("lift down"));    break;
    case 'H': enterManual(); lcdRow(0, F("MANUAL home"));      servoResetAll(); ok(F("home - arm down, gate at 90")); break;
    case 'C': enterManual(); lcdRow(0, F("MANUAL belt on"));   conveyorRun();   ok(F("conveyor on"));  break;
    case 'O': enterManual(); lcdRow(0, F("MANUAL belt off"));  conveyorStop();  ok(F("conveyor off")); break;

    // Sort gate test: swing it to a bin's side and leave it there, to check
    // it lines up (SORT_BIO / SORT_NONBIO in Servo.ino). 0 or H: back home.
    case '7': enterManual(); lcdRow(0, F("MANUAL gate bio"));  gateBio();    ok(F("sort gate to bio - stays there, 0 for home"));     break;
    case '9': enterManual(); lcdRow(0, F("MANUAL gate nbio")); gateNonBio(); ok(F("sort gate to non-bio - stays there, 0 for home")); break;
    case '0': enterManual(); lcdRow(0, F("MANUAL gate home")); gateHome();   ok(F("sort gate home - at 90"));                         break;

    case 'A': resumeAuto(); ok(F("autonomous - driving on its own, send 5 to stop")); break;
    case '?': printHelp(); printStatus(); break;
  }
}

void ok(const __FlashStringHelper *what) {
  Link.print(F("OK "));
  Link.println(what);
}

// The cool-down's MANUAL stop: count down on the LCD, then A by itself.
void serviceCoolHold() {
  if (piCoolingNow()) {
    lcdCooling((coolMs - (millis() - coolStartedAt) + 999) / 1000);
    return;
  }
  resumeAuto();
  ok(F("autonomous - cooldown over, A sent by itself"));
}

void enterManual() {
  cancelAutoStart();
  coolHold = false;     // a command typed mid-cool-down keeps the rover in MANUAL
  driveStop();
  manualMove = 0;
  approaching = false;
  stepPending = false;
  if (manualMode) return;
  manualMode = true;
  Link.println(F("MANUAL - wheels stopped, send A to resume"));
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
    lcdRow(0, F("MANUAL blocked"));
    Link.print(F("FAIL forward - "));
    printBlocked();
    return;
  }

  switch (move) {
    case '8': lcdRow(0, F("MANUAL forward"));  driveForward();   break;
    case '2': lcdRow(0, F("MANUAL backward")); driveBackward();  break;
    case '4': lcdRow(0, F("MANUAL turn L"));   driveTurnLeft();  break;
    case '6': lcdRow(0, F("MANUAL turn R"));   driveTurnRight(); break;
  }
  manualMove   = move;
  manualMoveAt = millis();   // after drive(), which may pause before a reversal

  Link.print(F("OK "));
  Link.print(moveName(move));
  Link.print(F(" - running "));
  Link.print(MANUAL_MOVE_MS);
  Link.println(F(" ms"));
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
    lcdRow(0, F("MANUAL stopped"));
    Link.print(F("DONE "));
    Link.print(moveName(manualMove));
    Link.println(F(" - time up, send it again to keep going"));
    manualMove = 0;
  } else if (manualMove == '8' && sideBlocked()) {
    driveStop();
    manualMove = 0;
    lcdRow(0, F("MANUAL blocked"));
    Link.print(F("STOP forward - "));
    printBlocked();
  }
}

bool sideBlocked() {
  readAllDistances();
  return distLeft < STOP_CM || distRight < STOP_CM;
}

void printBlocked() {
  Link.print(F("blocked: left "));
  Link.print(distLeft);
  Link.print(F(" cm, right "));
  Link.print(distRight);
  Link.print(F(" cm (limit "));
  Link.print(STOP_CM);
  Link.println(F(" cm)"));
}

/*
 * One line of state. After a silence it tells the two failures apart: a
 * small uptime means the Mega reset, a large one means it kept running
 * and only the USB link dropped.
 */
void printStatus() {
  readAllDistances();
  Link.print(F("STATUS "));
  Link.print(manualMode ? F("MANUAL") : F("AUTO"));
  if (approaching) Link.print(F(" approaching trash"));
  Link.print(F(", manual move: "));
  Link.print(manualMove ? moveName(manualMove) : F("none"));
  Link.print(F(", conveyor "));
  Link.print(conveyorIsOn() ? F("on") : F("off"));
  Link.print(F(", gate "));
  Link.print(gateName());
  Link.print(F(" ("));
  Link.print(gatePosition());
  Link.print(F(" deg)"));
  Link.print(F(", left "));
  Link.print(distLeft);
  Link.print(F(" cm, right "));
  Link.print(distRight);
  Link.print(F(" cm, up "));
  Link.print(millis() / 1000);
  Link.print(F(" s"));
  // The LCD's bottom row, for whoever cannot see the screen.
  switch (visionHost()) {
    case 'P': Link.print(F(", Pi OK"));
              if (piTempKnown) { Link.print(F(" ")); Link.print(piTempC); Link.print(F(" C")); }
              if (piCoolingNow()) Link.print(F(" cooling"));
              break;
    case 'U': Link.print(F(", USB OK"));    break;
    case 'L': Link.print(F(", Pi lost"));   break;
    default:  Link.print(F(", Pi not heard"));
  }
  if (autoStartPending) {
    uint32_t elapsed = millis() - initStartedAt;
    Link.print(F(", auto start in "));
    Link.print(elapsed >= INIT_MS ? 0 : (INIT_MS - elapsed + 999) / 1000);
    Link.print(F(" s"));
    if (!visionReady()) Link.print(F(", camera not heard yet"));
  }
  Link.println();
}

/*
 * Back to autonomous. The latched command is dropped on the way out: a B
 * or N that arrived during the bench session would otherwise fire a
 * pickup the moment the rover starts driving again. The belt runs the
 * whole time in AUTO, so it is switched on here, and a gate left at a
 * bin's side by 7 / 9 goes home, where sortTo() expects it.
 */
void resumeAuto() {
  autoStartPending = false;
  manualMode  = false;
  manualMove  = 0;
  coolHold    = false;
  lastCommand = CMD_NONE;
  approaching = false;
  stepPending = false;
  conveyorRun();
  gateHome();
}

/*
 * Has the camera pipeline been heard: vision.py sent a result (not just P)
 * and is still talking? Only reported now - the rover no longer waits for
 * it. It used to hold the wheels until the first result, but the Pi sends X
 * only when its result changes, so a Mega reset (opening the Serial Monitor
 * resets it) on an empty floor left it waiting for good: the Pi cannot see
 * the reset, its replies from the Mega arrive garbled (2026-11-22).
 */
bool visionReady() {
  char host = visionHost();
  return visionSeen && (host == 'P' || host == 'U');
}

// Every loop: count the init period down once a second, then drive - with
// the camera or without it; trash it reports later is picked up as usual.
// The belt starts with it (resumeAuto) and runs from then on, through
// pickups and cool-downs; only 5 or O stop it. Nothing happens once it has
// fired or been cancelled. A starts at once.
void serviceAutoStart() {
  static uint8_t shown = 0;   // last whole second printed
  if (!autoStartPending) return;

  uint32_t elapsed = millis() - initStartedAt;
  if (elapsed >= INIT_MS) {
    resumeAuto();
    Link.print(F("AUTO - init done, driving forward, belt on; turns away from anything within "));
    Link.print(AVOID_CM);
    Link.println(F(" cm. Send 5 to stop."));
    if (!visionReady())
      Link.println(F("AUTO - no result from vision.py yet: driving without the camera until it reports"));
    return;
  }

  uint8_t left = (INIT_MS - elapsed + 999) / 1000;   // whole seconds, rounded up
  lcdCountdown(left);
  if (left != shown && left < INIT_MS / 1000) {
    shown = left;
    Link.print(F("INIT - driving in "));
    Link.print(left);
    Link.println(F(" s"));
  }
}

// A drive or arm command during the countdown means a bench test: stay in
// MANUAL instead of driving off when the countdown ends.
void cancelAutoStart() {
  if (!autoStartPending) return;
  autoStartPending = false;
  Link.println(F("INIT - auto start cancelled, staying in MANUAL (send A to drive)"));
}

void printHelp() {
  Link.println(F("commands: 8 or FORWARD  2 or BACKWARD  4 or LEFT  6 or RIGHT  5 or STOP"));
  Link.println(F("          U lift up  D lift down"));
  Link.println(F("          7 or SORTBIO  9 or SORTNONBIO  0 or SORTHOME - sort gate test"));
  Link.println(F("          H home     C conveyor on  O conveyor off"));
  Link.println(F("          A autonomous  ? this list + status"));
  Link.println(F("          B bio      N non-bio    X none   - at the grab spot: pick up"));
  Link.println(F("          F L R trash ahead: one step forward / turn left / turn right"));
  Link.println(F("          P heartbeat the Pi sends every second - no reply"));
  Link.println(F("          T87 Pi temperature (LCD)  W30 Pi cooling: MANUAL stop, A after 30 s"));
  Link.println(F("replies:  RX received, OK done, FAIL refused, DONE/STOP move ended"));
  Link.println(F("          APPROACH stepping up to trash"));
  Link.println(F("          PICKUP/SORTED a pickup started / its item is in the bin"));
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
    lcdRow(0, F("AVOID backing up"));
    driveBackward();
    delay(BACKUP_MS);
    driveStop();
    readAllDistances();
    turnLeft = distLeft >= distRight;
  } else {
    turnLeft = distRight < AVOID_CM;
  }

  Link.print(F("AVOID - left "));
  Link.print(distLeft);
  Link.print(F(" cm, right "));
  Link.print(distRight);
  Link.println(turnLeft ? F(" cm -> turn left") : F(" cm -> turn right"));
  lcdRow(0, turnLeft ? F("AVOID turn left") : F("AVOID turn right"));

  if (turnLeft) {
    driveTurnLeft();
  } else {
    driveTurnRight();
  }

  uint32_t start = millis();
  while (millis() - start < TURN_MS) {
    heartbeat();
    readSerialCommand();
    lcdService();
    if (manualMode) return;   // a command took over and has already stopped the wheels
    delay(1);
  }
  driveStop();
}

#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <Servo.h>
#include <math.h>
#include <avr/wdt.h>
#include <AFMotor.h>
// ================= Sound (DFPlayer Mini) =================
// Sends the DFPlayer serial protocol manually (bit-banged, TX-only, no
// interrupts) instead of using SoftwareSerial. Two reasons:
//  1) SoftwareSerial uses pin-change interrupts, which are known to conflict
//     with the Servo library on an Uno (can cause jitter or lockups).
//  2) We never need to read anything back from the DFPlayer, so a full
//     duplex library (with its receive buffer) just wastes RAM we don't have.
// Only ONE wire is needed: this pin -> DFPlayer RX.
#define DF_TX_PIN 6

void dfWriteByte(uint8_t b) {
  // 9600 baud, 8N1, idle-high — bit-banged with no interrupts involved.
  noInterrupts();
  digitalWrite(DF_TX_PIN, LOW);           // start bit
  delayMicroseconds(104);
  for (uint8_t i = 0; i < 8; i++) {
    digitalWrite(DF_TX_PIN, (b & 0x01) ? HIGH : LOW);
    b >>= 1;
    delayMicroseconds(104);
  }
  digitalWrite(DF_TX_PIN, HIGH);          // stop bit
  delayMicroseconds(104);
  interrupts();
}

void dfSendCommand(uint8_t cmd, uint16_t param) {
  uint8_t packet[10];
  packet[0] = 0x7E;
  packet[1] = 0xFF;
  packet[2] = 0x06;
  packet[3] = cmd;
  packet[4] = 0x00; // no ack requested
  packet[5] = (param >> 8) & 0xFF;
  packet[6] = param & 0xFF;
  int16_t checksum = -(0xFF + 0x06 + cmd + packet[5] + packet[6]);
  packet[7] = (checksum >> 8) & 0xFF;
  packet[8] = checksum & 0xFF;
  packet[9] = 0xEF;
  for (int i = 0; i < 10; i++) dfWriteByte(packet[i]);
}

// Plays 001.mp3..010.mp3 (root of the SD card) by index — reverted back to
// this after 0x0F (folder-based playback) produced no sound at all on this
// particular DFPlayer clone. The ordering problem is instead fixed by how
// the files are copied onto the card — see the chat for exact steps.
void playTrack(uint8_t trackNumber) {
  dfSendCommand(0x03, trackNumber);
}

void setDfVolume(uint8_t level) { // 0..30
  dfSendCommand(0x06, level);
}
// ================= OLED =================
#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64
#define OLED_ADDR 0x3C
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, -1);
// ================= Motor Driver TB6612FNG =================
AF_DCMotor motorLeft(1);   // M1
AF_DCMotor motorRight(4);  // M4
#define LEFT_MOTOR  motorLeft
#define RIGHT_MOTOR motorRight
// ================= PIR =================
unsigned long lastPirClearTime = 0;
const unsigned long PIR_RETRIGGER_COOLDOWN = 1500; // 1.5 ثانیه
#define PIR_PIN A3
bool lastPirState = false;
bool pirIgnored   = false;
// ================= Bluetooth =================
// NOTE: BT and Serial are the SAME hardware UART on this board.
// Any Serial.print() goes to the HC-05 and will be received by the app.
// Debug output has been removed to avoid confusing the app parser.
#define BT Serial
// ================= Servo & Ultrasonic =================
#define TRIG_PIN  13
#define ECHO_PIN  9
#define SERVO_PIN 10
Servo radarServo;
// ================= Flame Sensor =================
#define FLAME_DO_PIN A2
#define FLAME_ACTIVE_STATE LOW
bool lastFlameDetected = false;
// ================= Edge Sensors =================
#define EDGE_FRONT_PIN 2
#define EDGE_BACK_PIN  A1
bool lastEdgeDetected = false;
bool flameIgnored     = false;
bool edgeIgnored      = false;
bool dangerFromSensor = false;
// ================= Per-sensor enable/disable (NEW) =================
// Each sensor can be independently toggled on/off over Bluetooth (commands
// below). PIR / flame / edge default ON and persist across mode changes —
// they are safety/environment sensors, not tied to a specific face mode.
// Servo sweep and ultrasonic reading default OFF and are set by mode
// commands (g/r/u), same as the old single "radarEnabled" flag used to
// work — but now they are two independent flags, so you can e.g. turn the
// servo sweep off while leaving the ultrasonic reading on (fixed forward
// reading, no sweeping).
bool pirSensorEnabled    = true;
bool flameSensorEnabled  = true;
bool edgeSensorsEnabled  = true;
bool servoSweepEnabled   = false;
bool ultrasonicEnabled   = false;
// ================= Face Modes =================
enum FaceMode {
  NORMAL, HAPPY, SAD, ANGRY, GUARD, RESCUE,
  DANGER, SLEEP, EDGE_WARNING, FRIENDLY
};
FaceMode currentMode           = NORMAL;
FaceMode returnModeAfterDanger = NORMAL;
FaceMode returnModeAfterEdge   = NORMAL;

// ================= FRIENDLY sub-mode =================
// Toggles 0(heart) <-> 1(flower) on every 'f' press — infinite, no exit.
int  friendlySubMode = 0;
bool heartBig        = false;
unsigned long heartBeatTime  = 0;
const unsigned long HEARTBEAT_MS = 380;
// ================= State / Sensor Reporting =================
// STATE:<label>   -> headline robot state, used for the app's top banner
// SENSOR:<name>:<ON|OFF|DETECTED|CLEARED> -> granular per-sensor events,
//                     used for the app's individual sensor dashboard cards
void sendSensorState(const char* label) {
  BT.print(F("STATE:"));
  BT.println(label);
}
void sendSensorEvent(const char* sensor, const char* state) {
  BT.print(F("SENSOR:"));
  BT.print(sensor);
  BT.print(F(":"));
  BT.println(state);
}
// ================= Wake Animation =================
bool wakeAnimActive = false;
int  wakeBlinkCount = 0;
unsigned long wakeBlinkTime = 0;
const int WAKE_BLINK_TOTAL       = 3;
const unsigned long WAKE_BLINK_MS = 120;
// ================= Dizzy Effect =================
unsigned long manualTurnStart  = 0;
bool          manualTurnActive = false;
unsigned long spinAccumMs      = 0;
const unsigned long DIZZY_THRESHOLD_MS = 5000;
const unsigned long DIZZY_DURATION_MS  = 4000;
bool dizzyActive = false;
unsigned long dizzyStart = 0;
void recordSpin(unsigned long ms) {
  spinAccumMs += ms;
  if (spinAccumMs >= DIZZY_THRESHOLD_MS) {
    dizzyActive  = true;
    dizzyStart   = millis();
    spinAccumMs  = 0;
  }
}
// ================= Face Timers =================
unsigned long lastBlinkTime = 0;
const unsigned long blinkInterval = 3000;
const unsigned long blinkDuration = 140;
unsigned long nextBlinkInterval = 2500;
unsigned long nextBlinkDuration = 120;
bool isBlinking = false;
unsigned long blinkStartTime = 0;
unsigned long lastAutoLookTime = 0;
const unsigned long autoLookInterval = 7000;
enum AutoLookStep { AUTO_NONE, AUTO_LEFT, AUTO_UP_RIGHT, AUTO_BACK };
AutoLookStep autoStep = AUTO_NONE;
unsigned long autoStepStartTime  = 0;
const unsigned long autoStepDuration = 350;
unsigned long lastActivityTime = 0;
FaceMode modeBeforeSleep = NORMAL;
unsigned long edgeAnimStart = 0;
const unsigned long edgeAnimDuration = 3000;
// ================= Danger =================
unsigned long dangerStartTime = 0;
unsigned long dangerFlashTime = 0;
const unsigned long dangerFlashInterval = 180;
const unsigned long dangerHoldMs        = 8000;
bool dangerFlashOn = true;
const unsigned long GUARD_90_DEG_MS = 380;
// ================= Radar =================
const int RADAR_MIN_ANGLE = 20;
const int RADAR_MAX_ANGLE = 160;
const int RADAR_STEP      = 2;
const unsigned long radarStepInterval = 40;
unsigned long lastRadarStepTime = 0;
int  radarAngle  = RADAR_MIN_ANGLE;
int  radarDir    = 1;
// ================= Auto Mode =================
bool autoMode = false;
enum AutoBehavior {
  AUTO_BEHAVIOR_NONE,
  AUTO_NORMAL,
  AUTO_GUARD,
  AUTO_RESCUE
};
AutoBehavior autoBehavior = AUTO_BEHAVIOR_NONE;
// Guard 4-direction patrol
// guardPatrolStep: 0=facing 0deg 1=90deg 2=180deg 3=270deg
// Every 30s turn 90 degrees right — stays in place, just rotates.
int  guardPatrolStep  = 0;
unsigned long lastGuardRotate    = 0;
const unsigned long guardRotateDelay = 30000UL;  // 30 seconds
// Idle movement for non-auto modes (NORMAL/HAPPY/SAD/ANGRY/FRIENDLY)
// After 2 min no command → small natural movement
unsigned long lastIdleMove = 0;
static int    idleStep     = 0;
bool idleSoundPlayed = false; // plays sound 008 once when idling begins
// ================= Robot Behaviour =================
unsigned long lastCommandTime = 0;
const unsigned long SLEEP_TIME = 240000UL;  // 4 minutes
const unsigned long IDLE_TIME  = 120000UL;  // 2 minutes
const unsigned long IDLE_STEP_INTERVAL = 40000; // 40 ثانیه بین هر حرکت
int lastMoveDirection = 0;  // 1=fwd 2=back 3=left 4=right
// ================= Motor Speed =================
#define MOTOR_SPEED 255
// ================= Forward declarations =================
void stopMotors() {
  motorLeft.run(RELEASE);
  motorRight.run(RELEASE);
}
void moveForward(int s) {
  lastMoveDirection = 1;
  motorLeft.setSpeed(s);
  motorRight.setSpeed(s);
  motorLeft.run(FORWARD);
  motorRight.run(FORWARD);
}
void moveBackward(int s) {
  lastMoveDirection = 2;
  motorLeft.setSpeed(s);
  motorRight.setSpeed(s);
  motorLeft.run(BACKWARD);
  motorRight.run(BACKWARD);
}
void turnLeft(int s) {
  lastMoveDirection = 3;
  motorLeft.setSpeed(s);
  motorRight.setSpeed(s);
  motorLeft.run(BACKWARD);
  motorRight.run(FORWARD);
}

void turnRight(int s) {
  lastMoveDirection = 4;
  motorLeft.setSpeed(s);
  motorRight.setSpeed(s);
  motorLeft.run(FORWARD);
  motorRight.run(BACKWARD);
}
void processCommand(char cmd);
void updateAnimations();
void updateRadar();
void updateFlameSensor();
void updateEdgeSensors();
void updatePirSensor();
void updateIdleMove();
void runAutoMode();
void avoidObstacleByAngle();
int  readDistanceCM();
void drawFace(FaceMode mode);
void drawNormalEyes(bool blink, int x, int y);
void drawHappyEyes(int x, int y);
void drawSadEyes(int x, int y);
void drawAngryEyes(int x, int y);
void drawSadMouth();
void drawAngryMouth();
void drawSmileMouth();
void drawFlatMouth();
void drawDangerTriangle();
void drawEdgeWarning();
void drawTiredEyes(int x, int y);
void drawTiredZS();
void drawHeart(bool big);
void drawFlower();
void drawStartup();
void drawDizzyFace();
void emergencyForward(int s) {
  motorLeft.setSpeed(s);
  motorRight.setSpeed(s);
  motorLeft.run(FORWARD);
  motorRight.run(FORWARD);
}
void emergencyBackward(int s) {
  motorLeft.setSpeed(s);
  motorRight.setSpeed(s);
  motorLeft.run(BACKWARD);
  motorRight.run(BACKWARD);
}
void resetRadar() {
  radarAngle = 90;
  radarDir   = -1;   // اول به سمت راست می‌رود؛ برای اول‌چپ radarDir=1 کن
  lastRadarStepTime = 0;
  radarServo.write(90);
}
// Sets servo-sweep / ultrasonic flags AND tells the app about it whenever the
// mode itself changes them (not just the manual p/q/e/w/x toggle commands) —
// otherwise the app's sensor switches go out of sync with what the robot is
// actually doing (e.g. switching to RESCUE silently turns both on).
void setServoUltrasonic(bool servoOn, bool ultraOn) {
  bool servoChanged = (servoSweepEnabled != servoOn);
  bool ultraChanged = (ultrasonicEnabled != ultraOn);
  servoSweepEnabled = servoOn;
  ultrasonicEnabled = ultraOn;
  if (servoChanged) sendSensorEvent("SERVO", servoOn ? "ON" : "OFF");
  if (ultraChanged) sendSensorEvent("ULTRASONIC", ultraOn ? "ON" : "OFF");
  if (!servoOn) { radarAngle = 90; radarServo.write(90); }
}
void resetFaceMotion() {
  isBlinking = false;
  autoStep   = AUTO_NONE;
  display.invertDisplay(false);
}
// ================= SETUP =================
void setup() {
  Serial.begin(9600);
  BT.begin(9600);
  pinMode(DF_TX_PIN, OUTPUT);
  digitalWrite(DF_TX_PIN, HIGH); // idle-high, matches UART idle state
  pinMode(TRIG_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT);
  digitalWrite(TRIG_PIN, LOW);
  pinMode(FLAME_DO_PIN, INPUT);
  pinMode(EDGE_FRONT_PIN, INPUT);
  pinMode(EDGE_BACK_PIN, INPUT);
  pinMode(PIR_PIN, INPUT);
  radarServo.attach(SERVO_PIN);
  radarServo.write(90);
  Wire.begin();
  Wire.setClock(100000);
  if (!display.begin(SSD1306_SWITCHCAPVCC, OLED_ADDR)) {
    for (;;);
  }
  drawStartup();
  display.clearDisplay();
  display.display();
  // DFPlayer needs ~1.5-2s after power-up to mount its SD card before it
  // will reliably accept commands — drawStartup() already used part of
  // that time, this tops it up before the boot sound.
  delay(700);
  setDfVolume(22);
  playTrack(1); // startup sound
  currentMode           = NORMAL;
  returnModeAfterDanger = NORMAL;
  lastBlinkTime         = millis();
  lastAutoLookTime      = millis();
  lastActivityTime      = millis();
  lastCommandTime       = millis();
  lastIdleMove          = millis();
  lastGuardRotate       = millis();
  randomSeed(analogRead(A0));
  wdt_enable(WDTO_2S);
}
// =============== LOOP =================
void loop() {
  while (BT.available() > 0) {
    char c = (char)BT.read();
    // NOTE: No Serial.print here — BT and Serial are the same UART,
    // any print would go to the app as garbage characters.
    if (currentMode == SLEEP) {
      currentMode = modeBeforeSleep;
      lastActivityTime = millis();
      wakeAnimActive = true;
      wakeBlinkCount = 0;
      wakeBlinkTime  = millis();
    }
    if (c != '\n' && c != '\r') {
      processCommand(c);
    }
  }
  if (!autoMode && millis() - lastCommandTime > SLEEP_TIME && currentMode != SLEEP) {
    stopMotors();
    autoMode = false;
    radarServo.write(90);
    modeBeforeSleep = currentMode;
    currentMode = SLEEP;
    playTrack(10);
  }

  updateEdgeSensors();
  updateFlameSensor();
  updatePirSensor();
  updateAnimations();
  updateRadar();
  updateIdleMove();
  if (autoMode) runAutoMode();
  drawFace(currentMode);
  wdt_reset();
}
// ================= COMMANDS =================
void processCommand(char cmd) {
  lastActivityTime = millis();
  lastCommandTime  = millis();
  idleSoundPlayed  = false; // any command means we're no longer idle
  // اگر قبلاً در حال چرخش دستی بودیم و الان فرمان دیگری اومده، مدت چرخش را ثبت کن
  if (manualTurnActive && cmd != 'j' && cmd != 'J' && cmd != 'l' && cmd != 'L') {
    recordSpin(millis() - manualTurnStart);
    manualTurnActive = false;
  }
  switch (cmd) {

    case 'n': case 'N':
      currentMode = NORMAL; returnModeAfterDanger = NORMAL;
      setServoUltrasonic(false, false); lastFlameDetected = false;
      resetRadar(); resetFaceMotion(); radarServo.write(90);
      break;

    case 'h': case 'H':
      currentMode = HAPPY; returnModeAfterDanger = HAPPY;
      setServoUltrasonic(false, false); lastFlameDetected = false;
      resetRadar(); resetFaceMotion(); radarServo.write(90);
      playTrack(2);
      break;

    case 's': case 'S':
      currentMode = SAD; returnModeAfterDanger = SAD;
      setServoUltrasonic(false, false); lastFlameDetected = false;
      resetRadar(); resetFaceMotion(); radarServo.write(90);
      break;

    case 'a': case 'A':
      currentMode = ANGRY; returnModeAfterDanger = ANGRY;
      setServoUltrasonic(false, false); lastFlameDetected = false;
      resetRadar(); resetFaceMotion(); radarServo.write(90);
      playTrack(4);
      break;

    case 'g': case 'G':
      currentMode = GUARD; returnModeAfterDanger = GUARD;
      setServoUltrasonic(true, true); lastFlameDetected = false;
      guardPatrolStep = 0; lastGuardRotate = millis();
      resetRadar(); resetFaceMotion();
      break;

    case 'r': case 'R':
      currentMode = RESCUE; returnModeAfterDanger = RESCUE;
      setServoUltrasonic(true, true); lastFlameDetected = false;
      resetRadar(); resetFaceMotion();
      break;

    // FRIENDLY: 0(heart) <-> 1(flower), toggles forever
    case 'f': case 'F':
      setServoUltrasonic(false, false); lastFlameDetected = false;
      resetRadar(); radarServo.write(90);
      if (currentMode != FRIENDLY) {
        currentMode     = FRIENDLY;
        friendlySubMode = 0;
        heartBeatTime   = millis();
        heartBig        = false;
        resetFaceMotion();
        playTrack(3);
      } else {
        friendlySubMode = (friendlySubMode == 0) ? 1 : 0;
      }
      break;

    case 'd': case 'D':
      if (currentMode != DANGER) {
        returnModeAfterDanger =
          (currentMode == GUARD || currentMode == RESCUE) ? currentMode : NORMAL;
        currentMode     = DANGER;
        dangerStartTime  = millis();
        dangerFlashTime  = millis();
        dangerFlashOn    = true;
        dangerFromSensor = false;
        resetFaceMotion();
      }
      break;

    case 'k': case 'K':
      lastPirClearTime = millis();
      flameIgnored = true;
      edgeIgnored  = true;
      pirIgnored   = true;
      if (currentMode == DANGER && dangerFromSensor) {
        currentMode = returnModeAfterDanger;
        sendSensorState("NORMAL");
      } else if (currentMode == EDGE_WARNING) {
        currentMode = returnModeAfterEdge;
        sendSensorState("NORMAL");
      }
      BT.println(F("ACK:CLEAR"));
      break;

    case 'i': case 'I': moveForward(MOTOR_SPEED);  break;
    case 'b': case 'B': moveBackward(MOTOR_SPEED); break;
    case 'j': case 'J':
    turnLeft(MOTOR_SPEED);
    manualTurnStart = millis();
    manualTurnActive = true;
    break;
    case 'l': case 'L':
    turnRight(MOTOR_SPEED);
    manualTurnStart = millis();
    manualTurnActive = true;
    break;
    case 't': case 'T': stopMotors();               break;

    case 'u':
      autoMode = true;
      setServoUltrasonic(true, true);

      if(currentMode == RESCUE){

        autoBehavior = AUTO_RESCUE;

      }
      else if(currentMode == GUARD){
        autoBehavior = AUTO_GUARD;

      }
      else{

        autoBehavior = AUTO_NORMAL;

      }

      BT.println(F("MODE:AUTO:ON"));

      break;

    case 'v': case 'V':
      autoMode     = false;
      autoBehavior = AUTO_BEHAVIOR_NONE;
      stopMotors();
      BT.println(F("MODE:AUTO:OFF"));
      break;

    // ============ NEW: per-sensor enable/disable toggles ============
    case 'p': case 'P':
      pirSensorEnabled = !pirSensorEnabled;
      if (!pirSensorEnabled) { lastPirState = false; pirIgnored = false; }
      sendSensorEvent("PIR", pirSensorEnabled ? "ON" : "OFF");
      break;

    case 'q': case 'Q':
      flameSensorEnabled = !flameSensorEnabled;
      if (!flameSensorEnabled) { lastFlameDetected = false; flameIgnored = false; }
      sendSensorEvent("FLAME", flameSensorEnabled ? "ON" : "OFF");
      break;

    case 'e': case 'E':
      edgeSensorsEnabled = !edgeSensorsEnabled;
      if (!edgeSensorsEnabled) { lastEdgeDetected = false; edgeIgnored = false; }
      sendSensorEvent("EDGE", edgeSensorsEnabled ? "ON" : "OFF");
      break;

    case 'w': case 'W':
      servoSweepEnabled = !servoSweepEnabled;
      if (!servoSweepEnabled) { radarAngle = 90; radarServo.write(90); }
      sendSensorEvent("SERVO", servoSweepEnabled ? "ON" : "OFF");
      break;

    case 'x': case 'X':
      ultrasonicEnabled = !ultrasonicEnabled;
      sendSensorEvent("ULTRASONIC", ultrasonicEnabled ? "ON" : "OFF");
      break;
  }
}

// ================= SENSORS =================
void updateFlameSensor() {
  if (!flameSensorEnabled) { lastFlameDetected = false; return; }
  if (currentMode == GUARD) { lastFlameDetected = false; return; } // pre-existing: guard mode ignores flame

  bool flameDetected = (digitalRead(FLAME_DO_PIN) == FLAME_ACTIVE_STATE);

  if (flameIgnored) {
    if (!flameDetected) flameIgnored = false;
    return;
  }

  if (flameDetected) {
    if (currentMode == SLEEP) {
      currentMode = modeBeforeSleep;
      lastActivityTime = millis();
      lastCommandTime = millis();
      wakeAnimActive = false;
    }
    if (!lastFlameDetected) {
      lastFlameDetected = true;
      sendSensorEvent("FLAME", "DETECTED");
      playTrack(9);
      if (currentMode != DANGER) {
        returnModeAfterDanger = currentMode;
        currentMode      = DANGER;
        dangerStartTime   = millis();
        dangerFlashTime   = millis();
        dangerFlashOn     = true;
        dangerFromSensor  = true;
        resetFaceMotion();
        stopMotors();
        sendSensorState("DANGER");
      }
    }
  } else {
    if (lastFlameDetected) sendSensorEvent("FLAME", "CLEARED");
    lastFlameDetected = false;
  }
}

// ============================================================
void updateEdgeSensors() {
  if (!edgeSensorsEnabled) { edgeIgnored = false; lastEdgeDetected = false; return; }

  bool frontEdge = digitalRead(EDGE_FRONT_PIN);
  bool backEdge  = digitalRead(EDGE_BACK_PIN);
  bool edgeDetected = (frontEdge == HIGH || backEdge == HIGH);

  if (edgeIgnored) {
    if (!edgeDetected) { edgeIgnored = false; lastEdgeDetected = false; }
    return;
  }

  if (edgeDetected) {
    if (currentMode == SLEEP) {
      currentMode = modeBeforeSleep;
      lastActivityTime = millis();
      lastCommandTime = millis();
      wakeAnimActive = false;
    }

    if (!lastEdgeDetected) {
      lastEdgeDetected = true;
      sendSensorEvent("EDGE", "DETECTED");
      playTrack(5);
    }

    if (currentMode != EDGE_WARNING) {
      returnModeAfterEdge = currentMode;
      currentMode = EDGE_WARNING;
      resetFaceMotion();
      sendSensorState("EDGE_WARNING");
      drawFace(EDGE_WARNING);

      // تا وقتی حسگر لبه فعاله، پیوسته در جهت مخالف حرکت کن
      // (سقف ایمنی ۳ ثانیه، تا اگر سنسور خراب/کثیف بود گیر نکنیم)
      unsigned long edgeMoveStart = millis();
      while ((digitalRead(EDGE_FRONT_PIN) == HIGH || digitalRead(EDGE_BACK_PIN) == HIGH)
             && millis() - edgeMoveStart < 3000) {
        if (digitalRead(EDGE_FRONT_PIN) == HIGH) emergencyBackward(255);
        else if (digitalRead(EDGE_BACK_PIN) == HIGH) emergencyForward(255);
        delay(50);
        wdt_reset();
      }

      // بعد از غیرفعال شدن سنسور، ۵۰۰ میلی‌ثانیه دیگر ادامه بده
      delay(500);
      wdt_reset();

      stopMotors();
      wdt_reset();

      edgeAnimStart = millis();
    }
  } else {
    if (lastEdgeDetected) {
      lastEdgeDetected = false;
      sendSensorEvent("EDGE", "CLEARED");
    }
  }
}
// ============================================================
void updatePirSensor() {
  if (!pirSensorEnabled) { lastPirState = false; return; }

  bool personDetected = (bool)digitalRead(PIR_PIN);

  // ── Step 1: check ignore flag regardless of detection ──
  if (pirIgnored) {
    if (!personDetected) pirIgnored = false;
    return;
  }

  // ── Step 2: rising edge — person just detected ──────────
  if (personDetected && !lastPirState && millis() - lastPirClearTime > PIR_RETRIGGER_COOLDOWN) {
    lastPirState = true;
    sendSensorEvent("PIR", "DETECTED");

    // PIR wakes robot from sleep
    if (currentMode == SLEEP) {
      currentMode      = modeBeforeSleep;
      lastCommandTime  = millis();
      lastActivityTime = millis();
      wakeAnimActive   = true;
      wakeBlinkCount   = 0;
      wakeBlinkTime    = millis();
    }

    // ── RESCUE: stop everything, send HELP, wait for command ─
    if (autoBehavior == AUTO_RESCUE) {
      stopMotors();
      autoMode = false;    // exit auto mode — waits for manual command
      setServoUltrasonic(false, ultrasonicEnabled); // stop sweeping, keep distance reading
      sendSensorState("PERSON_FOUND");
      playTrack(6);
      // Robot stays stopped until operator sends a new BT command
    }
    else if (currentMode == RESCUE && !autoMode) {
      sendSensorState("PERSON_FOUND");
      playTrack(6);
    }

    // ── GUARD: trigger DANGER alert ──────────────────────
    else if (autoBehavior == AUTO_GUARD || currentMode == GUARD) {
      if (autoMode) stopMotors();
      if (currentMode != DANGER) {
        returnModeAfterDanger = currentMode;
        currentMode     = DANGER;
        dangerStartTime  = millis();
        dangerFlashTime  = millis();
        dangerFlashOn    = true;
        dangerFromSensor = true;
        resetFaceMotion();
        sendSensorState("DANGER");
        playTrack(7);
      }
    }
  }

  if (!personDetected && lastPirState) {
    lastPirState = false;
    sendSensorEvent("PIR", "CLEARED");
  }
}

// ================= IDLE MOVEMENT =================

void updateIdleMove() {
  if (autoMode) return;
  if (currentMode == SLEEP    || currentMode == DANGER ||
      currentMode == EDGE_WARNING || currentMode == GUARD ||
      currentMode == RESCUE) return;

  if (millis() - lastCommandTime < IDLE_TIME) return;

  if (!idleSoundPlayed) {
    idleSoundPlayed = true;
    playTrack(8);
  }

  if (millis() - lastIdleMove   < IDLE_STEP_INTERVAL) return;

  lastIdleMove = millis();
  idleStep = (idleStep + 1) % 3;

  switch (idleStep) {
    case 0:
      // یک دور کامل (۳۶۰ درجه) — این عدد را با تست واقعی تنظیم کن
      turnRight(150);
      delay(750);
      stopMotors();
      wdt_reset();
      recordSpin(750);
      break;

    case 1:
      // چرخش ۱۰۰ درجه — این عدد را هم با تست واقعی تنظیم کن
      turnLeft(150);
      delay(420);
      stopMotors();
      wdt_reset();
      recordSpin(420);
      break;

    case 2:
      // عقب و جلو رفتن کوتاه
      moveBackward(150);
      delay(300);
      stopMotors();
      wdt_reset();
      delay(150);
      moveForward(150);
      delay(300);
      stopMotors();
      wdt_reset();
      break;
  }
}

// ================= ANIMATIONS =================
void updateAnimations() {
  unsigned long now = millis();

  if (wakeAnimActive) {
    if (now - wakeBlinkTime >= WAKE_BLINK_MS) {
      wakeBlinkCount++;
      wakeBlinkTime = now;
      if (wakeBlinkCount >= WAKE_BLINK_TOTAL * 2) wakeAnimActive = false;
    }
    return;
  }

  if (currentMode == FRIENDLY && friendlySubMode == 0) {
    if (now - heartBeatTime >= HEARTBEAT_MS) {
      heartBig     = !heartBig;
      heartBeatTime = now;
    }
  }

  if (currentMode == NORMAL) {
    if (autoStep == AUTO_NONE) {
      if (now - lastAutoLookTime >= autoLookInterval) {
        autoStep = AUTO_LEFT; autoStepStartTime = now; lastAutoLookTime = now;
      }
    } else {
      if (now - autoStepStartTime >= autoStepDuration) {
        switch (autoStep) {
          case AUTO_LEFT:     autoStep = AUTO_UP_RIGHT; break;
          case AUTO_UP_RIGHT: autoStep = AUTO_BACK;     break;
          case AUTO_BACK:     autoStep = AUTO_NONE;     break;
          default: break;
        }
        autoStepStartTime = now;
      }
    }
    if (!isBlinking) {
      if (now - lastBlinkTime >= nextBlinkInterval) {

        isBlinking = true;
        blinkStartTime = now;

        // پلک بعدی تصادفی بین ۲.۵ تا ۶ ثانیه
        nextBlinkInterval = random(2500, 6000);

        // مدت بسته بودن چشم بین ۹۰ تا ۱۸۰ میلی‌ثانیه
        nextBlinkDuration = random(90,180);
      }
    } else {
      if (now - blinkStartTime >= blinkDuration) { isBlinking = false; lastBlinkTime = now; }
    }
  } else {
    autoStep = AUTO_NONE; isBlinking = false;
  }

  if (currentMode == DANGER) {
    if (now - dangerFlashTime >= dangerFlashInterval) {
      dangerFlashOn = !dangerFlashOn; dangerFlashTime = now;
    }
    if (now - dangerStartTime >= dangerHoldMs) {
        currentMode = returnModeAfterDanger;
        display.invertDisplay(false);
        // وقتی از DANGER برگشتیم، اگر حالت GUARD یا RESCUE بود رادار دوباره فعال شود
        if (currentMode == GUARD || currentMode == RESCUE) {
          setServoUltrasonic(true, true);
          resetRadar();
        }
        if (dangerFromSensor) sendSensorState("NORMAL");
    }
  }

  if (currentMode == EDGE_WARNING) {
    if (now - edgeAnimStart >= edgeAnimDuration) {
      currentMode = returnModeAfterEdge;
      sendSensorState("NORMAL");
    }
  }
}

// ================= RADAR =================
void updateRadar() {
  if (!ultrasonicEnabled) return;
  unsigned long now = millis();
  if (now - lastRadarStepTime < radarStepInterval) return;
  lastRadarStepTime = now;

  if (servoSweepEnabled) {
    radarServo.write(radarAngle);
  } else {
    // servo sweep off but ultrasonic still on: park servo facing forward
    // and just keep reporting the fixed-forward distance
    radarAngle = 90;
    radarServo.write(90);
  }

  int distanceCM = readDistanceCM();

  BT.print(F("RADAR:"));
  BT.print(radarAngle);
  BT.print(",");
  BT.println(distanceCM);

  // تشخیص مانع در حین چرخش عادی رادار، فقط وقتی سروو هم واقعاً داره می‌چرخه
  if (servoSweepEnabled && autoMode &&
      (autoBehavior == AUTO_NORMAL || autoBehavior == AUTO_RESCUE || autoBehavior == AUTO_GUARD) &&
      currentMode != DANGER && currentMode != EDGE_WARNING && currentMode != SLEEP) {
    if (distanceCM > 0 && distanceCM < 15) {
      if (autoBehavior == AUTO_GUARD) {
        // نگهبان فقط کمی عقب می‌رود، سرجایش می‌ماند، نمی‌چرخد
        stopMotors(); delay(150); wdt_reset();
        moveBackward(160); delay(300); stopMotors(); wdt_reset();
        resetRadar();
      } else {
        avoidObstacleByAngle(radarAngle);
      }
      return;
    }
  }

  if (servoSweepEnabled) {
    if (radarDir > 0) {
      radarAngle += RADAR_STEP;
      if (radarAngle >= RADAR_MAX_ANGLE) { radarAngle = RADAR_MAX_ANGLE; radarDir = -1; }
    } else {
      radarAngle -= RADAR_STEP;
      if (radarAngle <= RADAR_MIN_ANGLE) { radarAngle = RADAR_MIN_ANGLE; radarDir = 1; }
    }
  }
}
// ================= SCAN & CHOOSE DIRECTION =================
// Servo scans 3 angles, picks the clearest, turns the robot that way.
// Servo 150° = robot's left side   |   Servo 30° = robot's right side
void avoidObstacleByAngle(int angle) {
  stopMotors();
  delay(200); wdt_reset();

  moveBackward(170);
  delay(500);
  stopMotors();
  wdt_reset();
  delay(300); wdt_reset();

  if (angle < 70) {
    // مانع سمت راست بود
    turnLeft(180); delay(400); stopMotors();
    recordSpin(400);
  } else if (angle > 110) {
    // مانع سمت چپ بود
    turnRight(180); delay(400); stopMotors();
    recordSpin(400);
  } else {
    // زاویه‌ی وسط — دو اندازه‌گیری سریع برای تصمیم
    radarServo.write(30); delay(250); wdt_reset();
    int dRight = readDistanceCM();
    radarServo.write(150); delay(400); wdt_reset();
    int dLeft = readDistanceCM();
    radarServo.write(90); delay(200); wdt_reset();

    if (dRight == 0) dRight = 400;
    if (dLeft == 0) dLeft = 400;

    if (dRight >= dLeft) { turnRight(180); delay(400); stopMotors(); }
    else                 { turnLeft(180);  delay(400); stopMotors(); }
  }
  wdt_reset();
  delay(400); wdt_reset();

  if (autoBehavior == AUTO_RESCUE) moveForward(160);
  else if (autoBehavior == AUTO_NORMAL) moveForward(170);
  // در GUARD حرکت رو به جلو انجام نمی‌شود، چون باید سرجایش بماند

  resetRadar();
}
// ================= AUTO MODE =================
void runAutoMode() {
  if (currentMode == DANGER || currentMode == EDGE_WARNING || currentMode == SLEEP) {
    stopMotors();
    return;
  }

  switch (autoBehavior) {

    // ── NORMAL auto: roam, avoid obstacles ────────────────
  case AUTO_NORMAL: {
    moveForward(170);
    break;
  }

    // ── GUARD auto: stay in place, rotate 90° every 30s ──
    // Obstacle too close → brief back-up only, not wandering.
    case AUTO_GUARD: {
      if (millis() - lastGuardRotate >= guardRotateDelay) {
        lastGuardRotate = millis();
        guardPatrolStep = (guardPatrolStep + 1) % 4;
        turnRight(180);
        delay(GUARD_90_DEG_MS);
        stopMotors();
        wdt_reset();
        recordSpin(GUARD_90_DEG_MS);
      } else {
        stopMotors();
      }
      break;
    }

    // ── RESCUE auto: push forward, avoid obstacles ────────
    // Goal is always to advance. Backs up + scans only when blocked.
    case AUTO_RESCUE: {
      moveForward(160);
      break;
    }

    default: break;
  }
}

int readDistanceCM() {
  digitalWrite(TRIG_PIN, LOW);
  delayMicroseconds(2);
  digitalWrite(TRIG_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG_PIN, LOW);
  unsigned long duration = pulseIn(ECHO_PIN, HIGH, 20000UL);
  if (duration == 0) return 0;
  int cm = (int)(duration * 0.0343f / 2.0f);
  return (cm > 0 && cm < 400) ? cm : 0;
}
// ================= DRAWING =================
void drawFace(FaceMode mode) {
  display.clearDisplay();
  if (dizzyActive) {
    if (millis() - dizzyStart >= DIZZY_DURATION_MS) {
      dizzyActive = false;
    } else {
      drawDizzyFace();
      display.display();
      return;
    }
  }

  if (wakeAnimActive) {
    drawNormalEyes((wakeBlinkCount % 2 == 0), 26, 18);
    drawFlatMouth();
    display.display();
    return;
  }

  int eyeX = 26, eyeY = 18;
  if (mode == NORMAL) {
    if (autoStep == AUTO_LEFT)            eyeX -= 10;
    else if (autoStep == AUTO_UP_RIGHT) { eyeX += 8; eyeY -= 6; }
  }

  switch (mode) {
    case NORMAL:  drawNormalEyes(isBlinking, eyeX, eyeY); drawFlatMouth();  break;
    case HAPPY:   drawHappyEyes(eyeX, eyeY);              drawSmileMouth(); break;
    case SAD:
      
      drawSadEyes(eyeX, eyeY); drawSadMouth(); break;
    case ANGRY:
    case GUARD:
      
      drawAngryEyes(eyeX, eyeY); drawAngryMouth(); break;
    case RESCUE:
      drawHappyEyes(eyeX, eyeY - 2); drawSmileMouth();
      display.drawRect(52, 0, 24, 18, SSD1306_WHITE);
      display.fillRect(62, 2, 4, 14, SSD1306_WHITE);
      display.fillRect(56, 7, 16, 4, SSD1306_WHITE);
      break;
    case FRIENDLY:
      if (friendlySubMode == 0) drawHeart(heartBig);
      else                      drawFlower();
      break;
    case DANGER:       drawDangerTriangle(); break;
    case SLEEP:
      drawTiredEyes(26, 18); drawFlatMouth(); drawTiredZS(); break;
    case EDGE_WARNING: drawEdgeWarning(); break;
    default: drawNormalEyes(false, eyeX, eyeY); drawFlatMouth(); break;
  }
  display.display();
}

void drawHeart(bool big) {
  int cx = 64, cy = 34;
  if (!big) {
    display.fillCircle(cx - 8,  cy - 5, 8,  SSD1306_WHITE);
    display.fillCircle(cx + 8,  cy - 5, 8,  SSD1306_WHITE);
    display.fillTriangle(cx-15, cy-1, cx+15, cy-1, cx, cy+14, SSD1306_WHITE);
  } else {
    display.fillCircle(cx - 10, cy - 6, 10, SSD1306_WHITE);
    display.fillCircle(cx + 10, cy - 6, 10, SSD1306_WHITE);
    display.fillTriangle(cx-18, cy-2, cx+18, cy-2, cx, cy+18, SSD1306_WHITE);
  }
}

void drawFlower() {
  int cx = 64, cy = 32;
  display.fillCircle(cx,      cy - 14, 8, SSD1306_WHITE);
  display.fillCircle(cx,      cy + 14, 8, SSD1306_WHITE);
  display.fillCircle(cx - 13, cy - 7,  8, SSD1306_WHITE);
  display.fillCircle(cx + 13, cy - 7,  8, SSD1306_WHITE);
  display.fillCircle(cx - 13, cy + 7,  8, SSD1306_WHITE);
  display.fillCircle(cx + 13, cy + 7,  8, SSD1306_WHITE);
  display.fillCircle(cx, cy, 10, SSD1306_BLACK);
  display.fillCircle(cx, cy,  7, SSD1306_WHITE);
  display.fillCircle(cx, cy,  3, SSD1306_BLACK);
}

void drawStartup() {
  for (int h = 2; h <= 26; h += 4) {
    display.clearDisplay();
    int y = 18 + (26 - h) / 2;
    display.fillRoundRect(26, y, 22, h, 5, SSD1306_WHITE);
    display.fillRoundRect(80, y, 22, h, 5, SSD1306_WHITE);
    display.display();
    delay(40);
    wdt_reset();
  }
  display.clearDisplay();
  display.fillRoundRect(26, 30, 22, 3, 2, SSD1306_WHITE);
  display.fillRoundRect(80, 30, 22, 3, 2, SSD1306_WHITE);
  display.display();
  delay(120); wdt_reset();
  display.clearDisplay();
  display.fillRoundRect(26, 18, 22, 26, 7, SSD1306_WHITE);
  display.fillRoundRect(80, 18, 22, 26, 7, SSD1306_WHITE);
  display.fillCircle(37, 31, 4, SSD1306_BLACK);
  display.fillCircle(91, 31, 4, SSD1306_BLACK);
  display.display();
  delay(400); wdt_reset();
}
void drawNormalEyes(bool blink, int x, int y)
{
  if(blink)
  {
    display.fillRoundRect(x, y+12, 22, 3, 2, SSD1306_WHITE);
    display.fillRoundRect(x+54, y+12, 22, 3, 2, SSD1306_WHITE);
    return;
  }

  // قاب چشم
  display.fillRoundRect(x, y, 22, 26, 7, SSD1306_WHITE);
  display.fillRoundRect(x+54, y, 22, 26, 7, SSD1306_WHITE);
  // مردمک
  int pupilOffsetX = 0;
  int pupilOffsetY = 0;
  display.fillCircle(x+11+pupilOffsetX, y+13+pupilOffsetY, 5, SSD1306_BLACK);
  display.fillCircle(x+65+pupilOffsetX, y+13+pupilOffsetY, 5, SSD1306_BLACK);
  // برق چشم
  display.fillCircle(x+9, y+11, 2, SSD1306_WHITE);
  display.fillCircle(x+63, y+11, 2, SSD1306_WHITE);
  int browLeftY  = y - 4;
  int browRightY = y - 4;
  if (autoStep == AUTO_LEFT) {
  browLeftY  += 2;   // ابروی چپ پایین‌تر
  browRightY -= 2;   // ابروی راست بالاتر
  }
  else if (autoStep == AUTO_UP_RIGHT) {
    browLeftY  -= 2;
    browRightY -= 2;
  }
  display.drawLine(x-2,     browLeftY,  x+22,     browLeftY, SSD1306_WHITE);
  display.drawLine(x+52,    browRightY, x+76,   browRightY,   SSD1306_WHITE);
  // برق دوم
  display.drawPixel(x+13, y+16, SSD1306_WHITE);
  display.drawPixel(x+67, y+16, SSD1306_WHITE);
}
void drawHappyEyes(int x,int y){
  // چشم چپ
  display.drawLine(x,y+16,x+8,y+8,SSD1306_WHITE);
  display.drawLine(x+8,y+8,x+16,y+16,SSD1306_WHITE);
  // ضخیم شدن
  display.drawLine(x,y+17,x+8,y+9,SSD1306_WHITE);
  display.drawLine(x+8,y+9,x+16,y+17,SSD1306_WHITE);
  // چشم راست
  x+=54;
  display.drawLine(x,y+16,x+8,y+8,SSD1306_WHITE);
  display.drawLine(x+8,y+8,x+16,y+16,SSD1306_WHITE);
  display.drawLine(x,y+17,x+8,y+9,SSD1306_WHITE);
  display.drawLine(x+8,y+9,x+16,y+17,SSD1306_WHITE);
  // گونه
  display.fillCircle(30,47,2,SSD1306_WHITE);
  display.fillCircle(98,47,2,SSD1306_WHITE);
}
void drawSadEyes(int x,int y){

  // فرم چشم مثل NORMAL
  display.fillRoundRect(26,20,22,26,7,SSD1306_WHITE);
  display.fillRoundRect(80,20,22,26,7,SSD1306_WHITE);
  // مردمک بزرگ‌تر و پایین
  display.fillCircle(37,35,6,SSD1306_BLACK);
  display.fillCircle(91,35,6,SSD1306_BLACK);

  // برق چشم
  display.fillCircle(35,31,3,SSD1306_WHITE);
  display.fillCircle(89,31,3,SSD1306_WHITE);

  display.drawPixel(38,34,SSD1306_WHITE);
  display.drawPixel(92,34,SSD1306_WHITE);
  

  // ابروی چپ
  display.drawLine(26,20,48,17,SSD1306_WHITE);

  // ابروی راست
  display.drawLine(80,17,102,20,SSD1306_WHITE);
}
void drawAngryEyes(int x,int y){

  // فرم چشم مثل حالت عادی
  display.fillRoundRect(26,20,22,26,7,SSD1306_WHITE);
  display.fillRoundRect(80,20,22,26,7,SSD1306_WHITE);

  // مردمک
  display.fillCircle(37,31,4,SSD1306_BLACK);
  display.fillCircle(91,31,4,SSD1306_BLACK);

  // برق کوچک چشم
  display.fillCircle(35,29,2,SSD1306_WHITE);
  display.fillCircle(89,29,2,SSD1306_WHITE);

  // ابروی چپ
  display.drawLine(24,14,48,19,SSD1306_WHITE);
  display.drawLine(24,15,48,20,SSD1306_WHITE);
  display.drawLine(24,16,48,21,SSD1306_WHITE);

  // ابروی راست
  display.drawLine(80,19,104,14,SSD1306_WHITE);
  display.drawLine(80,20,104,15,SSD1306_WHITE);
  display.drawLine(80,21,104,16,SSD1306_WHITE);
}
void drawSadMouth(){

  for(int i=0;i<24;i++){
    int yy = 50 - 2*sin(i*0.13);

    display.drawPixel(52+i,yy,SSD1306_WHITE);

  }

}
void drawAngryMouth(){
  display.drawLine(50,51,64,46,SSD1306_WHITE);
  display.drawLine(64,46,78,51,SSD1306_WHITE);
  display.drawLine(50,52,64,47,SSD1306_WHITE);
  display.drawLine(64,47,78,52,SSD1306_WHITE);
}
void drawSmileMouth(){
  for(int i=0;i<26;i++){
    int yy=48+3*sin(i*0.13);
    display.drawPixel(51+i,yy,SSD1306_WHITE);
    display.drawPixel(51+i,yy+1,SSD1306_WHITE);
  }
}
void drawFlatMouth()  { display.drawLine(52,50,76,50,SSD1306_WHITE); }
void drawTiredEyes(int x, int y) {
  display.fillRoundRect(x,      y+24, 22, 4, 2, SSD1306_WHITE);
  display.fillRoundRect(x + 54, y+24, 22, 4, 2, SSD1306_WHITE);
}
void drawTiredZS() {
  int phase = (millis()/300) % 20;
  display.setTextColor(SSD1306_WHITE);
  display.setTextSize(1); display.setCursor(95, 40-phase); display.print("z");
  display.setTextSize(2); display.setCursor(105,25-phase); display.print("Z");
  display.setTextSize(3); display.setCursor(115, 5-phase); display.print("Z");
}
void drawDangerTriangle() {
  if (dangerFlashOn) {
    display.fillTriangle(64,5,20,55,108,55,SSD1306_WHITE);
    display.setTextSize(3); display.setTextColor(SSD1306_BLACK);
    display.setCursor(57,24); display.print("!");
  } else {
    display.drawTriangle(64,5,20,55,108,55,SSD1306_WHITE);
    display.setTextSize(2); display.setTextColor(SSD1306_WHITE);
    display.setCursor(53,26); display.print("...");
  }
}
void drawDizzyFace() {
  display.clearDisplay();
  int cx1 = 37;
  int cx2 = 91;
  int cy  = 31;
  float rot = millis() * 0.012;
  //====================
  // LEFT SPIRAL
  //====================
  display.drawCircle(cx1, cy, 12, SSD1306_WHITE);
  for(float a=0;a<6.5*PI;a+=0.20){
      float r=a*0.55;
      int x=cx1+cos(a+rot)*r;
      int y=cy+sin(a+rot)*r;
      display.drawPixel(x,y,SSD1306_WHITE);
  }
  //====================
  // RIGHT SPIRAL
  //====================
  display.drawCircle(cx2, cy, 12, SSD1306_WHITE);
  for(float a=0;a<6.5*PI;a+=0.20){
      float r=a*0.55;
      int x=cx2+cos(a+rot)*r;
      int y=cy+sin(a+rot)*r;
      display.drawPixel(x,y,SSD1306_WHITE);
  }
  //====================
  // WAVY MOUTH
  //====================
  for(int i=0;i<30;i++){
      int x=49+i;
      int y=50+2*sin((i*0.6)+(millis()*0.01));
      display.drawPixel(x,y,SSD1306_WHITE);
  }
  //====================
  // SPINNING STARS
  //====================
  float s=millis()*0.006;
  display.fillCircle(64+cos(s)*28,10+sin(s)*6,2,SSD1306_WHITE);
  display.fillCircle(64+cos(s+2.09)*28,10+sin(s+2.09)*6,2,SSD1306_WHITE);
  display.fillCircle(64+cos(s+4.18)*28,10+sin(s+4.18)*6,2,SSD1306_WHITE);
  //====================
  // SWEAT DROP
  //====================
  display.fillCircle(108,18,2,SSD1306_WHITE);
  display.drawLine(108,20,106,24,SSD1306_WHITE);
  display.drawLine(108,20,110,24,SSD1306_WHITE);
  display.display();
}
void drawEdgeWarning() {
  unsigned long frame = (millis() - edgeAnimStart) / 250;
  display.drawLine(0,50,95,50,SSD1306_WHITE);
  if (frame < 4) {
    int x = 15 + frame*18;
    display.fillRect(x,40,18,8,SSD1306_WHITE);
    display.drawCircle(x+4,50,2,SSD1306_WHITE);
    display.drawCircle(x+14,50,2,SSD1306_WHITE);
  } else if (frame < 8) {
    int x = 85+(frame-4)*8, y = 42+(frame-4)*5;
    display.fillRect(x,y,18,8,SSD1306_WHITE);
    display.drawCircle(x+4,y+10,2,SSD1306_WHITE);
    display.drawCircle(x+14,y+10,2,SSD1306_WHITE);
  } else {
    display.setTextSize(2); display.setCursor(18,18); display.print("EDGE!");
    display.setTextSize(1); display.setCursor(20,42); display.print("Watch Out!");
  }
}

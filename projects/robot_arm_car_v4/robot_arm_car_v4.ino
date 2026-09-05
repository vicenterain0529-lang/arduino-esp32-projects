/**
 * ============================================================
 *  ROBOT ARM SMART CAR — COMPETITION BUILD v4
 * ============================================================
 *  CHANGES FROM v3:
 *
 *  [DUAL DEBUG SYSTEM]
 *  The JDY-16 shares hardware Serial (pins 0/1) with USB.
 *  To allow BOTH USB and BLE to work simultaneously, the
 *  JDY-16 is moved to SoftwareSerial on pins A2/A3.
 *
 *  ★ WIRING CHANGE FOR v4:
 *    JDY-16 TX  →  Arduino A2  (BT_RX in code)
 *    JDY-16 RX  →  Arduino A3  (BT_TX in code)
 *    JDY-16 VCC →  3.3V
 *    JDY-16 GND →  GND
 *
 *  USB Serial (pins 0/1) → Arduino IDE Serial Monitor @ 9600
 *  BT Serial  (A2/A3)    → JDY-16 Bluetooth Terminal  @ 9600
 *  Both channels: receive commands AND print all debug output.
 *
 *  [DEBUG & CALIBRATION COMMANDS]
 *  Works from EITHER USB monitor OR Bluetooth terminal:
 *
 *    ?   Full system status snapshot
 *          Prints: speeds, servo angles, sensor raw values,
 *          distance, calibration value, claw carry state,
 *          motor speeds, PID values, memory count, telem state
 *
 *    V   Single sensor snapshot (L/C/R + distance, one shot)
 *
 *    P   Toggle live telemetry stream ON/OFF
 *          Streams L/C/R + dist + servo angles every 200ms
 *          Also streams inside all autonomous modes
 *
 *    K   Auto-calibrate line sensors
 *          Place robot on FLOOR (not on line), send K.
 *          Waits 2s, samples 30 times, sets BLACK_LINE_VAL.
 *
 *    Q   Calibrate claw carry threshold
 *          Close claw until gripping object (send 'c'),
 *          then send Q to save current angle as threshold.
 *          Auto-relax suppressed when claw is at/past this angle.
 *
 *    ~   Reset ALL calibration to compile-time defaults
 *
 *  [ALL v3 COMMANDS STILL WORK ON BOTH CHANNELS]
 *    ARM:   o c u d l r H
 *    DRIVE: F B L R S
 *    SPEED: X Y Z
 *    MODES: A D W T G
 *    MEM:   m a
 *
 * ============================================================
 *  Pin Map:
 *    Motor A      : DIR=2,  PWM=5
 *    Motor B      : DIR=4,  PWM=6
 *    Servo        : Claw=9, Arm=10, Base=11
 *    Tracking     : Left=7, Center=8, Right=A1
 *    Ultrasonic   : Trig=12, Echo=13
 *    IR Receiver  : 3
 *    BT SoftSerial: RX=A2 (←JDY TX), TX=A3 (→JDY RX)  ← NEW
 * ============================================================
 */

#include "IR_remote.h"
#include "keymap.h"
#include <Servo.h>
#include <SoftwareSerial.h>

// ============================================================
//  *** COMPETITION TUNING ZONE ***
//  ★ ONLY CHANGE VALUES IN THIS SECTION AT COMPETITION ★
//  Everything below this block should NOT be touched.
// ============================================================

// ── ① DRIVE SPEEDS ──────────────────────────────────────────
// Range 0–255. Keep below 220 to avoid motor stress.
// ★ TWEAK if robot is too fast/slow on competition surface.
#define SPEED_LOW           80    // 'X' slow mode
#define SPEED_MEDIUM        140   // 'Y' medium mode
#define SPEED_HIGH          190   // 'Z' fast mode

// ── ② SPEED RAMPING ─────────────────────────────────────────
// ★ Wheels slip at start   → lower RAMP_STEP (try 5)
// ★ Ramping feels sluggish → raise RAMP_STEP (try 15)
#define RAMP_STEP           10
#define RAMP_DELAY_MS       10

// ── ③ PID LINE TRACKING ─────────────────────────────────────
// Tune order: KP → KD → KI (only if drifting on straights)
// ★ Enable 'P' telemetry to watch error values while tuning
#define PID_KP              28.0
#define PID_KD              12.0
#define PID_KI              0.0
#define PID_BASE_SPEED      140   // ★ TWEAK for course speed
#define PID_MAX_SPEED       200
#define PID_MIN_SPEED       35
#define TRACK_SEARCH_MS     700

// ── ④ OBSTACLE AVOIDANCE ────────────────────────────────────
// ★ Robot bumps into things → lower AVOID_BACKUP_DIST
// ★ Robot turns too early   → raise AVOID_TURN_DIST
#define AVOID_BACKUP_DIST   15
#define AVOID_TURN_DIST     28
#define AVOID_FORWARD_SPEED 120
#define AVOID_BACKUP_MS     700
#define AVOID_TURN_MS       750

// ── ⑤ SMART SWEEP ───────────────────────────────────────────
// ★ Doesn't rotate far enough to see → raise SWEEP_PEEK_MS
#define SWEEP_PEEK_MS       380
#define SWEEP_PEEK_SPEED    100

// ── ⑥ STUCK DETECTION ───────────────────────────────────────
// ★ False stucks on slippery surfaces → raise STUCK_ATTEMPT_LIMIT
#define STUCK_ATTEMPT_LIMIT   4
#define STUCK_ESCAPE_BACK_MS  1000
#define STUCK_ESCAPE_TURN_MS  900

// ── ⑦ OBSTACLE MEMORY ───────────────────────────────────────
#define OBSTACLE_MEMORY_SIZE  5
#define OBSTACLE_ZONE_MARGIN  8

// ── ⑧ OBJECT FOLLOWING ──────────────────────────────────────
// ★ Robot overshoots target → tighten FOLLOW_STOP_MAX
#define FOLLOW_TOO_CLOSE    15
#define FOLLOW_STOP_MIN     15
#define FOLLOW_STOP_MAX     22
#define FOLLOW_SLOW_MAX     28
#define FOLLOW_FAST_MAX     35
#define FOLLOW_SPEED_SLOW   100
#define FOLLOW_SPEED_FAST   130
#define FOLLOW_SPEED_BACK   100

// ── ⑨ ANTI-DROP ─────────────────────────────────────────────
#define ANTIDROP_SPEED      100
#define ANTIDROP_BACK_MS    600
#define ANTIDROP_TURN_MS    500

// ── ⑩ SERVO MOTION & LIMITS ─────────────────────────────────
// Do NOT set SERVO_STEP_MS below 10 — servos will skip steps
#define SERVO_STEP_MS       12
#define CLAW_MIN            50
#define CLAW_MAX            180
#define ARM_MIN             0
#define ARM_MAX             180
#define BASE_MIN            0
#define BASE_MAX            180
#define SERVO_INIT          90

// ── ⑪ CLAW CARRY THRESHOLD ──────────────────────────────────
// ★ CRITICAL: angle at which claw IS gripping something.
// Auto-relax is suppressed while gripping to prevent drops.
// ★ Use 'Q' command to calibrate this live.
// ★ Manual: set to angle where claw firmly holds your object.
//    Default 120. If objects drop during idle: lower this value.
#define CLAW_CARRYING_THRESHOLD_DEFAULT 120

// ── ⑫ SERVO AUTO-RELAX TIMEOUT ──────────────────────────────
// ★ Servos overheat during idle     → lower to 3000
// ★ Arm drops unexpectedly during run → raise to 8000
#define ARM_IDLE_TIMEOUT_MS 5000

// ── ⑬ ARM MEMORY ────────────────────────────────────────────
#define MAX_ACTIONS 20

// ── ⑭ TELEMETRY RATE ────────────────────────────────────────
// How often live telemetry ('P' mode) prints in milliseconds.
// ★ Lower = faster updates. Below 100ms may slow autonomous modes.
#define TELEM_INTERVAL_MS   200

// ============================================================
//  PIN DEFINITIONS — only change if you physically rewired
// ============================================================
#define PIN_MOTOR_A_DIR     2
#define PIN_MOTOR_A_PWM     5
#define PIN_MOTOR_B_DIR     4
#define PIN_MOTOR_B_PWM     6
#define PIN_SERVO_CLAW      9
#define PIN_SERVO_ARM       10
#define PIN_SERVO_BASE      11
#define PIN_TRAK_LEFT       7
#define PIN_TRAK_CENTER     8
#define PIN_TRAK_RIGHT      A1
#define PIN_ULTRASONIC_TRIG 12
#define PIN_ULTRASONIC_ECHO 13
#define PIN_IR_RECV         3
#define PIN_BT_RX           A2   // ← Connect to JDY-16 TX pin
#define PIN_BT_TX           A3   // ← Connect to JDY-16 RX pin

// ============================================================
//  GLOBAL OBJECTS & STATE
// ============================================================
SoftwareSerial BT(PIN_BT_RX, PIN_BT_TX);

IRremote ir(PIN_IR_RECV);

Servo servoClaw;
Servo servoArm;
Servo servoBase;

int clawDeg  = SERVO_INIT;
int armDeg   = SERVO_INIT;
int baseDeg  = SERVO_INIT;
int speedCar = SPEED_LOW;

int BLACK_LINE_VAL      = 1;
int clawCarryThreshold  = CLAW_CARRYING_THRESHOLD_DEFAULT;

int recClaw[MAX_ACTIONS];
int recArm [MAX_ACTIONS];
int recBase[MAX_ACTIONS];
int recCount  = 0;
int autoCount = 0;

float obstacleMemory[OBSTACLE_MEMORY_SIZE];
int   obstacleMemIdx  = 0;
bool  obstacleMemFull = false;

int currentSpeedA = 0;
int currentSpeedB = 0;
int lastDirA      = -1;
int lastDirB      = -1;

String usbBuffer = "";
String btBuffer  = "";

unsigned long lastArmActivityTime = 0;
bool servosRelaxed = false;

bool          telemActive   = false;
unsigned long lastTelemTime = 0;

const char* currentMode = "IDLE";

// ============================================================
//  DUAL OUTPUT — DBG() / DBGLN() print to USB + BT together
// ============================================================

void DBG   (const String &s) { Serial.print(s);   BT.print(s);   }
void DBGLN (const String &s) { Serial.println(s); BT.println(s); }
void DBG   (int   v)         { Serial.print(v);   BT.print(v);   }
void DBGLN (int   v)         { Serial.println(v); BT.println(v); }
void DBG   (float v)         { Serial.print(v);   BT.print(v);   }
void DBGLN (float v)         { Serial.println(v); BT.println(v); }
void DBGLN ()                { Serial.println();  BT.println();  }

// ============================================================
//  TELEMETRY
// ============================================================

/**
 * One-line sensor snapshot — printed by 'V', 'P' tick, and inside modes.
 * Format: [MODE] L:x C:x R:x | DIST:xx.xcm | CLAW:xxx ARM:xxx BASE:xxx
 */
void printTelemetry() {
  int L = digitalRead(PIN_TRAK_LEFT);
  int C = digitalRead(PIN_TRAK_CENTER);
  int R = digitalRead(PIN_TRAK_RIGHT);

  // Non-blocking distance read with 20ms timeout
  digitalWrite(PIN_ULTRASONIC_TRIG, LOW);  delayMicroseconds(2);
  digitalWrite(PIN_ULTRASONIC_TRIG, HIGH); delayMicroseconds(10);
  digitalWrite(PIN_ULTRASONIC_TRIG, LOW);
  float dist = pulseIn(PIN_ULTRASONIC_ECHO, HIGH, 20000) / 58.0f;

  DBG("["); DBG(currentMode); DBG("] ");
  DBG("L:"); DBG(L); DBG(" C:"); DBG(C); DBG(" R:"); DBG(R);
  DBG(" | DIST:"); DBG(dist); DBG("cm");
  DBG(" | CLAW:"); DBG(clawDeg);
  DBG(" ARM:"); DBG(armDeg);
  DBG(" BASE:"); DBG(baseDeg);
  DBGLN();
}

/**
 * Full status snapshot — '?' command.
 */
void printStatus() {
  float dist = checkDistance();
  int L = digitalRead(PIN_TRAK_LEFT);
  int C = digitalRead(PIN_TRAK_CENTER);
  int R = digitalRead(PIN_TRAK_RIGHT);

  DBGLN("========== SYSTEM STATUS ==========");
  DBG("Mode          : "); DBGLN(currentMode);
  DBGLN("--- SPEEDS ---");
  DBG("Current speed : "); DBGLN(speedCar);
  DBG("LOW/MED/HIGH  : "); DBG(SPEED_LOW); DBG("/"); DBG(SPEED_MEDIUM); DBG("/"); DBGLN(SPEED_HIGH);
  DBGLN("--- SENSORS ---");
  DBG("Track L/C/R   : "); DBG(L); DBG(" / "); DBG(C); DBG(" / "); DBGLN(R);
  DBG("BLACK_LINE_VAL: "); DBGLN(BLACK_LINE_VAL);
  DBG("L on line?    : "); DBGLN(L == BLACK_LINE_VAL ? "YES" : "no");
  DBG("C on line?    : "); DBGLN(C == BLACK_LINE_VAL ? "YES" : "no");
  DBG("R on line?    : "); DBGLN(R == BLACK_LINE_VAL ? "YES" : "no");
  DBG("Distance      : "); DBG(dist); DBGLN("cm");
  DBGLN("--- SERVOS ---");
  DBG("Claw / Arm / Base : "); DBG(clawDeg); DBG("/ "); DBG(armDeg); DBG("/ "); DBGLN(baseDeg);
  DBG("Servo state   : "); DBGLN(servosRelaxed ? "RELAXED (detached)" : "ACTIVE (attached)");
  DBG("Carry thresh  : "); DBG(clawCarryThreshold); DBGLN("deg");
  DBG("Carrying now? : "); DBGLN(clawDeg >= clawCarryThreshold ? "YES — relax suppressed" : "no");
  DBGLN("--- MOTORS ---");
  DBG("Motor A / B   : "); DBG(currentSpeedA); DBG(" / "); DBGLN(currentSpeedB);
  DBGLN("--- PID ---");
  DBG("KP / KD / KI  : "); DBG((float)PID_KP); DBG(" / "); DBG((float)PID_KD); DBG(" / "); DBGLN((float)PID_KI);
  DBG("Base speed    : "); DBGLN(PID_BASE_SPEED);
  DBGLN("--- MEMORY ---");
  DBG("Snapshots     : "); DBG(autoCount); DBG(" / "); DBGLN(MAX_ACTIONS);
  DBG("Telemetry     : "); DBGLN(telemActive ? "ON  (P to stop)" : "OFF (P to start)");
  DBGLN("====================================");
  DBGLN("CMDS: ? V P | K Q ~ | F B L R S | X Y Z | o c u d l r H | A D W T G | m a");
}

// ============================================================
//  UTILITY
// ============================================================

inline bool isBlack(int v) { return v == BLACK_LINE_VAL; }

void flushAllSerial() {
  while (Serial.available()) Serial.read();
  while (BT.available())     BT.read();
}

/** Read next char from USB first, then BT. Returns 0 if nothing. */
char readCmd() {
  if (Serial.available()) return (char)Serial.read();
  if (BT.available())     return (char)BT.read();
  return 0;
}

// ============================================================
//  MOTOR FUNCTIONS
// ============================================================

void driveStop() {
  analogWrite(PIN_MOTOR_A_PWM, 0);
  analogWrite(PIN_MOTOR_B_PWM, 0);
  digitalWrite(PIN_MOTOR_A_DIR, LOW);
  digitalWrite(PIN_MOTOR_B_DIR, HIGH);
  currentSpeedA = 0; currentSpeedB = 0;
  lastDirA = -1;     lastDirB = -1;
}

void rampMotors(int dirA, int dirB, int targetA, int targetB) {
  const int step = max(1, RAMP_STEP);
  digitalWrite(PIN_MOTOR_A_DIR, dirA);
  digitalWrite(PIN_MOTOR_B_DIR, dirB);
  if (dirA != lastDirA || dirB != lastDirB) {
    currentSpeedA = 0; currentSpeedB = 0;
    lastDirA = dirA;   lastDirB = dirB;
  }
  while (currentSpeedA != targetA || currentSpeedB != targetB) {
    if (currentSpeedA < targetA) currentSpeedA = min(currentSpeedA + step, targetA);
    if (currentSpeedA > targetA) currentSpeedA = max(currentSpeedA - step, targetA);
    if (currentSpeedB < targetB) currentSpeedB = min(currentSpeedB + step, targetB);
    if (currentSpeedB > targetB) currentSpeedB = max(currentSpeedB - step, targetB);
    analogWrite(PIN_MOTOR_A_PWM, currentSpeedA);
    analogWrite(PIN_MOTOR_B_PWM, currentSpeedB);
    delay(RAMP_DELAY_MS);
  }
}

void moveForward (int spd) { rampMotors(HIGH, LOW,  spd, spd); }
void moveBackward(int spd) { rampMotors(LOW,  HIGH, spd, spd); }
void rotateLeft  (int spd) { rampMotors(LOW,  LOW,  spd, spd); }
void rotateRight (int spd) { rampMotors(HIGH, HIGH, spd, spd); }

void setMotorSpeeds(int leftSpd, int rightSpd) {
  leftSpd  = constrain(leftSpd,  -255, 255);
  rightSpd = constrain(rightSpd, -255, 255);
  digitalWrite(PIN_MOTOR_A_DIR, leftSpd  >= 0 ? HIGH : LOW);
  analogWrite (PIN_MOTOR_A_PWM, abs(leftSpd));
  digitalWrite(PIN_MOTOR_B_DIR, rightSpd >= 0 ? LOW  : HIGH);
  analogWrite (PIN_MOTOR_B_PWM, abs(rightSpd));
}

// ============================================================
//  ULTRASONIC SENSOR
// ============================================================

float checkDistance() {
  digitalWrite(PIN_ULTRASONIC_TRIG, LOW);  delayMicroseconds(2);
  digitalWrite(PIN_ULTRASONIC_TRIG, HIGH); delayMicroseconds(10);
  digitalWrite(PIN_ULTRASONIC_TRIG, LOW);
  float d = pulseIn(PIN_ULTRASONIC_ECHO, HIGH) / 58.0f;
  delay(10);
  return d;
}

// ============================================================
//  SERVO HELPERS
// ============================================================

bool servoMoveTo(Servo &sv, int &current, int target, int minDeg, int maxDeg) {
  target = constrain(target, minDeg, maxDeg);
  int dir = (target > current) ? 1 : -1;
  while (current != target) {
    current += dir;
    sv.write(current);
    delay(SERVO_STEP_MS);
    if (readCmd() == 's') return false;
  }
  return true;
}

void servoContinuous(Servo &sv, int &deg, int direction, int minDeg, int maxDeg) {
  while (true) {
    if (direction > 0 && deg >= maxDeg) break;
    if (direction < 0 && deg <= minDeg) break;
    deg = constrain(deg + direction, minDeg, maxDeg);
    sv.write(deg);
    DBGLN(deg);   // Streams live angle to both debug channels
    delay(10);
    if (readCmd() == 's') break;
  }
}

void wakeServos() {
  lastArmActivityTime = millis();
  if (servosRelaxed) {
    servoClaw.attach(PIN_SERVO_CLAW);
    servoArm .attach(PIN_SERVO_ARM);
    servoBase.attach(PIN_SERVO_BASE);
    servoClaw.write(clawDeg);
    servoArm .write(armDeg);
    servoBase.write(baseDeg);
    servosRelaxed = false;
    delay(150);
    DBGLN("SERVOS_ACTIVE");
  } else {
    lastArmActivityTime = millis();
  }
}

bool servoNeedsRelax() {
  if (servosRelaxed) return false;
  if (millis() - lastArmActivityTime <= ARM_IDLE_TIMEOUT_MS) return false;
  if (clawDeg >= clawCarryThreshold) {
    lastArmActivityTime = millis(); // Keep alive while gripping
    return false;
  }
  return true;
}

void relaxServos() {
  servoClaw.detach();
  servoArm .detach();
  servoBase.detach();
  servosRelaxed = true;
  DBGLN("SERVOS_RELAXED");
}

// ============================================================
//  ARM COMMANDS
// ============================================================
void clawClose()  { wakeServos(); servoContinuous(servoClaw, clawDeg, +1, CLAW_MIN, CLAW_MAX); }
void clawOpen()   { wakeServos(); servoContinuous(servoClaw, clawDeg, -1, CLAW_MIN, CLAW_MAX); }
void armUp()      { wakeServos(); servoContinuous(servoArm,  armDeg,  +1, ARM_MIN,  ARM_MAX);  }
void armDown()    { wakeServos(); servoContinuous(servoArm,  armDeg,  -1, ARM_MIN,  ARM_MAX);  }
void baseAntiCW() { wakeServos(); servoContinuous(servoBase, baseDeg, +1, BASE_MIN, BASE_MAX); }
void baseCW()     { wakeServos(); servoContinuous(servoBase, baseDeg, -1, BASE_MIN, BASE_MAX); }

// ============================================================
//  ARM HOME — 'H'
// ============================================================
void armHome() {
  wakeServos();
  DBGLN("HOMING...");
  servoMoveTo(servoClaw, clawDeg, SERVO_INIT, CLAW_MIN, CLAW_MAX); delay(100);
  servoMoveTo(servoArm,  armDeg,  SERVO_INIT, ARM_MIN,  ARM_MAX);  delay(100);
  servoMoveTo(servoBase, baseDeg, SERVO_INIT, BASE_MIN, BASE_MAX);
  DBGLN("HOME_DONE");
}

// ============================================================
//  ARM MEMORY — 'm' record, 'a' playback
// ============================================================

void recordSnapshot() {
  if (recCount < MAX_ACTIONS) {
    recClaw[recCount] = servoClaw.read();
    recArm [recCount] = servoArm.read();
    recBase[recCount] = servoBase.read();
    recCount++;
    autoCount = recCount;
    DBG("SNAP #"); DBG(autoCount);
    DBG(" CLAW:"); DBG(recClaw[recCount-1]);
    DBG(" ARM:"); DBG(recArm[recCount-1]);
    DBG(" BASE:"); DBGLN(recBase[recCount-1]);
  } else {
    DBGLN("MEM_FULL — max snapshots reached");
  }
}

void playbackMemory() {
  if (autoCount == 0) { DBGLN("NO_SNAPSHOTS recorded"); return; }
  wakeServos();
  clawDeg = servoClaw.read();
  armDeg  = servoArm.read();
  baseDeg = servoBase.read();
  DBG("PLAYBACK_START — "); DBG(autoCount); DBGLN(" snapshots");
  bool running = true;
  while (running) {
    for (int i = 0; i < autoCount && running; i++) {
      DBG("PLAYING snap #"); DBGLN(i + 1);
      if (readCmd() == 's') { running = false; break; }
      if (!servoMoveTo(servoClaw, clawDeg, recClaw[i], CLAW_MIN, CLAW_MAX)) { running = false; break; }
      if (!servoMoveTo(servoArm,  armDeg,  recArm[i],  ARM_MIN,  ARM_MAX )) { running = false; break; }
      if (!servoMoveTo(servoBase, baseDeg, recBase[i], BASE_MIN, BASE_MAX)) { running = false; break; }
    }
  }
  DBGLN("PLAYBACK_STOP");
  flushAllSerial();
}

// ============================================================
//  CALIBRATION FUNCTIONS
// ============================================================

/**
 * 'K' — Line sensor calibration.
 * Place robot on FLOOR (not on the line), send K.
 * Waits 2s then samples 30 times to determine floor value.
 */
void calibrateSensors() {
  DBGLN("CAL LINE: Place robot on FLOOR (not on line).");
  DBGLN("Starting in 2s...");
  delay(2000);
  DBGLN("Sampling sensors...");
  int total = 0, readings = 0;
  for (int i = 0; i < 30; i++) {
    total += digitalRead(PIN_TRAK_LEFT);
    total += digitalRead(PIN_TRAK_CENTER);
    total += digitalRead(PIN_TRAK_RIGHT);
    readings += 3;
    delay(20);
  }
  int floorVal   = (total > readings / 2) ? 1 : 0;
  BLACK_LINE_VAL = (floorVal == 1) ? 0 : 1;
  DBG("CAL DONE — FLOOR val="); DBG(floorVal);
  DBG("  BLACK_LINE_VAL="); DBGLN(BLACK_LINE_VAL);
  DBGLN("Verify: put robot ON the black line and send V.");
  DBGLN("Expected: at least one sensor should show 'YES' for on-line.");
}

/**
 * 'Q' — Claw carry threshold calibration.
 * First close the claw until it's gripping your object (use 'c' command).
 * Then send Q — saves current clawDeg as the carry threshold.
 */
void calibrateClawCarry() {
  clawCarryThreshold = clawDeg;
  DBG("CLAW CARRY THRESHOLD set to: "); DBG(clawCarryThreshold); DBGLN("deg");
  DBG("Auto-relax will be suppressed when clawDeg >= "); DBGLN(clawCarryThreshold);
  DBGLN("To re-calibrate: close claw to grip, send Q again.");
}

/**
 * '~' — Reset all calibration to compile-time defaults.
 */
void resetCalibration() {
  BLACK_LINE_VAL     = 1;
  clawCarryThreshold = CLAW_CARRYING_THRESHOLD_DEFAULT;
  DBGLN("CAL RESET — All values returned to defaults.");
  DBG("BLACK_LINE_VAL     = "); DBGLN(BLACK_LINE_VAL);
  DBG("clawCarryThreshold = "); DBGLN(clawCarryThreshold);
}

// ============================================================
//  OBSTACLE MEMORY
// ============================================================

void rememberObstacle(float dist) {
  obstacleMemory[obstacleMemIdx] = dist;
  obstacleMemIdx = (obstacleMemIdx + 1) % OBSTACLE_MEMORY_SIZE;
  if (obstacleMemIdx == 0) obstacleMemFull = true;
}

bool isKnownObstacle(float dist) {
  int count = obstacleMemFull ? OBSTACLE_MEMORY_SIZE : obstacleMemIdx;
  for (int i = 0; i < count; i++)
    if (abs(dist - obstacleMemory[i]) <= OBSTACLE_ZONE_MARGIN) return true;
  return false;
}

void clearObstacleMemory() {
  obstacleMemIdx  = 0;
  obstacleMemFull = false;
  for (int i = 0; i < OBSTACLE_MEMORY_SIZE; i++) obstacleMemory[i] = -1;
}

// ============================================================
//  PID LINE TRACKING — 'T'
// ============================================================
void lineTrackingFunction() {
  currentMode = "LINE";
  DBGLN("MODE: Line Tracking ON  (S=stop, P=toggle telem)");

  float lastError = 0, integral = 0;
  int   lastTurnDir = 0;

  while (true) {
    int  L  = digitalRead(PIN_TRAK_LEFT);
    int  C  = digitalRead(PIN_TRAK_CENTER);
    int  R  = digitalRead(PIN_TRAK_RIGHT);
    bool lb = isBlack(L), cb = isBlack(C), rb = isBlack(R);

    if (!lb && !cb && !rb) {
      driveStop();
      DBGLN("LINE_LOST — searching...");
      unsigned long t0 = millis();
      bool recovered = false;
      while (millis() - t0 < TRACK_SEARCH_MS) {
        if (lastTurnDir >= 0) rotateRight(90); else rotateLeft(90);
        delay(10);
        if (isBlack(digitalRead(PIN_TRAK_LEFT))  ||
            isBlack(digitalRead(PIN_TRAK_CENTER)) ||
            isBlack(digitalRead(PIN_TRAK_RIGHT))) {
          recovered = true; integral = 0; lastError = 0;
          DBGLN("LINE_FOUND");
          break;
        }
      }
      if (!recovered) { driveStop(); DBGLN("LINE_LOST — stopped. Move robot to line."); }

    } else {
      float errSum = 0; int cnt = 0;
      if (lb) { errSum += -2.0f; cnt++; }
      if (cb) { errSum +=  0.0f; cnt++; }
      if (rb) { errSum += +2.0f; cnt++; }
      float error = (cnt > 0) ? (errSum / cnt) : lastError;

      integral += error;
      integral  = constrain(integral, -10.0f, 10.0f);
      float deriv = error - lastError;
      float corr  = (PID_KP * error) + (PID_KI * integral) + (PID_KD * deriv);
      lastError   = error;

      if      (error < -0.1f) lastTurnDir = -1;
      else if (error >  0.1f) lastTurnDir = +1;

      int lSpd = constrain((int)(PID_BASE_SPEED + corr), PID_MIN_SPEED, PID_MAX_SPEED);
      int rSpd = constrain((int)(PID_BASE_SPEED - corr), PID_MIN_SPEED, PID_MAX_SPEED);
      setMotorSpeeds(lSpd, rSpd);

      if (telemActive && millis() - lastTelemTime >= TELEM_INTERVAL_MS) {
        lastTelemTime = millis();
        DBG("[LINE] L:"); DBG(L); DBG(" C:"); DBG(C); DBG(" R:"); DBG(R);
        DBG(" | err:"); DBG(error);
        DBG(" | Lspd:"); DBG(lSpd); DBG(" Rspd:"); DBGLN(rSpd);
      }
    }

    char ch = readCmd();
    if (ch == 'S') { driveStop(); break; }
    if (ch == 'P') { telemActive = !telemActive; DBG("TELEM:"); DBGLN(telemActive ? "ON" : "OFF"); }
  }

  currentMode = "IDLE";
  DBGLN("MODE: Line Tracking OFF");
  flushAllSerial();
}

// ============================================================
//  OBSTACLE AVOIDANCE — 'A'
// ============================================================
void avoidanceFunction() {
  currentMode = "AVOID";
  DBGLN("MODE: Avoidance ON  (S=stop, P=toggle telem)");
  int stuckCount = 0;
  clearObstacleMemory();

  while (true) {
    float d = checkDistance();

    if (telemActive && millis() - lastTelemTime >= TELEM_INTERVAL_MS) {
      lastTelemTime = millis();
      DBG("[AVOID] dist:"); DBG(d); DBG("cm | stuck:"); DBGLN(stuckCount);
    }

    if (d > AVOID_TURN_DIST) {
      moveForward(AVOID_FORWARD_SPEED);
      stuckCount = 0;
    } else {
      rememberObstacle(d);
      DBG("OBSTACLE: "); DBG(d); DBGLN("cm — sweeping...");
      driveStop(); delay(80);

      rotateLeft(SWEEP_PEEK_SPEED); delay(SWEEP_PEEK_MS);
      driveStop(); delay(50);
      float lDist = checkDistance();

      rotateRight(SWEEP_PEEK_SPEED); delay(SWEEP_PEEK_MS * 2 - 60);
      driveStop(); delay(50);
      float rDist = checkDistance();

      rotateLeft(SWEEP_PEEK_SPEED); delay(SWEEP_PEEK_MS - 30);
      driveStop(); delay(80);

      DBG("SWEEP L="); DBG(lDist); DBG("cm  R="); DBG(rDist); DBGLN("cm");

      if (d <= AVOID_BACKUP_DIST) {
        DBGLN("TOO CLOSE — backing up");
        moveBackward(110); delay(AVOID_BACKUP_MS);
        driveStop(); delay(100);
      }

      if (lDist >= rDist) { rotateLeft(110);  DBGLN("CHOSE: LEFT");  }
      else                { rotateRight(110); DBGLN("CHOSE: RIGHT"); }
      delay(AVOID_TURN_MS);
      driveStop();
      stuckCount++;

      if (isKnownObstacle(d)) {
        stuckCount++;
        DBGLN("KNOWN WALL — stuck count +2");
      }
    }

    if (stuckCount >= STUCK_ATTEMPT_LIMIT) {
      DBGLN("STUCK_ESCAPE — reversing + hard turn");
      moveBackward(110); delay(STUCK_ESCAPE_BACK_MS);
      driveStop(); delay(150);
      rotateRight(110); delay(STUCK_ESCAPE_TURN_MS);
      driveStop(); delay(150);
      stuckCount = 0;
      clearObstacleMemory();
    }

    char ch = readCmd();
    if (ch == 'S') { driveStop(); break; }
    if (ch == 'P') { telemActive = !telemActive; DBG("TELEM:"); DBGLN(telemActive ? "ON" : "OFF"); }
  }

  currentMode = "IDLE";
  DBGLN("MODE: Avoidance OFF");
  flushAllSerial();
}

// ============================================================
//  OBJECT FOLLOWING — 'W'
// ============================================================
void followingFunction() {
  currentMode = "FOLLOW";
  DBGLN("MODE: Following ON  (S=stop, P=toggle telem)");

  while (true) {
    float d = checkDistance();

    if (telemActive && millis() - lastTelemTime >= TELEM_INTERVAL_MS) {
      lastTelemTime = millis();
      DBG("[FOLLOW] dist:"); DBG(d); DBGLN("cm");
    }

    if      (d < FOLLOW_TOO_CLOSE)                         { moveBackward(FOLLOW_SPEED_BACK); DBG("BACK  d="); DBGLN(d); }
    else if (d >= FOLLOW_STOP_MIN && d <= FOLLOW_STOP_MAX) { driveStop(); }
    else if (d > FOLLOW_STOP_MAX  && d <= FOLLOW_SLOW_MAX) { moveForward(FOLLOW_SPEED_SLOW); }
    else if (d > FOLLOW_SLOW_MAX  && d <= FOLLOW_FAST_MAX) { moveForward(FOLLOW_SPEED_FAST); }
    else                                                    { driveStop(); }

    char ch = readCmd();
    if (ch == 'S') { driveStop(); break; }
    if (ch == 'P') { telemActive = !telemActive; DBG("TELEM:"); DBGLN(telemActive ? "ON" : "OFF"); }
  }

  currentMode = "IDLE";
  DBGLN("MODE: Following OFF");
  flushAllSerial();
}

// ============================================================
//  ANTI-DROP — 'D'
// ============================================================
void antiDropFunction() {
  currentMode = "ANTIDROP";
  DBGLN("MODE: Anti-Drop ON  (S=stop, P=toggle telem)");

  while (true) {
    int L = digitalRead(PIN_TRAK_LEFT);
    int C = digitalRead(PIN_TRAK_CENTER);
    int R = digitalRead(PIN_TRAK_RIGHT);

    if (telemActive && millis() - lastTelemTime >= TELEM_INTERVAL_MS) {
      lastTelemTime = millis();
      DBG("[ANTIDROP] L:"); DBG(L); DBG(" C:"); DBG(C); DBG(" R:"); DBGLN(R);
    }

    if (L != BLACK_LINE_VAL && C != BLACK_LINE_VAL && R != BLACK_LINE_VAL) {
      moveForward(ANTIDROP_SPEED);
    } else {
      DBGLN("CLIFF detected — backing + turning");
      moveBackward(ANTIDROP_SPEED); delay(ANTIDROP_BACK_MS);
      rotateLeft(ANTIDROP_SPEED);   delay(ANTIDROP_TURN_MS);
    }

    char ch = readCmd();
    if (ch == 'S') { driveStop(); break; }
    if (ch == 'P') { telemActive = !telemActive; DBG("TELEM:"); DBGLN(telemActive ? "ON" : "OFF"); }
  }

  currentMode = "IDLE";
  DBGLN("MODE: Anti-Drop OFF");
  flushAllSerial();
}

// ============================================================
//  GRAVITY MODE — 'G'
// ============================================================
void gravitySensorFunction() {
  currentMode = "GRAVITY";
  DBGLN("MODE: Gravity ON  (S=stop)");

  while (true) {
    char ch = readCmd();
    if (ch) {
      DBG("G> "); DBGLN(String(ch));
      switch (ch) {
        case 'F': moveForward (speedCar);  break;
        case 'B': moveBackward(speedCar);  break;
        case 'L': rotateLeft  (speedCar);  break;
        case 'R': rotateRight (speedCar);  break;
        case 'p': driveStop();             break;
        case 'X': speedCar = SPEED_LOW;    DBG("SPEED LOW=");  DBGLN(speedCar); break;
        case 'Y': speedCar = SPEED_MEDIUM; DBG("SPEED MED=");  DBGLN(speedCar); break;
        case 'Z': speedCar = SPEED_HIGH;   DBG("SPEED HIGH="); DBGLN(speedCar); break;
        case 'S': driveStop(); flushAllSerial(); currentMode = "IDLE"; DBGLN("MODE: Gravity OFF"); return;
      }
    }
  }
}

// ============================================================
//  DRIVE WRAPPERS
// ============================================================
void moveForwardFunction()  { currentMode="FWD";  DBGLN("FWD  (S=stop)"); while(true){ moveForward (speedCar); if(readCmd()=='S'){driveStop();break;} } currentMode="IDLE"; flushAllSerial(); }
void moveBackwardFunction() { currentMode="BACK"; DBGLN("BACK (S=stop)"); while(true){ moveBackward(speedCar); if(readCmd()=='S'){driveStop();break;} } currentMode="IDLE"; flushAllSerial(); }
void turnLeftFunction()     { currentMode="LEFT"; DBGLN("LEFT (S=stop)"); while(true){ rotateLeft  (speedCar); if(readCmd()=='S'){driveStop();break;} } currentMode="IDLE"; flushAllSerial(); }
void turnRightFunction()    { currentMode="RGHT"; DBGLN("RGHT (S=stop)"); while(true){ rotateRight (speedCar); if(readCmd()=='S'){driveStop();break;} } currentMode="IDLE"; flushAllSerial(); }

// ============================================================
//  IR REMOTE
// ============================================================
void irControlFunction() {
  uint32_t code = ir.getCode();
  auto key = [&](int k) { return ir.getIrKey(code, 1) == k; };

  if      (key(IR_KEYCODE_UP))    { moveForward (110); delay(300); driveStop(); }
  else if (key(IR_KEYCODE_DOWN))  { moveBackward(110); delay(300); driveStop(); }
  else if (key(IR_KEYCODE_LEFT))  { rotateLeft  (80);  delay(300); driveStop(); }
  else if (key(IR_KEYCODE_RIGHT)) { rotateRight (80);  delay(300); driveStop(); }
  else if (key(IR_KEYCODE_OK))    { driveStop(); }
  else if (key(IR_KEYCODE_7))     { wakeServos(); clawDeg = constrain(clawDeg + 5, CLAW_MIN, CLAW_MAX); servoClaw.write(clawDeg); delay(15); }
  else if (key(IR_KEYCODE_9))     { wakeServos(); clawDeg = constrain(clawDeg - 5, CLAW_MIN, CLAW_MAX); servoClaw.write(clawDeg); delay(15); }
  else if (key(IR_KEYCODE_2))     { wakeServos(); armDeg  = constrain(armDeg  + 5, ARM_MIN,  ARM_MAX);  servoArm.write(armDeg);   delay(15); }
  else if (key(IR_KEYCODE_8))     { wakeServos(); armDeg  = constrain(armDeg  - 5, ARM_MIN,  ARM_MAX);  servoArm.write(armDeg);   delay(15); }
  else if (key(IR_KEYCODE_4))     { wakeServos(); baseDeg = constrain(baseDeg + 5, BASE_MIN, BASE_MAX); servoBase.write(baseDeg); delay(15); }
  else if (key(IR_KEYCODE_6))     { wakeServos(); baseDeg = constrain(baseDeg - 5, BASE_MIN, BASE_MAX); servoBase.write(baseDeg); delay(15); }
}

// ============================================================
//  COMMAND DISPATCHER — handles one char from USB or BT
// ============================================================
void handleCommand(char cmd) {
  switch (cmd) {
    case 'o': clawOpen();                                          break;
    case 'c': clawClose();                                         break;
    case 'u': armUp();                                             break;
    case 'd': armDown();                                           break;
    case 'l': baseAntiCW();                                        break;
    case 'r': baseCW();                                            break;
    case 'H': armHome();                                           break;

    case 'F': moveForwardFunction();                               break;
    case 'B': moveBackwardFunction();                              break;
    case 'L': turnLeftFunction();                                  break;
    case 'R': turnRightFunction();                                 break;
    case 'S': driveStop(); DBGLN("STOP");                          break;

    case 'X': speedCar = SPEED_LOW;    DBG("SPEED LOW=");  DBGLN(speedCar); break;
    case 'Y': speedCar = SPEED_MEDIUM; DBG("SPEED MED=");  DBGLN(speedCar); break;
    case 'Z': speedCar = SPEED_HIGH;   DBG("SPEED HIGH="); DBGLN(speedCar); break;

    case 'm': recordSnapshot();                                    break;
    case 'a': playbackMemory();                                    break;

    case 'A': avoidanceFunction();                                 break;
    case 'D': antiDropFunction();                                  break;
    case 'W': followingFunction();                                 break;
    case 'T': lineTrackingFunction();                              break;
    case 'G': gravitySensorFunction();                             break;

    case 'K': calibrateSensors();                                  break;
    case 'Q': calibrateClawCarry();                                break;
    case '~': resetCalibration();                                  break;

    case '?': printStatus();                                       break;
    case 'V': printTelemetry();                                    break;
    case 'P':
      telemActive = !telemActive;
      DBG("TELEMETRY: "); DBGLN(telemActive ? "ON  (sends every 200ms)" : "OFF");
      break;

    default: break;
  }
}

// ============================================================
//  SETUP
// ============================================================
void setup() {
  Serial.begin(9600);
  BT.begin(9600);

  pinMode(PIN_MOTOR_A_DIR, OUTPUT);
  pinMode(PIN_MOTOR_A_PWM, OUTPUT);
  pinMode(PIN_MOTOR_B_DIR, OUTPUT);
  pinMode(PIN_MOTOR_B_PWM, OUTPUT);
  pinMode(PIN_TRAK_LEFT,       INPUT);
  pinMode(PIN_TRAK_CENTER,     INPUT);
  pinMode(PIN_TRAK_RIGHT,      INPUT);
  pinMode(PIN_ULTRASONIC_TRIG, OUTPUT);
  pinMode(PIN_ULTRASONIC_ECHO, INPUT);

  servoClaw.attach(PIN_SERVO_CLAW);
  servoArm .attach(PIN_SERVO_ARM);
  servoBase.attach(PIN_SERVO_BASE);
  servoClaw.write(clawDeg); delay(500);
  servoArm .write(armDeg);  delay(500);
  servoBase.write(baseDeg); delay(500);

  clearObstacleMemory();
  driveStop();
  lastArmActivityTime = millis();

  DBGLN("============================================");
  DBGLN("  ROBOT READY v4 — DUAL DEBUG ACTIVE");
  DBGLN("============================================");
  DBGLN("  USB  : IDE Serial Monitor @ 9600 baud");
  DBGLN("  BT   : JDY-16 @ 9600  (A2=RX, A3=TX)");
  DBGLN("--------------------------------------------");
  DBGLN("  DEBUG COMMANDS (USB or BT):");
  DBGLN("  ?  full system status snapshot");
  DBGLN("  V  single sensor + distance snapshot");
  DBGLN("  P  toggle live telemetry stream ON/OFF");
  DBGLN("  K  calibrate line sensors (place on floor)");
  DBGLN("  Q  calibrate claw carry threshold");
  DBGLN("  ~  reset all calibration to defaults");
  DBGLN("--------------------------------------------");
  DBGLN("  DRIVE : F B L R S   SPEED: X Y Z");
  DBGLN("  ARM   : o c u d l r H");
  DBGLN("  MODES : A D W T G");
  DBGLN("  MEM   : m a");
  DBGLN("============================================");
}

// ============================================================
//  MAIN LOOP
// ============================================================
void loop() {
  // ── USB input ──
  while (Serial.available() > 0) { usbBuffer += (char)Serial.read(); delay(2); }
  if (usbBuffer.length() >= 1 && usbBuffer.length() <= 2) {
    char cmd = usbBuffer.charAt(0); usbBuffer = "";
    handleCommand(cmd);
  } else if (usbBuffer.length() > 2) { usbBuffer = ""; }

  // ── BT input ──
  while (BT.available() > 0) { btBuffer += (char)BT.read(); delay(2); }
  if (btBuffer.length() >= 1 && btBuffer.length() <= 2) {
    char cmd = btBuffer.charAt(0); btBuffer = "";
    handleCommand(cmd);
  } else if (btBuffer.length() > 2) { btBuffer = ""; }

  // ── Live telemetry tick (IDLE only) ──
  if (telemActive && millis() - lastTelemTime >= TELEM_INTERVAL_MS) {
    lastTelemTime = millis();
    printTelemetry();
  }

  // ── Servo auto-relax ──
  if (servoNeedsRelax()) relaxServos();

  // ── IR remote ──
  irControlFunction();
}

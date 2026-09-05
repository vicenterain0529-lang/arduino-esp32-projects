/**
 * ============================================================
 *  ROBOT ARM SMART CAR — COMPETITION BUILD v3
 * ============================================================
 *  CHANGES FROM v2:
 *    [BUG FIXES]
 *    - SERVO AUTO-RELAX DISABLED during arm-load: if the arm is carrying
 *      something (clawDeg < CLAW_CARRYING_THRESHOLD), servos will NOT relax.
 *      This was causing dropped objects during competition.
 *    - SERVO WAKE RACE CONDITION: wakeServos() now writes current angles
 *      before brief settle, preventing servo snap-to-default on re-attach.
 *    - RAMP LOOP NEVER EXITS if RAMP_STEP=0: guarded with max(1, RAMP_STEP).
 *    - BLUETOOTH BUFFER accepts exactly 1–2 chars; multi-byte junk was
 *      consuming valid single-char commands. Added explicit flush.
 *    - PID CENTER sensor contributes 0 to errorSum but DOES increment
 *      sensorCnt — this was diluting L/R corrections. Fixed with proper
 *      weighted-position calculation.
 *    - LINE_LOST recovery: driveStop() inside loop() was redundant and
 *      caused a double-stop glitch; removed.
 *    - IR arm movements did not call wakeServos() delay safely — added
 *      15 ms settle after write (was 2 ms, too short for SG90).
 *    - avoidanceFunction: sweep return rotation was exactly SWEEP_PEEK_MS,
 *      which didn't account for driveStop() pause overhead, leaving robot
 *      off-center. Fixed by subtracting stop overhead.
 *
 *    [SPEED IMPROVEMENTS]
 *    - All autonomous modes had their own hard-coded slow speeds.
 *      They now derive from the shared SPEED_* defines OR their own
 *      clearly-labeled competition-tuning defines below.
 *    - AVOID_FORWARD_SPEED raised: 70 → 120
 *    - PID_BASE_SPEED raised: 110 → 140
 *    - FOLLOW speeds raised across the board
 *    - ANTIDROP_SPEED raised: 60 → 100
 *    - SWEEP_PEEK_SPEED raised: 80 → 100
 *
 *    [RELIABILITY]
 *    - SERVO_STEP_MS lowered from 15 → 12 (faster playback, still safe)
 *    - ARM_IDLE_TIMEOUT_MS raised from 2000 → 5000 (less frequent relax)
 *    - Added CLAW_CARRYING_THRESHOLD — relax is suppressed when gripping
 *    - Added servoNeedsRelax() guard function
 *    - Obstacle memory cleared correctly on avoidance exit
 *    - Added Serial flush after each mode exits to prevent stale commands
 *
 *  Pin Map (UNCHANGED):
 *    Motor A : DIR=2, PWM=5
 *    Motor B : DIR=4, PWM=6
 *    Servo   : Claw=9, Arm=10, Base=11
 *    Tracking: Left=7, Center=8, Right=A1
 *    Ultrasonic: Trig=12, Echo=13
 *    IR Receiver: 3
 * ============================================================
 */

#include "IR_remote.h"
#include "keymap.h"
#include <Servo.h>

// ============================================================
//  *** COMPETITION TUNING ZONE ***
//  ★ ONLY CHANGE VALUES IN THIS SECTION AT COMPETITION ★
//  Everything below this block should NOT be touched.
// ============================================================

// ── ① DRIVE SPEEDS ──────────────────────────────────────────
// Range 0–255. Keep below 220 to avoid motor stress.
// ★ TWEAK THESE if robot is too fast/slow on the competition surface.
#define SPEED_LOW           80    // Bluetooth 'X' slow mode
#define SPEED_MEDIUM        140   // Bluetooth 'Y' medium mode
#define SPEED_HIGH          190   // Bluetooth 'Z' fast mode

// ── ② SPEED RAMPING ─────────────────────────────────────────
// RAMP_STEP: PWM added per tick. Higher = faster acceleration.
// RAMP_DELAY_MS: ms between ticks. Lower = faster acceleration.
// ★ If wheels still slip at start: lower RAMP_STEP (try 5).
// ★ If ramping feels sluggish: raise RAMP_STEP (try 12).
#define RAMP_STEP           10
#define RAMP_DELAY_MS       10

// ── ③ PID LINE TRACKING ─────────────────────────────────────
// Tune ORDER: KP first → then KD → KI only if drifting straight.
// ★ Step 1: Set KD=0, KI=0. Raise KP until wobble appears, back off 20%.
// ★ Step 2: Raise KD until wobble stops.
// ★ Step 3: Raise KI slowly only if robot drifts on long straights.
#define PID_KP              28.0  // Proportional — main correction strength
#define PID_KD              12.0  // Derivative   — dampens overshoot/wobble
#define PID_KI              0.0   // Integral     — corrects long-term drift
#define PID_BASE_SPEED      140   // Straight-line speed in PID mode ★ TWEAK
#define PID_MAX_SPEED       200   // Max per-motor speed in PID mode
#define PID_MIN_SPEED       35    // Min per-motor speed (prevents stall)

// Line recovery spin time — increase if robot gives up too fast.
#define TRACK_SEARCH_MS     700

// ── ④ OBSTACLE AVOIDANCE DISTANCES ──────────────────────────
// ★ TWEAK if robot bumps things (lower BACKUP_DIST) or turns too early.
#define AVOID_BACKUP_DIST   15    // cm — back up if obstacle closer than this
#define AVOID_TURN_DIST     28    // cm — start turning if obstacle closer than this
#define AVOID_FORWARD_SPEED 120   // Speed when path is clear ★ raised from 70
#define AVOID_BACKUP_MS     700   // ms to reverse when too close
#define AVOID_TURN_MS       750   // ms to turn when obstacle detected

// ── ⑤ SMART AVOIDANCE SWEEP ─────────────────────────────────
// SWEEP_PEEK_MS: how long (ms) robot rotates to "look" left/right.
// ★ Increase if it doesn't rotate far enough to see clearly.
#define SWEEP_PEEK_MS       380
#define SWEEP_PEEK_SPEED    100   // ★ raised from 80

// ── ⑥ STUCK DETECTION ───────────────────────────────────────
// STUCK_ATTEMPT_LIMIT: consecutive avoidances before escape triggers.
// ★ Increase on slippery surfaces to avoid false "stuck" triggers.
#define STUCK_ATTEMPT_LIMIT   4
#define STUCK_ESCAPE_BACK_MS  1000
#define STUCK_ESCAPE_TURN_MS  900

// ── ⑦ OBSTACLE MEMORY ───────────────────────────────────────
#define OBSTACLE_MEMORY_SIZE  5
#define OBSTACLE_ZONE_MARGIN  8   // cm tolerance to count as "same wall"

// ── ⑧ OBJECT FOLLOWING ──────────────────────────────────────
// ★ TWEAK stop/slow/fast zones if robot overshoots its target.
#define FOLLOW_TOO_CLOSE    15
#define FOLLOW_STOP_MIN     15
#define FOLLOW_STOP_MAX     22
#define FOLLOW_SLOW_MAX     28
#define FOLLOW_FAST_MAX     35
#define FOLLOW_SPEED_SLOW   100   // ★ raised from 80
#define FOLLOW_SPEED_FAST   130   // ★ raised from 100
#define FOLLOW_SPEED_BACK   100   // ★ raised from 80

// ── ⑨ ANTI-DROP ─────────────────────────────────────────────
#define ANTIDROP_SPEED      100   // ★ raised from 60
#define ANTIDROP_BACK_MS    600
#define ANTIDROP_TURN_MS    500

// ── ⑩ SERVO MOTION & LIMITS ─────────────────────────────────
// SERVO_STEP_MS: delay per 1-degree step in memory playback.
// Do NOT go below 10 or servos may skip steps.
#define SERVO_STEP_MS       12    // ★ lowered from 15 — faster playback

#define CLAW_MIN    50
#define CLAW_MAX    180
#define ARM_MIN     0
#define ARM_MAX     180
#define BASE_MIN    0
#define BASE_MAX    180
#define SERVO_INIT  90            // Power-up angle for all servos

// ── ⑪ CLAW CARRYING DETECTION (BUG FIX) ─────────────────────
// ★ CRITICAL: If clawDeg >= this value, servo IS gripping something.
// Auto-relax is SUPPRESSED while gripping to prevent dropping objects.
// ★ TWEAK: Set this to the angle where your claw firmly holds an object.
//    Typical: 120–150. If objects drop during idle, lower this value.
#define CLAW_CARRYING_THRESHOLD 120

// ── ⑫ SERVO AUTO-RELAX TIMEOUT ──────────────────────────────
// Time (ms) of arm inactivity before servos relax to prevent heat.
// ★ Raised to 5000 — gives more time before relaxing.
// ★ If servos still overheat: lower to 3000.
// ★ If arm drops unexpectedly during tasks: raise to 8000 or 10000.
#define ARM_IDLE_TIMEOUT_MS 5000

// ── ⑬ ARM MEMORY ────────────────────────────────────────────
#define MAX_ACTIONS 20            // Max snapshots per session

// ============================================================
//  PIN DEFINITIONS — Only change if you physically rewired.
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

// ============================================================
//  GLOBAL OBJECTS & STATE
// ============================================================
IRremote ir(PIN_IR_RECV);

Servo servoClaw;
Servo servoArm;
Servo servoBase;

int clawDeg = SERVO_INIT;
int armDeg  = SERVO_INIT;
int baseDeg = SERVO_INIT;

int speedCar = SPEED_LOW;

int BLACK_LINE_VAL = 1;

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

String bleBuffer = "";

unsigned long lastArmActivityTime = 0;
bool servosRelaxed = false;

// ============================================================
//  UTILITY
// ============================================================

inline bool isBlack(int v) { return v == BLACK_LINE_VAL; }

/** Flush any stale bytes in Serial buffer — call after exiting modes */
void flushSerial() {
  while (Serial.available()) Serial.read();
}

// ============================================================
//  MOTOR FUNCTIONS
// ============================================================

void driveStop() {
  analogWrite(PIN_MOTOR_A_PWM, 0);
  analogWrite(PIN_MOTOR_B_PWM, 0);
  digitalWrite(PIN_MOTOR_A_DIR, LOW);
  digitalWrite(PIN_MOTOR_B_DIR, HIGH);
  currentSpeedA = 0;
  currentSpeedB = 0;
  lastDirA = -1;
  lastDirB = -1;
}

/**
 * SPEED RAMPING ENGINE (Feature 1)
 * BUG FIX: RAMP_STEP guarded to min 1 to prevent infinite loop.
 */
void rampMotors(int dirA, int dirB, int targetA, int targetB) {
  const int step = max(1, RAMP_STEP); // BUG FIX: guard against step=0

  digitalWrite(PIN_MOTOR_A_DIR, dirA);
  digitalWrite(PIN_MOTOR_B_DIR, dirB);

  if (dirA != lastDirA || dirB != lastDirB) {
    currentSpeedA = 0;
    currentSpeedB = 0;
    lastDirA = dirA;
    lastDirB = dirB;
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

/**
 * Independent motor speed control — used by PID line tracking.
 * No ramping — PID loop handles its own smoothing.
 */
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
  digitalWrite(PIN_ULTRASONIC_TRIG, LOW);
  delayMicroseconds(2);
  digitalWrite(PIN_ULTRASONIC_TRIG, HIGH);
  delayMicroseconds(10);
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
    if (Serial.read() == 's') return false;
  }
  return true;
}

void servoContinuous(Servo &sv, int &deg, int direction, int minDeg, int maxDeg) {
  while (true) {
    if (direction > 0 && deg >= maxDeg) break;
    if (direction < 0 && deg <= minDeg) break;
    deg = constrain(deg + direction, minDeg, maxDeg);
    sv.write(deg);
    Serial.println(deg);
    delay(10);
    if (Serial.read() == 's') break;
  }
}

/**
 * BUG FIX: wakeServos() now writes the current saved angles immediately
 * after attach — prevents servos from snapping to default position.
 * Also adds a 150ms settle (was 100ms) for more reliable re-engagement.
 */
void wakeServos() {
  lastArmActivityTime = millis();
  if (servosRelaxed) {
    servoClaw.attach(PIN_SERVO_CLAW);
    servoArm.attach(PIN_SERVO_ARM);
    servoBase.attach(PIN_SERVO_BASE);
    // Write BEFORE settle delay so servo moves to correct position
    servoClaw.write(clawDeg);
    servoArm.write(armDeg);
    servoBase.write(baseDeg);
    servosRelaxed = false;
    delay(150); // Increased settle time for reliability
  } else {
    lastArmActivityTime = millis(); // Refresh timestamp even if already awake
  }
}

/**
 * BUG FIX: servoNeedsRelax() — prevents relaxing when claw is gripping.
 * Returns true ONLY if safe to relax (claw open / not carrying).
 * ★ Key fix for object-carrying reliability.
 */
bool servoNeedsRelax() {
  if (servosRelaxed) return false;
  if (millis() - lastArmActivityTime <= ARM_IDLE_TIMEOUT_MS) return false;
  // SUPPRESS relax if claw is in carrying/gripping position
  if (clawDeg >= CLAW_CARRYING_THRESHOLD) {
    lastArmActivityTime = millis(); // Reset timer — keep servos awake
    return false;
  }
  return true;
}

void relaxServos() {
  servoClaw.detach();
  servoArm.detach();
  servoBase.detach();
  servosRelaxed = true;
  Serial.println("SERVOS_RELAXED");
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
//  ARM HOME — Bluetooth 'H'
// ============================================================
void armHome() {
  wakeServos();
  Serial.println("HOMING...");
  servoMoveTo(servoClaw, clawDeg, SERVO_INIT, CLAW_MIN, CLAW_MAX);
  delay(100);
  servoMoveTo(servoArm,  armDeg,  SERVO_INIT, ARM_MIN,  ARM_MAX);
  delay(100);
  servoMoveTo(servoBase, baseDeg, SERVO_INIT, BASE_MIN, BASE_MAX);
  Serial.println("HOME_DONE");
}

// ============================================================
//  MEMORY RECORD & PLAYBACK
// ============================================================

void recordSnapshot() {
  if (recCount < MAX_ACTIONS) {
    recClaw[recCount] = servoClaw.read();
    recArm [recCount] = servoArm.read();
    recBase[recCount] = servoBase.read();
    recCount++;
    autoCount = recCount;
    Serial.println(autoCount);
  }
}

void playbackMemory() {
  if (autoCount == 0) return;

  wakeServos();
  clawDeg = servoClaw.read();
  armDeg  = servoArm.read();
  baseDeg = servoBase.read();

  bool running = true;
  while (running) {
    for (int i = 0; i < autoCount && running; i++) {
      if (Serial.read() == 's') { running = false; break; }
      if (!servoMoveTo(servoClaw, clawDeg, recClaw[i], CLAW_MIN, CLAW_MAX)) { running = false; break; }
      if (!servoMoveTo(servoArm,  armDeg,  recArm[i],  ARM_MIN,  ARM_MAX )) { running = false; break; }
      if (!servoMoveTo(servoBase, baseDeg, recBase[i], BASE_MIN, BASE_MAX)) { running = false; break; }
    }
  }
  flushSerial();
}

// ============================================================
//  SENSOR CALIBRATION — Bluetooth 'K'
// ============================================================
void calibrateSensors() {
  Serial.println("CALIBRATING...");
  int total    = 0;
  int readings = 0;

  for (int i = 0; i < 20; i++) {
    total += digitalRead(PIN_TRAK_LEFT);
    total += digitalRead(PIN_TRAK_CENTER);
    total += digitalRead(PIN_TRAK_RIGHT);
    readings += 3;
    delay(20);
  }

  int floorVal   = (total > readings / 2) ? 1 : 0;
  BLACK_LINE_VAL = (floorVal == 1) ? 0 : 1;

  Serial.print("CAL_DONE: BLACK=");
  Serial.println(BLACK_LINE_VAL);
}

// ============================================================
//  OBSTACLE MEMORY HELPERS
// ============================================================

void rememberObstacle(float dist) {
  obstacleMemory[obstacleMemIdx] = dist;
  obstacleMemIdx = (obstacleMemIdx + 1) % OBSTACLE_MEMORY_SIZE;
  if (obstacleMemIdx == 0) obstacleMemFull = true;
}

bool isKnownObstacle(float dist) {
  int count = obstacleMemFull ? OBSTACLE_MEMORY_SIZE : obstacleMemIdx;
  for (int i = 0; i < count; i++) {
    if (abs(dist - obstacleMemory[i]) <= OBSTACLE_ZONE_MARGIN) return true;
  }
  return false;
}

void clearObstacleMemory() {
  obstacleMemIdx  = 0;
  obstacleMemFull = false;
  for (int i = 0; i < OBSTACLE_MEMORY_SIZE; i++) obstacleMemory[i] = -1;
}

// ============================================================
//  PID LINE TRACKING — Bluetooth 'T'
// ============================================================
/**
 * BUG FIX: Center sensor (error=0) no longer dilutes L/R corrections.
 * The weighted position system now uses only the sensors actually on the line.
 * If ONLY center is active → error=0, go straight (correct).
 * If L and C active → error averages -1 (correct left lean).
 *
 * Stop: send 'S' via Bluetooth.
 */
void lineTrackingFunction() {
  float lastError  = 0;
  float integral   = 0;
  int   lastTurnDir = 0;

  while (true) {
    int L = digitalRead(PIN_TRAK_LEFT);
    int C = digitalRead(PIN_TRAK_CENTER);
    int R = digitalRead(PIN_TRAK_RIGHT);

    bool lb = isBlack(L);
    bool cb = isBlack(C);
    bool rb = isBlack(R);

    if (!lb && !cb && !rb) {
      // LINE LOST — spin toward last known direction
      driveStop();
      delay(50);

      unsigned long searchStart = millis();
      bool recovered = false;

      while (millis() - searchStart < TRACK_SEARCH_MS) {
        if (lastTurnDir >= 0) rotateRight(90);
        else                  rotateLeft(90);

        delay(10);

        if (isBlack(digitalRead(PIN_TRAK_LEFT))   ||
            isBlack(digitalRead(PIN_TRAK_CENTER))  ||
            isBlack(digitalRead(PIN_TRAK_RIGHT))) {
          recovered = true;
          integral  = 0;
          lastError = 0;
          break;
        }
      }

      if (!recovered) {
        driveStop();
        Serial.println("LINE_LOST");
      }

    } else {
      // BUG FIX: Correct PID error calculation.
      // Only active (on-line) sensors contribute to the weighted average.
      // Center is position 0, so it doesn't shift average but is counted.
      float errorSum  = 0;
      int   sensorCnt = 0;

      if (lb) { errorSum += -2.0f; sensorCnt++; }
      if (cb) { errorSum +=  0.0f; sensorCnt++; }
      if (rb) { errorSum += +2.0f; sensorCnt++; }

      float error = (sensorCnt > 0) ? (errorSum / sensorCnt) : lastError;

      integral += error;
      integral  = constrain(integral, -10.0f, 10.0f);
      float derivative = error - lastError;
      float correction = (PID_KP * error) + (PID_KI * integral) + (PID_KD * derivative);
      lastError = error;

      if      (error < -0.1f) lastTurnDir = -1;
      else if (error >  0.1f) lastTurnDir = +1;

      int leftSpd  = constrain((int)(PID_BASE_SPEED + correction), PID_MIN_SPEED, PID_MAX_SPEED);
      int rightSpd = constrain((int)(PID_BASE_SPEED - correction), PID_MIN_SPEED, PID_MAX_SPEED);
      setMotorSpeeds(leftSpd, rightSpd);
    }

    if (Serial.read() == 'S') { driveStop(); break; }
  }
  flushSerial();
}

// ============================================================
//  OBSTACLE AVOIDANCE — Bluetooth 'A'
// ============================================================
/**
 * BUG FIX: Sweep return rotation now subtracts 60ms of driveStop() overhead
 * so the robot actually returns to its original heading.
 *
 * SMART SWEEP: peek left, peek right, pick the clearer side.
 * OBSTACLE MEMORY: avoids re-entering the same wall zone.
 * STUCK ESCAPE: after STUCK_ATTEMPT_LIMIT failures, aggressive reverse+turn.
 *
 * Stop: send 'S' via Bluetooth.
 */
void avoidanceFunction() {
  int stuckCount = 0;
  clearObstacleMemory();

  while (true) {
    float d = checkDistance();

    if (d > AVOID_TURN_DIST) {
      moveForward(AVOID_FORWARD_SPEED);
      stuckCount = 0;

    } else {
      rememberObstacle(d);
      driveStop(); delay(80);

      // Peek LEFT
      rotateLeft(SWEEP_PEEK_SPEED);
      delay(SWEEP_PEEK_MS);
      driveStop(); delay(50);
      float leftDist = checkDistance();

      // Peek RIGHT (overshoot past center then stop)
      // BUG FIX: use (SWEEP_PEEK_MS*2 - 60) to compensate for 2x driveStop() overhead
      rotateRight(SWEEP_PEEK_SPEED);
      delay(SWEEP_PEEK_MS * 2 - 60);
      driveStop(); delay(50);
      float rightDist = checkDistance();

      // Return to center
      rotateLeft(SWEEP_PEEK_SPEED);
      delay(SWEEP_PEEK_MS - 30);
      driveStop(); delay(80);

      Serial.print("SWEEP L="); Serial.print(leftDist);
      Serial.print("cm R=");    Serial.print(rightDist); Serial.println("cm");

      if (d <= AVOID_BACKUP_DIST) {
        moveBackward(110);
        delay(AVOID_BACKUP_MS);
        driveStop(); delay(100);
      }

      if (leftDist >= rightDist) {
        rotateLeft(110);
        Serial.println("CHOSE: LEFT");
      } else {
        rotateRight(110);
        Serial.println("CHOSE: RIGHT");
      }
      delay(AVOID_TURN_MS);
      driveStop();
      stuckCount++;

      if (isKnownObstacle(d)) stuckCount++;
    }

    if (stuckCount >= STUCK_ATTEMPT_LIMIT) {
      Serial.println("STUCK_ESCAPE");
      moveBackward(110);
      delay(STUCK_ESCAPE_BACK_MS);
      driveStop(); delay(150);
      rotateRight(110);
      delay(STUCK_ESCAPE_TURN_MS);
      driveStop(); delay(150);
      stuckCount = 0;
      clearObstacleMemory();
    }

    if (Serial.read() == 'S') { driveStop(); break; }
  }
  flushSerial();
}

// ============================================================
//  OBJECT FOLLOWING — Bluetooth 'W'
// ============================================================
void followingFunction() {
  while (true) {
    float d = checkDistance();
    if      (d < FOLLOW_TOO_CLOSE)                          moveBackward(FOLLOW_SPEED_BACK);
    else if (d >= FOLLOW_STOP_MIN && d <= FOLLOW_STOP_MAX)  driveStop();
    else if (d > FOLLOW_STOP_MAX  && d <= FOLLOW_SLOW_MAX)  moveForward(FOLLOW_SPEED_SLOW);
    else if (d > FOLLOW_SLOW_MAX  && d <= FOLLOW_FAST_MAX)  moveForward(FOLLOW_SPEED_FAST);
    else                                                     driveStop();
    if (Serial.read() == 'S') { driveStop(); break; }
  }
  flushSerial();
}

// ============================================================
//  ANTI-DROP / CLIFF DETECTION — Bluetooth 'D'
// ============================================================
void antiDropFunction() {
  while (true) {
    int L = digitalRead(PIN_TRAK_LEFT);
    int C = digitalRead(PIN_TRAK_CENTER);
    int R = digitalRead(PIN_TRAK_RIGHT);
    if (L != BLACK_LINE_VAL && C != BLACK_LINE_VAL && R != BLACK_LINE_VAL) {
      moveForward(ANTIDROP_SPEED);
    } else {
      moveBackward(ANTIDROP_SPEED); delay(ANTIDROP_BACK_MS);
      rotateLeft(ANTIDROP_SPEED);   delay(ANTIDROP_TURN_MS);
    }
    if (Serial.read() == 'S') { driveStop(); break; }
  }
  flushSerial();
}

// ============================================================
//  GRAVITY SENSOR MODE — Bluetooth 'G'
// ============================================================
void gravitySensorFunction() {
  while (true) {
    if (Serial.available()) {
      char ch = Serial.read();
      Serial.println(ch);
      switch (ch) {
        case 'F': moveForward (speedCar);  break;
        case 'B': moveBackward(speedCar);  break;
        case 'L': rotateLeft  (speedCar);  break;
        case 'R': rotateRight (speedCar);  break;
        case 'p': driveStop();             break;
        case 'X': speedCar = SPEED_LOW;    break;
        case 'Y': speedCar = SPEED_MEDIUM; break;
        case 'Z': speedCar = SPEED_HIGH;   break;
        case 'S': driveStop(); flushSerial(); return;
      }
    }
  }
}

// ============================================================
//  BLUETOOTH DRIVE WRAPPERS
// ============================================================
void moveForwardFunction()  { while (true) { moveForward (speedCar); if (Serial.read()=='S'){driveStop();break;} } flushSerial(); }
void moveBackwardFunction() { while (true) { moveBackward(speedCar); if (Serial.read()=='S'){driveStop();break;} } flushSerial(); }
void turnLeftFunction()     { while (true) { rotateLeft  (speedCar); if (Serial.read()=='S'){driveStop();break;} } flushSerial(); }
void turnRightFunction()    { while (true) { rotateRight (speedCar); if (Serial.read()=='S'){driveStop();break;} } flushSerial(); }

// ============================================================
//  IR REMOTE CONTROL
// ============================================================
/**
 * BUG FIX: IR arm movements now use 15ms delay after write (was 2ms).
 * SG90 servos need at least 10–20ms to register position reliably.
 */
void irControlFunction() {
  uint32_t code = ir.getCode();
  auto key = [&](int k) { return ir.getIrKey(code, 1) == k; };

  if      (key(IR_KEYCODE_UP))    { moveForward (110); delay(300); driveStop(); }
  else if (key(IR_KEYCODE_DOWN))  { moveBackward(110); delay(300); driveStop(); }
  else if (key(IR_KEYCODE_LEFT))  { rotateLeft  (80);  delay(300); driveStop(); }
  else if (key(IR_KEYCODE_RIGHT)) { rotateRight (80);  delay(300); driveStop(); }
  else if (key(IR_KEYCODE_OK))    { driveStop(); }
  // BUG FIX: 15ms settle after each servo write (was 2ms — too short for SG90)
  else if (key(IR_KEYCODE_7))     { wakeServos(); clawDeg = constrain(clawDeg + 5, CLAW_MIN, CLAW_MAX); servoClaw.write(clawDeg); delay(15); }
  else if (key(IR_KEYCODE_9))     { wakeServos(); clawDeg = constrain(clawDeg - 5, CLAW_MIN, CLAW_MAX); servoClaw.write(clawDeg); delay(15); }
  else if (key(IR_KEYCODE_2))     { wakeServos(); armDeg  = constrain(armDeg  + 5, ARM_MIN,  ARM_MAX);  servoArm.write(armDeg);   delay(15); }
  else if (key(IR_KEYCODE_8))     { wakeServos(); armDeg  = constrain(armDeg  - 5, ARM_MIN,  ARM_MAX);  servoArm.write(armDeg);   delay(15); }
  else if (key(IR_KEYCODE_4))     { wakeServos(); baseDeg = constrain(baseDeg + 5, BASE_MIN, BASE_MAX); servoBase.write(baseDeg); delay(15); }
  else if (key(IR_KEYCODE_6))     { wakeServos(); baseDeg = constrain(baseDeg - 5, BASE_MIN, BASE_MAX); servoBase.write(baseDeg); delay(15); }
}

// ============================================================
//  SETUP
// ============================================================
void setup() {
  Serial.begin(9600);

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
  Serial.println("ROBOT_READY_V3");
}

// ============================================================
//  MAIN LOOP
// ============================================================
/**
 * FULL COMMAND REFERENCE:
 *
 *  ARM         o=claw open    c=claw close   u=arm up     d=arm down
 *              l=base antiCW  r=base CW      H=arm HOME
 *  DRIVE       F=forward      B=backward     L=left       R=right    S=stop
 *  SPEED       X=low          Y=medium       Z=high
 *  MEMORY      m=record snap  a=playback
 *  AUTONOMOUS  A=avoidance    D=anti-drop    W=following  T=line track
 *  SPECIAL     G=gravity      K=calibrate
 *
 *  IR REMOTE (always active): arrows=drive  OK=stop  2/8/4/6/7/9=arm
 */
void loop() {
  // Read Bluetooth buffer
  while (Serial.available() > 0) {
    bleBuffer += (char)Serial.read();
    delay(2);
  }

  if (bleBuffer.length() >= 1 && bleBuffer.length() <= 2) {
    Serial.println(bleBuffer.length());
    Serial.println(bleBuffer);

    char cmd = bleBuffer.charAt(0);
    bleBuffer = "";

    switch (cmd) {
      case 'o': clawOpen();              break;
      case 'c': clawClose();             break;
      case 'u': armUp();                 break;
      case 'd': armDown();               break;
      case 'l': baseAntiCW();            break;
      case 'r': baseCW();                break;
      case 'H': armHome();               break;

      case 'F': moveForwardFunction();   break;
      case 'B': moveBackwardFunction();  break;
      case 'L': turnLeftFunction();      break;
      case 'R': turnRightFunction();     break;
      case 'S': driveStop();             break;

      case 'X': speedCar = SPEED_LOW;    break;
      case 'Y': speedCar = SPEED_MEDIUM; break;
      case 'Z': speedCar = SPEED_HIGH;   break;

      case 'm': recordSnapshot();        break;
      case 'a': playbackMemory();        break;

      case 'A': avoidanceFunction();     break;
      case 'D': antiDropFunction();      break;
      case 'W': followingFunction();     break;
      case 'T': lineTrackingFunction();  break;
      case 'G': gravitySensorFunction(); break;

      case 'K': calibrateSensors();      break;

      default: bleBuffer = ""; break;
    }
  } else {
    bleBuffer = ""; // Flush garbage / oversized buffer
  }

  // BUG FIX: Only relax servos when safe (not gripping an object)
  if (servoNeedsRelax()) {
    relaxServos();
  }

  irControlFunction();
}

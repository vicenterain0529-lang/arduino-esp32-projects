/**
 * ============================================================
 *  ROBOT ARM SMART CAR — COMPETITION BUILD v2
 * ============================================================
 *  Features (ALL original features kept + new additions):
 *    [ORIGINAL - ALL PRESERVED]
 *    - Bluetooth serial control (all movement + arm)
 *    - IR Remote control (movement + arm)
 *    - 3-servo robotic arm (claw, arm, base)
 *    - IR line tracking with line-loss recovery
 *    - Obstacle avoidance with stuck detection
 *    - Object following mode
 *    - Anti-drop (cliff detection)
 *    - Gravity sensor mode (tilt-based phone control)
 *    - Arm position memory recorder & playback
 *
 *    [NEW in v2]
 *    1. Speed Ramping       — motors accelerate smoothly, no wheel slip
 *    2. PID Line Tracking   — proportional steering, smoother than on/off
 *    3. Smart Avoidance     — peeks LEFT & RIGHT, picks the clearer side
 *                             (no extra servo needed — uses timed rotation)
 *    6. Arm Home 'H'        — one command returns all servos to center
 *    8. Sensor Calibration  — 'K' auto-reads floor to set black line value
 *   11. Obstacle Memory     — remembers last 5 obstacles, avoids re-entering
 *
 *  Pin Map:
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
//  These are ALL the values to tweak at competition.
//  Change things HERE ONLY — nothing below needs to be touched.
// ============================================================

// --- Motor Speeds ---
// Range: 0-255. Don't exceed 220 (motor stress).
// These now affect ALL modes — manual drive AND all autonomous modes (T/A/W/D).
//   X = low  → autonomous modes run at 50% of their defined speeds
//   Y = medium→ autonomous modes run at 75% of their defined speeds
//   Z = high → autonomous modes run at 100% (full defined speeds)
#define SPEED_LOW           60    // Bluetooth 'X' slow mode
#define SPEED_MEDIUM        120   // Bluetooth 'Y' medium mode
#define SPEED_HIGH          160   // Bluetooth 'Z' fast mode

// --- Speed Ramping (Feature 1) ---
// RAMP_STEP    : PWM units added each ramp tick. Higher = faster acceleration.
// RAMP_DELAY_MS: ms between ticks. Lower = faster acceleration.
// If robot still slips at launch: lower RAMP_STEP or raise RAMP_DELAY_MS.
// If ramping feels too sluggish:  raise RAMP_STEP or lower RAMP_DELAY_MS.
#define RAMP_STEP           8
#define RAMP_DELAY_MS       12

// --- PID Line Tracking (Feature 2) ---
// Tune in this order:
//   Step 1: Set KD=0, KI=0. Raise KP until robot tracks but wobbles. Back off 20%.
//   Step 2: Raise KD until wobble disappears.
//   Step 3: Only touch KI if robot drifts left/right on straight lines.
//   Step 4: Adjust PID_BASE_SPEED for your required course speed.
#define PID_KP              25.0  // Proportional gain  (main correction strength)
#define PID_KD              10.0  // Derivative gain    (dampens overshoot)
#define PID_KI              0.0   // Integral gain      (corrects long-term drift, usually 0)
#define PID_BASE_SPEED      110   // Straight-line speed in PID mode
#define PID_MAX_SPEED       180   // Max speed either motor can reach
#define PID_MIN_SPEED       30    // Min speed (keeps motors turning in sharp turns)

// --- Line Loss Recovery ---
// How long (ms) to spin searching for the line before giving up.
// Increase if robot gives up line recovery too fast.
#define TRACK_SEARCH_MS     600

// --- Obstacle Avoidance Distances (cm) ---
#define AVOID_BACKUP_DIST   15    // Back up if obstacle closer than this
#define AVOID_TURN_DIST     25    // Turn if obstacle closer than this
#define AVOID_FORWARD_SPEED 70    // Speed when path is clear
#define AVOID_BACKUP_MS     700   // How long to reverse when too close
#define AVOID_TURN_MS       700   // How long to turn when obstacle detected

// --- Smart Avoidance Sweep (Feature 3) ---
// No servo needed. Robot briefly rotates left then right to measure both sides.
// SWEEP_PEEK_MS   : how long (ms) to rotate each direction during peek.
//                   Increase if robot doesn't rotate far enough to see clearly.
// SWEEP_PEEK_SPEED: rotation speed during the peek.
#define SWEEP_PEEK_MS       350
#define SWEEP_PEEK_SPEED    80

// --- EMA Distance Filter ---
// EMA (Exponential Moving Average) smooths ultrasonic readings over time.
// Sits on top of the multi-ping median for double-layer noise rejection.
// EMA_ALPHA range: 0.0 to 1.0
//   0.1 = very heavy smoothing (slow to react, very stable)
//   0.3 = balanced — good for competition  ← recommended default
//   0.7 = light smoothing (fast reaction, less stable)
//   1.0 = no filtering at all (raw reading)
// → Lower if robot reacts to phantom/ghost obstacles
// → Raise if robot reacts too slowly to real obstacles
#define EMA_ALPHA           0.3f

// --- Stuck Detection (preserved from v1) ---
// STUCK_ATTEMPT_LIMIT: consecutive avoidances before triggering escape.
// Increase if surface is slippery (too many false stucks).
#define STUCK_ATTEMPT_LIMIT   4
#define STUCK_ESCAPE_BACK_MS  900
#define STUCK_ESCAPE_TURN_MS  900

// --- Obstacle Memory (Feature 11) ---
// Robot remembers last N obstacle distances to detect repeated same-wall hits.
// OBSTACLE_MEMORY_SIZE: how many obstacles to remember (max 10).
// OBSTACLE_ZONE_MARGIN: cm tolerance for matching a "same" obstacle.
#define OBSTACLE_MEMORY_SIZE  5
#define OBSTACLE_ZONE_MARGIN  8

// --- Object Following ---
#define FOLLOW_TOO_CLOSE    15
#define FOLLOW_STOP_MIN     15
#define FOLLOW_STOP_MAX     20
#define FOLLOW_SLOW_MAX     25
#define FOLLOW_FAST_MAX     30
#define FOLLOW_SPEED_SLOW   80
#define FOLLOW_SPEED_FAST   100
#define FOLLOW_SPEED_BACK   80

// --- Anti-Drop ---
#define ANTIDROP_SPEED      60
#define ANTIDROP_BACK_MS    600
#define ANTIDROP_TURN_MS    500

// --- Servo Motion ---
// SERVO_STEP_MS: delay per 1-degree step during memory playback.
// Do not go below 10 or servos may skip steps.
#define SERVO_STEP_MS       15

// --- Servo Angle Limits ---
// Narrow these if your arm hits mechanical limits before 0 or 180.
#define CLAW_MIN    50
#define CLAW_MAX    180
#define ARM_MIN     0
#define ARM_MAX     180
#define BASE_MIN    0
#define BASE_MAX    180
#define SERVO_INIT  90    // Power-up angle for all servos

// --- Arm Memory ---
#define MAX_ACTIONS 20    // Max arm snapshots per session (do not exceed 20)

// ============================================================
//  PIN DEFINITIONS
//  Only change if you physically rewired the robot.
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

Servo servoClaw;  // Servo 1 — claw open/close
Servo servoArm;   // Servo 2 — arm up/down
Servo servoBase;  // Servo 3 — base rotation

int clawDeg = SERVO_INIT;
int armDeg  = SERVO_INIT;
int baseDeg = SERVO_INIT;

int speedCar = SPEED_LOW;

// --- Calibrated black line value (Feature 8) ---
// Default 1. 'K' command auto-sets this from your actual floor.
// If line tracking follows the wrong side, manually set this to 0.
int BLACK_LINE_VAL = 1;

// --- Arm memory ---
int recClaw[MAX_ACTIONS];
int recArm [MAX_ACTIONS];
int recBase[MAX_ACTIONS];
int recCount  = 0;
int autoCount = 0;

// --- Obstacle memory (Feature 11) ---
float obstacleMemory[OBSTACLE_MEMORY_SIZE];
int   obstacleMemIdx  = 0;
bool  obstacleMemFull = false;

// --- Speed ramping state ---
int currentSpeedA = 0;
int currentSpeedB = 0;
int lastDirA      = -1;
int lastDirB      = -1;

// --- Bluetooth buffer ---
String bleBuffer = "";

// ============================================================
//  UTILITY
// ============================================================

/** True if sensor value matches the calibrated black-line reading */
inline bool isBlack(int v) { return v == BLACK_LINE_VAL; }

/**
 * SPEED SCALE — maps current speedCar setting to a 0.0–1.0 multiplier.
 * Used by all autonomous modes so X/Y/Z affects their speeds too.
 *   X (SPEED_LOW=60)    → 0.5  (half speed)
 *   Y (SPEED_MEDIUM=120)→ 0.75 (three quarter speed)
 *   Z (SPEED_HIGH=160)  → 1.0  (full defined speed)
 * Apply with: scaledSpeed(BASE_SPEED)
 */
float speedScale() {
  if      (speedCar <= SPEED_LOW)    return 0.50f;
  else if (speedCar <= SPEED_MEDIUM) return 0.75f;
  else                               return 1.00f;
}

/** Returns a speed value scaled by current speedCar setting */
int scaledSpeed(int baseSpeed) {
  return constrain((int)(baseSpeed * speedScale()), 30, 255);
}

// ============================================================
//  MOTOR FUNCTIONS
//  Wiring polarity identical to original. Do not change HIGH/LOW.
// ============================================================

/** Instant full stop — no ramping, safe to call anytime */
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
 * Ramps both motors from their current speed to target simultaneously.
 * Resets speed to 0 if direction changed (prevents reverse-direction glitch).
 * Called by moveForward / moveBackward / rotateLeft / rotateRight.
 */
void rampMotors(int dirA, int dirB, int targetA, int targetB) {
  digitalWrite(PIN_MOTOR_A_DIR, dirA);
  digitalWrite(PIN_MOTOR_B_DIR, dirB);

  if (dirA != lastDirA || dirB != lastDirB) {
    currentSpeedA = 0;
    currentSpeedB = 0;
    lastDirA = dirA;
    lastDirB = dirB;
  }

  while (currentSpeedA != targetA || currentSpeedB != targetB) {
    if (currentSpeedA < targetA) currentSpeedA = min(currentSpeedA + RAMP_STEP, targetA);
    if (currentSpeedA > targetA) currentSpeedA = max(currentSpeedA - RAMP_STEP, targetA);
    if (currentSpeedB < targetB) currentSpeedB = min(currentSpeedB + RAMP_STEP, targetB);
    if (currentSpeedB > targetB) currentSpeedB = max(currentSpeedB - RAMP_STEP, targetB);
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
 * INDEPENDENT MOTOR SPEED CONTROL — used by PID line tracking.
 * Sets left and right motors to different speeds for smooth steering.
 * Signed values: positive = forward, negative = backward.
 * No ramping — PID loop updates fast enough to handle its own smoothing.
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

/**
 * Returns distance in cm. If reading 0 or erratic, check pins 12/13.
 * Takes ~20ms per call due to the delay(10) stabilizer.
 */
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
//  EMA DISTANCE FILTER
//  Smooths ultrasonic readings over time using an exponential
//  moving average. Works on top of checkDistance().
//
//  Formula: EMA = (alpha * newReading) + ((1 - alpha) * lastEMA)
//  A low alpha heavily weights past readings → more stable but slower.
//  A high alpha weights the new reading more → faster but noisier.
//
//  Tune EMA_ALPHA in the COMPETITION TUNING ZONE at the top.
//  All avoidance and following functions use this instead of
//  checkDistance() directly.
// ============================================================

float distEMA = 0;  // Holds the running EMA value between calls

/**
 * Returns an EMA-smoothed distance reading in cm.
 * First call initializes EMA to the raw reading (no cold-start lag).
 * Use this everywhere instead of checkDistance() for noise-free results.
 */
float getFilteredDistance() {
  float raw = checkDistance();
  if (distEMA == 0) distEMA = raw;  // Seed with first reading to avoid cold-start pull to 0
  distEMA = (EMA_ALPHA * raw) + ((1.0f - EMA_ALPHA) * distEMA);
  return distEMA;
}

// ============================================================
//  SERVO HELPERS
// ============================================================

/**
 * Smoothly moves servo to target, 1-degree per SERVO_STEP_MS.
 * Returns false immediately if 's' received (emergency stop signal).
 * Used by: playbackMemory(), armHome()
 */
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

/**
 * Moves servo continuously in one direction until 's' received on Serial.
 * direction: +1 = increase angle, -1 = decrease angle.
 * Prints current angle to Serial for live monitoring.
 */
void servoContinuous(Servo &sv, int &deg, int direction, int minDeg, int maxDeg) {
  while (true) {
    deg = constrain(deg + direction, minDeg, maxDeg);
    sv.write(deg);
    Serial.println(deg);
    delay(10);
    if (Serial.read() == 's') break;
  }
}

// ============================================================
//  ARM COMMANDS (Bluetooth single-char triggers)
// ============================================================
void clawClose()  { servoContinuous(servoClaw, clawDeg, +1, CLAW_MIN, CLAW_MAX); }
void clawOpen()   { servoContinuous(servoClaw, clawDeg, -1, CLAW_MIN, CLAW_MAX); }
void armUp()      { servoContinuous(servoArm,  armDeg,  +1, ARM_MIN,  ARM_MAX);  }
void armDown()    { servoContinuous(servoArm,  armDeg,  -1, ARM_MIN,  ARM_MAX);  }
void baseAntiCW() { servoContinuous(servoBase, baseDeg, +1, BASE_MIN, BASE_MAX); }
void baseCW()     { servoContinuous(servoBase, baseDeg, -1, BASE_MIN, BASE_MAX); }

// ============================================================
//  ARM HOME (Feature 6) — Bluetooth 'H'
// ============================================================
/**
 * Smoothly returns all 3 servos to SERVO_INIT (90 degrees) in sequence.
 * Use between competition runs to reset arm position quickly.
 * Moves claw first, then arm, then base to avoid mechanical clashes.
 * Prints HOMING... then HOME_DONE to Serial when complete.
 *
 * TUNING: Change SERVO_INIT at top if your neutral position isn't 90 degrees.
 */
void armHome() {
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

/**
 * Records current claw/arm/base angles as a snapshot ('m' command).
 * Up to MAX_ACTIONS (20) snapshots. Prints total count to Serial.
 */
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

/**
 * Plays back all recorded snapshots in a smooth loop ('a' command).
 * Moves each servo to each saved angle in sequence.
 * Loops until 's' received on Serial.
 */
void playbackMemory() {
  if (autoCount == 0) return;

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
}

// ============================================================
//  SENSOR CALIBRATION (Feature 8) — Bluetooth 'K'
// ============================================================
/**
 * Auto-calibrates which sensor value represents the black line.
 *
 * HOW TO USE:
 *   1. Place robot on the floor surface (NOT on the line)
 *   2. Send 'K' via Bluetooth
 *   3. Robot samples all 3 sensors 20 times
 *   4. Sets BLACK_LINE_VAL to the opposite of the floor reading
 *   5. Prints CAL_DONE: BLACK=x to Serial
 *
 * If tracking still seems inverted after calibration, send 'K' again
 * while the robot is placed directly ON the black line.
 *
 * TUNING: No values to change here — it's fully automatic.
 */
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

  // Majority vote: if most readings are HIGH, floor is HIGH, so line is LOW
  int floorVal = (total > readings / 2) ? 1 : 0;
  BLACK_LINE_VAL = (floorVal == 1) ? 0 : 1;

  Serial.print("CAL_DONE: BLACK=");
  Serial.println(BLACK_LINE_VAL);
}

// ============================================================
//  OBSTACLE MEMORY HELPERS (Feature 11)
// ============================================================

/** Stores a new obstacle distance in the circular memory buffer */
void rememberObstacle(float dist) {
  obstacleMemory[obstacleMemIdx] = dist;
  obstacleMemIdx = (obstacleMemIdx + 1) % OBSTACLE_MEMORY_SIZE;
  if (obstacleMemIdx == 0) obstacleMemFull = true;
}

/**
 * Returns true if the given distance is within OBSTACLE_ZONE_MARGIN
 * of any previously remembered obstacle.
 * Used to detect when the robot is stuck hitting the same wall repeatedly.
 */
bool isKnownObstacle(float dist) {
  int count = obstacleMemFull ? OBSTACLE_MEMORY_SIZE : obstacleMemIdx;
  for (int i = 0; i < count; i++) {
    if (abs(dist - obstacleMemory[i]) <= OBSTACLE_ZONE_MARGIN) return true;
  }
  return false;
}

/** Resets all remembered obstacle positions */
void clearObstacleMemory() {
  obstacleMemIdx  = 0;
  obstacleMemFull = false;
  for (int i = 0; i < OBSTACLE_MEMORY_SIZE; i++) obstacleMemory[i] = -1;
}

// ============================================================
//  PID LINE TRACKING (Feature 2) — Bluetooth 'T'
// ============================================================
/**
 * PID LINE TRACKING
 *
 * How it works:
 *   Each sensor has a weighted position: Left=-2, Center=0, Right=+2
 *   If a sensor is on the black line, it contributes its weight to the error.
 *   The averaged error tells us how far off-center the robot is.
 *   PID formula then steers the motors:
 *     error=-2 → sharp left (only left on line)
 *     error= 0 → straight   (only center on line)
 *     error=+2 → sharp right (only right on line)
 *
 * Line-loss recovery:
 *   If all 3 sensors go dark, spin toward the last known turn direction.
 *   If still lost after TRACK_SEARCH_MS ms, stop and print LINE_LOST.
 *   Robot will automatically resume when line is found again.
 *
 * TUNING: See PID_KP / PID_KD / PID_KI in the tuning zone above.
 *
 * Stop: send 'S' via Bluetooth.
 */
void lineTrackingFunction() {
  float lastError = 0;
  float integral  = 0;
  int   lastTurnDir = 0;  // -1=was turning left, +1=was turning right

  while (true) {
    int L = digitalRead(PIN_TRAK_LEFT);
    int C = digitalRead(PIN_TRAK_CENTER);
    int R = digitalRead(PIN_TRAK_RIGHT);

    bool lb = isBlack(L);
    bool cb = isBlack(C);
    bool rb = isBlack(R);

    if (!lb && !cb && !rb) {
      // --- LINE LOST: all sensors off the line ---
      driveStop();
      delay(50);

      unsigned long searchStart = millis();
      bool recovered = false;

      while (millis() - searchStart < TRACK_SEARCH_MS) {
        if (lastTurnDir >= 0) rotateRight(scaledSpeed(80));
        else                  rotateLeft(scaledSpeed(80));

        // Check for line on each spin step
        if (isBlack(digitalRead(PIN_TRAK_LEFT))   ||
            isBlack(digitalRead(PIN_TRAK_CENTER))  ||
            isBlack(digitalRead(PIN_TRAK_RIGHT))) {
          recovered = true;
          integral  = 0;   // Reset integral to prevent windup after recovery
          lastError = 0;
          break;
        }
        delay(10);
      }

      if (!recovered) {
        driveStop();
        Serial.println("LINE_LOST");
        // Robot stays in mode and will resume if line returns
      }

    } else {
      // --- NORMAL PID TRACKING ---
      // Compute weighted average error from active sensors
      float errorSum  = 0;
      int   sensorCnt = 0;
      if (lb) { errorSum += -2.0f; sensorCnt++; }
      if (cb) { errorSum +=  0.0f; sensorCnt++; }
      if (rb) { errorSum += +2.0f; sensorCnt++; }
      float error = (sensorCnt > 0) ? (errorSum / sensorCnt) : lastError;

      // PID calculation
      integral += error;
      integral  = constrain(integral, -10.0f, 10.0f);  // Anti-windup
      float derivative = error - lastError;
      float correction = (PID_KP * error) + (PID_KI * integral) + (PID_KD * derivative);
      lastError = error;

      // Track turn direction for line-loss recovery
      if      (error < -0.1f) lastTurnDir = -1;
      else if (error >  0.1f) lastTurnDir = +1;

      // Apply: positive correction slows right motor, speeds left
      // Scale base speed by current X/Y/Z setting
      int scaledBase = scaledSpeed(PID_BASE_SPEED);
      int leftSpd  = constrain((int)(scaledBase + correction), PID_MIN_SPEED, scaledSpeed(PID_MAX_SPEED));
      int rightSpd = constrain((int)(scaledBase - correction), PID_MIN_SPEED, scaledSpeed(PID_MAX_SPEED));
      setMotorSpeeds(leftSpd, rightSpd);
    }

    if (Serial.read() == 'S') { driveStop(); break; }
  }
}

// ============================================================
//  OBSTACLE AVOIDANCE — SMART SWEEP + OBSTACLE MEMORY (Feature 3 + 11)
// ============================================================
/**
 * OBSTACLE AVOIDANCE — Bluetooth 'A' command
 *
 * SMART SWEEP (Feature 3 — no servo):
 *   When obstacle detected, robot rotates left for SWEEP_PEEK_MS to "look" left
 *   and measures distance. Then rotates right (double duration) to look right
 *   and measures again. Then centers back. Picks the clearer side to turn toward.
 *   Much smarter than always turning left blindly.
 *
 * OBSTACLE MEMORY (Feature 11):
 *   Stores the last 5 obstacle distances in a circular buffer.
 *   If the robot keeps hitting the same zone (within OBSTACLE_ZONE_MARGIN),
 *   stuck count increments faster, triggering escape sooner.
 *
 * STUCK DETECTION (preserved from v1):
 *   After STUCK_ATTEMPT_LIMIT avoidances, robot backs up more and turns
 *   the opposite direction to break free from corners.
 *
 * TUNING:
 *   - Robot bumps things:       decrease AVOID_BACKUP_DIST
 *   - Turns too early:          increase AVOID_TURN_DIST
 *   - Sweep sees wrong thing:   adjust SWEEP_PEEK_MS
 *   - Escape not aggressive:    increase STUCK_ESCAPE_BACK/TURN_MS
 *
 * Stop: send 'S' via Bluetooth.
 */
void avoidanceFunction() {
  int stuckCount = 0;
  clearObstacleMemory();

  while (true) {
    float d = getFilteredDistance();  // EMA-smoothed reading

    if (d > AVOID_TURN_DIST) {
      // Clear path — move forward, reset stuck counter
      moveForward(scaledSpeed(AVOID_FORWARD_SPEED));
      stuckCount = 0;

    } else {
      // Obstacle detected — log it and sweep to decide direction
      rememberObstacle(d);

      // --- SMART SWEEP: peek left ---
      driveStop(); delay(80);
      rotateLeft(scaledSpeed(SWEEP_PEEK_SPEED));
      delay(SWEEP_PEEK_MS);
      driveStop(); delay(50);
      float leftDist = getFilteredDistance();  // EMA-filtered left peek

      // --- SMART SWEEP: rotate to right peek (overshoot past center) ---
      rotateRight(scaledSpeed(SWEEP_PEEK_SPEED));
      delay(SWEEP_PEEK_MS * 2);
      driveStop(); delay(50);
      float rightDist = getFilteredDistance();  // EMA-filtered right peek

      // --- Return to center ---
      rotateLeft(scaledSpeed(SWEEP_PEEK_SPEED));
      delay(SWEEP_PEEK_MS);
      driveStop(); delay(80);

      Serial.print("SWEEP L="); Serial.print(leftDist);
      Serial.print("cm R=");    Serial.print(rightDist); Serial.println("cm");

      // If very close, back up first before turning
      if (d <= AVOID_BACKUP_DIST) {
        moveBackward(scaledSpeed(100));
        delay(AVOID_BACKUP_MS);
        driveStop(); delay(100);
      }

      // Turn toward the clearer side
      if (leftDist >= rightDist) {
        rotateLeft(scaledSpeed(100));
        Serial.println("CHOSE: LEFT");
      } else {
        rotateRight(scaledSpeed(100));
        Serial.println("CHOSE: RIGHT");
      }
      delay(AVOID_TURN_MS);
      driveStop();
      stuckCount++;

      // If this zone was already remembered, count it as extra stuck
      if (isKnownObstacle(d)) stuckCount++;
    }

    // --- STUCK ESCAPE (from v1, preserved) ---
    if (stuckCount >= STUCK_ATTEMPT_LIMIT) {
      Serial.println("STUCK_ESCAPE");
      moveBackward(scaledSpeed(100));
      delay(STUCK_ESCAPE_BACK_MS);
      driveStop(); delay(150);
      rotateRight(scaledSpeed(100));
      delay(STUCK_ESCAPE_TURN_MS);
      driveStop(); delay(150);
      stuckCount = 0;
      clearObstacleMemory();  // Fresh memory after escape
    }

    if (Serial.read() == 'S') { driveStop(); break; }
  }
}

// ============================================================
//  OBJECT FOLLOWING (unchanged from v1)
// ============================================================
/**
 * FOLLOWING MODE — Bluetooth 'W' command
 * Keeps a set distance from an object using the ultrasonic sensor.
 *   <15cm: back away   |   15-20cm: stop   |   20-25: slow forward
 *   25-30: fast forward |   >30cm: stop (object lost)
 */
void followingFunction() {
  while (true) {
    float d = getFilteredDistance();  // EMA-smoothed for stable following
    if      (d < FOLLOW_TOO_CLOSE)                          moveBackward(scaledSpeed(FOLLOW_SPEED_BACK));
    else if (d >= FOLLOW_STOP_MIN && d <= FOLLOW_STOP_MAX)  driveStop();
    else if (d > FOLLOW_STOP_MAX  && d <= FOLLOW_SLOW_MAX)  moveForward(scaledSpeed(FOLLOW_SPEED_SLOW));
    else if (d > FOLLOW_SLOW_MAX  && d <= FOLLOW_FAST_MAX)  moveForward(scaledSpeed(FOLLOW_SPEED_FAST));
    else                                                     driveStop();
    if (Serial.read() == 'S') { driveStop(); break; }
  }
}

// ============================================================
//  ANTI-DROP / CLIFF DETECTION (unchanged from v1)
// ============================================================
/**
 * ANTI-DROP MODE — Bluetooth 'D' command
 * Uses line sensors as cliff detectors. All clear = forward. Any edge = back up + turn.
 * If floor is dark/black, swap != to == in the condition.
 */
void antiDropFunction() {
  while (true) {
    int L = digitalRead(PIN_TRAK_LEFT);
    int C = digitalRead(PIN_TRAK_CENTER);
    int R = digitalRead(PIN_TRAK_RIGHT);
    if (L != BLACK_LINE_VAL && C != BLACK_LINE_VAL && R != BLACK_LINE_VAL) {
      moveForward(scaledSpeed(ANTIDROP_SPEED));
    } else {
      moveBackward(scaledSpeed(ANTIDROP_SPEED)); delay(ANTIDROP_BACK_MS);
      rotateLeft(scaledSpeed(ANTIDROP_SPEED));   delay(ANTIDROP_TURN_MS);
    }
    if (Serial.read() == 'S') { driveStop(); break; }
  }
}

// ============================================================
//  GRAVITY SENSOR MODE (unchanged from v1)
// ============================================================
/**
 * GRAVITY SENSOR MODE — Bluetooth 'G' command
 * Phone tilt control via Bluetooth app.
 * F/B/L/R = movement  |  p = stop  |  X/Y/Z = speed  |  S = exit
 */
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
        case 'S': driveStop(); return;
      }
    }
  }
}

// ============================================================
//  BLUETOOTH CONTINUOUS DRIVE WRAPPERS
//  Each loops until 'S' received. Speed ramping applies automatically.
// ============================================================
void moveForwardFunction()  { while (true) { moveForward (speedCar); if (Serial.read()=='S'){driveStop();break;} } }
void moveBackwardFunction() { while (true) { moveBackward(speedCar); if (Serial.read()=='S'){driveStop();break;} } }
void turnLeftFunction()     { while (true) { rotateLeft  (speedCar); if (Serial.read()=='S'){driveStop();break;} } }
void turnRightFunction()    { while (true) { rotateRight (speedCar); if (Serial.read()=='S'){driveStop();break;} } }

// ============================================================
//  IR REMOTE CONTROL (unchanged from v1)
// ============================================================
/**
 * Polled every loop() — handles IR remote keypresses.
 *
 * Directional buttons : 300ms burst then stop (change delay for longer/shorter)
 * OK                  : immediate stop
 * 2 / 8               : arm up / down by 5 degrees
 * 4 / 6               : base anti-CW / CW by 5 degrees
 * 7 / 9               : claw close / open by 5 degrees
 */
void irControlFunction() {
  uint32_t code = ir.getCode();
  auto key = [&](int k) { return ir.getIrKey(code, 1) == k; };

  if      (key(IR_KEYCODE_UP))    { moveForward (100); delay(300); driveStop(); }
  else if (key(IR_KEYCODE_DOWN))  { moveBackward(100); delay(300); driveStop(); }
  else if (key(IR_KEYCODE_LEFT))  { rotateLeft  (70);  delay(300); driveStop(); }
  else if (key(IR_KEYCODE_RIGHT)) { rotateRight (70);  delay(300); driveStop(); }
  else if (key(IR_KEYCODE_OK))    { driveStop(); }
  else if (key(IR_KEYCODE_7))     { clawDeg = constrain(clawDeg + 5, CLAW_MIN, CLAW_MAX); servoClaw.write(clawDeg); delay(2); }
  else if (key(IR_KEYCODE_9))     { clawDeg = constrain(clawDeg - 5, CLAW_MIN, CLAW_MAX); servoClaw.write(clawDeg); delay(2); }
  else if (key(IR_KEYCODE_2))     { armDeg  = constrain(armDeg  + 5, ARM_MIN,  ARM_MAX);  servoArm.write(armDeg);   delay(2); }
  else if (key(IR_KEYCODE_8))     { armDeg  = constrain(armDeg  - 5, ARM_MIN,  ARM_MAX);  servoArm.write(armDeg);   delay(2); }
  else if (key(IR_KEYCODE_4))     { baseDeg = constrain(baseDeg + 5, BASE_MIN, BASE_MAX); servoBase.write(baseDeg); delay(2); }
  else if (key(IR_KEYCODE_6))     { baseDeg = constrain(baseDeg - 5, BASE_MIN, BASE_MAX); servoBase.write(baseDeg); delay(2); }
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

  Serial.println("ROBOT_READY_V2");
}

// ============================================================
//  MAIN LOOP
// ============================================================
/**
 * FULL COMMAND REFERENCE (send via Bluetooth, 1-2 chars):
 *
 *  ARM         o=claw open    c=claw close   u=arm up     d=arm down
 *              l=base antiCW  r=base CW      H=arm HOME   (NEW)
 *  DRIVE       F=forward      B=backward     L=left       R=right    S=stop
 *  SPEED       X=low(60)      Y=medium(120)  Z=high(160)
 *  MEMORY      m=record snap  a=playback
 *  AUTONOMOUS  A=avoidance    D=anti-drop    W=following  T=line track
 *  SPECIAL     G=gravity      K=calibrate    (NEW)
 *
 *  IR REMOTE (always active): arrows=drive  OK=stop  2/8/4/6/7/9=arm
 */
void loop() {
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
      // Arm
      case 'o': clawOpen();             break;
      case 'c': clawClose();            break;
      case 'u': armUp();                break;
      case 'd': armDown();              break;
      case 'l': baseAntiCW();           break;
      case 'r': baseCW();               break;
      case 'H': armHome();              break;  // NEW: arm home to center

      // Drive
      case 'F': moveForwardFunction();  break;
      case 'B': moveBackwardFunction(); break;
      case 'L': turnLeftFunction();     break;
      case 'R': turnRightFunction();    break;
      case 'S': driveStop();            break;

      // Speed
      case 'X': speedCar = SPEED_LOW;    break;
      case 'Y': speedCar = SPEED_MEDIUM; break;
      case 'Z': speedCar = SPEED_HIGH;   break;

      // Memory
      case 'm': recordSnapshot();       break;
      case 'a': playbackMemory();       break;

      // Autonomous
      case 'A': avoidanceFunction();    break;
      case 'D': antiDropFunction();     break;
      case 'W': followingFunction();    break;
      case 'T': lineTrackingFunction(); break;
      case 'G': gravitySensorFunction();break;

      // Utilities
      case 'K': calibrateSensors();     break;  // NEW: auto-calibrate sensors

      default: break;
    }
  } else {
    bleBuffer = "";
  }

  irControlFunction();
}

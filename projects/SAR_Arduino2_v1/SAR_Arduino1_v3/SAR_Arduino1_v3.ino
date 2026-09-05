/**
 * ============================================================
 * SAR DISASTER RESPONSE ROBOT - ARDUINO 1 (MOTOR CTRL v1.3)
 * ============================================================
 * QUICK COMMAND KEY
 * MOVEMENT: F=Fwd, B=Back, L=Left, R=Right, S=Stop
 * SAFETY  : H=Hazard (E-Stop), G=Resume (Clear Hazard)
 * UTILITY : 1/2/3=Speed Mode, A=Auto, M=Manual
 * SERVO   : <=Left, >=Right, C=Center
 * DEBUG   : P=Toggle Live Feed, V=Snapshot, ?=Status
 *
 * REMINDERS
 * I2C ADDR: 8 (Slave)
 * STEP    : Acceleration is 8 units per 50ms
 *
 * HARDWARE
 * Driver  : L298N dual H-bridge
 * LEFT side motors  : one L298N channel (ENB, IN3, IN4)
 * RIGHT side motors : one L298N channel (ENA, IN1, IN2)
 * Front ultrasonic servo : D3
 * Rear ultrasonic servo  : D11
 * ============================================================
 */

#include <Wire.h>
#include <Servo.h>
#include <stdio.h>
#include <stdlib.h>

// ============================================================
//  CRITICAL FIX: Enum must be declared before the compiler 
//  auto-generates function prototypes that use it!
// ============================================================
enum MotorDirection {
  MOTOR_STOP = 0,
  MOTOR_FORWARD,
  MOTOR_BACKWARD
};

// ============================================================
//  TUNING & CONFIG
// ============================================================
#define SPEED_SLOW           100
#define SPEED_MEDIUM         120
#define SPEED_FAST           140

#define SPEED_BACK_SLOW      100
#define SPEED_BACK_MED       120
#define SPEED_BACK_FAST      140

#define START_SPEED_FLOOR    75
#define ACCEL_STEP           15
#define TURN_FACTOR          0.7f
#define TURN_MIN_SPEED       110
#define TURN_MAX_SPEED       160
#define TURN_AUTO_BOOST      20

#define SERVO_CENTER         90
#define SERVO_MIN            0
#define SERVO_MAX            180
#define SERVO_STEP           10

#define TELEM_INTERVAL_MS    500
#define LOOP_INTERVAL_MS     20
#define AUTO_PWM_DEFAULT     80
#define AUTO_PWM_MIN         60
#define AUTO_PWM_MAX         150

#define ENABLE_I2C_SLAVE     1
#define ENABLE_USB_SERIAL    1
#define ENABLE_ESTOP         0

// Corrected standard pin mappings
#define PIN_L298N_ENA        6
#define PIN_L298N_IN1        7
#define PIN_L298N_IN2        8
#define PIN_L298N_ENB        5
#define PIN_L298N_IN3        9
#define PIN_L298N_IN4        10

#define PIN_SERVO_FRONT      3   // Shield Servo 2
#define PIN_SERVO_REAR       11  // Shield Servo 1
#define PIN_ESTOP            2
#define I2C_ADDR             8

// ============================================================
//  STATE & OBJECTS
// ============================================================
Servo frontSweepServo;
Servo rearSweepServo;

int speedMode = 2;
int liveSpeed = 0;
int liveBackSpeed = 0;
int targetSpeed = SPEED_MEDIUM;
int targetBackSpeed = SPEED_BACK_MED;
int servoAngle = SERVO_CENTER;
int autoModeTargetPwm = AUTO_PWM_DEFAULT;
char i2cConfigBuffer[12];
byte i2cConfigLen = 0;

volatile bool forwardFlag = false;
volatile bool backwardFlag = false;
volatile bool leftFlag = false;
volatile bool rightFlag = false;
volatile bool hazard = false;
volatile bool autonomous = false;

volatile bool flag_hazard = false;
volatile bool flag_resume = false;
volatile bool flag_servoL = false;
volatile bool flag_servoR = false;
volatile bool flag_servoC = false;
volatile bool flag_speed = false;
volatile bool flag_autonomous = false;
volatile bool flag_manual = false;
volatile char lastI2Ccmd = '-';

bool telemActive = false;
bool motorActive = false;
unsigned long lastTelemTime = 0;
const char *currentMode = "IDLE";

int rearServoAngle() {
  // Rear sensor is mounted facing the opposite direction, so mirror the
  // commanded front angle to keep LEFT/RIGHT meaningful in robot space.
  return SERVO_MAX - servoAngle;
}

void writeSweepServos() {
  frontSweepServo.write(servoAngle);
  rearSweepServo.write(rearServoAngle());
}

// ============================================================
//  DEBUGGING LAYER
// ============================================================
#if ENABLE_USB_SERIAL
#define DBG(x)   Serial.print(x)
#define DBGLN(x) Serial.println(x)
#define DBGNL()  Serial.println()
#else
#define DBG(x)
#define DBGLN(x)
#define DBGNL()
#endif

// ============================================================
//  MOTOR LOGIC
// ============================================================
void driveLeftSide(MotorDirection direction, int pwmValue) {
  pwmValue = constrain(pwmValue, 0, 255);

  switch (direction) {
    case MOTOR_FORWARD:
      digitalWrite(PIN_L298N_IN3, HIGH);
      digitalWrite(PIN_L298N_IN4, LOW);
      analogWrite(PIN_L298N_ENB, pwmValue);
      break;

    case MOTOR_BACKWARD:
      digitalWrite(PIN_L298N_IN3, LOW);
      digitalWrite(PIN_L298N_IN4, HIGH);
      analogWrite(PIN_L298N_ENB, pwmValue);
      break;

    default:
      digitalWrite(PIN_L298N_IN3, LOW);
      digitalWrite(PIN_L298N_IN4, LOW);
      analogWrite(PIN_L298N_ENB, 0);
      break;
  }
}

void driveRightSide(MotorDirection direction, int pwmValue) {
  pwmValue = constrain(pwmValue, 0, 255);

  switch (direction) {
    case MOTOR_FORWARD:
      digitalWrite(PIN_L298N_IN1, HIGH);
      digitalWrite(PIN_L298N_IN2, LOW);
      analogWrite(PIN_L298N_ENA, pwmValue);
      break;

    case MOTOR_BACKWARD:
      digitalWrite(PIN_L298N_IN1, LOW);
      digitalWrite(PIN_L298N_IN2, HIGH);
      analogWrite(PIN_L298N_ENA, pwmValue);
      break;

    default:
      digitalWrite(PIN_L298N_IN1, LOW);
      digitalWrite(PIN_L298N_IN2, LOW);
      analogWrite(PIN_L298N_ENA, 0);
      break;
  }
}

void stopMotors() {
  driveLeftSide(MOTOR_STOP, 0);
  driveRightSide(MOTOR_STOP, 0);
  liveSpeed = 0;
  liveBackSpeed = 0;
  motorActive = false;
  currentMode = "STOP";
}

void updateSpeedTargets() {
  switch (speedMode) {
    case 1:
      targetSpeed = SPEED_SLOW;
      targetBackSpeed = SPEED_BACK_SLOW;
      break;
    case 2:
      targetSpeed = SPEED_MEDIUM;
      targetBackSpeed = SPEED_BACK_MED;
      break;
    case 3:
      targetSpeed = SPEED_FAST;
      targetBackSpeed = SPEED_BACK_FAST;
      break;
    default:
      speedMode = 2;
      targetSpeed = SPEED_MEDIUM;
      targetBackSpeed = SPEED_BACK_MED;
      break;
  }

  if (autonomous) {
    targetSpeed = min(targetSpeed, autoModeTargetPwm);
    targetBackSpeed = min(targetBackSpeed, autoModeTargetPwm);
  }
}

int effectiveTurnTargetSpeed() {
  int base = autonomous
      ? max(autoModeTargetPwm + TURN_AUTO_BOOST, TURN_MIN_SPEED)
      : max(targetSpeed, TURN_MIN_SPEED);
  return constrain(base, TURN_MIN_SPEED, TURN_MAX_SPEED);
}

void updateStartupSpeed() {
  if (!motorActive) return;

  if (forwardFlag) {
    if (liveSpeed < targetSpeed) {
      liveSpeed = min(liveSpeed + ACCEL_STEP, targetSpeed);
    }
  } else if (leftFlag || rightFlag) {
    int turnTarget = effectiveTurnTargetSpeed();
    if (liveSpeed < turnTarget) {
      liveSpeed = min(liveSpeed + ACCEL_STEP, turnTarget);
    } else if (liveSpeed > turnTarget) {
      liveSpeed = turnTarget;
    }
  }

  if (backwardFlag) {
    if (liveBackSpeed < targetBackSpeed) {
      liveBackSpeed = min(liveBackSpeed + ACCEL_STEP, targetBackSpeed);
    }
  }
}

void primeStartupSpeed() {
  if (leftFlag || rightFlag) {
    if (liveSpeed < TURN_MIN_SPEED) {
      liveSpeed = min(effectiveTurnTargetSpeed(), TURN_MIN_SPEED);
    }
  } else if (liveSpeed < START_SPEED_FLOOR) {
    liveSpeed = min(targetSpeed, START_SPEED_FLOOR);
  }
  if (liveBackSpeed < START_SPEED_FLOOR) {
    liveBackSpeed = min(targetBackSpeed, START_SPEED_FLOOR);
  }
}

void applyMovement() {
  if (!forwardFlag && !backwardFlag && !leftFlag && !rightFlag) {
    if (motorActive) {
      stopMotors();
    }
    currentMode = autonomous ? "AUTO-IDLE" : "IDLE";
    return;
  }

  if (!motorActive) {
    primeStartupSpeed();
  }
  motorActive = true;
  updateStartupSpeed();

  if (forwardFlag) {
    driveLeftSide(MOTOR_FORWARD, liveSpeed);
    driveRightSide(MOTOR_FORWARD, liveSpeed);
    currentMode = autonomous ? "AUTO-FWD" : "FWD";
    return;
  }

  if (backwardFlag) {
    driveLeftSide(MOTOR_BACKWARD, liveBackSpeed);
    driveRightSide(MOTOR_BACKWARD, liveBackSpeed);
    currentMode = autonomous ? "AUTO-BACK" : "BACK";
    return;
  }

  if (leftFlag) {
    int turnPwm = liveSpeed;
    int counterTurnPwm = max(START_SPEED_FLOOR, (int)(turnPwm * TURN_FACTOR));
    driveLeftSide(MOTOR_BACKWARD, counterTurnPwm);
    driveRightSide(MOTOR_FORWARD, turnPwm);
    currentMode = autonomous ? "AUTO-LEFT" : "LEFT";
    return;
  }

  if (rightFlag) {
    int turnPwm = liveSpeed;
    int counterTurnPwm = max(START_SPEED_FLOOR, (int)(turnPwm * TURN_FACTOR));
    driveLeftSide(MOTOR_FORWARD, turnPwm);
    driveRightSide(MOTOR_BACKWARD, counterTurnPwm);
    currentMode = autonomous ? "AUTO-RGHT" : "RIGHT";
  }
}

// ============================================================
//  COMMUNICATIONS
// ============================================================
void receiveI2C(int bytes) {
  while (Wire.available()) {
    char cmd = (char)Wire.read();
    lastI2Ccmd = cmd;

    if (i2cConfigLen > 0 || cmd == '#') {
      if (cmd == '#') {
        i2cConfigLen = 0;
        continue;
      }

      if (cmd == '\r' || cmd == '\n') {
        i2cConfigBuffer[i2cConfigLen] = '\0';
        if (i2cConfigLen >= 2 && i2cConfigBuffer[0] == 'A') {
          autoModeTargetPwm = constrain(atoi(i2cConfigBuffer + 1), AUTO_PWM_MIN, AUTO_PWM_MAX);
          updateSpeedTargets();
        }
        i2cConfigLen = 0;
        continue;
      }

      if (i2cConfigLen < sizeof(i2cConfigBuffer) - 1) {
        i2cConfigBuffer[i2cConfigLen++] = cmd;
      }
      continue;
    }

    switch (cmd) {
      case 'F':
        forwardFlag = true;
        backwardFlag = false;
        leftFlag = false;
        rightFlag = false;
        break;

      case 'B':
        backwardFlag = true;
        forwardFlag = false;
        leftFlag = false;
        rightFlag = false;
        break;

      case 'L':
        leftFlag = true;
        forwardFlag = false;
        backwardFlag = false;
        rightFlag = false;
        break;

      case 'R':
        rightFlag = true;
        forwardFlag = false;
        backwardFlag = false;
        leftFlag = false;
        break;

      case 'S':
        forwardFlag = false;
        backwardFlag = false;
        leftFlag = false;
        rightFlag = false;
        stopMotors();
        break;

      case 'H':
        forwardFlag = false;
        backwardFlag = false;
        leftFlag = false;
        rightFlag = false;
        stopMotors();
        flag_hazard = true;
        break;

      case 'G':
        flag_resume = true;
        break;

      case '<':
        flag_servoL = true;
        break;

      case '>':
        flag_servoR = true;
        break;

      case 'C':
        flag_servoC = true;
        break;

      case '1':
        speedMode = 1;
        flag_speed = true;
        break;

      case '2':
        speedMode = 2;
        flag_speed = true;
        break;

      case '3':
        speedMode = 3;
        flag_speed = true;
        break;

      case 'A':
        flag_autonomous = true;
        break;

      case 'M':
        flag_manual = true;
        break;

      default:
        break;
    }
  }
}

void requestEvent() {
  char status[28];
  snprintf(status, sizeof(status), "A:%d P:%d H:%d", autonomous ? 1 : 0, autoModeTargetPwm, hazard ? 1 : 0);
  Wire.write((const uint8_t *)status, strlen(status));
}

void handleUSB(char cmd) {
  switch (cmd) {
    case '?':
      DBGLN(F("CMDS: F,B,L,R,S | 1,2,3 | P(Live), V(Snap), ?(Status)"));
      DBGLN(F("      H/G hazard-resume | </>/C servo | A/M auto-manual"));
      break;

    case 'V':
      printTelemetry();
      break;

    case 'P':
      telemActive = !telemActive;
      DBGLN(telemActive ? F("FEED ON") : F("FEED OFF"));
      break;

    case 'F':
      forwardFlag = true;
      backwardFlag = false;
      leftFlag = false;
      rightFlag = false;
      break;

    case 'B':
      backwardFlag = true;
      forwardFlag = false;
      leftFlag = false;
      rightFlag = false;
      break;

    case 'L':
      leftFlag = true;
      forwardFlag = false;
      backwardFlag = false;
      rightFlag = false;
      break;

    case 'R':
      rightFlag = true;
      forwardFlag = false;
      backwardFlag = false;
      leftFlag = false;
      break;

    case 'S':
      forwardFlag = false;
      backwardFlag = false;
      leftFlag = false;
      rightFlag = false;
      stopMotors();
      break;

    case '1':
      speedMode = 1;
      flag_speed = true;
      break;

    case '2':
      speedMode = 2;
      flag_speed = true;
      break;

    case '3':
      speedMode = 3;
      flag_speed = true;
      break;

    case '<':
      flag_servoL = true;
      break;

    case '>':
      flag_servoR = true;
      break;

    case 'C':
      flag_servoC = true;
      break;

    case 'A':
      flag_autonomous = true;
      break;

    case 'M':
      flag_manual = true;
      break;

    case 'H':
      flag_hazard = true;
      break;

    case 'G':
      flag_resume = true;
      break;

    default:
      break;
  }
}

// ============================================================
//  DEBUG / STATUS
// ============================================================
void printTelemetry() {
  DBG(F("[TELEM] Mode:"));
  DBG(currentMode);
  DBG(F(" | Spd:"));
  DBG(liveSpeed);
  DBG(F(" | Back:"));
  DBG(liveBackSpeed);
  DBG(F(" | SrvF:"));
  DBG(servoAngle);
  DBG(F(" | SrvR:"));
  DBG(rearServoAngle());
  DBG(F(" | Flags:"));
  DBG(forwardFlag ? 'F' : '-');
  DBG(backwardFlag ? 'B' : '-');
  DBG(leftFlag ? 'L' : '-');
  DBG(rightFlag ? 'R' : '-');
  DBG(F(" | Haz:"));
  DBG(hazard ? F("YES") : F("no"));
  DBG(F(" | Auto:"));
  DBG(autonomous ? F("YES") : F("no"));
  DBG(F(" | I2C:"));
  DBGLN(lastI2Ccmd);
}

void printStatus() {
  DBGLN(F("========== ARDUINO 1 STATUS =========="));
  DBG(F("Mode          : ")); DBGLN(currentMode);
  DBG(F("Hazard        : ")); DBGLN(hazard ? F("ACTIVE") : F("clear"));
  DBG(F("Autonomous    : ")); DBGLN(autonomous ? F("YES") : F("no"));
  DBG(F("Telemetry     : ")); DBGLN(telemActive ? F("ON") : F("OFF"));
  DBGLN(F("--- MOTORS ---"));
  DBG(F("Speed mode    : ")); DBGLN(speedMode);
  DBG(F("Live speed    : ")); DBGLN(liveSpeed);
  DBG(F("Live back spd : ")); DBGLN(liveBackSpeed);
  DBG(F("Target speed  : ")); DBGLN(targetSpeed);
  DBG(F("Target back   : ")); DBGLN(targetBackSpeed);
  DBG(F("Auto PWM cap  : ")); DBGLN(autoModeTargetPwm);
  DBGLN(F("--- FLAGS ---"));
  DBG(F("Forward       : ")); DBGLN(forwardFlag ? F("YES") : F("no"));
  DBG(F("Backward      : ")); DBGLN(backwardFlag ? F("YES") : F("no"));
  DBG(F("Left          : ")); DBGLN(leftFlag ? F("YES") : F("no"));
  DBG(F("Right         : ")); DBGLN(rightFlag ? F("YES") : F("no"));
  DBGLN(F("--- SERVO ---"));
  DBG(F("Front angle   : ")); DBG(servoAngle); DBGLN(F(" deg"));
  DBG(F("Rear angle    : ")); DBG(rearServoAngle()); DBGLN(F(" deg"));
  DBGLN(F("--- I2C ---"));
  DBG(F("Slave addr    : 0x0")); DBGLN(I2C_ADDR);
  DBG(F("Last I2C cmd  : ")); DBGLN(lastI2Ccmd);
  DBGLN(F("======================================"));
}

// ============================================================
//  SETUP
// ============================================================
void setup() {
#if ENABLE_USB_SERIAL
  Serial.begin(9600);
#endif

#if ENABLE_I2C_SLAVE
  Wire.begin(I2C_ADDR);
  Wire.onReceive(receiveI2C);
  Wire.onRequest(requestEvent);
#endif

  frontSweepServo.attach(PIN_SERVO_FRONT);
  rearSweepServo.attach(PIN_SERVO_REAR);
  writeSweepServos();

  pinMode(PIN_L298N_ENA, OUTPUT);
  pinMode(PIN_L298N_IN1, OUTPUT);
  pinMode(PIN_L298N_IN2, OUTPUT);
  pinMode(PIN_L298N_ENB, OUTPUT);
  pinMode(PIN_L298N_IN3, OUTPUT);
  pinMode(PIN_L298N_IN4, OUTPUT);

#if ENABLE_ESTOP
  pinMode(PIN_ESTOP, INPUT_PULLUP);
#endif

  updateSpeedTargets();
  stopMotors();

  DBGLN(F("============================================================"));
  DBGLN(F("SAR DISASTER RESPONSE ROBOT - ARDUINO 1 (MOTOR CTRL v1.3)"));
  DBGLN(F("============================================================"));
  DBGLN(ENABLE_ESTOP ? F("I2C ADDR: 8 | E-STOP pin 2 must stay HIGH to run") : F("I2C ADDR: 8 | E-STOP DISABLED"));
  DBGLN(F("Movement: F B L R S | Safety: H G | Speed: 1 2 3"));
  DBGLN(F("L298N LEFT : ENB D5, IN3 D9, IN4 D10"));
  DBGLN(F("L298N RIGHT: ENA D6, IN1 D7, IN2 D8"));
  DBGLN(F("Front servo: D3 | Rear servo: D11 (mirrored sweep)"));
  DBGLN(F("Servo: < > C | Modes: A M | Debug: P V ?"));
  DBGLN(F("============================================================"));
}

// ============================================================
//  MAIN LOOP
// ============================================================
void loop() {
#if ENABLE_USB_SERIAL
  if (Serial.available()) {
    handleUSB((char)Serial.read());
  }
#endif

#if ENABLE_ESTOP
  if (digitalRead(PIN_ESTOP) == LOW) {
    forwardFlag = false;
    backwardFlag = false;
    leftFlag = false;
    rightFlag = false;
    hazard = true;
    autonomous = false;
    stopMotors();
    return;
  }
#endif

  if (flag_hazard) {
    flag_hazard = false;
    hazard = true;
    autonomous = false;
    forwardFlag = false;
    backwardFlag = false;
    leftFlag = false;
    rightFlag = false;
    stopMotors();
  }

  if (flag_resume) {
    flag_resume = false;
    hazard = false;
  }

  if (flag_speed) {
    flag_speed = false;
    updateSpeedTargets();
  }

  if (flag_autonomous) {
    flag_autonomous = false;
    autonomous = true;
    updateSpeedTargets();
  }

  if (flag_manual) {
    flag_manual = false;
    autonomous = false;
    updateSpeedTargets();
  }

  if (flag_servoL) {
    flag_servoL = false;
    servoAngle = max(SERVO_MIN, servoAngle - SERVO_STEP);
    writeSweepServos();
  }

  if (flag_servoR) {
    flag_servoR = false;
    servoAngle = min(SERVO_MAX, servoAngle + SERVO_STEP);
    writeSweepServos();
  }

  if (flag_servoC) {
    flag_servoC = false;
    servoAngle = SERVO_CENTER;
    writeSweepServos();
  }

  if (!hazard) {
    applyMovement();
  }

#if ENABLE_USB_SERIAL
  if (telemActive && (millis() - lastTelemTime >= TELEM_INTERVAL_MS)) {
    lastTelemTime = millis();
    printTelemetry();
  }
#endif

  delay(LOOP_INTERVAL_MS);
}

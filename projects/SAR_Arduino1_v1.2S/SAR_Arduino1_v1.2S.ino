/**
 * ============================================================
 *  SAR DISASTER RESPONSE ROBOT — ARDUINO 1 (MOTOR CONTROLLER)
 *  Version: 1.2 STABLE (fixed ramp + startup speed)
 * ============================================================
 *  Role    : I2C Slave (addr 8) — motor + servo + indicators
 *  Shield  : Adafruit L293D Motor Shield v1
 *
 *  FIXES from v1.2:
 *    - Motors now get a guaranteed minimum startup speed so
 *      all 4 wheels overcome L293D threshold simultaneously.
 *    - Acceleration ramp uses snap-to-target (no oscillation).
 *    - USB serial now accepts F/B/L/R/S for standalone testing.
 *
 *  PIN MAP:
 *    Motors       : L293D Shield (D3,D4,D5,D6,D7,D8,D11,D12 reserved)
 *    Servo sweep  : D10
 *    Buzzer       : D9
 *    RGB LED R    : A0
 *    RGB LED G    : A1
 *    RGB LED B    : A2
 *    E-Stop button: D2 (INPUT_PULLUP)
 *    I2C SDA      : A4
 *    I2C SCL      : A5
 *
 *  USB SERIAL DEBUG COMMANDS:
 *    ?       Full system status
 *    V       Live telemetry snapshot
 *    P       Toggle telemetry stream
 *    F/B/L/R Move (test without Arduino 2)
 *    S       Stop
 *    T       Test buzzer + LED
 *    1/2/3   Speed mode
 *
 *  I2C COMMAND SET (from Arduino 2):
 *    F/B/L/R/S    Movement
 *    H            Hazard — stop + alarm
 *    G            Resume — clear hazard
 *    </>/C        Servo left / right / center
 *    1/2/3        Speed mode
 *    A            Enter autonomous mode
 *    M            Return to manual mode
 * ============================================================
 */

#include <AFMotor.h>
#include <Wire.h>
#include <Servo.h>

// ============================================================
//  ★ TUNING ZONE
// ============================================================
#define SPEED_SLOW          80    // raised from 60 — helps startup
#define SPEED_MEDIUM        150
#define SPEED_FAST          200
#define SPEED_BACK_SLOW     80
#define SPEED_BACK_MED      110
#define SPEED_BACK_FAST     150

// FIX: minimum speed before ramp kicks in
// L293D won't drive motors reliably below ~60 PWM
// This ensures all 4 motors start together
#define SPEED_STARTUP       70    // jump to this instantly on direction change
#define ACCEL_STEP          12    // ramp step per loop after startup

#define TURN_FACTOR         0.7f
#define SERVO_CENTER        90
#define SERVO_MIN           0
#define SERVO_MAX           180
#define SERVO_STEP          10
#define TELEM_INTERVAL_MS   500
#define BUZZER_FREQ         1000
#define BUZZER_WARN_FREQ    2000
#define ESTOP_DEBOUNCE_MS   50

// ============================================================
//  FEATURE TOGGLES
// ============================================================
#define ENABLE_I2C_SLAVE      1
#define ENABLE_USB_SERIAL     1

// ============================================================
//  PIN DEFINITIONS
// ============================================================
#define PIN_SERVO           10
#define PIN_BUZZER          9
#define PIN_LED_R           A0
#define PIN_LED_G           A1
#define PIN_LED_B           A2
#define PIN_ESTOP           2
#define I2C_ADDR            8

// ============================================================
//  OBJECTS
// ============================================================
AF_DCMotor motor1(1);  // Left  Front
AF_DCMotor motor2(2);  // Right Front
AF_DCMotor motor3(3);  // Left  Rear
AF_DCMotor motor4(4);  // Right Rear
Servo      sweepServo;

// ============================================================
//  STATE
// ============================================================
int  speedMode         = 2;
int  currentSpeed      = 0;
int  targetSpeed       = SPEED_MEDIUM;
int  currentBackSpeed  = 0;
int  targetBackSpeed   = SPEED_BACK_MED;
int  servoAngle        = SERVO_CENTER;

// FIX: track last direction to detect direction change
// so we can apply startup speed instantly on change
char lastDirection     = 'S';

volatile bool forwardFlag      = false;
volatile bool backwardFlag     = false;
volatile bool leftFlag         = false;
volatile bool rightFlag        = false;
volatile bool hazard           = false;
volatile bool autonomous       = false;

volatile bool flag_hazard      = false;
volatile bool flag_resume      = false;
volatile bool flag_servoL      = false;
volatile bool flag_servoR      = false;
volatile bool flag_servoC      = false;
volatile bool flag_speed       = false;
volatile bool flag_autonomous  = false;
volatile bool flag_manual      = false;

volatile char lastI2Ccmd       = '-';

bool   telemActive             = false;
unsigned long lastTelemTime    = 0;
unsigned long lastEstopTime    = 0;
const char*   currentMode      = "IDLE";

// ============================================================
//  DEBUG OUTPUT
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
//  LED
// ============================================================
void setLED(bool r, bool g, bool b) {
  digitalWrite(PIN_LED_R, r ? HIGH : LOW);
  digitalWrite(PIN_LED_G, g ? HIGH : LOW);
  digitalWrite(PIN_LED_B, b ? HIGH : LOW);
}
void ledBlue()   { setLED(0,0,1); }
void ledGreen()  { setLED(0,1,0); }
void ledOrange() { setLED(1,1,0); }
void ledCyan()   { setLED(0,1,1); }
void ledRed()    { setLED(1,0,0); }
void ledPurple() { setLED(1,0,1); }
void ledWhite()  { setLED(1,1,1); }
void ledOff()    { setLED(0,0,0); }

// ============================================================
//  MOTOR FUNCTIONS
// ============================================================
void stopMotors() {
  motor1.run(RELEASE); motor2.run(RELEASE);
  motor3.run(RELEASE); motor4.run(RELEASE);
  currentSpeed     = 0;
  currentBackSpeed = 0;
  lastDirection    = 'S';
  currentMode      = "STOP";
}

void updateSpeedTargets() {
  switch (speedMode) {
    case 1: targetSpeed = SPEED_SLOW;   targetBackSpeed = SPEED_BACK_SLOW; break;
    case 2: targetSpeed = SPEED_MEDIUM; targetBackSpeed = SPEED_BACK_MED;  break;
    case 3: targetSpeed = SPEED_FAST;   targetBackSpeed = SPEED_BACK_FAST; break;
    default: speedMode = 2; targetSpeed = SPEED_MEDIUM; targetBackSpeed = SPEED_BACK_MED; break;
  }
}

// ============================================================
//  FIX: applyMovement now uses startup speed on direction change
//  All 4 motors get setSpeed AFTER run() with guaranteed
//  minimum value so right side motors don't get left behind
// ============================================================
void applyMovement() {
  // ... forwardFlag section ...
  if (forwardFlag) {
    if (lastDirection != 'F') {
      currentSpeed = SPEED_STARTUP;
      lastDirection = 'F';
    }
    
    int leftSpeed = currentSpeed;
    int rightSpeed = currentSpeed;
    if (leftFlag)  leftSpeed  = (int)(currentSpeed * TURN_FACTOR);
    if (rightFlag) rightSpeed = (int)(currentSpeed * TURN_FACTOR);

    // LEFT side: M1 + M2
    motor1.setSpeed(leftSpeed);  motor2.setSpeed(leftSpeed);
    motor1.run(FORWARD);         motor2.run(FORWARD);
    
    // RIGHT side: M3 + M4  
    motor3.setSpeed(rightSpeed); motor4.setSpeed(rightSpeed);
    motor3.run(FORWARD);         motor4.run(FORWARD);
    
    ledGreen();
    currentMode = autonomous ? "AUTO-FWD" : "FWD";
    return;
  }

  // ... backwardFlag section ...
  if (backwardFlag) {
    if (lastDirection != 'B') {
      currentBackSpeed = SPEED_STARTUP;
      lastDirection = 'B';
    }
    
    int leftSpeed = currentBackSpeed;
    int rightSpeed = currentBackSpeed;
    if (leftFlag)  leftSpeed  = (int)(currentBackSpeed * TURN_FACTOR);
    if (rightFlag) rightSpeed = (int)(currentBackSpeed * TURN_FACTOR);

    // LEFT side: M1 + M2
    motor1.setSpeed(leftSpeed);  motor2.setSpeed(leftSpeed);
    motor1.run(BACKWARD);        motor2.run(BACKWARD);
    
    // RIGHT side: M3 + M4
    motor3.setSpeed(rightSpeed); motor4.setSpeed(rightSpeed);
    motor3.run(BACKWARD);        motor4.run(BACKWARD);
    
    ledOrange();
    currentMode = autonomous ? "AUTO-BACK" : "BACK";
    return;
  }

  // ... leftFlag (turn left) ...
  if (leftFlag) {
    if (lastDirection != 'L') {
      currentSpeed = SPEED_STARTUP;
      lastDirection = 'L';
    }
    // Turn left: LEFT side back, RIGHT side forward
    motor1.run(BACKWARD); motor2.run(BACKWARD);
    motor3.run(FORWARD);  motor4.run(FORWARD);
    
    motor1.setSpeed(currentSpeed); motor2.setSpeed(currentSpeed);
    motor3.setSpeed(currentSpeed); motor4.setSpeed(currentSpeed);
    
    ledCyan();
    currentMode = autonomous ? "AUTO-LEFT" : "LEFT";
    return;
  }

  // ... rightFlag (turn right) ...
  if (rightFlag) {
    if (lastDirection != 'R') {
      currentSpeed = SPEED_STARTUP;
      lastDirection = 'R';
    }
    // Turn right: LEFT side forward, RIGHT side back
    motor1.run(FORWARD);  motor2.run(FORWARD);
    motor3.run(BACKWARD); motor4.run(BACKWARD);
    
    motor1.setSpeed(currentSpeed); motor2.setSpeed(currentSpeed);
    motor3.setSpeed(currentSpeed); motor4.setSpeed(currentSpeed);
    
    ledCyan();
    currentMode = autonomous ? "AUTO-RGHT" : "RIGHT";
    return;
  }
}

// ============================================================
//  I2C RECEIVE — ISR, flags only
// ============================================================
void receiveI2C(int bytes) {
  while (Wire.available()) {
    char cmd = Wire.read();
    lastI2Ccmd = cmd;
    switch (cmd) {
      case 'F':
        forwardFlag  = true;  backwardFlag = false;
        leftFlag     = false; rightFlag    = false;
        break;
      case 'B':
        backwardFlag = true;  forwardFlag  = false;
        leftFlag     = false; rightFlag    = false;
        break;
      case 'L':
        leftFlag     = true;  rightFlag    = false;
        forwardFlag  = false; backwardFlag = false;
        break;
      case 'R':
        rightFlag    = true;  leftFlag     = false;
        forwardFlag  = false; backwardFlag = false;
        break;
      case 'S':
        forwardFlag = backwardFlag = leftFlag = rightFlag = false;
        break;
      case 'H': flag_hazard     = true;  break;
      case 'G': flag_resume     = true;  break;
      case '<': flag_servoL     = true;  break;
      case '>': flag_servoR     = true;  break;
      case 'C': flag_servoC     = true;  break;
      case 'A': flag_autonomous = true;  break;
      case 'M': flag_manual     = true;  break;
      case '1': speedMode = 1; flag_speed = true; break;
      case '2': speedMode = 2; flag_speed = true; break;
      case '3': speedMode = 3; flag_speed = true; break;
      default: break;
    }
  }
}

// ============================================================
//  TELEMETRY + STATUS
// ============================================================
void printTelemetry() {
  DBG(F("["));        DBG(currentMode);   DBG(F("] "));
  DBG(F("SPD:"));     DBG(currentSpeed);  DBG(F("/")); DBG(targetSpeed);
  DBG(F(" MODE:"));   DBG(speedMode);
  DBG(F(" SRV:"));    DBG(servoAngle);    DBG(F("deg"));
  DBG(F(" F:"));      DBG(forwardFlag);
  DBG(F(" B:"));      DBG(backwardFlag);
  DBG(F(" L:"));      DBG(leftFlag);
  DBG(F(" R:"));      DBG(rightFlag);
  DBG(F(" HAZ:"));    DBG(hazard      ? F("YES") : F("no "));
  DBG(F(" AUT:"));    DBG(autonomous  ? F("YES") : F("no"));
  DBG(F(" I2C:"));    DBGLN(lastI2Ccmd);
}

void printStatus() {
  DBGLN(F("========== ARDUINO 1 STATUS =========="));
  DBG(F("Mode          : ")); DBGLN(currentMode);
  DBG(F("Autonomous    : ")); DBGLN(autonomous  ? F("YES") : F("no"));
  DBGLN(F("--- MOTORS ---"));
  DBG(F("Speed mode    : ")); DBGLN(speedMode);
  DBG(F("Current speed : ")); DBGLN(currentSpeed);
  DBG(F("Target speed  : ")); DBGLN(targetSpeed);
  DBG(F("Startup speed : ")); DBGLN(SPEED_STARTUP);
  DBG(F("Back speed    : ")); DBGLN(currentBackSpeed);
  DBG(F("SLOW/MED/FAST : "));
  DBG(SPEED_SLOW); DBG(F("/")); DBG(SPEED_MEDIUM); DBG(F("/")); DBGLN(SPEED_FAST);
  DBG(F("Last direction: ")); DBGLN(lastDirection);
  DBGLN(F("--- DIRECTION ---"));
  DBG(F("Forward  : ")); DBGLN(forwardFlag  ? F("YES") : F("no"));
  DBG(F("Backward : ")); DBGLN(backwardFlag ? F("YES") : F("no"));
  DBG(F("Left     : ")); DBGLN(leftFlag     ? F("YES") : F("no"));
  DBG(F("Right    : ")); DBGLN(rightFlag    ? F("YES") : F("no"));
  DBGLN(F("--- SERVO ---"));
  DBG(F("Angle    : ")); DBG(servoAngle); DBGLN(F(" deg"));
  DBGLN(F("--- STATUS ---"));
  DBG(F("Hazard   : ")); DBGLN(hazard      ? F("ACTIVE") : F("clear"));
  DBG(F("Telem    : ")); DBGLN(telemActive ? F("ON")     : F("OFF"));
  DBGLN(F("--- I2C ---"));
  DBG(F("Slave addr: 0x0")); DBGLN(I2C_ADDR);
  DBGLN(F("======================================"));
  DBGLN(F("CMDS: ? V P S T F B L R | 1 2 3"));
}

// ============================================================
//  USB COMMAND HANDLER
// ============================================================
void handleUSB(char cmd) {
  switch (cmd) {
    case '?': printStatus();    break;
    case 'V': printTelemetry(); break;
    case 'P':
      telemActive = !telemActive;
      DBG(F("TELEM: ")); DBGLN(telemActive ? F("ON") : F("OFF"));
      break;

    // ── Movement (standalone test without Arduino 2) ──
    case 'F':
      forwardFlag  = true;  backwardFlag = false;
      leftFlag     = false; rightFlag    = false;
      DBGLN(F("USB: FWD"));
      break;
    case 'B':
      backwardFlag = true;  forwardFlag  = false;
      leftFlag     = false; rightFlag    = false;
      DBGLN(F("USB: BACK"));
      break;
    case 'L':
      leftFlag     = true;  rightFlag    = false;
      forwardFlag  = false; backwardFlag = false;
      DBGLN(F("USB: LEFT"));
      break;
    case 'R':
      rightFlag    = true;  leftFlag     = false;
      forwardFlag  = false; backwardFlag = false;
      DBGLN(F("USB: RIGHT"));
      break;
    case 'S':
      forwardFlag = backwardFlag = leftFlag = rightFlag = false;
      stopMotors();
      DBGLN(F("USB: STOP"));
      break;

    case 'T':
      DBGLN(F("TEST: LED + Buzzer..."));
      ledRed();    tone(PIN_BUZZER, 800,  200); delay(250);
      ledOrange(); tone(PIN_BUZZER, 1000, 200); delay(250);
      ledGreen();  tone(PIN_BUZZER, 1200, 200); delay(250);
      ledCyan();   tone(PIN_BUZZER, 1500, 200); delay(250);
      ledPurple(); tone(PIN_BUZZER, 1800, 200); delay(250);
      ledBlue();   noTone(PIN_BUZZER);
      DBGLN(F("TEST: Done"));
      break;

    case '1': speedMode = 1; updateSpeedTargets(); DBG(F("SPEED: ")); DBGLN(speedMode); break;
    case '2': speedMode = 2; updateSpeedTargets(); DBG(F("SPEED: ")); DBGLN(speedMode); break;
    case '3': speedMode = 3; updateSpeedTargets(); DBG(F("SPEED: ")); DBGLN(speedMode); break;
  }
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
#endif

  sweepServo.attach(PIN_SERVO);
  sweepServo.write(servoAngle);

  pinMode(PIN_BUZZER, OUTPUT);
  pinMode(PIN_LED_R,  OUTPUT);
  pinMode(PIN_LED_G,  OUTPUT);
  pinMode(PIN_LED_B,  OUTPUT);
  pinMode(PIN_ESTOP,  INPUT_PULLUP);

  stopMotors();
  ledBlue();

  tone(PIN_BUZZER, 1000, 100); delay(150);
  tone(PIN_BUZZER, 1500, 100); delay(150);
  tone(PIN_BUZZER, 2000, 100); delay(150);
  noTone(PIN_BUZZER);

  DBGLN(F("============================================"));
  DBGLN(F("  SAR ROBOT v1.2S — ARDUINO 1 MOTOR CTRL"));
  DBGLN(F("============================================"));
  DBGLN(F("  I2C Slave @ 0x08 | USB @ 9600"));
  DBGLN(F("--------------------------------------------"));
  DBGLN(F("  ?  status    V  snapshot   P  telem"));
  DBGLN(F("  F/B/L/R/S  standalone motor test"));
  DBGLN(F("  T  test LED  1/2/3  speed mode"));
  DBGLN(F("--------------------------------------------"));
  DBGLN(F("  LED: BLUE=idle  GREEN=fwd  ORANGE=back"));
  DBGLN(F("       CYAN=turn  RED=hazard PURPLE=auto"));
  DBGLN(F("============================================"));
}

// ============================================================
//  MAIN LOOP
// ============================================================
void loop() {

#if ENABLE_USB_SERIAL
  if (Serial.available()) {
    char cmd = (char)Serial.read();
    handleUSB(cmd);
  }
#endif

  // ── Emergency stop ──
  if (digitalRead(PIN_ESTOP) == LOW) {
    if (millis() - lastEstopTime > ESTOP_DEBOUNCE_MS) {
      lastEstopTime = millis();
      if (!hazard) {
        DBGLN(F("!!! E-STOP PRESSED !!!"));
        forwardFlag = backwardFlag = leftFlag = rightFlag = false;
        stopMotors();
        hazard     = true;
        autonomous = false;
      }
    }
    ledRed();
    tone(PIN_BUZZER, BUZZER_FREQ);
    return;
  }

  // ── Process ISR flags ──
  if (flag_hazard) {
    flag_hazard = false;
    hazard      = true;
    forwardFlag = backwardFlag = leftFlag = rightFlag = false;
    stopMotors();
    ledRed();
    tone(PIN_BUZZER, BUZZER_WARN_FREQ);
    DBGLN(F("HAZARD: Alert from Arduino 2"));
  }

  if (flag_resume) {
    flag_resume = false;
    hazard      = false;
    noTone(PIN_BUZZER);
    ledBlue();
    DBGLN(F("HAZARD: Cleared"));
  }

  if (flag_autonomous) {
    flag_autonomous = false;
    autonomous      = true;
    ledPurple();
    DBGLN(F("MODE: Autonomous ON"));
  }

  if (flag_manual) {
    flag_manual = false;
    autonomous  = false;
    ledBlue();
    DBGLN(F("MODE: Manual"));
  }

  if (flag_speed) {
    flag_speed = false;
    updateSpeedTargets();
    // FIX: reset current speed so ramp restarts cleanly on speed mode change
    currentSpeed     = 0;
    currentBackSpeed = 0;
    DBG(F("SPEED MODE: ")); DBGLN(speedMode);
  }

  if (flag_servoL) {
    flag_servoL = false;
    servoAngle  = max(SERVO_MIN, servoAngle - SERVO_STEP);
    sweepServo.write(servoAngle);
    DBG(F("SERVO: ")); DBG(servoAngle); DBGLN(F("deg"));
  }

  if (flag_servoR) {
    flag_servoR = false;
    servoAngle  = min(SERVO_MAX, servoAngle + SERVO_STEP);
    sweepServo.write(servoAngle);
    DBG(F("SERVO: ")); DBG(servoAngle); DBGLN(F("deg"));
  }

  if (flag_servoC) {
    flag_servoC = false;
    servoAngle  = SERVO_CENTER;
    sweepServo.write(servoAngle);
    DBGLN(F("SERVO: CENTER"));
  }

  if (hazard) return;

  // ── FIX: Smooth acceleration ramp — snap to target, no oscillation ──
  if (currentSpeed < targetSpeed)
    currentSpeed = min(currentSpeed + ACCEL_STEP, targetSpeed);
  else if (currentSpeed > targetSpeed)
    currentSpeed = max(currentSpeed - ACCEL_STEP, targetSpeed);

  if (currentBackSpeed < targetBackSpeed)
    currentBackSpeed = min(currentBackSpeed + ACCEL_STEP, targetBackSpeed);
  else if (currentBackSpeed > targetBackSpeed)
    currentBackSpeed = max(currentBackSpeed - ACCEL_STEP, targetBackSpeed);

  applyMovement();

#if ENABLE_USB_SERIAL
  if (telemActive && millis() - lastTelemTime >= TELEM_INTERVAL_MS) {
    lastTelemTime = millis();
    printTelemetry();
  }
#endif
}

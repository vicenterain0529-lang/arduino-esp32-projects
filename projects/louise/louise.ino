/*
 * HUMAN FOLLOWING ROBOT WITH DUAL AUTHENTICATION
 * Author: Grid (Grade 12 STEM)
 * Hardware: Arduino UNO, 2x Ultrasonic + Servo Sweep, RFID, Fingerprint, Load Cell, L298N
 */

#include <SPI.h>
#include <MFRC522.h>
#include <Servo.h>
#include <SoftwareSerial.h>
#include "HX711.h"
#include "NewPing.h"

// ========== PIN DEFINITIONS ==========
// Motors (L298N)
#define MOTOR_L_EN 5      // ENA - PWM
#define MOTOR_L_IN1 4     // IN1
#define MOTOR_L_IN2 7     // IN2
#define MOTOR_R_EN 6      // ENB - PWM  
#define MOTOR_R_IN1 8     // IN3
#define MOTOR_R_IN2 9     // IN4

// Sensors
#define SERVO_PIN 3
#define ULTRASONIC_L A5   // One-pin mode
#define ULTRASONIC_R 2    // One-pin mode (was RFID_RST, moved)
#define IR_SENSOR A0
#define RFID_RST A3       // Moved from D2
#define RFID_SS 10

// Serial Devices
#define FP_RX A4          // Fingerprint TX→Arduino RX
#define FP_TX A1          // Arduino TX→Fingerprint RX (conflicts with HX711? Use A1 for FP, move HX711)
// Correction: HX711 typically uses any digital pins, let's use D11,D12? No SPI...
// Use D0/D1 for FP (SoftwareSerial impossible on those), move HC-05 to SoftwareSerial?

// Actually, let's re-allocate properly:
#define HC05_RX 11        // Can't use, SPI...
#define HC05_TX 12        // Can't use, SPI...

// BETTER APPROACH: Hardware Serial for HC-05 (D0/D1), SoftwareSerial for Fingerprint on A4/A5
// But A5 is Ultrasonic... Use pins 2 and 3? 3 is servo...
// Use pins 2 (FP_RX) and A0 (FP_TX) but A0 is IR...

// FINAL PIN MAP (Optimized):
// D0/D1: HC-05 (Hardware Serial) - Remember to disconnect when uploading
// D2: Fingerprint RX (SoftwareSerial)
// A0: Fingerprint TX (SoftwareSerial) - use as digital output
// D3: Servo
// D4,D7,D8,D9: Motors
// D5,D6: Motor PWM
// D10,D11,D12,D13: RFID SPI + RST on A3
// A1: HX711 DT
// A2: HX711 SCK  
// A4: Ultrasonic Left (NewPing)
// A5: Ultrasonic Right (NewPing)
// A3: RFID RST (Digital output)

#define MAX_DISTANCE 200  // Max ultrasonic distance (cm)

// ========== LIBRARY INSTANCES ==========
MFRC522 rfid(RFID_SS, RFID_RST);
Servo panServo;
HX711 loadCell;
SoftwareSerial fpSerial(2, A0); // RX, TX for Fingerprint
NewPing sonarL(A4, A4, MAX_DISTANCE);
NewPing sonarR(A5, A5, MAX_DISTANCE);

// ========== SYSTEM PARAMETERS ==========
// Security
const unsigned long AUTH_TIMEOUT = 10000;    // 10 seconds to start after auth
const unsigned long SESSION_TIMEOUT = 300000; // 5 minutes
const unsigned long LOCK_DELAY = 30000;      // Lock if lost for 30s
byte authorizedUIDs[][4] = {
  {0xAA, 0xBB, 0xCC, 0xDD},  // Add your RFID cards here
  {0x11, 0x22, 0x33, 0x44}
};
const int authorizedFingerIDs[] = {1, 2, 3}; // Enrolled fingerprint IDs

// Navigation
const int FOLLOW_DIST = 60;      // cm
const int MIN_DIST = 20;         // cm - obstacle avoidance
const int MAX_SPEED = 255;       // PWM
const int TURN_SPEED = 180;
const int SLOW_SPEED = 150;
const int SERVO_LEFT = 45;
const int SERVO_CENTER = 90;
const int SERVO_RIGHT = 135;

// Load Cell
const float LOADCELL_CALIBRATION = -7050.0; // Adjust this value
const float PAYLOAD_THRESHOLD = 2.0;        // kg
const float THEFT_THRESHOLD = 5.0;          // kg change

// ========== GLOBAL VARIABLES ==========
enum State {STATE_LOCKED, STATE_AUTH, STATE_FOLLOW, STATE_OBSTACLE, 
            STATE_LOST, STATE_MANUAL, STATE_ERROR};
State currentState = STATE_LOCKED;

unsigned long authTimer = 0;
unsigned long sessionTimer = 0;
unsigned long lostTimer = 0;
int lostCounter = 0;

// Sensor Data
float currentWeight = 0;
int distL = 0, distC = 0, distR = 0; // Distances at servo positions
int currentServoPos = 90;
bool servoDirection = true; // true = left->right

// Bluetooth Command
char btCommand = '0';

// ========== SETUP ==========
void setup() {
  Serial.begin(9600);    // HC-05 Bluetooth (Hardware Serial)
  fpSerial.begin(57600); // Fingerprint sensor
  
  // Motor pins
  pinMode(MOTOR_L_IN1, OUTPUT);
  pinMode(MOTOR_L_IN2, OUTPUT);
  pinMode(MOTOR_R_IN1, OUTPUT);
  pinMode(MOTOR_R_IN2, OUTPUT);
  pinMode(MOTOR_L_EN, OUTPUT);
  pinMode(MOTOR_R_EN, OUTPUT);
  
  // Sensors
  pinMode(IR_SENSOR, INPUT);
  panServo.attach(SERVO_PIN);
  panServo.write(SERVO_CENTER);
  
  // RFID
  SPI.begin();
  rfid.PCD_Init();
  
  // Load Cell
  loadCell.begin(A1, A2); // DT, SCK
  loadCell.set_scale(LOADCELL_CALIBRATION);
  loadCell.tare();        // Zero the scale
  
  stopMotors();
  Serial.println("System: LOCKED - Waiting for authentication");
  sendBTStatus("LOCKED");
}

// ========== MAIN LOOP ==========
void loop() {
  // Always handle Bluetooth commands
  handleBluetooth();
  
  // Update load cell periodically (every 500ms)
  static unsigned long weightTimer = 0;
  if (millis() - weightTimer > 500) {
    currentWeight = loadCell.get_units(5); // Average 5 readings
    checkTheftDetection();
    weightTimer = millis();
  }
  
  // State Machine
  switch (currentState) {
    case STATE_LOCKED:
      handleLockedState();
      break;
      
    case STATE_AUTH:
      handleAuthenticatedState();
      break;
      
    case STATE_FOLLOW:
      handleFollowingState();
      break;
      
    case STATE_OBSTACLE:
      handleObstacleState();
      break;
      
    case STATE_LOST:
      handleLostState();
      break;
      
    case STATE_MANUAL:
      handleManualMode();
      break;
      
    case STATE_ERROR:
      // Halt system, send error
      stopMotors();
      delay(1000);
      break;
  }
}

// ========== STATE HANDLERS ==========

void handleLockedState() {
  stopMotors();
  
  // Check RFID
  if (checkRFID()) {
    grantAccess("RFID");
    return;
  }
  
  // Check Fingerprint (non-blocking check)
  if (checkFingerprint()) {
    grantAccess("FINGERPRINT");
    return;
  }
  
  // Visual: Slow red blink
  static unsigned long blinkTimer = 0;
  if (millis() - blinkTimer > 1000) {
    // Toggle LED (if connected) or send BT status
    sendBTStatus("WAITING_AUTH");
    blinkTimer = millis();
  }
}

void handleAuthenticatedState() {
  stopMotors();
  
  // Check timeout
  if (millis() - authTimer > AUTH_TIMEOUT) {
    currentState = STATE_LOCKED;
    Serial.println("Auth timeout - Locking");
    sendBTStatus("TIMEOUT_LOCK");
    return;
  }
  
  // Wait for start command (IR sensor trigger or BT command)
  if (digitalRead(IR_SENSOR) == LOW || btCommand == 'S') {
    currentState = STATE_FOLLOW;
    sessionTimer = millis();
    Serial.println("Following activated");
    sendBTStatus("FOLLOWING");
    btCommand = '0';
  }
  
  // Visual: Yellow fast blink
  static unsigned long blinkTimer = 0;
  if (millis() - blinkTimer > 200) {
    sendBTStatus("ARMED");
    blinkTimer = millis();
  }
}

void handleFollowingState() {
  // Check session timeout
  if (millis() - sessionTimer > SESSION_TIMEOUT) {
    currentState = STATE_LOCKED;
    Serial.println("Session expired");
    return;
  }
  
  // Check IR proximity (emergency stop)
  if (digitalRead(IR_SENSOR) == LOW) {
    currentState = STATE_OBSTACLE;
    return;
  }
  
  // Sweep and detect
  performServoSweep();
  
  // Determine human position based on ultrasonic readings
  // Logic: Find minimum distance between L and R sensors at each position
  int minDist = min(distL, distR);
  
  if (minDist > 150 || minDist == 0) {
    // No human detected
    lostCounter++;
    if (lostCounter > LOST_THRESHOLD) {
      currentState = STATE_LOST;
      lostTimer = millis();
    }
    return;
  }
  
  // Reset lost counter
  lostCounter = 0;
  
  // Navigation logic
  if (minDist < MIN_DIST) {
    // Too close - back up
    moveBackward(SLOW_SPEED);
  } 
  else if (minDist > FOLLOW_DIST + 10) {
    // Too far - speed up
    moveForward(MAX_SPEED);
  }
  else if (minDist > FOLLOW_DIST - 10 && minDist < FOLLOW_DIST + 10) {
    // Sweet spot - maintain
    moveForward(SLOW_SPEED);
  }
  
  // Direction correction based on which sensor sees human closer
  // AND servo position
  if (currentServoPos < 75) {
    // Human is left
    turnLeft(TURN_SPEED);
  } else if (currentServoPos > 105) {
    // Human is right
    turnRight(TURN_SPEED);
  }
  
  // Payload adjustment
  if (currentWeight > PAYLOAD_THRESHOLD) {
    // Reduce speed if carrying load
    analogWrite(MOTOR_L_EN, SLOW_SPEED * 0.7);
    analogWrite(MOTOR_R_EN, SLOW_SPEED * 0.7);
  }
  
  delay(50); // Small delay for stability
}

void handleObstacleState() {
  stopMotors();
  sendBTStatus("OBSTACLE_DETECTED");
  
  // Back up and scan
  moveBackward(SLOW_SPEED);
  delay(500);
  stopMotors();
  
  // Check if clear
  int d = sonarL.ping_cm();
  if (d == 0 || d > MIN_DIST + 10) {
    // Clear - resume following
    currentState = STATE_FOLLOW;
    turnRight(TURN_SPEED); // Turn to avoid obstacle
    delay(300);
  }
}

void handleLostState() {
  stopMotors();
  sendBTStatus("SEARCHING_HUMAN");
  
  // Rotate in place to find human
  turnRight(SLOW_SPEED);
  
  // Check ultrasonics while rotating
  int d = sonarL.ping_cm();
  if (d > 0 && d < 150) {
    // Found!
    currentState = STATE_FOLLOW;
    lostCounter = 0;
    return;
  }
  
  // Lock if lost too long
  if (millis() - lostTimer > LOCK_DELAY) {
    currentState = STATE_LOCKED;
    Serial.println("Human lost - Locking");
  }
}

void handleManualMode() {
  // Bluetooth remote control override
  switch (btCommand) {
    case 'W': moveForward(MAX_SPEED); break;
    case 'S': moveBackward(MAX_SPEED); break;
    case 'A': turnLeft(TURN_SPEED); break;
    case 'D': turnRight(TURN_SPEED); break;
    case 'X': stopMotors(); break;
    case 'Q': currentState = STATE_FOLLOW; break; // Return to auto
    default: stopMotors();
  }
}

// ========== SENSOR FUNCTIONS ==========

bool checkRFID() {
  if (!rfid.PICC_IsNewCardPresent() || !rfid.PICC_ReadCardSerial()) {
    return false;
  }
  
  // Check against authorized UIDs
  for (int i = 0; i < sizeof(authorizedUIDs)/4; i++) {
    if (memcmp(rfid.uid.uidByte, authorizedUIDs[i], 4) == 0) {
      rfid.PICC_HaltA();
      rfid.PCD_StopCrypto1();
      return true;
    }
  }
  
  rfid.PICC_HaltA();
  rfid.PCD_StopCrypto1();
  Serial.println("Unauthorized RFID attempted");
  return false;
}

bool checkFingerprint() {
  // Simplified - actual Adafruit Fingerprint library integration needed
  // This is placeholder for the concept
  if (fpSerial.available()) {
    // Parse fingerprint ID
    // Return true if ID matches authorizedFingerIDs
  }
  return false;
}

void performServoSweep() {
  // Simple 3-position sweep: Left, Center, Right
  static int sweepIndex = 0;
  static unsigned long sweepTimer = 0;
  
  if (millis() - sweepTimer < 200) return; // Wait for servo settle
  
  switch (sweepIndex) {
    case 0: // Left
      panServo.write(SERVO_LEFT);
      currentServoPos = SERVO_LEFT;
      delay(150);
      distL = sonarL.ping_cm();
      distR = sonarR.ping_cm();
      break;
      
    case 1: // Center
      panServo.write(SERVO_CENTER);
      currentServoPos = SERVO_CENTER;
      delay(150);
      distL = sonarL.ping_cm();
      distR = sonarR.ping_cm();
      break;
      
    case 2: // Right
      panServo.write(SERVO_RIGHT);
      currentServoPos = SERVO_RIGHT;
      delay(150);
      distL = sonarL.ping_cm();
      distR = sonarR.ping_cm();
      break;
  }
  
  sweepIndex = (sweepIndex + 1) % 3;
  sweepTimer = millis();
}

void checkTheftDetection() {
  static float lastWeight = 0;
  if (abs(currentWeight - lastWeight) > THEFT_THRESHOLD && currentState == STATE_LOCKED) {
    sendBTStatus("THEFT_ALERT");
    // Could add buzzer here
  }
  lastWeight = currentWeight;
}

// ========== MOTOR CONTROL ==========

void moveForward(int speed) {
  digitalWrite(MOTOR_L_IN1, HIGH);
  digitalWrite(MOTOR_L_IN2, LOW);
  digitalWrite(MOTOR_R_IN1, HIGH);
  digitalWrite(MOTOR_R_IN2, LOW);
  analogWrite(MOTOR_L_EN, speed);
  analogWrite(MOTOR_R_EN, speed);
}

void moveBackward(int speed) {
  digitalWrite(MOTOR_L_IN1, LOW);
  digitalWrite(MOTOR_L_IN2, HIGH);
  digitalWrite(MOTOR_R_IN1, LOW);
  digitalWrite(MOTOR_R_IN2, HIGH);
  analogWrite(MOTOR_L_EN, speed);
  analogWrite(MOTOR_R_EN, speed);
}

void turnLeft(int speed) {
  digitalWrite(MOTOR_L_IN1, LOW);
  digitalWrite(MOTOR_L_IN2, HIGH);
  digitalWrite(MOTOR_R_IN1, HIGH);
  digitalWrite(MOTOR_R_IN2, LOW);
  analogWrite(MOTOR_L_EN, speed);
  analogWrite(MOTOR_R_EN, speed);
}

void turnRight(int speed) {
  digitalWrite(MOTOR_L_IN1, HIGH);
  digitalWrite(MOTOR_L_IN2, LOW);
  digitalWrite(MOTOR_R_IN1, LOW);
  digitalWrite(MOTOR_R_IN2, HIGH);
  analogWrite(MOTOR_L_EN, speed);
  analogWrite(MOTOR_R_EN, speed);
}

void stopMotors() {
  digitalWrite(MOTOR_L_IN1, LOW);
  digitalWrite(MOTOR_L_IN2, LOW);
  digitalWrite(MOTOR_R_IN1, LOW);
  digitalWrite(MOTOR_R_IN2, LOW);
  analogWrite(MOTOR_L_EN, 0);
  analogWrite(MOTOR_R_EN, 0);
}

// ========== BLUETOOTH INTERFACE ==========

void handleBluetooth() {
  if (Serial.available()) {
    btCommand = Serial.read();
    
    switch (btCommand) {
      case 'U': // Unlock (master password mode)
        if (currentState == STATE_LOCKED) {
          grantAccess("BT_OVERRIDE");
        }
        break;
      case 'L': // Lock now
        currentState = STATE_LOCKED;
        sendBTStatus("MANUAL_LOCK");
        break;
      case 'M': // Manual mode
        currentState = STATE_MANUAL;
        sendBTStatus("MANUAL_MODE");
        break;
      case 'S': // Start following (if auth'd)
        if (currentState == STATE_AUTH) {
          // Handled in state
        }
        break;
      case 'R': // Report status
        sendDetailedStatus();
        break;
    }
  }
}

void grantAccess(const char* method) {
  currentState = STATE_AUTH;
  authTimer = millis();
  Serial.print("Access granted via ");
  Serial.println(method);
  sendBTStatus("AUTHENTICATED");
  
  // Visual/Audio feedback
  // Add buzzer beep or LED flash here
}

void sendBTStatus(const char* status) {
  Serial.print("STATUS:");
  Serial.println(status);
}

void sendDetailedStatus() {
  Serial.print("STATE:");
  Serial.print(currentState);
  Serial.print("|WEIGHT:");
  Serial.print(currentWeight);
  Serial.print("|DIST_L:");
  Serial.print(distL);
  Serial.print("|DIST_R:");
  Serial.print(distR);
  Serial.print("|SERVO:");
  Serial.println(currentServoPos);
}
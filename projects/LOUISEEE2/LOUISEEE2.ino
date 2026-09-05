/*
 * HUMAN FOLLOWING ROBOT - 2-Way Auth + Debug + EEPROM
 * Hardware: Arduino UNO R3, L298N, 4x DC Motors, Servo, HC-SR04,
 *           R307/AS608 Fingerprint, RC522 RFID, I2C LCD 16x2
 */

#include <Servo.h>
#include <Wire.h>
#include <LiquidCrystal_I2C.h>
#include <SPI.h>
#include <MFRC522.h>
#include <SoftwareSerial.h>
#include <Adafruit_Fingerprint.h>
#include <EEPROM.h>

// ==================== PIN DEFINITIONS ====================
// L298N Motor Driver (2 channels, 4 motors in 2 parallel pairs)
#define PIN_IN1 2
#define PIN_IN2 4
#define PIN_IN3 6
#define PIN_IN4 7
#define PIN_ENA 3   // PWM - Left side speed
#define PIN_ENB 5   // PWM - Right side speed

// Ultrasonic Sensor
#define PIN_TRIG 8
#define PIN_ECHO A1   // Analog pin used as digital input (pin 15)

// Servo
#define PIN_SERVO 9

// RFID RC522 (SPI)
#define PIN_RFID_RST A0   // Digital pin 14
#define PIN_RFID_SS  10

// Fingerprint Sensor (SoftwareSerial)
#define PIN_FP_RX A2   // Arduino receives from FP TX (pin 16)
#define PIN_FP_TX A3   // Arduino transmits to FP RX (pin 17)

// ==================== CONSTANTS ====================
#define LCD_ADDR 0x27       // Change to 0x3F if your module uses that address
#define EEPROM_MAGIC 0xAB   // Validates EEPROM has been configured

// EEPROM Address Map
#define ADDR_MAGIC    0   // 1 byte
#define ADDR_RFID_UID 1   // 4 bytes
#define ADDR_FP_ID    5   // 1 byte
#define ADDR_MTR_SPD  6   // 1 byte

// Follower parameters
#define DEF_MOTOR_SPEED 200
#define STOP_DISTANCE   20    // cm
#define FOLLOW_DISTANCE 50    // cm
#define SERVO_LEFT      135
#define SERVO_CENTER    90
#define SERVO_RIGHT     45

// ==================== OBJECTS ====================
Servo servo;
LiquidCrystal_I2C lcd(LCD_ADDR, 16, 2);
MFRC522 rfid(PIN_RFID_SS, PIN_RFID_RST);
SoftwareSerial fpSerial(PIN_FP_RX, PIN_FP_TX);
Adafruit_Fingerprint finger(&fpSerial);

// ==================== STATE MACHINE ====================
enum SystemState {
  ST_LOCKED,      // Waiting for RFID
  ST_RFID_OK,     // RFID passed, waiting for fingerprint
  ST_ACTIVE,      // Human following mode
  ST_DEBUG        // Maintenance / serial debug mode
};
SystemState state = ST_LOCKED;
unsigned long stateTimer = 0;
bool verboseDebug = false;   // Toggles Serial sensor reporting in ACTIVE mode

// ==================== SETUP ====================
void setup() {
  Serial.begin(115200);
  
  // Motor pins
  pinMode(PIN_IN1, OUTPUT); pinMode(PIN_IN2, OUTPUT);
  pinMode(PIN_IN3, OUTPUT); pinMode(PIN_IN4, OUTPUT);
  pinMode(PIN_ENA, OUTPUT); pinMode(PIN_ENB, OUTPUT);
  stopMotors();

  // Ultrasonic pins
  pinMode(PIN_TRIG, OUTPUT); pinMode(PIN_ECHO, INPUT);

  // Servo
  servo.attach(PIN_SERVO);
  servo.write(SERVO_CENTER);

  // LCD
  lcd.init();
  lcd.backlight();

  // RFID
  SPI.begin();
  rfid.PCD_Init();

  // Fingerprint
  fpSerial.begin(57600);
  finger.begin(57600);
  
  // Check sensor presence
  if (finger.verifyPassword()) {
    Serial.println(F("[INIT] Fingerprint sensor OK"));
  } else {
    Serial.println(F("[INIT] Fingerprint sensor NOT DETECTED!"));
  }

  // EEPROM validation
  if (!isConfigValid()) {
    factoryReset();
    lcdPrint("NO CONFIG", "Send 'h' Help");
    Serial.println(F("[INIT] No valid EEPROM config found."));
    Serial.println(F("[INIT] Enter debug mode ('D') to enroll RFID & Fingerprint."));
  } else {
    lcdPrint("System Locked", "Tap RFID Card");
  }
  
  Serial.println(F("[INIT] Ready."));
}

// ==================== MAIN LOOP ====================
void loop() {
  handleSerial();   // Always listen for debug commands
  
  switch (state) {
    case ST_LOCKED:
      handleLocked();
      break;
    case ST_RFID_OK:
      handleRfidOk();
      break;
    case ST_ACTIVE:
      handleActive();
      break;
    case ST_DEBUG:
      // Debug actions are triggered via serial interrupts
      break;
  }
}

// ==================== STATE HANDLERS ====================

void handleLocked() {
  if (checkRFID()) {
    state = ST_RFID_OK;
    stateTimer = millis();
    lcdPrint("RFID Verified", "Scan Finger");
    Serial.println(F("[AUTH] RFID accepted."));
  }
}

void handleRfidOk() {
  // 15-second timeout to scan finger
  if (millis() - stateTimer > 15000) {
    state = ST_LOCKED;
    lcdPrint("Auth Timeout", "Tap RFID Card");
    Serial.println(F("[AUTH] Timeout. Returning to LOCKED."));
    return;
  }

  uint8_t p = finger.getImage();
  if (p == FINGERPRINT_NOFINGER) return;
  if (p != FINGERPRINT_OK) {
    lcdPrint("FP Read Error", "Retry...");
    delay(500);
    return;
  }

  p = finger.image2Tz();
  if (p != FINGERPRINT_OK) return;

  p = finger.fingerFastSearch();
  if (p != FINGERPRINT_OK) {
    lcdPrint("Unknown Finger", "Retry...");
    Serial.println(F("[AUTH] Unknown fingerprint."));
    delay(800);
    return;
  }

  byte authID = EEPROM.read(ADDR_FP_ID);
  if (finger.fingerID == authID) {
    state = ST_ACTIVE;
    lcdPrint("System Active!", "Following...");
    Serial.println(F("[AUTH] Fingerprint verified. ACTIVE MODE."));
    delay(1000);
  } else {
    lcdPrint("Wrong FP ID", "Access Denied");
    Serial.print(F("[AUTH] Rejected FP ID: ")); Serial.println(finger.fingerID);
    delay(1000);
  }
}

void handleActive() {
  static unsigned long lastScan = 0;
  if (millis() - lastScan < 300) return; // Scan decision rate
  lastScan = millis();

  // Sweep and measure
  int distL = scanAtAngle(SERVO_LEFT);
  int distC = scanAtAngle(SERVO_CENTER);
  int distR = scanAtAngle(SERVO_RIGHT);
  
  // Return servo to center
  servo.write(SERVO_CENTER);

  if (verboseDebug) {
    Serial.print(F("[SENSOR] L:")); Serial.print(distL);
    Serial.print(F(" C:")); Serial.print(distC);
    Serial.print(F(" R:")); Serial.println(distR);
  }

  int minDist = min(min(distL, distC), distR);
  byte spd = EEPROM.read(ADDR_MTR_SPD);

  // Decision logic
  if (minDist > 200 || minDist == 0) {
    // Lost target
    stopMotors();
    lcdPrint("Searching...", "");
    if (verboseDebug) Serial.println(F("[FOLLOW] Lost target"));
  } 
  else if (minDist < STOP_DISTANCE) {
    stopMotors();
    lcdPrint("Too Close!", "Stopping");
    if (verboseDebug) Serial.println(F("[FOLLOW] Too close"));
  } 
  else {
    lcdPrint("Following", "");
    if (distC <= distL && distC <= distR && distC < FOLLOW_DISTANCE * 2) {
      moveForward(spd);
      if (verboseDebug) Serial.println(F("[FOLLOW] Forward"));
    } else if (distL < distR) {
      turnLeft(spd);
      if (verboseDebug) Serial.println(F("[FOLLOW] Left"));
    } else {
      turnRight(spd);
      if (verboseDebug) Serial.println(F("[FOLLOW] Right"));
    }
  }
}

// ==================== AUTHENTICATION HELPERS ====================

bool checkRFID() {
  if (!rfid.PICC_IsNewCardPresent()) return false;
  if (!rfid.PICC_ReadCardSerial()) return false;

  bool match = true;
  for (byte i = 0; i < 4; i++) {
    if (rfid.uid.uidByte[i] != EEPROM.read(ADDR_RFID_UID + i)) {
      match = false;
    }
  }

  rfid.PICC_HaltA();
  rfid.PCD_StopCrypto1();
  return match;
}

void readAndPrintRFID() {
  if (!rfid.PICC_IsNewCardPresent() || !rfid.PICC_ReadCardSerial()) {
    Serial.println(F("[RFID] No card detected."));
    return;
  }
  Serial.print(F("[RFID] UID: "));
  for (byte i = 0; i < rfid.uid.size; i++) {
    if (rfid.uid.uidByte[i] < 0x10) Serial.print(F("0"));
    Serial.print(rfid.uid.uidByte[i], HEX);
    Serial.print(F(" "));
  }
  Serial.println();
  rfid.PICC_HaltA();
  rfid.PCD_StopCrypto1();
}

uint8_t getFingerprintEnroll(byte id) {
  int p = -1;
  Serial.println(F("[FP] Place finger..."));
  while (p != FINGERPRINT_OK) {
    p = finger.getImage();
    if (p == FINGERPRINT_NOFINGER) continue;
    if (p != FINGERPRINT_OK) return p;
  }
  Serial.println(F("[FP] Image taken"));

  p = finger.image2Tz(1);
  if (p != FINGERPRINT_OK) return p;

  Serial.println(F("[FP] Remove finger"));
  delay(2000);
  while (finger.getImage() != FINGERPRINT_NOFINGER);

  Serial.println(F("[FP] Place same finger again"));
  p = -1;
  while (p != FINGERPRINT_OK) {
    p = finger.getImage();
    if (p == FINGERPRINT_NOFINGER) continue;
    if (p != FINGERPRINT_OK) return p;
  }
  Serial.println(F("[FP] Image taken"));

  p = finger.image2Tz(2);
  if (p != FINGERPRINT_OK) return p;

  p = finger.createModel();
  if (p != FINGERPRINT_OK) return p;

  p = finger.storeModel(id);
  if (p == FINGERPRINT_OK) {
    Serial.println(F("[FP] Stored successfully."));
  }
  return p;
}

// ==================== MOTOR CONTROL ====================

void stopMotors() {
  digitalWrite(PIN_IN1, LOW); digitalWrite(PIN_IN2, LOW);
  digitalWrite(PIN_IN3, LOW); digitalWrite(PIN_IN4, LOW);
  analogWrite(PIN_ENA, 0); analogWrite(PIN_ENB, 0);
}

void moveForward(byte spd) {
  digitalWrite(PIN_IN1, HIGH); digitalWrite(PIN_IN2, LOW);
  digitalWrite(PIN_IN3, HIGH); digitalWrite(PIN_IN4, LOW);
  analogWrite(PIN_ENA, spd); analogWrite(PIN_ENB, spd);
}

void moveBackward(byte spd) {
  digitalWrite(PIN_IN1, LOW); digitalWrite(PIN_IN2, HIGH);
  digitalWrite(PIN_IN3, LOW); digitalWrite(PIN_IN4, HIGH);
  analogWrite(PIN_ENA, spd); analogWrite(PIN_ENB, spd);
}

void turnLeft(byte spd) {
  // Pivot turn: left side back, right side forward
  digitalWrite(PIN_IN1, LOW); digitalWrite(PIN_IN2, HIGH);
  digitalWrite(PIN_IN3, HIGH); digitalWrite(PIN_IN4, LOW);
  analogWrite(PIN_ENA, spd); analogWrite(PIN_ENB, spd);
}

void turnRight(byte spd) {
  digitalWrite(PIN_IN1, HIGH); digitalWrite(PIN_IN2, LOW);
  digitalWrite(PIN_IN3, LOW); digitalWrite(PIN_IN4, HIGH);
  analogWrite(PIN_ENA, spd); analogWrite(PIN_ENB, spd);
}

// ==================== SENSOR HELPERS ====================

int getDistance() {
  digitalWrite(PIN_TRIG, LOW);
  delayMicroseconds(2);
  digitalWrite(PIN_TRIG, HIGH);
  delayMicroseconds(10);
  digitalWrite(PIN_TRIG, LOW);
  
  long duration = pulseIn(PIN_ECHO, HIGH, 30000); // 30ms timeout
  if (duration == 0) return 999; // Out of range
  int distance = duration * 0.034 / 2;
  return constrain(distance, 0, 400);
}

int scanAtAngle(byte angle) {
  servo.write(angle);
  delay(180); // Allow servo to reach position
  return getDistance();
}

// ==================== LCD HELPER ====================

void lcdPrint(const char* line1, const char* line2) {
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print(line1);
  lcd.setCursor(0, 1);
  lcd.print(line2);
}

// ==================== EEPROM HELPERS ====================

bool isConfigValid() {
  return (EEPROM.read(ADDR_MAGIC) == EEPROM_MAGIC);
}

void factoryReset() {
  EEPROM.write(ADDR_MAGIC, EEPROM_MAGIC);
  for (byte i = 0; i < 4; i++) EEPROM.write(ADDR_RFID_UID + i, 0x00);
  EEPROM.write(ADDR_FP_ID, 0xFF);       // 0xFF = no authorized ID
  EEPROM.write(ADDR_MTR_SPD, DEF_MOTOR_SPEED);
  Serial.println(F("[EEPROM] Factory reset complete."));
}

void saveRFID(byte* uid) {
  for (byte i = 0; i < 4; i++) EEPROM.write(ADDR_RFID_UID + i, uid[i]);
}

void viewEEPROM() {
  Serial.println(F("\n--- EEPROM CONFIG ---"));
  Serial.print(F("Magic: 0x")); Serial.println(EEPROM.read(ADDR_MAGIC), HEX);
  Serial.print(F("RFID UID: "));
  for (byte i = 0; i < 4; i++) {
    if (EEPROM.read(ADDR_RFID_UID + i) < 0x10) Serial.print(F("0"));
    Serial.print(EEPROM.read(ADDR_RFID_UID + i), HEX);
    Serial.print(F(" "));
  }
  Serial.println();
  Serial.print(F("FP ID: ")); Serial.println(EEPROM.read(ADDR_FP_ID));
  Serial.print(F("Speed: ")); Serial.println(EEPROM.read(ADDR_MTR_SPD));
  Serial.println(F("---------------------\n"));
}

// ==================== SERIAL DEBUG INTERFACE ====================

void handleSerial() {
  if (!Serial.available()) return;
  char cmd = Serial.read();

  switch (cmd) {
    // ---- System Control ----
    case 'h': printHelp(); break;
    case 'D':
      if (state != ST_DEBUG) {
        state = ST_DEBUG;
        stopMotors();
        lcdPrint("DEBUG MODE", "'h' for help");
        Serial.println(F("\n=== ENTERING DEBUG MODE ==="));
        printHelp();
      } else {
        state = ST_LOCKED;
        stopMotors();
        lcdPrint("Locked", "Tap RFID");
        Serial.println(F("[DEBUG] Exited to LOCKED."));
      }
      break;
    case 'a':
      state = ST_ACTIVE;
      lcdPrint("Debug: Active", "");
      Serial.println(F("[DEBUG] Force ACTIVE."));
      break;
    case 'l':
      state = ST_LOCKED;
      stopMotors();
      lcdPrint("Locked", "Tap RFID");
      Serial.println(F("[DEBUG] Force LOCKED."));
      break;
    case 'd':
      verboseDebug = !verboseDebug;
      Serial.println(verboseDebug ? F("[DEBUG] Verbose ON") : F("[DEBUG] Verbose OFF"));
      break;

    // ---- Component Tests ----
    case 'm':
      Serial.println(F("[TEST] Motors Fwd->Back->Left->Right"));
      moveForward(DEF_MOTOR_SPEED); delay(600);
      moveBackward(DEF_MOTOR_SPEED); delay(600);
      turnLeft(DEF_MOTOR_SPEED); delay(600);
      turnRight(DEF_MOTOR_SPEED); delay(600);
      stopMotors();
      Serial.println(F("[TEST] Motors done."));
      break;
    case 'u':
      Serial.println(F("[TEST] Ultrasonic (10 readings):"));
      for (byte i = 0; i < 10; i++) {
        Serial.println(getDistance());
        delay(250);
      }
      break;
    case 'o':
      Serial.println(F("[TEST] Servo sweep L->C->R"));
      servo.write(SERVO_LEFT); delay(400);
      servo.write(SERVO_CENTER); delay(400);
      servo.write(SERVO_RIGHT); delay(400);
      servo.write(SERVO_CENTER);
      break;
    case 'r':
      readAndPrintRFID();
      break;
    case 'f':
      Serial.println(F("[TEST] Place finger to search..."));
      if (checkFingerprintDebug()) {
        Serial.print(F("[TEST] Match ID: ")); 
        Serial.print(finger.fingerID); 
        Serial.print(F(" Conf: ")); 
        Serial.println(finger.confidence);
      } else {
        Serial.println(F("[TEST] No match or error."));
      }
      break;

    // ---- Enrollment ----
    case 'R':
      Serial.println(F("[ENROLL] Tap RFID card to save as authorized..."));
      enrollRFID();
      break;
    case 'e':
      Serial.println(F("[ENROLL] Enter FP ID (1-127) in Serial Monitor, then send:"));
      break;
    case '1': case '2': case '3': case '4': case '5':
    case '6': case '7': case '8': case '9':
      if (state == ST_DEBUG && Serial.peek() != -1) {
        // Catch multi-digit ID if user types quickly
      }
      break;

    // ---- EEPROM ----
    case 'v':
      viewEEPROM();
      break;
    case 'x':
      factoryReset();
      lcdPrint("EEPROM Reset", "");
      break;
      
    default:
      if (cmd >= '0' && cmd <= '9' && state == ST_DEBUG) {
        // Quick hack: read full number for FP ID
        byte val = cmd - '0';
        while (Serial.available() && Serial.peek() >= '0' && Serial.peek() <= '9') {
          val = val * 10 + (Serial.read() - '0');
        }
        if (val > 0 && val <= 127) {
          Serial.print(F("[ENROLL] Enrolling to ID ")); Serial.println(val);
          if (getFingerprintEnroll(val) == FINGERPRINT_OK) {
            EEPROM.write(ADDR_FP_ID, val);
            Serial.println(F("[ENROLL] Authorized FP ID saved to EEPROM."));
          } else {
            Serial.println(F("[ENROLL] Failed."));
          }
        }
      }
      break;
  }
  
  // Flush stray input
  while (Serial.available()) Serial.read();
}

void printHelp() {
  Serial.println(F("\n========== DEBUG MENU =========="));
  Serial.println(F(" h  : Print this help"));
  Serial.println(F(" D  : Toggle Debug/Run mode"));
  Serial.println(F(" a  : Force ACTIVATE (skip auth)"));
  Serial.println(F(" l  : Force LOCK"));
  Serial.println(F(" d  : Toggle verbose sensor output"));
  Serial.println(F("-------------------------------"));
  Serial.println(F(" m  : Test Motors"));
  Serial.println(F(" u  : Test Ultrasonic"));
  Serial.println(F(" o  : Test Servo sweep"));
  Serial.println(F(" r  : Read & print RFID UID"));
  Serial.println(F(" f  : Test Fingerprint search"));
  Serial.println(F("-------------------------------"));
  Serial.println(F(" R  : Enroll new RFID to EEPROM"));
  Serial.println(F(" e  : Enroll Fingerprint (send 'e', then type ID 1-127)"));
  Serial.println(F("-------------------------------"));
  Serial.println(F(" v  : View EEPROM config"));
  Serial.println(F(" x  : Factory Reset EEPROM"));
  Serial.println(F("================================\n"));
}

bool checkFingerprintDebug() {
  uint8_t p = finger.getImage();
  if (p != FINGERPRINT_OK) return false;
  p = finger.image2Tz();
  if (p != FINGERPRINT_OK) return false;
  p = finger.fingerFastSearch();
  return (p == FINGERPRINT_OK);
}

void enrollRFID() {
  unsigned long start = millis();
  while (millis() - start < 10000) {
    if (rfid.PICC_IsNewCardPresent() && rfid.PICC_ReadCardSerial()) {
      saveRFID(rfid.uid.uidByte);
      Serial.print(F("[ENROLL] Saved UID: "));
      for (byte i = 0; i < rfid.uid.size; i++) {
        if (rfid.uid.uidByte[i] < 0x10) Serial.print(F("0"));
        Serial.print(rfid.uid.uidByte[i], HEX);
        Serial.print(F(" "));
      }
      Serial.println(F("\n[ENROLL] RFID Authorized."));
      rfid.PICC_HaltA();
      rfid.PCD_StopCrypto1();
      return;
    }
  }
  Serial.println(F("[ENROLL] Timeout. No card read."));
}
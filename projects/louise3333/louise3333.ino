/* HUMAN FOLLOWING ROBOT v4.0
 * 180° Lock-On Tracker + Auto-Reverse + Safety Recovery + 2-Way Auth
 * Hardware: Arduino UNO R3, L298N, 4x DC Motors, Servo, HC-SR04,
 *           RC522 RFID, R307/AS608 FP, I2C LCD, HC-05 on Serial
 */

#include <Servo.h>
#include <Wire.h>
#include <LiquidCrystal_I2C.h>
#include <SPI.h>
#include <MFRC522.h>
#include <SoftwareSerial.h>
#include <Adafruit_Fingerprint.h>
#include <EEPROM.h>

// ==================== PIN MAP ====================
#define PIN_IN1 2
#define PIN_IN2 4
#define PIN_IN3 6
#define PIN_IN4 7
#define PIN_ENA 3
#define PIN_ENB 5

#define PIN_TRIG 8
#define PIN_ECHO A1
#define PIN_SERVO 9

#define PIN_RFID_RST A0
#define PIN_RFID_SS  10

#define PIN_FP_RX A2
#define PIN_FP_TX A3

// ==================== CONFIGURATION ====================
#define LCD_ADDR        0x27      // Change to 0x3F if your module requires it
#define EEPROM_MAGIC    0xAB

#define ADDR_MAGIC      0
#define ADDR_RFID_UID   1   // 4 bytes
#define ADDR_FP_ID      5
#define ADDR_MTR_SPD    6
#define ADDR_HYST       7
#define ADDR_CRC        8

#define DEF_MOTOR_SPEED 200
#define LOST_TIMEOUT    8000UL
#define COLLISION_DIST  10

// ==================== OBJECTS ====================
Servo servo;
LiquidCrystal_I2C lcd(LCD_ADDR, 16, 2);
MFRC522 rfid(PIN_RFID_SS, PIN_RFID_RST);
SoftwareSerial fpSerial(PIN_FP_RX, PIN_FP_TX);
Adafruit_Fingerprint finger(&fpSerial);

// ==================== STATE MACHINE ====================
enum SystemState { ST_LOCKED, ST_RFID_OK, ST_ACTIVE, ST_SAFETY_STOP, ST_DEBUG };
SystemState state = ST_LOCKED;

unsigned long stateTimer = 0;
unsigned long lastLive = 0;
unsigned long lostStartTime = 0;

bool verboseDebug = false;
bool liveMode = false;
byte currentSpeed = DEF_MOTOR_SPEED;

// 180° Tracking Globals
int g_targetDist = 999;     // Last measured distance (cm)
byte g_targetAngle = 90;    // Last measured servo angle (0-180)
bool g_targetValid = false; // Do we have a confirmed lock?
byte lostCounter = 0;

// Motor ramping targets (updated by logic, smoothed by updateMotors)
int targetL = 0;
int targetR = 0;

// Bluetooth parser
char cmdBuffer[32];
uint8_t bufIdx = 0;

// Safety recovery globals
byte safetyStep = 0;
unsigned long safetyTimer = 0;
byte safetyTurnCount = 0;

// ==================== SETUP ====================
void setup() {
  Serial.begin(9600);  // HC-05 Bluetooth (disconnect during upload)

  // Motor pins
  pinMode(PIN_IN1, OUTPUT); pinMode(PIN_IN2, OUTPUT);
  pinMode(PIN_IN3, OUTPUT); pinMode(PIN_IN4, OUTPUT);
  pinMode(PIN_ENA, OUTPUT); pinMode(PIN_ENB, OUTPUT);
  rawMotor(0, 0);

  pinMode(PIN_TRIG, OUTPUT); pinMode(PIN_ECHO, INPUT);

  servo.attach(PIN_SERVO);
  servo.write(90);

  lcd.init(); lcd.backlight();

  SPI.begin(); rfid.PCD_Init();

  fpSerial.begin(57600);
  finger.begin(57600);
  if (finger.verifyPassword()) {
    DEBUG_PRINTLN(F("[INIT] Fingerprint sensor OK"));
  } else {
    DEBUG_PRINTLN(F("[INIT] Fingerprint sensor NOT DETECTED"));
  }

  if (!isConfigValid()) {
    factoryReset();
    lcdPrint("NO CONFIG", "Send 'h' Help");
    DEBUG_PRINTLN(F("[INIT] EEPROM blank. Enroll RFID/FP first."));
  } else {
    currentSpeed = EEPROM.read(ADDR_MTR_SPD);
    lcdPrint("Locked", "Tap RFID Card");
  }

  DEBUG_PRINTLN(F("=== HUMAN FOLLOWER v4.0 180° LOCK-ON ==="));
}

// ==================== MAIN LOOP ====================
void loop() {
  handleBluetooth();
  unsigned long now = millis();

  // Active collision guard (independent of slow scan loop)
  if (state == ST_ACTIVE && g_targetValid && g_targetDist < COLLISION_DIST) {
    enterSafetyStop("Collision");
  }

  switch (state) {
    case ST_LOCKED:      handleLocked(); break;
    case ST_RFID_OK:     handleRfidOk(); break;
    case ST_ACTIVE:      handleActive(now); updateFollowing(); break;
    case ST_SAFETY_STOP: safetyStopHandler(now); break;
    case ST_DEBUG:       /* Debug is command-driven */ break;
  }

  updateMotors();  // Always run for smooth ramping

  if (liveMode && now - lastLive >= 800) {
    lastLive = now;
    sendLiveStatus();
  }
}

// ==================== BLUETOOTH / SERIAL COMMANDER ====================
void clearBuffer() {
  bufIdx = 0;
  cmdBuffer[0] = '\0';
}

void handleBluetooth() {
  while (Serial.available()) {
    char c = Serial.read();
    if (c == '\n' || c == '\r') {
      if (bufIdx > 0) {
        cmdBuffer[bufIdx] = '\0';
        procCmd(cmdBuffer);
      }
      clearBuffer();
    } else if (bufIdx < sizeof(cmdBuffer) - 1) {
      cmdBuffer[bufIdx++] = c;
    }
  }
}

void procCmd(char* input) {
  char* cmd = input;
  while (isspace(*cmd)) cmd++;
  char* end = cmd + strlen(cmd) - 1;
  while (end > cmd && isspace(*end)) *end-- = '\0';
  for (char* p = cmd; *p; p++) *p = tolower(*p);

  DEBUG_PRINT(F("> ")); DEBUG_PRINTLN(cmd);

  if (strcmp(cmd, "status") == 0 || strcmp(cmd, "st") == 0) sendJsonStatus();
  else if (strcmp(cmd, "live") == 0) { liveMode = !liveMode; DEBUG_PRINT(F("Live: ")); DEBUG_PRINTLN(liveMode ? "ON" : "OFF"); }
  else if (strcmp(cmd, "a") == 0) { forceState(ST_ACTIVE, "Active", "Following..."); }
  else if (strcmp(cmd, "l") == 0) { forceState(ST_LOCKED, "Locked", "Tap RFID"); }
  else if (strcmp(cmd, "s") == 0) enterSafetyStop("BT Command");
  else if (strcmp(cmd, "d") == 0) { verboseDebug = !verboseDebug; DEBUG_PRINTLN(verboseDebug ? F("Verbose ON") : F("Verbose OFF")); }
  else if (strcmp(cmd, "h") == 0) printHelp();
  else if (strncmp(cmd, "speed ", 6) == 0) {
    int spd = atoi(cmd + 6);
    if (spd >= 80 && spd <= 255) {
      currentSpeed = spd;
      EEPROM.write(ADDR_MTR_SPD, (byte)spd);
      saveEEPROM();
      DEBUG_PRINT(F("Speed: ")); DEBUG_PRINTLN(spd);
    } else DEBUG_PRINTLN(F("Range: 80-255"));
  }
  else if (strcmp(cmd, "v") == 0) viewEEPROM();
  else if (strcmp(cmd, "x") == 0) { factoryReset(); lcdPrint("EEPROM Reset", ""); }
  else if (strcmp(cmd, "m") == 0) testMotors();
  else if (strcmp(cmd, "u") == 0) testUltrasonic();
  else if (strcmp(cmd, "o") == 0) testServoSweep();
  else if (strcmp(cmd, "r") == 0) readAndPrintRFID();
  else if (strcmp(cmd, "f") == 0) testFingerprint();
  else if (strcmp(cmd, "R") == 0) enrollRFID();
  else if (cmd[0] == 'e' && isdigit(cmd[1])) {
    int id = atoi(cmd + 1);
    if (id >= 1 && id <= 127) enrollFingerprint((uint8_t)id);
    else DEBUG_PRINTLN(F("ID must be 1-127"));
  }
  else if (strcmp(cmd, "e") == 0) DEBUG_PRINTLN(F("Usage: e<ID>  (example: e5)"));
  else DEBUG_PRINTLN(F("Unknown. Send 'h' for help."));
}

void printHelp() {
  DEBUG_PRINTLN(F("\n========== COMMAND MENU =========="));
  DEBUG_PRINTLN(F(" h          : This help"));
  DEBUG_PRINTLN(F(" a / l / s  : Force Active / Lock / Safety"));
  DEBUG_PRINTLN(F(" d          : Toggle verbose debug"));
  DEBUG_PRINTLN(F(" live       : Toggle live telemetry"));
  DEBUG_PRINTLN(F(" status     : JSON status dump"));
  DEBUG_PRINTLN(F(" speed NNN  : Set motor speed (80-255)"));
  DEBUG_PRINTLN(F("----------------------------------"));
  DEBUG_PRINTLN(F(" m : Motor test   | u : Ultrasonic test"));
  DEBUG_PRINTLN(F(" o : Servo sweep  | r : Read RFID UID"));
  DEBUG_PRINTLN(F(" f : FP search    | R : Enroll RFID"));
  DEBUG_PRINTLN(F(" eN: Enroll FP    | v : View EEPROM"));
  DEBUG_PRINTLN(F(" x : Factory reset"));
  DEBUG_PRINTLN(F("==================================\n"));
}

// ==================== AUTHENTICATION ====================
void handleLocked() {
  if (checkRFID()) {
    state = ST_RFID_OK;
    stateTimer = millis();
    lcdPrint("RFID Verified", "Scan Finger");
    DEBUG_PRINTLN(F("[AUTH] RFID OK"));
  }
}

void handleRfidOk() {
  if (millis() - stateTimer > 15000UL) {
    forceState(ST_LOCKED, "Auth Timeout", "Tap RFID");
    return;
  }

  uint8_t p = finger.getImage();
  if (p == FINGERPRINT_NOFINGER) return;
  if (p != FINGERPRINT_OK) return;

  p = finger.image2Tz();
  if (p != FINGERPRINT_OK) return;

  p = finger.fingerFastSearch();
  if (p != FINGERPRINT_OK) {
    lcdPrint("Bad Finger", "Retry...");
    DEBUG_PRINTLN(F("[AUTH] FP not found"));
    delay(600);
    return;
  }

  byte authID = EEPROM.read(ADDR_FP_ID);
  if (finger.fingerID == authID) {
    forceState(ST_ACTIVE, "System Active!", "180° Tracking");
    DEBUG_PRINTLN(F("[AUTH] FP verified. ACTIVE."));
    delay(800);
  } else {
    lcdPrint("Wrong FP ID", "Denied");
    DEBUG_PRINT(F("[AUTH] Rejected ID: ")); DEBUG_PRINTLN(finger.fingerID);
    delay(800);
  }
}

bool checkRFID() {
  if (!rfid.PICC_IsNewCardPresent() || !rfid.PICC_ReadCardSerial()) return false;
  bool match = true;
  for (byte i = 0; i < 4; i++) {
    if (rfid.uid.uidByte[i] != EEPROM.read(ADDR_RFID_UID + i)) match = false;
  }
  rfid.PICC_HaltA();
  rfid.PCD_StopCrypto1();
  return match;
}

void forceState(SystemState s, const char* l1, const char* l2) {
  state = s;
  if (s != ST_ACTIVE && s != ST_SAFETY_STOP) {
    targetL = 0; targetR = 0;
    rawMotor(0, 0);
  }
  if (s == ST_LOCKED) {
    g_targetValid = false;
    lostStartTime = 0;
    lostCounter = 0;
    servo.write(90);
  }
  lcdPrint(l1, l2);
}

// ==================== 180° ACTIVE TRACKING ====================
void handleActive(unsigned long now) {
  static enum { WIDE_SWEEP, TRACK_WINDOW } scanMode = WIDE_SWEEP;
  static enum { SET_ANGLE, WAIT_SETTLE, READ_DIST, NEXT_POS } phase = SET_ANGLE;
  static unsigned long timer = 0;
  static byte idx = 0;
  static int bestDist = 999;
  static byte bestAngle = 90;

  const byte WIDE_ANGLES[] = {0, 20, 40, 60, 80, 100, 120, 140, 160, 180};
  const byte NUM_WIDE = sizeof(WIDE_ANGLES);
  static byte trackAngles[5];
  static byte numTrack = 0;

  if (now < timer) return;

  switch (phase) {
    case SET_ANGLE: {
      byte a = (scanMode == WIDE_SWEEP) ? WIDE_ANGLES[idx] : trackAngles[idx];
      servo.write(a);
      timer = now + ((scanMode == WIDE_SWEEP) ? 200 : 150);
      phase = READ_DIST;
      break;
    }

    case READ_DIST: {
      int d = getSmoothDistance();
      if (d > 0 && d < bestDist) {
        bestDist = d;
        bestAngle = (scanMode == WIDE_SWEEP) ? WIDE_ANGLES[idx] : trackAngles[idx];
      }
      timer = now + 40;
      phase = NEXT_POS;
      break;
    }

    case NEXT_POS:
      idx++;
      if (scanMode == WIDE_SWEEP) {
        if (idx >= NUM_WIDE) {
          if (bestDist < 160) {
            // Target acquired -> lock on
            g_targetValid = true;
            g_targetAngle = bestAngle;
            g_targetDist = bestDist;
            lostCounter = 0;
            lostStartTime = 0;

            numTrack = 0;
            for (int off = -20; off <= 20; off += 10) {
              int a = (int)bestAngle + off;
              if (a >= 0 && a <= 180) trackAngles[numTrack++] = (byte)a;
            }
            scanMode = TRACK_WINDOW;
          } else {
            g_targetValid = false;
            if (lostStartTime == 0) lostStartTime = now;
            if (now - lostStartTime > LOST_TIMEOUT) {
              enterSafetyStop("Target Lost");
            }
          }
          idx = 0; bestDist = 999;
        }
      } else { // TRACK_WINDOW
        if (idx >= numTrack) {
          if (bestDist < 200) {
            g_targetValid = true;
            g_targetAngle = bestAngle;
            g_targetDist = bestDist;
            lostCounter = 0;

            // Rebuild window around new angle
            numTrack = 0;
            for (int off = -20; off <= 20; off += 10) {
              int a = (int)bestAngle + off;
              if (a >= 0 && a <= 180) trackAngles[numTrack++] = (byte)a;
            }
          } else {
            lostCounter++;
            if (lostCounter > 2) {
              g_targetValid = false;
              if (lostStartTime == 0) lostStartTime = now;
              scanMode = WIDE_SWEEP;
            }
          }
          idx = 0; bestDist = 999;
        }
      }
      phase = SET_ANGLE;
      break;
  }

  // LCD update (throttled)
  static unsigned long lastLcd = 0;
  if (now - lastLcd > 300) {
    lastLcd = now;
    if (g_targetValid) {
      char b1[17], b2[17];
      snprintf(b1, 16, "FOLLOW %dCM", g_targetDist);
      char dir = (g_targetAngle < 80) ? '<' : (g_targetAngle > 100) ? '>' : '^';
      snprintf(b2, 16, "ANG:%d%c L:%d", g_targetAngle, dir, lostCounter);
      lcdPrint(b1, b2);
    } else {
      lcdPrint("SCANNING 180", "NO TARGET");
    }
  }
}

// ==================== FOLLOWING LOGIC (PROPORTIONAL) ====================
void updateFollowing() {
  if (state != ST_ACTIVE) { targetL = 0; targetR = 0; return; }
  if (!g_targetValid) {
    // Slow search spin while sweeping happens
    targetL = 110; targetR = -110;
    return;
  }

  int dist = g_targetDist;
  byte ang = g_targetAngle;
  int base = currentSpeed;

  int spd = 0;
  if (dist < 15) {
    // Human walked in front / too close -> reverse away
    spd = -map(constrain(dist, 5, 15), 5, 15, base, 90);
  } else if (dist < 30) {
    spd = 0; // Hold position in sweet zone
  } else if (dist < 120) {
    spd = map(constrain(dist, 30, 120), 30, 120, 80, base);
  } else {
    spd = base;
  }

  // Proportional steering: 90 = center
  // turn is positive for right, negative for left
  int turn = ((int)ang - 90) * 3;

  targetL = spd - turn;
  targetR = spd + turn;
  targetL = constrain(targetL, -255, 255);
  targetR = constrain(targetR, -255, 255);

  if (verboseDebug) {
    DEBUG_PRINT(F("[FOLLOW] D:")); DEBUG_PRINT(dist);
    DEBUG_PRINT(F(" A:")); DEBUG_PRINT(ang);
    DEBUG_PRINT(F(" L:")); DEBUG_PRINT(targetL);
    DEBUG_PRINT(F(" R:")); DEBUG_PRINTLN(targetR);
  }
}

// ==================== MOTOR CONTROL (RAMPED) ====================
void updateMotors() {
  static unsigned long lastUpdate = 0;
  if (millis() - lastUpdate < 25) return; // 40 Hz loop
  lastUpdate = millis();

  static int curL = 0, curR = 0;
  const int maxStep = 28; // Max PWM delta per tick (smooth acceleration)

  curL += constrain(targetL - curL, -maxStep, maxStep);
  curR += constrain(targetR - curR, -maxStep, maxStep);

  rawMotor(curL, curR);
}

void rawMotor(int l, int r) {
  if (l >= 0) { digitalWrite(PIN_IN1, HIGH); digitalWrite(PIN_IN2, LOW); }
  else        { digitalWrite(PIN_IN1, LOW);  digitalWrite(PIN_IN2, HIGH); }
  if (r >= 0) { digitalWrite(PIN_IN3, HIGH); digitalWrite(PIN_IN4, LOW); }
  else        { digitalWrite(PIN_IN3, LOW);  digitalWrite(PIN_IN4, HIGH); }
  analogWrite(PIN_ENA, abs(l));
  analogWrite(PIN_ENB, abs(r));
}

void moveForward(int s)  { targetL = s; targetR = s; }
void moveBackward(int s) { targetL = -s; targetR = -s; }
void turnLeft(int s)     { targetL = -s; targetR = s; }
void turnRight(int s)    { targetL = s; targetR = -s; }
void stopMotors()        { targetL = 0; targetR = 0; rawMotor(0, 0); }

// ==================== SAFETY STOP & RECOVERY ====================
void enterSafetyStop(const char* reason) {
  state = ST_SAFETY_STOP;
  stopMotors();
  safetyStep = 0;
  safetyTimer = millis();
  safetyTurnCount = 0;
  lcdPrint("SAFETY STOP", reason);
  DEBUG_PRINT(F("[SAFETY] ")); DEBUG_PRINTLN(reason);
}

void safetyStopHandler(unsigned long now) {
  switch (safetyStep) {
    case 0: // Back up
      moveBackward(140);
      if (now - safetyTimer > 500) {
        stopMotors();
        safetyTimer = now;
        safetyStep = 1;
      }
      break;
    case 1: // Turn to scan new quadrant
      turnRight(160);
      if (now - safetyTimer > 400) {
        stopMotors();
        safetyTimer = now;
        safetyStep = 2;
      }
      break;
    case 2: // Scan
      if (now - safetyTimer > 250) {
        int d = getSmoothDistance();
        if (d > 25 && d < 150) {
          // Reacquired
          g_targetDist = d;
          g_targetAngle = 90;
          g_targetValid = true;
          lostCounter = 0;
          lostStartTime = 0;
          state = ST_ACTIVE;
          lcdPrint("Recovered", "Re-engaging");
          DEBUG_PRINTLN(F("[SAFETY] Target reacquired"));
        } else {
          safetyTurnCount++;
          if (safetyTurnCount >= 4) {
            forceState(ST_LOCKED, "Auth Required", "Tap RFID");
            DEBUG_PRINTLN(F("[SAFETY] Give up, locked."));
          } else {
            safetyTimer = now;
            safetyStep = 1;
          }
        }
      }
      break;
  }
}

// ==================== SENSORS ====================
int getSmoothDistance() {
  long sum = 0;
  for (byte i = 0; i < 3; i++) {
    int d = getDistance();
    if (d == 999) return 999;
    sum += d;
    delayMicroseconds(500);
  }
  return (int)(sum / 3);
}

int getDistance() {
  digitalWrite(PIN_TRIG, LOW); delayMicroseconds(2);
  digitalWrite(PIN_TRIG, HIGH); delayMicroseconds(10);
  digitalWrite(PIN_TRIG, LOW);
  long dur = pulseIn(PIN_ECHO, HIGH, 30000);
  return (dur == 0) ? 999 : constrain((int)(dur * 0.034 / 2), 0, 400);
}

// ==================== LCD ====================
void lcdPrint(const char* line1, const char* line2) {
  lcd.clear();
  lcd.setCursor(0, 0); lcd.print(line1);
  lcd.setCursor(0, 1); lcd.print(line2);
}

// ==================== EEPROM ====================
byte calcCRC() {
  byte sum = 0;
  for (byte i = ADDR_MAGIC; i <= ADDR_HYST; i++) sum += EEPROM.read(i);
  return sum;
}

bool isConfigValid() {
  return (EEPROM.read(ADDR_MAGIC) == EEPROM_MAGIC) && (EEPROM.read(ADDR_CRC) == calcCRC());
}

void saveEEPROM() {
  EEPROM.write(ADDR_MAGIC, EEPROM_MAGIC);
  EEPROM.write(ADDR_CRC, calcCRC());
}

void factoryReset() {
  EEPROM.write(ADDR_MAGIC, EEPROM_MAGIC);
  for (byte i = 0; i < 4; i++) EEPROM.write(ADDR_RFID_UID + i, 0x00);
  EEPROM.write(ADDR_FP_ID, 0xFF);
  EEPROM.write(ADDR_MTR_SPD, DEF_MOTOR_SPEED);
  EEPROM.write(ADDR_HYST, 5);
  saveEEPROM();
  currentSpeed = DEF_MOTOR_SPEED;
  DEBUG_PRINTLN(F("[EEPROM] Factory reset"));
}

void viewEEPROM() {
  DEBUG_PRINTLN(F("\n--- EEPROM ---"));
  DEBUG_PRINT(F("Magic: 0x")); DEBUG_PRINTLN(EEPROM.read(ADDR_MAGIC), HEX);
  DEBUG_PRINT(F("RFID: "));
  for (byte i = 0; i < 4; i++) {
    if (EEPROM.read(ADDR_RFID_UID + i) < 0x10) DEBUG_PRINT(F("0"));
    DEBUG_PRINT(EEPROM.read(ADDR_RFID_UID + i), HEX); DEBUG_PRINT(F(" "));
  }
  DEBUG_PRINTLN();
  DEBUG_PRINT(F("FP ID: ")); DEBUG_PRINTLN(EEPROM.read(ADDR_FP_ID));
  DEBUG_PRINT(F("Speed: ")); DEBUG_PRINTLN(EEPROM.read(ADDR_MTR_SPD));
  DEBUG_PRINT(F("CRC:   ")); DEBUG_PRINTLN(EEPROM.read(ADDR_CRC));
  DEBUG_PRINTLN(F("--------------\n"));
}

// ==================== ENROLLMENT ====================
void enrollRFID() {
  lcdPrint("Enroll RFID", "Tap card now");
  DEBUG_PRINTLN(F("[ENROLL] Waiting for RFID..."));
  unsigned long t = millis();
  while (millis() - t < 10000) {
    if (rfid.PICC_IsNewCardPresent() && rfid.PICC_ReadCardSerial()) {
      for (byte i = 0; i < 4; i++) EEPROM.write(ADDR_RFID_UID + i, rfid.uid.uidByte[i]);
      saveEEPROM();
      DEBUG_PRINT(F("[ENROLL] Saved UID: "));
      for (byte i = 0; i < 4; i++) {
        if (rfid.uid.uidByte[i] < 0x10) DEBUG_PRINT(F("0"));
        DEBUG_PRINT(rfid.uid.uidByte[i], HEX); DEBUG_PRINT(F(" "));
      }
      DEBUG_PRINTLN();
      rfid.PICC_HaltA();
      rfid.PCD_StopCrypto1();
      lcdPrint("RFID Saved", "");
      return;
    }
  }
  DEBUG_PRINTLN(F("[ENROLL] Timeout"));
  lcdPrint("RFID Timeout", "");
}

void enrollFingerprint(uint8_t id) {
  lcdPrint("Enroll FP", "ID:");
  lcd.print(id);
  DEBUG_PRINT(F("[ENROLL] FP ID ")); DEBUG_PRINTLN(id);
  DEBUG_PRINTLN(F("Place finger..."));

  int p = -1;
  while (p != FINGERPRINT_OK) {
    p = finger.getImage();
    if (p == FINGERPRINT_NOFINGER) continue;
    if (p != FINGERPRINT_OK) { DEBUG_PRINTLN(F("Image fail")); return; }
  }
  DEBUG_PRINTLN(F("Image 1 OK"));
  p = finger.image2Tz(1);
  if (p != FINGERPRINT_OK) return;

  DEBUG_PRINTLN(F("Remove finger"));
  unsigned long t = millis();
  while (millis() - t < 2000) if (finger.getImage() == FINGERPRINT_NOFINGER) break;
  while (finger.getImage() != FINGERPRINT_NOFINGER);

  DEBUG_PRINTLN(F("Place same finger"));
  p = -1;
  while (p != FINGERPRINT_OK) {
    p = finger.getImage();
    if (p == FINGERPRINT_NOFINGER) continue;
    if (p != FINGERPRINT_OK) return;
  }
  DEBUG_PRINTLN(F("Image 2 OK"));
  p = finger.image2Tz(2);
  if (p != FINGERPRINT_OK) return;

  p = finger.createModel();
  if (p != FINGERPRINT_OK) { DEBUG_PRINTLN(F("Model fail")); return; }

  p = finger.storeModel(id);
  if (p == FINGERPRINT_OK) {
    EEPROM.write(ADDR_FP_ID, id);
    saveEEPROM();
    DEBUG_PRINTLN(F("[ENROLL] Stored OK"));
    lcdPrint("FP Stored", "OK");
  } else {
    DEBUG_PRINTLN(F("[ENROLL] Store fail"));
    lcdPrint("FP Store", "Fail");
  }
}

// ==================== TEST / DEBUG HELPERS ====================
void testMotors() {
  DEBUG_PRINTLN(F("[TEST] Forward"));
  moveForward(180); runForMs(600);
  DEBUG_PRINTLN(F("[TEST] Backward"));
  moveBackward(180); runForMs(600);
  DEBUG_PRINTLN(F("[TEST] Left"));
  turnLeft(180); runForMs(600);
  DEBUG_PRINTLN(F("[TEST] Right"));
  turnRight(180); runForMs(600);
  stopMotors();
  DEBUG_PRINTLN(F("[TEST] Done"));
}

void runForMs(unsigned long ms) {
  unsigned long t = millis();
  while (millis() - t < ms) updateMotors();
}

void testUltrasonic() {
  DEBUG_PRINTLN(F("[TEST] Ultrasonic (10 reads):"));
  for (byte i = 0; i < 10; i++) {
    DEBUG_PRINTLN(getSmoothDistance());
    delay(250);
  }
}

void testServoSweep() {
  DEBUG_PRINTLN(F("[TEST] Servo 0->180->90"));
  servo.write(0);   delay(400);
  servo.write(180); delay(400);
  servo.write(90);  delay(400);
  DEBUG_PRINTLN(F("[TEST] Done"));
}

void readAndPrintRFID() {
  if (!rfid.PICC_IsNewCardPresent() || !rfid.PICC_ReadCardSerial()) {
    DEBUG_PRINTLN(F("[RFID] No card"));
    return;
  }
  DEBUG_PRINT(F("[RFID] UID: "));
  for (byte i = 0; i < rfid.uid.size; i++) {
    if (rfid.uid.uidByte[i] < 0x10) DEBUG_PRINT(F("0"));
    DEBUG_PRINT(rfid.uid.uidByte[i], HEX); DEBUG_PRINT(F(" "));
  }
  DEBUG_PRINTLN();
  rfid.PICC_HaltA();
  rfid.PCD_StopCrypto1();
}

void testFingerprint() {
  DEBUG_PRINTLN(F("[TEST] Scan finger..."));
  uint8_t p = finger.getImage();
  if (p != FINGERPRINT_OK) { DEBUG_PRINTLN(F("No image")); return; }
  p = finger.image2Tz();
  if (p != FINGERPRINT_OK) return;
  p = finger.fingerFastSearch();
  if (p == FINGERPRINT_OK) {
    DEBUG_PRINT(F("Match ID: ")); DEBUG_PRINT(finger.fingerID);
    DEBUG_PRINT(F(" Conf: ")); DEBUG_PRINTLN(finger.confidence);
  } else {
    DEBUG_PRINTLN(F("No match"));
  }
}

void sendLiveStatus() {
  DEBUG_PRINT(F("LIVE|State:"));
  DEBUG_PRINT(state == ST_ACTIVE ? "ACTIVE" : state == ST_SAFETY_STOP ? "SAFETY" : "LOCKED");
  DEBUG_PRINT(F("|Dist:")); DEBUG_PRINT(g_targetDist);
  DEBUG_PRINT(F("|Ang:")); DEBUG_PRINT(g_targetAngle);
  DEBUG_PRINT(F("|Valid:")); DEBUG_PRINT(g_targetValid ? "1" : "0");
  DEBUG_PRINT(F("|Spd:")); DEBUG_PRINTLN(currentSpeed);
}

void sendJsonStatus() {
  DEBUG_PRINTLN(F("{"));
  DEBUG_PRINT(F("  \"state\": \""));
  DEBUG_PRINT(state == ST_ACTIVE ? "ACTIVE" : state == ST_SAFETY_STOP ? "SAFETY_STOP" : "LOCKED");
  DEBUG_PRINTLN(F("\","));
  DEBUG_PRINT(F("  \"distance\": ")); DEBUG_PRINT(g_targetDist); DEBUG_PRINTLN(F(","));
  DEBUG_PRINT(F("  \"angle\": ")); DEBUG_PRINT(g_targetAngle); DEBUG_PRINTLN(F(","));
  DEBUG_PRINT(F("  \"locked\": ")); DEBUG_PRINTLN(g_targetValid ? F("true,") : F("false,"));
  DEBUG_PRINT(F("  \"speed\": ")); DEBUG_PRINT(currentSpeed); DEBUG_PRINTLN(F(","));
  DEBUG_PRINT(F("  \"live\": ")); DEBUG_PRINTLN(liveMode ? F("true") : F("false"));
  DEBUG_PRINTLN(F("}"));
}
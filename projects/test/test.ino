/**
 * ============================================================
 *  SAR DISASTER RESPONSE ROBOT - ARDUINO 2 (MASTER BRAIN)
 *  Version: 3.0 SENSOR FUSION & SAR INTELLIGENCE
 * ============================================================
 *  Role    : I2C Master - Autonomous SAR operations with 
 *            sensor fusion, spatial memory, and victim detection
 *
 *  PIN MAP (UNCHANGED):
 *    HC-05 BT RX  : D2  (SoftwareSerial RX)
 *    HC-05 BT TX  : D3  (SoftwareSerial TX)
 *    NeoPixel x8  : D6
 *    DHT11        : D5
 *    Front US TRIG: D4
 *    PIR 1        : D7
 *    PIR 2        : D8
 *    Rear US TRIG : D9
 *    Rear US ECHO : D10
 *    Front US ECHO: D11
 *    Flame Left   : A0
 *    Flame Center : A1
 *    Flame Right  : A2
 *    MQ2          : A3
 *    I2C SDA      : A4
 *    I2C SCL      : A5
 * ============================================================
 */

#include <SoftwareSerial.h>
#include <DHT.h>
#include <Wire.h>
#include <EEPROM.h>
#include <Adafruit_NeoPixel.h>
#include <string.h>

// ============================================================
//  TUNING ZONE - SAR OPTIMIZED PARAMETERS
// ============================================================

#define DIST_STOP             15
#define DIST_WARN             30
#define DIST_CLEAR            40
#define DIST_VICTIM_APPROACH  35
#define DIST_SAFE_RETREAT     50

#define GAS_THRESHOLD         400
#define GAS_DANGER            600
#define GAS_CLEAR_MARGIN      60

#define TEMP_WARN             38
#define TEMP_DANGER           50
#define TEMP_VICTIM_DELTA     -2

#define FLAME_THRESHOLD       120
#define FLAME_ARM_DELAY_MS    8000
#define FLAME_CONFIRM_COUNT   2

#define ENABLE_FLAME_SENSORS  1
#define ENABLE_REAR_US        1
#define ENABLE_MOTOR_I2C      1
#define ENABLE_BT_DEBUG       1
#define ENABLE_BOOT_I2C_SCAN  0
#define ENABLE_I2C_SCAN_DEBUG 1
#define ENABLE_NEOPIXEL       1
#define ENABLE_FLAME_HAZARD   1

#define AUTO_BACKUP_MS        600
#define AUTO_TURN_MS          700
#define AUTO_SCAN_PAUSE       220
#define AUTO_SCAN_RIGHT_PAUSE 300
#define AUTO_CENTER_PAUSE     120
#define AUTO_PATROL_SPEED     1
#define AUTO_MOTION_PAUSE_MS  2000
#define AUTO_MODE_TARGET_PWM  80
#define HAZARD_OBSERVE_DIST   60
#define OBSTACLE_MEMORY_SLOTS 9
#define OBSTACLE_DECAY_MS     8000

#define INVESTIGATE_APPROACH_MS 1500
#define VICTIM_CONFIRM_DELAY    1500
#define SEARCH_PATTERN_STEP     4000
#define STUCK_TIMEOUT_MS        30000
#define PIR_HISTORY_DEPTH       4

#define TELEM_INTERVAL_MS       500
#define SENSOR_INTERVAL_MS      200
#define DHT_INTERVAL_MS         2000
#define COMMAND_GAP_MS          20
#define USB_FALLBACK_TO_BT_MS   5000
#define US_TIMEOUT_US           18000UL
#define US_SAMPLES              2

#define MQ2_WARMUP_MS           20000
#define MQ2_EEPROM_MAGIC        0x4D51
#define MQ2_EEPROM_ADDR         0
#define MQ2_SAMPLES             8
#define MQ2_FILTER_WEIGHT_OLD   3
#define MQ2_FILTER_WEIGHT_NEW   1

#define RADAR_SERVO_MIN         0
#define RADAR_SERVO_CENTER      90
#define RADAR_SERVO_MAX         180
#define RADAR_SERVO_STEP        10

// ============================================================
//  PIN DEFINITIONS
// ============================================================

#define PIN_BT_RX             2
#define PIN_BT_TX             3
#define PIN_NEOPIXEL          6
#define PIN_DHT               5
#define PIN_PIR1              7
#define PIN_PIR2              8
#define PIN_US_REAR_TRIG      9
#define PIN_US_REAR_ECHO      10
#define PIN_US_FRONT_ECHO     11
#define PIN_US_FRONT_TRIG     4
#define PIN_FLAME_LEFT        A0
#define PIN_FLAME_CENTER      A1
#define PIN_FLAME_RIGHT       A2
#define PIN_MQ2               A3
#define I2C_ADDR              8
#define NEOPIXEL_COUNT        8
#define MOTOR_I2C_CLOCK_HZ    100000L

// ============================================================
//  OBJECTS
// ============================================================

SoftwareSerial BT(PIN_BT_RX, PIN_BT_TX);
DHT dht(PIN_DHT, DHT11);
#if ENABLE_NEOPIXEL
Adafruit_NeoPixel statusPixels(NEOPIXEL_COUNT, PIN_NEOPIXEL, NEO_GRB + NEO_KHZ800);
#endif

// ============================================================
//  SENSOR FUSION STRUCTURES
// ============================================================

struct ObstacleMap {
  uint8_t confidence[OBSTACLE_MEMORY_SLOTS];
  unsigned long timestamp[OBSTACLE_MEMORY_SLOTS];
  float distance[OBSTACLE_MEMORY_SLOTS];
};

struct VictimTracker {
  bool detected;
  int bearing;
  uint8_t confidence;
  unsigned long firstDetection;
  uint8_t pirHistory;
  float approachDistance;
};

struct HazardAssessment {
  uint8_t fireRisk;
  uint8_t gasRisk;
  uint8_t thermalRisk;
  uint8_t overall;
  bool critical;
  int fireBearing;
};

struct NavigationState {
  int lastTurnDirection;
  uint8_t stuckCounter;
  unsigned long patrolStartTime;
  float victimVectorX;
  float victimVectorY;
};

// ============================================================
//  GLOBAL STATE
// ============================================================

float sTemp = 0.0f;
float sHumid = 0.0f;
float sFrontDist = 999.0f;
float sRearDist = 999.0f;
int sPIR1 = LOW;
int sPIR2 = LOW;
int sFlameA1 = 1023;
int sFlameA2 = 1023;
int sFlameA3 = 1023;
int sGasRaw = 0;
int sGas = 0;
int mq2Baseline = 0;

bool alertFlame = false;
bool alertGas = false;
bool alertObstacleFront = false;
bool alertObstacleRear = false;
bool alertMotion = false;
bool alertTemp = false;

bool autonomous = false;
bool hazardActive = false;
bool autoSafety = true;
bool telemActive = false;
bool nightMode = false;

enum AutoState {
  AUTO_PATROL,
  AUTO_OBSTACLE_FOUND,
  AUTO_BACKING,
  AUTO_SCANNING,
  AUTO_TURNING,
  AUTO_HAZARD,
  AUTO_INVESTIGATE,
  AUTO_CONFIRM_VICTIM,
  AUTO_VICTIM_FOUND,
  AUTO_SEARCH_PATTERN,
  AUTO_RETREAT_HAZARD
};

AutoState autoState = AUTO_PATROL;
AutoState preHazardState = AUTO_PATROL;

enum ScanPhase {
  SCAN_IDLE,
  SCAN_WAIT_LEFT,
  SCAN_WAIT_RIGHT,
  SCAN_WAIT_CENTER
};
ScanPhase scanPhase = SCAN_IDLE;

bool lastTurnLeft = true;
char lastCmd = 'S';
char pendingTurnCmd = 'S';
const char *currentMode = "IDLE";
float scanLeftDist = 999.0f;
float scanRightDist = 999.0f;
int radarServoAngle = RADAR_SERVO_CENTER;

unsigned long lastSensorTime = 0;
unsigned long lastTelemTime = 0;
unsigned long lastDhtTime = 0;
unsigned long autoTimer = 0;
unsigned long motionPauseUntil = 0;
unsigned long bootTime = 0;
unsigned long lastNeoPixelFrame = 0;
unsigned long lastStuckCheck = 0;
unsigned long searchPatternTimer = 0;
int searchPatternLeg = 0;

bool usbConsoleSeen = false;
bool btConsoleFallback = false;
bool btFallbackAnnounced = false;
bool btReplyMode = false;
byte neoPixelPhase = 0;

ObstacleMap obstacleMap;
VictimTracker victim;
NavigationState nav;
float previousFrontDist = 999.0f;

// ============================================================
//  DEBUG HELPERS
// ============================================================

bool useBtDebug() { return ENABLE_BT_DEBUG || btConsoleFallback; }
bool dbgMirrorBt() { return useBtDebug() || btReplyMode; }

void DBG(const __FlashStringHelper *s) {
  Serial.print(s);
  if (dbgMirrorBt()) BT.print(s);
}
void DBGLN(const __FlashStringHelper *s) {
  Serial.println(s);
  if (dbgMirrorBt()) BT.println(s);
}
void DBG(const char *s) {
  Serial.print(s);
  if (dbgMirrorBt()) BT.print(s);
}
void DBGLN(const char *s) {
  Serial.println(s);
  if (dbgMirrorBt()) BT.println(s);
}
void DBG(char c) {
  Serial.print(c);
  if (dbgMirrorBt()) BT.print(c);
}
void DBGLN(char c) {
  Serial.println(c);
  if (dbgMirrorBt()) BT.println(c);
}
void DBG(int v) {
  Serial.print(v);
  if (dbgMirrorBt()) BT.print(v);
}
void DBGLN(int v) {
  Serial.println(v);
  if (dbgMirrorBt()) BT.println(v);
}
void DBG(unsigned long v) {
  Serial.print(v);
  if (dbgMirrorBt()) BT.print(v);
}
void DBGLN(unsigned long v) {
  Serial.println(v);
  if (dbgMirrorBt()) BT.println(v);
}
void DBG(float v) {
  Serial.print(v);
  if (dbgMirrorBt()) BT.print(v);
}
void DBGLN(float v) {
  Serial.println(v);
  if (dbgMirrorBt()) BT.println(v);
}
void DBGNL() {
  Serial.println();
  if (dbgMirrorBt()) BT.println();
}
void printHexByte(byte value) {
  if (value < 16) DBG('0');
  Serial.print(value, HEX);
  if (dbgMirrorBt()) BT.print(value, HEX);
}

// ============================================================
//  SENSOR FUSION FUNCTIONS
// ============================================================

void updateObstacleMap() {
  unsigned long now = millis();
  
  for (int i = 0; i < OBSTACLE_MEMORY_SLOTS; i++) {
    if (now - obstacleMap.timestamp[i] > OBSTACLE_DECAY_MS) {
      obstacleMap.confidence[i] = max(0, obstacleMap.confidence[i] - 10);
      if (obstacleMap.confidence[i] == 0) obstacleMap.distance[i] = 999.0f;
    }
  }
  
  if (sFrontDist < DIST_CLEAR) {
    uint8_t confidence = (sFrontDist < DIST_STOP) ? 100 : (sFrontDist < DIST_WARN) ? 70 : 40;
    for (int i = 3; i <= 5; i++) {
      if (confidence > obstacleMap.confidence[i]) {
        obstacleMap.confidence[i] = confidence;
        obstacleMap.distance[i] = sFrontDist;
        obstacleMap.timestamp[i] = now;
      }
    }
  }
  
  if (sPIR1 == HIGH) {
    obstacleMap.confidence[0] = 80;
    obstacleMap.timestamp[0] = now;
  }
  if (sPIR2 == HIGH) {
    obstacleMap.confidence[8] = 80;
    obstacleMap.timestamp[8] = now;
  }
  
  if (abs(sFrontDist - previousFrontDist) < 2.0f && sFrontDist < DIST_WARN) {
    obstacleMap.confidence[4] = min(100, obstacleMap.confidence[4] + 5);
  }
  previousFrontDist = sFrontDist;
}

HazardAssessment calculateHazard() {
  HazardAssessment h;
  h.fireRisk = 0;
  h.gasRisk = 0;
  h.thermalRisk = 0;
  h.overall = 0;
  h.critical = false;
  h.fireBearing = -99;
  
  int bestFlame = max(sFlameA1, max(sFlameA2, sFlameA3));
  if (bestFlame > FLAME_THRESHOLD) {
    h.fireRisk = map(bestFlame, FLAME_THRESHOLD, 1023, 30, 100);
    if (sFlameA1 > sFlameA2 && sFlameA1 > sFlameA3) h.fireBearing = -1;
    else if (sFlameA3 > sFlameA2 && sFlameA3 > sFlameA1) h.fireBearing = 1;
    else h.fireBearing = 0;
  }
  
  if (sTemp > TEMP_WARN) {
    h.thermalRisk = map(sTemp, TEMP_WARN, TEMP_DANGER, 20, 100);
    h.fireRisk = max(h.fireRisk, h.thermalRisk - 20);
  }
  
  int gasD = gasDelta();
  if (gasD > GAS_THRESHOLD) {
    h.gasRisk = map(gasD, GAS_THRESHOLD, 900, 20, 100);
  }
  
  h.overall = max(h.fireRisk, h.gasRisk);
  if (h.fireRisk > 50 && h.gasRisk > 50) h.overall = 100;
  
  h.critical = (gasD > GAS_DANGER) || (h.fireRisk > 80) || (sTemp > TEMP_DANGER);
  
  return h;
}

void updateVictimTracking() {
  victim.pirHistory = (victim.pirHistory << 1) | ((sPIR1 == HIGH || sPIR2 == HIGH) ? 1 : 0);
  
  uint8_t popcount = 0;
  for (int i = 0; i < PIR_HISTORY_DEPTH; i++) {
    if (victim.pirHistory & (1 << i)) popcount++;
  }
  
  if (popcount >= 2) {
    victim.confidence = min(100, victim.confidence + 25);
    if (!victim.detected && victim.confidence > 60) {
      victim.firstDetection = millis();
      if (sPIR1 == HIGH && sPIR2 == LOW) victim.bearing = -45;
      else if (sPIR2 == HIGH && sPIR1 == LOW) victim.bearing = 45;
      else victim.bearing = 0;
    }
  } else {
    victim.confidence = max(0, victim.confidence - 10);
  }
  
  if (victim.confidence > 80 && victim.approachDistance < DIST_VICTIM_APPROACH) {
    victim.detected = true;
  } else if (victim.confidence < 20) {
    victim.detected = false;
  }
}

int calculateBestTurn() {
  if (victim.confidence > 50 && !victim.detected) {
    DBG(F("FUSION: Turning toward victim bearing ")); DBGLN(victim.bearing);
    return (victim.bearing < 0) ? -1 : 1;
  }
  
  int leftCost = 0, rightCost = 0;
  for (int i = 0; i < 4; i++) leftCost += obstacleMap.confidence[i];
  for (int i = 5; i < 9; i++) rightCost += obstacleMap.confidence[i];
  
  leftCost += obstacleMap.confidence[4] * 2;
  rightCost += obstacleMap.confidence[4] * 2;
  
  if (abs(leftCost - rightCost) < 50) {
    return nav.lastTurnDirection;
  }
  
  return (leftCost < rightCost) ? -1 : 1;
}

// ============================================================
//  SMALL HELPERS
// ============================================================

int gasDelta() {
  int delta = sGas - mq2Baseline;
  return (delta > 0) ? delta : 0;
}

int readMQ2StableRaw() {
  long sum = 0;
  analogRead(PIN_MQ2);
  delayMicroseconds(200);
  for (byte i = 0; i < MQ2_SAMPLES; i++) {
    sum += analogRead(PIN_MQ2);
    delay(2);
  }
  return sum / MQ2_SAMPLES;
}

const __FlashStringHelper *flameDirectionLabel() {
  int bestValue = sFlameA1;
  byte bestIndex = 0;
  if (sFlameA2 > bestValue) { bestValue = sFlameA2; bestIndex = 1; }
  if (sFlameA3 > bestValue) { bestValue = sFlameA3; bestIndex = 2; }
  
  if (bestValue <= FLAME_THRESHOLD) return F("NONE");
  switch (bestIndex) {
    case 0: return F("LEFT");
    case 1: return F("CENTER");
    default: return F("RIGHT");
  }
}

bool flameSensorArmed() { return (millis() - bootTime) >= FLAME_ARM_DELAY_MS; }

bool flameDetectedNow() {
  if (!flameSensorArmed()) return false;
  return (sFlameA1 > FLAME_THRESHOLD) || (sFlameA2 > FLAME_THRESHOLD) || (sFlameA3 > FLAME_THRESHOLD);
}

void setMode(const char *modeName) { currentMode = modeName; }

void saveMQ2Baseline() {
  struct MQ2CalibrationData { uint16_t magic; int baseline; };
  MQ2CalibrationData data = {MQ2_EEPROM_MAGIC, mq2Baseline};
  EEPROM.put(MQ2_EEPROM_ADDR, data);
}

bool loadMQ2Baseline() {
  struct MQ2CalibrationData { uint16_t magic; int baseline; };
  MQ2CalibrationData data;
  EEPROM.get(MQ2_EEPROM_ADDR, data);
  if (data.magic != MQ2_EEPROM_MAGIC) return false;
  if (data.baseline < 0 || data.baseline > 1023) return false;
  mq2Baseline = data.baseline;
  return true;
}

void clearMQ2BaselineStorage() {
  struct MQ2CalibrationData { uint16_t magic; int baseline; };
  MQ2CalibrationData data = {0, 0};
  EEPROM.put(MQ2_EEPROM_ADDR, data);
}

void emergencyReset() {
  sendCmd('S');
  autonomous = false;
  hazardActive = false;
  autoState = AUTO_PATROL;
  scanPhase = SCAN_IDLE;
  victim.detected = false;
  victim.confidence = 0;
  memset(&obstacleMap, 0, sizeof(obstacleMap));
  setMode("IDLE");
  DBGLN(F("EMERGENCY RESET: All states cleared"));
}

void enterHazard(const __FlashStringHelper *reason) {
  if (!hazardActive) {
    preHazardState = autoState;
    sendCmd('S');
    sendCmd('H');
  }
  hazardActive = true;
  autoState = AUTO_HAZARD;
  scanPhase = SCAN_IDLE;
  setMode("HAZARD");
  DBG(F("HAZARD: ")); DBGLN(reason);
}

void clearHazardState() {
  hazardActive = false;
  scanPhase = SCAN_IDLE;
  sendCmd('G');
  if (autonomous) {
    autoState = preHazardState == AUTO_HAZARD ? AUTO_PATROL : preHazardState;
    setMode("AUTO");
  } else {
    autoState = AUTO_PATROL;
    setMode("IDLE");
  }
}

// ============================================================
//  ENHANCED NEOPIXEL
// ============================================================

#if ENABLE_NEOPIXEL
uint32_t pxColor(uint8_t r, uint8_t g, uint8_t b) {
  if (nightMode) { r /= 4; g /= 4; b /= 4; }
  return statusPixels.Color(r, g, b);
}

void fillPixels(uint32_t color) {
  for (uint8_t i = 0; i < NEOPIXEL_COUNT; i++) statusPixels.setPixelColor(i, color);
}

void setPixelSafe(uint8_t idx, uint32_t color) {
  if (idx < NEOPIXEL_COUNT) statusPixels.setPixelColor(idx, color);
}

void fillRange(uint8_t start, uint8_t end, uint32_t color) {
  for (uint8_t i = start; i <= end && i < NEOPIXEL_COUNT; i++) 
    statusPixels.setPixelColor(i, color);
}

void setFrontArc(uint32_t color) { fillRange(0, 3, color); }
void setRearArc(uint32_t color) { fillRange(4, 7, color); }
void setLeftArc(uint32_t color) {
  setPixelSafe(0, color); setPixelSafe(1, color);
  setPixelSafe(6, color); setPixelSafe(7, color);
}
void setRightArc(uint32_t color) {
  setPixelSafe(2, color); setPixelSafe(3, color);
  setPixelSafe(4, color); setPixelSafe(5, color);
}

void renderNeoPixelStatus() {
  const unsigned long frameMs = 80;
  unsigned long now = millis();
  if (now - lastNeoPixelFrame < frameMs) return;
  lastNeoPixelFrame = now;
  neoPixelPhase++;
  
  statusPixels.clear();
  
  HazardAssessment hazard = calculateHazard();
  updateObstacleMap();
  
  uint32_t baseColor;
  if (hazard.critical) baseColor = pxColor(255, 0, 0);
  else if (hazard.overall > 70) baseColor = pxColor(255, 0, 100);
  else if (hazard.overall > 40) baseColor = pxColor(255, 100, 0);
  else if (victim.confidence > 60) baseColor = pxColor(0, 255, 100);
  else baseColor = pxColor(0, 50 + (victim.confidence/2), 100);
  
  if (hazardActive && hazard.critical) {
    if ((neoPixelPhase & 1) == 0) fillPixels(pxColor(255, 255, 255));
    else fillPixels(pxColor(255, 0, 0));
  }
  else if (autoState == AUTO_VICTIM_FOUND) {
    uint8_t hue = (neoPixelPhase * 8) % 255;
    for (int i = 0; i < NEOPIXEL_COUNT; i++) {
      uint8_t pixelHue = (hue + (i * 255 / NEOPIXEL_COUNT)) % 255;
      if (pixelHue < 85) setPixelSafe(i, pxColor(255 - pixelHue*3, pixelHue*3, 0));
      else if (pixelHue < 170) setPixelSafe(i, pxColor(0, 255 - (pixelHue-85)*3, (pixelHue-85)*3));
      else setPixelSafe(i, pxColor((pixelHue-170)*3, 0, 255 - (pixelHue-170)*3));
    }
  }
  else if (autoState == AUTO_INVESTIGATE || autoState == AUTO_CONFIRM_VICTIM) {
    uint8_t lead = (victim.bearing < 0) ? (7 - (neoPixelPhase % 4)) : (neoPixelPhase % 4);
    setPixelSafe(lead, pxColor(0, 100, 255));
    setPixelSafe((lead+1)%8, pxColor(0, 40, 100));
  }
  else if (autoState == AUTO_RETREAT_HAZARD) {
    uint8_t pulse = 50 + ((neoPixelPhase % 5) * 40);
    setRearArc(pxColor(pulse, pulse, 0));
    setFrontArc(pxColor(20, 20, 0));
  }
  else if (autoState == AUTO_SCANNING) {
    uint8_t lead = neoPixelPhase % NEOPIXEL_COUNT;
    for (uint8_t i = 0; i < NEOPIXEL_COUNT; i++) {
      uint8_t dist = (i + NEOPIXEL_COUNT - lead) % NEOPIXEL_COUNT;
      if (dist == 0) setPixelSafe(i, pxColor(0, 255, 100));
      else if (dist < 3) setPixelSafe(i, pxColor(0, 60, 30));
    }
  }
  else if (autonomous) {
    fillPixels(baseColor);
    
    if (obstacleMap.confidence[0] > 50 || obstacleMap.confidence[1] > 50) 
      setLeftArc(pxColor(255, 0, 0));
    if (obstacleMap.confidence[7] > 50 || obstacleMap.confidence[8] > 50) 
      setRightArc(pxColor(255, 0, 0));
    if (obstacleMap.confidence[4] > 50)
      setFrontArc(pxColor(255, 100, 0));
      
    if (autoState == AUTO_TURNING) {
      if (pendingTurnCmd == 'L') setLeftArc(pxColor(255, 255, 255));
      else setRightArc(pxColor(255, 255, 255));
    }
  }
  else {
    fillPixels(baseColor);
    
    uint8_t frontSegs = 0;
    if (sFrontDist < 120) frontSegs = 1;
    if (sFrontDist < DIST_CLEAR) frontSegs = 2;
    if (sFrontDist < DIST_WARN) frontSegs = 3;
    if (sFrontDist < DIST_STOP) frontSegs = 4;
    for (uint8_t i = 0; i < frontSegs; i++) 
      setPixelSafe(i, pxColor(255 - i*40, i*60, 0));
      
    uint8_t rearSegs = 0;
    if (sRearDist < 120) rearSegs = 1;
    if (sRearDist < DIST_CLEAR) rearSegs = 2;
    if (sRearDist < DIST_WARN) rearSegs = 3;
    if (sRearDist < DIST_STOP) rearSegs = 4;
    for (uint8_t i = 0; i < rearSegs; i++) 
      setPixelSafe(7-i, pxColor(255 - i*40, i*40, 50));
  }
  
  statusPixels.show();
}
#else
void renderNeoPixelStatus() {}
#endif

// ============================================================
//  I2C AND MOTOR CONTROL
// ============================================================

void recoverMotorI2cBus() {
#if ENABLE_MOTOR_I2C
  pinMode(SDA, INPUT_PULLUP);
  pinMode(SCL, INPUT_PULLUP);
  delayMicroseconds(5);
  pinMode(SCL, OUTPUT);
  for (byte i = 0; i < 9; i++) {
    digitalWrite(SCL, LOW);
    delayMicroseconds(5);
    digitalWrite(SCL, HIGH);
    delayMicroseconds(5);
  }
  pinMode(SCL, INPUT_PULLUP);
  delayMicroseconds(10);
  Wire.begin();
  Wire.setClock(MOTOR_I2C_CLOCK_HZ);
#endif
}

void trackRadarServoCommand(char cmd) {
  switch (cmd) {
    case '<': radarServoAngle = max(RADAR_SERVO_MIN, radarServoAngle - RADAR_SERVO_STEP); break;
    case '>': radarServoAngle = min(RADAR_SERVO_MAX, radarServoAngle + RADAR_SERVO_STEP); break;
    case 'C': radarServoAngle = RADAR_SERVO_CENTER; break;
  }
}

void sendCmd(char cmd) {
  trackRadarServoCommand(cmd);
#if ENABLE_MOTOR_I2C
  Wire.beginTransmission(I2C_ADDR);
  Wire.write(cmd);
  byte err = Wire.endTransmission();
  if (err != 0) {
    DBG(F("[I2C] FAIL sending '")); DBG(cmd); DBG(F("' err=")); DBGLN((int)err);
    if (err == 4) recoverMotorI2cBus();
  }
#else
  DBG(F("[BENCH] Motor cmd: ")); DBGLN(cmd);
#endif
}

void sendCmdSilent(char cmd) {
  trackRadarServoCommand(cmd);
#if ENABLE_MOTOR_I2C
  Wire.beginTransmission(I2C_ADDR);
  Wire.write(cmd);
  byte err = Wire.endTransmission();
  if (err == 4) recoverMotorI2cBus();
#else
  (void)cmd;
#endif
}

// ============================================================
//  SENSOR READING - FIXED TO RETURN FLOATS
// ============================================================

float readUltrasonicRaw(byte trigPin, byte echoPin) {
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);
  unsigned long dur = pulseIn(echoPin, HIGH, US_TIMEOUT_US);
  if (dur == 0) return 999.0f;
  return dur * 0.0343f * 0.5f;
}

float readUltrasonicSensor(byte trigPin, byte echoPin) {
  float best = 999.0f;
  for (byte i = 0; i < US_SAMPLES; i++) {
    float sample = readUltrasonicRaw(trigPin, echoPin);
    if (sample < best) best = sample;
    delay(5);
  }
  return best;
}

// FIXED: Now returns float instead of void
float readFrontUltrasonic() {
  sFrontDist = readUltrasonicSensor(PIN_US_FRONT_TRIG, PIN_US_FRONT_ECHO);
  return sFrontDist;
}

// FIXED: Now returns float instead of void
float readRearUltrasonic() {
#if ENABLE_REAR_US
  sRearDist = readUltrasonicSensor(PIN_US_REAR_TRIG, PIN_US_REAR_ECHO);
#else
  sRearDist = 999.0f;
#endif
  return sRearDist;
}

void readDHT() {
  unsigned long now = millis();
  if (now - lastDhtTime < DHT_INTERVAL_MS) return;
  float h = dht.readHumidity();
  float t = dht.readTemperature();
  if (!isnan(h)) sHumid = h;
  if (!isnan(t)) sTemp = t;
  lastDhtTime = now;
}

void readPIR() { sPIR1 = digitalRead(PIN_PIR1); sPIR2 = digitalRead(PIN_PIR2); }

void readFlame() {
#if ENABLE_FLAME_SENSORS
  sFlameA1 = analogRead(PIN_FLAME_LEFT);
  sFlameA2 = analogRead(PIN_FLAME_CENTER);
  sFlameA3 = analogRead(PIN_FLAME_RIGHT);
#else
  sFlameA1 = sFlameA2 = sFlameA3 = 1023;
#endif
}

void readGas() {
  sGasRaw = readMQ2StableRaw();
  if (sGas <= 0) { sGas = sGasRaw; return; }
  sGas = ((long)sGas * MQ2_FILTER_WEIGHT_OLD + (long)sGasRaw * MQ2_FILTER_WEIGHT_NEW) / 
         (MQ2_FILTER_WEIGHT_OLD + MQ2_FILTER_WEIGHT_NEW);
}

void readAllSensors() {
  readFrontUltrasonic();
  readRearUltrasonic();
  readDHT();
  readPIR();
  readFlame();
  readGas();
}

// ============================================================
//  ALERT EVALUATION
// ============================================================

void evaluateAlerts() {
  bool flameNow = flameDetectedNow();
  if (flameNow && !alertFlame) {
    alertFlame = true;
    DBGLN(F("!!! ALERT: FLAME DETECTED !!!"));
    DBG(F("DIR=")); DBGLN(flameDirectionLabel());
  } else if (!flameNow && alertFlame) {
    alertFlame = false;
    DBGLN(F("ALERT: Flame cleared"));
  }

  int gasWarnThreshold = alertGas ? (GAS_THRESHOLD - GAS_CLEAR_MARGIN) : GAS_THRESHOLD;
  if (gasWarnThreshold < 0) gasWarnThreshold = 0;
  bool gasNow = (gasDelta() > gasWarnThreshold);
  if (gasNow && !alertGas) {
    alertGas = true;
    DBG(F("!!! ALERT: GAS delta=")); DBGLN(gasDelta());
  } else if (!gasNow && alertGas) {
    alertGas = false;
    DBGLN(F("ALERT: Gas cleared"));
  }

  bool frontNow = (sFrontDist > 0.0f && sFrontDist < DIST_STOP);
  bool rearNow = (sRearDist > 0.0f && sRearDist < DIST_STOP);
  if (frontNow && !alertObstacleFront) {
    alertObstacleFront = true;
    DBG(F("!!! OBSTACLE FRONT ")); DBG(sFrontDist); DBGLN(F("cm"));
  } else if (!frontNow && alertObstacleFront) {
    alertObstacleFront = false;
  }
  if (rearNow && !alertObstacleRear) {
    alertObstacleRear = true;
    DBG(F("!!! OBSTACLE REAR ")); DBG(sRearDist); DBGLN(F("cm"));
  } else if (!rearNow && alertObstacleRear) {
    alertObstacleRear = false;
  }

  bool motionNow = (sPIR1 == HIGH || sPIR2 == HIGH);
  if (motionNow && !alertMotion) {
    alertMotion = true;
    DBG(F("!!! MOTION PIR1=")); DBG(sPIR1); DBG(F(" PIR2=")); DBGLN(sPIR2);
  } else if (!motionNow && alertMotion) {
    alertMotion = false;
  }

  bool tempNow = (sTemp > 0.0f && sTemp > TEMP_WARN);
  if (tempNow && !alertTemp) {
    alertTemp = true;
    DBG(F("!!! HIGH TEMP ")); DBG(sTemp); DBGLN(F("C"));
  } else if (!tempNow && alertTemp) {
    alertTemp = false;
  }
  
  updateVictimTracking();
  updateObstacleMap();
}

// ============================================================
//  AUTO SAFETY - ADDED MISSING FUNCTION
// ============================================================

void checkAutoSafety() {
  if (!autoSafety) return;
  
  HazardAssessment hazard = calculateHazard();
  
  if (hazard.critical) {
    if (hazard.gasRisk > hazard.fireRisk) {
      enterHazard(F("AUTO-SAFETY GAS"));
    } else {
      enterHazard(F("AUTO-SAFETY HEAT/FIRE"));
    }
    return;
  }

  if (hazardActive && !alertGas && sTemp <= TEMP_WARN && !alertFlame) {
    DBGLN(F("AUTO-SAFETY: Hazard cleared"));
    clearHazardState();
  }
}

// ============================================================
//  SMART AUTONOMOUS & SAR LOGIC
// ============================================================

void retreatFromHazard(HazardAssessment h) {
  setMode("HAZARD-RETREAT");
  sendCmdSilent('B');
  delay(AUTO_BACKUP_MS);
  
  int turnDir = calculateBestTurn();
  sendCmdSilent(turnDir < 0 ? 'L' : 'R');
  nav.lastTurnDirection = turnDir;
  delay(AUTO_TURN_MS);
  sendCmdSilent('S');
  
  autoState = AUTO_RETREAT_HAZARD;
  autoTimer = millis();
}

void approachVictim() {
  static unsigned long approachStart = 0;
  
  switch (autoState) {
    case AUTO_INVESTIGATE:
      if (approachStart == 0) {
        approachStart = millis();
        DBGLN(F("SAR: Approaching suspected victim"));
      }
      
      if (millis() - approachStart < INVESTIGATE_APPROACH_MS) {
        if (sFrontDist > DIST_VICTIM_APPROACH && sFrontDist < 999) {
          sendCmdSilent('1');
          sendCmdSilent('F');
          victim.approachDistance = sFrontDist;
        } else {
          sendCmdSilent('S');
          autoState = AUTO_CONFIRM_VICTIM;
          autoTimer = millis();
          approachStart = 0;
        }
      } else {
        sendCmdSilent('S');
        autoState = AUTO_CONFIRM_VICTIM;
        autoTimer = millis();
        approachStart = 0;
      }
      break;
      
    case AUTO_CONFIRM_VICTIM:
      if (millis() - autoTimer < VICTIM_CONFIRM_DELAY) {
        if (victim.confidence > 80) {
          victim.detected = true;
          autoState = AUTO_VICTIM_FOUND;
          DBGLN(F("SAR: VICTIM CONFIRMED"));
        }
      } else {
        DBGLN(F("SAR: False alarm, resuming patrol"));
        victim.confidence = 0;
        autoState = AUTO_PATROL;
        motionPauseUntil = millis() + 5000;
      }
      break;
      
    case AUTO_VICTIM_FOUND:
      sendCmdSilent('S');
      setMode("VICTIM-FOUND");
      if ((millis() % 2000) < 100) {
        DBGLN(F("SAR: VICTIM LOCATED - AWAITING EXTRACTION"));
      }
      break;
  }
}

void searchPattern() {
  if (millis() - searchPatternTimer > SEARCH_PATTERN_STEP) {
    searchPatternTimer = millis();
    searchPatternLeg = (searchPatternLeg + 1) % 4;
    DBGLN(F("SAR: Search pattern leg ")); DBG(searchPatternLeg);
  }
  
  if (searchPatternLeg % 2 == 0) {
    if (sFrontDist > DIST_WARN) {
      sendCmdSilent('F');
    } else {
      searchPatternTimer = 0;
    }
  } else {
    sendCmdSilent('R');
    delay(AUTO_TURN_MS);
    sendCmdSilent('S');
    delay(100);
  }
}

char decideBestTurn(float leftDist, float rightDist) {
  int fusionDir = calculateBestTurn();
  
  if (fusionDir == -1) return 'L';
  if (fusionDir == 1) return 'R';
  
  if (leftDist >= DIST_CLEAR && leftDist >= rightDist) return 'L';
  if (rightDist >= DIST_CLEAR && rightDist > leftDist) return 'R';
  if (leftDist == rightDist) return lastTurnLeft ? 'R' : 'L';
  return (leftDist > rightDist) ? 'L' : 'R';
}

void startScanSequence() {
  scanLeftDist = 999.0f;
  scanRightDist = 999.0f;
  scanPhase = SCAN_WAIT_LEFT;
  autoTimer = millis();
  sendCmd('<');
}

bool processScanSequence() {
  switch (scanPhase) {
    case SCAN_WAIT_LEFT:
      if (millis() - autoTimer >= AUTO_SCAN_PAUSE) {
        scanLeftDist = readFrontUltrasonic();  // Now works because function returns float
        sendCmd('>');
        autoTimer = millis();
        scanPhase = SCAN_WAIT_RIGHT;
      }
      return false;
    case SCAN_WAIT_RIGHT:
      if (millis() - autoTimer >= AUTO_SCAN_RIGHT_PAUSE) {
        scanRightDist = readFrontUltrasonic();  // Now works because function returns float
        sendCmd('C');
        autoTimer = millis();
        scanPhase = SCAN_WAIT_CENTER;
      }
      return false;
    case SCAN_WAIT_CENTER:
      if (millis() - autoTimer >= AUTO_CENTER_PAUSE) {
        pendingTurnCmd = decideBestTurn(scanLeftDist, scanRightDist);
        scanPhase = SCAN_IDLE;
        DBG(F("AUTO: L=")); DBG(scanLeftDist); DBG(F("cm R=")); DBG(scanRightDist); 
        DBG(F("cm -> Turn ")); DBGLN(pendingTurnCmd);
        return true;
      }
      return false;
    default: return false;
  }
}

void runAutonomous() {
  if (!autonomous) return;
  
  HazardAssessment hazard = calculateHazard();
  if (hazard.critical && !hazardActive) {
    enterHazard(hazard.gasRisk > hazard.fireRisk ? F("CRITICAL GAS") : F("CRITICAL FIRE/HEAT"));
    retreatFromHazard(hazard);
    return;
  }
  
  if (hazardActive) {
    if (!hazard.critical && hazard.overall < 20) {
      clearHazardState();
    } else {
      if (autoState == AUTO_RETREAT_HAZARD && millis() - autoTimer > 1000) {
        sendCmdSilent('S');
      }
      return;
    }
  }
  
  if (millis() - lastStuckCheck > STUCK_TIMEOUT_MS) {
    lastStuckCheck = millis();
    if (nav.stuckCounter > 3) {
      DBGLN(F("AUTO: Stuck detected, initiating search pattern"));
      autoState = AUTO_SEARCH_PATTERN;
      nav.stuckCounter = 0;
    }
  }
  
  switch (autoState) {
    case AUTO_PATROL:
      setMode("AUTO-PTRL");
      
      if (victim.confidence > 40 && !victim.detected) {
        sendCmdSilent('S');
        autoState = AUTO_INVESTIGATE;
        DBGLN(F("AUTO: Motion detected, investigating"));
        break;
      }
      
      if (sFrontDist > 0.0f && sFrontDist < DIST_STOP) {
        sendCmdSilent('S');
        autoState = AUTO_OBSTACLE_FOUND;
        nav.stuckCounter++;
        autoTimer = millis();
        break;
      }
      
      if (sFrontDist > 0.0f && sFrontDist < DIST_WARN) {
        setMode("AUTO-SLOW");
        sendCmd('1');
        sendCmdSilent('F');
      } else {
        if (millis() - nav.patrolStartTime > 20000 && victim.confidence == 0) {
          autoState = AUTO_SEARCH_PATTERN;
          searchPatternTimer = millis();
          nav.patrolStartTime = millis();
        } else {
          sendCmd((char)('0' + AUTO_PATROL_SPEED));
          sendCmdSilent('F');
        }
      }
      break;
      
    case AUTO_INVESTIGATE:
    case AUTO_CONFIRM_VICTIM:
    case AUTO_VICTIM_FOUND:
      approachVictim();
      break;
      
    case AUTO_SEARCH_PATTERN:
      setMode("AUTO-SEARCH");
      searchPattern();
      if (victim.confidence > 30) {
        autoState = AUTO_PATROL;
      }
      break;
      
    case AUTO_OBSTACLE_FOUND:
      setMode("AUTO-OBST");
      sendCmdSilent('B');
      autoTimer = millis();
      autoState = AUTO_BACKING;
      break;
      
    case AUTO_BACKING:
      setMode("AUTO-BACK");
      readRearUltrasonic();
      if (sRearDist > 0.0f && sRearDist < DIST_STOP) {
        sendCmdSilent('S');
        autoState = AUTO_SCANNING;
        scanPhase = SCAN_IDLE;
      } else if (millis() - autoTimer >= AUTO_BACKUP_MS) {
        sendCmdSilent('S');
        autoState = AUTO_SCANNING;
        scanPhase = SCAN_IDLE;
      }
      break;
      
    case AUTO_SCANNING:
      setMode("AUTO-SCAN");
      if (scanPhase == SCAN_IDLE) {
        startScanSequence();
      } else if (processScanSequence()) {
        sendCmdSilent(pendingTurnCmd);
        lastTurnLeft = (pendingTurnCmd == 'L');
        nav.lastTurnDirection = lastTurnLeft ? -1 : 1;
        autoTimer = millis();
        autoState = AUTO_TURNING;
      }
      break;
      
    case AUTO_TURNING:
      setMode("AUTO-TURN");
      if (millis() - autoTimer >= AUTO_TURN_MS) {
        sendCmdSilent('S');
        autoState = AUTO_PATROL;
        nav.patrolStartTime = millis();
      }
      break;
      
    case AUTO_RETREAT_HAZARD:
      if (millis() - autoTimer > 2000) {
        sendCmdSilent('S');
      }
      break;
      
    default:
      autoState = AUTO_PATROL;
      break;
  }
}

// ============================================================
//  TELEMETRY
// ============================================================

void printTelemetry() {
  HazardAssessment h = calculateHazard();
  
  DBG(F("[")); DBG(currentMode); DBG(F("] "));
  DBG(F("US:")); DBG((int)sFrontDist); DBG(F("/")); DBG((int)sRearDist); DBG(F("cm "));
  DBG(F("T:")); DBG((int)sTemp); DBG(F("C "));
  DBG(F("GAS:")); DBG(gasDelta()); DBG(F(" "));
  DBG(F("FL:")); DBG(flameDirectionLabel()); DBG(F(" "));
  DBG(F("VIC:")); DBG(victim.confidence); DBG(F("%/")); DBG(victim.detected ? F("YES") : F("NO")); DBG(F(" "));
  DBG(F("HAZ:")); DBG(h.overall); DBG(F("% "));
  DBG(F("ALRT:"));
  DBG(alertFlame ? F("F") : F("-"));
  DBG(alertGas ? F("G") : F("-"));
  DBG(alertMotion ? F("M") : F("-"));
  DBG(F(" ST:")); DBG((int)autoState);
  DBGLN(F(""));
}

void printStatus() {
  DBGLN(F("========== SAR ROBOT V3.0 STATUS =========="));
  DBG(F("Mode          : ")); DBGLN(currentMode);
  DBG(F("Autonomous    : ")); DBGLN(autonomous ? F("ON") : F("OFF"));
  DBG(F("Auto-Safety   : ")); DBGLN(autoSafety ? F("ON") : F("OFF"));
  DBG(F("Hazard        : ")); DBGLN(hazardActive ? F("ACTIVE") : F("CLEAR"));
  DBG(F("Night Mode    : ")); DBGLN(nightMode ? F("ON") : F("OFF"));
  
  DBGLN(F("--- FUSION DATA ---"));
  HazardAssessment h = calculateHazard();
  DBG(F("Fire Risk     : ")); DBG(h.fireRisk); DBGLN(F("%"));
  DBG(F("Gas Risk      : ")); DBG(h.gasRisk); DBGLN(F("%"));
  DBG(F("Overall Hazard: ")); DBG(h.overall); DBGLN(F("%"));
  DBG(F("Victim Conf   : ")); DBG(victim.confidence); DBGLN(F("%"));
  DBG(F("Victim Found  : ")); DBGLN(victim.detected ? F("YES") : F("NO"));
  if (victim.detected) {
    DBG(F("Victim Bearing: ")); DBG(victim.bearing); DBGLN(F("deg"));
  }
  
  DBGLN(F("--- SENSORS ---"));
  DBG(F("Front/Rear    : ")); DBG(sFrontDist); DBG(F(" / ")); DBGLN(sRearDist);
  DBG(F("Temp/Humid    : ")); DBG(sTemp); DBG(F("C / ")); DBG(sHumid); DBGLN(F("%"));
  DBG(F("Gas raw/delta : ")); DBG(sGasRaw); DBG(F(" / ")); DBGLN(gasDelta());
  DBG(F("Flame L/C/R   : ")); DBG(sFlameA1); DBG(F("/")); DBG(sFlameA2); DBG(F("/")); DBGLN(sFlameA3);
  
  DBGLN(F("--- OBSTACLE MAP ---"));
  for (int i = 0; i < OBSTACLE_MEMORY_SLOTS; i++) {
    DBG(F("Sector ")); DBG(i); DBG(F(": ")); DBG(obstacleMap.confidence[i]); DBG(F("% @ ")); 
    DBG((int)obstacleMap.distance[i]); DBG(F("cm  "));
    if (i % 3 == 2) DBGLN(F(""));
  }
  
  DBGLN(F("======================================"));
}

void printObstacleMap() {
  DBGLN(F("--- OBSTACLE MEMORY MAP ---"));
  const char* dirLabels[] = {"L90", "L70", "L50", "L30", "CTR", "R30", "R50", "R70", "R90"};
  for (int i = 0; i < OBSTACLE_MEMORY_SLOTS; i++) {
    DBG(dirLabels[i]); DBG(F(": "));
    int bars = obstacleMap.confidence[i] / 10;
    for (int b = 0; b < 10; b++) DBG(b < bars ? F("=") : F("-"));
    DBG(F(" ")); DBG((int)obstacleMap.distance[i]); DBGLN(F("cm"));
  }
  DBGLN(F("---------------------------"));
}

// ============================================================
//  CALIBRATION
// ============================================================

void calibrateMQ2() {
  DBGLN(F("MQ2 CAL: Keep in clean air..."));
  long sum = 0;
  for (byte i = 0; i < 20; i++) {
    for (byte j = 0; j < 10; j++) {
      delay(10);
      processInput(Serial, F("USB> "));
      processInput(BT, F("BT>  "));
    }
    sum += readMQ2StableRaw();
    DBG(F("."));
  }
  DBGNL();
  mq2Baseline = sum / 20;
  sGasRaw = mq2Baseline;
  sGas = mq2Baseline;
  saveMQ2Baseline();
  DBG(F("Baseline=")); DBGLN(mq2Baseline);
}

void resetThresholds() {
  mq2Baseline = 0;
  clearMQ2BaselineStorage();
  emergencyReset();
  DBGLN(F("RESET: Complete"));
}

// ============================================================
//  COMMAND HANDLER
// ============================================================

void handleMovementOverride() {
  if (autonomous) {
    autonomous = false;
    autoState = AUTO_PATROL;
    scanPhase = SCAN_IDLE;
    sendCmd('M');
    DBG(F(" (auto OFF)"));
  }
  DBGNL();
}

void setTelemetryStreaming(bool enabled) {
  telemActive = enabled;
  DBG(F("TELEM: ")); DBGLN(telemActive ? F("ON") : F("OFF"));
}

void handleCommand(char cmd) {
  if (cmd >= 'a' && cmd <= 'z') cmd = (char)(cmd - ('a' - 'A'));
  
  switch (cmd) {
    case '?':
      readAllSensors();
      evaluateAlerts();
      printStatus();
      break;
      
    case 'V':
      readAllSensors();
      evaluateAlerts();
      printTelemetry();
      break;
      
    case 'P':
      setTelemetryStreaming(!telemActive);
      break;
      
    case '[':
      setTelemetryStreaming(true);
      break;
      
    case ']':
      setTelemetryStreaming(false);
      break;
      
    case 'I':
      scanI2C();
      break;
      
    case 'T':
      DBGLN(F("--- SENSOR TEST ---"));
      readAllSensors();
      DBG(F("US FRONT: ")); DBG(sFrontDist); DBGLN(F(" cm"));
      DBG(F("US REAR : ")); DBG(sRearDist); DBGLN(F(" cm"));
      DBG(F("DHT11   : ")); DBG(sTemp); DBG(F("C ")); DBG(sHumid); DBGLN(F("%"));
      DBG(F("PIR1/2  : ")); DBG(sPIR1); DBG(F("/")); DBGLN(sPIR2);
      DBG(F("MQ2     : ")); DBG(sGasRaw); DBG(F("/")); DBG(sGas); DBG(F(" d=")); DBGLN(gasDelta());
      DBG(F("Flame   : ")); DBG(sFlameA1); DBG(F("/")); DBG(sFlameA2); DBG(F("/")); DBGLN(sFlameA3);
      DBGLN(F("-------------------"));
      break;
      
    case 'Z':
      autoSafety = !autoSafety;
      DBG(F("AUTO-SAFETY: ")); DBGLN(autoSafety ? F("ON") : F("OFF"));
      break;
      
    case 'K':
      calibrateMQ2();
      break;
      
    case '~':
      resetThresholds();
      break;
      
    case 'A':
      autonomous = !autonomous;
      if (autonomous) {
        autoState = AUTO_PATROL;
        scanPhase = SCAN_IDLE;
        nav.patrolStartTime = millis();
        sendCmd('A');
        setMode("AUTO");
        DBGLN(F("MODE: Autonomous ON"));
      } else {
        sendCmd('M');
        sendCmd('S');
        scanPhase = SCAN_IDLE;
        setMode("IDLE");
        DBGLN(F("MODE: Manual control"));
      }
      break;
      
    case 'E':
      hazardActive = !hazardActive;
      if (hazardActive) {
        scanPhase = SCAN_IDLE;
        sendCmd('S');
        sendCmd('H');
        setMode("HAZARD");
        DBGLN(F("HAZARD: Manually triggered"));
      } else {
        clearHazardState();
        DBGLN(F("HAZARD: Manually cleared"));
      }
      break;
      
    case 'F':
      if (!hazardActive) {
        lastCmd = 'F';
        setMode("FWD");
        sendCmd('F');
        DBG(F("CMD: FWD"));
        handleMovementOverride();
      } else {
        DBGLN(F("CMD blocked - hazard active"));
      }
      break;
      
    case 'B':
      if (!hazardActive) {
        lastCmd = 'B';
        setMode("BACK");
        sendCmd('B');
        DBG(F("CMD: BACK"));
        handleMovementOverride();
      } else {
        DBGLN(F("CMD blocked - hazard active"));
      }
      break;
      
    case 'L':
      if (!hazardActive) {
        lastCmd = 'L';
        setMode("LEFT");
        sendCmd('L');
        DBGLN(F("CMD: LEFT"));
        if (autonomous) {
          autonomous = false;
          autoState = AUTO_PATROL;
          sendCmd('M');
        }
      }
      break;
      
    case 'R':
      if (!hazardActive) {
        lastCmd = 'R';
        setMode("RGHT");
        sendCmd('R');
        DBGLN(F("CMD: RIGHT"));
        if (autonomous) {
          autonomous = false;
          autoState = AUTO_PATROL;
          sendCmd('M');
        }
      }
      break;
      
    case 'S':
      lastCmd = 'S';
      setMode("IDLE");
      if (autonomous) {
        autonomous = false;
        autoState = AUTO_PATROL;
        scanPhase = SCAN_IDLE;
        sendCmd('M');
      }
      sendCmd('S');
      DBGLN(F("CMD: STOP"));
      break;
      
    case '1':
    case '2':
    case '3':
      sendCmd(cmd);
      DBG(F("SPEED: ")); DBGLN(cmd);
      break;
      
    case '<':
    case '>':
    case 'C':
      sendCmd(cmd);
      DBG(F("SERVO: ")); DBGLN(cmd == '<' ? F("left") : cmd == '>' ? F("right") : F("center"));
      break;
      
    case 'X':
      emergencyReset();
      break;
      
    case 'N':
      nightMode = !nightMode;
      DBG(F("NIGHT MODE: ")); DBGLN(nightMode ? F("ON (dimmed)") : F("OFF"));
      break;
      
    case 'D':
      printStatus();
      break;
      
    case 'M':
      printObstacleMap();
      break;
      
    default:
      DBG(F("Unknown: '")); DBG(cmd); DBG(F("' 0x"));
      Serial.print((unsigned char)cmd, HEX);
      if (dbgMirrorBt()) BT.print((unsigned char)cmd, HEX);
      DBGLN(F(""));
      DBGLN(F("Commands: F/B/L/R/S 1/2/3 <>/C A E ? V P I T Z K ~ X N D M"));
      break;
  }
}

// ============================================================
//  INPUT HANDLING
// ============================================================

void processInput(Stream &port, const __FlashStringHelper *label) {
  static unsigned long lastCmdTimeUSB = 0;
  static unsigned long lastCmdTimeBT = 0;
  unsigned long *lastCmdTime = (&port == &Serial) ? &lastCmdTimeUSB : &lastCmdTimeBT;

  while (port.available() > 0) {
    int raw = port.peek();
    if (raw < 0) break;
    char c = (char)raw;
    
    if (c == '\r' || c == '\n' || c == ' ') {
      port.read();
      continue;
    }
    
    unsigned long now = millis();
    if (!isImmediateCommand(c) && *lastCmdTime != 0 && 
        (now - *lastCmdTime) < COMMAND_GAP_MS) {
      break;
    }
    
    port.read();
    *lastCmdTime = now;
    
    if (&port == &Serial) {
      usbConsoleSeen = true;
      btConsoleFallback = false;
    }
    
    bool fromBt = (&port == &BT);
    if (fromBt) btReplyMode = true;
    
    DBG(label);
    DBG(c);
    DBGNL();
    handleCommand(c);
    
    if (fromBt) btReplyMode = false;
  }
}

void updateConsoleRouting() {
  if (ENABLE_BT_DEBUG) return;
  if (!usbConsoleSeen && !btConsoleFallback && millis() - bootTime >= USB_FALLBACK_TO_BT_MS) {
    btConsoleFallback = true;
  }
  if (btConsoleFallback && !btFallbackAnnounced) {
    btFallbackAnnounced = true;
    BT.println(F("BT fallback active"));
  }
}

bool isImmediateCommand(char cmd) {
  return cmd == 'S' || cmd == 's' || cmd == 'E' || cmd == 'e' || 
         cmd == ']' || cmd == 'H' || cmd == 'G' || cmd == 'X' || cmd == 'x';
}

// ============================================================
//  I2C SCAN
// ============================================================

void scanI2C() {
#if !ENABLE_MOTOR_I2C
  DBGLN(F("--- I2C SCAN ---"));
  DBGLN(F("BENCH MODE: Motor I2C disabled"));
  return;
#endif
  DBGLN(F("--- I2C SCAN ---"));
  pinMode(SDA, INPUT_PULLUP);
  pinMode(SCL, INPUT_PULLUP);
  delayMicroseconds(10);
  DBG(F("Lines: SDA=")); DBG(digitalRead(SDA));
  DBG(F(" SCL=")); DBG(digitalRead(SCL)); DBGLN(F(""));
  
#if ENABLE_I2C_SCAN_DEBUG
  if (digitalRead(SDA) == LOW || digitalRead(SCL) == LOW) {
    DBGLN(F("Bus stuck, recovering..."));
    recoverMotorI2cBus();
    delay(20);
  }
#endif

  Wire.setClock(MOTOR_I2C_CLOCK_HZ);
  byte found = 0;
  for (byte addr = 1; addr < 127; addr++) {
    Wire.beginTransmission(addr);
    byte err = Wire.endTransmission();
    if (err == 0) {
      DBG(F("  ACK 0x")); printHexByte(addr); DBGNL();
      found++;
    } else if (addr == I2C_ADDR) {
      DBG(F("  Target 0x")); printHexByte(addr); DBG(F(" err=")); DBGLN((int)err);
    }
  }
  DBG(F("Found: ")); DBGLN((int)found);
  DBGLN(F("----------------"));
}

// ============================================================
//  SETUP
// ============================================================

void setup() {
  memset(&obstacleMap, 0, sizeof(obstacleMap));
  memset(&victim, 0, sizeof(victim));
  memset(&nav, 0, sizeof(nav));
  
  Serial.begin(9600);
  BT.begin(9600);
  bootTime = millis();
  delay(20);
  
  while (Serial.available() > 0) Serial.read();
  while (BT.available() > 0) BT.read();
  
#if ENABLE_MOTOR_I2C
  Wire.begin();
  Wire.setClock(MOTOR_I2C_CLOCK_HZ);
#endif

  dht.begin();
  
#if ENABLE_NEOPIXEL
  statusPixels.begin();
  statusPixels.clear();
  statusPixels.show();
#endif

  pinMode(PIN_US_FRONT_TRIG, OUTPUT);
  pinMode(PIN_US_FRONT_ECHO, INPUT);
#if ENABLE_REAR_US
  pinMode(PIN_US_REAR_TRIG, OUTPUT);
  pinMode(PIN_US_REAR_ECHO, INPUT);
#endif
  pinMode(PIN_PIR1, INPUT);
  pinMode(PIN_PIR2, INPUT);
  pinMode(PIN_MQ2, INPUT);

  DBGLN(F("MQ2 warming up (20s)..."));
  unsigned long start = millis();
  while (millis() - start < MQ2_WARMUP_MS) {
    for (byte k = 0; k < 50; k++) {
      delay(10);
      processInput(Serial, F("USB> "));
      processInput(BT, F("BT>  "));
      renderNeoPixelStatus();
    }
    DBG(F("."));
#if ENABLE_NEOPIXEL
    int progress = ((millis() - start) * 8) / MQ2_WARMUP_MS;
    statusPixels.clear();
    for (int i = 0; i < progress && i < 8; i++) {
      statusPixels.setPixelColor(i, statusPixels.Color(0, 0, 50));
    }
    statusPixels.show();
#endif
  }
  DBGNL();

  if (loadMQ2Baseline()) {
    DBG(F("MQ2 baseline loaded: ")); DBGLN(mq2Baseline);
  } else {
    DBGLN(F("MQ2 calibrating..."));
    calibrateMQ2();
  }

  DBGLN(F("============================================"));
  DBGLN(F("  SAR ROBOT v3.0 - SENSOR FUSION EDITION"));
  DBGLN(F("============================================"));
  DBGLN(F("  NEW: Victim detection + Obstacle memory"));
  DBGLN(F("  NEW: Strategic hazard retreat"));
  DBGLN(F("  NEW: Search pattern + Fusion lighting"));
  DBGLN(F("--------------------------------------------"));
  DBGLN(F("  MOVE : F B L R S  |  SPD: 1 2 3"));
  DBGLN(F("  SERVO: < > C      |  MODE: A E"));
  DBGLN(F("  DEBUG: ? V P I T  |  NEW: X N D M"));
  DBGLN(F("  CAL  : Z K ~"));
  DBGLN(F("--------------------------------------------"));
  DBGLN(F("  X=Emergency Reset | N=Night Mode"));
  DBGLN(F("  D=Dump Fusion     | M=Obstacle Map"));
  DBGLN(F("============================================"));

#if ENABLE_BOOT_I2C_SCAN
  scanI2C();
#else
  DBGLN(F("Send 'I' to scan I2C bus"));
#endif

  readAllSensors();
  evaluateAlerts();
  printTelemetry();
}

// ============================================================
//  MAIN LOOP
// ============================================================

void loop() {
  updateConsoleRouting();
  processInput(Serial, F("USB> "));
  processInput(BT, F("BT>  "));

  if (millis() - lastSensorTime >= SENSOR_INTERVAL_MS) {
    lastSensorTime = millis();
    readAllSensors();
    evaluateAlerts();
    checkAutoSafety();  // Now declared and defined above
  }

  if (autonomous && !hazardActive) {
    runAutonomous();
  }

  if (telemActive && millis() - lastTelemTime >= TELEM_INTERVAL_MS) {
    lastTelemTime = millis();
    printTelemetry();
  }

  renderNeoPixelStatus();
}
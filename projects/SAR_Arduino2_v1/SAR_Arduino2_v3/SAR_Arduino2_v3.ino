/**
 * ============================================================
 *  SAR DISASTER RESPONSE ROBOT - ARDUINO 2 (MASTER BRAIN)
 *  Version: 1.1 UNO-R3 STABLE
 * ============================================================
 *  Role    : I2C Master - Bluetooth control + sensors +
 *            autonomous decision making
 *
 *  ARDUINO UNO R3 NOTES:
 *    - Avoid heavy String usage to reduce RAM fragmentation.
 *    - DHT11 should not be polled too fast.
 *    - A4/A5 are reserved for I2C (SDA/SCL).
 *
 *  PIN MAP:
 *    HC-05 BT RX  : D2  (SoftwareSerial RX)
 *    HC-05 BT TX  : D3  (SoftwareSerial TX)
 *    NeoPixel x8  : D6
 *    DHT11        : D5
 *    Front US TRIG: D4
 *    PIR 1        : D7 (right side)
 *    PIR 2        : D8 (left side)
 *    Rear US TRIG : D9
 *    Rear US ECHO : D10
 *    Front US ECHO: D11
 *    IR Prox Left : D12
 *    IR Prox Right: D13
 *    Flame A1  left   : A0 
 *    Flame A3  middle : A1
 *    Flame A5 right : A2
 \\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\
 \     : A2
 *    MQ2          : A3
 *    I2C SDA      : A4
 *    I2C SCL      : A5
 *
 *  COMMAND SET:
 *    F/B/L/R/S    Movement
 *    1/2/3        Speed
 *    </>/C        Servo left/right/center
 *    A            Toggle autonomous
 *    E            Toggle hazard
 *    ?            Full status
 *    V            One telemetry line
 *    P            Toggle live telemetry
 *    I            Scan I2C bus
 *    T            Test sensors
 *    Z            Toggle auto safety
 *    K            Calibrate MQ2 baseline
 *    ~            Reset baseline/default state
 * ============================================================
 */

#include <SoftwareSerial.h>
#include <DHT.h>
#include <Wire.h>
#include <EEPROM.h>
#include <Adafruit_NeoPixel.h>
#include <stdlib.h>
#include <string.h>

// ============================================================
//  TUNING ZONE
// ============================================================

#define DIST_STOP             15
#define DIST_WARN             30
#define DIST_CLEAR            40
#define DIST_VICTIM_APPROACH  35
#define DIST_SAFE_RETREAT     50

#define GAS_THRESHOLD         400
#define GAS_DANGER            600
#define GAS_CLEAR_MARGIN      60

// Feature toggles: keep these 0/1 defines (used in #if across this sketch).
#define ENABLE_FLAME_SENSORS  1

#define ENABLE_REAR_US        1
#define ENABLE_MOTOR_I2C      1
#define ENABLE_BT_DEBUG       0
#define ENABLE_BOOT_I2C_SCAN  0
#define ENABLE_I2C_SCAN_DEBUG 0   // keep scan command, trim extra recovery debug for Uno flash
#define ENABLE_NEOPIXEL       1
#define ENABLE_VERBOSE_LOGS   0
#define ENABLE_FLAME_HAZARD   0
// This analog flame module is "reversed" in this build:
// low reading at idle, higher reading when flame is detected.
#define FLAME_THRESHOLD       120
#define FLAME_ARM_DELAY_MS    8000

#define TEMP_WARN             38
#define TEMP_DANGER           50

#define AUTO_BACKUP_MS        750
#define AUTO_TURN_MS          950
#define AUTO_SCAN_PAUSE       220
#define AUTO_SCAN_RIGHT_PAUSE 300
#define AUTO_CENTER_PAUSE     120
#define AUTO_SCAN_LEFT_ANGLE  40
#define AUTO_SCAN_RIGHT_ANGLE 140
#define AUTO_SCAN_STEP_DELAY  35
#define AUTO_PATROL_SPEED     2
#define AUTO_MOTION_PAUSE_MS  1200
#define AUTO_CONFIRM_WINDOW_MS 4500
#define AUTO_RECOVERY_MAX     4
#define AUTO_RECOVERY_WINDOW_MS 9000UL
#define AUTO_RECOVERY_CLEAR_MS 3500UL
#define AUTO_BACK_MIN_MS      320UL
#define AUTO_BACK_STEP_MS     180UL
#define AUTO_TURN_MIN_MS      420UL
#define AUTO_TURN_STEP_MS     180UL
#define AUTO_TURN_CLEAR_MARGIN 12
#define AUTO_MODE_TARGET_PWM  80
#define HAZARD_OBSERVE_MS     2500
#define INVESTIGATE_APPROACH_MS 1200
#define INVESTIGATE_HOLD_MS   1000
#define INVESTIGATE_ALIGN_MS  450
#define VICTIM_CONFIRM_DELAY  1400
#define VICTIM_TRACK_THRESHOLD 45
#define VICTIM_PENDING_THRESHOLD 60
#define VICTIM_CONFIRM_THRESHOLD 78
#define VICTIM_STALE_MS      2200UL
#define SEARCH_PATTERN_STEP_MS 2400
#define SEARCH_PATTERN_LEGS   4
#define SEARCH_MEMORY_BLOCK   70
#define PIR_HISTORY_DEPTH     4
#define OBSTACLE_MEMORY_SLOTS 9
#define OBSTACLE_DECAY_MS     8000
#define SENSOR_STALE_MS       4000
#define AUTO_PWM_MIN          60
#define AUTO_PWM_MAX          150

#define TELEM_INTERVAL_MS     500
#define SENSOR_INTERVAL_MS    200
#define FAST_OBSTACLE_SENSOR_INTERVAL_MS 25
#define DHT_INTERVAL_MS       2000
#define COMMAND_GAP_MS        20
#define USB_FALLBACK_TO_BT_MS 5000
#define US_TIMEOUT_US         18000UL
#define US_SAMPLES            3

#define MQ2_WARMUP_MS         20000
#define MQ2_EEPROM_MAGIC      0x4D51
#define MQ2_EEPROM_ADDR       0
#define MQ2_SAMPLES           8
#define MQ2_FILTER_WEIGHT_OLD 3
#define MQ2_FILTER_WEIGHT_NEW 1

#define RADAR_SERVO_MIN       0
#define RADAR_SERVO_CENTER    90
#define RADAR_SERVO_MAX       180
#define RADAR_SERVO_STEP      10

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
#define PIN_IR_PROX_LEFT      12
#define PIN_IR_PROX_RIGHT     13
#define PIN_FLAME_LEFT        A0
#define PIN_FLAME_CENTER      A1
#define PIN_FLAME_RIGHT       A2
#define PIN_MQ2               A3
#define I2C_ADDR              8
#define NEOPIXEL_COUNT        8
// Most digital IR proximity/comparator boards drive DO LOW when an obstacle is detected.
#define IR_PROX_ACTIVE_LOW    1
// 100 kHz is safest on a breadboard / long dupont leads between two Unos.
#define MOTOR_I2C_CLOCK_HZ    100000L

/*
 * I2C + USB/BT: If the firmware “freezes” when the motor Uno is plugged in, the
 * TWI bus is usually the cause (not the serial code). Checklist:
 *   - One common GND between both Arduinos (mandatory).
 *   - SDA -> SDA (A4–A4), SCL -> SCL (A5–A5); keep leads short.
 *   - Pull-ups on SDA+SCL (often 4.7k–10k to 5V on ONE board is enough).
 *   - Motor Uno powered and running the I2C slave sketch @ address 0x08.
 *   - Nothing else driving A4/A5. Bench-test the brain with ENABLE_MOTOR_I2C 0.
 */

// ============================================================
//  OBJECTS
// ============================================================

SoftwareSerial BT(PIN_BT_RX, PIN_BT_TX);
DHT dht(PIN_DHT, DHT11);
#if ENABLE_NEOPIXEL
Adafruit_NeoPixel statusPixels(NEOPIXEL_COUNT, PIN_NEOPIXEL, NEO_GRB + NEO_KHZ800);
#endif

// True while handling a command that arrived on Bluetooth — mirrors DBG to BT even
// if USB was used earlier (ENABLE_BT_DEBUG off + usbConsoleSeen would hide BT RX).
bool btReplyMode = false;
extern bool autonomous;
extern bool telemActive;

// ============================================================
//  DEBUG HELPERS
// ============================================================

void sendCmd(char cmd);
void sendCmdSilent(char cmd);
void recoverMotorI2cBus();
void updateConsoleRouting();
void processInput(Stream &port);
bool useBtDebug();
bool dbgMirrorBt();
bool isImmediateCommand(char cmd);
void setTelemetryStreaming(bool enabled);
void trackRadarServoCommand(char cmd);
extern bool btConsoleFallback;
void processLineCommand(const char *line);
void printConfigSummary();
void sendAutoPwmConfig();
const __FlashStringHelper *autoStateLabel();
void clearInvestigateMission(bool preserveConfirmedVictim);
void beginOperatorInvestigate(int bearingHint);
void markMotionVictimPending();
void emitVictimEvent(char eventType, char source);
bool victimTrackingActive();
bool victimReadyForApproval();
char oppositeTurn(char turnCmd);
bool searchForwardAllowed();
char preferredSearchTurn();
int calculateBestTurnFromMemory();
bool pathBlockedAhead();
void resetRecoveryState();
void noteRecoveryAttempt();
void updateRecoveryClearWindow();
unsigned long currentBackupLimitMs();
unsigned long currentTurnLimitMs();
bool frontExitClearEnough();
bool backupExitSatisfied();
bool turnExitSatisfied();

struct MQ2CalibrationData {
  uint16_t magic;
  int baseline;
};

bool useBtDebug() {
  return ENABLE_BT_DEBUG || btConsoleFallback;
}

bool dbgMirrorBt() {
  return btReplyMode || (useBtDebug() && !(telemActive && !autonomous));
}

void DBG(const __FlashStringHelper *s) {
  Serial.print(s);
  if (dbgMirrorBt()) {
    BT.print(s);
  }
}
void DBGLN(const __FlashStringHelper *s) {
  Serial.println(s);
  if (dbgMirrorBt()) {
    BT.println(s);
  }
}
void DBG(const char *s) {
  Serial.print(s);
  if (dbgMirrorBt()) {
    BT.print(s);
  }
}
void DBGLN(const char *s) {
  Serial.println(s);
  if (dbgMirrorBt()) {
    BT.println(s);
  }
}
void DBG(char c) {
  Serial.print(c);
  if (dbgMirrorBt()) {
    BT.print(c);
  }
}
void DBGLN(char c) {
  Serial.println(c);
  if (dbgMirrorBt()) {
    BT.println(c);
  }
}
void DBG(int v) {
  Serial.print(v);
  if (dbgMirrorBt()) {
    BT.print(v);
  }
}
void DBGLN(int v) {
  Serial.println(v);
  if (dbgMirrorBt()) {
    BT.println(v);
  }
}
void DBG(unsigned int v) {
  Serial.print(v);
  if (dbgMirrorBt()) {
    BT.print(v);
  }
}
void DBGLN(unsigned int v) {
  Serial.println(v);
  if (dbgMirrorBt()) {
    BT.println(v);
  }
}
void DBG(unsigned long v) {
  Serial.print(v);
  if (dbgMirrorBt()) {
    BT.print(v);
  }
}
void DBGLN(unsigned long v) {
  Serial.println(v);
  if (dbgMirrorBt()) {
    BT.println(v);
  }
}
void DBG(float v) {
  Serial.print(v);
  if (dbgMirrorBt()) {
    BT.print(v);
  }
}
void DBGLN(float v) {
  Serial.println(v);
  if (dbgMirrorBt()) {
    BT.println(v);
  }
}
void DBGNL() {
  Serial.println();
  if (dbgMirrorBt()) {
    BT.println();
  }
}

#if ENABLE_VERBOSE_LOGS
#define VDBG(x) DBG(x)
#define VDBGLN(x) DBGLN(x)
#define VDBGNL() DBGNL()
#else
#define VDBG(x) do { } while (0)
#define VDBGLN(x) do { } while (0)
#define VDBGNL() do { } while (0)
#endif

void printHexByte(byte value) {
  if (value < 16) {
    DBG('0');
  }
  Serial.print(value, HEX);
  if (dbgMirrorBt()) {
    BT.print(value, HEX);
  }
}

// ============================================================
//  SYSTEM STATE
// ============================================================

float sTemp = 0.0f;
float sHumid = 0.0f;
float sFrontDist = 999.0f;
float sRearDist = 999.0f;
bool sIrProxLeft = false;
bool sIrProxRight = false;
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
bool manualLock = true;
extern unsigned long lastMotionTime;

// Physical mounting note:
//   PIR1 is mounted on the robot's RIGHT side.
//   PIR2 is mounted on the robot's LEFT side.
bool pirLeftActive() {
  return sPIR2 == HIGH;
}

bool pirRightActive() {
  return sPIR1 == HIGH;
}

int pirDirectionalHint() {
  // PIR is intentionally treated as a weak, short-lived direction hint only.
  if ((millis() - lastMotionTime) > 1600UL) {
    return 0;
  }
  if (pirLeftActive() && !pirRightActive()) {
    return -1;
  }
  if (pirRightActive() && !pirLeftActive()) {
    return 1;
  }
  return 0;
}

struct ObstacleMap {
  uint8_t confidence[OBSTACLE_MEMORY_SLOTS];
  unsigned long timestamp[OBSTACLE_MEMORY_SLOTS];
  float distance[OBSTACLE_MEMORY_SLOTS];
};

struct VictimTracker {
  bool detected;
  bool confirmed;
  uint8_t confidence;
  int bearing;
  float approachDistance;
  unsigned long firstDetection;
  uint8_t pirHistory;
};

struct HazardAssessment {
  uint8_t fireRisk;
  uint8_t gasRisk;
  uint8_t thermalRisk;
  uint8_t overall;
  bool critical;
  int fireBearing;
};

HazardAssessment calculateHazard();
void startHazardRetreat(HazardAssessment hazard);

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
  AUTO_RETREAT_HAZARD,
  AUTO_OBSERVE_HAZARD,
  AUTO_SEARCH_PATTERN
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
float scanCenterDist = 999.0f;
int radarServoAngle = RADAR_SERVO_CENTER;
ObstacleMap obstacleMap;
VictimTracker victim;
HazardAssessment lastHazard = {0, 0, 0, 0, false, 0};
float previousFrontDist = 999.0f;
unsigned long lastFrontValidTime = 0;
unsigned long lastRearValidTime = 0;
unsigned long lastEnvValidTime = 0;
unsigned long lastMotionTime = 0;
unsigned long searchPatternTimer = 0;
uint8_t searchPatternLeg = 0;
int cfgDistStop = DIST_STOP;
int cfgDistWarn = DIST_WARN;
int cfgDistClear = DIST_CLEAR;
int cfgGasThreshold = GAS_THRESHOLD;
int cfgGasDanger = GAS_DANGER;
int cfgTempWarn = TEMP_WARN;
int cfgTempDanger = TEMP_DANGER;
int cfgAutoPwm = AUTO_MODE_TARGET_PWM;
uint16_t cfgAutoBackupMs = AUTO_BACKUP_MS;
uint16_t cfgAutoTurnMs = AUTO_TURN_MS;
uint16_t cfgAutoScanPauseMs = AUTO_SCAN_PAUSE;
uint16_t cfgAutoScanRightPauseMs = AUTO_SCAN_RIGHT_PAUSE;
uint16_t cfgAutoCenterPauseMs = AUTO_CENTER_PAUSE;
uint16_t cfgAutoMotionPauseMs = AUTO_MOTION_PAUSE_MS;
uint16_t cfgAutoBackMinMs = AUTO_BACK_MIN_MS;
uint16_t cfgAutoBackStepMs = AUTO_BACK_STEP_MS;
uint16_t cfgAutoTurnMinMs = AUTO_TURN_MIN_MS;
uint16_t cfgAutoTurnStepMs = AUTO_TURN_STEP_MS;
uint16_t cfgInvestigateApproachMs = INVESTIGATE_APPROACH_MS;
uint16_t cfgInvestigateHoldMs = INVESTIGATE_HOLD_MS;
uint16_t cfgInvestigateAlignMs = INVESTIGATE_ALIGN_MS;
bool operatorInvestigateActive = false;
int investigateBearingHint = 0;
unsigned long lastVisionInvestigateMs = 0;
bool victimApprovalPending = false;
bool motionEventArmed = true;
uint8_t recoveryTier = 0;
unsigned long lastRecoveryTime = 0;
unsigned long patrolClearSince = 0;
float turnEntryFrontDist = 999.0f;

unsigned long lastSensorTime = 0;
unsigned long lastFastObstacleTime = 0;
unsigned long lastTelemTime = 0;
unsigned long lastDhtTime = 0;
unsigned long autoTimer = 0;
unsigned long motionPauseUntil = 0;
unsigned long bootTime = 0;
unsigned long lastNeoPixelFrame = 0;

bool usbConsoleSeen = false;
bool btConsoleFallback = false;
bool btFallbackAnnounced = false;
byte neoPixelPhase = 0;

// ============================================================
//  SMALL HELPERS
// ============================================================

int gasDelta() {
  int delta = sGas - mq2Baseline;
  return (delta > 0) ? delta : 0;
}

bool frontSensorOffline() {
  return (millis() - lastFrontValidTime) > SENSOR_STALE_MS || sFrontDist >= 999.0f;
}

bool irProximityObstacleDetected() {
  return sIrProxLeft || sIrProxRight;
}

bool frontUltrasonicBlocked() {
  return sFrontDist > 0.0f && sFrontDist < cfgDistStop;
}

bool frontObstacleDetected() {
  return frontUltrasonicBlocked() || irProximityObstacleDetected();
}

bool rearSensorOffline() {
  return (millis() - lastRearValidTime) > SENSOR_STALE_MS || sRearDist >= 999.0f;
}

bool envSensorOffline() {
  return (millis() - lastEnvValidTime) > SENSOR_STALE_MS || sTemp <= 0.0f;
}

int sectorFromAngle(int angle) {
  int clamped = constrain(angle, RADAR_SERVO_MIN, RADAR_SERVO_MAX);
  long mapped = map(clamped, RADAR_SERVO_MIN, RADAR_SERVO_MAX, 0, OBSTACLE_MEMORY_SLOTS - 1);
  return constrain((int)mapped, 0, OBSTACLE_MEMORY_SLOTS - 1);
}

void clearVictimTracking() {
  victim.detected = false;
  victim.confirmed = false;
  victim.confidence = 0;
  victim.bearing = 0;
  victim.approachDistance = 999.0f;
  victim.firstDetection = 0;
  victim.pirHistory = 0;
  victimApprovalPending = false;
}

void clearInvestigateMission(bool preserveConfirmedVictim) {
  bool hadVictimState = victim.detected || victimApprovalPending || victim.confirmed;
  operatorInvestigateActive = false;
  investigateBearingHint = 0;
  victimApprovalPending = false;
  if (!preserveConfirmedVictim || !victim.confirmed) {
    clearVictimTracking();
    if (hadVictimState) {
      emitVictimEvent('X', 'S');
    }
  }
}

void beginOperatorInvestigate(int bearingHint) {
  if (manualLock) {
    return;
  }
  investigateBearingHint = constrain(bearingHint, -90, 90);
  operatorInvestigateActive = true;
  victimApprovalPending = false;
  victim.detected = true;
  victim.confirmed = false;
  if (victim.confidence < 90) {
    victim.confidence = 90;
  }
  victim.bearing = investigateBearingHint;
  victim.approachDistance = 999.0f;
  victim.firstDetection = millis();
  victim.pirHistory = 0;
  scanPhase = SCAN_IDLE;
  motionPauseUntil = 0;
  searchPatternTimer = millis();
  autoState = AUTO_INVESTIGATE;
  autoTimer = millis();
  if (!autonomous) {
    autonomous = true;
    sendCmd('A');
  }
  sendAutoPwmConfig();
  sendCmdSilent('S');
  setMode("AUTO-INVEST");
}

void markMotionVictimPending() {
  if ((millis() - lastVisionInvestigateMs) < 6000UL) {
    return;
  }
  if (!motionEventArmed && victimApprovalPending) {
    return;
  }
  victim.detected = true;
  victim.confirmed = false;
  victimApprovalPending = true;
  if (victim.confidence < VICTIM_PENDING_THRESHOLD) {
    victim.confidence = VICTIM_PENDING_THRESHOLD;
  }
  if (victim.firstDetection == 0) {
    victim.firstDetection = millis();
  }
  if (pirLeftActive() && !pirRightActive()) {
    victim.bearing = -45;
  } else if (pirRightActive() && !pirLeftActive()) {
    victim.bearing = 45;
  } else {
    victim.bearing = 0;
  }
  victim.approachDistance = 999.0f;
  autoTimer = millis();
  motionEventArmed = false;
  emitVictimEvent('P', 'M');
  // A PIR hit should create a pending investigate target without stalling patrol.
  setMode("AUTO-PTRL");
}

void emitVictimEvent(char eventType, char source) {
  DBG(F("EV:V"));
  DBG(eventType);
  DBG(F(" B="));
  DBG(victim.bearing);
  DBG(F(" C="));
  DBG((int)victim.confidence);
  DBG(F(" S="));
  DBGLN(source);
}

bool victimTrackingActive() {
  return victim.confidence >= VICTIM_TRACK_THRESHOLD;
}

bool victimReadyForApproval() {
  return victim.confidence >= VICTIM_PENDING_THRESHOLD;
}

char oppositeTurn(char turnCmd) {
  return turnCmd == 'L' ? 'R' : 'L';
}

bool searchForwardAllowed() {
  if (frontSensorOffline() || pathBlockedAhead()) {
    return false;
  }
  if (obstacleMap.confidence[4] >= SEARCH_MEMORY_BLOCK && sFrontDist < cfgDistClear) {
    return false;
  }
  return sFrontDist >= cfgDistWarn && sFrontDist < 999.0f;
}

char preferredSearchTurn() {
  int pirHint = pirDirectionalHint();
  char preferred;
  if (pirHint < 0) {
    preferred = 'L';
  } else if (pirHint > 0) {
    preferred = 'R';
  } else {
    preferred = calculateBestTurnFromMemory() < 0 ? 'L' : 'R';
  }
  if (searchPatternLeg >= 2) {
    preferred = oppositeTurn(preferred);
  }
  if (searchPatternLeg == 1 && preferred == (lastTurnLeft ? 'L' : 'R')) {
    preferred = oppositeTurn(preferred);
  }
  return preferred;
}

void resetRecoveryState() {
  recoveryTier = 0;
  lastRecoveryTime = 0;
  patrolClearSince = 0;
  turnEntryFrontDist = 999.0f;
}

void noteRecoveryAttempt() {
  unsigned long now = millis();
  if (lastRecoveryTime != 0 && (now - lastRecoveryTime) < AUTO_RECOVERY_WINDOW_MS) {
    if (recoveryTier < AUTO_RECOVERY_MAX) {
      recoveryTier++;
    }
  } else {
    recoveryTier = 0;
  }
  lastRecoveryTime = now;
  patrolClearSince = 0;
}

void updateRecoveryClearWindow() {
  if (hazardActive || victim.detected || alertMotion || frontSensorOffline() ||
      sFrontDist < cfgDistClear || irProximityObstacleDetected()) {
    patrolClearSince = 0;
    return;
  }

  if (patrolClearSince == 0) {
    patrolClearSince = millis();
  } else if ((millis() - patrolClearSince) >= AUTO_RECOVERY_CLEAR_MS) {
    recoveryTier = 0;
    lastRecoveryTime = 0;
  }
}

unsigned long currentBackupLimitMs() {
  return cfgAutoBackupMs + ((unsigned long) recoveryTier * cfgAutoBackStepMs);
}

unsigned long currentTurnLimitMs() {
  return cfgAutoTurnMs + ((unsigned long) recoveryTier * cfgAutoTurnStepMs);
}

bool frontExitClearEnough() {
  return !frontSensorOffline() &&
         !irProximityObstacleDetected() &&
         sFrontDist >= (cfgDistClear + AUTO_TURN_CLEAR_MARGIN);
}

bool backupExitSatisfied() {
  return (millis() - autoTimer) >= cfgAutoBackMinMs && frontExitClearEnough();
}

bool turnExitSatisfied() {
  if ((millis() - autoTimer) < cfgAutoTurnMinMs || !frontExitClearEnough()) {
    return false;
  }
  if (turnEntryFrontDist <= 0.0f || turnEntryFrontDist >= 999.0f) {
    return true;
  }
  return sFrontDist >= (turnEntryFrontDist + AUTO_TURN_CLEAR_MARGIN);
}

void resetObstacleMap() {
  memset(&obstacleMap, 0, sizeof(obstacleMap));
  for (byte i = 0; i < OBSTACLE_MEMORY_SLOTS; i++) {
    obstacleMap.distance[i] = 999.0f;
  }
}

const __FlashStringHelper *autoStateLabel() {
  switch (autoState) {
    case AUTO_PATROL: return F("PATROL");
    case AUTO_OBSTACLE_FOUND: return F("OBSTACLE");
    case AUTO_BACKING: return F("BACKING");
    case AUTO_SCANNING: return F("SCAN");
    case AUTO_TURNING: return F("TURN");
    case AUTO_HAZARD: return F("HAZARD");
    case AUTO_INVESTIGATE: return F("INVEST");
    case AUTO_CONFIRM_VICTIM: return F("CONFIRM");
    case AUTO_VICTIM_FOUND: return F("VICTIM");
    case AUTO_RETREAT_HAZARD: return F("RETREAT");
    case AUTO_OBSERVE_HAZARD: return F("OBSERVE");
    case AUTO_SEARCH_PATTERN: return F("SEARCH");
    default: return F("UNKNOWN");
  }
}

HazardAssessment calculateHazard() {
  HazardAssessment hazard;
  hazard.fireRisk = 0;
  hazard.gasRisk = 0;
  hazard.thermalRisk = 0;
  hazard.overall = 0;
  hazard.critical = false;
  hazard.fireBearing = 0;

#if ENABLE_FLAME_SENSORS
  int bestFlame = max(sFlameA1, max(sFlameA2, sFlameA3));
  if (bestFlame > FLAME_THRESHOLD) {
    hazard.fireRisk = constrain(map(bestFlame, FLAME_THRESHOLD, 1023, 20, 100), 0, 100);
    if (sFlameA1 >= sFlameA2 && sFlameA1 >= sFlameA3) {
      hazard.fireBearing = -1;
    } else if (sFlameA3 >= sFlameA1 && sFlameA3 >= sFlameA2) {
      hazard.fireBearing = 1;
    }
  }
#endif

  if (sTemp > cfgTempWarn) {
    hazard.thermalRisk = constrain(map((int)sTemp, cfgTempWarn, cfgTempDanger, 20, 100), 0, 100);
  }

  int delta = gasDelta();
  if (delta > cfgGasThreshold) {
    hazard.gasRisk = constrain(map(delta, cfgGasThreshold, 900, 20, 100), 0, 100);
  }

  hazard.overall = max(hazard.gasRisk, max(hazard.fireRisk, hazard.thermalRisk));
  if (hazard.fireRisk > 55 && hazard.gasRisk > 55) {
    hazard.overall = 100;
  }
  hazard.critical = (delta > cfgGasDanger) || (sTemp > cfgTempDanger) || (hazard.fireRisk > 80);
  return hazard;
}

void updateObstacleMap() {
  unsigned long now = millis();
  for (byte i = 0; i < OBSTACLE_MEMORY_SLOTS; i++) {
    if (obstacleMap.confidence[i] > 0 && (now - obstacleMap.timestamp[i]) > OBSTACLE_DECAY_MS) {
      obstacleMap.confidence[i] = (obstacleMap.confidence[i] > 8) ? obstacleMap.confidence[i] - 8 : 0;
      if (obstacleMap.confidence[i] == 0) {
        obstacleMap.distance[i] = 999.0f;
      }
      obstacleMap.timestamp[i] = now;
    }
  }

  if (sFrontDist > 0.0f && sFrontDist < 999.0f) {
    int sector = sectorFromAngle(radarServoAngle);
    uint8_t confidence = 30;
    if (sFrontDist < cfgDistStop) {
      confidence = 100;
    } else if (sFrontDist < cfgDistWarn) {
      confidence = 80;
    } else if (sFrontDist < cfgDistClear) {
      confidence = 55;
    }

    if (sFrontDist < cfgDistClear) {
      obstacleMap.confidence[sector] = max(obstacleMap.confidence[sector], confidence);
      obstacleMap.distance[sector] = sFrontDist;
      obstacleMap.timestamp[sector] = now;
    }
  }

  // PIR sensors are side-motion detectors, not obstacle sensors.
  // Keep them out of obstacle memory so patrol/turning stays driven by ultrasonic scans.

  if (sFrontDist < cfgDistWarn && abs((int)(sFrontDist - previousFrontDist)) < 2) {
    obstacleMap.confidence[4] = min(100, obstacleMap.confidence[4] + 4);
    obstacleMap.distance[4] = sFrontDist;
    obstacleMap.timestamp[4] = now;
  }
  previousFrontDist = sFrontDist;
}

void updateVictimTracking() {
  bool wasPending = victimApprovalPending;
  bool hadVictimState = victim.detected || victimApprovalPending || victim.confirmed;

  if (victim.confirmed) {
    victim.detected = true;
    victim.confidence = max((uint8_t)85, victim.confidence);
    return;
  }

  if (!operatorInvestigateActive && motionPauseUntil != 0 && millis() < motionPauseUntil) {
    clearVictimTracking();
    if (hadVictimState) {
      emitVictimEvent('X', 'D');
    }
    return;
  }

  bool motionNow = (sPIR1 == HIGH || sPIR2 == HIGH);
  if (motionNow) {
    lastMotionTime = millis();
  }

  victim.pirHistory = (victim.pirHistory << 1) | (motionNow ? 1 : 0);

  uint8_t hits = 0;
  for (byte i = 0; i < PIR_HISTORY_DEPTH; i++) {
    if (victim.pirHistory & (1 << i)) {
      hits++;
    }
  }

  HazardAssessment hazard = calculateHazard();
  bool suspiciousEnv = hazard.overall >= 25 || gasDelta() > (cfgGasThreshold / 2);
  uint8_t gain = 0;
  if (hits >= 3) {
    gain = suspiciousEnv ? 24 : 16;
  } else if (hits == 2) {
    gain = suspiciousEnv ? 16 : 10;
  } else if (hits == 1 && victimTrackingActive()) {
    gain = suspiciousEnv ? 6 : 3;
  }

  if (gain > 0) {
    victim.confidence = min(100, victim.confidence + gain);
    if (victim.firstDetection == 0) {
      victim.firstDetection = millis();
    }
    if (pirLeftActive() && !pirRightActive()) {
      victim.bearing = -45;
    } else if (pirRightActive() && !pirLeftActive()) {
      victim.bearing = 45;
    } else {
      victim.bearing = 0;
    }
  } else if (victim.confidence > 0) {
    uint8_t decay = ((millis() - lastMotionTime) > VICTIM_STALE_MS) ? 10 : 5;
    victim.confidence = (victim.confidence > decay) ? victim.confidence - decay : 0;
  }

  victim.detected = victimTrackingActive();
  if (!victim.detected) {
    victim.confirmed = false;
    victim.approachDistance = 999.0f;
    victimApprovalPending = false;
    if (wasPending || hadVictimState) {
      emitVictimEvent('X', 'T');
    }
  } else if (!operatorInvestigateActive && !victim.confirmed) {
    victimApprovalPending = victimReadyForApproval();
    if (victimApprovalPending && !wasPending) {
      autoTimer = millis();
      emitVictimEvent('P', motionNow ? 'M' : 'T');
    }
  }
}

int calculateBestTurnFromMemory() {
  int leftCost = 0;
  int rightCost = 0;
  for (byte i = 0; i < 4; i++) {
    leftCost += obstacleMap.confidence[i];
  }
  for (byte i = 5; i < OBSTACLE_MEMORY_SLOTS; i++) {
    rightCost += obstacleMap.confidence[i];
  }
  leftCost += obstacleMap.confidence[4] * 2;
  rightCost += obstacleMap.confidence[4] * 2;

  if (victim.detected && !victim.confirmed) {
    return victim.bearing <= 0 ? -1 : 1;
  }
  if (abs(leftCost - rightCost) < 25) {
    return lastTurnLeft ? 1 : -1;
  }
  return (leftCost < rightCost) ? -1 : 1;
}

int readMQ2StableRaw() {
  long sum = 0;

  // Throw away the first read after switching from the flame sensor channels.
  analogRead(PIN_MQ2);
  delayMicroseconds(200);

  for (byte i = 0; i < MQ2_SAMPLES; i++) {
    sum += analogRead(PIN_MQ2);
    delay(2);
  }

  return sum / MQ2_SAMPLES;
}

const __FlashStringHelper *flameDirectionLabel() {
#if ENABLE_FLAME_SENSORS
  int bestValue = sFlameA1;
  byte bestIndex = 0;

  if (sFlameA2 > bestValue) {
    bestValue = sFlameA2;
    bestIndex = 1;
  }
  if (sFlameA3 > bestValue) {
    bestValue = sFlameA3;
    bestIndex = 2;
  }

  if (bestValue <= FLAME_THRESHOLD) {
    return F("NONE");
  }

  switch (bestIndex) {
    case 0: return F("LEFT");
    case 1: return F("CENTER");
    default: return F("RIGHT");
  }
#else
  return F("OFF");
#endif
}

bool flameSensorArmed() {
  return (millis() - bootTime) >= FLAME_ARM_DELAY_MS;
}

bool flameDetectedNow() {
#if ENABLE_FLAME_SENSORS
  if (!flameSensorArmed()) {
    return false;
  }
  return (sFlameA1 > FLAME_THRESHOLD) ||
         (sFlameA2 > FLAME_THRESHOLD) ||
         (sFlameA3 > FLAME_THRESHOLD);
#else
  return false;
#endif
}

void setMode(const char *modeName) {
  currentMode = modeName;
}

void saveMQ2Baseline() {
  MQ2CalibrationData data;
  data.magic = MQ2_EEPROM_MAGIC;
  data.baseline = mq2Baseline;
  EEPROM.put(MQ2_EEPROM_ADDR, data);
}

bool loadMQ2Baseline() {
  MQ2CalibrationData data;
  EEPROM.get(MQ2_EEPROM_ADDR, data);
  if (data.magic != MQ2_EEPROM_MAGIC) {
    return false;
  }
  if (data.baseline < 0 || data.baseline > 1023) {
    return false;
  }
  mq2Baseline = data.baseline;
  return true;
}

void clearMQ2BaselineStorage() {
  MQ2CalibrationData data;
  data.magic = 0;
  data.baseline = 0;
  EEPROM.put(MQ2_EEPROM_ADDR, data);
}

void enterHazard(const __FlashStringHelper *reason) {
  if (!hazardActive) {
    preHazardState = autoState;
    sendCmd('S');
    sendCmd('H');
  }
  clearInvestigateMission(false);
  hazardActive = true;
  autoState = AUTO_HAZARD;
  scanPhase = SCAN_IDLE;
  setMode("HAZARD");
  if (reason != NULL) {
    VDBG(F("HAZARD: "));
    VDBGLN(reason);
  }
}

void clearHazardState() {
  hazardActive = false;
  scanPhase = SCAN_IDLE;
  patrolClearSince = 0;
  sendCmd('G');
  if (autonomous) {
    bool resumeInvestigate =
        operatorInvestigateActive &&
        (preHazardState == AUTO_INVESTIGATE ||
         preHazardState == AUTO_CONFIRM_VICTIM ||
         preHazardState == AUTO_VICTIM_FOUND);
    if (resumeInvestigate) {
      autoState = AUTO_INVESTIGATE;
      autoTimer = millis();
    } else {
      autoState = AUTO_PATROL;
    }
    setMode("AUTO");
  } else {
    setMode("IDLE");
  }
}

#if ENABLE_NEOPIXEL
uint32_t pxColor(uint8_t r, uint8_t g, uint8_t b) {
  return statusPixels.Color(r, g, b);
}

bool modeIs(const char *name) {
  return strcmp(currentMode, name) == 0;
}

void fillPixels(uint32_t color) {
  for (uint8_t i = 0; i < NEOPIXEL_COUNT; i++) {
    statusPixels.setPixelColor(i, color);
  }
}

void setPixelSafe(uint8_t idx, uint32_t color) {
  if (idx < NEOPIXEL_COUNT) {
    statusPixels.setPixelColor(idx, color);
  }
}

void fillRange(uint8_t start, uint8_t end, uint32_t color) {
  for (uint8_t i = start; i <= end && i < NEOPIXEL_COUNT; i++) {
    statusPixels.setPixelColor(i, color);
  }
}

void setFrontArc(uint32_t color) {
  fillRange(0, 3, color);
}

void setRearArc(uint32_t color) {
  fillRange(4, 7, color);
}

void setLeftArc(uint32_t color) {
  setPixelSafe(0, color);
  setPixelSafe(1, color);
  setPixelSafe(6, color);
  setPixelSafe(7, color);
}

void setRightArc(uint32_t color) {
  setPixelSafe(2, color);
  setPixelSafe(3, color);
  setPixelSafe(4, color);
  setPixelSafe(5, color);
}

void addFrontRearDistanceBars() {
  uint8_t frontCount = 0;
  uint8_t rearCount = 0;

  if (sFrontDist < 999.0f) {
    if (sFrontDist < DIST_STOP) {
      frontCount = 4;
    } else if (sFrontDist < DIST_WARN) {
      frontCount = 3;
    } else if (sFrontDist < DIST_CLEAR) {
      frontCount = 2;
    } else if (sFrontDist < 120.0f) {
      frontCount = 1;
    }
  }
  if (irProximityObstacleDetected()) {
    frontCount = 4;
  }

  if (sRearDist < 999.0f) {
    if (sRearDist < DIST_STOP) {
      rearCount = 4;
    } else if (sRearDist < DIST_WARN) {
      rearCount = 3;
    } else if (sRearDist < DIST_CLEAR) {
      rearCount = 2;
    } else if (sRearDist < 120.0f) {
      rearCount = 1;
    }
  }

  for (uint8_t i = 0; i < frontCount; i++) {
    setPixelSafe(i, pxColor(0, 0, 36));
  }
  for (uint8_t i = 0; i < rearCount; i++) {
    setPixelSafe(7 - i, pxColor(0, 18, 28));
  }
}

void renderNeoPixelStatus() {
  const unsigned long frameMs = 90;
  unsigned long now = millis();
  if (now - lastNeoPixelFrame < frameMs) {
    return;
  }
  lastNeoPixelFrame = now;
  neoPixelPhase++;

  statusPixels.clear();

  int gasRise = gasDelta();
  bool criticalFire = alertFlame;
  bool criticalGas = gasRise > GAS_DANGER;
  bool criticalHeat = sTemp > TEMP_DANGER;
  bool manualHazard = hazardActive && !criticalFire && !criticalGas && !criticalHeat;
  bool warningGas = alertGas;
  bool warningHeat = alertTemp;
  bool cautionFront = alertObstacleFront;
  bool cautionRear = alertObstacleRear;
  bool cautionMotion = alertMotion;
  bool autoScanning = autonomous && (modeIs("AUTO-SCAN") || modeIs("AUTO-MOTION"));
  bool autoTurning = autonomous && modeIs("AUTO-TURN");

  if (criticalFire) {
    // Fire: red/yellow flame flicker.
    for (uint8_t i = 0; i < NEOPIXEL_COUNT; i++) {
      uint8_t flicker = (byte)((neoPixelPhase * 23 + i * 41) & 0x3F);
      uint8_t red = 190 + (flicker >> 1);
      uint8_t green = 16 + (flicker >> 2);
      setPixelSafe(i, pxColor(red, green, 0));
    }
  } else if (criticalGas) {
    // Toxic gas danger: sickly green pulse with magenta warning core.
    uint8_t pulse = 40 + ((neoPixelPhase % 8) * 24);
    if (pulse > 120) {
      pulse = 240 - pulse;
    }
    fillPixels(pxColor(20 + (pulse / 4), 80 + pulse, 20 + (pulse / 5)));
    setPixelSafe((neoPixelPhase + 1) % NEOPIXEL_COUNT, pxColor(180, 0, 90));
    setPixelSafe((neoPixelPhase + 5) % NEOPIXEL_COUNT, pxColor(180, 0, 90));
  } else if (criticalHeat) {
    // Heat danger: orange-red heat wave expanding from the center/front.
    uint8_t hot = 90 + ((neoPixelPhase % 6) * 25);
    fillPixels(pxColor(hot, hot / 3, 0));
    setFrontArc(pxColor(255, 30 + (hot / 6), 0));
  } else if (manualHazard) {
    // Manual hazard / emergency hold: red-white strobe.
    if ((neoPixelPhase & 1) == 0) {
      fillPixels(pxColor(255, 0, 0));
    } else {
      fillPixels(pxColor(120, 120, 120));
    }
  } else if (warningGas && warningHeat) {
    // Combined env warning: alternating toxic green and hot amber.
    uint32_t a = pxColor(100, 160, 0);
    uint32_t b = pxColor(180, 70, 0);
    for (uint8_t i = 0; i < NEOPIXEL_COUNT; i++) {
      setPixelSafe(i, ((i + neoPixelPhase) & 1) ? a : b);
    }
  } else if (warningGas) {
    uint8_t pulse = 12 + ((neoPixelPhase % 10) * 10);
    if (pulse > 60) {
      pulse = 120 - pulse;
    }
    fillPixels(pxColor(0, 70 + pulse, 10));
  } else if (warningHeat) {
    uint8_t pulse = 18 + ((neoPixelPhase % 10) * 12);
    if (pulse > 72) {
      pulse = 144 - pulse;
    }
    fillPixels(pxColor(100 + pulse, 22 + (pulse / 2), 0));
  } else if (cautionFront || cautionRear) {
    // Directional obstacle warning.
    if (cautionFront) {
      setFrontArc((neoPixelPhase & 1) ? pxColor(255, 0, 0) : pxColor(80, 10, 0));
    }
    if (cautionRear) {
      setRearArc((neoPixelPhase & 1) ? pxColor(255, 70, 0) : pxColor(70, 10, 0));
    }
    if (cautionMotion) {
      setLeftArc(pxColor(80, 0, 80));
      setRightArc(pxColor(80, 0, 80));
    }
  } else if (cautionMotion) {
    // Motion only: purple ripple on side arcs.
    if ((neoPixelPhase & 1) == 0) {
      setLeftArc(pxColor(100, 0, 120));
    } else {
      setRightArc(pxColor(100, 0, 120));
    }
  } else if (autoScanning) {
    // Investigating / scanning: green radar sweep with cyan tail.
    uint8_t lead = neoPixelPhase % NEOPIXEL_COUNT;
    for (uint8_t i = 0; i < NEOPIXEL_COUNT; i++) {
      uint8_t distance = (i + NEOPIXEL_COUNT - lead) % NEOPIXEL_COUNT;
      if (distance == 0) {
        setPixelSafe(i, pxColor(0, 255, 80));
      } else if (distance == 1 || distance == 2) {
        setPixelSafe(i, pxColor(0, 70, 45));
      }
    }
  } else if (autoTurning) {
    // Turning decision: yellow sweep in the chosen direction.
    uint8_t lead = neoPixelPhase % 4;
    if (pendingTurnCmd == 'L') {
      setPixelSafe((7 - lead), pxColor(180, 130, 0));
      setPixelSafe((6 - lead + NEOPIXEL_COUNT) % NEOPIXEL_COUNT, pxColor(70, 35, 0));
    } else {
      setPixelSafe(lead, pxColor(180, 130, 0));
      setPixelSafe((lead + 1) % NEOPIXEL_COUNT, pxColor(70, 35, 0));
    }
  } else {
    addFrontRearDistanceBars();

    if (autonomous) {
      uint8_t lead = neoPixelPhase % NEOPIXEL_COUNT;
      for (uint8_t i = 0; i < NEOPIXEL_COUNT; i++) {
        uint8_t distance = (i + NEOPIXEL_COUNT - lead) % NEOPIXEL_COUNT;
        if (distance == 0) {
          setPixelSafe(i, pxColor(0, 170, 70));
        } else if (distance == 1 || distance == 2) {
          setPixelSafe(i, pxColor(0, 40, 16));
        }
      }
    } else if (modeIs("FWD")) {
      setFrontArc(pxColor(0, 120, 255));
      setRearArc(pxColor(0, 15, 30));
    } else if (modeIs("BACK")) {
      setRearArc(pxColor(0, 120, 255));
      setFrontArc(pxColor(0, 15, 30));
    } else if (modeIs("LEFT")) {
      setLeftArc(pxColor(0, 110, 220));
    } else if (modeIs("RGHT")) {
      setRightArc(pxColor(0, 110, 220));
    } else {
      uint8_t heartbeat = 10 + ((neoPixelPhase % 12) * 6);
      if (heartbeat > 42) {
        heartbeat = 94 - heartbeat;
      }
      fillPixels(pxColor(0, heartbeat, heartbeat + 10));
    }
  }

  statusPixels.show();
}
#else
void renderNeoPixelStatus() {
}
#endif

// ============================================================
//  I2C SEND TO ARDUINO 1
// ============================================================

void recoverMotorI2cBus() {
#if ENABLE_MOTOR_I2C
  // If SDA is stuck low, clock SCL up to 9 times so a slave can release the bus.
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

void sendAutoPwmConfig() {
#if ENABLE_MOTOR_I2C
  int clamped = constrain(cfgAutoPwm, AUTO_PWM_MIN, AUTO_PWM_MAX);
  Wire.beginTransmission(I2C_ADDR);
  Wire.write('#');
  Wire.write('A');
  if (clamped >= 100) {
    Wire.write((char)('0' + (clamped / 100)));
  }
  if (clamped >= 10) {
    Wire.write((char)('0' + ((clamped / 10) % 10)));
  }
  Wire.write((char)('0' + (clamped % 10)));
  Wire.write('\n');
  byte err = Wire.endTransmission();
  if (err != 0) {
    VDBG(F("[I2C] AUTO PWM cfg err="));
    VDBGLN((int)err);
    if (err == 4) {
      recoverMotorI2cBus();
    }
  }
#endif
}

void sendCmd(char cmd) {
  trackRadarServoCommand(cmd);
#if ENABLE_MOTOR_I2C
  Wire.beginTransmission(I2C_ADDR);
  Wire.write(cmd);
  byte err = Wire.endTransmission();
  if (err != 0) {
    VDBG(F("[I2C] FAIL sending '"));
    VDBG(cmd);
    VDBG(F("' err="));
    VDBGLN((int)err);
    if (err == 4) {
      recoverMotorI2cBus();
    }
  }
#else
  VDBG(F("[BENCH] Motor cmd skipped: "));
  VDBGLN(cmd);
#endif
}

void sendCmdSilent(char cmd) {
  trackRadarServoCommand(cmd);
#if ENABLE_MOTOR_I2C
  Wire.beginTransmission(I2C_ADDR);
  Wire.write(cmd);
  byte err = Wire.endTransmission();
  if (err == 4) {
    recoverMotorI2cBus();
  }
#else
  (void)cmd;
#endif
}

void trackRadarServoCommand(char cmd) {
  switch (cmd) {
    case '<':
      radarServoAngle = max(RADAR_SERVO_MIN, radarServoAngle - RADAR_SERVO_STEP);
      break;
    case '>':
      radarServoAngle = min(RADAR_SERVO_MAX, radarServoAngle + RADAR_SERVO_STEP);
      break;
    case 'C':
      radarServoAngle = RADAR_SERVO_CENTER;
      break;
    default:
      break;
  }
}

void moveRadarServoToAngle(int targetAngle) {
  targetAngle = constrain(targetAngle, RADAR_SERVO_MIN, RADAR_SERVO_MAX);
  while (radarServoAngle < targetAngle) {
    sendCmdSilent('>');
    delay(AUTO_SCAN_STEP_DELAY);
  }
  while (radarServoAngle > targetAngle) {
    sendCmdSilent('<');
    delay(AUTO_SCAN_STEP_DELAY);
  }
}

// ============================================================
//  SENSOR READS
// ============================================================

float readUltrasonicRaw(byte trigPin, byte echoPin) {
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);

  unsigned long dur = pulseIn(echoPin, HIGH, US_TIMEOUT_US);
  if (dur == 0) {
    return 999.0f;
  }
  return dur * 0.0343f * 0.5f;
}

float readUltrasonicSensor(byte trigPin, byte echoPin) {
  float samples[US_SAMPLES];
  byte validCount = 0;
  byte sampleCount = (telemActive && !autonomous) ? 2 : US_SAMPLES;
  for (byte i = 0; i < sampleCount; i++) {
    float sample = readUltrasonicRaw(trigPin, echoPin);
    if (sample > 1.0f && sample < 999.0f) {
      samples[validCount++] = sample;
    }
    delay(4);
  }
  if (validCount == 0) {
    return 999.0f;
  }
  if (validCount == 1) {
    return samples[0];
  }
  if (validCount == 2) {
    return (samples[0] + samples[1]) * 0.5f;
  }
  if (samples[0] > samples[1]) {
    float t = samples[0];
    samples[0] = samples[1];
    samples[1] = t;
  }
  if (samples[1] > samples[2]) {
    float t = samples[1];
    samples[1] = samples[2];
    samples[2] = t;
  }
  if (samples[0] > samples[1]) {
    float t = samples[0];
    samples[0] = samples[1];
    samples[1] = t;
  }
  return samples[1];
}

float readFrontUltrasonic() {
  sFrontDist = readUltrasonicSensor(PIN_US_FRONT_TRIG, PIN_US_FRONT_ECHO);
  if (sFrontDist > 0.0f && sFrontDist < 999.0f) {
    lastFrontValidTime = millis();
  }
  return sFrontDist;
}

float readFrontUltrasonicFast() {
  float sample = readUltrasonicRaw(PIN_US_FRONT_TRIG, PIN_US_FRONT_ECHO);
  if (sample > 1.0f && sample < 999.0f) {
    sFrontDist = sample;
    lastFrontValidTime = millis();
  } else {
    sFrontDist = 999.0f;
  }
  return sFrontDist;
}

float readFrontUltrasonicStable() {
  float a = readUltrasonicSensor(PIN_US_FRONT_TRIG, PIN_US_FRONT_ECHO);
  delay(4);
  float b = readUltrasonicSensor(PIN_US_FRONT_TRIG, PIN_US_FRONT_ECHO);
  float best = 999.0f;
  bool aValid = (a > 0.0f && a < 999.0f);
  bool bValid = (b > 0.0f && b < 999.0f);
  if (aValid && bValid) {
    best = (a + b) * 0.5f;
  } else if (aValid) {
    best = a;
  } else if (bValid) {
    best = b;
  }
  sFrontDist = best;
  if (best > 0.0f && best < 999.0f) {
    lastFrontValidTime = millis();
  }
  return best;
}

float readRearUltrasonic() {
#if ENABLE_REAR_US
  sRearDist = readUltrasonicSensor(PIN_US_REAR_TRIG, PIN_US_REAR_ECHO);
  if (sRearDist > 0.0f && sRearDist < 999.0f) {
    lastRearValidTime = millis();
  }
#else
  sRearDist = 999.0f;
#endif
  return sRearDist;
}

bool readIrProximityObstacle(byte pin) {
#if IR_PROX_ACTIVE_LOW
  return digitalRead(pin) == LOW;
#else
  return digitalRead(pin) == HIGH;
#endif
}

void readIrProximitySensors() {
  sIrProxLeft = readIrProximityObstacle(PIN_IR_PROX_LEFT);
  sIrProxRight = readIrProximityObstacle(PIN_IR_PROX_RIGHT);
}

void readDHT() {
  unsigned long now = millis();
  if (now - lastDhtTime < DHT_INTERVAL_MS) {
    return;
  }

  float h = dht.readHumidity();
  float t = dht.readTemperature();
  if (!isnan(h)) {
    sHumid = h;
    lastEnvValidTime = now;
  }
  if (!isnan(t)) {
    sTemp = t;
    lastEnvValidTime = now;
  }
  lastDhtTime = now;
}

void readPIR() {
  sPIR1 = digitalRead(PIN_PIR1);
  sPIR2 = digitalRead(PIN_PIR2);
}

void readFlame() {
#if ENABLE_FLAME_SENSORS
  sFlameA1 = analogRead(PIN_FLAME_LEFT);
  sFlameA2 = analogRead(PIN_FLAME_CENTER);
  sFlameA3 = analogRead(PIN_FLAME_RIGHT);
#else
  sFlameA1 = 0;
  sFlameA2 = 0;
  sFlameA3 = 0;
#endif
}

void readGas() {
  sGasRaw = readMQ2StableRaw();

  if (sGas <= 0) {
    sGas = sGasRaw;
    return;
  }

  sGas = ((long)sGas * MQ2_FILTER_WEIGHT_OLD + (long)sGasRaw * MQ2_FILTER_WEIGHT_NEW) /
         (MQ2_FILTER_WEIGHT_OLD + MQ2_FILTER_WEIGHT_NEW);
}

void readAllSensors() {
  readFrontUltrasonic();
  readRearUltrasonic();
  readIrProximitySensors();
  lastFastObstacleTime = millis();
  readDHT();
  readPIR();
  readFlame();
  readGas();
}

void readFastObstacleSensors() {
  readIrProximitySensors();

  unsigned long now = millis();
  if ((now - lastFastObstacleTime) < FAST_OBSTACLE_SENSOR_INTERVAL_MS) {
    return;
  }

  lastFastObstacleTime = now;
  readFrontUltrasonicFast();
}

// ============================================================
//  ALERT EVALUATION
// ============================================================

void evaluateAlerts() {
  bool flameNow = flameDetectedNow();
  lastHazard = calculateHazard();
  if (flameNow && !alertFlame) {
    alertFlame = true;
    VDBGLN(F("ALR FLAME"));
    VDBG(F("A1=")); VDBG(sFlameA1);
    VDBG(F(" A2=")); VDBG(sFlameA2);
    VDBG(F(" A3=")); VDBG(sFlameA3);
    VDBG(F(" DIR=")); VDBGLN(flameDirectionLabel());
  } else if (!flameNow && alertFlame) {
    alertFlame = false;
    VDBGLN(F("ALR FLAME CLR"));
  }

  int gasWarnThreshold = alertGas ? (cfgGasThreshold - GAS_CLEAR_MARGIN) : cfgGasThreshold;
  if (gasWarnThreshold < 0) {
    gasWarnThreshold = 0;
  }

  bool gasNow = (gasDelta() > gasWarnThreshold);
  if (gasNow && !alertGas) {
    alertGas = true;
    VDBG(F("ALR GAS "));
    VDBG(sGasRaw);
    VDBG(F("/"));
    VDBG(sGas);
    VDBG(F(" delta="));
    VDBGLN(gasDelta());
  } else if (!gasNow && alertGas) {
    alertGas = false;
    VDBGLN(F("ALR GAS CLR"));
  }

  bool frontNow = frontObstacleDetected();
  bool rearNow = (sRearDist > 0.0f && sRearDist < cfgDistStop);
  if (frontNow && !alertObstacleFront) {
    alertObstacleFront = true;
    if (frontUltrasonicBlocked()) {
      VDBG(F("ALR F "));
      VDBG(sFrontDist);
      VDBG(F("cm"));
      if (irProximityObstacleDetected()) {
        VDBG(F(" +IR "));
        VDBG(sIrProxLeft ? 1 : 0);
        VDBG(F("/"));
        VDBG(sIrProxRight ? 1 : 0);
      }
      VDBGNL();
    } else {
      VDBG(F("ALR F IR "));
      VDBG(sIrProxLeft ? 1 : 0);
      VDBG(F("/"));
      VDBGLN(sIrProxRight ? 1 : 0);
    }
  } else if (!frontNow && alertObstacleFront) {
    alertObstacleFront = false;
    VDBGLN(F("ALR F CLR"));
  }

  if (rearNow && !alertObstacleRear) {
    alertObstacleRear = true;
    VDBG(F("ALR R "));
    VDBG(sRearDist);
    VDBGLN(F("cm"));
  } else if (!rearNow && alertObstacleRear) {
    alertObstacleRear = false;
    VDBGLN(F("ALR R CLR"));
  }

  bool motionNow = (sPIR1 == HIGH || sPIR2 == HIGH);
  if (motionNow && !alertMotion) {
    alertMotion = true;
    VDBG(F("ALR M "));
    VDBG(sPIR1);
    VDBG(F(" PIR2="));
    VDBGLN(sPIR2);
  } else if (!motionNow && alertMotion) {
    alertMotion = false;
    motionEventArmed = true;
  }

  bool tempNow = (sTemp > 0.0f && sTemp > cfgTempWarn);
  if (tempNow && !alertTemp) {
    alertTemp = true;
    VDBG(F("ALR T "));
    VDBG(sTemp);
    VDBGLN(F("C"));
  } else if (!tempNow && alertTemp) {
    alertTemp = false;
    VDBGLN(F("ALR T CLR"));
  }

  updateObstacleMap();
  updateVictimTracking();
}

// ============================================================
//  AUTO SAFETY
// ============================================================

void checkAutoSafety() {
  if (!autoSafety) {
    return;
  }

  HazardAssessment hazard = calculateHazard();

  if (hazard.critical) {
    if (hazard.gasRisk >= hazard.fireRisk && hazard.gasRisk >= hazard.thermalRisk) {
      enterHazard(NULL);
    } else {
      enterHazard(NULL);
    }
    return;
  }

  if (hazardActive && !alertGas && !alertFlame && sTemp <= cfgTempWarn) {
    VDBGLN(F("SAFE CLR"));
    clearHazardState();
  }
}

// ============================================================
//  SMART AUTONOMOUS HELPERS
// ============================================================

char decideBestTurn(float leftDist, float centerDist, float rightDist) {
  // Immediate short-range IR proximity bias for corner protection.
  if (sIrProxLeft && !sIrProxRight) {
    return 'R';
  }
  if (sIrProxRight && !sIrProxLeft) {
    return 'L';
  }

  // Strong center blockage: prefer the clearer side first.
  if (centerDist < cfgDistStop) {
    if (leftDist >= cfgDistWarn && rightDist < cfgDistWarn) {
      return 'L';
    }
    if (rightDist >= cfgDistWarn && leftDist < cfgDistWarn) {
      return 'R';
    }
  }

  // PIR remains useful for search behavior, but it is too noisy to steer
  // obstacle recovery turns. Let the scan distances and obstacle memory win here
  // so the robot commits to escaping instead of wobbling in place.
  int memoryBias = calculateBestTurnFromMemory();
  if (memoryBias < 0 && leftDist >= cfgDistWarn) {
    return 'L';
  }
  if (memoryBias > 0 && rightDist >= cfgDistWarn) {
    return 'R';
  }
  if (centerDist >= cfgDistClear && centerDist >= leftDist && centerDist >= rightDist) {
    return lastTurnLeft ? 'R' : 'L';
  }
  if (leftDist >= cfgDistClear && leftDist >= rightDist) {
    return 'L';
  }
  if (rightDist >= cfgDistClear && rightDist > leftDist) {
    return 'R';
  }

  if (leftDist == rightDist) {
    if (lastTurnLeft) {
      return 'R';
    }
    return 'L';
  }

  return (leftDist > rightDist) ? 'L' : 'R';
}

bool pathBlockedAhead() {
  return frontObstacleDetected();
}

void enterSearchPattern() {
  autoState = AUTO_SEARCH_PATTERN;
  searchPatternTimer = millis();
  searchPatternLeg = 0;
  sendCmdSilent('S');
  setMode("AUTO-SEARCH");
}

void startHazardRetreat(HazardAssessment hazard) {
  int turnBias = calculateBestTurnFromMemory();
  autoState = AUTO_RETREAT_HAZARD;
  autoTimer = millis();
  setMode("HAZ-RETREAT");
  sendCmdSilent('B');
  if (hazard.fireBearing < 0) {
    pendingTurnCmd = 'R';
  } else if (hazard.fireBearing > 0) {
    pendingTurnCmd = 'L';
  } else {
    pendingTurnCmd = turnBias < 0 ? 'L' : 'R';
  }
}

void updateSearchPattern() {
  if (pathBlockedAhead()) {
    autoState = AUTO_OBSTACLE_FOUND;
    autoTimer = millis();
    return;
  }

  if ((millis() - searchPatternTimer) >= SEARCH_PATTERN_STEP_MS) {
    searchPatternLeg = (searchPatternLeg + 1) % SEARCH_PATTERN_LEGS;
    searchPatternTimer = millis();
  }

  if ((searchPatternLeg & 1) == 0 && searchForwardAllowed()) {
    sendCmdSilent('F');
    setMode("AUTO-SWEEP");
  } else {
    pendingTurnCmd = preferredSearchTurn();
    lastTurnLeft = (pendingTurnCmd == 'L');
    sendCmdSilent(pendingTurnCmd);
    setMode("AUTO-SEARCH");
  }
}

void updateVictimApproach() {
  switch (autoState) {
    case AUTO_INVESTIGATE: {
      setMode("AUTO-INVEST");
      if (pathBlockedAhead()) {
        sendCmdSilent('S');
        autoState = AUTO_OBSTACLE_FOUND;
        autoTimer = millis();
        break;
      }

      unsigned long investigateElapsed = millis() - autoTimer;
      if (investigateElapsed < cfgInvestigateHoldMs) {
        sendCmdSilent('S');
      } else if (operatorInvestigateActive && investigateElapsed < (cfgInvestigateHoldMs + cfgInvestigateAlignMs)) {
        if (investigateBearingHint < -5) {
          sendCmdSilent('L');
        } else if (investigateBearingHint > 5) {
          sendCmdSilent('R');
        } else {
          sendCmdSilent('S');
        }
      } else if (investigateElapsed < (cfgInvestigateHoldMs + cfgInvestigateAlignMs + cfgInvestigateApproachMs) &&
                 sFrontDist > DIST_VICTIM_APPROACH &&
                 !frontSensorOffline()) {
        sendCmdSilent('F');
        victim.approachDistance = sFrontDist;
      } else {
        sendCmdSilent('S');
        autoState = AUTO_CONFIRM_VICTIM;
        autoTimer = millis();
      }
      break;
    }

    case AUTO_CONFIRM_VICTIM:
      setMode("AUTO-VERIFY");
      sendCmdSilent('S');
      if ((millis() - autoTimer) >= VICTIM_CONFIRM_DELAY) {
        bool operatorConfirmed = operatorInvestigateActive;
        if (operatorConfirmed && victim.approachDistance >= 999.0f && !frontSensorOffline()) {
          victim.approachDistance = sFrontDist;
        }
        if ((operatorConfirmed && victim.approachDistance < 999.0f) ||
            (victim.confidence >= VICTIM_CONFIRM_THRESHOLD && victim.approachDistance < 999.0f)) {
          victim.confirmed = true;
          victim.detected = true;
          operatorInvestigateActive = false;
          investigateBearingHint = 0;
          victimApprovalPending = false;
          emitVictimEvent('C', operatorConfirmed ? 'O' : 'A');
          autoState = AUTO_VICTIM_FOUND;
          setMode("VICTIM");
          VDBGLN(F("VIC OK"));
        } else {
          clearInvestigateMission(false);
          autoState = AUTO_PATROL;
          motionPauseUntil = millis() + cfgAutoMotionPauseMs;
          VDBGLN(F("INV CLR"));
        }
      }
      break;

    case AUTO_VICTIM_FOUND:
      setMode("VICTIM");
      sendCmdSilent('S');
      break;

    default:
      break;
  }
}

void startScanSequence() {
  scanLeftDist = 999.0f;
  scanRightDist = 999.0f;
  scanCenterDist = sFrontDist;
  scanPhase = SCAN_WAIT_LEFT;
  moveRadarServoToAngle(AUTO_SCAN_LEFT_ANGLE);
  autoTimer = millis();
}

bool processScanSequence() {
  switch (scanPhase) {
    case SCAN_WAIT_LEFT:
      if (millis() - autoTimer >= cfgAutoScanPauseMs) {
        scanLeftDist = readFrontUltrasonicStable();
        updateObstacleMap();
        moveRadarServoToAngle(AUTO_SCAN_RIGHT_ANGLE);
        autoTimer = millis();
        scanPhase = SCAN_WAIT_RIGHT;
      }
      return false;

    case SCAN_WAIT_RIGHT:
      if (millis() - autoTimer >= cfgAutoScanRightPauseMs) {
        scanRightDist = readFrontUltrasonicStable();
        updateObstacleMap();
        moveRadarServoToAngle(RADAR_SERVO_CENTER);
        autoTimer = millis();
        scanPhase = SCAN_WAIT_CENTER;
      }
      return false;

    case SCAN_WAIT_CENTER:
      if (millis() - autoTimer >= cfgAutoCenterPauseMs) {
        scanCenterDist = readFrontUltrasonicStable();
        updateObstacleMap();
        pendingTurnCmd = decideBestTurn(scanLeftDist, scanCenterDist, scanRightDist);
        scanPhase = SCAN_IDLE;
        VDBG(F("SC "));
        VDBG(scanLeftDist);
        VDBG(F("/"));
        VDBG(scanCenterDist);
        VDBG(F("/"));
        VDBG(scanRightDist);
        VDBGLN(F("cm"));
        return true;
      }
      return false;

    default:
      return false;
  }
}

// ============================================================
//  AUTONOMOUS MODE
// ============================================================

void runAutonomous() {
  if (!autonomous) {
    return;
  }

  HazardAssessment hazard = calculateHazard();
  lastHazard = hazard;

  if (hazard.critical && !hazardActive) {
    enterHazard(NULL);
    startHazardRetreat(hazard);
    return;
  }

  switch (autoState) {
    case AUTO_PATROL:
      setMode("AUTO-PTRL");
      updateRecoveryClearWindow();

      if (pathBlockedAhead()) {
        sendCmdSilent('S');
        autoState = AUTO_OBSTACLE_FOUND;
        autoTimer = millis();
        VDBG(F("AUTO OBS "));
        VDBG(sFrontDist);
        VDBGLN(F("cm"));
        break;
      }

      if (frontSensorOffline()) {
        enterSearchPattern();
        break;
      }

      // PIR remains assistive only (directional hint in turn/search decisions),
      // not a direct objective trigger.

      if (victim.detected && !victim.confirmed) {
        if (operatorInvestigateActive) {
          sendCmdSilent('S');
          victimApprovalPending = false;
          autoState = AUTO_INVESTIGATE;
          autoTimer = millis();
          break;
        }
        if (victimApprovalPending && (millis() - autoTimer) >= AUTO_CONFIRM_WINDOW_MS) {
          VDBGLN(F("VIC AUTO"));
          beginOperatorInvestigate(victim.bearing);
          break;
        }
      }

      if (sFrontDist > 0.0f && sFrontDist < cfgDistWarn) {
        setMode(victimApprovalPending ? "AUTO-PTRL" : "AUTO-SLOW");
        sendCmdSilent('1');
        sendCmdSilent('F');
      } else {
        if (victimApprovalPending) {
          setMode("AUTO-PTRL");
        }
        sendCmd((char)('0' + AUTO_PATROL_SPEED));
        sendCmdSilent('F');
      }

      if ((millis() - searchPatternTimer) > (SEARCH_PATTERN_STEP_MS * 2UL)
          && !victim.detected
          && !operatorInvestigateActive
          && !victimApprovalPending) {
        enterSearchPattern();
      }
      break;

    case AUTO_OBSTACLE_FOUND:
      noteRecoveryAttempt();
      setMode("AUTO-OBST");
      sendCmdSilent('B');
      autoTimer = millis();
      autoState = AUTO_BACKING;
      VDBGLN(F("AUTO BACK"));
      break;

    case AUTO_BACKING:
      setMode(recoveryTier >= 2 ? "AUTO-RECOV" : "AUTO-BACK");
      readRearUltrasonic();
      if (sRearDist > 0.0f && sRearDist < cfgDistStop) {
        sendCmdSilent('S');
        autoState = AUTO_SCANNING;
        scanPhase = SCAN_IDLE;
        VDBGLN(F("AUTO REAR"));
      } else if (backupExitSatisfied()) {
        sendCmdSilent('S');
        autoState = AUTO_SCANNING;
        scanPhase = SCAN_IDLE;
        VDBGLN(F("AUTO CLR"));
      } else if (millis() - autoTimer >= currentBackupLimitMs()) {
        sendCmdSilent('S');
        autoState = AUTO_SCANNING;
        scanPhase = SCAN_IDLE;
        VDBGLN(F("AUTO SCAN"));
      }
      break;

    case AUTO_SCANNING:
      if (motionPauseUntil != 0 && millis() < motionPauseUntil) {
        break;
      }
      motionPauseUntil = 0;
      setMode("AUTO-SCAN");
      if (scanPhase == SCAN_IDLE) {
        startScanSequence();
      } else if (processScanSequence()) {
        // If all sampled directions are tight, force a turn flip + re-scan cadence.
        if (scanLeftDist < cfgDistWarn && scanCenterDist < cfgDistWarn && scanRightDist < cfgDistWarn) {
          pendingTurnCmd = oppositeTurn(lastTurnLeft ? 'L' : 'R');
        }
        if (recoveryTier >= 2 && abs((int) (scanLeftDist - scanRightDist)) <= AUTO_TURN_CLEAR_MARGIN) {
          pendingTurnCmd = lastTurnLeft ? 'R' : 'L';
        }
        sendCmdSilent(pendingTurnCmd);
        lastTurnLeft = (pendingTurnCmd == 'L');
        turnEntryFrontDist = scanCenterDist;
        autoTimer = millis();
        autoState = AUTO_TURNING;
        VDBG(F("AUTO TURN "));
        VDBGLN(pendingTurnCmd == 'L' ? F("LEFT") : F("RIGHT"));
      }
      break;

    case AUTO_TURNING:
      setMode(recoveryTier >= 2 ? "AUTO-RECOV" : "AUTO-TURN");
      if (turnExitSatisfied()) {
        sendCmdSilent('S');
        autoState = AUTO_PATROL;
        patrolClearSince = millis();
        VDBGLN(F("AUTO TURN CLR"));
      } else if (millis() - autoTimer >= currentTurnLimitMs()) {
        sendCmdSilent('S');
        if (operatorInvestigateActive && !victim.confirmed) {
          autoState = AUTO_INVESTIGATE;
          autoTimer = millis();
          VDBGLN(F("AUTO RES INV"));
        } else if (pathBlockedAhead()) {
          if (recoveryTier >= AUTO_RECOVERY_MAX) {
            enterSearchPattern();
            VDBGLN(F("AUTO ESC"));
          } else {
            autoState = AUTO_SCANNING;
            scanPhase = SCAN_IDLE;
            autoTimer = millis();
            VDBGLN(F("AUTO RESCAN"));
          }
        } else if (recoveryTier >= AUTO_RECOVERY_MAX) {
          enterSearchPattern();
          VDBGLN(F("AUTO ESC"));
        } else {
          autoState = AUTO_PATROL;
          VDBGLN(F("AUTO RES PTRL"));
        }
      }
      break;

    case AUTO_INVESTIGATE:
    case AUTO_CONFIRM_VICTIM:
    case AUTO_VICTIM_FOUND:
      updateVictimApproach();
      break;

    case AUTO_RETREAT_HAZARD:
      setMode("HAZ-RETREAT");
      if ((millis() - autoTimer) >= cfgAutoBackupMs) {
        sendCmdSilent(pendingTurnCmd);
        autoTimer = millis();
        autoState = AUTO_OBSERVE_HAZARD;
      }
      break;

    case AUTO_OBSERVE_HAZARD:
      setMode("HAZ-OBSERVE");
      if ((millis() - autoTimer) < cfgAutoTurnMs) {
        sendCmdSilent(pendingTurnCmd);
      } else {
        sendCmdSilent('S');
        if (!hazard.critical && hazard.overall < 30) {
          clearHazardState();
          autoState = AUTO_PATROL;
        }
      }
      break;

    case AUTO_SEARCH_PATTERN:
      updateSearchPattern();
      if (victim.detected || alertMotion) {
        autoState = AUTO_PATROL;
      }
      break;

    case AUTO_HAZARD:
      if (!hazard.critical && hazard.overall < 25) {
        clearHazardState();
        autoState = AUTO_PATROL;
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
  bool previousBtReplyMode = btReplyMode;
  if (telemActive && !autonomous) {
    btReplyMode = true;
  }
  DBG(F("[")); DBG(currentMode); DBG(F("] "));
  DBG(F("USF:")); DBG(sFrontDist); DBG(F("cm "));
  DBG(F("USR:")); DBG(sRearDist);  DBG(F("cm "));
  DBG(F("IR:")); DBG(sIrProxLeft ? 1 : 0); DBG(F("/")); DBG(sIrProxRight ? 1 : 0); DBG(F(" "));
  DBG(F("T:"));   DBG(sTemp);       DBG(F("C "));
  DBG(F("H:"));   DBG(sHumid);      DBG(F("% "));
  DBG(F("GAS:")); DBG(sGasRaw);     DBG(F("/")); DBG(sGas); DBG(F(" dG:")); DBG(gasDelta()); DBG(F(" "));
  DBG(F("FL:"));  DBG(sFlameA1);   DBG(F("/")); DBG(sFlameA2); DBG(F("/")); DBG(sFlameA3); DBG(F(" "));
  DBG(F("FLDIR:")); DBG(flameDirectionLabel()); DBG(F(" "));
  DBG(F("SRV:")); DBG(radarServoAngle); DBG(F(" "));
  DBG(F("PIR:")); DBG(sPIR1);      DBG(F("/")); DBG(sPIR2); DBG(F(" "));
  DBG(F("ALRT:"));
  DBG(alertFlame ? F("FLM ") : F("--- "));
  DBG(alertGas ? F("GAS ") : F("--- "));
  DBG(alertObstacleFront ? F("F-OBS ") : F("------ "));
  DBG(alertObstacleRear ? F("R-OBS ") : F("------ "));
  DBG(alertMotion ? F("MOT ") : F("--- "));
  DBG(alertTemp ? F("TMP ") : F("--- "));
  DBG(F("AUT:")); DBG(autonomous ? F("ON") : F("OFF")); DBG(F(" "));
  DBG(F("HAZ:")); DBG(hazardActive ? F("YES") : F("NO"));
  DBG(F(" AST:")); DBG(autoStateLabel());
  DBG(F(" VIC:")); DBG((int)victim.confidence); DBG(F("/")); DBG(victim.confirmed ? F("YES") : F("NO"));
  DBG(F(" VBR:")); DBG(victim.bearing);
  DBG(F(" VP:")); DBG(victimApprovalPending ? F("YES") : F("NO"));
  DBG(F(" HR:")); DBG((int)lastHazard.overall);
  DBG(F(" GR:")); DBG((int)lastHazard.gasRisk);
  DBG(F(" FR:")); DBG((int)lastHazard.fireRisk);
  DBG(F(" APWM:")); DBG(cfgAutoPwm);
  DBG(F(" OFF:")); DBG(frontSensorOffline() ? F("F") : F("-")); DBG(rearSensorOffline() ? F("R") : F("-")); DBG(envSensorOffline() ? F("E") : F("-"));
  DBGLN(F(""));
  btReplyMode = previousBtReplyMode;
}

void printConfigSummary() {
  DBG(F("CFG DS=")); DBG(cfgDistStop);
  DBG(F(" DW=")); DBG(cfgDistWarn);
  DBG(F(" DC=")); DBG(cfgDistClear);
  DBG(F(" GW=")); DBG(cfgGasThreshold);
  DBG(F(" GD=")); DBG(cfgGasDanger);
  DBG(F(" TW=")); DBG(cfgTempWarn);
  DBG(F(" TD=")); DBG(cfgTempDanger);
  DBG(F(" AP=")); DBGLN(cfgAutoPwm);
  DBG(F("TIME BK=")); DBG(cfgAutoBackupMs);
  DBG(F(" TR=")); DBG(cfgAutoTurnMs);
  DBG(F(" BMIN=")); DBG(cfgAutoBackMinMs);
  DBG(F(" TMIN=")); DBG(cfgAutoTurnMinMs);
  DBG(F(" BSTP=")); DBG(cfgAutoBackStepMs);
  DBG(F(" TSTP=")); DBG(cfgAutoTurnStepMs);
  DBG(F(" SL=")); DBG(cfgAutoScanPauseMs);
  DBG(F(" SR=")); DBG(cfgAutoScanRightPauseMs);
  DBG(F(" SC=")); DBGLN(cfgAutoCenterPauseMs);
  DBG(F("TIME MP=")); DBG(cfgAutoMotionPauseMs);
  DBG(F(" IH=")); DBG(cfgInvestigateHoldMs);
  DBG(F(" IA=")); DBG(cfgInvestigateAlignMs);
  DBG(F(" IV=")); DBGLN(cfgInvestigateApproachMs);
}

void printStatus() {
  printTelemetry();
  printConfigSummary();
}

// ============================================================
//  I2C SCAN
// ============================================================

void scanI2C() {
#if !ENABLE_MOTOR_I2C
  DBGLN(F("I2C OFF"));
  return;
#endif

#if ENABLE_I2C_SCAN_DEBUG
  // If either line is stuck LOW, attempt recovery to unstick SDA.
  if (digitalRead(SDA) == LOW || digitalRead(SCL) == LOW) {
    VDBGLN(F("I2C recover"));
    recoverMotorI2cBus();
    delay(20);
  }
#endif

  Wire.setClock(MOTOR_I2C_CLOCK_HZ);

  byte found = 0;
  byte targetErr = 255;
  for (byte addr = 1; addr < 127; addr++) {
    Wire.beginTransmission(addr);
    byte err = Wire.endTransmission();
    if (err == 0) {
      found++;
    } else if (addr == I2C_ADDR) {
      targetErr = err;
    }
  }

  DBG(F("I2C "));
  DBG((int)found);
  DBG(F(" T="));
  DBG((int)targetErr);
  DBGLN(F(""));
}

// ============================================================
//  CALIBRATION
// ============================================================

void calibrateMQ2() {
  VDBGLN(F("MQ2 CAL"));
  long sum = 0;
  for (byte i = 0; i < 20; i++) {
    for (byte j = 0; j < 10; j++) {
      delay(10);
      processInput(Serial);
      processInput(BT);
    }
    sum += readMQ2StableRaw();
    VDBG(F("."));
  }
  VDBGNL();
  mq2Baseline = sum / 20;
  sGasRaw = mq2Baseline;
  sGas = mq2Baseline;
  saveMQ2Baseline();
  VDBG(F("MQ2="));
  VDBGLN(mq2Baseline);
  VDBGLN(F("MQ2 SAVE"));
}

void resetThresholds() {
  mq2Baseline = 0;
  clearMQ2BaselineStorage();
  cfgDistStop = DIST_STOP;
  cfgDistWarn = DIST_WARN;
  cfgDistClear = DIST_CLEAR;
  cfgGasThreshold = GAS_THRESHOLD;
  cfgGasDanger = GAS_DANGER;
  cfgTempWarn = TEMP_WARN;
  cfgTempDanger = TEMP_DANGER;
  cfgAutoPwm = AUTO_MODE_TARGET_PWM;
  cfgAutoBackupMs = AUTO_BACKUP_MS;
  cfgAutoTurnMs = AUTO_TURN_MS;
  cfgAutoScanPauseMs = AUTO_SCAN_PAUSE;
  cfgAutoScanRightPauseMs = AUTO_SCAN_RIGHT_PAUSE;
  cfgAutoCenterPauseMs = AUTO_CENTER_PAUSE;
  cfgAutoMotionPauseMs = AUTO_MOTION_PAUSE_MS;
  cfgAutoBackMinMs = AUTO_BACK_MIN_MS;
  cfgAutoBackStepMs = AUTO_BACK_STEP_MS;
  cfgAutoTurnMinMs = AUTO_TURN_MIN_MS;
  cfgAutoTurnStepMs = AUTO_TURN_STEP_MS;
  cfgInvestigateApproachMs = INVESTIGATE_APPROACH_MS;
  cfgInvestigateHoldMs = INVESTIGATE_HOLD_MS;
  cfgInvestigateAlignMs = INVESTIGATE_ALIGN_MS;
  sendAutoPwmConfig();
  hazardActive = false;
  autoState = AUTO_PATROL;
  scanPhase = SCAN_IDLE;
  resetRecoveryState();
  clearInvestigateMission(false);
  setMode("IDLE");
  VDBGLN(F("RESET OK"));
  printConfigSummary();
}

// ============================================================
//  COMMAND HANDLER
// ============================================================

void handleMovementOverride() {
  manualLock = true;
  clearInvestigateMission(false);
  if (autonomous) {
    autonomous = false;
    autoState = AUTO_PATROL;
    scanPhase = SCAN_IDLE;
    sendCmd('M');
    VDBG(F(" (auto OFF)"));
  }
  VDBGNL();
}

void setTelemetryStreaming(bool enabled) {
  telemActive = enabled;
  DBG(F("TELEM: "));
  DBGLN(telemActive ? F("ON") : F("OFF"));
}

bool applyRuntimeConfig(const char *key, int value) {
  if (strcmp(key, "DS") == 0) {
    cfgDistStop = constrain(value, 8, 60);
    if (cfgDistWarn < cfgDistStop) {
      cfgDistWarn = cfgDistStop + 5;
    }
    if (cfgDistClear < cfgDistWarn) {
      cfgDistClear = cfgDistWarn + 5;
    }
    return true;
  }
  if (strcmp(key, "DW") == 0) {
    cfgDistWarn = constrain(value, cfgDistStop + 2, 100);
    if (cfgDistClear < cfgDistWarn) {
      cfgDistClear = cfgDistWarn + 5;
    }
    return true;
  }
  if (strcmp(key, "DC") == 0) {
    cfgDistClear = constrain(value, cfgDistWarn + 2, 150);
    return true;
  }
  if (strcmp(key, "GW") == 0) {
    cfgGasThreshold = constrain(value, 50, 900);
    if (cfgGasDanger < cfgGasThreshold + 50) {
      cfgGasDanger = cfgGasThreshold + 50;
    }
    return true;
  }
  if (strcmp(key, "GD") == 0) {
    cfgGasDanger = constrain(value, cfgGasThreshold + 20, 1000);
    return true;
  }
  if (strcmp(key, "TW") == 0) {
    cfgTempWarn = constrain(value, 20, 80);
    if (cfgTempDanger < cfgTempWarn + 3) {
      cfgTempDanger = cfgTempWarn + 3;
    }
    return true;
  }
  if (strcmp(key, "TD") == 0) {
    cfgTempDanger = constrain(value, cfgTempWarn + 2, 100);
    return true;
  }
  if (strcmp(key, "AP") == 0) {
    cfgAutoPwm = constrain(value, AUTO_PWM_MIN, AUTO_PWM_MAX);
    sendAutoPwmConfig();
    return true;
  }
  if (strcmp(key, "BK") == 0) {
    cfgAutoBackupMs = (uint16_t)constrain(value, 150, 5000);
    if (cfgAutoBackMinMs > cfgAutoBackupMs) {
      cfgAutoBackMinMs = cfgAutoBackupMs;
    }
    return true;
  }
  if (strcmp(key, "TR") == 0) {
    cfgAutoTurnMs = (uint16_t)constrain(value, 150, 5000);
    if (cfgAutoTurnMinMs > cfgAutoTurnMs) {
      cfgAutoTurnMinMs = cfgAutoTurnMs;
    }
    return true;
  }
  if (strcmp(key, "BMIN") == 0) {
    cfgAutoBackMinMs = (uint16_t)constrain(value, 0, (int)cfgAutoBackupMs);
    return true;
  }
  if (strcmp(key, "TMIN") == 0) {
    cfgAutoTurnMinMs = (uint16_t)constrain(value, 0, (int)cfgAutoTurnMs);
    return true;
  }
  if (strcmp(key, "BSTP") == 0) {
    cfgAutoBackStepMs = (uint16_t)constrain(value, 0, 2000);
    return true;
  }
  if (strcmp(key, "TSTP") == 0) {
    cfgAutoTurnStepMs = (uint16_t)constrain(value, 0, 2000);
    return true;
  }
  if (strcmp(key, "SL") == 0) {
    cfgAutoScanPauseMs = (uint16_t)constrain(value, 40, 1500);
    return true;
  }
  if (strcmp(key, "SR") == 0) {
    cfgAutoScanRightPauseMs = (uint16_t)constrain(value, 40, 1500);
    return true;
  }
  if (strcmp(key, "SC") == 0) {
    cfgAutoCenterPauseMs = (uint16_t)constrain(value, 40, 1500);
    return true;
  }
  if (strcmp(key, "MP") == 0) {
    cfgAutoMotionPauseMs = (uint16_t)constrain(value, 0, 10000);
    return true;
  }
  if (strcmp(key, "IH") == 0) {
    cfgInvestigateHoldMs = (uint16_t)constrain(value, 0, 5000);
    return true;
  }
  if (strcmp(key, "IA") == 0) {
    cfgInvestigateAlignMs = (uint16_t)constrain(value, 0, 5000);
    return true;
  }
  if (strcmp(key, "IV") == 0) {
    cfgInvestigateApproachMs = (uint16_t)constrain(value, 0, 5000);
    return true;
  }
  return false;
}

void processLineCommand(const char *line) {
  if (line == NULL || line[0] == '\0') {
    return;
  }

  char work[48];
  strncpy(work, line, sizeof(work) - 1);
  work[sizeof(work) - 1] = '\0';

  if (work[0] == ':') {
    size_t len = strlen(work);
    if (len > 0) {
      memmove(work, work + 1, len);
    }
  }

  for (char *p = work; *p; ++p) {
    if (*p >= 'a' && *p <= 'z') {
      *p = (char)(*p - ('a' - 'A'));
    }
  }

  char *token = strtok(work, " ");
  if (token == NULL) {
    return;
  }

  if (strcmp(token, "GETCFG") == 0 || strcmp(token, "CFG?") == 0) {
    printConfigSummary();
    return;
  }

  if (strcmp(token, "CFG") == 0) {
    char *key = strtok(NULL, " ");
    if (key == NULL) {
      DBGLN(F("CFG ERR"));
      return;
    }

    if (strcmp(key, "RESET") == 0) {
      resetThresholds();
      return;
    }

    char *valueText = strtok(NULL, " ");
    if (valueText == NULL) {
      DBGLN(F("CFG ERR"));
      return;
    }

    int value = atoi(valueText);
    if (applyRuntimeConfig(key, value)) {
      printConfigSummary();
      DBGLN(F("CFG OK"));
    } else {
      DBG(F("CFG BAD "));
      DBGLN(key);
    }
    return;
  }

  if (strcmp(token, "HELP") == 0) {
    DBGLN(F("CFG? | CFG RESET"));
    DBGLN(F("CFG DS/DW/DC GW/GD TW/TD AP BK/TR BMIN/TMIN BSTP/TSTP SL/SR/SC MP IH/IA/IV"));
    return;
  }

  if (strcmp(token, "MODE") == 0) {
    char *action = strtok(NULL, " ");
    if (action == NULL) {
      DBGLN(F("MODE ERR"));
      return;
    }

    if (strcmp(action, "AUTO") == 0) {
      manualLock = false;
      if (!autonomous) {
        handleCommand('A');
      } else {
        sendAutoPwmConfig();
        setMode("AUTO");
        DBGLN(F("MODE AUTO"));
      }
      return;
    }

    if (strcmp(action, "MANUAL") == 0) {
      manualLock = true;
      lastCmd = 'S';
      setMode("IDLE");
      clearInvestigateMission(false);
      if (autonomous) {
        autonomous = false;
        autoState = AUTO_PATROL;
        scanPhase = SCAN_IDLE;
        sendCmd('M');
      }
      sendCmd('S');
      DBGLN(F("MODE MAN"));
      return;
    }

    DBGLN(F("MODE ERR"));
    return;
  }

  if (strcmp(token, "EVENT") == 0) {
    char *action = strtok(NULL, " ");
    if (action == NULL) {
      DBGLN(F("EVENT ERR"));
      return;
    }

    if (strcmp(action, "INVESTIGATE") == 0) {
      if (hazardActive) {
        DBGLN(F("EVENT BLOCK HAZ"));
        return;
      }
      if (manualLock) {
        DBGLN(F("EVENT MAN"));
        return;
      }
      char *direction = strtok(NULL, " ");
      int bearingHint = 0;
      bool bearingProvided = false;
      if (direction != NULL) {
        if (strcmp(direction, "LEFT") == 0) {
          bearingHint = -45;
        } else if (strcmp(direction, "RIGHT") == 0) {
          bearingHint = 45;
        }
      }

      // Optional metadata tokens are mostly ignored in the Uno-fit build.
      // Only SRC=VISION is still used so recent PC-vision events do not
      // immediately retrigger a motion-based investigate target.
      char *meta = NULL;
      while ((meta = strtok(NULL, " ")) != NULL) {
        if (meta[0] == 'B' && meta[1] == '=') {
          int parsedBearing = atoi(meta + 2);
          bearingHint = constrain(parsedBearing, -90, 90);
          bearingProvided = true;
        } else if (strncmp(meta, "SRC=", 4) == 0) {
          const char *src = meta + 4;
          if (strcmp(src, "VISION") == 0) {
            lastVisionInvestigateMs = millis();
          }
        }
      }

      if (!bearingProvided && direction != NULL && strcmp(direction, "CENTER") == 0) {
        bearingHint = 0;
      }

      beginOperatorInvestigate(bearingHint);
      DBG(F("EVENT INV "));
      if (direction == NULL || strcmp(direction, "CENTER") == 0 || strcmp(direction, "CLOSE") == 0) {
        DBGLN(F("CENTER"));
      } else if (strcmp(direction, "LEFT") == 0) {
        DBGLN(F("LEFT"));
      } else if (strcmp(direction, "RIGHT") == 0) {
        DBGLN(F("RIGHT"));
      } else {
        DBGLN(direction);
      }
      return;
    }

    if (strcmp(action, "CLEAR") == 0) {
      clearInvestigateMission(false);
      motionPauseUntil = millis() + 5000UL;
      if (autonomous) {
        autoState = AUTO_PATROL;
        scanPhase = SCAN_IDLE;
      }
      DBGLN(F("EVENT CLR"));
      return;
    }

    if (strcmp(action, "HOLD") == 0) {
      manualLock = true;
      lastCmd = 'S';
      setMode("IDLE");
      clearInvestigateMission(false);
      if (autonomous) {
        autonomous = false;
        autoState = AUTO_PATROL;
        scanPhase = SCAN_IDLE;
        sendCmd('M');
      }
      sendCmd('S');
      DBGLN(F("EVENT HOLD"));
      return;
    }

    DBG(F("EVENT BAD "));
    DBGLN(action);
    return;
  }

  DBG(F("LINE BAD "));
  DBGLN(work);
}

void handleCommand(char cmd) {
  if (cmd >= 'a' && cmd <= 'z') {
    cmd = (char)(cmd - ('a' - 'A'));
  }

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
      VDBGLN(F("TEST"));
      readAllSensors();
      printTelemetry();
      break;

    case 'Z':
      autoSafety = !autoSafety;
      VDBG(F("SAFE "));
      VDBGLN(autoSafety ? F("ON") : F("OFF"));
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
        manualLock = false;
        autoState = AUTO_PATROL;
        scanPhase = SCAN_IDLE;
        motionPauseUntil = 0;
        searchPatternTimer = millis();
        resetRecoveryState();
        clearInvestigateMission(false);
        sendCmd('A');
        sendAutoPwmConfig();
        setMode("AUTO");
        DBGLN(F("MODE AUTO"));
      } else {
        manualLock = true;
        sendCmd('M');
        sendCmd('S');
        scanPhase = SCAN_IDLE;
        resetRecoveryState();
        clearInvestigateMission(false);
        setMode("IDLE");
        DBGLN(F("MODE MAN"));
      }
      break;

    case 'E':
      if (!hazardActive) {
        enterHazard(NULL);
        VDBGLN(F("HAZ ON"));
      } else {
        clearHazardState();
        VDBGLN(F("HAZ OFF"));
      }
      break;

    case 'F':
      if (!hazardActive) {
        lastCmd = 'F';
        setMode("FWD");
        sendCmd('F');
        VDBG(F("CMD: FWD"));
        handleMovementOverride();
      } else {
        VDBGLN(F("CMD BLOCK HAZ"));
      }
      break;

    case 'B':
      if (!hazardActive) {
        lastCmd = 'B';
        setMode("BACK");
        sendCmd('B');
        VDBG(F("CMD: BACK"));
        handleMovementOverride();
      } else {
        VDBGLN(F("CMD BLOCK HAZ"));
      }
      break;

    case 'L':
      if (!hazardActive) {
        lastCmd = 'L';
        setMode("LEFT");
        sendCmd('L');
        VDBGLN(F("CMD: LEFT"));
        handleMovementOverride();
      }
      break;

    case 'R':
      if (!hazardActive) {
        lastCmd = 'R';
        setMode("RGHT");
        sendCmd('R');
        VDBGLN(F("CMD: RIGHT"));
        handleMovementOverride();
      }
      break;

    case 'S':
      manualLock = true;
      lastCmd = 'S';
      setMode("IDLE");
      clearInvestigateMission(false);
      if (autonomous) {
        autonomous = false;
        autoState = AUTO_PATROL;
        scanPhase = SCAN_IDLE;
        sendCmd('M');
      }
      sendCmd('S');
      VDBGLN(F("CMD: STOP"));
      break;

    case '1':
      sendCmd('1');
      VDBGLN(F("SPD 1"));
      break;

    case '2':
      sendCmd('2');
      VDBGLN(F("SPD 2"));
      break;

    case '3':
      sendCmd('3');
      VDBGLN(F("SPD 3"));
      break;

    case '<':
      sendCmd('<');
      VDBGLN(F("SRV L"));
      break;

    case '>':
      sendCmd('>');
      VDBGLN(F("SRV R"));
      break;

    case 'C':
      sendCmd('C');
      VDBGLN(F("SRV C"));
      break;

    default:
      VDBG(F("BAD CMD '"));
      VDBG(cmd);
      VDBG(F("' (0x"));
      {
        // DBG() only supports one argument; print hex manually to avoid DBG(x, HEX).
        if (ENABLE_VERBOSE_LOGS) {
          unsigned char ub = (unsigned char)cmd;
          Serial.print(ub, HEX);
          if (dbgMirrorBt()) {
            BT.print(ub, HEX);
          }
        }
      }
      VDBGLN(F(")"));
      break;
  }
}

// ============================================================
//  INPUT HANDLING
// ============================================================

void processInput(Stream &port) {
  static unsigned long lastCmdTimeUSB = 0;
  static unsigned long lastCmdTimeBT = 0;
  static char lineBufferUSB[48];
  static char lineBufferBT[48];
  static byte lineLenUSB = 0;
  static byte lineLenBT = 0;
  unsigned long *lastCmdTime = (&port == &Serial) ? &lastCmdTimeUSB : &lastCmdTimeBT;
  char *lineBuffer = (&port == &Serial) ? lineBufferUSB : lineBufferBT;
  byte *lineLen = (&port == &Serial) ? &lineLenUSB : &lineLenBT;

  while (port.available() > 0) {
    int raw = port.peek();
    if (raw < 0) {
      break;
    }
    char c = (char)raw;

    bool fromBt = (&port == &BT);

    if (*lineLen > 0 || c == ':') {
      port.read();

      if (&port == &Serial) {
        usbConsoleSeen = true;
        btConsoleFallback = false;
      }

      if (c == '\r') {
        continue;
      }

      if (c == '\n') {
        lineBuffer[*lineLen] = '\0';
        if (*lineLen > 0) {
          if (fromBt) {
            btReplyMode = !(telemActive && !autonomous);
          }
          VDBG(lineBuffer);
          VDBGNL();
          processLineCommand(lineBuffer);
          if (fromBt) {
            btReplyMode = false;
          }
        }
        *lineLen = 0;
        continue;
      }

      if (*lineLen < 47) {
        lineBuffer[*lineLen] = c;
        (*lineLen)++;
      }
      continue;
    }
    
    if (c == '\r' || c == '\n' || c == ' ') {
      port.read();
      continue;
    }

    unsigned long now = millis();
    // Throttle without losing bytes: old code used read()+continue and *threw away* the character.
    if (!isImmediateCommand(c) &&
        *lastCmdTime != 0U &&
        (unsigned long)(now - *lastCmdTime) < (unsigned long)COMMAND_GAP_MS) {
      break;
    }

    port.read();
    *lastCmdTime = now;

    if (&port == &Serial) {
      usbConsoleSeen = true;
      btConsoleFallback = false;
    }

    if (fromBt) {
      btReplyMode = !(telemActive && !autonomous);
    }

    VDBG(c);
    VDBGNL();
    handleCommand(c);

    if (fromBt) {
      btReplyMode = false;
    }
  }
}

void updateConsoleRouting() {
  if (ENABLE_BT_DEBUG) {
    return;
  }

  if (!usbConsoleSeen && !btConsoleFallback && millis() - bootTime >= USB_FALLBACK_TO_BT_MS) {
    btConsoleFallback = true;
  }

  if (btConsoleFallback && !btFallbackAnnounced) {
    btFallbackAnnounced = true;
    BT.println(F("BT fallback"));
  }
}

bool isImmediateCommand(char cmd) {
  return cmd == 'S' || cmd == 's' || cmd == 'E' || cmd == 'e' || cmd == ']' || cmd == 'H' || cmd == 'G';
}

// ============================================================
//  SETUP
// ============================================================

void setup() {
  resetObstacleMap();
  clearVictimTracking();

  Serial.begin(9600);
  BT.begin(9600);
  bootTime = millis();
  delay(20);
  while (Serial.available() > 0) {
    Serial.read();
  }
  while (BT.available() > 0) {
    BT.read();
  }
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
  pinMode(PIN_IR_PROX_LEFT, INPUT);
  pinMode(PIN_IR_PROX_RIGHT, INPUT);
  pinMode(PIN_PIR1, INPUT);
  pinMode(PIN_PIR2, INPUT);
  pinMode(PIN_MQ2, INPUT);

  VDBGLN(F("MQ2"));
  unsigned long start = millis();
  while (millis() - start < MQ2_WARMUP_MS) {
    for (byte k = 0; k < 50; k++) {
      delay(10);
      processInput(Serial);
      processInput(BT);
      renderNeoPixelStatus();
    }
    VDBG(F("."));
  }
  VDBGNL();

  if (loadMQ2Baseline()) {
    VDBG(F("MQ2 LOAD "));
    VDBGLN(mq2Baseline);
  } else {
    VDBGLN(F("MQ2 CAL"));
    calibrateMQ2();
  }

#if ENABLE_BOOT_I2C_SCAN
  scanI2C();
#endif
  readAllSensors();
  evaluateAlerts();
  sendAutoPwmConfig();
  printConfigSummary();
  printTelemetry();
}

// ============================================================
//  MAIN LOOP
// ============================================================

void loop() {
  updateConsoleRouting();
  processInput(Serial);
  processInput(BT);

  // Keep front obstacle sensing on a fast path so ultrasonic/IR threshold
  // crossings can stop patrol immediately without waiting for the slower
  // full sensor sweep.
  readFastObstacleSensors();

  if (millis() - lastSensorTime >= SENSOR_INTERVAL_MS) {
    lastSensorTime = millis();
    readAllSensors();
    evaluateAlerts();
    checkAutoSafety();
  }

  if (autonomous && !hazardActive) {
    runAutonomous();
  }

  if (telemActive && millis() - lastTelemTime >= ((telemActive && !autonomous) ? 650UL : TELEM_INTERVAL_MS)) {
    if (!(telemActive && !autonomous && (BT.available() > 0 || Serial.available() > 0))) {
      lastTelemTime = millis();
      printTelemetry();
    }
  }

  renderNeoPixelStatus();
}

/**
 * ============================================================
 *  SAR DISASTER RESPONSE ROBOT - ARDUINO 2 (MASTER BRAIN)
 *  Version: 2.0 INTEGRATION SANDBOX
 * ============================================================
 *  Role    : I2C Master - Bluetooth control + sensors +
 *            autonomous decision making
 *
 *  V2 INTEGRATION GOALS:
 *    1. Manual override always wins
 *    2. Hazard response retreats and observes from distance
 *    3. Collision avoidance uses angular ultrasonic memory
 *    4. Investigation fuses motion + suspicious environment
 *    5. Patrol stays cautious and lowest priority
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
 *    PIR 1        : D7
 *    PIR 2        : D8
 *    Rear US TRIG : D9
 *    Rear US ECHO : D10
 *    Front US ECHO: D11
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
#include <string.h>

// ============================================================
//  TUNING ZONE
// ============================================================

#define DIST_STOP             15
#define DIST_WARN             30
#define DIST_CLEAR            40

#define GAS_THRESHOLD         400
#define GAS_DANGER            600
#define GAS_CLEAR_MARGIN      60

// Feature toggles: keep these 0/1 defines (used in #if across this sketch).
#define ENABLE_FLAME_SENSORS  1

#define ENABLE_REAR_US        1
#define ENABLE_MOTOR_I2C      1
#define ENABLE_BT_DEBUG       1
#define ENABLE_BOOT_I2C_SCAN  0
#define ENABLE_I2C_SCAN_DEBUG 1   // extra debug on the master I2C scan
#define ENABLE_NEOPIXEL       1
#define ENABLE_FLAME_HAZARD   0
// This analog flame module is "reversed" in this build:
// low reading at idle, higher reading when flame is detected.
#define FLAME_THRESHOLD       120
#define FLAME_ARM_DELAY_MS    8000

#define TEMP_WARN             38
#define TEMP_DANGER           50

#define AUTO_BACKUP_MS        600
#define AUTO_TURN_MS          700
#define AUTO_SCAN_PAUSE       220
#define AUTO_SCAN_RIGHT_PAUSE 300
#define AUTO_CENTER_PAUSE     120
#define AUTO_PATROL_SPEED     1
#define AUTO_MOTION_PAUSE_MS  1200
#define AUTO_MODE_TARGET_PWM  80
#define HAZARD_OBSERVE_DIST   60
#define OBSTACLE_MEMORY_SLOTS 9
#define FLAME_CONFIRM_COUNT   2

#define TELEM_INTERVAL_MS     500
#define SENSOR_INTERVAL_MS    200
#define DHT_INTERVAL_MS       2000
#define COMMAND_GAP_MS        20
#define USB_FALLBACK_TO_BT_MS 5000
#define US_TIMEOUT_US         18000UL
#define US_SAMPLES            2

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
#define PIN_FLAME_LEFT        A0
#define PIN_FLAME_CENTER      A1
#define PIN_FLAME_RIGHT       A2
#define PIN_MQ2               A3
#define I2C_ADDR              8
#define NEOPIXEL_COUNT        8
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

// ============================================================
//  DEBUG HELPERS
// ============================================================

void sendCmd(char cmd);
void sendCmdSilent(char cmd);
void recoverMotorI2cBus();
void updateConsoleRouting();
void processInput(Stream &port, const __FlashStringHelper *label);
bool useBtDebug();
bool dbgMirrorBt();
bool isImmediateCommand(char cmd);
void setTelemetryStreaming(bool enabled);
void trackRadarServoCommand(char cmd);
extern bool btConsoleFallback;

struct MQ2CalibrationData {
  uint16_t magic;
  int baseline;
};

bool useBtDebug() {
  return ENABLE_BT_DEBUG || btConsoleFallback;
}

bool dbgMirrorBt() {
  return useBtDebug() || btReplyMode;
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

enum AutoState {
  AUTO_PATROL,
  AUTO_OBSTACLE_FOUND,
  AUTO_BACKING,
  AUTO_SCANNING,
  AUTO_TURNING,
  AUTO_HAZARD
};

AutoState autoState = AUTO_PATROL;

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
    sendCmd('S');
    sendCmd('H');
  }
  hazardActive = true;
  autoState = AUTO_HAZARD;
  scanPhase = SCAN_IDLE;
  setMode("HAZARD");
  DBG(F("HAZARD: "));
  DBGLN(reason);
}

void clearHazardState() {
  hazardActive = false;
  scanPhase = SCAN_IDLE;
  sendCmd('G');
  if (autonomous) {
    autoState = AUTO_PATROL;
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

void sendCmd(char cmd) {
  trackRadarServoCommand(cmd);
#if ENABLE_MOTOR_I2C
  Wire.beginTransmission(I2C_ADDR);
  Wire.write(cmd);
  byte err = Wire.endTransmission();
  if (err != 0) {
    DBG(F("[I2C] FAIL sending '"));
    DBG(cmd);
    DBG(F("' err="));
    DBGLN((int)err);
    if (err == 4) {
      recoverMotorI2cBus();
    }
  }
#else
  DBG(F("[BENCH] Motor cmd skipped: "));
  DBGLN(cmd);
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
  float best = 999.0f;
  for (byte i = 0; i < US_SAMPLES; i++) {
    float sample = readUltrasonicRaw(trigPin, echoPin);
    if (sample < best) {
      best = sample;
    }
    delay(5);
  }
  return best;
}

float readFrontUltrasonic() {
  sFrontDist = readUltrasonicSensor(PIN_US_FRONT_TRIG, PIN_US_FRONT_ECHO);
  return sFrontDist;
}

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
  if (now - lastDhtTime < DHT_INTERVAL_MS) {
    return;
  }

  float h = dht.readHumidity();
  float t = dht.readTemperature();
  if (!isnan(h)) {
    sHumid = h;
  }
  if (!isnan(t)) {
    sTemp = t;
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
  sFlameA1 = 1023;
  sFlameA2 = 1023;
  sFlameA3 = 1023;
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
    DBG(F("A1=")); DBG(sFlameA1);
    DBG(F(" A2=")); DBG(sFlameA2);
    DBG(F(" A3=")); DBG(sFlameA3);
    DBG(F(" DIR=")); DBGLN(flameDirectionLabel());
  } else if (!flameNow && alertFlame) {
    alertFlame = false;
    DBGLN(F("ALERT: Flame cleared"));
  }

  int gasWarnThreshold = alertGas ? (GAS_THRESHOLD - GAS_CLEAR_MARGIN) : GAS_THRESHOLD;
  if (gasWarnThreshold < 0) {
    gasWarnThreshold = 0;
  }

  bool gasNow = (gasDelta() > gasWarnThreshold);
  if (gasNow && !alertGas) {
    alertGas = true;
    DBG(F("!!! ALERT: GAS DETECTED raw/filtered="));
    DBG(sGasRaw);
    DBG(F("/"));
    DBG(sGas);
    DBG(F(" delta="));
    DBGLN(gasDelta());
  } else if (!gasNow && alertGas) {
    alertGas = false;
    DBGLN(F("ALERT: Gas cleared"));
  }

  bool frontNow = (sFrontDist > 0.0f && sFrontDist < DIST_STOP);
  bool rearNow = (sRearDist > 0.0f && sRearDist < DIST_STOP);
  if (frontNow && !alertObstacleFront) {
    alertObstacleFront = true;
    DBG(F("!!! ALERT: FRONT OBSTACLE at "));
    DBG(sFrontDist);
    DBGLN(F("cm"));
  } else if (!frontNow && alertObstacleFront) {
    alertObstacleFront = false;
    DBGLN(F("ALERT: Front obstacle cleared"));
  }

  if (rearNow && !alertObstacleRear) {
    alertObstacleRear = true;
    DBG(F("!!! ALERT: REAR OBSTACLE at "));
    DBG(sRearDist);
    DBGLN(F("cm"));
  } else if (!rearNow && alertObstacleRear) {
    alertObstacleRear = false;
    DBGLN(F("ALERT: Rear obstacle cleared"));
  }

  bool motionNow = (sPIR1 == HIGH || sPIR2 == HIGH);
  if (motionNow && !alertMotion) {
    alertMotion = true;
    DBG(F("!!! ALERT: MOTION PIR1="));
    DBG(sPIR1);
    DBG(F(" PIR2="));
    DBGLN(sPIR2);
  } else if (!motionNow && alertMotion) {
    alertMotion = false;
  }

  bool tempNow = (sTemp > 0.0f && sTemp > TEMP_WARN);
  if (tempNow && !alertTemp) {
    alertTemp = true;
    DBG(F("!!! ALERT: HIGH TEMP "));
    DBG(sTemp);
    DBGLN(F("C"));
  } else if (!tempNow && alertTemp) {
    alertTemp = false;
    DBGLN(F("ALERT: Temp normal"));
  }
}

// ============================================================
//  AUTO SAFETY
// ============================================================

void checkAutoSafety() {
  if (!autoSafety) {
    return;
  }

  bool critical = (gasDelta() > GAS_DANGER) || (sTemp > TEMP_DANGER);

  if (critical) {
    if (gasDelta() > GAS_DANGER) {
      enterHazard(F("AUTO-SAFETY GAS"));
    } else {
      enterHazard(F("AUTO-SAFETY HEAT"));
    }
    return;
  }

  if (hazardActive && !alertGas && sTemp <= TEMP_WARN) {
    DBGLN(F("AUTO-SAFETY: Hazard cleared"));
    clearHazardState();
  }
}

// ============================================================
//  SMART AUTONOMOUS HELPERS
// ============================================================

char decideBestTurn(float leftDist, float rightDist) {
  if (leftDist >= DIST_CLEAR && leftDist >= rightDist) {
    return 'L';
  }
  if (rightDist >= DIST_CLEAR && rightDist > leftDist) {
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
        scanLeftDist = readFrontUltrasonic();
        sendCmd('>');
        autoTimer = millis();
        scanPhase = SCAN_WAIT_RIGHT;
      }
      return false;

    case SCAN_WAIT_RIGHT:
      if (millis() - autoTimer >= AUTO_SCAN_RIGHT_PAUSE) {
        scanRightDist = readFrontUltrasonic();
        sendCmd('C');
        autoTimer = millis();
        scanPhase = SCAN_WAIT_CENTER;
      }
      return false;

    case SCAN_WAIT_CENTER:
      if (millis() - autoTimer >= AUTO_CENTER_PAUSE) {
        pendingTurnCmd = decideBestTurn(scanLeftDist, scanRightDist);
        scanPhase = SCAN_IDLE;
        DBG(F("AUTO: Scan L="));
        DBG(scanLeftDist);
        DBG(F("cm R="));
        DBG(scanRightDist);
        DBGLN(F("cm"));
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
  if (!autonomous || hazardActive) {
    return;
  }

  if (alertGas) {
    enterHazard(F("AUTO GAS"));
    return;
  }

  switch (autoState) {
    case AUTO_PATROL:
      setMode("AUTO-PTRL");

      if (alertMotion) {
        sendCmdSilent('S');
        motionPauseUntil = millis() + AUTO_MOTION_PAUSE_MS;
        autoState = AUTO_SCANNING;
        setMode("AUTO-MOTION");
        DBGLN(F("AUTO: Motion detected, pausing"));
        break;
      }

      if (sFrontDist > 0.0f && sFrontDist < DIST_STOP) {
        sendCmdSilent('S');
        autoState = AUTO_OBSTACLE_FOUND;
        autoTimer = millis();
        DBG(F("AUTO: Obstacle at "));
        DBG(sFrontDist);
        DBGLN(F("cm"));
        break;
      }

      if (sFrontDist > 0.0f && sFrontDist < DIST_WARN) {
        setMode("AUTO-SLOW");
        sendCmd('1');
        sendCmdSilent('F');
      } else {
        sendCmd((char)('0' + AUTO_PATROL_SPEED));
        sendCmdSilent('F');
      }
      break;

    case AUTO_OBSTACLE_FOUND:
      setMode("AUTO-OBST");
      sendCmdSilent('B');
      autoTimer = millis();
      autoState = AUTO_BACKING;
      DBGLN(F("AUTO: Backing up"));
      break;

    case AUTO_BACKING:
      setMode("AUTO-BACK");
      readRearUltrasonic();
      if (sRearDist > 0.0f && sRearDist < DIST_STOP) {
        sendCmdSilent('S');
        autoState = AUTO_SCANNING;
        scanPhase = SCAN_IDLE;
        DBGLN(F("AUTO: Rear blocked, stop backing"));
      } else if (millis() - autoTimer >= AUTO_BACKUP_MS) {
        sendCmdSilent('S');
        autoState = AUTO_SCANNING;
        scanPhase = SCAN_IDLE;
        DBGLN(F("AUTO: Scanning path"));
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
        sendCmdSilent(pendingTurnCmd);
        lastTurnLeft = (pendingTurnCmd == 'L');
        autoTimer = millis();
        autoState = AUTO_TURNING;
        DBG(F("AUTO: Turning "));
        DBGLN(pendingTurnCmd == 'L' ? F("LEFT") : F("RIGHT"));
      }
      break;

    case AUTO_TURNING:
      setMode("AUTO-TURN");
      if (millis() - autoTimer >= AUTO_TURN_MS) {
        sendCmdSilent('S');
        autoState = AUTO_PATROL;
        DBGLN(F("AUTO: Resume patrol"));
      }
      break;

    case AUTO_HAZARD:
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
  // One compact line per telemetry tick (used by the live telemetry toggle).
  DBG(F("[")); DBG(currentMode); DBG(F("] "));
  DBG(F("USF:")); DBG(sFrontDist); DBG(F("cm "));
  DBG(F("USR:")); DBG(sRearDist);  DBG(F("cm "));
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
  DBG(F("HAZ:")); DBGLN(hazardActive ? F("YES") : F("NO"));
}

void printStatus() {
  DBGLN(F("========== ARDUINO 2 STATUS =========="));
  DBG(F("Mode          : ")); DBGLN(currentMode);
  DBG(F("Autonomous    : ")); DBGLN(autonomous ? F("ON") : F("OFF"));
  DBG(F("Auto-Safety   : ")); DBGLN(autoSafety ? F("ON") : F("OFF"));
  DBG(F("Hazard        : ")); DBGLN(hazardActive ? F("ACTIVE") : F("CLEAR"));
  DBG(F("Telemetry     : ")); DBGLN(telemActive ? F("ON") : F("OFF"));
  DBG(F("Last cmd      : ")); DBGLN(lastCmd);
  DBGLN(F("--- SENSORS ---"));
  DBG(F("Front dist    : ")); DBG(sFrontDist); DBGLN(F(" cm"));
  DBG(F("Rear dist     : ")); DBG(sRearDist); DBGLN(F(" cm"));
  DBG(F("Temperature   : ")); DBG(sTemp); DBGLN(F(" C"));
  DBG(F("Humidity      : ")); DBG(sHumid); DBGLN(F(" %"));
  DBG(F("PIR 1 / 2     : ")); DBG(sPIR1); DBG(F(" / ")); DBGLN(sPIR2);
  DBG(F("Gas raw/filt/delta : ")); DBG(sGasRaw); DBG(F(" / ")); DBG(sGas); DBG(F(" / ")); DBGLN(gasDelta());
  DBG(F("MQ2 baseline  : ")); DBGLN(mq2Baseline);
  DBG(F("Flame L/C/R   : ")); DBG(sFlameA1); DBG(F(" / ")); DBG(sFlameA2); DBG(F(" / ")); DBGLN(sFlameA3);
  DBG(F("Flame dir     : ")); DBGLN(flameDirectionLabel());
  DBG(F("NeoPixel data : ")); DBGLN(ENABLE_NEOPIXEL ? F("D6 (8 LEDs)") : F("OFF"));
  DBGLN(F("--- ALERTS ---"));
  DBG(F("Flame         : ")); DBGLN(alertFlame ? F("YES") : F("clear"));
  DBG(F("Gas           : ")); DBGLN(alertGas ? F("YES") : F("clear"));
  DBG(F("Front obst    : ")); DBGLN(alertObstacleFront ? F("YES") : F("clear"));
  DBG(F("Rear obst     : ")); DBGLN(alertObstacleRear ? F("YES") : F("clear"));
  DBG(F("Motion        : ")); DBGLN(alertMotion ? F("YES") : F("clear"));
  DBG(F("Temp high     : ")); DBGLN(alertTemp ? F("YES") : F("clear"));
  DBGLN(F("--- THRESHOLDS ---"));
  DBG(F("Dist stop/warn: ")); DBG(DIST_STOP); DBG(F(" / ")); DBGLN(DIST_WARN);
  DBG(F("Gas warn/dngr : ")); DBG(GAS_THRESHOLD); DBG(F(" / ")); DBGLN(GAS_DANGER);
  DBG(F("Temp warn/dngr: ")); DBG(TEMP_WARN); DBG(F(" / ")); DBGLN(TEMP_DANGER);
  DBG(F("Flame thresh  : ")); DBGLN(FLAME_THRESHOLD);
  DBGLN(F("--- I2C ---"));
  DBG(F("Motor I2C     : ")); DBGLN(ENABLE_MOTOR_I2C ? F("ON") : F("BENCH MODE / OFF"));
  DBG(F("Arduino 1 addr: 0x")); printHexByte(I2C_ADDR); DBGNL();
  DBGLN(F("======================================"));
}

// ============================================================
//  I2C SCAN
// ============================================================

void scanI2C() {
#if !ENABLE_MOTOR_I2C
  DBGLN(F("--- I2C SCAN ---"));
  DBGLN(F("BENCH MODE: Motor I2C is disabled in this build"));
  DBGLN(F("Set ENABLE_MOTOR_I2C to 1 when Arduino 1 is connected"));
  DBGLN(F("----------------"));
  return;
#endif
  DBGLN(F("--- I2C SCAN (master) ---"));

  // Show current line states (helpful when shield power/noise is involved).
  pinMode(SDA, INPUT_PULLUP);
  pinMode(SCL, INPUT_PULLUP);
  delayMicroseconds(10);
  DBG(F("Lines: SDA="));
  DBG(digitalRead(SDA));
  DBG(F(" SCL="));
  DBG(digitalRead(SCL));
  DBGLN(F(""));

#if ENABLE_I2C_SCAN_DEBUG
  // If either line is stuck LOW, attempt recovery to unstick SDA.
  if (digitalRead(SDA) == LOW || digitalRead(SCL) == LOW) {
    DBGLN(F("I2C bus looks stuck; attempting recovery..."));
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
      DBG(F("  ACK 0x"));
      printHexByte(addr);
      DBGNL();
      found++;
    } else if (addr == I2C_ADDR) {
      // Always show what happened to the expected motor controller.
      DBG(F("  Target 0x"));
      printHexByte(addr);
      DBG(F(" err="));
      DBGLN((int)err);
    }
  }

  DBG(F("  Total: "));
  DBG((int)found);
  DBGLN(F(" device(s)"));
  DBGLN(F("----------------"));
}

// ============================================================
//  CALIBRATION
// ============================================================

void calibrateMQ2() {
  DBGLN(F("MQ2 CAL: Keep sensor in clean air"));
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
  DBG(F("MQ2 CAL: Baseline = "));
  DBGLN(mq2Baseline);
  DBGLN(F("MQ2 CAL: Saved to EEPROM"));
}

void resetThresholds() {
  mq2Baseline = 0;
  clearMQ2BaselineStorage();
  hazardActive = false;
  autoState = AUTO_PATROL;
  scanPhase = SCAN_IDLE;
  setMode("IDLE");
  DBGLN(F("RESET: Baseline/state restored"));
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
  DBG(F("TELEM: "));
  DBGLN(telemActive ? F("ON") : F("OFF"));
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
      DBGLN(F("--- SENSOR TEST ---"));
      readAllSensors();
      DBG(F("US FRONT: ")); DBG(sFrontDist); DBGLN(F(" cm"));
      DBG(F("US REAR : ")); DBG(sRearDist); DBGLN(F(" cm"));
      DBG(F("DHT11   : ")); DBG(sTemp); DBG(F(" C  ")); DBG(sHumid); DBGLN(F(" %"));
      DBG(F("PIR1    : ")); DBGLN(sPIR1 ? F("MOTION") : F("clear"));
      DBG(F("PIR2    : ")); DBGLN(sPIR2 ? F("MOTION") : F("clear"));
      DBG(F("MQ2     : ")); DBG(sGasRaw); DBG(F("/")); DBG(sGas); DBG(F(" delta=")); DBGLN(gasDelta());
      DBG(F("Flame left  : ")); DBGLN(sFlameA1);
      DBG(F("Flame center: ")); DBGLN(sFlameA2);
      DBG(F("Flame right : ")); DBGLN(sFlameA3);
      DBG(F("Flame dir   : ")); DBGLN(flameDirectionLabel());
      DBGLN(F("NeoPixel: Status strip active on D6"));
      DBGLN(F("-------------------"));
      break;

    case 'Z':
      autoSafety = !autoSafety;
      DBG(F("AUTO-SAFETY: "));
      DBGLN(autoSafety ? F("ON") : F("OFF"));
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
      sendCmd('1');
      DBGLN(F("SPEED: 1 slow"));
      break;

    case '2':
      sendCmd('2');
      DBGLN(F("SPEED: 2 medium"));
      break;

    case '3':
      sendCmd('3');
      DBGLN(F("SPEED: 3 fast"));
      break;

    case '<':
      sendCmd('<');
      DBGLN(F("SERVO: left"));
      break;

    case '>':
      sendCmd('>');
      DBGLN(F("SERVO: right"));
      break;

    case 'C':
      sendCmd('C');
      DBGLN(F("SERVO: center"));
      break;

    default:
      DBG(F("Unknown cmd: '"));
      DBG(cmd);
      DBG(F("' (0x"));
      {
        // DBG() only supports one argument; print hex manually to avoid DBG(x, HEX).
        unsigned char ub = (unsigned char)cmd;
        Serial.print(ub, HEX);
        if (dbgMirrorBt()) {
          BT.print(ub, HEX);
        }
      }
      DBGLN(F(")"));
      DBGLN(F("Use uppercase single-character commands: F B L R S 1 2 3 < > C ? V P A E K ~"));
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
    if (raw < 0) {
      break;
    }
    char c = (char)raw;

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

    bool fromBt = (&port == &BT);
    if (fromBt) {
      btReplyMode = true;
    }

    DBG(label);
    DBG(c);
    DBGNL();
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
    BT.println(F("BT fallback active"));
    BT.println(F("No USB activity detected. Bluetooth console enabled."));
  }
}

bool isImmediateCommand(char cmd) {
  return cmd == 'S' || cmd == 's' || cmd == 'E' || cmd == 'e' || cmd == ']' || cmd == 'H' || cmd == 'G';
}

// ============================================================
//  SETUP
// ============================================================

void setup() {
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
  pinMode(PIN_PIR1, INPUT);
  pinMode(PIN_PIR2, INPUT);
  pinMode(PIN_MQ2, INPUT);

  DBGLN(F("MQ2 warming up..."));
  unsigned long start = millis();
  while (millis() - start < MQ2_WARMUP_MS) {
    for (byte k = 0; k < 50; k++) {
      delay(10);
      processInput(Serial, F("USB> "));
      processInput(BT, F("BT>  "));
      renderNeoPixelStatus();
    }
    DBG(F("."));
  }
  DBGNL();

  if (loadMQ2Baseline()) {
    DBG(F("MQ2 CAL: Loaded baseline from EEPROM = "));
    DBGLN(mq2Baseline);
  } else {
    DBGLN(F("MQ2 CAL: No saved baseline, calibrating now"));
    calibrateMQ2();
  }

  DBGLN(F("============================================"));
  DBGLN(F("  SAR ROBOT v1.1 - ARDUINO 2 MASTER BRAIN"));
  DBGLN(F("============================================"));
  DBGLN(F("  I2C Master | USB+BT dual debug @ 9600"));
  DBGLN(F("--------------------------------------------"));
  DBGLN(F("  MOVE : F B L R S"));
  DBGLN(F("  SPEED: 1 2 3"));
  DBGLN(F("  SERVO: < > C"));
  DBGLN(F("  MODE : A=autonomous  E=hazard"));
  DBGLN(F("  DEBUG: ? V P I T Z K ~"));
  DBGLN(F("--------------------------------------------"));
  DBGLN(F("  UNO SAFE MODE ENABLED"));
  DBGLN(ENABLE_FLAME_SENSORS ? F("  Flame sensors: ANALOG only (A0/A1/A2)") : F("  Flame sensors: OFF"));
  DBGLN(ENABLE_FLAME_HAZARD ? F("  Flame action : HAZARD stop enabled") : F("  Flame action : DIRECTION only / no auto-stop"));
  DBGLN(ENABLE_NEOPIXEL ? F("  NeoPixel     : ON (D6, 8 LEDs)") : F("  NeoPixel     : OFF"));
  DBGLN(ENABLE_REAR_US ? F("  Rear ultrasonic: ON (D9/D10)") : F("  Rear ultrasonic: OFF"));
  DBGLN(F("  Front ultrasonic: ON (TRIG=D4, ECHO=D11)"));
  DBGLN(ENABLE_MOTOR_I2C ? F("  Motor I2C    : ON") : F("  Motor I2C    : BENCH MODE (commands not forwarded)"));
  DBGLN(ENABLE_BT_DEBUG ? F("  BT debug     : ON") : F("  BT debug     : AUTO fallback if USB idle"));
  DBGLN(ENABLE_BOOT_I2C_SCAN ? F("  Boot I2C scan: ON") : F("  Boot I2C scan: OFF"));
  DBG(F("  Stop dist : ")); DBG(DIST_STOP); DBGLN(F(" cm"));
  DBG(F("  Gas thresh: ")); DBGLN(GAS_THRESHOLD);
  DBG(F("  Flame thr : ")); DBGLN(FLAME_THRESHOLD);
  DBGLN(F("============================================"));

#if ENABLE_BOOT_I2C_SCAN
  scanI2C();
#else
  DBGLN(F("I2C boot scan skipped. Send 'I' to scan manually."));
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
    checkAutoSafety();
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

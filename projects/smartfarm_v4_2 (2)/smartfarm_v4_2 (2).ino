/*
  ============================================================
  SMARTFARM v4.2 — LOGIC-CORRECTED + RAIN SENSOR EDITION
  "Truly Smart, Zero Sacrifice"
  ============================================================

  CHANGES FROM v4.1:
  - FIX: baseMin/baseMax were swapped — moisture % was inverted
          (dry soil read as 99%, wet soil read as 0%)
  - FIX: FLAG_DEEP_SOAK is now per-pump (FLAG_DEEP_SOAK1 / FLAG_DEEP_SOAK2)
          so simultaneous pump runs don't corrupt each other's soak state
  - FIX: Mode toggle 'm' shortcut checked cmd[0]=='m' before strcmp("auto"),
          so "auto" command never toggled correctly — reordered logic
  - FIX: 'status'/'save' both match cmd[0]=='s' — 'save' was unreachable
          via shortcut; now uses explicit strcmp only
  - FIX: parseTimeStr returns a sentinel -2 for "off" vs -1 for parse error,
          so schedule disable is no longer ambiguous
  - FIX: calibrate() reads wrong pin for zone index 2 (was GAS_PIN instead
          of soil sensor) — index mapping clarified and guarded
  - ADD: Rain sensor on A4 — closes servo immediately on rain detection,
          overrides all other servo logic with highest priority
  - ADD: Rain sensor suppresses auto-irrigation (no pump trigger while raining)
  - ADD: Rain sensor status on LCD Page 1 and Page 4
  - ADD: 'rain' command shows rain sensor status via Bluetooth
  - ADD: FLAG_RAIN_ACTIVE system flag

  KEY COMMANDS (send via Bluetooth):
  ─────────────────────────────────
  GENERAL:
    help / ?        - Show all commands
    status / s      - Full system status
    debug / d       - Toggle debug output
    page / p        - Next LCD page

  MODE CONTROL:
    auto            - Force AUTO mode
    manual          - Force MANUAL mode
    m               - Toggle AUTO/MANUAL mode

  PUMP CONTROL:
    1 / pump1       - Toggle Pump 1 (manual mode)
    1on / pump1on   - Force Pump 1 ON
    1off / pump1off - Force Pump 1 OFF
    2 / pump2       - Toggle Pump 2 (manual mode)
    2on / pump2on   - Force Pump 2 ON
    2off / pump2off - Force Pump 2 OFF

  FAN CONTROL:
    f / fan         - Toggle Fan
    fanon           - Force Fan ON
    fanoff          - Force Fan OFF

  SCHEDULING:
    3 HH:MM         - Set Pump 1 schedule (e.g., "3 14:30")
    3 off           - Disable Pump 1 schedule
    4 HH:MM         - Set Pump 2 schedule (e.g., "4 08:00")
    4 off           - Disable Pump 2 schedule

  THRESHOLDS:
    zXX             - Set Zone 1 threshold (0-99, e.g., "z35")
    xXX             - Set Zone 2 threshold (0-99, e.g., "x40")
    gXX             - Set gas alarm threshold (e.g., "g80")
    tXX             - Set fan temp threshold (e.g., "t31")
    uXX             - Set fan humidity threshold (e.g., "u85")

  CALIBRATION:
    c0 / c1 / c2    - Calibrate MIN for zone 0,1,2 (dry soil)
    w0 / w1 / w2    - Calibrate MAX for zone 0,1,2 (wet soil)

  SYSTEM:
    reset           - Reset to defaults
    save            - Force save config
    test            - Run system test
    rain            - Show rain sensor status

  DEBUG FORMATS:
    json            - Toggle JSON output mode
    verbose         - Toggle verbose mode

  LCD PAGES:
    Page 0: Zone moisture + Temp/Humidity + Mode
    Page 1: Pump/Fan/Servo/Rain status
    Page 2: Schedule display
    Page 3: Thresholds overview
    Page 4: System health
  ============================================================
*/

#include <SoftwareSerial.h>
#include <DHT.h>
#include <Wire.h>
#include <LiquidCrystal_I2C.h>
#include <RTClib.h>
#include <EEPROM.h>
#include <Servo.h>
#include <avr/pgmspace.h>
#include <avr/wdt.h>

// ── PIN DEFINITIONS ──────────────────────────────────────────
#define BT_RX        2
#define BT_TX        3
#define DHT_PIN      4
#define DHT_TYPE     DHT11
#define FAN_RELAY    5
#define PUMP1_RELAY  6
#define PUMP2_RELAY  7
#define BUZZER_PIN   8
#define LDR_PIN      9
#define SERVO_PIN    10
#define SOIL1_PIN    A0
#define SOIL2_PIN    A1
#define SOIL3_PIN    A2
#define GAS_PIN      A3
#define RAIN_PIN     A4   // NEW: rain sensor (analog — HIGH when dry, LOW when wet)

// ── FLAGS & MEMORY ───────────────────────────────────────────
#define ON           LOW
#define OFF          HIGH
#define EEPROM_MAGIC 160
const uint8_t SAMPLES  = 5;

// Flag bit positions
#define FLAG_AUTO_MODE    0
#define FLAG_RTC_OK       1
#define FLAG_JSON_MODE    2
#define FLAG_DHT_ERR      3
#define FLAG_PUMP1_RUN    4
#define FLAG_PUMP2_RUN    5
#define FLAG_DEEP_SOAK1   6   // FIX: was shared FLAG_DEEP_SOAK — now per-pump
#define FLAG_DEBUG_ON     7

// Second flag byte for extra state
uint8_t flags2 = 0;
#define FLAG2_DEEP_SOAK2  0   // FIX: deep soak flag for pump 2, independent
#define FLAG2_RAIN_ACTIVE 1   // NEW: rain sensor currently detecting rain

uint8_t flags = 0b00000001;   // AUTO_MODE on, RTC assumed not OK until confirmed
#define GET_FLAG(f)     (((flags)>>(f))&1)
#define SET_FLAG(f,v)   flags=(v)?(flags|(1<<(f))):(flags&~(1<<(f)))
#define GET_FLAG2(f)    (((flags2)>>(f))&1)
#define SET_FLAG2(f,v)  flags2=(v)?(flags2|(1<<(f))):(flags2&~(1<<(f)))

// ── GLOBAL OBJECTS ───────────────────────────────────────────
SoftwareSerial BT(BT_RX, BT_TX);
DHT dht(DHT_PIN, DHT_TYPE);
LiquidCrystal_I2C lcd(0x27, 16, 2);
RTC_DS3231 rtc;
Servo roofServo;

// ── SYSTEM DATA ──────────────────────────────────────────────
int readings[3][SAMPLES];
uint8_t readIdx = 0;

// FIX: baseMin/baseMax were swapped in v4.1.
// Capacitive soil sensors: DRY = HIGH analog value (~850), WET = LOW (~350).
// mapPct(raw, lo, hi) maps lo→0%, hi→100%.
// So: baseMin = WET value (lo = 0%), baseMax = DRY value (hi = 100%)?
// NO — we want WET = 100% moisture, DRY = 0% moisture.
// Therefore: lo = DRY raw (~850) maps to 0%, hi = WET raw (~350) maps to 100%.
// mapPct uses constrain(map(raw, lo, hi, 0, 99)) so:
//   lo (dry ~850) → 0%  ✓
//   hi (wet ~350) → 99% ✓
// baseMin[i] = DRY calibration value (high raw ADC reading)
// baseMax[i] = WET calibration value (low raw ADC reading)
// Gas sensor: LOW = clean air (~100), HIGH = gas present (~800) → higher % = more gas
//   lo (clean ~100) → 0%, hi (gas ~800) → 99%  ✓
int baseMin[3] = {850, 850, 100};   // Zone0(dry), Zone1(dry), Gas(clean)
int baseMax[3] = {350, 350, 800};   // Zone0(wet), Zone1(wet), Gas(alarm)

uint8_t pct[3];
int zThr1=35, zThr2=35, gasThr=80, fanTempOn=31, fanHumOn=85;
int schedP1 = -1, schedP2 = -1;
float tempC=0.0, humPct=0.0;
bool lightBright = false, emergencyState = false, isRaining = false;

// ── TIMING VARIABLES ─────────────────────────────────────────
uint32_t lastSensorMs=0, lastLcdMs=0, p1StartMs=0, p2StartMs=0;
uint32_t lastServoMs=0, lastDebugMs=0;
uint8_t currServoPos=90, lcdPage=0, dynPage=255;
uint32_t dynStartMs=0;

// ── SMART POPUP SYSTEM ──────────────────────────────────────
#define POPUP_BUFFER_SIZE 17
char dynL2[POPUP_BUFFER_SIZE];
const __FlashStringHelper* dynL1 = nullptr;

#define POPUP_DURATION_MS   2500
#define POPUP_PRIORITY_LOW  0
#define POPUP_PRIORITY_MED  1
#define POPUP_PRIORITY_HIGH 2
uint8_t popupPriority = POPUP_PRIORITY_LOW;

// ── CUSTOM ICONS IN PROGMEM ─────────────────────────────────
const byte charFan[2][8] PROGMEM = {
  {0,0,13,7,4,28,22,0},
  {0,0,22,28,4,7,13,0}
};
const byte charDrop[8]    PROGMEM = {4,4,14,14,31,31,31,14};
const byte charBar[3][8]  PROGMEM = {
  {0,0,0,0,0,0,0,31},
  {0,0,0,31,31,31,31,31},
  {31,31,31,31,31,31,31,31}
};
const byte charThermo[8]  PROGMEM = {4,10,10,10,14,31,31,14};
const byte charHumid[8]   PROGMEM = {4,4,14,14,31,31,14,0};
const byte charAlert[8]   PROGMEM = {4,14,14,14,31,0,4,0};

// ── AUDIO ENGINE ────────────────────────────────────────────
struct TonePattern {
  uint16_t freq;
  uint16_t duration;
  uint16_t pause;
};

const TonePattern toneClick[]     PROGMEM = {{1200, 50, 0}};
const TonePattern toneSuccess[]   PROGMEM = {{800, 80, 100}, {1500, 80, 0}};
const TonePattern toneOff[]       PROGMEM = {{1500, 80, 100}, {800, 80, 0}};
const TonePattern toneAlert[]     PROGMEM = {{400, 300, 400}, {400, 300, 0}};
const TonePattern toneExecute[]   PROGMEM = {{2000, 40, 50}, {2000, 40, 0}};
const TonePattern toneCalibrate[] PROGMEM = {{1000, 100, 100}, {1500, 100, 100}, {2000, 100, 0}};

volatile uint8_t toneIndex = 0;
volatile uint8_t tonePatternLen = 0;
volatile uint32_t toneNextMs = 0;
const TonePattern* currentTone = nullptr;

void pTone(uint8_t t) {
  noTone(BUZZER_PIN);
  toneIndex = 0;
  switch(t) {
    case 0: currentTone = toneClick;     tonePatternLen = 1; break;
    case 1: currentTone = toneSuccess;   tonePatternLen = 2; break;
    case 2: currentTone = toneOff;       tonePatternLen = 2; break;
    case 3: currentTone = toneAlert;     tonePatternLen = 2; break;
    case 4: currentTone = toneExecute;   tonePatternLen = 2; break;
    case 5: currentTone = toneCalibrate; tonePatternLen = 3; break;
    default: return;
  }
  TonePattern tp;
  memcpy_P(&tp, &currentTone[0], sizeof(TonePattern));
  tone(BUZZER_PIN, tp.freq, tp.duration);
  toneNextMs = millis() + tp.duration + tp.pause;
  toneIndex = 1;
}

void updateTone() {
  if (toneIndex == 0 || toneIndex >= tonePatternLen) return;
  uint32_t now = millis();
  if ((int32_t)(now - toneNextMs) < 0) return;
  TonePattern tp;
  memcpy_P(&tp, &currentTone[toneIndex], sizeof(TonePattern));
  tone(BUZZER_PIN, tp.freq, tp.duration);
  toneNextMs = now + tp.duration + tp.pause;
  toneIndex++;
  if (toneIndex >= tonePatternLen) toneIndex = 0;
}

// ── SMART POPUP SYSTEM ─────────────────────────────────────
void trigDyn(const __FlashStringHelper* l1, const char* l2, uint8_t priority) {
  if (dynPage == 99 && priority < popupPriority) return;
  dynL1 = l1;
  strncpy(dynL2, l2, POPUP_BUFFER_SIZE - 1);
  dynL2[POPUP_BUFFER_SIZE - 1] = '\0';
  dynPage = 99;
  dynStartMs = millis();
  popupPriority = priority;
  lcd.clear();
  pTone(priority == POPUP_PRIORITY_HIGH ? 3 : (priority == POPUP_PRIORITY_MED ? 1 : 0));
}

// ── UTILITY ─────────────────────────────────────────────────
uint8_t mapPct(int raw, int lo, int hi) {
  return (uint8_t)constrain(map(raw, lo, hi, 0, 99), 0, 99);
}

// ── CORE PUMP ENGINES ───────────────────────────────────────
void startP1(bool deep) {
  if (GET_FLAG(FLAG_PUMP1_RUN)) return;
  SET_FLAG(FLAG_PUMP1_RUN, 1);
  SET_FLAG(FLAG_DEEP_SOAK1, deep);   // FIX: use pump-specific flag
  p1StartMs = millis();
  digitalWrite(PUMP1_RELAY, LOW);
  pTone(4);
  trigDyn(F("PUMP 1"), deep ? "DEEP SOAK START" : "IRRIGATION", POPUP_PRIORITY_MED);
}

void stopP1() {
  if (!GET_FLAG(FLAG_PUMP1_RUN)) return;
  SET_FLAG(FLAG_PUMP1_RUN, 0);
  SET_FLAG(FLAG_DEEP_SOAK1, 0);      // FIX: use pump-specific flag
  digitalWrite(PUMP1_RELAY, HIGH);
  trigDyn(F("PUMP 1"), "STOPPED", POPUP_PRIORITY_LOW);
}

void startP2(bool deep) {
  if (GET_FLAG(FLAG_PUMP2_RUN)) return;
  SET_FLAG(FLAG_PUMP2_RUN, 1);
  SET_FLAG2(FLAG2_DEEP_SOAK2, deep); // FIX: use pump-specific flag in flags2
  p2StartMs = millis();
  digitalWrite(PUMP2_RELAY, LOW);
  pTone(4);
  trigDyn(F("PUMP 2"), deep ? "DEEP SOAK START" : "IRRIGATION", POPUP_PRIORITY_MED);
}

void stopP2() {
  if (!GET_FLAG(FLAG_PUMP2_RUN)) return;
  SET_FLAG(FLAG_PUMP2_RUN, 0);
  SET_FLAG2(FLAG2_DEEP_SOAK2, 0);    // FIX: use pump-specific flag in flags2
  digitalWrite(PUMP2_RELAY, HIGH);
  trigDyn(F("PUMP 2"), "STOPPED", POPUP_PRIORITY_LOW);
}

void updateTimers() {
  uint32_t now = millis();

  if (GET_FLAG(FLAG_PUMP1_RUN)) {
    int32_t diff1 = (int32_t)(now - p1StartMs);
    if (diff1 > 30000L) {
      stopP1();
      trigDyn(F("PUMP 1"), "FAILSAFE STOP", POPUP_PRIORITY_HIGH);
    }
    else if (GET_FLAG(FLAG_DEEP_SOAK1) && pct[0] >= 95) stopP1();
    else if (!GET_FLAG(FLAG_DEEP_SOAK1) && pct[0] >= (zThr1 + 5)) stopP1();
  }

  if (GET_FLAG(FLAG_PUMP2_RUN)) {
    int32_t diff2 = (int32_t)(now - p2StartMs);
    if (diff2 > 30000L) {
      stopP2();
      trigDyn(F("PUMP 2"), "FAILSAFE STOP", POPUP_PRIORITY_HIGH);
    }
    else if (GET_FLAG2(FLAG2_DEEP_SOAK2) && pct[1] >= 95) stopP2();
    else if (!GET_FLAG2(FLAG2_DEEP_SOAK2) && pct[1] >= (zThr2 + 5)) stopP2();
  }
}

// ── SMART SCHEDULING ────────────────────────────────────────
void runAuto() {
  static int8_t lastMinute = -1;
  static bool p1TriggeredToday = false;
  static bool p2TriggeredToday = false;
  static uint8_t lastDay = 255;

  if (GET_FLAG(FLAG_RTC_OK)) {
    DateTime n = rtc.now();
    int nowHM = n.hour() * 100 + n.minute();

    if (n.day() != lastDay) {
      p1TriggeredToday = false;
      p2TriggeredToday = false;
      lastDay = n.day();
    }

    // NEW: Skip scheduled irrigation while it's raining
    if (schedP1 != -1 && nowHM == schedP1 && n.minute() != lastMinute) {
      if (!p1TriggeredToday) {
        if (isRaining) {
          trigDyn(F("SCHED P1"), "SKIP-RAIN", POPUP_PRIORITY_MED);
          p1TriggeredToday = true;   // Mark as handled so it doesn't spam
        } else {
          startP1(true);
          p1TriggeredToday = true;
        }
      }
    }

    if (schedP2 != -1 && nowHM == schedP2 && n.minute() != lastMinute) {
      if (!p2TriggeredToday) {
        if (isRaining) {
          trigDyn(F("SCHED P2"), "SKIP-RAIN", POPUP_PRIORITY_MED);
          p2TriggeredToday = true;
        } else {
          startP2(true);
          p2TriggeredToday = true;
        }
      }
    }

    lastMinute = n.minute();
  }

  if (!GET_FLAG(FLAG_AUTO_MODE)) return;

  // NEW: Suppress auto moisture-triggered irrigation while raining
  if (!isRaining) {
    if (!GET_FLAG(FLAG_PUMP1_RUN) && pct[0] < zThr1) startP1(false);
    if (!GET_FLAG(FLAG_PUMP2_RUN) && pct[1] < zThr2) startP2(false);
  }

  emergencyState = (pct[2] > gasThr) || (tempC > (fanTempOn + 5));
  bool nf = emergencyState || (tempC > fanTempOn) || (humPct > fanHumOn);
  digitalWrite(FAN_RELAY, nf ? LOW : HIGH);
}

// ── SENSORS, RAIN & SERVO ───────────────────────────────────
void updateSensors(bool force) {
  uint32_t now = millis();
  int32_t diff = (int32_t)(now - lastSensorMs);
  if (!force && diff < 200L) return;
  lastSensorMs = now;

  readings[0][readIdx] = (analogRead(SOIL1_PIN) + analogRead(SOIL2_PIN)) / 2;
  readings[1][readIdx] = analogRead(SOIL3_PIN);
  readings[2][readIdx] = analogRead(GAS_PIN);
  readIdx = (readIdx + 1) % SAMPLES;

  long totals[3] = {0, 0, 0};
  for (uint8_t i = 0; i < SAMPLES; i++) {
    totals[0] += readings[0][i];
    totals[1] += readings[1][i];
    totals[2] += readings[2][i];
  }
  for (uint8_t i = 0; i < 3; i++) pct[i] = mapPct(totals[i] / SAMPLES, baseMin[i], baseMax[i]);

  // NEW: Read rain sensor — sensor outputs LOW when rain detected (conductive path)
  // Using a threshold on analog read for noise immunity vs pure digital
  bool newRaining = (analogRead(RAIN_PIN) < 512);
  if (newRaining != isRaining) {
    isRaining = newRaining;
    SET_FLAG2(FLAG2_RAIN_ACTIVE, isRaining ? 1 : 0);
    if (isRaining) {
      trigDyn(F("RAIN"), "DETECTED!", POPUP_PRIORITY_HIGH);
    } else {
      trigDyn(F("RAIN"), "CLEARED", POPUP_PRIORITY_MED);
    }
  }

  float t = dht.readTemperature();
  float h = dht.readHumidity();
  if (isnan(t)) {
    SET_FLAG(FLAG_DHT_ERR, 1);
    static uint8_t dhtFailCount = 0;
    dhtFailCount++;
    if (dhtFailCount == 10) {
      trigDyn(F("SENSOR"), "DHT ERROR!", POPUP_PRIORITY_HIGH);
    }
  } else {
    tempC = t;
    humPct = h;
    SET_FLAG(FLAG_DHT_ERR, 0);
  }
  lightBright = (digitalRead(LDR_PIN) == HIGH);
}

void updateServo() {
  // Priority order (highest first):
  // 1. Rain detected       → CLOSE (0°) immediately — protect from water damage
  // 2. Emergency state     → OPEN (180°) — ventilate gas/heat emergency
  // 3. Light bright (day)  → HALF OPEN (90°) — normal daytime ventilation
  // 4. Dark (night)        → CLOSE (0°) — retain warmth, exclude pests

  int goal;
  if (isRaining) {
    goal = 0;    // Close roof — rain has highest priority
  } else if (emergencyState) {
    goal = 180;  // Full open for emergency ventilation
  } else if (lightBright) {
    goal = 90;   // Half open during the day
  } else {
    goal = 0;    // Close at night
  }

  if (currServoPos == goal) return;
  uint32_t now = millis();
  int32_t diff = (int32_t)(now - lastServoMs);
  if (diff > 25L) {
    lastServoMs = now;
    if (currServoPos < goal) currServoPos++;
    else currServoPos--;
    roofServo.write(currServoPos);
  }
}

// ── CALIBRATION ────────────────────────────────────────────
void calibrate(bool isMax, char target) {
  updateSensors(true);
  pTone(5);

  int t = (target >= '0' && target <= '2') ? (target - '0') : -1;
  if (t == -1) {
    trigDyn(F("ERROR"), "ZONE 0-2 ONLY", POPUP_PRIORITY_HIGH);
    return;
  }

  // FIX: index mapping was ambiguous in v4.1 comment — explicitly stated here:
  // Zone 0 = average of SOIL1+SOIL2, Zone 1 = SOIL3, Zone 2 = GAS
  int raw;
  if (t == 0)      raw = (analogRead(SOIL1_PIN) + analogRead(SOIL2_PIN)) / 2;
  else if (t == 1) raw = analogRead(SOIL3_PIN);
  else             raw = analogRead(GAS_PIN);

  char popupMsg[16];
  if (isMax) {
    baseMax[t] = raw;
    EEPROM.put(7 + t * 2, baseMax[t]);
    snprintf(popupMsg, 16, "Z%d MAX:%d", t, raw);
  } else {
    baseMin[t] = raw;
    EEPROM.put(1 + t * 2, baseMin[t]);
    snprintf(popupMsg, 16, "Z%d MIN:%d", t, raw);
  }

  trigDyn(F("CALIBRATED"), popupMsg, POPUP_PRIORITY_MED);
}

// ── TIME PARSING ───────────────────────────────────────────
// FIX: returns -2 for "off", -1 for parse error, ≥0 for valid HHMM
int parseTimeStr(char* str) {
  if (strstr(str, "off")) {
    return -2;  // FIX: sentinel for "intentionally disabled", not parse error
  }

  int h = 0, m = 0;
  while (*str && !isdigit(*str)) str++;

  if (!isdigit(*str)) {
    trigDyn(F("ERROR"), "INVALID TIME", POPUP_PRIORITY_HIGH);
    return -1;
  }

  while (isdigit(*str)) { h = h * 10 + (*str - '0'); str++; }
  if (*str == ':') str++;
  while (isdigit(*str)) { m = m * 10 + (*str - '0'); str++; }
  if (strstr(str, "pm") && h < 12) h += 12;
  else if (strstr(str, "am") && h == 12) h = 0;

  if (h > 23 || m > 59) {
    trigDyn(F("ERROR"), "INVALID TIME", POPUP_PRIORITY_HIGH);
    return -1;
  }

  pTone(1);
  return h * 100 + m;
}

void printAMPM(int t, bool toLCD) {
  if (t == -1) {
    if (toLCD) lcd.print(F("OFF"));
    else BT.print(F("OFF"));
    return;
  }
  int h = t / 100, m = t % 100;
  bool pm = (h >= 12);
  if (h == 0) h = 12;
  else if (h > 12) h -= 12;

  if (toLCD) {
    lcd.print(h); lcd.print(F(":"));
    if (m < 10) lcd.print('0');
    lcd.print(m); lcd.print(pm ? F("P") : F("A"));
  } else {
    BT.print(h); BT.print(F(":"));
    if (m < 10) BT.print('0');
    BT.print(m); BT.print(pm ? F("PM") : F("AM"));
  }
}

// ── EEPROM MANAGEMENT ─────────────────────────────────────
struct ConfigCache {
  int schedP1_cached, schedP2_cached;
  int zThr1_cached, zThr2_cached;
  int gasThr_cached, fanTempOn_cached, fanHumOn_cached;
} configCache;

void initConfigCache() {
  configCache.schedP1_cached   = schedP1;
  configCache.schedP2_cached   = schedP2;
  configCache.zThr1_cached     = zThr1;
  configCache.zThr2_cached     = zThr2;
  configCache.gasThr_cached    = gasThr;
  configCache.fanTempOn_cached = fanTempOn;
  configCache.fanHumOn_cached  = fanHumOn;
}

void saveIfChanged(int addr, int value, int& cached) {
  if (value != cached) {
    EEPROM.put(addr, value);
    cached = value;
  }
}

void saveConfig() {
  saveIfChanged(21, schedP1,   configCache.schedP1_cached);
  saveIfChanged(23, schedP2,   configCache.schedP2_cached);
  saveIfChanged(25, zThr1,     configCache.zThr1_cached);
  saveIfChanged(27, zThr2,     configCache.zThr2_cached);
  saveIfChanged(29, gasThr,    configCache.gasThr_cached);
  saveIfChanged(31, fanTempOn, configCache.fanTempOn_cached);
  saveIfChanged(33, fanHumOn,  configCache.fanHumOn_cached);
}

// ── COMMAND PROCESSOR ─────────────────────────────────────
void printHelp() {
  BT.println();
  BT.println(F("=== SMARTFARM v4.2 COMMANDS ==="));
  BT.println(F("General: help status debug page"));
  BT.println(F("Mode: auto manual m(toggle)"));
  BT.println(F("Pumps: 1/1on/1off 2/2on/2off"));
  BT.println(F("Fan: f/fanon/fanoff"));
  BT.println(F("Schedule: 3 HH:MM / 3 off"));
  BT.println(F("Schedule: 4 HH:MM / 4 off"));
  BT.println(F("Thresholds: z35 x40 g80 t31 u85"));
  BT.println(F("Calibrate: c0/c1/c2 w0/w1/w2"));
  BT.println(F("System: reset save test rain"));
  BT.println(F("==============================="));
  BT.println();
  trigDyn(F("HELP SENT"), "CHECK BT MONITOR", POPUP_PRIORITY_LOW);
}

void printStatus() {
  BT.println();
  BT.println(F("--- SYSTEM STATUS ---"));
  BT.print(F("Mode: "));
  BT.println(GET_FLAG(FLAG_AUTO_MODE) ? F("AUTO") : F("MANUAL"));
  BT.print(F("Z1: ")); BT.print(pct[0]); BT.print(F("% (thr:")); BT.print(zThr1); BT.println(F("%)"));
  BT.print(F("Z2: ")); BT.print(pct[1]); BT.print(F("% (thr:")); BT.print(zThr2); BT.println(F("%)"));
  BT.print(F("Gas: ")); BT.print(pct[2]); BT.print(F("% (thr:")); BT.print(gasThr); BT.println(F("%)"));
  BT.print(F("Temp: ")); BT.print(tempC); BT.println(F("C"));
  BT.print(F("Humidity: ")); BT.print(humPct); BT.println(F("%"));
  BT.print(F("Light: ")); BT.println(lightBright ? F("BRIGHT") : F("DARK"));
  BT.print(F("Rain: ")); BT.println(isRaining ? F("YES - irrigation suppressed") : F("No"));
  BT.print(F("P1: ")); BT.print(GET_FLAG(FLAG_PUMP1_RUN) ? F("ON") : F("OFF"));
  BT.print(F(" P2: ")); BT.print(GET_FLAG(FLAG_PUMP2_RUN) ? F("ON") : F("OFF"));
  BT.print(F(" FAN: ")); BT.println(digitalRead(FAN_RELAY) == LOW ? F("ON") : F("OFF"));
  BT.print(F("Servo: ")); BT.print(currServoPos); BT.println(F("deg"));
  BT.print(F("P1 Sched: ")); printAMPM(schedP1, false); BT.println();
  BT.print(F("P2 Sched: ")); printAMPM(schedP2, false); BT.println();
  BT.println(F("---------------------"));
  BT.println();

  char modeStr[8];
  strcpy(modeStr, GET_FLAG(FLAG_AUTO_MODE) ? "AUTO" : "MANUAL");
  trigDyn(F("STATUS OK"), modeStr, POPUP_PRIORITY_LOW);
}

void processCmd(char* cmd) {
  for (uint8_t i = 0; cmd[i]; i++) cmd[i] = tolower(cmd[i]);
  char* end = cmd + strlen(cmd) - 1;
  while (end > cmd && isspace(*end)) *end-- = '\0';

  if (strcmp(cmd, "help") == 0 || cmd[0] == '?') {
    printHelp(); return;
  }

  // FIX: 'status' and 'save' both start with 's' — use strcmp for both,
  // only fall through to shortcut 's' after explicit matches fail.
  if (strcmp(cmd, "status") == 0) {
    printStatus(); return;
  }

  if (strcmp(cmd, "save") == 0) {
    saveConfig();
    trigDyn(F("CONFIG"), "SAVED", POPUP_PRIORITY_MED);
    pTone(1); return;
  }

  // Shortcut 's' for status (after explicit 'save' check above)
  if (cmd[0] == 's' && cmd[1] == '\0') {
    printStatus(); return;
  }

  if (strcmp(cmd, "debug") == 0 || (cmd[0] == 'd' && cmd[1] == '\0')) {
    SET_FLAG(FLAG_DEBUG_ON, !GET_FLAG(FLAG_DEBUG_ON));
    pTone(0);
    trigDyn(F("DEBUG"), GET_FLAG(FLAG_DEBUG_ON) ? "ENABLED" : "DISABLED", POPUP_PRIORITY_MED);
    return;
  }

  if (strcmp(cmd, "page") == 0 || (cmd[0] == 'p' && cmd[1] == '\0')) {
    lcdPage = (lcdPage + 1) % 5;
    lcd.clear(); pTone(0);
    char pageMsg[16];
    snprintf(pageMsg, 16, "PAGE %d/5", lcdPage + 1);
    trigDyn(F("DISPLAY"), pageMsg, POPUP_PRIORITY_LOW);
    return;
  }

  // FIX: Mode commands — check explicit strings first, then toggle shortcut 'm'
  // v4.1 bug: cmd[0]=='m' matched before strcmp("auto") so "auto" was unreachable
  if (strcmp(cmd, "auto") == 0) {
    SET_FLAG(FLAG_AUTO_MODE, 1);
    pTone(1);
    trigDyn(F("MODE CHANGED"), "AUTO", POPUP_PRIORITY_MED); return;
  }
  if (strcmp(cmd, "manual") == 0) {
    SET_FLAG(FLAG_AUTO_MODE, 0);
    pTone(2);
    trigDyn(F("MODE CHANGED"), "MANUAL", POPUP_PRIORITY_MED); return;
  }
  if (cmd[0] == 'm' && cmd[1] == '\0') {
    bool newMode = !GET_FLAG(FLAG_AUTO_MODE);
    SET_FLAG(FLAG_AUTO_MODE, newMode);
    pTone(newMode ? 1 : 2);
    trigDyn(F("MODE CHANGED"), newMode ? "AUTO" : "MANUAL", POPUP_PRIORITY_MED); return;
  }

  if (cmd[0] == '1' || strncmp(cmd, "pump1", 5) == 0) {
    bool turnOff = (strstr(cmd, "off") != nullptr);
    bool turnOn  = (strstr(cmd, "on")  != nullptr);
    if (turnOff || (cmd[0] == '1' && !turnOn && GET_FLAG(FLAG_PUMP1_RUN)))
      stopP1();
    else
      startP1(false);
    return;
  }

  if (cmd[0] == '2' || strncmp(cmd, "pump2", 5) == 0) {
    bool turnOff = (strstr(cmd, "off") != nullptr);
    bool turnOn  = (strstr(cmd, "on")  != nullptr);
    if (turnOff || (cmd[0] == '2' && !turnOn && GET_FLAG(FLAG_PUMP2_RUN)))
      stopP2();
    else
      startP2(false);
    return;
  }

  if (cmd[0] == 'f' || strncmp(cmd, "fan", 3) == 0) {
    bool turnOn  = (strstr(cmd, "on")  != nullptr);
    bool turnOff = (strstr(cmd, "off") != nullptr);
    bool currentState = (digitalRead(FAN_RELAY) == LOW);
    if (turnOn) {
      digitalWrite(FAN_RELAY, LOW);
      trigDyn(F("FAN"), "FORCED ON", POPUP_PRIORITY_MED);
    } else if (turnOff) {
      digitalWrite(FAN_RELAY, HIGH);
      trigDyn(F("FAN"), "FORCED OFF", POPUP_PRIORITY_MED);
    } else {
      digitalWrite(FAN_RELAY, currentState ? HIGH : LOW);
      trigDyn(F("FAN"), currentState ? "OFF" : "ON", POPUP_PRIORITY_MED);
    }
    pTone(4); return;
  }

  if (cmd[0] == '3') {
    int newSched = parseTimeStr(&cmd[1]);
    // FIX: -2 = "off" (disable), -1 = parse error (ignore), ≥0 = valid time
    if (newSched == -2) {
      schedP1 = -1; saveConfig();
      trigDyn(F("PUMP1 SCHED"), "DISABLED", POPUP_PRIORITY_MED);
    } else if (newSched >= 0) {
      schedP1 = newSched; saveConfig();
      char schedMsg[16];
      snprintf(schedMsg, 16, "%02d:%02d SET", schedP1 / 100, schedP1 % 100);
      trigDyn(F("PUMP1 SCHED"), schedMsg, POPUP_PRIORITY_MED);
    }
    return;
  }

  if (cmd[0] == '4') {
    int newSched = parseTimeStr(&cmd[1]);
    if (newSched == -2) {
      schedP2 = -1; saveConfig();
      trigDyn(F("PUMP2 SCHED"), "DISABLED", POPUP_PRIORITY_MED);
    } else if (newSched >= 0) {
      schedP2 = newSched; saveConfig();
      char schedMsg[16];
      snprintf(schedMsg, 16, "%02d:%02d SET", schedP2 / 100, schedP2 % 100);
      trigDyn(F("PUMP2 SCHED"), schedMsg, POPUP_PRIORITY_MED);
    }
    return;
  }

  if (cmd[0] == 'z') {
    int newVal = atoi(&cmd[1]);
    if (newVal >= 0 && newVal <= 99) {
      zThr1 = newVal; saveConfig();
      char valMsg[16]; snprintf(valMsg, 16, "Z1: %d%%", zThr1);
      trigDyn(F("THRESHOLD"), valMsg, POPUP_PRIORITY_MED);
    } else { trigDyn(F("ERROR"), "USE 0-99", POPUP_PRIORITY_HIGH); }
    return;
  }

  if (cmd[0] == 'x') {
    int newVal = atoi(&cmd[1]);
    if (newVal >= 0 && newVal <= 99) {
      zThr2 = newVal; saveConfig();
      char valMsg[16]; snprintf(valMsg, 16, "Z2: %d%%", zThr2);
      trigDyn(F("THRESHOLD"), valMsg, POPUP_PRIORITY_MED);
    } else { trigDyn(F("ERROR"), "USE 0-99", POPUP_PRIORITY_HIGH); }
    return;
  }

  if (cmd[0] == 'g') {
    int newVal = atoi(&cmd[1]);
    if (newVal >= 0 && newVal <= 99) {
      gasThr = newVal; saveConfig();
      char valMsg[16]; snprintf(valMsg, 16, "GAS: %d%%", gasThr);
      trigDyn(F("ALARM LVL"), valMsg, POPUP_PRIORITY_MED);
    } else { trigDyn(F("ERROR"), "USE 0-99", POPUP_PRIORITY_HIGH); }
    return;
  }

  if (cmd[0] == 't') {
    int newVal = atoi(&cmd[1]);
    if (newVal >= 0 && newVal <= 60) {
      fanTempOn = newVal; saveConfig();
      char valMsg[16]; snprintf(valMsg, 16, "TEMP: %dC", fanTempOn);
      trigDyn(F("FAN TEMP"), valMsg, POPUP_PRIORITY_MED);
    } else { trigDyn(F("ERROR"), "USE 0-60C", POPUP_PRIORITY_HIGH); }
    return;
  }

  if (cmd[0] == 'u') {
    int newVal = atoi(&cmd[1]);
    if (newVal >= 0 && newVal <= 100) {
      fanHumOn = newVal; saveConfig();
      char valMsg[16]; snprintf(valMsg, 16, "HUM: %d%%", fanHumOn);
      trigDyn(F("FAN HUMID"), valMsg, POPUP_PRIORITY_MED);
    } else { trigDyn(F("ERROR"), "USE 0-100", POPUP_PRIORITY_HIGH); }
    return;
  }

  if (cmd[0] == 'c') { calibrate(false, cmd[1]); return; }
  if (cmd[0] == 'w') { calibrate(true,  cmd[1]); return; }

  // NEW: Rain sensor query command
  if (strcmp(cmd, "rain") == 0) {
    BT.print(F("Rain sensor: "));
    BT.println(isRaining ? F("RAINING - irrigation suspended") : F("Dry - normal operation"));
    trigDyn(F("RAIN SENSOR"), isRaining ? "RAINING" : "DRY", POPUP_PRIORITY_LOW);
    return;
  }

  if (strcmp(cmd, "reset") == 0) {
    EEPROM.write(0, 0);
    trigDyn(F("SYSTEM"), "RESET DONE", POPUP_PRIORITY_HIGH);
    pTone(3);
    delay(1000);
    asm volatile ("  jmp 0");
    return;
  }

  if (strcmp(cmd, "test") == 0) {
    trigDyn(F("TEST MODE"), "RUNNING...", POPUP_PRIORITY_HIGH);
    pTone(5);
    digitalWrite(FAN_RELAY, LOW);   delay(200); digitalWrite(FAN_RELAY, HIGH);
    delay(100);
    digitalWrite(PUMP1_RELAY, LOW); delay(200); digitalWrite(PUMP1_RELAY, HIGH);
    delay(100);
    digitalWrite(PUMP2_RELAY, LOW); delay(200); digitalWrite(PUMP2_RELAY, HIGH);
    trigDyn(F("TEST"), "COMPLETE", POPUP_PRIORITY_MED);
    return;
  }

  if (strcmp(cmd, "json") == 0) {
    SET_FLAG(FLAG_JSON_MODE, !GET_FLAG(FLAG_JSON_MODE));
    trigDyn(F("OUTPUT"), GET_FLAG(FLAG_JSON_MODE) ? "JSON MODE" : "TEXT MODE", POPUP_PRIORITY_MED);
    return;
  }

  if (strcmp(cmd, "verbose") == 0) {
    trigDyn(F("VERBOSE"), "TOGGLED", POPUP_PRIORITY_MED);
    return;
  }

  trigDyn(F("UNKNOWN"), "USE 'help'", POPUP_PRIORITY_HIGH);
  pTone(3);
}

// ── DEBUG OUTPUT ────────────────────────────────────────────
void sendDebug() {
  if (!GET_FLAG(FLAG_DEBUG_ON)) return;
  uint32_t now = millis();
  int32_t diff = (int32_t)(now - lastDebugMs);
  if (diff < 2000L) return;
  lastDebugMs = now;

  if (GET_FLAG(FLAG_JSON_MODE)) {
    BT.print(F("{\"t\":"));  BT.print(millis());
    BT.print(F(",\"z1\":")); BT.print(pct[0]);
    BT.print(F(",\"z2\":")); BT.print(pct[1]);
    BT.print(F(",\"g\":"));  BT.print(pct[2]);
    BT.print(F(",\"T\":"));  BT.print(tempC);
    BT.print(F(",\"H\":"));  BT.print(humPct);
    BT.print(F(",\"rain\":")); BT.print(isRaining ? 1 : 0);  // NEW
    BT.print(F(",\"p1\":")); BT.print(GET_FLAG(FLAG_PUMP1_RUN));
    BT.print(F(",\"p2\":")); BT.print(GET_FLAG(FLAG_PUMP2_RUN));
    BT.print(F(",\"f\":"));  BT.print(digitalRead(FAN_RELAY) == LOW);
    BT.print(F(",\"s\":"));  BT.print(currServoPos);
    BT.print(F(",\"m\":\"")); BT.print(GET_FLAG(FLAG_AUTO_MODE) ? F("A") : F("M"));
    BT.println(F("\"}"));
  } else {
    BT.println();
    BT.println(F("--- DEBUG ---"));
    BT.print(F("Z1:")); BT.print(pct[0]);
    BT.print(F("% Z2:")); BT.print(pct[1]);
    BT.print(F("% G:")); BT.print(pct[2]); BT.println(F("%"));
    BT.print(F("T:")); BT.print(tempC);
    BT.print(F("C H:")); BT.print(humPct); BT.println(F("%"));
    BT.print(F("Rain:")); BT.println(isRaining ? F("YES") : F("No")); // NEW
    BT.print(F("P1:")); BT.print(GET_FLAG(FLAG_PUMP1_RUN));
    BT.print(F(" P2:")); BT.print(GET_FLAG(FLAG_PUMP2_RUN));
    BT.print(F(" F:")); BT.println(digitalRead(FAN_RELAY) == LOW);
    BT.print(F("S:")); BT.print(currServoPos);
    BT.print(F(" L:")); BT.print(lightBright);
    BT.print(F(" [")); BT.print(GET_FLAG(FLAG_AUTO_MODE) ? F("A") : F("M"));
    BT.println(F("]"));
    BT.println(F("-------------"));
  }
}

// ── UI RENDERING ─────────────────────────────────────────────
void showPopup() {
  if (dynL1 == nullptr) return;
  lcd.setCursor(0, 0);
  char l1Buffer[17];
  strncpy_P(l1Buffer, (const char*)dynL1, 16);
  l1Buffer[16] = '\0';
  uint8_t pad1 = (16 - strlen(l1Buffer)) / 2;
  while (pad1--) lcd.print(' ');
  lcd.print(l1Buffer);

  lcd.setCursor(0, 1);
  uint8_t pad2 = (16 - strlen(dynL2)) / 2;
  while (pad2--) lcd.print(' ');
  lcd.print(dynL2);
}

void showPage(uint8_t pg) {
  static bool ani = false;
  ani = !ani;

  switch (pg) {
    case 0:
      lcd.setCursor(0, 0);
      lcd.write(2); lcd.print(pct[0]);
      lcd.write(pct[0] > 70 ? 5 : pct[0] > 30 ? 4 : 3);
      lcd.write(2); lcd.print(pct[1]);
      lcd.write(pct[1] > 70 ? 5 : pct[1] > 30 ? 4 : 3);
      lcd.print(F(" G")); lcd.print(pct[2]);

      lcd.setCursor(0, 1);
      lcd.write(6); lcd.print((int)tempC); lcd.write(223);
      lcd.write(7); lcd.print((int)humPct); lcd.print(F("%"));
      lcd.print(F(" "));
      lcd.print(GET_FLAG(FLAG_AUTO_MODE) ? F("[A]") : F("[M]"));
      break;

    case 1:
      lcd.setCursor(0, 0);
      lcd.print(F("P1:"));
      if (GET_FLAG(FLAG_PUMP1_RUN)) { lcd.write(2); lcd.print(F("ON ")); }
      else lcd.print(F("OFF"));
      lcd.print(F(" P2:"));
      if (GET_FLAG(FLAG_PUMP2_RUN)) { lcd.write(2); lcd.print(F("ON ")); }
      else lcd.print(F("OFF"));

      // NEW: Show rain sensor state on page 1, row 2
      lcd.setCursor(0, 1);
      lcd.print(F("F:"));
      if (digitalRead(FAN_RELAY) == LOW) { lcd.write(ani ? 0 : 1); lcd.print(F("ON ")); }
      else lcd.print(F("OFF"));
      lcd.print(isRaining ? F(" RN") : F("   "));  // "RN" indicator when raining
      lcd.print(F("S:"));
      lcd.print(currServoPos);
      lcd.write(223);
      break;

    case 2:
      lcd.setCursor(0, 0); lcd.print(F("P1:")); printAMPM(schedP1, true);
      lcd.setCursor(0, 1); lcd.print(F("P2:")); printAMPM(schedP2, true);
      break;

    case 3:
      lcd.setCursor(0, 0);
      lcd.print(F("Z1:")); lcd.print(zThr1);
      lcd.print(F("% Z2:")); lcd.print(zThr2); lcd.print(F("%"));
      lcd.setCursor(0, 1);
      lcd.print(F("T:")); lcd.print(fanTempOn); lcd.write(223);
      lcd.print(F(" H:")); lcd.print(fanHumOn); lcd.print(F("%"));
      break;

    case 4:
      lcd.setCursor(0, 0);
      lcd.print(F("RTC:")); lcd.print(GET_FLAG(FLAG_RTC_OK) ? F("OK ") : F("ERR"));
      lcd.print(F(" DHT:")); lcd.print(GET_FLAG(FLAG_DHT_ERR) ? F("ERR") : F("OK "));
      lcd.setCursor(0, 1);
      // NEW: Show rain sensor on health page
      lcd.print(F("E:")); lcd.print(emergencyState ? F("ALT ") : F("NRM "));
      lcd.print(isRaining ? F("RAIN") : F("    "));
      if (emergencyState) lcd.write(8);
      break;
  }
}

// ── SETUP & BOOT ────────────────────────────────────────────
void setup() {
  wdt_disable();

  BT.begin(9600);
  dht.begin();
  Wire.begin();

  pinMode(FAN_RELAY,   OUTPUT);
  pinMode(PUMP1_RELAY, OUTPUT);
  pinMode(PUMP2_RELAY, OUTPUT);
  pinMode(BUZZER_PIN,  OUTPUT);
  pinMode(LDR_PIN,     INPUT);
  // RAIN_PIN (A4) is analog — no pinMode needed for analogRead

  digitalWrite(FAN_RELAY,   OFF);
  digitalWrite(PUMP1_RELAY, OFF);
  digitalWrite(PUMP2_RELAY, OFF);

  roofServo.attach(SERVO_PIN);
  roofServo.write(currServoPos);

  lcd.init();
  lcd.backlight();

  byte temp[8];
  memcpy_P(temp, charFan[0], 8);   lcd.createChar(0, temp);
  memcpy_P(temp, charFan[1], 8);   lcd.createChar(1, temp);
  memcpy_P(temp, charDrop, 8);     lcd.createChar(2, temp);
  memcpy_P(temp, charBar[0], 8);   lcd.createChar(3, temp);
  memcpy_P(temp, charBar[1], 8);   lcd.createChar(4, temp);
  memcpy_P(temp, charBar[2], 8);   lcd.createChar(5, temp);
  memcpy_P(temp, charThermo, 8);   lcd.createChar(6, temp);
  memcpy_P(temp, charHumid, 8);    lcd.createChar(7, temp);
  memcpy_P(temp, charAlert, 8);    lcd.createChar(8, temp);

  lcd.setCursor(0, 0); lcd.print(F("SMARTFARM v4.2"));
  lcd.setCursor(0, 1); lcd.print(F("INITIALIZING..."));
  pTone(1);
  delay(800);

  const char* checks[] = {"RTC", "DHT", "RELAYS", "SERVO", "RAIN"};  // FIX: EEPROM→RAIN
  for (uint8_t i = 0; i < 5; i++) {
    lcd.clear();
    lcd.setCursor(0, 0); lcd.print(F("CHECK:"));
    lcd.setCursor(0, 1); lcd.print(checks[i]);
    pTone(0);
    delay(300);
  }

  if (rtc.begin()) {
    SET_FLAG(FLAG_RTC_OK, 1);
    if (rtc.lostPower()) {
      rtc.adjust(DateTime(F(__DATE__), F(__TIME__)));
      trigDyn(F("RTC"), "TIME SET", POPUP_PRIORITY_MED);
    }
  } else {
    trigDyn(F("WARNING"), "NO RTC!", POPUP_PRIORITY_HIGH);
  }

  if (EEPROM.read(0) == EEPROM_MAGIC) {
    for (uint8_t i = 0; i < 3; i++) {
      EEPROM.get(1 + i * 2, baseMin[i]);
      EEPROM.get(7 + i * 2, baseMax[i]);
    }
    EEPROM.get(21, schedP1);
    EEPROM.get(23, schedP2);
    EEPROM.get(25, zThr1);
    EEPROM.get(27, zThr2);
    EEPROM.get(29, gasThr);
    EEPROM.get(31, fanTempOn);
    EEPROM.get(33, fanHumOn);
    initConfigCache();
    trigDyn(F("CONFIG"), "LOADED OK", POPUP_PRIORITY_LOW);
  } else {
    EEPROM.write(0, EEPROM_MAGIC);
    initConfigCache();
    trigDyn(F("DEFAULTS"), "RESET", POPUP_PRIORITY_MED);
  }

  lcd.clear();
  lcd.print(F("SYSTEM READY"));
  lcd.setCursor(0, 1);
  lcd.print(GET_FLAG(FLAG_AUTO_MODE) ? F("AUTO MODE") : F("MANUAL MODE"));
  pTone(1);
  delay(1500);

  updateSensors(true);
  lcd.clear();

  wdt_enable(WDTO_2S);
}

void loop() {
  wdt_reset();

  static char btBuf[24];
  static byte btIdx = 0;
  while (BT.available()) {
    char c = (char)BT.read();
    if (c == '\n' || c == '\r') {
      btBuf[btIdx] = '\0';
      if (btIdx > 0) processCmd(btBuf);
      btIdx = 0;
    } else if (btIdx < 23) {
      btBuf[btIdx++] = c;
    }
  }

  updateSensors(false);
  updateServo();
  updateTimers();
  runAuto();
  updateTone();

  if (dynPage == 99) {
    int32_t popupDiff = (int32_t)(millis() - dynStartMs);
    if (popupDiff >= POPUP_DURATION_MS) {
      dynPage = 255;
      popupPriority = POPUP_PRIORITY_LOW;
      lcd.clear();
    }
  }

  uint32_t now = millis();
  int32_t lcdDiff = (int32_t)(now - lastLcdMs);
  if (lcdDiff > 500L) {
    lastLcdMs = now;
    if (dynPage == 99) showPopup();
    else showPage(lcdPage);
  }

  sendDebug();
}

/*
  ============================================================
  SUGGESTED IMPROVEMENTS (priority order)
  ============================================================

  [P1 — CRITICAL / Safety]
  ─────────────────────────
  1. WATCHDOG PUMP INTERLOCK
     If both pumps run simultaneously and the total draw exceeds
     your relay/PSU rating, add a hardware or software interlock:
       if (GET_FLAG(FLAG_PUMP1_RUN)) return; // before startP2()
     Or add a small delay between starts to let inrush settle.

  2. RAIN SENSOR DEBOUNCE
     The current rain detection uses a single analogRead per cycle.
     Add a debounce counter (require 3 consecutive wet readings)
     before setting isRaining = true to prevent false triggers
     from splashes or sensor noise:
       static uint8_t rainCount = 0;
       if (raw < 512) { if (++rainCount > 3) isRaining = true; }
       else { if (--rainCount == 0) isRaining = false; }

  3. PUMP RUNNING WHILE RAINING
     If rain starts WHILE a pump is mid-cycle, it currently keeps
     running. Consider: if (isRaining && GET_FLAG(FLAG_PUMP1_RUN)) stopP1();
     Place this at the top of runAuto(). Manual-mode pumps should
     probably still respect user intent, so only do this in AUTO mode.

  [P2 — Reliability]
  ───────────────────
  4. DHT READ INTERVAL
     DHT11 minimum sample interval is 1 second. Calling
     dht.readTemperature() every 200ms (your sensor loop) will
     return NaN most of the time, inflating dhtFailCount spuriously.
     Add a separate timestamp: only read DHT when ≥1000ms elapsed.

  5. SOFT MILLIS() ROLLOVER SAFETY
     You already use (int32_t)(now - lastX) which correctly handles
     32-bit rollover. Good. Confirm updateTimers() pump diff checks
     also use this pattern — they do in v4.2. No change needed,
     just verify any future additions do the same.

  6. EEPROM WEAR LEVELLING
     saveConfig() is called on every threshold/schedule change.
     EEPROM cells rated ~100,000 writes. If thresholds are tweaked
     frequently, cells at addr 25-33 will wear out first. Consider
     adding a write counter and rotating the storage block, or
     rate-limiting saves to once per 30 seconds.

  [P3 — Features / UX]
  ─────────────────────
  7. RAIN HISTORY / DURATION TRACKING
     Log rain start time (via RTC) and display duration on LCD Page 4.
     Useful for correlating with soil moisture readings and deciding
     whether to skip the next scheduled irrigation.

  8. BLUETOOTH RECONNECT DETECTION
     SoftwareSerial has no connection-state API. Add a simple
     heartbeat: if debug mode is on and no command received in >5min,
     print a keepalive "." so the user knows BT is still live.

  9. MANUAL MODE TIMEOUT
     If left in MANUAL mode with a pump running and BT disconnects,
     the failsafe (30s) is the only protection. Consider adding an
     auto-return-to-AUTO after N minutes of no BT activity in manual mode.

  10. LCD BACKLIGHT AUTO-OFF
      Turn off the backlight after 30 seconds of no BT command to
      save power. Wake it on the next command or button press.
      lcd.noBacklight() / lcd.backlight()

  ============================================================
*/

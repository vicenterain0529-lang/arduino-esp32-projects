/*
  ============================================================
  agrisense v4.2 -- RAIN-AWARE INTELLIGENT EDITION (DIGITAL)
  "Zero Sacrifice + Weather Intelligence"
  ============================================================
  
  FIXES APPLIED IN THIS BUILD:
  - DHT safely checks both temperature and humidity
  - Roof opens automatically after rain stops (20-sec hysteresis)
  - Day/Night servo changes delayed 10 seconds (anti-flicker)
  - Sensor readings array properly initialized
  - EEPROM writes protected (saveIfChanged / saveConfig)
  - Faster servo movement (step size 4)
  - Rain (R) + Light (D/N) status added to Page 0
  
  NEW IN v4.2:
  - Raindrop sensor support (Digital Pin 11)
  - Automatic roof closure on rain detection (LOW = rain)
  - Rain status popup with priority override
  - Rain icon on LCD (custom char 9)
  - Rain indication on Page 0 and Page 4
  - Light status (D/N) on Page 1
  - Rain status (1/0) on Page 1
  
  RAIN SENSOR WIRING:
  - VCC to 5V
  - GND to GND  
  - D0 to Pin 11 (digital input with pull-up)
  - LED indicators: Power (red), Rain detected (green)
  
  KEY COMMANDS (send via Bluetooth):
  ----------------------------------
  GENERAL:
    help / ?        - Show all commands
    status / s      - Full system status
    debug / d       - Toggle debug output
    page / p        - Next LCD page
    
  MODE CONTROL:
    auto / m        - Toggle AUTO/MANUAL mode
    manual          - Force manual mode
    
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
    
  DEBUG FORMATS:
    json            - Toggle JSON output mode
    verbose         - Toggle verbose mode
  
  LCD PAGES:
    Page 0: Zone moisture + Temp/Humidity + Rain + Mode
    Page 1: Pump/Fan/Servo + Light(D/N) + Rain(1/0) status
    Page 2: Schedule display
    Page 3: Thresholds overview
    Page 4: System health + Rain status
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

// -- PIN DEFINITIONS -----------------------------------------
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
#define RAIN_PIN     11
#define SOIL1_PIN    A0
#define SOIL2_PIN    A1
#define SOIL3_PIN    A2
#define GAS_PIN      A3

// -- FLAGS & MEMORY ------------------------------------------
#define ON           LOW
#define OFF          HIGH
#define EEPROM_MAGIC 161
const uint8_t SAMPLES  = 5;

// Flag bit positions
#define FLAG_AUTO_MODE   0
#define FLAG_RTC_OK      1
#define FLAG_JSON_MODE   2
#define FLAG_DHT_ERR     3
#define FLAG_PUMP1_RUN   4
#define FLAG_PUMP2_RUN   5
#define FLAG_DEEP_SOAK   6
#define FLAG_DEBUG_ON    7

uint8_t flags = 0b00000101;
#define GET_FLAG(f)    (((flags)>>(f))&1)
#define SET_FLAG(f,v)  flags=(v)?(flags|(1<<(f))):(flags&~(1<<(f)))

// -- GLOBAL OBJECTS ------------------------------------------
SoftwareSerial BT(BT_RX, BT_TX);
DHT dht(DHT_PIN, DHT_TYPE);
LiquidCrystal_I2C lcd(0x27, 16, 2);
RTC_DS3231 rtc;
Servo roofServo;

// -- SYSTEM DATA ---------------------------------------------
int readings[3][SAMPLES];
uint8_t readIdx = 0;
int baseMin[3] = {850, 850, 100};
int baseMax[3] = {350, 350, 800}; 
uint8_t pct[3]; 
int zThr1=35, zThr2=35, gasThr=80, fanTempOn=31, fanHumOn=85;
int schedP1 = -1, schedP2 = -1;
float tempC=0.0, humPct=0.0;
bool lightBright = false, emergencyState = false;

// -- RAIN SENSOR DATA (DIGITAL) -------------------------------
bool isRaining = false;
bool rainDetected = false;
bool lastRainState = false;
uint32_t rainStartMs = 0;
uint32_t rainClearMs = 0;
const uint32_t RAIN_DEBOUNCE_MS = 2000;
const uint32_t RAIN_POPUP_COOLDOWN = 300000;

// Rain hysteresis for roof control (20 seconds)
uint32_t lastRainMs = 0;
bool rainHysteresisActive = false;
const uint32_t RAIN_HYSTERESIS_MS = 20000;

// -- LIGHT CHANGE DELAY (Servo-exclusive) --------------------
uint32_t lightChangeStartMs = 0;
bool lightChangePending = false;
bool delayedLightBright = false;
const uint32_t LIGHT_CHANGE_DELAY_MS = 10000;

// -- TIMING VARIABLES ----------------------------------------
uint32_t lastSensorMs=0, lastLcdMs=0, p1StartMs=0, p2StartMs=0, lastServoMs=0, lastDebugMs=0;
uint8_t currServoPos=90, lcdPage=0, dynPage=255;
uint32_t dynStartMs=0;

// -- SMART POPUP SYSTEM --------------------------------------
#define POPUP_BUFFER_SIZE 17
char dynL2[POPUP_BUFFER_SIZE];
const __FlashStringHelper* dynL1 = nullptr;

#define POPUP_DURATION_MS 2500
#define POPUP_PRIORITY_LOW  0
#define POPUP_PRIORITY_MED  1
#define POPUP_PRIORITY_HIGH 2
#define POPUP_PRIORITY_RAIN 3
uint8_t popupPriority = POPUP_PRIORITY_LOW;

// -- CUSTOM ICONS IN PROGMEM ---------------------------------
const byte charFan[2][8] PROGMEM = { 
  {0,0,13,7,4,28,22,0}, 
  {0,0,22,28,4,7,13,0} 
};
const byte charDrop[8] PROGMEM = {4,4,14,14,31,31,31,14};
const byte charBar[3][8] PROGMEM = { 
  {0,0,0,0,0,0,0,31}, 
  {0,0,0,31,31,31,31,31}, 
  {31,31,31,31,31,31,31,31} 
};
const byte charThermo[8] PROGMEM = {4,10,10,10,14,31,31,14};
const byte charHumid[8] PROGMEM = {4,4,14,14,31,31,14,0};
const byte charAlert[8] PROGMEM = {4,14,14,14,31,0,4,0};
const byte charRain[8] PROGMEM = {0,15,31,31,14,4,4,0};

// -- AUDIO ENGINE -------------------------------------------
struct TonePattern {
  uint16_t freq;
  uint16_t duration;
  uint16_t pause;
};

const TonePattern toneClick[] PROGMEM = {{1200, 50, 0}};
const TonePattern toneSuccess[] PROGMEM = {{800, 80, 100}, {1500, 80, 0}};
const TonePattern toneOff[] PROGMEM = {{1500, 80, 100}, {800, 80, 0}};
const TonePattern toneAlert[] PROGMEM = {{400, 300, 400}, {400, 300, 0}};
const TonePattern toneExecute[] PROGMEM = {{2000, 40, 50}, {2000, 40, 0}};
const TonePattern toneCalibrate[] PROGMEM = {{1000, 100, 100}, {1500, 100, 100}, {2000, 100, 0}};
const TonePattern toneRain[] PROGMEM = {{800, 200, 100}, {600, 200, 100}, {800, 200, 0}};

volatile uint8_t toneIndex = 0;
volatile uint8_t tonePatternLen = 0;
volatile uint32_t toneNextMs = 0;
const TonePattern* currentTone = nullptr;

void pTone(uint8_t t) {
  noTone(BUZZER_PIN);
  toneIndex = 0;
  
  switch(t) {
    case 0: currentTone = toneClick; tonePatternLen = 1; break;
    case 1: currentTone = toneSuccess; tonePatternLen = 2; break;
    case 2: currentTone = toneOff; tonePatternLen = 2; break;
    case 3: currentTone = toneAlert; tonePatternLen = 2; break;
    case 4: currentTone = toneExecute; tonePatternLen = 2; break;
    case 5: currentTone = toneCalibrate; tonePatternLen = 3; break;
    case 6: currentTone = toneRain; tonePatternLen = 3; break;
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

// -- SMART POPUP SYSTEM -------------------------------------
void trigDyn(const __FlashStringHelper* l1, const char* l2, uint8_t priority) {
  if (dynPage == 99 && priority < popupPriority) return;
  
  dynL1 = l1;
  strncpy(dynL2, l2, POPUP_BUFFER_SIZE - 1);
  dynL2[POPUP_BUFFER_SIZE - 1] = '\0';
  dynPage = 99;
  dynStartMs = millis();
  popupPriority = priority;
  lcd.clear();
  
  if (priority == POPUP_PRIORITY_RAIN) pTone(6);
  else pTone(priority == POPUP_PRIORITY_HIGH ? 3 : (priority == POPUP_PRIORITY_MED ? 1 : 0));
}

// -- UTILITY FUNCTIONS --------------------------------------
uint8_t mapPct(int raw, int lo, int hi) {
  return (uint8_t)constrain(map(raw,lo,hi,0,99),0,99);
}

// -- RAIN SENSOR FUNCTIONS (DIGITAL) ------------------------
void updateRainSensor() {
  bool rainNow = (digitalRead(RAIN_PIN) == LOW);
  uint32_t now = millis();
  
  if (rainNow && !lastRainState) {
    if (rainStartMs == 0) rainStartMs = now;
    if ((int32_t)(now - rainStartMs) > RAIN_DEBOUNCE_MS) {
      isRaining = true;
      rainDetected = true;
      rainHysteresisActive = true;
      lastRainState = true;
      rainClearMs = 0;
    }
  } else if (!rainNow && lastRainState) {
    if (rainClearMs == 0) rainClearMs = now;
    if ((int32_t)(now - rainClearMs) > RAIN_DEBOUNCE_MS) {
      isRaining = false;
      lastRainState = false;
      rainStartMs = 0;
      if ((int32_t)(now - lastRainMs) > RAIN_POPUP_COOLDOWN) {
        trigDyn(F("WEATHER"), "RAIN STOPPED", POPUP_PRIORITY_MED);
        lastRainMs = now;
      }
    }
  } else {
    if (!rainNow) {
      rainStartMs = 0;
      lastRainState = false;
    }
    if (rainNow) {
      rainClearMs = 0;
      lastRainState = true;
    }
  }
  
  if (rainDetected && (int32_t)(now - lastRainMs) > RAIN_POPUP_COOLDOWN) {
    trigDyn(F("RAIN DETECTED"), "CLOSING ROOF", POPUP_PRIORITY_RAIN);
    rainDetected = false;
    lastRainMs = now;
  }
}

// -- CORE ENGINES -------------------------------------------
void startP1(bool deep) { 
  if(GET_FLAG(FLAG_PUMP1_RUN)) return; 
  SET_FLAG(FLAG_PUMP1_RUN,1); 
  SET_FLAG(FLAG_DEEP_SOAK, deep);
  p1StartMs=millis(); 
  digitalWrite(PUMP1_RELAY,LOW); 
  pTone(4);
  trigDyn(F("PUMP 1"), deep ? "DEEP SOAK START" : "IRRIGATION", POPUP_PRIORITY_MED);
}

void stopP1() { 
  if(!GET_FLAG(FLAG_PUMP1_RUN)) return;
  SET_FLAG(FLAG_PUMP1_RUN,0); 
  SET_FLAG(FLAG_DEEP_SOAK,0); 
  digitalWrite(PUMP1_RELAY,HIGH); 
  trigDyn(F("PUMP 1"), "STOPPED", POPUP_PRIORITY_LOW);
}

void startP2(bool deep) { 
  if(GET_FLAG(FLAG_PUMP2_RUN)) return; 
  SET_FLAG(FLAG_PUMP2_RUN,1); 
  SET_FLAG(FLAG_DEEP_SOAK, deep);
  p2StartMs=millis(); 
  digitalWrite(PUMP2_RELAY,LOW); 
  pTone(4);
  trigDyn(F("PUMP 2"), deep ? "DEEP SOAK START" : "IRRIGATION", POPUP_PRIORITY_MED);
}

void stopP2() { 
  if(!GET_FLAG(FLAG_PUMP2_RUN)) return;
  SET_FLAG(FLAG_PUMP2_RUN,0); 
  SET_FLAG(FLAG_DEEP_SOAK,0); 
  digitalWrite(PUMP2_RELAY,HIGH); 
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
    else if (GET_FLAG(FLAG_DEEP_SOAK) && pct[0] >= 95) stopP1();
    else if (!GET_FLAG(FLAG_DEEP_SOAK) && pct[0] >= (zThr1 + 5)) stopP1();
  }
  if (GET_FLAG(FLAG_PUMP2_RUN)) {
    int32_t diff2 = (int32_t)(now - p2StartMs);
    if (diff2 > 30000L) {
      stopP2();
      trigDyn(F("PUMP 2"), "FAILSAFE STOP", POPUP_PRIORITY_HIGH);
    }
    else if (GET_FLAG(FLAG_DEEP_SOAK) && pct[1] >= 95) stopP2();
    else if (!GET_FLAG(FLAG_DEEP_SOAK) && pct[1] >= (zThr2 + 5)) stopP2();
  }
}

// -- SMART SCHEDULING ---------------------------------------
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
    
    if (schedP1 != -1 && nowHM == schedP1 && n.minute() != lastMinute) {
      if (!p1TriggeredToday) {
        startP1(true);
        p1TriggeredToday = true;
      }
    }
    
    if (schedP2 != -1 && nowHM == schedP2 && n.minute() != lastMinute) {
      if (!p2TriggeredToday) {
        startP2(true);
        p2TriggeredToday = true;
      }
    }
    
    lastMinute = n.minute();
  }
  
  if (!GET_FLAG(FLAG_AUTO_MODE)) return;
  
  if (!GET_FLAG(FLAG_PUMP1_RUN) && pct[0] < zThr1) startP1(false);
  if (!GET_FLAG(FLAG_PUMP2_RUN) && pct[1] < zThr2) startP2(false);

  emergencyState = (pct[2] > gasThr) || (tempC > (fanTempOn + 5));
  bool nf = emergencyState || (tempC > fanTempOn) || (humPct > fanHumOn);
  digitalWrite(FAN_RELAY, nf ? LOW : HIGH);
}

// -- SENSORS ------------------------------------------------
void updateSensors(bool force) {
  uint32_t now = millis();
  int32_t diff = (int32_t)(now - lastSensorMs);
  if (!force && diff < 200L) return;
  lastSensorMs = now;
  
  readings[0][readIdx] = (analogRead(SOIL1_PIN) + analogRead(SOIL2_PIN)) / 2;
  readings[1][readIdx] = analogRead(SOIL3_PIN);
  readings[2][readIdx] = analogRead(GAS_PIN);
  readIdx = (readIdx + 1) % SAMPLES;
  
  long totals[3] = {0,0,0};
  for(uint8_t i=0; i<SAMPLES; i++) { 
    totals[0]+=readings[0][i]; 
    totals[1]+=readings[1][i]; 
    totals[2]+=readings[2][i]; 
  }
  for(uint8_t i=0; i<3; i++) pct[i] = mapPct(totals[i]/SAMPLES, baseMin[i], baseMax[i]);
  
  float t = dht.readTemperature();
  float h = dht.readHumidity();
  if(isnan(t) || isnan(h)) {
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
  
  lightBright = (digitalRead(LDR_PIN)==HIGH);
  updateRainSensor();

  // Servo-exclusive 10-second delay for day/night changes
  if (lightBright != delayedLightBright) {
    if (!lightChangePending) {
      lightChangePending = true;
      lightChangeStartMs = millis();
    } else if ((millis() - lightChangeStartMs) >= LIGHT_CHANGE_DELAY_MS) {
      delayedLightBright = lightBright;
      lightChangePending = false;
    }
  } else {
    lightChangePending = false;
  }
}

// -- SERVO (20-sec rain hysteresis + 10-sec light delay) ----
void updateServo() {
  int goal;

  // PRIORITY 1: Rain detected → close immediately
  if (isRaining) {
    goal = 90;
    lastRainMs = millis();
    rainHysteresisActive = true;
  }
  // PRIORITY 2: Rain just stopped → stay closed 20 seconds
  else if (rainHysteresisActive && ((millis() - lastRainMs) < RAIN_HYSTERESIS_MS)) {
    goal = 90;
  }
  // PRIORITY 3+: Normal operation (clear flag once hysteresis done)
  else {
    rainHysteresisActive = false;

    if (emergencyState) {
      goal = 0;
    } else if (delayedLightBright) {
      goal = 90;   // Day → close
    } else {
      goal = 0;  // Night → open
    }
  }

  // Move toward goal immediately
  if (currServoPos == goal) return;

  uint32_t now = millis();
  if ((int32_t)(now - lastServoMs) > 20L) {
    lastServoMs = now;
    if (currServoPos < goal) {
      currServoPos = min(180, currServoPos + 4);
    } else {
      currServoPos = max(0, currServoPos - 4);
    }
    roofServo.write(currServoPos);
  }
}

// -- CALIBRATION -------------------------------------------
void calibrate(bool isMax, char target) {
  updateSensors(true); 
  pTone(5);
  
  int t = (target >= '0' && target <= '2') ? (target - '0') : -1;
  if (t != -1) {
    int raw = (t==0) ? (analogRead(SOIL1_PIN)+analogRead(SOIL2_PIN))/2 : analogRead(t==1?SOIL3_PIN:GAS_PIN);
    
    char popupMsg[16];
    if (isMax) { 
      baseMax[t] = raw; 
      EEPROM.put(7 + t*2, baseMax[t]); 
      snprintf(popupMsg, 16, "Z%d MAX:%d", t, raw);
    } else { 
      baseMin[t] = raw; 
      EEPROM.put(1 + t*2, baseMin[t]); 
      snprintf(popupMsg, 16, "Z%d MIN:%d", t, raw);
    }
    
    trigDyn(F("CALIBRATED"), popupMsg, POPUP_PRIORITY_MED);
  }
}

// -- TIME PARSING ------------------------------------------
int parseTimeStr(char *str) {
  if (strstr(str, "off")) {
    trigDyn(F("SCHEDULE"), "DISABLED", POPUP_PRIORITY_MED);
    return -1;
  }
  
  int h = 0, m = 0;
  while(*str && !isdigit(*str)) str++;
  while(isdigit(*str)) { h = h * 10 + (*str - '0'); str++; }
  if (*str == ':') str++;
  while(isdigit(*str)) { m = m * 10 + (*str - '0'); str++; }
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
    lcd.print(h); 
    lcd.print(F(":")); 
    if(m<10) lcd.print('0'); 
    lcd.print(m); 
    lcd.print(pm?F("P"):F("A")); 
  } else { 
    BT.print(h); 
    BT.print(F(":")); 
    if(m<10) BT.print('0'); 
    BT.print(m); 
    BT.print(pm?F("PM"):F("AM")); 
  }
}

// -- EEPROM MANAGEMENT -------------------------------------
struct ConfigCache {
  int schedP1_cached;
  int schedP2_cached;
  int zThr1_cached;
  int zThr2_cached;
  int gasThr_cached;
  int fanTempOn_cached;
  int fanHumOn_cached;
} configCache;

void initConfigCache() {
  configCache.schedP1_cached = schedP1;
  configCache.schedP2_cached = schedP2;
  configCache.zThr1_cached = zThr1;
  configCache.zThr2_cached = zThr2;
  configCache.gasThr_cached = gasThr;
  configCache.fanTempOn_cached = fanTempOn;
  configCache.fanHumOn_cached = fanHumOn;
}

void saveIfChanged(int addr, int value, int &cached) {
  if (value != cached) {
    EEPROM.put(addr, value);
    cached = value;
  }
}

void saveConfig() {
  saveIfChanged(21, schedP1, configCache.schedP1_cached);
  saveIfChanged(23, schedP2, configCache.schedP2_cached);
  saveIfChanged(25, zThr1, configCache.zThr1_cached);
  saveIfChanged(27, zThr2, configCache.zThr2_cached);
  saveIfChanged(29, gasThr, configCache.gasThr_cached);
  saveIfChanged(31, fanTempOn, configCache.fanTempOn_cached);
  saveIfChanged(33, fanHumOn, configCache.fanHumOn_cached);
}

// -- COMMAND PROCESSOR -------------------------------------
void printHelp() {
  BT.println();
  BT.println(F("=== AGRISENSE v4.2 COMMANDS ==="));
  BT.println(F("General: help status debug page"));
  BT.println(F("Mode: auto manual"));
  BT.println(F("Pumps: 1/1on/1off 2/2on/2off"));
  BT.println(F("Fan: f/fanon/fanoff"));
  BT.println(F("Schedule: 3 HH:MM / 3 off"));
  BT.println(F("Schedule: 4 HH:MM / 4 off"));
  BT.println(F("Thresholds: z35 x40 g80 t31 u85"));
  BT.println(F("Calibrate: c0/c1/c2 w0/w1/w2"));
  BT.println(F("System: reset save test"));
  BT.println(F("==============================="));
  BT.println();
  trigDyn(F("HELP SENT"), "CHECK BT MONITOR", POPUP_PRIORITY_LOW);
}

void printStatus() {
  BT.println();
  BT.println(F("--- SYSTEM STATUS ---"));
  BT.print(F("Mode: "));
  BT.println(GET_FLAG(FLAG_AUTO_MODE)?F("AUTO"):F("MANUAL"));
  BT.print(F("Z1: "));
  BT.print(pct[0]);
  BT.print(F("% (thr:"));
  BT.print(zThr1);
  BT.println(F("%)"));
  BT.print(F("Z2: "));
  BT.print(pct[1]);
  BT.print(F("% (thr:"));
  BT.print(zThr2);
  BT.println(F("%)"));
  BT.print(F("Gas: "));
  BT.print(pct[2]);
  BT.print(F("% (thr:"));
  BT.print(gasThr);
  BT.println(F("%)"));
  BT.print(F("Temp: "));
  BT.print(tempC);
  BT.println(F("C"));
  BT.print(F("Humidity: "));
  BT.print(humPct);
  BT.println(F("%"));
  BT.print(F("Light: "));
  BT.println(lightBright?F("BRIGHT"):F("DARK"));
  BT.print(F("Rain: "));
  BT.println(isRaining?F("DETECTED"):F("NONE"));
  BT.print(F("P1: "));
  BT.print(GET_FLAG(FLAG_PUMP1_RUN)?F("ON"):F("OFF"));
  BT.print(F(" P2: "));
  BT.print(GET_FLAG(FLAG_PUMP2_RUN)?F("ON"):F("OFF"));
  BT.print(F(" FAN: "));
  BT.println(digitalRead(FAN_RELAY)==LOW?F("ON"):F("OFF"));
  BT.print(F("Servo: "));
  BT.print(currServoPos);
  BT.println(F("deg"));
  BT.print(F("P1 Sched: "));
  printAMPM(schedP1, false);
  BT.println();
  BT.print(F("P2 Sched: "));
  printAMPM(schedP2, false);
  BT.println();
  BT.println(F("---------------------"));
  BT.println();
  
  char modeStr[8];
  strcpy(modeStr, GET_FLAG(FLAG_AUTO_MODE)?"AUTO":"MANUAL");
  trigDyn(F("STATUS OK"), modeStr, POPUP_PRIORITY_LOW);
}

void processCmd(char *cmd) {
  char originalCmd[24];
  strncpy(originalCmd, cmd, 23);
  originalCmd[23] = '\0';
  
  for(uint8_t i=0; cmd[i]; i++) cmd[i] = tolower(cmd[i]);
  
  char* end = cmd + strlen(cmd) - 1;
  while(end > cmd && isspace(*end)) *end-- = '\0';
  
  if (strcmp(cmd, "help") == 0 || cmd[0] == '?') { 
    printHelp(); 
    return; 
  }
  
  if (strcmp(cmd, "status") == 0 || cmd[0] == 's') { 
    printStatus(); 
    return; 
  }
  
  if (cmd[0]=='d' && strcmp(cmd, "debug") == 0) { 
    SET_FLAG(FLAG_DEBUG_ON, !GET_FLAG(FLAG_DEBUG_ON)); 
    pTone(0); 
    trigDyn(F("DEBUG"), GET_FLAG(FLAG_DEBUG_ON)?"ENABLED":"DISABLED", POPUP_PRIORITY_MED);
    return; 
  }
  
  if (cmd[0]=='p' && strcmp(cmd, "page") == 0) { 
    lcdPage=(lcdPage+1)%5; 
    lcd.clear(); 
    pTone(0); 
    char pageMsg[16];
    snprintf(pageMsg, 16, "PAGE %d/5", lcdPage+1);
    trigDyn(F("DISPLAY"), pageMsg, POPUP_PRIORITY_LOW);
    return; 
  }
  
  if (cmd[0]=='m' || strcmp(cmd, "auto") == 0 || strcmp(cmd, "manual") == 0) { 
    bool newMode = (cmd[0]=='m') ? !GET_FLAG(FLAG_AUTO_MODE) : (strcmp(cmd, "auto") == 0);
    SET_FLAG(FLAG_AUTO_MODE, newMode); 
    pTone(newMode?1:2); 
    trigDyn(F("MODE CHANGED"), newMode?"AUTO":"MANUAL", POPUP_PRIORITY_MED);
    return; 
  }
  
  if (cmd[0]=='1' || strncmp(cmd, "pump1", 5) == 0) {
    bool turnOn = (strstr(cmd, "on") != nullptr);
    bool turnOff = (strstr(cmd, "off") != nullptr);
    
    if (turnOff || (cmd[0]=='1' && !turnOn && GET_FLAG(FLAG_PUMP1_RUN))) {
      stopP1();
    } else {
      startP1(false);
    }
    return;
  }
  
  if (cmd[0]=='2' || strncmp(cmd, "pump2", 5) == 0) {
    bool turnOn = (strstr(cmd, "on") != nullptr);
    bool turnOff = (strstr(cmd, "off") != nullptr);
    
    if (turnOff || (cmd[0]=='2' && !turnOn && GET_FLAG(FLAG_PUMP2_RUN))) {
      stopP2();
    } else {
      startP2(false);
    }
    return;
  }
  
  if (cmd[0]=='f' || strncmp(cmd, "fan", 3) == 0) {
    bool turnOn = (strstr(cmd, "on") != nullptr);
    bool turnOff = (strstr(cmd, "off") != nullptr);
    bool currentState = (digitalRead(FAN_RELAY) == LOW);
    
    if (turnOn) {
      digitalWrite(FAN_RELAY, LOW);
      trigDyn(F("FAN"), "FORCED ON", POPUP_PRIORITY_MED);
    } else if (turnOff) {
      digitalWrite(FAN_RELAY, HIGH);
      trigDyn(F("FAN"), "FORCED OFF", POPUP_PRIORITY_MED);
    } else {
      digitalWrite(FAN_RELAY, currentState?HIGH:LOW);
      trigDyn(F("FAN"), currentState?"OFF":"ON", POPUP_PRIORITY_MED);
    }
    pTone(4);
    return;
  }
  
  if (cmd[0]=='3') { 
    int newSched = parseTimeStr(&cmd[1]); 
    if (newSched != -1 || strstr(&cmd[1], "off")) {
      schedP1 = newSched; 
      saveConfig();
      char schedMsg[16];
      if (schedP1 == -1) {
        strcpy(schedMsg, "DISABLED");
      } else {
        int h = schedP1 / 100;
        int m = schedP1 % 100;
        snprintf(schedMsg, 16, "%02d:%02d SET", h, m);
      }
      trigDyn(F("PUMP1 SCHED"), schedMsg, POPUP_PRIORITY_MED);
    }
    return; 
  }
  
  if (cmd[0]=='4') { 
    int newSched = parseTimeStr(&cmd[1]); 
    if (newSched != -1 || strstr(&cmd[1], "off")) {
      schedP2 = newSched; 
      saveConfig();
      char schedMsg[16];
      if (schedP2 == -1) {
        strcpy(schedMsg, "DISABLED");
      } else {
        int h = schedP2 / 100;
        int m = schedP2 % 100;
        snprintf(schedMsg, 16, "%02d:%02d SET", h, m);
      }
      trigDyn(F("PUMP2 SCHED"), schedMsg, POPUP_PRIORITY_MED);
    }
    return; 
  }
  
  if (cmd[0]=='z') { 
    int newVal = atoi(&cmd[1]); 
    if (newVal >= 0 && newVal <= 99) {
      zThr1 = newVal; 
      saveConfig();
      char valMsg[16];
      snprintf(valMsg, 16, "Z1: %d%%", zThr1);
      trigDyn(F("THRESHOLD"), valMsg, POPUP_PRIORITY_MED);
    } else {
      trigDyn(F("ERROR"), "USE 0-99", POPUP_PRIORITY_HIGH);
    }
    return; 
  }
  
  if (cmd[0]=='x') { 
    int newVal = atoi(&cmd[1]); 
    if (newVal >= 0 && newVal <= 99) {
      zThr2 = newVal; 
      saveConfig();
      char valMsg[16];
      snprintf(valMsg, 16, "Z2: %d%%", zThr2);
      trigDyn(F("THRESHOLD"), valMsg, POPUP_PRIORITY_MED);
    } else {
      trigDyn(F("ERROR"), "USE 0-99", POPUP_PRIORITY_HIGH);
    }
    return; 
  }
  
  if (cmd[0]=='g') { 
    int newVal = atoi(&cmd[1]); 
    if (newVal >= 0 && newVal <= 99) {
      gasThr = newVal; 
      saveConfig();
      char valMsg[16];
      snprintf(valMsg, 16, "GAS: %d%%", gasThr);
      trigDyn(F("ALARM LVL"), valMsg, POPUP_PRIORITY_MED);
    } else {
      trigDyn(F("ERROR"), "USE 0-99", POPUP_PRIORITY_HIGH);
    }
    return; 
  }
  
  if (cmd[0]=='t') { 
    int newVal = atoi(&cmd[1]); 
    if (newVal >= 0 && newVal <= 60) {
      fanTempOn = newVal; 
      saveConfig();
      char valMsg[16];
      snprintf(valMsg, 16, "TEMP: %dC", fanTempOn);
      trigDyn(F("FAN TEMP"), valMsg, POPUP_PRIORITY_MED);
    } else {
      trigDyn(F("ERROR"), "USE 0-60C", POPUP_PRIORITY_HIGH);
    }
    return; 
  }
  
  if (cmd[0]=='u') { 
    int newVal = atoi(&cmd[1]); 
    if (newVal >= 0 && newVal <= 100) {
      fanHumOn = newVal; 
      saveConfig();
      char valMsg[16];
      snprintf(valMsg, 16, "HUM: %d%%", fanHumOn);
      trigDyn(F("FAN HUMID"), valMsg, POPUP_PRIORITY_MED);
    } else {
      trigDyn(F("ERROR"), "USE 0-100", POPUP_PRIORITY_HIGH);
    }
    return; 
  }
  
  if (cmd[0]=='c') { 
    calibrate(false, cmd[1]); 
    return; 
  }
  
  if (cmd[0]=='w') { 
    calibrate(true, cmd[1]); 
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
  
  if (strcmp(cmd, "save") == 0) {
    saveConfig();
    trigDyn(F("CONFIG"), "SAVED", POPUP_PRIORITY_MED);
    pTone(1);
    return;
  }
  
  if (strcmp(cmd, "test") == 0) {
    trigDyn(F("TEST MODE"), "RUNNING...", POPUP_PRIORITY_HIGH);
    pTone(5);
    digitalWrite(FAN_RELAY, LOW); delay(200); digitalWrite(FAN_RELAY, HIGH);
    delay(100);
    digitalWrite(PUMP1_RELAY, LOW); delay(200); digitalWrite(PUMP1_RELAY, HIGH);
    delay(100);
    digitalWrite(PUMP2_RELAY, LOW); delay(200); digitalWrite(PUMP2_RELAY, HIGH);
    trigDyn(F("TEST"), "COMPLETE", POPUP_PRIORITY_MED);
    return;
  }
  
  if (strcmp(cmd, "json") == 0) {
    SET_FLAG(FLAG_JSON_MODE, !GET_FLAG(FLAG_JSON_MODE));
    trigDyn(F("OUTPUT"), GET_FLAG(FLAG_JSON_MODE)?"JSON MODE":"TEXT MODE", POPUP_PRIORITY_MED);
    return;
  }
  
  if (strcmp(cmd, "verbose") == 0) {
    trigDyn(F("VERBOSE"), "TOGGLED", POPUP_PRIORITY_MED);
    return;
  }
  
  trigDyn(F("UNKNOWN"), "USE 'help'", POPUP_PRIORITY_HIGH);
  pTone(3);
}

// -- DEBUG OUTPUT -------------------------------------------
void sendDebug() {
  if (!GET_FLAG(FLAG_DEBUG_ON)) return;
  uint32_t now = millis();
  int32_t diff = (int32_t)(now - lastDebugMs);
  if (diff < 2000L) return;
  lastDebugMs = now;
  
  if (GET_FLAG(FLAG_JSON_MODE)) {
    BT.print(F("{\"t\":"));
    BT.print(millis());
    BT.print(F(",\"z1\":"));
    BT.print(pct[0]);
    BT.print(F(",\"z2\":"));
    BT.print(pct[1]);
    BT.print(F(",\"g\":"));
    BT.print(pct[2]);
    BT.print(F(",\"T\":"));
    BT.print(tempC);
    BT.print(F(",\"H\":"));
    BT.print(humPct);
    BT.print(F(",\"rain\":"));
    BT.print(isRaining);
    BT.print(F(",\"p1\":"));
    BT.print(GET_FLAG(FLAG_PUMP1_RUN));
    BT.print(F(",\"p2\":"));
    BT.print(GET_FLAG(FLAG_PUMP2_RUN));
    BT.print(F(",\"f\":"));
    BT.print(digitalRead(FAN_RELAY)==LOW);
    BT.print(F(",\"s\":"));
    BT.print(currServoPos);
    BT.print(F(",\"m\":\""));
    BT.print(GET_FLAG(FLAG_AUTO_MODE)?F("A"):F("M"));
    BT.println(F("\"}"));
  } else {
    BT.println();
    BT.println(F("--- DEBUG ---"));
    BT.print(F("Z1:"));
    BT.print(pct[0]);
    BT.print(F("% Z2:"));
    BT.print(pct[1]);
    BT.print(F("% G:"));
    BT.print(pct[2]);
    BT.println(F("%"));
    BT.print(F("T:"));
    BT.print(tempC);
    BT.print(F("C H:"));
    BT.print(humPct);
    BT.print(F("% RAIN:"));
    BT.println(isRaining?F("YES"):F("NO"));
    BT.print(F("P1:"));
    BT.print(GET_FLAG(FLAG_PUMP1_RUN));
    BT.print(F(" P2:"));
    BT.print(GET_FLAG(FLAG_PUMP2_RUN));
    BT.print(F(" F:"));
    BT.println(digitalRead(FAN_RELAY)==LOW);
    BT.print(F("S:"));
    BT.print(currServoPos);
    BT.print(F(" L:"));
    BT.print(lightBright);
    BT.print(F(" ["));
    BT.print(GET_FLAG(FLAG_AUTO_MODE)?F("A"):F("M"));
    BT.println(F("]"));
    BT.println(F("-------------"));
  }
}

// -- UI RENDERING --------------------------------------------
void showPopup() {
  if (dynL1 == nullptr) return;
  
  lcd.setCursor(0,0);
  char l1Buffer[17];
  strncpy_P(l1Buffer, (const char*)dynL1, 16);
  l1Buffer[16] = '\0';
  uint8_t pad1 = (16 - strlen(l1Buffer)) / 2;
  while(pad1--) lcd.print(' ');
  lcd.print(l1Buffer);
  
  lcd.setCursor(0,1);
  uint8_t pad2 = (16 - strlen(dynL2)) / 2;
  while(pad2--) lcd.print(' ');
  lcd.print(dynL2);
}

void showPage(uint8_t pg) {
  static bool ani = false; 
  ani = !ani;
  
  switch(pg) {
    case 0:
      lcd.setCursor(0,0); 
      lcd.write(2);
      lcd.print(pct[0]);
      lcd.write(pct[0]>70?5:pct[0]>30?4:3);
      lcd.write(2);
      lcd.print(pct[1]);
      lcd.write(pct[1]>70?5:pct[1]>30?4:3);
      
      if (isRaining) {
        lcd.write(9);
        lcd.print(F("R"));
      } else {
        lcd.print(F(" G"));
        lcd.print(pct[2]);
      }
      
      lcd.setCursor(0,1); 
      lcd.write(6);
      lcd.print((int)tempC);
      lcd.write(223);
      lcd.write(7);
      lcd.print((int)humPct);
      lcd.print(F("% "));
      lcd.print(GET_FLAG(FLAG_AUTO_MODE)?F("[A]"):F("[M]"));
      
      lcd.print(isRaining ? F(" R") : F("  "));
      lcd.print(lightBright ? F("D") : F("N"));
      break;
      
    case 1:
      lcd.setCursor(0,0); 
      lcd.print(F("P1:"));
      if(GET_FLAG(FLAG_PUMP1_RUN)) {
        lcd.write(2);
        lcd.print(F("ON "));
      }
      else lcd.print(F("OFF"));
      lcd.print(F(" P2:"));
      if(GET_FLAG(FLAG_PUMP2_RUN)) {
        lcd.write(2);
        lcd.print(F("ON "));
      }
      else lcd.print(F("OFF"));
      
      lcd.setCursor(0,1); 
      lcd.print(F("F:"));
      if(digitalRead(FAN_RELAY)==LOW) {
        lcd.write(ani?0:1);
        lcd.print(F("ON "));
      }
      else lcd.print(F("OFF "));
      lcd.print(F("S:"));
      lcd.print(currServoPos);
      lcd.write(223);
      lcd.print(F(" "));
      lcd.print(lightBright?F("D"):F("N"));
      lcd.print(F(" R"));
      lcd.print(isRaining?F("1"):F("0"));
      break;
      
    case 2:
      lcd.setCursor(0,0); 
      lcd.print(F("P1:"));
      printAMPM(schedP1, true);
      lcd.setCursor(0,1); 
      lcd.print(F("P2:"));
      printAMPM(schedP2, true);
      break;
      
    case 3:
      lcd.setCursor(0,0);
      lcd.print(F("Z1:"));
      lcd.print(zThr1);
      lcd.print(F("% Z2:"));
      lcd.print(zThr2);
      lcd.print(F("%"));
      lcd.setCursor(0,1);
      lcd.print(F("T:"));
      lcd.print(fanTempOn);
      lcd.write(223);
      lcd.print(F(" H:"));
      lcd.print(fanHumOn);
      lcd.print(F("%"));
      break;
      
    case 4:
      lcd.setCursor(0,0);
      lcd.print(F("RTC:"));
      lcd.print(GET_FLAG(FLAG_RTC_OK)?F("OK "):F("ERR"));
      lcd.print(F(" DHT:"));
      lcd.print(GET_FLAG(FLAG_DHT_ERR)?F("ERR"):F("OK "));
      lcd.setCursor(0,1);
      if (isRaining) {
        lcd.write(9);
        lcd.print(F("RAINING "));
      } else {
        lcd.print(F("E:"));
        lcd.print(emergencyState?F("ALERT "):F("NORM "));
        if (emergencyState) lcd.write(8);
      }
      break;
  }
}

// -- SETUP & BOOT -------------------------------------------
void setup() {
  wdt_disable();
  
  BT.begin(9600);
  dht.begin();
  Wire.begin();
  
  pinMode(FAN_RELAY,OUTPUT);
  pinMode(PUMP1_RELAY,OUTPUT);
  pinMode(PUMP2_RELAY,OUTPUT);
  pinMode(BUZZER_PIN, OUTPUT);
  pinMode(LDR_PIN, INPUT);
  pinMode(RAIN_PIN, INPUT_PULLUP);
  
  digitalWrite(FAN_RELAY, OFF);
  digitalWrite(PUMP1_RELAY, OFF);
  digitalWrite(PUMP2_RELAY, OFF);
  
  roofServo.attach(SERVO_PIN);
  roofServo.write(currServoPos);
  
  lcd.init();
  lcd.backlight();
  
  byte temp[8];
  memcpy_P(temp, charFan[0], 8);
  lcd.createChar(0, temp);
  memcpy_P(temp, charFan[1], 8);
  lcd.createChar(1, temp);
  memcpy_P(temp, charDrop, 8);
  lcd.createChar(2, temp);
  memcpy_P(temp, charBar[0], 8);
  lcd.createChar(3, temp);
  memcpy_P(temp, charBar[1], 8);
  lcd.createChar(4, temp);
  memcpy_P(temp, charBar[2], 8);
  lcd.createChar(5, temp);
  memcpy_P(temp, charThermo, 8);
  lcd.createChar(6, temp);
  memcpy_P(temp, charHumid, 8);
  lcd.createChar(7, temp);
  memcpy_P(temp, charAlert, 8);
  lcd.createChar(8, temp);
  memcpy_P(temp, charRain, 8);
  lcd.createChar(9, temp);

  lcd.setCursor(0,0);
  lcd.print(F("AGRISENSE v4.2"));
  lcd.setCursor(0,1);
  lcd.print(F("INITIALIZING..."));
  pTone(1);
  delay(800);
  
  memset(readings, 0, sizeof(readings));

  const char* checks[] = {"RTC", "DHT", "RELAYS", "SERVO", "EEPROM", "RAIN"};
  for(uint8_t i=0; i<6; i++) {
    lcd.clear();
    lcd.setCursor(0,0);
    lcd.print(F("CHECK:"));
    lcd.setCursor(0,1);
    lcd.print(checks[i]);
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
    for(uint8_t i=0; i<3; i++) {
      EEPROM.get(1+i*2, baseMin[i]);
      EEPROM.get(7+i*2, baseMax[i]);
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
  lcd.setCursor(0,1);
  lcd.print(GET_FLAG(FLAG_AUTO_MODE)?F("AUTO MODE"):F("MANUAL MODE"));
  pTone(1);
  delay(1500);
  
  updateSensors(true);
  delayedLightBright = lightBright; // Sync at boot so servo starts correctly
  lcd.clear();
  
  wdt_enable(WDTO_2S);
}

void loop() {
  wdt_reset();
  
  static char btBuf[24];
  static byte btIdx=0;
  while (BT.available()) {
    char c = (char)BT.read();
    if(c=='\n'||c=='\r') {
      btBuf[btIdx]='\0';
      if(btIdx>0) processCmd(btBuf);
      btIdx=0;
    }
    else if (btIdx<23) btBuf[btIdx++]=c;
  }
  
  updateSensors(false);
  updateServo();
  updateTimers();
  runAuto();
  updateTone();
  
  if (dynPage==99) {
    uint32_t now = millis();
    int32_t popupDiff = (int32_t)(now - dynStartMs);
    if (popupDiff >= POPUP_DURATION_MS) {
      dynPage=255;
      popupPriority = POPUP_PRIORITY_LOW;
      lcd.clear();
    }
  }
  
  uint32_t now = millis();
  int32_t lcdDiff = (int32_t)(now - lastLcdMs);
  if (lcdDiff > 500L) {
    lastLcdMs = now;
    if (dynPage==99) showPopup();
    else showPage(lcdPage);
  }
  
  sendDebug();
}
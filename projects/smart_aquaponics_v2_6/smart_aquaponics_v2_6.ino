/* ================================================================
 * SMART AQUAPONICS v2.6.0
 * Arduino Uno R3 | Active-HIGH relays | BT on pins 0/1
 * GREEN = OFF (LOW) | RED = ON (HIGH)
 *
 * FIXES in v2.6.0:
 *  - pH NaN: root cause was EEPROM NaN propagation + no isnan() guard
 *  - pH NaN: ADC channel crosstalk fixed with dummy reads between channels
 *  - pH NaN: phVal now always initialized safe, guarded before assignment
 *  - sensorOK logic simplified - ultrasonic 999 no longer kills pH display
 *  - Servo fully manual: no auto timing, no feedTrigger, no state machine
 *  - Servo commands: servo open / servo close / servo stop
 *  - p4 command added for servo: p4 open / p4 close
 *  - Sensor reads staggered - each sensor on its own timer
 *  - DS18B20 non-blocking: requestTemperatures() called separately
 *  - Event log static bug fixed (two competing statics replaced with one)
 * ================================================================ */

#include <Wire.h>
#include <LiquidCrystal_I2C.h>
#include <OneWire.h>
#include <DallasTemperature.h>
#include <EEPROM.h>
#include <Servo.h>
#include <math.h>   // isnan(), isinf()

// ==================== PINS ====================
#define R_SUPPLY   8
#define R_FILTER   7
#define R_PLANT    6
#define TRIG_PIN   5
#define ECHO_PIN   3
#define TEMP_PIN   4
#define PH_PIN     A0
#define TURB_PIN   A1
#define SOIL_PIN   A2
#define SERVO_PIN  9

// ==================== RELAY ====================
#define RELAY_OFF  LOW
#define RELAY_ON   HIGH

// ==================== DEBUG ====================
#define DEBUG_MODE 1
#if DEBUG_MODE
  #define DBG(msg)      do { Serial.print(F("[D] ")); Serial.println(F(msg)); } while(0)
  #define DBGV(l, v)    do { Serial.print(F("[D] ")); Serial.print(F(l)); Serial.print(':'); Serial.println(v); } while(0)
#else
  #define DBG(msg)      do {} while(0)
  #define DBGV(l, v)    do {} while(0)
#endif

// ==================== EEPROM MAP ====================
// 0  : wTarget    int   2b
// 2  : phMin      float 4b
// 6  : phMax      float 4b
// 10 : turbTh     int   2b
// 12 : soilTh     int   2b
// 14 : feedMin    int   2b  (kept for compat, servo is now manual only)
// 16 : xferSec    int   2b
// 18 : waterSec   int   2b
// 20 : checksum   byte  1b  (XOR of bytes 0-19)
// 21 : magic      byte  0xAB
// 22 : phOffset   float 4b

// ==================== CONFIG ====================
int   wTarget   = 5;
float phMin     = 6.5f, phMax = 8.5f;
int   turbTh    = 500,  soilTh = 800;
int   xferSec   = 10,   waterSec = 15;
int   waterHyst = 2;
float phHyst    = 0.2f;
int   turbHyst  = 50;
// +1.7 default: real sensor read pH 5.0-5.3 on clean water at offset 0.0
// Fine-tune via BT: set phoffset [val]
float phOffset  = 1.7f;

bool liveMode = false;

// ==================== OBJECTS ====================
LiquidCrystal_I2C lcd(0x27, 20, 4);
OneWire           ow(TEMP_PIN);
DallasTemperature ts(&ow);
Servo             sv;

// ==================== STATE ====================
bool manualP1 = false, manualP2 = false, manualP3 = false;
bool manualMode = false;

enum WaterState { WS_IDLE, WS_XFER, WS_WATER } wsState = WS_IDLE;

unsigned long wsTimer    = 0;
unsigned long lastSensor = 0, lastLCD  = 0, lastLive = 0;
unsigned long graceStart = 0;
bool startupGrace = true;

// Separate timers for staggered sensor reads
unsigned long lastPH    = 0;
unsigned long lastTurb  = 0;
unsigned long lastSoil  = 0;
unsigned long lastDist  = 0;
unsigned long lastTemp  = 0;
bool tempRequested = false;

// ==================== SENSOR VALUES ====================
// All initialized to safe defaults - never NaN
float phVal   = 7.0f;
float tempC   = 25.0f;
int   distCm  = 0;
int   turbRaw = 500;
int   soilRaw = 500;

// pH averaging - simple windowed average, no delay()
#define PH_AVG_SIZE 8
int   phBuf[PH_AVG_SIZE];
uint8_t phBufIdx  = 0;
bool    phBufFull = false;

// Individual sensor OK flags - independent, no cross-contamination
bool phOK   = true;
bool tempOK = true;
bool distOK = true;

// Soil debounce
int dryCount = 0;
const int DRY_DEBOUNCE = 3;

// ==================== EVENT LOG ====================
#define LOG_SIZE 10
char    eventLog[LOG_SIZE][32];
uint8_t logHead = 0;

void addEvent(const __FlashStringHelper* msg) {
  strncpy_P(eventLog[logHead], (const char*)msg, 31);
  eventLog[logHead][31] = '\0';
  logHead = (logHead + 1) % LOG_SIZE;
  #if DEBUG_MODE
    Serial.print(F("[EVT] "));
    Serial.println(msg);
  #endif
}

// ==================== BT BUFFER ====================
static char    cmdBuf[64];
static uint8_t cmdIdx = 0;

// ==================== FORWARD DECLARATIONS ====================
void loadEEPROM();
void saveEEPROM();
void defaultEEPROM();
void allPumpsOff();
void readPH();
void readTurb();
void readSoil();
void readDist();
void readTemp();
void autoLogic();
void updateWaterState();
void handleBluetooth();
void processCommand(char* input);
void handleSet(String param);
void handlePump(int num, String state);
void handleServo(String state);
void setManualMode(bool manual);
void sendLiveStatus();
void sendJSONStatus();
void printEventLog();
void printModeStatus();
void updateLCD();

// ================================================================
// SETUP
// ================================================================
void setup() {
  Serial.begin(9600);

  pinMode(R_SUPPLY, OUTPUT);
  pinMode(R_FILTER, OUTPUT);
  pinMode(R_PLANT,  OUTPUT);
  allPumpsOff();

  pinMode(TRIG_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT);

  // Servo: park at 0, then detach - fully manual from here
  sv.attach(SERVO_PIN);
  sv.write(0);
  delay(400);
  sv.detach();

  ts.begin();
  ts.setWaitForConversion(false); // non-blocking temp reads

  // Init pH buffer with neutral-ish ADC value
  for (uint8_t i = 0; i < PH_AVG_SIZE; i++) phBuf[i] = 512;

  loadEEPROM();

  lcd.init();
  lcd.backlight();
  lcd.print(F("SMART AQUAPONICS"));
  lcd.setCursor(0, 1);
  lcd.print(F("v2.6.0"));
  delay(2000);

  graceStart = millis();
  lastLive   = millis();

  // Kick off first temp request
  ts.requestTemperatures();
  tempRequested = true;
  lastTemp = millis();

  addEvent(F("System started v2.6.0"));
  printModeStatus();
}

// ================================================================
// HELPERS
// ================================================================
void allPumpsOff() {
  digitalWrite(R_SUPPLY, RELAY_OFF);
  digitalWrite(R_FILTER, RELAY_OFF);
  digitalWrite(R_PLANT,  RELAY_OFF);
}

// Safe analogRead with dummy read for ADC channel settling
// The Uno ADC mux needs ~100us after switching channels.
// One dummy read is enough; no delay() needed.
int safeAnalogRead(uint8_t pin) {
  analogRead(pin); // dummy - discard, allows mux to settle
  return analogRead(pin);
}

// ================================================================
// STAGGERED SENSOR READS
// Each sensor has its own 1-second timer, offset from each other
// so they never all fire at once and never crosstalk on ADC.
// ================================================================

// --- pH: every 1000ms, read one sample into rolling buffer ---
void readPH() {
  if (millis() - lastPH < 1000UL) return;
  lastPH = millis();

  int raw = safeAnalogRead(PH_PIN);

  // Store in rolling buffer
  phBuf[phBufIdx] = raw;
  phBufIdx = (phBufIdx + 1) % PH_AVG_SIZE;
  if (phBufIdx == 0) phBufFull = true;

  // Average the buffer
  uint8_t count = phBufFull ? PH_AVG_SIZE : phBufIdx;
  if (count == 0) return;

  long sum = 0;
  for (uint8_t i = 0; i < count; i++) sum += phBuf[i];
  float avgRaw = (float)sum / (float)count;

  // Open-circuit / short detection
  if ((int)avgRaw < 10 || (int)avgRaw > 1013) {
    phOK = false;
    DBG("pH open circuit");
    return; // phVal unchanged - holds last good value
  }

  float voltage  = avgRaw * (5.0f / 1023.0f);
  float computed = 7.0f + ((2.5f - voltage) / 0.18f) + phOffset;

  // NaN / Inf guard - if formula produces garbage, reject it
  if (isnan(computed) || isinf(computed)) {
    phOK = false;
    DBG("pH formula produced NaN/Inf - rejected");
    return;
  }

  // Clamp to valid pH range
  computed = constrain(computed, 0.0f, 14.0f);
  phVal = computed;
  phOK  = true;
}

// --- Turbidity: every 1100ms (offset from pH) ---
void readTurb() {
  if (millis() - lastTurb < 1100UL) return;
  lastTurb = millis();
  turbRaw = safeAnalogRead(TURB_PIN);
}

// --- Soil: every 1200ms (offset from turb) ---
void readSoil() {
  if (millis() - lastSoil < 1200UL) return;
  lastSoil = millis();
  soilRaw = safeAnalogRead(SOIL_PIN);
}

// --- Ultrasonic: every 1300ms ---
void readDist() {
  if (millis() - lastDist < 1300UL) return;
  lastDist = millis();

  digitalWrite(TRIG_PIN, LOW);
  delayMicroseconds(2);
  digitalWrite(TRIG_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG_PIN, LOW);

  long dur = pulseIn(ECHO_PIN, HIGH, 25000UL); // 25ms timeout = ~4m max
  if (dur > 0) {
    distCm = (int)(dur * 0.0170f);
    distOK = true;
  } else {
    distOK = false;
    // keep last distCm - don't set to 999 and falsely trigger supply pump
  }
}

// --- Temperature: request every 2s, read result 1s later ---
void readTemp() {
  unsigned long now = millis();

  if (!tempRequested && now - lastTemp >= 2000UL) {
    ts.requestTemperatures();
    tempRequested = true;
    lastTemp = now;
    return;
  }

  if (tempRequested && now - lastTemp >= 1000UL) {
    float t = ts.getTempCByIndex(0);
    tempRequested = false;

    if (t == DEVICE_DISCONNECTED_C || isnan(t) || t < -55.0f || t > 125.0f) {
      tempOK = false;
      DBG("Temp sensor error");
    } else {
      tempC  = t;
      tempOK = true;
    }
  }
}

// ================================================================
// AUTO LOGIC
// ================================================================
void autoLogic() {
  // Supply pump - only act if distance sensor is reliable
  if (!manualP1 && distOK) {
    static bool lastSupplyOn = false;
    int trigLevel = wTarget + (lastSupplyOn ? 0 : waterHyst);
    bool shouldOn = (distCm > trigLevel);
    if (shouldOn != lastSupplyOn) {
      digitalWrite(R_SUPPLY, shouldOn ? RELAY_ON : RELAY_OFF);
      lastSupplyOn = shouldOn;
      addEvent(shouldOn ? F("AUTO: Supply ON") : F("AUTO: Supply OFF"));
    }
  }

  // Filter pump - only act if pH sensor is reliable
  if (!manualP2) {
    bool badPH   = phOK && (phVal < (phMin - phHyst) || phVal > (phMax + phHyst));
    bool badTurb = (turbRaw > (turbTh + turbHyst));
    bool badWater = badPH || badTurb;

    static bool lastBad = false;
    if (badWater != lastBad) {
      digitalWrite(R_FILTER, badWater ? RELAY_ON : RELAY_OFF);
      addEvent(badWater ? F("AUTO: Filter ON") : F("AUTO: Filter OFF"));
      lastBad = badWater;
    }
  }

  // Plant pump follows watering state machine
  if (!manualP3) {
    digitalWrite(R_PLANT, (wsState == WS_WATER) ? RELAY_ON : RELAY_OFF);
  }
}

// ================================================================
// WATERING STATE MACHINE
// ================================================================
void updateWaterState() {
  if (manualMode || startupGrace) return;
  unsigned long now = millis();

  switch (wsState) {
    case WS_IDLE:
      if (soilRaw > soilTh) {
        if (++dryCount >= DRY_DEBOUNCE) {
          wsState  = WS_XFER;
          wsTimer  = now;
          dryCount = 0;
          addEvent(F("AUTO: Transfer started"));
        }
      } else {
        dryCount = 0;
      }
      break;

    case WS_XFER:
      if (now - wsTimer >= (unsigned long)xferSec * 1000UL) {
        wsState = WS_WATER;
        wsTimer = now;
        addEvent(F("AUTO: Plant pump ON"));
      }
      break;

    case WS_WATER:
      if (now - wsTimer >= (unsigned long)waterSec * 1000UL) {
        wsState = WS_IDLE;
        addEvent(F("AUTO: Watering done"));
      }
      break;
  }
}

// ================================================================
// LOOP
// ================================================================
void loop() {
  unsigned long now = millis();
  handleBluetooth();

  // Grace period: 20 seconds, all pumps off
  if (startupGrace) {
    allPumpsOff();
    if (now - graceStart >= 20000UL) {
      startupGrace = false;
      addEvent(F("Grace ended - AUTO active"));
      Serial.println(F(">>> GRACE ENDED"));
      lcd.clear();
    } else {
      if (now - lastLCD >= 500UL) {
        lastLCD = now;
        int secs = (int)((20000UL - (now - graceStart)) / 1000UL);
        lcd.clear();
        lcd.setCursor(0, 0); lcd.print(F("SAFETY GRACE"));
        lcd.setCursor(0, 1); lcd.print(F("Pumps OFF "));
        lcd.print(secs); lcd.print('s');
      }
      // Still read sensors during grace so they warm up
      readPH();
      readTurb();
      readSoil();
      readDist();
      readTemp();
      return;
    }
  }

  // Staggered sensor reads (each has its own internal timer)
  readPH();
  readTurb();
  readSoil();
  readDist();
  readTemp();

  // Auto logic runs every 500ms
  if (now - lastSensor >= 500UL) {
    lastSensor = now;
    if (!manualMode) autoLogic();
  }

  updateWaterState();

  // LCD update every 600ms
  if (now - lastLCD >= 600UL) {
    lastLCD = now;
    updateLCD();
  }

  // Live mode every 2 seconds
  if (liveMode && now - lastLive >= 2000UL) {
    lastLive = now;
    sendLiveStatus();
  }
}

// ================================================================
// BLUETOOTH
// ================================================================
void handleBluetooth() {
  while (Serial.available()) {
    char c = Serial.read();
    if (c == '\n' || c == '\r') {
      if (cmdIdx > 0) {
        cmdBuf[cmdIdx] = '\0';
        processCommand(cmdBuf);
        cmdIdx = 0;
        cmdBuf[0] = '\0';
      }
    } else if (c == '\b' || c == 127) {
      if (cmdIdx > 0) cmdIdx--;
    } else if (c >= 32 && c < 127 && cmdIdx < sizeof(cmdBuf) - 1) {
      cmdBuf[cmdIdx++] = c;
    }
  }
}

void processCommand(char* input) {
  String cmd = String(input);
  cmd.toLowerCase();
  cmd.trim();

  Serial.print(F("> ")); Serial.println(cmd);

  if      (cmd == F("status") || cmd == F("st")) sendJSONStatus();
  else if (cmd == F("live"))   { liveMode = !liveMode; Serial.println(liveMode ? F("Live ON") : F("Live OFF")); }
  else if (cmd == F("auto"))   setManualMode(false);
  else if (cmd == F("manual")) setManualMode(true);
  else if (cmd == F("reset"))  { defaultEEPROM(); addEvent(F("Config reset")); }
  else if (cmd == F("save"))   { saveEEPROM();    addEvent(F("Config saved")); }
  else if (cmd == F("log"))    printEventLog();
  else if (cmd == F("mode"))   printModeStatus();
  else if (cmd.startsWith(F("set ")))    handleSet(cmd.substring(4));
  else if (cmd.startsWith(F("p1 ")))     handlePump(1, cmd.substring(3));
  else if (cmd.startsWith(F("p2 ")))     handlePump(2, cmd.substring(3));
  else if (cmd.startsWith(F("p3 ")))     handlePump(3, cmd.substring(3));
  else if (cmd.startsWith(F("p4 ")))     handleServo(cmd.substring(3));
  else if (cmd.startsWith(F("servo ")))  handleServo(cmd.substring(6));
  else {
    Serial.println(F("Commands:"));
    Serial.println(F("  status/st  live  auto  manual  reset  save  log  mode"));
    Serial.println(F("  p1/p2/p3 on/off"));
    Serial.println(F("  p4 open/close  OR  servo open/close"));
    Serial.println(F("  set [param] [val]"));
    Serial.println(F("  Params: water phmin phmax turb soil transfer waterdur"));
    Serial.println(F("          phoffset hystwater hystph hystturb"));
  }
}

// ================================================================
// SERVO - FULLY MANUAL, NO AUTO TIMING
// Commands: servo open | servo close | p4 open | p4 close
// ================================================================
void handleServo(String state) {
  state.trim();
  if (state == F("open") || state == F("on") || state == F("1")) {
    sv.attach(SERVO_PIN);
    sv.write(180);
    Serial.println(F("Servo: OPEN"));
    addEvent(F("Servo: opened"));
  } else if (state == F("close") || state == F("off") || state == F("0")) {
    sv.attach(SERVO_PIN);
    sv.write(0);
    delay(500);
    sv.detach();
    Serial.println(F("Servo: CLOSED"));
    addEvent(F("Servo: closed"));
  } else {
    Serial.println(F("Usage: servo open | servo close"));
  }
}

// ================================================================
// PUMP HANDLER
// ================================================================
void handlePump(int num, String state) {
  state.trim();
  bool on = (state == F("on") || state == F("1") || state == F("high"));
  setManualMode(true);

  if (num == 1) {
    manualP1 = on;
    digitalWrite(R_SUPPLY, on ? RELAY_ON : RELAY_OFF);
    Serial.print(F("P1 Supply: ")); Serial.println(on ? F("ON") : F("OFF"));
  }
  if (num == 2) {
    manualP2 = on;
    digitalWrite(R_FILTER, on ? RELAY_ON : RELAY_OFF);
    if (wsState != WS_IDLE) { wsState = WS_IDLE; addEvent(F("WaterState -> IDLE")); }
    Serial.print(F("P2 Filter: ")); Serial.println(on ? F("ON") : F("OFF"));
  }
  if (num == 3) {
    manualP3 = on;
    digitalWrite(R_PLANT, on ? RELAY_ON : RELAY_OFF);
    if (wsState != WS_IDLE) { wsState = WS_IDLE; addEvent(F("WaterState -> IDLE")); }
    Serial.print(F("P3 Plant: ")); Serial.println(on ? F("ON") : F("OFF"));
  }
}

void setManualMode(bool manual) {
  manualMode = manual;
  if (!manual) {
    manualP1 = manualP2 = manualP3 = false;
    addEvent(F("Mode: AUTO"));
  } else {
    addEvent(F("Mode: MANUAL"));
  }
  printModeStatus();
}

void printModeStatus() {
  Serial.print(F("MODE: ")); Serial.println(manualMode ? F("MANUAL") : F("AUTO"));
  Serial.print(F("P1:")); Serial.print(manualP1 ? F("MAN") : F("AUTO"));
  Serial.print(F(" P2:")); Serial.print(manualP2 ? F("MAN") : F("AUTO"));
  Serial.print(F(" P3:")); Serial.println(manualP3 ? F("MAN") : F("AUTO"));
}

// ================================================================
// SET HANDLER
// ================================================================
void handleSet(String param) {
  param.trim();
  int sp = param.indexOf(' ');
  String key = (sp < 0) ? param : param.substring(0, sp);
  String val = (sp < 0) ? String("") : param.substring(sp + 1);
  val.trim();

  float fv = val.toFloat();
  int   iv = (int)val.toInt();
  bool  ok = true;

  if      (key == F("water"))     wTarget   = iv;
  else if (key == F("phmin"))     { if (fv >= 0 && fv <= 14)    phMin     = fv; else ok = false; }
  else if (key == F("phmax"))     { if (fv >= 0 && fv <= 14)    phMax     = fv; else ok = false; }
  else if (key == F("turb"))      turbTh    = iv;
  else if (key == F("soil"))      soilTh    = iv;
  else if (key == F("transfer"))  { if (iv >= 1 && iv <= 300)   xferSec   = iv; else ok = false; }
  else if (key == F("waterdur"))  { if (iv >= 1 && iv <= 300)   waterSec  = iv; else ok = false; }
  else if (key == F("hystwater")) { if (iv >= 0 && iv <= 10)    waterHyst = iv; else ok = false; }
  else if (key == F("hystph"))    { if (fv >= 0 && fv <= 2.0f)  phHyst    = fv; else ok = false; }
  else if (key == F("hystturb"))  { if (iv >= 0 && iv <= 200)   turbHyst  = iv; else ok = false; }
  else if (key == F("phoffset"))  {
    if (fv >= -7.0f && fv <= 7.0f) {
      phOffset = fv;
      // Clear pH buffer so new offset takes effect immediately
      for (uint8_t i = 0; i < PH_AVG_SIZE; i++) phBuf[i] = 512;
      phBufIdx  = 0;
      phBufFull = false;
    } else ok = false;
  }
  else ok = false;

  if (ok) {
    Serial.print(key); Serial.print(F("=")); Serial.println(val);
    saveEEPROM();
    addEvent(F("Config updated"));
  } else {
    Serial.println(F("ERR: bad param or out of range"));
  }
}

// ================================================================
// EEPROM
// ================================================================
void loadEEPROM() {
  if (EEPROM.read(21) != 0xAB) {
    addEvent(F("EEPROM blank - defaults"));
    defaultEEPROM();
    return;
  }

  byte chk = 0;
  for (int i = 0; i < 20; i++) chk ^= EEPROM.read(i);
  if (chk != EEPROM.read(20)) {
    addEvent(F("EEPROM bad chk - defaults"));
    DBG("EEPROM checksum fail");
    defaultEEPROM();
    return;
  }

  EEPROM.get(0,  wTarget);
  EEPROM.get(2,  phMin);
  EEPROM.get(6,  phMax);
  EEPROM.get(10, turbTh);
  EEPROM.get(12, soilTh);
  EEPROM.get(16, xferSec);
  EEPROM.get(18, waterSec);
  EEPROM.get(22, phOffset);

  // Validate loaded floats - corrupt EEPROM can produce NaN
  if (isnan(phMin)    || phMin    < 0 || phMin    > 14)  phMin    = 6.5f;
  if (isnan(phMax)    || phMax    < 0 || phMax    > 14)  phMax    = 8.5f;
  if (isnan(phOffset) || phOffset < -7 || phOffset > 7)  phOffset = 1.7f;

  addEvent(F("Config loaded OK"));
  DBG("EEPROM load OK");
}

void saveEEPROM() {
  EEPROM.put(0,  wTarget);
  EEPROM.put(2,  phMin);
  EEPROM.put(6,  phMax);
  EEPROM.put(10, turbTh);
  EEPROM.put(12, soilTh);
  EEPROM.put(16, xferSec);
  EEPROM.put(18, waterSec);
  EEPROM.put(22, phOffset);

  byte chk = 0;
  for (int i = 0; i < 20; i++) chk ^= EEPROM.read(i);
  EEPROM.write(20, chk);
  EEPROM.write(21, 0xAB);

  addEvent(F("Config saved"));
}

void defaultEEPROM() {
  wTarget   = 5;
  phMin     = 6.5f;
  phMax     = 8.5f;
  turbTh    = 500;
  soilTh    = 800;
  xferSec   = 10;
  waterSec  = 15;
  waterHyst = 2;
  phHyst    = 0.2f;
  turbHyst  = 50;
  phOffset  = 1.7f;
  saveEEPROM();
}

// ================================================================
// STATUS OUTPUT
// ================================================================
void sendLiveStatus() {
  Serial.print(F("LIVE"));
  Serial.print(F("|T:"));   Serial.print(tempOK ? tempC : -127, 1);
  Serial.print(F("|pH:"));  Serial.print(phOK   ? phVal : -1,   2);
  Serial.print(F("|D:"));   Serial.print(distOK ? distCm : -1);
  Serial.print(F("|Tur:")); Serial.print(turbRaw);
  Serial.print(F("|Soil:")); Serial.print(soilRaw);
  Serial.print(F("|P1:")); Serial.print(digitalRead(R_SUPPLY) == RELAY_ON ? F("ON") : F("OFF"));
  Serial.print(F("|P2:")); Serial.print(digitalRead(R_FILTER) == RELAY_ON ? F("ON") : F("OFF"));
  Serial.print(F("|P3:")); Serial.print(digitalRead(R_PLANT)  == RELAY_ON ? F("ON") : F("OFF"));
  Serial.print(F("|WS:"));
  Serial.print(wsState == WS_IDLE ? F("IDLE") : wsState == WS_XFER ? F("XFER") : F("WATER"));
  Serial.print(F("|Mode:")); Serial.println(manualMode ? F("MAN") : F("AUTO"));
}

void sendJSONStatus() {
  Serial.println(F("{"));
  Serial.print(F("  \"temp\":")); 
  if (tempOK) Serial.print(tempC, 1); else Serial.print(F("-127"));
  Serial.println(F(","));

  Serial.print(F("  \"ph\":"));
  if (phOK) Serial.print(phVal, 2); else Serial.print(F("-1"));
  Serial.println(F(","));

  Serial.print(F("  \"phOffset\":")); Serial.print(phOffset, 2); Serial.println(F(","));
  Serial.print(F("  \"distance\":")); Serial.print(distOK ? distCm : -1); Serial.println(F(","));
  Serial.print(F("  \"turbidity\":")); Serial.print(turbRaw); Serial.println(F(","));
  Serial.print(F("  \"soil\":")); Serial.print(soilRaw); Serial.println(F(","));
  Serial.print(F("  \"phOK\":")); Serial.print(phOK ? F("true") : F("false")); Serial.println(F(","));
  Serial.print(F("  \"tempOK\":")); Serial.print(tempOK ? F("true") : F("false")); Serial.println(F(","));
  Serial.print(F("  \"distOK\":")); Serial.print(distOK ? F("true") : F("false")); Serial.println(F(","));
  Serial.print(F("  \"p1\":\"")); Serial.print(digitalRead(R_SUPPLY)==RELAY_ON ? F("ON") : F("OFF")); Serial.println(F("\","));
  Serial.print(F("  \"p2\":\"")); Serial.print(digitalRead(R_FILTER)==RELAY_ON ? F("ON") : F("OFF")); Serial.println(F("\","));
  Serial.print(F("  \"p3\":\"")); Serial.print(digitalRead(R_PLANT) ==RELAY_ON ? F("ON") : F("OFF")); Serial.println(F("\","));
  Serial.print(F("  \"watering\":\""));
  Serial.print(wsState==WS_IDLE ? F("IDLE") : wsState==WS_XFER ? F("XFER") : F("WATER"));
  Serial.println(F("\","));
  Serial.print(F("  \"mode\":\"")); Serial.print(manualMode ? F("MANUAL") : F("AUTO")); Serial.println(F("\""));
  Serial.println(F("}"));
}

void printEventLog() {
  Serial.println(F("=== EVENT LOG ==="));
  for (int i = 0; i < LOG_SIZE; i++) {
    int idx = (logHead + i) % LOG_SIZE;
    if (eventLog[idx][0]) Serial.println(eventLog[idx]);
  }
  Serial.println(F("================="));
}

// ================================================================
// LCD  (20x4)
// Line 0: T:xx.x  pH:xx.xx
// Line 1: D:xxcm  Tur:xxxx
// Line 2: S:xxxx P1:x P2:x P3:x
// Line 3: AUTO/MAN  IDLE/XFER/WATR [ERR]
// ================================================================
void updateLCD() {
  lcd.clear();

  // Line 0
  lcd.setCursor(0, 0);
  lcd.print(F("T:"));
  if (!tempOK) { lcd.print(F("ERR")); }
  else         { lcd.print(tempC, 1); }
  lcd.print(F(" pH:"));
  if (!phOK)   { lcd.print(F("ERR")); }
  else         { lcd.print(phVal, 2); }

  // Line 1
  lcd.setCursor(0, 1);
  lcd.print(F("D:"));
  if (!distOK) { lcd.print(F("---")); }
  else         { lcd.print(distCm); lcd.print(F("cm")); }
  lcd.print(F(" Tur:"));
  lcd.print(turbRaw);

  // Line 2
  lcd.setCursor(0, 2);
  lcd.print(F("S:"));
  lcd.print(soilRaw);
  lcd.print(F(" P1:"));
  lcd.print(digitalRead(R_SUPPLY)==RELAY_ON ? 'R' : 'G');
  lcd.print(F(" P2:"));
  lcd.print(digitalRead(R_FILTER)==RELAY_ON ? 'R' : 'G');
  lcd.print(F(" P3:"));
  lcd.print(digitalRead(R_PLANT) ==RELAY_ON ? 'R' : 'G');

  // Line 3
  lcd.setCursor(0, 3);
  lcd.print(manualMode ? F("MAN ") : F("AUTO"));
  lcd.print(' ');
  lcd.print(wsState==WS_IDLE ? F("IDLE") : wsState==WS_XFER ? F("XFER") : F("WATR"));
  if (!phOK || !tempOK || !distOK) lcd.print(F(" ERR"));
  if (liveMode) lcd.print(F(" LV"));
}

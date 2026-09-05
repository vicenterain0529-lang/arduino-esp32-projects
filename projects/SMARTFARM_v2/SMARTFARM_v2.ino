/*
  ============================================================
  SMARTFARM Controller v2.0
  Target  : Arduino Uno R3 (ATmega328P) — 2KB SRAM / 32KB Flash
  Author  : SMARTFARM Project
  Updated : 2025
  ============================================================

  WIRING SUMMARY
  ─────────────────────────────────────────────────────────────
  D2  → HC-05 BT module TX  (BT_RX on Arduino)
  D3  → HC-05 BT module RX  (BT_TX on Arduino)
  D4  → DHT11 data pin
  D5  → Relay IN1  (FAN)      — active LOW
  D6  → Relay IN2  (PUMP 1)   — active LOW
  D7  → Relay IN3  (PUMP 2)   — active LOW
  A0  → Soil moisture sensor 1 (Zone 1 left)
  A1  → Soil moisture sensor 2 (Zone 1 right)
  A2  → Soil moisture sensor 3 (Zone 2)
  A3  → MQ-series gas sensor (analog)
  SDA → LCD I2C SDA + RTC SDA  (A4 on Uno)
  SCL → LCD I2C SCL + RTC SCL  (A5 on Uno)
  ─────────────────────────────────────────────────────────────

  SERIAL / BLUETOOTH COMMAND REFERENCE  (9600 baud, \n terminated)
  ─────────────────────────────────────────────────────────────
  INFO & STATUS
    status          Full system status dump
    sensors         Raw + mapped sensor values
    time            Current RTC timestamp
    help            Command list

  DEVICE CONTROL (manual)
    pump1 on/off    Direct pump 1 control (no auto-timer)
    pump2 on/off    Direct pump 2 control (no auto-timer)
    fan on/off      Fan control
    allon           Timed run on all actuators
    alloff          Stop all actuators

  QUICK IRRIGATION
    zone1now        Timed pulse on pump 1  (PUMP_DURATION_MS)
    zone2now        Timed pulse on pump 2  (PUMP_DURATION_MS)
    waternow        Alias for zone1now
    pump2now        Alias for zone2now

  MODE
    autoon          Enable automation loop
    autooff         Disable automation loop (manual mode)
    mode auto       Same as autoon
    mode manual     Same as autooff

  LCD
    lcd on          Enable LCD + backlight
    lcd off         Disable LCD + backlight
    lcd page N      Jump to a specific LCD page (0-5)

  THRESHOLDS (tunable at runtime)
    set zone1 0-100     Soil % threshold for Zone 1 (default 35)
    set zone2 0-100     Soil % threshold for Zone 2 (default 35)
    set gas 0-1023      Gas raw ADC alarm level     (default 450)
    set temp 13-60      Fan turn-on temp °C         (default 31)
    set humidity 30-100 Fan turn-on humidity %      (default 85)
    set duration 1000-60000  Pump timed-run ms      (default 8000)
    set cooldown 60-7200     Auto cooldown seconds  (default 1800)

  DEBUG LAYER
    d               Toggle debug mode on/off
    debug           Same as 'd'
    d sensors       Verbose raw + pct + zone avg
    d flags         Current flags byte in binary
    d mem           Free SRAM estimate
    d relays        All relay pin states
    d thresholds    All current threshold values
    d timing        All timing counters (ms)
    d reset         Cold-reset all timing counters
  ─────────────────────────────────────────────────────────────

  LCD PAGE MAP  (auto-cycles every LCD_INTERVAL_MS)
  ─────────────────────────────────────────────────────────────
  Page 0 — MAIN UI (all key info condensed)
    Row0: Mode  FAN-state  Pump-states
    Row1: T:xx.xC H:xx% GAS:xxxx

  Page 1 — Zone 1 Soil
    Row0: S1:xxx% S2:xxx%
    Row1: Z1Avg:xxx% [PUMP1:ON/--]

  Page 2 — Zone 2 Soil
    Row0: S3:xxx%  GAS:xxxx
    Row1: Z2:xxx% [PUMP2:ON/--]

  Page 3 — Temperature / Humidity
    Row0: Temp: xx.xC [FAN:ON/--]
    Row1: Hum:  xx.x% Thr:xx

  Page 4 — Gas / Alerts
    Row0: GAS: xxxx  [ALERT/OK]
    Row1: Thr:xxxx Fan:[ON/--]

  Page 5 — RTC Clock
    Row0: RTC OK / NONE
    Row1: MM/DD HH:MM:SS

  DYNAMIC PAGES (auto-shown for 6 s on event, then returns to cycle)
    ALERT page  — shown immediately when gas/DHT alert fires
    PUMP page   — shown when any pump starts/stops
  ─────────────────────────────────────────────────────────────

  CALIBRATION
  ─────────────────────────────────────────────────────────────
  Soil dry ADC  = 850  (sensor in air)
  Soil wet ADC  = 350  (sensor submerged)
  Adjust SOIL_DRY_VALUE / SOIL_WET_VALUE to match your sensors.
  ─────────────────────────────────────────────────────────────

  SRAM NOTES
  ─────────────────────────────────────────────────────────────
  All constant strings stored in Flash via PROGMEM / F().
  Flags packed into a single uint8_t with bit macros.
  tempBuffer (32 B) is the sole shared formatting buffer.
  Use 'd mem' command to check live free-SRAM at runtime.
  SoftwareSerial RX buffer = 64 B (hidden, inside the library).
  Estimated static SRAM usage: ~450 B  Free: ~1550 B min.
  ─────────────────────────────────────────────────────────────

  KNOWN LIMITATIONS
  ─────────────────────────────────────────────────────────────
  - millis() rolls over after ~49 days. Cooldown sentinel (==0)
    may misbehave for one cooldown window post-rollover.
  - SoftwareSerial cannot RX and TX simultaneously.
  - DHT11 minimum sampling interval is 1 s; code uses 2 s.
  ─────────────────────────────────────────────────────────────
*/

#include <SoftwareSerial.h>
#include <DHT.h>
#include <Wire.h>
#include <LiquidCrystal_I2C.h>
#include <RTClib.h>

// ============================================================
//  PIN DEFINITIONS
// ============================================================
#define BT_RX        2
#define BT_TX        3
#define DHT_PIN      4
#define DHT_TYPE     DHT11
#define FAN_RELAY    5
#define PUMP1_RELAY  6
#define PUMP2_RELAY  7
#define SOIL1_PIN    A0
#define SOIL2_PIN    A1
#define SOIL3_PIN    A2
#define GAS_PIN      A3
#define LCD_ADDRESS  0x27
#define LCD_COLUMNS  16
#define LCD_ROWS     2
#define ON           LOW
#define OFF          HIGH

// ============================================================
//  FLAGS  — single byte, accessed via GET_FLAG / SET_FLAG
// ============================================================
// Bit layout:
//   0 = FLAG_AUTO_MODE     1=auto, 0=manual
//   1 = FLAG_RTC_OK        1=RTC detected
//   2 = FLAG_LCD_ENABLED   1=LCD on
//   3 = FLAG_DHT_OK        1=last DHT read valid
//   4 = FLAG_PUMP1_TIMED   1=pump1 running on timer
//   5 = FLAG_PUMP2_TIMED   1=pump2 running on timer
//   6 = FLAG_DHT_ALERT     1=DHT fault alert active
//   7 = FLAG_GAS_ALERT     1=gas alert active
uint8_t flags = 0b00000101;  // autoMode=1, lcdEnabled=1

#define FLAG_AUTO_MODE   0
#define FLAG_RTC_OK      1
#define FLAG_LCD_ENABLED 2
#define FLAG_DHT_OK      3
#define FLAG_PUMP1_TIMED 4
#define FLAG_PUMP2_TIMED 5
#define FLAG_DHT_ALERT   6
#define FLAG_GAS_ALERT   7

#define GET_FLAG(f)    (((flags) >> (f)) & 1)
#define SET_FLAG(f, v) flags = (v) ? (flags | (1<<(f))) : (flags & ~(1<<(f)))

// Debug mode flag (RAM bool — not packed, checked frequently)
bool debugMode = false;

// ============================================================
//  HARDWARE OBJECTS
// ============================================================
SoftwareSerial BT(BT_RX, BT_TX);
DHT            dht(DHT_PIN, DHT_TYPE);
LiquidCrystal_I2C lcd(LCD_ADDRESS, LCD_COLUMNS, LCD_ROWS);
RTC_DS3231     rtc;

// ============================================================
//  COMMAND BUFFERS
// ============================================================
char    btBuffer[32];
char    usbBuffer[32];
char    tempBuffer[32];   // shared formatting buffer — never nest calls that both use this
byte    btIndex  = 0;
byte    usbIndex = 0;

// ============================================================
//  SENSOR DATA
// ============================================================
int16_t  soilRaw[3];      // raw ADC 0-1023
uint8_t  soilPct[3];      // mapped 0-100 %
uint8_t  zone1Avg;        // (soilPct[0]+soilPct[1])/2
uint16_t gasRaw;          // raw ADC 0-1023
float    temperatureC = 0.0;
float    humidityPct  = 0.0;

// ============================================================
//  CALIBRATION
// ============================================================
const int SOIL_DRY_VALUE = 850;
const int SOIL_WET_VALUE = 350;

// ============================================================
//  THRESHOLDS  (tunable via serial)
// ============================================================
uint8_t  soilZone1Threshold = 35;
uint8_t  soilZone2Threshold = 35;
uint16_t gasThreshold       = 450;
float    fanTempOnC         = 31.0;
float    fanTempOffC        = 28.0;   // always fanTempOnC - 3
float    fanHumidityOn      = 85.0;

// ============================================================
//  TIMING
// ============================================================
const uint32_t SENSOR_INTERVAL_MS  = 2000UL;
const uint32_t LCD_INTERVAL_MS     = 3000UL;
const uint32_t ALERT_PAGE_HOLD_MS  = 6000UL;  // dynamic page hold time
const uint32_t EVENT_PAGE_HOLD_MS  = 4000UL;

uint32_t pumpDurationMs  = 8000UL;   // mutable via 'set duration'
uint32_t autoCooldownMs  = 1800000UL;// mutable via 'set cooldown' (30 min default)

uint32_t lastSensorReadMs  = 0;
uint32_t lastLcdUpdateMs   = 0;
uint32_t lastPump1AutoMs   = 0;
uint32_t lastPump2AutoMs   = 0;
uint32_t pump1StartMs      = 0;
uint32_t pump2StartMs      = 0;
uint32_t pump1DurationMs   = 0;
uint32_t pump2DurationMs   = 0;

// ============================================================
//  LCD PAGE MANAGEMENT
// ============================================================
// Normal pages cycle 0-5.  dynamicPage = -1 means no override.
#define LCD_PAGES       6
#define DPAGE_ALERT    10   // dynamic page: alert
#define DPAGE_PUMP     11   // dynamic page: pump event

uint8_t  lcdPage           = 0;        // current normal page
int8_t   dynamicPage       = -1;       // -1 = no dynamic override
uint32_t dynamicPageStartMs= 0;

// ============================================================
//  PROGMEM STRINGS  — stored in flash, not SRAM
// ============================================================
const char PROGMEM STR_BANNER[]   = "=== SMARTFARM v2.0 ===";
const char PROGMEM STR_READY[]    = "System READY.";
const char PROGMEM STR_SEP[]      = "------------------------";
const char PROGMEM STR_RTC_OK[]   = "RTC: OK";
const char PROGMEM STR_RTC_FAIL[] = "RTC: NOT FOUND";
const char PROGMEM STR_HELP[]     = "Type 'help' for commands.";
const char PROGMEM STR_STATUS[]   = "=== STATUS ===";
const char PROGMEM STR_SENSORS[]  = "=== SENSORS ===";
const char PROGMEM STR_CMDS[]     = "=== COMMANDS ===";
const char PROGMEM STR_DBG[]      = "=== DEBUG ===";

// PROGMEM command tokens
const char PROGMEM CMD_HELP[]      = "help";
const char PROGMEM CMD_STATUS[]    = "status";
const char PROGMEM CMD_SENSORS[]   = "sensors";
const char PROGMEM CMD_TIME[]      = "time";
const char PROGMEM CMD_ALLON[]     = "allon";
const char PROGMEM CMD_ALLOFF[]    = "alloff";
const char PROGMEM CMD_PUMP1ON[]   = "pump1on";
const char PROGMEM CMD_PUMP1OFF[]  = "pump1off";
const char PROGMEM CMD_PUMP2ON[]   = "pump2on";
const char PROGMEM CMD_PUMP2OFF[]  = "pump2off";
const char PROGMEM CMD_FANON[]     = "fanon";
const char PROGMEM CMD_FANOFF[]    = "fanoff";
const char PROGMEM CMD_AUTOON[]    = "autoon";
const char PROGMEM CMD_AUTOOFF[]   = "autooff";
const char PROGMEM CMD_ZONE1[]     = "zone1now";
const char PROGMEM CMD_WATER[]     = "waternow";
const char PROGMEM CMD_ZONE2[]     = "zone2now";
const char PROGMEM CMD_PUMP2NOW[]  = "pump2now";
const char PROGMEM CMD_D[]         = "d";
const char PROGMEM CMD_DEBUG[]     = "debug";

// ============================================================
//  FREE SRAM UTILITY
// ============================================================
int freeRam() {
  extern int __heap_start, *__brkval;
  int v;
  return (int)&v - (__brkval == 0 ? (int)&__heap_start : (int)__brkval);
}

// ============================================================
//  FUNCTION PROTOTYPES
// ============================================================
void readCommandStream(Stream &port, char *buffer, byte &index, const char *source);
void processCommand(char *cmd, const char *source);
void normalizeCommand(char *cmd);
void handleDeviceCommand(const char *device, const char *action);
void handleSetCommand(const char *target, const char *valueText);
void handleDebugCommand(const char *subcmd, const char *source);
void updateSensors(bool forceRead);
void runAutomation();
void updateTimedRuns();
void updateLcd();
void showLcdPage(uint8_t page);
void showLcdMainUI();
void showLcdZone1();
void showLcdZone2();
void showLcdTempHum();
void showLcdGas();
void showLcdRTC();
void showLcdAlert();
void showLcdPumpEvent();
void triggerDynamicPage(int8_t page);
void printStatus();
void printSensors();
void printTime();
void printHelp();
void printDebugSensors(const char *source);
void printDebugFlags(const char *source);
void printDebugMem(const char *source);
void printDebugRelays(const char *source);
void printDebugThresholds(const char *source);
void printDebugTiming(const char *source);
void sendAlertIfChanged(bool condition, uint8_t flagBit, const char *onMsg, const char *offMsg);
void setRelay(uint8_t pin, bool state, const char *name, bool announce);
bool relayIsOn(uint8_t pin);
void startTimedPump1(uint32_t durationMs, const char *reason, bool markAutoTime);
void startTimedPump2(uint32_t durationMs, const char *reason, bool markAutoTime);
void stopTimedPump1(const char *reason);
void stopTimedPump2(const char *reason);
uint8_t mapPercent(int raw, int rawLow, int rawHigh);
bool str_eq_P(const char *ram, const char *pgm);
void printPGM(const char *pgmStr);
void printLineF(const __FlashStringHelper *msg);
void printLine(const char *msg);
void printKeyVal(const char *key, const char *val);
void dbgLine(const char *msg, const char *source);

// ============================================================
//  SETUP
// ============================================================
void setup() {
  Serial.begin(9600);
  BT.begin(9600);
  dht.begin();
  Wire.begin();

  pinMode(FAN_RELAY,   OUTPUT);
  pinMode(PUMP1_RELAY, OUTPUT);
  pinMode(PUMP2_RELAY, OUTPUT);

  // Safe state: all relays OFF
  setRelay(FAN_RELAY,   OFF, NULL, false);
  setRelay(PUMP1_RELAY, OFF, NULL, false);
  setRelay(PUMP2_RELAY, OFF, NULL, false);

  // LCD startup splash
  lcd.init();
  lcd.backlight();
  lcd.clear();
  lcd.setCursor(0, 0); lcd.print(F("SMARTFARM  v2.0"));
  lcd.setCursor(0, 1); lcd.print(F("Initialising..."));

  // RTC
  bool rtcFound = rtc.begin();
  SET_FLAG(FLAG_RTC_OK, rtcFound);
  if (rtcFound && rtc.lostPower()) {
    rtc.adjust(DateTime(F(__DATE__), F(__TIME__)));
  }

  delay(1500);
  updateSensors(true);
  updateLcd();

  // Boot banner to both ports
  printPGM(STR_BANNER);
  printPGM(STR_SEP);
  printPGM(GET_FLAG(FLAG_RTC_OK) ? STR_RTC_OK : STR_RTC_FAIL);
  printLine(GET_FLAG(FLAG_AUTO_MODE) ? "Mode: AUTO" : "Mode: MANUAL");
  snprintf(tempBuffer, sizeof(tempBuffer), "Free SRAM: %d B", freeRam());
  printLine(tempBuffer);
  printPGM(STR_READY);
  printPGM(STR_HELP);
}

// ============================================================
//  MAIN LOOP
// ============================================================
void loop() {
  readCommandStream(BT,     btBuffer,  btIndex,  "BT");
  readCommandStream(Serial, usbBuffer, usbIndex, "USB");
  updateSensors(false);
  updateTimedRuns();
  runAutomation();
  updateLcd();
}

// ============================================================
//  COMMAND STREAM READER
// ============================================================
void readCommandStream(Stream &port, char *buffer, byte &index, const char *source) {
  while (port.available()) {
    char c = (char)port.read();
    if (c == '\n' || c == '\r') {
      buffer[index] = '\0';
      if (index > 0) processCommand(buffer, source);
      index = 0;
    } else if (index < 31) {
      buffer[index++] = c;
    } else {
      // Buffer overflow: flush and warn
      index = 0;
      printLine("ERROR: Cmd too long (max 31 chars)");
    }
  }
}

// ============================================================
//  COMMAND PROCESSOR
// ============================================================
void processCommand(char *cmd, const char *source) {
  normalizeCommand(cmd);
  if (cmd[0] == '\0') return;

  // Debug echo (only when debug mode is on)
  if (debugMode) {
    printPGM(STR_SEP);
    snprintf(tempBuffer, sizeof(tempBuffer), "[DBG][%s] CMD: %s", source, cmd);
    printLine(tempBuffer);
  }

  // ── Single-word PROGMEM commands ─────────────────────────
  if (str_eq_P(cmd, CMD_HELP))    { printHelp();    return; }
  if (str_eq_P(cmd, CMD_STATUS))  { printStatus();  return; }
  if (str_eq_P(cmd, CMD_SENSORS)) { printSensors(); return; }
  if (str_eq_P(cmd, CMD_TIME))    { printTime();    return; }

  if (str_eq_P(cmd, CMD_AUTOON))  { SET_FLAG(FLAG_AUTO_MODE, 1); printLineF(F("OK: MODE=AUTO"));   return; }
  if (str_eq_P(cmd, CMD_AUTOOFF)) { SET_FLAG(FLAG_AUTO_MODE, 0); printLineF(F("OK: MODE=MANUAL")); return; }

  if (str_eq_P(cmd, CMD_PUMP1ON))  { handleDeviceCommand("pump1", "on");  return; }
  if (str_eq_P(cmd, CMD_PUMP1OFF)) { handleDeviceCommand("pump1", "off"); return; }
  if (str_eq_P(cmd, CMD_PUMP2ON))  { handleDeviceCommand("pump2", "on");  return; }
  if (str_eq_P(cmd, CMD_PUMP2OFF)) { handleDeviceCommand("pump2", "off"); return; }
  if (str_eq_P(cmd, CMD_FANON))    { handleDeviceCommand("fan",   "on");  return; }
  if (str_eq_P(cmd, CMD_FANOFF))   { handleDeviceCommand("fan",   "off"); return; }

  if (str_eq_P(cmd, CMD_ALLON)) {
    startTimedPump1(pumpDurationMs, "allon", false);
    startTimedPump2(pumpDurationMs, "allon", false);
    setRelay(FAN_RELAY, ON, "FAN", true);
    printLineF(F("OK: ALL ON (timed)"));
    return;
  }
  if (str_eq_P(cmd, CMD_ALLOFF)) {
    stopTimedPump1("alloff");
    stopTimedPump2("alloff");
    setRelay(FAN_RELAY, OFF, "FAN", true);
    printLineF(F("OK: ALL OFF"));
    return;
  }

  if (str_eq_P(cmd, CMD_ZONE1) || str_eq_P(cmd, CMD_WATER)) {
    startTimedPump1(pumpDurationMs, "Manual Z1", false); return;
  }
  if (str_eq_P(cmd, CMD_ZONE2) || str_eq_P(cmd, CMD_PUMP2NOW)) {
    startTimedPump2(pumpDurationMs, "Manual Z2", false); return;
  }

  // ── Debug commands ────────────────────────────────────────
  if (str_eq_P(cmd, CMD_D) || str_eq_P(cmd, CMD_DEBUG)) {
    handleDebugCommand(NULL, source); return;
  }

  // ── Tokenise for multi-word commands ─────────────────────
  // IMPORTANT: strtok mutates cmd — all PROGMEM checks must be done above
  char *token = strtok(cmd, " ");
  char *arg1  = strtok(NULL, " ");
  char *arg2  = strtok(NULL, " ");

  if (!token) { printLineF(F("ERROR: Empty")); return; }

  // mode auto / mode manual
  if (strcmp(token, "mode") == 0 && arg1) {
    if (strcmp(arg1, "auto")   == 0) { SET_FLAG(FLAG_AUTO_MODE, 1); printLineF(F("OK: MODE=AUTO"));   return; }
    if (strcmp(arg1, "manual") == 0) { SET_FLAG(FLAG_AUTO_MODE, 0); printLineF(F("OK: MODE=MANUAL")); return; }
  }

  // pump1/pump2/fan on/off
  if ((strcmp(token,"pump1")==0 || strcmp(token,"pump2")==0 || strcmp(token,"fan")==0) && arg1) {
    handleDeviceCommand(token, arg1); return;
  }

  // lcd on / lcd off / lcd page N
  if (strcmp(token, "lcd") == 0 && arg1) {
    if (strcmp(arg1, "on") == 0) {
      SET_FLAG(FLAG_LCD_ENABLED, 1);
      lcd.backlight(); lcd.clear(); updateLcd();
      printLineF(F("OK: LCD=ON")); return;
    }
    if (strcmp(arg1, "off") == 0) {
      SET_FLAG(FLAG_LCD_ENABLED, 0);
      lcd.clear(); lcd.noBacklight();
      printLineF(F("OK: LCD=OFF")); return;
    }
    if (strcmp(arg1, "page") == 0 && arg2) {
      uint8_t pg = (uint8_t)constrain(atoi(arg2), 0, LCD_PAGES - 1);
      lcdPage = pg;
      dynamicPage = -1;        // cancel any dynamic override
      lastLcdUpdateMs = 0;     // force immediate redraw
      snprintf(tempBuffer, sizeof(tempBuffer), "OK: LCD PAGE=%d", pg);
      printLine(tempBuffer); return;
    }
  }

  // set <param> <value>
  if (strcmp(token, "set") == 0 && arg1 && arg2) {
    handleSetCommand(arg1, arg2); return;
  }

  // d <subcmd>  — debug with subcommand
  if (strcmp(token, "d") == 0) {
    handleDebugCommand(arg1, source); return;
  }

  printLineF(F("ERROR: Unknown cmd. Type 'help'."));
}

// ============================================================
//  NORMALIZE COMMAND  (trim, collapse spaces, lowercase)
// ============================================================
void normalizeCommand(char *cmd) {
  int start = 0;
  while (cmd[start] == ' ' || cmd[start] == '\t') start++;
  int len = strlen(cmd);
  while (len > start && (cmd[len-1] == ' ' || cmd[len-1] == '\t')) len--;

  int  dst = 0;
  bool prevSpace = false;
  for (int i = start; i < len; i++) {
    char c = cmd[i];
    if (c == '\t') c = ' ';
    if (c == ' ') {
      if (!prevSpace) { cmd[dst++] = c; prevSpace = true; }
    } else {
      cmd[dst++] = (char)tolower((unsigned char)c);
      prevSpace = false;
    }
  }
  cmd[dst] = '\0';
}

// ============================================================
//  DEVICE COMMAND HANDLER
// ============================================================
void handleDeviceCommand(const char *device, const char *action) {
  if (strcmp(device, "pump1") == 0) {
    if (strcmp(action, "on")  == 0) { SET_FLAG(FLAG_PUMP1_TIMED,0); setRelay(PUMP1_RELAY, ON,  "PUMP1", true); return; }
    if (strcmp(action, "off") == 0) { stopTimedPump1("Manual stop"); return; }
  }
  if (strcmp(device, "pump2") == 0) {
    if (strcmp(action, "on")  == 0) { SET_FLAG(FLAG_PUMP2_TIMED,0); setRelay(PUMP2_RELAY, ON,  "PUMP2", true); return; }
    if (strcmp(action, "off") == 0) { stopTimedPump2("Manual stop"); return; }
  }
  if (strcmp(device, "fan") == 0) {
    if (strcmp(action, "on")  == 0) { setRelay(FAN_RELAY, ON,  "FAN", true); return; }
    if (strcmp(action, "off") == 0) { setRelay(FAN_RELAY, OFF, "FAN", true); return; }
  }
  printLineF(F("ERROR: Invalid device/action"));
}

// ============================================================
//  SET COMMAND HANDLER
// ============================================================
void handleSetCommand(const char *target, const char *valueText) {
  long value = atol(valueText);

  if (strcmp(target,"zone1")==0 || strcmp(target,"soil1")==0) {
    soilZone1Threshold = (uint8_t)constrain(value, 0, 100);
    snprintf(tempBuffer, sizeof(tempBuffer), "OK: ZONE1_THR=%d%%", soilZone1Threshold);
    printLine(tempBuffer); return;
  }
  if (strcmp(target,"zone2")==0 || strcmp(target,"soil2")==0) {
    soilZone2Threshold = (uint8_t)constrain(value, 0, 100);
    snprintf(tempBuffer, sizeof(tempBuffer), "OK: ZONE2_THR=%d%%", soilZone2Threshold);
    printLine(tempBuffer); return;
  }
  if (strcmp(target,"gas")==0) {
    gasThreshold = (uint16_t)constrain(value, 0, 1023);
    snprintf(tempBuffer, sizeof(tempBuffer), "OK: GAS_THR=%d", gasThreshold);
    printLine(tempBuffer); return;
  }
  if (strcmp(target,"temp")==0) {
    // Minimum 13 so fanTempOffC (= on-3) stays above 10
    fanTempOnC  = (float)constrain(value, 13, 60);
    fanTempOffC = fanTempOnC - 3.0f;
    snprintf(tempBuffer, sizeof(tempBuffer), "OK: TEMP_ON=%.0fC OFF=%.0fC", fanTempOnC, fanTempOffC);
    printLine(tempBuffer); return;
  }
  if (strcmp(target,"humidity")==0 || strcmp(target,"humid")==0) {
    fanHumidityOn = (float)constrain(value, 30, 100);
    snprintf(tempBuffer, sizeof(tempBuffer), "OK: HUM_THR=%.0f%%", fanHumidityOn);
    printLine(tempBuffer); return;
  }
  if (strcmp(target,"duration")==0) {
    pumpDurationMs = (uint32_t)constrain(value, 1000L, 60000L);
    snprintf(tempBuffer, sizeof(tempBuffer), "OK: PUMP_DUR=%lus", pumpDurationMs/1000);
    printLine(tempBuffer); return;
  }
  if (strcmp(target,"cooldown")==0) {
    autoCooldownMs = (uint32_t)constrain(value, 60L, 7200L) * 1000UL;
    snprintf(tempBuffer, sizeof(tempBuffer), "OK: COOLDOWN=%lus", autoCooldownMs/1000);
    printLine(tempBuffer); return;
  }
  printLineF(F("ERROR: Valid: zone1/2 gas temp humidity duration cooldown"));
}

// ============================================================
//  DEBUG COMMAND HANDLER
// ============================================================
void handleDebugCommand(const char *subcmd, const char *source) {
  // 'd' with no subcommand — toggle debug mode
  if (!subcmd || subcmd[0] == '\0') {
    debugMode = !debugMode;
    snprintf(tempBuffer, sizeof(tempBuffer), "OK: DEBUG=%s", debugMode ? "ON" : "OFF");
    printLine(tempBuffer);
    return;
  }

  printPGM(STR_DBG);

  if (strcmp(subcmd, "sensors")    == 0) { printDebugSensors(source);    return; }
  if (strcmp(subcmd, "flags")      == 0) { printDebugFlags(source);      return; }
  if (strcmp(subcmd, "mem")        == 0) { printDebugMem(source);        return; }
  if (strcmp(subcmd, "relays")     == 0) { printDebugRelays(source);     return; }
  if (strcmp(subcmd, "thresholds") == 0) { printDebugThresholds(source); return; }
  if (strcmp(subcmd, "timing")     == 0) { printDebugTiming(source);     return; }

  if (strcmp(subcmd, "reset") == 0) {
    lastPump1AutoMs = 0;
    lastPump2AutoMs = 0;
    lastSensorReadMs = 0;
    printLineF(F("OK: Timing counters reset"));
    return;
  }

  printLineF(F("ERROR: d subcmds: sensors flags mem relays thresholds timing reset"));
}

// ── Debug sub-printers ──────────────────────────────────────

void printDebugSensors(const char *source) {
  updateSensors(true);
  snprintf(tempBuffer, sizeof(tempBuffer), "[%s] SOIL1 raw=%d pct=%d%%", source, soilRaw[0], soilPct[0]);
  printLine(tempBuffer);
  snprintf(tempBuffer, sizeof(tempBuffer), "[%s] SOIL2 raw=%d pct=%d%%", source, soilRaw[1], soilPct[1]);
  printLine(tempBuffer);
  snprintf(tempBuffer, sizeof(tempBuffer), "[%s] SOIL3 raw=%d pct=%d%%", source, soilRaw[2], soilPct[2]);
  printLine(tempBuffer);
  snprintf(tempBuffer, sizeof(tempBuffer), "[%s] Z1AVG=%d%% Z2=%d%%",    source, zone1Avg, soilPct[2]);
  printLine(tempBuffer);
  snprintf(tempBuffer, sizeof(tempBuffer), "[%s] GAS   raw=%d thr=%d",   source, gasRaw, gasThreshold);
  printLine(tempBuffer);
  if (GET_FLAG(FLAG_DHT_OK)) {
    // Use dtostrf to avoid float in snprintf on AVR
    char ts[8], hs[8];
    dtostrf(temperatureC, 5, 1, ts);
    dtostrf(humidityPct,  5, 1, hs);
    snprintf(tempBuffer, sizeof(tempBuffer), "[%s] TEMP=%sC HUM=%s%%", source, ts, hs);
    printLine(tempBuffer);
  } else {
    snprintf(tempBuffer, sizeof(tempBuffer), "[%s] DHT: FAILED", source);
    printLine(tempBuffer);
  }
}

void printDebugFlags(const char *source) {
  // Print flags byte as binary string
  char bin[9];
  for (int i = 7; i >= 0; i--) bin[7-i] = ((flags >> i) & 1) ? '1' : '0';
  bin[8] = '\0';
  snprintf(tempBuffer, sizeof(tempBuffer), "[%s] flags=0b%s (0x%02X)", source, bin, flags);
  printLine(tempBuffer);
  snprintf(tempBuffer, sizeof(tempBuffer), "  AUTO=%d RTC=%d LCD=%d DHT=%d",
           GET_FLAG(FLAG_AUTO_MODE), GET_FLAG(FLAG_RTC_OK),
           GET_FLAG(FLAG_LCD_ENABLED), GET_FLAG(FLAG_DHT_OK));
  printLine(tempBuffer);
  snprintf(tempBuffer, sizeof(tempBuffer), "  P1T=%d P2T=%d DHT_ALT=%d GAS_ALT=%d",
           GET_FLAG(FLAG_PUMP1_TIMED), GET_FLAG(FLAG_PUMP2_TIMED),
           GET_FLAG(FLAG_DHT_ALERT),   GET_FLAG(FLAG_GAS_ALERT));
  printLine(tempBuffer);
  snprintf(tempBuffer, sizeof(tempBuffer), "  debugMode=%d", debugMode ? 1 : 0);
  printLine(tempBuffer);
}

void printDebugMem(const char *source) {
  int fr = freeRam();
  snprintf(tempBuffer, sizeof(tempBuffer), "[%s] Free SRAM: %d B", source, fr);
  printLine(tempBuffer);
  if (fr < 200) printLineF(F("WARN: Low SRAM — risk of corruption!"));
}

void printDebugRelays(const char *source) {
  snprintf(tempBuffer, sizeof(tempBuffer), "[%s] FAN   D%d pin=%d ON=%d",
           source, FAN_RELAY,   digitalRead(FAN_RELAY),   relayIsOn(FAN_RELAY)   ? 1:0);
  printLine(tempBuffer);
  snprintf(tempBuffer, sizeof(tempBuffer), "[%s] PUMP1 D%d pin=%d ON=%d",
           source, PUMP1_RELAY, digitalRead(PUMP1_RELAY), relayIsOn(PUMP1_RELAY) ? 1:0);
  printLine(tempBuffer);
  snprintf(tempBuffer, sizeof(tempBuffer), "[%s] PUMP2 D%d pin=%d ON=%d",
           source, PUMP2_RELAY, digitalRead(PUMP2_RELAY), relayIsOn(PUMP2_RELAY) ? 1:0);
  printLine(tempBuffer);
}

void printDebugThresholds(const char *source) {
  char ts[8], tf[8], fh[8];
  dtostrf(fanTempOnC,   4, 1, ts);
  dtostrf(fanTempOffC,  4, 1, tf);
  dtostrf(fanHumidityOn,4, 1, fh);
  snprintf(tempBuffer, sizeof(tempBuffer), "[%s] Z1_THR=%d%% Z2_THR=%d%%", source, soilZone1Threshold, soilZone2Threshold);
  printLine(tempBuffer);
  snprintf(tempBuffer, sizeof(tempBuffer), "[%s] GAS_THR=%d", source, gasThreshold);
  printLine(tempBuffer);
  snprintf(tempBuffer, sizeof(tempBuffer), "[%s] FAN_ON=%sC OFF=%sC HUM=%s%%", source, ts, tf, fh);
  printLine(tempBuffer);
  snprintf(tempBuffer, sizeof(tempBuffer), "[%s] PUMP_DUR=%lus COOLDOWN=%lus",
           source, pumpDurationMs/1000, autoCooldownMs/1000);
  printLine(tempBuffer);
}

void printDebugTiming(const char *source) {
  uint32_t now = millis();
  snprintf(tempBuffer, sizeof(tempBuffer), "[%s] millis=%lu", source, now);
  printLine(tempBuffer);
  snprintf(tempBuffer, sizeof(tempBuffer), "  lastSensor=%lu (%lus ago)", lastSensorReadMs, (now-lastSensorReadMs)/1000);
  printLine(tempBuffer);
  snprintf(tempBuffer, sizeof(tempBuffer), "  lastLcd=%lu (%lus ago)", lastLcdUpdateMs, (now-lastLcdUpdateMs)/1000);
  printLine(tempBuffer);
  snprintf(tempBuffer, sizeof(tempBuffer), "  p1Auto=%lu p2Auto=%lu", lastPump1AutoMs, lastPump2AutoMs);
  printLine(tempBuffer);
  if (GET_FLAG(FLAG_PUMP1_TIMED)) {
    snprintf(tempBuffer, sizeof(tempBuffer), "  PUMP1 running: %lums / %lums", now-pump1StartMs, pump1DurationMs);
    printLine(tempBuffer);
  }
  if (GET_FLAG(FLAG_PUMP2_TIMED)) {
    snprintf(tempBuffer, sizeof(tempBuffer), "  PUMP2 running: %lums / %lums", now-pump2StartMs, pump2DurationMs);
    printLine(tempBuffer);
  }
}

// ============================================================
//  SENSOR UPDATE
// ============================================================
void updateSensors(bool forceRead) {
  uint32_t now = millis();
  if (!forceRead && (now - lastSensorReadMs) < SENSOR_INTERVAL_MS) return;
  lastSensorReadMs = now;

  soilRaw[0] = analogRead(SOIL1_PIN);
  soilRaw[1] = analogRead(SOIL2_PIN);
  soilRaw[2] = analogRead(SOIL3_PIN);
  gasRaw     = analogRead(GAS_PIN);

  soilPct[0] = mapPercent(soilRaw[0], SOIL_DRY_VALUE, SOIL_WET_VALUE);
  soilPct[1] = mapPercent(soilRaw[1], SOIL_DRY_VALUE, SOIL_WET_VALUE);
  soilPct[2] = mapPercent(soilRaw[2], SOIL_DRY_VALUE, SOIL_WET_VALUE);
  zone1Avg   = (uint8_t)(((uint16_t)soilPct[0] + soilPct[1]) / 2);

  float newHum  = dht.readHumidity();
  float newTemp = dht.readTemperature();
  if (!isnan(newHum) && !isnan(newTemp)) {
    humidityPct  = newHum;
    temperatureC = newTemp;
    SET_FLAG(FLAG_DHT_OK, 1);
  } else {
    SET_FLAG(FLAG_DHT_OK, 0);
  }

  bool prevGasAlert = GET_FLAG(FLAG_GAS_ALERT);
  sendAlertIfChanged(!GET_FLAG(FLAG_DHT_OK),     FLAG_DHT_ALERT, "ALERT: DHT fail",  "INFO: DHT OK");
  sendAlertIfChanged(gasRaw >= gasThreshold,      FLAG_GAS_ALERT, "ALERT: Gas HIGH!", "INFO: Gas normal");

  // Trigger alert LCD page on new alert
  if (!prevGasAlert && GET_FLAG(FLAG_GAS_ALERT)) {
    triggerDynamicPage(DPAGE_ALERT);
  }

  // Debug: print readings every cycle if debug on
  if (debugMode) {
    snprintf(tempBuffer, sizeof(tempBuffer), "[DBG] S=%d/%d/%d G=%d",
             soilPct[0], soilPct[1], soilPct[2], gasRaw);
    printLine(tempBuffer);
  }
}

// ============================================================
//  AUTOMATION
// ============================================================
void runAutomation() {
  if (!GET_FLAG(FLAG_AUTO_MODE)) return;

  uint32_t now = millis();
  bool zone1Ready = (lastPump1AutoMs == 0) || ((now - lastPump1AutoMs) >= autoCooldownMs);
  bool zone2Ready = (lastPump2AutoMs == 0) || ((now - lastPump2AutoMs) >= autoCooldownMs);

  if (!relayIsOn(PUMP1_RELAY) && !GET_FLAG(FLAG_PUMP1_TIMED)
      && zone1Ready && zone1Avg < soilZone1Threshold) {
    startTimedPump1(pumpDurationMs, "Auto Z1", true);
  }

  if (!relayIsOn(PUMP2_RELAY) && !GET_FLAG(FLAG_PUMP2_TIMED)
      && zone2Ready && soilPct[2] < soilZone2Threshold) {
    startTimedPump2(pumpDurationMs, "Auto Z2", true);
  }

  if (GET_FLAG(FLAG_DHT_OK)) {
    bool shouldFan = (temperatureC >= fanTempOnC)
                  || (humidityPct  >= fanHumidityOn)
                  || (gasRaw       >= gasThreshold);
    if (!relayIsOn(FAN_RELAY) && shouldFan) {
      setRelay(FAN_RELAY, ON, NULL, false);
      printLineF(F("INFO: Fan ON (auto)"));
    } else if (relayIsOn(FAN_RELAY)
               && temperatureC <= fanTempOffC
               && humidityPct   < fanHumidityOn
               && gasRaw        < gasThreshold) {
      setRelay(FAN_RELAY, OFF, NULL, false);
      printLineF(F("INFO: Fan OFF (auto)"));
    }
  } else if (gasRaw >= gasThreshold && !relayIsOn(FAN_RELAY)) {
    setRelay(FAN_RELAY, ON, NULL, false);
    printLineF(F("INFO: Fan ON (gas safety)"));
  }
}

// ============================================================
//  TIMED PUMP UPDATE
// ============================================================
void updateTimedRuns() {
  uint32_t now = millis();
  if (GET_FLAG(FLAG_PUMP1_TIMED) && (now - pump1StartMs) >= pump1DurationMs)
    stopTimedPump1("Timed finish");
  if (GET_FLAG(FLAG_PUMP2_TIMED) && (now - pump2StartMs) >= pump2DurationMs)
    stopTimedPump2("Timed finish");
}

// ============================================================
//  LCD — MAIN UPDATE DISPATCHER
// ============================================================
void updateLcd() {
  if (!GET_FLAG(FLAG_LCD_ENABLED)) return;

  uint32_t now = millis();

  // Handle dynamic page expiry
  if (dynamicPage >= 0 && (now - dynamicPageStartMs) >= ALERT_PAGE_HOLD_MS) {
    dynamicPage = -1;
    lastLcdUpdateMs = 0; // force redraw of normal page immediately
  }

  if ((now - lastLcdUpdateMs) < LCD_INTERVAL_MS) return;
  lastLcdUpdateMs = now;

  lcd.clear();

  if (dynamicPage >= 0) {
    showLcdPage((uint8_t)dynamicPage);
    return;
  }

  showLcdPage(lcdPage);
  lcdPage = (lcdPage + 1) % LCD_PAGES;
}

void showLcdPage(uint8_t page) {
  switch (page) {
    case 0:             showLcdMainUI();    break;
    case 1:             showLcdZone1();     break;
    case 2:             showLcdZone2();     break;
    case 3:             showLcdTempHum();   break;
    case 4:             showLcdGas();       break;
    case 5:             showLcdRTC();       break;
    case DPAGE_ALERT:   showLcdAlert();     break;
    case DPAGE_PUMP:    showLcdPumpEvent(); break;
    default:            showLcdMainUI();    break;
  }
}

void triggerDynamicPage(int8_t page) {
  dynamicPage       = page;
  dynamicPageStartMs= millis();
  lastLcdUpdateMs   = 0; // force immediate draw
}

// ── Page 0: MAIN UI (all critical info) ─────────────────────
// Row0: A/M P1:X P2:X FAN:X     (mode + relay states)
// Row1: T:00.0 H:00% G:000
void showLcdMainUI() {
  // Row 0 — Mode + relay states
  lcd.setCursor(0, 0);
  lcd.print(GET_FLAG(FLAG_AUTO_MODE) ? F("A") : F("M"));
  lcd.print(F(" P1:"));
  lcd.print(relayIsOn(PUMP1_RELAY) ? F("O") : F("-"));
  lcd.print(F(" P2:"));
  lcd.print(relayIsOn(PUMP2_RELAY) ? F("O") : F("-"));
  lcd.print(F(" F:"));
  lcd.print(relayIsOn(FAN_RELAY) ? F("O") : F("-"));

  // Row 1 — Temp / Humidity / Gas
  lcd.setCursor(0, 1);
  if (GET_FLAG(FLAG_DHT_OK)) {
    // Format temp as XX.X (5 chars) + space + H:XX + space + G:XXXX
    lcd.print(F("T:"));
    lcd.print(temperatureC, 1);
    lcd.print(F(" H:"));
    lcd.print((int)humidityPct);
    lcd.print(F("%"));
  } else {
    lcd.print(F("DHT ERR G:"));
    lcd.print(gasRaw);
  }
}

// ── Page 1: Zone 1 soil ──────────────────────────────────────
void showLcdZone1() {
  lcd.setCursor(0, 0);
  lcd.print(F("S1:"));
  lcd.print(soilPct[0]);
  lcd.print(F("% S2:"));
  lcd.print(soilPct[1]);
  lcd.print(F("%"));

  lcd.setCursor(0, 1);
  lcd.print(F("Z1Avg:"));
  lcd.print(zone1Avg);
  lcd.print(F("% P1:"));
  lcd.print(relayIsOn(PUMP1_RELAY) ? F("ON") : F("--"));
}

// ── Page 2: Zone 2 soil ──────────────────────────────────────
void showLcdZone2() {
  lcd.setCursor(0, 0);
  lcd.print(F("S3:"));
  lcd.print(soilPct[2]);
  lcd.print(F("% G:"));
  lcd.print(gasRaw);

  lcd.setCursor(0, 1);
  lcd.print(F("Z2:"));
  lcd.print(soilPct[2]);
  lcd.print(F("% P2:"));
  lcd.print(relayIsOn(PUMP2_RELAY) ? F("ON") : F("--"));
}

// ── Page 3: Temperature / Humidity ───────────────────────────
void showLcdTempHum() {
  lcd.setCursor(0, 0);
  if (GET_FLAG(FLAG_DHT_OK)) {
    lcd.print(F("Tmp:"));
    lcd.print(temperatureC, 1);
    lcd.print(F("C"));
  } else {
    lcd.print(F("Tmp: DHT ERR"));
  }
  lcd.print(F(" F:"));
  lcd.print(relayIsOn(FAN_RELAY) ? F("O") : F("-"));

  lcd.setCursor(0, 1);
  if (GET_FLAG(FLAG_DHT_OK)) {
    lcd.print(F("Hum:"));
    lcd.print((int)humidityPct);
    lcd.print(F("% T>"));
    lcd.print((int)fanTempOnC);
    lcd.print(F("C"));
  } else {
    lcd.print(F("Check DHT11 wiring"));
  }
}

// ── Page 4: Gas ───────────────────────────────────────────────
void showLcdGas() {
  lcd.setCursor(0, 0);
  lcd.print(F("GAS:"));
  lcd.print(gasRaw);
  lcd.print(GET_FLAG(FLAG_GAS_ALERT) ? F(" [!ALERT]") : F(" [OK]   "));

  lcd.setCursor(0, 1);
  lcd.print(F("Thr:"));
  lcd.print(gasThreshold);
  lcd.print(F(" Fan:"));
  lcd.print(relayIsOn(FAN_RELAY) ? F("ON") : F("--"));
}

// ── Page 5: RTC Clock ─────────────────────────────────────────
void showLcdRTC() {
  lcd.setCursor(0, 0);
  lcd.print(F("RTC:"));
  lcd.print(GET_FLAG(FLAG_RTC_OK) ? F("OK  ") : F("NONE"));

  lcd.setCursor(0, 1);
  if (GET_FLAG(FLAG_RTC_OK)) {
    DateTime n = rtc.now();
    snprintf(tempBuffer, sizeof(tempBuffer), "%02d/%02d %02d:%02d:%02d",
             n.month(), n.day(), n.hour(), n.minute(), n.second());
    lcd.print(tempBuffer);
  } else {
    lcd.print(F("--/-- --:--:--"));
  }
}

// ── Dynamic: Alert page ───────────────────────────────────────
void showLcdAlert() {
  lcd.setCursor(0, 0);
  if (GET_FLAG(FLAG_GAS_ALERT)) {
    lcd.print(F("!! GAS  ALERT !!"));
  } else if (GET_FLAG(FLAG_DHT_ALERT)) {
    lcd.print(F("!! DHT  FAULT  !"));
  } else {
    lcd.print(F("!! ALERT !!     "));
  }
  lcd.setCursor(0, 1);
  lcd.print(F("GAS:"));
  lcd.print(gasRaw);
  lcd.print(F(" Thr:"));
  lcd.print(gasThreshold);
}

// ── Dynamic: Pump event page ──────────────────────────────────
void showLcdPumpEvent() {
  lcd.setCursor(0, 0);
  lcd.print(F("PUMP EVENT      "));
  lcd.setCursor(0, 1);
  lcd.print(F("P1:"));
  lcd.print(relayIsOn(PUMP1_RELAY) ? F("RUN") : F("STP"));
  lcd.print(F(" P2:"));
  lcd.print(relayIsOn(PUMP2_RELAY) ? F("RUN") : F("STP"));
}

// ============================================================
//  STATUS / SENSORS / TIME / HELP PRINTERS
// ============================================================
void printStatus() {
  updateSensors(true);
  char ts[8], hs[8];
  dtostrf(temperatureC, 5, 1, ts);
  dtostrf(humidityPct,  5, 1, hs);

  printPGM(STR_STATUS);
  printLine(GET_FLAG(FLAG_AUTO_MODE) ? "MODE: AUTO" : "MODE: MANUAL");

  snprintf(tempBuffer, sizeof(tempBuffer), "PUMP1: %s%s",
           relayIsOn(PUMP1_RELAY) ? "ON" : "OFF",
           GET_FLAG(FLAG_PUMP1_TIMED) ? " (timed)" : "");
  printLine(tempBuffer);

  snprintf(tempBuffer, sizeof(tempBuffer), "PUMP2: %s%s",
           relayIsOn(PUMP2_RELAY) ? "ON" : "OFF",
           GET_FLAG(FLAG_PUMP2_TIMED) ? " (timed)" : "");
  printLine(tempBuffer);

  snprintf(tempBuffer, sizeof(tempBuffer), "FAN:   %s", relayIsOn(FAN_RELAY) ? "ON" : "OFF");
  printLine(tempBuffer);
  snprintf(tempBuffer, sizeof(tempBuffer), "ZONE1: %d%%  (S1=%d%% S2=%d%%)", zone1Avg, soilPct[0], soilPct[1]);
  printLine(tempBuffer);
  snprintf(tempBuffer, sizeof(tempBuffer), "ZONE2: %d%%", soilPct[2]);
  printLine(tempBuffer);
  snprintf(tempBuffer, sizeof(tempBuffer), "GAS:   %d  (%s)", gasRaw, GET_FLAG(FLAG_GAS_ALERT) ? "ALERT" : "OK");
  printLine(tempBuffer);

  if (GET_FLAG(FLAG_DHT_OK)) {
    snprintf(tempBuffer, sizeof(tempBuffer), "TEMP:  %sC", ts);  printLine(tempBuffer);
    snprintf(tempBuffer, sizeof(tempBuffer), "HUM:   %s%%", hs); printLine(tempBuffer);
  } else {
    printLineF(F("TEMP/HUM: DHT ERROR"));
  }

  snprintf(tempBuffer, sizeof(tempBuffer), "DEBUG: %s", debugMode ? "ON" : "OFF");
  printLine(tempBuffer);
  snprintf(tempBuffer, sizeof(tempBuffer), "FREE SRAM: %d B", freeRam());
  printLine(tempBuffer);

  printTime();
  printPGM(STR_SEP);
}

void printSensors() {
  updateSensors(true);
  printPGM(STR_SENSORS);
  for (uint8_t i = 0; i < 3; i++) {
    snprintf(tempBuffer, sizeof(tempBuffer), "SOIL%d: raw=%d  pct=%d%%", i+1, soilRaw[i], soilPct[i]);
    printLine(tempBuffer);
  }
  snprintf(tempBuffer, sizeof(tempBuffer), "ZONE1 AVG: %d%%", zone1Avg);
  printLine(tempBuffer);
  snprintf(tempBuffer, sizeof(tempBuffer), "GAS:  raw=%d  thr=%d  %s",
           gasRaw, gasThreshold, GET_FLAG(FLAG_GAS_ALERT) ? "[ALERT]" : "[OK]");
  printLine(tempBuffer);
  if (GET_FLAG(FLAG_DHT_OK)) {
    char ts[8], hs[8];
    dtostrf(temperatureC, 5, 1, ts);
    dtostrf(humidityPct,  5, 1, hs);
    snprintf(tempBuffer, sizeof(tempBuffer), "TEMP: %sC  HUM: %s%%", ts, hs);
    printLine(tempBuffer);
  } else {
    printLineF(F("DHT: FAILED"));
  }
  printPGM(STR_SEP);
}

void printTime() {
  if (!GET_FLAG(FLAG_RTC_OK)) { printLineF(F("TIME: RTC NOT READY")); return; }
  DateTime n = rtc.now();
  snprintf(tempBuffer, sizeof(tempBuffer), "TIME: %02d/%02d/%04d %02d:%02d:%02d",
           n.month(), n.day(), n.year(), n.hour(), n.minute(), n.second());
  printLine(tempBuffer);
}

void printHelp() {
  printPGM(STR_CMDS);
  printLineF(F("status  sensors  time  help"));
  printLineF(F("mode auto/manual"));
  printLineF(F("pump1/2 on/off | fan on/off"));
  printLineF(F("zone1now  zone2now  waternow"));
  printLineF(F("allon  alloff"));
  printLineF(F("lcd on/off | lcd page 0-5"));
  printLineF(F("set zone1/2 0-100"));
  printLineF(F("set gas 0-1023"));
  printLineF(F("set temp 13-60"));
  printLineF(F("set humidity 30-100"));
  printLineF(F("set duration 1000-60000"));
  printLineF(F("set cooldown 60-7200"));
  printLineF(F("--- DEBUG ---"));
  printLineF(F("d              toggle debug"));
  printLineF(F("d sensors/flags/mem"));
  printLineF(F("d relays/thresholds/timing"));
  printLineF(F("d reset        reset timers"));
  printPGM(STR_SEP);
}

// ============================================================
//  UTILITY
// ============================================================
void sendAlertIfChanged(bool condition, uint8_t flagBit,
                        const char *onMsg, const char *offMsg) {
  bool current = GET_FLAG(flagBit);
  if (condition != current) {
    SET_FLAG(flagBit, condition);
    printLine(condition ? onMsg : offMsg);
  }
}

void setRelay(uint8_t pin, bool state, const char *name, bool announce) {
  bool changed = (digitalRead(pin) != (uint8_t)state);
  digitalWrite(pin, state);
  if (announce && name) {
    snprintf(tempBuffer, sizeof(tempBuffer), "OK: %s=%s", name, state==ON ? "ON" : "OFF");
    printLine(tempBuffer);
  }
}

bool relayIsOn(uint8_t pin) {
  return digitalRead(pin) == ON;
}

void startTimedPump1(uint32_t durationMs, const char *reason, bool markAutoTime) {
  SET_FLAG(FLAG_PUMP1_TIMED, 1);
  pump1StartMs    = millis();
  pump1DurationMs = durationMs;
  if (markAutoTime) lastPump1AutoMs = pump1StartMs;
  setRelay(PUMP1_RELAY, ON, "PUMP1", true);
  snprintf(tempBuffer, sizeof(tempBuffer), "INFO: P1 start [%s] %lus", reason, durationMs/1000);
  printLine(tempBuffer);
  triggerDynamicPage(DPAGE_PUMP);
}

void startTimedPump2(uint32_t durationMs, const char *reason, bool markAutoTime) {
  SET_FLAG(FLAG_PUMP2_TIMED, 1);
  pump2StartMs    = millis();
  pump2DurationMs = durationMs;
  if (markAutoTime) lastPump2AutoMs = pump2StartMs;
  setRelay(PUMP2_RELAY, ON, "PUMP2", true);
  snprintf(tempBuffer, sizeof(tempBuffer), "INFO: P2 start [%s] %lus", reason, durationMs/1000);
  printLine(tempBuffer);
  triggerDynamicPage(DPAGE_PUMP);
}

void stopTimedPump1(const char *reason) {
  bool wasOn = GET_FLAG(FLAG_PUMP1_TIMED) || relayIsOn(PUMP1_RELAY);
  SET_FLAG(FLAG_PUMP1_TIMED, 0);
  pump1DurationMs = 0;
  setRelay(PUMP1_RELAY, OFF, "PUMP1", true);
  if (wasOn) {
    snprintf(tempBuffer, sizeof(tempBuffer), "INFO: P1 stop  [%s]", reason);
    printLine(tempBuffer);
    triggerDynamicPage(DPAGE_PUMP);
  }
}

void stopTimedPump2(const char *reason) {
  bool wasOn = GET_FLAG(FLAG_PUMP2_TIMED) || relayIsOn(PUMP2_RELAY);
  SET_FLAG(FLAG_PUMP2_TIMED, 0);
  pump2DurationMs = 0;
  setRelay(PUMP2_RELAY, OFF, "PUMP2", true);
  if (wasOn) {
    snprintf(tempBuffer, sizeof(tempBuffer), "INFO: P2 stop  [%s]", reason);
    printLine(tempBuffer);
    triggerDynamicPage(DPAGE_PUMP);
  }
}

uint8_t mapPercent(int raw, int rawLow, int rawHigh) {
  if (rawLow == rawHigh) return 0;
  long pct = map(raw, rawLow, rawHigh, 0, 100);
  return (uint8_t)constrain(pct, 0, 100);
}

// Safe PROGMEM string comparison (fixes prefix-match bug from v1)
bool str_eq_P(const char *ram, const char *pgm) {
  while (true) {
    char c1 = *ram++;
    char c2 = (char)pgm_read_byte(pgm++);
    if (c1 != c2) return false;
    if (c1 == '\0' && c2 == '\0') return true;  // both ended — true match
    if (c1 == '\0' || c2 == '\0') return false;  // one ended — prefix mismatch
  }
}

// Print a PROGMEM string to both Serial and BT
void printPGM(const char *pgmStr) {
  char c;
  while ((c = (char)pgm_read_byte(pgmStr++))) {
    Serial.print(c);
    BT.print(c);
  }
  Serial.println();
  BT.println();
}

// Print an F() string to both ports
void printLineF(const __FlashStringHelper *msg) {
  Serial.println(msg);
  BT.println(msg);
}

// Print a RAM string to both ports
void printLine(const char *msg) {
  Serial.println(msg);
  BT.println(msg);
}

// Print key: value pair using tempBuffer
void printKeyVal(const char *key, const char *val) {
  snprintf(tempBuffer, sizeof(tempBuffer), "%s: %s", key, val);
  printLine(tempBuffer);
}

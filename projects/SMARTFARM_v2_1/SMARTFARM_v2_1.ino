/*
  ============================================================
  SMARTFARM Controller v2.1  —  SRAM-OPTIMIZED
  Target  : Arduino Uno R3 (ATmega328P) — 2 KB SRAM / 32 KB Flash
  ============================================================

  KEY FIX vs v2.0
  ───────────────────────────────────────────────────────────────
  v2.0 used snprintf(..., "format %d", ...) — on AVR-GCC every
  format-string literal in a function body is copied into the
  .data segment (SRAM) at link time.  With 59 such strings that
  was 1,268 bytes of SRAM just for format strings, overflowing
  the 2 KB budget.

  v2.1 uses snprintf_P(..., PSTR("format %d"), ...) throughout.
  PSTR() marks the string for storage in Flash; snprintf_P reads
  it one byte at a time from Flash, never loading it into SRAM.

  SRAM BUDGET  (v2.1 estimate)
  ───────────────────────────────────────────────────────────────
  Explicit globals          ~160 B
  Library objects           ~302 B   (Serial 128, SoftSerial 68,
                                      Wire 34, DHT 12, LCD 40,
                                      RTC 20)
  String .data              ~20  B   (only printLineF F() which
                                      AVR stores in flash; PSTR
                                      strings also in flash)
  Stack worst-case          ~280 B
  ─────────────────────────────────────────────────────────────
  TOTAL                     ~762 B  /  2048 B  (~37% used)
  FREE                      ~1286 B             (~63% free)
  ─────────────────────────────────────────────────────────────
  Use 'd mem' at runtime to verify.  Target: never below 400 B.

  WIRING
  ───────────────────────────────────────────────────────────────
  D2  → HC-05 TX  (BT_RX)       D4  → DHT11 data
  D3  → HC-05 RX  (BT_TX)       D5  → Relay FAN  (active LOW)
  A0  → Soil sensor 1 (Zone 1)  D6  → Relay PUMP1 (active LOW)
  A1  → Soil sensor 2 (Zone 1)  D7  → Relay PUMP2 (active LOW)
  A2  → Soil sensor 3 (Zone 2)  SDA/SCL → LCD I2C + RTC DS3231
  A3  → MQ gas sensor

  COMMANDS  (9600 baud, send with \n)
  ───────────────────────────────────────────────────────────────
  status        Full status dump         sensors   Raw sensor dump
  time          RTC timestamp            help      Command list

  pump1 on/off  Pump 1 direct            pump2 on/off
  fan on/off    Fan direct
  allon         Timed run all            alloff    Stop all
  zone1now      Timed pump 1 pulse       zone2now  Timed pump 2
  waternow      Alias zone1now           pump2now  Alias zone2now

  autoon / autooff / mode auto / mode manual
  lcd on / lcd off / lcd page 0-5

  set zone1   0-100     Soil % threshold Zone 1  (def 35)
  set zone2   0-100     Soil % threshold Zone 2  (def 35)
  set gas     0-1023    Gas alarm level          (def 450)
  set temp    13-60     Fan on temp °C           (def 31)
  set humidity 30-100   Fan on humidity %        (def 85)
  set duration 1-60     Pump timed run seconds   (def 8)
  set cooldown 60-7200  Auto cooldown seconds    (def 1800)

  DEBUG COMMANDS
  ───────────────────────────────────────────────────────────────
  d               Toggle debug mode on/off
  debug           Same as 'd'
  d sensors       Verbose sensor dump (forced read)
  d flags         Flags byte in binary + named bits
  d mem           Free SRAM estimate + warning if < 400 B
  d relays        Pin states for all relays
  d thresholds    All tunable thresholds
  d timing        millis counters + pump run progress
  d reset         Zero all timing counters

  LCD PAGES  (auto-cycle every 3 s, or 'lcd page N' to jump)
  ───────────────────────────────────────────────────────────────
  0  MAIN  — A/M P1:x P2:x F:x / T:xx.x H:xx% G:xxxx
  1  ZONE1 — S1:xxx% S2:xxx% / Z1Avg:xxx% P1:ON/--
  2  ZONE2 — S3:xxx% G:xxxx / Z2:xxx% P2:ON/--
  3  TEMP  — Tmp:xx.xC F:O/- / Hum:xx% T>xx C
  4  GAS   — GAS:xxxx [!ALERT/OK] / Thr:xxxx Fan:ON/--
  5  RTC   — RTC:OK/NONE / MM/DD HH:MM:SS
  Dynamic ALERT page (6 s) on gas/DHT alert
  Dynamic PUMP  page (4 s) on pump start/stop

  CALIBRATION
  ───────────────────────────────────────────────────────────────
  SOIL_DRY_VALUE 850  (sensor in air)
  SOIL_WET_VALUE 350  (sensor submerged)
  Adjust to match your sensors.
  ============================================================
*/

#include <SoftwareSerial.h>
#include <DHT.h>
#include <Wire.h>
#include <LiquidCrystal_I2C.h>
#include <RTClib.h>

// ── Pin definitions ──────────────────────────────────────────
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
#define LCD_COLS     16
#define LCD_ROWS     2
#define ON           LOW
#define OFF          HIGH

// ── Flags byte  (bit positions) ──────────────────────────────
// 0=AUTO_MODE 1=RTC_OK 2=LCD_EN 3=DHT_OK 4=P1_TIMED 5=P2_TIMED
// 6=DHT_ALERT 7=GAS_ALERT
uint8_t flags = 0b00000101;  // autoMode=1, lcdEnabled=1
#define FLAG_AUTO_MODE   0
#define FLAG_RTC_OK      1
#define FLAG_LCD_ENABLED 2
#define FLAG_DHT_OK      3
#define FLAG_PUMP1_TIMED 4
#define FLAG_PUMP2_TIMED 5
#define FLAG_DHT_ALERT   6
#define FLAG_GAS_ALERT   7
#define GET_FLAG(f)    (((flags)>>(f))&1)
#define SET_FLAG(f,v)  flags=(v)?(flags|(1<<(f))):(flags&~(1<<(f)))

bool debugMode = false;

// ── Hardware ─────────────────────────────────────────────────
SoftwareSerial    BT(BT_RX, BT_TX);
DHT               dht(DHT_PIN, DHT_TYPE);
LiquidCrystal_I2C lcd(LCD_ADDRESS, LCD_COLS, LCD_ROWS);
RTC_DS3231        rtc;

// ── Buffers — 24 B each (longest cmd ~20 chars) ──────────────
char    btBuf[24], usbBuf[24], tmp[32];
byte    btIdx=0, usbIdx=0;

// ── Sensor data ───────────────────────────────────────────────
int16_t  soilRaw[3];
uint8_t  soilPct[3], zone1Avg;
uint16_t gasRaw;
float    tempC=0.0, humPct=0.0;

// ── Calibration (flash) ───────────────────────────────────────
const int SOIL_DRY = 850;
const int SOIL_WET = 350;

// ── Thresholds ────────────────────────────────────────────────
uint8_t  zThr1=35, zThr2=35;
uint16_t gasThr=450;
uint8_t  fanTempOn=31;    // integer °C — saves 4 B vs float pair
uint8_t  fanHumOn=85;     // integer %

// ── Timing ───────────────────────────────────────────────────
const uint32_t SENSOR_MS  = 2000UL;
const uint32_t LCD_MS     = 3000UL;
const uint32_t DYN_HOLD   = 6000UL;  // alert page hold
const uint32_t PUMP_HOLD  = 4000UL;  // pump event page hold

uint16_t pumpSec     = 8;      // pump timed-run seconds (1-60)
uint16_t cooldownSec = 1800;   // auto cooldown seconds  (60-7200)

uint32_t lastSensorMs=0, lastLcdMs=0;
uint32_t p1AutoMs=0, p2AutoMs=0;
uint32_t p1StartMs=0, p2StartMs=0;
uint32_t p1DurMs=0,   p2DurMs=0;

// ── LCD state ─────────────────────────────────────────────────
#define LCD_PAGES   6
#define DPAGE_NONE  255
#define DPAGE_ALERT 10
#define DPAGE_PUMP  11
uint8_t  lcdPage=0;
uint8_t  dynPage=DPAGE_NONE;
uint32_t dynStartMs=0;

// ── PROGMEM command tokens ────────────────────────────────────
const char PROGMEM C_HELP[]    ="help";
const char PROGMEM C_STATUS[]  ="status";
const char PROGMEM C_SENSORS[] ="sensors";
const char PROGMEM C_TIME[]    ="time";
const char PROGMEM C_ALLON[]   ="allon";
const char PROGMEM C_ALLOFF[]  ="alloff";
const char PROGMEM C_P1ON[]    ="pump1on";
const char PROGMEM C_P1OFF[]   ="pump1off";
const char PROGMEM C_P2ON[]    ="pump2on";
const char PROGMEM C_P2OFF[]   ="pump2off";
const char PROGMEM C_FANON[]   ="fanon";
const char PROGMEM C_FANOFF[]  ="fanoff";
const char PROGMEM C_AUTOON[]  ="autoon";
const char PROGMEM C_AUTOOFF[] ="autooff";
const char PROGMEM C_Z1NOW[]   ="zone1now";
const char PROGMEM C_WATER[]   ="waternow";
const char PROGMEM C_Z2NOW[]   ="zone2now";
const char PROGMEM C_P2NOW[]   ="pump2now";
const char PROGMEM C_D[]       ="d";
const char PROGMEM C_DEBUG[]   ="debug";

// ── PROGMEM banner strings ────────────────────────────────────
const char PROGMEM S_BANNER[] ="=== SMARTFARM v2.1 ===";
const char PROGMEM S_SEP[]    ="------------------------";
const char PROGMEM S_STATUS[] ="=== STATUS ===";
const char PROGMEM S_SENSORS[]="=== SENSORS ===";
const char PROGMEM S_CMDS[]   ="=== COMMANDS ===";
const char PROGMEM S_DBG[]    ="=== DEBUG ===";
const char PROGMEM S_RTC_OK[] ="RTC: OK";
const char PROGMEM S_RTC_NO[] ="RTC: NOT FOUND";

// ── Free SRAM helper ─────────────────────────────────────────
int freeRam() {
  extern int __heap_start, *__brkval;
  int v;
  return (int)&v - (__brkval==0 ? (int)&__heap_start : (int)__brkval);
}

// ── Forward declarations ──────────────────────────────────────
void readStream(Stream &p, char *buf, byte &idx, const char *src);
void processCmd(char *cmd, const char *src);
void normalizeCmd(char *cmd);
void doDevice(const char *dev, const char *act);
void doSet(const char *tgt, const char *val);
void doDebug(const char *sub);
void updateSensors(bool force);
void runAuto();
void updateTimers();
void updateLcd();
void showPage(uint8_t pg);
void trigDyn(uint8_t pg);
void printStatus();
void printSensors();
void printTime();
void printHelp();
void alertChanged(bool cond, uint8_t bit, const __FlashStringHelper *on,
                                          const __FlashStringHelper *off);
void setRelay(uint8_t pin, bool state, const __FlashStringHelper *name, bool ann);
bool relayOn(uint8_t pin);
void startP1(uint32_t ms, const __FlashStringHelper *why, bool mark);
void startP2(uint32_t ms, const __FlashStringHelper *why, bool mark);
void stopP1(const __FlashStringHelper *why);
void stopP2(const __FlashStringHelper *why);
uint8_t mapPct(int raw, int lo, int hi);
bool eqP(const char *ram, const char *pgm);
void pPGM(const char *pgm);
void pF(const __FlashStringHelper *s);
void pL(const char *s);

// ============================================================
//  SETUP
// ============================================================
void setup() {
  Serial.begin(9600);
  BT.begin(9600);
  dht.begin();
  Wire.begin();

  pinMode(FAN_RELAY,OUTPUT);
  pinMode(PUMP1_RELAY,OUTPUT);
  pinMode(PUMP2_RELAY,OUTPUT);
  setRelay(FAN_RELAY,  OFF, NULL, false);
  setRelay(PUMP1_RELAY,OFF, NULL, false);
  setRelay(PUMP2_RELAY,OFF, NULL, false);

  lcd.init(); lcd.backlight(); lcd.clear();
  lcd.setCursor(0,0); lcd.print(F("SMARTFARM  v2.1"));
  lcd.setCursor(0,1); lcd.print(F("Starting...    "));

  SET_FLAG(FLAG_RTC_OK, rtc.begin());
  if (GET_FLAG(FLAG_RTC_OK) && rtc.lostPower())
    rtc.adjust(DateTime(F(__DATE__),F(__TIME__)));

  delay(1200);
  updateSensors(true);
  updateLcd();

  pPGM(S_BANNER);
  pPGM(S_SEP);
  pPGM(GET_FLAG(FLAG_RTC_OK) ? S_RTC_OK : S_RTC_NO);
  pF(GET_FLAG(FLAG_AUTO_MODE) ? F("Mode: AUTO") : F("Mode: MANUAL"));
  snprintf_P(tmp,sizeof(tmp),PSTR("Free SRAM: %d B"),freeRam()); pL(tmp);
  pF(F("Type 'help' for commands."));
}

// ============================================================
//  LOOP
// ============================================================
void loop() {
  readStream(BT,    btBuf,  btIdx,  "BT");
  readStream(Serial,usbBuf, usbIdx, "USB");
  updateSensors(false);
  updateTimers();
  runAuto();
  updateLcd();
}

// ============================================================
//  STREAM READER
// ============================================================
void readStream(Stream &p, char *buf, byte &idx, const char *src) {
  while (p.available()) {
    char c = (char)p.read();
    if (c=='\n'||c=='\r') {
      buf[idx]='\0';
      if (idx>0) processCmd(buf,src);
      idx=0;
    } else if (idx<23) {
      buf[idx++]=c;
    } else {
      idx=0;
      pF(F("ERROR: Cmd too long"));
    }
  }
}

// ============================================================
//  COMMAND PROCESSOR
// ============================================================
void processCmd(char *cmd, const char *src) {
  normalizeCmd(cmd);
  if (!cmd[0]) return;

  if (debugMode) {
    snprintf_P(tmp,sizeof(tmp),PSTR("[DBG/%s] %s"),src,cmd);
    pL(tmp);
  }

  // ── Single-word PROGMEM checks first (before strtok) ──────
  if (eqP(cmd,C_HELP))    { printHelp();   return; }
  if (eqP(cmd,C_STATUS))  { printStatus(); return; }
  if (eqP(cmd,C_SENSORS)) { printSensors();return; }
  if (eqP(cmd,C_TIME))    { printTime();   return; }

  if (eqP(cmd,C_AUTOON))  { SET_FLAG(FLAG_AUTO_MODE,1); pF(F("OK: AUTO"));   return; }
  if (eqP(cmd,C_AUTOOFF)) { SET_FLAG(FLAG_AUTO_MODE,0); pF(F("OK: MANUAL")); return; }

  if (eqP(cmd,C_P1ON))  { doDevice("pump1","on");  return; }
  if (eqP(cmd,C_P1OFF)) { doDevice("pump1","off"); return; }
  if (eqP(cmd,C_P2ON))  { doDevice("pump2","on");  return; }
  if (eqP(cmd,C_P2OFF)) { doDevice("pump2","off"); return; }
  if (eqP(cmd,C_FANON)) { doDevice("fan",  "on");  return; }
  if (eqP(cmd,C_FANOFF)){ doDevice("fan",  "off"); return; }

  if (eqP(cmd,C_ALLON)) {
    startP1((uint32_t)pumpSec*1000UL, F("allon"), false);
    startP2((uint32_t)pumpSec*1000UL, F("allon"), false);
    setRelay(FAN_RELAY,ON,F("FAN"),true);
    pF(F("OK: ALL ON")); return;
  }
  if (eqP(cmd,C_ALLOFF)) {
    stopP1(F("alloff")); stopP2(F("alloff"));
    setRelay(FAN_RELAY,OFF,F("FAN"),true);
    pF(F("OK: ALL OFF")); return;
  }

  if (eqP(cmd,C_Z1NOW)||eqP(cmd,C_WATER)) {
    startP1((uint32_t)pumpSec*1000UL, F("Manual Z1"), false); return;
  }
  if (eqP(cmd,C_Z2NOW)||eqP(cmd,C_P2NOW)) {
    startP2((uint32_t)pumpSec*1000UL, F("Manual Z2"), false); return;
  }

  if (eqP(cmd,C_D)||eqP(cmd,C_DEBUG)) { doDebug(NULL); return; }

  // ── Multi-word: tokenize (mutates cmd) ─────────────────────
  char *tok=strtok(cmd," ");
  char *a1 =strtok(NULL," ");
  char *a2 =strtok(NULL," ");
  if (!tok) { pF(F("ERROR: Empty")); return; }

  if (!strcmp(tok,"mode")&&a1) {
    if (!strcmp(a1,"auto"))  { SET_FLAG(FLAG_AUTO_MODE,1); pF(F("OK: AUTO"));   return; }
    if (!strcmp(a1,"manual")){ SET_FLAG(FLAG_AUTO_MODE,0); pF(F("OK: MANUAL")); return; }
  }
  if ((!strcmp(tok,"pump1")||!strcmp(tok,"pump2")||!strcmp(tok,"fan"))&&a1) {
    doDevice(tok,a1); return;
  }
  if (!strcmp(tok,"lcd")&&a1) {
    if (!strcmp(a1,"on"))  { SET_FLAG(FLAG_LCD_ENABLED,1); lcd.backlight(); lcd.clear(); updateLcd(); pF(F("OK: LCD ON")); return; }
    if (!strcmp(a1,"off")) { SET_FLAG(FLAG_LCD_ENABLED,0); lcd.clear(); lcd.noBacklight(); pF(F("OK: LCD OFF")); return; }
    if (!strcmp(a1,"page")&&a2) {
      lcdPage=(uint8_t)constrain(atoi(a2),0,LCD_PAGES-1);
      dynPage=DPAGE_NONE; lastLcdMs=0;
      snprintf_P(tmp,sizeof(tmp),PSTR("OK: PAGE %d"),lcdPage); pL(tmp); return;
    }
  }
  if (!strcmp(tok,"set")&&a1&&a2) { doSet(a1,a2); return; }
  if (!strcmp(tok,"d"))            { doDebug(a1);  return; }

  pF(F("ERROR: Unknown. Type 'help'"));
}

// ============================================================
//  NORMALIZE
// ============================================================
void normalizeCmd(char *cmd) {
  int s=0; while(cmd[s]==' '||cmd[s]=='\t') s++;
  int n=strlen(cmd); while(n>s&&(cmd[n-1]==' '||cmd[n-1]=='\t')) n--;
  int d=0; bool sp=false;
  for(int i=s;i<n;i++){
    char c=cmd[i]; if(c=='\t')c=' ';
    if(c==' '){ if(!sp){cmd[d++]=c;sp=true;} }
    else{ cmd[d++]=tolower((unsigned char)c); sp=false; }
  }
  cmd[d]='\0';
}

// ============================================================
//  DEVICE COMMAND
// ============================================================
void doDevice(const char *dev, const char *act) {
  bool isOn=!strcmp(act,"on");
  if (!strcmp(dev,"pump1")) {
    if (isOn) { SET_FLAG(FLAG_PUMP1_TIMED,0); setRelay(PUMP1_RELAY,ON, F("PUMP1"),true); }
    else       stopP1(F("Manual"));
    return;
  }
  if (!strcmp(dev,"pump2")) {
    if (isOn) { SET_FLAG(FLAG_PUMP2_TIMED,0); setRelay(PUMP2_RELAY,ON, F("PUMP2"),true); }
    else       stopP2(F("Manual"));
    return;
  }
  if (!strcmp(dev,"fan")) {
    setRelay(FAN_RELAY, isOn?ON:OFF, F("FAN"), true);
    return;
  }
  pF(F("ERROR: Bad device"));
}

// ============================================================
//  SET COMMAND
// ============================================================
void doSet(const char *tgt, const char *val) {
  int v=atoi(val);
  if (!strcmp(tgt,"zone1")||!strcmp(tgt,"soil1")) {
    zThr1=(uint8_t)constrain(v,0,100);
    snprintf_P(tmp,sizeof(tmp),PSTR("OK: Z1=%d%%"),zThr1); pL(tmp); return;
  }
  if (!strcmp(tgt,"zone2")||!strcmp(tgt,"soil2")) {
    zThr2=(uint8_t)constrain(v,0,100);
    snprintf_P(tmp,sizeof(tmp),PSTR("OK: Z2=%d%%"),zThr2); pL(tmp); return;
  }
  if (!strcmp(tgt,"gas")) {
    gasThr=(uint16_t)constrain(v,0,1023);
    snprintf_P(tmp,sizeof(tmp),PSTR("OK: GAS=%d"),gasThr); pL(tmp); return;
  }
  if (!strcmp(tgt,"temp")) {
    fanTempOn=(uint8_t)constrain(v,13,60);
    snprintf_P(tmp,sizeof(tmp),PSTR("OK: T_ON=%dC OFF=%dC"),fanTempOn,fanTempOn-3); pL(tmp); return;
  }
  if (!strcmp(tgt,"humidity")||!strcmp(tgt,"humid")) {
    fanHumOn=(uint8_t)constrain(v,30,100);
    snprintf_P(tmp,sizeof(tmp),PSTR("OK: HUM=%d%%"),fanHumOn); pL(tmp); return;
  }
  if (!strcmp(tgt,"duration")) {
    pumpSec=(uint16_t)constrain(v,1,60);
    snprintf_P(tmp,sizeof(tmp),PSTR("OK: DUR=%ds"),pumpSec); pL(tmp); return;
  }
  if (!strcmp(tgt,"cooldown")) {
    cooldownSec=(uint16_t)constrain(v,60,7200);
    snprintf_P(tmp,sizeof(tmp),PSTR("OK: COOL=%ds"),cooldownSec); pL(tmp); return;
  }
  pF(F("Valid: zone1/2 gas temp humidity duration cooldown"));
}

// ============================================================
//  DEBUG COMMANDS
// ============================================================
void doDebug(const char *sub) {
  if (!sub||!sub[0]) {
    debugMode=!debugMode;
    snprintf_P(tmp,sizeof(tmp),PSTR("DEBUG=%s"),debugMode?"ON":"OFF"); pL(tmp); return;
  }
  pPGM(S_DBG);

  if (!strcmp(sub,"sensors")) {
    updateSensors(true);
    snprintf_P(tmp,sizeof(tmp),PSTR("S1=%d(%d%%) S2=%d(%d%%) S3=%d(%d%%)"),
      soilRaw[0],soilPct[0],soilRaw[1],soilPct[1],soilRaw[2],soilPct[2]); pL(tmp);
    snprintf_P(tmp,sizeof(tmp),PSTR("Z1avg=%d%% Z2=%d%% GAS=%d(thr=%d)"),
      zone1Avg,soilPct[2],gasRaw,gasThr); pL(tmp);
    if (GET_FLAG(FLAG_DHT_OK)) {
      // Use integer arithmetic — avoids dtostrf stack buffers
      snprintf_P(tmp,sizeof(tmp),PSTR("T=%d.%dC H=%d%%"),
        (int)tempC, abs((int)(tempC*10)%10), (int)humPct);
    } else {
      snprintf_P(tmp,sizeof(tmp),PSTR("DHT: FAILED"));
    }
    pL(tmp); return;
  }

  if (!strcmp(sub,"flags")) {
    char bin[9];
    for(int i=7;i>=0;i--) bin[7-i]=((flags>>i)&1)?'1':'0';
    bin[8]='\0';
    snprintf_P(tmp,sizeof(tmp),PSTR("flags=0b%s"),bin); pL(tmp);
    snprintf_P(tmp,sizeof(tmp),PSTR("AU=%d RT=%d LC=%d DH=%d P1=%d P2=%d DA=%d GA=%d"),
      GET_FLAG(0),GET_FLAG(1),GET_FLAG(2),GET_FLAG(3),
      GET_FLAG(4),GET_FLAG(5),GET_FLAG(6),GET_FLAG(7)); pL(tmp);
    snprintf_P(tmp,sizeof(tmp),PSTR("dbg=%d"),debugMode?1:0); pL(tmp);
    return;
  }

  if (!strcmp(sub,"mem")) {
    int fr=freeRam();
    snprintf_P(tmp,sizeof(tmp),PSTR("Free SRAM: %d B"),fr); pL(tmp);
    if(fr<400) pF(F("WARN: Low SRAM!"));
    return;
  }

  if (!strcmp(sub,"relays")) {
    snprintf_P(tmp,sizeof(tmp),PSTR("FAN D%d=%d P1 D%d=%d P2 D%d=%d"),
      FAN_RELAY,  relayOn(FAN_RELAY)  ?1:0,
      PUMP1_RELAY,relayOn(PUMP1_RELAY)?1:0,
      PUMP2_RELAY,relayOn(PUMP2_RELAY)?1:0); pL(tmp);
    return;
  }

  if (!strcmp(sub,"thresholds")) {
    snprintf_P(tmp,sizeof(tmp),PSTR("Z1=%d%% Z2=%d%% GAS=%d"),zThr1,zThr2,gasThr); pL(tmp);
    snprintf_P(tmp,sizeof(tmp),PSTR("T_ON=%dC T_OFF=%dC HUM=%d%%"),fanTempOn,fanTempOn-3,fanHumOn); pL(tmp);
    snprintf_P(tmp,sizeof(tmp),PSTR("DUR=%ds COOL=%ds"),pumpSec,cooldownSec); pL(tmp);
    return;
  }

  if (!strcmp(sub,"timing")) {
    uint32_t now=millis();
    snprintf_P(tmp,sizeof(tmp),PSTR("uptime=%lus"),now/1000); pL(tmp);
    snprintf_P(tmp,sizeof(tmp),PSTR("sens=%lus ago lcd=%lus ago"),
      (now-lastSensorMs)/1000,(now-lastLcdMs)/1000); pL(tmp);
    if (GET_FLAG(FLAG_PUMP1_TIMED)) {
      snprintf_P(tmp,sizeof(tmp),PSTR("P1 %lus/%us rem"),
        (now-p1StartMs)/1000,pumpSec); pL(tmp);
    }
    if (GET_FLAG(FLAG_PUMP2_TIMED)) {
      snprintf_P(tmp,sizeof(tmp),PSTR("P2 %lus/%us rem"),
        (now-p2StartMs)/1000,pumpSec); pL(tmp);
    }
    snprintf_P(tmp,sizeof(tmp),PSTR("p1auto=%lus p2auto=%lus ago"),
      p1AutoMs?(now-p1AutoMs)/1000:0,
      p2AutoMs?(now-p2AutoMs)/1000:0); pL(tmp);
    return;
  }

  if (!strcmp(sub,"reset")) {
    p1AutoMs=p2AutoMs=lastSensorMs=0;
    pF(F("OK: Timers reset")); return;
  }

  pF(F("d: sensors flags mem relays thresholds timing reset"));
}

// ============================================================
//  SENSOR UPDATE
// ============================================================
void updateSensors(bool force) {
  uint32_t now=millis();
  if (!force&&(now-lastSensorMs)<SENSOR_MS) return;
  lastSensorMs=now;

  soilRaw[0]=analogRead(SOIL1_PIN);
  soilRaw[1]=analogRead(SOIL2_PIN);
  soilRaw[2]=analogRead(SOIL3_PIN);
  gasRaw    =analogRead(GAS_PIN);

  soilPct[0]=mapPct(soilRaw[0],SOIL_DRY,SOIL_WET);
  soilPct[1]=mapPct(soilRaw[1],SOIL_DRY,SOIL_WET);
  soilPct[2]=mapPct(soilRaw[2],SOIL_DRY,SOIL_WET);
  zone1Avg  =(uint8_t)(((uint16_t)soilPct[0]+soilPct[1])/2);

  float h=dht.readHumidity(), t=dht.readTemperature();
  if (!isnan(h)&&!isnan(t)) { humPct=h; tempC=t; SET_FLAG(FLAG_DHT_OK,1); }
  else                        SET_FLAG(FLAG_DHT_OK,0);

  bool prevGas=GET_FLAG(FLAG_GAS_ALERT);
  alertChanged(!GET_FLAG(FLAG_DHT_OK), FLAG_DHT_ALERT, F("ALERT: DHT fail"), F("INFO: DHT OK"));
  alertChanged(gasRaw>=gasThr,         FLAG_GAS_ALERT,  F("ALERT: Gas HIGH!"),F("INFO: Gas OK"));
  if (!prevGas&&GET_FLAG(FLAG_GAS_ALERT)) trigDyn(DPAGE_ALERT);

  if (debugMode) {
    snprintf_P(tmp,sizeof(tmp),PSTR("[D] S=%d/%d/%d G=%d"),
      soilPct[0],soilPct[1],soilPct[2],gasRaw);
    pL(tmp);
  }
}

// ============================================================
//  AUTOMATION
// ============================================================
void runAuto() {
  if (!GET_FLAG(FLAG_AUTO_MODE)) return;
  uint32_t now=millis();
  uint32_t cool=(uint32_t)cooldownSec*1000UL;
  uint32_t pdur=(uint32_t)pumpSec   *1000UL;

  bool z1rdy=(!p1AutoMs)||((now-p1AutoMs)>=cool);
  bool z2rdy=(!p2AutoMs)||((now-p2AutoMs)>=cool);

  if (!relayOn(PUMP1_RELAY)&&!GET_FLAG(FLAG_PUMP1_TIMED)&&z1rdy&&zone1Avg<zThr1)
    startP1(pdur,F("Auto Z1"),true);
  if (!relayOn(PUMP2_RELAY)&&!GET_FLAG(FLAG_PUMP2_TIMED)&&z2rdy&&soilPct[2]<zThr2)
    startP2(pdur,F("Auto Z2"),true);

  if (GET_FLAG(FLAG_DHT_OK)) {
    bool fan=(tempC>=fanTempOn)||(humPct>=fanHumOn)||(gasRaw>=gasThr);
    if (!relayOn(FAN_RELAY)&&fan) {
      setRelay(FAN_RELAY,ON,NULL,false); pF(F("INFO: Fan ON (auto)"));
    } else if (relayOn(FAN_RELAY)&&!fan&&tempC<=(fanTempOn-3)&&humPct<fanHumOn&&gasRaw<gasThr) {
      setRelay(FAN_RELAY,OFF,NULL,false); pF(F("INFO: Fan OFF (auto)"));
    }
  } else if (gasRaw>=gasThr&&!relayOn(FAN_RELAY)) {
    setRelay(FAN_RELAY,ON,NULL,false); pF(F("INFO: Fan ON (gas)"));
  }
}

// ============================================================
//  TIMED PUMP TRACKER
// ============================================================
void updateTimers() {
  uint32_t now=millis();
  if (GET_FLAG(FLAG_PUMP1_TIMED)&&(now-p1StartMs)>=p1DurMs) stopP1(F("Timer"));
  if (GET_FLAG(FLAG_PUMP2_TIMED)&&(now-p2StartMs)>=p2DurMs) stopP2(F("Timer"));
}

// ============================================================
//  LCD
// ============================================================
void updateLcd() {
  if (!GET_FLAG(FLAG_LCD_ENABLED)) return;
  uint32_t now=millis();

  // Expire dynamic page
  if (dynPage!=DPAGE_NONE) {
    uint32_t hold=(dynPage==DPAGE_ALERT)?DYN_HOLD:PUMP_HOLD;
    if ((now-dynStartMs)>=hold) { dynPage=DPAGE_NONE; lastLcdMs=0; }
  }

  if ((now-lastLcdMs)<LCD_MS) return;
  lastLcdMs=now;
  lcd.clear();
  showPage(dynPage!=DPAGE_NONE ? dynPage : lcdPage);
  if (dynPage==DPAGE_NONE) lcdPage=(lcdPage+1)%LCD_PAGES;
}

void trigDyn(uint8_t pg) { dynPage=pg; dynStartMs=millis(); lastLcdMs=0; }

void showPage(uint8_t pg) {
  switch(pg) {

  // ── Page 0: MAIN (all critical info) ──────────────────────
  case 0:
    lcd.setCursor(0,0);
    lcd.print(GET_FLAG(FLAG_AUTO_MODE)?F("A"):F("M"));
    lcd.print(F(" P1:"));
    lcd.print(relayOn(PUMP1_RELAY)?F("O"):F("-"));
    lcd.print(F(" P2:"));
    lcd.print(relayOn(PUMP2_RELAY)?F("O"):F("-"));
    lcd.print(F(" F:"));
    lcd.print(relayOn(FAN_RELAY)?F("O"):F("-"));
    lcd.setCursor(0,1);
    if (GET_FLAG(FLAG_DHT_OK)) {
      lcd.print(F("T:"));
      lcd.print(tempC,1);
      lcd.print(F(" H:"));
      lcd.print((int)humPct);
      lcd.print(F("%"));
    } else {
      lcd.print(F("DHT ERR G:"));
      lcd.print(gasRaw);
    }
    break;

  // ── Page 1: Zone 1 soil ────────────────────────────────────
  case 1:
    lcd.setCursor(0,0);
    lcd.print(F("S1:")); lcd.print(soilPct[0]);
    lcd.print(F("% S2:")); lcd.print(soilPct[1]); lcd.print(F("%"));
    lcd.setCursor(0,1);
    lcd.print(F("Z1Avg:")); lcd.print(zone1Avg);
    lcd.print(F("% P1:")); lcd.print(relayOn(PUMP1_RELAY)?F("ON"):F("--"));
    break;

  // ── Page 2: Zone 2 soil ────────────────────────────────────
  case 2:
    lcd.setCursor(0,0);
    lcd.print(F("S3:")); lcd.print(soilPct[2]);
    lcd.print(F("% G:")); lcd.print(gasRaw);
    lcd.setCursor(0,1);
    lcd.print(F("Z2:")); lcd.print(soilPct[2]);
    lcd.print(F("% P2:")); lcd.print(relayOn(PUMP2_RELAY)?F("ON"):F("--"));
    break;

  // ── Page 3: Temperature / Humidity ────────────────────────
  case 3:
    lcd.setCursor(0,0);
    if (GET_FLAG(FLAG_DHT_OK)) {
      lcd.print(F("Tmp:")); lcd.print(tempC,1); lcd.print(F("C"));
    } else { lcd.print(F("Tmp:ERR")); }
    lcd.print(F(" F:")); lcd.print(relayOn(FAN_RELAY)?F("O"):F("-"));
    lcd.setCursor(0,1);
    if (GET_FLAG(FLAG_DHT_OK)) {
      lcd.print(F("Hum:")); lcd.print((int)humPct);
      lcd.print(F("% T>")); lcd.print(fanTempOn); lcd.print(F("C"));
    } else { lcd.print(F("Check DHT wiring")); }
    break;

  // ── Page 4: Gas ───────────────────────────────────────────
  case 4:
    lcd.setCursor(0,0);
    lcd.print(F("GAS:"));  lcd.print(gasRaw);
    lcd.print(GET_FLAG(FLAG_GAS_ALERT)?F(" [ALERT]"):F(" [OK]  "));
    lcd.setCursor(0,1);
    lcd.print(F("Thr:")); lcd.print(gasThr);
    lcd.print(F(" Fan:")); lcd.print(relayOn(FAN_RELAY)?F("ON"):F("--"));
    break;

  // ── Page 5: RTC Clock ────────────────────────────────────
  case 5:
    lcd.setCursor(0,0);
    lcd.print(F("RTC:")); lcd.print(GET_FLAG(FLAG_RTC_OK)?F("OK  "):F("NONE"));
    lcd.setCursor(0,1);
    if (GET_FLAG(FLAG_RTC_OK)) {
      DateTime n=rtc.now();
      snprintf_P(tmp,sizeof(tmp),PSTR("%02d/%02d %02d:%02d:%02d"),
        n.month(),n.day(),n.hour(),n.minute(),n.second());
      lcd.print(tmp);
    } else { lcd.print(F("--/-- --:--:--")); }
    break;

  // ── Dynamic: Gas / DHT Alert ─────────────────────────────
  case DPAGE_ALERT:
    lcd.setCursor(0,0);
    lcd.print(GET_FLAG(FLAG_GAS_ALERT)?F("!! GAS  ALERT !!"):F("!! DHT  FAULT !"));
    lcd.setCursor(0,1);
    lcd.print(F("GAS:")); lcd.print(gasRaw);
    lcd.print(F(" Thr:")); lcd.print(gasThr);
    break;

  // ── Dynamic: Pump event ──────────────────────────────────
  case DPAGE_PUMP:
    lcd.setCursor(0,0);
    lcd.print(F(">> PUMP EVENT   "));
    lcd.setCursor(0,1);
    lcd.print(F("P1:")); lcd.print(relayOn(PUMP1_RELAY)?F("RUN "):F("STP "));
    lcd.print(F("P2:")); lcd.print(relayOn(PUMP2_RELAY)?F("RUN"):F("STP"));
    break;

  default: showPage(0); break;
  }
}

// ============================================================
//  STATUS / SENSORS / TIME / HELP
// ============================================================
void printStatus() {
  updateSensors(true);
  pPGM(S_STATUS);
  pF(GET_FLAG(FLAG_AUTO_MODE)?F("MODE: AUTO"):F("MODE: MANUAL"));
  snprintf_P(tmp,sizeof(tmp),PSTR("PUMP1:%s%s"),
    relayOn(PUMP1_RELAY)?"ON":"OFF", GET_FLAG(FLAG_PUMP1_TIMED)?" (tmr)":""); pL(tmp);
  snprintf_P(tmp,sizeof(tmp),PSTR("PUMP2:%s%s"),
    relayOn(PUMP2_RELAY)?"ON":"OFF", GET_FLAG(FLAG_PUMP2_TIMED)?" (tmr)":""); pL(tmp);
  snprintf_P(tmp,sizeof(tmp),PSTR("FAN:  %s"),relayOn(FAN_RELAY)?"ON":"OFF"); pL(tmp);
  snprintf_P(tmp,sizeof(tmp),PSTR("Z1: %d%% (S1=%d S2=%d)"),zone1Avg,soilPct[0],soilPct[1]); pL(tmp);
  snprintf_P(tmp,sizeof(tmp),PSTR("Z2: %d%%"),soilPct[2]); pL(tmp);
  snprintf_P(tmp,sizeof(tmp),PSTR("GAS: %d %s"),gasRaw,GET_FLAG(FLAG_GAS_ALERT)?"[!]":"[OK]"); pL(tmp);
  if (GET_FLAG(FLAG_DHT_OK)) {
    snprintf_P(tmp,sizeof(tmp),PSTR("T: %d.%dC H: %d%%"),
      (int)tempC,abs((int)(tempC*10)%10),(int)humPct); pL(tmp);
  } else { pF(F("T/H: DHT ERROR")); }
  snprintf_P(tmp,sizeof(tmp),PSTR("DBG:%s SRAM:%dB"),
    debugMode?"ON":"OFF",freeRam()); pL(tmp);
  printTime();
  pPGM(S_SEP);
}

void printSensors() {
  updateSensors(true);
  pPGM(S_SENSORS);
  for(uint8_t i=0;i<3;i++){
    snprintf_P(tmp,sizeof(tmp),PSTR("SOIL%d: %d => %d%%"),i+1,soilRaw[i],soilPct[i]); pL(tmp);
  }
  snprintf_P(tmp,sizeof(tmp),PSTR("Z1avg: %d%%"),zone1Avg); pL(tmp);
  snprintf_P(tmp,sizeof(tmp),PSTR("GAS: %d  thr=%d  %s"),
    gasRaw,gasThr,GET_FLAG(FLAG_GAS_ALERT)?"[ALERT]":"[OK]"); pL(tmp);
  if (GET_FLAG(FLAG_DHT_OK)) {
    snprintf_P(tmp,sizeof(tmp),PSTR("T:%d.%dC H:%d%%"),
      (int)tempC,abs((int)(tempC*10)%10),(int)humPct);
  } else {
    snprintf_P(tmp,sizeof(tmp),PSTR("DHT: FAILED"));
  }
  pL(tmp);
  pPGM(S_SEP);
}

void printTime() {
  if (!GET_FLAG(FLAG_RTC_OK)) { pF(F("TIME: RTC NOT READY")); return; }
  DateTime n=rtc.now();
  snprintf_P(tmp,sizeof(tmp),PSTR("TIME: %02d/%02d/%04d %02d:%02d:%02d"),
    n.month(),n.day(),n.year(),n.hour(),n.minute(),n.second());
  pL(tmp);
}

void printHelp() {
  pPGM(S_CMDS);
  pF(F("status  sensors  time  help"));
  pF(F("mode auto/manual"));
  pF(F("pump1/2 on/off | fan on/off"));
  pF(F("zone1now  zone2now  allon  alloff"));
  pF(F("lcd on/off | lcd page 0-5"));
  pF(F("set zone1/2 gas temp humidity"));
  pF(F("set duration(s) cooldown(s)"));
  pF(F("d  d sensors/flags/mem"));
  pF(F("d relays/thresholds/timing"));
  pF(F("d reset"));
  pPGM(S_SEP);
}

// ============================================================
//  UTILITIES
// ============================================================
void alertChanged(bool cond, uint8_t bit,
                  const __FlashStringHelper *on,
                  const __FlashStringHelper *off) {
  if (cond!=(bool)GET_FLAG(bit)) { SET_FLAG(bit,cond); pF(cond?on:off); }
}

void setRelay(uint8_t pin, bool state, const __FlashStringHelper *name, bool ann) {
  digitalWrite(pin,state);
  if (ann&&name) {
    // Write name from flash char-by-char to avoid a RAM buffer
    Serial.print(F("OK: ")); BT.print(F("OK: "));
    Serial.print(name);      BT.print(name);
    Serial.print(state==ON?F("=ON"):F("=OFF"));
    BT.print(state==ON?F("=ON"):F("=OFF"));
    Serial.println(); BT.println();
  }
}

bool relayOn(uint8_t pin) { return digitalRead(pin)==ON; }

void startP1(uint32_t ms, const __FlashStringHelper *why, bool mark) {
  SET_FLAG(FLAG_PUMP1_TIMED,1);
  p1StartMs=millis(); p1DurMs=ms;
  if (mark) p1AutoMs=p1StartMs;
  setRelay(PUMP1_RELAY,ON,F("PUMP1"),true);
  Serial.print(F("P1 start: ")); Serial.println(why);
  BT.print(F("P1 start: "));     BT.println(why);
  trigDyn(DPAGE_PUMP);
}

void startP2(uint32_t ms, const __FlashStringHelper *why, bool mark) {
  SET_FLAG(FLAG_PUMP2_TIMED,1);
  p2StartMs=millis(); p2DurMs=ms;
  if (mark) p2AutoMs=p2StartMs;
  setRelay(PUMP2_RELAY,ON,F("PUMP2"),true);
  Serial.print(F("P2 start: ")); Serial.println(why);
  BT.print(F("P2 start: "));     BT.println(why);
  trigDyn(DPAGE_PUMP);
}

void stopP1(const __FlashStringHelper *why) {
  bool was=GET_FLAG(FLAG_PUMP1_TIMED)||relayOn(PUMP1_RELAY);
  SET_FLAG(FLAG_PUMP1_TIMED,0); p1DurMs=0;
  setRelay(PUMP1_RELAY,OFF,F("PUMP1"),true);
  if (was) {
    Serial.print(F("P1 stop: ")); Serial.println(why);
    BT.print(F("P1 stop: "));     BT.println(why);
    trigDyn(DPAGE_PUMP);
  }
}

void stopP2(const __FlashStringHelper *why) {
  bool was=GET_FLAG(FLAG_PUMP2_TIMED)||relayOn(PUMP2_RELAY);
  SET_FLAG(FLAG_PUMP2_TIMED,0); p2DurMs=0;
  setRelay(PUMP2_RELAY,OFF,F("PUMP2"),true);
  if (was) {
    Serial.print(F("P2 stop: ")); Serial.println(why);
    BT.print(F("P2 stop: "));     BT.println(why);
    trigDyn(DPAGE_PUMP);
  }
}

uint8_t mapPct(int raw, int lo, int hi) {
  if (lo==hi) return 0;
  return (uint8_t)constrain(map(raw,lo,hi,0,100),0,100);
}

// PROGMEM-safe strcmp (fixed: checks both chars at \0)
bool eqP(const char *ram, const char *pgm) {
  for(;;) {
    char a=*ram++, b=(char)pgm_read_byte(pgm++);
    if (a!=b) return false;
    if (a=='\0') return true;
  }
}

// Print PROGMEM string to both ports
void pPGM(const char *p) {
  char c; while((c=(char)pgm_read_byte(p++))){Serial.print(c);BT.print(c);}
  Serial.println(); BT.println();
}

// Print F() string to both ports
void pF(const __FlashStringHelper *s) { Serial.println(s); BT.println(s); }

// Print RAM string to both ports
void pL(const char *s) { Serial.println(s); BT.println(s); }

/* SMART AQUAPONICS v2.4.1 - FIXED & COMPLETE
 * Logic Overhaul + Hysteresis + Event Log + Full BT Automation
 * Active-low relays (LOW = ON/red) | 20s safety grace
 */

#include <Wire.h>
#include <LiquidCrystal_I2C.h>
#include <OneWire.h>
#include <DallasTemperature.h>
#include <EEPROM.h>
#include <Servo.h>

// ==================== PINS ====================
#define R_SUPPLY  8
#define R_FILTER  7
#define R_PLANT   6
#define TRIG      5
#define ECHO      3
#define TEMP      4
#define PH        A0
#define TURB      A1
#define SOIL      A2
#define SERVO     9

// ==================== CONFIG ====================
int  wTarget = 5;
float phMin = 6.5, phMax = 8.5;
int  turbTh = 500, soilTh = 800;
int  feedMin = 1, xferSec = 10, waterSec = 15;

int waterHyst = 2;
float phHyst = 0.2;
int turbHyst = 50;

bool liveMode = false;          // Bluetooth live streaming

// Objects
LiquidCrystal_I2C lcd(0x27, 20, 4);
OneWire oneWire(TEMP);
DallasTemperature ts(&oneWire);
Servo sv;

// State
bool mP1 = 0, mP2 = 0, mP3 = 0, manual = 0;
enum {WS_IDLE, WS_XFER, WS_WATER} ws = WS_IDLE;
enum {FD_IDLE, FD_OPEN, FD_HOLD, FD_CLOSE} fs = FD_IDLE;

unsigned long wsT = 0, lastW = 0, fdT = 0, lastF = 0, lastSen = 0, lastLCD = 0, graceStart = 0, lastLive = 0;
bool startupGrace = true;
bool fdTrig = 0;

// Sensors
float tC = 0; int dCm = 0, tR = 0, sM = 0; float pH = 0;
bool sensorOK = true;

// Soil debounce
int dryCount = 0;
const int DRY_DEBOUNCE = 3;

// Event logger
#define LOG_SIZE 8
char eventLog[LOG_SIZE][32];
uint8_t logIdx = 0;

void addEvent(const __FlashStringHelper* msg) {
  strncpy_P(eventLog[logIdx], (const char*)msg, 31);
  eventLog[logIdx][31] = '\0';
  logIdx = (logIdx + 1) % LOG_SIZE;
}

// Safe BT buffer
char cmdBuffer[64] = "";
uint8_t bufIdx = 0;

void clearBuffer() {
  bufIdx = 0;
  cmdBuffer[0] = '\0';
}

// ==================== FUNCTION DECLARATIONS (forward) ====================
void loadE();
void saveE();
void defE();
void hSet(String p);
void hP(int n, String s);
void setM3(bool on);
void sendLiveStatus();
void sendJsonStatus();
void printEventLog();
void updLCD();

// ==================== SETUP ====================
void setup() {
  Serial.begin(9600);

  // FORCE RELAYS OFF (GREEN)
  pinMode(R_SUPPLY, OUTPUT); digitalWrite(R_SUPPLY, HIGH);
  pinMode(R_FILTER, OUTPUT); digitalWrite(R_FILTER, HIGH);
  pinMode(R_PLANT,  OUTPUT); digitalWrite(R_PLANT,  HIGH);

  pinMode(TRIG, OUTPUT);
  pinMode(ECHO, INPUT);

  ts.begin();
  sv.attach(SERVO); sv.write(0); sv.detach();

  loadE();

  lcd.init(); lcd.backlight();
  lcd.print(F("SMART AQUAPONICS")); 
  lcd.setCursor(0,1); lcd.print(F("v2.4.1 FIXED")); 
  delay(1500);

  graceStart = millis();
  startupGrace = true;
  lastLive = millis();
  lastF = millis();

  addEvent(F("System boot"));
  Serial.println(F("=== AQUAPONICS v2.4.1 READY ==="));
}

// ==================== LOOP ====================
void loop() {
  unsigned long n = millis();
  handleBT();

  if (startupGrace) {
    if (n - graceStart >= 20000UL) {
      startupGrace = false;
      addEvent(F("Grace ended - AUTO active"));
      Serial.println(F(">>> SAFETY GRACE ENDED"));
      lcd.clear();
    } else {
      if (n - lastLCD >= 500) {
        lastLCD = n;
        int secs = (20000UL - (n - graceStart)) / 1000;
        lcd.clear();
        lcd.setCursor(0,0); lcd.print(F("SAFETY GRACE"));
        lcd.setCursor(0,1); lcd.print(F("All pumps OFF "));
        lcd.print(secs); lcd.print(F("s"));
      }
      return;
    }
  }

  if (n - lastSen >= 1000) {
    lastSen = n;
    readS();
    autoLogic();
  }

  updWS();
  updFD();

  if (n - lastLCD >= 500) {
    lastLCD = n;
    updLCD();
  }

  if (liveMode && n - lastLive >= 2000) {
    lastLive = n;
    sendLiveStatus();
  }
}

// ==================== SENSORS ====================
void readS() {
  digitalWrite(TRIG, LOW); delayMicroseconds(2);
  digitalWrite(TRIG, HIGH); delayMicroseconds(10);
  digitalWrite(TRIG, LOW);
  long dur = pulseIn(ECHO, HIGH, 30000UL);
  dCm = (dur > 0) ? (int)(dur * 0.034 / 2.0) : 999;

  ts.requestTemperatures();
  tC = ts.getTempCByIndex(0);
  if (tC == DEVICE_DISCONNECTED_C) tC = -127.0;

  pH = analogRead(PH) * 0.004887586 * 3.5;
  tR = analogRead(TURB);
  sM = analogRead(SOIL);

  sensorOK = (tC > -100 && dCm < 999 && pH > 0 && pH < 14);
  if (!sensorOK) addEvent(F("Sensor error!"));
}

// ==================== AUTO LOGIC ====================
bool lastSupplyOn = false;

void autoLogic() {
  if (manual || startupGrace || !sensorOK) {
    if (!sensorOK) {
      digitalWrite(R_SUPPLY, HIGH);
      digitalWrite(R_FILTER, HIGH);
      digitalWrite(R_PLANT,  HIGH);
    }
    return;
  }

  // Supply pump with hysteresis
  if (!mP1) {
    int trigger = wTarget + (lastSupplyOn ? 0 : waterHyst);
    bool shouldOn = (dCm > trigger);
    if (shouldOn != lastSupplyOn) {
      digitalWrite(R_SUPPLY, shouldOn ? LOW : HIGH);
      lastSupplyOn = shouldOn;
      if (shouldOn) addEvent(F("Supply pump ON"));
    }
  }

  // Filter pump
  if (!mP2) {
    bool badWater = (pH < (phMin - phHyst) || pH > (phMax + phHyst) || tR > (turbTh + turbHyst));
    digitalWrite(R_FILTER, (ws == WS_XFER || (ws == WS_IDLE && badWater)) ? LOW : HIGH);
  }

  // Plant pump
  if (!mP3) digitalWrite(R_PLANT, (ws == WS_WATER) ? LOW : HIGH);
}

// ==================== WATERING STATE MACHINE ====================
void updWS() {
  if (manual || startupGrace) return;
  unsigned long n = millis();

  switch (ws) {
    case WS_IDLE:
      if (sM > soilTh) {
        if (++dryCount >= DRY_DEBOUNCE) {
          ws = WS_XFER; wsT = n;
          addEvent(F("Xfer started (soil dry)"));
          Serial.println(F("[AUTO] Xfer start"));
        }
      } else dryCount = 0;
      break;

    case WS_XFER:
      if (n - wsT >= (unsigned long)xferSec * 1000) {
        ws = WS_WATER; wsT = n;
        addEvent(F("Watering plants"));
        Serial.println(F("[AUTO] Watering"));
      }
      break;

    case WS_WATER:
      if (n - wsT >= (unsigned long)waterSec * 1000) {
        ws = WS_IDLE; lastW = n; dryCount = 0;
        addEvent(F("Watering complete"));
        Serial.println(F("[AUTO] Done"));
      }
      break;
  }
}

// ==================== FEEDER ====================
void updFD() {
  unsigned long n = millis();
  switch (fs) {
    case FD_IDLE:
      if (fdTrig || n - lastF >= (unsigned long)feedMin * 60000UL) {
        fdTrig = 0; fs = FD_OPEN; fdT = n;
        sv.attach(SERVO); sv.write(0);
        addEvent(F("Feeder activated"));
        Serial.println(F("[FEED] Go"));
      }
      break;
    case FD_OPEN:  if (n - fdT > 600)  { sv.write(180); fs = FD_HOLD; fdT = n; } break;
    case FD_HOLD:  if (n - fdT > 1200) { sv.write(0);   fs = FD_CLOSE; fdT = n; } break;
    case FD_CLOSE: if (n - fdT > 600)  { sv.detach(); fs = FD_IDLE; lastF = n; Serial.println(F("[FEED] Fin")); } break;
  }
}

// ==================== BLUETOOTH ====================
void handleBT() {
  while (Serial.available()) {
    char c = Serial.read();
    if (c == 'D' || c == 'd') { setM3(1); clearBuffer(); return; }
    if (c == 'S' || c == 's') { setM3(0); clearBuffer(); return; }

    if (c == '\n' || c == '\r') {
      if (bufIdx > 0) {
        cmdBuffer[bufIdx] = '\0';
        procCmd(cmdBuffer);
      }
      clearBuffer();
    } 
    else if (bufIdx < sizeof(cmdBuffer)-1) {
      cmdBuffer[bufIdx++] = c;
    }
  }
}

void procCmd(char* input) {
  String cmd = input;
  cmd.toLowerCase(); cmd.trim();
  Serial.print(F("> ")); Serial.println(cmd);

  if (cmd == "status" || cmd == "st") sendJsonStatus();
  else if (cmd == "live") { liveMode = !liveMode; Serial.print(F("Live streaming: ")); Serial.println(liveMode ? F("ON") : F("OFF")); }
  else if (cmd == "feed") { fdTrig = 1; Serial.println(F(">>> Feed triggered")); }
  else if (cmd == "auto") { manual = mP1 = mP2 = mP3 = 0; Serial.println(F(">>> AUTO mode")); }
  else if (cmd == "reset") { defE(); Serial.println(F(">>> Reset to defaults")); }
  else if (cmd == "save") { saveE(); Serial.println(F(">>> Saved to EEPROM")); }
  else if (cmd == "log") printEventLog();
  else if (cmd.startsWith("set ")) hSet(cmd.substring(4));
  else if (cmd.startsWith("p1 ")) hP(1, cmd.substring(3));
  else if (cmd.startsWith("p2 ")) hP(2, cmd.substring(3));
  else if (cmd.startsWith("p3 ")) hP(3, cmd.substring(3));
  else Serial.println(F("Commands: status, live, feed, auto, reset, save, log, set ..., p1/p2/p3 on/off"));
}

// ==================== THRESHOLD SETTING ====================
void hSet(String p) {
  p.trim();
  int sp = p.indexOf(' ');
  String k = (sp == -1) ? p : p.substring(0, sp);
  String v = (sp == -1) ? "" : p.substring(sp + 1);
  v.trim();

  float f = v.toFloat();
  int i = v.toInt();
  bool ok = true;

  if (k == "water") wTarget = i;
  else if (k == "phmin") { if (f>=0 && f<=14) phMin = f; else ok=false; }
  else if (k == "phmax") { if (f>=0 && f<=14) phMax = f; else ok=false; }
  else if (k == "turb") turbTh = i;
  else if (k == "soil") soilTh = i;
  else if (k == "feedint") { if(i>=1&&i<=60) feedMin=i; else ok=false; }
  else if (k == "transfer") { if(i>=1) xferSec=i; else ok=false; }
  else if (k == "waterdur") { if(i>=1) waterSec=i; else ok=false; }
  else if (k == "hystwater") { if(i>=0&&i<=10) waterHyst=i; else ok=false; }
  else if (k == "hystph") { if(f>=0&&f<=1) phHyst=f; else ok=false; }
  else if (k == "hystturb") { if(i>=0&&i<=100) turbHyst=i; else ok=false; }
  else { ok = false; }

  if (ok) {
    Serial.print(k); Serial.print(F(" = ")); Serial.println(v);
    saveE();
    addEvent(F("Config changed"));
  } else {
    Serial.println(F("ERR: Invalid value or unknown key"));
  }
}

void hP(int n, String s) {
  s.trim(); manual = 1;
  bool on = (s == "on" || s == "1" || s == "high");

  if (n==1) { mP1=1; digitalWrite(R_SUPPLY, on?LOW:HIGH); }
  if (n==2) { mP2=1; digitalWrite(R_FILTER, on?LOW:HIGH); if(ws!=WS_IDLE) ws=WS_IDLE; }
  if (n==3) { mP3=1; digitalWrite(R_PLANT,  on?LOW:HIGH); if(ws!=WS_IDLE) ws=WS_IDLE; }

  Serial.print(F(">>> P")); Serial.print(n); Serial.println(on ? F(" ON") : F(" OFF"));
}

void setM3(bool on) {
  manual = mP3 = 1;
  digitalWrite(R_PLANT, on ? LOW : HIGH);
  if (ws != WS_IDLE) ws = WS_IDLE;
  Serial.print(F(">>> P3 ")); Serial.println(on ? F("ON") : F("OFF"));
}

// ==================== EEPROM ====================
void loadE() {
  if (EEPROM.read(21) == 0xAB) {
    EEPROM.get(0, wTarget); EEPROM.get(2, phMin); EEPROM.get(6, phMax);
    EEPROM.get(10, turbTh); EEPROM.get(12, soilTh); EEPROM.get(14, feedMin);
    EEPROM.get(16, xferSec); EEPROM.get(18, waterSec);
    byte cs = 0; for (int i = 0; i < 20; i++) cs ^= EEPROM.read(i);
    if (cs == EEPROM.read(20)) { Serial.println(F("[E] Loaded")); return; }
  }
  Serial.println(F("[E] Using defaults")); defE();
}

void saveE() {
  EEPROM.put(0, wTarget); EEPROM.put(2, phMin); EEPROM.put(6, phMax);
  EEPROM.put(10, turbTh); EEPROM.put(12, soilTh); EEPROM.put(14, feedMin);
  EEPROM.put(16, xferSec); EEPROM.put(18, waterSec);
  byte cs = 0; for (int i = 0; i < 20; i++) cs ^= EEPROM.read(i);
  EEPROM.write(20, cs); EEPROM.write(21, 0xAB);
}

void defE() {
  wTarget = 5; phMin = 6.5; phMax = 8.5; turbTh = 500; soilTh = 800;
  feedMin = 1; xferSec = 10; waterSec = 15;
  waterHyst = 2; phHyst = 0.2; turbHyst = 50;
  saveE();
}

// ==================== STATUS & LCD ====================
void sendLiveStatus() {
  Serial.print(F("LIVE|"));
  Serial.print(F("T:")); Serial.print(tC,1);
  Serial.print(F("|pH:")); Serial.print(pH,1);
  Serial.print(F("|D:")); Serial.print(dCm);
  Serial.print(F("|Tur:")); Serial.print(tR);
  Serial.print(F("|Soil:")); Serial.print(sM);
  Serial.print(F("|P1:")); Serial.print(digitalRead(R_SUPPLY) ? "OFF" : "ON");
  Serial.print(F("|P2:")); Serial.print(digitalRead(R_FILTER) ? "OFF" : "ON");
  Serial.print(F("|P3:")); Serial.print(digitalRead(R_PLANT) ? "OFF" : "ON");
  Serial.print(F("|WS:")); Serial.print(ws == WS_IDLE ? "IDLE" : ws == WS_XFER ? "XFER" : "WATER");
  Serial.print(F("|Feed:")); Serial.print(fs != FD_IDLE ? "ACT" : "IDLE");
  Serial.print(F("|Mode:")); Serial.println(manual ? "MAN" : "AUTO");
}

void sendJsonStatus() {
  Serial.println(F("{"));
  Serial.print(F("  \"temp\": ")); Serial.print(tC,1); Serial.println(F(","));
  Serial.print(F("  \"ph\": ")); Serial.print(pH,1); Serial.println(F(","));
  Serial.print(F("  \"distance\": ")); Serial.print(dCm); Serial.println(F(","));
  Serial.print(F("  \"turbidity\": ")); Serial.print(tR); Serial.println(F(","));
  Serial.print(F("  \"soil\": ")); Serial.print(sM); Serial.println(F(","));
  Serial.print(F("  \"p1\": \"")); Serial.print(digitalRead(R_SUPPLY) ? "OFF" : "ON"); Serial.println(F("\","));
  Serial.print(F("  \"p2\": \"")); Serial.print(digitalRead(R_FILTER) ? "OFF" : "ON"); Serial.println(F("\","));
  Serial.print(F("  \"p3\": \"")); Serial.print(digitalRead(R_PLANT) ? "OFF" : "ON"); Serial.println(F("\","));
  Serial.print(F("  \"watering\": \"")); Serial.print(ws == WS_IDLE ? "IDLE" : ws == WS_XFER ? "XFER" : "WATER"); Serial.println(F("\","));
  Serial.print(F("  \"feeder\": \"")); Serial.print(fs != FD_IDLE ? "ACTIVE" : "IDLE"); Serial.println(F("\","));
  Serial.print(F("  \"mode\": \"")); Serial.print(manual ? "MANUAL" : "AUTO"); Serial.println(F("\""));
  Serial.println(F("}"));
}

void printEventLog() {
  Serial.println(F("=== LAST 8 EVENTS ==="));
  for (int i = 0; i < LOG_SIZE; i++) {
    int idx = (logIdx + i) % LOG_SIZE;
    if (eventLog[idx][0]) Serial.println(eventLog[idx]);
  }
  Serial.println(F("=================="));
}

void updLCD() {
  lcd.clear();
  lcd.setCursor(0,0); lcd.print(F("T:")); lcd.print(tC,1); lcd.print(F(" pH:")); lcd.print(pH,1);
  lcd.setCursor(0,1); lcd.print(F("D:")); lcd.print(dCm); lcd.print(F(" S:")); lcd.print(sM);
  lcd.setCursor(0,2); 
  lcd.print(F("P1:")); lcd.print(digitalRead(R_SUPPLY)?F("OFF"):F("ON "));
  lcd.print(F("P2:")); lcd.print(digitalRead(R_FILTER)?F("OFF"):F("ON "));
  lcd.print(F("P3:")); lcd.print(digitalRead(R_PLANT)?F("OFF"):F("ON "));
  lcd.setCursor(0,3); lcd.print(manual ? F("MAN") : F("AUTO"));
  if (!sensorOK) lcd.print(F(" ERR"));
  if (liveMode) lcd.print(F(" LIVE"));
}
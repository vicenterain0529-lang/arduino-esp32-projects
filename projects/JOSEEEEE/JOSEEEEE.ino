#include <Wire.h>
#include <RTClib.h>
#include <Servo.h>
#include <LiquidCrystal_I2C.h>
#include <SoftwareSerial.h>
#include <EEPROM.h>
#include <avr/pgmspace.h>
#include <DFRobotDFPlayerMini.h>

// ═══════════════════════════════════════════════════════════
//  MEDICINE DISPENSER v4.0 — FIXED SERVO BEHAVIOR
// ═══════════════════════════════════════════════════════════

// ─── HARDWARE PINS ─────────────────────────────────────────
#define IR_PIN       2
#define SERVO_PIN    9
#define RED_LED      5
#define GREEN_LED    6
#define BUZZER_PIN   7

// DFPlayer Mini pins
#define DF_RX        4
#define DF_TX        3

// ─── CONSTANTS ─────────────────────────────────────────────
#define MAX_DOSES     6
#define SERVO_CLOSED  0
#define POPUP_MS      2000
#define CMD_MS        1000
#define SOFT_TIMEOUT  30000UL
#define HARD_TIMEOUT  60000UL
#define SNOOZE_MS     30000UL
#define DF_VOLUME     30

// EEPROM Map
#define ADDR_MAGIC    0
#define ADDR_DATA     4
#define MAGIC_NUM     0xCD

// ─── LCD CUSTOM CHARS ──────────────────────────────────────
const uint8_t CHAR_BELL[8]  PROGMEM = {0x04,0x0E,0x0E,0x0E,0x1F,0x00,0x04,0x00};
const uint8_t CHAR_CHECK[8] PROGMEM = {0x00,0x01,0x03,0x16,0x1C,0x08,0x00,0x00};
const uint8_t CHAR_CROSS[8] PROGMEM = {0x00,0x1B,0x0E,0x04,0x0E,0x1B,0x00,0x00};
const uint8_t CHAR_PILL[8]  PROGMEM = {0x00,0x0E,0x1F,0x1F,0x1F,0x0E,0x00,0x00};
const uint8_t CHAR_CLOCK[8] PROGMEM = {0x00,0x0E,0x15,0x17,0x11,0x0E,0x00,0x00};
const uint8_t CHAR_BAR1[8]  PROGMEM = {0x10,0x10,0x10,0x10,0x10,0x10,0x10,0x00};
const uint8_t CHAR_BAR5[8]  PROGMEM = {0x1F,0x1F,0x1F,0x1F,0x1F,0x1F,0x1F,0x00};

#define LCD_BELL   0
#define LCD_CHECK  1
#define LCD_CROSS  2
#define LCD_PILL   3
#define LCD_CLOCK  4
#define LCD_BAR1   5
#define LCD_BAR5   6

// ─── STATE MACHINE ─────────────────────────────────────────
enum State : uint8_t {
  ST_IDLE,
  ST_ALERT_SOFT,
  ST_ALERT_URGENT,
  ST_SNOOZED,
  ST_DISPENSED
};

// ─── DOSE STRUCTURE ────────────────────────────────────────
struct __attribute__((packed)) Dose {
  int8_t  hour;
  uint8_t minute;
  uint8_t taken;
  uint16_t timeVal;
};

// ─── GLOBALS ───────────────────────────────────────────────
RTC_DS3231         rtc;
Servo              servo;
LiquidCrystal_I2C  lcd(0x27, 16, 2);
SoftwareSerial     dfSerial(DF_RX, DF_TX);
DFRobotDFPlayerMini dfPlayer;

Dose    doses[MAX_DOSES];
State   state       = ST_IDLE;
uint8_t curDose     = 0xFF;
uint8_t lastDay     = 0;
uint8_t lcdPage     = 0;
bool    debugMode   = false;
bool    dfReady     = false;

bool    preAlerted[MAX_DOSES] = {false};

uint32_t alertStart  = 0;
uint32_t snoozeEnd   = 0;
uint32_t popupEnd    = 0;
uint32_t cmdEnd      = 0;
uint32_t lastToggle  = 0;
uint32_t lcdRefresh  = 0;

char    btBuf[24];
uint8_t btLen = 0;

// ═══════════════════════════════════════════════════════════
//  DFPLAYER HELPERS
// ═══════════════════════════════════════════════════════════
void dfPlay(int track) {
  if(!dfReady) return;
  dfPlayer.playMp3Folder(track);
}

// ═══════════════════════════════════════════════════════════
//  AUDIO — Buzzer tones
// ═══════════════════════════════════════════════════════════
void toneMs(uint16_t freq, uint16_t ms) { tone(BUZZER_PIN, freq, ms); }

void sndBoot()    { toneMs(880,80); delay(90); toneMs(1100,80); delay(90); toneMs(1320,200); }
void sndChirp()   { toneMs(2000, 60); }
void sndSuccess() { toneMs(1047,100); delay(110); toneMs(1319,100); delay(110); toneMs(1568,300); }
void sndError()   { toneMs(330,250); delay(60); toneMs(220,400); }
void sndAlert1()  { toneMs(880, 180); }
void sndAlert2()  { toneMs(1400,90); delay(40); toneMs(1000,90); }
void sndSnooze()  { toneMs(600,150); delay(80); toneMs(400,200); }
void sndMissed()  { toneMs(440,100); delay(50); toneMs(330,100); delay(50); toneMs(220,400); }
void sndDismiss() { toneMs(1200,60); delay(40); toneMs(800,60); }

// ═══════════════════════════════════════════════════════════
//  LED HELPERS
// ═══════════════════════════════════════════════════════════
void ledIdle()   { digitalWrite(RED_LED,LOW);  digitalWrite(GREEN_LED,LOW); }
void ledReady()  { digitalWrite(RED_LED,LOW);  digitalWrite(GREEN_LED,HIGH); }
void ledWarn()   { digitalWrite(RED_LED,HIGH); digitalWrite(GREEN_LED,LOW); }
void ledBlink(bool fast) {
  uint16_t interval = fast ? 150 : 400;
  if(millis() - lastToggle >= interval) {
    lastToggle = millis();
    digitalWrite(RED_LED, !digitalRead(RED_LED));
    digitalWrite(GREEN_LED, LOW);
  }
}

// ═══════════════════════════════════════════════════════════
//  TIME PRINTING (12-hour with AM/PM)
// ═══════════════════════════════════════════════════════════
void printTime12(uint8_t h24, uint8_t m, bool toSerial = false) {
  uint8_t h12 = h24 % 12;
  if (h12 == 0) h12 = 12;
  char ampm = (h24 >= 12) ? 'P' : 'A';

  if (toSerial) {
    if (h12 < 10) Serial.print('0');
    Serial.print(h12); Serial.print(':');
    if (m < 10) Serial.print('0');
    Serial.print(m);
    Serial.print(ampm);
  } else {
    if (h12 < 10) lcd.print('0');
    lcd.print(h12); lcd.print(':');
    if (m < 10) lcd.print('0');
    lcd.print(m);
    lcd.print(ampm);
  }
}

void lcdTimePrint(uint8_t h, uint8_t m) { printTime12(h, m, false); }
void btPrintTime(uint8_t h, uint8_t m)  { printTime12(h, m, true); }

// ═══════════════════════════════════════════════════════════
//  BLUETOOTH
// ═══════════════════════════════════════════════════════════
void btPln(const __FlashStringHelper* s) { Serial.println(s); }
void btPrt(const __FlashStringHelper* s) { Serial.print(s); }
void btPln(const char* s)               { Serial.println(s); }
void btPrt(const char* s)               { Serial.print(s); }

void btStatus() {
  btPln(F("\r\n------- SCHEDULE -------"));
  for(uint8_t i = 0; i < MAX_DOSES; i++) {
    Serial.print(i+1); Serial.print(F(": "));
    if(doses[i].hour < 0) {
      btPln(F("--:--  [unset]"));
    } else {
      btPrintTime(doses[i].hour, doses[i].minute);
      Serial.print(F("  ["));
      switch(doses[i].taken) {
        case 0: btPrt(F("PENDING")); break;
        case 1: btPrt(F("TAKEN"));   break;
        case 2: btPrt(F("MISSED"));  break;
        case 3: btPrt(F("SKIPPED")); break;
      }
      btPln(F("]"));
    }
  }
  btPln(F("------------------------"));
}

// NEW: Live Status Command
void btLiveStatus() {
  btPln(F("\r\n======= LIVE MEDICINE ADHERENCE ======="));
  
  DateTime now = rtc.now();
  uint16_t currentHM = (uint16_t)now.hour() * 100 + now.minute();

  btPrt(F("Current Time: "));
  btPrintTime(now.hour(), now.minute());
  btPln("");

  int16_t nextDiff = 9999;
  int8_t  nextIdx  = -1;

  btPln(F("\nDOSE LIST:"));
  for(uint8_t i = 0; i < MAX_DOSES; i++) {
    if(doses[i].hour < 0) continue;  // skip unset doses

    Serial.print(i+1); 
    Serial.print(F(": "));
    btPrintTime(doses[i].hour, doses[i].minute);
    Serial.print(F("  →  "));

    switch(doses[i].taken) {
      case 0: 
        btPrt(F("PENDING")); 
        // Calculate remaining time to this dose (today only)
        int16_t diff = (int16_t)doses[i].timeVal - (int16_t)currentHM;
        if(diff < 0) diff += 2400;
        if(diff < nextDiff) {
          nextDiff = diff;
          nextIdx = i;
        }
        break;
      case 1: btPrt(F("TAKEN"));   break;
      case 2: btPrt(F("MISSED"));  break;
      case 3: btPrt(F("SKIPPED")); break;
    }
    btPln("");
  }

  btPln(F("---------------------------------------"));

  if(nextIdx >= 0) {
    uint16_t hoursLeft = nextDiff / 100;
    uint16_t minsLeft  = nextDiff % 100;

    btPrt(F("→ NEXT DOSE: Dose "));
    Serial.print(nextIdx + 1);
    btPrt(F(" at "));
    btPrintTime(doses[nextIdx].hour, doses[nextIdx].minute);
    btPrt(F("  (in "));

    if(hoursLeft > 0) {
      Serial.print(hoursLeft);
      btPrt(F("h "));
    }
    Serial.print(minsLeft);
    btPln(F("min)"));
  } else {
    btPln(F("→ No more pending doses today"));
  }

  btPln(F("======================================="));
}

void btHelp() {
  btPln(F("\r\n======= COMMANDS ======="));
  btPln(F(" setN HH:MM     Set dose N (supports 24h or 12h AM/PM)"));
  btPln(F(" delN           Delete dose N"));
  btPln(F(" takeN          Manual dispense N"));
  btPln(F(" skipN          Skip dose N today"));
  btPln(F(" ok / yes       Confirm active dose"));
  btPln(F(" snooze / z     Delay alert 30 sec"));
  btPln(F(" status / s     Show all doses"));
  btPln(F(" live / l       Live adherence view (taken/pending + next dose time)"));
  btPln(F(" page / p       Cycle LCD screen"));
  btPln(F(" debug / d      Toggle debug info"));
  btPln(F(" reset          Reset today status"));
  btPln(F(" clear          Erase ALL schedules"));
  btPln(F("========================"));
}

// ═══════════════════════════════════════════════════════════
//  EEPROM
// ═══════════════════════════════════════════════════════════
void saveData() {
  EEPROM.put(ADDR_DATA,  doses);
  EEPROM.write(ADDR_MAGIC, MAGIC_NUM);
  if(debugMode) btPln(F("[EEPROM] Saved"));
}

void loadData() {
  if(EEPROM.read(ADDR_MAGIC) != MAGIC_NUM) {
    btPln(F("[EEPROM] No data (first boot)"));
    return;
  }
  EEPROM.get(ADDR_DATA, doses);
  for(uint8_t i = 0; i < MAX_DOSES; i++) {
    if(doses[i].hour >= 0)
      doses[i].timeVal = (uint16_t)doses[i].hour * 100 + doses[i].minute;
  }
  btPln(F("[EEPROM] Schedules loaded"));
}

// ═══════════════════════════════════════════════════════════
//  LCD CUSTOM CHARS
// ═══════════════════════════════════════════════════════════
void loadCustomChars() {
  uint8_t buf[8];
  auto loadChar = [&](uint8_t slot, const uint8_t* src) {
    memcpy_P(buf, src, 8);
    lcd.createChar(slot, buf);
  };
  loadChar(LCD_BELL,  CHAR_BELL);
  loadChar(LCD_CHECK, CHAR_CHECK);
  loadChar(LCD_CROSS, CHAR_CROSS);
  loadChar(LCD_PILL,  CHAR_PILL);
  loadChar(LCD_CLOCK, CHAR_CLOCK);
  loadChar(LCD_BAR1,  CHAR_BAR1);
  loadChar(LCD_BAR5,  CHAR_BAR5);
}

// ═══════════════════════════════════════════════════════════
//  LCD UI
// ═══════════════════════════════════════════════════════════
void lcdPopup(const char* l1, const char* l2, uint16_t ms = POPUP_MS) {
  lcd.clear();
  lcd.setCursor(0,0); lcd.print(l1);
  lcd.setCursor(0,1); lcd.print(l2);
  popupEnd = millis() + ms;
}

void lcdPopupF(const __FlashStringHelper* l1, const __FlashStringHelper* l2, uint16_t ms = POPUP_MS) {
  lcd.clear();
  lcd.setCursor(0,0); lcd.print(l1);
  lcd.setCursor(0,1); lcd.print(l2);
  popupEnd = millis() + ms;
}

void lcdProgressBar(uint32_t elapsed, uint32_t total) {
  uint8_t filled = (uint8_t)((elapsed * 16UL) / total);
  if(filled > 16) filled = 16;
  lcd.setCursor(0, 1);
  for(uint8_t i = 0; i < 16; i++) {
    if(i < filled) lcd.write(LCD_BAR5);
    else           lcd.write(LCD_BAR1);
  }
}

void lcdIdle() {
  DateTime now = rtc.now();
  lcd.setCursor(0,0);
  lcd.write(LCD_CLOCK);
  lcd.print(' ');
  lcdTimePrint(now.hour(), now.minute());
  lcd.print("  ");
  const char days[][4] PROGMEM = {"Sun","Mon","Tue","Wed","Thu","Fri","Sat"};
  lcd.print(days[now.dayOfTheWeek()]);

  lcd.setCursor(0,1);
  lcd.write(LCD_PILL);
  lcd.print(F(" Next: "));

  uint16_t hm = (uint16_t)now.hour() * 100 + now.minute();
  int16_t best = 9999;
  int8_t  bestIdx = -1;
  for(uint8_t i = 0; i < MAX_DOSES; i++) {
    if(doses[i].hour < 0 || doses[i].taken != 0) continue;
    int16_t diff = (int16_t)doses[i].timeVal - (int16_t)hm;
    if(diff < 0) diff += 2400;
    if(diff < best) { best = diff; bestIdx = i; }
  }
  if(bestIdx < 0) {
    lcd.print(F("None   "));
  } else {
    lcdTimePrint(doses[bestIdx].hour, doses[bestIdx].minute);
    lcd.print(F(" #"));
    lcd.print(bestIdx+1);
  }
}

void lcdDoseList(uint8_t start) {
  for(uint8_t row = 0; row < 2; row++) {
    uint8_t i = start + row;
    lcd.setCursor(0, row);
    lcd.print(i+1); lcd.print(')');
    if(i >= MAX_DOSES || doses[i].hour < 0) {
      lcd.print(F(" --:-- ---"));
    } else {
      lcdTimePrint(doses[i].hour, doses[i].minute);
      lcd.print(' ');
      switch(doses[i].taken) {
        case 0: lcd.write(LCD_PILL);  lcd.print(F("PEND")); break;
        case 1: lcd.write(LCD_CHECK); lcd.print(F("DONE")); break;
        case 2: lcd.write(LCD_CROSS); lcd.print(F("MISS")); break;
        case 3: lcd.print(F("-SKIP")); break;
      }
    }
  }
}

void lcdDebugScreen() {
  lcd.setCursor(0,0);
  lcd.print(F("ST:"));
  lcd.print(state);
  lcd.print(F(" D:"));
  lcd.print(curDose == 0xFF ? 0 : curDose+1);
  lcd.print(F(" dbg:"));
  lcd.print(debugMode ? 'Y' : 'N');

  lcd.setCursor(0,1);
  lcd.print(F("IR:"));
  lcd.print(digitalRead(IR_PIN));
  lcd.print(F(" pg:"));
  lcd.print(lcdPage);
}

void lcdAlertScreen() {
  uint32_t elapsed = millis() - alertStart;
  uint32_t total   = (state == ST_ALERT_SOFT) ? SOFT_TIMEOUT : HARD_TIMEOUT;

  lcd.setCursor(0, 0);
  lcd.write(LCD_BELL);
  lcd.print(F(" DOSE "));
  lcd.print(curDose + 1);
  lcd.print(F("  "));
  lcdTimePrint(doses[curDose].hour, doses[curDose].minute);
  lcdProgressBar(elapsed, total);
}

void lcdSnoozeScreen() {
  uint32_t rem = (snoozeEnd > millis()) ? (snoozeEnd - millis()) / 1000UL : 0;
  lcd.setCursor(0,0);
  lcd.write(LCD_CLOCK);
  lcd.print(F(" SNOOZED D"));
  lcd.print(curDose+1);
  lcd.setCursor(0,1);
  lcd.print(F("Resume in "));
  uint16_t mins = rem / 60;
  uint8_t  secs = rem % 60;
  if(mins < 10) lcd.print('0');
  lcd.print(mins); lcd.print(':');
  if(secs < 10) lcd.print('0');
  lcd.print(secs);
  lcd.print(F("   "));
}

void updateLCD() {
  if(millis() < popupEnd || millis() < cmdEnd) return;
  if(millis() - lcdRefresh < 500) return;
  lcdRefresh = millis();

  lcd.clear();

  switch(state) {
    case ST_ALERT_SOFT:
    case ST_ALERT_URGENT:
      lcdAlertScreen();
      return;
    case ST_SNOOZED:
      lcdSnoozeScreen();
      return;
    default: break;
  }

  switch(lcdPage) {
    case 0: lcdIdle();       break;
    case 1: lcdDoseList(0);  break;
    case 2: lcdDoseList(2);  break;
    case 3: lcdDoseList(4);  break;
    case 4: lcdDebugScreen();break;
  }
}

// ═══════════════════════════════════════════════════════════
//  SENSORS
// ═══════════════════════════════════════════════════════════
int readDistance() {
  return digitalRead(IR_PIN) == LOW ? 0 : 999;
}

// ═══════════════════════════════════════════════════════════
//  ALERT STATE MACHINE — FIXED SERVO
// ═══════════════════════════════════════════════════════════
void triggerAlert(uint8_t idx) {
  curDose    = idx;
  state      = ST_ALERT_SOFT;
  alertStart = millis();

  // Move to progressive position and STAY there
  uint8_t servoPos = (idx + 1) * 30;   // Dose1=30°, Dose2=60°, ..., Dose6=180°
  servo.write(servoPos);

  ledWarn();

  btPln(F("\r\n**** ALERT ****"));
  Serial.print(F("Dose ")); Serial.print(idx+1); Serial.print(F("  "));
  btPrintTime(doses[idx].hour, doses[idx].minute);
  Serial.println();
  btPln(F("Hold hand near IR  OR  send 'ok'"));
  btPln(F("Send 'snooze' to delay 30 sec"));
  btPln(F("***************"));

  dfPlay(1);
  sndAlert1();
}

void confirmDose() {
  noTone(BUZZER_PIN);
  ledReady();
  doses[curDose].taken = 1;
  saveData();

  char buf[17];
  snprintf(buf, sizeof(buf), "Dose %d TAKEN!", curDose+1);
  lcdPopup(buf, "  Great job!  ", 3000);

  btPln(F("\r\n**** TAKEN ****"));

  dfPlay(3);
  sndSuccess();

  // Only reset servo to 0° if ALL doses for the day are completed
  bool allDone = true;
  for(uint8_t i = 0; i < MAX_DOSES; i++) {
    if(doses[i].hour >= 0 && doses[i].taken == 0) {
      allDone = false;
      break;
    }
  }
  if(allDone) {
    servo.write(SERVO_CLOSED);
  }

  delay(3000);
  resetSystem();
}

void missedDose() {
  noTone(BUZZER_PIN);
  ledWarn();
  doses[curDose].taken = 2;
  saveData();

  char buf[17];
  snprintf(buf, sizeof(buf), "Dose %d MISSED", curDose+1);
  lcdPopup(buf, " Check sched  ", 3000);

  btPrt(F("\r\n>>>> MISSED: Dose "));
  Serial.println(curDose+1);

  dfPlay(4);
  sndMissed();
  delay(3000);
  resetSystem();
}

void resetSystem() {
  state   = ST_IDLE;
  curDose = 0xFF;
  ledIdle();
  popupEnd = 0;
  cmdEnd   = 0;
  lcd.clear();
}

void doSnooze() {
  if(state != ST_ALERT_SOFT && state != ST_ALERT_URGENT) {
    btPln(F("No active alert to snooze"));
    return;
  }
  noTone(BUZZER_PIN);
  state     = ST_SNOOZED;
  snoozeEnd = millis() + SNOOZE_MS;
  ledIdle();

  btPrt(F(">>>> SNOOZED: Dose "));
  Serial.print(curDose+1);
  btPln(F(" — 30 sec"));

  lcdPopupF(F("  SNOOZED  "), F("Resume in 30s"));
  dfPlay(5);
  sndSnooze();
}

void checkSchedule() {
  if(state == ST_SNOOZED) {
    if(millis() >= snoozeEnd) {
      btPrt(F("[SNOOZE] Ended, re-alerting dose "));
      Serial.println(curDose+1);
      state      = ST_ALERT_SOFT;
      alertStart = millis();
      ledWarn();
      dfPlay(1);
      sndAlert1();
    }
    return;
  }

  if(state != ST_IDLE) return;

  DateTime now = rtc.now();
  uint16_t hm  = (uint16_t)now.hour() * 100 + now.minute();

  static uint8_t lastCheckedMin = 0xFF;
  if(now.minute() == lastCheckedMin) return;
  lastCheckedMin = now.minute();

  if(debugMode) {
    Serial.print(F("[CHECK] ")); Serial.println(hm);
  }

  // 10-minute pre-alert
  for(uint8_t i = 0; i < MAX_DOSES; i++) {
    if(doses[i].hour < 0)        continue;
    if(doses[i].taken != 0)      continue;
    if(preAlerted[i])            continue;

    int16_t h = doses[i].hour;
    int16_t m = (int16_t)doses[i].minute - 10;
    if(m < 0) { m += 60; h -= 1; }
    if(h < 0) h += 24;
    uint16_t warnTime = (uint16_t)h * 100 + (uint16_t)m;

    if(hm == warnTime) {
      preAlerted[i] = true;
      dfPlay(2);
      lcdPopupF(F("UPCOMING DOSE"), F("In 10 minutes"));
      Serial.print(F("[PRE-ALERT] Dose ")); Serial.print(i+1);
      Serial.print(F(" at ")); btPrintTime(doses[i].hour, doses[i].minute);
      btPln(F(" — 10 min warning"));
    }
  }

  // Standard alert check
  for(uint8_t i = 0; i < MAX_DOSES; i++) {
    if(doses[i].hour < 0)        continue;
    if(doses[i].taken != 0)      continue;
    if(doses[i].timeVal != hm)   continue;
    triggerAlert(i);
    return;
  }
}

void updateAlert() {
  if(state == ST_IDLE || state == ST_SNOOZED) return;

  if(readDistance() < 4) {
    confirmDose();
    return;
  }

  uint32_t elapsed = millis() - alertStart;

  switch(state) {
    case ST_ALERT_SOFT:
      if(elapsed >= SOFT_TIMEOUT) {
        state = ST_ALERT_URGENT;
        ledWarn();
        lcdPopupF(F("\x00 URGENT DOSE \x00"), F(" TAKE NOW!!   "), 1000);
        btPln(F("[ESCALATE] URGENT!"));
        dfPlay(6);
        sndAlert2();
      } else {
        if((elapsed % 2500) < 200) sndAlert1();
        ledBlink(false);
      }
      break;

    case ST_ALERT_URGENT:
      if(elapsed >= HARD_TIMEOUT) {
        missedDose();
      } else {
        if((elapsed % 800) < 120) sndAlert2();
        ledBlink(true);
      }
      break;

    default: break;
  }
}

// ═══════════════════════════════════════════════════════════
//  COMMAND PROCESSOR
// ═══════════════════════════════════════════════════════════
void normalizeCmd(char* s) {
  uint8_t len = strlen(s);
  while(len > 0 && (s[len-1] == '\r' || s[len-1] == '\n' || s[len-1] == ' '))
    s[--len] = '\0';
  for(uint8_t i = 0; i < len; i++)
    s[i] = tolower((uint8_t)s[i]);
}

void handleCommand(char* raw) {
  normalizeCmd(raw);
  if(raw[0] == '\0') return;

  lcd.clear();
  lcd.setCursor(0,0); lcd.print(F("CMD: ")); lcd.print(raw);
  cmdEnd = millis() + CMD_MS;

  if(debugMode) {
    btPrt(F("[CMD] ")); btPln(raw);
  }

  if(strcmp_P(raw, PSTR("help")) == 0 || strcmp_P(raw, PSTR("h")) == 0 || strcmp_P(raw, PSTR("?")) == 0) {
    btHelp(); sndChirp(); return;
  }
  if(strcmp_P(raw, PSTR("status")) == 0 || strcmp_P(raw, PSTR("s")) == 0) {
    btStatus(); sndChirp(); return;
  }
  if(strcmp_P(raw, PSTR("live")) == 0 || strcmp_P(raw, PSTR("l")) == 0) {
    btLiveStatus(); sndChirp(); return;
  }
  if(strcmp_P(raw, PSTR("ok")) == 0 || strcmp_P(raw, PSTR("yes")) == 0) {
    if(state == ST_ALERT_SOFT || state == ST_ALERT_URGENT) confirmDose();
    else btPln(F("No active alert"));
    return;
  }
  if(strcmp_P(raw, PSTR("snooze")) == 0 || strcmp_P(raw, PSTR("z")) == 0) {
    doSnooze(); return;
  }
  if(strcmp_P(raw, PSTR("page")) == 0 || strcmp_P(raw, PSTR("p")) == 0) {
    lcdPage = (lcdPage + 1) % 5;
    Serial.print(F("Page ")); Serial.println(lcdPage);
    sndChirp(); return;
  }
  if(strcmp_P(raw, PSTR("debug")) == 0 || strcmp_P(raw, PSTR("d")) == 0) {
    debugMode = !debugMode;
    Serial.print(F("Debug: ")); btPln(debugMode ? F("ON") : F("OFF"));
    sndChirp(); return;
  }
  if(strcmp_P(raw, PSTR("reset")) == 0) {
    for(uint8_t i = 0; i < MAX_DOSES; i++)
      if(doses[i].taken != 1) doses[i].taken = 0;
    saveData();
    btPln(F("Today reset (non-taken doses cleared)"));
    lcdPopupF(F("Today Reset"), F("Pending cleared"));
    sndChirp(); return;
  }
  if(strcmp_P(raw, PSTR("clear")) == 0) {
    for(uint8_t i = 0; i < MAX_DOSES; i++)
      { doses[i].hour = -1; doses[i].taken = 0; doses[i].timeVal = 0; }
    saveData();
    btPln(F("ALL schedules erased"));
    lcdPopupF(F("ALL CLEARED"), F("Schedules reset"));
    sndError(); return;
  }

  // set command with AM/PM
  if(strncmp_P(raw, PSTR("set"), 3) == 0 && isdigit(raw[3])) {
    uint8_t idx = raw[3] - '1';
    if(idx >= MAX_DOSES) {
      btPln(F("ERROR: Use set1..set6")); sndError(); return;
    }

    char* p = raw + 4;
    while(*p && !isdigit(*p)) p++;

    int8_t h = 0, m = 0;
    bool isPM = false;

    uint8_t digits = 0;
    while(isdigit(*p) && digits < 2) { h = h*10 + (*p++ - '0'); digits++; }
    if(*p == ':' || *p == '.') p++;
    digits = 0;
    while(isdigit(*p) && digits < 2) { m = m*10 + (*p++ - '0'); digits++; }

    while(*p) {
      if(tolower(*p) == 'p') { isPM = true; break; }
      if(tolower(*p) == 'a') { isPM = false; break; }
      p++;
    }

    if(h == 0) h = 12;
    if(isPM && h != 12) h += 12;
    if(!isPM && h == 12) h = 0;

    if(h > 23 || m > 59) {
      btPln(F("ERROR: Bad time. Use HH:MM or HH:MMam/pm"));
      lcdPopupF(F("BAD TIME"), F("HH:MM or AM/PM"));
      sndError(); return;
    }

    doses[idx].hour    = h;
    doses[idx].minute  = m;
    doses[idx].timeVal = (uint16_t)h * 100 + m;
    doses[idx].taken   = 0;
    saveData();

    Serial.print(F("Dose ")); Serial.print(idx+1);
    Serial.print(F(" set to ")); btPrintTime(h, m); Serial.println();

    char l1[17], l2[17];
    snprintf(l1, sizeof(l1), "Dose %d SET", idx+1);
    snprintf(l2, sizeof(l2), "%02d:%02d saved", (h%12==0)?12:h%12, m);
    lcdPopup(l1, l2);
    sndSuccess();
    return;
  }

  if(strncmp_P(raw, PSTR("del"), 3) == 0 && isdigit(raw[3])) {
    uint8_t idx = raw[3] - '1';
    if(idx < MAX_DOSES) {
      doses[idx].hour = -1; doses[idx].taken = 0; doses[idx].timeVal = 0;
      saveData();
      Serial.print(F("Dose ")); Serial.print(idx+1); btPln(F(" deleted"));
      char l1[17]; snprintf(l1, sizeof(l1), "Dose %d DELETED", idx+1);
      lcdPopup(l1, ""); sndDismiss();
    } else {
      btPln(F("ERROR: Use del1..del6")); sndError();
    }
    return;
  }

  if(strncmp_P(raw, PSTR("take"), 4) == 0 && isdigit(raw[4])) {
    uint8_t idx = raw[4] - '1';
    if(idx < MAX_DOSES && doses[idx].hour >= 0) {
      if(state != ST_IDLE) { btPln(F("ERROR: Alert already active")); sndError(); return; }
      triggerAlert(idx);
    } else {
      btPln(F("ERROR: Dose not set")); lcdPopupF(F("NOT SET"), F("Schedule it first")); sndError();
    }
    return;
  }

  if(strncmp_P(raw, PSTR("skip"), 4) == 0 && isdigit(raw[4])) {
    uint8_t idx = raw[4] - '1';
    if(idx < MAX_DOSES && doses[idx].hour >= 0) {
      doses[idx].taken = 3; saveData();
      Serial.print(F("Dose ")); Serial.print(idx+1); btPln(F(" skipped today"));
      char l1[17]; snprintf(l1, sizeof(l1), "Dose %d SKIPPED", idx+1);
      lcdPopup(l1, "Will reset tmrw"); sndDismiss();
    } else {
      btPln(F("ERROR: Dose not set or out of range")); sndError();
    }
    return;
  }

  btPrt(F("Unknown: ")); btPln(raw);
  btPln(F("Type 'help' for commands"));
  lcdPopupF(F("UNKNOWN CMD"), F("Type 'help'"));
  sndError();
}

void processBT() {
  while(Serial.available()) {
    char c = Serial.read();
    if(c == '\n' || c == '\r') {
      if(btLen > 0) {
        btBuf[btLen] = '\0';
        handleCommand(btBuf);
        btLen = 0;
      }
    } else if(btLen < (uint8_t)(sizeof(btBuf) - 1)) {
      btBuf[btLen++] = c;
    }
  }
}

// ═══════════════════════════════════════════════════════════
//  SETUP
// ═══════════════════════════════════════════════════════════
void setup() {
  for(uint8_t i = 0; i < MAX_DOSES; i++) {
    doses[i].hour = -1;
    doses[i].taken = 0;
    doses[i].timeVal = 0;
    preAlerted[i] = false;
  }

  Serial.begin(9600);
  delay(300);

  pinMode(IR_PIN, INPUT_PULLUP);
  pinMode(RED_LED,  OUTPUT);
  pinMode(GREEN_LED,OUTPUT);
  ledIdle();

  servo.attach(SERVO_PIN);
  servo.write(SERVO_CLOSED);

  Wire.begin();
  lcd.init();
  lcd.backlight();
  loadCustomChars();

  lcd.clear();
  lcd.setCursor(0,0); lcd.print(F("MED DISPENSER"));
  lcd.setCursor(0,1); lcd.print(F("   v4.0+VOICE"));

  if(!rtc.begin()) {
    lcd.clear();
    lcd.setCursor(0,0); lcd.print(F("RTC ERROR!"));
    lcd.setCursor(0,1); lcd.print(F("Check wiring"));
    while(1) { ledBlink(true); delay(100); }
  }

  if(rtc.lostPower()) {
    rtc.adjust(DateTime(F(__DATE__), F(__TIME__)));
    lcd.clear(); lcd.print(F("RTC Adjusted"));
    delay(800);
  }

  loadData();

  DateTime now = rtc.now();
  lastDay = now.day();

  delay(500);

  dfSerial.begin(9600);

  lcd.clear();
  lcd.setCursor(0,0); lcd.print(F("DFPlayer Init..."));
  
  if(dfPlayer.begin(dfSerial)) {
    dfReady = true;
    dfPlayer.volume(DF_VOLUME);
    delay(1000);
    lcd.setCursor(0,1); lcd.print(F("Voice OK!"));
    dfPlayer.playMp3Folder(7);
    delay(2500);
  } else {
    dfReady = false;
    lcd.setCursor(0,1); lcd.print(F("Voice FAIL"));
    delay(1000);
  }

  btPln(F("\r\n================================"));
  btPln(F("  MEDICINE DISPENSER v4.0+VOICE"));
  btPln(F("================================"));
  Serial.print(F("Time: "));
  btPrintTime(now.hour(), now.minute());
  Serial.println();
  btPln(F("Type 'help' for commands"));
  btPln(F("================================"));

  lcd.clear();
  lcd.setCursor(0,0); lcd.print(F("Ready!"));
  lcd.setCursor(0,1); lcd.write(LCD_PILL); lcd.print(F(" Voice Active"));

  sndBoot();
}

// ═══════════════════════════════════════════════════════════
//  MAIN LOOP
// ═══════════════════════════════════════════════════════════
void loop() {
  processBT();

  DateTime now = rtc.now();

  if(now.day() != lastDay) {
    lastDay = now.day();
    for(uint8_t i = 0; i < MAX_DOSES; i++) {
      if(doses[i].taken != 1) doses[i].taken = 0;
      preAlerted[i] = false;
    }
    saveData();
    btPln(F("[DAILY] New day — schedules refreshed"));
    lcdPopupF(F("  NEW DAY  "), F("Doses refreshed!"));
    servo.write(SERVO_CLOSED);   // Reset only on new day
  }

  checkSchedule();
  updateAlert();
  updateLCD();
}
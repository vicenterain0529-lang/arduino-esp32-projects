#include <Wire.h>
#include <LiquidCrystal_I2C.h>

// Common I2C addresses: 0x27 or 0x3F
LiquidCrystal_I2C lcd(0x27, 16, 2);

void setup() {
  lcd.init();
  lcd.backlight();

  lcd.setCursor(0, 0);
  lcd.print("LCD Test OK!");

  lcd.setCursor(0, 1);
  lcd.print("Hello, Rain!");
}

void loop() {
  // Scrolling text demo
  lcd.setCursor(0, 1);
  lcd.print("                "); // Clear row 1
  delay(300);

  lcd.setCursor(0, 1);
  lcd.print("Hello, Rain!");
  delay(1000);
}
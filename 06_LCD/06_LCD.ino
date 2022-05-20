#include <Wire.h>
#include <LiquidCrystal_I2C.h>

LiquidCrystal_I2C lcd(0x3F, 16, 2);

void setup() {
  lcd.begin();
  lcd.backlight();
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.home ();
  lcd.print("Hello World");

  lcd.setCursor(0, 1);
  lcd.print("waiting setup...");

}

void loop() {
  // put your main code here, to run repeatedly:

}

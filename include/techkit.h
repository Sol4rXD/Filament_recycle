#include <Arduino.h>
#include <OneWire.h>
#include <Wire.h>
#include <DallasTemperature.h>
#include <LiquidCrystal_I2C.h>
#include "HX711.h"
#include "defination.h"

// Thermo couple
OneWire oneWire_1(ONE_WIRE_BUS_1);
DallasTemperature thermo_1(&oneWire_1);

OneWire oneWire_2(ONE_WIRE_BUS_2);
DallasTemperature thermo_2(&oneWire_2);

OneWire oneWire_3(ONE_WIRE_BUS_3);
DallasTemperature thermo_3(&oneWire_3);

// LCD
LiquidCrystal_I2C lcd(0x27, text_number, line_number); // Set your I2c address

// Load cell
HX711 scale;

void take_temp() {
    thermo_1.requestTemperatures(); 
    temperature_1 = thermo_1.getTempCByIndex(0);
    thermo_2.requestTemperatures(); 
    temperature_2 = thermo_2.getTempCByIndex(0);
    thermo_3.requestTemperatures(); 
    temperature_3 = thermo_3.getTempCByIndex(0);
}

void lcd_display(String x, String y) {
    lcd.clear();  
    lcd.setCursor(0, 0);
    lcd.print(x);
 
    lcd.setCursor(0, 1);
    lcd.print(y);
}

void take_weight() {
    weight = scale.get_units();
}

void pumpOn() {
  digitalWrite(pumpPin, HIGH); 
}

void pumpOff() {
  digitalWrite(pumpPin, LOW); 
}

void moveMotorsForward() {
  digitalWrite(motorA1, HIGH);
  digitalWrite(motorA2, LOW);
}

void moveMotorsBackward() {
  digitalWrite(motorA1, LOW);
  digitalWrite(motorA2, HIGH);
}

void stopMotors() {
  digitalWrite(motorA1, LOW);
  digitalWrite(motorA2, LOW);
}

// Heatcoil

// Filament detector
#include <Arduino.h>
#include <OneWire.h>
#include <Wire.h>
#include <DallasTemperature.h>
#include <LiquidCrystal_I2C.h>
#include "HX711.h"
#include <Rotary.h>
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

// Rotary encoder
Rotary encoder(ROTARY_CLK, ROTARY_DT);

void take_temp() {
    thermo_1.requestTemperatures(); 
    temperature_1 = thermo_1.getTempCByIndex(0);
    thermo_2.requestTemperatures(); 
    temperature_2 = thermo_2.getTempCByIndex(0);
    thermo_3.requestTemperatures(); 
    temperature_3 = thermo_3.getTempCByIndex(0);
}

void lcd_display(String w = "", String x = "", String y = "", String z = "") {
    lcd.setCursor(0, 0);
    lcd.print(w);
 
    lcd.setCursor(0, 1);
    lcd.print(x);
 
    lcd.setCursor(0, 2);
    lcd.print(y);
 
    lcd.setCursor(0, 3);
    lcd.print(z);
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
  digitalWrite(motorB1, HIGH);
  digitalWrite(motorB2, LOW);
  digitalWrite(motorC1, HIGH);
  digitalWrite(motorC2, LOW);
  digitalWrite(motorD1, HIGH);
  digitalWrite(motorD2, LOW);
}

void moveMotorsBackward() {
  digitalWrite(motorA1, LOW);
  digitalWrite(motorA2, HIGH);
  digitalWrite(motorB1, LOW);
  digitalWrite(motorB2, HIGH);
  digitalWrite(motorC1, LOW);
  digitalWrite(motorC2, HIGH);
  digitalWrite(motorD1, LOW);
  digitalWrite(motorD2, HIGH);
}

void stopMotors() {
  digitalWrite(motorA1, LOW);
  digitalWrite(motorA2, LOW);
  digitalWrite(motorB1, LOW);
  digitalWrite(motorB2, LOW);
  digitalWrite(motorC1, LOW);
  digitalWrite(motorC2, LOW);
  digitalWrite(motorD1, LOW);
  digitalWrite(motorD2, LOW);
}

void all_stop() {
  pumpOff();
  stopMotors();
  current_state = Stop;
}

void detech_filament() {
  switchState = digitalRead(switchPin);
  if (switchState == LOW) {
    Serial.println("Switch is pressed"); 
  } else {
    all_stop(); 
  }
}

void heatcoil_up(int x) {
  for (dutyCycle = 0; dutyCycle <= x; dutyCycle++) {
    analogWrite(dimmerPin, dutyCycle);
    delay(10); 
  }
}

void heatcoil_down(int x) {
    for (dutyCycle = x; dutyCycle >= 0; dutyCycle--) {
    analogWrite(dimmerPin, dutyCycle);
    delay(10); 
  }
}

void statement() {
    switch (current_state) {
        case Start:
            lcd_display("System start.....");
            if (digitalRead(ROTARY_BUTTON) == LOW) {
                lcd_display("Going to Setup mode.........");
                delay(2500);
                current_state = Setup;
            }
            break;
        case Setup:
            // Modify here
            lcd_display("Status: Setup",
                        "Temp 1:" + String(temperature_1, 2) + " Temp 2:" + String(temperature_2, 2) + " Temp 3:" + String(temperature_3, 2),
                        "Weight: " + String(weight, 2));
            if (digitalRead(ROTARY_BUTTON) == LOW) {
                lcd_display("Going to Normal mode.........");
                delay(2000);
                current_state = Normal;
            }
            break;
        case Normal:
            // Modify here
            lcd_display("Status: Normal",
                        "Temp 1:" + String(temperature_1, 2) + " Temp 2:" + String(temperature_2, 2) + " Temp 3:" + String(temperature_3, 2),
                        "Weight: " + String(weight, 2),
                        "Good luck!");
            break;
        case Stop:
            // Modify here
            lcd_display("Status: Stop",
                        "Temp 1:" + String(temperature_1, 2) + " Temp 2:" + String(temperature_2, 2) + " Temp 3:" + String(temperature_3, 2),
                        "Weight: " + String(weight, 2),
                        "Press to go");
            if (digitalRead(ROTARY_BUTTON) == LOW) {
                lcd_display("Going to normal mode........");
                delay(2000);
                current_state = Normal;
            }
            break;
        default:
            break;
    }
}








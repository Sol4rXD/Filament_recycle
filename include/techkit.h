#include <Arduino.h>
#include <OneWire.h>
#include <Wire.h>
#include <DallasTemperature.h>
#include <LiquidCrystal_I2C.h>
#include "HX711.h"
#include <Rotary.h>
#include "defination.h"
#include "smartdelay.h"
#include <Servo.h>

Millis timer_1(1000);

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

// Servo
Servo myservo;

void take_temp() {
    thermo_1.requestTemperatures(); 
    double newTemp1 = thermo_1.getTempCByIndex(0);
    if (newTemp1 >= 0) {
        temperature_1 = newTemp1;
    }
    
    thermo_2.requestTemperatures(); 
    double newTemp2 = thermo_2.getTempCByIndex(0);
    if (newTemp2 >= 0) {
        temperature_2 = newTemp2;
    }
    
    thermo_3.requestTemperatures(); 
    double newTemp3 = thermo_3.getTempCByIndex(0);
    if (newTemp3 >= 0) {
        temperature_3 = newTemp3;
    }
    
    if (temperature_1 < 0) {
        temperature_1 = prevTemperature_1;
    }
    
    if (temperature_2 < 0) {
        temperature_2 = prevTemperature_2;
    }
    
    if (temperature_3 < 0) {
        temperature_3 = prevTemperature_3;
    }
    
    temperature_av = (temperature_1 + temperature_2 + temperature_3) / 3;
    
    prevTemperature_1 = temperature_1;
    prevTemperature_2 = temperature_2;
    prevTemperature_3 = temperature_3;
    
    Serial.println(temperature_1);
    Serial.println(temperature_2);
    Serial.println(temperature_3);
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
  digitalWrite(pumpPinA, HIGH); 
  digitalWrite(pumpPinB, LOW);
}

void pumpOff() {
  digitalWrite(pumpPinA, LOW); 
  digitalWrite(pumpPinB, LOW); 
}

void moveMotorsForward(int pwmValue) {
  analogWrite(motorA_pwm, pwmValue);
  digitalWrite(motorA1, HIGH);
  digitalWrite(motorA2, LOW);
  
  analogWrite(motorB_pwm, pwmValue);
  digitalWrite(motorB1, HIGH);
  digitalWrite(motorB2, LOW);
  
  analogWrite(motorC_pwm, pwmValue);
  digitalWrite(motorC1, HIGH); 
  digitalWrite(motorC2, LOW);
  
  analogWrite(motorD_pwm, pwmValue);
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

// Need to use this one first
void heatcoil_up(int targetTemp) {
  take_temp();  

  double currentTemperature = 0;

  if (temperature_1 >= 0) {
    currentTemperature += temperature_1;
  } else {
    currentTemperature += prevTemperature_1;
  }

  if (temperature_2 >= 0) {
    currentTemperature += temperature_2;
  } else {
    currentTemperature += prevTemperature_2;
  }

  if (temperature_3 >= 0) {
    currentTemperature += temperature_3;
  } else {
    currentTemperature += prevTemperature_3;
  }

  currentTemperature /= 3; 

  prevTemperature_1 = temperature_1;
  prevTemperature_2 = temperature_2;
  prevTemperature_3 = temperature_3;

  double error = targetTemp - currentTemperature;

  if (abs(error) < tempTolerance) {
    integral += error;
  }

  integral = constrain(integral, -100, 100);  

  double pidOutput = kp * error + ki * integral + kd * (error - prevError);
  prevError = error;
  
  dutyCycle = constrain(dutyCycle + pidOutput, 0, maxDutyCycle);
  analogWrite(dimmerPin, dutyCycle);

  delay(10);  

  HEATCOIL_TEMP = currentTemperature;
}

void heatcoil_down(int x) {
  for (dutyCycle = HEATCOIL_TEMP; dutyCycle >= x; dutyCycle--) {
    analogWrite(dimmerPin, dutyCycle);
    delay(10); 
  }
  HEATCOIL_TEMP = x;
}

void all_stop() {
  pumpOff();
  stopMotors();
  heatcoil_down(0);
  lcd.clear();
  lcd_display("Check filament");
  delay(3000);
  lcd.clear();
  delay(500);
  current_state = Stop;
}

void detech_filament() {
  switchState = digitalRead(switchPin);
  if (switchState == LOW) {
    Serial.println("Filament is not in");  
    all_stop();
  } else {
    Serial.println("Filament is in");  
  }
}

void servo_spin() {
  myservo.write(45); 
  delay(5000); 
  myservo.write(90); 
  delay(5000); 
  myservo.write(135); 
  delay(5000); 
}

void all_operations() {
    switch (current_state) {
        case Start:
            lcd_display("System start.....","Press to go!");
            Serial.println("System start.....");
            if (digitalRead(ROTARY_BUTTON) == LOW) {
                lcd.clear();
                lcd_display("Going to Setup mode.........");
                Serial.println("Going to Setup mode.........");
                delay(2500);
                lcd.clear();
                current_state = Setup;
            }
            break;
        case Setup:
            // Modify here
            if (timer_1) {
              take_temp();
            }
            heatcoil_up(240);
            pumpOn();
            lcd_display("Status: Setup",
                        "Temperature: " + String(temperature_av, 2),
                        "Hold to normal");
            if (digitalRead(ROTARY_BUTTON) == LOW) {
                Serial.println("Going to Normal mode.........");
                lcd.clear();
                lcd_display("Going to Normal mode.........");
                delay(2000);
                lcd.clear();
                current_state = Normal;
            }
            break;
        case Normal:
            if (timer_1) {
              take_temp();
            }
            // heatcoil_up(240);
            pumpOn();
            moveMotorsForward(255);
            lcd_display("Status: Normal",
                        "Temperature: " + String(temperature_av, 2),
                        "Hold to stop");
            if (digitalRead(ROTARY_BUTTON) == LOW) {
                lcd.clear();
                lcd_display("Going to stop........"); 
                delay(3000);
                lcd.clear();
                current_state = Stop;
            }
            detech_filament();
            break;
        case Stop:
            // Modify here
            if (timer_1) {
              take_temp();
            }
            stopMotors();
            pumpOff();
            lcd_display("Status: Stop",
                        "Temperature:" + String(temperature_av, 2),
                        "Check filament",
                        "Hold to go");
            if (digitalRead(ROTARY_BUTTON) == LOW) {
                Serial.println("Going to normal mode........");
                lcd.clear();
                lcd_display("Going back to normal...");
                delay(2000);
                lcd.clear();
                current_state = Normal;
            }
            break;
        default:
            break;
    }
}



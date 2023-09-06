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
Millis timer_100ms(100);

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
    
    Serial.println("temperature is: ");
    Serial.println(temperature_1);
    Serial.println(temperature_2);
    Serial.println(temperature_3);
    Serial.println("temperature av :");
    Serial.println(temperature_av);
    Serial.println("---------------------");
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

void moveMotorsForward() {
  // analogWrite(motorA_pwm, pwmValue);
  digitalWrite(motorA1, HIGH);
  digitalWrite(motorA2, LOW);
  
  // analogWrite(motorB_pwm, pwmValue);
  digitalWrite(motorB1, HIGH);
  digitalWrite(motorB2, LOW);
  
  // analogWrite(motorC_pwm, pwmValue);
  digitalWrite(motorC1, HIGH); 
  digitalWrite(motorC2, LOW);
  
  // analogWrite(motorD_pwm, pwmValue);
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

  double error = targetTemp - temperature_av;

  if (abs(error) < tempTolerance) {
    integral += error;
  }

  integral = constrain(integral, -100, 100);  

  double pidOutput = kp * error + ki * integral + kd * (error - prevError);
  prevError = error;
  
  dutyCycle = constrain(dutyCycle + pidOutput, 0, maxDutyCycle);
  analogWrite(dimmerPWM_1, dutyCycle);
  analogWrite(dimmerPWM_2, dutyCycle);

  delay(10);  

  HEATCOIL_TEMP = temperature_av;
}

void heatcoil_down(int x) {
  for (dutyCycle = HEATCOIL_TEMP; dutyCycle >= x; dutyCycle--) {
    analogWrite(dimmerPin, dutyCycle);
    delay(10); 
  }
  HEATCOIL_TEMP = x;
}

// MAX is 255
void heatcoil_up_test(int x) {
  for (int dutyCycle = HEATCOIL_TEMP; dutyCycle <= x; dutyCycle++) {
    analogWrite(dimmerPWM_1, dutyCycle);
    delay(10); 
  }
  HEATCOIL_TEMP = x;
}

void zeroCrossing() {
  static unsigned long lastInterruptTime = 0;
  unsigned long currentTime = micros();

  if (currentTime - lastInterruptTime >= 800) {
    lastInterruptTime = currentTime;

    unsigned long delayTime = (pwmValue > 0) ? pwmValue * 2 : 0;

    delayMicroseconds(delayTime);
    digitalWrite(dimmerPWM_1, HIGH);
  }
}

void stop_servo() {
  myservo.detach(); 
}

void all_stop() {
  pumpOff();
  stopMotors();
  heatcoil_down(0);
  stop_servo();
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
  }
}

void servo_spin() {
  myservo.writeMicroseconds(1375);
}

// Not finish
void magnetic_check() {
  if(digitalRead(magneticPin) == HIGH) {
    // Do something
  }
  else{
    // Do something
  }
}

void all_operations() {
    switch (current_state) {
        case Start:
            lcd_display("System start.....","Press to go!");
            // Serial.println("System start.....");
            if (Serial.available() > 0) {
                String input = Serial.readString();
                input.trim(); 
                if (input.toInt() == 1) {
                    Serial.println("Changing to Normal mode...");
                    lcd.clear();
                    lcd_display("Changing to Normal mode...");
                    delay(2000);
                    lcd.clear();
                    current_state = Setup;
                }
            }
            if (digitalRead(ROTARY_BUTTON) == LOW) {
                lcd.clear();
                lcd_display("Going to Setup mode.........");
                // Serial.println("Going to Setup mode.........");
                delay(2500);
                lcd.clear();
                current_state = Setup;
            }
            break;
        case Setup:
            // Modify here
            if (timer_100ms) {
              take_temp();
            }
            heatcoil_up(250);
            // moveMotorsForward();
            pumpOn();
            lcd_display("Status: Setup",
                        "Temperature: " + String(temperature_av, 2),
                        "Hold to normal");
            if (Serial.available() > 0) {
                String input = Serial.readString();
                input.trim(); 
                if (input.toInt() == 0) {
                    Serial.println("Changing to Stop mode...");
                    lcd.clear();
                    lcd_display("Changing to Stop mode...");
                    delay(2000);
                    lcd.clear();
                    current_state = Stop;
                }
            }
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
            if (timer_100ms) {
              take_temp();
            }
            servo_spin();
            heatcoil_up(250);
            pumpOn();
            // moveMotorsForward();
            lcd_display("Status: Normal",
                        "Temperature: " + String(temperature_av, 2),
                        "Hold to stop");
            if (Serial.available() > 0) {
                String input = Serial.readString();
                input.trim(); // Remove leading/trailing spaces
                if (input.toInt() == 0) {
                    Serial.println("Changing to Stop mode...");
                    lcd.clear();
                    lcd_display("Changing to Stop mode...");
                    delay(2000);
                    lcd.clear();
                    current_state = Stop;
                }
            }
            if (digitalRead(ROTARY_BUTTON) == LOW) {
                lcd.clear();
                lcd_display("Going to stop........"); 
                delay(1000);
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
            heatcoil_down(0);
            lcd_display("Status: Stop",
                        "Temperature:" + String(temperature_av, 2),
                        "Check filament",
                        "Hold to go");
            if (Serial.available() > 0) {
                String input = Serial.readString();
                input.trim(); // Remove leading/trailing spaces
                if (input.toInt() == 1) {
                    Serial.println("Changing to Normal mode...");
                    lcd.clear();
                    lcd_display("Changing to Normal mode...");
                    delay(2000);
                    lcd.clear();
                    current_state = Normal;
                }
            }
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



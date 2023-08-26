#include <Arduino.h>
#include "techkit.h"

void setup() {
    Serial.begin(9600);
    Serial.println("System start");

    thermo_1.begin();
    thermo_2.begin();
    thermo_3.begin();

    lcd.init();
    lcd.init();
    lcd.backlight();
    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print("System start");

    scale.begin(DOUT_PIN, SCK_PIN);

    pinMode(pumpPinA, OUTPUT);
    pinMode(pumpPinB, OUTPUT);

    pinMode(switchPin, INPUT_PULLUP);

    pinMode(dimmerPin, OUTPUT);
    analogWrite(dimmerPin, 0);

    pinMode(ROTARY_BUTTON, INPUT_PULLUP);

    pinMode(motorA_pwm, OUTPUT);
    pinMode(motorB_pwm, OUTPUT);
    pinMode(motorC_pwm, OUTPUT);
    pinMode(motorD_pwm, OUTPUT);

    pinMode(motorA1, OUTPUT);
    pinMode(motorA2, OUTPUT);
    pinMode(motorB1, OUTPUT);
    pinMode(motorB2, OUTPUT);
    pinMode(motorC1, OUTPUT);
    pinMode(motorC2, OUTPUT);
    pinMode(motorD1, OUTPUT);
    pinMode(motorD2, OUTPUT);
}

void loop() {
    // all_operations();
    pumpOn();
    moveMotorsForward(255);
}

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

    pinMode(pumpPin, OUTPUT);
    pumpOff();

    pinMode(switchPin, INPUT_PULLUP);

    pinMode(dimmerPin, OUTPUT);
    analogWrite(dimmerPin, 0);

    pinMode(ROTARY_BUTTON, INPUT_PULLUP);
}
 
void loop() {
    all_operations();
}

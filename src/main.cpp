#include <Arduino.h>
#include "techkit.h"

void setup() {
    Serial.begin(9600);
    Serial.println("System start");

    thermo_1.begin();
    thermo_2.begin();
    thermo_3.begin();

    lcd.begin(text_number, line_number);
    lcd.backlight();
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
    // Test all sensor (Ex)
    take_temp();
    delay(100);
    lcd.clear();
    lcd_display("Test 1", "Test 2", "Test 3", "Test 4");
    delay(100);
    take_weight();
    pumpOn();
    delay(5000);
    pumpOff();
    moveMotorsForward();
    delay(3000);
    moveMotorsBackward();
    delay(3000);
    stopMotors();
    detech_filament();
    heatcoil_up(255);
    delay(5000);
    heatcoil_down(0);
    statement();
}

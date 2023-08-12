#include <Arduino.h>
#include "techkit.h"

extern void take_temp();
extern void lcd_display(String x, String y);
extern void take_weight();
extern void pumpOn();
extern void pumpOff();
extern void moveMotorsForward();
extern void moveMotorsBackward();
extern void stopMotors();
extern void detech_filament();
extern void heatcoil();

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
}
 
void loop() {
    // Test all sensor
    take_temp();
    delay(100);
    lcd_display("Test 1", "Test 2");
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
    heatcoil();
    delay(1000);
}

#include <Arduino.h>
#include "techkit.h"

void setup() {
    Serial.begin(115200);
    pinMode(dimmerPin, OUTPUT);
    analogWrite(dimmerPin, 0);
}

void adjust_heatcoil_level() {
  Serial.println("Enter desired heat coil level:");
  while (!Serial.available()) {
    // Wait for user input
  }
  int targetLevel = Serial.parseInt();
  
  if (targetLevel > HEATCOIL_TEMP) {
    for (int newLevel = HEATCOIL_TEMP; newLevel <= targetLevel; newLevel++) {
      analogWrite(dimmerPin, newLevel);
      delay(10);
    }
    Serial.println(targetLevel);
  } else if (targetLevel < HEATCOIL_TEMP) {
    for (int newLevel = HEATCOIL_TEMP; newLevel >= targetLevel; newLevel--) {
      analogWrite(dimmerPin, newLevel);
      delay(10);
    }
    Serial.println(targetLevel);
  }
  
  HEATCOIL_TEMP = targetLevel;
}

void loop() {
  if (Serial.available()) {
    adjust_heatcoil_level();
  }
}



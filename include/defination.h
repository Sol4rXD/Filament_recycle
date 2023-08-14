#include <Arduino.h>

String payload;

// LCD
int text_number = 20;
int line_number = 4;

// Thermo couple
#define ONE_WIRE_BUS_1 13
#define ONE_WIRE_BUS_2 14
#define ONE_WIRE_BUS_3 15

uint32_t temperature_1;
uint32_t temperature_2;
uint32_t temperature_3;

// Load cell
#define DOUT_PIN  11
#define SCK_PIN   12

float weight;

// Pump
#define pumpPin 1

// Motor 1
#define motorA1 2
#define motorA2 3   

// Motor 2
#define motorB1 4   
#define motorB2 5  

// Motor 3
#define motorC1 6
#define motorC2 7 

// Motor 4
#define motorD1 8   
#define motorD2 9 

// Filament detector
#define switchPin 2
int switchState;

// Heat coil
#define dimmerPin 10
int dutyCycle;
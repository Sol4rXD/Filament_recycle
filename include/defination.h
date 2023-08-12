#include <Arduino.h>

String payload;

// LCD
int text_number = 16;
int line_number = 2;

// Thermo couple
#define ONE_WIRE_BUS_1 4
#define ONE_WIRE_BUS_2 5 
#define ONE_WIRE_BUS_3 6 

uint32_t temperature_1;
uint32_t temperature_2;
uint32_t temperature_3;

// Load cell
#define DOUT_PIN  2
#define SCK_PIN   3

float weight;

// Pump
#define pumpPin 9

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
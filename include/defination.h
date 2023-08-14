#include <Arduino.h>

String payload;

// Statements
enum state {Start,
            Setup,
            Normal};

enum state current_state = Start;

// LCD
int text_number = 20;
int line_number = 4;

// Thermo couple
#define ONE_WIRE_BUS_1 13
#define ONE_WIRE_BUS_2 14
#define ONE_WIRE_BUS_3 15

float temperature_1;
float temperature_2;
float temperature_3;

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

// Rotary encoder
#define ROTARY_CLK 2
#define ROTARY_DT 3
#define ROTARY_BUTTON 4



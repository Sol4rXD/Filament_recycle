#include <Arduino.h>

String payload;

// PID
const int targetTemperature = 100;  
const int tempTolerance = 2;        
const int maxDutyCycle = 255;       

// PID constants
const double kp = 1.0;  
const double ki = 0.1;     
const double kd = 0.01;  

int dutyCycle = 0;
int prevError = 0;
int integral = 0;

// Statements
enum state {Start,
            Setup,
            Normal,
            Stop};

enum state current_state = Start;

// LCD
uint8_t text_number = 20;
uint8_t line_number = 4;

// Thermo couple
#define ONE_WIRE_BUS_1 11
#define ONE_WIRE_BUS_2 9
#define ONE_WIRE_BUS_3 15

float temperature_1;
float temperature_2;
float temperature_3;
float temperature_av;

float prevTemperature_1;
float prevTemperature_2;
float prevTemperature_3;

// Load cell
#define DOUT_PIN  5
#define SCK_PIN   6

float weight;

// Pump
#define pumpPinA 10
#define pumpPinB 8

// Motor Pwm
#define motorA_pwm 2
#define motorB_pwm 3 
#define motorC_pwm 6
#define motorD_pwm 7

// Motor 1
#define motorA1 31
#define motorA2 29  

// Motor 2
#define motorB1 27  
#define motorB2 25 

// Motor 3
#define motorC1 30
#define motorC2 28

// Motor 4
#define motorD1 26  
#define motorD2 24

// Filament detector
#define switchPin 16
int switchState;

// Heat coil (Max temp is 255)
#define dimmerPin 2
#define dimmerPWM_1 6
#define dimmerPWM_2 7
#define dimmerPWM_3 13

int pwmValue = 255;   
int HEATCOIL_TEMP; 

// Rotary encoder
#define ROTARY_CLK 2
#define ROTARY_DT 3
#define ROTARY_BUTTON 4

unsigned long buttonPressStartTime = 0;
bool stopConfirmation = false; 

// Magnetic Hall sensor
#define magneticPin 5

// Servo
const int Servo_PIN = 5;




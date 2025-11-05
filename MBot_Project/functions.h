#ifndef FUNCTIONS_H
#define FUNCTIONS_H

#include <MeMCore.h>
#include <math.h>

extern MePort port4;
extern MePort port3;

#define TURNING_TIME_MS 390 // The time duration (ms) for turning
// Integrated Ultrasonic Sensor and Line Following code
extern MeUltrasonicSensor ultrasonic;
extern MeLineFollower lineSensor;

// --- Wall-following Parameters ---
extern float targetDist;
extern float tolerance;
extern int baseSpeed;
extern int correction;
extern int timeout_ms;

// Defines pin to recieve LDR reading
#define LDR A1
// Defines time delay before taking another RGB/LDR reading
#define RGBWait 200
#define LDRWait 10

// Selector pins to be used from 2-4 Decoder
extern int selA; // 1A on HD74LS139 = A2 pin
extern int selB;

// Motor port assignment
extern MeDCMotor leftMotor;
extern MeDCMotor rightMotor;
extern int IRPin;
extern uint8_t motorSpeed;

// Setting motor speed to an integer between 1 and 255
// The larger the number, the faster the speed

extern MeBuzzer buzzer; // create the buzzer object

// Array to store logic values(A2, A3) to turn on LED in the order red, blue, green
extern int RGBPins[3][2];
// Array to store RGB values in the order black, white and range
extern float calibrate[3][3];
extern String calibrateNames[3];
// Array to store RGB values in the order red, green, orange, pink, light blue and white
extern float colours[6][3];
extern String coloursNames[6];

void celebrate();
void stopMotor();
void moveForward();
void turnRight();
void turnLeft();
void uTurn();
void doubleLeftTurn();
void doubleRightTurn();
void nudgeLeft();
void nudgeRight();
int shineIR();
void shineRed();
void shineGreen();
void shineBlue();
int detectColour();
void calibrateSensor();
void calibrateColour();
int getColour();
float getAvgReading();

#endif
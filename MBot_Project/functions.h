#ifndef FUNCTIONS_H
#define FUNCTIONS_H

#include <MeMCore.h>
#include <math.h>

/* ---DEFINITIONS--- */
#define LDR A1      // Defines pin to recieve LDR reading
#define RGBWait 200 // Defines time delay before taking another RGB/LDR reading
#define LDRWait 10
#define TURNING_TIME_MS 345 // The time duration (ms) for turning
#define FORWARD_TIME_MS 795
#define COOLDOWN 80
/* ---PORT AND PIN DECLARATIONS--- */
extern MePort port4;
extern MePort port3;
extern MeUltrasonicSensor ultrasonic;
extern MeLineFollower lineSensor;
extern MeBuzzer buzzer;
extern MeDCMotor leftMotor;
extern MeDCMotor rightMotor;
extern MeRGBLed mLed;

extern int IRPin;
// Selector pins to be used from 2-4 Decoder
extern const int selA; // 1A on HD74LS139 = A2 pin
extern const int selB; // 1B on HD74LS139 = A3 pin

/* ---WALL-FOLLOWING PARAMTERS--- */
extern float targetDist;
extern float targetDistIR;
extern int correction;
extern int timeout_ms;

/* ---MOTOR PARAMETERS--- */
extern int baseSpeed; // LARGER = FASTER
extern float leftSpeed;
extern float rightSpeed;

/* ---STORED COLOUR SENSOR VALUES--- */
extern int RGBPins[3][2];
extern float calibrate[3][3];
extern String calibrateNames[3];
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
int shineIR();
void calibrateSensor();
void calibrateColour();
int getColour();
float getAvgReading();
void doChallenge(int);
void calibrateDistance();
#endif
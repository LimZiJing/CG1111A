#ifndef FUNCTIONS_H
#define FUNCTIONS_H

#include <MeMCore.h>
#include <math.h>

MePort port4(PORT_4);
MePort port3(PORT_3);

MeDCMotor motorL(M1);
MeDCMotor motorR(M2);
#define TURNING_TIME_MS 390 // The time duration (ms) for turning
// Integrated Ultrasonic Sensor and Line Following code
MeUltrasonicSensor ultrasonic(PORT_1); // Side-mounted ultrasonic sensor (right side)
MeLineFollower lineSensor(PORT_2);     // Line follower underneath

// --- Wall-following Parameters ---
float targetDist = 8.0; // Desired distance (cm) from side wall
float tolerance = 1.0;  // Acceptable deviation
int baseSpeed = 150;    // Forward speed
int correction = 40;    // Adjustment for small turns
int timeout_ms = 30;    // Ultrasonic read timeout

// Defines pin to recieve LDR reading
#define LDR A1
// Defines time delay before taking another RGB/LDR reading
#define RGBWait 150
#define LDRWait 10

// Selector pins to be used from 2-4 Decoder
const int selA = port3.pin1(); // 1A on HD74LS139 = A2 pin
const int selB = port3.pin2(); // 1B on HD74LS139 = A3 pin

// Motor port assignment
MeDCMotor leftMotor(M1);
MeDCMotor rightMotor(M2);
int IRPin = 0;
uint8_t motorSpeed = 255;

// Setting motor speed to an integer between 1 and 255
// The larger the number, the faster the speed

MeBuzzer buzzer; // create the buzzer object

// Array to store logic values(A2, A3) to turn on LED in the order red, blue, green
int RGBPins[3][2] = {{HIGH, LOW}, {LOW, HIGH}, {HIGH, HIGH}};
// Array to store RGB values in the order black, white and range
int calibrate[3][3] = {{0, 0, 0}, {0, 0, 0}, {0, 0, 0}};
String calibrateNames[3] = {"black", "white", "range"};
// Array to store RGB values in the order red, green, orange, pink, light blue and white
int colours[6][3] = {{0, 0, 0}, {0, 0, 0}, {0, 0, 0}, {0, 0, 0}, {0, 0, 0}, {0, 0, 0}};
String coloursNames[6] = {"red", "green", "orange", "pink", "light blue", "white"};

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
int getAvgReading();

#endif
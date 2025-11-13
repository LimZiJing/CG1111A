#include "functions.h"

/* ---PORT AND PIN DECLARATIONS--- */
MePort port4(PORT_4);
MePort port3(PORT_3);
MeUltrasonicSensor ultrasonic(PORT_1);
MeLineFollower lineSensor(PORT_2);
MeBuzzer buzzer;
MeDCMotor leftMotor(M1);
MeDCMotor rightMotor(M2);
MeRGBLed mled(0, 30);

// Selector pins to be used from 2-4 Decoder
const int selA = port3.pin1(); // 1A on HD74LS139 = A2 pin
const int selB = port3.pin2(); // 1B on HD74LS139 = A3 pin

/* ---WALL-FOLLOWING PARAMTERS--- */
float targetDist = 10.39; // Desired distance (cm) from side wall
float targetDistIR = 58;  // Desired reflected IR value
int correction = 0;       // Correction for wheel speed used in PID
int timeout_ms = 30;      // Ultrasonic read timeout

/* ---PID CONSTANTS--- */
float Kp = 31;
float Kp_IR = 0.4;

float error = 0;

/* ---MOTOR PARAMETERS--- */
int baseSpeed = 245; // LARGER = FASTER
// Initialise PID movement speed to baseSpeed
float leftSpeed = (float)baseSpeed;
float rightSpeed = (float)baseSpeed;

/* ---STORED COLOUR SENSOR VALUES--- */
// Array to store logic values(A2, A3) to turn on LED in the order red, blue, green
int RGBPins[3][2] = {{HIGH, LOW}, {LOW, HIGH}, {HIGH, HIGH}};
String calibrateNames[3] = {"black", "white", "range"};
float calibrate[3][3] = {{801.43, 784.29, 827.71}, {914.57, 979.29, 971.00}, {113.14, 195.00, 143.29}};
String coloursNames[6] = {"red", "green", "orange", "pink", "light blue", "white"};
float colours[6][3] = {{252.10, 126.85, 89.49}, {126.86, 233.52, 181.02}, {259.19, 192.79, 86.69}, {261.12, 239.12, 231.1}, {111.72, 228.10, 244.07}, {252.42, 256.49, 254.24}};

void setup() {
    Serial.begin(9600);

    // Pin initialisation for selector pins and LED
    pinMode(selA, OUTPUT);
    pinMode(selB, OUTPUT);
    mLed.setpin(13);

    // Set to true if calibration is required
    bool calibrateColoursOn = false;
    bool calibrateDistanceOn = true;

    if (calibrateColoursOn == true) {
        calibrateSensor();
        calibrateColour();
    }
    delay(1000);

    // calibrate targetDist and targetDist IR base on starting position
    if (calibrateDistanceOn == true) {
        calibrateDistance();
    }

    Serial.println("=== Setup Complete! Start in 5s. ===");

    delay(5000);
}

void loop() {

    // Check for presence of black strip
    int lineState = lineSensor.readSensors();

    // Get current distance and reflected irValues
    float distance = ultrasonic.distanceCm(timeout_ms);
    int irValue = shineIR();

    /* ---PID ALGORITHM--- */

    // If no walls on both sides, move straight without PID
    if (distance < targetDist + 2 && targetDistIR < 35) {

        mLed.setColor(0, 0, 255); // Blue colour --> No sensor mode
        mLed.show();
        leftMotor.run(-baseSpeed);
        rightMotor.run(baseSpeed);

    }
    // If no wall on right side, use IR sensor with PID
    else if (distance > targetDist) {
        mLed.setColor(255, 0, 0); // Red colour --> IR sensor mode
        mLed.show();

        error = targetDistIR - irValue;
        float correction = Kp_IR * error;

        // Adjust motor speeds
        leftSpeed = baseSpeed - correction;
        rightSpeed = baseSpeed + correction;
        moveForward();
    }
    // Else, use ultrasonic sensor with PID
    else {
        mLed.setColor(0, 255, 0); // Green colour --> Ultrasonic sensor mode
        mLed.show();

        error = targetDist - distance;
        float correction = Kp * error;

        // Adjust motor speeds
        leftSpeed = baseSpeed - correction;
        rightSpeed = baseSpeed + correction;
        moveForward();
    }

    // Check if black strip is detected
    if (lineState == S1_IN_S2_IN || lineState == S1_IN_S2_OUT | lineState == S1_OUT_S2_IN) {
        stopMotor();
        // Get the colour of the challenge
        int colour = getColour();
        // Do the challenge required by the colour
        doChallenge(colour);
    }
}

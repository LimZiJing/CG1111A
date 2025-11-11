#include "functions.h"

/* ---PORT AND PIN DECLARATIONS--- */
MePort port4(PORT_4);
MePort port3(PORT_3);
MeUltrasonicSensor ultrasonic(PORT_1);
MeLineFollower lineSensor(PORT_2);
MeBuzzer buzzer;
MeDCMotor leftMotor(M1);
MeDCMotor rightMotor(M2);

int IRPin = 0;
// Selector pins to be used from 2-4 Decoder
const int selA = port3.pin1(); // 1A on HD74LS139 = A2 pin
const int selB = port3.pin2(); // 1B on HD74LS139 = A3 pin

/* ---WALL-FOLLOWING PARAMTERS--- */
float targetDist = 11.0; // Desired distance (cm) from side wall
float DistErr = 1.5;
int targetDistIR = 230;
int correction = 40; // Adjustment for small turns
int timeout_ms = 30; // Ultrasonic read timeout

/* ---PID CONSTANTS--- */
float Kp = 28.0;
float Ki = 0.0;
float Kd = 0.0;

float Kp_IR = 0.45;
float Ki_IR = 0.0;
float Kd_IR = 0.0;

float integral_IR = 0;
float derivative_IR = 0;
float error = 0;
float previous_error = 0;
float derivative = 0;
float lastError = 0;
float integral = 0;

/* ---MOTOR PARAMETERS--- */
int baseSpeed = 250; // LARGER = FASTER
float leftSpeed = (float)baseSpeed;
float rightSpeed = (float)baseSpeed;

/* ---STORED COLOUR SENSOR VALUES--- */
// Array to store logic values(A2, A3) to turn on LED in the order red, blue, green
int RGBPins[3][2] = {{HIGH, LOW}, {LOW, HIGH}, {HIGH, HIGH}};
String calibrateNames[3] = {"black", "white", "range"};
float calibrate[3][3] = {{871.00, 797.14, 793.43}, {928.43, 980.86, 969.71}, {57.43, 183.71, 176.29}};
String coloursNames[6] = {"red", "green", "orange", "pink", "light blue", "white"};
float colours[6][3] = {{226.46, 109.65, 92.78}, {104.66, 232.59, 193.42}, {258.81, 190.56, 100.22}, {258.32, 237.35, 231.86}, {106.57, 225.85, 244.05}, {259.544, 256.19, 254.79}};

// FIXME (UNCALIBRATED): float calibrate[3][3] = {{0, 0, 0}, {0, 0, 0}, {0, 0, 0}};
// FIXME (UNCALIBRATED): float colours[6][3] = {{0, 0, 0}, {0, 0, 0}, {0, 0, 0}, {0, 0, 0}, {0, 0, 0}, {0, 0, 0}};

void setup() {
    Serial.begin(9600);

    pinMode(selA, OUTPUT);
    pinMode(selB, OUTPUT);
    //calibrateSensor();
    //calibrateColour();

    Serial.println("=== Setup Complete! Start in 5s. ===");
    delay(5000);
}

void loop() {

    // --- Step 1: Check for black strip using line sensor ---
    int lineState = lineSensor.readSensors();

    // Move Robot until a challenge is detected (line sensor detects black)
    // Get distance from wall via ultrasonic and IR sensors
    float distance = ultrasonic.distanceCm(timeout_ms);
    int irValue = shineIR();

    /*
    Serial.print("Distance: ");
    Serial.print(distance);
    Serial.println(" cm");
    */

/*
    Serial.print("IRVALUE: ");
    Serial.print(irValue);
    Serial.println(" V");
    */

 

    /* ---PID ALGORITHM--- */
    error = targetDist - distance;
    // Serial.println(error);

    if (distance > targetDist + DistErr) {
        //Serial.println("ir using");
        error = targetDistIR - irValue;
        integral_IR += error;
        derivative_IR = error - previous_error;
        float correction = Kp_IR * error + Ki_IR * integral_IR + Kd_IR * derivative_IR;
        leftSpeed = baseSpeed - correction;
        rightSpeed = baseSpeed + correction;
        moveForward();
        previous_error = error;

    } else {

        integral += error;                   // accumulate error
        derivative = error - previous_error; // how fast error is changing
        float correction = Kp * error + Ki * integral + Kd * derivative;
        // Adjust motor speeds
        leftSpeed = baseSpeed - correction;
        rightSpeed = baseSpeed + correction;
        moveForward();
        previous_error = error;
    }

    if (lineState == S1_IN_S2_IN || lineState == S1_IN_S2_OUT | lineState == S1_OUT_S2_IN) {
        stopMotor();
        // Serial.println(">> Black strip detected! Stop for waypoint challenge.");
        //  Code to return RGB values of current colour: 0->red, 1->green, 2->orange, 3->pink, 4->light blue and 5->white
        int colour = getColour();
        // Serial.print("Detected colour code: ");
        // Serial.println(colour);

        // Step 3: Perform waypoint action based on colour

        doChallenge(colour);

        // After completing action, resume maze navigation
        // Serial.println("Resuming wall following...");
        delay(200);
    }
}

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
int targetDistIR = 250;
int correction = 0;  // Adjustment for small turns
int timeout_ms = 30; // Ultrasonic read timeout
float tolerance = 2.0;

/* ---PID CONSTANTS--- */
float Kp = 70;
float Ki = 0.0;
float Kd = 10.0;

float integral = 0;
float derivative = 0;

float error = 0;
float previous_error = 0;
float lastError = 0;

/* ---MOTOR PARAMETERS--- */
int baseSpeed = 250; // LARGER = FASTER
float leftSpeed = (float)baseSpeed;
float rightSpeed = (float)baseSpeed;

/* ---STORED COLOUR SENSOR VALUES--- */
// Array to store logic values(A2, A3) to turn on LED in the order red, blue, green
int RGBPins[3][2] = {{HIGH, LOW}, {LOW, HIGH}, {HIGH, HIGH}};
String calibrateNames[3] = {"black", "white", "range"};
float calibrate[3][3] = {{757.43, 761.14, 812.71}, {906.00, 979.14, 968.57}, {148.57, 218.00, 155.86}};
String coloursNames[6] = {"red", "green", "orange", "pink", "light blue", "white"};
float colours[6][3] = {{242.00, 121.15, 85.55}, {134.12, 230.10, 180.44}, {255.00, 188.33, 89.05}, {262.36, 237.96, 233.50}, {138.53, 226.43, 245.65}, {261.13, 254.00, 254.64}};

// FIXME (UNCALIBRATED): float calibrate[3][3] = {{0, 0, 0}, {0, 0, 0}, {0, 0, 0}};
// FIXME (UNCALIBRATED): float colours[6][3] = {{0, 0, 0}, {0, 0, 0}, {0, 0, 0}, {0, 0, 0}, {0, 0, 0}, {0, 0, 0}};

void setup() {
    Serial.begin(9600);

    pinMode(selA, OUTPUT);
    pinMode(selB, OUTPUT);

    bool calibrateColoursOn = false;
    bool calibrateDistanceOn = true;

    if (calibrateColoursOn == true) {
        calibrateSensor();
        calibrateColour();
    }

    if (calibrateDistanceOn == true) {
        calibrateDistance();
    }
    Serial.println("=== Setup Complete! Start in 5s. ===");
    delay(5000);
}

void loop() {

    // --- Check for black strip using line sensor ---
    int lineState = lineSensor.readSensors();

    float distance = ultrasonic.distanceCm(timeout_ms);
    int irValue = shineIR();

    // determine wall presence
    bool leftWall = (irValue > 150 && irValue < 800); // valid IR range
    bool rightWall = (distance > 4 && distance < 20); // valid ultrasonic range

    /* ---PID ALGORITHM--- */
    if (leftWall && rightWall) {
        // both walls detected — use combined normalization
        float norm_IR = (irValue - targetDistIR) / targetDistIR;
        float norm_US = (distance - targetDist) / targetDist;
        error = norm_IR + norm_US; // balanced between both
    } else if (leftWall) {
        // only left wall — follow IR
        error = -(irValue - targetDistIR) / targetDistIR;
    } else if (rightWall) {
        // only right wall — follow ultrasonic
        error = (distance - targetDist) / targetDist;
    } else {
        // no wall (shouldn’t happen)
        error = 0;
    }

    // Adjust motor speeds
    leftSpeed = baseSpeed - correction;
    rightSpeed = baseSpeed + correction;
    leftSpeed = constrain(leftSpeed, 0, 255);
    rightSpeed = constrain(rightSpeed, 0, 255);

    moveForward();
    previous_error = error;

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

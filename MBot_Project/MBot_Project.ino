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
int targetDistIR = 370;
int correction = 0;  // Adjustment for small turns
int timeout_ms = 30; // Ultrasonic read timeout
float tolerance = 2.0;

/* ---PID CONSTANTS--- */
float Kp = 31;
float Ki = 0.0;
float Kd = 0.0;

float Kp_IR = 0.45;
float Ki_IR = 0.0;
float Kd_IR = 0.0;

float integral = 0;
float derivative = 0;
float integral_IR = 0;
float derivative_IR = 0;

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

    /* ---PID ALGORITHM--- */
    // Serial.println(error);
    if (distance > targetDist + tolerance) {

        /*
            Serial.print("IRVALUE: ");
            Serial.print(irValue);
            Serial.println(" V");
        */

        error = targetDistIR - irValue;
        float correction = Kp_IR * error + Ki_IR * integral_IR + Kd_IR * derivative_IR;
        leftSpeed = baseSpeed - correction;
        rightSpeed = baseSpeed + correction;
        moveForward();

    } else {

        /*
        Serial.print("Distance: ");
        Serial.print(distance);
        Serial.println(" cm");
        */

        error = targetDist - distance;
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

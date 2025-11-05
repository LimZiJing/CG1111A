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
float targetDist = 8.0; // Desired distance (cm) from side wall
float tolerance = 1.0;  // Acceptable deviation
int baseSpeed = 150;    // Forward speed
int correction = 40;    // Adjustment for small turns
int timeout_ms = 30;    // Ultrasonic read timeout

/* ---MOTOR PARAMETERS--- */
uint8_t motorSpeed = 255; // LARGER = FASTER

/* ---STORED COLOUR SENSOR VALUES--- */
// Array to store logic values(A2, A3) to turn on LED in the order red, blue, green
int RGBPins[3][2] = {{HIGH, LOW}, {LOW, HIGH}, {HIGH, HIGH}};
String calibrateNames[3] = {"black", "white", "range"};
float calibrate[3][3] = {{748.71, 790.86, 804.29}, {907.14, 982.86, 971.00}, {158.43, 192.00, 166.71}};
String coloursNames[6] = {"red", "green", "orange", "pink", "light blue", "white"};
float colours[6][3] = {{253.39, 121.81, 91.56}, {123.25, 228.82, 178.96}, {255.46, 192.96, 93.96}, {259.14, 236.41, 230.09}, {105.08, 223.69, 241.89}, {259.37, 255.95, 255.66}};

// FIXME (UNCALIBRATED): float calibrate[3][3] = {{0, 0, 0}, {0, 0, 0}, {0, 0, 0}};
// FIXME (UNCALIBRATED): float colours[6][3] = {{0, 0, 0}, {0, 0, 0}, {0, 0, 0}, {0, 0, 0}, {0, 0, 0}, {0, 0, 0}};

void setup() {
    Serial.begin(9600);

    pinMode(selA, OUTPUT);
    pinMode(selB, OUTPUT);
    // FIXME (TO CALIBRATE): calibrateSensor();
    // FIXME (TO CALIBRATE): calibrateColour();

    Serial.println("=== Setup Complete! ===");
}

void loop() {

    delay(5000);

    // --- Step 1: Check for black strip using line sensor ---
    int lineState = lineSensor.readSensors();

    // Move Robot until a challenge is detected (line sensor detects black)
    while (!(lineState == S1_IN_S2_IN || lineState == S1_IN_S2_OUT | lineState == S1_OUT_S2_IN)) {
        // Get distance from wall via ultrasonic and IR sensors
        float distance = ultrasonic.distanceCm(timeout_ms);
        int irValue = shineIR();

        if (distance <= 0 || distance > 40) {
            // Invalid or no reading -> assume open space
            moveForward();
            return;
        }

        // Too close to the left (IR Sensor)
        if (irValue < 300) {
            nudgeRight();
        }

        Serial.print("Distance: ");
        Serial.print(distance);
        Serial.println(" cm");

        // Maintain target distance from wall
        if (distance > targetDist + tolerance) {
            // Too far from wall → steer toward wall
            nudgeRight();
        } else if (distance < targetDist - tolerance) {
            // Too close → steer away from wall
            nudgeLeft();
        } else {
            // Within range → go straight
            moveForward();
        }
    }

    if (lineState == S1_IN_S2_IN || lineState == S1_IN_S2_OUT | lineState == S1_OUT_S2_IN) {
        stopMotor();
        Serial.println(">> Black strip detected! Stop for waypoint challenge.");
        // Code to return RGB values of current colour: 0->red, 1->green, 2->orange, 3->pink, 4->light blue and 5->white
        int colour = getColour();
        Serial.print("Detected colour code: ");
        Serial.println(colour);

        // Step 3: Perform waypoint action based on colour
        switch (colour) {
        case 0: // RED → Turn Left
            Serial.println("Action: TURN LEFT");
            turnLeft();
            break;

        case 1: // GREEN → Turn Right
            Serial.println("Action: TURN RIGHT");
            turnRight();
            break;

        case 2: // ORANGE → 180° Turn
            Serial.println("Action: TURN AROUND");
            uTurn();
            break;

        case 3: // PINK → Two Left Turns
            Serial.println("Action: TWO LEFT TURNS");
            doubleLeftTurn();
            break;

        case 4: // LIGHT BLUE → Two Right Turns
            Serial.println("Action: TWO RIGHT TURNS");
            doubleRightTurn();
            break;

        case 5: // WHITE → End of maze
            Serial.println("Action: END OF MAZE — STOP");
            stopMotor();
            delay(500);
            celebrate();
            while (true)
                ; // stop forever
            break;
        }

        // After completing action, resume maze navigation
        Serial.println("Resuming wall following...");
        delay(200);
    }
}

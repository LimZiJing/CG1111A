#include "functions.h"

void setup() {
    Serial.begin(9600);

    // put your setup code here, to run once:
    pinMode(selA, OUTPUT);
    pinMode(selB, OUTPUT);
    calibrateSensor();
    calibrateColour();

    digitalWrite(selA, LOW);
    digitalWrite(selB, LOW);

    // IR Sensor Testing
    Serial.println("=== Setup Complete! ===");
}

void loop() {
    // LDR CODE

    // delay(5000);
    // Serial.println(coloursNames[getColour()]);

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
            while (true)
                ; // stop forever
            break;
        }

        // After completing action, resume maze navigation
        Serial.println("Resuming wall following...");
        delay(200);
    }
}

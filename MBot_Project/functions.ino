#include "functions.h"

void celebrate() { // Code for playing celebratory tune
    // Each of the following "function calls" plays a single tone.
    // The numbers in the bracket specify the frequency and the duration (ms)
    buzzer.tone(392, 200);
    buzzer.tone(523, 200);
    buzzer.tone(659, 200);
    buzzer.tone(784, 200);
    buzzer.tone(659, 150);
    buzzer.tone(784, 400);
    buzzer.noTone();
}

void stopMotor() {     // Code for stopping motor
    leftMotor.stop();  // Stop left motor
    rightMotor.stop(); // Stop right motor
}
void moveForward() {            // Code for moving forward for some short interval
    leftMotor.run(-motorSpeed); // Negative: wheel turns anti-clockwise
    rightMotor.run(motorSpeed); // Positive: wheel turns clockwise
}
void turnRight() {               // Code for turning right 90 deg
    leftMotor.run(-motorSpeed);  // Positive: wheel turns clockwise
    rightMotor.run(-motorSpeed); // Positive: wheel turns clockwise
    delay(TURNING_TIME_MS);
    leftMotor.stop();  // Stop left motor
    rightMotor.stop(); // Stop right motor
    delay(1000);       // Stop for 1000 ms
}
void turnLeft() { // Code for turning left 90 deg
    // Turning left (on the spot):
    leftMotor.run(motorSpeed);  // Positive: wheel turns clockwise
    rightMotor.run(motorSpeed); // Positive: wheel turns clockwise
    delay(TURNING_TIME_MS);     // Keep turning left for this time duration
    leftMotor.stop();           // Stop left motor
    rightMotor.stop();          // Stop right motor
    delay(1000);                // Stop for 1000 ms
}
void uTurn() { // Code for u-turn
    turnLeft();
    moveForward();
    delay(500);
    turnLeft();
}
void doubleLeftTurn() { // Code for double left turn
    turnLeft();
    moveForward();
    delay(800);
    turnLeft();
}
void doubleRightTurn() { // Code for double right turn
    turnRight();
    moveForward();
    delay(800);
    turnRight();
}
void nudgeLeft() {              // Code for nudging slightly to the left for some short interval
    leftMotor.run(motorSpeed);  // Positive: wheel turns clockwise
    rightMotor.run(motorSpeed); // Positive: wheel turns clockwise
    delay(100);                 // Keep turning left for this time duration
    leftMotor.stop();           // Stop left motor
    rightMotor.stop();          // Stop right motor
    delay(1000);                // Stop for 1000 ms
}
void nudgeRight() {              // Code for nudging slightly to the right for some short interval
    leftMotor.run(-motorSpeed);  // Positive: wheel turns clockwise
    rightMotor.run(-motorSpeed); // Positive: wheel turns clockwise
    delay(TURNING_TIME_MS);
    leftMotor.stop();  // Stop left motor
    rightMotor.stop(); // Stop right motor
    delay(1000);       // Stop for 1000 ms
}
int shineIR() { // Code for turning on the IR emitter only
    digitalWrite(selA, LOW);
    digitalWrite(selB, LOW);

    delay(5); // small delay to stabilize

    // Read reflected light
    int readingOn = port4.aRead1();

    // Turn IR emitter OFF and read ambient light (for baseline)
    digitalWrite(selA, LOW);
    digitalWrite(selB, HIGH);
    delay(5);
    int readingOff = port4.aRead1();

    // Compute reflected IR difference
    int irValue = readingOff - readingOn;

    return irValue;
}

void calibrateSensor() {
    // Code to calibrate sensor and store RGB values of black, white and range
    for (int i = 0; i < 2; i++) {
        Serial.println("Put " + calibrateNames[i] + " sample for calibration...");
        delay(10000);
        for (int j = 0; j < 3; j++) {
            digitalWrite(selA, RGBPins[j][0]);
            digitalWrite(selB, RGBPins[j][1]);
            delay(RGBWait);
            calibrate[i][j] = getAvgReading();
            digitalWrite(selA, LOW);
            digitalWrite(selB, LOW);
            delay(RGBWait);
        }
        Serial.println("Completed for" + calibrateNames[i] + "!");
    }
    for (int k = 0; k < 3; k++) {
        calibrate[2][k] = calibrate[1][k] - calibrate[0][k];
    }
    // Code to print out stored RGB values of black, white and range
    Serial.println("Black, white and range values successfully stored!");
    for (int a = 0; a < 3; a++) {
        Serial.println(calibrateNames[a]);
        for (int b = 0; b < 3; b++) {
            Serial.println(calibrate[a][b]);
        }
    }
    delay(5000);
}

void calibrateColour() {
    // Code to store RGB values of red, green, orange, pink, light blue, white
    for (int i = 0; i < 6; i++) {
        Serial.println("Put " + coloursNames[i] + " sample for calibration...");
        delay(10000);
        for (int j = 0; j < 3; j++) {
            digitalWrite(selA, RGBPins[j][0]);
            digitalWrite(selB, RGBPins[j][1]);
            delay(RGBWait);
            colours[i][j] = getAvgReading();
            colours[i][j] = ((colours[i][j] - calibrate[0][j]) / calibrate[2][j]) * 255.0;
            digitalWrite(selA, LOW);
            digitalWrite(selB, LOW);
            delay(RGBWait);
        }
    }
    // Code to print out stored RGB values of the colours
    Serial.println("All colour values successfully stored!");
    for (int a = 0; a < 6; a++) {
        Serial.println(coloursNames[a]);
        for (int b = 0; b < 3; b++) {
            Serial.println(colours[a][b]);
        }
    }
    delay(5000);
}

int getColour() {
    // Code to return RGB values of current colour: 0->red, 1->green, 2->orange, 3->pink, 4->light blue and 5->white
    float current[3] = {0, 0, 0};
    for (int i = 0; i < 3; i++) {
        digitalWrite(selA, RGBPins[i][0]);
        digitalWrite(selB, RGBPins[i][1]);
        delay(RGBWait);
        current[i] = getAvgReading();
        current[i] = (current[i] - calibrate[0][i]) / calibrate[2][i] * 255.0;
        digitalWrite(selA, LOW);
        digitalWrite(selB, LOW);
        delay(RGBWait);
    }
    // Code to run colour matching algorithm
    float minimum = 300.0;
    int minidx = 0;
    for (int j = 0; j < 6; j++) {
        float temp = 0;
        for (int k = 0; k < 3; k++) {
            temp += abs(current[k] - colours[j][k]);
        }
        if (temp < minimum) {
            minidx = j;
            minimum = temp;
        }
    }
    return minidx;
}

float getAvgReading() {
    // Code to return average reading of LDR
    float total = 0;
    for (int i = 0; i < 7; i++) {
        total += analogRead(LDR);
        delay(LDRWait);
    }
    return total / 7;
}

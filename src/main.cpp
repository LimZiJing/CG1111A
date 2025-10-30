#include <MeMCore.h>
#include <math.h>

MePort port4(PORT_4);
MePort port3(PORT_3);
MeUltrasonicSensor ultrasonic(PORT_1);  // Mount on side wall (left or right)
MeDCMotor motorL(M1);
MeDCMotor motorR(M2);
#define TURNING_TIME_MS 390 // The time duration (ms) for turning

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
void shineIR() { // Code for turning on the IR emitter only
}
void shineRed() { // Code for turning on the red LED only
}
void shineGreen() { // Code for turning on the green LED only
}
void shineBlue() { // Code for turning on the blue LED only
}
int detectColour() {
    // Shine Red, read LDR after some delay
    // Shine Green, read LDR after some delay
    // Shine Blue, read LDR after some delay
    // Run algorithm for colour decoding
}

// Array to store logic values(A2, A3) to turn on LED in the order red, blue, green
int RGBPins[3][2] = {{HIGH, LOW}, {LOW, HIGH}, {HIGH, HIGH}};
// Array to store RGB values in the order black, white and range
int calibrate[3][3] = {{0, 0, 0}, {0, 0, 0}, {0, 0, 0}};
String calibrateNames[3] = {"black", "white", "range"};
// Array to store RGB values in the order red, green, orange, pink, light blue and white
int colours[6][3] = {{0, 0, 0}, {0, 0, 0}, {0, 0, 0}, {0, 0, 0}, {0, 0, 0}, {0, 0, 0}};
String coloursNames[6] = {"red", "green", "orange", "pink", "light blue", "white"};

void calibrateSensor() {
    // Code to calibrate sensor and store RGB values of black, white and range
    for (int i = 0; i < 2; i++) {
        Serial.println("Put " + calibrateNames[i] + " sample for calibration...");
        delay(5000);
        for (int j = 0; j < 3; j++) {
            digitalWrite(selA, RGBPins[j][0]);
            digitalWrite(selB, RGBPins[j][1]);
            delay(RGBWait);
            calibrate[i][j] = getAvgReading();
            digitalWrite(selA, LOW);
            digitalWrite(selB, LOW);
            delay(RGBWait);
        }
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
        delay(5000);
        for (int j = 0; j < 3; j++) {
            digitalWrite(selA, RGBPins[j][0]);
            digitalWrite(selB, RGBPins[j][1]);
            delay(RGBWait);
            colours[i][j] = getAvgReading();
            colours[i][j] = (colours[i][j] - calibrate[0][j]) / calibrate[2][j] * 255;
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
    int current[3] = {0, 0, 0};
    for (int i = 0; i < 3; i++) {
        digitalWrite(selA, RGBPins[i][0]);
        digitalWrite(selB, RGBPins[i][1]);
        delay(RGBWait);
        current[i] = getAvgReading();
        current[i] = (current[i] - calibrate[0][i]) / calibrate[2][i] * 255;
        digitalWrite(selA, LOW);
        digitalWrite(selB, LOW);
        delay(RGBWait);
    }
    // Code to run colour matching algorithm
    int min = 300;
    int minidx = 0;
    for (int j = 0; j < 6; j++) {
        int temp = 0;
        for (int k = 0; k < 3; k++) {
            temp += abs(current[k] - colours[j][k]);
        }
        if (temp < min) {
            minidx = j;
            min = temp;
        }
    }
    return minidx;
}

int getAvgReading() {
    // Code to return average reading of LDR
    int total = 0;
    for (int i = 0; i < 7; i++) {
        total += analogRead(LDR);
        delay(LDRWait);
    }
    return total / 7;
}

void setup() {
    Serial.begin(9600);

    // put your setup code here, to run once:
    pinMode(selA, OUTPUT);
    pinMode(selB, OUTPUT);
    calibrateSensor();
    calibrateColour();

    // IR Sensor Code
    pinMode(selA, OUTPUT);
    pinMode(selB, OUTPUT);

    digitalWrite(selA, LOW);
    digitalWrite(selB, LOW);

    // IR Sensor Testing
    Serial.println("=== IR Sensor Test Started ===");
}

void loop() {

    // LDR CODE

    delay(5000);
    Serial.println(coloursNames[getColour()]);

    // IR SENSOR CODE

    // Turn IR emitter ON
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

    Serial.print("Ambient: ");
    Serial.print(readingOff);
    Serial.print("  Reflected: ");
    Serial.print(readingOn);
    Serial.print("  Δ = ");
    Serial.println(irValue);

    delay(200);
}
float targetDist = 8.0;    // Target side distance from wall (cm)
float tolerance  = 1.0;    // Allowable error range (cm)
int baseSpeed    = 150;    // Normal forward speed
int correction   = 40;     // Turning adjustment
int timeout_ms   = 30;     // Timeout for faster responsiveness

void setup() {
  Serial.begin(9600);
  Serial.println("=== Ultrasonic Sensor Wall-Following Test ===");
}

void loop() {
  // Measure distance with timeout (recommended from the brief)
  float distance = ultrasonic.distanceCm(timeout_ms);

  // Handle invalid readings
  if (distance <= 0 || distance > 40) {
    Serial.println("No echo - possible open wall");
    moveForward();
    return;
  }

  Serial.print("Distance: ");
  Serial.print(distance);
  Serial.println(" cm");

  // Maintain target distance from side wall
  if (distance > targetDist + tolerance) {
    // Too far from wall → turn right
    nudgeRight();
  } 
  else if (distance < targetDist - tolerance) {
    // Too close to wall → turn left
    nudgeLeft();
  } 
  else {
    // Within target distance → go straight
    moveForward();
  }

  delay(50);
}


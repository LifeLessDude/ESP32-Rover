#include <Bluepad32.h>
#include <Adafruit_PWMServoDriver.h>

// Initialize the PCA9685 servo driver
Adafruit_PWMServoDriver pca9685 =
Adafruit_PWMServoDriver(0x40);

// Define motor control pins for Front Left motor (Motor 1)
const int motor1Pin1 = 12;
const int motor1Pin2 = 13;

// Define motor control pins for Front Right motor (Motor 2)
const int motor2Pin1 = 14;
const int motor2Pin2 = 15;

// Define motor control pins for Back Left motor (Motor 3)
const int motor3Pin1 = 16;
const int motor3Pin2 = 17;

// Define motor control pins for Back Right motor (Motor 4)
const int motor4Pin1 = 18;
const int motor4Pin2 = 19;

// Threshold for joystick deadzone
int joystickThreshold = 15;

#define servoMin 10
#define servoMax 150
#define servoSpeed 15

int minPulse = 150; // Minimum pulse length count
int maxPulse = 600; // Maximum pulse length count

// Servo assignments for right and left arms
const int servoBaseLeft = 6;
const int servoShoulderLeft = 7;

const int servoElbowLeft = 8;
const int servoWristRollLeft = 9;
const int servoWristPitchLeft = 10;
const int servoGripLeft = 11;

const int servoBaseRight = 0;
const int servoShoulderRight = 1;
const int servoElbowRight = 2;
const int servoWristRollRight = 3;
const int servoWristPitchRight = 4;
const int servoGripRight = 5;

int baseAngleLeft = 90, shoulderAngleLeft = 90, elbowAngleLeft
= 90, wristRollAngleLeft = 90, wristPitchAngleLeft = 90,
gripAngleLeft = 90;
int baseAngleRight = 90, shoulderAngleRight = 90,
elbowAngleRight = 90, wristRollAngleRight = 90,
wristPitchAngleRight = 90, gripAngleRight = 90;

// Gamepad object
GamepadPtr myGamepad;

void setup() {
Serial.begin(115200);
BP32.setup(&onConnectedGamepad, &onDisconnectedGamepad);

// Initialize motor control pins as outputs
pinMode(motor1Pin1, OUTPUT);
pinMode(motor1Pin2, OUTPUT);
pinMode(motor2Pin1, OUTPUT);
pinMode(motor2Pin2, OUTPUT);
pinMode(motor3Pin1, OUTPUT);
pinMode(motor3Pin2, OUTPUT);
pinMode(motor4Pin1, OUTPUT);
pinMode(motor4Pin2, OUTPUT);

// Initialize PCA9685
pca9685.begin();
pca9685.setOscillatorFrequency(27000000);
pca9685.setPWMFreq(50);
delay(10);
resetServos();
}

void loop() {
BP32.update(); // Update gamepad state

if (myGamepad) {
processGamepadInput();
}
}

void onConnectedGamepad(GamepadPtr gp) {
myGamepad = gp;
Serial.println("Gamepad connected");
}

void onDisconnectedGamepad(GamepadPtr gp) {
myGamepad = nullptr;
Serial.println("Gamepad disconnected");
}

void processGamepadInput() {
int leftX = myGamepad->axisX();
int leftY = myGamepad->axisY();
int rightX = myGamepad->axisRX();
int rightY = myGamepad->axisRY();
uint16_t buttons = myGamepad->buttons();
uint16_t dpad = myGamepad->dpad();

// Rover controls using buttons
if (buttons & 0x0010) { moveLeft(); } // L1 - Horizontal

left
else if (buttons & 0x0040) { moveBackward(); } // L2 -
Backward
else if (buttons & 0x0020) { moveRight(); } // R1 -
Horizontal right
else if (buttons & 0x0080) { moveForward(); } // R2 -
Forward
else if (buttons & 0x0050) { turnLeft(); } // L1 + L2 -
Turn left
else if (buttons & 0x00A0) { turnRight(); } // R1 + R2 -
Turn right
else {
stopMovement(); // Stop the rover if no button is pressed
}

// Base and shoulder movement for both arms using joysticks
baseAngleRight = constrain(baseAngleRight + map(rightX,
-512, 508, -1, 1), servoMin, servoMax);
updateServo(servoBaseRight, baseAngleRight);
shoulderAngleRight = constrain(shoulderAngleRight +
map(rightY, -512, 508, -1, 1), servoMin, servoMax);
updateServo(servoShoulderRight, shoulderAngleRight);

baseAngleLeft = constrain(baseAngleLeft + map(leftX, -512,
508, -1, 1), servoMin, servoMax);
updateServo(servoBaseLeft, baseAngleLeft);
shoulderAngleLeft = constrain(shoulderAngleLeft + map(leftY,
-512, 508, -1, 1), servoMin, servoMax);
updateServo(servoShoulderLeft, shoulderAngleLeft);

// Right Arm Controls (prioritize button combinations)
if ((buttons & 0x000C) == 0x000C) { // Square + Triangle for
wrist pitch up
wristPitchAngleRight = constrain(wristPitchAngleRight + 1,
servoMin, servoMax);
updateServo(servoWristPitchRight, wristPitchAngleRight);

}
else if ((buttons & 0x000A) == 0x000A) { // Triangle +
Circle for wrist pitch down
wristPitchAngleRight = constrain(wristPitchAngleRight - 1,
servoMin, servoMax);
updateServo(servoWristPitchRight, wristPitchAngleRight);
}
else if ((buttons & 0x0005) == 0x0005) { // Square + Cross
for gripper open
gripAngleRight = constrain(gripAngleRight + 1, servoMin,
servoMax);
updateServo(servoGripRight, gripAngleRight);
}
else if ((buttons & 0x0003) == 0x0003) { // Circle + Cross
for gripper close
gripAngleRight = constrain(gripAngleRight - 1, servoMin,
servoMax);
updateServo(servoGripRight, gripAngleRight);
}
// Individual button checks (only executed if no combination
is active)
else if (buttons & 0x0008) { // Triangle - Elbow up
elbowAngleRight = constrain(elbowAngleRight + 1, servoMin,
servoMax);
updateServo(servoElbowRight, elbowAngleRight);
}
else if (buttons & 0x0001) { // Cross - Elbow down
elbowAngleRight = constrain(elbowAngleRight - 1, servoMin,
servoMax);
updateServo(servoElbowRight, elbowAngleRight);
}
else if (buttons & 0x0004) { // Square - Wrist roll
anticlockwise
wristRollAngleRight = constrain(wristRollAngleRight - 1,
servoMin, servoMax);
updateServo(servoWristRollRight, wristRollAngleRight);

}
else if (buttons & 0x0002) { // Circle - Wrist roll clockwise
wristRollAngleRight = constrain(wristRollAngleRight + 1,
servoMin, servoMax);
updateServo(servoWristRollRight, wristRollAngleRight);
}

// Left Arm Controls (using D-pad, prioritize combinations)
if ((dpad & 0x09) == 0x09) { // Up + Left for wrist pitch up
wristPitchAngleLeft = constrain(wristPitchAngleLeft + 1,
servoMin, servoMax);
updateServo(servoWristPitchLeft, wristPitchAngleLeft);
}
else if ((dpad & 0x05) == 0x05) { // Up + Right for wrist
pitch down
wristPitchAngleLeft = constrain(wristPitchAngleLeft - 1,
servoMin, servoMax);
updateServo(servoWristPitchLeft, wristPitchAngleLeft);
}
else if ((dpad & 0x0A) == 0x0A) { // Down + Left for gripper
open
gripAngleLeft = constrain(gripAngleLeft + 1, servoMin,
servoMax);
updateServo(servoGripLeft, gripAngleLeft);
}
else if ((dpad & 0x06) == 0x06) { // Down + Right for
gripper close
gripAngleLeft = constrain(gripAngleLeft - 1, servoMin,
servoMax);
updateServo(servoGripLeft, gripAngleLeft);
}
// Individual D-pad checks
else if (dpad & 0x01) { // Up - Elbow up
elbowAngleLeft = constrain(elbowAngleLeft + 1, servoMin,
servoMax);
updateServo(servoElbowLeft, elbowAngleLeft);

}
else if (dpad & 0x02) { // Down - Elbow down
elbowAngleLeft = constrain(elbowAngleLeft - 1, servoMin,
servoMax);
updateServo(servoElbowLeft, elbowAngleLeft);
}
else if (dpad & 0x08) { // Left - Wrist roll anticlockwise
wristRollAngleLeft = constrain(wristRollAngleLeft - 1,
servoMin, servoMax);
updateServo(servoWristRollLeft, wristRollAngleLeft);
}
else if (dpad & 0x04) { // Right - Wrist roll clockwise
wristRollAngleLeft = constrain(wristRollAngleLeft + 1,
servoMin, servoMax);
updateServo(servoWristRollLeft, wristRollAngleLeft);
}
}

void updateServo(int servoNum, int angle) {
pca9685.setPWM(servoNum, 0, angleToPulse(angle));
delay(servoSpeed);
}

int angleToPulse(int angle) {
return map(angle, servoMin, servoMax, minPulse, maxPulse);
}

void resetServos() {
for (int i = 0; i <= 11; i++) {
pca9685.setPWM(i, 0, angleToPulse(90));
}
}

// Rover movement functions
void moveForward() {
digitalWrite(motor1Pin1, HIGH); digitalWrite(motor1Pin2,

LOW);
digitalWrite(motor2Pin1, HIGH); digitalWrite(motor2Pin2,
LOW);
digitalWrite(motor3Pin1, HIGH); digitalWrite(motor3Pin2,
LOW);
digitalWrite(motor4Pin1, HIGH); digitalWrite(motor4Pin2,
LOW);
}

void moveBackward() {
digitalWrite(motor1Pin1, LOW); digitalWrite(motor1Pin2,
HIGH);
digitalWrite(motor2Pin1, LOW); digitalWrite(motor2Pin2,
HIGH);
digitalWrite(motor3Pin1, LOW); digitalWrite(motor3Pin2,
HIGH);
digitalWrite(motor4Pin1, LOW); digitalWrite(motor4Pin2,
HIGH);
}

void moveLeft() {
digitalWrite(motor1Pin1, LOW); digitalWrite(motor1Pin2,
HIGH);
digitalWrite(motor2Pin1, HIGH); digitalWrite(motor2Pin2,
LOW);
digitalWrite(motor3Pin1, LOW); digitalWrite(motor3Pin2,
HIGH);
digitalWrite(motor4Pin1, HIGH); digitalWrite(motor4Pin2,
LOW);
}

void moveRight() {
digitalWrite(motor1Pin1, HIGH); digitalWrite(motor1Pin2,
LOW);
digitalWrite(motor2Pin1, LOW); digitalWrite(motor2Pin2,
HIGH);

digitalWrite(motor3Pin1, HIGH); digitalWrite(motor3Pin2,
LOW);
digitalWrite(motor4Pin1, LOW); digitalWrite(motor4Pin2,
HIGH);
}

void turnLeft() {
digitalWrite(motor1Pin1, HIGH); digitalWrite(motor1Pin2,
LOW);
digitalWrite(motor2Pin1, LOW); digitalWrite(motor2Pin2,
HIGH);
digitalWrite(motor3Pin1, HIGH); digitalWrite(motor3Pin2,
LOW);
digitalWrite(motor4Pin1, LOW); digitalWrite(motor4Pin2,
HIGH);
}

void turnRight() {
digitalWrite(motor1Pin1, LOW); digitalWrite(motor1Pin2,
HIGH);
digitalWrite(motor2Pin1, HIGH); digitalWrite(motor2Pin2,
LOW);
digitalWrite(motor3Pin1, LOW); digitalWrite(motor3Pin2,
HIGH);
digitalWrite(motor4Pin1, HIGH); digitalWrite(motor4Pin2,
LOW);
}

void stopMovement() {
// Stop all motors
digitalWrite(motor1Pin1, LOW);
digitalWrite(motor1Pin2, LOW);
digitalWrite(motor2Pin1, LOW);
digitalWrite(motor2Pin2, LOW);
digitalWrite(motor3Pin1, LOW);
digitalWrite(motor3Pin2, LOW);

digitalWrite(motor4Pin1, LOW);
digitalWrite(motor4Pin2, LOW);
}

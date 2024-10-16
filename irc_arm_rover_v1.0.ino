#include <Bluepad32.h>
#include <ESP32Servo.h>

//I don't know how many pins are available and which ones to use.
//Please modify according to preference and Expansion board of ESP32.

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

int joystickThreshold = 15;  // Deadzone for joystick input

// Create servo objects for 6 servos on Left Arm
Servo servoBaseLeft;
Servo servoShoulderLeft;
Servo servoElbowLeft;
Servo servoWristRollLeft;
Servo servoWristPitchLeft;
Servo servoGripLeft;

// Create servo objects for 6 servos on Right Arm
Servo servoBaseRight;
Servo servoShoulderRight;
Servo servoElbowRight;
Servo servoWristRollRight;
Servo servoWristPitchRight;
Servo servoGripRight;

// Define servo pin assignments for Left Arm
int servoBaseLeftPin = 20;
int servoShoulderLeftPin = 21;
int servoElbowLeftPin = 22;
int servoWristRollLeftPin = 23;
int servoWristPitchLeftPin = 24;
int servoGripLeftPin = 25;

// Define servo pin assignments for Right Arm
int servoBaseRightPin = 26;
int servoShoulderRightPin = 27;
int servoElbowRightPin = 28;
int servoWristRollRightPin = 29;
int servoWristPitchRightPin = 30;
int servoGripRightPin = 31;

// Define rover control pins
const int roverForwardPin = 32;  // Example pin, change as needed
const int roverBackwardPin = 33; // Example pin, change as needed
const int roverLeftPin = 34;     // Example pin, change as needed
const int roverRightPin = 35;    // Example pin, change as needed

int minUs = 1000; // Minimum microsecond pulse width
int maxUs = 2000; // Maximum microsecond pulse width

int gripAngleLeft = 90;
int wristRollAngleLeft = 90;
int shoulderAngleLeft = 90;
int baseAngleLeft = 90;
int elbowAngleLeft = 90;
int wristPitchAngleLeft = 90;

int gripAngleRight = 90;
int wristRollAngleRight = 90;
int shoulderAngleRight = 90;
int baseAngleRight = 90;
int elbowAngleRight = 90;
int wristPitchAngleRight = 90;

int servoSpeed = 20;  

enum ControlMode { LEFT_ARM, RIGHT_ARM, BOTH_ARMS, ROVER };
ControlMode currentMode = BOTH_ARMS;  // Start in both arms mode


GamepadPtr myGamepad;

void setup() {
  Serial.begin(115200);

  // Initialize motor control pins as outputs
  pinMode(motor1Pin1, OUTPUT);
  pinMode(motor1Pin2, OUTPUT);
  pinMode(motor2Pin1, OUTPUT);
  pinMode(motor2Pin2, OUTPUT);
  pinMode(motor3Pin1, OUTPUT);
  pinMode(motor3Pin2, OUTPUT);
  pinMode(motor4Pin1, OUTPUT);
  pinMode(motor4Pin2, OUTPUT);

  // Setup Bluepad32 with connection and disconnection callbacks
  BP32.setup(&onConnectedGamepad, &onDisconnectedGamepad);

  // Allow allocation of all timers
  ESP32PWM::allocateTimer(0);
  ESP32PWM::allocateTimer(1);
  ESP32PWM::allocateTimer(2);
  ESP32PWM::allocateTimer(3);

  // Attach servos to Left Arm pins
  servoBaseLeft.attach(servoBaseLeftPin, minUs, maxUs);
  servoShoulderLeft.attach(servoShoulderLeftPin, minUs, maxUs);
  servoElbowLeft.attach(servoElbowLeftPin, minUs, maxUs);
  servoWristRollLeft.attach(servoWristRollLeftPin, minUs, maxUs);
  servoWristPitchLeft.attach(servoWristPitchLeftPin, minUs, maxUs);
  servoGripLeft.attach(servoGripLeftPin, minUs, maxUs);

  // Attach servos to Right Arm pins
  servoBaseRight.attach(servoBaseRightPin, minUs, maxUs);
  servoShoulderRight.attach(servoShoulderRightPin, minUs, maxUs);
  servoElbowRight.attach(servoElbowRightPin, minUs, maxUs);
  servoWristRollRight.attach(servoWristRollRightPin, minUs, maxUs);
  servoWristPitchRight.attach(servoWristPitchRightPin, minUs, maxUs);
  servoGripRight.attach(servoGripRightPin, minUs, maxUs);

  // Initialize servos for both arms
  resetServos();

  // Set rover control pins as outputs
  pinMode(roverForwardPin, OUTPUT);
  pinMode(roverBackwardPin, OUTPUT);
  pinMode(roverLeftPin, OUTPUT);
  pinMode(roverRightPin, OUTPUT);
}

void loop() {
  BP32.update();  // Update Bluepad32 state

  if (myGamepad) {
    checkModeChange();  // Check for mode switch
    if (currentMode == ROVER) {
      controlRover();
    } else {
      controlServos();
    }
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

// Function to control the rover using gamepad inputs
void controlRover() {
  int leftX = myGamepad->axisX();  // Left joystick X-axis for left/right movement
  int leftY = myGamepad->axisY();  // Left joystick Y-axis for forward/backward movement

  // Forward and backward movement
  if (abs(leftY) > joystickThreshold) {
    if (leftY < 0) {
      moveForward();
    } else {
      moveBackward();
    }
  } else if (abs(leftX) > joystickThreshold) {
    // Check for side-to-side movement
    if (leftX < 0) {
      moveLeft();
    } else {
      moveRight();
    }
  } else {
    stopMovement();
  }

  // Turning
  if (abs(leftX) > joystickThreshold && abs(leftY) < joystickThreshold) {
    if (leftX < 0) {
      turnLeft();   // Turn left when joystick is moved left
    } else {
      turnRight();  // Turn right when joystick is moved right
    }
  }

  // Diagonal movements
  if (abs(leftY) > joystickThreshold && abs(leftX) > joystickThreshold) {
    if (leftY < 0 && leftX < 0) {
      moveForwardLeft();
    } else if (leftY < 0 && leftX > 0) {
      moveForwardRight();
    } else if (leftY > 0 && leftX < 0) {
      moveBackwardLeft();
    } else if (leftY > 0 && leftX > 0) {
      moveBackwardRight();
    }
  }
}

// Rover movement functions
void moveForward() {
  // Move all motors forward
  digitalWrite(motor1Pin1, HIGH);
  digitalWrite(motor1Pin2, LOW);
  digitalWrite(motor2Pin1, HIGH);
  digitalWrite(motor2Pin2, LOW);
  digitalWrite(motor3Pin1, HIGH);
  digitalWrite(motor3Pin2, LOW);
  digitalWrite(motor4Pin1, HIGH);
  digitalWrite(motor4Pin2, LOW);
}

void moveBackward() {
  // Move all motors backward
  digitalWrite(motor1Pin1, LOW);
  digitalWrite(motor1Pin2, HIGH);
  digitalWrite(motor2Pin1, LOW);
  digitalWrite(motor2Pin2, HIGH);
  digitalWrite(motor3Pin1, LOW);
  digitalWrite(motor3Pin2, HIGH);
  digitalWrite(motor4Pin1, LOW);
  digitalWrite(motor4Pin2, HIGH);
}

void moveLeft() {
  // Move left by adjusting motor directions
  digitalWrite(motor1Pin1, LOW);
  digitalWrite(motor1Pin2, HIGH);
  digitalWrite(motor2Pin1, HIGH);
  digitalWrite(motor2Pin2, LOW);
  digitalWrite(motor3Pin1, LOW);
  digitalWrite(motor3Pin2, HIGH);
  digitalWrite(motor4Pin1, HIGH);
  digitalWrite(motor4Pin2, LOW);
}

void moveRight() {
  // Move right by adjusting motor directions
  digitalWrite(motor1Pin1, HIGH);
  digitalWrite(motor1Pin2, LOW);
  digitalWrite(motor2Pin1, LOW);
  digitalWrite(motor2Pin2, HIGH);
  digitalWrite(motor3Pin1, HIGH);
  digitalWrite(motor3Pin2, LOW);
  digitalWrite(motor4Pin1, LOW);
  digitalWrite(motor4Pin2, HIGH);
}

void turnLeft() {
  // Turn left by reversing right motors and driving left motors forward
  digitalWrite(motor1Pin1, HIGH);
  digitalWrite(motor1Pin2, LOW);
  digitalWrite(motor2Pin1, LOW);
  digitalWrite(motor2Pin2, HIGH);
  digitalWrite(motor3Pin1, HIGH);
  digitalWrite(motor3Pin2, LOW);
  digitalWrite(motor4Pin1, LOW);
  digitalWrite(motor4Pin2, HIGH);
}

void turnRight() {
  // Turn right by reversing left motors and driving right motors forward
  digitalWrite(motor1Pin1, LOW);
  digitalWrite(motor1Pin2, HIGH);
  digitalWrite(motor2Pin1, HIGH);
  digitalWrite(motor2Pin2, LOW);
  digitalWrite(motor3Pin1, LOW);
  digitalWrite(motor3Pin2, HIGH);
  digitalWrite(motor4Pin1, HIGH);
  digitalWrite(motor4Pin2, LOW);
}

void moveForwardLeft() {
  // Move diagonally forward-left
  digitalWrite(motor1Pin1, HIGH);
  digitalWrite(motor1Pin2, LOW);
  digitalWrite(motor2Pin1, HIGH);
  digitalWrite(motor2Pin2, LOW);
  digitalWrite(motor3Pin1, HIGH);
  digitalWrite(motor3Pin2, LOW);
  digitalWrite(motor4Pin1, LOW);
  digitalWrite(motor4Pin2, HIGH);
}

void moveForwardRight() {
  // Move diagonally forward-right
  digitalWrite(motor1Pin1, HIGH);
  digitalWrite(motor1Pin2, LOW);
  digitalWrite(motor2Pin1, LOW);
  digitalWrite(motor2Pin2, HIGH);
  digitalWrite(motor3Pin1, LOW);
  digitalWrite(motor3Pin2, HIGH);
  digitalWrite(motor4Pin1, HIGH);
  digitalWrite(motor4Pin2, LOW);
}

void moveBackwardLeft() {
  // Move diagonally backward-left
  digitalWrite(motor1Pin1, LOW);
  digitalWrite(motor1Pin2, HIGH);
  digitalWrite(motor2Pin1, HIGH);
  digitalWrite(motor2Pin2, LOW);
  digitalWrite(motor3Pin1, LOW);
  digitalWrite(motor3Pin2, HIGH);
  digitalWrite(motor4Pin1, HIGH);
  digitalWrite(motor4Pin2, LOW);
}

void moveBackwardRight() {
  // Move diagonally backward-right
  digitalWrite(motor1Pin1, LOW);
  digitalWrite(motor1Pin2, HIGH);
  digitalWrite(motor2Pin1, LOW);
  digitalWrite(motor2Pin2, HIGH);
  digitalWrite(motor3Pin1, HIGH);
  digitalWrite(motor3Pin2, LOW);
  digitalWrite(motor4Pin1, LOW);
  digitalWrite(motor4Pin2, HIGH);
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

// Reset servo positions
void resetServos() {
  servoGripLeft.write(gripAngleLeft);
  servoWristRollLeft.write(wristRollAngleLeft);

  servoGripRight.write(gripAngleRight);
  servoWristRollRight.write(wristRollAngleRight);
}

// Switch between different control modes
void checkModeChange() {
  uint16_t buttons = myGamepad->buttons();  // Read the button values

  // X button (Left arm)
  if (buttons & 0x0001) {  // X button pressed
    currentMode = LEFT_ARM;
    Serial.println("Control mode: LEFT ARM");
  }
  // O button (Right arm)
  else if (buttons & 0x0002) {  // O button pressed
    currentMode = RIGHT_ARM;
    Serial.println("Control mode: RIGHT ARM");
  }
  // L1 button (Both arms)
  else if (buttons & 0x0010) {  // L1 button pressed
    currentMode = BOTH_ARMS;
    Serial.println("Control mode: BOTH ARMS");
  }
  // R2 button (Rover mode)
  else if (buttons & 0x0020) {  // R2 button pressed
    currentMode = ROVER;
    Serial.println("Control mode: ROVER");
  }
}

// Control servos based on the current mode
void controlServos() {
  int leftX = myGamepad->axisX();   // Base rotation
  int leftY = myGamepad->axisY();   // Shoulder movement
  int rightX = myGamepad->axisRX(); // Elbow movement
  int rightY = myGamepad->axisRY(); // Wrist pitch

  switch (currentMode) {
    case LEFT_ARM:
      Serial.println("Controlling LEFT ARM");  // Debugging: check which arm is controlled
      controlLeftArm(leftX, leftY, rightX, rightY);
      break;
    case RIGHT_ARM:
      Serial.println("Controlling RIGHT ARM");  // Debugging: check which arm is controlled
      controlRightArm(leftX, leftY, rightX, rightY);
      break;
    case BOTH_ARMS:
      Serial.println("Controlling BOTH ARMS");  // Debugging: check which arms are controlled
      controlLeftArm(leftX, leftY, rightX, rightY);
      controlRightArm(leftX, leftY, rightX, rightY);
      break;
  }
}

void controlLeftArm(int leftX, int leftY, int rightX, int rightY) {
  // Base movement
  if (leftX < -joystickThreshold && baseAngleLeft < 180) {
    baseAngleLeft++;
    servoBaseLeft.write(baseAngleLeft);
    delay(servoSpeed);
  } else if (leftX > joystickThreshold && baseAngleLeft > 0) {
    baseAngleLeft--;
    servoBaseLeft.write(baseAngleLeft);
    delay(servoSpeed);
  }

  // Shoulder movement
  if (leftY < -joystickThreshold && shoulderAngleLeft < 180) {
    shoulderAngleLeft++;
    servoShoulderLeft.write(shoulderAngleLeft);
    delay(servoSpeed);
  } else if (leftY > joystickThreshold && shoulderAngleLeft > 0) {
    shoulderAngleLeft--;
    servoShoulderLeft.write(shoulderAngleLeft);
    delay(servoSpeed);
  }

  // Elbow movement
  if (rightX < -joystickThreshold && elbowAngleLeft < 180) {
    elbowAngleLeft++;
    servoElbowLeft.write(elbowAngleLeft);
    delay(servoSpeed);
  } else if (rightX > joystickThreshold && elbowAngleLeft > 0) {
    elbowAngleLeft--;
    servoElbowLeft.write(elbowAngleLeft);
    delay(servoSpeed);
  }

  // Wrist pitch movement
  if (rightY < -joystickThreshold && wristPitchAngleLeft < 180) {
    wristPitchAngleLeft++;
    servoWristPitchLeft.write(wristPitchAngleLeft);
    delay(servoSpeed);
  } else if (rightY > joystickThreshold && wristPitchAngleLeft > 0) {
    wristPitchAngleLeft--;
    servoWristPitchLeft.write(wristPitchAngleLeft);
    delay(servoSpeed);
  }

  // Wrist roll control (D-Pad left/right)
  uint16_t buttons = myGamepad->buttons();  // Read the button values
  if (buttons & 0x04) {  // D-Pad right pressed
    wristRollAngleLeft++;
    servoWristRollLeft.write(wristRollAngleLeft);
    delay(servoSpeed);
  } else if (buttons & 0x08) {  // D-Pad left pressed
    wristRollAngleLeft--;
    servoWristRollLeft.write(wristRollAngleLeft);
    delay(servoSpeed);
  }

  // Grip control (D-Pad down)
  if (buttons & 0x02) {  // D-Pad down pressed
    gripAngleLeft = gripAngleLeft == 90 ? 0 : 90;  // Toggle grip
    servoGripLeft.write(gripAngleLeft);
    delay(servoSpeed);
  }
}

void controlRightArm(int leftX, int leftY, int rightX, int rightY) {
  // Base movement
  if (leftX < -joystickThreshold && baseAngleRight < 180) {
    baseAngleRight++;
    servoBaseRight.write(baseAngleRight);
    delay(servoSpeed);
  } else if (leftX > joystickThreshold && baseAngleRight > 0) {
    baseAngleRight--;
    servoBaseRight.write(baseAngleRight);
    delay(servoSpeed);
  }

  // Shoulder movement
  if (leftY < -joystickThreshold && shoulderAngleRight < 180) {
    shoulderAngleRight++;
    servoShoulderRight.write(shoulderAngleRight);
    delay(servoSpeed);
  } else if (leftY > joystickThreshold && shoulderAngleRight > 0) {
    shoulderAngleRight--;
    servoShoulderRight.write(shoulderAngleRight);
    delay(servoSpeed);
  }

  // Elbow movement
  if (rightX < -joystickThreshold && elbowAngleRight < 180) {
    elbowAngleRight++;
    servoElbowRight.write(elbowAngleRight);
    delay(servoSpeed);
  } else if (rightX > joystickThreshold && elbowAngleRight > 0) {
    elbowAngleRight--;
    servoElbowRight.write(elbowAngleRight);
    delay(servoSpeed);
  }

  // Wrist pitch movement
  if (rightY < -joystickThreshold && wristPitchAngleRight < 180) {
    wristPitchAngleRight++;
    servoWristPitchRight.write(wristPitchAngleRight);
    delay(servoSpeed);
  } else if (rightY > joystickThreshold && wristPitchAngleRight > 0) {
    wristPitchAngleRight--;
    servoWristPitchRight.write(wristPitchAngleRight);
    delay(servoSpeed);
  }

  // Wrist roll control (D-Pad left/right)
  uint16_t buttons = myGamepad->buttons();  // Read the button values
Serial.print("Button state: ");
Serial.println(buttons, HEX);

  if (buttons & 0x04) {  // D-Pad right pressed
    wristRollAngleRight++;
    servoWristRollRight.write(wristRollAngleRight);
    Serial.println("D-Pad Right pressed - Increasing Left Wrist Roll");
    delay(servoSpeed);
  } else if (buttons & 0x08) {  // D-Pad left pressed
    wristRollAngleRight--;
    servoWristRollRight.write(wristRollAngleRight);
    Serial.println("D-Pad Left pressed - Decreasing Left Wrist Roll");
    delay(servoSpeed);
  }

  // Grip control (D-Pad down)
  if (buttons & 0x02) {  // D-Pad down pressed
    gripAngleRight = gripAngleRight == 90 ? 0 : 90;  // Toggle grip
    servoGripRight.write(gripAngleRight);
    delay(servoSpeed);
  }
}


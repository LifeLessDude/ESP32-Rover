#include <Bluepad32.h>
#include <ESP32Servo.h>
#include <Adafruit_PWMServoDriver.h>
#include <Wire.h>

Adafruit_PWMServoDriver pca9685 = Adafruit_PWMServoDriver(0x40);

//Please modify according to preference and Expansion board of ESP32.
//Modify PCA9685 Pins accordingly.
//Configured for MG996R and MG958 Servo motors

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

#define numServo 12
#define SERVOMIN 800  // Minimum pulse width value
#define SERVOMAX 2150  // Maximum pulse width value

// Define servo motor connections (expand as required)
#define SER_LB 0 //Servo Motor 1 on connector 0, LeftBase
#define SER_LS 1 //Servo Motor 2 on connector 1, LeftShoulder
#define SER_LE 2 //Servo Motor 3 on connector 2, LeftElbow
#define SER_LW 3  //Servo Motor 4 on connector 3, LeftWrist
#define SER_LR 4 //Servo Motor 5 on connector 4, LeftRoll
#define SER_LC 5  //Servo Motor 6 on connector 5, LeftClaw
#define SER_RB 6  //Servo Motor 7 on connector 6, RightBase
#define SER_RS 7  //Servo Motor 8 on connector 7, RightShoulder
#define SER_RE 8  //Servo Motor 9 on connector 8, RightElbow
#define SER_RW 9  //Servo Motor 10 on connector 9, RightWrist
#define SER_RR 10  //Servo Motor 11 on connector 10, RightRoll
#define SER_RC 11 //Servo Motor 12 on connector 11, RightClaw

// Define rover control pins
const int roverForwardPin = 21;  // Example pin, change as needed
const int roverBackwardPin = 22; // Example pin, change as needed
const int roverLeftPin = 23;     // Example pin, change as needed
const int roverRightPin = 24;    // Example pin, change as needed

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

//Define Gamepad
GamepadPtr myGamepad;

void setup() 
{
  Serial.begin(115200);
  //Initialize PCA9685
  pca9685.begin();
  //Set PCA9685 Frequency to 50Hz
  pca9685.setPWMFreq(50);

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
  
  // Initialize servos for both arms
  resetServos();

  // Set rover control pins as outputs
  pinMode(roverForwardPin, OUTPUT);
  pinMode(roverBackwardPin, OUTPUT);
  pinMode(roverLeftPin, OUTPUT);
  pinMode(roverRightPin, OUTPUT);
}

void loop() 
{
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

void onConnectedGamepad(GamepadPtr gp) 
{
  myGamepad = gp;
  Serial.println("Gamepad connected");
}

void onDisconnectedGamepad(GamepadPtr gp) 
{
  myGamepad = nullptr;
  Serial.println("Gamepad disconnected");
}

// Function to control the rover using gamepad inputs
void controlRover() 
{
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
void moveForward() 
{
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

void moveBackward() 
{
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

void moveLeft() 
{
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

void moveRight() 
{
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

void turnLeft() 
{
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

void turnRight() 
{
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

void moveForwardLeft() 
{
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

void moveForwardRight() 
{
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

void moveBackwardLeft() 
{
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

void moveBackwardRight() 
{
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

void stopMovement() 
{
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

int angleToPulse(int angle)
{
  return map(angle, 0, 180, SERVOMIN, SERVOMAX); 
}

// Reset servo positions
void resetServos() 
{
  int pwm = angleToPulse(90);
  pca9685.setPWM(SER_LB, 0, pwm);
  pca9685.setPWM(SER_LS, 0, pwm);
  pca9685.setPWM(SER_LE, 0, pwm);
  pca9685.setPWM(SER_LW, 0, pwm);
  pca9685.setPWM(SER_LR, 0, pwm);
  pca9685.setPWM(SER_LC, 0, pwm);
  pca9685.setPWM(SER_RB, 0, pwm);
  pca9685.setPWM(SER_RS, 0, pwm);
  pca9685.setPWM(SER_RE, 0, pwm);
  pca9685.setPWM(SER_RW, 0, pwm);
  pca9685.setPWM(SER_RR, 0, pwm);
  pca9685.setPWM(SER_RC, 0, pwm);
}

// Switch between different control modes
void checkModeChange() 
{
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
void controlServos() 
{
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

void controlLeftArm(int leftX, int leftY, int rightX, int rightY) 
{
  // Base movement
  if (leftX < -joystickThreshold && baseAngleLeft < 180) {
    baseAngleLeft++;
    pca9685.setPWM(SER_LB, 0, angleToPulse(baseAngleLeft))
    delay(servoSpeed);
  } else if (leftX > joystickThreshold && baseAngleLeft > 0) {
    baseAngleLeft--;
    pca9685.setPWM(SER_LB, 0, angleToPulse(baseAngleLeft));
    delay(servoSpeed);
  }

  // Shoulder movement
  if (leftY < -joystickThreshold && shoulderAngleLeft < 180) {
    shoulderAngleLeft++;
    pca9685.setPWM(SER_LS, 0, angleToPulse(shoulderAngleLeft));
    delay(servoSpeed);
  } else if (leftY > joystickThreshold && shoulderAngleLeft > 0) {
    shoulderAngleLeft--;
    pca9685.setPWM(SER_LS, 0, angleToPulse(shoulderAngleLeft));
    delay(servoSpeed);
  }

  // Elbow movement
  if (rightX < -joystickThreshold && elbowAngleLeft < 180) {
    elbowAngleLeft++;
    pca9685.setPWM(SER_LE, 0, angleToPulse(elbowAngleLeft));
    delay(servoSpeed);
  } else if (rightX > joystickThreshold && elbowAngleLeft > 0) {
    elbowAngleLeft--;
    pca9685.setPWM(SER_LE, 0, angleToPulse(elbowAngleLeft));
    delay(servoSpeed);
  }

  // Wrist pitch movement
  if (rightY < -joystickThreshold && wristPitchAngleLeft < 180) {
    wristPitchAngleLeft++;
    pca9685.setPWM(SER_LW, 0, angleToPulse(wristPitchAngleLeft));
    delay(servoSpeed);
  } else if (rightY > joystickThreshold && wristPitchAngleLeft > 0) {
    wristPitchAngleLeft--;
    pca9685.setPWM(SER_LW, 0, angleToPulse(wristPitchAngleLeft));
    delay(servoSpeed);
  }

  // Wrist roll control (D-Pad left/right)
  uint16_t buttons = myGamepad->buttons();  // Read the button values
  if (buttons & 0x04) {  // D-Pad right pressed
    wristRollAngleLeft++;
    pca9685.setPWM(SER_LR, 0, angleToPulse(wristRollAngleLeft));
    delay(servoSpeed);
  } else if (buttons & 0x08) {  // D-Pad left pressed
    wristRollAngleLeft--;
    pca9685.setPWM(SER_LR, 0, angleToPulse(wristRollAngleLeft));
    delay(servoSpeed);
  }

  // Grip control (D-Pad down)
  if (buttons & 0x02) {  // D-Pad down pressed
    gripAngleLeft = gripAngleLeft == 90 ? 0 : 90;  // Toggle grip
    pca9685.setPWM(SER_LC, 0, angleToPulse(gripAngleLeft));
    delay(servoSpeed);
  }
}

void controlRightArm(int leftX, int leftY, int rightX, int rightY) 
{
  // Base movement
  if (leftX < -joystickThreshold && baseAngleRight < 180) {
    baseAngleRight++;
    pca9685.setPWM(SER_RB, 0, angleToPulse(baseAngleRight));
    delay(servoSpeed);
  } else if (leftX > joystickThreshold && baseAngleRight > 0) {
    baseAngleRight--;
    pca9685.setPWM(SER_RB, 0, angleToPulse(baseAngleRight));
    delay(servoSpeed);
  }

  // Shoulder movement
  if (leftY < -joystickThreshold && shoulderAngleRight < 180) {
    shoulderAngleRight++;
    pca9685.setPWM(SER_RS, 0, angleToPulse(shoulderAngleRight));
    delay(servoSpeed);
  } else if (leftY > joystickThreshold && shoulderAngleRight > 0) {
    shoulderAngleRight--;
    pca9685.setPWM(SER_RS, 0, angleToPulse(shoulderAngleRight));
    delay(servoSpeed);
  }

  // Elbow movement
  if (rightX < -joystickThreshold && elbowAngleRight < 180) {
    elbowAngleRight++;
    pca9685.setPWM(SER_RE, 0, angleToPulse(elbowAngleRight));
    delay(servoSpeed);
  } else if (rightX > joystickThreshold && elbowAngleRight > 0) {
    elbowAngleRight--;
    pca9685.setPWM(SER_RE, 0, angleToPulse(elbowAngleRight));
    delay(servoSpeed);
  }

  // Wrist pitch movement
  if (rightY < -joystickThreshold && wristPitchAngleRight < 180) {
    wristPitchAngleRight++;
    pca9685.setPWM(SER_RW, 0, angleToPulse(wristPitchAngleRight));
    delay(servoSpeed);
  } else if (rightY > joystickThreshold && wristPitchAngleRight > 0) {
    wristPitchAngleRight--;
    pca9685.setPWM(SER_RW, 0, angleToPulse(wristPitchAngleRight));
    delay(servoSpeed);
  }

  // Wrist roll control (D-Pad left/right)
  uint16_t buttons = myGamepad->buttons();  // Read the button values
  Serial.print("Button state: ");
  Serial.println(buttons, HEX);

  if (buttons & 0x04) {  // D-Pad right pressed
    wristRollAngleRight++;
    pca9685.setPWM(SER_RR, 0, angleToPulse(wristRollAngleRight));
    Serial.println("D-Pad Right pressed - Increasing Left Wrist Roll");
    delay(servoSpeed);
  } else if (buttons & 0x08) {  // D-Pad left pressed
    wristRollAngleRight--;
    pca9685.setPWM(SER_RR, 0, angleToPulse(wristRollAngleRight));
    Serial.println("D-Pad Left pressed - Decreasing Left Wrist Roll");
    delay(servoSpeed);
  }

  // Grip control (D-Pad down)
  if (buttons & 0x02) {  // D-Pad down pressed
    gripAngleRight = gripAngleRight == 90 ? 0 : 90;  // Toggle grip
    pca9685.setPWM(SER_RC, 0, angleToPulse(gripAngleRight));
    delay(servoSpeed);
  }
}


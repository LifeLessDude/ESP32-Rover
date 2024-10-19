#include <Bluepad32.h>
#include <ESP32Servo.h>
#include <Adafruit_PWMServoDriver.h>
#include <Wire.h>

Adafruit_PWMServoDriver pca9685 = Adafruit_PWMServoDriver(0x40);

int joystickThreshold = 15;  // Deadzone for joystick input

#define SERVOMIN 900  // Minimum pulse width value
#define SERVOMAX 2050  // Maximum pulse width value

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

enum ControlMode { LEFT_ARM, RIGHT_ARM, BOTH_ARMS};
ControlMode currentMode = BOTH_ARMS;  // Start in both arms mode

int servoSpeed = 20;

GamepadPtr myGamepad;

void setup()
{
  Serial.begin(115200);

  pca9685.begin();
  pca9685.setPWMFreq(50);

  BP32.setup(&onConnectedGamepad, &onDisconnectedGamepad);

  // Allow allocation of all timers
  ESP32PWM::allocateTimer(0);
  ESP32PWM::allocateTimer(1);
  ESP32PWM::allocateTimer(2);
  ESP32PWM::allocateTimer(3);

   // Initialize servos for both arms
  resetServos();

  if (myGamepad) {
    checkModeChange();  // Check for mode switch
    if (currentMode == LEFT_ARM || RIGHT_ARM || BOTH_ARMS) {
      controlServos();
    } else {
      Serial.println("Not found!");
    }
  }
}

void loop(){
  BP32.update();
}

void onConnectedGamepad(GamepadPtr gp) {
  myGamepad = gp;
  Serial.println("Gamepad connected");
}

void onDisconnectedGamepad(GamepadPtr gp) {
  myGamepad = nullptr;
  Serial.println("Gamepad disconnected");
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
  else {  // R2 button pressed
    Serial.println("Not Found");
  }
}


// Reset servo positions
void resetServos() {
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

int angleToPulse(int angle)
{
  return map(angle, 0, 180, SERVOMIN, SERVOMAX); 
}

void controlLeftArm(int leftX, int leftY, int rightX, int rightY) 
{
  // Base movement
  if (leftX < -joystickThreshold && baseAngleLeft < 180) {
    baseAngleLeft++;
    pca9685.setPWM(SER_LB, 0, angleToPulse(baseAngleLeft));
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
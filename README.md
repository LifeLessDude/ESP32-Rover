# ESP32-Rover
Code for the logic of a 2-Armed Rover using the ESP32 microcontroller and controlled by a PS3 Controller.
Uses the servo driver PCA9685 along with the MG996R and MG958 servo motors.
Uses libraries of Bluepad32, ESP32Servo and Adafruit_PWMServoDriver.

The arms use 6 servos each - 1 for the base, 3 for the joints, 1 for the Roll axis, and 1 for the claw which holds objects.

The motor used for the wheels are Johnson DC gear motors. The wheels of the rover are in a 4-wheel drive formation using Mecanum wheels for omnidirectional movement.


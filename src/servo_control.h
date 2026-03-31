#ifndef SERVO_CONTROL_H
#define SERVO_CONTROL_H

#include <Arduino.h>
#include <Servo.h>

#define SERVO_PIN 9  // change to whatever pin you use

void initServo();
void servoGoTo(int angle);  // 0-180 degrees

#endif

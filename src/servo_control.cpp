#include "servo_control.h"

static Servo myServo;

void initServo() {
    myServo.attach(SERVO_PIN);
    myServo.write(0);  // start position
}

void servoGoTo(int angle) {
    angle = constrain(angle, 0, 180);
    myServo.write(angle);
}

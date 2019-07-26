
#include "Arduino.h"
#include "Motors.h"
#include "Constants.h"
#include <Servo.h>

Motors::Motors(int leftServoPin, int rightServoPin, int leftBLDCPin, int rightBLDCPin, int leftEncoderPin, int rightEncoderPin) {
    _leftServoPin = leftServoPin;
    _rightServoPin = rightServoPin;
    _leftBLDCPin = leftBLDCPin;
    _rightBLDCPin = rightBLDCPin;
    _leftEncoderPin = leftEncoderPin;
    _rightEncoderPin = rightEncoderPin;
}

void Motors::attachMotors() { //Servo.attach needs to be called in a function outside of the constructor https://forum.arduino.cc/index.php?topic=62854.0
    leftServo.attach(_leftServoPin);
    rightServo.attach(_rightServoPin);
    leftBLDC.attach(_leftBLDCPin);
    rightBLDC.attach(_rightBLDCPin);
}

void Motors::writeMotors(int u1, int u2,  int u3, int u4) {
    leftServo.write(max(min(u1 - 90 + LEFT_ANGLE_VERTICAL, SERVO_MAX),SERVO_MIN));
    rightServo.write(max(min(-u2 + 90 + RIGHT_ANGLE_VERTICAL, SERVO_MAX),SERVO_MIN));
    leftBLDC.writeMicroseconds(max(min(u3 + PWM_MIN, PWM_MAX),PWM_MIN));
    rightBLDC.writeMicroseconds(max(min(u4 + PWM_MIN, PWM_MAX),PWM_MIN));
}

int Motors::get_y1() {
    return map(analogRead(_leftEncoderPin),LEFT_ENCODER_MIN, LEFT_ENCODER_MAX, -45, 135);
}

int Motors::get_y2() {
    return map(analogRead(_rightEncoderPin), RIGHT_ENCODER_MIN, RIGHT_ENCODER_MAX, 135, -45);
}


#include "Motors.h"
#include "Constants.h"

Motors::Motors(int leftServoPin, int rightServoPin, int leftBLDCPin, int rightBLDCPin, int leftEncoderPin, int rightEncoderPin) {
    this->leftServoPin = leftServoPin;
    this->rightServoPin = rightServoPin;
    this->leftBLDCPin = leftBLDCPin;
    this->rightBLDCPin = rightBLDCPin;
    this->leftEncoderPin = leftEncoderPin;
    this->rightEncoderPin = rightEncoderPin;
}

void Motors::attachMotors() { //Servo.attach needs to be called in a function outside of the constructor https://forum.arduino.cc/index.php?topic=62854.0
    leftServo.attach(leftServoPin);
    rightServo.attach(rightServoPin);
    leftBLDC.attach(leftBLDCPin);
    rightBLDC.attach(rightBLDCPin);
    writeMotors(90,90,0,0);
}

void Motors::writeMotors(int u1, int u2,  int u3, int u4) {
    int leftServoVal = max(min(u1 - 90 + LEFT_ANGLE_VERTICAL, SERVO_MAX),SERVO_MIN);
    int rightServoVal = max(min(-u2 + 90 + RIGHT_ANGLE_VERTICAL, SERVO_MAX),SERVO_MIN);
    int leftBLDCVal = max(min(map(u3, SPEED_MIN, SPEED_MAX, PWM_MIN, PWM_MAX), PWM_MAX),PWM_MIN);
    int rightBLDCVal = max(min(map(u4, SPEED_MIN, SPEED_MAX, PWM_MIN, PWM_MAX), PWM_MAX),PWM_MIN);

    leftServo.write(leftServoVal);
    rightServo.write(rightServoVal);
    leftBLDC.write(leftBLDCVal);
    rightBLDC.write(rightBLDCVal);
}

int Motors::get_y1() {
    return map(analogRead(leftEncoderPin),LEFT_ENCODER_MIN, LEFT_ENCODER_MAX, -45, 135);
}

int Motors::get_y2() {
    return map(analogRead(rightEncoderPin), RIGHT_ENCODER_MIN, RIGHT_ENCODER_MAX, 135, -45);
}

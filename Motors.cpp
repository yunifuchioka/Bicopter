
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
    writeMotors(0,0,0,0);
}

void Motors::writeMotors(int u1, int u2,  int u3, int u4) {
    int leftServoVal = constrain(u1 + LEFT_ANGLE_VERTICAL, SERVO_MIN, SERVO_MAX);
    int rightServoVal = constrain(-u2 + RIGHT_ANGLE_VERTICAL, SERVO_MIN, SERVO_MAX);
    int leftBLDCVal = constrain(map(u3, SPEED_MIN, SPEED_MAX, PWM_MIN, PWM_MAX), PWM_MIN,PWM_MAX);
    int rightBLDCVal = constrain(map(u4, SPEED_MIN, SPEED_MAX, PWM_MIN, PWM_MAX), PWM_MIN,PWM_MAX);

    leftServo.write(leftServoVal);
    rightServo.write(rightServoVal);
    leftBLDC.write(leftBLDCVal);
    rightBLDC.write(rightBLDCVal);

    /*
    //uncomment for encoder calibration
    Serial.print(u1);
    Serial.print('\t');
    Serial.print(analogRead(leftEncoderPin));
    Serial.print('\t');
    Serial.print(getLeftAngle());
    Serial.print('\t');
    Serial.print(u2);
    Serial.print('\t');
    Serial.print(analogRead(rightEncoderPin));
    Serial.print('\t');
    Serial.print(getRightAngle());
    Serial.print('\n');
    */
}

int Motors::getLeftAngle() {
    return map(analogRead(leftEncoderPin),LEFT_ENCODER_MIN, LEFT_ENCODER_MAX, -135, 45);
}

int Motors::getRightAngle() {
    return map(analogRead(rightEncoderPin), RIGHT_ENCODER_MIN, RIGHT_ENCODER_MAX, 45, -135);
}

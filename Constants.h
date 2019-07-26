
#ifndef CONSTANTS_H
#define CONSTANTS_H

#include "Arduino.h"

//digital pins
#define LEFT_SERVO_PIN 9
#define LEFT_BLDC_PIN 10
#define RIGHT_SERVO_PIN 11
#define RIGHT_BLDC_PIN 12

//analog pins
#define LEFT_ENCODER_PIN A0
#define RIGHT_ENCODER_PIN A1
#define SDA_PIN A4
#define SCL_PIN A5

//physical calibration parameters for motors
#define LEFT_ANGLE_VERTICAL 130//value of the servo angle that results in the nacelle being vertical
#define RIGHT_ANGLE_VERTICAL 39
#define LEFT_ENCODER_MIN 109 //the analog value of the feedback wire when u1 = -45
#define LEFT_ENCODER_MAX 464 //the analog value of the feedback wire when u1 = 135
#define RIGHT_ENCODER_MIN 115 //the analog value of the feedback wire when u2 = 135
#define RIGHT_ENCODER_MAX 472 //the analog value of the feedback wire when u2 = -45

#define PWM_MIN 1000 //min pulse width for PWM for servo signals
#define PWM_MAX 2000 //max pulse width for PWM for servo signals
#define SERVO_MIN 0 //min angle for servo signals
#define SERVO_MAX 180 //max angle for servo signals

#endif

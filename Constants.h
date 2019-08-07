
#ifndef CONSTANTS_H
#define CONSTANTS_H

//digital pins
#define SERIAL_RX_PIN 0
#define INTERRUPT_PIN 2
#define LEFT_SERVO_PIN 4
#define LEFT_BLDC_PIN 5
#define RIGHT_SERVO_PIN 6
#define RIGHT_BLDC_PIN 7
#define LED_PIN 13

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
#define SPEED_MIN 0 //minimum value used for speed signals
#define SPEED_MAX 1000 //maximum value used for speed signals
#define SERVO_MIN 0 //min angle for servo signals
#define SERVO_MAX 180 //max angle for servo signals

/*
 * Watchdog timer https://folk.uio.no/jeanra/Microelectronics/ArduinoWatchdog.html
 * 
 * Watchdog timer value table
 * 15mS                           WDTO_15MS
 * 30mS                           WDTO_30MS
 * 60mS                           WDTO_60MS
 * 120mS                          WDTO_120MS
 * 250mS                          WDTO_250MS
 * 500mS                          WDTO_500MS
 * 1S                             WDTO_1S            
 * 2S                             WDTO_2S
 * 4S                             WDTO_4S
 * 8S                             WDTO_8S
 */
#define WATCHDOG_TIME WDTO_1S

#endif

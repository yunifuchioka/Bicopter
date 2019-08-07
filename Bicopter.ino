
#include <avr/wdt.h> //watchdog
#include "Constants.h"
#include "Motors.h"
#include "IMU.h" //requires I2Cdev https://github.com/jrowberg/i2cdevlib
#include "RC.h" //requires FlySkyIBus https://github.com/jrowberg/i2cdevlib

Motors motors(LEFT_SERVO_PIN, RIGHT_SERVO_PIN, LEFT_BLDC_PIN, RIGHT_BLDC_PIN, LEFT_ENCODER_PIN, RIGHT_ENCODER_PIN);
IMU imu(SDA_PIN, SCL_PIN, INTERRUPT_PIN);
RC rc;

//Interrupt service routine (ISR) triggered by IMU
volatile bool imuInterrupt = false; // indicates whether MPU interrupt pin has gone high
void imuInterruptISR() {
    imuInterrupt = true;
}

void setup() {
    Serial.begin(115200);
    pinMode(INTERRUPT_PIN, INPUT);
    attachInterrupt(digitalPinToInterrupt(INTERRUPT_PIN), imuInterruptISR, RISING);
    pinMode(LED_PIN, OUTPUT);

    motors.attachMotors();
    rc.begin(Serial);
    if (!imu.initialize()) {
        while (true) { Serial.println("dmp initialization failure"); }
    }
    
    digitalWrite(LED_PIN, HIGH);
    wdt_enable(WATCHDOG_TIME); //initialize watchdog. This needs to happen at the end since imu initialization takes a while
}

void loop() {
    wdt_reset(); //reset watchdog
    rc.loop();
    if (imuInterrupt) {
        imuInterrupt = false;
        imu.update();
    }

    int yawApplied = 0;
    int pitchApplied = 0;
    int rollApplied = 0;

    if (rc.getSWD()) { //if switch D is activated, apply feedback control
        //target orientation
        int yawDes = 0;
        int pitchDes = 0;
        int rollDes = 0;
        int yawDotDes = 0;
        int pitchDotDes = 0;
        int rollDotDes = 0;

        //IMU readings
        int yaw = imu.read().yaw;
        int pitch = imu.read().pitch;
        int roll = imu.read().roll;
        int yawDot = 0; //TODO: sync these with gyro measurements
        int pitchDot = 0;
        int rollDot = 0;

        //PD feedback control parameters
        int kpYaw = 0;
        int kdYaw = 0;
        int kpPitch = 2;
        int kdPitch = 0;
        int kpRoll = 2;
        int kdRoll = 0;

        //PD feedback control
        yawApplied = kpYaw*(yawDes-yaw) + kdYaw*(yawDotDes-yawDot);
        pitchApplied = kpPitch*(pitchDes-pitch) + kdPitch*(pitchDotDes-pitchDot);
        rollApplied = kpRoll*(rollDes-roll) + kdRoll*(rollDotDes-rollDot);
    }
    else { //if switch D is not activated, motors are operated manually
        //set sensitivity of servo angles based on position of variable resistors
        double yawSensitivity = rc.getVRA()/1000.0;
        //double pitchSensitivity = rc.getVRB()/1000.0;
        double pitchSensitivity = 1.0;
        double rollSensitivity = rc.getVRB()/1000.0;

        yawApplied = (int) map(rc.getRightHor(), SPEED_MIN, SPEED_MAX, -45, 45)*yawSensitivity;
        pitchApplied = (int) map(rc.getRightVer(), SPEED_MIN, SPEED_MAX, 45, -45)*pitchSensitivity;
        rollApplied = map(rc.getLeftHor(), SPEED_MIN, SPEED_MAX, -300, 300)*rollSensitivity;
    }

    int BLDCSpeed = rc.getLeftVer();

    int u1 = 90 + pitchApplied - yawApplied;
    int u2 = 90 + pitchApplied + yawApplied;
    int u3 = rc.getSWA() ? BLDCSpeed + rollApplied : 0; //if switch A is turned off, set BLDC speed to 0
    int u4 = rc.getSWA() ? BLDCSpeed - rollApplied : 0;

    motors.writeMotors(u1, u2, u3, u4);
}

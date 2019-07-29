
#include <avr/wdt.h> //watchdog
#include "Constants.h"
#include "Motors.h"
#include "IMU.h" //requires I2Cdev https://github.com/jrowberg/i2cdevlib
#include "FlySkyIBus.h" //https://gitlab.com/timwilkinson/FlySkyIBus

Motors motors(LEFT_SERVO_PIN, RIGHT_SERVO_PIN, LEFT_BLDC_PIN, RIGHT_BLDC_PIN, LEFT_ENCODER_PIN, RIGHT_ENCODER_PIN);
IMU imu(SDA_PIN, SCL_PIN, INTERRUPT_PIN);

//Interrupt service routine (ISR) triggered by IMU
volatile bool imuInterrupt = false; // indicates whether MPU interrupt pin has gone high
void imuInterruptISR() {
    imuInterrupt = true;
}

void setup() {
    Serial.begin(115200);
    pinMode(INTERRUPT_PIN, INPUT);
    attachInterrupt(digitalPinToInterrupt(INTERRUPT_PIN), imuInterruptISR, RISING);

    motors.attachMotors();
    IBus.begin(Serial);
    if (!imu.initialize()) {
        while (true) { Serial.println("dmp initialization failure"); }
    }
    
    wdt_enable(WATCHDOG_TIME); //initialize watchdog. This needs to happen at the end since imu initialization takes a while
}

void loop() {
    wdt_reset(); //reset watchdog
    IBus.loop();
    if (imuInterrupt) {
        imuInterrupt = false;
        imu.update();
    }

    /*
    Serial.print(imu.read().yaw);
    Serial.print('\t');
    Serial.print(imu.read().pitch);
    Serial.print('\t');
    Serial.print(imu.read().roll);
    Serial.print('\n');
    */

    bool manualControl = (IBus.readChannel(6)-PWM_MIN > (PWM_MAX-PWM_MIN)/2);

    int yawApplied = 0;
    int pitchApplied = 0;
    int rollApplied = 0;

    if (manualControl) {
        double yawSensitivity = (IBus.readChannel(4)-PWM_MIN)/1000.0;
        //double pitchSensitivity = (IBus.readChannel(5)-PWM_MIN)/1000.0;
        double pitchSensitivity = 1.0;
        double rollSensitivity = (IBus.readChannel(5)-PWM_MIN)/1000.0;
        
        yawApplied = (int) map(IBus.readChannel(0), PWM_MIN, PWM_MAX, -45, 45)*yawSensitivity;
        pitchApplied = (int) map(IBus.readChannel(1), PWM_MIN, PWM_MAX, 45, -45)*pitchSensitivity;
        rollApplied = map(IBus.readChannel(3), PWM_MIN, PWM_MAX, -300, 300)*rollSensitivity;
    }
    else {
        int yawDes = 0;
        int pitchDes = 0;
        int rollDes = 0;
        int yawDotDes = 0;
        int pitchDotDes = 0;
        int rollDotDes = 0;
        
        int yaw = imu.read().yaw;
        int pitch = imu.read().pitch;
        int roll = imu.read().roll;
        int yawDot = 0; //TODO: sync these with gyro measurements
        int pitchDot = 0;
        int rollDot = 0;

        int kpYaw = 1;
        int kdYaw = 0;
        int kpPitch = 1;
        int kdPitch = 0;
        int kpRoll = 5;
        int kdRoll = 0;

        yawApplied = kpYaw*(yawDes-yaw) + kdYaw*(yawDotDes-yawDot);
        pitchApplied = kpPitch*(pitchDes-pitch) + kdPitch*(pitchDotDes-pitchDot);
        rollApplied = kpRoll*(rollDes-roll) + kdRoll*(rollDotDes-rollDot);
    }

    int BLDCSpeed = map(IBus.readChannel(2), PWM_MIN, PWM_MAX, 0, 1000);

    int u1 = 90;
    int u2 = 90;
    int u3 = 0;
    int u4 = 0;

    if (IBus.readChannel(0) == 0) { //no signal received from RC receiver
        u1 = 90;
        u2 = 90;
        u3 = 0;
        u4 = 0;
    }
    else {
        u1 = 90 + pitchApplied - yawApplied;
        u2 = 90 + pitchApplied + yawApplied;
        u3 = BLDCSpeed + rollApplied;
        u4 = BLDCSpeed - rollApplied;
    }

    motors.writeMotors(u1, u2, u3, u4);
}


#include <avr/wdt.h> //watchdog
#include "Constants.h"
#include "Motors.h"
#include "IMU.h" //requires I2Cdev https://github.com/jrowberg/i2cdevlib
#include "RC.h" //requires FlySkyIBus https://gitlab.com/timwilkinson/FlySkyIBus

Motors motors(LEFT_SERVO_PIN, RIGHT_SERVO_PIN, LEFT_BLDC_PIN, RIGHT_BLDC_PIN, LEFT_ENCODER_PIN, RIGHT_ENCODER_PIN);
IMU imu(SDA_PIN, SCL_PIN, INTERRUPT_PIN);
RC rc;

//Interrupt service routine (ISR) triggered by IMU
volatile bool imuInterrupt = false; // indicates whether MPU interrupt pin has gone high, most likely indicating that a packet has arrived
void imuInterruptISR() {
    imuInterrupt = true;
}

void setup() {
    Serial.begin(115200);
    pinMode(INTERRUPT_PIN, INPUT);
    attachInterrupt(digitalPinToInterrupt(INTERRUPT_PIN), imuInterruptISR, RISING);
    pinMode(LED_PIN, OUTPUT); //on-board LED used to indicate successful arduino initialization

    motors.attachMotors();
    rc.begin(Serial);
    if (!imu.initialize()) {
        wdt_enable(WATCHDOG_TIME); //enable the watchdog to force the software to reset if an infinite loop is reached
        wdt_reset();
        while (true) { Serial.println("dmp initialization failure"); }
    }
    
    digitalWrite(LED_PIN, HIGH);
    wdt_enable(WATCHDOG_TIME); //initialize watchdog. This needs to happen at the end since imu initialization takes a while
}

void loop() {
    wdt_reset(); //reset watchdog
    rc.loop();
    if (imuInterrupt) { //if the IMU interrupt has been triggered, most likely indicating that a packet has been received from the IMU
        imuInterrupt = false;
        imu.update();
    }

    int yawApplied = 0;
    int pitchApplied = 0;
    int rollApplied = 0;
    int BLDCSpeed = 0;

    if (rc.getReadyState()) { //update control values if the RC receiver has started receiving data
        if (rc.getSWD()) { //if switch D is activated, apply feedback control
            //target orientation
            static float yawDes = 0;
            yawDes += rc.getLeftHor()*0.001 - 0.5;
            int pitchDes = map(rc.getRightVer(), SPEED_MIN, SPEED_MAX, 15, -15);
            int rollDes =  map(rc.getRightHor(), SPEED_MIN, SPEED_MAX, -15, 15);
            int yawDotDes = 0;
            int pitchDotDes = 0;
            int rollDotDes = 0;

            //IMU readings
            float yaw = imu.read().yaw;
            float pitch = imu.read().pitch;
            float roll = imu.read().roll;
            int yawDot = imu.read().yawDot;
            int pitchDot = imu.read().pitchDot;
            int rollDot = imu.read().rollDot;
    
            //PD feedback control parameters
            static float kpYaw = 0.85; //corresponds to a 12 o'clock position on the variable resistor when set to rc.getVRA()*0.005
            static float kdYaw = 0.2; //corresponds to the 2.5th dot position on the variable resistor when set to rc.getVRB()*0.005
            static float kpPitch = 0.85;
            static float kdPitch = 0.2;
            static float kpRoll = 1.88;
            static float kdRoll = 1.88;
    
            if (rc.getSWB()) { //gain setting mode
                if (rc.getSWC() == 0) { //set k yaw values
                    kpYaw = rc.getVRA()*0.005;
                    kdYaw = rc.getVRB()*0.005;
                }
                else if (rc.getSWC() == 1) { //set k pitch values
                    kpPitch = rc.getVRA()*0.005;
                    kdPitch = rc.getVRB()*0.005;
                }
                else if (rc.getSWC() == 2) { //set k roll values
                    kpRoll = rc.getVRA()*0.005;
                    kdRoll = rc.getVRB()*0.005;
                }
            }
    
            //PD feedback control
            yawApplied = (int) (kpYaw*(yawDes-yaw) + kdYaw*(yawDotDes-yawDot));
            pitchApplied = (int) (kpPitch*(pitchDes-pitch) + kdPitch*(pitchDotDes-pitchDot));
            rollApplied = (int) (kpRoll*(rollDes-roll) + kdRoll*(rollDotDes-rollDot));
    
            BLDCSpeed = map(rc.getLeftVer(), SPEED_MIN, SPEED_MAX, SPEED_MIN, SPEED_MAX*0.75);
        }
        else { //if switch D is not activated, motors are operated manually with no feedback control
            yawApplied = map(rc.getLeftHor(), SPEED_MIN, SPEED_MAX, -45, 45);
            pitchApplied = map(rc.getRightVer(), SPEED_MIN, SPEED_MAX, 45, -45);
            rollApplied = map(rc.getRightHor(), SPEED_MIN, SPEED_MAX, -300, 300);
    
            BLDCSpeed = map(rc.getLeftVer(), SPEED_MIN, SPEED_MAX, SPEED_MIN, SPEED_MAX*0.75);
        }
    }

    int u1 = pitchApplied - yawApplied;
    int u2 = pitchApplied + yawApplied;
    int u3 = rc.getSWA() ? BLDCSpeed + rollApplied : 0; //if switch A is turned off, set BLDC speed to 0
    int u4 = rc.getSWA() ? BLDCSpeed - rollApplied : 0;

    motors.writeMotors(u1, u2, u3, u4);
}


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
}

void loop() {
    IBus.loop();

    if (imuInterrupt || imu.packetAvailable()) {
        imuInterrupt = false;
        imu.update();
    }

    Serial.print(imu.read().yaw);
    Serial.print('\t');
    Serial.print(imu.read().pitch);
    Serial.print('\t');
    Serial.print(imu.read().roll);
    Serial.print('\n');

    motors.writeMotors(90, 90, 0, 0);

    /*
    double yawSensitivity = (IBus.readChannel(4)-PWM_MIN)/1000.0;
    //double pitchSensitivity = (IBus.readChannel(5)-PWM_MIN)/1000.0;
    double pitchSensitivity = 1.0;
    double rollSensitivity = (IBus.readChannel(5)-PWM_MIN)/1000.0;
    
    int yaw = (int) map(IBus.readChannel(0), PWM_MIN, PWM_MAX, 45, -45)*yawSensitivity;
    int pitch = (int) map(IBus.readChannel(1), PWM_MIN, PWM_MAX, 45, -45)*pitchSensitivity + 90;
    int BLDCSpeed = map(IBus.readChannel(2), PWM_MIN, PWM_MAX, 0, 1000);
    int roll = map(IBus.readChannel(3), PWM_MIN, PWM_MAX, -300, 300)*rollSensitivity;

    int u1 = pitch + yaw;
    int u2 = pitch - yaw;
    int u3 = BLDCSpeed + roll;
    int u4 = BLDCSpeed - roll;

    if (IBus.readChannel(0) == 0) {
        motors.writeMotors(90, 90, 0, 0);
    }
    else {
        motors.writeMotors(u1, u2, u3, u4);
    }
    */

}

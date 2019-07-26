
#include "Constants.h"
#include "Motors.h"
#include "IMU.h"
#include "FlySkyIBus.h"

Motors motors(LEFT_SERVO_PIN, RIGHT_SERVO_PIN, LEFT_BLDC_PIN, RIGHT_BLDC_PIN, LEFT_ENCODER_PIN, RIGHT_ENCODER_PIN);

//IMU imu(SDA_PIN, SCL_PIN);

void setup() {
    Serial.begin(115200);

    motors.attachMotors();
    IBus.begin(Serial);
    //imu.initialize();
}

void loop() {
    IBus.loop();

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

}

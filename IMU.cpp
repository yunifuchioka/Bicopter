
/*
 * Written based on Jeff Rowberg's example MPU6050_RAW code
 * https://github.com/jrowberg/i2cdevlib
 */
 
#include "Arduino.h"
#include "IMU.h"
#include "Constants.h"
#include "I2Cdev.h"
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
    #include "Wire.h"
#endif

IMU::IMU(int SDAPin, int SCLPin) {
    _SDAPin = SDAPin;
    _SCLPin = SCLPin;
}

void IMU::initialize() {
    #if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
        Wire.begin();
    #elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
        Fastwire::setup(400, true);
    #endif

    mpu6050.initialize();
}

IMUReading IMU::read() {
    IMUReading returnVal;
    mpu6050.getMotion6(&returnVal.ax, &returnVal.ay, &returnVal.az, &returnVal.gx, &returnVal.gy, &returnVal.gz);

    return returnVal;
}

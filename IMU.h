
#ifndef IMU_H
#define IMU_H

#include "Arduino.h"
#include "MPU6050.h"

struct IMUReading {
    int ax, ay, az, gx, gy, gz;
};

class IMU {
    public:
        IMU(int SDAPin, int SCLPin);
        void initialize();
        IMUReading read();
        
    private:
        int _SDAPin;
        int _SCLPin;
        MPU6050 mpu6050;
};

#endif

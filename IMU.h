
#ifndef IMU_H
#define IMU_H

#include "Arduino.h"
#include "MPU6050DMP.h"

struct IMUReading {
    float yaw, pitch, roll;
};

class IMU {
    public:
        IMU(int SDAPin, int SCLPin, int interruptPin);
        bool initialize();
        bool packetAvailable();
        bool update();
        IMUReading read();
        
    private:
        int SDAPin;
        int SCLPin;
        int interruptPin;
        MPU6050 mpu6050;
        uint8_t mpuIntStatus;   // holds actual interrupt status byte from MPU
        uint16_t packetSize;    // expected DMP packet size (default is 42 bytes)
        uint16_t fifoCount;     // count of all bytes currently in FIFO
        uint8_t fifoBuffer[64]; // FIFO storage buffer
        Quaternion q;           // [w, x, y, z]         quaternion container
        VectorFloat gravity;    // [x, y, z]            gravity vector
        float ypr[3];           // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector
};

#endif

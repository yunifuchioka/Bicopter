
/*
 * Written based on Jeff Rowberg's example MPU6050_DMP6 code
 * https://github.com/jrowberg/i2cdevlib
 * Assumes that MPU6050_6Axis_MotionApps20.h has been included in the ino file
 */
 
#include "IMU.h"
#include "Constants.h"

#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
    #include "Wire.h"
#endif


IMU::IMU(int SDAPin, int SCLPin, int interruptPin) {
    this->SDAPin = SDAPin;
    this->SCLPin = SCLPin;
    this->interruptPin = interruptPin;
}

bool IMU::initialize() { //returns false if anything goes wrong
    #if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
        Wire.begin();
        Wire.setClock(400000); // 400kHz I2C clock. Comment this line if having compilation difficulties
    #elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
        Fastwire::setup(400, true);
    #endif

    mpu6050.initialize();
    if (!mpu6050.testConnection()) {
        return false; //MPU6050 initialization unsuccessful
    }

    bool dmpInitStatus = mpu6050.dmpInitialize();
    if (dmpInitStatus) {
        return false; //DMP initialization unsuccessful. Note MPU6050.dmpInitialize() returns 0 if successful
    }

    //TODO: calibrate these values
    mpu6050.setXGyroOffset(220);
    mpu6050.setYGyroOffset(76);
    mpu6050.setZGyroOffset(-85);
    mpu6050.setZAccelOffset(1788);

    mpu6050.CalibrateAccel(6);
    mpu6050.CalibrateGyro(6);
    mpu6050.setDMPEnabled(true);

    mpuIntStatus = mpu6050.getIntStatus();
    packetSize = mpu6050.dmpGetFIFOPacketSize();

    return true;
}

bool IMU::update() {
    mpuIntStatus = mpu6050.getIntStatus();
    fifoCount = mpu6050.getFIFOCount();

    if(fifoCount < packetSize){
        //Lets go back and wait for another interrupt. We shouldn't be here, we got an interrupt from another event
        return false;
    }
    // check for overflow (this should never happen unless our code is too inefficient)
    else if ((mpuIntStatus & _BV(MPU6050_INTERRUPT_FIFO_OFLOW_BIT)) || fifoCount >= 1024) {
        // reset so we can continue cleanly
        mpu6050.resetFIFO();
        return false;
    } // otherwise, check for DMP data ready interrupt (this should happen frequently)
    else if (mpuIntStatus & _BV(MPU6050_INTERRUPT_DMP_INT_BIT)) {
        // read a packet from FIFO
        while(fifoCount >= packetSize){ // Lets catch up to NOW, someone is using the dreaded delay()!
            mpu6050.getFIFOBytes(fifoBuffer, packetSize); //warning: it was observed that getFIFOBytes can hang occasionally
            // track FIFO count here in case there is > 1 packet available
            // (this lets us immediately read more without waiting for an interrupt)
            fifoCount -= packetSize;
        }
        mpu6050.dmpGetQuaternion(&q, fifoBuffer);
        mpu6050.dmpGetAccel(&aa, fifoBuffer);
        mpu6050.dmpGetGravity(&gravity, &q);
        mpu6050.dmpGetYawPitchRoll(ypr, &q, &gravity);
        mpu6050.dmpGetGyro(gyro, fifoBuffer);
        mpu6050.dmpGetLinearAccel(&aaReal, &aa, &gravity);
    }
    else {
        return false;
    }
}

IMUReading IMU::read() {
    IMUReading returnVal;
    returnVal.yaw = ypr[0] * 180/M_PI;
    returnVal.pitch = ypr[1] * 180/M_PI;
    returnVal.roll = ypr[2] * 180/M_PI;
    returnVal.yawDot = -gyro[2];    //not sure what the units here are... (need to look into how MPU6050DMP.h is implemented)
    returnVal.pitchDot = -gyro[1];
    returnVal.rollDot = gyro[0];
    returnVal.accelX = aaReal.x;
    returnVal.accelY = aaReal.y;
    returnVal.accelZ = aaReal.z;
    
    return returnVal;
}

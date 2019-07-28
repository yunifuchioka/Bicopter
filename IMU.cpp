
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
    _SDAPin = SDAPin;
    _SCLPin = SCLPin;
    _interruptPin = interruptPin;
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

    _mpuIntStatus = mpu6050.getIntStatus();
    _packetSize = mpu6050.dmpGetFIFOPacketSize();

    return true;
}

bool IMU::packetAvailable() {
    _fifoCount = mpu6050.getFIFOCount();
    return (_fifoCount > _packetSize);
}

bool IMU::update() {
    //IMUReading returnVal;
    _mpuIntStatus = mpu6050.getIntStatus();
    _fifoCount = mpu6050.getFIFOCount();

    if(_fifoCount < _packetSize){
        //Lets go back and wait for another interrupt. We shouldn't be here, we got an interrupt from another event
        return false;
    }
    // check for overflow (this should never happen unless our code is too inefficient)
    else if ((_mpuIntStatus & _BV(MPU6050_INTERRUPT_FIFO_OFLOW_BIT)) || _fifoCount >= 1024) {
        // reset so we can continue cleanly
        mpu6050.resetFIFO();
        return false;
    } // otherwise, check for DMP data ready interrupt (this should happen frequently)
    else if (_mpuIntStatus & _BV(MPU6050_INTERRUPT_DMP_INT_BIT)) {
        // read a packet from FIFO
        while(_fifoCount >= _packetSize){ // Lets catch up to NOW, someone is using the dreaded delay()!
            mpu6050.getFIFOBytes(_fifoBuffer, _packetSize);
            // track FIFO count here in case there is > 1 packet available
            // (this lets us immediately read more without waiting for an interrupt)
            _fifoCount -= _packetSize;
        }
        mpu6050.dmpGetQuaternion(&q, _fifoBuffer);
        mpu6050.dmpGetGravity(&gravity, &q);
        mpu6050.dmpGetYawPitchRoll(ypr, &q, &gravity);
    }
    else {
        return false;
    }
}

IMUReading IMU::read() {
    IMUReading returnVal;
    returnVal.yaw = ypr[0];
    returnVal.pitch = ypr[1];
    returnVal.roll = ypr[2];
    
    return returnVal;
}

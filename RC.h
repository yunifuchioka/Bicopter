
#ifndef RC_H
#define RC_H

#include "Arduino.h"

class RC {
    public:
        void begin(HardwareSerial& serial);
        void loop();
        int getLeftVer(); //return value ranges from 0 to 1000
        int getLeftHor(); //return value ranges from 0 to 1000
        int getRightVer(); //return value ranges from 0 to 1000
        int getRightHor(); //return value ranges from 0 to 1000
        int getVRA(); //return value ranges from 0 to 1000
        int getVRB(); //return value ranges from 0 to 1000
        bool getSWA(); //returns true or false
        bool getSWB(); //return true or false
        int getSWC(); //returns 0, 1 or 2
        bool getSWD(); //returns true or false
};

#endif

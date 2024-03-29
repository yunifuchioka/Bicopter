
#include "RC.h"
#include "Constants.h"
#include "FlySkyIBus.h" //https://gitlab.com/timwilkinson/FlySkyIBus

void RC::begin(HardwareSerial& serial) {
    IBus.begin(Serial);
    readyState = false;
}

void RC::loop() {
    IBus.loop();
    if (!readyState && IBus.readChannel(0)!=0) { //adding this adds about 4-12 microseconds to the loop function call
        readyState = true;
    }
}

bool RC::getReadyState() {
    return readyState;
}

int RC::getLeftVer() {
    return map(IBus.readChannel(2), PWM_MIN, PWM_MAX, SPEED_MIN, SPEED_MAX);
}

int RC::getLeftHor() {
    return map(IBus.readChannel(3), PWM_MIN, PWM_MAX, SPEED_MIN, SPEED_MAX);
}
int RC::getRightVer() {
    return map(IBus.readChannel(1), PWM_MIN, PWM_MAX, SPEED_MIN, SPEED_MAX);
}

int RC::getRightHor() {
    return map(IBus.readChannel(0), PWM_MIN, PWM_MAX, SPEED_MIN, SPEED_MAX);
}

int RC::getVRA() {
    return map(IBus.readChannel(4), PWM_MIN, PWM_MAX, SPEED_MIN, SPEED_MAX);
}

int RC::getVRB() {
    return map(IBus.readChannel(5), PWM_MIN, PWM_MAX, SPEED_MIN, SPEED_MAX);
}

bool RC::getSWA() {
    return (IBus.readChannel(6)-PWM_MIN > (PWM_MAX-PWM_MIN)/2);
}

bool RC::getSWB() {
    return (IBus.readChannel(7)-PWM_MIN > (PWM_MAX-PWM_MIN)/2);
}

int RC::getSWC() {
    int value = IBus.readChannel(8);
    if (value - PWM_MIN < (PWM_MAX-PWM_MIN)/3) {
        return 0;
    }
    else if (value - PWM_MIN >  (PWM_MAX-PWM_MIN)*2/3) {
        return 2;
    }
    else {
        return 1;
    }
}

bool RC::getSWD() {
    return (IBus.readChannel(9)-PWM_MIN > (PWM_MAX-PWM_MIN)/2);
}

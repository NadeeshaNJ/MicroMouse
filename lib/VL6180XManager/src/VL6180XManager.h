#ifndef VL6180XManager_h
#define VL6180XManager_h

#include <Arduino.h>
#include <Wire.h>
#include "Adafruit_VL6180X.h"

class VL6180XManager {
    private:
        int* xshutPins;
        int count;
        unsigned long delayTime;
        Adafruit_VL6180X* sensors;

        int currentSensor = 0;
        unsigned long sensorStartTime = 0;
        bool sensorPowerOn = false;
        bool sensorInitDone = false;

        int* distance;
    public:
        VL6180XManager(int* _xshutPins, int _count, unsigned long _delayTime = 50);
        void begin();
        void update();
        void updateWithSerial();
        int getDistance() const { return *distance; }

};

#endif

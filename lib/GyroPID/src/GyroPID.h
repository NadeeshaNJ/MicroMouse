#ifndef MPUHELPER_H
#define MPUHELPER_H

#include <Arduino.h>
#include <Wire.h>
#include <MPU9250_asukiaaa.h>

class GyroPID {
 private:
    MPU9250_asukiaaa sensor;
    float gyroX, gyroY, gyroZ;
    //int sdaPin, sclPin;
 public:
    GyroPID(int sda = 21, int scl = 22);
    void begin();
    void update();
    float getGyroX();
    float getGyroY();
    float getGyroZ();

};

#endif

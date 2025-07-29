#ifndef MPUHELPER_H
#define MPUHELPER_H

#include <Arduino.h>
#include <Wire.h>
#include <MPU9250_asukiaaa.h>

class GyroPID {
 private:
   MPU9250_asukiaaa sensor;
   uint8_t sensorId;

   float gyroX, gyroY, gyroZ;
   float accelX, accelY, accelZ;
   void applyFilter(float &smoothedValue, float newValue, float alpha);
   const float alpha = 0.2; // smoothing factor for EMA

 public:
   GyroPID();
   void begin();
   void update();

   float getGyroX();
   float getGyroY();
   float getGyroZ();

   float getAccelX();
   float getAccelY();
   float getAccelZ();

};

#endif

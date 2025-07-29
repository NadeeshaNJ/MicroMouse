#ifndef MPUHELPER_H
#define MPUHELPER_H

#include <Arduino.h>
#include <Wire.h>
#include <MPU9250_asukiaaa.h>

class GyroPID {
 private:
   MPU9250_asukiaaa sensor;

   float gyroX, gyroY, gyroZ;
   float accelX, accelY, accelZ;
   void applyFilter(float &smoothedValue, float newValue, float alpha);
   const float alpha = 1; // smoothing factor for EMA

   float gyroZ_Correction = 1.05;// Correction factor for gyro Z axis

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

   float getYaw(); // Returns yaw angle based on accelerometer data

};

#endif

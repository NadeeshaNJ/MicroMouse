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

    float gyroZ_Correction = 0.891;// Correction factor for gyro Z axis

    
    float integralError;
    long previousTime;
    float previousError;

    float imuYaw;
    float targetYaw; //in degrees
    float Kp = 2; // Proportional gain for angle PID
    float Ki = 0.1; // Integral gain for angle PID
    float Kd = 0.5; // Derivative gain for angle PID

    float toleranceYaw = 2.0; // Tolerance for angle PID


 public:
   GyroPID();
   void begin();
   void update();
   void setTargetYaw(float target) { targetYaw = target; }

   float getGyroX();
   float getGyroY();
   float getGyroZ();

   float getAccelX();
   float getAccelY();
   float getAccelZ();

   float getYaw(); // Returns yaw angle based on accelerometer data

   int calculateAnglePID();
   bool checkDone();

};

#endif

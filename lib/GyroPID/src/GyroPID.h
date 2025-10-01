#ifndef GYROPID_H
#define GYROPID_H

#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>

class GyroPID {
 private:
   Adafruit_BNO055 sensor;
   float lastYaw;
    ///////////////////////////////////////

   float integralError = 0.0f;
   unsigned long previousTime = 0;
   float previousError = 0.0f;

   float imuYaw = 0.0f;

   float deltaTime = 0.14; // assuming a fixed loop time of 14ms

 public:
   GyroPID();
   bool begin(uint8_t address = 0x28); // default I2C address
   float getYaw();          // fused yaw from BNO055
   float angleDiff(float target, float current);
   int calculateAnglePID(); // PID control for yaw
   bool checkDone();
   void setTargetYaw(float target);

   float targetYaw; //in degrees
   float Kp = 1; // Proportional gain for angle PID
   float Ki = 0.02; // Integral gain for angle PID
   float Kd = 0.15; // Derivative gain for angle PID

    float toleranceYaw = 4.0; // Tolerance for angle PID
   ///////////////////////// 
};

#endif

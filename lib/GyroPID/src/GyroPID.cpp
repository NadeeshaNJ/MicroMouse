#include "GyroPID.h"

GyroPID::GyroPID() : sensor(55, 0x28, &Wire) {}

bool GyroPID::begin(uint8_t address) {
  if (!sensor.begin()) {
    return false; // not detected
  }
  sensor.setExtCrystalUse(true); // improves heading stability
  return true;
}
float GyroPID::getYaw() {
  sensors_event_t orientationData;
  sensor.getEvent(&orientationData, Adafruit_BNO055::VECTOR_EULER);
  return orientationData.orientation.x; // yaw (heading)
}
float GyroPID::angleDiff(float target, float current) {
    float diff = fmodf(target - current + 540.0f, 360.0f) - 180.0f;
    return diff;
}

int GyroPID::calculateAnglePID() {
    imuYaw = getYaw();
    float angleError = angleDiff(targetYaw, imuYaw);
    // long currentTime = micros();
    // float deltaTime = ((float)(currentTime - previousTime)) / 1.0e6;
    // if (deltaTime <= 0.000001) deltaTime = 0.000001;     

    // Calculate derivative
    float derivative = (angleError - previousError) / deltaTime;

    // Calculate integral
    integralError += angleError * deltaTime;
    integralError = constrain(integralError, -200, 200);  // Clamp integral

    // Calculate PID output
    float anglePID = Kp * angleError + Ki * integralError + Kd * derivative;
    anglePID = constrain(anglePID, -200, 200); // Clamp output to motor limits
    previousError = angleError;
    // previousTime = currentTime;

  return (int)anglePID;
}
bool GyroPID::checkDone(){
    //imuYaw = getYaw(); //removes because getYaw() is called in calculateAnglePID() so this will be efficient
  float err = angleDiff(targetYaw, imuYaw);
  Serial.println("Current Yaw: " + String(imuYaw) + " err:" + String(err));
  return fabs(err) < toleranceYaw;
}
void GyroPID::setTargetYaw(float target) {
    targetYaw = fmodf(target + 360.0f, 360.0f); // always in [0,360)
    integralError = 0;
    previousError = 0;
}
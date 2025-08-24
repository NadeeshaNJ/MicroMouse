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

int GyroPID::calculateAnglePID() {
    imuYaw = getYaw();
    float angleError = targetYaw - imuYaw; // Assuming targetYaw is in degrees
    long currentTime = millis();
    float deltaTime = ((float)(currentTime - previousTime)) / 1000.0;

    // Calculate derivative
    float derivative = (angleError - previousError) / deltaTime;

    // Calculate integral
    integralError += angleError * deltaTime;
    integralError = constrain(integralError, -300, 300);  // Clamp integral

    // Calculate PID output
    float anglePID = Kp * angleError + Ki * integralError + Kd * derivative;

    previousError = angleError;
    previousTime = currentTime;

    return (int)anglePID;
}
bool GyroPID::checkDone(){
    //imuYaw = getYaw(); //removes because getYaw() is called in calculateAnglePID() so this will be efficient
    Serial.println("Current Yaw: " + String(imuYaw));
    return abs(targetYaw - imuYaw) < toleranceYaw;
}
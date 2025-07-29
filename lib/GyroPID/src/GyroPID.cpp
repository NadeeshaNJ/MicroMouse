#include "GyroPID.h"

GyroPID::GyroPID() {
//   sdaPin = sda;
//   sclPin = scl;
}

void GyroPID::begin() {
    Wire.begin();
    sensor.setWire(&Wire);

    uint8_t sensorId;
    int result;
    result = sensor.readId(&sensorId);
    if (result == 0) {
        Serial.println("sensorId: " + String(sensorId));
    } else {
        Serial.println("Cannot read sensorId " + String(result));
    }

    sensor.beginAccel();
    sensor.beginGyro();

    // Initialize smoothed values to zero
    gyroX = gyroY = gyroZ = 0;
    accelX = accelY = accelZ = 0;
}

void GyroPID::update() {
  sensor.accelUpdate();
  sensor.gyroUpdate();

  applyFilter(gyroX, sensor.gyroX(), alpha);
  applyFilter(gyroY, sensor.gyroY(), alpha);
  applyFilter(gyroZ, sensor.gyroZ(), alpha);

  applyFilter(accelX, sensor.accelX(), alpha);
  applyFilter(accelY, sensor.accelY(), alpha);
  applyFilter(accelZ, sensor.accelZ(), alpha);
}
void GyroPID::applyFilter(float &smoothedValue, float newValue, float alpha) {
    smoothedValue = (1 - alpha) * smoothedValue + alpha * newValue;
}

float GyroPID::getGyroX() { return gyroX; }
float GyroPID::getGyroY() { return gyroY; }
float GyroPID::getGyroZ() { return gyroZ; }

float GyroPID::getAccelX() { return accelX; }
float GyroPID::getAccelY() { return accelY; }
float GyroPID::getAccelZ() { return accelZ; }

//float GyroPID::getYaw() { return atan2(accelY, accelZ) * 180 / M_PI; }
float GyroPID::getYaw() {
    // Yaw calculation using atan2 of accelY and accelX (for demonstration)
    // You may want to use gyro integration or sensor fusion for better results
    // Integrate gyroZ over time to estimate yaw (assuming update() is called at a fixed interval)
    static float yaw = 0.0f;
    static unsigned long lastUpdate = 0;
    unsigned long now = millis();
    float dt = (lastUpdate == 0) ? 0 : (now - lastUpdate) / 1000.0f; // seconds
    lastUpdate = now;

    // GyroZ is in degrees/sec, integrate to get degrees
    yaw += (sensor.gyroZ() + gyroZ_Correction) * dt;

    // Optionally, keep yaw in [-180, 180] range
    if (yaw < -180.0f) yaw -= 360.0f;
    if (yaw > 180.0f) yaw += 360.0f;

    return yaw;
}
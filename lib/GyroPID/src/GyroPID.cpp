#include "GyroPID.h"

GyroPID::GyroPID() {
//   sdaPin = sda;
//   sclPin = scl;
}

void GyroPID::begin() {
    Wire.begin();
    sensor.setWire(&Wire);


    if (sensor.readId(&sensorId) == 0) {
    Serial.print("Sensor ID: "); 
    Serial.println(sensorId);
    } else {
    Serial.println("Sensor not found!");
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

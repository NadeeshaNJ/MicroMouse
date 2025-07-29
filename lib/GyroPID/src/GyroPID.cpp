#include "GyroPID.h"

GyroPID::GyroPID(int sda, int scl) {
  sdaPin = sda;
  sclPin = scl;
}

void GyroPID::begin() {
  Wire.begin(sdaPin, sclPin);
  sensor.setWire(&Wire);
  sensor.beginAccel();
  sensor.beginGyro();
  sensor.beginMag();
}

void GyroPID::update() {
  if (sensor.gyroUpdate() == 0) {
    gyroX = sensor.gyroX();
    gyroY = sensor.gyroY();
    gyroZ = sensor.gyroZ();
  }
}

float GyroPID::getGyroX() {
  return gyroX;
}

float GyroPID::getGyroY() {
  return gyroY;
}

float GyroPID::getGyroZ() {
  return gyroZ;
}

#include <GyroPID.h>

GyroPID gyro; // Uses default SDA=21, SCL=22

void setup() {
  Serial.begin(115200);
  gyro.begin();
}

void loop() {
  gyro.update();

  // Serial.print("Gyro X: ");
  // Serial.print(gyro.getGyroX());
  // Serial.print(" Y: ");
  // Serial.print(gyro.getGyroY());
  // Serial.print(" Z: ");
  // Serial.println(gyro.getGyroZ());

  // Serial.print("Accel X: ");
  // Serial.print(gyro.getAccelX());
  // Serial.print(" Y: ");
  // Serial.print(gyro.getAccelY());
  // Serial.print(" Z: ");
  // Serial.println(gyro.getAccelZ());

  Serial.print("Yaw: ");
  Serial.println(gyro.getYaw());

  //delay(400);
}

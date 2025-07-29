#include <GyroPID.h>

GyroPID gyro; 
//What I need is gyro Z axis

void setup() {
  Serial.begin(115200);
  gyro.begin();
}

void loop() {
  gyro.update();

  Serial.print("Gyro X: ");
  Serial.print(gyro.getGyroX());
  Serial.print(" Y: ");
  Serial.print(gyro.getGyroY());
  Serial.print(" Z: ");
  Serial.println(gyro.getGyroZ());

  Serial.print("Accel X: ");
  Serial.print(gyro.getAccelX());
  Serial.print(" Y: ");
  Serial.print(gyro.getAccelY());
  Serial.print(" Z: ");
  Serial.println(gyro.getAccelZ());

  delay(100);
}

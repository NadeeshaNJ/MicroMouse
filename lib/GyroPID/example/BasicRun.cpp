#include "GyroPID.h"

GyroPID gyro;

void setup() {
  Serial.begin(115200);
  if (!gyro.begin()) {
    Serial.println("BNO055 not detected!");
    while (1);
  }
  gyro.targetYaw = 90.0; // example: rotate to 90°
  delay(100);
}

void loop() {
  Serial.print("Yaw: "); Serial.println(gyro.getYaw());

  delay(100);
}

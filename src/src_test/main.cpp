#include <GyroPID.h>

GyroPID gyro; 
//What I need is gyro Z axis

void setup() {
  Serial.begin(115200);
  gyro.begin();
}

void loop() {
  gyro.update();

  Serial.print("Yaw: ");
  Serial.print(gyro.getYaw());
  Serial.println("°");

}

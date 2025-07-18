#include <VL6180XManagerV2.h>
#include "MotorPIDbyNJ.h"
#include <Arduino.h>

MotorPIDbyNJ leftMotor(25, 26, 18, 5);
MotorPIDbyNJ rightMotor(14, 27, 19, 23);

// ISR handlers
void updateLeftEncoder() { leftMotor.updateEncoder(); }
void updateRightEncoder() { rightMotor.updateEncoder(); }

int xshutPins[] = {17, 16, 4};
VL6180XManagerV2 sensorGroup(xshutPins, 3);

void setup() {
  Wire.begin();
  Serial.begin(115200);
  sensorGroup.begin();

  leftMotor.attachEncoderInterrupt(updateLeftEncoder);
  rightMotor.attachEncoderInterrupt(updateRightEncoder);

  leftMotor.setPID(0.6, 0.0, 0.06, 50); // kp, ki, kd, error tolerance
  rightMotor.setPID(0.6, 0.0, 0.07, 50);

}

void loop() {
  std::vector<int> distances = sensorGroup.readAll();
  
  leftMotor.reachTarget(distances[1]*258); //run the motor
  rightMotor.reachTarget(distances[1]*258);

  if (leftMotor.done() && rightMotor.done()) {
    Serial.println("target reached!");
    while (1); // Stop further loop
  }

  delay(10); // Reduce CPU stress
}

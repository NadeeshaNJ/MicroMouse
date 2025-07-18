#include "MotorPIDbyNJ.h"
#include <Arduino.h>

MotorPIDbyNJ leftMotor(25, 26, 18, 5);
MotorPIDbyNJ rightMotor(14, 27, 19, 23);

// ISR handlers
void updateLeftEncoder() { leftMotor.updateEncoder(); }
void updateRightEncoder() { rightMotor.updateEncoder(); }

void setup() {
  Serial.begin(115200);
  
  leftMotor.attachEncoderInterrupt(updateLeftEncoder);
  rightMotor.attachEncoderInterrupt(updateRightEncoder);

  leftMotor.setPID(0.68, 0.0, 0.04, 50); // kp, ki, kd, tolerance
  rightMotor.setPID(0.68, 0.0, 0.04, 50);

  leftMotor.setTarget(4000);
  rightMotor.setTarget(4000);
}

void loop() {
  leftMotor.update(); //run the motors
  rightMotor.update();

  if (leftMotor.done() && rightMotor.done()) {
    Serial.println("Both motors reached target!");
    while (1); // Stop further loop
  }

  delay(10); // Reduce CPU stress
}

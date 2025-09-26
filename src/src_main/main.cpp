// Keep main minimal: implement the extern primitives expected by Floodfill module
#include <Arduino.h>
#include <vector>
#include <VL6180XManagerV2.h>
#include <MotorPIDbyNJ.h>
#include <RobotNavigatorV2.h>
#include <GyroPID.h>
#include "Floodfill.h"

// Forward-declare the floodfill step function defined in the Floodfill module
void runFloodfillStep();

// Hardware instances
int xshutPins[] = {32, 17, 16, 15, 4};
int sensorCorrections[] = { 6, 16, 0, 43, 26};  // mm to subtract from each sensor
VL6180XManagerV2 sensorGroup(xshutPins, 5, sensorCorrections);

MotorPIDbyNJ leftMotor(25, 26, 18, 5);
MotorPIDbyNJ rightMotor(14, 27, 19, 23);
GyroPID imuController;
RobotNavigatorV2 Motors(&leftMotor, &rightMotor, &imuController);

// Encoder ISRs
void updateLeftEncoder() { leftMotor.updateEncoder(); }
void updateRightEncoder() { rightMotor.updateEncoder(); }

// --- Extern hooks required by Floodfill_SearchRun.cpp ---
void moveForward() { Motors.moveForward(); }
void turnLeft() { Motors.turnLeft(); }
void turnRight() { Motors.turnRight(); }
void turnAround() { Motors.turnAround(); }
void moveForwardUpdatePos(); // defined in Floodfill module now does pose update
std::vector<int> getDistances() { return sensorGroup.readAll(); }

void setup() {
  Serial.begin(115200);

  // Explicit I2C init helps avoid pin mux issues when WiFi is enabled
  Wire.begin();
  sensorGroup.begin();
  imuController.begin();

  Motors.setSensorGroup(&sensorGroup);

  leftMotor.attachEncoderInterrupt(updateLeftEncoder);
  rightMotor.attachEncoderInterrupt(updateRightEncoder);

  // Basic PID values; tune as needed
  leftMotor.setPID(3, 0.05, 0.4, 8);
  rightMotor.setPID(3, 0.05, 0.4, 8);
}

void loop() {
  // Run one solver step each tick so loop stays responsive
  runFloodfillStep();
  delay(20);
}

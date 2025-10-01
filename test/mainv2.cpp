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

// Global variables for robot position and maze solver
int row = 0, col = 0, facingDirection = 0;
Floodfill solveMaze;

// Function to detect walls based on sensor readings
void detectWalls(const std::vector<int>& distances, int currentRow, int currentCol, int facing);

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
}
void detectWalls(const std::vector<int>& distances, int currentRow, int currentCol, int facing) {
  // Implementation for wall detection based on sensor readings
  // This is a placeholder - implement based on your sensor arrangement and thresholds
}


void loop() {
  // 1) Read sensors
  std::vector<int> distances = sensorGroup.readAll();

  // 2) Update walls at current cell/orientation
  detectWalls(distances, row, col, facingDirection);

  // 3) Recompute distances to goal
  solveMaze.floodfill();

  // 4) Decide next best direction
  int bestDir = solveMaze.getNextMove(row, col);
  if (bestDir < 0) {
    // No valid move; idle this tick
    delay(20);
    return;
  }

  // 5) Rotate to bestDir, then move forward one cell
  int diff = (bestDir - facingDirection + 4) % 4;
  if (diff == 1) {
    Motors.turnRight();
    facingDirection = (facingDirection + 1) % 4;
  } else if (diff == 3) {
    Motors.turnLeft();
    facingDirection = (facingDirection + 3) % 4;
  } else if (diff == 2) {
    Motors.turnAround();
    facingDirection = (facingDirection + 2) % 4;
  }

  Motors.moveForward();

  // 6) Update pose after one cell
  if (facingDirection == 0) row++;
  else if (facingDirection == 1) col++;
  else if (facingDirection == 2) row--;
  else if (facingDirection == 3) col--;

  delay(20);
}

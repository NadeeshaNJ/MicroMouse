#include <Arduino.h>
#include <VL6180XManagerV2.h>
#include <Floodfill.h>
#include <MotorPIDbyNJ.h>
#include <RobotNavigatorV2.h>

int xshutPins[] = {32, 17, 15, 4};
int sensorCorrections[] = {0, 6, 43, 26};  // mm to subtract from each sensor
VL6180XManagerV2 sensorGroup(xshutPins, 4, sensorCorrections);

Floodfill solveMaze;

MotorPIDbyNJ leftMotor(25, 26, 18, 5);
MotorPIDbyNJ rightMotor(14, 27, 19, 23);
RobotNavigatorV2 Motors(&leftMotor, &rightMotor);
void updateLeftEncoder() { leftMotor.updateEncoder(); }
void updateRightEncoder() { rightMotor.updateEncoder(); }

bool firstTurn = true;
bool leftWall, rightWall, frontWall = false;

int row = 15;
int col = 0;
int facingDirection = 0;

void setup() {
  Serial.begin(115200);
  Wire.begin();
  sensorGroup.begin();
  
  solveMaze.setThreshhold(80);

  leftMotor.attachEncoderInterrupt(updateLeftEncoder);
  rightMotor.attachEncoderInterrupt(updateRightEncoder);

  leftMotor.setPID(0.68, 0.0, 0.04, 50);
  rightMotor.setPID(0.68, 0.0, 0.04, 50);
}

void loop() {
  vector<int> sensorDistances = sensorGroup.readAll(); 
  
  solveMaze.detectWalls(sensorDistances, row, col, facingDirection);
  solveMaze.floodfill();
  //int nextMove = solveMaze.getNextMove(row, col); //row and column for the next move will also be updated from here

  if(firstTurn){
    Motors.resetEncoders();
    firstTurn = false;
  }

  Motors.moveForward();
}

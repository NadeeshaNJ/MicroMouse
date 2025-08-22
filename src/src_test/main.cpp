#include <Arduino.h>
#include <VL6180XManagerV2.h>
//#include <Floodfill.h>
#include <MotorPIDbyNJ.h>
#include <RobotNavigatorV2.h>
#include <GyroPID.h>

int xshutPins[] = {32, 17, 16, 15, 4};
int sensorCorrections[] = { 6, 16, 0, 43, 26};  // mm to subtract from each sensor
VL6180XManagerV2 sensorGroup(xshutPins, 5, sensorCorrections);

//Floodfill solveMaze;
int dist = 0;
MotorPIDbyNJ leftMotor(25, 26, 18, 5);
MotorPIDbyNJ rightMotor(14, 27, 19, 23);
GyroPID imu;
RobotNavigatorV2 Motors(&leftMotor, &rightMotor, &imu);
// Motors.setSensorGroup(&sensorGroup); // moved to setup()
void updateLeftEncoder() { leftMotor.updateEncoder(); }
void updateRightEncoder() { rightMotor.updateEncoder(); }

int row = 0;
int col = 0;
int facingDirection = 0;

int nextMove = 0; // 0 = North, 1 = East, 2 = South, 3 = West

int lastMove = -1;
bool justFinishedMove = false;

void setup() {
  Serial.begin(115200);
  Wire.begin();
  sensorGroup.begin();
  imu.begin();

  Motors.setSensorGroup(&sensorGroup);
  
  //solveMaze.setThreshhold(80);

  leftMotor.attachEncoderInterrupt(updateLeftEncoder);
  rightMotor.attachEncoderInterrupt(updateRightEncoder);

  leftMotor.setPID(3, 0.1, 0.4, 8);
  rightMotor.setPID(3, 0.1, 0.4, 8);

}

bool testMoveDone = false;
void loop() {
  if (!testMoveDone) {
    Motors.moveForward();
    Motors.moveForward();
    Motors.turnRight();
    Motors.moveForward();
    Motors.turnRight();
    Motors.moveForward();

    Motors.turnLeft();
    testMoveDone = true;
    Serial.println("Exited");
  }
  // ...existing code or idle...
}

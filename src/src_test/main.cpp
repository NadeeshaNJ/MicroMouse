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
GyroPID gyro;
RobotNavigatorV2 Motors(&leftMotor, &rightMotor, &gyro);
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
  gyro.begin();

  Motors.setSensorGroup(&sensorGroup);
  
  //solveMaze.setThreshhold(80);

  leftMotor.attachEncoderInterrupt(updateLeftEncoder);
  rightMotor.attachEncoderInterrupt(updateRightEncoder);

  leftMotor.setPID(3, 0.05, 0.4, 8);
  rightMotor.setPID(3, 0.05, 0.4, 8);

}

bool testMoveDone = false;
void loop() {
  // static bool testMoveDone = false;
  //   if (!testMoveDone) {
  //       runFloodfillRobot();
  //       testMoveDone = true;
  //   }
  if (!testMoveDone) {

    //Motors.turnLeft();

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

// #include <Arduino.h>
// #include <WiFiHandler.h>

// // WiFi credentials
// const char* ssid = "Dialog NNJ";
// const char* password = "Mixtures";
// // Web server and WebSerial
// WiFiHandler OTA;
// int count =0;
// int last = millis();
// void setup() {
//   Serial.begin(115200);
//   OTA.begin(ssid, password);

// }

// void loop() {
//   ArduinoOTA.handle();
//   if (millis() - last > 1000) {
//     count++;
//     OTA.webSerial.println("Hello World!");
//     last = millis();
//   }

// }




// #include <WiFiHandler.h>
// #include <Arduino.h>
// #include <VL6180XManagerV2.h>
// //#include <Floodfill.h>
// #include <MotorPIDbyNJ.h>
// #include <RobotNavigatorV2.h>
// #include <GyroPID.h>

// //#define Serial OTA.webSerial

// // WiFi credentials
// const char* ssid = "Dialog NNJ";
// const char* password = "Mixtures";
// // Web server and WebSerial
// WiFiHandler OTA;
// int count = 0;
// int last = millis();

// int xshutPins[] = {32, 17, 16, 15, 4};
// int sensorCorrections[] = { 6, 16, 0, 43, 26};  // mm to subtract from each sensor
// VL6180XManagerV2 sensorGroup(xshutPins, 5, sensorCorrections);

// //Floodfill solveMaze;
// //Floodfill solveMaze;
// int dist = 0;
// MotorPIDbyNJ leftMotor(25, 26, 18, 5);
// MotorPIDbyNJ rightMotor(14, 27, 19, 23);
// GyroPID imuController;
// RobotNavigatorV2 Motors(&leftMotor, &rightMotor, &imuController);
// // Motors.setSensorGroup(&sensorGroup); // moved to setup()
// void updateLeftEncoder() { leftMotor.updateEncoder(); }
// void updateRightEncoder() { rightMotor.updateEncoder(); }

// int row = 0;
// int col = 0;
// int facingDirection = 0;

// int nextMove = 0; // 0 = North, 1 = East, 2 = South, 3 = West

// int lastMove = -1;
// bool justFinishedMove = false;
// void setup() {
//   Serial.begin(115200);
//   Wire.begin();
//   sensorGroup.begin();
//   imuController.begin();
  
//   OTA.begin(ssid, password);

//   Motors.setSensorGroup(&sensorGroup);
  
//   //solveMaze.setThreshhold(80);

//   leftMotor.attachEncoderInterrupt(updateLeftEncoder);
//   rightMotor.attachEncoderInterrupt(updateRightEncoder);

//   leftMotor.setPID(3, 0.05, 0.4, 8);
//   rightMotor.setPID(3, 0.05, 0.4, 8);

// }

// bool testMoveDone = false;
// void loop() {  
//   ArduinoOTA.handle();
//   //OTA.webSerial.println("Hello World!");
//   sensorGroup.begin();
//   Serial.println("Trying Serial..");
//   if (!testMoveDone) {
//     Motors.moveForward();
//   //   Motors.moveForward();
//   //   Motors.turnRight();
//   //   Motors.moveForward();
//   //   Motors.turnRight();
//   //   Motors.moveForward();

//   //   Motors.turnLeft();
//     testMoveDone = true;
//     Serial.println("Exited");
//   }
//   // ...existing code or idle...
// }

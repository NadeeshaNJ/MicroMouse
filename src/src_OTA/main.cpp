// #include <Arduino.h>
// #include <WiFiHandler.h>
// #include <VL6180XManagerV2.h>
// #include <Floodfill.h>
// #include <MotorPIDbyNJ.h>
// #include <RobotNavigatorbyNJ.h>
// #include <Arduino.h>
// #include <button.h>

// button myButton(2); 

// // WiFi credentials
// const char* ssid = "Dialog NNJ";
// const char* password = "Mixtures";
// // Web server and WebSerial
// WiFiHandler OTA;
// int count =0;
// int last = millis();

// int xshutPins[] = {32, 17, 15, 4};
// int sensorCorrections[] = {0, 6, 43, 26};  // mm to subtract from each sensor
// VL6180XManagerV2 sensorGroup(xshutPins, 4, sensorCorrections);

// Floodfill solveMaze;

// MotorPIDbyNJ leftMotor(25, 26, 18, 5);
// MotorPIDbyNJ rightMotor(14, 27, 19, 23);
// RobotNavigatorbyNJ Motors(&leftMotor, &rightMotor);
// void updateLeftEncoder() { leftMotor.updateEncoder(); }
// void updateRightEncoder() { rightMotor.updateEncoder(); }

// bool firstTurn = true;
// bool leftWall, rightWall, frontWall = false;

// int row = 15;
// int col = 0;
// int facingDirection = 0;

// float error;
// long previousTime;
// float previousError;
// float integral;
// float kp,ki,kd;
// float directionL, directionR;
// float speedL,speedR;

// void onSinglePress() {
//   analogWrite(25, 0);
//   analogWrite(26, 0);
//   analogWrite(14, 0);
//   analogWrite(27, 0);
//   delay(100);
//   while(1);
// }

// void setup() {
//   Serial.begin(115200);
//   OTA.begin(ssid, password);
//   Wire.begin();
//   sensorGroup.begin();

  
//   myButton.setSinglePressCallback(onSinglePress);
  
//   solveMaze.setThreshhold(80);

//   leftMotor.attachEncoderInterrupt(updateLeftEncoder);
//   rightMotor.attachEncoderInterrupt(updateRightEncoder);

//   leftMotor.setPID(0.68, 0.0, 0.04, 50);
//   rightMotor.setPID(0.68, 0.0, 0.04, 50);
//   previousTime = micros();
//   kp = 0.68;
//   ki=0;
//   kd=0.04;
// }

// void loop() {
//   myButton.update();
//   ArduinoOTA.handle();
//   if (millis() - last > 1000) {
//     OTA.webSerial.println("Hello World!");
//     last = millis();
//   }
//   vector<int> sensorDistances = sensorGroup.readAll(); 
//   error = sensorDistances[0] - sensorDistances[3];
  
//   //solveMaze.detectWalls(sensorDistances, row, col, facingDirection);
  
//   long currentTime = micros();
//     float deltaTime = ((float)(currentTime - previousTime)) / 1.0e6;    
//     if (deltaTime <= 0.000001) deltaTime = 0.000001;     
    
//     // Calculate derivative
//     float derivative = (error - previousError) / deltaTime;
    
//     // Calculate integral
//     integral += error * deltaTime;
//     integral = constrain(integral, -300, 300);  // Clamp integral
    
//     // Calculate PID output
//     float u = kp * error + ki * integral + kd * derivative;
//     //directionL = (u < 0) ? -1 : 1;
//     //directionR = (u < 0) ? -1 : 1;
//     directionR = directionL = 1;
//     speedL = constrain((255 - 5*u),0,255); //speed = constrain(abs(255 - 0.1*u),0,255); 
//     speedR = constrain((255 + 5*u),0,255); //speed = constrain(abs(255 - 0.1*u),0,255); 
//     // Update previous values
//     if (directionL == 1) {
//     analogWrite(25, speedL);
//     analogWrite(26, 0);
//   } else {
//     analogWrite(25, 0);
//     analogWrite(26, speedL);
//   }
//   if (directionR == 1) {
//     analogWrite(14, speedR);
//     analogWrite(27, 0);
//   } else {
//     analogWrite(14, 0);
//     analogWrite(27, speedR);
//   }
//     previousError = error;
//     previousTime = currentTime;

// }

#include <WiFiHandler.h>
#include <Arduino.h>
#include <VL6180XManagerV2.h>
#include <Floodfill.h>
#include <MotorPIDbyNJ.h>
#include <RobotNavigatorV2.h>
#include <GyroPID.h>

#define Serial OTA.webSerial

// WiFi credentials
const char* ssid = "Dialog NNJ";
const char* password = "Mixtures";
// Web server and WebSerial
WiFiHandler OTA;
int count = 0;
int last = millis();

int xshutPins[] = {32, 17, 16, 15, 4};
int sensorCorrections[] = { 6, 16, 0, 43, 26};  // mm to subtract from each sensor
VL6180XManagerV2 sensorGroup(xshutPins, 5, sensorCorrections);

Floodfill solveMaze;
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
  //Serial.begin(115200);
  Wire.begin();
  sensorGroup.begin();
  imu.begin();
  
  OTA.begin(ssid, password);

  Motors.setSensorGroup(&sensorGroup);
  
  solveMaze.setThreshhold(80);

  leftMotor.attachEncoderInterrupt(updateLeftEncoder);
  rightMotor.attachEncoderInterrupt(updateRightEncoder);

  leftMotor.setPID(0.8, 0.0, 0.04, 2);
  rightMotor.setPID(0.8, 0.0, 0.04, 2);

}

bool testMoveDone = false;
void loop() {
  ArduinoOTA.handle();

  if (!testMoveDone) {
    Motors.moveForward();
    testMoveDone = true;
  }
  // ...existing code or idle...
}

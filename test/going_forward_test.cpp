// #include <Arduino.h>
// #include <VL6180XManagerV2.h>
// #include <Floodfill.h>
// #include <MotorPIDbyNJ.h>
// #include <RobotNavigatorbyNJ.h>

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
// float previousTime;
// float previousError;
// float integral;
// float kp,ki,kd;
// float directionL, directionR;
// float speedL,speedR;

// void setup() {
//   Serial.begin(115200);
//   Wire.begin();
//   sensorGroup.begin();
  
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

//     speedL = constrain((255 - 0.1*u),0,255); //speed = constrain(abs(255 - 0.1*u),0,255); 
//     speedR = constrain((255 + 0.1*u),0,255); //speed = constrain(abs(255 - 0.1*u),0,255); 

//     Serial.print(sensorDistances[0]);
//     Serial.print(" L  R ");
//     Serial.println(sensorDistances[3]);


//     Serial.println(u);

//     Serial.print(speedL);
//     Serial.print(" L  R ");
//     Serial.println(speedR);
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

#include <Arduino.h>
#include <WiFiHandler.h>
#include <VL6180XManagerV2.h>
#include <Floodfill.h>
#include <MotorPIDbyNJ.h>
#include <RobotNavigatorbyNJ.h>
#include <Arduino.h>
#include <button.h>

button myButton(2); 

// WiFi credentials
const char* ssid = "Dialog NNJ";
const char* password = "Mixtures";
// Web server and WebSerial
WiFiHandler OTA;
int count =0;
int last = millis();

int xshutPins[] = {32, 17, 15, 4};
int sensorCorrections[] = {0, 6, 43, 26};  // mm to subtract from each sensor
VL6180XManagerV2 sensorGroup(xshutPins, 4, sensorCorrections);

Floodfill solveMaze;

MotorPIDbyNJ leftMotor(25, 26, 18, 5);
MotorPIDbyNJ rightMotor(14, 27, 19, 23);
RobotNavigatorbyNJ Motors(&leftMotor, &rightMotor);
void updateLeftEncoder() { leftMotor.updateEncoder(); }
void updateRightEncoder() { rightMotor.updateEncoder(); }

bool firstTurn = true;
bool leftWall, rightWall, frontWall = false;

int row = 15;
int col = 0;
int facingDirection = 0;

float error;
long previousTime;
float previousError;
float integral;
float kp,ki,kd;
float directionL, directionR;
float speedL,speedR;

void onSinglePress() {
  analogWrite(25, 0);
  analogWrite(26, 0);
  analogWrite(14, 0);
  analogWrite(27, 0);
  delay(100);
  while(1);
}

void setup() {
  Serial.begin(115200);
  OTA.begin(ssid, password);
  Wire.begin();
  sensorGroup.begin();

  
  myButton.setSinglePressCallback(onSinglePress);
  
  solveMaze.setThreshhold(80);

  leftMotor.attachEncoderInterrupt(updateLeftEncoder);
  rightMotor.attachEncoderInterrupt(updateRightEncoder);

  leftMotor.setPID(0.68, 0.0, 0.04, 50);
  rightMotor.setPID(0.68, 0.0, 0.04, 50);
  previousTime = micros();
  kp = 0.68;
  ki=0;
  kd=0.04;
}

void loop() {
  myButton.update();
  ArduinoOTA.handle();
  if (millis() - last > 1000) {
    OTA.webSerial.println("Hello World!");
    last = millis();
  }
  vector<int> sensorDistances = sensorGroup.readAll(); 
  error = sensorDistances[0] - sensorDistances[3];
  
  //solveMaze.detectWalls(sensorDistances, row, col, facingDirection);
  
  long currentTime = micros();
    float deltaTime = ((float)(currentTime - previousTime)) / 1.0e6;    
    if (deltaTime <= 0.000001) deltaTime = 0.000001;     
    
    // Calculate derivative
    float derivative = (error - previousError) / deltaTime;
    
    // Calculate integral
    integral += error * deltaTime;
    integral = constrain(integral, -300, 300);  // Clamp integral
    
    // Calculate PID output
    float u = kp * error + ki * integral + kd * derivative;
    //directionL = (u < 0) ? -1 : 1;
    //directionR = (u < 0) ? -1 : 1;
    directionR = directionL = 1;
    speedL = constrain((255 - 5*u),0,255); //speed = constrain(abs(255 - 0.1*u),0,255); 
    speedR = constrain((255 + 5*u),0,255); //speed = constrain(abs(255 - 0.1*u),0,255); 
    // Update previous values
    if (directionL == 1) {
    analogWrite(25, speedL);
    analogWrite(26, 0);
  } else {
    analogWrite(25, 0);
    analogWrite(26, speedL);
  }
  if (directionR == 1) {
    analogWrite(14, speedR);
    analogWrite(27, 0);
  } else {
    analogWrite(14, 0);
    analogWrite(27, speedR);
  }
    previousError = error;
    previousTime = currentTime;

}
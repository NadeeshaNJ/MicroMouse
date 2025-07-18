#include "MotorPIDbyNJ.h"
#include <Arduino.h>
// Static member initialization

MotorPIDbyNJ::MotorPIDbyNJ(int motorPin1, int motorPin2, int encPin1, int encPin2) {
    pin1 = motorPin1;
    pin2 = motorPin2;
    encoderPin1 = encPin1;
    encoderPin2 = encPin2;
    //currentTicks = 0;

    // Initialize variables
    speed = 0;
    previousError = 0;
    integral = 0;
    previousTime = 0;
    encoderValue = 0;
    lastEncoded = 0;
    direction = 1;
    isDone = false;
    target = 0;
    tolerance = 50;
}

void MotorPIDbyNJ::setPID(float kp_val, float ki_val, float kd_val, int tol) {
    kp = kp_val;
    ki = ki_val;
    kd = kd_val;
    tolerance = tol;
}
void MotorPIDbyNJ::setTarget(long targetTicks) {
    target = targetTicks;
    isDone = false;
    integral = 0;  // Reset integral term
    previousError = 0;
}

// Update encoder reading
void MotorPIDbyNJ::updateEncoder() {
    int MSB = digitalRead(encoderPin1);
    int LSB = digitalRead(encoderPin2);
    
    int encoded = (MSB << 1) | LSB;
    int sum = (lastEncoded << 2) | encoded;
    
    if (sum == 0b1101 || sum == 0b0100 || sum == 0b0010 || sum == 0b1011) encoderValue--;
    if (sum == 0b1110 || sum == 0b0111 || sum == 0b0001 || sum == 0b1000) encoderValue++;
    
    lastEncoded = encoded;
}

void MotorPIDbyNJ::calculatePID(long error) {
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
    direction = (u < 0) ? -1 : 1;
    speed = constrain(abs(u), 0, 255); 
    // Update previous values
    previousError = error;
    previousTime = currentTime;

    if (!isDone && abs(error) < tolerance) {
        analogWrite(pin1, 0);
        analogWrite(pin2, 0);
        isDone = true;
    }
    if (isDone) speed = 0;

}
// Apply motor power
void MotorPIDbyNJ::runMotor() {
  if (direction == 1) {
    analogWrite(pin1, speed);
    analogWrite(pin2, 0);
  } else {
    analogWrite(pin1, 0);
    analogWrite(pin2, speed);
  }
}

void MotorPIDbyNJ::attachEncoderInterrupt(void (*ISR)()) {
  pinMode(encoderPin1, INPUT_PULLUP);
  pinMode(encoderPin2, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(encoderPin1), ISR, CHANGE);
  attachInterrupt(digitalPinToInterrupt(encoderPin2), ISR, CHANGE);
}

void MotorPIDbyNJ::update() {  
  // Calculate error
  float error = target - encoderValue;
  // Update PID control
  calculatePID(error);
  // Update motor speed
  runMotor();
  return;
}
void MotorPIDbyNJ::reachTarget(long target) {
  calculatePID(target);
  runMotor();
  return;
}
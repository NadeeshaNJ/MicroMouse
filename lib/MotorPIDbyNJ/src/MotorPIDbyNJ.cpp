#include "MotorPIDbyNJ.h"
#include <Arduino.h>
// Static member initialization

MotorPIDbyNJ::MotorPIDbyNJ(int motorPin1, int motorPin2, int encPin1, int encPin2) {
    pin1 = motorPin1;
    pin2 = motorPin2;
    encoderPin1 = encPin1;
    encoderPin2 = encPin2;
}

void MotorPIDbyNJ::setPID(float kp_val, float ki_val, float kd_val, int tol) {
    kp = kp_val;
    ki = ki_val;
    kd = kd_val;
    tolerance = tol;
}

void MotorPIDbyNJ::updateEncoder() {
    int MSB = digitalRead(encoderPin1);
    int LSB = digitalRead(encoderPin2);
    
    int encoded = (MSB << 1) | LSB;
    int sum = (lastEncoded << 2) | encoded;
    
    if (sum == 0b1101 || sum == 0b0100 || sum == 0b0010 || sum == 0b1011) encoderValue--;
    if (sum == 0b1110 || sum == 0b0111 || sum == 0b0001 || sum == 0b1000) encoderValue++;
    
    lastEncoded = encoded;
}
long MotorPIDbyNJ::getEncoderValue() const {
    return encoderValue / metricConverter;
}

void MotorPIDbyNJ::attachEncoderInterrupt(void (*ISR)()) {
  pinMode(encoderPin1, INPUT_PULLUP);
  pinMode(encoderPin2, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(encoderPin1), ISR, CHANGE);
  attachInterrupt(digitalPinToInterrupt(encoderPin2), ISR, CHANGE);
}

int MotorPIDbyNJ::calculateEncoderPID() {
    long encoderValue = getEncoderValue();
    float error = target - encoderValue;

    long currentTime = micros();
    float deltaTime = ((float)(currentTime - previousTime)) / 1.0e6;    
    if (deltaTime <= 0.000001) deltaTime = 0.000001;     
    
    // Calculate derivative
    float derivative = (error - previousError) / deltaTime;

    // Calculate integral
    integralEncoderError += error * deltaTime;
    integralEncoderError = constrain(integralEncoderError, -300, 300);  // Clamp integral

    // Calculate PID output
    encoderPID = kp * error + ki * integralEncoderError + kd * derivative;
    encoderPID = constrain(encoderPID, -255, 255); // Clamp PID output to motor speed range
    
    previousError = error;
    previousTime = currentTime;

    return encoderPID;
}
void MotorPIDbyNJ::runMotor(int speed){
    if(speed < 0) {
        analogWrite(pin1, 0);
        analogWrite(pin2, -speed);
    } else {
        analogWrite(pin1, speed);
        analogWrite(pin2, 0);
    }
}
bool MotorPIDbyNJ::checkDone(){
    return abs(target - getEncoderValue()) < tolerance;
}
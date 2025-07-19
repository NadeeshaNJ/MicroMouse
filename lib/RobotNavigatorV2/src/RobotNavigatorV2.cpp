#include <RobotNavigatorV2.h>

RobotNavigatorV2::RobotNavigatorV2(MotorPIDbyNJ* left, MotorPIDbyNJ* right) {
    leftMotor = left;
    rightMotor = right;
}

void RobotNavigatorV2::resetEncoders() {
    leftMotor->resetEncoder();
    rightMotor->resetEncoder();
}
void RobotNavigatorV2::calculatePID(long error) {
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
  //   if (directionL == 1) {
  //   analogWrite(25, speedL);
  //   analogWrite(26, 0);
  // } else {
  //   analogWrite(25, 0);
  //   analogWrite(26, speedL);
  // }
  // if (directionR == 1) {
  //   analogWrite(14, speedR);
  //   analogWrite(27, 0);
  // } else {
  //   analogWrite(14, 0);
  //   analogWrite(27, speedR);
  // }
    previousError = error;
    previousTime = currentTime;

}
void RobotNavigatorV2::moveForward() {
    Serial.println("Moving Forward");

    resetEncoders();

    leftMotor->setDirection(1);
    rightMotor->setDirection(1);
    leftMotor->runMotor();
    rightMotor->runMotor();
}
void RobotNavigatorV2::go(int& facingDirection, int direction) {
    if (direction == -1) return;
    int diff = (facingDirection - direction + 4) % 4;
    
    if (diff == 1) turnLeft();
    else if (diff == 2) turnAround();
    else if (diff == 3) turnRight();    

    moveForward();

    updatePosition(row, col, facingDirection, direction);
}

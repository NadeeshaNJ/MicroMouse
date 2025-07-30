#include "RobotNavigatorbyNJ.h"

RobotNavigatorbyNJ::RobotNavigatorbyNJ(MotorPIDbyNJ* left, MotorPIDbyNJ* right) {
    leftMotor = left;
    rightMotor = right;
}

void RobotNavigatorbyNJ::resetEncoders() {
    leftMotor->resetEncoder();
    rightMotor->resetEncoder();
}

void RobotNavigatorbyNJ::go(int& facingDirection, int direction) {
    if (direction == -1) return;
    int diff = (facingDirection - direction + 4) % 4;
    
    if (diff == 1) turnLeft();
    else if (diff == 2) turnAround();
    else if (diff == 3) turnRight();    

    moveForward();

    updatePosition(row, col, facingDirection, direction);
}
void RobotNavigatorbyNJ::moveForward() {
    Serial.println("Moving Forward");

    resetEncoders();
    leftMotor->setDirection(1);
    rightMotor->setDirection(1);
    leftMotor->reachTarget(4000);
    rightMotor->reachTarget(4000);
}
void RobotNavigatorbyNJ::moveBackward() {
    Serial.println("Moving Backward");

    resetEncoders();
    leftMotor->setDirection(-1);
    rightMotor->setDirection(-1);
    leftMotor->reachTarget(4000);
    rightMotor->reachTarget(4000);    
}

void RobotNavigatorbyNJ::turnLeft() {
    Serial.println("Turning left");

    resetEncoders();
    leftMotor->setDirection(-1);
    rightMotor->setDirection(1);
    leftMotor->reachTarget(1800);
    rightMotor->reachTarget(1800);    
}

void RobotNavigatorbyNJ::turnRight() {
    Serial.println("Turning right");

    resetEncoders();
    leftMotor->setDirection(1);
    rightMotor->setDirection(-1);
    leftMotor->reachTarget(1800);
    rightMotor->reachTarget(1800);   
}

void RobotNavigatorbyNJ::turnAround() {
    Serial.println("Turning around");

    resetEncoders();
    leftMotor->setDirection(1);
    rightMotor->setDirection(-1);
    leftMotor->reachTarget(1800);
    rightMotor->reachTarget(1800);
}
// void RobotNavigatorbyNJ::wallFollowing(std::vector<int> sensorDistances) {
//     long currentTime = micros();
//     float deltaTime = ((float)(currentTime - previousTime)) / 1.0e6;    
//     if (deltaTime <= 0.000001) deltaTime = 0.000001;     
    
//     // Calculate derivative
//     float derivative = (error - previousError) / deltaTime;
    
//     // Calculate integral
//     integral += error * deltaTime;
//     integral = constrain(integral, -300, 300);  // Clamp integral
    
//     // Calculate PID output
//     float u = kp * error + ki * integral + kd * derivative;
//     direction = (u < 0) ? -1 : 1;
//     speed = constrain(abs(u), 0, 255); 
//     // Update previous values
//     previousError = error;
//     previousTime = currentTime;
// }

void RobotNavigatorbyNJ::updatePosition(int& row, int& col, int& facingDirection, int direction) {
    if(direction != -1){
        row += (direction == 0) ? -1 : (direction == 2) ? 1 : 0; // Update row
        col += (direction == 1) ? 1 : (direction == 3) ? -1 : 0; // Update column
    }
    facingDirection = direction;
}
bool RobotNavigatorbyNJ::isIdle() {
    return leftMotor->done() && rightMotor->done();
}

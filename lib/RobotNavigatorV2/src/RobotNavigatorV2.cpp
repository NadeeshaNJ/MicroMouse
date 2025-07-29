#include <RobotNavigatorV2.h>

RobotNavigatorV2::RobotNavigatorV2(MotorPIDbyNJ* left, MotorPIDbyNJ* right) {
    leftMotor = left;
    rightMotor = right;
}

void RobotNavigatorV2::resetEncoders() {
    leftMotor->resetEncoder();
    rightMotor->resetEncoder();
}
int RobotNavigatorV2::calculateEncoderPID() {
    long leftEncoderValue = leftMotor->getEncoderValue();
    long rightEncoderValue = rightMotor->getEncoderValue();
    float leftError = targetL - leftEncoderValue;
    float rightError = targetR - rightEncoderValue;

    /////////////////////////////////////////////////
    integralLeftEncoderError += leftError;
    diffLeftEncoderError = leftError - previousLeftError;
    integralRightEncoderError += rightError;
    diffRightEncoderError = rightError - previousRightError;

    long currentTime = micros();
    float deltaTime = ((float)(currentTime - previousTime)) / 1.0e6;    
    if (deltaTime <= 0.000001) deltaTime = 0.000001;     
    
    // Calculate derivative
    float derivativeL = (leftError - previousLeftError) / deltaTime;
    float derivativeR = (rightError - previousRightError) / deltaTime;

    // Calculate integral
    integralEncoderError += leftError * deltaTime;
    integralEncoderError = constrain(integralEncoderError, -300, 300);  // Clamp integral

    // Calculate PID output
    LeftEncoderPID = encoderKp * leftError + encoderKi * integralLeftEncoderError + encoderKd * derivativeL;
    RightEncoderPID = encoderKp * rightError + encoderKi * integralRightEncoderError + encoderKd * derivativeR;

    previousLeftError = leftError;
    previousRightError = rightError;
    previousTime = currentTime;
    /////////////////////////////////////////////////

}
void RobotNavigatorV2::reachTargets(long targetLeft, long targetRight) {
    leftMotorSpeed = leftMotor->reachTarget(targetLeft);
    rightMotorSpeed = rightMotor->reachTarget(targetRight);
    
}
void RobotNavigatorV2::moveForward() {
    Serial.println("Moving Forward");
    
    resetEncoders();
    int wallCorrection = wallPID();
    reachTargets(4000, 4000);
    speedL = constrain((leftMotorSpeed - 5*wallCorrection),0,255);
    speedR = constrain((rightMotorSpeed + 5*wallCorrection),0,255);
    leftMotor->setDirection(1);
    rightMotor->setDirection(1);
    leftMotor->setSpeed(speedL);
    rightMotor->setSpeed(speedR);
    
}
void RobotNavigatorV2::turnLeft() {
    Serial.println("Turning Left");
    resetEncoders();
    leftMotor->setDirection(1);
    rightMotor->setDirection(1);
    leftMotor->reachTarget(400);
    rightMotor->reachTarget(4000);
}
void RobotNavigatorV2::turnRight() {
    Serial.println("Turning Right");
    resetEncoders();
    leftMotor->setDirection(1);
    rightMotor->setDirection(1);
    leftMotor->reachTarget(4000);
    rightMotor->reachTarget(400);
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
 int RobotNavigatorV2::calculateWallPID(std::vector<int> sensorDistances) {
    float wallError = (sensorDistances[0] - sensorDistances[3]);
    sumWallError += wallError;
    diffWallError = wallError - previousError;
    
    long currentTime = micros();
    float deltaTime = ((float)(currentTime - previousTime)) / 1.0e6;    
    if (deltaTime <= 0.000001) deltaTime = 0.000001;     
    
    // Calculate derivative
    float derivative = (wallError - previousError) / deltaTime;
    
    // Calculate integral
    sumWallError += wallError * deltaTime;
    sumWallError = constrain(sumWallError, -300, 300);  // Clamp integral

    // Calculate PID output
    float u = wallKp * wallError + wallKi * sumWallError + wallKd * derivative;
    
    directionL = (u < 0) ? -1 : 1;
    directionR = (u < 0) ? -1 : 1;
    speedL = constrain((255 - 5*u),0,255); //speed = constrain(abs(255 - 0.1*u),0,255); 
    speedR = constrain((255 + 5*u),0,255); //speed = constrain(abs(255 - 0.1*u),0,255); 

    previousError = wallError;
    previousTime = currentTime;
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
//     //directionL = (u < 0) ? -1 : 1;
//     //directionR = (u < 0) ? -1 : 1;
//     directionR = directionL = 1;
//     speedL = constrain((255 - 5*u),0,255); //speed = constrain(abs(255 - 0.1*u),0,255); 
//     speedR = constrain((255 + 5*u),0,255); //speed = constrain(abs(255 - 0.1*u),0,255); 
//     // Update previous values
//   //   if (directionL == 1) {
//   //   analogWrite(25, speedL);
//   //   analogWrite(26, 0);
//   // } else {
//   //   analogWrite(25, 0);
//   //   analogWrite(26, speedL);
//   // }
//   // if (directionR == 1) {
//   //   analogWrite(14, speedR);
//   //   analogWrite(27, 0);
//   // } else {
//   //   analogWrite(14, 0);
//   //   analogWrite(27, speedR);
//   // }
//     previousError = error;
//     previousTime = currentTime;

}
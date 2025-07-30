#include <RobotNavigatorV2.h>

RobotNavigatorV2::RobotNavigatorV2(MotorPIDbyNJ* left, MotorPIDbyNJ* right) {
    leftMotor = left;
    rightMotor = right;
}

void RobotNavigatorV2::resetEncoders() {
    leftMotor->resetEncoder();
    rightMotor->resetEncoder();
}
void RobotNavigatorV2::getEncoderPID() {
    leftEncoderPID = leftMotor->calculateEncoderPID();
    rightEncoderPID = rightMotor->calculateEncoderPID();
}
void RobotNavigatorV2::getAnglePID() {
    imuYaw = imu->getYaw();
}
void RobotNavigatorV2::setTargets(long targetLeft, long targetRight) {
    leftMotor->setTarget(targetLeft);
    rightMotor->setTarget(targetRight);
}
int RobotNavigatorV2::calculateWallPID(std::vector<int> sensorDistances) {
    float wallError = (sensorDistances[0] - sensorDistances[4]);
    
    long currentTime = micros();
    float deltaTime = ((float)(currentTime - previousSensorTime)) / 1.0e6;    
    if (deltaTime <= 0.000001) deltaTime = 0.000001;     
    
    // Calculate derivative
    float derivative = (wallError - previousSensorError) / deltaTime;
    
    // Calculate integral
    integralWallError += wallError * deltaTime;
    integralWallError = constrain(integralWallError, -300, 300);  // Clamp integral

    // Calculate PID output
    float wallPID = wallKp * wallError + wallKi * integralWallError + wallKd * derivative;

    previousSensorError = wallError;
    previousSensorTime = currentTime;

    return wallPID;
}

void RobotNavigatorV2::moveForward() {
    Serial.println("Moving Forward");
    
    resetEncoders();

    setTargets(180,180); //180 mm

    speedL = constrain((leftEncoderPID - 5*calculateWallPID(sensorDistances)),-255,255);
    speedR = constrain((rightEncoderPID + 5*calculateWallPID(sensorDistances)),-255,255);
    if(!leftMotor->checkDone()) leftMotor->runMotor(speedL);
    if(!rightMotor->checkDone()) rightMotor->runMotor(speedR);
}
void RobotNavigatorV2::turnLeft() {
    Serial.println("Turning Left");
    resetEncoders();
    imu->setTargetYaw(imu->getYaw() + 90); // Adjust target yaw for left turn
    int baseAngleSpeed = 0;
    speedL = constrain((baseAngleSpeed - imu->calculateAnglePID()),-255,255);
    speedR = constrain((baseAngleSpeed + imu->calculateAnglePID()),-255,255);
    if(!leftMotor->checkDone()) leftMotor->runMotor(speedL);
    if(!rightMotor->checkDone()) rightMotor->runMotor(speedR);
}
void RobotNavigatorV2::turnRight() {
    Serial.println("Turning Right");
    resetEncoders();
    imu->setTargetYaw(imu->getYaw() - 90); // Adjust target yaw for right turn
    int baseAngleSpeed = 0;
    speedL = constrain((baseAngleSpeed - imu->calculateAnglePID()),-255,255);
    speedR = constrain((baseAngleSpeed + imu->calculateAnglePID()),-255,255);
    if(!leftMotor->checkDone()) leftMotor->runMotor(speedL);
    if(!rightMotor->checkDone()) rightMotor->runMotor(speedR);
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

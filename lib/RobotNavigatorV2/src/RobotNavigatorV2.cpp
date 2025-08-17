#include <RobotNavigatorV2.h>

RobotNavigatorV2::RobotNavigatorV2(MotorPIDbyNJ* left, MotorPIDbyNJ* right, GyroPID* gyro) {
    leftMotor = left;
    rightMotor = right;
    imu = gyro;
}

void RobotNavigatorV2::resetEncoders() {
    leftMotor->resetEncoder();
    rightMotor->resetEncoder();
}
void RobotNavigatorV2::getEncoderPID() {
    leftEncoderPID = leftMotor->calculateEncoderPID();
    rightEncoderPID = rightMotor->calculateEncoderPID();
}
void RobotNavigatorV2::setTargets(long targetLeft, long targetRight) {
    leftMotor->setTarget(targetLeft);
    rightMotor->setTarget(targetRight);
}

int RobotNavigatorV2::calculateWallPID() {
    sensorDistances = sensorGroup->readAll();
    if (sensorDistances.size() < 5) {
        Serial.println("Error: Not enough sensor data");
        return 0; // or handle error
    }
    // float wallError = (constrain(sensorDistances[0], 0, 100) - constrain(sensorDistances[4], 0, 100));

    // long currentTime = micros();
    // float deltaTime = ((float)(currentTime - previousSensorTime)) / 1.0e6;    
    // if (deltaTime <= 0.000001) deltaTime = 0.000001;     
    
    // // Calculate derivative
    // float derivative = (wallError - previousSensorError) / deltaTime;
    
    // // Calculate integral
    // integralWallError += wallError * deltaTime;
    // integralWallError = constrain(integralWallError, -300, 300);  // Clamp integral

    // // Calculate PID output
    // float wallPID = wallKp * wallError + wallKi * integralWallError + wallKd * derivative;

    // previousSensorError = wallError;
    // previousSensorTime = currentTime;
    // return WallPID;
    return sensorDistances[0]-sensorDistances[4];
}

void RobotNavigatorV2::moveForward() {
    if (!moving) {
        Serial.println("Moving Forward");
        resetEncoders();
        leftMotor->setTarget(192);
        rightMotor->setTarget(192);
        moving = true;
        cellDone = false;
        //unsigned long moveStartTime = millis(); // Start time for move timeout
    }
    while(moving){
        
        int currentWallPID = calculateWallPID(); //THIS IS ONLY GIVING THE ERROR NOT THE PID
        Serial.println(currentWallPID);
        getEncoderPID();
        if(leftMotor->checkDone() && rightMotor->checkDone()) {
            Serial.println("Move Done");
            cellDone = true;
            moving = false; 
        }
        else{
            speedL = constrain((leftEncoderPID - currentWallPID/5),-255,255);
            speedR = constrain((rightEncoderPID + currentWallPID/5),-255,255);
            leftMotor->runMotor(speedL);
            rightMotor->runMotor(speedR);
        }
        // if (millis() - moveStartTime > 3000) {
        //     Serial.println("Move timeout! Aborting move.");
        //     moving = false;
        //     cellDone = false;
        // }
        
        
    }
}
void RobotNavigatorV2::turnLeft() {
    if(!moving) {
        Serial.println("Turning Left");
        resetEncoders();
        imu->setTargetYaw(imu->getYaw() + 90); // Adjust target yaw for left turn
        moving = true;
        cellDone = false;
    }
    while(moving){
        int currentAnglePID = imu->calculateAnglePID();

        if(leftMotor->checkDone() && rightMotor->checkDone()) {
            Serial.println("Turn Done");
            cellDone = true;
            moving = false; 
        }
        else{
            speedL = constrain((0 - currentAnglePID),-255,255);
            speedR = constrain((0 + currentAnglePID),-255,255);
            leftMotor->runMotor(speedL);
            rightMotor->runMotor(speedR);
        }
    }
}
void RobotNavigatorV2::turnRight() {
    if(!moving) {
        Serial.println("Turning Right");
        resetEncoders();
        imu->setTargetYaw(imu->getYaw() - 90); // Adjust target yaw for right turn
        moving = true;
        cellDone = false;
    }
    while(moving){
        speedL = constrain((0 - imu->calculateAnglePID()),-255,255);
        speedR = constrain((0 + imu->calculateAnglePID()),-255,255);
        if(!leftMotor->checkDone()) leftMotor->runMotor(speedL);
        if(!rightMotor->checkDone()) rightMotor->runMotor(speedR);
        if(leftMotor->checkDone() && rightMotor->checkDone()) {
            cellDone = true; 
            moving = false; // ready for next move
        }
    }
}
void RobotNavigatorV2::turnAround() {
    if(!moving) {
        Serial.println("Turning Around");
        resetEncoders();
        imu->setTargetYaw(imu->getYaw() + 180); // Adjust target yaw for 180 turn
        moving = true;
        cellDone = false;
    }
    while(moving){
        speedL = constrain((0 - imu->calculateAnglePID()),-255,255);
        speedR = constrain((0 + imu->calculateAnglePID()),-255,255);
        if(!leftMotor->checkDone()) leftMotor->runMotor(speedL);
        if(!rightMotor->checkDone()) rightMotor->runMotor(speedR);
        if(leftMotor->checkDone() && rightMotor->checkDone()) {
            cellDone = true; 
            moving = false; // ready for next move
        }
    }
}

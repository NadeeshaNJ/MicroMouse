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
void RobotNavigatorV2::setTargets(long targetLeft, long targetRight) {
    leftMotor->setTarget(targetLeft);
    rightMotor->setTarget(targetRight);
}

void RobotNavigatorV2::getEncoderPID() {
    leftEncoderPID = leftMotor->calculateEncoderPID();
    Serial.println("Left Encoder PID: " + String(leftEncoderPID));
    rightEncoderPID = rightMotor->calculateEncoderPID();
    Serial.println("Right Encoder PID: " + String(rightEncoderPID));
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
    int difference = constrain(sensorDistances[0]-sensorDistances[4], -80, 80);
    return difference;
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
            leftMotor->runMotor(0);
            rightMotor->runMotor(0);
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
        float yaw = imu->getYaw();
        while (abs(yaw) < 0.1) { // or another threshold for "invalid" yaw
            delay(10);
            yaw = imu->getYaw();
        }
        imu->setTargetYaw(yaw - 90); // Adjust target yaw for left turn
        moving = true;
        cellDone = false;
    }
    while(moving){
        centerInCell();
        int currentAnglePID = imu->calculateAnglePID();
        Serial.println("Current Angle PID: " + String(currentAnglePID));
        Serial.println("Target Yaw:               " + String(imu->targetYaw) );

        if(imu->checkDone()) {
            leftMotor->runMotor(0);
            rightMotor->runMotor(0);
            Serial.println("Turn Done");
            cellDone = true;
            moving = false; 
        }
        else{
            speedL = constrain((0 + currentAnglePID),-255,255);
            speedR = constrain((0 - currentAnglePID),-255,255);
            leftMotor->runMotor(speedL);
            rightMotor->runMotor(speedR);
        }
    }
}
void RobotNavigatorV2::turnRight() {
    if(!moving) {
        Serial.println("Turning Right");
        resetEncoders();
        float yaw = imu->getYaw();
        while (abs(yaw) < 0.1) { // or another threshold for "invalid" yaw
            delay(10);
            yaw = imu->getYaw();
        }
        imu->setTargetYaw(yaw + 90); // Adjust target yaw for left turn
        moving = true;
        cellDone = false;
    }
    while(moving){        
        centerInCell()
        int currentAnglePID = imu->calculateAnglePID();
        Serial.println("Current Angle PID: " + String(currentAnglePID));
        Serial.println("Target Yaw:               " + String(imu->targetYaw) );

        if(imu->checkDone()) {            
            leftMotor->runMotor(0);
            rightMotor->runMotor(0);
            Serial.println("Turn Done");
            cellDone = true;
            moving = false; 
        }
        else{
            speedL = constrain((0 + currentAnglePID),-255,255);
            speedR = constrain((0 - currentAnglePID),-255,255);
            leftMotor->runMotor(speedL);
            rightMotor->runMotor(speedR);
        }
    }
}
void RobotNavigatorV2::turnAround() {
    if(!moving) {
        Serial.println("Turning Around");
        resetEncoders();
        imu->setTargetYaw(imu->getYaw() - 180); // Adjust target yaw for left turn
        moving = true;
        cellDone = false;
    }
    while(moving){        
        
        int currentAnglePID = imu->calculateAnglePID();
        Serial.println("Current Angle PID: " + String(currentAnglePID));

        if(imu->checkDone() && imu->checkDone()) {
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
int RobotNavigatorV2::centeringPID() {
    sensorDistances = sensorGroup->readAll();
    if (sensorDistances.size() < 5) {
        Serial.println("Error: Not enough sensor data");
        return 0; // or handle error
    }
    //float wallError = (constrain(sensorDistances[0], 0, 100) - constrain(sensorDistances[4], 0, 100));
    float wallError = sensorDistances[4];

    long currentTime = millis();
    float deltaTime = ((float)(currentTime - previousSensorTime)) / 1000.0;      
    
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
void RobotNavigatorV2::centerInCell() {
    // Only center if there is a wall on at least one side
    sensorDistances = sensorGroup->readAll();
    //bool leftWall = sensorDistances[0] < 80;   // adjust threshold as needed
    //bool rightWall = sensorDistances[4] < 80;  // adjust threshold as needed
    bool frontWall = sensorDistances[2] < 80;  // adjust threshold as needed

    if (!frontWall) return; // No wall to reference
    if(sensorDistances[2] > 20 && sensorDistances[2] < 40) return;

    Serial.println("Centering in cell...");
    resetEncoders();

    unsigned long startTime = millis();
    while (millis() - startTime < 600) { // Center for up to 0.6s, adjust as needed
        int wallPID = calculateWallPID();
        int correction = constrain(wallPID, -60, 60); // adjust as needed
        leftMotor->runMotor(-correction);
        rightMotor->runMotor(correction);
    }
    leftMotor->runMotor(0);
    rightMotor->runMotor(0);
    Serial.println("Centered.");
}
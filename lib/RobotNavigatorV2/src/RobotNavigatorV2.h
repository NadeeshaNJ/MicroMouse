#ifndef RobotNavigatorV2_h
#define RobotNavigatorV2_h

#include <Arduino.h>
#include <MotorPIDbyNJ.h>
#include <vector>
#include <GyroPID.h>

class RobotNavigatorV2 {
private:
    MotorPIDbyNJ* leftMotor;
    MotorPIDbyNJ* rightMotor;
    GyroPID* imu;

    int row;
    int col;
    int direction; // 0 = North, 1 = East, 2 = South, 3 = West
    int facingDirection;
    float integralEncoderError;
    float integralWallError;
    float directionL, directionR;
    float speedL, speedR;
    long previousSensorTime;
    float previousSensorError;
    float wallKp = 0.1; // Proportional gain for wall following
    float wallKi = 0.0; // Integral gain for wall following
    float wallKd = 0.05; // Derivative gain for wall following


    std::vector<int> sensorDistances;
    float imuYaw;
    float targetYaw; //in degrees
    float angleKp = 0.1; // Proportional gain for angle PID
    float angleKi = 0.0; // Integral gain for angle PID
    float angleKd = 0.05; // Derivative gain for angle PID

    
    int leftMotorSpeed;
    int rightMotorSpeed;

    int leftEncoderPID = 0;
    int rightEncoderPID = 0;

    int encoderPID;


public:
    RobotNavigatorV2(MotorPIDbyNJ* left, MotorPIDbyNJ* right);

    void resetEncoders();         // Initialize starting move
    int calculateWallPID(std::vector<int> sensorDistances);
    void getEncoderPID();
    void getAnglePID();           // Get angle PID from GyroPID
    void setTargets(long targetLeft, long targetRight); // Set targets for both motors
    void go(int& facingDirection, int direction); // Move in a given global direction
    void moveForward();  // Move forward in the current direction
    void moveBackward(); // Move backward in the current direction (not implemented)
    void turnLeft();
    void turnRight();
    void turnAround();
    
    void updatePosition(int& row, int& col, int& facingDirection, int direction);

    bool isIdle(); // checks if both motors are done
};

#endif

#ifndef RobotNavigatorV2_h
#define RobotNavigatorV2_h

#include <Arduino.h>
#include <MotorPIDbyNJ.h>
#include <vector>
#include <GyroPID.h>
#include <VL6180XManagerV2.h>

class RobotNavigatorV2 {
private:
    MotorPIDbyNJ* leftMotor;
    MotorPIDbyNJ* rightMotor;
    GyroPID* imu;

    std::array<int, 5> xshutPins = {32, 17, 16, 15, 4};
    std::array<int, 5> sensorCorrections = {0, 6, 16, 43, 26};  // mm to subtract from each sensor
    VL6180XManagerV2 sensorGroup;

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
    // Add constructor declaration for initialization
    RobotNavigatorV2(MotorPIDbyNJ* left, MotorPIDbyNJ* right, 
                     std::array<int, 5> xshutPins = {32, 17, 16, 15, 4}, 
                     std::array<int, 5> sensorCorrections = {0, 6, 16, 43, 26});
    
    bool cellDone = true; // Flag to check if the cell is done processing
    bool moving = false; // Flag to check if the robot is currently moving
    
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
};

#endif

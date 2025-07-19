#ifndef RobotNavigatorV2_h
#define RobotNavigatorV2_h

#include <Arduino.h>
#include <MotorPIDbyNJ.h>
#include <vector>

class RobotNavigatorV2 {
private:
    MotorPIDbyNJ* leftMotor;
    MotorPIDbyNJ* rightMotor;

    int row;
    int col;
    int direction; // 0 = North, 1 = East, 2 = South, 3 = West
    int facingDirection;
    float integral;
    float kp, ki, kd;
    float directionL, directionR;
    float speedL, speedR;
    long previousTime;
    float previousError;
    std::vector<int> sensorDistances;
public:
    RobotNavigatorV2(MotorPIDbyNJ* left, MotorPIDbyNJ* right);

    void resetEncoders();         // Initialize starting move
    void calculatePID(long error);

    void go(int& facingDirection, int direction); // Move in a given global direction
    void moveForward();  // Move forward in the current direction
    void moveBackward(); // Move backward in the current direction (not implemented)
    void turnLeft();
    void turnRight();
    void turnAround();

    void wallFollowing(std::vector<int> sensorDistances);


    void updatePosition(int& row, int& col, int& facingDirection, int direction);

    bool isIdle(); // checks if both motors are done
};

#endif

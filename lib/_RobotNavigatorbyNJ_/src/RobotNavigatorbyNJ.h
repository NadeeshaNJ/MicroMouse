#ifndef RobotNavigatorbyNJ_h
#define RobotNavigatorbyNJ_h

#include <Arduino.h>
#include <MotorPIDbyNJ.h>
#include <vector>

class RobotNavigatorbyNJ {
private:
    MotorPIDbyNJ* leftMotor;
    MotorPIDbyNJ* rightMotor;

    int row;
    int col;
    int direction; // 0 = North, 1 = East, 2 = South, 3 = West
    int facingDirection;
    std::vector<int> sensorDistances;
public:
    RobotNavigatorbyNJ(MotorPIDbyNJ* left, MotorPIDbyNJ* right);

    void resetEncoders();         // Initialize starting move
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

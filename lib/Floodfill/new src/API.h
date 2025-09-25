#pragma once

#include <string>
#include <vector>
#include <VL6180XManagerV2.h>
#include <RobotNavigatorV2.h>
using namespace std;

class API {
private:
    VL6180XManagerV2* sensorGroup;
    RobotNavigatorV2* robotNavigator;
    int wall_threshhold;
    int row, col;
public:

    static int mazeWidth();
    static int mazeHeight();

    bool wallFront();
    bool wallRight();
    bool wallLeft();

    void moveForward();
    void turnRight();
    void turnLeft();
    static bool wasReset();
    static void ackReset();

};

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
    vector<int> sensorDistances;
    int wall_threshhold;
    int row, col;
public:

    static int mazeWidth();
    static int mazeHeight();

    static bool wallFront();
    static bool wallRight();
    static bool wallLeft();

    static void moveForward();
    static void turnRight();
    static void turnLeft();

    static void setWall(int x, int y, char direction);
    static void clearWall(int x, int y, char direction);

    static void setColor(int x, int y, char color);
    static void clearColor(int x, int y);
    static void clearAllColor();

    static void setText(int x, int y, const std::string& text);
    static void clearText(int x, int y);
    static void clearAllText();

    static bool wasReset();
    static void ackReset();

};

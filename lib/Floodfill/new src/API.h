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

    void setWall(int x, int y, char direction);
    void clearWall(int x, int y, char direction);

    void setColor(int x, int y, char color);
    void clearColor(int x, int y);
    void clearAllColor();

    static void setText(int x, int y, const std::string& text);
    static void clearText(int x, int y);
    static void clearAllText();

    static bool wasReset();
    static void ackReset();

};

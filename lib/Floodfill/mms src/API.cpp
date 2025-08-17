#include "API.h"

#include <cstdlib>
#include <iostream>

int API::mazeWidth() {
    std::cout << "mazeWidth" << std::endl;
    std::string response;
    std::cin >> response;
    return atoi(response.c_str());
}

int API::mazeHeight() {
    std::cout << "mazeHeight" << std::endl;
    std::string response;
    std::cin >> response;
    return atoi(response.c_str());
}

bool API::wallFront() {
    auto sensorDistances = sensorGroup->readAll();
    if(sensorDistances[2] > 0 && sensorDistances[2] < wall_threshhold) { // North Wall
        return true;
    }
    return false;
}

bool API::wallRight() {
    auto sensorDistances = sensorGroup->readAll();
    if(sensorDistances[4] > 0 && sensorDistances[4] < wall_threshhold) {
        return true;
    }
    return false;
}

bool API::wallLeft() {
    auto sensorDistances = sensorGroup->readAll();
    if(sensorDistances[0] > 0 && sensorDistances[0] < wall_threshhold) {
        return true;
    }
    return false;
}

void API::moveForward() {
    robotNavigator->moveForward();
}

void API::turnRight() {
    robotNavigator->turnRight();
}

void API::turnLeft() {
    robotNavigator->turnLeft();
}

void API::setWall(int x, int y, char direction) {
    std::cout << "setWall " << x << " " << y << " " << direction << std::endl;
}

void API::clearWall(int x, int y, char direction) {
    std::cout << "clearWall " << x << " " << y << " " << direction << std::endl;
}

void API::setColor(int x, int y, char color) {
    std::cout << "setColor " << x << " " << y << " " << color << std::endl;
}

void API::clearColor(int x, int y) {
    std::cout << "clearColor " << x << " " << y << std::endl;
}

void API::clearAllColor() {
    std::cout << "clearAllColor" << std::endl;
}

void API::setText(int x, int y, const std::string& text) {
    std::cout << "setText " << x << " " << y << " " << text << std::endl;
}

void API::clearText(int x, int y) {
    std::cout << "clearText " << x << " " << y << std::endl;
}

void API::clearAllText() {
    std::cout << "clearAllText" << std::endl;
}

bool API::wasReset() {
    std::cout << "wasReset" << std::endl;
    std::string response;
    std::cin >> response;
    return response == "true";
}

void API::ackReset() {
    std::cout << "ackReset" << std::endl;
    std::string ack;
    std::cin >> ack;
}

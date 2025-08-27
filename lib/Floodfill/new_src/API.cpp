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
    if (sensorDistances[2] > 0 && sensorDistances[2] < wall_threshhold) { // North Wall
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

#ifndef Floodfill_h
#define Floodfill_h
#include <queue>
#include <array>
#include <utility>
#include <Arduino.h>
using namespace std;

struct grid {
        array<array<pair<int, int>, 16>, 16> horizontal_walls; // (left,right)
        array<array<pair<int, int>, 16>, 16> vertical_walls; // (up,down)
        array<array<int, 16>, 16> manhattan_distances = {}; // assigned false at the beginning
        array<array<int, 16>, 16> reverse_manhattan_distances = {}; // assigned false at the beginning
        array<array<bool, 16>, 16> visited{};
    };

class Floodfill {
private:   
    vector<int> sensorDistances;
    int wall_threshhold = 120;
    int direction;
    int row, col;
    bool front, left, right;    
    
public:
    grid maze;
    // void setThreshhold(int threshhold) {wall_threshhold = threshhold;}
    bool RobotDone = false;

    void setWall(int row, int col, int direction);
    void detectWalls(vector<int> sensorDistances, int row, int col, int direction);    
    void floodfill(std::array<std::array<int, 16>, 16> &manhattan_distances, int goalRow, int goalCol);
    bool hasWall(int row, int col, int dir);
    int getNextMove(array<array<int, 16>, 16> &dist,int row, int col); //next row column will also be automatically update from this function
    int reverse_getNextMove(int row, int col); //next row column will also be automatically update from this function
    bool atGoal(int row, int col); //check if robot came to the goal

    void floodfillToStart();
};
//both detectWall and updateWall do the same thing but with different parameters
#endif

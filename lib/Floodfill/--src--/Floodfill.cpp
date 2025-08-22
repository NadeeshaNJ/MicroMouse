#include "Floodfill.h"
#include <queue>
#include <utility>
using namespace std;

bool Floodfill::atGoal(int row, int col) {
    return (row == 7 || row == 8) && (col == 7 || col == 8);
}
void Floodfill::setWall(int row, int col, int direction) { //direction: 0 = north, 1 = east, 2 = south, 3 = west
    if(direction == 0) {
        maze.vertical_walls[row][col].first = 1; // North wall
        if(row+1 < 16) maze.vertical_walls[row + 1][col].second = 1; // South wall of the cell above
    }
    if(direction == 1) {
        maze.horizontal_walls[row][col].second = 1; // East wall
        if(col + 1 < 16) maze.horizontal_walls[row][col + 1].first = 1; // West wall of the cell to the right
    }
    if(direction == 2) {
        maze.vertical_walls[row][col].second = 1; // South wall
        if(row - 1 >= 0) maze.vertical_walls[row - 1][col].first = 1; // North wall of the cell below
    }
    if(direction == 3) {
        maze.horizontal_walls[row][col].first = 1; // West wall
        if(col - 1 >= 0) maze.horizontal_walls[row][col - 1].second = 1; // East wall of the cell to the left
    }
}
void Floodfill::detectWalls(vector<int> sensorDistances, int row, int col, int direction){
    if(row < 0 || row >= 16 || col < 0 || col >= 16) return; // out of bounds
    if(sensorDistances[2] > 0 && sensorDistances[2] < wall_threshhold) { // North Wall
        setWall(row, col, direction);
        Serial.print("Wall detected at ");
        Serial.print(direction);
        Serial.print(" with distance: ");
        Serial.println(sensorDistances[2]);
    }
    if(sensorDistances[0] > 0 && sensorDistances[0] < wall_threshhold) {
        setWall(row, col, (direction + 3) % 4);
        Serial.print("Wall detected at ");
        Serial.print((direction + 3) % 4);
        Serial.print(" with distance: ");
        Serial.println(sensorDistances[0]);
    }
    //if(sensorDistances[1] < wall_threshhold) setWall(row, col, direction);
    if(sensorDistances[4] > 0 && sensorDistances[4] < wall_threshhold) {
        setWall(row, col, (direction + 1) % 4);
        Serial.print("Wall detected at ");
        Serial.print((direction + 1) % 4);
        Serial.print(" with distance: ");
        Serial.println(sensorDistances[4]);
    }
}

void Floodfill::floodfill() {
    queue<pair<int, int>> q;    

    for (auto& row : maze.manhattan_distances)
        row.fill(255); // Re-initialize all distances to a large number

    maze.manhattan_distances[8][8] = 0; //row,column
    maze.manhattan_distances[8][7] = 0;
    maze.manhattan_distances[7][8] = 0;
    maze.manhattan_distances[7][7] = 0; // Goal
    
    q.push({8, 8}); q.push({8, 7}); q.push({7, 8}); q.push({7, 7});       

    array<array<bool, 16>, 16> reached = {};
    reached[8][8] = reached[8][7] = reached[7][8] = reached[7][7] = true; // Marking the goal cells as reached
    
    // BFS to fill the manhattan distances
    while(!q.empty()){
        pair<int,int> cell = q.front();
        q.pop();

        if(cell.first > 15 || cell.second > 15 || cell.first < 0 || cell.second < 0) continue; // out of bounds

        if(cell.second > 0 && !maze.horizontal_walls[cell.first][cell.second].first && !reached[cell.first][cell.second - 1]) {// No wall to the left
            maze.manhattan_distances[cell.first][cell.second - 1] = maze.manhattan_distances[cell.first][cell.second] + 1;
            q.push({cell.first, cell.second - 1});
            reached[cell.first][cell.second - 1] = true; // Marking the cell as reached            
        }
        if(cell.first < 15 && !maze.vertical_walls[cell.first][cell.second].first && !reached[cell.first + 1][cell.second]) {// No wall above
            maze.manhattan_distances[cell.first + 1][cell.second] = maze.manhattan_distances[cell.first][cell.second] + 1;
            q.push({cell.first + 1, cell.second});
            reached[cell.first + 1][cell.second] = true; // Marking the cell as reached
        }
        if(cell.second < 15 && !maze.horizontal_walls[cell.first][cell.second].second && !reached[cell.first][cell.second + 1]) {// No wall to the right
            maze.manhattan_distances[cell.first][cell.second + 1] = maze.manhattan_distances[cell.first][cell.second] + 1;
            q.push({cell.first, cell.second + 1});
            reached[cell.first][cell.second + 1] = true; // Marking the cell as reached
        }
        if(cell.first > 0 && !maze.vertical_walls[cell.first][cell.second].second && !reached[cell.first - 1][cell.second]) {// No wall below
            maze.manhattan_distances[cell.first - 1][cell.second] = maze.manhattan_distances[cell.first][cell.second] + 1;
            q.push({cell.first - 1, cell.second});
            reached[cell.first - 1][cell.second] = true; // Marking the cell as reached
        }
    }

}
bool Floodfill::hasWall(int row, int col, int dir) {
    if(dir == 0) return maze.vertical_walls[row][col].first; // North wall
    if(dir == 1) return maze.horizontal_walls[row][col].second; // East wall
    if(dir == 2) return maze.vertical_walls[row][col].second; // South wall
    if(dir == 3) return maze.horizontal_walls[row][col].first; // West wall
    return false; // No walls
}

int Floodfill::getNextMove(int row, int col /*int direction //for optimize movement(for later improvement)*/) {
    int minDist = 255;
    int bestDirection = -1;

    // int nextRow = row;
    // int nextCol = col;

    if(!hasWall(row, col, 0) && maze.manhattan_distances[row + 1][col] < minDist) {
        minDist = maze.manhattan_distances[row + 1][col];
        bestDirection = 0;
    }
    if(!hasWall(row, col, 1) && maze.manhattan_distances[row][col + 1] < minDist) {
        minDist = maze.manhattan_distances[row][col + 1];
        bestDirection = 1;
    }
    if(!hasWall(row, col, 2) && maze.manhattan_distances[row - 1][col] < minDist) {
        minDist = maze.manhattan_distances[row - 1][col];
        bestDirection = 2;
    }
    if(!hasWall(row, col, 3) && maze.manhattan_distances[row][col - 1] < minDist) {
        minDist = maze.manhattan_distances[row][col - 1];
        bestDirection = 3;
    }

    Serial.print("Let's move to the direction coded as ");
    Serial.println(bestDirection);

    return bestDirection; // Return the best direction to move based on the minimum distance
}

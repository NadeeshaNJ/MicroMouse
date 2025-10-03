// Micromouse floodfill maze-solving algorithm (on-robot variant)
// Removes MMS Simulator API usage and main(); integrates with robot by:
// 1) Detecting walls from sensor distances (threshold-based)
// 2) Providing floodfill() and getNextMove() you can call from your code
#include <array>
#include <queue>
#include <utility>
#include <vector>
#include "Floodfill.h"

using namespace std;

// Extern primitives to be implemented in main.cpp
extern void moveForward();
extern void turnLeft();
extern void turnRight();
extern std::vector<int> getDistances(); // returns VL6180X distances: [left,...,front,...,right]

enum RunMode { MODE_SEARCH_ONLY = 1, MODE_DOUBLE = 2, MODE_QUICK = 3 };
static RunMode runMode = MODE_DOUBLE;
// Action and robot pose/state (kept here so main.cpp stays minimal)
enum Action { FORWARD, LEFT, RIGHT, IDLE, AROUND };
static int curRow = 0, curCol = 0, curDir = 0;  // 0=N,1=E,2=S,3=W
static bool reachedCenter = false;
static bool quickRun = false; // after returning to start, perform fast run using known map only
static Floodfill floodfill; // solver instance

extern "C" void ffSetRunMode(int mode) {
  if (mode == 1) { runMode = MODE_SEARCH_ONLY; quickRun = false; reachedCenter = false; }
  else if (mode == 2) { runMode = MODE_DOUBLE; quickRun = false; }
  else if (mode == 3) { runMode = MODE_QUICK; quickRun = true; }
}

extern "C" bool ffAtCenter() {
  return (curRow == 7 || curRow == 8) && (curCol == 7 || curCol == 8);
}

extern "C" bool ffAtStart() {
  return (curRow == 0 && curCol == 0);
} 

bool Floodfill::atGoal(int row, int col) {
    return (row == 7 || row == 8) && (col == 7 || col == 8);
}

void Floodfill::setWall(int row, int col, int direction) {
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
    // On-robot: no external API call here
}
void Floodfill::detectWalls(vector<int> sensorDistances, int row, int col, int direction){
    if(row < 0 || row >= 16 || col < 0 || col >= 16) return; // out of bounds
    if (sensorDistances[2] >= 0 && sensorDistances[2] < wall_threshhold) { //front
        setWall(row, col, direction);
    }
    if (sensorDistances[0] >= 0 && sensorDistances[0] < wall_threshhold) { //left
        setWall(row, col, (direction + 3) % 4);
    } 
    if (sensorDistances[4] >= 0 && sensorDistances[4] < wall_threshhold) { //right
        setWall(row, col, (direction + 1) % 4);
    }
}


bool Floodfill::hasWall(int row, int col, int dir) {
    if(dir == 0) return maze.vertical_walls[row][col].first; // North wall
    if(dir == 1) return maze.horizontal_walls[row][col].second; // East wall
    if(dir == 2) return maze.vertical_walls[row][col].second; // South wall
    if(dir == 3) return maze.horizontal_walls[row][col].first; // West wall
    return false; // No walls
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

    for (int r = 0; r < 16; ++r) {
        for (int c = 0; c < 16; ++c) {
            int dist = maze.manhattan_distances[r][c];
        }
    }
}

// Compute distances to center but ONLY through visited cells (for quick run)
void Floodfill::final_floodfill() {
    queue<pair<int, int>> q;    

    for (auto& row : maze.manhattan_distances)
        row.fill(255); // Re-initialize all distances to a large number

    // Center goals
    maze.manhattan_distances[8][8] = 0; //row,column
    maze.manhattan_distances[8][7] = 0;
    maze.manhattan_distances[7][8] = 0;
    maze.manhattan_distances[7][7] = 0; // Goal

    q.push({8, 8}); q.push({8, 7}); q.push({7, 8}); q.push({7, 7});       

    array<array<bool, 16>, 16> reached = {};
    reached[8][8] = reached[8][7] = reached[7][8] = reached[7][7] = true; // Marking the goal cells as reached
    
    // BFS constrained to already visited cells only
    while(!q.empty()){
        pair<int,int> cell = q.front();
        q.pop();
        
        int row = cell.first;
        int col = cell.second;

        if (row < 0 || row > 15 || col < 0 || col > 15) continue;

        // WEST
        if (col > 0 && !maze.horizontal_walls[row][col].first && maze.visited[row][col - 1] && !reached[row][col - 1]) {
            maze.manhattan_distances[row][col - 1] = maze.manhattan_distances[row][col] + 1;
            q.push({row, col - 1});
            reached[row][col - 1] = true;
        }

        // NORTH
        if (row < 15 && !maze.vertical_walls[row][col].first && maze.visited[row + 1][col] && !reached[row + 1][col]) {
            maze.manhattan_distances[row + 1][col] = maze.manhattan_distances[row][col] + 1;
            q.push({row + 1, col});
            reached[row + 1][col] = true;
        }

        // EAST
        if (col < 15 && !maze.horizontal_walls[row][col].second && maze.visited[row][col + 1] && !reached[row][col + 1]) {
            maze.manhattan_distances[row][col + 1] = maze.manhattan_distances[row][col] + 1;
            q.push({row, col + 1});
            reached[row][col + 1] = true;
        }

        // SOUTH
        if (row > 0 && !maze.vertical_walls[row][col].second && maze.visited[row - 1][col] && !reached[row - 1][col]) {
            maze.manhattan_distances[row - 1][col] = maze.manhattan_distances[row][col] + 1;
            q.push({row - 1, col});
            reached[row - 1][col] = true;
        }
    }
}
// Compute distances from START (0,0) for backtracking to start
void Floodfill::floodfillToStart() {
    std::queue<std::pair<int, int>> q;

    // Reset reverse distances to large value
    for (auto& row : maze.reverse_manhattan_distances) row.fill(255);

    // Goal for reverse is the start cell
    maze.reverse_manhattan_distances[0][0] = 0;
    q.push({0, 0});

    std::array<std::array<bool, 16>, 16> reached = {};
    reached[0][0] = true;

    while (!q.empty()) {
        auto cell = q.front(); q.pop();
        int r = cell.first, c = cell.second;
        if (r < 0 || r > 15 || c < 0 || c > 15) continue;

        // WEST (c-1) if no WEST wall
        if (c > 0 && !maze.horizontal_walls[r][c].first && !reached[r][c - 1]) {
            maze.reverse_manhattan_distances[r][c - 1] = maze.reverse_manhattan_distances[r][c] + 1;
            q.push({r, c - 1});
            reached[r][c - 1] = true;
        }
        // NORTH (r+1) if no NORTH wall
        if (r < 15 && !maze.vertical_walls[r][c].first && !reached[r + 1][c]) {
            maze.reverse_manhattan_distances[r + 1][c] = maze.reverse_manhattan_distances[r][c] + 1;
            q.push({r + 1, c});
            reached[r + 1][c] = true;
        }
        // EAST (c+1) if no EAST wall
        if (c < 15 && !maze.horizontal_walls[r][c].second && !reached[r][c + 1]) {
            maze.reverse_manhattan_distances[r][c + 1] = maze.reverse_manhattan_distances[r][c] + 1;
            q.push({r, c + 1});
            reached[r][c + 1] = true;
        }
        // SOUTH (r-1) if no SOUTH wall
        if (r > 0 && !maze.vertical_walls[r][c].second && !reached[r - 1][c]) {
            maze.reverse_manhattan_distances[r - 1][c] = maze.reverse_manhattan_distances[r][c] + 1;
            q.push({r - 1, c});
            reached[r - 1][c] = true;
        }
    }
}

// Choose next move by minimizing reverse distances (to get back to start)
int Floodfill::reverse_getNextMove(int row, int col) {
    int minDist = 255;
    int bestDirection = -1;

    for (int dir = 0; dir < 4; ++dir) {
        int r = row, c = col;

        if (dir == 0 && !hasWall(row, col, 0)) r++;        // NORTH
        else if (dir == 1 && !hasWall(row, col, 1)) c++;   // EAST
        else if (dir == 2 && !hasWall(row, col, 2)) r--;   // SOUTH
        else if (dir == 3 && !hasWall(row, col, 3)) c--;   // WEST
        else continue;

        if (r < 0 || r >= 16 || c < 0 || c >= 16) continue;

        int dist = maze.reverse_manhattan_distances[r][c];

        // tie-breaker: prefer continuing in current direction
        if (dist < minDist || (dist == minDist && dir == curDir)) {
            minDist = dist;
            bestDirection = dir;
        }
    }

    return bestDirection;
}

int Floodfill::getNextMove(int row, int col /*...*/) {
    int minDist = 255;
    int bestDirection = -1;

    for (int dir = 0; dir < 4; ++dir) {
        int r = row, c = col;

        if (dir == 0 && !hasWall(row, col, 0)) r++;
        else if (dir == 1 && !hasWall(row, col, 1)) c++;
        else if (dir == 2 && !hasWall(row, col, 2)) r--;
        else if (dir == 3 && !hasWall(row, col, 3)) c--;
        else continue;

        if (r < 0 || r >= 16 || c < 0 || c >= 16) continue;

        int dist = maze.manhattan_distances[r][c];

        // Prefer lower distance; on tie prefer current direction
        if (dist < minDist || (dist == minDist && dir == curDir)) {
            minDist = dist;
            bestDirection = dir;
        }
    }

    return bestDirection;
}

// --- Solver helpers and main loop (preserve your structure) ---
static Action rotateTo(int newDir) {
    int diff = (newDir - curDir + 4) % 4;
    if (diff == 0) return FORWARD;
    if (diff == 1) return RIGHT;
    if (diff == 3) return LEFT;
    if (diff == 2) return AROUND;
    return IDLE;
}

void moveForwardUpdatePos() {
    if (curDir == 0) curRow++;
    if (curDir == 1) curCol++;
    if (curDir == 2) curRow--;
    if (curDir == 3) curCol--;

}

static Action solver() {
  // In quick mode, do NOT update walls; otherwise keep sensing
  if (!quickRun) {
    std::vector<int> distances = getDistances();
    floodfill.detectWalls(distances, curRow, curCol, curDir);
  }

  // Only flip center flag automatically in modes that want it
  if (runMode != MODE_SEARCH_ONLY && !reachedCenter && floodfill.atGoal(curRow, curCol)) {
    reachedCenter = true;
  }

  // If we got back to start after reaching center, auto-switch to quick run unless user forced a mode
  if (runMode == MODE_DOUBLE && reachedCenter && curRow == 0 && curCol == 0) {
    quickRun = true;
  }

  int bestDir = -1;
  if (runMode == MODE_QUICK || quickRun) {
    floodfill.final_floodfill();
    bestDir = floodfill.getNextMove(curRow, curCol);
  } else if (runMode == MODE_DOUBLE && reachedCenter) {
    floodfill.floodfillToStart();
    floodfill.floodfill();
    bestDir = floodfill.reverse_getNextMove(curRow, curCol);
  } else {
    floodfill.floodfill();
    bestDir = floodfill.getNextMove(curRow, curCol);
  }

  // If user chose search-only and we are at center, stop (report IDLE)
  if (runMode == MODE_SEARCH_ONLY && floodfill.atGoal(curRow, curCol)) {
    return IDLE;
  }

  return rotateTo(bestDir);
}

// Perform a single floodfill decision + action; call repeatedly from loop()
void runFloodfillStep() {
    // if (floodfill.atGoal(curRow, curCol)) return; // optional early exit

    Action action = solver();
    
    if (action == FORWARD) {
        moveForward();
        floodfill.maze.visited[curRow][curCol] = true; // requires visited[] in grid
        moveForwardUpdatePos(); // Update pose used by solver
    } else if (action == LEFT) {
        turnLeft();
        curDir = (curDir + 3) % 4;
    } else if (action == RIGHT) {
        turnRight();
        curDir = (curDir + 1) % 4;
    } else if (action == AROUND) {
        // keep structure identical to your sample
        turnRight();
        turnRight();
        curDir = (curDir + 2) % 4;
    } else {
        // IDLE or no valid move; nothing to do this tick
        return;
    }
}

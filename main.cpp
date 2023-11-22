#include <iostream>
#include <vector>
#include <set>
#include <cmath>
#include <algorithm>
#include <limits>

using namespace std;

const int INF = numeric_limits<int>::max();

struct Node {
    int x, y, z;
    int g, h;

    Node(int x, int y, int z, int g, int h) : x(x), y(y), z(z), g(g), h(h) {}

    bool operator<(const Node& other) const {
        return tie(g, h, x, y, z) < tie(other.g, other.h, other.x, other.y, other.z);
    }
};


int calculateHeuristic(int x, int y, int z, int goalX, int goalY, int goalZ) {
    return abs(pow((x - goalX),2) + pow((y - goalY),2) + pow((z - goalZ),2));
}


void printGrid(const vector<vector<vector<int>>>& grid) {
    for (const auto& layer : grid) {
        for (const auto& row : layer) {
            for (const int& cell : row) {
                if (cell == 0) {
                    cout << ". "; // open path
                } else if (cell == 1) {
                    cout << "# "; // obstacle
                } else if (cell == 2) {
                    cout << "* "; // path
                }
            }
            cout << endl;
        }
        cout << "-----" << endl;
    }
}

void astar(vector<vector<vector<int>>>& grid, int startX, int startY, int startZ, int goalX, int goalY, int goalZ) {
    int layers = grid.size();
    int rows = grid[0].size();
    int cols = grid[0][0].size();

    set<Node> openSet;
    vector<vector<vector<bool>>> closedSet(layers, vector<vector<bool>>(rows, vector<bool>(cols, false)));
    vector<vector<vector<Node>>> cameFrom(layers, vector<vector<Node>>(rows, vector<Node>(cols, Node(-1, -1, -1, INF, INF))));

    Node start(startX, startY, startZ, 0, calculateHeuristic(startX, startY, startZ, goalX, goalY, goalZ));
    openSet.insert(start);

    while (!openSet.empty()) {
        Node current = *openSet.begin();
        openSet.erase(openSet.begin());
        if (current.x == goalX && current.y == goalY && current.z == goalZ) {
            while (current.x != -1 && current.y != -1 && current.z != -1) {
                grid[current.x][current.y][current.z] = 2;
                current = cameFrom[current.x][current.y][current.z];
            }
            grid[startX][startY][startZ] = 2;
            grid[goalX][goalY][goalZ] = 2;
            cout << "Path found:" << endl;
            printGrid(grid);
            return;
        }
        closedSet[current.x][current.y][current.z] = true;
//remove last 8 tuples in this vector to prevent diagonal movement
        vector<tuple<int, int, int>> moves = {{-1, 0, 0}, {1, 0, 0}, {0, -1, 0}, {0, 1, 0}, {0, 0, -1}, {0, 0, 1}, {1, 1, 0}, {1, -1, 0}, {-1, 1, 0},{-1,-1, 0}, {1, 0, 1}, {1, 0, -1}, {-1, 0, 1}, {-1, 0, -1}, {0, 1, 1}, {0, -1, 1}, {0, 1, -1}, {0, -1, -1}};
        for (const auto& move : moves) {
            int newX = current.x + get<0>(move);
            int newY = current.y + get<1>(move);
            int newZ = current.z + get<2>(move);

            if (newX >= 0 && newX < layers && newY >= 0 && newY < rows && newZ >= 0 && newZ < cols &&
            grid[newX][newY][newZ] != 1 && !closedSet[newX][newY][newZ]) {
                int newG = current.g + 1;
                int newH = calculateHeuristic(newX, newY, newZ, goalX, goalY, goalZ);
                Node newNode(newX, newY, newZ, newG, newH);
                if (newG < cameFrom[newX][newY][newZ].g) {
                    openSet.insert(newNode);
                    cameFrom[newX][newY][newZ] = current;
                }
            }
        }
    }
    cout << "No path found." << endl;
}

int main() {
    vector<vector<vector<int>>> grid = {
                    {{0, 0, 1, 0, 0},
                    {0, 0, 1, 0, 0},
                    {0, 1, 1, 1, 0},
                    {0, 0, 1, 0, 0},
                    {0, 0, 1, 0, 0}},

                    {{0, 0, 1, 0, 0},
                    {0, 0, 1, 0, 0},
                    {0, 1, 1, 1, 0},
                    {0, 0, 1, 0, 0},
                    {0, 0, 1, 0, 0}},

                    {{0, 0, 0, 0, 0},
                    {0, 0, 1, 0, 0},
                    {0, 1, 1, 1, 0},
                    {0, 0, 1, 0, 0},
                    {0, 0, 1, 0, 0}}
    };
    clock_t start = clock();
    int startX = 0;
    int startY = 0;
    int startZ = 0;
    int goalX = 2;
    int goalY = 4;
    int goalZ = 4;

    cout << "Original Grid:" << endl;
    printGrid(grid);

    astar(grid, startX, startY, startZ, goalX, goalY, goalZ);


    return 0;
}

#include <iostream>
#include <vector>
#include <cmath>
#include <random>
#include <chrono>
#include <limits>

// Define a 2D point
struct Point {
    double x, y;
};

// Check for collisions with obstacles
bool isCollisionFree(Point p, const std::vector<Point>& obstacles, double radius) {
    for (const auto& obs : obstacles) {
        double dist = std::sqrt(std::pow(p.x - obs.x, 2) + std::pow(p.y - obs.y, 2));
        if (dist < radius) return false;
    }
    return true;
}

int main() {
    // Start and goal points
    Point start = {0.0, 0.0};
    Point goal = {9.0, 9.0};

    // Obstacles
    std::vector<Point> obstacles;
    obstacles.push_back({4.0, 4.0});
    obstacles.push_back({5.0, 5.0});
    obstacles.push_back({6.0, 6.0});

    // RRT tree
    std::vector<Point> tree;
    tree.push_back(start);

    // Parameters
    int maxIterations = 1000;
    double stepSize = 1.0;

    // Replace atomic bool with regular bool
    bool goalReached = false;
    int goalIterationNumber = -1;

    auto start_time = std::chrono::high_resolution_clock::now();

    // Initialize single random generator
    std::random_device rd;
    std::mt19937 gen(rd());
    std::uniform_real_distribution<> dis(0.0, 10.0);

    // Remove OpenMP pragmas and simplify the loop
    for (int i = 0; i < maxIterations; ++i) {
        if (goalReached) continue;
        
        Point randomPoint = {dis(gen), dis(gen)};

        // Find the nearest point in the tree
        Point nearest = tree[0];
        double minDist = std::numeric_limits<double>::max();
        
        for (const auto& node : tree) {
            double dist = std::sqrt(std::pow(randomPoint.x - node.x, 2) + 
                                  std::pow(randomPoint.y - node.y, 2));
            if (dist < minDist) {
                minDist = dist;
                nearest = node;
            }
        }

           Point newPoint = {
                nearest.x + stepSize * (randomPoint.x - nearest.x) / minDist,
                nearest.y + stepSize * (randomPoint.y - nearest.y) / minDist
            };

            // Check for collisions
            if (isCollisionFree(newPoint, obstacles, 1.0)) {
                tree.push_back(newPoint);
                
                // Check if goal is reached
                if (!goalReached && 
                    std::sqrt(std::pow(newPoint.x - goal.x, 2) + 
                            std::pow(newPoint.y - goal.y, 2)) < 1.0) {
                    goalReached = true;
                    goalIterationNumber = i;
                }
        }
    }

    auto end_time = std::chrono::high_resolution_clock::now();
    auto duration = std::chrono::duration_cast<std::chrono::microseconds>(end_time - start_time);
    std::cout << "Execution time: " << duration.count() << " milliseconds" << std::endl;

    if (goalReached) {
        std::cout << "Goal reached at iteration " << goalIterationNumber << "!" << std::endl;
    } else {
        std::cout << "Goal not reached within maximum iterations." << std::endl;
    }

    std::cout << "RRT tree size: " << tree.size() << std::endl;
}
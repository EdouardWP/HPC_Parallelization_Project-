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
    Point goal = {297.0, 297.0};

    // Generate a large number of random obstacles
    std::vector<Point> obstacles;
    int numObstacles = 1500;
    std::random_device rd;
    std::mt19937 gen(rd());
    std::uniform_real_distribution<> dis(0.0, 300.0);

    for (int i = 0; i < numObstacles; ++i) {
        obstacles.push_back({dis(gen), dis(gen)});
    }

    // Global RRT tree
    std::vector<Point> tree;
    tree.push_back(start);

    // Parameters
    int maxIterations = 30000;
    double stepSize = 3.0;

    bool goalReached = false;
    int goalIterationNumber = -1;

    auto start_time = std::chrono::high_resolution_clock::now();

    for (int i = 0; i < maxIterations; ++i) {
        if (goalReached) break; // Stop if the goal is already reached

        // Generate a random point
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

        // Move towards the random point by stepSize
        Point newPoint = {
            nearest.x + stepSize * (randomPoint.x - nearest.x) / minDist,
            nearest.y + stepSize * (randomPoint.y - nearest.y) / minDist
        };

        // Check for collisions
        if (isCollisionFree(newPoint, obstacles, 1.0)) {
            tree.push_back(newPoint);

            // Check if the goal is reached
            if (!goalReached &&
                std::sqrt(std::pow(newPoint.x - goal.x, 2) +
                          std::pow(newPoint.y - goal.y, 2)) < 1.0) {
                goalReached = true;
                goalIterationNumber = i;
            }
        }
    }

    auto end_time = std::chrono::high_resolution_clock::now();
    auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(end_time - start_time);

    // Output results
    if (goalReached) {
        std::cout << "Goal reached at iteration " << goalIterationNumber << "!" << std::endl;
    } else {
        std::cout << "Goal not reached within maximum iterations." << std::endl;
    }

    std::cout << "RRT tree size: " << tree.size() << std::endl;
    std::cout << "Execution time: " << duration.count() << " milliseconds" << std::endl;

    return 0;
}
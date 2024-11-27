#include <iostream>
#include <vector>
#include <cmath>
#include <omp.h>
#include <random>
#include <atomic>
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
    std::vector<Point> obstacles = {{4.0, 4.0}, {5.0, 5.0}, {6.0, 6.0}};

    // Global RRT tree
    std::vector<Point> globalTree;
    globalTree.push_back(start);

    // Parameters
    int maxIterations = 1000;
    double stepSize = 1.0;

    std::atomic<bool> goalReached(false);
    int goalIterationNumber = -1;

    auto start_time = std::chrono::high_resolution_clock::now();

    // Parallel region with thread-local data
    #pragma omp parallel
    {
        // Thread-local tree
        std::vector<Point> localTree;
        localTree.push_back(start);

        // Thread-local random generator
        std::random_device rd;
        std::mt19937 gen(rd() + omp_get_thread_num());
        std::uniform_real_distribution<> dis(0.0, 10.0);

        for (int i = 0; i < maxIterations; ++i) {
            if (goalReached) break; // Stop if the goal is already reached

            // Generate a random point
            Point randomPoint = {dis(gen), dis(gen)};

            // Find the nearest point in the local tree
            Point nearest = localTree[0];
            double minDist = std::numeric_limits<double>::max();
            for (const auto& node : localTree) {
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
                localTree.push_back(newPoint);

                // Check if the goal is reached
                if (!goalReached &&
                    std::sqrt(std::pow(newPoint.x - goal.x, 2) +
                              std::pow(newPoint.y - goal.y, 2)) < 1.0) {
                    goalReached = true;
                    goalIterationNumber = i;
                }
            }
        }

        // Merge local tree into the global tree
        #pragma omp critical
        {
            globalTree.insert(globalTree.end(), localTree.begin(), localTree.end());
        }
    }

    auto end_time = std::chrono::high_resolution_clock::now();
    auto duration = std::chrono::duration_cast<std::chrono::microseconds>(end_time - start_time);

    // Output results
    if (goalReached) {
        std::cout << "Goal reached at iteration " << goalIterationNumber << "!" << std::endl;
    } else {
        std::cout << "Goal not reached within maximum iterations." << std::endl;
    }

    std::cout << "RRT tree size: " << globalTree.size() << std::endl;
    std::cout << "Execution time: " << duration.count() << " milliseconds" << std::endl;

    return 0;
}

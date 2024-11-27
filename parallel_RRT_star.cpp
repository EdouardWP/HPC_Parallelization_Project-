#include <iostream>
#include <vector>
#include <cmath>
#include <omp.h>
#include <random>
#include <atomic>
#include <chrono>
#include <limits>
#include <algorithm>

// Define a 2D point
struct Point {
    double x, y;

    // Define equality operator for Point
    bool operator==(const Point& other) const {
        return x == other.x && y == other.y;
    }
};

// Helper function: Euclidean distance
double distance(Point p1, Point p2) {
    return std::sqrt(std::pow(p1.x - p2.x, 2) + std::pow(p1.y - p2.y, 2));
}

// Check for collisions with obstacles
bool isCollisionFree(Point p, const std::vector<Point>& obstacles, double radius) {
    for (const auto& obs : obstacles) {
        if (distance(p, obs) < radius) return false;
    }
    return true;
}

// Rewire nodes to maintain optimality
void rewire(std::vector<Point>& tree, Point newPoint, const std::vector<Point>& obstacles, double radius, double stepSize) {
    int rewireCount = 0;
    const int MAX_REWIRES = 10; // Limit number of rewires per iteration
    
    for (auto& node : tree) {
        if (rewireCount >= MAX_REWIRES) break;
        if (distance(node, newPoint) < radius) {
            // Check if rewiring improves path quality
            Point potentialNew = {
                node.x + stepSize * (newPoint.x - node.x) / distance(node, newPoint),
                node.y + stepSize * (newPoint.y - node.y) / distance(node, newPoint)
            };
            if (isCollisionFree(potentialNew, obstacles, 1.0)) {
                double currentDist = distance(node, Point{0.0, 0.0}); // Current path cost
                double newDist = distance(newPoint, Point{0.0, 0.0}) + distance(node, newPoint);
                if (newDist < currentDist) { // Only rewire if cost is reduced
                    node = potentialNew;
                }
            }
            rewireCount++;
        }
    }
}

int main() {
    // Start and goal points
    Point start = {0.0, 0.0};
    Point goal = {297.0, 297.0};

    // Generate random obstacles
    std::vector<Point> obstacles;
    int numObstacles = 1500;
    std::random_device rd;
    std::mt19937 gen(rd());
    std::uniform_real_distribution<> dis(0.0, 300.0);
    for (int i = 0; i < numObstacles; ++i) {
        obstacles.push_back({dis(gen), dis(gen)});
    }

    // Global RRT* tree
    std::vector<Point> globalTree;
    globalTree.push_back(start);

    // Parameters
    int maxIterations = 30000;
    double stepSize = 3.0;
    double radius = 3.0;

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
        std::mt19937 gen(rd() + omp_get_thread_num());
        std::uniform_real_distribution<> dis(0.0, 300.0);

        for (int i = 0; i < maxIterations; ++i) {
            if (goalReached) break;

            // Generate a random point
            Point randomPoint = {dis(gen), dis(gen)};

            // Find the nearest point in the local tree
            Point nearest = localTree[0];
            double minDist = std::numeric_limits<double>::max();
            for (const auto& node : localTree) {
                double dist = distance(randomPoint, node);
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

                // Rewire local tree
                rewire(localTree, newPoint, obstacles, radius, stepSize);

                // Check if the goal is reached
                if (!goalReached &&
                    distance(newPoint, goal) < 1.0) {
                    goalReached = true;
                    goalIterationNumber = i;
                }
            }
        }

        // Merge local tree into the global tree
        #pragma omp critical
        {
            globalTree.insert(globalTree.end(), localTree.begin(), localTree.end());
            // Ensure no duplicate nodes are added
            std::sort(globalTree.begin(), globalTree.end(), [](const Point& a, const Point& b) {
                return a.x < b.x || (a.x == b.x && a.y < b.y);
            });
            globalTree.erase(std::unique(globalTree.begin(), globalTree.end()), globalTree.end());
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

    std::cout << "RRT* tree size: " << globalTree.size() << std::endl;
    std::cout << "Execution time: " << duration.count() << " milliseconds" << std::endl;

    return 0;
}

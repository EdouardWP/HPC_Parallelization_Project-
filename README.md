# RRT Path Planning Implementations

This repository contains three different implementations of the Rapidly-exploring Random Tree (RRT) algorithm for path planning:

1. **Sequential RRT** (`sequential.cpp`)
   - Basic RRT implementation
   - Single-threaded execution

2. **Parallel RRT** (`parallelization.cpp`)
   - Parallel implementation of basic RRT
   - Uses OpenMP for parallelization

3. **Parallel RRT*** (`parallel_RRT_star.cpp`)
   - Parallel implementation of RRT* algorithm
   - Includes path optimization through rewiring
   - Uses OpenMP for parallelization

## Implementation Details

### Common Features
- Start point: (0.0, 0.0)
- Goal point: (297.0, 297.0)
- Search space: 300x300 units
- Number of obstacles: 1500
- Maximum iterations: 30000
- Step size: 3.0 units

## Output

Each implementation outputs:
- Whether the goal was reached
- Iteration number when goal was reached (if successful)
- Final tree size
- Total execution time in milliseconds

## Performance Considerations

- Parallel implementations may show better performance on multi-core systems
- RRT* typically produces more optimal paths but requires more computation time
- Performance varies based on:
  - Number of obstacles
  - Search space size
  - Maximum iterations
  - Available CPU cores (for parallel versions)

## Requirements

- C++ compiler with C++11 support
- OpenMP library
- macOS/Linux environment

## Compilation

For macOS using g++ (with Homebrew's OpenMP):

Sequential RRT
- g++ -std=c++11 sequential.cpp -o sequential
  
Parallel RRT
- g++ -std=c++11 -Xpreprocessor -fopenmp parallelization.cpp -o parallel_rrt -lomp -I/opt/homebrew/opt/libomp/include -L/opt/homebrew/opt/libomp/lib

Parallel RRT*
- g++ -std=c++11 -fopenmp parallel_RRT_star.cpp -o parallel_rrt_star

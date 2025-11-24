# Terminal Maze A* Solver

## Overview
A terminal-based maze solver that implements the A* (A-Star) pathfinding algorithm to find the shortest path through a maze. The project demonstrates classic AI search algorithms with visual console output.

**Purpose:** Educational implementation of A* pathfinding algorithm  
**Language:** Python 3.11  
**Environment:** Linux Terminal (Gitpod/Replit compatible)

## Current State
✅ Complete and functional  
✅ Console-based visualization with ANSI colors  
✅ Hardcoded 10x10 sample maze with obstacles  
✅ Manhattan Distance heuristic  
✅ Unreachable goal detection  

## Recent Changes
- **2025-11-23**: Initial project creation
  - Implemented A* Search algorithm using heapq priority queue
  - Created MazeGrid class for maze representation
  - Added Manhattan Distance heuristic function
  - Implemented console visualization with colored output
  - Added comprehensive comments explaining algorithm steps
  - Created 10x10 sample maze demonstrating obstacle navigation

## Project Architecture

### File Structure
```
terminal-maze-astor/
├── main.py          # Complete maze solver implementation
├── .gitignore       # Python-specific ignores
└── replit.md        # This documentation file
```

### Key Components

#### 1. MazeGrid Class
- Represents the maze as a 2D grid
- Identifies start (S), goal (G), and obstacles (#)
- Provides neighbor discovery and validation

#### 2. A* Search Algorithm
- **Priority Queue:** Uses Python's heapq for efficient min-heap operations
- **Scoring System:**
  - `g_score`: Actual cost from start to current node
  - `h_score`: Heuristic (Manhattan Distance) from current to goal
  - `f_score`: g_score + h_score (total estimated cost)
- **Path Reconstruction:** Backtracking through came_from dictionary

#### 3. Manhattan Distance Heuristic
- Sum of absolute coordinate differences
- Admissible for 4-directional grid movement
- Formula: `|x1 - x2| + |y1 - y2|`

#### 4. Console Visualization
- ANSI color codes for terminal output
- Color-coded elements:
  - 🟢 Green (S) = Start position
  - 🔴 Red (G) = Goal position
  - 🔵 Blue (#) = Walls/Obstacles
  - 🟡 Yellow (*) = Solution path
  - ⚪ White (.) = Open spaces

## How to Run
```bash
python main.py
```

The program will:
1. Display the original maze
2. Run A* search algorithm
3. Show the solved maze with the path marked
4. Print statistics (path length, coordinates, etc.)

## Algorithm Details

### A* Search Explained
A* is an informed search algorithm that finds the shortest path by:
1. Maintaining a priority queue of nodes to explore
2. Always exploring the node with lowest f_score first
3. Using heuristic to guide search toward goal
4. Guaranteeing shortest path (with admissible heuristic)

### Time Complexity
- Worst case: O(b^d) where b is branching factor, d is depth
- With good heuristic: Much better than uninformed search

### Space Complexity
- O(b^d) for storing nodes in open and closed sets

## Future Enhancements
- Interactive maze input (user-defined mazes)
- Multiple heuristic options (Euclidean, Chebyshev)
- Step-by-step animation of algorithm exploration
- Random maze generation
- Performance metrics (nodes explored, execution time)
- Diagonal movement support
- Different pathfinding algorithms (Dijkstra, BFS, DFS) for comparison

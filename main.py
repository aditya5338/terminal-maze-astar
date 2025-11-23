#!/usr/bin/env python3
"""
Terminal Maze Solver using A* Search Algorithm

This program implements the A* pathfinding algorithm to find the shortest
path through a maze from a start point to a goal point, navigating around obstacles.
"""

import heapq
from typing import List, Tuple, Optional, Set
from collections import defaultdict


class MazeGrid:
    """
    Represents a maze grid with walls, start, and goal positions.
    """
    
    def __init__(self, grid: List[List[str]]):
        """
        Initialize the maze grid.
        
        Args:
            grid: 2D list representing the maze
                  'S' = Start position
                  'G' = Goal position
                  '#' = Wall/Obstacle
                  ' ' = Open path
        """
        self.grid = grid
        self.rows = len(grid)
        self.cols = len(grid[0]) if grid else 0
        
        start_pos = self._find_position('S')
        goal_pos = self._find_position('G')
        
        if not start_pos:
            raise ValueError("Start position 'S' not found in maze!")
        if not goal_pos:
            raise ValueError("Goal position 'G' not found in maze!")
        
        self.start: Tuple[int, int] = start_pos
        self.goal: Tuple[int, int] = goal_pos
    
    def _find_position(self, symbol: str) -> Optional[Tuple[int, int]]:
        """Find the position of a specific symbol in the grid."""
        for r in range(self.rows):
            for c in range(self.cols):
                if self.grid[r][c] == symbol:
                    return (r, c)
        return None
    
    def is_valid(self, pos: Tuple[int, int]) -> bool:
        """
        Check if a position is valid and walkable.
        
        Args:
            pos: (row, col) tuple
            
        Returns:
            True if position is within bounds and not a wall
        """
        row, col = pos
        if 0 <= row < self.rows and 0 <= col < self.cols:
            return self.grid[row][col] != '#'
        return False
    
    def get_neighbors(self, pos: Tuple[int, int]) -> List[Tuple[int, int]]:
        """
        Get valid neighboring positions (up, down, left, right).
        
        Args:
            pos: Current position (row, col)
            
        Returns:
            List of valid neighbor positions
        """
        row, col = pos
        # Four directions: up, down, left, right
        directions = [(-1, 0), (1, 0), (0, -1), (0, 1)]
        neighbors = []
        
        for dr, dc in directions:
            new_pos = (row + dr, col + dc)
            if self.is_valid(new_pos):
                neighbors.append(new_pos)
        
        return neighbors


def manhattan_distance(pos1: Tuple[int, int], pos2: Tuple[int, int]) -> int:
    """
    Calculate Manhattan Distance heuristic between two positions.
    
    Manhattan Distance is the sum of absolute differences in coordinates.
    It's admissible for grid-based movement (up/down/left/right only).
    
    Args:
        pos1: First position (row, col)
        pos2: Second position (row, col)
        
    Returns:
        Manhattan distance between the two positions
    """
    return abs(pos1[0] - pos2[0]) + abs(pos1[1] - pos2[1])


def a_star_search(maze: MazeGrid) -> Optional[List[Tuple[int, int]]]:
    """
    Implement A* Search algorithm to find the shortest path.
    
    A* uses: f(n) = g(n) + h(n)
    where:
        g(n) = actual cost from start to node n
        h(n) = heuristic estimated cost from node n to goal
        f(n) = total estimated cost of path through node n
    
    Args:
        maze: MazeGrid object containing the maze
        
    Returns:
        List of (row, col) tuples representing the path, or None if unreachable
    """
    start = maze.start
    goal = maze.goal
    
    # Priority queue: stores (f_score, counter, position)
    # counter ensures consistent ordering when f_scores are equal
    open_set = []
    counter = 0
    heapq.heappush(open_set, (0, counter, start))
    
    # Track where each node came from (for path reconstruction)
    came_from = {}
    
    # g_score: cost from start to each node
    g_score = defaultdict(lambda: float('inf'))
    g_score[start] = 0
    
    # f_score: g_score + heuristic (estimated total cost)
    f_score = defaultdict(lambda: float('inf'))
    f_score[start] = manhattan_distance(start, goal)
    
    # Track nodes currently in open_set
    open_set_hash: Set[Tuple[int, int]] = {start}
    
    while open_set:
        # Get node with lowest f_score
        current_f, _, current = heapq.heappop(open_set)
        open_set_hash.discard(current)
        
        # Goal reached! Reconstruct and return the path
        if current == goal:
            return reconstruct_path(came_from, current)
        
        # Explore neighbors
        for neighbor in maze.get_neighbors(current):
            # g_score for neighbor through current path
            # Each step has cost of 1
            tentative_g_score = g_score[current] + 1
            
            # If this path to neighbor is better than previous ones
            if tentative_g_score < g_score[neighbor]:
                # This is the best path to neighbor so far, record it
                came_from[neighbor] = current
                g_score[neighbor] = tentative_g_score
                f_score[neighbor] = tentative_g_score + manhattan_distance(neighbor, goal)
                
                # Add neighbor to open_set if not already there
                if neighbor not in open_set_hash:
                    counter += 1
                    heapq.heappush(open_set, (f_score[neighbor], counter, neighbor))
                    open_set_hash.add(neighbor)
    
    # No path found - goal is unreachable
    return None


def reconstruct_path(came_from: dict, current: Tuple[int, int]) -> List[Tuple[int, int]]:
    """
    Reconstruct the path from start to goal by backtracking.
    
    Args:
        came_from: Dictionary mapping each node to its predecessor
        current: Goal position
        
    Returns:
        List of positions from start to goal
    """
    path = [current]
    while current in came_from:
        current = came_from[current]
        path.append(current)
    path.reverse()
    return path


def visualize_maze(maze: MazeGrid, path: Optional[List[Tuple[int, int]]] = None):
    """
    Display the maze in the console with the solved path.
    
    Args:
        maze: MazeGrid object
        path: Optional list of positions representing the solution path
    """
    # ANSI color codes for terminal
    RESET = '\033[0m'
    GREEN = '\033[92m'      # Start
    RED = '\033[91m'        # Goal
    YELLOW = '\033[93m'     # Path
    BLUE = '\033[94m'       # Wall
    WHITE = '\033[97m'      # Empty space
    
    # Create a copy of the grid for visualization
    visual_grid = [row[:] for row in maze.grid]
    
    # Mark the path with '*' (excluding start and goal)
    if path:
        for pos in path[1:-1]:  # Skip start and goal
            row, col = pos
            visual_grid[row][col] = '*'
    
    # Print the maze
    print("\n" + "=" * (maze.cols * 2 + 3))
    print(" MAZE SOLVER - A* SEARCH ALGORITHM")
    print("=" * (maze.cols * 2 + 3))
    
    for r in range(maze.rows):
        print(" ", end="")
        for c in range(maze.cols):
            cell = visual_grid[r][c]
            
            if cell == 'S':
                print(f"{GREEN}S{RESET}", end=" ")
            elif cell == 'G':
                print(f"{RED}G{RESET}", end=" ")
            elif cell == '#':
                print(f"{BLUE}#{RESET}", end=" ")
            elif cell == '*':
                print(f"{YELLOW}*{RESET}", end=" ")
            else:
                print(f"{WHITE}.{RESET}", end=" ")
        print()
    
    print("=" * (maze.cols * 2 + 3))
    
    # Legend
    print(f"\nLegend:")
    print(f"  {GREEN}S{RESET} = Start")
    print(f"  {RED}G{RESET} = Goal")
    print(f"  {BLUE}#{RESET} = Wall/Obstacle")
    print(f"  {YELLOW}*{RESET} = Solution Path")
    print(f"  {WHITE}.{RESET} = Open Space")

def get_maze_dimensions() -> Tuple[int, int]:
    """Get maze dimensions from user input."""
    while True:
        try:
            rows = int(input("Enter the number of rows: "))
            cols = int(input("Enter the number of columns: "))
            if rows > 0 and cols > 0:
                return rows, cols
            print("Dimensions must be positive integers.")
        except ValueError:
            print("Invalid input. Please enter integers for dimensions.")

def create_custom_maze() -> Optional[List[List[str]]]:
    """Create a custom maze from user input."""
    try:
        rows, cols = get_maze_dimensions()
        
        print(f"\nEnter the maze layout ({rows}x{cols}). Use 'S' for start, 'G' for goal, '#' for walls, and ' ' for open paths.")
        
        maze = []
        for r in range(rows):
            while True:
                row_input = input(f"Row {r + 1}: ")
                if len(row_input) == cols:
                    maze.append(list(row_input))
                    break
                print(f"Invalid input. Please enter exactly {cols} characters for each row.")
        
        # Validate maze content (e.g., ensure 'S' and 'G' are present)
        flat_maze = [cell for row in maze for cell in row]
        if 'S' not in flat_maze or 'G' not in flat_maze:
            print("\nError: The maze must contain at least one 'S' (start) and one 'G' (goal).")
            return None
            
        return maze

    except Exception as e:
        print(f"An error occurred: {e}")
        return None


def main():
    """
    Main function to demonstrate the A* Maze Solver.
    """
    # Define a 10x10 maze with obstacles
    # 'S' = Start, 'G' = Goal, '#' = Wall, ' ' = Open path
    sample_maze = [
        ['S', ' ', ' ', '#', ' ', ' ', ' ', ' ', ' ', ' '],
        [' ', '#', ' ', '#', ' ', '#', '#', '#', '#', ' '],
        [' ', '#', ' ', ' ', ' ', ' ', ' ', ' ', '#', ' '],
        [' ', '#', '#', '#', '#', '#', '#', ' ', '#', ' '],
        [' ', ' ', ' ', ' ', ' ', ' ', ' ', ' ', '#', ' '],
        [' ', '#', '#', '#', '#', '#', '#', '#', '#', ' '],
        [' ', ' ', ' ', ' ', ' ', ' ', ' ', ' ', ' ', ' '],
        ['#', '#', '#', '#', '#', '#', ' ', '#', '#', '#'],
        [' ', ' ', ' ', ' ', ' ', ' ', ' ', ' ', ' ', ' '],
        [' ', '#', '#', '#', '#', '#', '#', '#', '#', 'G'],
    ]
    
    print("\n" + "╔" + "═" * 50 + "╗")
    print("║" + " " * 12 + "A* MAZE SOLVER" + " " * 24 + "║")
    print("║" + " " * 8 + "Manhattan Distance Heuristic" + " " * 14 + "║")
    print("╚" + "═" * 50 + "╝")

    maze_to_solve = None
    while True:
        choice = input("\nWould you like to use the (d)efault maze or (c)reate a new one? ").lower()
        if choice in ['d', 'c']:
            break
        print("Invalid choice. Please enter 'd' or 'c'.")

    if choice == 'd':
        maze_to_solve = sample_maze
    else:
        maze_to_solve = create_custom_maze()
        if not maze_to_solve:
            print("\nExiting program.")
            return

    # Create maze object
    try:
        maze = MazeGrid(maze_to_solve)
    except ValueError as e:
        print(f"Error creating maze: {e}")
        return
    
    # Display original maze
    print("\n📍 Original Maze:")
    visualize_maze(maze)
    
    print(f"\n🔍 Searching for path from {maze.start} to {maze.goal}...")
    print("   Using A* Search Algorithm with Manhattan Distance heuristic")
    
    # Run A* algorithm
    path = a_star_search(maze)
    
    # Display results
    if path:
        print(f"\n✅ SUCCESS! Path found!")
        print(f"   Path length: {len(path)} steps")
        print(f"   Path coordinates: {path[:5]}{'...' if len(path) > 5 else ''}")
        print(f"\n🗺️  Maze with Solution Path:")
        visualize_maze(maze, path)
        
        # Additional statistics
        print(f"\n📊 Statistics:")
        print(f"   Start position: {maze.start}")
        print(f"   Goal position: {maze.goal}")
        print(f"   Total steps: {len(path) - 1}")  # Excluding start position
        print(f"   Manhattan distance: {manhattan_distance(maze.start, maze.goal)}")
    else:
        print(f"\n❌ UNREACHABLE! No path exists from start to goal.")
        print(f"   The goal at {maze.goal} cannot be reached from start at {maze.start}")
        print(f"   This maze has obstacles blocking all possible paths.")
    
    print("\n" + "─" * 52)
    print("Algorithm: A* Search")
    print("Heuristic: Manhattan Distance")
    print("Data Structure: Min-Heap Priority Queue (heapq)")
    print("─" * 52 + "\n")


if __name__ == "__main__":
    main()

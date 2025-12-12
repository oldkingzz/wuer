
import math
import heapq
import matplotlib.pyplot as plt
import numpy as np
import os

# Map Configuration
FIELD_WIDTH_INCH = 60
FIELD_LENGTH_INCH = 144
MAP_RESOLUTION = 1  # 1 pixel/inch
MAP_WIDTH = int(FIELD_WIDTH_INCH * MAP_RESOLUTION)   # 60
MAP_HEIGHT = int(FIELD_LENGTH_INCH * MAP_RESOLUTION) # 144

COST_FREE = 1
COST_OBSTACLE = 255
COST_TOWER = 5
ROBOT_RADIUS = 6.0  # Inches

# Initialize Map
grid_map = np.full((MAP_WIDTH, MAP_HEIGHT), COST_FREE, dtype=np.uint8)

def add_obstacle(x_in, y_in, w_in, h_in, cost, inflate=0.0):
    # Inflate
    x_start = int((x_in - inflate) * MAP_RESOLUTION)
    y_start = int((y_in - inflate) * MAP_RESOLUTION)
    w_pixel = int((w_in + 2 * inflate) * MAP_RESOLUTION)
    h_pixel = int((h_in + 2 * inflate) * MAP_RESOLUTION)

    for x in range(x_start, x_start + w_pixel):
        for y in range(y_start, y_start + h_pixel):
            if 0 <= x < MAP_WIDTH and 0 <= y < MAP_HEIGHT:
                grid_map[x, y] = cost

def init_map():
    # Boundary Walls (Inflated)
    add_obstacle(0, 0, 0, 144, COST_OBSTACLE, ROBOT_RADIUS)   # Left
    add_obstacle(60, 0, 0, 144, COST_OBSTACLE, ROBOT_RADIUS)  # Right
    add_obstacle(0, 0, 60, 0, COST_OBSTACLE, ROBOT_RADIUS)    # Bottom
    add_obstacle(0, 144, 60, 0, COST_OBSTACLE, ROBOT_RADIUS)  # Top

    # Nexus Bases (Inflated)
    add_obstacle(26, 0, 8, 5, COST_OBSTACLE, ROBOT_RADIUS)    # South
    add_obstacle(26, 139, 8, 5, COST_OBSTACLE, ROBOT_RADIUS)  # North

    # Towers (Inflated)
    # Low Tower (40, 72) logic -> Box 6x8 -> x:[37, 43], y:[68, 76]
    add_obstacle(37, 68, 6, 8, COST_OBSTACLE, ROBOT_RADIUS) 

    # High Tower (9.5, 72) -> Box 6x8 -> x:[6.5, 12.5], y:[68, 76]
    add_obstacle(6.5, 68, 6, 8, COST_OBSTACLE, ROBOT_RADIUS)

    # ---------------------------------------------------------
    # NEW: High Ground Side Wall (X=20)
    # User Requested: Wall from (20, 16.1) to (20, 127.9)
    # Thickness 1.0" -> X: 19.5 to 20.5 (Centered at 20)
    # Height: 127.9 - 16.1 = 111.8
    add_obstacle(19.5, 16.1, 1.0, 111.8, COST_OBSTACLE, ROBOT_RADIUS)
    # ---------------------------------------------------------

class Node:
    def __init__(self, x, y, g, h, parent=None):
        self.x = x
        self.y = y
        self.g = g
        self.h = h
        self.f = g + h
        self.parent = parent
    
    def __lt__(self, other):
        return self.f < other.f

def heuristic(x1, y1, x2, y2):
    return math.sqrt((x1-x2)**2 + (y1-y2)**2)

def astar(start, goal):
    start_x, start_y = start
    goal_x, goal_y = goal

    if not (0 <= start_x < MAP_WIDTH and 0 <= start_y < MAP_HEIGHT):
        print(f"Start {start} out of bounds")
        return None
        
    if grid_map[start_x, start_y] >= COST_OBSTACLE:
        print(f"START BLOCKED: Cost {grid_map[start_x, start_y]}")
        return None
    if grid_map[goal_x, goal_y] >= COST_OBSTACLE:
        print(f"GOAL BLOCKED: Cost {grid_map[goal_x, goal_y]}")
        return None

    open_list = []
    start_node = Node(start_x, start_y, 0, heuristic(start_x, start_y, goal_x, goal_y))
    heapq.heappush(open_list, start_node)

    visited = np.zeros((MAP_WIDTH, MAP_HEIGHT), dtype=bool)
    iterations = 0

    while open_list:
        current = heapq.heappop(open_list)
        iterations += 1

        if visited[current.x, current.y]:
            continue
        visited[current.x, current.y] = True

        if current.x == goal_x and current.y == goal_y:
            print(f"Path Found in {iterations} iterations!")
            path = []
            while current:
                path.append((current.x, current.y))
                current = current.parent
            return path[::-1]

        # Neighbors 8-direction
        for dx, dy in [(-1,-1), (-1,0), (-1,1), (0,-1), (0,1), (1,-1), (1,0), (1,1)]:
            nx, ny = current.x + dx, current.y + dy

            if 0 <= nx < MAP_WIDTH and 0 <= ny < MAP_HEIGHT:
                if grid_map[nx, ny] >= COST_OBSTACLE:
                    continue
                if visited[nx, ny]:
                    continue
                
                # Move Cost
                dist = math.sqrt(dx*dx + dy*dy)
                new_g = current.g + dist # Assuming uniform cost for simpler visualization
                
                neighbor = Node(nx, ny, new_g, heuristic(nx, ny, goal_x, goal_y), current)
                heapq.heappush(open_list, neighbor)

    print(f"No path found after {iterations} iterations.")
    return None

def visualize(path, start, goal):
    plt.figure(figsize=(8, 12)) # Make it bigger
    # Transpose for visualization (x=width, y=height)
    plt.imshow(grid_map.T, origin='lower', cmap='gray_r', extent=[0, MAP_WIDTH, 0, MAP_HEIGHT], vmin=0, vmax=255)
    
    # Highlight the specific High Ground Wall for the user
    # Wall from (19.5, 16.1) to (20.5, 127.9)
    # We draw a red rectangle to make it obvious
    from matplotlib.patches import Rectangle
    rect = Rectangle((19.5, 16.1), 1.0, 111.8, linewidth=2, edgecolor='red', facecolor='none', label='High Ground Wall')
    plt.gca().add_patch(rect)
    
    if path:
        px = [p[0] for p in path]
        py = [p[1] for p in path]
        plt.plot(px, py, 'b-', linewidth=2, label='Path')
    
    plt.plot(start[0], start[1], 'go', markersize=10, label='Start')
    plt.plot(goal[0], goal[1], 'rx', markersize=10, label='Goal')
    
    plt.title(f"A* Test (Radius={ROBOT_RADIUS}\")\nHigh Ground Wall marked in RED")
    plt.xlabel("X (inch)")
    plt.ylabel("Y (inch)")
    plt.legend()
    plt.grid(True, alpha=0.3)
    
    # Save to current directory (which will be /scripts)
    output_path = 'astar_test_result.png'
    plt.savefig(output_path)
    print(f"Result saved to {os.path.abspath(output_path)}")

if __name__ == "__main__":
    init_map()
    
    # User Request: Check path from (46, 96) to (40, 110)
    start = (46, 96)
    goal = (40, 110)
    
    print(f"Testing A* from {start} to {goal}...")
    path = astar(start, goal)
    
    visualize(path, start, goal)

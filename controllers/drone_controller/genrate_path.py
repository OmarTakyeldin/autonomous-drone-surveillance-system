import os
import numpy as np
import cv2
from scipy.ndimage import binary_dilation
from scipy.spatial.distance import cdist
from queue import PriorityQueue
from skimage.draw import line
import matplotlib.pyplot as plt
from typing import List, Tuple


def px_to_world(col: int, row: int) -> Tuple[float, float]:
    # Convert grid coordinates to world coordinates
    return col - 60, 50 - row


def world_to_px(x: float, y: float) -> Tuple[int, int]:
    # Convert world coordinates to grid coordinates
    return int(round(x + 60)), int(round(50 - y))


def nearest_free(px: int, py: int, safe_grid: np.ndarray, max_radius: int = 5) -> Tuple[int, int]:
    # Find the nearest free cell to (px, py) within max_radius
    if safe_grid[py, px] == 0:
        return px, py
    for R in range(1, max_radius + 1):
        for dx in range(-R, R + 1):
            for dy in range(-R, R + 1):
                nx, ny = px + dx, py + dy
                if 0 <= nx < safe_grid.shape[1] and 0 <= ny < safe_grid.shape[0] and safe_grid[ny, nx] == 0:
                    return nx, ny
    return None

def has_los(p1: Tuple[int, int], p2: Tuple[int, int], safe_grid: np.ndarray) -> bool:
    # Check line of sight between two grid points
    rr, cc = line(p1[1], p1[0], p2[1], p2[0])
    return np.all(safe_grid[rr, cc] == 0)


def smooth_path(raw: List[Tuple[int, int]], safe_grid: np.ndarray) -> List[Tuple[int, int]]:
    # Smooth a raw grid path by skipping intermediate points with LOS
    if not raw:
        return []
    smoothed = [raw[0]]
    i = 0
    while i < len(raw) - 1:
        j = len(raw) - 1
        while j > i + 1 and not has_los(raw[i], raw[j], safe_grid):
            j -= 1
        smoothed.append(raw[j])
        i = j
    return smoothed

def astar(start: Tuple[int, int], goal: Tuple[int, int], safe_grid: np.ndarray) -> List[Tuple[int, int]]:
    # Perform A* search on the grid from start to goal
    rows, cols = safe_grid.shape
    g_score = {start: 0}
    f_score = {start: np.hypot(goal[0] - start[0], goal[1] - start[1])}
    open_set = PriorityQueue()
    open_set.put((f_score[start], start))
    came_from = {}
    closed = set()
    directions = [(-1, 0), (1, 0), (0, -1), (0, 1), (-1, -1), (-1, 1), (1, -1), (1, 1)]

    while not open_set.empty():
        _, current = open_set.get()
        if current == goal:
            path = []
            while current in came_from:
                path.append(current)
                current = came_from[current]
            return path[::-1] + [goal]
        closed.add(current)
        for dx, dy in directions:
            neighbor = (current[0] + dx, current[1] + dy)
            if (0 <= neighbor[0] < cols and 0 <= neighbor[1] < rows and safe_grid[neighbor[1], neighbor[0]] == 0):
                tentative_g = g_score[current] + np.hypot(dx, dy)
                if neighbor not in g_score or tentative_g < g_score[neighbor]:
                    came_from[neighbor] = current
                    g_score[neighbor] = tentative_g
                    f_score[neighbor] = tentative_g + np.hypot(goal[0] - neighbor[0], goal[1] - neighbor[1])
                    if neighbor not in closed:
                        open_set.put((f_score[neighbor], neighbor))
    return []


def generate_drone_path(
    start_pos: Tuple[float, float],
    altitude: float,
    occupancy_map: str = "occupancy_map.csv",
) -> Tuple[List[List[float]], np.ndarray]:

    # Generate a 3D waypoint list for drone navigation over the buffered occupancy grid
    # Get the directory of the current file
    base_dir = os.path.dirname(os.path.abspath(__file__))
    
    # Construct the absolute path to the occupancy map file
    map_path = occupancy_map if os.path.isabs(occupancy_map) else os.path.join(base_dir, occupancy_map)

    # Load the occupancy map as a 2D numpy array (0 = free, 1 = occupied)
    safe_grid = np.loadtxt(map_path, delimiter=",", dtype=int)

    # STEP 1: Find Edge Points from the Occupancy Map

    # Convert occupancy grid to an 8-bit image format (needed by OpenCV)
    bin_img = (safe_grid * 255).astype(np.uint8)

    # Detect external contours (edges of obstacles)
    contours, _ = cv2.findContours(bin_img, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    # Extract edge points and convert them to world coordinates
    edges = []
    for cnt in contours:
        eps = 0.01 * cv2.arcLength(cnt, True)           # Set approximation accuracy
        approx = cv2.approxPolyDP(cnt, eps, True)       # Simplify contour to fewer points
        for p in approx:
            c0, r0 = p[0]                                      # Extract column and row
            edges.append(px_to_world(c0, r0))                  # Convert to (x, y) world coordinates

    # STEP 2: Plan Visit Order using Nearest Neighbor Heuristic

    # Combine start position with edge points
    pts = [start_pos] + edges

    # Convert to numpy array for fast distance computations
    coords = np.array(pts)

    # Compute all pairwise Euclidean distances between points
    D = cdist(coords, coords)

    # Use a greedy nearest-neighbor approach to order the edge visits
    route_idx = [0]                         # Start at the first point (start_pos)
    unvisited = set(range(1, len(pts)))     # Set of all edge indices

    while unvisited:
        last = route_idx[-1]                            # Get the last visited point
        nxt = min(unvisited, key=lambda i: D[last, i])  # Find the closest unvisited point
        route_idx.append(nxt)                           # Add it to the route
        unvisited.remove(nxt)                           # Mark as visited

    # Build the final visiting route as a list of (x, y) tuples
    route = [tuple(coords[i]) for i in route_idx]

    # STEP 3: Build a Full 2D Path Between All Route Points

    full2d = []

    # For each segment in the route, plan a path using A* from one to the next
    for (x1, y1), (x2, y2) in zip(route, route[1:]):
        start_px = world_to_px(x1, y1)                     # Convert start point to grid
        end_px = world_to_px(x2, y2)                       # Convert end point to grid
        s1 = nearest_free(*start_px, safe_grid)            # Ensure start is in a free cell
        s2 = nearest_free(*end_px, safe_grid)              # Ensure end is in a free cell
        if s1 and s2:
            raw_path = astar(s1, s2, safe_grid)            # Compute raw A* path
            full2d += smooth_path(raw_path, safe_grid)     # Smooth the path and add to full path

    # STEP 4: Return to Starting Point

    end_px = nearest_free(*world_to_px(*route[-1]), safe_grid)   # Last point
    home_px = nearest_free(*world_to_px(*start_pos), safe_grid)  # Start point
    if end_px and home_px:
        full2d += smooth_path(astar(end_px, home_px, safe_grid), safe_grid)  # Return path

    # STEP 5: Convert 2D Path to 3D by Adding Altitude

    path3d = [
        [start_pos[0], start_pos[1], altitude]  # First point at specified altitude
    ]
    for px, py in full2d:
        wx, wy = px_to_world(px, py)            # Convert back to world coordinates
        path3d.append([wx, wy, altitude])       # Append 3D waypoint

    # Return final path and the occupancy grid
    return path3d, safe_grid

def generate_direct_path(
    start_pos: Tuple[float, float],
    end_pos: Tuple[float, float],
    altitude: float,
    occupancy_map: str = "occupancy_map.csv",
) -> Tuple[List[List[float]], np.ndarray]:

    # Generate the direct shortest 3D path from start_pos to end_pos, avoiding obstacles
    # Get the directory of the current file
    base_dir = os.path.dirname(os.path.abspath(__file__))

    # Create the full path to the occupancy map file
    map_path = occupancy_map if os.path.isabs(occupancy_map) else os.path.join(base_dir, occupancy_map)

    # Load the occupancy map from CSV as a 2D numpy array
    safe_grid = np.loadtxt(map_path, delimiter=",", dtype=int)

    # Convert world coordinates to grid (pixel) coordinates for both start and end positions
    s_px = world_to_px(*start_pos)
    e_px = world_to_px(*end_pos)

    # Find the nearest free (unoccupied) grid cells to start and end points
    s_free = nearest_free(*s_px, safe_grid)
    e_free = nearest_free(*e_px, safe_grid)

    # If either start or end can't be mapped to a free space, return empty path
    if not s_free or not e_free:
        return [], safe_grid

    # Run A* pathfinding from start to end grid cells
    raw_path = astar(s_free, e_free, safe_grid)

    # Smooth the resulting path to reduce unnecessary waypoints
    smooth = smooth_path(raw_path, safe_grid)

    # Initialize 3D path with the starting point at the given altitude
    path3d = [
        [start_pos[0], start_pos[1], altitude]
    ]

    # Convert each 2D grid point in the smoothed path to 3D world coordinates
    for px, py in smooth:
        wx, wy = px_to_world(px, py)
        path3d.append([wx, wy, altitude])

    # Return the computed 3D path and the occupancy grid
    return path3d, safe_grid

def plot_drone_path(path3d: List[List[float]], safe_grid: np.ndarray):
    # Visualize the 2D projection of the 3D drone path on the map
    cruise = [p for p in path3d if p[2] == path3d[1][2]]
    xs, ys = zip(*[(p[0], p[1]) for p in cruise])

    plt.figure(figsize=(12, 10))
    plt.imshow(1 - safe_grid, cmap="gray", origin="upper", extent=[-60, 60, -50, 50])
    plt.plot(xs, ys, 'b-', lw=1.2, label="Smoothed Path")
    for i in range(len(xs) - 1):
        dx, dy = xs[i+1] - xs[i], ys[i+1] - ys[i]
        plt.arrow(xs[i], ys[i], dx, dy, head_width=0.8, head_length=0.8, fc='orange', ec='orange', alpha=0.5)
    plt.scatter(xs[0], ys[0], c='green', s=100, marker='*', label="Start")
    plt.title("Drone Path Visiting Detected Edges")
    plt.xlabel("X (m)")
    plt.ylabel("Y (m)")
    plt.axis('equal')
    plt.grid(True)
    plt.legend()
    plt.show()

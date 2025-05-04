import os
import numpy as np
import matplotlib.pyplot as plt
import cv2
from scipy.ndimage import binary_dilation
from queue import PriorityQueue
from skimage.draw import line

# Paths and config
base_dir       = os.path.dirname(os.path.abspath(__file__))
csv_path       = os.path.join(base_dir, "occupancy_map.csv")
marked_map_csv = os.path.join(base_dir, "marked_map_with_edges.csv")
output_csv     = os.path.join(base_dir, "drone_waypoints.csv")
output_img     = os.path.join(base_dir, "drone_waypoints_map.png")

start_point = (-12, 40)

# Load and buffer map
safe_grid     = np.loadtxt(csv_path, delimiter=",", dtype=int)  # 0=free,1=occ
#safe_grid = binary_dilation(grid, iterations=2).astype(int)

# World to Grid converters and vice versa
def px_to_world(col, row):
    return col - 60, 50 - row

def world_to_px(x, y):
    return int(round(x + 60)), int(round(50 - y))

# Nearest free-cell finder
def nearest_free(px, py, grid, max_radius=5):
    if grid[py, px] == 0:
        return (px, py)
    for r in range(1, max_radius+1):
        for dx in range(-r, r+1):
            for dy in range(-r, r+1):
                nx, ny = px+dx, py+dy
                if 0 <= nx < grid.shape[1] and 0 <= ny < grid.shape[0]:
                    if grid[ny, nx] == 0:
                        return (nx, ny)
    return None

binary_img = (safe_grid * 255).astype(np.uint8)
contours, _ = cv2.findContours(binary_img, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

marked_map = safe_grid.copy()
edge_points = []

for contour in contours:
    for pt in contour:
        col, row = pt[0]
        # mark edges
        if 0 <= row < marked_map.shape[0] and 0 <= col < marked_map.shape[1]:
            marked_map[row, col] = 2
        # collect world coords
        edge_points.append(px_to_world(col, row))

# save marked map
np.savetxt(marked_map_csv, marked_map, fmt="%d", delimiter=",")

# visualize detection once
plt.figure(figsize=(6,5))
plt.imshow(marked_map, cmap="viridis", origin="upper", extent=[-60,60,-50,50])
plt.title("Map with Edges Marked (2)")
plt.colorbar(label="0=Free,1=Occ,2=Edge")
plt.show()

# TSP nearest-neighbor to order visits
def tsp_nearest_neighbor(points, start_idx=0):
    visited   = [start_idx]
    unvisited = set(range(len(points))) - {start_idx}
    while unvisited:
        last = visited[-1]
        next_idx = min(unvisited,
                       key=lambda i: np.linalg.norm(np.array(points[last]) - np.array(points[i])))
        visited.append(next_idx)
        unvisited.remove(next_idx)
    return [points[i] for i in visited]

all_points = [start_point] + edge_points
tsp_route  = tsp_nearest_neighbor(all_points)

# Bresenham LOS check
def has_line_of_sight(grid, p1, p2):
    rr, cc = line(p1[1], p1[0], p2[1], p2[0])
    return np.all(grid[rr, cc] == 0)

# Path smoothing
def smooth_path(grid, path):
    if not path:
        return []
    sm = [path[0]]
    i  = 0
    while i < len(path)-1:
        j = len(path)-1
        while j > i+1:
            if has_line_of_sight(grid, path[i], path[j]):
                break
            j -= 1
        sm.append(path[j])
        i = j
    return sm

# 8-dir A*
def astar(grid, start, goal):
    rows, cols = grid.shape
    g_score = {start:0}
    f_score = {start:np.linalg.norm(np.subtract(start, goal))}
    from queue import PriorityQueue
    pq = PriorityQueue(); pq.put((f_score[start], start))
    came_from, visited = {}, set()
    nbrs = [(-1,0),(1,0),(0,-1),(0,1),(-1,-1),(-1,1),(1,-1),(1,1)]

    while not pq.empty():
        _, curr = pq.get()
        if curr == goal:
            path=[]
            while curr in came_from:
                path.append(curr); curr=came_from[curr]
            path.append(start)
            return path[::-1]
        visited.add(curr)
        for dx,dy in nbrs:
            nx,ny = curr[0]+dx, curr[1]+dy
            if 0<=nx<cols and 0<=ny<rows and grid[ny,nx]==0:
                neigh = (nx,ny)
                tentative = g_score[curr] + np.hypot(dx,dy)
                if neigh not in g_score or tentative<g_score[neigh]:
                    came_from[neigh]=curr
                    g_score[neigh]=tentative
                    f_score[neigh]=tentative + np.linalg.norm(np.subtract(neigh,goal))
                    if neigh not in visited:
                        pq.put((f_score[neigh], neigh))
    return []

# Build and smooth the full path
full_path = []
for i in range(len(tsp_route)-1):
    (x1,y1),(x2,y2) = tsp_route[i], tsp_route[i+1]
    p1 = world_to_px(x1,y1)
    p2 = world_to_px(x2,y2)
    start_cell = nearest_free(*p1, safe_grid)
    end_cell   = nearest_free(*p2, safe_grid)
    if not (start_cell and end_cell):
        print(f"Skipping {tsp_route[i]}→{tsp_route[i+1]}")
        continue
    raw = astar(safe_grid, start_cell, end_cell)
    if not raw:
        print(f"A* failed {start_cell}→{end_cell}")
        continue
    for px,py in smooth_path(safe_grid, raw):
        full_path.append(px_to_world(px,py))

# return to home
last_px = nearest_free(*world_to_px(*tsp_route[-1]), safe_grid)
home_px = nearest_free(*world_to_px(*start_point), safe_grid)
if last_px and home_px:
    ret = astar(safe_grid, last_px, home_px)
    for px,py in smooth_path(safe_grid, ret):
        full_path.append(px_to_world(px,py))
else:
    print("Can't return home")

if not full_path:
    raise RuntimeError("No path built")

# Save & visualize final path
np.savetxt(output_csv, full_path, delimiter=",", fmt="%.2f")

plt.figure(figsize=(12,10))
plt.imshow(1-safe_grid, cmap="gray", origin="upper", extent=[-60,60,-50,50])
xs, ys = zip(*full_path)
plt.plot(xs, ys, 'b-', lw=1.2, label="Smoothed Path")
plt.scatter(*start_point, c='green', s=100, marker='*', label="Start")

for i in range(len(full_path)-1):
    dx, dy = xs[i+1]-xs[i], ys[i+1]-ys[i]
    plt.arrow(xs[i], ys[i], dx, dy,
              head_width=0.8, head_length=0.8, fc='orange', ec='orange', alpha=0.5)

plt.title("Drone Path Visiting Only Detected Edges")
plt.xlabel("X (m)"); plt.ylabel("Y (m)")
plt.axis('equal'); plt.grid(True); plt.legend()
plt.savefig(output_img)
plt.show()

print(f"✅ Waypoints saved to: {output_csv}")

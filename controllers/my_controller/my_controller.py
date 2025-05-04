"""my_controller controller."""

# Import necessary Webots controller classes and Python libraries
from controller import Camera, Supervisor, Field, Node, GPS, Compass
import matplotlib.pyplot as plt
import numpy as np
import re
import math
import cv2

# Helper function to check if a given angle is a multiple of 90° (π/2) within a small tolerance
def is_multiple_of_pi_over_two(theta: float, tol: float = 1e-3) -> bool:
    k = round(theta / (math.pi/2))
    return abs(theta - k * (math.pi/2)) < tol

# Initialize Webots Supervisor and timestep
robot = Supervisor()
timestep = int(robot.getBasicTimeStep())

# Access the root node and read a custom field ("title") from the first child node
root = robot.getRoot()
node = root.getField("children").getMFNode(0)
name = node.getField("title").getSFString()

# Depending on the scene title, limit the number of objects considered
limit = 31
if name == "smaller":
    limit = 10

# Parse buildings and their positions/rotations from the scene
buildings = {}
for i in range(6, limit):  # Starting from 6 to skip static elements like ground, etc.
    node = root.getField("children").getMFNode(i)
    pos = node.getField("translation").getSFVec3f()[:2]  # Extract X and Y position
    rot4 = node.getField("rotation").getSFRotation()     # Extract rotation: axis + angle
    name = re.sub(r"\s*\(.*?\)", "", node.getField("name").getSFString()).strip()
    buildings.setdefault(name, []).extend([pos, rot4])

# Define building dimensions with boundary buffer
bounry = 1  # Add 1 meter buffer around each structure
raw_dimensions = {
    "residential tower":      [7.5 + bounry,  5.5 + bounry],
    "large residential tower":[13.0 + bounry, 21.0 + bounry],
    "fast food restaurant":   [10.0 + bounry, 10.0 + bounry]
}
# Normalize keys to lowercase for easier lookup
dimensions = { k.lower(): v for k, v in raw_dimensions.items() }

# Set of all occupied (grid) points
occupied = set()

# For each building type found
for btype, entries in buildings.items():
    key = btype.lower()
    if key not in dimensions:
        print(f"⚠️  Warning: no dimensions for building type '{btype}', skipping.")
        continue

    # Extract dimensions and add extra 2m buffer
    base_w, base_h = dimensions[key]
    w, h = base_w + 2.0, base_h + 2.0
    half_w, half_h = w/2, h/2

    # Iterate through each instance of this building
    for i in range(0, len(entries), 2):
        cx, cy = entries[i]             # Center position
        ax, ay, az, angle = entries[i+1]  # Rotation about Z axis

        # Create rectangle around building (before rotation)
        corners = np.array([
            [+half_w, +half_h],
            [+half_w, -half_h],
            [-half_w, -half_h],
            [-half_w, +half_h]
        ])

        # Construct 2D rotation matrix using yaw
        yaw = angle
        c, s = math.cos(yaw), math.sin(yaw)
        R = np.array([[c, -s], [s, c]])

        # Rotate and shift corners to world coordinates
        rotated_corners = corners @ R.T
        world_corners = rotated_corners + np.array([cx, cy])

        # Compute bounding box around rotated rectangle
        min_x, min_y = np.floor(world_corners.min(axis=0)).astype(int)
        max_x, max_y = np.ceil(world_corners.max(axis=0)).astype(int)
        min_x, max_x = max(min_x, -60), min(max_x, 60)
        min_y, max_y = max(min_y, -50), min(max_y, 50)

        # Mark all grid cells inside bounding box as occupied
        for gx in range(min_x, max_x + 1):
            for gy in range(min_y, max_y + 1):
                if is_multiple_of_pi_over_two(angle):
                    # If clean 90° rotation, use inverse transform to check if inside
                    tx, ty = gx - cx, gy - cy
                    rx = tx * c + ty * s
                    ry = -tx * s + ty * c
                    if -half_w <= rx <= half_w and -half_h <= ry <= half_h:
                        occupied.add((gx, gy))
                else:
                    # Approximate for arbitrary rotation
                    occupied.add((gx, gy))

# Build the occupancy map (101x121 grid centered at [0,0])
map_grid = np.zeros((101, 121), dtype=int)
for ox, oy in occupied:
    map_grid[50 - oy, ox + 60] = 1  # Convert world to grid indices

# Save the occupancy map to CSV
np.savetxt("../drone_controller/occupancy_map.csv", map_grid, fmt="%d", delimiter=",")

# Convert to binary image for contour detection
binary_img = (map_grid * 255).astype(np.uint8)

# Find outer contours of the occupied regions
contours, _ = cv2.findContours(binary_img, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

# Copy map for marking edges
marked_map = map_grid.copy()

# Mark all contour points with '2'
for contour in contours:
    for point in contour:
        col, row = point[0]
        if 0 <= row < marked_map.shape[0] and 0 <= col < marked_map.shape[1]:
            marked_map[row, col] = 2

# Convert marked contour points to world coordinates for visualization/debugging
edge_points_world = []
for contour in contours:
    for point in contour:
        col, row = point[0]
        x = col - 60
        y = 50 - row
        edge_points_world.append((x, y))

# Save the marked map with edges to CSV
np.savetxt("../drone_controller/marked_map_with_edges.csv", marked_map, fmt='%d', delimiter=",")

# Visualization
plt.imshow(marked_map, cmap='viridis')
plt.title("Map with Edges Marked")
plt.colorbar(label="0=Free, 1=Occupied, 2=Edge")
plt.show()

print("Sample Edge Points (world coordinates):")
print(edge_points_world[:10])

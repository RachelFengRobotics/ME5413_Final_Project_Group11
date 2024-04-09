import numpy as np
import open3d as o3d
from PIL import Image
import os

def apply_pass_through_filter(pcd, y_min, y_max):
    # Apply a pass-through filter on the Y-axis
    bounds = np.array([[-np.inf, np.inf], [y_min, y_max], [-np.inf, np.inf]])
    bbox = o3d.geometry.AxisAlignedBoundingBox(min_bound=bounds[:, 0], max_bound=bounds[:, 1])
    filtered_pcd = pcd.crop(bbox)
    return filtered_pcd

def radius_outlier_removal(pcd, radius, min_neighbors):
    # Apply radius outlier removal
    cl, ind = pcd.remove_radius_outlier(nb_points=min_neighbors, radius=radius)
    return pcd.select_by_index(ind, invert=False)

def create_occupancy_grid(pcd, resolution):
    # Adjust to create an occupancy grid based on the X and Z axes, as a pass-through filter is now applied on the Y-axis
    points = np.asarray(pcd.points)
    min_bound = np.min(points, axis=0)
    max_bound = np.max(points, axis=0)
    width = int(np.ceil((max_bound[0] - min_bound[0]) / resolution))
    depth = int(np.ceil((max_bound[2] - min_bound[2]) / resolution))
    grid = np.zeros((width, depth), dtype=np.uint8)

    # Fill the grid, by increasing count in cells
    # Adjust X and Z coordinates, as Y is being filtered
    indices = np.floor((points[:, [0, 2]] - min_bound[[0, 2]]) / resolution).astype(np.int32)
    for idx in indices:
        x_idx, z_idx = idx
        if 0 <= x_idx < width and 0 <= z_idx < depth:
            grid[x_idx, z_idx] = 255  # Mark as occupied

    return grid

def save_pgm(grid, output_path):
    # Invert grid colors: occupied (previously 255) becomes 0, unoccupied becomes 255
    inverted_grid = 255 - grid  # This way, colors are inverted: black becomes white, white becomes black
    
    # Save the inverted occupancy grid as a PGM file
    image = Image.fromarray(inverted_grid)
    image.save(output_path)

# Script parameters
input_pcd_path = 'PCD/LEGO.pcd'
output_pgm_path = 'PCD/LEGO.pgm'
map_resolution = 0.04  # Adjust the resolution of the occupancy grid
y_min = 0  # Lower limit Y for the pass-through filter
y_max = 2.5   # Upper limit Y for the pass-through filter
thre_radius = 0.5  # Radius for outlier filter
thres_point_count = 5  # Minimum neighbor count for outlier filter

# Load PCD file
pcd = o3d.io.read_point_cloud(input_pcd_path)

# Apply a pass-through filter on the Y-axis
filtered_pcd = apply_pass_through_filter(pcd, y_min, y_max)

# Apply radius outlier removal
filtered_pcd = radius_outlier_removal(filtered_pcd, thre_radius, thres_point_count)

# Create an occupancy grid with the specified resolution
occupancy_grid = create_occupancy_grid(filtered_pcd, map_resolution)

# Save the occupancy grid as a PGM file
save_pgm(occupancy_grid, output_pgm_path)

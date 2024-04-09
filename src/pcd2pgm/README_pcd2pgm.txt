## Getting Started

### Prerequisites

Before running this utility, ensure you have the following installed:
- Python 3.6 or higher
- NumPy
- Open3D
- Pillow

### Usage

1. Place your `.pcd` point cloud data files inside the `PCD` directory relative to the script.
2. Open `main.py` (or whatever you've named the script) in a text editor and adjust the script parameters as needed. Specifically, you may want to change the following variables:
   - `input_pcd_path`: The relative path to your input `.pcd` file.
   - `output_pgm_path`: The desired relative path for the output `.pgm` file.
   - `map_resolution`: The resolution for the occupancy grid.
   - `y_min` and `y_max`: The limits for the pass-through filter on the Y-axis.
   - `thre_radius` and `thres_point_count`: Parameters for the radius outlier removal.
3. Run the script with Python:

```sh
python main.py
```

4. After execution, the output `.pgm` file will be located at the path specified in `output_pgm_path`.

## Functions

### `apply_pass_through_filter(pcd, y_min, y_max)`

Applies a pass-through filter on the Y-axis of the point cloud data.

### `radius_outlier_removal(pcd, radius, min_neighbors)`

Removes outliers from the point cloud data based on the specified radius and minimum number of neighbors.

### `create_occupancy_grid(pcd, resolution)`

Creates an occupancy grid from the point cloud data based on the specified resolution.

### `save_pgm(grid, output_path)`

Saves the generated occupancy grid as a PGM file.

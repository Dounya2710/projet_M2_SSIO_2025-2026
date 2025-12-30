"""Coordinate conversion utilities.

These helpers are intentionally ROS/Gazebo-agnostic so they can be used in unit tests
and on machines without ROS installed.

We support a 3D grid addressed as (i, j, k):
- i: row (y axis)
- j: column (x axis)
- k: height layer (z axis)

For a ground robot, simply keep k=0.
"""

from typing import Tuple, Union

# Default grid parameters (kept consistent with representation_env defaults)
cell_size: float = 1.0          # meters for X/Y
cell_size_z: float = 1.0        # meters for Z (layers)
room_size: float = 8.0          # meters for X/Y span
room_height: float = 3.0        # meters for Z span

grid_size: int = int(room_size)           # number of cells in X/Y
grid_height: int = int(room_height)       # number of layers in Z

def set_grid_params(*, cell: float = 1.0, room: float = 8.0, cell_z: float | None = None, height: float = 3.0) -> None:
    """Set global grid parameters.

    Args:
        cell: X/Y cell size in meters.
        room: X/Y room size in meters.
        cell_z: Z cell size in meters (defaults to `cell` if None).
        height: Z room height in meters.
    """
    global cell_size, room_size, grid_size, cell_size_z, room_height, grid_height
    cell_size = float(cell)
    room_size = float(room)
    cell_size_z = float(cell if cell_z is None else cell_z)
    room_height = float(height)

    grid_size = int(round(room_size / cell_size))
    grid_height = max(1, int(round(room_height / cell_size_z)))

def cell_to_world(i: int, j: int, k: int = 0) -> Tuple[float, float, float]:
    """Convert a grid cell (i,j,k) to world coordinates (x,y,z) at the cell center."""
    x = (j + 0.5) * cell_size - room_size / 2
    y = (i + 0.5) * cell_size - room_size / 2
    z = (k + 0.5) * cell_size_z  # z starts at 0
    return x, y, z

def world_to_cell(x: float, y: float, z: float = 0.0) -> Tuple[int, int, int]:
    """Convert world coordinates (x,y,z) to grid cell indices (i,j,k)."""
    j = int((x + room_size / 2) / cell_size)
    i = int((y + room_size / 2) / cell_size)
    k = int(z / cell_size_z)
    return i, j, k

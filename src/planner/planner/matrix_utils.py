"""Coordinate conversion utilities.

These helpers are intentionally ROS/Gazebo-agnostic so they can be used in unit tests
and on machines without ROS installed.
"""
from typing import Tuple

# Default grid parameters (kept consistent with representation_env defaults)
cell_size: float = 1.0
room_size: float = 8.0
grid_size: int = int(room_size)

def set_grid_params(*, cell: float = 1.0, room: float = 8.0) -> None:
    """Override grid parameters used by conversions."""
    global cell_size, room_size, grid_size
    cell_size = float(cell)
    room_size = float(room)
    grid_size = int(round(room_size / cell_size))

def cell_to_world(i: int, j: int) -> Tuple[float, float]:
    """Convert a grid cell (i,j) to world coordinates (x,y) at the cell center."""
    x = (j + 0.5) * cell_size - room_size / 2
    y = (i + 0.5) * cell_size - room_size / 2
    return x, y

def world_to_cell(x: float, y: float) -> Tuple[int, int]:
    """Convert world coordinates (x,y) to grid cell indices (i,j)."""
    j = int((x + room_size / 2) / cell_size)
    i = int((y + room_size / 2) / cell_size)
    return i, j

"""Grid representation utilities.

This module was originally written to build an occupancy grid (matrix) from a Gazebo SDF
environment file. In the original version, the SDF was parsed **at import time**, which
made the package unusable on machines where the SDF file is not present (e.g. outside
the ROS/Gazebo lab).

This version is safe to import anywhere:
- the grid is always initialized
- SDF parsing is performed only when explicitly requested (or when the default SDF exists)
"""

import math
import os
import xml.etree.ElementTree as ET
from typing import List, Optional

# -----------------------------
# Grid parameters (default 8x8, 1m per cell)
# -----------------------------
cell_size: float = 1.0
room_size: int = 8
grid_size: int = room_size

# Grid initialised to 0 (free)
matrix: List[List[int]] = [[0 for _ in range(grid_size)] for _ in range(grid_size)]

# -----------------------------
# Helpers
# -----------------------------
IGNORE_MODEL_NAMES = {
    "ground_plane", "ground", "floor", "world", "plane", "room"
}
BIG_SIZE_THRESHOLD = room_size * 0.9  # e.g. > 7.2m => consider as floor/walls


def reset_matrix(value: int = 0) -> None:
    """Reset the global matrix in-place."""
    for i in range(grid_size):
        for j in range(grid_size):
            matrix[i][j] = value


def is_big_structure(sx: float, sy: float) -> bool:
    """Return True if a box is so large that it should be treated as structural (floor/walls)."""
    return sx >= BIG_SIZE_THRESHOLD or sy >= BIG_SIZE_THRESHOLD


def _cell_bounds(i: int, j: int):
    # Center the room at (0,0): x,y in [-room_size/2, room_size/2]
    cell_x_min = j * cell_size - room_size / 2
    cell_x_max = (j + 1) * cell_size - room_size / 2
    cell_y_min = i * cell_size - room_size / 2
    cell_y_max = (i + 1) * cell_size - room_size / 2
    return cell_x_min, cell_x_max, cell_y_min, cell_y_max


def add_obstacle_to_matrix(x: float, y: float, sx: float, sy: float, *, model_name: str = "") -> None:
    """Mark the cells intersecting an obstacle box of size (sx, sy) centered at (x, y) as 1."""
    x_min = x - sx / 2
    x_max = x + sx / 2
    y_min = y - sy / 2
    y_max = y + sy / 2

    for i in range(grid_size):
        for j in range(grid_size):
            cell_x_min, cell_x_max, cell_y_min, cell_y_max = _cell_bounds(i, j)

            # Intersection test (axis-aligned)
            if (x_min < cell_x_max and x_max > cell_x_min and y_min < cell_y_max and y_max > cell_y_min):
                matrix[i][j] = 1


def _get_model_pose_xy(model) -> tuple[float, float]:
    pose = model.find("pose")
    if pose is None or not (pose.text or "").strip():
        return 0.0, 0.0
    parts = pose.text.split()
    try:
        x, y = float(parts[0]), float(parts[1])
    except Exception:
        x = y = 0.0
    return x, y


def load_matrix_from_sdf(sdf_path: str, *, clear_first: bool = True) -> List[List[int]]:
    """Parse an SDF file and fill the global grid matrix.

    Parameters
    ----------
    sdf_path:
        Path to the SDF file.
    clear_first:
        If True, reset the matrix to 0 before adding obstacles.

    Returns
    -------
    The global matrix (also updated in place).
    """
    if clear_first:
        reset_matrix(0)

    tree = ET.parse(sdf_path)
    root = tree.getroot()

    for model in root.findall(".//model"):
        name = (model.get("name", "") or "").lower()

        # Ignore floor/world etc.
        if any(ign in name for ign in IGNORE_MODEL_NAMES):
            continue

        static = model.find("static")
        if static is None or (static.text or "").strip().lower() != "true":
            continue  # keep only static obstacles

        x, y = _get_model_pose_xy(model)

        link = model.find("link")
        if link is None:
            continue
        collision = link.find("collision")
        if collision is None:
            continue
        geometry = collision.find("geometry")
        if geometry is None:
            continue

        # BOX
        box = geometry.find("box")
        if box is not None:
            size = box.find("size")
            if size is None or not (size.text or "").strip():
                continue
            try:
                sx, sy, _sz = map(float, size.text.split())
            except Exception:
                continue

            if is_big_structure(sx, sy):
                continue

            add_obstacle_to_matrix(x, y, sx, sy, model_name=name)
            continue

        # CYLINDER (approximated as a box)
        cylinder = geometry.find("cylinder")
        if cylinder is not None:
            radius_elem = cylinder.find("radius")
            length_elem = cylinder.find("length")
            if radius_elem is None or length_elem is None:
                continue
            try:
                radius = float(radius_elem.text)
            except Exception:
                continue

            sx = sy = 2 * radius
            if is_big_structure(sx, sy):
                continue

            add_obstacle_to_matrix(x, y, sx, sy, model_name=name)
            continue

    return matrix


# Try to load a default SDF if it exists (keeps import safe).
_DEFAULT_SDF = os.path.normpath(os.path.join(os.path.dirname(__file__), "..", "..", "..", "env", "env.sdf"))
if os.path.exists(_DEFAULT_SDF):
    try:
        load_matrix_from_sdf(_DEFAULT_SDF, clear_first=True)
    except Exception:
        # Keep an empty grid if parsing fails.
        reset_matrix(0)


if __name__ == "__main__":
    # Debug view
    for row in matrix:
        print(" ".join(str(cell) for cell in row))

"""Grid representation utilities (3D).

We represent the environment as a 3D occupancy grid addressed by (i, j, k):
- i: row (y axis)
- j: column (x axis)
- k: height layer (z axis)

This module can:
- build a default empty grid (always safe to import)
- optionally parse a Gazebo SDF file to mark obstacles
"""

from __future__ import annotations

import math
import os
import xml.etree.ElementTree as ET
from typing import List, Tuple

from planner.matrix_utils import (
    cell_size, cell_size_z, room_size, room_height, grid_size, grid_height, set_grid_params
)

# -----------------------------
# Grid parameters (defaults)
# -----------------------------

# Grid initialised to 0 (free)
matrix: List[List[List[int]]] = [
    [[0 for _ in range(grid_height)] for _ in range(grid_size)]
    for _ in range(grid_size)
]

# -----------------------------
# Helpers
# -----------------------------
IGNORE_MODEL_NAMES = {"ground_plane", "ground", "floor", "world", "plane", "room"}
BIG_SIZE_THRESHOLD = room_size * 0.9  # e.g. > 7.2m => consider as floor/walls


def rebuild_empty_matrix(value: int = 0) -> None:
    """Rebuild the global matrix with current grid parameters."""
    global matrix
    matrix = [
        [[value for _ in range(grid_height)] for _ in range(grid_size)]
        for _ in range(grid_size)
    ]


def reset_matrix(value: int = 0) -> None:
    """Reset the global matrix in-place."""
    for i in range(grid_size):
        for j in range(grid_size):
            for k in range(grid_height):
                matrix[i][j][k] = value


def is_big_structure(sx: float, sy: float) -> bool:
    """Return True if a box is so large that it should be treated as structural (floor/walls)."""
    return sx >= BIG_SIZE_THRESHOLD or sy >= BIG_SIZE_THRESHOLD


def _cell_bounds_xy(i: int, j: int) -> Tuple[float, float, float, float]:
    # Center the room at (0,0): x,y in [-room_size/2, room_size/2]
    cell_x_min = j * cell_size - room_size / 2
    cell_x_max = (j + 1) * cell_size - room_size / 2
    cell_y_min = i * cell_size - room_size / 2
    cell_y_max = (i + 1) * cell_size - room_size / 2
    return cell_x_min, cell_x_max, cell_y_min, cell_y_max


def _layer_bounds_z(k: int) -> Tuple[float, float]:
    # We consider z in [0, room_height], layers from ground upward.
    z_min = k * cell_size_z
    z_max = (k + 1) * cell_size_z
    return z_min, z_max


def add_obstacle_to_matrix(
    x: float,
    y: float,
    z: float,
    sx: float,
    sy: float,
    sz: float,
    *,
    model_name: str = ""
) -> None:
    """Mark obstacle cells that intersect an axis-aligned box obstacle."""
    x_min = x - sx / 2
    x_max = x + sx / 2
    y_min = y - sy / 2
    y_max = y + sy / 2
    z_min = z - sz / 2
    z_max = z + sz / 2

    for i in range(grid_size):
        for j in range(grid_size):
            cell_x_min, cell_x_max, cell_y_min, cell_y_max = _cell_bounds_xy(i, j)
            # XY intersection
            if not (x_min < cell_x_max and x_max > cell_x_min and y_min < cell_y_max and y_max > cell_y_min):
                continue

            for k in range(grid_height):
                layer_z_min, layer_z_max = _layer_bounds_z(k)
                if z_min < layer_z_max and z_max > layer_z_min:
                    matrix[i][j][k] = 1


def _get_model_pose_xyz(model) -> Tuple[float, float, float]:
    pose = model.find("pose")
    if pose is None or not (pose.text or "").strip():
        return 0.0, 0.0, 0.0
    parts = pose.text.split()
    try:
        x = float(parts[0]); y = float(parts[1]); z = float(parts[2])
    except Exception:
        x = y = z = 0.0
    return x, y, z


def _model_name(model) -> str:
    return model.get("name") or ""


def load_matrix_from_sdf(sdf_path: str, *, clear_first: bool = True) -> List[List[List[int]]]:
    """Parse an SDF file and fill the global 3D grid matrix."""
    if clear_first:
        reset_matrix(0)

    tree = ET.parse(sdf_path)
    root = tree.getroot()

    for model in root.iter("model"):
        name = _model_name(model)
        if name in IGNORE_MODEL_NAMES:
            continue

        x, y, z = _get_model_pose_xyz(model)

        # Find collision geometry
        collision = model.find(".//collision")
        if collision is None:
            continue
        geom = collision.find("geometry")
        if geom is None:
            continue

        # Box geometry
        box = geom.find("box")
        if box is not None:
            size = box.find("size")
            if size is None or not (size.text or "").strip():
                continue
            try:
                sx, sy, sz = (float(v) for v in size.text.split()[:3])
            except Exception:
                continue

            if is_big_structure(sx, sy):
                continue

            add_obstacle_to_matrix(x, y, z, sx, sy, sz, model_name=name)
            continue

        # Cylinder geometry
        cylinder = geom.find("cylinder")
        if cylinder is not None:
            radius_elem = cylinder.find("radius")
            length_elem = cylinder.find("length")
            if radius_elem is None or length_elem is None:
                continue
            try:
                radius = float(radius_elem.text)
                length = float(length_elem.text)
            except Exception:
                continue

            sx = sy = 2 * radius
            sz = length
            if is_big_structure(sx, sy):
                continue

            add_obstacle_to_matrix(x, y, z, sx, sy, sz, model_name=name)
            continue

        # Sphere geometry (rare)
        sphere = geom.find("sphere")
        if sphere is not None:
            radius_elem = sphere.find("radius")
            if radius_elem is None:
                continue
            try:
                radius = float(radius_elem.text)
            except Exception:
                continue
            sx = sy = sz = 2 * radius
            if is_big_structure(sx, sy):
                continue
            add_obstacle_to_matrix(x, y, z, sx, sy, sz, model_name=name)
            continue

    return matrix


def pretty_print_layer(k: int = 0) -> None:
    """Print one Z layer (k) as a 2D grid (useful for debugging without ROS)."""
    if not (0 <= k < grid_height):
        raise ValueError(f"k must be in [0,{grid_height-1}]")
    for i in range(grid_size):
        print(" ".join(str(matrix[i][j][k]) for j in range(grid_size)))


def auto_load_default_sdf() -> None:
    """Best-effort: if env.sdf exists next to the project, load it."""
    # Try a few common locations
    candidates = []
    # 1) Current working directory
    candidates.append(os.path.join(os.getcwd(), "env.sdf"))
    # 2) Package directory
    candidates.append(os.path.join(os.path.dirname(__file__), "..", "..", "..", "env", "env.sdf"))

    for p in candidates:
        p = os.path.abspath(p)
        if os.path.exists(p):
            try:
                load_matrix_from_sdf(p, clear_first=True)
            except Exception:
                pass
            return

if __name__ == "__main__":
    # Debug view
    for row in matrix:
        auto_load_default_sdf()
        print(" ".join(str(cell) for cell in row))




# Shared test helpers for path planning algorithms.

from typing import List, Tuple

Cell = Tuple[int,int]

def sample_grid() -> List[List[int]]:
    # 8x8 grid, 1 = obstacle
    # A narrow corridor exists from (0,0) to (7,7)
    return [
        [0,0,0,0,0,0,0,0],
        [1,1,1,1,1,1,1,0],
        [0,0,0,0,0,0,1,0],
        [0,1,1,1,1,0,1,0],
        [0,0,0,0,1,0,0,0],
        [0,1,1,0,1,1,1,0],
        [0,0,0,0,0,0,0,0],
        [0,1,1,1,1,1,1,0],
    ]

def is_free(c: Cell, grid: List[List[int]]) -> bool:
    i,j=c
    return 0 <= i < len(grid) and 0 <= j < len(grid[0]) and grid[i][j]==0

def is_4_neighbor(a: Cell, b: Cell) -> bool:
    return abs(a[0]-b[0]) + abs(a[1]-b[1]) == 1

def validate_path(path: List[Cell], start: Cell, goal: Cell, grid: List[List[int]]) -> None:
    assert isinstance(path, list)
    assert path, "Path is empty"
    assert path[0] == start, f"Path must start at {start}, got {path[0]}"
    assert path[-1] == goal, f"Path must end at {goal}, got {path[-1]}"
    for c in path:
        assert is_free(c, grid), f"Cell {c} is not free"
    for a,b in zip(path, path[1:]):
        assert is_4_neighbor(a,b), f"Non 4-neighbor step: {a} -> {b}"

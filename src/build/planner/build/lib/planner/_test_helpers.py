from typing import List, Tuple

Cell = Tuple[int, int, int]  # (i, j, k)


def sample_grid_2d() -> List[List[int]]:
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


def sample_grid() -> List[List[List[int]]]:
    # One-layer 3D grid: matrix[i][j][k]
    g2 = sample_grid_2d()
    return [[[g2[i][j]] for j in range(len(g2[0]))] for i in range(len(g2))]


def is_free(c: Cell, grid: List[List[List[int]]]) -> bool:
    i, j, k = c
    return (
        0 <= i < len(grid)
        and 0 <= j < len(grid[0])
        and 0 <= k < len(grid[0][0])
        and grid[i][j][k] == 0
    )


def is_neighbor(a: Cell, b: Cell) -> bool:
    # 6-neighborhood (will reduce to 4-neighborhood if k is constant)
    return abs(a[0]-b[0]) + abs(a[1]-b[1]) + abs(a[2]-b[2]) == 1


def validate_path(path: List[Cell], start: Cell, goal: Cell, grid: List[List[List[int]]]) -> None:
    assert path, "Path should not be empty"
    assert path[0] == start, f"Start mismatch: {path[0]} != {start}"
    assert path[-1] == goal, f"Goal mismatch: {path[-1]} != {goal}"
    for idx, c in enumerate(path):
        assert is_free(c, grid), f"Path goes through obstacle/out-of-bounds at {c}"
        if idx > 0:
            assert is_neighbor(path[idx - 1], c), f"Non-neighbor move: {path[idx - 1]} -> {c}"

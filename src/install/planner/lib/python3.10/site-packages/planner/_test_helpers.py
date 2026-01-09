from typing import List, Tuple, Sequence, Any

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

def _split_cell(c: Sequence[int]) -> Cell:
    """
    Accepte (i,j) ou (i,j,k) et renvoie toujours (i,j,k), avec k=0 par défaut.
    """
    if len(c) == 2:
        i, j = c
        return int(i), int(j), 0
    if len(c) == 3:
        i, j, k = c
        return int(i), int(j), int(k)
    raise ValueError(f"Cell has unexpected format: {c}")

def sample_grid() -> List[List[List[int]]]:
    # One-layer 3D grid: matrix[i][j][k]
    g2 = sample_grid_2d()
    return [[[g2[i][j]] for j in range(len(g2[0]))] for i in range(len(g2))]

def _grid_is_2d(grid: Any) -> bool:
    """
    Retourne True si la grille est de type 2D: grid[i][j] = int (0/1).
    """
    return isinstance(grid[0][0], int)

def is_free(c: Cell, grid: List[List[List[int]]]) -> bool:
    i, j, k = _split_cell(c)

    if _grid_is_2d(grid):
        if i < 0 or j < 0 or i >= len(grid) or j >= len(grid[0]):
            return False
        return grid[i][j] == 0

    # grille 3D
    if i < 0 or j < 0 or i >= len(grid) or j >= len(grid[0]):
        return False
    if k < 0 or k >= len(grid[i][j]):
        return False
    return grid[i][j][k] == 0


def is_neighbor(a: Cell, b: Cell) -> bool:
    # 6-neighborhood (will reduce to 4-neighborhood if k is constant)
    ai, aj, ak = _split_cell(a)
    bi, bj, bk = _split_cell(b)
    return abs(ai - bi) + abs(aj - bj) + abs(ak - bk) == 1


def validate_path(path: List[Cell], start: Cell, goal: Cell, grid: List[List[List[int]]]) -> None:
    assert path, "Path should not be empty"
    assert path[0] == start, f"Start mismatch: {path[0]} != {start}"
    assert path[-1] == goal, f"Goal mismatch: {path[-1]} != {goal}"
    for idx, c in enumerate(path):
        assert is_free(c, grid), f"Path goes through obstacle/out-of-bounds at {c}"
        if idx > 0:
            assert is_neighbor(path[idx - 1], c), f"Non-neighbor move: {path[idx - 1]} -> {c}"

import heapq
from typing import Dict, List, Tuple, Union, Sequence

Cell2D = Tuple[int, int]
Cell3D = Tuple[int, int, int]
Cell = Union[Cell2D, Cell3D]


def _grid_dims(matrix) -> Tuple[int, int, int]:
    n_i = len(matrix)
    n_j = len(matrix[0]) if n_i else 0
    first = matrix[0][0] if (n_i and n_j) else 0
    if isinstance(first, list):
        n_k = len(first)
    else:
        n_k = 1
    return n_i, n_j, n_k


def _normalize_cell(cell: Cell) -> Cell3D:
    if not isinstance(cell, (tuple, list)):
        raise ValueError(f"cell doit être un tuple/list, reçu: {cell}")
    if len(cell) == 2:
        i, j = cell
        return int(i), int(j), 0
    if len(cell) == 3:
        i, j, k = cell
        return int(i), int(j), int(k)
    raise ValueError(f"cell doit être (i,j) ou (i,j,k), reçu: {cell}")


def _in_bounds(cell: Cell3D, dims: Tuple[int, int, int]) -> bool:
    i, j, k = cell
    ni, nj, nk = dims
    return 0 <= i < ni and 0 <= j < nj and 0 <= k < nk


def _is_free(cell: Cell3D, matrix) -> bool:
    i, j, k = cell
    if _grid_dims(matrix)[2] == 1:
        return matrix[i][j] == 0
    return matrix[i][j][k] == 0


def _heuristic(a: Cell3D, b: Cell3D) -> float:
    # Manhattan distance (works for both 2D and 3D; k=0 for 2D)
    return abs(a[0] - b[0]) + abs(a[1] - b[1]) + abs(a[2] - b[2])


def _neighbors(cell: Cell3D, matrix) -> List[Cell3D]:
    dims = _grid_dims(matrix)
    i, j, k = cell
    cand = [
        (i - 1, j, k), (i + 1, j, k),
        (i, j - 1, k), (i, j + 1, k),
    ]
    if dims[2] > 1:
        cand += [(i, j, k - 1), (i, j, k + 1)]

    out: List[Cell3D] = []
    for c in cand:
        if _in_bounds(c, dims) and _is_free(c, matrix):
            out.append(c)
    return out


def _reconstruct(came: Dict[Cell3D, Cell3D], start: Cell3D, goal: Cell3D) -> List[Cell3D]:
    cur = goal
    path = [cur]
    while cur != start:
        cur = came.get(cur)
        if cur is None:
            return []
        path.append(cur)
    path.reverse()
    return path


def a_star(start: Cell, goal: Cell, matrix) -> List[Cell3D]:
    """A* shortest path on a 2D or 3D occupancy grid."""
    s = _normalize_cell(start)
    g = _normalize_cell(goal)
    dims = _grid_dims(matrix)

    if not _in_bounds(s, dims) or not _in_bounds(g, dims):
        return []
    if not _is_free(s, matrix) or not _is_free(g, matrix):
        return []

    open_heap: List[Tuple[float, Cell3D]] = []
    heapq.heappush(open_heap, (0.0, s))

    came_from: Dict[Cell3D, Cell3D] = {}
    g_score: Dict[Cell3D, float] = {s: 0.0}
    f_score: Dict[Cell3D, float] = {s: _heuristic(s, g)}

    closed = set()

    while open_heap:
        _, current = heapq.heappop(open_heap)
        if current in closed:
            continue
        closed.add(current)

        if current == g:
            return _reconstruct(came_from, s, g)

        for nb in _neighbors(current, matrix):
            tentative_g = g_score[current] + 1.0
            if tentative_g < g_score.get(nb, float("inf")):
                came_from[nb] = current
                g_score[nb] = tentative_g
                f = tentative_g + _heuristic(nb, g)
                f_score[nb] = f
                heapq.heappush(open_heap, (f, nb))

    return []

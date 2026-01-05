import heapq
from typing import Dict, List, Tuple, Sequence, Union

Cell2D = Tuple[int, int]
Cell3D = Tuple[int, int, int]
Cell = Union[Cell2D, Cell3D]


def _cell_dim(cell: Sequence[int]) -> int:
    return len(cell)


def _grid_dims(matrix) -> Tuple[int, int, int]:
    # matrix[i][j][k] (3D) or matrix[i][j] (2D)
    n_i = len(matrix)
    n_j = len(matrix[0]) if n_i else 0
    # Detect 3D by looking at element type
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
    # 2D fallback
    if _grid_dims(matrix)[2] == 1:
        return matrix[i][j] == 0
    return matrix[i][j][k] == 0


def _neighbors(cell: Cell3D, matrix) -> List[Cell3D]:
    dims = _grid_dims(matrix)
    i, j, k = cell
    cand = [
        (i - 1, j, k), (i + 1, j, k),
        (i, j - 1, k), (i, j + 1, k),
    ]
    # Allow vertical moves if 3D grid
    if dims[2] > 1:
        cand += [(i, j, k - 1), (i, j, k + 1)]

    out: List[Cell3D] = []
    for c in cand:
        if _in_bounds(c, dims) and _is_free(c, matrix):
            out.append(c)
    return out


def _reconstruct(prev: Dict[Cell3D, Cell3D], start: Cell3D, goal: Cell3D) -> List[Cell3D]:
    path: List[Cell3D] = []
    cur = goal
    while cur != start:
        path.append(cur)
        cur = prev.get(cur)
        if cur is None:
            return []
    path.append(start)
    path.reverse()
    return path


def dijkstra(start: Cell, goal: Cell, matrix) -> List[Cell3D]:
    """Dijkstra shortest path on a 2D or 3D occupancy grid."""
    s = _normalize_cell(start)
    g = _normalize_cell(goal)
    dims = _grid_dims(matrix)

    if not _in_bounds(s, dims) or not _in_bounds(g, dims):
        return []
    if not _is_free(s, matrix) or not _is_free(g, matrix):
        return []

    dist: Dict[Cell3D, float] = {s: 0.0}
    prev: Dict[Cell3D, Cell3D] = {}
    pq = [(0.0, s)]
    visited = set()

    while pq:
        d, u = heapq.heappop(pq)
        if u in visited:
            continue
        visited.add(u)

        if u == g:
            return _reconstruct(prev, s, g)

        for v in _neighbors(u, matrix):
            nd = d + 1.0
            if nd < dist.get(v, float("inf")):
                dist[v] = nd
                prev[v] = u
                heapq.heappush(pq, (nd, v))

    return []

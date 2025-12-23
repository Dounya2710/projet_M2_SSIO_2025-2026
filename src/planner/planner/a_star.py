import heapq
from typing import Dict, List, Optional, Tuple

Cell = Tuple[int, int]  # (i, j)


def _heuristic(a: Cell, b: Cell) -> int:
    # Manhattan (grille 4-connexe)
    return abs(a[0] - b[0]) + abs(a[1] - b[1])


def _in_bounds(cell: Cell, grid_size: int) -> bool:
    i, j = cell
    return 0 <= i < grid_size and 0 <= j < grid_size


def _is_free(cell: Cell, matrix: List[List[int]]) -> bool:
    i, j = cell
    return matrix[i][j] == 0


def _neighbors(cell: Cell, matrix: List[List[int]]) -> List[Cell]:
    grid_size = len(matrix)
    i, j = cell
    cand = [(i - 1, j), (i + 1, j), (i, j - 1), (i, j + 1)]
    out = []
    for c in cand:
        if _in_bounds(c, grid_size) and _is_free(c, matrix):
            out.append(c)
    return out


def a_star(start: Cell, goal: Cell, matrix: List[List[int]]) -> List[Cell]:
    """
    A* sur grille 2D.
    Retourne une liste de cellules [start, ..., goal] ou [] si impossible.
    """
    if not matrix:
        return []
    grid_size = len(matrix)

    if not _in_bounds(start, grid_size) or not _in_bounds(goal, grid_size):
        return []
    if not _is_free(start, matrix) or not _is_free(goal, matrix):
        return []

    open_heap: List[Tuple[int, int, Cell]] = []
    heapq.heappush(open_heap, (0, 0, start))

    came_from: Dict[Cell, Cell] = {}
    g_cost: Dict[Cell, int] = {start: 0}

    tie = 0
    while open_heap:
        _, _, current = heapq.heappop(open_heap)

        if current == goal:
            # reconstruction
            path = [current]
            while current in came_from:
                current = came_from[current]
                path.append(current)
            path.reverse()
            return path

        for nxt in _neighbors(current, matrix):
            tentative_g = g_cost[current] + 1
            if nxt not in g_cost or tentative_g < g_cost[nxt]:
                g_cost[nxt] = tentative_g
                f = tentative_g + _heuristic(nxt, goal)
                tie += 1
                heapq.heappush(open_heap, (f, tie, nxt))
                came_from[nxt] = current

    return []

import heapq
from typing import Dict, List, Tuple

Cell = Tuple[int, int]  # (i, j)


def _in_bounds(cell, grid_size):
    if not isinstance(cell, (tuple, list)) or len(cell) != 2:
        raise ValueError(f"cell doit être un (i,j) à 2 valeurs, reçu: {cell}")
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


def dijkstra(start: Cell, goal: Cell, matrix: List[List[int]]) -> List[Cell]:
    """
    Dijkstra sur grille 2D (coût uniforme = 1).
    Retourne une liste de cellules [start, ..., goal] ou [] si impossible.
    """
    if not matrix:
        return []
    grid_size = len(matrix)

    if not _in_bounds(start, grid_size) or not _in_bounds(goal, grid_size):
        return []
    if not _is_free(start, matrix) or not _is_free(goal, matrix):
        return []

    pq: List[Tuple[int, Cell]] = []
    heapq.heappush(pq, (0, start))

    came_from: Dict[Cell, Cell] = {}
    cost: Dict[Cell, int] = {start: 0}

    while pq:
        cur_cost, current = heapq.heappop(pq)
        if current == goal:
            break
        if cur_cost != cost[current]:
            continue

        for nxt in _neighbors(current, matrix):
            new_cost = cost[current] + 1
            if nxt not in cost or new_cost < cost[nxt]:
                cost[nxt] = new_cost
                came_from[nxt] = current
                heapq.heappush(pq, (new_cost, nxt))

    if goal not in cost:
        return []

    # reconstruction
    path = [goal]
    cur = goal
    while cur != start:
        cur = came_from.get(cur)
        if cur is None:
            return []
        path.append(cur)
    path.reverse()
    return path

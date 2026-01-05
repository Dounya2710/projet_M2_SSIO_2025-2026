import random
from typing import Dict, List, Tuple, Union

Cell2D = Tuple[int, int]
Cell3D = Tuple[int, int, int]
Cell = Union[Cell2D, Cell3D]

# Actions: (di, dj, dk)
ACTIONS_2D = [(-1, 0, 0), (1, 0, 0), (0, -1, 0), (0, 1, 0)]
ACTIONS_3D = ACTIONS_2D + [(0, 0, -1), (0, 0, 1)]


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
    if len(cell) == 2:
        return int(cell[0]), int(cell[1]), 0
    if len(cell) == 3:
        return int(cell[0]), int(cell[1]), int(cell[2])
    raise ValueError(f"cell doit être (i,j) ou (i,j,k), reçu: {cell}")


def _in_bounds(s: Cell3D, dims: Tuple[int, int, int]) -> bool:
    i, j, k = s
    ni, nj, nk = dims
    return 0 <= i < ni and 0 <= j < nj and 0 <= k < nk


def _is_free(s: Cell3D, matrix) -> bool:
    i, j, k = s
    if _grid_dims(matrix)[2] == 1:
        return matrix[i][j] == 0
    return matrix[i][j][k] == 0


def _step(state: Cell3D, action: Tuple[int, int, int], matrix) -> Cell3D:
    dims = _grid_dims(matrix)
    ni, nj, nk = dims
    i, j, k = state
    di, dj, dk = action
    ns = (i + di, j + dj, k + dk)
    if not _in_bounds(ns, dims):
        return state
    if not _is_free(ns, matrix):
        return state
    return ns


def q_learning_plan(
    start: Cell,
    goal: Cell,
    matrix,
    *,
    episodes: int = 800,
    alpha: float = 0.2,
    gamma: float = 0.95,
    epsilon: float = 0.25,
    max_steps: int = 800,
) -> List[Cell3D]:
    """Learn a simple Q-learning policy on the fly and extract a greedy path.

    Notes:
      - This is meant for *demo/testing* (not a production RL planner).
      - For a ground robot, keep k=0 and a 1-layer grid.
    """
    s0 = _normalize_cell(start)
    g = _normalize_cell(goal)
    dims = _grid_dims(matrix)

    if not _in_bounds(s0, dims) or not _in_bounds(g, dims):
        return []
    if not _is_free(s0, matrix) or not _is_free(g, matrix):
        return []

    actions = ACTIONS_3D if dims[2] > 1 else ACTIONS_2D

    # Q[(state, action_index)] = value
    Q: Dict[Tuple[Cell3D, int], float] = {}

    def reward(state: Cell3D) -> float:
        if state == g:
            return 100.0
        return -1.0  # step cost

    # Training
    for _ in range(episodes):
        state = s0
        for _t in range(max_steps):
            if state == g:
                break
            # epsilon-greedy
            if random.random() < epsilon:
                a_idx = random.randrange(len(actions))
            else:
                # pick best action
                vals = [Q.get((state, idx), 0.0) for idx in range(len(actions))]
                a_idx = max(range(len(actions)), key=lambda idx: vals[idx])

            ns = _step(state, actions[a_idx], matrix)
            r = reward(ns)

            # next best
            next_vals = [Q.get((ns, idx), 0.0) for idx in range(len(actions))]
            best_next = max(next_vals) if next_vals else 0.0

            old = Q.get((state, a_idx), 0.0)
            Q[(state, a_idx)] = old + alpha * (r + gamma * best_next - old)

            state = ns

    # Extract greedy path
    path: List[Cell3D] = [s0]
    state = s0
    seen = {state}
    for _ in range(max_steps):
        if state == g:
            return path
        vals = [Q.get((state, idx), 0.0) for idx in range(len(actions))]
        a_idx = max(range(len(actions)), key=lambda idx: vals[idx])
        ns = _step(state, actions[a_idx], matrix)
        if ns == state or ns in seen:
            # stuck
            break
        path.append(ns)
        seen.add(ns)
        state = ns

    return []

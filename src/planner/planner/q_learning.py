import random
from typing import Dict, List, Tuple

Cell = Tuple[int, int]  # (i, j)
Action = int            # 0:up 1:down 2:left 3:right

ACTIONS = [0, 1, 2, 3]
DELTA = {
    0: (-1, 0),
    1: (1, 0),
    2: (0, -1),
    3: (0, 1),
}


def _in_bounds(cell: Cell, grid_size: int) -> bool:
    i, j = cell
    return 0 <= i < grid_size and 0 <= j < grid_size


def _is_free(cell: Cell, matrix: List[List[int]]) -> bool:
    i, j = cell
    return matrix[i][j] == 0


def _step(cell: Cell, action: Action, matrix: List[List[int]]) -> Tuple[Cell, float, bool]:
    """
    Applique une action.
    - si on sort ou obstacle => on reste sur place et pénalité.
    - reward: -1 par pas, +100 si goal (géré ailleurs), -10 si collision.
    """
    grid_size = len(matrix)
    di, dj = DELTA[action]
    nxt = (cell[0] + di, cell[1] + dj)

    # tentative de déplacement
    if not _in_bounds(nxt, grid_size) or not _is_free(nxt, matrix):
        return cell, -10.0, False  # collision / mur => pénalité, pas terminal
    return nxt, -1.0, False


def q_learning_plan(
    start: Cell,
    goal: Cell,
    matrix: List[List[int]],
    episodes: int = 4000,
    alpha: float = 0.2,
    gamma: float = 0.95,
    epsilon: float = 0.2,
    max_steps_per_episode: int = 200,
    max_steps_extract: int = 200,
) -> List[Cell]:
    """
    Entraîne un Q-learning tabulaire puis extrait un chemin start->goal.
    Retourne [] si non trouvé / boucle.
    """
    if not matrix:
        return []
    grid_size = len(matrix)

    if not _in_bounds(start, grid_size) or not _in_bounds(goal, grid_size):
        return []
    if not _is_free(start, matrix) or not _is_free(goal, matrix):
        return []

    # Q-table: dict[(cell, action)] -> value
    Q: Dict[Tuple[Cell, Action], float] = {}

    def q(cell: Cell, action: Action) -> float:
        return Q.get((cell, action), 0.0)

    def best_action(cell: Cell) -> Action:
        # argmax_a Q(s,a)
        vals = [(q(cell, a), a) for a in ACTIONS]
        vals.sort(reverse=True, key=lambda x: x[0])
        return vals[0][1]

    # Training
    for _ in range(episodes):
        s = start

        for _step_idx in range(max_steps_per_episode):
            # epsilon-greedy
            if random.random() < epsilon:
                a = random.choice(ACTIONS)
            else:
                a = best_action(s)

            s2, r, _ = _step(s, a, matrix)

            # bonus si on atteint le goal
            if s2 == goal:
                r = 100.0

            # update
            a2 = best_action(s2)
            td_target = r + gamma * q(s2, a2)
            td_error = td_target - q(s, a)
            Q[(s, a)] = q(s, a) + alpha * td_error

            s = s2
            if s == goal:
                break

        # petit decay (optionnel) pour stabiliser
        epsilon = max(0.05, epsilon * 0.999)

    # Extract path from learned policy
    path = [start]
    visited = set([start])
    s = start

    for _ in range(max_steps_extract):
        if s == goal:
            return path

        a = best_action(s)
        s2, _, _ = _step(s, a, matrix)

        # si bloqué (action mène nulle part), on tente une action alternative
        if s2 == s:
            alt = ACTIONS[:]
            random.shuffle(alt)
            moved = False
            for a_alt in alt:
                t, _, _ = _step(s, a_alt, matrix)
                if t != s:
                    s2 = t
                    moved = True
                    break
            if not moved:
                return []

        if s2 in visited:
            return []  # boucle => échec extraction
        visited.add(s2)
        path.append(s2)
        s = s2

    return []


# Backward/short alias (used in local tests)
q_learning = q_learning_plan

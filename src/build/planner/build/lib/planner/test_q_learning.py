import random

from planner.q_learning import q_learning_plan
from planner._test_helpers import sample_grid, validate_path

def main() -> None:
    random.seed(0)

    grid = sample_grid()
    start = (0, 0)
    goal = (6, 7)

    path = q_learning_plan(
        start,
        goal,
        grid,
        episodes=6000,
        alpha=0.2,
        gamma=0.95,
        epsilon=0.2,
        max_steps_per_episode=200,
        max_steps_extract=200,
    )
    validate_path(path, start, goal, grid)
    print("Q-learning OK | length =", len(path))

if __name__ == "__main__":
    main()

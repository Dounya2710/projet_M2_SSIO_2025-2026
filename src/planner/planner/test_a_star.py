from planner.a_star import a_star
from planner._test_helpers import sample_grid, validate_path

def main() -> None:
    grid = sample_grid()
    start = (0, 0)
    goal = (6, 7)
    path = a_star(start, goal, grid)
    validate_path(path, start, goal, grid)
    print("A* OK | length =", len(path))

if __name__ == "__main__":
    main()

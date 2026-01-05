from planner.dijkstra import dijkstra
from planner._test_helpers import sample_grid, validate_path

def main() -> None:
    grid = sample_grid()
    start = (0, 0)
    goal = (6, 7)
    path = dijkstra(start, goal, grid)
    validate_path(path, start, goal, grid)
    print("Dijkstra OK | length =", len(path))

if __name__ == "__main__":
    main()

from planner.test_a_star import main as test_a_star
from planner.test_dijkstra import main as test_dijkstra
from planner.test_q_learning import main as test_q_learning

def main() -> None:
    test_a_star()
    test_dijkstra()
    test_q_learning()
    print("\nAll tests passed.")

if __name__ == "__main__":
    main()

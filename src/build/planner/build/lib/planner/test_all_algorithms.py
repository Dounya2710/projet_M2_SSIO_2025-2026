from planner.representation_env import load_matrix_from_sdf
from planner.dijkstra import dijkstra
from planner.a_star import a_star
from planner.q_learning import q_learning_plan

SDF_PATH = "../../../env/env.sdf"

matrix = load_matrix_from_sdf(SDF_PATH, clear_first=True)

start = (1, 0)
goal  = (6, 7)

for name, fn in [
    ("Dijkstra", dijkstra),
    ("A*", a_star),
    ("Q-learning", q_learning_plan),
]:
    path = fn(start, goal, matrix)
    print(f"{name}: {'OK' if path else 'NO PATH'} | len={len(path) if path else 0}")

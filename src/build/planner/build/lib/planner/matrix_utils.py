from planner.representation_env import matrix

cell_size = 1
room_size = 8
grid_size = room_size

def cell_to_world(i, j):
    x = (j + 0.5) * cell_size - room_size / 2
    y = (i + 0.5) * cell_size - room_size / 2
    return x, y

def world_to_cell(x, y):
    j = int((x + room_size / 2) / cell_size)
    i = int((y + room_size / 2) / cell_size)
    return i, j


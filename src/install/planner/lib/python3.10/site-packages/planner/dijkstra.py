import heapq
from planner.matrix_utils import matrix, grid_size

def get_neighbors(cell):
    i, j = cell
    neighbors = []
    for di, dj in [(-1,0),(1,0),(0,-1),(0,1)]:
        ni, nj = i + di, j + dj
        if 0 <= ni < grid_size and 0 <= nj < grid_size:
            if matrix[ni][nj] == 0:
                neighbors.append((ni, nj))
    return neighbors

def dijkstra(start, goal):
    pq = [(0, start)]
    came_from = {}
    cost = {start: 0}

    while pq:
        _, current = heapq.heappop(pq)
        if current == goal:
            break

        for n in get_neighbors(current):
            new_cost = cost[current] + 1
            if n not in cost or new_cost < cost[n]:
                cost[n] = new_cost
                came_from[n] = current
                heapq.heappush(pq, (new_cost, n))

    path = []
    cur = goal
    while cur != start:
        path.append(cur)
        cur = came_from.get(cur)
        if cur is None:
            return []
    path.append(start)
    path.reverse()
    return path


from queue import PriorityQueue
from collections import namedtuple

def heuristic(a, b):
    return abs(a[0] - b[0]) + abs(a[1] - b[1])

def is_valid(grid, pos):
    return 0 <= pos[0] < len(grid) and 0 <= pos[1] < len(grid[0]) and grid[pos[0]][pos[1]] != 1

def get_neighbors(pos):
    return [(pos[0] + dx, pos[1] + dy) for dx, dy in [(1, 0), (-1, 0), (0, 1), (0, -1)]]

def reconstruct_path(came_from, start, goal):
    path = []
    current = goal
    while current != start:
        path.append(current)
        current = came_from[current]
    path.append(start)
    path.reverse()
    return path

def a_star(start, goal, grid):
    frontier = PriorityQueue()
    frontier.put((0, start))
    came_from = {start: None}
    cost_so_far = {start: 0}

    while not frontier.empty():
        _, current = frontier.get()

        if current == goal:
            break

        for next in get_neighbors(current):
            if not is_valid(grid, next):
                continue
            
            new_cost = cost_so_far[current] + 1
            if next not in cost_so_far or new_cost < cost_so_far[next]:
                cost_so_far[next] = new_cost
                priority = new_cost + heuristic(goal, next)
                frontier.put((priority, next))
                came_from[next] = current

    return reconstruct_path(came_from, start, goal)

# Define the grid (0 is free, 1 is an obstacle)
grid = [
    [0,1,0,0,0],
    [0,1,0,1,0],
    [0,0,0,1,0],
    [0,1,0,0,0],
    [0,0,0,0,0]
]

start = (0, 0)
goal = (4, 4)
path = a_star(start, goal, grid)
print("Path:", path)

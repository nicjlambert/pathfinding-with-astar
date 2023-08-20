from queue import PriorityQueue

# Define the heuristic function
def heuristic(a, b):
    return abs(a[0] - b[0]) + abs(a[1] - b[1])

def a_star(start, goal, grid):
    # Initialize priority queue
    frontier = PriorityQueue()
    frontier.put(start, 0)
    came_from = {start: None}
    cost_so_far = {start: 0}

    while not frontier.empty():
        current = frontier.get()

        if current == goal:
            break

        # Iterate through neighbors
        for dx, dy in [(1,0),(-1,0),(0,1),(0,-1)]:
            next = (current[0] + dx, current[1] + dy)

            # Skip if outside of the grid or an obstacle
            if next[0] < 0 or next[0] >= len(grid) or next[1] < 0 or next[1] >= len(grid[0]) or grid[next[0]][next[1]] == 1:
                continue

            new_cost = cost_so_far[current] + 1
            if next not in cost_so_far or new_cost < cost_so_far[next]:
                cost_so_far[next] = new_cost
                priority = new_cost + heuristic(goal, next)
                frontier.put(next, priority)
                came_from[next] = current

    # Reconstruct the path
    path = []
    current = goal
    while current != start:
        path.append(current)
        current = came_from[current]
    path.append(start)
    path.reverse()
    return path

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

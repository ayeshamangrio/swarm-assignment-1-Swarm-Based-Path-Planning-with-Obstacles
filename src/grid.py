"""Grid, obstacle, start and goal generation (seeded by roll number)."""
import random
from collections import deque


def _reachable(blocked, start, goal, n):
    """BFS (4-connected) check so the generated instance is always solvable."""
    q, seen = deque([start]), {start}
    while q:
        x, y = q.popleft()
        if (x, y) == goal:
            return True
        for dx, dy in ((1, 0), (-1, 0), (0, 1), (0, -1)):
            nx, ny = x + dx, y + dy
            if 0 <= nx < n and 0 <= ny < n and (nx, ny) not in blocked and (nx, ny) not in seen:
                seen.add((nx, ny))
                q.append((nx, ny))
    return False


def generate_problem(seed, n, density, min_dist):
    """Return (blocked_cells:set, start:(x,y), goal:(x,y)) generated from `seed`.

    Nothing is hardcoded: obstacles, start and goal all come from random.seed(seed).
    """
    random.seed(seed)
    cells = [(x, y) for x in range(n) for y in range(n)]
    while True:
        blocked = set(random.sample(cells, int(density * n * n)))
        free = [c for c in cells if c not in blocked]
        start = random.choice(free)
        goal = random.choice(free)
        far_enough = abs(start[0] - goal[0]) + abs(start[1] - goal[1]) >= min_dist
        if far_enough and _reachable(blocked, start, goal, n):
            return blocked, start, goal

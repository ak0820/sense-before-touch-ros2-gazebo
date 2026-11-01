"""Pure-Python AI classifier and 2D A* planner shared by the ROS 2 demo."""

from __future__ import annotations

import heapq
import math
import random

CLASS_NAMES = ('MEDICAL', 'BATTERY', 'FOOD')
SIGNATURES = ((0.93, 0.12, 0.22), (0.13, 0.91, 0.86), (0.30, 0.21, 0.80))
PACKAGES = (
    {'name': 'A', 'pos': (3.14, 5.68), 'scan': (2.07, 5.44), 'class': 0},
    {'name': 'B', 'pos': (6.75, 5.35), 'scan': (5.74, 5.52), 'class': 1},
    {'name': 'C', 'pos': (3.10, 1.98), 'scan': (2.06, 1.80), 'class': 2},
)
# Include all physical Gazebo collision objects in the planning map.
# The packages have 0.27 m square footprints in rescue_warehouse.sdf.
RUBBLE = ((4.18, 2.45, 5.42, 4.45), (6.25, 2.10, 7.25, 2.80), (7.94, 3.10, 8.72, 3.72))
BOX_HALF_SIZE = 0.135
PACKAGE_OBSTACLES = tuple(
    (p['pos'][0] - BOX_HALF_SIZE, p['pos'][1] - BOX_HALF_SIZE,
     p['pos'][0] + BOX_HALF_SIZE, p['pos'][1] + BOX_HALF_SIZE)
    for p in PACKAGES
)
OBSTACLES = RUBBLE + PACKAGE_OBSTACLES
RESOLUTION = 0.2
RADIUS = 0.55


def _distance_to_rect(x, y, rect):
    x0, y0, x1, y1 = rect
    return math.hypot(x - min(max(x, x0), x1), y - min(max(y, y0), y1))


def blocked(x, y):
    if not (RADIUS <= x <= 12 - RADIUS and RADIUS <= y <= 7 - RADIUS):
        return True
    return any(_distance_to_rect(x, y, o) < RADIUS for o in OBSTACLES)


def clear(a, b):
    count = max(2, math.ceil(math.dist(a, b) / .08))
    return all(not blocked(a[0] + (b[0] - a[0]) * k / count,
                           a[1] + (b[1] - a[1]) * k / count) for k in range(count + 1))


def plan(start, goal):
    """Use 8-neighbor A* and line-of-sight smoothing."""
    if blocked(*start) or blocked(*goal):
        raise ValueError(f'Blocked start or goal: {start} -> {goal}')
    if clear(start, goal):
        return [goal]
    source = tuple(round(z / RESOLUTION) for z in start)
    target = tuple(round(z / RESOLUTION) for z in goal)
    front = [(0.0, source)]
    costs = {source: 0.0}
    parent = {}
    visited = set()
    while front:
        _, u = heapq.heappop(front)
        if u in visited:
            continue
        visited.add(u)
        if u == target:
            break
        for dx in (-1, 0, 1):
            for dy in (-1, 0, 1):
                if not (dx or dy):
                    continue
                v = (u[0] + dx, u[1] + dy)
                pa = (u[0] * RESOLUTION, u[1] * RESOLUTION)
                pb = (v[0] * RESOLUTION, v[1] * RESOLUTION)
                if not clear(pa, pb):
                    continue
                cost = costs[u] + math.hypot(dx, dy)
                if cost < costs.get(v, math.inf):
                    costs[v] = cost
                    parent[v] = u
                    heapq.heappush(front, (cost + math.dist(v, target), v))
    if target not in visited:
        raise RuntimeError(f'No route for {start} -> {goal}')
    cells = [target]
    while cells[-1] != source:
        cells.append(parent[cells[-1]])
    cells.reverse()
    raw = [start] + [(x * RESOLUTION, y * RESOLUTION) for x, y in cells[1:-1]] + [goal]
    waypoints = []
    i = 0
    while i < len(raw) - 1:
        j = len(raw) - 1
        while j > i + 1 and not clear(raw[i], raw[j]):
            j -= 1
        waypoints.append(raw[j])
        i = j
    return waypoints


class GaussianMaterialAI:
    """Trained diagonal Gaussian classifier over simulated multispectral readings.

    The sensor values are synthetic and are NOT readings from a Gazebo camera.
    """

    def __init__(self, seed=7):
        rng = random.Random(seed)
        self.means = []
        self.variances = []
        for signature in SIGNATURES:
            observations = [tuple(min(1, max(0, c + rng.gauss(0, .115))) for c in signature)
                            for _ in range(360)]
            mu = [sum(o[j] for o in observations) / len(observations) for j in range(3)]
            var = [sum((o[j] - mu[j]) ** 2 for o in observations) / len(observations) + 1e-4
                   for j in range(3)]
            self.means.append(mu)
            self.variances.append(var)

    def sense(self, label, distance, rng):
        sigma = .055 + .48 * min(1, max(0, (distance - .35) / .85))
        return tuple(min(1, max(0, c + rng.gauss(0, sigma))) for c in SIGNATURES[label])

    def predict(self, reading, distance):
        log_scores = []
        for mu, var in zip(self.means, self.variances):
            log_scores.append(-.5 * sum(math.log(v) + (x - m) ** 2 / v
                                          for x, m, v in zip(reading, mu, var)))
        peak = max(log_scores)
        vals = [math.exp(v - peak) for v in log_scores]
        probs = [v / sum(vals) for v in vals]
        quality = .30 + .70 * (1 - min(1, max(0, (distance - .32) / .88)))
        probabilities = tuple(quality * p + (1 - quality) / 3 for p in probs)
        label = max(range(3), key=lambda i: probabilities[i])
        return label, probabilities[label], probabilities

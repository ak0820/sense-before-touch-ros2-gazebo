"""Dependency-free smoke tests for path planning and AI confidence."""

import math
import random
import sys
from pathlib import Path

pkg = Path(__file__).resolve().parents[1] / 'sense_before_touch_ros2'
sys.path.insert(0, str(pkg))
from sense_before_touch_ros2.core import GaussianMaterialAI, PACKAGES, OBSTACLES, blocked, clear, plan


def test_routes():
    p = (1.05, 1.08)
    for goal in [x['scan'] for x in PACKAGES] + [(1.05, 1.08)]:
        waypoints = plan(p, goal)
        assert waypoints
        for q in waypoints:
            assert clear(p, q), (p, q)
            p = q



def test_package_collision_avoidance():
    # BOX A is a solid obstacle in Gazebo; a straight line toward BOX B hits it.
    start = (2.35, 5.57)
    goal = PACKAGES[1]['scan']
    assert not clear(start, goal), 'A straight line must be blocked by BOX A'
    waypoints = plan(start, goal)
    assert len(waypoints) >= 2, 'The path must detour around BOX A'
    point = start
    for waypoint in waypoints:
        assert clear(point, waypoint), (point, waypoint)
        point = waypoint


def test_inspection_standoff():
    # All inspection goals must remain outside every collision-inflated object.
    for package in PACKAGES:
        x, y = package['pos']
        target = (x - .80, y - .08)
        assert not blocked(*target), (package['name'], target)
        prior = package['scan']
        for point in plan(prior, target):
            assert clear(prior, point)
            prior = point


def test_complete_mission_geometry():
    # Run the planned sequence including inspection approaches and the return.
    position = (1.05, 1.08)
    goals = []
    for package in PACKAGES:
        x, y = package['pos']
        goals.extend([package['scan'], (x - .80, y - .08)])
    goals.append((1.05, 1.08))
    for goal in goals:
        path = plan(position, goal)
        assert path
        for waypoint in path:
            assert clear(position, waypoint), (position, waypoint)
            position = waypoint


def test_active_confidence():
    model = GaussianMaterialAI()
    rng = random.Random(25)
    for label in range(3):
        values = model.sense(label, .35, rng)
        prediction, conf, _ = model.predict(values, .35)
        assert prediction == label
        assert conf > .77
        far = model.sense(label, 1.15, rng)
        _, far_conf, _ = model.predict(far, 1.15)
        assert far_conf < .77


if __name__ == '__main__':
    test_routes()
    test_package_collision_avoidance()
    test_inspection_standoff()
    test_complete_mission_geometry()
    test_active_confidence()
    print('OK: routes and confidence behavior')

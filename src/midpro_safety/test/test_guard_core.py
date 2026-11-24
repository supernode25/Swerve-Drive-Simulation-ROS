import math
import numpy as np
from midpro_safety.guard_core import Guard, sweep_distance, rotation_clearance

FP = [[0.42, 0.25], [0.42, -0.25], [-0.48, -0.25], [-0.48, 0.25]]


def wall(x0, x1, y, n=60):
    return np.linspace(x0, x1, n), np.full(n, y)


def test_side_walls_do_not_block_forward():
    # the case from the field: 5 cm each side, walls along the whole body and beyond
    lx, ly = wall(-1.5, 2.0, 0.30)
    rx, ry = wall(-1.5, 2.0, -0.30)
    g = Guard(FP)
    for _ in range(30):
        a = g.update(np.r_[lx, rx], np.r_[ly, ry], dt=0.1)
    assert a['x+'] > 0.9 and a['x-'] > 0.9            # free to drive straight
    assert a['y+'] < 0.2 and a['y-'] < 0.2             # only a creep: 2 cm of room before the 3 cm side margin
    assert 0.0 < a['ccw'] < 0.5 and 0.0 < a['cw'] < 0.5  # a few degrees of turn left before a corner touches
    vx, vy, wz, why = g.filter(0.4, 0.0, 0.0)
    assert vx == 0.4 and why == []


def test_obstacle_ahead_slows_and_stops():
    g = Guard(FP)
    px, py = wall(1.30, 1.30, 0.0, 5)          # 0.88 m ahead of the front edge
    for _ in range(30):
        a = g.update(px, py, dt=0.1)
    assert 0.0 < a['x+'] < 1.0 and a['x-'] > 0.9
    px, py = wall(0.50, 0.50, 0.0, 5)          # 8 cm ahead (< stop margin 10 cm)
    a = g.update(px, py, dt=0.1)
    assert a['x+'] == 0.0 and a['x-'] > 0.0


def test_corner_rotation_direction():
    # point just ahead of the left half of the front edge: turning right (CW) moves that part forward into it,
    # turning left moves it back (free)
    g = Guard(FP)
    a = g.update(np.array([0.46]), np.array([0.15]), dt=0.1)
    assert a['cw'] < a['ccw'] and a['ccw'] == 1.0


def test_point_outside_swing_circle_never_limits_rotation():
    # ahead of the front-left corner but beyond the corner radius: pure rotation can never reach it
    g = Guard(FP)
    a = g.update(np.array([0.47]), np.array([0.20]), dt=0.1)
    assert a['ccw'] == 1.0 and a['cw'] == 1.0 and a['x+'] == 0.0


def test_oblique_corner_case():
    g = Guard(FP)
    px, py = np.array([0.60]), np.array([0.40])           # off the x and y sweeps, on the diagonal
    for _ in range(30):
        g.update(px, py, dt=0.1)
    vx, vy, wz, why = g.filter(0.4, 0.4, 0.0)
    assert 'oblique' in why and math.hypot(vx, vy) < 0.4 * math.sqrt(2)


def test_sweep_basics():
    s = sweep_distance(FP, np.array([1.0, 0.0, 1.0]), np.array([0.0, 0.5, 0.26]), 1.0, 0.0)
    assert abs(s[0] - 0.58) < 1e-6 and math.isinf(s[1]) and math.isinf(s[2])
    th = rotation_clearance(FP, np.array([0.0]), np.array([0.7]), 1)
    assert th == math.radians(60) or th > 0


def test_contact_side_allows_turning_away():
    # wall 2 cm from the left side, from the front corner forwards: turning right swings the front-left corner away
    px, py = np.linspace(0.40, 1.4, 40), np.full(40, 0.27)
    g = Guard(FP)
    for _ in range(30):
        a = g.update(px, py, dt=0.1)
    assert a['ccw'] == 0.0 and a['cw'] > 0.2 and a['x+'] > 0.9

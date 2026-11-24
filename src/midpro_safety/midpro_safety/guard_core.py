"""guard_core.py — direction-aware obstacle guard geometry (no ROS; unit-testable).

Port and extension of the midpro ROS1 obstacle_detector / obstacle_filter / obstacle_stop design:
a laser point only restricts the motions that would bring the robot body towards it.

ROS1 design                                  this version
-------------------------------------------  ---------------------------------------------------------------
8 angular sectors from the laser origin      geometry in base_link (laser offset handled)
point in sector -> direction on/off          per direction: distance the footprint can travel before touching
time-based ramp ((dt-T)^4) to min speed      distance-based limit v = sqrt(2 a (s - margin)) (stops in time)
x / y / z components only                    + combined direction check for oblique (vx, vy) commands
rotation blocked by sector                   rotation clearance angle by sweeping the footprint
counter_to_false hysteresis                  limits drop at once, rise at a bounded rate

Directions: x+ forward, x- backward, y+ left, y- right, ccw (z+), cw (z-).
"""
import math

import numpy as np

DIRS = ('x+', 'x-', 'y+', 'y-', 'ccw', 'cw')


def expand(fp, front, rear, side):
    """Grow a (rectangle-like) footprint outward per side."""
    return [[x + (front if x >= 0 else -rear), y + (side if y >= 0 else -side)] for x, y in fp]


def shrink(fp, d):
    return [[x - math.copysign(d, x), y - math.copysign(d, y)] for x, y in fp]


def inside(poly, px, py):
    """Vectorised even-odd point-in-polygon test (px, py arrays of any equal shape)."""
    px = np.asarray(px, dtype=float)
    py = np.asarray(py, dtype=float)
    res = np.zeros(px.shape, dtype=bool)
    n = len(poly)
    for i in range(n):
        x1, y1 = poly[i]
        x2, y2 = poly[(i + 1) % n]
        if y1 == y2:
            continue
        cond = (y1 > py) != (y2 > py)
        xint = (x2 - x1) * (py - y1) / (y2 - y1) + x1
        res ^= cond & (px < xint)
    return res


def sweep_distance(poly, px, py, ux, uy):
    """Distance s >= 0 the polygon can translate along unit u before point p is on its boundary.
    Computed as the ray p - s*u against every edge. inf when the point is never reached; 0 when inside."""
    px = np.asarray(px, dtype=float)
    py = np.asarray(py, dtype=float)
    best = np.full(px.shape, np.inf)
    n = len(poly)
    for i in range(n):
        ax, ay = poly[i]
        bx, by = poly[(i + 1) % n]
        ex, ey = bx - ax, by - ay
        # p - s u = a + t e   ->   s * (-u) + t * (-e) = a - p
        det = (-ux) * (-ey) - (-uy) * (-ex)
        if abs(det) < 1e-12:
            continue
        rx, ry = ax - px, ay - py
        s = (rx * (-ey) - ry * (-ex)) / det
        t = ((-ux) * ry - (-uy) * rx) / det
        ok = (s >= -1e-9) & (t >= -1e-9) & (t <= 1.0 + 1e-9)
        best = np.where(ok & (s < best), np.maximum(s, 0.0), best)
    best[inside(poly, px, py)] = 0.0
    return best


def rotation_hits(poly, px, py, sign, max_angle=math.radians(60), step=math.radians(1.0)):
    """Per point: smallest rotation (CCW for sign > 0) of the robot at which the point enters the polygon
    (inf when not within max_angle). Rotating the robot by +th equals rotating the points by -th."""
    px = np.asarray(px, dtype=float)
    py = np.asarray(py, dtype=float)
    if px.size == 0:
        return np.zeros(0)
    ths = np.arange(step, max_angle + 1e-9, step)
    c = np.cos(-sign * ths)[:, None]
    s = np.sin(-sign * ths)[:, None]
    rx = c * px[None, :] - s * py[None, :]
    ry = s * px[None, :] + c * py[None, :]
    hit = inside(poly, rx, ry)                                 # (angles, points)
    first = np.where(hit.any(axis=0), ths[np.argmax(hit, axis=0)], np.inf)
    first[inside(poly, px, py)] = 0.0                          # already touching
    return first


def rotation_clearance(poly, px, py, sign, max_angle=math.radians(60), step=math.radians(1.0)):
    """Smallest rotation angle at which any point enters the polygon (max_angle when none)."""
    h = rotation_hits(poly, px, py, sign, max_angle, step)
    return float(np.min(h)) if h.size and np.isfinite(h).any() else max_angle


class Guard:
    def __init__(self, footprint, stop_front=0.10, stop_rear=0.10, stop_side=0.03, rot_margin=0.03,
                 brake_acc=0.5, brake_alpha=1.0, latency=0.15, slow_dist=0.8, slow_speed=0.3,
                 rise_rate=0.6, v_max=1.0, w_max=1.0):
        self.fp = [[float(a), float(b)] for a, b in footprint]
        self.margin = {'x+': stop_front, 'x-': stop_rear, 'y+': stop_side, 'y-': stop_side}
        self.rot_margin = rot_margin
        self.brake_acc, self.brake_alpha, self.latency = brake_acc, brake_alpha, latency
        self.slow_dist, self.slow_speed, self.rise_rate = slow_dist, slow_speed, rise_rate
        self.v_max, self.w_max = v_max, w_max
        self.circ = max(math.hypot(x, y) for x, y in self.fp)
        self.allow = {d: (w_max if d in ('ccw', 'cw') else v_max) for d in DIRS}
        self.dist = {d: math.inf for d in DIRS}
        self.px = np.zeros(0)
        self.py = np.zeros(0)

    # ------------------------------------------------------------------ distances / limits
    def _lin_limit(self, s, margin, v_now):
        """Speed that still stops `margin` short of a point s metres away along the motion."""
        if not math.isfinite(s):
            return self.v_max
        room = s - margin - self.latency * max(v_now, 0.0)
        v = math.sqrt(2.0 * self.brake_acc * room) if room > 0.0 else 0.0
        if s - margin < self.slow_dist:          # gentle approach cap inside the slow zone (ROS1 decel zone)
            frac = max(0.0, (s - margin) / self.slow_dist)
            v = min(v, self.slow_speed + (self.v_max - self.slow_speed) * frac)
        return min(v, self.v_max)

    def directional_distance(self, ux, uy):
        """Distance the bare footprint can travel along u before touching a point (inf if none on the way).
        Points already inside the footprint only block directions that point at them."""
        if self.px.size == 0:
            return math.inf
        s = sweep_distance(self.fp, self.px, self.py, ux, uy)
        ins = s == 0.0
        if ins.any():
            ahead = (self.px * ux + self.py * uy) > 0.0
            s = np.where(ins & ~ahead, np.inf, s)
        return float(np.min(s)) if s.size else math.inf

    def update(self, px, py, v_now=(0.0, 0.0, 0.0), dt=0.1):
        """New scan points in base_link -> per-direction distances and allowed speeds."""
        self.px = np.asarray(px, dtype=float)
        self.py = np.asarray(py, dtype=float)
        vx, vy, wz = v_now
        new = {}
        for d, (ux, uy, vn) in {'x+': (1.0, 0.0, vx), 'x-': (-1.0, 0.0, -vx),
                                'y+': (0.0, 1.0, vy), 'y-': (0.0, -1.0, -vy)}.items():
            s = self.directional_distance(ux, uy)
            self.dist[d] = s
            new[d] = self._lin_limit(s, self.margin[d], vn)
        if self.px.size:
            near = np.hypot(self.px, self.py) <= self.circ + 0.25
            npx, npy = self.px[near], self.py[near]
        else:
            npx = npy = np.zeros(0)
        # rotation: angle the BARE footprint can turn before touching a point, minus the angle that the
        # margin (rot_margin) represents at that point's radius. A turn that swings the body away from a point
        # never reaches it, so it is never limited by it (a margin polygon would block both directions).
        rr = np.maximum(np.hypot(npx, npy), 0.1)
        for d, sign, wn in (('ccw', 1, wz), ('cw', -1, -wz)):
            hits = rotation_hits(self.fp, npx, npy, sign)
            per = hits - self.rot_margin / rr
            th = float(np.min(per)) if per.size and np.isfinite(per).any() else math.radians(60)
            self.dist[d] = th
            room = th - self.latency * max(wn, 0.0)
            new[d] = min(self.w_max, math.sqrt(2.0 * self.brake_alpha * room)) if room > 0.0 else 0.0
        for d in DIRS:                            # drop at once, recover at a bounded rate (no flicker)
            cap = self.allow[d] + self.rise_rate * dt * (2.0 if d in ('ccw', 'cw') else 1.0)
            self.allow[d] = min(new[d], cap)
        return dict(self.allow)

    # ------------------------------------------------------------------ command filter
    def filter(self, vx, vy, wz):
        """Clamp only the components that move towards an obstacle (ROS1 obstacle_filter behaviour),
        then check the combined direction of an oblique command."""
        a = self.allow
        reasons = []
        ox, oy, ow = vx, vy, wz
        if vx > 0 and vx > a['x+']:
            ox = a['x+']; reasons.append('x+')
        elif vx < 0 and -vx > a['x-']:
            ox = -a['x-']; reasons.append('x-')
        if vy > 0 and vy > a['y+']:
            oy = a['y+']; reasons.append('y+')
        elif vy < 0 and -vy > a['y-']:
            oy = -a['y-']; reasons.append('y-')
        if wz > 0 and wz > a['ccw']:
            ow = a['ccw']; reasons.append('ccw')
        elif wz < 0 and -wz > a['cw']:
            ow = -a['cw']; reasons.append('cw')
        sp = math.hypot(ox, oy)
        if sp > 1e-3 and abs(ox) > 1e-3 and abs(oy) > 1e-3:      # oblique: the corner may meet what x/y miss
            ux, uy = ox / sp, oy / sp
            s = self.directional_distance(ux, uy)
            m = abs(ux) * (self.margin['x+'] if ux > 0 else self.margin['x-']) + abs(uy) * self.margin['y+']
            lim = self._lin_limit(s, m, sp)
            if sp > lim:
                k = lim / sp
                ox *= k; oy *= k; reasons.append('oblique')
        return ox, oy, ow, reasons

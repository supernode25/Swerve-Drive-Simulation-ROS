#!/usr/bin/env python3
"""goal_footprint.py — project the real robot footprint onto the goal (and the last 1.5 m of the plan) in RViz
and say whether the robot can get in.

    ros2 run midpro_dash goal_footprint          (started by sim_nav.launch.py, goal_viz:=true)

Inputs : /plan (planner output, map frame — its end is the goal actually used, after the planner tolerance),
         /goal_pose (2D Goal Pose tool preview), /global_costmap/costmap (+ _updates), /map (fallback)
Output : /midpro/goal_footprint  visualization_msgs/MarkerArray
         /midpro/goal_entry      std_msgs/String  JSON {"status": ok|tight|blocked, "clearance": m, "approach": ...}
Status : blocked = the footprint + footprint_padding touches a lethal cell
         tight   = fits with the padding but not with straight_clearance around it
         ok      = at least straight_clearance all around
Footprint / padding / clearance are read from the running nodes (local_costmap footprint,
controller FollowPath.footprint_padding / straight_clearance); defaults = midpro 0.90 x 0.50 m.
"""
import ast
import json
import math

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy

from geometry_msgs.msg import Point, PoseStamped
from nav_msgs.msg import OccupancyGrid, Path
from std_msgs.msg import ColorRGBA, String
from visualization_msgs.msg import Marker, MarkerArray
from rcl_interfaces.srv import GetParameters

try:
    from map_msgs.msg import OccupancyGridUpdate
except ImportError:  # pragma: no cover
    OccupancyGridUpdate = None

COL = {'ok': (0.22, 0.54, 0.87), 'tight': (0.94, 0.62, 0.15), 'blocked': (0.89, 0.29, 0.24)}
DEFAULT_FP = [[0.42, 0.25], [0.42, -0.25], [-0.48, -0.25], [-0.48, 0.25]]


def yaw_of(q):
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))


def padded(fp, pad):
    return [[x + (pad if x >= 0 else -pad), y + (pad if y >= 0 else -pad)] for x, y in fp]


def to_world(fp, x, y, yaw):
    c, s = math.cos(yaw), math.sin(yaw)
    return [(x + c * px - s * py, y + s * px + c * py) for px, py in fp]


def inside(poly, xs, ys):
    """Vectorised even-odd point-in-polygon test."""
    res = np.zeros(xs.shape, dtype=bool)
    n = len(poly)
    for i in range(n):
        x1, y1 = poly[i]; x2, y2 = poly[(i + 1) % n]
        cond = (y1 > ys) != (y2 > ys)
        with np.errstate(divide='ignore', invalid='ignore'):
            xint = (x2 - x1) * (ys - y1) / (y2 - y1 + 1e-12) + x1
        res ^= cond & (xs < xint)
    return res


class Grid:
    def __init__(self, msg, lethal):
        self.info = msg.info
        self.data = np.array(msg.data, dtype=np.int16).reshape(msg.info.height, msg.info.width)
        self.lethal = lethal

    def update(self, up):
        d = np.array(up.data, dtype=np.int16).reshape(up.height, up.width)
        self.data[up.y:up.y + up.height, up.x:up.x + up.width] = d

    def collides(self, poly):
        res = self.info.resolution
        ox, oy = self.info.origin.position.x, self.info.origin.position.y
        pxs = [p[0] for p in poly]; pys = [p[1] for p in poly]
        c0 = int(math.floor((min(pxs) - ox) / res)); c1 = int(math.ceil((max(pxs) - ox) / res))
        r0 = int(math.floor((min(pys) - oy) / res)); r1 = int(math.ceil((max(pys) - oy) / res))
        h, w = self.data.shape
        if c0 < 0 or r0 < 0 or c1 >= w or r1 >= h:
            return True                                    # outside the map
        cc, rr = np.meshgrid(np.arange(c0, c1 + 1), np.arange(r0, r1 + 1))
        xs = ox + (cc + 0.5) * res; ys = oy + (rr + 0.5) * res
        m = inside(poly, xs, ys)
        if not m.any():
            return False
        v = self.data[r0:r1 + 1, c0:c1 + 1][m]
        return bool(((v >= self.lethal) | (v < 0)).any())


class GoalFootprint(Node):
    PADS = [0.0, 0.03, 0.06, 0.10, 0.15, 0.20, 0.30]

    def __init__(self):
        super().__init__('goal_footprint')
        self.fp = [list(p) for p in DEFAULT_FP]
        self.pad = 0.03
        self.comfort = 0.15
        self.costmap = None
        self.map = None
        self.last_plan = None
        self.last_goal = None
        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL, reliability=ReliabilityPolicy.RELIABLE)
        self.create_subscription(OccupancyGrid, '/global_costmap/costmap', self.on_costmap, latched)
        if OccupancyGridUpdate is not None:
            self.create_subscription(OccupancyGridUpdate, '/global_costmap/costmap_updates', self.on_update, 10)
        self.create_subscription(OccupancyGrid, '/map', self.on_map, latched)
        self.create_subscription(Path, '/plan', self.on_plan, 5)
        self.create_subscription(PoseStamped, '/goal_pose', self.on_goal_pose, 5)
        self.pub = self.create_publisher(MarkerArray, '/midpro/goal_footprint', latched)
        self.entry_pub = self.create_publisher(String, '/midpro/goal_entry', latched)
        self.cli_cm = self.create_client(GetParameters, '/local_costmap/local_costmap/get_parameters')
        self.cli_ctl = self.create_client(GetParameters, '/controller_server/get_parameters')
        self.create_timer(2.0, self.fetch_params)
        self.create_timer(2.0, self.refresh)        # re-evaluate (costmap changes); publishes only if the result changed
        self._fetched = False
        self._last_sig = None

    # ------------------------------------------------------------------ inputs
    def fetch_params(self):
        if self._fetched:
            return
        if self.cli_cm.service_is_ready():
            req = GetParameters.Request(); req.names = ['footprint']
            self.cli_cm.call_async(req).add_done_callback(self._on_fp)
        if self.cli_ctl.service_is_ready():
            req = GetParameters.Request(); req.names = ['FollowPath.footprint_padding', 'FollowPath.straight_clearance']
            self.cli_ctl.call_async(req).add_done_callback(self._on_ctl)

    def _on_fp(self, fut):
        try:
            s = fut.result().values[0].string_value
            fp = ast.literal_eval(s) if s else None
            if fp and len(fp) >= 3:
                self.fp = [[float(a), float(b)] for a, b in fp]; self._fetched = True
                self.get_logger().info(f'footprint from local_costmap: {self.fp}')
        except Exception:  # noqa: BLE001
            pass

    def _on_ctl(self, fut):
        try:
            v = fut.result().values
            if v[0].type == 3:
                self.pad = v[0].double_value
            if v[1].type == 3:
                self.comfort = v[1].double_value
        except Exception:  # noqa: BLE001
            pass

    def on_costmap(self, m):
        self.costmap = Grid(m, 100)
        self.refresh()

    def on_update(self, up):
        if self.costmap is not None:
            try:
                self.costmap.update(up)
            except Exception:  # noqa: BLE001  (size changed: wait for the next full map)
                self.costmap = None

    def on_map(self, m):
        self.map = Grid(m, 65)

    def on_plan(self, m):
        if m.poses:
            self.last_plan = m
            self.refresh()

    def on_goal_pose(self, m):
        self.last_goal = m
        self.last_plan = None        # preview until the plan for this goal arrives
        self.refresh()

    # ------------------------------------------------------------------ evaluation
    def grid(self):
        return self.costmap if self.costmap is not None else self.map

    def clearance(self, x, y, yaw):
        g = self.grid()
        if g is None:
            return None
        best = -1.0
        for p in self.PADS:
            if g.collides(to_world(padded(self.fp, p), x, y, yaw)):
                break
            best = p
        return best

    def status(self, clr):
        if clr is None:
            return 'unknown'
        if clr < self.pad - 1e-6:
            return 'blocked'
        if clr < self.comfort - 1e-6:
            return 'tight'
        return 'ok'

    def refresh(self):
        frame, gx, gy, gyaw, approach = None, None, None, None, []
        if self.last_plan is not None:
            ps = self.last_plan.poses
            frame = self.last_plan.header.frame_id or 'map'
            gp = ps[-1].pose; gx, gy, gyaw = gp.position.x, gp.position.y, yaw_of(gp.orientation)
            acc, nxt = 0.0, 0.5
            for i in range(len(ps) - 1, 0, -1):
                a, b = ps[i].pose.position, ps[i - 1].pose.position
                acc += math.hypot(a.x - b.x, a.y - b.y)
                if acc >= nxt and acc <= 1.51:
                    tng = math.atan2(a.y - b.y, a.x - b.x)
                    approach.append((b.x, b.y, tng)); nxt += 0.5
                if acc > 1.51:
                    break
        elif self.last_goal is not None:
            frame = self.last_goal.header.frame_id or 'map'
            gp = self.last_goal.pose; gx, gy, gyaw = gp.position.x, gp.position.y, yaw_of(gp.orientation)
        if gx is None:
            return
        clr = self.clearance(gx, gy, gyaw)
        st = self.status(clr)
        app = [(x, y, yw, self.status(self.clearance(x, y, yw))) for (x, y, yw) in approach]
        worst_app = 'ok'
        for (_x, _y, _yw, s) in app:
            if s == 'blocked' or (s == 'tight' and worst_app == 'ok'):
                worst_app = s
        sig = (round(gx, 2), round(gy, 2), round(gyaw, 2), st, None if clr is None else round(clr, 2),
               tuple((round(x, 1), round(y, 1), s) for (x, y, _yw, s) in app))
        if sig == self._last_sig:
            return                       # unchanged: do not make RViz delete and rebuild the markers
        self._last_sig = sig
        self.publish(frame, gx, gy, gyaw, st, clr, app, worst_app)

    # ------------------------------------------------------------------ markers
    def publish(self, frame, gx, gy, gyaw, st, clr, app, worst_app):
        ma = MarkerArray()
        clear = Marker(); clear.action = Marker.DELETEALL; ma.markers.append(clear)
        stamp = self.get_clock().now().to_msg()
        rgb = COL.get(st, (0.6, 0.6, 0.6))

        def base(mid, mtype, a):
            m = Marker(); m.header.frame_id = frame; m.header.stamp = stamp; m.ns = 'goal_footprint'; m.id = mid
            m.type = mtype; m.action = Marker.ADD; m.pose.orientation.w = 1.0
            m.color = ColorRGBA(r=rgb[0], g=rgb[1], b=rgb[2], a=a)
            return m
        poly = to_world(self.fp, gx, gy, gyaw)
        fill = base(1, Marker.TRIANGLE_LIST, 0.35); fill.scale.x = fill.scale.y = fill.scale.z = 1.0
        for i in range(1, len(poly) - 1):
            for (x, y) in (poly[0], poly[i], poly[i + 1]):
                fill.points.append(Point(x=x, y=y, z=0.02))
        ma.markers.append(fill)
        out = base(2, Marker.LINE_STRIP, 1.0); out.scale.x = 0.03
        for (x, y) in poly + [poly[0]]:
            out.points.append(Point(x=x, y=y, z=0.03))
        ma.markers.append(out)
        pad_poly = to_world(padded(self.fp, self.pad), gx, gy, gyaw)
        pd = base(3, Marker.LINE_STRIP, 0.6); pd.scale.x = 0.012
        for (x, y) in pad_poly + [pad_poly[0]]:
            pd.points.append(Point(x=x, y=y, z=0.03))
        ma.markers.append(pd)
        ar = base(4, Marker.ARROW, 1.0); ar.scale.x, ar.scale.y, ar.scale.z = 0.04, 0.09, 0.09
        ar.points = [Point(x=gx, y=gy, z=0.05), Point(x=gx + 0.45 * math.cos(gyaw), y=gy + 0.45 * math.sin(gyaw), z=0.05)]
        ma.markers.append(ar)
        txt = base(5, Marker.TEXT_VIEW_FACING, 1.0); txt.scale.z = 0.22
        txt.pose.position.x, txt.pose.position.y, txt.pose.position.z = gx, gy, 0.8
        if st == 'blocked':
            label = 'GOAL BLOCKED'
        elif st == 'unknown':
            label = 'GOAL ? (no costmap)'
        else:
            label = f"GOAL {'OK' if st == 'ok' else 'TIGHT'}  clear {clr:.2f} m"
        if app and worst_app != 'ok':
            label += f'  | approach {worst_app.upper()}'
        txt.text = label
        ma.markers.append(txt)
        for k, (x, y, yw, s) in enumerate(app):
            c = COL.get(s, (0.6, 0.6, 0.6))
            gpoly = to_world(self.fp, x, y, yw)
            gm = base(10 + k, Marker.LINE_STRIP, 0.55); gm.scale.x = 0.015
            gm.color = ColorRGBA(r=c[0], g=c[1], b=c[2], a=0.55)
            for (px, py) in gpoly + [gpoly[0]]:
                gm.points.append(Point(x=px, y=py, z=0.025))
            ma.markers.append(gm)
        self.pub.publish(ma)
        self.entry_pub.publish(String(data=json.dumps({'status': st, 'clearance': clr, 'approach': worst_app,
                                                       'goal': [round(gx, 2), round(gy, 2), round(math.degrees(gyaw), 1)]})))


def main():
    rclpy.init()
    node = GoalFootprint()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    rclpy.shutdown()


if __name__ == '__main__':
    main()

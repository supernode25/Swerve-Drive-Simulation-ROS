#!/usr/bin/env python3
"""obstacle_guard.py — direction-aware reactive safety layer (replaces nav2 collision_monitor in the chain).

    controller -> /cmd_vel_nav -> [obstacle_guard] -> /cmd_vel_cm -> velocity_smoother -> /cmd_vel

A laser point only limits the motions that bring the robot towards it: side walls never stop forward driving,
an obstacle ahead slows the robot so it stops `stop_front` short of it, rotation is limited by the angle the
footprint can swing before a corner touches something. Port of the midpro ROS1 obstacle_detector /
obstacle_filter / laser_filter / obstacle_stop nodes (see guard_core.py for what changed).

Topics
  in : scan (LaserScan), cmd_in (Twist), odom (Odometry, optional, for the latency term)
  out: cmd_out (Twist), ~/state (String JSON, per-direction allowed speed and distance),
       ~/markers (MarkerArray: footprint, stop zone, blocked-direction arrows), ~/alarm (String JSON)
"""
import ast
import json
import math
import time

import numpy as np
import rclpy
from geometry_msgs.msg import Point, Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan
from std_msgs.msg import ColorRGBA, String
from visualization_msgs.msg import Marker, MarkerArray

from midpro_safety.guard_core import DIRS, Guard, expand, inside, shrink

try:
    import tf2_ros
except ImportError:  # pragma: no cover
    tf2_ros = None

NAMES = {'x+': '전진', 'x-': '후진', 'y+': '좌측', 'y-': '우측', 'ccw': '좌회전', 'cw': '우회전'}


class ObstacleGuard(Node):
    def __init__(self):
        super().__init__('obstacle_guard')
        P = self.declare_parameter
        P('footprint', '[[0.42, 0.25], [0.42, -0.25], [-0.48, -0.25], [-0.48, 0.25]]')
        P('base_frame', 'base_link')
        P('laser_pose', [0.25, 0.0, 0.0])           # fallback when TF base_frame <- scan frame is not available
        P('stop_front', 0.10); P('stop_rear', 0.10); P('stop_side', 0.03); P('rot_margin', 0.03)
        P('brake_acc', 0.5); P('brake_alpha', 1.0); P('latency', 0.15)
        P('slow_dist', 0.8); P('slow_speed', 0.3); P('rise_rate', 0.6)
        P('v_max', 1.0); P('w_max', 1.0)
        P('self_filter', True)                     # ROS1 laser_filter: drop returns from the robot itself
        P('self_filter_shrink', 0.03)              # ...but keep the outer 3 cm so a real contact is still seen
        P('max_range', 6.0)                        # points further away cannot matter for one control step
        P('scan_timeout', 0.5)                     # s without scan -> stop (fail safe)
        P('alarm_sec', 60.0)                       # ROS1 obstacle_stop: blocked this long -> alarm
        P('marker_rate', 5.0)
        g = lambda n: self.get_parameter(n).value  # noqa: E731
        fp = ast.literal_eval(g('footprint')) if isinstance(g('footprint'), str) else g('footprint')
        self.guard = Guard(fp, g('stop_front'), g('stop_rear'), g('stop_side'), g('rot_margin'), g('brake_acc'),
                           g('brake_alpha'), g('latency'), g('slow_dist'), g('slow_speed'), g('rise_rate'),
                           g('v_max'), g('w_max'))
        self.fp = self.guard.fp
        self.stop_zone = expand(self.fp, g('stop_front'), g('stop_rear'), g('stop_side'))
        self.self_poly = shrink(self.fp, g('self_filter_shrink')) if g('self_filter') else None
        self.base_frame = g('base_frame')
        self.laser_pose = None
        self.fallback_pose = tuple(g('laser_pose'))
        self.max_range = g('max_range'); self.scan_timeout = g('scan_timeout'); self.alarm_sec = g('alarm_sec')
        self.marker_period = 1.0 / max(0.5, g('marker_rate'))
        self.v_now = (0.0, 0.0, 0.0)
        self.last_scan_t = None; self.last_marker_t = 0.0; self.last_update_t = None
        self.blocked_since = {}; self.last_reasons = []; self._last_alarm_log = 0.0
        if tf2_ros is not None:
            self.tf_buffer = tf2_ros.Buffer(); self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        else:
            self.tf_buffer = None
        self.create_subscription(LaserScan, 'scan', self.on_scan, qos_profile_sensor_data)
        self.create_subscription(Twist, 'cmd_in', self.on_cmd, 10)
        self.create_subscription(Odometry, 'odom', self.on_odom, 10)
        self.cmd_pub = self.create_publisher(Twist, 'cmd_out', 10)
        self.state_pub = self.create_publisher(String, '~/state', 10)
        self.alarm_pub = self.create_publisher(String, '~/alarm', 10)
        self.marker_pub = self.create_publisher(MarkerArray, '~/markers', 5)
        self.get_logger().info(f'obstacle_guard: footprint {self.fp}, stop margins front/rear/side '
                               f'{g("stop_front")}/{g("stop_rear")}/{g("stop_side")} m')

    # ------------------------------------------------------------------ inputs
    def on_odom(self, m):
        t = m.twist.twist
        self.v_now = (t.linear.x, t.linear.y, t.angular.z)

    def _laser_pose(self, frame):
        if self.laser_pose is not None:
            return self.laser_pose
        if self.tf_buffer is not None:
            try:
                tr = self.tf_buffer.lookup_transform(self.base_frame, frame, rclpy.time.Time())
                q = tr.transform.rotation
                yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))
                self.laser_pose = (tr.transform.translation.x, tr.transform.translation.y, yaw)
                self.get_logger().info(f'laser pose from TF {self.base_frame} <- {frame}: {self.laser_pose}')
                return self.laser_pose
            except Exception:  # noqa: BLE001  (TF not up yet: use the fallback for now)
                pass
        return self.fallback_pose

    def on_scan(self, m):
        now = time.monotonic()
        dt = 0.1 if self.last_update_t is None else min(0.5, now - self.last_update_t)
        self.last_update_t = now; self.last_scan_t = now
        r = np.asarray(m.ranges, dtype=float)
        ang = m.angle_min + m.angle_increment * np.arange(r.size)
        ok = np.isfinite(r) & (r > max(m.range_min, 0.01)) & (r < min(m.range_max, self.max_range))
        lx, ly, lyaw = self._laser_pose(m.header.frame_id)
        a = ang[ok] + lyaw
        px = lx + r[ok] * np.cos(a)
        py = ly + r[ok] * np.sin(a)
        if self.self_poly is not None and px.size:
            keep = ~inside(self.self_poly, px, py)
            px, py = px[keep], py[keep]
        allow = self.guard.update(px, py, self.v_now, dt)
        st = {'allow': {d: round(allow[d], 3) for d in DIRS},
              'dist': {d: (None if not math.isfinite(self.guard.dist[d]) else round(self.guard.dist[d], 3)) for d in DIRS},
              'limited': self.last_reasons}
        self.state_pub.publish(String(data=json.dumps(st)))
        if now - self.last_marker_t >= self.marker_period:
            self.last_marker_t = now
            self.publish_markers(m.header.stamp)

    def on_cmd(self, m):
        now = time.monotonic()
        out = Twist()
        if self.last_scan_t is None or now - self.last_scan_t > self.scan_timeout:
            self.get_logger().warn('obstacle_guard: no recent scan -> stop', throttle_duration_sec=2.0)
            self.cmd_pub.publish(out)
            return
        vx, vy, wz, why = self.guard.filter(m.linear.x, m.linear.y, m.angular.z)
        out.linear.x, out.linear.y, out.angular.z = vx, vy, wz
        self.cmd_pub.publish(out)
        self.last_reasons = why
        self._alarm(now, why, m)

    # ------------------------------------------------------------------ ROS1 obstacle_stop: persistent block alarm
    def _alarm(self, now, why, m):
        moving = abs(m.linear.x) > 1e-3 or abs(m.linear.y) > 1e-3 or abs(m.angular.z) > 1e-3
        blocked = [d for d in why if d != 'oblique' and self.guard.allow.get(d, 1.0) == 0.0]
        for d in list(self.blocked_since):
            if d not in blocked or not moving:
                self.blocked_since.pop(d)
        for d in blocked:
            self.blocked_since.setdefault(d, now)
        if self.blocked_since:
            d, t0 = min(self.blocked_since.items(), key=lambda kv: kv[1])
            dur = now - t0
            if dur > self.alarm_sec and now - self._last_alarm_log > 3.0:
                self._last_alarm_log = now
                msg = {'result': True, 'direction': d, 'blocked_sec': round(dur, 1)}
                self.alarm_pub.publish(String(data=json.dumps(msg)))
                self.get_logger().warn(f'obstacle alarm: {NAMES.get(d, d)} blocked for {dur:.0f} s')

    # ------------------------------------------------------------------ RViz
    def publish_markers(self, stamp):
        ma = MarkerArray()
        any_block = any(self.guard.allow[d] == 0.0 for d in ('x+', 'x-', 'y+', 'y-'))

        def mk(i, mtype, rgba, width):
            k = Marker(); k.header.frame_id = self.base_frame; k.header.stamp = stamp
            k.ns = 'obstacle_guard'; k.id = i; k.type = mtype; k.action = Marker.ADD
            k.pose.orientation.w = 1.0; k.scale.x = width
            k.color = ColorRGBA(r=rgba[0], g=rgba[1], b=rgba[2], a=rgba[3])
            return k
        fpm = mk(0, Marker.LINE_STRIP, (0.95, 0.85, 0.2, 1.0), 0.015)
        for x, y in self.fp + [self.fp[0]]:
            fpm.points.append(Point(x=x, y=y, z=0.05))
        ma.markers.append(fpm)
        zc = (0.9, 0.25, 0.2, 1.0) if any_block else (0.3, 0.85, 0.35, 1.0)
        zm = mk(1, Marker.LINE_STRIP, zc, 0.015)
        for x, y in self.stop_zone + [self.stop_zone[0]]:
            zm.points.append(Point(x=x, y=y, z=0.05))
        ma.markers.append(zm)
        fx = max(p[0] for p in self.fp); rx = min(p[0] for p in self.fp); hw = max(abs(p[1]) for p in self.fp)
        arrows = {'x+': ((fx, 0.0), (fx + 0.35, 0.0)), 'x-': ((rx, 0.0), (rx - 0.35, 0.0)),
                  'y+': ((0.0, hw), (0.0, hw + 0.35)), 'y-': ((0.0, -hw), (0.0, -hw - 0.35))}
        for i, d in enumerate(('x+', 'x-', 'y+', 'y-')):
            ar = mk(10 + i, Marker.ARROW, (0.9, 0.25, 0.2, 0.9), 0.04)
            ar.scale.y = 0.08; ar.scale.z = 0.08
            if self.guard.allow[d] == 0.0:
                (x0, y0), (x1, y1) = arrows[d]
                ar.points = [Point(x=x0, y=y0, z=0.1), Point(x=x1, y=y1, z=0.1)]
            else:
                ar.action = Marker.DELETE
            ma.markers.append(ar)
        self.marker_pub.publish(ma)


def main():
    rclpy.init()
    node = ObstacleGuard()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    rclpy.shutdown()


if __name__ == '__main__':
    main()

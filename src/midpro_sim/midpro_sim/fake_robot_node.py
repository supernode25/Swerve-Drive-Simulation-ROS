#!/usr/bin/env python3
"""fake_robot_node.py — lightweight 2D simulator for the midpro Nav2 stack (no Gazebo).

Publishes
  /clock                     rosgraph_msgs/Clock      (sim time, real-time factor configurable)
  /odom                      nav_msgs/Odometry
  /tf  odom->base_link       (and optionally map->odom identity when no localization is running)
  /scan                      sensor_msgs/LaserScan    ray-cast against the *world* map + injected obstacles
  /sim/ground_truth          geometry_msgs/PoseStamped
  /sim/obstacles             visualization_msgs/MarkerArray
Subscribes
  /cmd_vel                   geometry_msgs/Twist      (vx, vy, wz in base frame)
  /clicked_point             geometry_msgs/PointStamped  -> adds a circular obstacle (RViz "Publish Point")
Services
  /sim/reset                 std_srvs/Trigger         -> back to initial pose, clears dynamic obstacles
  /sim/clear_obstacles       std_srvs/Trigger

Kinematics: holonomic body velocity (swerve). Each axis follows the command through a first-order
lag (tau_vx, tau_vy, tau_wz) plus acceleration limits — a stand-in for wheel steering alignment
delay until real CAN logs give better numbers.
"""
import json
import math
import os
import threading
import time

import numpy as np
import rclpy
import yaml
from builtin_interfaces.msg import Time as TimeMsg
from geometry_msgs.msg import PointStamped, PoseStamped, PoseWithCovarianceStamped, Quaternion, TransformStamped, Twist
from nav2_msgs.srv import ClearEntireCostmap
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSDurabilityPolicy, QoSHistoryPolicy
from rosgraph_msgs.msg import Clock
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String
from std_srvs.srv import Trigger
from tf2_ros import TransformBroadcaster
from visualization_msgs.msg import Marker, MarkerArray

try:
    from PIL import Image
except ImportError:  # pragma: no cover
    Image = None


def yaw_to_quat(yaw: float) -> Quaternion:
    q = Quaternion()
    q.z = math.sin(yaw / 2.0)
    q.w = math.cos(yaw / 2.0)
    return q


class WorldMap:
    """Occupancy grid loaded from a map_server style yaml+pgm. True = occupied."""

    def __init__(self, yaml_path: str, unknown_as_occupied: bool = True):
        with open(yaml_path, 'r') as f:
            meta = yaml.safe_load(f)
        img_path = meta['image']
        if not os.path.isabs(img_path):
            img_path = os.path.join(os.path.dirname(yaml_path), img_path)
        if Image is None:
            raise RuntimeError('Pillow is required to load the world map (pip install pillow)')
        img = Image.open(img_path).convert('L')
        arr = np.array(img, dtype=np.float32) / 255.0
        negate = int(meta.get('negate', 0))
        occ_th = float(meta.get('occupied_thresh', 0.65))
        free_th = float(meta.get('free_thresh', 0.196))
        if negate:
            occ = arr
        else:
            occ = 1.0 - arr
        grid = occ >= occ_th
        if unknown_as_occupied:
            # real SLAM maps (e.g. ecoproBM) have large unknown (grey) areas; treat them as walls so
            # the simulated laser does not see through them and the robot cannot drive into them
            grid |= (occ > free_th) & (occ < occ_th)
        # pgm row 0 is the top of the image; map row 0 is the bottom
        self.grid = np.flipud(grid)
        self.res = float(meta['resolution'])
        self.origin = meta['origin'][:2]
        self.h, self.w = self.grid.shape

    def occupied(self, x: float, y: float) -> bool:
        mx = int((x - self.origin[0]) / self.res)
        my = int((y - self.origin[1]) / self.res)
        if mx < 0 or my < 0 or mx >= self.w or my >= self.h:
            return True  # outside the map counts as a wall
        return bool(self.grid[my, mx])


class FakeRobot(Node):
    def __init__(self):
        super().__init__('fake_robot')
        # ---- parameters
        self.declare_parameter('world_map_yaml', '')
        self.declare_parameter('unknown_as_occupied', True)
        self.declare_parameter('rate', 50.0)                # physics Hz
        self.declare_parameter('real_time_factor', 1.0)
        self.declare_parameter('publish_clock', False)      # True: drive /clock (all other nodes use_sim_time)
        self.declare_parameter('publish_map_odom_tf', False)  # true when no AMCL/SLAM is running
        self.declare_parameter('initial_x', 1.5)
        self.declare_parameter('initial_y', 1.5)
        self.declare_parameter('initial_yaw', 0.0)
        self.declare_parameter('tau_vx', 0.15)              # s, first-order lag
        self.declare_parameter('tau_vy', 0.40)              # lateral: wheels must re-align -> slower
        self.declare_parameter('tau_wz', 0.25)
        self.declare_parameter('acc_vx', 0.5)
        self.declare_parameter('acc_vy', 0.3)
        self.declare_parameter('acc_wz', 1.0)
        self.declare_parameter('cmd_timeout', 0.5)
        self.declare_parameter('scan_rate', 10.0)
        self.declare_parameter('scan_range_max', 12.0)
        self.declare_parameter('scan_range_min', 0.05)
        self.declare_parameter('scan_samples', 360)
        self.declare_parameter('scan_fov_deg', 360.0)
        self.declare_parameter('scan_noise_std', 0.01)
        self.declare_parameter('laser_offset', [0.3, 0.0, 0.0])  # x, y, yaw in base_link
        self.declare_parameter('odom_noise_std', [0.0, 0.0, 0.0])  # per-step noise vx, vy, wz
        self.declare_parameter('obstacle_yaml', '')
        self.declare_parameter('clicked_obstacle_radius', 0.15)
        self.declare_parameter('base_frame', 'base_link')
        self.declare_parameter('odom_frame', 'odom')
        self.declare_parameter('map_frame', 'map')
        self.declare_parameter('laser_frame', 'laser')

        gp = lambda n: self.get_parameter(n).value  # noqa: E731
        self.rate = float(gp('rate'))
        self.rtf = float(gp('real_time_factor'))
        self.publish_clock_ = bool(gp('publish_clock'))
        self.publish_map_odom = bool(gp('publish_map_odom_tf'))
        self.init_pose = [float(gp('initial_x')), float(gp('initial_y')), float(gp('initial_yaw'))]
        self.tau = np.array([gp('tau_vx'), gp('tau_vy'), gp('tau_wz')], dtype=float)
        self.acc = np.array([gp('acc_vx'), gp('acc_vy'), gp('acc_wz')], dtype=float)
        self.cmd_timeout = float(gp('cmd_timeout'))
        self.scan_rate = float(gp('scan_rate'))
        self.range_max = float(gp('scan_range_max'))
        self.range_min = float(gp('scan_range_min'))
        self.samples = int(gp('scan_samples'))
        self.fov = math.radians(float(gp('scan_fov_deg')))
        self.scan_noise = float(gp('scan_noise_std'))
        self.laser_off = list(gp('laser_offset'))
        self.odom_noise = np.array(gp('odom_noise_std'), dtype=float)
        self.click_r = float(gp('clicked_obstacle_radius'))
        self.base_frame, self.odom_frame = gp('base_frame'), gp('odom_frame')
        self.map_frame, self.laser_frame = gp('map_frame'), gp('laser_frame')

        wm = gp('world_map_yaml')
        if not wm:
            raise RuntimeError('world_map_yaml parameter is required')
        self.world = WorldMap(wm, bool(gp('unknown_as_occupied')))
        self.get_logger().info(f'world map {self.world.w}x{self.world.h} @ {self.world.res} m')

        # obstacles: list of dicts {type: circle, x, y, r} or {type: box, x, y, w, h, yaw}
        self.static_obstacles = []
        ob = gp('obstacle_yaml')
        if ob:
            with open(ob, 'r') as f:
                self.static_obstacles = yaml.safe_load(f).get('obstacles', [])
        self.dynamic_obstacles = []

        # ---- state (map frame ground truth; odom == map at start)
        self.sim_time = 0.0
        self.pose = np.array(self.init_pose, dtype=float)      # x, y, yaw
        self.vel = np.zeros(3)                                  # actual body vel vx, vy, wz
        self.cmd = np.zeros(3)
        self.last_cmd_wall = time.monotonic()
        self.odom_pose = np.array(self.init_pose, dtype=float)  # integrated with noise (what odom reports)

        # ---- ROS I/O
        sensor_qos = QoSProfile(depth=5, reliability=QoSReliabilityPolicy.BEST_EFFORT,
                                history=QoSHistoryPolicy.KEEP_LAST)
        clock_qos = QoSProfile(depth=1, reliability=QoSReliabilityPolicy.RELIABLE,
                               durability=QoSDurabilityPolicy.VOLATILE)
        self.clock_pub = self.create_publisher(Clock, '/clock', clock_qos)
        self.odom_pub = self.create_publisher(Odometry, '/odom', 10)
        self.scan_pub = self.create_publisher(LaserScan, '/scan', sensor_qos)
        self.gt_pub = self.create_publisher(PoseStamped, '/sim/ground_truth', 10)
        self.marker_pub = self.create_publisher(MarkerArray, '/sim/obstacles', 1)
        self.tf_bc = TransformBroadcaster(self)
        self.create_subscription(Twist, '/cmd_vel', self.on_cmd, 10)
        self.create_subscription(PointStamped, '/clicked_point', self.on_click, 10)
        self.create_subscription(String, '/sim/obstacle_cmd', self.on_obstacle_cmd, 10)
        self.obst_state_pub = self.create_publisher(String, '/sim/obstacle_state', 1)   # v0.50: was /sim/obstacles (clashed with the MarkerArray)
        self._obst_seq = 0
        self._last_obst_pub = 0.0
        self.create_service(Trigger, '/sim/reset', self.on_reset)
        self.create_service(Trigger, '/sim/clear_obstacles', self.on_clear)
        self.create_subscription(PoseStamped, '/sim/set_pose', self.on_set_pose, 10)
        self._clear_cli = {'local': self.create_client(ClearEntireCostmap, '/local_costmap/clear_entirely_local_costmap'),
                           'global': self.create_client(ClearEntireCostmap, '/global_costmap/clear_entirely_global_costmap')}
        self._clear_pending = set(); self._clear_deadline = 0.0
        self._amcl_confirm = None                    # (x, y, yaw, due) -> re-send /initialpose after the teleport
        self._init_pub = self.create_publisher(PoseWithCovarianceStamped, '/initialpose', 1)
        self.create_timer(0.1, self._teleport_followup)
        self._last_teleport = (None, 0.0)

        self.dt = 1.0 / self.rate
        self.step_count = 0
        self.publish_markers()
        # physics runs in its own thread with precise pacing, independent of the rclpy executor
        self._lock = threading.Lock()
        self._running = True
        self._thread = threading.Thread(target=self._loop, daemon=True)
        self._thread.start()

    def _loop(self):
        period = self.dt
        next_t = time.perf_counter()
        while self._running and rclpy.ok():
            try:
                with self._lock:
                    self.step()
            except Exception as e:  # noqa: BLE001
                self.get_logger().error(f'sim step failed: {e}')
            next_t += period
            delay = next_t - time.perf_counter()
            if delay > 0:
                time.sleep(delay)
            else:
                next_t = time.perf_counter()   # fell behind: resync instead of bursting

    # ------------------------------------------------------------------ callbacks
    def on_cmd(self, msg: Twist):
        with self._lock:
            self.cmd = np.array([msg.linear.x, msg.linear.y, msg.angular.z], dtype=float)
            self.last_cmd_wall = time.monotonic()


    # ------------------------------------------------------------------ obstacle editor (dashboard tab)
    def on_obstacle_cmd(self, msg: String):
        """JSON commands from the dashboard obstacle editor.
        add:    {"op":"add","type":"circle"|"box","x":..,"y":..,"r":..|"w":..,"h":..,"yaw":..,
                 "motion":{"kind":"pingpong"|"loop"|"velocity","points":[[x,y],...],"speed":..,"vx":..,"vy":..}}
        move:   {"op":"move","id":..,"x":..,"y":..}
        update: {"op":"update","id":..,"x":..,"y":..,"r"|"w","h","yaw":..}   (editor drag / resize / rotate)
        remove: {"op":"remove","id":..}    clear: {"op":"clear"}
        Obstacles are sensor-level only: they exist in the ray-cast (scan), never written to the map/costmap.
        """
        try:
            cmd = json.loads(msg.data)
        except Exception as e:  # noqa: BLE001
            self.get_logger().warn(f'bad obstacle command: {e}')
            return
        with self._lock:
            op = cmd.get('op')
            if op == 'clear':
                self.dynamic_obstacles.clear()
            elif op == 'remove':
                self.dynamic_obstacles = [o for o in self.dynamic_obstacles if o.get('id') != cmd.get('id')]
            elif op in ('move', 'update'):
                # move / update: change position (and size / angle for 'update'); a motion path moves with it
                for o in self.dynamic_obstacles:
                    if o.get('id') != cmd.get('id'):
                        continue
                    dx = float(cmd.get('x', o['x'])) - o['x']; dy = float(cmd.get('y', o['y'])) - o['y']
                    for k in ('x', 'y', 'r', 'w', 'h', 'yaw'):
                        if k in cmd:
                            o[k] = float(cmd[k])
                    mo = o.get('motion')
                    if mo and (abs(dx) > 1e-9 or abs(dy) > 1e-9):
                        mo['points'] = [[p[0] + dx, p[1] + dy] for p in mo['points']]
            elif op == 'add':
                self._obst_seq += 1
                o = {'id': cmd.get('id', f'o{self._obst_seq}'), 'type': cmd.get('type', 'circle'),
                     'x': float(cmd['x']), 'y': float(cmd['y']), 'r': float(cmd.get('r', 0.15)),
                     'w': float(cmd.get('w', 0.5)), 'h': float(cmd.get('h', 0.5)), 'yaw': float(cmd.get('yaw', 0.0))}
                mo = cmd.get('motion')
                if mo:
                    o['motion'] = {'kind': mo.get('kind', 'pingpong'), 'points': [[float(a), float(b)] for a, b in mo.get('points', [[o['x'], o['y']]])],
                                   'speed': float(mo.get('speed', 0.5)), 'vx': float(mo.get('vx', 0.0)), 'vy': float(mo.get('vy', 0.0)),
                                   'seg': 0, 'dir': 1}
                self.dynamic_obstacles.append(o)
            self.publish_markers()

    def update_obstacle_motion(self, dt: float):
        for o in self.dynamic_obstacles:
            mo = o.get('motion')
            if not mo:
                continue
            if mo['kind'] == 'velocity':
                o['x'] += mo['vx'] * dt; o['y'] += mo['vy'] * dt
                continue
            pts = mo['points']
            if len(pts) < 2:
                continue
            tgt = pts[mo['seg'] + mo['dir']] if 0 <= mo['seg'] + mo['dir'] < len(pts) else pts[mo['seg']]
            dx, dy = tgt[0] - o['x'], tgt[1] - o['y']
            d = math.hypot(dx, dy)
            step = mo['speed'] * dt
            if d <= step:
                o['x'], o['y'] = tgt
                nxt = mo['seg'] + mo['dir']
                if mo['kind'] == 'loop':
                    mo['seg'] = nxt % len(pts)
                    if nxt >= len(pts):
                        mo['seg'] = 0
                else:  # pingpong
                    mo['seg'] = nxt
                    if mo['seg'] >= len(pts) - 1 or mo['seg'] <= 0:
                        mo['dir'] = -mo['dir']
            elif d > 1e-6:
                o['x'] += dx / d * step; o['y'] += dy / d * step

    def publish_obstacle_state(self):
        m = String()
        m.data = json.dumps([{k: v for k, v in o.items() if k != 'motion'} | ({'motion': {'kind': o['motion']['kind'], 'points': o['motion']['points'], 'speed': o['motion']['speed']}} if o.get('motion') else {})
                             for o in self.dynamic_obstacles])
        self.obst_state_pub.publish(m)

    def on_click(self, msg: PointStamped):
        with self._lock:
            self.dynamic_obstacles.append({'type': 'circle', 'x': msg.point.x, 'y': msg.point.y, 'r': self.click_r})
        self.get_logger().info(f'obstacle added at ({msg.point.x:.2f}, {msg.point.y:.2f})')
        self.publish_markers()

    def on_reset(self, _req, res):
        with self._lock:
            self.pose = np.array(self.init_pose, dtype=float)
            self.odom_pose = np.array(self.init_pose, dtype=float)
            self.vel[:] = 0.0
            self.cmd[:] = 0.0
            self.dynamic_obstacles.clear()
        self.publish_markers()
        res.success = True
        res.message = 'reset'
        return res

    def on_set_pose(self, msg: PoseStamped):
        """Teleport (regression runner / dashboard layout load) via /sim/set_pose. RViz '2D Pose Estimate'
        (/initialpose) does NOT move the simulated robot: it only re-initialises AMCL, as in Nav2."""
        self._teleport(msg.pose.position.x, msg.pose.position.y, msg.pose.orientation, 'set_pose')

    def _teleport(self, x, y, q, src):
        """Move the ground-truth pose, keep odometry continuous so map->odom jumps like a localisation
        correction; zero velocities and commands; clear the local costmap (odom frame), which would otherwise
        keep the obstacles seen at the old place around the robot."""
        yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))
        key = (round(x, 3), round(y, 3), round(yaw, 3))
        now = time.monotonic()
        if self._last_teleport[0] == key and now - self._last_teleport[1] < 1.0:
            return                                   # same request twice
        self._last_teleport = (key, now)
        with self._lock:
            self.pose = np.array([x, y, yaw], dtype=float)
            self.vel[:] = 0.0
            self.cmd[:] = 0.0
        self.get_logger().info(f'teleport ({src}) to ({x:.2f}, {y:.2f}, {math.degrees(yaw):.0f} deg)')
        # costmaps keep what was seen at the old place (local: in odom, now around the robot); clear both,
        # retrying until the services answer
        self._clear_pending = {'local', 'global'}; self._clear_deadline = now + 3.0
        # AMCL may have evaluated the same /initialpose against a scan from the old place: confirm once more
        # after the teleport, from the true pose (own message is de-duplicated above)
        if not self.publish_map_odom:
            self._amcl_confirm = (x, y, yaw, now + 0.3)

    def _teleport_followup(self):
        now = time.monotonic()
        for k in list(self._clear_pending):
            cli = self._clear_cli[k]
            if cli.service_is_ready():
                cli.call_async(ClearEntireCostmap.Request())
                self._clear_pending.discard(k)
        if self._clear_pending and now > self._clear_deadline:
            self.get_logger().warn(f'costmap clear after teleport not done ({", ".join(sorted(self._clear_pending))}): '
                                   'old obstacles may remain until the laser sees through them')
            self._clear_pending.clear()
        if self._amcl_confirm and now >= self._amcl_confirm[3]:
            x, y, yaw, _ = self._amcl_confirm; self._amcl_confirm = None
            m = PoseWithCovarianceStamped(); m.header.frame_id = self.map_frame
            m.header.stamp = self.get_clock().now().to_msg()
            m.pose.pose.position.x, m.pose.pose.position.y = float(x), float(y)
            m.pose.pose.orientation = yaw_to_quat(float(yaw))
            m.pose.covariance[0] = m.pose.covariance[7] = 0.0025; m.pose.covariance[35] = 0.0012
            self._init_pub.publish(m)
            self.get_logger().info('AMCL initial pose confirmed from the true pose after the teleport')

    def on_clear(self, _req, res):
        with self._lock:
            self.dynamic_obstacles.clear()
        self.publish_markers()
        res.success = True
        res.message = 'cleared'
        return res

    # ------------------------------------------------------------------ physics
    def step(self):
        # advance sim time by the real elapsed time (x real_time_factor) so the simulation keeps
        # real-time pace even when the timer cannot hold its nominal rate (WSL, heavy RViz, ...)
        now_wall = time.monotonic()
        if not hasattr(self, '_last_step_wall'):
            self._last_step_wall = now_wall
        dt = min(max((now_wall - self._last_step_wall) * self.rtf, 1e-4), 0.1)
        self._last_step_wall = now_wall
        self.sim_time += dt
        self.update_obstacle_motion(dt)
        if now_wall - self.last_cmd_wall > self.cmd_timeout:
            self.cmd[:] = 0.0
        # first-order lag toward command, then acceleration clamp
        dv = (self.cmd - self.vel) * (dt / np.maximum(self.tau, 1e-3))
        dv = np.clip(dv, -self.acc * dt, self.acc * dt)
        self.vel += dv

        yaw = self.pose[2]
        c, s = math.cos(yaw), math.sin(yaw)
        dx = (self.vel[0] * c - self.vel[1] * s) * dt
        dy = (self.vel[0] * s + self.vel[1] * c) * dt
        nx, ny = self.pose[0] + dx, self.pose[1] + dy
        # very simple collision: stop if the body centre would enter an occupied cell / obstacle
        if not self.blocked(nx, ny):
            self.pose[0], self.pose[1] = nx, ny
        else:
            self.vel[0] = self.vel[1] = 0.0
        self.pose[2] = self.wrap(yaw + self.vel[2] * dt)

        # odom integrates the same velocity plus optional noise (drift model)
        nv = self.vel + np.random.normal(0.0, 1.0, 3) * self.odom_noise
        oy = self.odom_pose[2]
        self.odom_pose[0] += (nv[0] * math.cos(oy) - nv[1] * math.sin(oy)) * dt
        self.odom_pose[1] += (nv[0] * math.sin(oy) + nv[1] * math.cos(oy)) * dt
        self.odom_pose[2] = self.wrap(oy + nv[2] * dt)

        stamp = self.stamp()
        self.publish_clock(stamp)
        self.publish_odom(stamp)
        self.publish_tf(stamp)
        self.step_count += 1
        if self.sim_time - self._last_obst_pub >= 0.5:
            self._last_obst_pub = self.sim_time
            self.publish_obstacle_state()
            if any(o.get('motion') for o in self.dynamic_obstacles):
                self.publish_markers()
        if self.sim_time - getattr(self, '_last_scan_time', -1.0) >= 1.0 / self.scan_rate:
            self._last_scan_time = self.sim_time
            self.publish_scan(stamp)
            self.publish_gt(stamp)

    # ------------------------------------------------------------------ world queries
    def blocked(self, x: float, y: float) -> bool:
        if self.world.occupied(x, y):
            return True
        return self.hit_obstacle(x, y)

    def hit_obstacle(self, x: float, y: float) -> bool:
        for o in self.static_obstacles + self.dynamic_obstacles:
            if o['type'] == 'circle':
                if (x - o['x']) ** 2 + (y - o['y']) ** 2 <= o['r'] ** 2:
                    return True
            elif o['type'] == 'box':
                yaw = float(o.get('yaw', 0.0))
                lx = (x - o['x']) * math.cos(-yaw) - (y - o['y']) * math.sin(-yaw)
                ly = (x - o['x']) * math.sin(-yaw) + (y - o['y']) * math.cos(-yaw)
                if abs(lx) <= o['w'] / 2.0 and abs(ly) <= o['h'] / 2.0:
                    return True
        return False

    def raycast_all(self, x0: float, y0: float, angles: np.ndarray) -> np.ndarray:
        """Vectorised ray casting: returns range per angle (inf if no hit)."""
        step = self.world.res * 0.5
        rs = np.arange(self.range_min, self.range_max, step)              # (M,)
        ca, sa = np.cos(angles)[:, None], np.sin(angles)[:, None]           # (N,1)
        xs = x0 + rs[None, :] * ca                                         # (N,M)
        ys = y0 + rs[None, :] * sa
        mx = ((xs - self.world.origin[0]) / self.world.res).astype(np.int64)
        my = ((ys - self.world.origin[1]) / self.world.res).astype(np.int64)
        inside = (mx >= 0) & (my >= 0) & (mx < self.world.w) & (my < self.world.h)
        occ = ~inside                                                      # outside the map = wall
        occ[inside] = self.world.grid[my[inside], mx[inside]]
        for o in self.static_obstacles + self.dynamic_obstacles:
            if o['type'] == 'circle':
                occ |= (xs - o['x']) ** 2 + (ys - o['y']) ** 2 <= o['r'] ** 2
            elif o['type'] == 'box':
                yaw = float(o.get('yaw', 0.0))
                c, sn = math.cos(-yaw), math.sin(-yaw)
                lx = (xs - o['x']) * c - (ys - o['y']) * sn
                ly = (xs - o['x']) * sn + (ys - o['y']) * c
                occ |= (np.abs(lx) <= o['w'] / 2.0) & (np.abs(ly) <= o['h'] / 2.0)
        hit = occ.any(axis=1)
        first = occ.argmax(axis=1)
        out = np.full(angles.shape[0], np.inf)
        out[hit] = rs[first[hit]]
        return out

    # ------------------------------------------------------------------ publishers
    def stamp(self) -> TimeMsg:
        if not self.publish_clock_:
            return self.get_clock().now().to_msg()   # wall clock, same as every other node
        t = TimeMsg()
        t.sec = int(self.sim_time)
        t.nanosec = int((self.sim_time - t.sec) * 1e9)
        return t

    def publish_clock(self, stamp):
        if not self.publish_clock_:
            return
        m = Clock()
        m.clock = stamp
        self.clock_pub.publish(m)

    def publish_odom(self, stamp):
        o = Odometry()
        o.header.stamp = stamp
        o.header.frame_id = self.odom_frame
        o.child_frame_id = self.base_frame
        o.pose.pose.position.x = float(self.odom_pose[0])
        o.pose.pose.position.y = float(self.odom_pose[1])
        o.pose.pose.orientation = yaw_to_quat(float(self.odom_pose[2]))
        o.twist.twist.linear.x = float(self.vel[0])
        o.twist.twist.linear.y = float(self.vel[1])
        o.twist.twist.angular.z = float(self.vel[2])
        cov = [0.0] * 36
        cov[0] = cov[7] = 1e-3
        cov[35] = 1e-3
        o.pose.covariance = cov
        o.twist.covariance = cov
        self.odom_pub.publish(o)

    def publish_tf(self, stamp):
        tfs = []
        t = TransformStamped()
        t.header.stamp = stamp
        t.header.frame_id = self.odom_frame
        t.child_frame_id = self.base_frame
        t.transform.translation.x = float(self.odom_pose[0])
        t.transform.translation.y = float(self.odom_pose[1])
        t.transform.rotation = yaw_to_quat(float(self.odom_pose[2]))
        tfs.append(t)
        if self.publish_map_odom:
            # ground truth vs odom -> map->odom (identity if odom noise is zero)
            dx = self.pose[0] - self.odom_pose[0]
            dy = self.pose[1] - self.odom_pose[1]
            dyaw = self.wrap(self.pose[2] - self.odom_pose[2])
            m = TransformStamped()
            m.header.stamp = stamp
            m.header.frame_id = self.map_frame
            m.child_frame_id = self.odom_frame
            # map->odom such that map->base = ground truth: T_mo = T_mb * T_ob^-1
            ox, oy, oyaw = self.odom_pose
            c, s = math.cos(dyaw), math.sin(dyaw)
            m.transform.translation.x = float(self.pose[0] - (c * ox - s * oy))
            m.transform.translation.y = float(self.pose[1] - (s * ox + c * oy))
            m.transform.rotation = yaw_to_quat(float(dyaw))
            tfs.append(m)
        self.tf_bc.sendTransform(tfs)

    def publish_scan(self, stamp):
        yaw = self.pose[2]
        lx, ly, lyaw = self.laser_off
        sx = self.pose[0] + lx * math.cos(yaw) - ly * math.sin(yaw)
        sy = self.pose[1] + lx * math.sin(yaw) + ly * math.cos(yaw)
        syaw = yaw + lyaw
        scan = LaserScan()
        scan.header.stamp = stamp
        scan.header.frame_id = self.laser_frame
        scan.angle_min = -self.fov / 2.0
        scan.angle_max = self.fov / 2.0
        scan.angle_increment = self.fov / self.samples
        scan.time_increment = 0.0
        scan.scan_time = 1.0 / self.scan_rate
        scan.range_min = self.range_min
        scan.range_max = self.range_max
        angles = syaw + scan.angle_min + np.arange(self.samples) * scan.angle_increment
        rng = self.raycast_all(sx, sy, angles)
        if self.scan_noise > 0.0:
            finite = np.isfinite(rng)
            rng[finite] = np.maximum(self.range_min, rng[finite] + np.random.normal(0.0, self.scan_noise, finite.sum()))
        ranges = [float(v) for v in rng]
        scan.ranges = ranges
        self.scan_pub.publish(scan)

    def publish_gt(self, stamp):
        p = PoseStamped()
        p.header.stamp = stamp
        p.header.frame_id = self.map_frame
        p.pose.position.x = float(self.pose[0])
        p.pose.position.y = float(self.pose[1])
        p.pose.orientation = yaw_to_quat(float(self.pose[2]))
        self.gt_pub.publish(p)

    def publish_markers(self):
        arr = MarkerArray()
        clear = Marker()
        clear.action = Marker.DELETEALL
        arr.markers.append(clear)
        for i, o in enumerate(self.static_obstacles + self.dynamic_obstacles):
            m = Marker()
            m.header.frame_id = self.map_frame
            m.ns = 'sim_obstacles'
            m.id = i
            m.action = Marker.ADD
            m.pose.position.x = float(o['x'])
            m.pose.position.y = float(o['y'])
            m.pose.position.z = 0.25
            m.color.r, m.color.g, m.color.b, m.color.a = 0.35, 0.35, 0.35, 0.9
            if o['type'] == 'circle':
                m.type = Marker.CYLINDER
                m.scale.x = m.scale.y = float(o['r']) * 2.0
            else:
                m.type = Marker.CUBE
                m.pose.orientation = yaw_to_quat(float(o.get('yaw', 0.0)))
                m.scale.x, m.scale.y = float(o['w']), float(o['h'])
            m.scale.z = 0.5
            arr.markers.append(m)
        self.marker_pub.publish(arr)

    @staticmethod
    def wrap(a: float) -> float:
        while a > math.pi:
            a -= 2.0 * math.pi
        while a < -math.pi:
            a += 2.0 * math.pi
        return a


def main(args=None):
    rclpy.init(args=args)
    node = FakeRobot()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node._running = False
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

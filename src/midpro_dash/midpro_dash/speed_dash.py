#!/usr/bin/env python3
"""speed_dash.py — live driving dashboard for the midpro Nav2 stack (PyQt5 + pyqtgraph).

    ros2 run midpro_dash speed_dash            # default 30 s window, dark background
    ros2 run midpro_dash speed_dash --light    # light background
    ros2 run midpro_dash speed_dash --window 60 --csv run1.csv

Plots (left)                                     Text (right)
  1. speed: controller out / final cmd / actual    FSM state, path mode, speed-limit reason
  2. acceleration (actual, filtered) + jerk        v, a, |a|max(5 s), accel RMS, steering reversals/min
  3. steering: cmd wz, actual wz, cmd vy           distance since goal, elapsed, profile cap
  4. planned speed profile v(s) with the robot's progress marker

Everything is pulled from topics the stack already publishes; no changes to the robot side.
"""
import argparse
import collections
import csv
import math
import sys
import threading
import time

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry, Path
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
from std_msgs.msg import Float32MultiArray, String
from rcl_interfaces.srv import GetParameters, SetParameters
from rcl_interfaces.msg import Parameter as ParamMsg, ParameterValue, ParameterType

from midpro_dash import report as rp
from midpro_dash.events import LiveRules
from midpro_dash.obstacle_tab import ObstacleTab
from nav_msgs.msg import OccupancyGrid
import os

from PyQt5 import QtCore, QtGui, QtWidgets
import pyqtgraph as pg

ACCENT = '#1f4e8c'
GREY = '#8a93a0'
INK = '#1b1f24'
LINE = '#d9dde3'


class Bus(Node):
    """ROS side: buffers everything the dashboard draws (thread-safe snapshots)."""

    def __init__(self, window_s):
        super().__init__('midpro_speed_dash')
        self.lock = threading.Lock()
        self.t0 = time.monotonic()
        n = int(window_s * 60)
        self.cmd_nav = collections.deque(maxlen=n)   # (t, vx, vy, wz)
        self.cmd = collections.deque(maxlen=n)
        self.odom = collections.deque(maxlen=n)      # (t, v, vx, vy, wz)
        self.acc = collections.deque(maxlen=n)       # (t, a)
        self.jerk = collections.deque(maxlen=n)
        self.state = '-'; self.path_mode = '-'; self.limit = '-'; self.transition = '-'
        self.profile_s = []; self.profile_v = []; self.profile_t = None
        self.dist_since_plan = 0.0; self.last_odom = None
        self.goal_t = None
        self._v_f = 0.0; self._a_f = 0.0; self._t_prev = None
        # run-level scoring
        self.plan_xy = []; self.plan_goal = None; self._plan_hint = 0
        self.gt = None                      # (x, y, yaw) from /sim/ground_truth, else odom pose
        self.xte = collections.deque(maxlen=n)   # (t, cross-track error)
        self.run_active = False; self.run_frozen = False
        self.final_err = None               # (pos_err, yaw_err_deg)
        self.vmax = 0.6
        # tuning-report evidence
        self.reason_log = collections.deque(maxlen=n)        # (t, reason key)
        self.profile_gap_log = collections.deque(maxlen=n)   # (t, v_act, profile_v, state)
        self.state_time = {}; self._state_t0 = None
        self.rotate_count = 0; self.rotate_count_transit = 0
        self.plan_count_run = 0; self.detour_time = 0.0; self._mode_t = None
        self.params = {}
        self._param_cli = self.create_client(GetParameters, '/controller_server/get_parameters')
        self._set_cli = self.create_client(SetParameters, '/controller_server/set_parameters')
        self.manual = False
        self.map_msg = None; self.map_dirty = False
        self.obstacles = []; self.obstacles_dirty = False
        self.create_subscription(OccupancyGrid, '/map', self.on_map, rclpy.qos.QoSProfile(depth=1, durability=rclpy.qos.DurabilityPolicy.TRANSIENT_LOCAL))
        self.create_subscription(String, '/sim/obstacle_state', self.on_obstacles, 5)
        self.obst_pub = self.create_publisher(String, '/sim/obstacle_cmd', 10)
        from nav2_msgs.srv import ClearEntireCostmap
        self._ClearSrv = ClearEntireCostmap
        self._clear_cli = [self.create_client(ClearEntireCostmap, '/local_costmap/clear_entirely_local_costmap'),
                           self.create_client(ClearEntireCostmap, '/global_costmap/clear_entirely_global_costmap')]
        self.entry = '-'
        self.last_goal = None
        self.guard_txt = '-'
        self.loc_txt = '-'; self._amcl = None
        self.create_subscription(PoseWithCovarianceStamped, '/amcl_pose', self.on_amcl, 5)
        self.create_subscription(String, '/obstacle_guard/state', self.on_guard, 10)
        self.map_name = ''
        self.pose_pub = self.create_publisher(PoseStamped, '/sim/set_pose', 1)
        self.init_pub = self.create_publisher(PoseWithCovarianceStamped, '/initialpose', 1)
        self.goal_pub = self.create_publisher(PoseStamped, '/goal_pose', 1)
        self.block_events = collections.deque(maxlen=500)     # (t, dict) from /midpro/block_event
        self.clear_log = collections.deque(maxlen=n)          # (t, footprint clearance m) from /scan
        self.create_subscription(String, '/midpro/block_event', self.on_block_event, 20)
        from sensor_msgs.msg import LaserScan
        self.create_subscription(LaserScan, '/scan', self.on_scan, rclpy.qos.qos_profile_sensor_data)
        self._scan_skip = 0
        self.create_subscription(String, '/midpro/goal_entry', self.on_entry, rclpy.qos.QoSProfile(depth=1, durability=rclpy.qos.DurabilityPolicy.TRANSIENT_LOCAL))
        self.events_log = []

    def on_map(self, m):
        with self.lock:
            self.map_msg = m; self.map_dirty = True

    def on_obstacles(self, m):
        import json
        try:
            obs = json.loads(m.data)
        except Exception:  # noqa: BLE001
            return
        with self.lock:
            self.obstacles = obs; self.obstacles_dirty = True

    def set_bool_param(self, name, value, done=None):
        """Set FollowPath.<name> on controller_server at runtime (plugin dynamic parameter callback)."""
        if not self._set_cli.service_is_ready():
            if done:
                done(False)
            return
        p = ParamMsg(); p.name = 'FollowPath.' + name
        p.value = ParameterValue(type=ParameterType.PARAMETER_BOOL, bool_value=bool(value))
        req = SetParameters.Request(); req.parameters = [p]
        fut = self._set_cli.call_async(req)

        def cb(f):
            ok = False
            try:
                ok = all(r.successful for r in f.result().results)
            except Exception:  # noqa: BLE001
                pass
            if done:
                done(ok)
        fut.add_done_callback(cb)

    def on_block_event(self, m):
        import json
        try:
            e = json.loads(m.data)
        except Exception:  # noqa: BLE001
            return
        with self.lock:
            self.block_events.append((self.now(), e))

    def on_scan(self, m):
        self._scan_skip = (self._scan_skip + 1) % 2           # 5 Hz is plenty for the clearance trace
        if self._scan_skip or not (self.run_active and not self.run_frozen):
            return
        from midpro_dash import metrics as MM
        c = MM.footprint_clearance(m.ranges, m.angle_min, m.angle_increment, (0.25, 0.0, 0.0), (0.42, -0.48, 0.25))
        with self.lock:
            self.clear_log.append((self.now(), c))

    def on_entry(self, m):
        import json
        try:
            e = json.loads(m.data)
        except Exception:  # noqa: BLE001
            return
        names = {'ok': '가능', 'tight': '좁음', 'blocked': '불가', 'unknown': '?'}
        txt = names.get(e.get('status'), '?')
        if e.get('clearance') is not None and e.get('status') in ('ok', 'tight'):
            txt += f" (여유 {e['clearance'] * 100:.0f} cm)"
        if e.get('approach') not in (None, 'ok'):
            txt += f" · 접근 {names.get(e['approach'], '?')}"
        with self.lock:
            self.entry = txt
            if e.get('goal'):
                self.last_goal = tuple(e['goal'])          # (x, y, yaw_deg) of the goal actually used

    def on_amcl(self, m):
        p = m.pose.pose; q = p.orientation
        self._amcl = (p.position.x, p.position.y, math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z)))

    def localization_error(self):
        """AMCL estimate vs simulator ground truth (sim + AMCL only)."""
        gt = getattr(self, 'gt', None)
        if self._amcl is None or gt is None:
            return None
        de = math.hypot(self._amcl[0] - gt[0], self._amcl[1] - gt[1])
        dy = abs((self._amcl[2] - gt[2] + math.pi) % (2 * math.pi) - math.pi)
        return de, math.degrees(dy)

    def on_guard(self, m):
        import json
        try:
            st = json.loads(m.data)
        except Exception:  # noqa: BLE001
            return
        names = {'x+': '전진', 'x-': '후진', 'y+': '좌', 'y-': '우', 'ccw': '좌회전', 'cw': '우회전'}
        parts = []
        for d in ('x+', 'x-', 'y+', 'y-', 'ccw', 'cw'):
            a = st.get('allow', {}).get(d)
            if a is None or a >= 0.95:
                continue
            parts.append(f"{names[d]} 정지" if a == 0.0 else f"{names[d]} {a:.2f}")
        with self.lock:
            self.guard_txt = ' · '.join(parts) if parts else '제한 없음'

    def robot_pose(self):
        """(x, y, yaw_deg) of the robot in the map frame (ground truth in sim), or None."""
        with self.lock:
            p = getattr(self, '_pose', None)
        return None if p is None else (round(p[0], 3), round(p[1], 3), round(math.degrees(p[2]), 1))

    def _pose_msg(self, x, y, yaw_deg, cls=PoseStamped):
        m = cls(); m.header.frame_id = 'map'; m.header.stamp = self.get_clock().now().to_msg()
        pose = m.pose if cls is PoseStamped else m.pose.pose
        pose.position.x, pose.position.y = float(x), float(y)
        pose.orientation.z, pose.orientation.w = math.sin(math.radians(yaw_deg) / 2), math.cos(math.radians(yaw_deg) / 2)
        return m

    def teleport(self, x, y, yaw_deg):
        self.pose_pub.publish(self._pose_msg(x, y, yaw_deg))
        ip = self._pose_msg(x, y, yaw_deg, PoseWithCovarianceStamped)
        ip.pose.covariance[0] = ip.pose.covariance[7] = 0.01; ip.pose.covariance[35] = 0.005
        self.init_pub.publish(ip)

    def send_goal(self, x, y, yaw_deg):
        self.goal_pub.publish(self._pose_msg(x, y, yaw_deg))   # bt_navigator listens on /goal_pose

    def clear_costmaps(self):
        for cli in self._clear_cli:
            if cli.service_is_ready():
                cli.call_async(self._ClearSrv.Request())

    def send_obstacle_cmd(self, cmd):
        import json
        m = String(); m.data = json.dumps(cmd); self.obst_pub.publish(m)

    def start_run(self, t=None):
        """Reset all run-level buffers and start scoring (called on a new plan, or by the manual button)."""
        self.goal_t = self.now() if t is None else t; self.run_active = True; self.run_frozen = False
        self.xte.clear(); self.acc.clear(); self.jerk.clear(); self.final_err = None
        self.state_time = {}; self.rotate_count = 0; self.rotate_count_transit = 0
        self.plan_count_run = 0; self.detour_time = 0.0; self.profile_gap_log.clear(); self.reason_log.clear()
        self.dist_since_plan = 0.0
        self.block_events.clear(); self.clear_log.clear()

    def stop_run(self):
        self.run_frozen = True
        self.create_subscription(Twist, '/cmd_vel_nav', self.on_cmd_nav, 10)
        self.create_subscription(Twist, '/cmd_vel', self.on_cmd, 10)
        self.create_subscription(Odometry, '/odom', self.on_odom, 20)
        self.create_subscription(String, '/midpro/ctrl_state', self.on_state, 10)
        self.create_subscription(String, '/midpro/path_mode', self.on_mode, 10)
        self.create_subscription(String, '/midpro/speed_limit', self.on_limit, 10)
        self.create_subscription(String, '/midpro/ctrl_transition', self.on_transition, 10)
        qos = rclpy.qos.QoSProfile(depth=1, durability=rclpy.qos.DurabilityPolicy.TRANSIENT_LOCAL)
        self.create_subscription(Float32MultiArray, '/midpro/speed_profile', self.on_profile, qos)
        self.create_subscription(Path, '/plan', self.on_plan, 10)
        self.create_subscription(PoseStamped, '/sim/ground_truth', self.on_gt, 10)

    def now(self):
        return time.monotonic() - self.t0

    def on_cmd_nav(self, m):
        with self.lock:
            self.cmd_nav.append((self.now(), m.linear.x, m.linear.y, m.angular.z))

    def on_cmd(self, m):
        with self.lock:
            self.cmd.append((self.now(), m.linear.x, m.linear.y, m.angular.z))

    def on_odom(self, m):
        t = self.now()
        vx, vy, wz = m.twist.twist.linear.x, m.twist.twist.linear.y, m.twist.twist.angular.z
        v = math.hypot(vx, vy)
        with self.lock:
            # first-order filtered speed -> acceleration -> jerk (odometry is noisy at 50 Hz)
            if self._t_prev is None:
                self._v_f = v; self._t_prev = t
            else:
                dt = max(t - self._t_prev, 1e-3)
                alpha = min(1.0, dt / 0.15)
                v_f = self._v_f + alpha * (v - self._v_f)
                a = (v_f - self._v_f) / dt
                a_f = self._a_f + alpha * (a - self._a_f)
                self.acc.append((t, a_f))
                self.jerk.append((t, (a_f - self._a_f) / dt))
                self._v_f, self._a_f, self._t_prev = v_f, a_f, t
            self.odom.append((t, v, vx, vy, wz))
            p = m.pose.pose.position
            if self.last_odom is not None:
                self.dist_since_plan += math.hypot(p.x - self.last_odom[0], p.y - self.last_odom[1])
            self.last_odom = (p.x, p.y)
            if self.gt is None:
                q = m.pose.pose.orientation
                self._pose = (p.x, p.y, math.atan2(2*(q.w*q.z+q.x*q.y), 1-2*(q.y*q.y+q.z*q.z)))
            else:
                self._pose = self.gt
            if self.plan_xy and self.run_active and not self.run_frozen:
                self.xte.append((t, self.cross_track(self._pose[0], self._pose[1])))
            if self.profile_s and self.run_active and not self.run_frozen:
                s_now = min(self.dist_since_plan, self.profile_s[-1])
                i = min(range(len(self.profile_s)), key=lambda k: abs(self.profile_s[k] - s_now))
                self.profile_gap_log.append((t, v, self.profile_v[i], self.state))

    def cross_track(self, x, y):
        n = len(self.plan_xy); best = 1e9; bi = self._plan_hint
        lo = max(0, self._plan_hint - 50)
        for i in range(lo, n):
            d = (self.plan_xy[i][0]-x)**2 + (self.plan_xy[i][1]-y)**2
            if d < best:
                best = d; bi = i
            if i > self._plan_hint + 300 and d > best + 4.0:
                break
        self._plan_hint = bi
        return math.sqrt(best)

    def on_plan(self, m):
        with self.lock:
            if not m.poses:
                return
            self.plan_xy = [(p.pose.position.x, p.pose.position.y) for p in m.poses]
            q = m.poses[-1].pose.orientation
            self.plan_goal = (m.poses[-1].pose.position.x, m.poses[-1].pose.position.y, math.atan2(2*(q.w*q.z+q.x*q.y), 1-2*(q.y*q.y+q.z*q.z)))
            self._plan_hint = 0
            if self.run_active and not self.run_frozen:
                self.plan_count_run += 1

    def fetch_params(self, done):
        """Read the FollowPath parameters from controller_server (async; `done(params)` on the ROS thread)."""
        if not self._param_cli.service_is_ready():
            done(dict(self.params)); return
        req = GetParameters.Request(); req.names = rp.PARAM_NAMES
        fut = self._param_cli.call_async(req)

        def cb(f):
            out = {}
            try:
                for name, val in zip(rp.PARAM_NAMES, f.result().values):
                    out[name] = {1: val.bool_value, 2: val.integer_value, 3: val.double_value, 4: val.string_value}.get(val.type)
            except Exception:  # noqa: BLE001
                pass
            with self.lock:
                self.params = {k: v for k, v in out.items() if v is not None}
            done(dict(self.params))
        fut.add_done_callback(cb)

    def on_gt(self, m):
        q = m.pose.orientation
        with self.lock:
            self.gt = (m.pose.position.x, m.pose.position.y, math.atan2(2*(q.w*q.z+q.x*q.y), 1-2*(q.y*q.y+q.z*q.z)))

    def on_state(self, m):
        with self.lock:
            t = self.now()
            if self._state_t0 is not None and self.run_active and not self.run_frozen:
                self.state_time[self.state] = self.state_time.get(self.state, 0.0) + (t - self._state_t0)
            self._state_t0 = t
            self.state = m.data

    def on_mode(self, m):
        with self.lock:
            t = self.now()
            if self._mode_t is not None and self.path_mode.startswith('detour') and self.run_active and not self.run_frozen:
                self.detour_time += t - self._mode_t
            self._mode_t = t
            self.path_mode = m.data

    def on_limit(self, m):
        with self.lock:
            self.limit = m.data
            key = m.data.split('|')[-1].strip().split(' ')[0] if '|' in m.data else m.data.split(' ')[0]
            self.reason_log.append((self.now(), key))

    def on_transition(self, m):
        with self.lock:
            self.transition = m.data
            if 'new plan' in m.data and not self.manual:
                self.start_run()
            if '-> ROTATE_IN_PLACE' in m.data and self.run_active and not self.run_frozen:
                self.rotate_count += 1
                if m.data.startswith('FOLLOW_PATH'):
                    self.rotate_count_transit += 1
            if 'goal reached' in m.data and self.run_active and self.plan_goal and hasattr(self, '_pose'):
                dx = self._pose[0]-self.plan_goal[0]; dy = self._pose[1]-self.plan_goal[1]
                dyaw = (self._pose[2]-self.plan_goal[2]+math.pi) % (2*math.pi) - math.pi
                self.final_err = (math.hypot(dx, dy), abs(math.degrees(dyaw)))
                if not self.manual:
                    self.run_frozen = True

    def on_profile(self, m):
        d = list(m.data)
        with self.lock:
            self.profile_s = d[0::2]; self.profile_v = d[1::2]
            self.profile_t = self.now(); self.dist_since_plan = 0.0


class Dash(QtWidgets.QMainWindow):
    report_ready = QtCore.pyqtSignal(str, str)
    param_result = QtCore.pyqtSignal(str, bool, bool)
    params_ready = QtCore.pyqtSignal(dict)

    def __init__(self, bus, window_s, csv_path=None, dark=True):
        super().__init__()
        self.bus = bus; self.window = window_s
        self.setWindowTitle('midpro speed dashboard')
        self.resize(*getattr(self, '_size', (690, 430)))
        pg.setConfigOptions(antialias=True, background='w', foreground=INK)
        root = QtWidgets.QWidget(); self.setCentralWidget(root)
        lay = QtWidgets.QHBoxLayout(root); lay.setContentsMargins(4, 4, 4, 4); lay.setSpacing(4)
        self.tabs = QtWidgets.QTabWidget(); lay.addWidget(self.tabs, 4)
        left = pg.GraphicsLayoutWidget(); self.tabs.addTab(left, '그래프'); self.gl = left
        # ---- events tab
        ev_w = QtWidgets.QWidget(); ev_l = QtWidgets.QVBoxLayout(ev_w)
        self.ev_table = QtWidgets.QTableWidget(0, 5); self.ev_table.setHorizontalHeaderLabels(['시각', '심각도', '현상', '권장 파라미터 (현재 → 제안)', '이유'])
        hh = self.ev_table.horizontalHeader(); hh.setSectionResizeMode(0, QtWidgets.QHeaderView.ResizeToContents); hh.setSectionResizeMode(1, QtWidgets.QHeaderView.ResizeToContents)
        for c in (2, 3, 4):
            hh.setSectionResizeMode(c, QtWidgets.QHeaderView.Stretch)
        self.ev_table.setWordWrap(True); self.ev_table.verticalHeader().setVisible(False)
        self.ev_table.setEditTriggers(QtWidgets.QAbstractItemView.NoEditTriggers); ev_l.addWidget(self.ev_table)
        ev_row = QtWidgets.QHBoxLayout(); self.ev_count = QtWidgets.QLabel('이벤트 0건'); ev_row.addWidget(self.ev_count); ev_row.addStretch(1)
        b_clear = QtWidgets.QPushButton('지우기'); b_clear.clicked.connect(self.clear_events); ev_row.addWidget(b_clear)
        b_csv = QtWidgets.QPushButton('CSV 저장'); b_csv.clicked.connect(self.save_events); ev_row.addWidget(b_csv); ev_l.addLayout(ev_row)
        self.tabs.addTab(ev_w, '이벤트 (실시간)')
        # ---- obstacle editor tab
        self.obst_tab = ObstacleTab(bus, bus.send_obstacle_cmd, bus.clear_costmaps); self.tabs.addTab(self.obst_tab, '장애물 편집')
        self.rules = LiveRules(); self._last_rules_t = 0.0; self._last_param_t = 0.0
        self.tabs.currentChanged.connect(self.on_tab_changed)
        # right panel in a scroll area: never cut off, however small the window
        right_w = QtWidgets.QWidget(); right = QtWidgets.QVBoxLayout(right_w)
        right.setContentsMargins(2, 2, 2, 2); right.setSpacing(4)
        self.right_scroll = QtWidgets.QScrollArea(); self.right_scroll.setWidgetResizable(True)
        self.right_scroll.setWidget(right_w); self.right_scroll.setMinimumWidth(300)
        self.right_scroll.setHorizontalScrollBarPolicy(QtCore.Qt.ScrollBarAlwaysOff)
        lay.addWidget(self.right_scroll, 1)

        self.p1 = left.addPlot(title='속도 (m/s)'); self.p1.addLegend(offset=(10, 10))
        self.c_nav = self.p1.plot(pen=pg.mkPen(GREY, width=1.2), name='컨트롤러 출력 vx')
        self.c_cmd = self.p1.plot(pen=pg.mkPen(ACCENT, width=1.4, style=QtCore.Qt.DashLine), name='최종 cmd_vel vx')
        self.c_act = self.p1.plot(pen=pg.mkPen(ACCENT, width=2.2), name='실제 |v| (odom)')
        self.p1.showGrid(x=True, y=True, alpha=0.15)
        left.nextRow()
        self.p2 = left.addPlot(title='가속도 (m/s²) / 저크 (m/s³)'); self.p2.addLegend(offset=(10, 10))
        self.c_acc = self.p2.plot(pen=pg.mkPen(ACCENT, width=2), name='a (filtered)')
        self.c_jerk = self.p2.plot(pen=pg.mkPen(GREY, width=1), name='jerk')
        self.p2.showGrid(x=True, y=True, alpha=0.15)
        for y in (0.4, -0.4):
            self.p2.addLine(y=y, pen=pg.mkPen(LINE, style=QtCore.Qt.DashLine))
        left.nextRow()
        self.p3 = left.addPlot(title='조향 (rad/s) / 횡속도 (m/s)'); self.p3.addLegend(offset=(10, 10))
        self.c_wz_cmd = self.p3.plot(pen=pg.mkPen(GREY, width=1.2), name='cmd wz')
        self.c_wz_act = self.p3.plot(pen=pg.mkPen(ACCENT, width=2), name='실제 wz')
        self.c_vy = self.p3.plot(pen=pg.mkPen(ACCENT, width=1.2, style=QtCore.Qt.DotLine), name='cmd vy')
        self.p3.showGrid(x=True, y=True, alpha=0.15)
        left.nextRow()
        self.p4 = left.addPlot(title='계획 속도 프로파일 v(s) — 파란 점: 현재 진행 위치')
        self.c_prof = self.p4.plot(pen=pg.mkPen(ACCENT, width=2))
        self.c_pos = self.p4.plot(symbol='o', symbolSize=10, symbolBrush=ACCENT, pen=None)
        self.p4.setLabel('bottom', '경로 거리 s (m)'); self.p4.showGrid(x=True, y=True, alpha=0.15)
        for p in (self.p1, self.p2, self.p3, self.p4):
            p.enableAutoRange(axis='y', enable=False)
            p.getAxis('left').setWidth(40)                  # same width on every plot -> columns line up
            p.getAxis('left').enableAutoSIPrefix(False)
            for ax in ('left', 'bottom'):
                f = QtGui.QFont(); f.setPointSize(7); p.getAxis(ax).setTickFont(f)
            if getattr(p, 'legend', None) is not None:
                p.legend.setLabelTextSize('7pt')
        self.p1.setXLink(self.p2); self.p2.setXLink(self.p3)
        self.titles = {self.p1: '속도 (m/s) · x축 시간 s', self.p2: '가속도 / 저크', self.p3: '조향 wz / 횡속도 vy', self.p4: '계획 속도 프로파일 v(s) · 점 = 현재'}
        self.xlabels = {self.p1: '', self.p2: '', self.p3: '', self.p4: '경로 거리 s (m)'}

        self.labels = {}
        box = QtWidgets.QGroupBox('상태'); form = QtWidgets.QFormLayout(box)
        for key, name in [('state', 'FSM 상태'), ('mode', '경로 상태 / 구동'), ('limit', '속도 제한 사유'), ('transition', '마지막 전이'), ('entry', '목표 진입'), ('guard', '안전 가드'), ('loc', '측위 오차')]:
            lb = QtWidgets.QLabel('-'); lb.setWordWrap(True); lb.setStyleSheet(f'color:{INK}; font-size:13px'); form.addRow(name, lb); self.labels[key] = lb
        right.addWidget(box)
        box2 = QtWidgets.QGroupBox('수치'); form2 = QtWidgets.QFormLayout(box2)
        for key, name in [('v', '실제 속도'), ('vcmd', '명령 속도'), ('a', '가속도'), ('amax', '|a| 최대 (5 s)'), ('arms', '가속 RMS (10 s)'),
                          ('jrms', '저크 RMS (10 s)'), ('rev', '조향 반전 /분'), ('dist', '경로 진행 거리'), ('elapsed', '목표 후 경과'), ('prof', '프로파일 상한(현재)')]:
            lb = QtWidgets.QLabel('-'); lb.setStyleSheet(f'color:{ACCENT}; font-size:12px; font-weight:600'); form2.addRow(name, lb); self.labels[key] = lb
        right.addWidget(box2)
        for key, name in [('v', '실제 속도'), ('vcmd', '명령 속도'), ('a', '가속도'), ('amax', '|a| 최대 (5 s)'), ('arms', '가속 RMS (10 s)'),
                          ('jrms', '저크 RMS (10 s)'), ('rev', '조향 반전 /분'), ('dist', '경로 진행 거리'), ('elapsed', '목표 후 경과'), ('prof', '프로파일 상한')]:
            self.obst_tab.add_metric(key, name, 'font-size:12px; font-weight:600')
        score = QtWidgets.QGroupBox('주행 점수표'); sl = QtWidgets.QGridLayout(score)
        self.score_items = [('track', '경로 추종'), ('accel', '가감속 부드러움'), ('jerk', '울컥임 없음'),
                            ('steer', '조향 안정'), ('speed', '속도 효율'), ('stop', '정지 정밀'), ('obstacle', '장애물 대응')]
        self.bars = {}; self.score_lbls = {}
        for r, (key, name) in enumerate(self.score_items):
            sl.addWidget(QtWidgets.QLabel(name), r, 0)
            bar = QtWidgets.QProgressBar(); bar.setRange(0, 100); bar.setTextVisible(False); bar.setFixedHeight(10); sl.addWidget(bar, r, 1); self.bars[key] = bar
            lb = QtWidgets.QLabel('-'); lb.setAlignment(QtCore.Qt.AlignRight); sl.addWidget(lb, r, 2); self.score_lbls[key] = lb
        self.total_lbl = QtWidgets.QLabel('-'); self.total_lbl.setStyleSheet('font-size:16px;font-weight:700'); sl.addWidget(self.total_lbl, len(self.score_items), 0, 1, 2)
        self.grade_lbl = QtWidgets.QLabel('-'); self.grade_lbl.setStyleSheet('font-size:16px;font-weight:700'); self.grade_lbl.setAlignment(QtCore.Qt.AlignRight); sl.addWidget(self.grade_lbl, len(self.score_items), 2)
        self.verdict = QtWidgets.QLabel('목표를 주면 집계가 시작됩니다.'); self.verdict.setWordWrap(True); sl.addWidget(self.verdict, len(self.score_items)+1, 0, 1, 3)
        rowc = QtWidgets.QHBoxLayout()
        self.btn_mode = QtWidgets.QPushButton('자동 집계'); self.btn_mode.setToolTip('자동: 목표마다 새로 집계 / 수동: 시작·정지 버튼으로 구간 지정'); self.btn_mode.setCheckable(True); self.btn_mode.toggled.connect(self.toggle_mode); rowc.addWidget(self.btn_mode)
        self.btn_run = QtWidgets.QPushButton('집계 시작'); self.btn_run.setEnabled(False); self.btn_run.clicked.connect(self.toggle_run); rowc.addWidget(self.btn_run)
        sl.addLayout(rowc, len(self.score_items)+2, 0, 1, 3)
        self.run_status = QtWidgets.QLabel('자동 · 대기 (목표를 주면 시작)'); self.run_status.setStyleSheet('font-size:12px'); sl.addWidget(self.run_status, len(self.score_items)+3, 0, 1, 3)
        right.addWidget(score)
        ctl = QtWidgets.QGroupBox('제어'); cl = QtWidgets.QGridLayout(ctl); cl.setSpacing(3); self._ctl_n = 0
        self.btn_pause = QtWidgets.QPushButton('일시정지'); self.btn_pause.setCheckable(True); self._grid_add(cl, self.btn_pause)
        self.btn_reset = QtWidgets.QPushButton('초기화'); self.btn_reset.clicked.connect(self.reset_stats); self._grid_add(cl, self.btn_reset)
        self.btn_report = QtWidgets.QPushButton('리포트'); self.btn_report.clicked.connect(self.make_report); self._grid_add(cl, self.btn_report)
        self.report_ready.connect(self.show_report)
        self.param_result.connect(self.on_param_result)
        self.params_ready.connect(self.sync_options)
        self.btn_dark = QtWidgets.QPushButton('밤'); self.btn_dark.setCheckable(True); self.btn_dark.toggled.connect(self.set_dark); self._grid_add(cl, self.btn_dark)
        self.spin = QtWidgets.QSpinBox(); self.spin.setRange(5, 300); self.spin.setValue(window_s); self.spin.setSuffix(' s'); self.spin.valueChanged.connect(self.set_window); self._grid_add(cl, self.spin)
        opt = QtWidgets.QGroupBox('주행 옵션 (실시간)'); ol = QtWidgets.QVBoxLayout(opt)
        self.cb_switch = QtWidgets.QCheckBox('경로 스위칭'); self.cb_switch.setToolTip('같은 목표에서 더 짧은 경로가 나오면 갈아탐 (장애물이 사라지면 지름길 복귀)')
        self.cb_holo = QtWidgets.QCheckBox('직선도 홀로노믹'); self.cb_holo.setToolTip('직선 주행까지 방향 유지 크랩(midpro 원래 동작). 꺼져 있으면 직선은 차체 정렬 주행')
        self.cb_traj = QtWidgets.QCheckBox('궤적 안전 필터'); self.cb_traj.setToolTip('명령을 1 s 굴려 차체(특히 회전 시 뒤쪽)가 닿으면 보정. 경로를 잘 따라갈 땐 개입 안 함')
        self.cb_orth = QtWidgets.QCheckBox('직각 회피 (옆 → 직진)'); self.cb_orth.setToolTip('코앞에 장애물·목표는 그 너머일 때: 방향 유지한 채 옆으로 필요한 만큼 이동, 정지, 직진')
        self.cb_side = QtWidgets.QCheckBox('side/front 회피 (midpro)'); self.cb_side.setToolTip('직선이 막히면 midpro side_front/front_side: 방향 유지, 옆(또는 앞) 지점까지 크랩 후 목표로')
        self.cb_stage = QtWidgets.QCheckBox('목표 전 정렬 (주차)'); self.cb_stage.setToolTip('목표점에서 돌면 닿을 때: 공간 있는 곳에서 미리 목표 방향으로 돌고, 방향 유지한 채 밀고 들어감')
        self.cb_cond = QtWidgets.QCheckBox('경로 정리기'); self.cb_cond.setToolTip('플래너 경로를 긴 차체에 맞게 정리: 출발 평행 이동, 통로 직선화, 경로 밖 준비 지점')
        for cb, name in ((self.cb_switch, 'plan_switching'), (self.cb_holo, 'transit_holonomic'), (self.cb_traj, 'traj_check_enabled'), (self.cb_orth, 'orthogonal_avoid'), (self.cb_side, 'side_front_enabled'), (self.cb_stage, 'goal_staging'), (self.cb_cond, 'path_conditioner_enabled')):
            cb.setChecked(False)
            cb.toggled.connect(lambda v, n=name, w=cb: self.toggle_param(n, v, w))
            ol.addWidget(cb)
        self.opt_status = QtWidgets.QLabel('기본: 스위칭·홀로노믹 꺼짐, 필터·직각 회피 켜짐'); self.opt_status.setStyleSheet('font-size:12px'); ol.addWidget(self.opt_status)
        right.addWidget(opt)
        self._opt_synced = False
        for w in (ctl, opt):
            right.removeWidget(w)
        right.insertWidget(0, ctl); right.insertWidget(1, opt)
        right.addStretch(1)
        # let every widget in the right panel shrink to the panel width (no clipped second column)
        shrink = QtWidgets.QSizePolicy.Ignored
        for wcls in (QtWidgets.QPushButton, QtWidgets.QProgressBar, QtWidgets.QSpinBox):
            for wdg in right_w.findChildren(wcls):
                wdg.setMinimumWidth(0); wdg.setSizePolicy(shrink, wdg.sizePolicy().verticalPolicy())
        for lbl in right_w.findChildren(QtWidgets.QLabel):
            lbl.setMinimumWidth(0)
        right_w.setMinimumWidth(0)
        self.tabs.setMinimumWidth(320)
        self.setStyleSheet(f'QGroupBox{{font-weight:600;color:{INK};border:1px solid {LINE};border-radius:4px;margin-top:16px;padding:8px 4px 4px 4px}} '
                           f'QGroupBox::title{{subcontrol-origin:margin;subcontrol-position:top left;left:6px;top:1px;padding:0 3px}} '
                           f'QLabel{{color:{INK}}} QPushButton{{padding:6px}}')

        self.csv = None
        if csv_path:
            self.csv = csv.writer(open(csv_path, 'w', newline=''))
            self.csv.writerow(['t', 'v_act', 'vx_cmd', 'vy_cmd', 'wz_cmd', 'a', 'state', 'limit'])
        self.btn_dark.setChecked(dark); self.set_dark(dark)
        # re-apply once the window is shown: widgets inside the scroll area are polished only then
        QtCore.QTimer.singleShot(0, lambda: self.set_dark(self.btn_dark.isChecked()))
        self.timer = QtCore.QTimer(); self.timer.timeout.connect(self.refresh); self.timer.start(50)

    def on_tab_changed(self, idx):
        # obstacle editor: hide the wide right panel, numbers are shown under the obstacle list instead
        on_obst = self.tabs.widget(idx) is self.obst_tab
        self.right_scroll.setVisible(not on_obst)

    @staticmethod
    def _fit_y(plot, series, lo_min, hi_min):
        """Fixed minimum y range (expands only when data leave it). Auto-range zoomed into 1e-9 / 1e-36 noise,
        which made the tick labels long and the stacked plots misaligned."""
        lo, hi = lo_min, hi_min
        for ys in series:
            if len(ys):
                lo = min(lo, min(ys)); hi = max(hi, max(ys))
        if lo < lo_min or hi > hi_min:
            pad = 0.05 * (hi - lo)
            lo, hi = lo - (pad if lo < lo_min else 0.0), hi + (pad if hi > hi_min else 0.0)
        vb = plot.getViewBox()
        if getattr(plot, '_yr', None) != (round(lo, 3), round(hi, 3)):
            vb.setYRange(lo, hi, padding=0)
            plot._yr = (round(lo, 3), round(hi, 3))
        return lo, hi

    @staticmethod
    def _grid_add(layout, widget):
        n = layout.count()
        layout.addWidget(widget, n // 2, n % 2)

    def set_window(self, v):
        self.window = v

    def toggle_param(self, name, value, widget):
        if getattr(self, '_syncing', False):
            return
        self.opt_status.setText(f'{name} = {value} 적용 중…')
        self.bus.set_bool_param(name, value, lambda ok, n=name, v=value: self.param_result.emit(n, v, ok))

    def on_param_result(self, name, value, ok):
        self.opt_status.setText(f"{name} = {value} {'적용됨' if ok else '실패 (controller_server 응답 없음)'}")

    def sync_options(self, params):
        if self._opt_synced or not params:
            return
        self._syncing = True
        if 'FollowPath.plan_switching' in params:
            self.cb_switch.setChecked(bool(params['FollowPath.plan_switching']))
        if 'FollowPath.transit_holonomic' in params:
            self.cb_holo.setChecked(bool(params['FollowPath.transit_holonomic']))
        if 'FollowPath.traj_check_enabled' in params:
            self.cb_traj.setChecked(bool(params['FollowPath.traj_check_enabled']))
        if 'FollowPath.orthogonal_avoid' in params:
            self.cb_orth.setChecked(bool(params['FollowPath.orthogonal_avoid']))
        if 'FollowPath.side_front_enabled' in params:
            self.cb_side.setChecked(bool(params['FollowPath.side_front_enabled']))
        if 'FollowPath.goal_staging' in params:
            self.cb_stage.setChecked(bool(params['FollowPath.goal_staging']))
        if 'FollowPath.path_conditioner_enabled' in params:
            self.cb_cond.setChecked(bool(params['FollowPath.path_conditioner_enabled']))
        self._syncing = False; self._opt_synced = True
        self.opt_status.setText('현재 값과 동기화됨')

    def add_event(self, ev):
        self.bus.events_log.append(ev)
        r = self.ev_table.rowCount(); self.ev_table.insertRow(r)
        sev = {3: '높음', 2: '중간', 1: '낮음'}[ev.sev]
        cur = '' if ev.cur is None else (f'{ev.cur:.3g}' if isinstance(ev.cur, float) else str(ev.cur))
        new = '' if ev.new is None else (f'{ev.new:.3g}' if isinstance(ev.new, float) else str(ev.new))
        param = f'{ev.param}: {cur} → {new}' if ev.param else '-'
        for c, txt in enumerate([ev.wall, sev, ev.msg, param, ev.why]):
            it = QtWidgets.QTableWidgetItem(txt)
            if c == 1:
                it.setForeground(pg.mkColor({3: '#e07a6a', 2: '#e0b35a', 1: '#9aa3ae'}[ev.sev]))
            self.ev_table.setItem(r, c, it)
        self.ev_table.resizeRowsToContents(); self.ev_table.scrollToBottom(); self.ev_count.setText(f'이벤트 {self.ev_table.rowCount()}건')

    def clear_events(self):
        self.ev_table.setRowCount(0); self.bus.events_log.clear(); self.rules.events.clear(); self.ev_count.setText('이벤트 0건')

    def save_events(self):
        out_dir = os.path.expanduser('~/midpro_reports'); os.makedirs(out_dir, exist_ok=True)
        path = os.path.join(out_dir, time.strftime('events_%Y%m%d_%H%M%S.csv'))
        with open(path, 'w', newline='', encoding='utf-8') as f:
            w = csv.writer(f); w.writerow(['time', 'severity', 'event', 'param', 'current', 'suggested', 'why'])
            for ev in self.bus.events_log:
                w.writerow([ev.wall, ev.sev, ev.msg, ev.param or '', ev.cur, ev.new, ev.why])
        self.ev_count.setText(f'이벤트 {self.ev_table.rowCount()}건 · 저장: {path}')

    def toggle_mode(self, manual):
        with self.bus.lock:
            self.bus.manual = manual
        self.btn_mode.setText('수동 집계' if manual else '자동 집계')
        self.btn_run.setEnabled(manual)
        if not manual:
            self.btn_run.setText('집계 시작')

    def toggle_run(self):
        b = self.bus
        with b.lock:
            running = b.run_active and not b.run_frozen
            if running:
                b.stop_run()
            else:
                b.start_run()
        self.btn_run.setText('집계 정지' if not running else '집계 시작')

    def make_report(self):
        self.btn_report.setEnabled(False); self.btn_report.setText('생성 중…')
        self.bus.fetch_params(self._build_report)

    def _build_report(self, params):
        b = self.bus
        with b.lock:
            if b.run_active:
                self.update_scores(b, b.now())          # include events that arrived after the last screen refresh
            st = rp.collect_stats(b)
            scores = self.last_scores if hasattr(self, 'last_scores') else {}
        history = rp.load_history()
        target = getattr(b, 'target', 'B')
        findings = rp.analyze(st, params, scores, history, target)
        history = rp.record_run(history, st, scores, findings, params)
        conv = rp.convergence(history, target)
        rp.save_history(history)
        html = rp.render_html(st, findings, params, scores, self.score_items, history, target, conv, events=list(b.events_log))
        out_dir = os.path.expanduser('~/midpro_reports'); os.makedirs(out_dir, exist_ok=True)
        path = os.path.join(out_dir, time.strftime('report_%Y%m%d_%H%M%S.html'))
        with open(path, 'w', encoding='utf-8') as f:
            f.write(html)
        self.report_ready.emit(html, path)

    def show_report(self, html, path):
        self.btn_report.setEnabled(True); self.btn_report.setText('리포트')
        dlg = QtWidgets.QDialog(self); dlg.setWindowTitle(f'튜닝 리포트 — {path}'); dlg.resize(980, 820)
        lay = QtWidgets.QVBoxLayout(dlg)
        tb = QtWidgets.QTextBrowser(); tb.setOpenExternalLinks(True); tb.setHtml(html); lay.addWidget(tb)
        row = QtWidgets.QHBoxLayout(); lb = QtWidgets.QLabel(f'저장됨: {path}'); row.addWidget(lb); row.addStretch(1)
        btn = QtWidgets.QPushButton('닫기'); btn.clicked.connect(dlg.accept); row.addWidget(btn); lay.addLayout(row)
        dlg.show()

    def reset_stats(self):
        with self.bus.lock:
            self.bus.acc.clear(); self.bus.jerk.clear()

    def set_dark(self, on):
        bg = '#14171c' if on else 'w'; fg = '#e6e9ee' if on else INK; accent = '#7fb0ec' if on else ACCENT
        grey = '#6b7480' if on else GREY
        self.c_nav.setPen(pg.mkPen(grey, width=1.2)); self.c_jerk.setPen(pg.mkPen(grey, width=1)); self.c_wz_cmd.setPen(pg.mkPen(grey, width=1.2))
        for c in (self.c_cmd, self.c_act, self.c_acc, self.c_wz_act, self.c_vy, self.c_prof):
            pen = c.opts['pen']; c.setPen(pg.mkPen(accent, width=pen.widthF() if hasattr(pen, 'widthF') else 1.5, style=pen.style() if hasattr(pen, 'style') else QtCore.Qt.SolidLine))
        self.c_pos.setSymbolBrush(accent)
        pg.setConfigOptions(background=bg, foreground=fg)
        self.gl.setBackground(bg)
        for p in (self.p1, self.p2, self.p3, self.p4):
            p.getViewBox().setBackgroundColor(bg)
            for ax in ('left', 'bottom'):
                p.getAxis(ax).setPen(pg.mkPen(fg)); p.getAxis(ax).setTextPen(pg.mkPen(fg))
            p.setTitle(self.titles[p], color=fg, size='9pt')
            p.setLabel('bottom', self.xlabels[p], color=fg)
            if getattr(p, 'legend', None) is not None:
                p.legend.setLabelTextColor(fg)
                for _sample, label in p.legend.items:
                    label.setText(label.text, color=fg)
        for lb in self.obst_tab.metrics.values():
            lb.setStyleSheet(f'color:{accent}; font-size:12px; font-weight:600')
        for key, lb in self.labels.items():
            if key in ('state', 'mode', 'limit', 'transition', 'entry', 'guard', 'loc'):
                lb.setStyleSheet(f'color:{fg}; font-size:13px')
            else:
                lb.setStyleSheet(f'color:{accent}; font-size:12px; font-weight:600')
        self.setStyleSheet(f'QMainWindow,QWidget{{background:{bg};color:{fg}}} QGroupBox{{font-weight:600;color:{fg};border:1px solid {"#2b313a" if on else LINE};border-radius:4px;margin-top:16px;padding:8px 4px 4px 4px}} QGroupBox::title{{subcontrol-origin:margin;subcontrol-position:top left;left:6px;top:1px;padding:0 3px}} QPushButton{{padding:3px 6px;color:{fg}}} QSpinBox,QDoubleSpinBox,QComboBox{{color:{fg};background:{"#1b2027" if on else "#f4f5f7"}}} QTableWidget,QListWidget{{background:{"#1b2027" if on else "#fff"};color:{fg};gridline-color:{"#2b313a" if on else LINE}}} QListWidget::item:selected{{background:{accent};color:{bg}}} QHeaderView::section{{background:{"#1f242b" if on else "#f4f5f7"};color:{fg};border:0;padding:4px}} QCheckBox{{color:{fg}}} QCheckBox::indicator{{width:13px;height:13px;border:1px solid {fg};background:{bg}}} QCheckBox::indicator:checked{{background:{accent};border:1px solid {accent}}} QTabBar::tab{{padding:4px 10px;color:{fg};background:{"#1b2027" if on else "#f4f5f7"}}} QTabBar::tab:selected{{border-bottom:2px solid {accent}}} QProgressBar{{border:1px solid {LINE if not on else "#2b313a"};background:{"#1b2027" if on else "#f4f5f7"}}} QProgressBar::chunk{{background:{accent}}}')
        self.total_lbl.setStyleSheet(f'font-size:16px;font-weight:700;color:{accent}'); self.grade_lbl.setStyleSheet(f'font-size:16px;font-weight:700;color:{accent}')
        self.btn_dark.setText('아침' if on else '밤')

    @staticmethod
    def lin_score(x, good, bad):
        if x is None:
            return None
        if good < bad:
            return 100.0 if x <= good else (0.0 if x >= bad else 100.0 * (bad - x) / (bad - good))
        return 100.0 if x >= good else (0.0 if x <= bad else 100.0 * (x - bad) / (good - bad))

    def update_scores(self, b, now):
        xs = [r[1] for r in b.xte]
        xte_rms = math.sqrt(sum(x*x for x in xs)/len(xs)) if xs else None
        ar = [r[1] for r in b.acc if b.goal_t is None or r[0] >= b.goal_t]
        acc_rms = math.sqrt(sum(x*x for x in ar)/len(ar)) if ar else None
        jr = [r[1] for r in b.jerk if b.goal_t is None or r[0] >= b.goal_t]
        jerk_rms = math.sqrt(sum(x*x for x in jr)/len(jr)) if jr else None
        rev = 0; prev = 0.0; t_first = None
        for r in b.cmd:
            if b.goal_t is not None and r[0] < b.goal_t:
                continue
            t_first = r[0] if t_first is None else t_first
            if r[3] * prev < 0 and abs(r[3]) > 0.05:
                rev += 1
            if abs(r[3]) > 0.05:
                prev = r[3]
        span = max((now - t_first) / 60.0, 0.25) if t_first is not None else None
        rev_pm = rev / span if span else None
        vs = [r[1] for r in b.odom if (b.goal_t is None or r[0] >= b.goal_t) and r[1] > 0.05]
        v_ratio = (sum(vs)/len(vs))/b.vmax if vs else None
        stop = None
        if b.final_err is not None:
            stop = 0.5*self.lin_score(b.final_err[0], 0.02, 0.10) + 0.5*self.lin_score(b.final_err[1], 1.4, 5.0)
        from midpro_dash import metrics as MM
        bs = MM.block_stats([e for e in b.block_events if b.goal_t is None or e[0] >= b.goal_t],
                            [c for c in b.clear_log if b.goal_t is None or c[0] >= b.goal_t])
        obst = MM.obstacle_score(bs)
        sc = {'obstacle': obst, 'track': self.lin_score(xte_rms, 0.05, 0.30), 'accel': self.lin_score(acc_rms, 0.10, 0.50), 'jerk': self.lin_score(jerk_rms, 0.5, 3.0),
              'steer': self.lin_score(rev_pm, 2.0, 20.0), 'speed': self.lin_score(v_ratio, 0.90, 0.30), 'stop': stop}
        raw = {'track': f'{xte_rms*100:.1f} cm' if xte_rms is not None else '-', 'accel': f'{acc_rms:.2f} m/s²' if acc_rms is not None else '-',
               'jerk': f'{jerk_rms:.2f} m/s³' if jerk_rms is not None else '-', 'steer': f'{rev_pm:.1f} /분' if rev_pm is not None else '-',
               'obstacle': (f"감지 {bs['detect']} · 우회 {len(bs['static_at']) + len(bs['persistent_age'])}" if bs['detect'] else '장애물 없음'),
               'speed': f'{v_ratio*100:.0f} %' if v_ratio is not None else '-', 'stop': f'{b.final_err[0]*100:.1f} cm · {b.final_err[1]:.1f}°' if b.final_err else '주행 중'}
        for key, _ in self.score_items:
            v = sc[key]
            self.bars[key].setValue(int(v) if v is not None else 0); self.score_lbls[key].setText(raw[key])
        self.last_scores = dict(sc)
        vals = [v for v in sc.values() if v is not None]
        if vals:
            total = sum(vals)/len(vals)
            grade = 'A' if total >= 85 else 'B' if total >= 70 else 'C' if total >= 50 else 'D'
            self.total_lbl.setText(f'{total:.0f} 점'); self.grade_lbl.setText(grade)
            worst = min((k for k in sc if sc[k] is not None), key=lambda k: sc[k])
            names = dict(self.score_items)
            tips = {'track': '경로에서 벗어납니다. 조향 게인(k_prop)이나 기준점 거리를 조정하세요.',
                    'accel': '가속·감속이 거칩니다. 감속 주체가 겹치는지 속도 제한 사유를 확인하세요.',
                    'jerk': '명령이 자주 바뀝니다. 프로파일·거버너 문턱값을 확인하세요.',
                    'steer': '핸들을 자주 되돌립니다. 게인을 낮추거나 기준점을 멀리 두세요.',
                    'speed': '속도를 충분히 내지 못합니다. 어떤 제한이 걸리는지 확인하세요.',
                    'stop': '도착 위치·방향 오차가 큽니다. 접근 구간과 허용 오차를 확인하세요.',
                    'obstacle': '장애물 판단·대기·근접에 감점이 있습니다. 이벤트 탭과 리포트의 장애물 대응 절을 확인하세요.'}
            status = '도착 완료 · 점수 확정' if b.run_frozen else '주행 중 · 집계 갱신 중'
            self.verdict.setText(f"{status}. 가장 약한 항목: {names.get(worst, worst)} — {tips.get(worst, '')}" if sc[worst] < 85 else f'{status}. 모든 항목이 양호합니다.')

    @staticmethod
    def series(dq, idx, t_from):
        xs, ys = [], []
        for row in dq:
            if row[0] >= t_from:
                xs.append(row[0]); ys.append(row[idx])
        return xs, ys

    def refresh(self):
        if self.btn_pause.isChecked():
            return
        b = self.bus
        if time.monotonic() - self._last_param_t > 5.0:
            self._last_param_t = time.monotonic(); b.fetch_params(lambda p: self.params_ready.emit(p))
        with b.lock:
            now = b.now(); t_from = now - self.window
            vmax = b.vmax
            xs, y_nav = self.series(b.cmd_nav, 1, t_from); self.c_nav.setData(xs, y_nav)
            xs, y_cmd = self.series(b.cmd, 1, t_from); self.c_cmd.setData(xs, y_cmd)
            xs, y_act = self.series(b.odom, 1, t_from); self.c_act.setData(xs, y_act)
            self._fit_y(self.p1, (y_nav, y_cmd, y_act), 0.0, vmax + 0.1)
            xa, ya = self.series(b.acc, 1, t_from); self.c_acc.setData(xa, ya)
            lo_a, hi_a = self._fit_y(self.p2, (ya,), -0.6, 0.6)
            xj, yj = self.series(b.jerk, 1, t_from)
            self.c_jerk.setData(xj, [min(max(j, lo_a), hi_a) for j in yj])      # jerk clipped to the accel view
            xs, y_wc = self.series(b.cmd, 3, t_from); self.c_wz_cmd.setData(xs, y_wc)
            xw, y_wa = self.series(b.odom, 4, t_from); self.c_wz_act.setData(xw, y_wa)
            xs, y_vy = self.series(b.cmd, 2, t_from); self.c_vy.setData(xs, y_vy)
            self._fit_y(self.p3, (y_wc, y_wa, y_vy), -0.3, 0.3)
            if b.profile_s:
                self.c_prof.setData(b.profile_s, b.profile_v)
                self._fit_y(self.p4, (b.profile_v,), 0.0, vmax + 0.1)
                s_now = min(b.dist_since_plan, b.profile_s[-1])
                # nearest profile sample
                i = min(range(len(b.profile_s)), key=lambda k: abs(b.profile_s[k] - s_now))
                self.c_pos.setData([b.profile_s[i]], [b.profile_v[i]])
                self.labels['prof'].setText(f'{b.profile_v[i]:.2f} m/s @ {b.profile_s[i]:.1f} m')
            v = b.odom[-1][1] if b.odom else 0.0
            vcmd = math.hypot(b.cmd[-1][1], b.cmd[-1][2]) if b.cmd else 0.0
            a = b.acc[-1][1] if b.acc else 0.0
            a5 = [abs(r[1]) for r in b.acc if r[0] >= now - 5]; a10 = [r[1] for r in b.acc if r[0] >= now - 10]; j10 = [r[1] for r in b.jerk if r[0] >= now - 10]
            rev = 0; prev = 0.0
            for r in b.cmd:
                if r[0] >= now - 60:
                    if r[3] * prev < 0 and abs(r[3]) > 0.05:
                        rev += 1
                    if abs(r[3]) > 0.05:
                        prev = r[3]
            self.labels['state'].setText(b.state); self.labels['mode'].setText(b.path_mode); self.labels['entry'].setText(b.entry); self.labels['guard'].setText(b.guard_txt)
            le = b.localization_error()
            self.labels['loc'].setText('gt 모드 (오차 없음)' if le is None else
                                       (f'{le[0]*100:.0f} cm · {le[1]:.1f}°' + ('  ← 위치 틀어짐' if le[0] > 0.2 or le[1] > 5 else '')))
            self.labels['limit'].setText(b.limit); self.labels['transition'].setText(b.transition)
            self.labels['v'].setText(f'{v:.2f} m/s'); self.labels['vcmd'].setText(f'{vcmd:.2f} m/s'); self.labels['a'].setText(f'{a:+.2f} m/s²')
            self.labels['amax'].setText(f'{max(a5) if a5 else 0:.2f} m/s²')
            self.labels['arms'].setText(f'{math.sqrt(sum(x*x for x in a10)/len(a10)) if a10 else 0:.3f} m/s²')
            self.labels['jrms'].setText(f'{math.sqrt(sum(x*x for x in j10)/len(j10)) if j10 else 0:.2f} m/s³')
            self.labels['rev'].setText(f'{rev}')
            self.labels['dist'].setText(f'{b.dist_since_plan:.1f} m')
            self.labels['elapsed'].setText(f'{(now - b.goal_t):.1f} s' if b.goal_t else '-')
            for key, lb in self.obst_tab.metrics.items():
                if key in self.labels:
                    lb.setText(self.labels[key].text())
            if b.run_active:
                self.update_scores(b, now)
            if now - self._last_rules_t >= 0.5:
                self._last_rules_t = now
                for ev in self.rules.evaluate(b, now, b.params):
                    self.add_event(ev)
            if b.map_dirty:
                b.map_dirty = False; mm = b.map_msg
                import numpy as np
                grid = np.array(mm.data, dtype=np.int16).reshape(mm.info.height, mm.info.width)
                self.obst_tab.set_map(grid, mm.info.resolution, mm.info.origin.position.x, mm.info.origin.position.y)
            if b.obstacles_dirty:
                b.obstacles_dirty = False; self.obst_tab.update_obstacles(b.obstacles)
            if hasattr(b, '_pose'):
                self.obst_tab.update_robot(*b._pose)
            mode = '수동' if b.manual else '자동'
            if not b.run_active:
                stat = f'{mode} · 대기 (' + ('집계 시작을 누르세요' if b.manual else '목표를 주면 시작') + ')'
            elif b.run_frozen:
                stat = f'{mode} · 고정됨 ({(now - b.goal_t):.1f} s 주행, 리포트 생성 가능)'
            else:
                stat = f'{mode} · 집계 중 {(now - b.goal_t):.1f} s'
            self.run_status.setText(stat)
            if self.csv and b.cmd and b.odom:
                self.csv.writerow([f'{now:.3f}', f'{v:.3f}', f'{b.cmd[-1][1]:.3f}', f'{b.cmd[-1][2]:.3f}', f'{b.cmd[-1][3]:.3f}', f'{a:.3f}', b.state, b.limit])


def main():
    ap = argparse.ArgumentParser(); ap.add_argument('--window', type=int, default=30); ap.add_argument('--csv', default=None)
    ap.add_argument('--light', action='store_true', help='light background (default: dark)')
    ap.add_argument('--vmax', type=float, default=0.6, help='max_linear_vel used for the speed score')
    ap.add_argument('--size', default='690x430', help='window size WxH (default: a quarter of the old 1380x860)')
    ap.add_argument('--target', default='90', help="acceptance target: a number (e.g. 95) or a grade letter A/B/C; tuning stops when reached")
    args, ros_args = ap.parse_known_args()
    rclpy.init(args=ros_args)
    bus = Bus(args.window); bus.vmax = args.vmax; bus.target = args.target
    th = threading.Thread(target=rclpy.spin, args=(bus,), daemon=True); th.start()
    app = QtWidgets.QApplication(sys.argv)
    f = app.font(); f.setPointSize(9); app.setFont(f)
    try:
        Dash._size = tuple(int(v) for v in args.size.lower().split('x'))
    except Exception:  # noqa: BLE001
        Dash._size = (690, 430)
    w = Dash(bus, args.window, args.csv, dark=not args.light); w.show()
    code = app.exec_()
    rclpy.shutdown(); sys.exit(code)


if __name__ == '__main__':
    main()

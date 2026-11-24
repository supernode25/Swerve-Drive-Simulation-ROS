#!/usr/bin/env python3
"""regression.py — run the scenario file against a running sim_nav launch and score every run.

    ros2 run midpro_dash run_regression --label v028_narrow              # default scenario file, 3 repeats
    ros2 run midpro_dash run_regression --label quick --repeats 1 --only straight_20m,gap_w10

Per run: teleport (/sim/set_pose + /initialpose), clear obstacles and costmaps, inject the scenario obstacles
(/sim/obstacle_cmd), send NavigateToPose, record until result or timeout, compute metrics (metrics.py).
Output: ~/midpro_reports/regression/<label>_<time>/{runs.csv, summary.json, summary.md}
"""
import argparse
import csv
import json
import math
import os
import threading
import time

import rclpy
import yaml
from rclpy.action import ActionClient
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile

from action_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped, Twist
from nav2_msgs.action import NavigateToPose
from nav2_msgs.srv import ClearEntireCostmap
from nav_msgs.msg import Odometry, Path
from sensor_msgs.msg import LaserScan
from rclpy.qos import qos_profile_sensor_data
from std_msgs.msg import String

from midpro_dash import metrics as M

STATUS = {GoalStatus.STATUS_SUCCEEDED: 'SUCCEEDED', GoalStatus.STATUS_ABORTED: 'ABORTED',
          GoalStatus.STATUS_CANCELED: 'CANCELED'}


def yaw_q(yaw):
    return (0.0, 0.0, math.sin(yaw / 2.0), math.cos(yaw / 2.0))


class Recorder(Node):
    def __init__(self):
        super().__init__('midpro_regression')
        self.lock = threading.Lock()
        self.t0 = time.monotonic()
        self.recording = False
        self.reset_buffers()
        self.create_subscription(Odometry, '/odom', self.on_odom, 20)
        self.create_subscription(Twist, '/cmd_vel', lambda m: self._push('cmd', (self.now(), m.linear.x, m.linear.y, m.angular.z)), 20)
        self.create_subscription(Twist, '/cmd_vel_nav', lambda m: self._push('cmd_nav', (self.now(), m.linear.x, m.linear.y, m.angular.z)), 20)
        self.create_subscription(PoseStamped, '/sim/ground_truth', self.on_gt, 20)
        self.create_subscription(Path, '/plan', self.on_plan, 5)
        self.create_subscription(String, '/midpro/ctrl_transition', lambda m: self._push('trans', (self.now(), m.data)), 20)
        self.create_subscription(String, '/midpro/speed_limit', lambda m: self._push('reasons', (self.now(), m.data)), 50)
        self.create_subscription(String, '/midpro/block_event', self.on_block, 20)
        self.pose_pub = self.create_publisher(PoseStamped, '/sim/set_pose', 1)
        self.init_pub = self.create_publisher(PoseWithCovarianceStamped, '/initialpose', 1)
        self.obst_pub = self.create_publisher(String, '/sim/obstacle_cmd', 10)
        self.nav = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        self.clear_g = self.create_client(ClearEntireCostmap, '/global_costmap/clear_entirely_global_costmap')
        self.clear_l = self.create_client(ClearEntireCostmap, '/local_costmap/clear_entirely_local_costmap')
        self.last_gt = None
        self.laser = (0.25, 0.0, 0.0)
        self.fp = (0.42, -0.48, 0.25)
        self.create_subscription(LaserScan, '/scan', self.on_scan, qos_profile_sensor_data)

    def on_block(self, m):
        try:
            self._push('block', (self.now(), json.loads(m.data)))
        except Exception:  # noqa: BLE001
            pass

    def on_scan(self, m):
        if self.recording:
            c = M.footprint_clearance(m.ranges, m.angle_min, m.angle_increment, self.laser, self.fp)
            self._push('clear', (self.now(), c))

    def now(self):
        return time.monotonic() - self.t0

    def reset_buffers(self):
        with getattr(self, 'lock', threading.Lock()):
            self.buf = {'odom': [], 'cmd': [], 'cmd_nav': [], 'gt': [], 'trans': [], 'reasons': [], 'clear': [], 'block': []}
            self.plan = []

    def _push(self, key, item):
        if self.recording:
            with self.lock:
                self.buf[key].append(item)

    def on_odom(self, m):
        tw = m.twist.twist
        self._push('odom', (self.now(), math.hypot(tw.linear.x, tw.linear.y), tw.linear.x, tw.linear.y, tw.angular.z))

    def on_gt(self, m):
        q = m.pose.orientation
        yaw = math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))
        item = (self.now(), m.pose.position.x, m.pose.position.y, yaw)
        self.last_gt = item
        self._push('gt', item)

    def on_plan(self, m):
        if self.recording and not self.plan and m.poses:
            with self.lock:
                self.plan = [(p.pose.position.x, p.pose.position.y) for p in m.poses]

    # -------------------------------------------------------------- actions
    def teleport(self, x, y, yaw_deg):
        yaw = math.radians(yaw_deg)
        p = PoseStamped(); p.header.frame_id = 'map'; p.header.stamp = self.get_clock().now().to_msg()
        p.pose.position.x, p.pose.position.y = float(x), float(y)
        q = yaw_q(yaw); p.pose.orientation.x, p.pose.orientation.y, p.pose.orientation.z, p.pose.orientation.w = q
        self.pose_pub.publish(p)
        ip = PoseWithCovarianceStamped(); ip.header = p.header; ip.pose.pose = p.pose
        ip.pose.covariance[0] = ip.pose.covariance[7] = 0.01; ip.pose.covariance[35] = 0.005
        self.init_pub.publish(ip)

    def obstacles(self, items):
        self.obst_pub.publish(String(data=json.dumps({'op': 'clear'})))
        time.sleep(0.2)
        for o in items or []:
            cmd = dict(o); cmd['op'] = 'add'
            self.obst_pub.publish(String(data=json.dumps(cmd)))
            time.sleep(0.05)

    def clear_costmaps(self):
        for cli in (self.clear_l, self.clear_g):
            if cli.wait_for_service(timeout_sec=1.0):
                cli.call_async(ClearEntireCostmap.Request())

    def run_goal(self, goal, timeout, timed=None):
        gx, gy, gyaw_deg = goal
        msg = NavigateToPose.Goal()
        msg.pose.header.frame_id = 'map'; msg.pose.header.stamp = self.get_clock().now().to_msg()
        msg.pose.pose.position.x, msg.pose.pose.position.y = float(gx), float(gy)
        q = yaw_q(math.radians(gyaw_deg))
        msg.pose.pose.orientation.x, msg.pose.pose.orientation.y, msg.pose.pose.orientation.z, msg.pose.pose.orientation.w = q
        fut = self.nav.send_goal_async(msg)
        t_end = time.monotonic() + 10.0
        while not fut.done() and time.monotonic() < t_end:
            time.sleep(0.05)
        if not fut.done() or not fut.result().accepted:
            return 'REJECTED', 0.0
        handle = fut.result()
        t_start = time.monotonic()
        res_fut = handle.get_result_async()
        # timed events: {"at_s": t, ...obstacle} adds an obstacle, {"remove_at_s": t, "id": ...} removes one
        events = []
        for o in timed or []:
            if 'at_s' in o:
                cmd = {k: v for k, v in o.items() if k not in ('at_s', 'remove_at_s')}; cmd['op'] = 'add'
                events.append((float(o['at_s']), cmd))
            if 'remove_at_s' in o and 'id' in o:
                events.append((float(o['remove_at_s']), {'op': 'remove', 'id': o['id']}))
        events.sort(key=lambda e: e[0])
        while not res_fut.done():
            el = time.monotonic() - t_start
            while events and el >= events[0][0]:
                self.obst_pub.publish(String(data=json.dumps(events.pop(0)[1])))
            if time.monotonic() - t_start > timeout:
                handle.cancel_goal_async()
                time.sleep(1.0)
                return 'TIMEOUT', time.monotonic() - t_start
            time.sleep(0.05)
        return STATUS.get(res_fut.result().status, f'STATUS_{res_fut.result().status}'), time.monotonic() - t_start


def default_scenarios():
    try:
        from ament_index_python.packages import get_package_share_directory
        return os.path.join(get_package_share_directory('midpro_bringup'), 'scenarios', 'ecoprobm_regression.yaml')
    except Exception:  # noqa: BLE001
        return 'ecoprobm_regression.yaml'


def write_outputs(out_dir, label, rows, summ, gate, meta):
    os.makedirs(out_dir, exist_ok=True)
    keys = list(rows[0].keys()) if rows else []
    with open(os.path.join(out_dir, 'runs.csv'), 'w', newline='', encoding='utf-8') as f:
        w = csv.DictWriter(f, fieldnames=keys); w.writeheader(); [w.writerow(r) for r in rows]
    with open(os.path.join(out_dir, 'summary.json'), 'w', encoding='utf-8') as f:
        json.dump({'label': label, 'meta': meta, 'summary': summ, 'gate': gate}, f, ensure_ascii=False, indent=1)
    lines = [f'# 회귀 결과 — {label}', '', f"{meta['time']} · 반복 {meta['repeats']}회 · vmax {meta['vmax']}", '',
             '| 시나리오 | 통과율 | 점수(평균/최저) | 시간 s | 이탈 cm | 저크 RMS | 조향반전/분 | 주행중 회전 | 끼임 | 최소여유 cm | 우회비율 | 틈 통과 | 판단거리 m | 대기후우회 | 실패 | 장애물점수 |',
             '|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|']
    for name, s in summ.items():
        gp = '-' if s['gap_passed'] is None else f"{s['gap_passed']*100:.0f}%"
        lines.append(f"| {name} | {s['pass_rate']*100:.0f}% | {s['score_mean']}/{s['score_min']} | {s['time_mean']} | {s['xte_mean_cm']} | "
                     f"{s['jerk_rms_mean']} | {s['steer_rev_mean']} | {s['rotate_transit_mean']} | {s['wedged_runs']} | "
                     f"{s['min_clear_cm']} | {s['detour_ratio_mean']} | {gp} | {s.get('decision_min_m')} | {s.get('wait_switch_total')} | "
                     f"{s.get('block_fail_total')} | {s.get('obstacle_score_mean')} |")
    lines += ['', f"**1단계 G1 기준(전 시나리오 3회 중 2회 이상 통과, 최저 ≥ 60)**: {'통과' if gate['all_scenarios_B'] else '미달'} · 평균 통과율 {gate['pass_rate_mean']*100:.0f}%"]
    with open(os.path.join(out_dir, 'summary.md'), 'w', encoding='utf-8') as f:
        f.write('\n'.join(lines) + '\n')
    return '\n'.join(lines)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--scenarios', default=None)
    ap.add_argument('--label', default='run')
    ap.add_argument('--repeats', type=int, default=None)
    ap.add_argument('--only', default='', help='comma separated scenario names')
    ap.add_argument('--group', default='', help='base | wedge | detour (comma separated)')
    ap.add_argument('--vmax', type=float, default=0.6)
    ap.add_argument('--out', default=os.path.expanduser('~/midpro_reports/regression'))
    args, ros_args = ap.parse_known_args()
    path = args.scenarios or default_scenarios()
    spec = yaml.safe_load(open(path, encoding='utf-8'))
    fpspec = spec.get('footprint', {'front': 0.42, 'rear': -0.48, 'half_width': 0.25})
    lspec = spec.get('laser', [0.25, 0.0, 0.0])
    repeats = args.repeats or int(spec.get('repeats', 3))
    only = {s.strip() for s in args.only.split(',') if s.strip()}
    groups = {g.strip() for g in args.group.split(',') if g.strip()}
    scns = [s for s in spec['scenarios'] if (not only or s['name'] in only) and (not groups or s.get('group', 'base') in groups)]

    rclpy.init(args=ros_args)
    node = Recorder()
    node.fp = (float(fpspec['front']), float(fpspec['rear']), float(fpspec['half_width'])); node.laser = tuple(float(v) for v in lspec)
    ex = MultiThreadedExecutor(num_threads=4); ex.add_node(node)
    th = threading.Thread(target=ex.spin, daemon=True); th.start()
    print(f'[regression] waiting for navigate_to_pose ... ({path})')
    if not node.nav.wait_for_server(timeout_sec=90.0):
        print('[regression] navigate_to_pose not available'); rclpy.shutdown(); return
    time.sleep(3.0)
    rows = []
    out_dir = os.path.join(args.out, f"{args.label}_{time.strftime('%Y%m%d_%H%M%S')}")
    for rep in range(repeats):
        for scn in scns:
            print(f'[regression] {scn["name"]} ({rep + 1}/{repeats}) — {scn.get("desc", "")}')
            if not scn.get('goal'):
                print('    -> skipped: no goal in this scenario (save the layout after giving a goal)'); continue
            if not scn.get('start'):
                scn['start'] = None
            node.obstacles([])
            if scn.get('start'):
                node.teleport(*scn['start'])
            time.sleep(0.8)
            node.clear_costmaps(); time.sleep(0.5)
            node.obstacles([{k: v for k, v in o.items() if k != 'remove_at_s'} for o in scn.get('obstacles', []) if 'at_s' not in o]); time.sleep(float(spec.get('settle_s', 2.0)))
            node.reset_buffers(); node.recording = True
            result, dur = node.run_goal(scn['goal'], float(scn.get('timeout_s', 90)), [o for o in scn.get('obstacles', []) if 'at_s' in o or 'remove_at_s' in o])
            time.sleep(0.5); node.recording = False
            with node.lock:
                rec = {k: list(v) for k, v in node.buf.items()}; rec['plan'] = list(node.plan)
            rec.update(result=result, duration=dur, goal=(scn['goal'][0], scn['goal'][1], math.radians(scn['goal'][2])))
            row = M.compute(scn, rec, args.vmax, float(spec.get('stuck_window_s', 10.0)), float(spec.get('stuck_dist', 0.05)))
            row['repeat'] = rep + 1
            rows.append(row)
            print(f"    -> {result} {dur:.1f}s score {row['score_total']} pass {row['passed']} wedged {row['wedged']} clear {row['min_clear_cm']}cm detour {row['detour_ratio']} gap {row['gap_passed']}")
    node.obstacles([])
    summ, gate = M.summarize(rows)
    meta = {'time': time.strftime('%Y-%m-%d %H:%M:%S'), 'repeats': repeats, 'vmax': args.vmax, 'scenarios': path}
    print(write_outputs(out_dir, args.label, rows, summ, gate, meta))
    print(f'[regression] saved to {out_dir}')
    ex.shutdown(); rclpy.shutdown()


if __name__ == '__main__':
    main()

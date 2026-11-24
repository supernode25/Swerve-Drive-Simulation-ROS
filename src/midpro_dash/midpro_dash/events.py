"""events.py — live rule evaluator: emits timestamped findings as they happen (no repetition gating).

Each rule looks at a short sliding window of the Bus buffers and returns an Event when the
condition holds. The dashboard shows events immediately in the 이벤트 tab and the tuning report
lists them as a timeline. A rule re-fires for the same cause only after `cooldown` seconds.
"""
import math
import statistics
import time


class Event:
    __slots__ = ('t', 'wall', 'sev', 'kind', 'msg', 'param', 'cur', 'new', 'why')

    def __init__(self, t, sev, kind, msg, param=None, cur=None, new=None, why=''):
        self.t = t; self.wall = time.strftime('%H:%M:%S') + f'.{int((time.time() % 1) * 10)}'
        self.sev = sev; self.kind = kind; self.msg = msg; self.param = param; self.cur = cur; self.new = new; self.why = why


def _rms(xs):
    return math.sqrt(sum(x * x for x in xs) / len(xs)) if xs else 0.0


class LiveRules:
    def __init__(self, cooldown=10.0):
        self.cooldown = cooldown
        self.last_fire = {}
        self.events = []
        self.prev_state = None
        self.prev_reason = None
        self.reason_switches = []
        self.block_idx = 0

    def _fire(self, now, ev):
        if now - self.last_fire.get(ev.kind, -1e9) < self.cooldown:
            return
        self.last_fire[ev.kind] = now
        self.events.append(ev)

    def evaluate(self, b, now, params):
        """Call with b.lock held, ~2 Hz. params: current FollowPath parameters (may be empty)."""
        if not (b.run_active and not b.run_frozen):
            return []
        start = len(self.events)
        P = lambda k, d: params.get('FollowPath.' + k, d)  # noqa: E731
        params = dict(params)
        w3 = now - 3.0
        odom = [r for r in b.odom if r[0] >= w3]
        cmd = [r for r in b.cmd if r[0] >= w3]
        cmdn = [r for r in b.cmd_nav if r[0] >= w3]
        acc = [r for r in b.acc if r[0] >= w3]
        jerk = [r for r in b.jerk if r[0] >= w3]
        v = odom[-1][1] if odom else 0.0

        # 1. lateral acceleration on a curve: a_lat = v * wz (actual)
        if odom:
            a_lat = abs(odom[-1][1] * odom[-1][4])
            lim = P('lateral_acc_limit', 0.4)
            if a_lat > lim * 1.5 and v > 0.2:
                self._fire(now, Event(now, 3, 'lat_acc', f'코너 횡가속 {a_lat:.2f} m/s² (한계 {lim:.2f}) — 코너 진입 속도 과다',
                                      'lateral_acc_limit', lim, round(max(0.2, lim - 0.1), 2), '프로파일 코너 상한을 낮추면 코너 전 감속이 일찍 시작됩니다'))
        # 2. steering reversals in the last 10 s
        c10 = [r for r in b.cmd if r[0] >= now - 10]
        rev, prev = 0, 0.0
        for r in c10:
            if r[3] * prev < 0 and abs(r[3]) > 0.05:
                rev += 1
            if abs(r[3]) > 0.05:
                prev = r[3]
        wmin = P('min_angular_vel', 0.05)
        nz = [r[3] for r in c10 if abs(r[3]) > 1e-4]
        at_min = sum(1 for w in nz if abs(abs(w) - wmin) <= 0.15 * wmin) / max(len(nz), 1)
        if rev >= 4 and v > 0.2 and at_min > 0.4:
            self._fire(now, Event(now, 3, 'steer_floor', f'조향 하한 진동: 10 s에 {rev}회 반전, 명령의 {at_min*100:.0f}%가 ±{wmin}', 'min_angular_in_place_only', P('min_angular_in_place_only', False), True,
                                  '게인 문제가 아니라 최소 각속도 하한이 미세 보정을 부풀리는 리밋 사이클'))
        elif rev >= 4 and v > 0.2:
            k = P('k_prop', 1.2); la = P('heading_lookahead', 0.4)
            self._fire(now, Event(now, 3, 'steer_osc', f'조향 반전 {rev}회/10 s — 좌우 진동', 'k_prop', k, round(max(0.6, k - 0.2), 2),
                                  f'루프 이득 과다. 대안: heading_lookahead {la} → {round(min(0.8, la + 0.15), 2)}'))
        # 3. accel spike / jerk
        if acc and max(abs(r[1]) for r in acc) > 0.9:
            self._fire(now, Event(now, 2, 'acc_spike', f'가속 스파이크 {max(abs(r[1]) for r in acc):.2f} m/s²',
                                  'linear_acc_limit', P('linear_acc_limit', 0.4), P('linear_acc_limit', 0.4), '컨트롤러 램프보다 큰 계단 — 아래 "사유 전환"·"collision_monitor" 이벤트와 같이 보세요'))
        if jerk and _rms([r[1] for r in jerk]) > 2.5:
            self._fire(now, Event(now, 2, 'jerk', f'저크 RMS {_rms([r[1] for r in jerk]):.1f} m/s³ (3 s)', 'speed_lookahead_dist', P('speed_lookahead_dist', 1.2), round(P('speed_lookahead_dist', 1.2) + 0.3, 2), '상한이 틱마다 바뀜 — 전방 검사 구간을 늘리면 덜 깜빡임'))
        # 4. limit reason switching
        if b.reason_log:
            r = b.reason_log[-1][1]
            if self.prev_reason is not None and r != self.prev_reason:
                self.reason_switches.append(now)
            self.prev_reason = r
            self.reason_switches = [t for t in self.reason_switches if t >= now - 10]
            if len(self.reason_switches) >= 6:
                self._fire(now, Event(now, 2, 'reason_flap', f'속도 제한 사유가 10 s에 {len(self.reason_switches)}번 바뀜 ({r})', 'inflation_cost_threshold', P('inflation_cost_threshold', 80), min(95, P('inflation_cost_threshold', 80) + 5), '감속 주체가 문턱 근처에서 번갈아 작동 — 문턱을 올리거나 한 곳으로 통합'))
        # 5. collision monitor cutting
        if cmd and cmdn:
            j = 0; cut = 0
            for r in cmd:
                while j + 1 < len(cmdn) and cmdn[j + 1][0] <= r[0]:
                    j += 1
                if cmdn[j][1] > 0.1 and r[1] < 0.85 * cmdn[j][1]:
                    cut += 1
            if cut / len(cmd) > 0.5:
                self._fire(now, Event(now, 2, 'cm_cut', f'collision_monitor가 3 s의 {cut/len(cmd)*100:.0f}% 동안 속도를 깎음', '(collision_monitor) time_before_collision', 1.0, 0.7, '접근 제동이 벽 옆 조향에도 걸림. 정지 폴리곤이 footprint보다 크면 함께 축소'))
        # 6. cross-track error
        xs = [r[1] for r in b.xte if r[0] >= w3]
        if xs and max(xs) > 0.25:
            k = P('k_prop', 1.2)
            self._fire(now, Event(now, 3, 'xte', f'경로 이탈 {max(xs)*100:.0f} cm', 'curvature_feedforward', P('curvature_feedforward', False), True,
                                  f'곡선에서 P제어의 정상상태 오차(v·κ/k). 피드포워드를 켜거나 k_prop {k} → {round(min(2.0, k + 0.2), 2)}'))
        # 7. rotate in place while far from the goal
        st = b.state
        if st == 'ROTATE_IN_PLACE' and self.prev_state in ('FOLLOW_PATH',):
            self._fire(now, Event(now, 2, 'rotate_transit', '주행 중 제자리 회전 진입', 'heading_lookahead', P('heading_lookahead', 0.4), round(min(0.8, P('heading_lookahead', 0.4) + 0.1), 2), 'yaw_tolerance×6(8.6°)를 넘는 기준점 흔들림. 장애물 옆이면 NEAR 탈출 규칙이 먼저 작동해야 함'))
        if st == 'ROTATE_IN_PLACE' and self.prev_state == 'FINAL_APPROACH':
            self._fire(now, Event(now, 1, 'rotate_final', '접근 중 제자리 회전(목표 yaw 정렬)', 'holonomic_only_when_needed', P('holonomic_only_when_needed', True), True, '열린 공간이면 도착 후 FINAL_ROTATE가 맞음 — 목표 근처 NEAR 셀 여부 확인'))
        self.prev_state = st
        # 8. lateral motion in transit
        if cmd and sum(1 for r in cmd if abs(r[2]) > 0.05) / len(cmd) > 0.6 and st == 'FOLLOW_PATH':
            self._fire(now, Event(now, 1, 'vy_transit', '주행 중 횡이동(vy) 지속', 'max_linear_y', P('max_linear_y', 0.4), 0.25, '홀로노믹 판정이 직선에서 켜짐. NEAR 탈출 중이면 정상(escape 사유 확인)'))
        # 9. profile gap
        g = [p_ - v_ for (t, v_, p_, s_) in b.profile_gap_log if t >= w3 and s_ == 'FOLLOW_PATH' and p_ is not None]
        if g and statistics.mean(g) > 0.15 and (b.reason_log and b.reason_log[-1][1] in ('none', 'profile')):
            self._fire(now, Event(now, 2, 'profile_gap', f'계획 속도보다 실제가 {statistics.mean(g):.2f} m/s 낮음 (제한 사유 없음)', '(sim) acc_vx / (VCU) 가속 한계', '-', '-', '명령이 아니라 실행이 느림 — 구동부 한계 또는 smoother 상한'))
        # 11. obstacle handling (controller /midpro/block_event)
        evs = list(b.block_events)
        if self.block_idx > len(evs):
            self.block_idx = 0
        for (_t, be) in evs[self.block_idx:]:
            k = be.get('ev'); at = float(be.get('at', 0.0)); age = float(be.get('age', 0.0))
            if k == 'switch' and be.get('why') == 'static' and at < 2.5:
                self._fire(now, Event(now, 2, 'late_static', f'고정 장애물 판단이 늦음: {at:.1f} m 앞에서 우회 결정', 'block_check_dist', P('block_check_dist', 4.5), round(min(4.8, P('block_check_dist', 4.5) + 0.3), 2),
                                      '2.5 m 안에서 결정하면 긴 차체가 돌아 나갈 여유가 부족. 더 멀리 보거나 block_classify_time을 줄이기'))
            elif k == 'switch' and be.get('why') == 'persistent':
                self._fire(now, Event(now, 2, 'wait_switch', f'이동체로 보고 {age:.1f} s 대기한 뒤 우회', 'block_move_tol', P('block_move_tol', 0.2), round(min(0.4, P('block_move_tol', 0.2) + 0.1), 2),
                                      '고정물인데 가장 가까운 면이 바뀌어(크게 드러나는 물체) 이동체로 판정됐을 가능성. 허용 이동량을 키우면 바로 우회'))
            elif k == 'clear' and age > 2.5:
                self._fire(now, Event(now, 1, 'long_wait', f'{age:.1f} s 기다린 뒤 장애물이 비켜 감', 'block_persist_time', P('block_persist_time', 3.0), P('block_persist_time', 3.0),
                                      '대기 한도 직전에 해소. 자주 반복되면 사람이 많은 구역이므로 대기 시간 유지 또는 상향'))
            elif k == 'fail':
                self._fire(now, Event(now, 3, 'block_fail', f'막힌 경로의 대안이 없어 실패 ({be.get("why")})', 'hybrid_fallback', P('hybrid_fallback', False), True,
                                      '글로벌 경로가 장애물을 반영하지 못했거나 대안 경로도 막힘. 국소 Hybrid-A* 폴백을 켜면 좁은 창 안에서 우회 시도'))
        self.block_idx = len(evs)
        cl = [c for (tt, c) in b.clear_log if tt >= now - 1.0]
        if cl and min(cl) < 0.10 and v > 0.05:
            self._fire(now, Event(now, 2, 'too_close', f'차체와 장애물 여유 {min(cl)*100:.0f} cm', 'yield_stop_margin', P('yield_stop_margin', 1.2), round(P('yield_stop_margin', 1.2) + 0.3, 2),
                                  '회피·대기 중 너무 붙음. 정지 거리를 늘리거나 footprint_padding 상향'))
        # 10d. yielding to a blockage on the plan
        if b.reason_log and b.reason_log[-1][1].startswith('yield'):
            self._fire(now, Event(now, 1, 'yield', '경로 위 장애물 대기 중 (지나가면 원래 경로로)', 'block_persist_time', P('block_persist_time', 3.0), P('block_persist_time', 3.0),
                                  '3 s 넘게 계속 막히면 고정 장애물로 보고 우회. 보행자가 많은 곳은 4~5 s, 정적 장애물이 많은 곳은 2 s'))
        # 10c. boxed in (no rotation room, no fitting move)
        if b.reason_log and b.reason_log[-1][1].startswith('boxed_in'):
            self._fire(now, Event(now, 3, 'boxed_in', '회전 공간도, 들어가는 이동도 없음 → 정지', 'footprint_padding', P('footprint_padding', 0.03), round(max(0.0, P('footprint_padding', 0.03) - 0.01), 3),
                                  '차체가 장애물 사이에 끼인 자세. 반복되면 이 위치로 들어오게 만든 경로(전역 경로가 좁은 곳을 비스듬히 통과)를 확인'))
        # 10b. trajectory safety filter acted
        if b.reason_log and b.reason_log[-1][1].startswith('traj_safety'):
            self._fire(now, Event(now, 2, 'traj_safety', f"궤적 예측으로 충돌 회피 ({b.limit.split('traj_safety')[-1].strip()})", 'align_tolerance', P('align_tolerance', 0.12), round(max(0.06, P('align_tolerance', 0.12) - 0.04), 2),
                                  '명령대로 1 s 움직이면 차체(특히 회전 시 뒤쪽)가 닿는다는 예측. 자주 뜨면 좁은 구간 진입 전 정렬을 더 엄격하게, 또는 footprint_padding 상향'))
        # 10. replanning
        if b.plan_count_run > 1 and not getattr(self, '_replan_seen', False):
            self._replan_seen = True
            self._fire(now, Event(now, 2, 'replan', f'재계획 발생 ({b.plan_count_run}회)', '(costmap) observation_persistence', 2.0, 3.0, '경로 무효 판정이 깜빡이면 조향이 점프. 실제 장애물이면 정상'))
        if b.plan_count_run <= 1:
            self._replan_seen = False
        return self.events[start:]

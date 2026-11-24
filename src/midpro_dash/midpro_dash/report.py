"""report.py — tuning report for one goal run of the midpro Nav2 stack.

collect_stats(bus)   -> dict of evidence computed from the dashboard buffers
analyze(stats, params) -> list of findings, each with evidence, cause, parameter changes, expected effect
render_html(stats, findings, params, scores) -> self-contained HTML (dark, one accent colour)

The rules encode what was actually observed while bringing the stack up in simulation
(stacked slowdowns, replanning jitter, loop-gain drop with a farther steering reference,
steady-state drift on curves, approach braking, deadband at the goal) plus standard control
practice. Every recommendation names the parameter, its current value (read from the
controller_server), a suggested value, the reason, and the side effect to watch.
"""
import math
import statistics
import time
from collections import Counter

PARAM_NAMES = [
    'FollowPath.k_prop', 'FollowPath.heading_lookahead', 'FollowPath.curvature_feedforward',
    'FollowPath.max_linear_vel', 'FollowPath.linear_acc_limit', 'FollowPath.linear_dec_limit',
    'FollowPath.lateral_acc_limit', 'FollowPath.profile_curv_window', 'FollowPath.profile_min_speed',
    'FollowPath.near_obstacle_speed', 'FollowPath.inflation_speed', 'FollowPath.inflation_cost_threshold',
    'FollowPath.governor_floor', 'FollowPath.speed_lookahead_dist', 'FollowPath.approach_dist',
    'FollowPath.dist_tolerance', 'FollowPath.yaw_tolerance', 'FollowPath.min_linear_vel',
    'FollowPath.min_angular_vel', 'FollowPath.turn_angular_vel', 'FollowPath.max_angular_vel',
    'FollowPath.max_linear_y', 'FollowPath.linear_y_lpf_alpha', 'FollowPath.oblique_yaw_tolerance',
    'FollowPath.min_turn_speed_ratio', 'FollowPath.holonomic_zone',
    'FollowPath.plan_switching', 'FollowPath.transit_holonomic', 'FollowPath.collision_model', 'FollowPath.traj_check_enabled', 'FollowPath.orthogonal_avoid', 'FollowPath.side_front_enabled', 'FollowPath.goal_staging', 'FollowPath.path_conditioner_enabled',
    'FollowPath.min_angular_in_place_only', 'FollowPath.steer_deadband',
    'FollowPath.block_check_dist', 'FollowPath.block_classify_time', 'FollowPath.block_move_tol',
    'FollowPath.yield_stop_margin', 'FollowPath.block_persist_time', 'FollowPath.hybrid_fallback', 'FollowPath.footprint_padding',
]


# ----------------------------------------------------------------------------- helpers
def rms(xs):
    return math.sqrt(sum(x * x for x in xs) / len(xs)) if xs else None


def dominant_period(ts, ys, min_p=0.08, max_p=2.5):
    """Rough dominant period (s) of a signal via peak-to-peak intervals of |y| above 1.5*RMS."""
    if len(ts) < 20:
        return None
    r = rms(ys)
    if not r or r <= 0:
        return None
    thr = 1.5 * r
    peaks = []
    for i in range(1, len(ys) - 1):
        if abs(ys[i]) > thr and abs(ys[i]) >= abs(ys[i - 1]) and abs(ys[i]) >= abs(ys[i + 1]):
            if not peaks or ts[i] - peaks[-1] > min_p:
                peaks.append(ts[i])
    if len(peaks) < 4:
        return None
    gaps = [b - a for a, b in zip(peaks, peaks[1:])]
    gaps = [g for g in gaps if min_p <= g <= max_p]
    return statistics.median(gaps) if len(gaps) >= 3 else None


def lag_estimate(t_cmd, v_cmd, t_act, v_act, max_lag=0.8, step=0.05):
    """Delay (s) that best aligns actual speed to commanded speed (cross-correlation on a 20 Hz grid)."""
    if len(t_cmd) < 40 or len(t_act) < 40:
        return None
    t0, t1 = max(t_cmd[0], t_act[0]), min(t_cmd[-1], t_act[-1])
    if t1 - t0 < 5.0:
        return None
    grid = [t0 + k * step for k in range(int((t1 - t0) / step))]

    def resample(ts, ys):
        out, j = [], 0
        for g in grid:
            while j + 1 < len(ts) and ts[j + 1] <= g:
                j += 1
            out.append(ys[j])
        return out
    c = resample(t_cmd, v_cmd)
    a = resample(t_act, v_act)
    cm, am = statistics.mean(c), statistics.mean(a)
    c = [x - cm for x in c]
    a = [x - am for x in a]
    best, best_lag = -1e18, 0.0
    for k in range(0, int(max_lag / step) + 1):
        s = sum(c[i] * a[i + k] for i in range(len(c) - k))
        if s > best:
            best, best_lag = s, k * step
    return best_lag


def plan_curvature(plan_xy, window=0.3):
    """Curvature per plan point from tangent change over +-window (m)."""
    n = len(plan_xy)
    if n < 3:
        return [0.0] * n
    s = [0.0] * n
    for i in range(1, n):
        s[i] = s[i - 1] + math.hypot(plan_xy[i][0] - plan_xy[i - 1][0], plan_xy[i][1] - plan_xy[i - 1][1])

    def tang(i):
        a, b = max(i - 1, 0), min(i + 1, n - 1)
        return math.atan2(plan_xy[b][1] - plan_xy[a][1], plan_xy[b][0] - plan_xy[a][0])
    out = []
    for i in range(n):
        a = b = i
        while a > 0 and s[i] - s[a] < window:
            a -= 1
        while b + 1 < n and s[b] - s[i] < window:
            b += 1
        ds = max(s[b] - s[a], 1e-3)
        d = tang(b) - tang(a)
        d = (d + math.pi) % (2 * math.pi) - math.pi
        out.append(abs(d) / ds)
    return out


# ----------------------------------------------------------------------------- evidence
def collect_stats(bus):
    """Compute run evidence from the Bus buffers (call with bus.lock held)."""
    g = bus.goal_t if bus.goal_t is not None else 0.0
    now = bus.now()
    st = {'t_start': g, 't_end': now, 'duration': now - g, 'distance': bus.dist_since_plan, 'vmax': bus.vmax,
          'frozen': bus.run_frozen, 'final_err': bus.final_err}
    odom = [r for r in bus.odom if r[0] >= g]
    cmd = [r for r in bus.cmd if r[0] >= g]
    cmdn = [r for r in bus.cmd_nav if r[0] >= g]
    acc = [r for r in bus.acc if r[0] >= g]
    jerk = [r for r in bus.jerk if r[0] >= g]
    xte = [r for r in bus.xte if r[0] >= g]
    # speed
    vs = [r[1] for r in odom]
    moving = [v for v in vs if v > 0.05]
    st['v_mean'] = statistics.mean(moving) if moving else 0.0
    st['v_ratio'] = st['v_mean'] / bus.vmax if bus.vmax else 0.0
    st['t_at_vmax'] = sum(1 for v in vs if v >= 0.92 * bus.vmax) / max(len(vs), 1)
    # ramp slope: first time reaching 80% vmax
    t80 = next((r[0] for r in odom if r[1] >= 0.8 * bus.vmax), None)
    st['ramp_slope'] = (0.8 * bus.vmax) / (t80 - g) if t80 and t80 > g else None
    # accel / jerk
    st['acc_rms'] = rms([r[1] for r in acc])
    st['acc_max'] = max((abs(r[1]) for r in acc), default=0.0)
    st['jerk_rms'] = rms([r[1] for r in jerk])
    st['spike_period'] = dominant_period([r[0] for r in jerk], [r[1] for r in jerk])
    # steering
    rev, prev = 0, 0.0
    wz_all = []
    for r in cmd:
        if r[3] * prev < 0 and abs(r[3]) > 0.05:
            rev += 1
        if abs(r[3]) > 0.05:
            prev = r[3]
        wz_all.append(r[3])
    st['rev_per_min'] = rev / max(st['duration'] / 60.0, 0.25)
    wmax = bus.params.get('FollowPath.max_angular_vel', 0.8) if hasattr(bus, 'params') else 0.8
    st['wz_sat_frac'] = sum(1 for w in wz_all if abs(w) >= 0.95 * wmax) / max(len(wz_all), 1)
    wmin = bus.params.get('FollowPath.min_angular_vel', 0.05) if hasattr(bus, 'params') else 0.05
    moving_wz = [w for w in wz_all if abs(w) > 1e-4]
    st['wz_at_min_frac'] = sum(1 for w in moving_wz if abs(abs(w) - wmin) <= 0.15 * wmin) / max(len(moving_wz), 1)
    # straight vs curve segments from the plan curvature at the robot's nearest plan index
    kappa = plan_curvature(bus.plan_xy) if bus.plan_xy else []
    xte_straight, xte_curve, wz_straight, outward = [], [], [], []
    if kappa and xte:
        for (t, e) in xte:
            # nearest plan index was stored by cross_track hint at that time; approximate by distance fraction
            frac = min(max((t - g) / max(st['duration'], 1e-3), 0.0), 1.0)
            i = min(int(frac * (len(kappa) - 1)), len(kappa) - 1)
            (xte_curve if kappa[i] > 0.15 else xte_straight).append(e)
    for r in cmd:
        frac = min(max((r[0] - g) / max(st['duration'], 1e-3), 0.0), 1.0)
        if kappa:
            i = min(int(frac * (len(kappa) - 1)), len(kappa) - 1)
            if kappa[i] <= 0.15:
                wz_straight.append(r[3])
    st['xte_rms'] = rms([r[1] for r in xte])
    st['xte_max'] = max((r[1] for r in xte), default=0.0)
    st['xte_straight_rms'] = rms(xte_straight)
    st['xte_curve_rms'] = rms(xte_curve)
    st['wz_straight_rms'] = rms(wz_straight)
    st['curve_share'] = sum(1 for k in kappa if k > 0.15) / max(len(kappa), 1) if kappa else 0.0
    # lag between command and actual speed
    st['lag'] = lag_estimate([r[0] for r in cmd], [math.hypot(r[1], r[2]) for r in cmd], [r[0] for r in odom], [r[1] for r in odom])
    # collision monitor cut: final cmd much lower than controller output at the same time
    cut = 0
    j = 0
    for r in cmd:
        while j + 1 < len(cmdn) and cmdn[j + 1][0] <= r[0]:
            j += 1
        if cmdn and cmdn[j][1] > 0.1 and r[1] < 0.9 * cmdn[j][1]:
            cut += 1
    st['cm_cut_frac'] = cut / max(len(cmd), 1)
    # lateral motion outside the goal zone
    hz = bus.params.get('FollowPath.holonomic_zone', 1.5) if hasattr(bus, 'params') else 1.5
    st['vy_transit_frac'] = sum(1 for r in cmd if abs(r[2]) > 0.05) / max(len(cmd), 1)
    # limit reasons
    reasons = Counter(k for (t, k) in bus.reason_log if t >= g)
    tot = sum(reasons.values()) or 1
    st['reason_share'] = {k: v / tot for k, v in reasons.items()}
    changes = 0
    last = None
    for (t, k) in bus.reason_log:
        if t >= g:
            if last is not None and k != last:
                changes += 1
            last = k
    st['reason_changes_per_min'] = changes / max(st['duration'] / 60.0, 0.25)
    # profile gap while FOLLOW_PATH
    gaps = [p - v for (t, v, p, s_) in bus.profile_gap_log if t >= g and s_ == 'FOLLOW_PATH' and p is not None]
    st['profile_gap'] = statistics.mean(gaps) if gaps else None
    # FSM
    st['state_time'] = {k: v for k, v in bus.state_time.items()}
    st['rotate_transit'] = bus.rotate_count_transit
    st['rotate_total'] = bus.rotate_count
    st['replans'] = bus.plan_count_run
    st['detour_share'] = bus.detour_time / max(st['duration'], 1e-3)
    # goal overshoot: speed within last 0.3 m (needs xte progress) -> use last 1.5 s before freeze
    from midpro_dash import metrics as MM
    bev = [e for e in getattr(bus, 'block_events', []) if e[0] >= g]
    st['block'] = MM.block_stats(bev, [c for c in getattr(bus, 'clear_log', []) if c[0] >= g])
    st['block_events'] = [(round(t - g, 1), e) for (t, e) in bev]
    tail = [r[1] for r in odom if r[0] >= now - 1.5] if bus.run_frozen else []
    st['tail_speed_max'] = max(tail, default=None) if tail else None
    return st



# ----------------------------------------------------------------------------- history / convergence
import json
import os

HISTORY = os.path.expanduser('~/midpro_reports/history.json')
GRADE_MIN = {'A': 85, 'B': 70, 'C': 50}


def target_score(target):
    """target may be a grade letter or a number ('95')."""
    try:
        return float(target)
    except (TypeError, ValueError):
        return GRADE_MIN.get(str(target), 70)


# Absolute acceptance spec per item: (90-point equivalent, 95-point equivalent) in measured units.
# These are the numbers to judge by; the score is just a normalised view of them.
ACCEPT = {
    'track': ('경로 이탈 RMS', '≤ 7.5 cm', '≤ 6.3 cm', 'cm'),
    'accel': ('가속 RMS', '≤ 0.14 m/s²', '≤ 0.12 m/s²', 'm/s²'),
    'jerk': ('저크 RMS', '≤ 0.75 m/s³', '≤ 0.63 m/s³', 'm/s³'),
    'steer': ('조향 반전', '≤ 3.8 회/분', '≤ 2.9 회/분', '회/분'),
    'speed': ('평균/상한', '≥ 84 %', '≥ 87 %', '%'),
    'stop': ('도착 오차', '≤ 2.8 cm · 1.8°', '≤ 2.4 cm · 1.6°', ''),
    'obstacle': ('장애물 대응', '감점 ≤ 10', '감점 ≤ 5', ''),
}


def load_history():
    try:
        with open(HISTORY, 'r', encoding='utf-8') as f:
            return json.load(f)
    except Exception:  # noqa: BLE001
        return []


def save_history(runs):
    os.makedirs(os.path.dirname(HISTORY), exist_ok=True)
    with open(HISTORY, 'w', encoding='utf-8') as f:
        json.dump(runs[-50:], f, ensure_ascii=False, indent=1)


def record_run(history, st, scores, findings, params):
    vals = [v for v in scores.values() if v is not None]
    total = sum(vals) / len(vals) if vals else 0.0
    prev = history[-1] if history else None
    changed = {}
    if prev:
        for k, v in params.items():
            if k in prev.get('params', {}) and prev['params'][k] != v:
                changed[k] = (prev['params'][k], v)
    history.append({'time': time.strftime('%Y-%m-%d %H:%M:%S'), 'total': total, 'scores': {k: (None if v is None else round(v, 1)) for k, v in scores.items()},
                    'findings': [f['title'] for f in findings], 'sev3': [f['title'] for f in findings if f['severity'] == 3],
                    'params': params, 'changed': changed,
                    'distance': st.get('distance'), 'duration': st.get('duration'), 'frozen': st.get('frozen')})
    return history


def convergence(history, target):
    """Return (status, message, locked_params) describing whether tuning should stop."""
    locked = []
    if len(history) >= 4:
        # a parameter that went up then down (or down then up) within the last 4 runs is oscillating
        for k in history[-1].get('params', {}):
            vals = [h['params'].get(k) for h in history[-4:] if isinstance(h.get('params', {}).get(k), (int, float))]
            if len(vals) == 4:
                d = [b - a for a, b in zip(vals, vals[1:])]
                if any(x > 0 for x in d) and any(x < 0 for x in d) and any(abs(x) > 1e-9 for x in d):
                    locked.append(k)
    totals = [h['total'] for h in history[-3:]]
    last = history[-1] if history else None
    sev3 = last.get('sev3', []) if last else []
    tgt = target_score(target)
    weak = [k for k, v in (last.get('scores') or {}).items() if v is not None and v < tgt - 10] if last else []
    if last and last['total'] >= tgt and not sev3 and not weak:
        return 'done', f"목표 {tgt:.0f}점 달성, 모든 항목이 기준 근처 — 파라미터를 고정하고 회귀 테스트 기준선으로 저장하세요.", locked
    if len(totals) == 3 and max(totals) - min(totals) < 3.0:
        return 'converged', f"최근 3회 점수 변동이 {max(totals)-min(totals):.1f}점 — 파라미터 조정으로는 더 좋아지지 않는 구간입니다. 남은 항목은 코드 수정(아래 '코드 피드백')이나 하드웨어·지도 쪽 원인입니다.", locked
    if len(history) >= 2 and history[-1]['changed'] and history[-1]['total'] < history[-2]['total'] - 3.0:
        return 'worse', f"직전 변경 {', '.join(history[-1]['changed'])} 후 점수가 {history[-2]['total']-history[-1]['total']:.0f}점 내려갔습니다 — 되돌리세요.", locked
    return 'tuning', '', locked


def repeat_count(history, title, n=3):
    return sum(1 for h in history[-n:] if title in h.get('findings', []))


# ----------------------------------------------------------------------------- rules
def _p(params, name, default):
    return params.get('FollowPath.' + name, default)


def analyze(st, params, scores, history=None, target='B'):
    """Return findings: dicts with keys title, severity(1..3), evidence, cause, changes[list], effect, caution, code, repeat, apply."""
    F = []
    history = history or []
    k_prop = _p(params, 'k_prop', 1.2)
    la = _p(params, 'heading_lookahead', 0.4)
    ff = _p(params, 'curvature_feedforward', False)
    a_lat = _p(params, 'lateral_acc_limit', 0.4)
    dec = _p(params, 'linear_dec_limit', 0.4)
    accl = _p(params, 'linear_acc_limit', 0.4)
    vmax = _p(params, 'max_linear_vel', 0.6)
    share = st.get('reason_share', {})

    # 1. curve drift (steady-state heading error of the P law)
    if st.get('xte_curve_rms') and st['xte_curve_rms'] > 0.08 and (st.get('xte_straight_rms') or 0) < 0.6 * st['xte_curve_rms']:
        if st.get('rev_per_min', 0) < 8:
            changes = ([('curvature_feedforward', ff, True, '곡선에서 필요한 각속도 v·κ를 미리 더해 정상상태 오차를 제거'),
                        ('k_prop', k_prop, round(max(0.8, k_prop - 0.2), 2), '피드포워드가 있으면 P 이득은 낮춰도 됨(진동 여유 확보)')]
                       if not ff else
                       [('lateral_acc_limit', a_lat, round(max(0.2, a_lat - 0.1), 2), '코너 속도를 낮춰 요구 각속도를 줄임'),
                        ('heading_lookahead', la, round(max(0.25, la - 0.1), 2), '기준점을 가깝게 해 곡선 추종 민감도를 올림')])
        else:
            changes = [('curvature_feedforward', ff, True, '이득을 올리면 진동이 커지는 상태라 피드포워드로 해결'),
                       ('lateral_acc_limit', a_lat, round(max(0.2, a_lat - 0.1), 2), '코너 속도 하향')]
        F.append(dict(title='곡선에서 경로 바깥으로 밀림', severity=3,
                      evidence=f"곡선 구간 이탈 RMS {st['xte_curve_rms']*100:.1f} cm vs 직선 {((st.get('xte_straight_rms') or 0)*100):.1f} cm, 조향 반전 {st.get('rev_per_min',0):.1f}/분",
                      cause='midpro 조향은 횡오차 항이 없는 방향 P제어라, 곡률 κ를 속도 v로 따라가려면 e = v·κ/k_prop 만큼의 방향 오차가 항상 남습니다. 그 오차가 바깥쪽 이탈로 나타납니다.',
                      changes=changes, effect='곡선 이탈 RMS 50% 이상 감소', caution='피드포워드는 경로 곡률 추정에 의존하므로 격자 경로가 거칠면 profile_curv_window를 함께 0.5로'))

    # 2. steering oscillation
    if (st.get('rev_per_min', 0) > 8 or (st.get('wz_straight_rms') or 0) > 0.12) and st.get('wz_at_min_frac', 0) > 0.4:
        F.append(dict(title='조향 하한 진동 (±min_angular_vel 네모파)', severity=3,
                      evidence=f"조향 반전 {st.get('rev_per_min',0):.1f}/분, 명령의 {st['wz_at_min_frac']*100:.0f}%가 ±{_p(params,'min_angular_vel',0.05)} 근처",
                      cause='게인 문제가 아닙니다. 최소 각속도 하한이 주행 중 미세 보정(0.01)을 0.05로 부풀려 과회전 → 부호 반전 → 반복하는 리밋 사이클입니다. 명령이 매끈한 곡선이 아니라 ± 최소값을 오가는 네모파면 이 경우입니다.',
                      changes=[('min_angular_in_place_only', _p(params, 'min_angular_in_place_only', False), True, '하한은 제자리·도착 회전에서만'),
                               ('steer_deadband', _p(params, 'steer_deadband', 0.0), 0.01, '주행 중 아주 작은 보정은 0')],
                      effect='직진 중 조향 반전 ≈ 0', caution='실기에서는 구동부가 0.01~0.05 명령에 반응하는지 확인'))
    elif st.get('rev_per_min', 0) > 8 or (st.get('wz_straight_rms') or 0) > 0.12:
        lag = st.get('lag')
        changes = [('k_prop', k_prop, round(max(0.6, k_prop - 0.3), 2), '루프 이득 감소'),
                   ('heading_lookahead', la, round(min(0.8, la + 0.15), 2), '기준점을 멀리 두면 격자 흔들림이 평균되고 이득 민감도가 낮아짐')]
        cause = '유효 루프 이득이 속도에 비례(ė ≈ −v·k·e)하므로 저속에서 맞춘 이득이 고속에서 진동으로 나타납니다.'
        if lag and lag > 0.25:
            cause += f' 명령→실제 지연이 {lag:.2f} s로 커서 위상 여유가 더 줄어든 상태입니다(조향 정렬 지연 또는 필터).'
            changes.append(('linear_y_lpf_alpha', _p(params, 'linear_y_lpf_alpha', 0.3), 0.5, '출력 필터 지연 축소(지연이 큰 경우)'))
        F.append(dict(title='조향이 좌우로 진동', severity=3 if st.get('rev_per_min', 0) > 15 else 2,
                      evidence=f"조향 반전 {st.get('rev_per_min',0):.1f}/분, 직선 구간 wz RMS {((st.get('wz_straight_rms') or 0)):.2f} rad/s, 지연 추정 {lag if lag else '-'} s",
                      cause=cause, changes=changes, effect='반전/분 5 이하, 직선 wz RMS 0.05 이하', caution='이득을 낮추면 곡선 이탈이 늘 수 있음 → 곡선 이탈 항목이 함께 있으면 curvature_feedforward를 먼저'))

    # 3. stacked / flapping slowdowns
    if st.get('reason_changes_per_min', 0) > 20 or (st.get('jerk_rms') or 0) > 2.0:
        changes = [('inflation_cost_threshold', _p(params, 'inflation_cost_threshold', 80), 90, '팽창 감속 문턱을 벽에 더 가깝게'),
                   ('speed_lookahead_dist', _p(params, 'speed_lookahead_dist', 1.2), 1.5, '전방 검사 구간을 늘려 상한이 덜 깜빡이게'),
                   ('governor_floor', _p(params, 'governor_floor', 0.4), _p(params, 'governor_floor', 0.4), '하한은 유지(중첩 방지)')]
        F.append(dict(title='속도 상한이 자주 바뀜(상한 깜빡임)', severity=2,
                      evidence=f"제한 사유 전환 {st.get('reason_changes_per_min',0):.0f}회/분, 저크 RMS {(st.get('jerk_rms') or 0):.2f} m/s³, 사유 점유 {', '.join(f'{k} {v*100:.0f}%' for k,v in share.items())}",
                      cause='여러 감속 주체가 문턱 근처에서 번갈아 작동하면 20 Hz 틱마다 상한이 바뀌어 저크가 커집니다. v0.10에서 collision_monitor 비율 감속과 거버너가 곱으로 겹쳤던 것과 같은 유형입니다.',
                      changes=changes, effect='저크 RMS 1.0 이하', caution='문턱을 올리면 벽 가까이에서 감속이 늦어지므로 NEAR(0.59 m) 밴드는 그대로 둘 것'))

    # 4. periodic spikes -> replanning / costmap
    sp = st.get('spike_period')
    if sp and (st.get('acc_rms') or 0) > 0.15:
        if 0.8 <= sp <= 1.3 and st.get('replans', 0) > 1:
            F.append(dict(title='약 1 s 주기의 가속 스파이크 — 재계획 떨림', severity=2,
                          evidence=f"스파이크 주기 {sp:.2f} s, 이번 주행 재계획 {st.get('replans',0)}회",
                          cause='글로벌 경로가 주기적으로 바뀌면 조향 기준점이 점프합니다. 경로가 무효일 때만 재계획해야 하는데 재계획이 반복되고 있습니다(장애물 관측이 깜빡이거나 IsPathValid가 흔들림).',
                          changes=[('(costmap) obstacle_layer.scan.observation_persistence', 2.0, 3.0, '관측 유지 시간을 늘려 경로 무효 판정 깜빡임 억제'),
                                   ('(BT) RateController hz', 2.0, 1.0, '재계획 검사 주기 완화')],
                          effect='재계획 1회/목표', caution='실제 장애물 출현에 대한 반응이 1 s 늦어짐'))
        elif 0.08 <= sp <= 0.15:
            F.append(dict(title='약 0.1 s 주기의 스파이크 — costmap 갱신에 따른 반응형 감속', severity=1,
                          evidence=f'스파이크 주기 {sp:.2f} s',
                          cause='로컬 costmap(10 Hz) 갱신마다 거버너의 전방 비용이 바뀌어 상한이 흔들립니다.',
                          changes=[('speed_lookahead_dist', _p(params, 'speed_lookahead_dist', 1.2), 1.5, '검사 구간을 늘려 한 셀 변화의 영향 축소'),
                                   ('inflation_cost_threshold', _p(params, 'inflation_cost_threshold', 80), 90, '문턱 상향')],
                          effect='가속 RMS 감소', caution='없음'))

    # 5. collision monitor cutting
    if st.get('cm_cut_frac', 0) > 0.15:
        F.append(dict(title='collision_monitor가 속도를 깎는 시간이 큼', severity=2,
                      evidence=f"최종 cmd_vel이 컨트롤러 출력의 90% 미만인 시간 {st['cm_cut_frac']*100:.0f}%",
                      cause='footprint 접근 제동(1 s 안 충돌 예측)이 벽 옆 조향 중에도 걸리거나 정지 폴리곤이 넓습니다. 거버너와 겹치면 실제 속도가 프로파일 아래로 떨어집니다.',
                      changes=[('(collision_monitor) FootprintApproach.time_before_collision', 1.0, 0.7, '접근 제동을 더 늦게'),
                               ('(collision_monitor) PolygonStop points', '0.52/−0.50 × ±0.35', '0.45/−0.45 × ±0.30', '정지 구역을 footprint에 가깝게(실기 안전 요구와 상의)')],
                      effect='순항 속도 복원', caution='안전 여유가 줄어드므로 실기에서는 안전 라이다 필드와 역할 분담 후 조정'))

    # 6. speed efficiency
    if scores.get('speed') is not None and scores['speed'] < 70:
        dom = max(share, key=share.get) if share else 'none'
        if dom == 'profile':
            changes = [('lateral_acc_limit', a_lat, round(min(0.8, a_lat + 0.15), 2), '코너 허용 속도 상향'),
                       ('profile_min_speed', _p(params, 'profile_min_speed', 0.15), 0.2, '프로파일 하한 상향')]
            cause = '속도 프로파일이 주된 제한입니다. 곡률이 큰 구간이 많거나 a_lat이 보수적입니다.'
        elif dom == 'near_obstacle':
            changes = [('near_obstacle_speed', _p(params, 'near_obstacle_speed', 0.4), 0.45, 'NEAR 반응 속도 상향'),
                       ('(global_costmap) cost_travel_multiplier', 6.0, 8.0, '글로벌 경로를 통로 중앙으로 더 당겨 NEAR 진입 자체를 줄임')]
            cause='경로가 NEAR(0.59 m) 밴드를 자주 지나갑니다. 경로가 벽에 붙는 것이 근본 원인입니다.'
        elif dom == 'goal_brake':
            changes = [('linear_dec_limit', dec, round(min(0.8, dec + 0.1), 2), '제동 시작을 늦춤')]
            cause='짧은 주행이라 목표 제동이 전체의 큰 부분을 차지합니다. 코스가 짧으면 정상입니다.'
        elif st.get('cm_cut_frac', 0) > 0.15:
            changes = []
            cause='collision_monitor 감속이 원인입니다(5번 참조).'
        elif st.get('profile_gap') and st['profile_gap'] > 0.1:
            changes = [('(sim) acc_vx / (VCU) 가속 한계', '-', '-', '실제가 명령을 못 따라감: 구동부 한계 확인')]
            cause=f"프로파일보다 실제가 평균 {st['profile_gap']:.2f} m/s 낮습니다. 명령이 아니라 실행 쪽이 느립니다."
        else:
            changes = [('linear_acc_limit', accl, round(min(0.8, accl + 0.1), 2), '램프를 가파르게')]
            cause='특정 제한 없이 평균 속도가 낮습니다. 램프·제동이 전체 구간에서 차지하는 비중이 큽니다.'
        F.append(dict(title='속도를 충분히 내지 못함', severity=2,
                      evidence=f"평균/상한 {st.get('v_ratio',0)*100:.0f}%, 상한 유지 시간 {st.get('t_at_vmax',0)*100:.0f}%, 주된 사유 {dom}",
                      cause=cause, changes=changes, effect='평균 속도 상한의 90% 이상', caution='속도를 올리면 1·2번 항목이 다시 나빠질 수 있음'))

    # 7. stop precision
    fe = st.get('final_err')
    if fe and (fe[0] > 0.03 or fe[1] > 2.0):
        changes = []
        if fe[0] > 0.03:
            changes += [('min_linear_vel', _p(params, 'min_linear_vel', 0.02), 0.03, '구동 데드밴드 바로 위로(너무 낮으면 마지막 2 cm를 못 밀고 멈춤)'),
                        ('approach_dist', _p(params, 'approach_dist', 0.8), 0.6, '접근 구간을 줄여 저속 구간 단축')]
            if st.get('tail_speed_max') and st['tail_speed_max'] > 0.15:
                changes.append(('linear_dec_limit', dec, round(max(0.2, dec - 0.1), 2), f"도착 직전 속도 {st['tail_speed_max']:.2f} m/s → 오버슈트, 제동을 일찍"))
        if fe[1] > 2.0:
            changes += [('min_angular_vel', _p(params, 'min_angular_vel', 0.05), 0.08, '마지막 정렬에서 회전 데드밴드 극복'),
                        ('turn_angular_vel', _p(params, 'turn_angular_vel', 0.8), 0.6, 'FINAL_ROTATE 과회전 억제')]
        F.append(dict(title='도착 정밀도 부족', severity=2,
                      evidence=f'위치 오차 {fe[0]*100:.1f} cm, 방향 오차 {fe[1]:.1f}°',
                      cause='마지막 수 cm는 가속 램프·거리 비례 제동·데드밴드의 균형으로 결정됩니다. 위치 오차는 보통 min_linear_vel 또는 오버슈트, 방향 오차는 회전 데드밴드입니다.',
                      changes=changes, effect='2 cm · 1.4° 이내', caution='min 속도를 올리면 접근이 거칠어질 수 있음'))

    # 8. rotate in place during transit
    if st.get('rotate_transit', 0) > 0:
        yt = _p(params, 'yaw_tolerance', 0.025)
        F.append(dict(title='주행 중 제자리 회전 발생', severity=2,
                      evidence=f"주행 중 ROTATE_IN_PLACE {st['rotate_transit']}회 (도착 정렬 제외)",
                      cause=f'홀로노믹 구간은 기준점 yaw 오차가 yaw_tolerance×6 = {math.degrees(yt*6):.1f}°만 넘어도 회전 모드에 들어갑니다. 격자 경로의 각이나 기준점 흔들림이 원인입니다.',
                      changes=[('heading_lookahead', la, round(min(0.8, la + 0.1), 2), '기준점을 멀리 두어 yaw 흔들림 평균화'),
                               ('(planner) smoother.w_smooth', 0.3, 0.5, '경로 각을 둥글게'),
                               ('oblique_yaw_tolerance', _p(params, 'oblique_yaw_tolerance', 1e-4), 1e-3, '홀로노믹 판정 흔들림 억제')],
                      effect='주행 중 회전 0회', caution='w_smooth를 올리면 코너가 둥글어져 좁은 곳에서 벽에 가까워질 수 있음'))

    # 9. lateral motion in transit
    if st.get('vy_transit_frac', 0) > 0.2 and st.get('curve_share', 0) < 0.3:
        F.append(dict(title='주행 중 횡이동(vy)이 많음', severity=1,
                      evidence=f"|vy|>0.05 시간 비율 {st['vy_transit_frac']*100:.0f}%, 곡선 비율 {st.get('curve_share',0)*100:.0f}%",
                      cause='홀로노믹 판정이 직선 구간에서 켜져 차체 방향과 진행 방향이 어긋난 채 게걸음합니다. 스웨브는 휠 재정렬 부담이 커집니다.',
                      changes=[('max_linear_y', _p(params, 'max_linear_y', 0.4), 0.25, '횡속도 클램프 축소'),
                               ('oblique_yaw_tolerance', _p(params, 'oblique_yaw_tolerance', 1e-4), 1e-3, '판정 안정화')],
                      effect='직선 구간 vy ≈ 0', caution='없음'))

    # 10. accel RMS high without other cause
    if (st.get('acc_rms') or 0) > 0.3 and not any(f['title'].startswith(('속도 상한', '약 1 s', '약 0.1 s', 'collision')) for f in F):
        F.append(dict(title='가감속이 거침', severity=2,
                      evidence=f"가속 RMS {st['acc_rms']:.2f} m/s², |a| 최대 {st.get('acc_max',0):.2f}",
                      cause='램프와 제동의 기울기가 크거나, 컨트롤러 가속 한계와 velocity_smoother 한계가 어긋나 계단이 생깁니다.',
                      changes=[('linear_acc_limit', accl, round(max(0.2, accl - 0.1), 2), '램프 완화'),
                               ('(velocity_smoother) max_accel[0]', 0.5, round(max(0.2, accl - 0.1) + 0.1, 2), '컨트롤러 한계보다 약간 위로 유지')],
                      effect='가속 RMS 0.15 이하', caution='가속을 낮추면 속도 효율 점수가 내려감'))

    # --- obstacle handling (block events from the controller)
    bs = st.get('block') or {}
    if bs.get('detect'):
        def pv(k, d):
            return _p(params, k, d)
        if bs.get('decision_min') is not None and bs['decision_min'] < 2.5:
            F.append(dict(title='고정 장애물 판단이 늦음', severity=2, direct=True,
                          evidence=f"우회 결정 거리 최소 {bs['decision_min']:.1f} m (기준 2.5 m 이상), 고정물 우회 {len(bs['static_at'])}회",
                          cause='막힘을 감지하고 고정물로 판정하기까지 걸린 거리가 짧아, 0.9 m 차체가 돌아 나갈 여유가 부족한 위치에서 우회를 시작했습니다.',
                          changes=[('block_check_dist', pv('block_check_dist', 4.5), round(min(4.8, pv('block_check_dist', 4.5) + 0.3), 2), '더 멀리서 감지 (로컬 costmap 반경 5 m 안)'),
                                   ('block_classify_time', pv('block_classify_time', 1.0), round(max(0.5, pv('block_classify_time', 1.0) - 0.3), 2), '고정물 판정을 빠르게 (사람 오판이 늘 수 있음)')],
                          effect='우회 결정 거리 2.5 m 이상', caution='판정 시간을 줄이면 지나가는 사람을 고정물로 보고 우회하는 경우가 늘어남'))
        if bs.get('persistent_age'):
            F.append(dict(title='이동체로 판정해 기다린 뒤 우회', severity=2, direct=True,
                          evidence=f"대기 후 우회 {len(bs['persistent_age'])}회 (대기 {', '.join(f'{a:.1f}' for a in bs['persistent_age'])} s), 이동체 판정 {bs['moving']}회",
                          cause='결국 비키지 않은 장애물(고정물)을 이동체로 판정했습니다. 큰 물체가 다가가며 조금씩 드러나 가장 가까운 면이 바뀌면 움직인 것으로 보입니다.',
                          changes=[('block_move_tol', pv('block_move_tol', 0.2), round(min(0.4, pv('block_move_tol', 0.2) + 0.1), 2), '움직임 판정 기준 완화 → 고정물로 바로 우회'),
                                   ('block_persist_time', pv('block_persist_time', 3.0), round(max(1.5, pv('block_persist_time', 3.0) - 1.0), 1), '정적 물건이 많은 현장이면 대기 단축')],
                          effect='대기 후 우회 0회', caution='기준을 완화하면 느리게 걷는 사람을 고정물로 볼 수 있음'))
        long_waits = [a for a in bs.get('clear_age', []) if a > 2.5]
        if len(long_waits) >= 2:
            F.append(dict(title='대기 한도 직전에 해소되는 장애물이 많음', severity=1, direct=True,
                          evidence=f"2.5 s 넘게 기다린 뒤 해소 {len(long_waits)}회",
                          cause='사람이 자주 지나가는 구역입니다. 지금 대기 한도에서 조금만 짧으면 불필요하게 우회했을 경우들입니다.',
                          changes=[('block_persist_time', pv('block_persist_time', 3.0), round(pv('block_persist_time', 3.0) + 1.0, 1), '이동체 대기 한도 상향')],
                          effect='불필요한 우회 방지', caution='고정물 우회도 그만큼 늦어짐(고정물은 1 s 판정으로 따로 처리되므로 영향 작음)'))
        if bs.get('fail'):
            F.append(dict(title='막힌 경로의 대안이 없어 실패', severity=3, direct=True,
                          evidence=f"실패 {bs['fail']}회 ({', '.join(bs.get('fail_why', []))})",
                          cause='막힘이 지속됐는데 BT가 준 대안 경로가 없거나 같은 곳을 지나서, FollowPath 실패와 복구 동작(후진·회전)으로 넘어갔습니다.',
                          changes=[('hybrid_fallback', pv('hybrid_fallback', False), True, '국소 Hybrid-A*로 좁은 창 안에서 우회 시도 (launch hybrid_fallback:=true)')],
                          effect='복구 동작 없이 우회', caution='Hybrid 서버 CPU 사용 증가'))
        mc = bs.get('min_clear')
        if mc is not None and mc < 0.10:
            F.append(dict(title='장애물에 너무 가까이 접근', severity=2, direct=True,
                          evidence=f"주행 중 차체–장애물 최소 여유 {mc*100:.0f} cm",
                          cause='대기 정지 위치나 회피 경로가 장애물에 붙었습니다. 실기에서는 측위 오차만큼 더 가까워집니다.',
                          changes=[('yield_stop_margin', pv('yield_stop_margin', 1.2), round(pv('yield_stop_margin', 1.2) + 0.3, 2), '대기 정지 거리 확대'),
                                   ('footprint_padding', pv('footprint_padding', 0.03), round(pv('footprint_padding', 0.03) + 0.01, 3), '통과 판정 여유 확대 (좁은 틈 통과 능력은 약간 감소)')],
                          effect='최소 여유 10 cm 이상', caution='패딩을 키우면 폭 + 10 cm 틈 통과가 어려워질 수 있음'))

    # --- fine tuning: any score below the target gets a concrete suggestion even if no hard rule fired
    tgt = target_score(target)
    covered = {'track': ('곡선에서', '경로'), 'steer': ('조향이', '조향 하한'), 'jerk': ('속도 상한', '약 0.1', '약 1 s'), 'accel': ('가감속', '약 1 s', 'collision'),
               'speed': ('속도를',), 'stop': ('도착',), 'obstacle': ('고정 장애물', '이동체로', '대기 한도', '막힌 경로', '장애물에 너무')}
    names = {'track': '경로 추종', 'accel': '가감속 부드러움', 'jerk': '저크', 'steer': '조향 안정', 'speed': '속도 효율', 'stop': '도착 정밀도', 'obstacle': '장애물 대응'}
    fine = {
        'track': [('k_prop', k_prop, round(min(2.0, k_prop + 0.1), 2), '추종 이득 소폭 상향'),
                  ('curvature_feedforward', ff, True, '곡선 정상상태 오차 제거(이득을 올리지 않고 추종 개선)')],
        'accel': [('linear_acc_limit', accl, round(max(0.2, accl - 0.05), 2), '램프 완화'),
                  ('(velocity_smoother) max_accel[0]', 0.5, round(max(0.2, accl - 0.05) + 0.1, 2), '컨트롤러 한계보다 약간 위로 유지')],
        'jerk': [('speed_lookahead_dist', _p(params, 'speed_lookahead_dist', 1.2), round(_p(params, 'speed_lookahead_dist', 1.2) + 0.2, 2), '전방 검사 구간 확대'),
                 ('profile_curv_window', _p(params, 'profile_curv_window', 0.3), round(_p(params, 'profile_curv_window', 0.3) + 0.1, 2), '곡률 추정 평활화')],
        'steer': [('k_prop', k_prop, round(max(0.6, k_prop - 0.1), 2), '이득 소폭 하향'),
                  ('heading_lookahead', la, round(min(0.8, la + 0.05), 2), '기준점 소폭 상향')],
        'speed': [('lateral_acc_limit', a_lat, round(min(0.8, a_lat + 0.05), 2), '코너 상한 소폭 상향'),
                  ('max_linear_vel', vmax, round(min(1.0, vmax + 0.05), 2), '상한 소폭 상향(다른 항목이 기준 안일 때만)')],
        'obstacle': [('block_check_dist', _p(params, 'block_check_dist', 4.5), round(min(4.8, _p(params, 'block_check_dist', 4.5) + 0.3), 2), '더 멀리서 판단'),
                     ('yield_stop_margin', _p(params, 'yield_stop_margin', 1.2), round(_p(params, 'yield_stop_margin', 1.2) + 0.2, 2), '대기 정지 여유')],
        'stop': [('min_linear_vel', _p(params, 'min_linear_vel', 0.02), 0.03, '데드밴드 극복'),
                 ('approach_dist', _p(params, 'approach_dist', 0.8), round(max(0.4, _p(params, 'approach_dist', 0.8) - 0.1), 2), '저속 접근 구간 단축')],
    }
    for key, sc_v in scores.items():
        if sc_v is None or sc_v >= tgt:
            continue
        if any(any(t.startswith(pfx) for pfx in covered.get(key, ())) for t in [f['title'] for f in F]):
            continue
        spec = ACCEPT.get(key)
        ev = f"{names.get(key, key)} {sc_v:.0f}점 (목표 {tgt:.0f})"
        if spec:
            ev += f" · {spec[0]} 기준 90점 {spec[1]} / 95점 {spec[2]}"
        F.append(dict(title=f'{names.get(key, key)} 미세 조정', severity=1 if sc_v >= tgt - 10 else 2, evidence=ev,
                      cause='규칙이 걸릴 만큼 나쁘지는 않지만 목표 점수에 미달합니다. 아래 값을 한 단계만 움직여 재측정하세요.',
                      changes=fine.get(key, []), effect=f'{names.get(key, key)} {tgt:.0f}점 이상',
                      caution='한 번에 하나만. 3점 이상 개선이 없으면 되돌리고 다음 항목으로'))

    # repeatability gate, code feedback, change budget
    for f in F:
        f['repeat'] = repeat_count(history, f['title']) + 1          # including this run
        f['code'] = CODE_FEEDBACK.get(f['title'], [])
        f['apply'] = f['severity'] == 3 or f['repeat'] >= 2 or f['title'].endswith('미세 조정') or f.get('direct', False)
    F.sort(key=lambda f: (-int(f['apply']), -f['severity'], -f['repeat']))
    budget = 1
    for f in F:
        if f['apply'] and budget > 0:
            f['label'] = '적용 권장'; budget -= 1
        elif f['apply']:
            f['label'] = '다음 차례'
        else:
            f['label'] = '관찰 (1회, 재현 시 권장)'
    return F



CODE_FEEDBACK = {
    '곡선에서 경로 바깥으로 밀림': [
        ('midpro_nav2_controller/src/midpro_control_loop.cpp', 'controlTick() 애커만 분기',
         'P제어에 횡오차 항을 추가합니다(Stanley 형). 기준점까지의 횡오차 e_y를 구해 angular.z = k_prop·Δyaw + v·κ + atan(k_e·e_y / max(v, 0.1)). '
         'k_e 0.5부터. 피드포워드(curvature_feedforward)를 켜도 곡선 이탈이 2회 이상 재현될 때만.'),
        ('midpro_nav2_controller/src/midpro_local_path.cpp', 'finalizeLocalPath()',
         '격자 경로의 접선 yaw를 3점 이동평균으로 다듬어 κ 추정 잡음을 줄입니다. 피드포워드 항의 떨림이 조향에 그대로 들어가는 것을 막습니다.')],
    '조향이 좌우로 진동': [
        ('midpro_nav2_controller/src/midpro_control_loop.cpp', 'controlTick() 조향 명령 출력부',
         'angular.z에 1차 저역 필터(τ≈0.1 s)와 변화율 제한(angular_acc_limit/controller_frequency)을 적용합니다. 현재는 linear.x만 rate limit이 있고 angular.z는 없습니다.'),
        ('midpro_nav2_controller/src/midpro_control_loop.cpp', 'heading_lookahead 적용부',
         '기준점 거리를 고정값이 아니라 속도 비례(L = 0.3 + 0.5·v)로 바꿉니다. 저속에서는 민감하게, 고속에서는 멀리 보게 됩니다.')],
    '속도 상한이 자주 바뀜(상한 깜빡임)': [
        ('midpro_nav2_controller/src/midpro_control_loop.cpp', 'speedGovernor()',
         '감속 판단을 프로파일 한 곳으로 통합합니다. 거버너에는 NEAR(95) 반응과 목표 제동만 남기고, 상한이 바뀔 때 즉시 적용하지 말고 1차 필터(τ 0.2 s)로 완화합니다.'),
        ('midpro_nav2_controller/src/midpro_local_path.cpp', 'buildSpeedProfile()',
         '장애물 근접 상한에 히스테리시스를 둡니다(진입 문턱 80, 해제 문턱 70). 셀 하나의 변화로 상한이 왕복하지 않게.')],
    '약 1 s 주기의 가속 스파이크 — 재계획 떨림': [
        ('midpro_nav2_controller/src/midpro_local_path.cpp', 'setPlan()',
         '경로 히스테리시스: 같은 goal의 재계획이 오면 기존 경로가 막혔거나 새 경로 길이가 5% 이상 짧을 때만 채택합니다. 로봇 근처 2 m는 기존 경로를 유지하고 그 뒤를 이어 붙입니다.')],
    '약 0.1 s 주기의 스파이크 — costmap 갱신에 따른 반응형 감속': [
        ('midpro_nav2_controller/src/midpro_control_loop.cpp', 'speedGovernor()',
         '전방 비용을 셀 단위 최댓값이 아니라 구간 평균(또는 상위 3셀 평균)으로 평가해 한 셀의 깜빡임을 무시합니다.')],
    'collision_monitor가 속도를 깎는 시간이 큼': [
        ('midpro_bringup/params (collision_monitor)', 'FootprintApproach / PolygonStop',
         '파라미터로 해결되지 않으면 midpro obstacle_filter의 방향별 감속(x±, y± 축만)을 커스텀 노드(C++ ~200줄)로 복원하고 approach 폴리곤은 전방 전용으로 한정합니다.')],
    '도착 정밀도 부족': [
        ('midpro_nav2_controller/src/midpro_control_loop.cpp', 'FINAL_APPROACH 분기',
         '마지막 10 cm에 위치 오차 적분항(I, 포화 0.05 m/s)을 추가해 데드밴드를 밀어 넣습니다. 오버슈트가 보이면 FINAL_APPROACH 진입 시 prev_linear_x_를 현재 속도로 초기화해 램프가 다시 시작되지 않게.'),
        ('midpro_nav2_controller/src/midpro_control_loop.cpp', 'FINAL_ROTATE 분기',
         '각속도에도 거리 비례 제동(√(2·α·Δyaw))을 적용하고 min_angular_vel 근처에서 멈추지 않도록 잔여각 < 2° 구간에 작은 피드포워드를 둡니다.')],
    '주행 중 제자리 회전 발생': [
        ('midpro_nav2_controller/src/midpro_control_loop.cpp', 'FOLLOW_PATH 홀로노믹 분기',
         'ROTATE_IN_PLACE 진입 조건(yaw_tolerance×6)에 지속 시간 조건(0.3 s 이상 초과)을 추가해 한 틱의 흔들림으로 회전 모드에 들어가지 않게 합니다. 진입/해제 임계를 파라미터로 분리.')],
    '주행 중 횡이동(vy)이 많음': [
        ('midpro_nav2_controller/src/midpro_local_path.cpp', 'finalizeLocalPath() 홀로노믹 판정',
         'is_oblique_ 판정에 히스테리시스(연속 N틱 동일 판정 후 전환)를 둡니다. 또는 holonomic_policy처럼 목표 근처에서만 vy를 허용하도록 midpro 법칙에도 goal_zone 게이트를 적용.')],
    '속도를 충분히 내지 못함': [
        ('midpro_nav2_controller/src/midpro_local_path.cpp', 'buildSpeedProfile()',
         '프로파일 계산 시 로컬 costmap 창(10 m) 밖은 "정보 없음"으로 두는데, 글로벌 costmap을 함께 조회하면 먼 구간의 근접 상한도 미리 반영됩니다(costmap_ros 두 개 접근).')],
    '가감속이 거침': [
        ('midpro_nav2_controller/src/midpro_control_loop.cpp', '출력 필터부',
         'linear.x rate limit을 가속/감속 비대칭(가속 0.4, 감속 0.6)으로 분리하고, velocity_smoother와 역할이 겹치지 않도록 한쪽만 남깁니다.')],
}

# ----------------------------------------------------------------------------- render
def _fmt(v):
    if isinstance(v, bool):
        return 'true' if v else 'false'
    if isinstance(v, float):
        return f'{v:.3g}'
    return str(v)


def render_html(st, findings, params, scores, score_names, history=None, target='B', conv=None, events=None):
    history = history or []
    events = events or []
    bs = st.get('block') or {}
    conv = conv or ('tuning', '', [])
    css = ('body{background:#14171c;color:#e6e9ee;font-family:"Noto Sans KR","Apple SD Gothic Neo",sans-serif;font-size:14px;line-height:1.6;margin:0}'
           'main{max-width:960px;margin:0 auto;padding:28px 24px 60px}h1{font-size:1.45rem;margin:0 0 .2rem}h2{font-size:1.1rem;border-top:1px solid #2b313a;padding-top:.8rem;margin:1.8rem 0 .5rem}'
           '.sub{color:#9aa3ae}table{border-collapse:collapse;width:100%;font-size:13px;margin:.4rem 0 .8rem}th{text-align:left;color:#9aa3ae;border-bottom:1px solid #2b313a;padding:.35rem .5rem}td{padding:.35rem .5rem;border-bottom:1px solid #1f242b;vertical-align:top}'
           '.f{border:1px solid #2b313a;border-left:3px solid #7fb0ec;padding:.6rem .9rem;margin:.7rem 0}.f h3{margin:0 0 .3rem;font-size:1rem}.s3{border-left-color:#e07a6a}.s2{border-left-color:#7fb0ec}.s1{border-left-color:#6b7480}'
           '.k{color:#9aa3ae;font-size:12.5px}.a{color:#7fb0ec;font-weight:700}code{background:#1b2027;padding:0 .3em;border-radius:3px}.bar{display:inline-block;height:8px;background:#7fb0ec;vertical-align:middle;border-radius:2px}.bg{display:inline-block;width:160px;height:8px;background:#1b2027;vertical-align:middle;border-radius:2px;margin-right:.5rem}')
    vals = [v for v in scores.values() if v is not None]
    total = sum(vals) / len(vals) if vals else 0
    grade = 'A' if total >= 85 else 'B' if total >= 70 else 'C' if total >= 50 else 'D'
    h = [f'<!DOCTYPE html><html lang="ko"><head><meta charset="utf-8"><title>midpro 튜닝 리포트</title><style>{css}</style></head><body><main>']
    h.append(f'<h1>주행 튜닝 리포트 <span class="a">{total:.0f}점 · {grade}</span></h1><p class="sub">{time.strftime("%Y-%m-%d %H:%M")} · 주행 {st["duration"]:.1f} s · {st["distance"]:.1f} m · 상한 {st["vmax"]} m/s · {"도착 완료" if st["frozen"] else "주행 중 스냅샷"}</p>')
    status, msg, locked = conv
    if status == 'done':
        h.append(f'<div class="f" style="border-left-color:#5fb77d"><h3>튜닝 완료</h3><p>{msg}</p></div>')
    elif status == 'converged':
        h.append(f'<div class="f" style="border-left-color:#e0b35a"><h3>수렴</h3><p>{msg}</p></div>')
    elif status == 'worse':
        h.append(f'<div class="f s3"><h3>되돌리기</h3><p>{msg}</p></div>')
    if locked:
        h.append('<p class="k">고정 권장(최근 4회에서 올렸다 내렸다 반복): ' + ', '.join(f'<code>{k.replace("FollowPath.","")}</code>' for k in locked) + '</p>')
    if len(history) >= 2:
        h.append('<h2>이력</h2><table><tr><th>시각</th><th>종합</th><th>변경한 파라미터</th><th>진단</th></tr>')
        for hrun in history[-6:]:
            ch = ', '.join(f'{k.replace("FollowPath.","")} {_fmt(a)}→{_fmt(b)}' for k, (a, b) in hrun.get('changed', {}).items()) or '-'
            h.append(f"<tr><td>{hrun['time']}</td><td class='a'>{hrun['total']:.0f}</td><td class='k'>{ch}</td><td class='k'>{', '.join(hrun.get('findings', [])) or '-'}</td></tr>")
        h.append('</table>')
    h.append('<h2>점수</h2><table><tr><th>항목</th><th>점수</th><th>측정값</th></tr>')
    raw = {'track': f"{(st.get('xte_rms') or 0)*100:.1f} cm (직선 {((st.get('xte_straight_rms') or 0)*100):.1f} / 곡선 {((st.get('xte_curve_rms') or 0)*100):.1f})",
           'accel': f"{(st.get('acc_rms') or 0):.2f} m/s² (최대 {st.get('acc_max',0):.2f})", 'jerk': f"{(st.get('jerk_rms') or 0):.2f} m/s³",
           'steer': f"{st.get('rev_per_min',0):.1f} /분 (직선 wz RMS {(st.get('wz_straight_rms') or 0):.2f})", 'speed': f"{st.get('v_ratio',0)*100:.0f} % (상한 유지 {st.get('t_at_vmax',0)*100:.0f}%)",
           'stop': f"{st['final_err'][0]*100:.1f} cm · {st['final_err'][1]:.1f}°" if st.get('final_err') else '주행 중',
           'obstacle': (f"감지 {bs.get('detect', 0)} · 판단 최소 {bs['decision_min']:.1f} m" if bs.get('decision_min') is not None else (f"감지 {bs.get('detect', 0)}" if bs.get('detect') else '장애물 없음'))}
    for k, name in score_names:
        v = scores.get(k)
        bar = f'<span class="bg"><span class="bar" style="width:{(v or 0)*1.6:.0f}px"></span></span>' if v is not None else ''
        h.append(f'<tr><td>{name}</td><td>{bar}{"" if v is None else f"{v:.0f}"}</td><td class="k">{raw.get(k,"-")}</td></tr>')
    h.append('</table>')
    bs = st.get('block') or {}
    if bs.get('detect'):
        h.append('<h2>장애물 대응</h2><p class="k">컨트롤러의 막힘 판단(감지 → 고정물/이동체 분류 → 우회 또는 대기)을 이벤트 단위로 기록한 것입니다.</p>')
        h.append('<table><tr><th>지표</th><th>값</th><th>기준</th></tr>')
        rows_b = [('막힘 감지', f"{bs['detect']}회", ''),
                  ('고정물로 바로 우회', f"{len(bs['static_at'])}회" + (f" · 결정 거리 {', '.join(f'{a:.1f}' for a in bs['static_at'])} m" if bs['static_at'] else ''), '결정 거리 ≥ 2.5 m'),
                  ('이동체 판정', f"{bs['moving']}회", ''),
                  ('대기 후 우회', f"{len(bs['persistent_age'])}회", '0회가 이상적 (고정물을 이동체로 오판한 경우)'),
                  ('비켜서 해소', f"{len(bs['clear_age'])}회" + (f" · 대기 {', '.join(f'{a:.1f}' for a in bs['clear_age'])} s" if bs['clear_age'] else ''), '대기 없이 해소될수록 좋음'),
                  ('대안 없음 실패', f"{bs['fail']}회", '0회'),
                  ('최소 차체 여유', '-' if bs.get('min_clear') is None else f"{bs['min_clear']*100:.0f} cm", '≥ 10 cm')]
        for a, b_, c_ in rows_b:
            h.append(f'<tr><td>{a}</td><td class="a">{b_}</td><td class="k">{c_}</td></tr>')
        h.append('</table>')
        evname = {'detect': '막힘 감지', 'moving': '이동체 판정', 'switch': '우회 전환', 'clear': '해소', 'fail': '실패'}
        h.append('<table><tr><th>주행 후 s</th><th>이벤트</th><th>거리 m</th><th>대기 s</th><th>구분</th></tr>')
        for (tt, e) in st.get('block_events', [])[-40:]:
            why = {'static': '고정물', 'persistent': '지속 막힘', 'moving': '이동체', 'unclassified': '분류 전'}.get(e.get('why', ''), e.get('why', ''))
            h.append(f"<tr><td>{tt}</td><td>{evname.get(e.get('ev'), e.get('ev'))}</td><td>{e.get('at', '')}</td><td>{e.get('age', '')}</td><td class='k'>{why}</td></tr>")
        h.append('</table>')
    h.append('<h2>증거 요약</h2><table><tr><th>지표</th><th>값</th><th>해석</th></tr>')
    rows = [('램프 기울기', f"{st['ramp_slope']:.2f} m/s²" if st.get('ramp_slope') else '-', '설정 linear_acc_limit와 비교'),
            ('명령→실제 지연', f"{st['lag']:.2f} s" if st.get('lag') else '-', '0.25 s 이상이면 조향 정렬·필터 지연 의심'),
            ('스파이크 주기', f"{st['spike_period']:.2f} s" if st.get('spike_period') else '-', '1 s 재계획 / 0.5 s Hybrid / 0.1 s costmap'),
            ('제한 사유 점유', ', '.join(f'{k} {v*100:.0f}%' for k, v in st.get('reason_share', {}).items()) or '-', '어느 제한이 주행을 지배했나'),
            ('사유 전환', f"{st.get('reason_changes_per_min',0):.0f} /분", '20 이상이면 상한 깜빡임'),
            ('collision_monitor 감속 시간', f"{st.get('cm_cut_frac',0)*100:.0f} %", '15% 이상이면 접근 제동·정지 구역 점검'),
            ('프로파일 대비 실제 격차', f"{st['profile_gap']:.2f} m/s" if st.get('profile_gap') is not None else '-', '0.1 이상이면 구동부 또는 추가 감속'),
            ('재계획 횟수', str(st.get('replans', 0)), '1회가 정상'),
            ('주행 중 제자리 회전', f"{st.get('rotate_transit',0)}회 (전체 {st.get('rotate_total',0)})", '0회가 정상'),
            ('detour 비율', f"{st.get('detour_share',0)*100:.0f} %", '먼 목표는 높아도 정상'),
            ('횡이동 시간 비율', f"{st.get('vy_transit_frac',0)*100:.0f} %", '직선 코스에서 20% 이상이면 판정 흔들림'),
            ('FSM 상태 시간', ', '.join(f'{k} {v:.1f}s' for k, v in st.get('state_time', {}).items()) or '-', '')]
    for a, b, c in rows:
        h.append(f'<tr><td>{a}</td><td>{b}</td><td class="k">{c}</td></tr>')
    h.append('</table>')
    tgt = target_score(target)
    h.append('<h2>합격 기준 (절대값)</h2><p class="k">점수는 아래 측정값을 0~100으로 환산한 것일 뿐입니다. 합격 판단은 이 절대값으로 하세요.</p><table><tr><th>항목</th><th>이번 값</th><th>90점 상당</th><th>95점 상당</th><th>판정</th></tr>')
    meas = {'track': (st.get('xte_rms') or 0) * 100, 'accel': st.get('acc_rms') or 0, 'jerk': st.get('jerk_rms') or 0,
            'steer': st.get('rev_per_min') or 0, 'speed': (st.get('v_ratio') or 0) * 100,
            'stop': (st['final_err'][0] * 100 if st.get('final_err') else None),
            'obstacle': (None if scores.get('obstacle') is None else 100 - scores['obstacle'])}
    for k, (label, s90, s95, unit) in ACCEPT.items():
        v = meas.get(k); sc_v = scores.get(k)
        if v is None or (k in ('accel', 'jerk') and (st.get('acc_rms') is None)):
            ok = '데이터 없음'
        else:
            ok = '통과' if (sc_v is not None and sc_v >= tgt) else '미달'
        vs = '-' if v is None else (f'{v:.1f} {unit}'.strip() + (f" · {st['final_err'][1]:.1f}°" if k == 'stop' and st.get('final_err') else ''))
        h.append(f"<tr><td>{label}</td><td class='a'>{vs}</td><td class='k'>{s90}</td><td class='k'>{s95}</td><td>{ok}</td></tr>")
    h.append('</table>')
    h.append(f'<h2>진단과 조정 제안</h2><p class="k">목표 {tgt:.0f}점 미만인 항목에는 항상 제안이 나옵니다. 규칙에 걸린 항목 중 1회만 나온 것은 \'관찰\'로 두고, 적용은 한 번에 하나씩 하세요. 목표에 도달하고 모든 항목이 기준 근처면 조정을 멈추고 고정합니다.</p>')
    if status in ('done',):
        findings = [f for f in findings if f['severity'] == 3]
    if not findings:
        h.append(f'<p>모든 항목이 목표 {tgt:.0f}점 이상입니다. 같은 코스를 2회 더 돌려 재현성을 확인한 뒤 값을 고정하세요.</p>')
    for i, f in enumerate(findings, 1):
        tag = f.get('label', '')
        rep = f"재현 {f.get('repeat',1)}/3회" if history else ''
        h.append(f'<div class="f s{f["severity"]}"><h3>{i}. {f["title"]} <span class="k">— {tag} · {rep}</span></h3><div class="k">증거: {f["evidence"]}</div><p>{f["cause"]}</p>')
        if f['changes']:
            h.append('<table><tr><th>파라미터</th><th>현재</th><th>제안</th><th>이유</th></tr>')
            for (name, cur, new, why) in f['changes']:
                h.append(f'<tr><td><code>{name}</code></td><td>{_fmt(cur)}</td><td class="a">{_fmt(new)}</td><td class="k">{why}</td></tr>')
            h.append('</table>')
        h.append(f'<div class="k">기대 효과: {f["effect"]} · 주의: {f["caution"]}</div>')
        if f.get('code') and (f.get('repeat', 1) >= 2 or f['severity'] == 3):
            h.append('<details><summary class="k" style="cursor:pointer">코드 피드백 — 파라미터로 안 잡힐 때</summary><table><tr><th>파일</th><th>함수/위치</th><th>수정 내용</th></tr>')
            for (fp, fn, what) in f['code']:
                h.append(f'<tr><td><code>{fp}</code></td><td>{fn}</td><td class="k">{what}</td></tr>')
            h.append('</table></details>')
        h.append('</div>')
    if events:
        h.append('<h2>실시간 이벤트 타임라인</h2><p class="k">주행 중 발생 즉시 기록된 항목입니다(같은 종류는 10 s 간격). 위 진단은 이 이벤트들의 누적 결과입니다.</p><table><tr><th>시각</th><th>심각도</th><th>현상</th><th>권장 파라미터</th><th>이유</th></tr>')
        for ev in events[-60:]:
            param = f"<code>{ev.param}</code> {_fmt(ev.cur) if ev.cur is not None else ''} → <span class='a'>{_fmt(ev.new) if ev.new is not None else ''}</span>" if ev.param else '-'
            h.append(f"<tr><td>{ev.wall}</td><td>{ {3:'높음',2:'중간',1:'낮음'}[ev.sev] }</td><td>{ev.msg}</td><td>{param}</td><td class='k'>{ev.why}</td></tr>")
        h.append('</table>')
    h.append('<h2>이번에 할 일</h2><ol>')
    todo = [f for f in findings if f.get('label') == '적용 권장' and f['changes']]
    if status == 'done':
        h.append('<li>조정 없음. 현재 yaml을 기준선으로 저장하고 회귀 시나리오(직진·코너·장애물)를 3회씩 돌려 재현성을 확인합니다.</li>')
    elif status == 'converged' and not todo:
        h.append('<li>파라미터 조정 중단. 남은 항목은 코드 피드백 항목을 하나 골라 구현 → 같은 코스 3회 재측정.</li>')
    elif todo:
        f = todo[0]
        h.append(f'<li><b>{f["title"]}</b>: <code>{f["changes"][0][0]}</code> {_fmt(f["changes"][0][1])} → {_fmt(f["changes"][0][2])} 하나만 바꾸고 같은 코스 3회 재측정</li>')
        h.append('<li>점수가 3점 이상 오르지 않으면 되돌리고, 그 항목의 코드 피드백으로 넘어갑니다.</li>')
    else:
        h.append('<li>적용 권장 항목 없음(관찰만). 같은 코스를 한 번 더 돌려 재현 여부를 확인합니다.</li>')
    h.append('</ol><p class="k">yaml만 바꾸는 항목은 재빌드 없이 launch 재시작이면 됩니다. 두 개 이상 동시에 바꾸면 어느 것이 효과였는지 알 수 없습니다.</p>')
    h.append('<h2>현재 파라미터 (controller_server)</h2><table><tr><th>이름</th><th>값</th></tr>')
    for k in PARAM_NAMES:
        if k in params:
            h.append(f'<tr><td><code>{k.replace("FollowPath.","")}</code></td><td>{_fmt(params[k])}</td></tr>')
    h.append('</table></main></body></html>')
    return ''.join(h)

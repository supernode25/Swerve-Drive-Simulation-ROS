"""metrics.py — metrics and scores for one regression run (no ROS imports, unit-testable).

Inputs are plain lists recorded by the runner:
  odom   [(t, v, vx, vy, wz)]        cmd / cmd_nav [(t, vx, vy, wz)]
  gt     [(t, x, y, yaw)]            plan [(x, y)] (first global plan after the goal)
  trans  [(t, text)] ctrl_transition reasons [(t, text)] speed_limit
"""
import math

SCORE_BANDS = {           # (100-point value, 0-point value) — same as the dashboard score card
    'track': (0.05, 0.30), 'accel': (0.10, 0.50), 'jerk': (0.5, 3.0),
    'steer': (2.0, 20.0), 'speed': (0.90, 0.30),
}


def lin_score(x, good, bad):
    if x is None:
        return None
    if good < bad:
        return 100.0 if x <= good else (0.0 if x >= bad else 100.0 * (bad - x) / (bad - good))
    return 100.0 if x >= good else (0.0 if x <= bad else 100.0 * (x - bad) / (good - bad))


def rms(xs):
    return math.sqrt(sum(v * v for v in xs) / len(xs)) if xs else None


def wrap(a):
    return (a + math.pi) % (2.0 * math.pi) - math.pi


def accel_jerk(odom):
    """First-order filtered speed -> acceleration -> jerk (same filter as the dashboard)."""
    acc, jerk = [], []
    v_f = a_f = None
    t_prev = None
    for (t, v, *_rest) in odom:
        if t_prev is None:
            v_f, a_f, t_prev = v, 0.0, t
            continue
        dt = max(t - t_prev, 1e-3)
        alpha = min(1.0, dt / 0.15)
        nv = v_f + alpha * (v - v_f)
        a = (nv - v_f) / dt
        na = a_f + alpha * (a - a_f)
        acc.append(na)
        jerk.append((na - a_f) / dt)
        v_f, a_f, t_prev = nv, na, t
    return acc, jerk


def steering_reversals(cmd, thr=0.05):
    rev, prev = 0, 0.0
    for (_t, _vx, _vy, wz) in cmd:
        if wz * prev < 0 and abs(wz) > thr:
            rev += 1
        if abs(wz) > thr:
            prev = wz
    return rev


def cross_track(gt, plan, moving_times=None):
    if not plan or not gt:
        return None
    out = []
    hint = 0
    for (t, x, y, _yaw) in gt:
        if moving_times is not None and t not in moving_times:
            continue
        best, bi = 1e18, hint
        lo = max(0, hint - 60)
        for i in range(lo, len(plan)):
            d = (plan[i][0] - x) ** 2 + (plan[i][1] - y) ** 2
            if d < best:
                best, bi = d, i
            if i > hint + 400 and d > best + 4.0:
                break
        hint = bi
        out.append(math.sqrt(best))
    return rms(out)


def stuck_time(gt, window, dist):
    """Longest interval (s) during which the robot stayed within `dist` of a point for at least `window`."""
    worst = 0.0
    j = 0
    for i in range(len(gt)):
        while j < i and gt[i][0] - gt[j][0] > window:
            j += 1
        if gt[i][0] - gt[j][0] >= window * 0.95:
            seg = gt[j:i + 1]
            xs = [p[1] for p in seg]; ys = [p[2] for p in seg]
            if max(xs) - min(xs) < dist and max(ys) - min(ys) < dist:
                worst = max(worst, gt[i][0] - gt[j][0])
    return worst


def gap_passed(gt, gap):
    """True if the robot crossed x = gap.x within the gap opening."""
    if not gap or len(gt) < 2:
        return None
    gx, gy, hw = gap['x'], gap['y'], gap['half_width']
    for (a, b) in zip(gt, gt[1:]):
        if (a[1] - gx) * (b[1] - gx) <= 0 and abs(b[2] - gy) <= hw:
            return True
    return False


def footprint_clearance(ranges, angle_min, angle_inc, laser, fp):
    """Min distance (m) from scan points to the robot footprint rectangle (negative = inside = contact).
    laser = (x, y, yaw) of the laser in base_link; fp = (front, rear, half_width), rear negative."""
    lx, ly, lyaw = laser
    front, rear, hw = fp
    best = 1e9
    c, s_ = math.cos(lyaw), math.sin(lyaw)
    for i, r in enumerate(ranges):
        if not (r > 0.0) or math.isinf(r) or math.isnan(r):
            continue
        a = angle_min + i * angle_inc
        px0, py0 = r * math.cos(a), r * math.sin(a)
        px, py = lx + c * px0 - s_ * py0, ly + s_ * px0 + c * py0
        dx = max(px - front, rear - px, 0.0)
        dy = max(abs(py) - hw, 0.0)
        if dx == 0.0 and dy == 0.0:
            d = -min(front - px, px - rear, hw - abs(py))
        else:
            d = math.hypot(dx, dy)
        best = min(best, d)
    return best


def episodes(items, prefix):
    """Count separate episodes of a speed_limit reason (consecutive messages = one episode)."""
    n, active = 0, False
    for (_t, txt) in items:
        hit = prefix in txt
        if hit and not active:
            n += 1
        active = hit
    return n


def block_stats(events, clear=None):
    """Summarise /midpro/block_event events [(t, dict)] and footprint clearances [(t, m)]."""
    st = {'detect': 0, 'moving': 0, 'static_at': [], 'persistent_age': [], 'clear_age': [], 'fail': 0, 'fail_why': []}
    for (_t, e) in events:
        ev = e.get('ev')
        if ev == 'detect':
            st['detect'] += 1
        elif ev == 'moving':
            st['moving'] += 1
        elif ev == 'switch' and e.get('why') == 'static':
            st['static_at'].append(float(e.get('at', 0.0)))
        elif ev == 'switch':
            st['persistent_age'].append(float(e.get('age', 0.0)))
        elif ev == 'clear':
            st['clear_age'].append(float(e.get('age', 0.0)))
        elif ev == 'fail':
            st['fail'] += 1; st['fail_why'].append(e.get('why', ''))
    cl = [c for (_t, c) in (clear or [])]
    st['min_clear'] = min(cl) if cl else None
    st['decision_min'] = min(st['static_at']) if st['static_at'] else None
    return st


def obstacle_score(bs):
    """0..100 for obstacle handling; None when no blockage happened."""
    if not bs or bs['detect'] == 0:
        return None
    pen = 0.0
    for at in bs['static_at']:
        if at < 2.5:
            pen += min(25.0, 15.0 * (2.5 - at) / 1.0 + 5.0)          # decided late, close to the obstacle
    pen += 10.0 * len(bs['persistent_age'])                          # waited block_persist_time, then went around
    pen += 30.0 * bs['fail']                                         # no alternative: FollowPath failed
    pen += 5.0 * sum(1 for a in bs['clear_age'] if a > 2.5)          # long waits
    mc = bs.get('min_clear')
    if mc is not None:
        pen += 25.0 if mc < 0.05 else (10.0 if mc < 0.15 else 0.0)
    return max(0.0, 100.0 - pen)


def compute(scn, rec, vmax, stuck_window=10.0, stuck_dist=0.05):
    odom, cmd, cmdn, gt = rec['odom'], rec['cmd'], rec['cmd_nav'], rec['gt']
    out = {'scenario': scn['name'], 'result': rec['result'], 'success': rec['result'] == 'SUCCEEDED',
           'time_s': round(rec['duration'], 2)}
    out['dist_m'] = round(sum(math.hypot(b[1] - a[1], b[2] - a[2]) for a, b in zip(gt, gt[1:])), 2) if gt else 0.0
    moving = {t for (t, v, *_r) in odom if v > 0.05}
    xte = cross_track(gt, rec['plan'], None)
    acc, jerk = accel_jerk(odom)
    out['xte_rms_cm'] = None if xte is None else round(xte * 100.0, 1)
    out['acc_rms'] = None if not acc else round(rms(acc), 3)
    out['jerk_rms'] = None if not jerk else round(rms(jerk), 2)
    minutes = max(rec['duration'] / 60.0, 0.25)
    out['steer_rev_per_min'] = round(steering_reversals(cmd) / minutes, 1)
    vs = [v for (t, v, *_r) in odom if v > 0.05]
    out['v_ratio'] = round((sum(vs) / len(vs)) / vmax, 3) if vs and vmax > 0 else 0.0
    if gt:
        gx, gy, gyaw = rec['goal']
        out['final_pos_err_cm'] = round(math.hypot(gt[-1][1] - gx, gt[-1][2] - gy) * 100.0, 1)
        out['final_yaw_err_deg'] = round(abs(math.degrees(wrap(gt[-1][3] - gyaw))), 1)
    else:
        out['final_pos_err_cm'] = out['final_yaw_err_deg'] = None
    trans = rec['trans']
    out['rotate_total'] = sum(1 for (_t, s) in trans if '-> ROTATE_IN_PLACE' in s)
    out['rotate_transit'] = sum(1 for (_t, s) in trans if s.startswith('FOLLOW_PATH') and '-> ROTATE_IN_PLACE' in s)
    out['traj_safety_ep'] = episodes(rec['reasons'], 'traj_safety')
    out['escape_ep'] = episodes(rec['reasons'], 'escape')
    out['align_ep'] = episodes(rec['reasons'], 'align')
    out['crab_ep'] = episodes(rec['reasons'], 'crab_no_room')
    out['boxed_ep'] = episodes(rec['reasons'], 'boxed_in')
    out['yield_ep'] = episodes(rec['reasons'], 'yield')
    out['back_off_ep'] = episodes(rec['reasons'], 'back_off')
    out['orth_ep'] = episodes(rec['reasons'], 'orth_side')
    cl = [c for (_t, c) in rec.get('clear', [])]
    out['min_clear_cm'] = round(min(cl) * 100.0, 1) if cl else None
    bs = block_stats(rec.get('block', []), rec.get('clear', []))
    out['block_detect'] = bs['detect']
    out['decision_min_m'] = None if bs['decision_min'] is None else round(bs['decision_min'], 2)
    out['wait_switch'] = len(bs['persistent_age'])
    out['block_fail'] = bs['fail']
    sx, sy = (scn['start'][0], scn['start'][1]) if scn.get('start') else ((gt[0][1], gt[0][2]) if gt else (rec['goal'][0], rec['goal'][1]))
    ref = float(scn.get('ref_len', math.hypot(rec['goal'][0] - sx, rec['goal'][1] - sy)))
    out['detour_ratio'] = round(out['dist_m'] / ref, 2) if ref > 0.1 and out['dist_m'] > 0 else None
    cut = 0
    j = 0
    for r in cmd:
        while j + 1 < len(cmdn) and cmdn[j + 1][0] <= r[0]:
            j += 1
        if cmdn and cmdn[j][1] > 0.1 and r[1] < 0.85 * cmdn[j][1]:
            cut += 1
    out['cm_cut_frac'] = round(cut / len(cmd), 3) if cmd else 0.0
    out['stuck_s'] = round(stuck_time(gt, stuck_window, stuck_dist), 1)
    out['wedged'] = bool(out['stuck_s'] >= stuck_window * 0.95 or out['boxed_ep'] > 0 or
                         (out['min_clear_cm'] is not None and out['min_clear_cm'] < 1.0 and not out['success']))
    gp = gap_passed(gt, scn.get('gap'))
    out['gap_passed'] = gp
    # scores
    sc = {
        'track': lin_score(xte, *SCORE_BANDS['track']),
        'accel': lin_score(rms(acc) if acc else None, *SCORE_BANDS['accel']),
        'jerk': lin_score(rms(jerk) if jerk else None, *SCORE_BANDS['jerk']),
        'steer': lin_score(out['steer_rev_per_min'], *SCORE_BANDS['steer']),
        'speed': lin_score(out['v_ratio'], *SCORE_BANDS['speed']),
        'stop': None,
        'obstacle': obstacle_score(bs),
    }
    if out['success'] and out['final_pos_err_cm'] is not None:
        sc['stop'] = 0.5 * lin_score(out['final_pos_err_cm'] / 100.0, 0.02, 0.10) + 0.5 * lin_score(out['final_yaw_err_deg'], 1.4, 5.0)
    for k, v in sc.items():
        out['score_' + k] = None if v is None else round(v, 1)
    vals = [v for v in sc.values() if v is not None]
    out['score_total'] = round(sum(vals) / len(vals), 1) if vals else 0.0
    out['score_min_item'] = round(min(vals), 1) if vals else 0.0
    # a run passes only if it arrives, averages >= 70, no single item collapses (< 40), never wedges,
    # and (for gap scenarios) actually goes through the gap
    max_ratio = scn.get('max_detour_ratio')
    detour_ok = max_ratio is None or (out['detour_ratio'] is not None and out['detour_ratio'] <= float(max_ratio))
    out['detour_ok'] = detour_ok
    out['passed'] = bool(out['success'] and out['score_total'] >= 70.0 and out['score_min_item'] >= 40.0
                         and not out['wedged'] and (gp is not False) and detour_ok)
    return out


def summarize(rows):
    """Per-scenario aggregate: pass rate, mean/min total, means of key metrics."""
    by = {}
    for r in rows:
        by.setdefault(r['scenario'], []).append(r)
    summ = {}
    for name, rs in by.items():
        def mean(key):
            xs = [r[key] for r in rs if isinstance(r.get(key), (int, float)) and not isinstance(r.get(key), bool)]
            return round(sum(xs) / len(xs), 2) if xs else None
        summ[name] = {
            'runs': len(rs), 'pass_rate': round(sum(1 for r in rs if r['passed']) / len(rs), 2),
            'success_rate': round(sum(1 for r in rs if r['success']) / len(rs), 2),
            'score_mean': mean('score_total'), 'score_min': min(r['score_total'] for r in rs),
            'time_mean': mean('time_s'), 'xte_mean_cm': mean('xte_rms_cm'), 'acc_rms_mean': mean('acc_rms'),
            'jerk_rms_mean': mean('jerk_rms'), 'steer_rev_mean': mean('steer_rev_per_min'),
            'rotate_transit_mean': mean('rotate_transit'), 'stuck_runs': sum(1 for r in rs if r['stuck_s'] > 0),
            'traj_safety_mean': mean('traj_safety_ep'), 'gap_passed': None if rs[0].get('gap_passed') is None else
            sum(1 for r in rs if r.get('gap_passed')) / len(rs),
            'final_pos_err_mean_cm': mean('final_pos_err_cm'),
            'wedged_runs': sum(1 for r in rs if r.get('wedged')),
            'min_clear_cm': min((r['min_clear_cm'] for r in rs if r.get('min_clear_cm') is not None), default=None),
            'detour_ratio_mean': mean('detour_ratio'),
            'detour_ok_rate': round(sum(1 for r in rs if r.get('detour_ok', True)) / len(rs), 2),
            'decision_min_m': min((r['decision_min_m'] for r in rs if r.get('decision_min_m') is not None), default=None),
            'wait_switch_total': sum(r.get('wait_switch', 0) for r in rs),
            'block_fail_total': sum(r.get('block_fail', 0) for r in rs),
            'obstacle_score_mean': mean('score_obstacle'),
        }
    allpass = [s['pass_rate'] for s in summ.values()]
    gate = {
        # G1: every scenario passes in at least 2 of 3 runs and never drops below 60
        'all_scenarios_B': all(s['pass_rate'] >= 0.66 and s['score_min'] >= 60 for s in summ.values()),
        'pass_rate_mean': round(sum(allpass) / len(allpass), 2) if allpass else 0.0,
    }
    return summ, gate

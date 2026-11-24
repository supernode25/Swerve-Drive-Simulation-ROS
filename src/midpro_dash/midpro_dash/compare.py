#!/usr/bin/env python3
"""compare.py — compare regression summaries (baseline first).

    ros2 run midpro_dash compare_regression ~/midpro_reports/regression/v028_* ~/midpro_reports/regression/v028_narrow_*
Writes compare.md next to the last summary and prints it.
"""
import json
import os
import sys

KEYS = [('pass_rate', '통과율', 1, 0.1), ('score_mean', '점수', 1, 3.0), ('time_mean', '시간', -1, 2.0),
        ('xte_mean_cm', '이탈cm', -1, 1.0), ('jerk_rms_mean', '저크', -1, 0.2), ('steer_rev_mean', '조향반전', -1, 1.0),
        ('rotate_transit_mean', '주행중회전', -1, 0.5), ('wedged_runs', '끼임', -1, 0.5), ('min_clear_cm', '최소여유cm', 1, 1.0),
        ('detour_ratio_mean', '우회비율', -1, 0.05)]


def load(d):
    p = d if d.endswith('.json') else os.path.join(d, 'summary.json')
    with open(p, encoding='utf-8') as f:
        return json.load(f)


def verdict(base, cur):
    better = worse = 0
    for key, _n, sign, tol in KEYS:
        a, b = base.get(key), cur.get(key)
        if a is None or b is None:
            continue
        d = (b - a) * sign
        if d > tol:
            better += 1
        elif d < -tol:
            worse += 1
    return '개선' if better > worse else ('악화' if worse > better else '비슷')


def main():
    dirs = [a for a in sys.argv[1:] if not a.startswith('-')]
    if len(dirs) < 2:
        print(__doc__); return
    runs = [load(d) for d in dirs]
    base = runs[0]
    lines = [f"# 비교 — 기준: {base['label']}", '']
    for r in runs[1:]:
        lines += [f"## {r['label']} vs {base['label']}", '', '| 시나리오 | ' + ' | '.join(n for _k, n, _s, _t in KEYS) + ' | 판정 |',
                  '|---|' + '---|' * (len(KEYS) + 1)]
        for name, b in base['summary'].items():
            c = r['summary'].get(name)
            if not c:
                continue
            cells = []
            for key, _n, sign, tol in KEYS:
                a, v = b.get(key), c.get(key)
                if a is None or v is None:
                    cells.append('-'); continue
                d = v - a
                mark = '' if abs(d) <= tol else (' ▲' if d * sign > 0 else ' ▼')
                cells.append(f"{a}→{v}{mark}")
            gp = '' if c.get('gap_passed') is None else f" (틈 통과 {b.get('gap_passed', 0)*100:.0f}%→{c['gap_passed']*100:.0f}%)"
            lines.append(f"| {name}{gp} | " + ' | '.join(cells) + f" | {verdict(b, c)} |")
        lines += ['', f"G1(전 시나리오 2/3 통과, 최저 ≥ 60): {base['label']} {'통과' if base['gate']['all_scenarios_B'] else '미달'} → "
                      f"{r['label']} {'통과' if r['gate']['all_scenarios_B'] else '미달'}", '']
    text = '\n'.join(lines)
    out = os.path.join(dirs[-1] if os.path.isdir(dirs[-1]) else os.path.dirname(dirs[-1]), 'compare.md')
    with open(out, 'w', encoding='utf-8') as f:
        f.write(text + '\n')
    print(text); print(f'\n[compare] saved {out}')


if __name__ == '__main__':
    main()

#!/usr/bin/env python3
"""run_variants.py — bisect the footprint features: launch each variant headless, run the regression, compare.

    ros2 run midpro_dash run_variants                    # all variants, repeats from the scenario file
    ros2 run midpro_dash run_variants --repeats 1 --variants v028,v028_narrow,narrow_no_align

Variants (generated from the installed parameter files):
  v028              baseline (circle model)
  v028_narrow       footprint minimum set (all on)
  narrow_no_align   v028_narrow with align_gate_enabled false
  narrow_no_rotchk  v028_narrow with rotation_check false
  narrow_no_latch   v028_narrow with escape_latch false (escape whenever inside NEAR, as v028)
  narrow_no_tight   v028_narrow with tight_passage_speed = max_linear_vel (no tight-passage slow-down)
  narrow_res05      v028_narrow with local costmap resolution 0.05
  current           latest profile (v0.33 behaviour)
Each variant needs ~ (scenarios x repeats x ~1 min). Results: ~/midpro_reports/regression/variants_<time>/
"""
import argparse
import glob
import os
import signal
import subprocess
import sys
import time

import yaml

VARIANTS = {
    'v028': ('nav2_params_v028.yaml', 'v028', {}),
    'v028_narrow': ('nav2_params_v028_narrow.yaml', 'v028_narrow', {}),
    'narrow_no_align': ('nav2_params_v028_narrow.yaml', 'v028_narrow', {('controller_server', 'FollowPath', 'align_gate_enabled'): False}),
    'narrow_no_rotchk': ('nav2_params_v028_narrow.yaml', 'v028_narrow', {('controller_server', 'FollowPath', 'rotation_check'): False}),
    'narrow_no_latch': ('nav2_params_v028_narrow.yaml', 'v028_narrow', {('controller_server', 'FollowPath', 'escape_latch'): False}),
    'narrow_no_tight': ('nav2_params_v028_narrow.yaml', 'v028_narrow', {('controller_server', 'FollowPath', 'tight_passage_speed'): 0.6}),
    'narrow_res05': ('nav2_params_v028_narrow.yaml', 'v028_narrow', {('local_costmap', 'local_costmap', 'resolution'): 0.05}),
    'current': ('nav2_params.yaml', 'current', {}),
}


def set_param(doc, path, value):
    node = path[0]
    if node in ('controller_server',):
        d = doc[node]['ros__parameters']
        for k in path[1:-1]:
            d = d[k]
        d[path[-1]] = value
    else:   # costmaps: local_costmap: local_costmap: ros__parameters
        d = doc[path[0]][path[1]]['ros__parameters']
        d[path[2]] = value


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--variants', default=','.join(VARIANTS))
    ap.add_argument('--repeats', type=int, default=None)
    ap.add_argument('--only', default='')
    ap.add_argument('--group', default='', help='base | wedge | detour')
    ap.add_argument('--model', default='swerve')
    args = ap.parse_args()
    from ament_index_python.packages import get_package_share_directory
    share = get_package_share_directory('midpro_bringup')
    stamp = time.strftime('%Y%m%d_%H%M%S')
    root = os.path.expanduser(f'~/midpro_reports/regression/variants_{stamp}')
    gen = os.path.join(root, 'params'); os.makedirs(gen, exist_ok=True)
    results = []
    for name in [v.strip() for v in args.variants.split(',') if v.strip()]:
        base, profile, overrides = VARIANTS[name]
        doc = yaml.safe_load(open(os.path.join(share, 'params', base), encoding='utf-8'))
        for path, val in overrides.items():
            set_param(doc, path, val)
        pf = os.path.join(gen, f'{name}.yaml')
        with open(pf, 'w', encoding='utf-8') as f:
            yaml.safe_dump(doc, f, sort_keys=False, allow_unicode=True)
        print(f'\n===== {name} ({profile}) =====')
        launch = subprocess.Popen(['ros2', 'launch', 'midpro_bringup', 'sim_nav.launch.py', 'localization:=gt', f'model:={args.model}',
                                   'rviz:=false', 'dash:=false', f'profile:={profile}', f'params_file:={pf}'],
                                  stdout=open(os.path.join(root, f'{name}_launch.log'), 'w'), stderr=subprocess.STDOUT,
                                  start_new_session=True)
        try:
            time.sleep(12.0)
            cmd = ['ros2', 'run', 'midpro_dash', 'run_regression', '--label', name, '--out', root]
            if args.repeats:
                cmd += ['--repeats', str(args.repeats)]
            if args.only:
                cmd += ['--only', args.only]
            if args.group:
                cmd += ['--group', args.group]
            subprocess.run(cmd, check=False)
        finally:
            os.killpg(os.getpgid(launch.pid), signal.SIGINT)
            try:
                launch.wait(timeout=20)
            except subprocess.TimeoutExpired:
                os.killpg(os.getpgid(launch.pid), signal.SIGKILL)
            time.sleep(5.0)
        found = sorted(glob.glob(os.path.join(root, f'{name}_*')))
        found = [d for d in found if os.path.isdir(d) and os.path.exists(os.path.join(d, 'summary.json'))]
        if found:
            results.append(found[-1])
    if len(results) >= 2:
        subprocess.run([sys.executable, '-m', 'midpro_dash.compare'] + results, check=False)
    print(f'\n[variants] all results in {root}')


if __name__ == '__main__':
    main()

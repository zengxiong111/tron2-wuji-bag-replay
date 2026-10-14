"""Audit the delivered linear-interpolated recorded path without hardware I/O."""
import argparse
import csv
import hashlib
import json
from pathlib import Path
import subprocess
import sys
import xml.etree.ElementTree as ET

import numpy as np

ROOT = Path(__file__).resolve().parents[1]


def sha(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def inspect_limits(data, urdf):
    time = np.asarray(data['time'], dtype=float)
    q = np.asarray(data['q'], dtype=float)
    names = [str(n) for n in data['joint_names']]
    limits = {j.attrib['name']: j.find('limit').attrib
              for j in ET.parse(urdf).getroot().findall('joint')
              if j.attrib['type'] == 'revolute'}
    if (time.ndim != 1 or len(time) < 2 or q.shape != (len(time), 56)
            or len(names) != 56 or set(names) != set(limits)
            or not np.isfinite(time).all() or not np.isfinite(q).all()
            or not np.all(np.diff(time) > 0)):
        raise ValueError('Invalid 56-joint trajectory or joint identity')
    slopes = np.diff(q, axis=0) / np.diff(time)[:, None]
    rows = []
    for i, name in enumerate(names):
        limit = limits[name]
        lower, upper, speed = [float(limit[k]) for k in ('lower', 'upper', 'velocity')]
        if not np.isfinite([lower, upper, speed]).all() or speed <= 0 or lower >= upper:
            raise ValueError('Invalid model limit: '+name)
        excess = max(0., lower-float(q[:, i].min()), float(q[:, i].max())-upper)
        peak = float(abs(slopes[:, i]).max())
        rows.append({'joint': name, 'group': 'wuji' if name.startswith(('l_', 'r_')) else ('head' if name.startswith('head') else 'tron2_arm'),
                     'minimum_rad': float(q[:, i].min()), 'maximum_rad': float(q[:, i].max()),
                     'model_lower_rad': lower, 'model_upper_rad': upper,
                     'position_excess_rad': excess, 'position_excess_deg': float(np.degrees(excess)),
                     'peak_linear_speed_rad_s': peak, 'model_speed_limit_rad_s': speed,
                     'speed_limit_ratio': peak/speed})
    return time, q, rows


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--input', type=Path, default=ROOT/json.loads((ROOT/'configs/trajectory.json').read_text())['trajectory'])
    parser.add_argument('--out', type=Path, required=True)
    parser.add_argument('--limits-only', action='store_true')
    args = parser.parse_args()
    output = args.out.expanduser().resolve()
    if output == ROOT or ROOT in output.parents:
        raise ValueError('Write generated evidence outside the repository')
    data = np.load(args.input, allow_pickle=False)
    urdf = ROOT/'outputs/robot/tron2_wuji_hand2.urdf'
    time, q, rows = inspect_limits(data, urdf)
    output.mkdir(parents=True, exist_ok=False)
    with (output/'joint_limits.csv').open('w', newline='') as stream:
        writer = csv.DictWriter(stream, fieldnames=list(rows[0]))
        writer.writeheader()
        writer.writerows(rows)
    groups = {}
    for group in ('tron2_arm', 'wuji', 'head'):
        selected = [r for r in rows if r['group'] == group]
        groups[group] = {'worst_speed': max(selected, key=lambda x: x['speed_limit_ratio']),
                         'speed_over_limit_joints': sum(r['speed_limit_ratio'] > 1 for r in selected),
                         'angle_over_limit_joints': sum(r['position_excess_rad'] > 0 for r in selected)}
    report = {'trajectory_sha256': sha(args.input), 'urdf_sha256': sha(urdf),
              'producer_sha256': sha(Path(__file__)), 'interpolation': 'piecewise linear in execution seconds',
              'duration_s': float(time[-1]-time[0]), 'states': len(time), 'groups': groups,
              'model_velocity_passed': all(r['speed_limit_ratio'] <= 1 for r in rows),
              'model_position_strict_passed': all(r['position_excess_rad'] == 0 for r in rows),
              'maximum_position_excess_deg': max(r['position_excess_deg'] for r in rows),
              'acceleration_jerk_status': 'Not bounded: linear interpolation has velocity jumps at knots; no motor acceleration/jerk admission',
              'hardware_motor_limits_verified': False, 'hardware_execution_authorized': False,
              'physical_base_to_table_m': .45,
              'boundary': 'Model-only position and interval-slope velocity checks; not device ratings, current/torque/thermal verification or new physics execution'}
    if not args.limits_only:
        grid = np.unique(np.r_[time, (time[:-1]+time[1:])/2,
                               np.arange(time[0], time[-1], 1/240), time[-1]])
        sampled = output/'sampled_linear.npz'
        np.savez_compressed(sampled, time=grid,
                            q=np.array([np.interp(grid, time, q[:, i]) for i in range(56)]).T,
                            joint_names=data['joint_names'])
        result = subprocess.run([sys.executable, str(ROOT/'scripts/115_audit_collision_fixed_replay.py'),
                                 '--input', str(sampled), '--q-key', 'q', '--time-key', 'time',
                                 '--strict-intersections', '--out', str(output/'collision')])
        if result.returncode not in (0, 2):
            raise RuntimeError('Collision audit failed to execute')
        gate = json.loads((output/'collision/gate.json').read_text())
        if gate['input_sha256'] != sha(sampled):
            raise ValueError('Collision audit/input binding mismatch')
        coverage = json.loads((output/'collision/coverage.json').read_text())
        if (coverage['robot_collision_geometry_count'] != 75 or coverage['pairs']['total'] != 2717
                or coverage['physical_base_above_table_m'] != .45 or not gate['raw_structural_only']):
            raise ValueError('Unexpected collision coverage or mounting geometry')
        report.update(collision=gate, collision_samples=len(grid),
                      sampled_sha256=sha(sampled), collision_sample_definition='All native knots, all interval midpoints, and 240 Hz execution grid including endpoints',
                      collision_boundary='Strict discrete triangle intersections with structural exclusions only; no Wuji assembly exemptions; not continuous CCD or exact solid clearance')
    (output/'report.json').write_text(json.dumps(report, indent=2)+'\n')
    print(json.dumps(report, indent=2))
    if not report['model_velocity_passed'] or not report['model_position_strict_passed'] or (not args.limits_only and not report['collision']['passed']):
        raise SystemExit(2)


if __name__ == '__main__':
    main()

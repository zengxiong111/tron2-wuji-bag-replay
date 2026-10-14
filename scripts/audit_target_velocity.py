"""Offline per-joint speed admission against authored model limits, not hardware."""
import argparse
import csv
import hashlib
import json
from pathlib import Path
import xml.etree.ElementTree as ET

import numpy as np
from pxr import Usd, UsdPhysics

parser = argparse.ArgumentParser(description=__doc__)
parser.add_argument('--out', type=Path, required=True)
args = parser.parse_args()
root = Path(__file__).resolve().parents[1]
urdf = root/'outputs/robot/tron2_wuji_hand2.urdf'
reference = root/'trajectories/repaired_source.npz'
trace = root/'trajectories/recorded_actual_6x.npz'
record = json.loads((root/'evidence/physics/report.json').read_text())
sha = lambda path: hashlib.sha256(path.read_bytes()).hexdigest()
if sha(reference) != record['reference_sha256'] or sha(root/record['usd']) != record['usd_sha256']:
    raise ValueError('Replay input binding mismatch')
targets, recorded = np.load(reference), np.load(trace)
actual = {'joint_names':recorded['joint_names'], 'source_time':recorded['source_time'], 'q_actual':recorded['q']}
if not np.array_equal(targets['joint_names'], actual['joint_names']):
    raise ValueError('Joint identity mismatch')
speed = float(record['motion_speed'])
dt = np.diff(actual['source_time'])/speed
if not np.allclose(dt, 1/record['physics_hz'], atol=1e-8, rtol=0):
    raise ValueError('Invalid physical trace sample period')
if not np.isfinite(targets['q']).all() or not np.isfinite(actual['q_actual']).all():
    raise ValueError('Non-finite positions')
joints = {j.attrib['name']: j for j in ET.parse(urdf).getroot().findall('joint')}
stage = Usd.Stage.Open(str(root/record['usd']))
usd_joints = {str(p.GetName()): p for p in stage.Traverse() if p.IsA(UsdPhysics.RevoluteJoint)}
actual_velocity = np.diff(actual['q_actual'].astype(float), axis=0)/dt[:, None]
rows = []
for column, name in enumerate(targets['joint_names'].tolist()):
    limit = float(joints[name].find('limit').attrib['velocity'])
    if not np.isfinite(limit) or limit <= 0:
        raise ValueError('Missing positive model velocity limit')
    if record['target_interpolation'] != 'piecewise linear':
        raise ValueError('Expected published linear reference')
    times = targets['time'][:-1]
    velocities = abs(np.diff(targets['q'][:, column])/np.diff(targets['time']))*speed
    peak = int(velocities.argmax())
    feedback_peak = int(abs(actual_velocity[:, column]).argmax())
    usd_limit_deg = usd_joints[name].GetAttribute('physxJoint:maxJointVelocity').Get()
    usd_limit = float(usd_limit_deg)*np.pi/180
    if not np.isclose(usd_limit, limit, atol=1e-5, rtol=1e-6):
        raise ValueError(f'URDF/USD velocity limit mismatch: {name}')
    rows.append({'joint': name,
                 'group': 'wuji' if name.startswith(('l_', 'r_')) else ('head' if name.startswith('head') else 'tron2_arm'),
                 'model_limit_rad_s': limit, 'usd_limit_deg_s': float(usd_limit_deg),
                 'target_peak_rad_s': float(velocities[peak]), 'target_peak_source_s': float(times[peak]),
                 'target_limit_ratio': float(velocities[peak]/limit),
                 'actual_fd_peak_rad_s': float(abs(actual_velocity[feedback_peak, column])),
                 'actual_fd_peak_source_s': float(actual['source_time'][feedback_peak]),
                 'actual_fd_limit_ratio': float(abs(actual_velocity[feedback_peak, column])/limit),
                 'actual_fd_time_over_limit_s': float(np.count_nonzero(abs(actual_velocity[:, column]) > limit)*dt.mean())})
groups = {}
for group in ['tron2_arm', 'wuji', 'head']:
    selected = [r for r in rows if r['group'] == group]
    groups[group] = {'target_over_limit_joints': sum(r['target_limit_ratio'] > 1 for r in selected),
                     'actual_fd_over_limit_joints': sum(r['actual_fd_limit_ratio'] > 1 for r in selected),
                     'worst_target': max(selected, key=lambda r: r['target_limit_ratio']),
                     'worst_actual_fd': max(selected, key=lambda r: r['actual_fd_limit_ratio'])}
maximum_ratio = max(r['target_limit_ratio'] for r in rows)
report = {'motion_speed': speed, 'groups': groups,
          'model_velocity_target_admission_passed': maximum_ratio <= 1,
          'hardware_limit_status': 'unverified: device version, per-axis rated limits, current/torque and thermal envelope required',
          'uniform_speed_kinematic_upper_bound_x': speed/maximum_ratio,
          'uniform_speed_with_example_20_percent_velocity_margin_x': .8*speed/maximum_ratio,
          'actual_velocity_method': '240 Hz position finite differences; not directly recorded joint velocity or continuous peak proof',
          'target_velocity_method': 'exact interval slopes of piecewise linear replay interpolation; source-time derivatives multiplied by motion speed',
          'usd_velocity_unit': 'angular maxJointVelocity in degrees/s converted to rad/s',
          'body_mapping_status': record['body_mapping_status'], 'torque_current_thermal_verified': False,
          'sha256': {'reference': sha(reference), 'physical_trace': sha(trace), 'urdf': sha(urdf),
                     'usd': record['usd_sha256'], 'producer': sha(Path(__file__))}, 'joints': rows}
args.out = args.out.resolve()
if args.out == root or root in args.out.parents:
    raise ValueError('Write generated target-speed reports outside the repository')
args.out.mkdir(parents=True, exist_ok=False)
(args.out/'report.json').write_text(json.dumps(report, indent=2))
with (args.out/'joint_speeds.csv').open('w', newline='') as file:
    writer = csv.DictWriter(file, fieldnames=list(rows[0]))
    writer.writeheader()
    writer.writerows(rows)
print(json.dumps({k: v for k, v in report.items() if k not in ['joints', 'sha256']}, indent=2))
if not report['model_velocity_target_admission_passed']:
    raise SystemExit(2)

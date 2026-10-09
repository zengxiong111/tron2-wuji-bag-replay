"""Reproduce the published model and documented-range checks, without hardware I/O."""
import argparse
import csv
import hashlib
import json
from pathlib import Path
import xml.etree.ElementTree as ET

import numpy as np
import pinocchio as pin
from scipy.interpolate import CubicSpline, PPoly

ROOT = Path(__file__).resolve().parents[1]


def extrema(curve):
    roots = curve.derivative().roots(extrapolate=False)
    times = np.r_[curve.x, roots[np.isfinite(roots)]]
    values = curve(times)
    return float(values.min()), float(values.max())


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--out', type=Path, required=True)
    args = parser.parse_args()
    source = ROOT/'trajectories/head_forward_source.npz'
    trace = ROOT/'evidence/final/validation/tracking.npz'
    urdf = ROOT/'outputs/robot/tron2_wuji_hand2.urdf'
    reference, actual = np.load(source), np.load(trace)
    record = json.loads((ROOT/'evidence/final/validation/report.json').read_text())
    sha = lambda path: hashlib.sha256(path.read_bytes()).hexdigest()
    assert sha(source) == record['reference_sha256'], 'Reference binding changed'
    acceptance = json.loads((ROOT/'evidence/final/acceptance.json').read_text())
    assert sha(trace) == acceptance['sha256']['validation/tracking.npz'], 'Physical trace changed'
    names = reference['joint_names'].tolist()
    assert names == actual['joint_names'].tolist()
    speed = float(record['motion_speed'])
    dt = 1/float(record['physics_hz'])
    assert speed == 4 and record['status'] == 'complete'
    assert actual['source_time'][0] == 0
    assert np.allclose(np.diff(actual['source_time']), speed*dt, atol=1e-8, rtol=0)
    assert abs(actual['source_time'][-1] + speed*dt-reference['time'][-1]) <= speed*dt+1e-7
    spline = CubicSpline(reference['time'], reference['q'], axis=0,
                         bc_type=((1, np.zeros(56)), (1, np.zeros(56))))
    joints = {j.attrib['name']: j for j in ET.parse(urdf).getroot().findall('joint')}
    # TRON2 manual section 1.5: named joints, angular limits in degrees.
    lower = [-180,-15,-210,-150,-100,-45,-90,-180,-183,-85,-150,-80,-45,-90,-90,-45]
    upper = [149,183,85,15,80,45,90,149,15,210,15,100,45,90,90,60]
    documented = dict(zip(names[:16], zip(lower, upper)))
    for name in names[16:]:
        if 'thumb' in name:
            bounds = {'cmc_flex':(-68,74), 'cmc_abd':(-85,40), 'mcp':(-60,90), 'ip':(-60,90)}
            key = name.split('thumb_')[1]
        else:
            bounds = {'mcp_flex':(-60,90), 'mcp_abd':(-40,40), 'pip':(-60,120), 'dip':(-60,90)}
            key = next(k for k in bounds if name.endswith('_'+k))
        documented[name] = bounds[key]
    finite = np.isfinite(actual['q_actual']).all() and np.isfinite(reference['q']).all()
    if not finite:
        raise ValueError('Non-finite trajectory')
    actual_velocity = np.diff(actual['q_actual'].astype(float), axis=0)/dt
    rows = []
    for column, name in enumerate(names):
        curve = PPoly(spline.c[:, :, column], spline.x)
        lo, hi = extrema(curve)
        v = max(map(abs, extrema(curve.derivative())))*speed
        acc = max(map(abs, extrema(curve.derivative(2))))*speed**2
        limit = joints[name].find('limit').attrib
        dlo, dhi = np.radians(documented[name])
        rows.append({'joint':name, 'target_min_deg':float(np.degrees(lo)),
                     'target_max_deg':float(np.degrees(hi)),
                     'documented_min_deg':documented[name][0], 'documented_max_deg':documented[name][1],
                     'documented_position_excess_rad':max(0., dlo-lo, hi-dhi),
                     'model_position_excess_rad':max(0., float(limit['lower'])-lo, hi-float(limit['upper'])),
                     'target_speed_rad_s':v, 'model_speed_limit_rad_s':float(limit['velocity']),
                     'target_speed_ratio':v/float(limit['velocity']),
                     'actual_fd_speed_ratio':float(abs(actual_velocity[:,column]).max()/float(limit['velocity'])),
                     'target_acceleration_peak_rad_s2':acc,
                     'joint_acceleration_limit_available':False})
    model = pin.buildModelFromUrdf(str(urdf))
    data = model.createData()
    indices = np.array([model.idx_qs[model.getJointId(n)] for n in names])
    frames = [model.getFrameId(n) for n in ['wrist_roll_L_Link','wrist_roll_R_Link']]
    wrist = {}
    for label, positions in [('target',spline(actual['source_time'])), ('actual',actual['q_actual'])]:
        xyz = []
        for row in positions:
            full = pin.neutral(model)
            full[indices] = row
            pin.framesForwardKinematics(model, data, full)
            xyz.append([data.oMf[f].translation.copy() for f in frames])
        velocity = np.diff(xyz, axis=0)/dt
        acceleration = np.diff(velocity, axis=0)/dt
        wrist[label] = {'speed_peak_m_s_left_right':np.linalg.norm(velocity,axis=2).max(axis=0).tolist(),
                        'acceleration_peak_m_s2_left_right':np.linalg.norm(acceleration,axis=2).max(axis=0).tolist()}
    report = {'checked_date':'2026-10-09', 'motion_speed':speed,
              'model_velocity_passed':all(r['target_speed_ratio'] <= 1 and r['actual_fd_speed_ratio'] <= 1 for r in rows),
              'model_target_position_passed':all(r['model_position_excess_rad'] <= 1e-10 for r in rows),
              'documented_target_position_passed':all(r['documented_position_excess_rad'] <= 1e-10 for r in rows),
              'hardware_motor_limits_verified':False,
              'missing':['device-specific rated/peak speed and acceleration','torque/current recordings at 4x',
                         'thermal and load envelope','confirmed bag body axis mapping','Wuji SDK actuator mapping'],
              'wrist_reference':wrist,
              'published_tron2_endpoint_specs':{'speed_m_s':5,'acceleration_m_s2':36},
              'wrist_boundary':'240 Hz FK finite differences at wrist_roll link origins; not a continuous peak proof or the vendor tool endpoint',
              'head_mapping_note':'Manual indices 14=pitch,15=yaw; tentative bag mapping reversed. Both head targets are zero in this release.',
              'sources':{'wuji_control':'https://docs.wuji.tech/docs/zh/wuji-hand/latest/control-guide/',
                         'wuji_notice':'https://docs.wuji.tech/docs/zh/wuji-hand/latest/user-notice/',
                         'tron2_manual':'https://www.limxdynamics.com/zh/documents/844648486841487360',
                         'tron2_sdk':'https://www.limxdynamics.com/zh/documents/847884267345285120'},
              'sha256':{'reference':sha(source),'trace':sha(trace),'portable_urdf':sha(urdf),'producer':sha(Path(__file__))},
              'joints':rows}
    args.out.mkdir(parents=True, exist_ok=False)
    (args.out/'report.json').write_text(json.dumps(report,indent=2))
    with (args.out/'joint_limits.csv').open('w',newline='') as stream:
        writer = csv.DictWriter(stream,fieldnames=list(rows[0]),lineterminator="\n")
        writer.writeheader()
        writer.writerows(rows)
    print(json.dumps({k:report[k] for k in ['model_velocity_passed','model_target_position_passed',
                                          'documented_target_position_passed','hardware_motor_limits_verified','wrist_reference']},indent=2))
    if not all(report[k] for k in ['model_velocity_passed','model_target_position_passed','documented_target_position_passed']):
        raise SystemExit(2)


if __name__ == '__main__':
    main()

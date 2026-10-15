"""Compare preserved reference and optional physical response against original 5x poses."""
import argparse
import hashlib
import json
from pathlib import Path

import numpy as np
import pinocchio as pin
from scipy.signal import butter, sosfiltfilt

p = argparse.ArgumentParser(description=__doc__)
p.add_argument('--before', type=Path, required=True, help='Original 5x physical trace or published recorded path')
p.add_argument('--trajectory', type=Path, required=True)
p.add_argument('--physical', type=Path)
p.add_argument('--out', type=Path, required=True)
a = p.parse_args()
repo_root = Path(__file__).resolve().parents[1]
if a.out.resolve() == repo_root or repo_root in a.out.resolve().parents:
    raise ValueError('Write generated results outside the repository')
root = Path(__file__).resolve().parents[1]
before, target = np.load(a.before), np.load(a.trajectory)
if 'q_actual' not in before.files:
    before = {'joint_names': before['joint_names'], 'source_time': before['source_time'], 'q_actual': before['q']}
names = before['joint_names']
model = pin.buildModelFromUrdf(str(root/'outputs/robot/tron2_wuji_hand2.urdf'))
indices = np.array([model.idx_qs[model.getJointId(str(n))] for n in names])
frames = [model.getFrameId(f'{side}_{name}') for side in ['l', 'r']
          for name in ['index_finger_tip', 'middle_finger_tip', 'pinky_tip', 'ring_finger_tip', 'thumb_tip']]
data = model.createData()
sha = lambda path: hashlib.sha256(path.read_bytes()).hexdigest()


def fingertips(q):
    rows = []
    for row in q:
        full = pin.neutral(model)
        full[indices] = row
        pin.framesForwardKinematics(model, data, full)
        rows.append([data.oMf[f].translation.copy() for f in frames])
    return np.array(rows)


def compare(samples, qkey):
    if not np.array_equal(names, samples['joint_names']):
        raise ValueError('Joint identity changed')
    time = samples['source_time']
    q = samples[qkey].astype(float)
    if not np.isfinite(q).all() or np.any(np.diff(time) <= 0):
        raise ValueError('Invalid samples')
    old = np.array([np.interp(time, before['source_time'], before['q_actual'][:, j]) for j in range(56)]).T
    delta = q-old
    old_tips, new_tips = fingertips(old), fingertips(q)
    distance = np.linalg.norm(new_tips-old_tips, axis=-1)*1000
    spacing = [np.linalg.norm(t[:, :5, None]-t[:, None, 5:], axis=-1).min(axis=(1, 2))*1000
               for t in [old_tips, new_tips]]
    return {'joint_delta_rms_deg': float(np.degrees(np.sqrt(np.mean(delta*delta)))),
            'joint_delta_max_deg': float(np.degrees(abs(delta).max())),
            'fingertip_delta_mean_mm': float(distance.mean()),
            'fingertip_delta_p95_mm': float(np.percentile(distance, 95)),
            'fingertip_delta_max_mm': float(distance.max()),
            'nearest_opposite_tip_spacing_before_min_mm': float(spacing[0].min()),
            'nearest_opposite_tip_spacing_after_min_mm': float(spacing[1].min()),
            'tip_spacing_delta_max_mm': float(abs(spacing[1]-spacing[0]).max())}


result = {'reference': compare(target, 'q'),
          'sha256': {'before': sha(a.before), 'trajectory': sha(a.trajectory), 'producer': sha(Path(__file__))},
          'boundary': 'Source-aligned fingertip frame positions, not surface contact or force preservation. Reference fidelity and new physical actual states are separate.'}
rate = 1/float(np.diff(target['time']).mean())
if not np.allclose(np.diff(target['time']), 1/rate, atol=1e-8, rtol=0):
    raise ValueError('Expected uniform reference execution time')
quality = {}
for label, q in [('before', before['q_actual']), ('after', target['q'])]:
    q = q.astype(float)
    residual = q-sosfiltfilt(butter(3, 6, fs=rate, output='sos'), q, axis=0)
    quality[label] = {'joint_hf_rms_rad': float(np.sqrt(np.mean(residual*residual))),
                      'acceleration_abs_p95_rad_s2': float(np.percentile(abs(np.diff(q, n=2, axis=0))*rate**2, 95)),
                      'jerk_abs_p95_rad_s3': float(np.percentile(abs(np.diff(q, n=3, axis=0))*rate**3, 95))}
result['temporal_quality'] = quality
result['temporal_definition'] = 'Both pose arrays evaluated at6x/288Hz;6Hz cutoff corresponds to5Hz at original5x speed; no global smoothing claim'
if a.physical:
    actual = np.load(a.physical)
    result['physical_actual'] = compare(actual, 'q_actual')
    result['sha256']['physical'] = sha(a.physical)
    error = actual['q_actual']-actual['q_target']
    result['physical_actual'].update(tracking_rms_rad=float(np.sqrt(np.mean(error*error))),
                                     tracking_max_rad=float(abs(error).max()))
a.out.write_text(json.dumps(result, indent=2))
print(json.dumps(result, indent=2))

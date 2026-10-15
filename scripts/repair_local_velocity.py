"""Repair only speed-violating neighborhoods of an otherwise locked recorded path."""
import argparse
import hashlib
import json
from pathlib import Path
import xml.etree.ElementTree as ET

import numpy as np
import osqp
from scipy import sparse
from scipy.ndimage import maximum_filter1d

p = argparse.ArgumentParser(description=__doc__)
p.add_argument('--input', type=Path, required=True)
p.add_argument('--out', type=Path, required=True)
p.add_argument('--context-seconds', type=float, default=.15)
p.add_argument('--max-delta-deg', type=float, default=4)
p.add_argument('--local-pose-hints', type=Path,
               help='Measured collision windows: joint, source_center_s, halfwidth_source_s, delta_deg')
a = p.parse_args()
repo_root = Path(__file__).resolve().parents[1]
if a.out.resolve() == repo_root or repo_root in a.out.resolve().parents:
    raise ValueError('Write generated results outside the repository')
if not np.isfinite([a.context_seconds, a.max_delta_deg]).all() or a.context_seconds < 0 or a.max_delta_deg <= 0:
    raise ValueError('Expected nonnegative local context and positive angular budget')
root = Path(__file__).resolve().parents[1]
sha = lambda path: hashlib.sha256(path.read_bytes()).hexdigest()
d = np.load(a.input, allow_pickle=False)
t, original = d['time'], d['q']
q = original.astype(float)
dt = np.diff(t)
if q.shape != (len(t), 56) or not np.isfinite(q).all() or not np.all(dt > 0):
    raise ValueError('Invalid named trajectory')
if not np.allclose(dt, dt.mean(), atol=1e-8, rtol=0):
    raise ValueError('Expected uniform time before local repair')
urdf = root/'outputs/robot/tron2_wuji_hand2.urdf'
joints = {j.get('name'): j for j in ET.parse(urdf).getroot().findall('joint')}
N = len(q)
D = sparse.diags([-np.ones(N-1), np.ones(N-1)], [0, 1], shape=(N-1, N), format='csc')
D2 = sparse.diags([np.ones(N-2), -2*np.ones(N-2), np.ones(N-2)], [0, 1, 2], shape=(N-2, N), format='csc')
mask_all = np.zeros_like(q, dtype=bool)
result = original.copy()
columns = []
hints = json.loads(a.local_pose_hints.read_text()) if a.local_pose_hints else []
hint = np.zeros_like(q)
for item in hints:
    column = d['joint_names'].tolist().index(item['joint'])
    halfwidth = float(item['halfwidth_source_s'])
    if halfwidth <= 0 or not np.isfinite(halfwidth):
        raise ValueError('Invalid local collision window')
    u = (d['source_time']-float(item['source_center_s']))/halfwidth
    active_hint = abs(u) < 1
    hint[active_hint, column] += np.radians(float(item['delta_deg']))*(1-u[active_hint]**2)**3
width = 2*int(np.ceil(a.context_seconds/dt.mean()))+1
for j, name in enumerate(d['joint_names']):
    limit = joints[str(name)].find('limit')
    cap = float(limit.get('velocity'))
    diff = np.diff(q[:, j])
    bad = abs(diff) > cap*dt
    if not bad.any() and not hint[:, j].any():
        continue
    mask = maximum_filter1d((np.r_[bad, False] | np.r_[False, bad]).astype(int), size=width) > 0
    mask |= hint[:, j] != 0
    mask[[0, -1]] = False
    active = np.flatnonzero(mask)
    rows = np.flatnonzero(mask[:-1] | mask[1:])
    K = len(active)
    velocity = D[rows][:, active]
    curvature = D2[:, active]
    identity = sparse.eye(K, format='csc')
    matrix = sparse.vstack([velocity, identity], format='csc')
    max_delta = np.radians(a.max_delta_deg)
    lower = np.maximum(-max_delta, float(limit.get('lower'))-q[active, j])
    upper = np.minimum(max_delta, float(limit.get('upper'))-q[active, j])
    bound = cap*dt[rows]*.9999
    solver = osqp.OSQP()
    solver.setup(P=sparse.triu(identity+20*curvature.T@curvature, format='csc'),
                 q=-hint[active, j], A=matrix,
                 l=np.r_[-bound-diff[rows], lower], u=np.r_[bound-diff[rows], upper],
                 eps_abs=1e-9, eps_rel=1e-9, max_iter=100000,
                 polishing=True, verbose=False)
    solution = solver.solve()
    if solution.info.status != 'solved':
        raise RuntimeError(f'{name}: {solution.info.status}; do not silently expand correction limits')
    result[active, j] = q[active, j]+solution.x
    mask_all[:, j] = mask
    columns.append({'joint': str(name), 'allowed_samples': K,
                    'max_delta_deg': float(np.degrees(abs(result[:, j].astype(float)-q[:, j]).max()))})
assert np.array_equal(result[~mask_all], original[~mask_all])
caps = np.array([float(joints[str(n)].find('limit').get('velocity')) for n in d['joint_names']])
peak = float((abs(np.diff(result.astype(float), axis=0))/dt[:, None]/caps).max())
if peak > 1:
    raise RuntimeError(f'Float-output velocity check failed: {peak}')
a.out.mkdir(parents=True, exist_ok=False)
output = a.out/'actual_6x.npz'
np.savez_compressed(output, **{k: d[k] for k in d.files if k != 'q'}, q=result)
np.savez_compressed(a.out/'local_masks.npz', time=t, joint_names=d['joint_names'], allowed=mask_all,
                    changed=result != original)
delta = result.astype(float)-q
report = {'input_sha256': sha(a.input), 'output_sha256': sha(output),
          'producer_sha256': sha(Path(__file__)), 'urdf_sha256': sha(urdf),
          'duration_s': float(t[-1]), 'native_hz': 1/float(dt.mean()),
          'context_seconds': a.context_seconds, 'max_delta_budget_deg': a.max_delta_deg,
          'max_delta_deg': float(np.degrees(abs(delta).max())),
          'rms_delta_deg': float(np.degrees(np.sqrt(np.mean(delta*delta)))),
          'changed_cells': int(np.count_nonzero(result != original)),
          'changed_cell_fraction': float(np.mean(result != original)),
          'changed_frames': int(np.count_nonzero(np.any(result != original, axis=1))),
          'changed_frame_fraction': float(np.mean(np.any(result != original, axis=1))),
          'context_samples_each_side': (width-1)//2,
          'actual_context_seconds_each_side': float((width-1)//2*dt.mean()),
          'outside_allowed_windows_bitwise_unchanged': True,
          'arms_and_head_bitwise_unchanged': bool(np.array_equal(result[:, :16], original[:, :16])),
          'first_and_last_pose_bitwise_unchanged': bool(np.array_equal(result[[0, -1]], original[[0, -1]])),
          'peak_model_velocity_ratio': peak, 'joints': columns,
          'local_collision_hints': hints,
          'method': 'Convex closest-correction QP with correction-curvature penalty; only original speed-violation neighborhoods and explicitly measured collision windows are variables',
          'boundary': 'Recorded-path local correction, not a new physical trace; mesh, fidelity and temporal checks required',
          'physical_base_to_table_m': .45, 'hardware_execution_authorized': False}
(a.out/'repair.json').write_text(json.dumps(report, indent=2))
print(json.dumps(report, indent=2))

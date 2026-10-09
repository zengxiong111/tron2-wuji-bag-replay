"""Compare recorded physical trajectories in simulation time, including wrist FK."""
import argparse
import hashlib
import json
from pathlib import Path

import numpy as np
import pinocchio as pin
from scipy.interpolate import CubicSpline, PPoly
from scipy.signal import butter, sosfiltfilt

parser = argparse.ArgumentParser(description=__doc__)
parser.add_argument('--before', type=Path, required=True)
parser.add_argument('--after', type=Path, required=True)
parser.add_argument('--reference', type=Path, required=True)
parser.add_argument('--out', type=Path, required=True)
args = parser.parse_args()
root = Path(__file__).resolve().parents[1]
model = pin.buildModelFromUrdf(str(root / 'outputs/robot/tron2_wuji_hand2.urdf'))
reference = np.load(args.reference)
indices = np.array([model.idx_qs[model.getJointId(str(n))] for n in reference['joint_names']])
lower, upper = model.lowerPositionLimit[indices], model.upperPositionLimit[indices]
result = {}
high_frequency = {}
baseline_time = None
for label, path in [('before', args.before), ('after', args.after)]:
    samples = np.load(path)
    assert np.array_equal(samples['joint_names'], reference['joint_names'])
    times = samples['source_time']
    if baseline_time is None:
        baseline_time = times.copy()
    if times.shape != baseline_time.shape or not np.allclose(times, baseline_time, atol=1e-7, rtol=0):
        raise ValueError('Comparison requires identical source-time coverage')
    if not np.allclose(np.diff(times), 5/240, atol=1e-7, rtol=0):
        raise ValueError('Expected uniformly sampled 5x motion at 240 Hz')
    positions = samples['q_actual'].astype(float)
    if not np.isfinite(positions).all():
        raise ValueError('Non-finite physical states')
    rate = 5 / float(np.diff(samples['source_time']).mean())
    lowpass = butter(3, 5, fs=rate, output='sos')
    residual = positions - sosfiltfilt(lowpass, positions, axis=0)
    high_frequency[label] = residual
    velocity = np.diff(positions, axis=0) * rate
    acceleration = np.diff(velocity, axis=0) * rate
    jerk = np.diff(acceleration, axis=0) * rate
    result[label] = {
        'input_sha256': hashlib.sha256(path.read_bytes()).hexdigest(),
        'joint_hf_rms_rad': float(np.sqrt(np.mean(residual**2))),
        'acceleration_abs_p95_rad_s2': float(np.percentile(abs(acceleration), 95)),
        'jerk_abs_p95_rad_s3': float(np.percentile(abs(jerk), 95)),
        'max_joint_velocity_rad_s': float(abs(velocity).max()),
        'tracking_rms_rad': float(np.sqrt(np.mean((positions - samples['q_target'])**2))),
        'tracking_max_all_steps_rad': float(abs(positions - samples['q_target']).max()),
        'position_limit_excess_rad': float(max(0, (lower-positions).max(), (positions-upper).max())),
    }
    data = model.createData()
    frames = [model.getFrameId(n) for n in ['wrist_roll_L_Link', 'wrist_roll_R_Link']]
    wrists = []
    for row in positions:
        full = pin.neutral(model)
        full[indices] = row
        pin.framesForwardKinematics(model, data, full)
        wrists.append([data.oMf[f].translation.copy() for f in frames])
    wrists = np.array(wrists)
    wrist_residual = wrists - sosfiltfilt(lowpass, wrists, axis=0)
    result[label]['wrist_translation_hf_rms_mm'] = float(np.sqrt(np.mean(wrist_residual**2)) * 1000)

spline = CubicSpline(reference['time'], reference['q'], axis=0,
                     bc_type=((1, np.zeros(56)), (1, np.zeros(56))))
# Check analytical extrema in every interval, rather than only sampled knots.
limit_excess = 0.
for joint in range(56):
    curve = PPoly(spline.c[:, :, joint], spline.x)
    roots = curve.derivative().roots(extrapolate=False)
    values = curve(np.r_[spline.x, roots[np.isfinite(roots)]])
    limit_excess = max(limit_excess, float((lower[joint]-values).max()),
                       float((values-upper[joint]).max()))
times = np.arange(0, reference['time'][-1], 5/240)
result['cubic_target'] = {'analytical_position_limit_excess_rad': limit_excess,
                          'max_velocity_rad_s': float(abs(spline(times, 1)*5).max()),
                          'finite': bool(np.isfinite(spline(times)).all())}
metrics = ['joint_hf_rms_rad', 'acceleration_abs_p95_rad_s2',
           'jerk_abs_p95_rad_s3', 'wrist_translation_hf_rms_mm']
result['reduction_percent'] = {key: 100*(1-result['after'][key]/result['before'][key]) for key in metrics}
energy = np.mean(high_frequency['before']**2, axis=1)
rolling = np.convolve(energy, np.ones(720)/720, mode='valid')
start = int(np.argmax(rolling[240:-240])) + 240
result['comparison_source_start_s'] = start/240*5
result['comparison_simulation_duration_s'] = 3
result['definition'] = '5 Hz order-3 zero-phase residual; derivatives at 240 Hz; pooled absolute p95; authored wrist frame FK'
result['passed'] = bool(limit_excess == 0 and result['cubic_target']['finite']
                        and result['after']['position_limit_excess_rad'] <= 1e-4
                        and result['reduction_percent']['joint_hf_rms_rad'] >= 30
                        and result['reduction_percent']['acceleration_abs_p95_rad_s2'] >= 30)
result['gate_boundary'] = 'Relative smoothness improvement only; separate collision gate required; not a hardware motion limit'
result['actual_position_limit_numerical_tolerance_rad'] = 1e-4
args.out.parent.mkdir(parents=True, exist_ok=True)
args.out.write_text(json.dumps(result, indent=2))
print(json.dumps(result, indent=2))
if not result['passed']:
    raise SystemExit(2)

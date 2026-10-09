"""Export a standalone simulation-time trajectory without changing positions."""
import argparse
import hashlib
import json
from pathlib import Path

import numpy as np
from scipy.interpolate import CubicSpline

def export(source, destination, smooth=False, speed=4):
    if not np.isfinite(speed) or speed <= 0:
        raise ValueError('Speed must be finite and positive')
    data = np.load(source)
    axis = data['time']
    q = data['q']
    simulated = axis / speed
    if smooth:
        curve = CubicSpline(axis, q, axis=0, bc_type=((1, np.zeros(q.shape[1])), (1, np.zeros(q.shape[1]))))
        velocity = curve(axis, 1) * speed
    else:
        velocity = np.diff(q, axis=0) / np.diff(simulated)[:, None]
        velocity = np.vstack([velocity, velocity[-1]])
    destination.parent.mkdir(parents=True, exist_ok=True)
    if destination.exists():
        raise FileExistsError(destination)
    np.savez_compressed(destination, time=simulated, source_time=axis, q=q,
                        q_velocity=velocity, joint_names=data['joint_names'], motion_speed=np.float64(speed))
    return {'source_sha256': hashlib.sha256(source.read_bytes()).hexdigest(),
            'sha256': hashlib.sha256(destination.read_bytes()).hexdigest(),
            'motion_speed': speed, 'duration_s': float(simulated[-1]), 'position_unchanged': True,
            'velocity': 'analytic C2 cubic derivative' if smooth else 'piecewise-linear interval slope',
            'boundary': 'Joint targets; not recorded physical states or a collision certificate'}

if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--input', type=Path, required=True)
    parser.add_argument('--out', type=Path, required=True)
    parser.add_argument('--smooth', action='store_true')
    parser.add_argument('--motion-speed',type=float,default=4)
    args = parser.parse_args()
    print(json.dumps(export(args.input, args.out, args.smooth, args.motion_speed), indent=2))

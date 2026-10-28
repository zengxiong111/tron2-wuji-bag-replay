"""Play the accepted actual-state trajectory, or explicitly run a fresh PhysX diagnostic."""
import argparse
import os
from pathlib import Path
import subprocess
import sys

import numpy as np

ROOT = Path(__file__).resolve().parents[1]


def load_actual_trajectory(path):
    """Load the accepted actual-state NPZ on simulation time, without retiming it."""
    with np.load(path, allow_pickle=False) as data:
        required = {'simulation_time', 'q_actual', 'joint_names'}
        if not required.issubset(data.files):
            raise ValueError(f'Actual-state trajectory is missing keys: {sorted(required - set(data.files))}')
        axis = np.asarray(data['simulation_time'], dtype=np.float64)
        q = np.asarray(data['q_actual'], dtype=np.float64)
        names = data['joint_names'].astype(str).tolist()
    if (axis.ndim != 1 or q.shape != (len(axis), 56) or len(names) != 56
            or not np.isfinite(axis).all() or not np.isfinite(q).all()
            or not np.all(np.diff(axis) > 0)):
        raise ValueError('Invalid actual-state trajectory: expected finite increasing simulation_time and 56-DOF q_actual')
    return axis, q, names


def build_command(args):
    if args.physics:
        command = [args.isaac_python, str(ROOT/'scripts/portable_fresh_physics.py'),
                   '--isaac-python', args.isaac_python]
        if args.headless:
            command.append('--headless')
        if args.out:
            command += ['--out', str(args.out.resolve())]
        if args.max_source_seconds is not None:
            command += ['--max-source-seconds', str(args.max_source_seconds)]
        return command
    command = [args.isaac_python, str(ROOT/'scripts/replay_recorded_path.py')]
    if args.headless:
        command.append('--headless')
    if args.out:
        command += ['--out', str(args.out.resolve())]
    if args.max_seconds is not None:
        command += ['--max-seconds', str(args.max_seconds)]
    return command


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--isaac-python', default=os.environ.get('ISAAC_PYTHON', sys.executable))
    parser.add_argument('--physics', action='store_true', help='Explicitly run a fresh PhysX diagnostic instead of recorded actual-state playback')
    parser.add_argument('--headless', action='store_true')
    parser.add_argument('--out', type=Path)
    parser.add_argument('--max-seconds', type=float, help='Bounded recorded playback')
    parser.add_argument('--max-source-seconds', type=float, help='Bounded fresh-physics diagnostic')
    args = parser.parse_args(argv)
    subprocess.run(build_command(args), check=True)


if __name__ == '__main__':
    main()

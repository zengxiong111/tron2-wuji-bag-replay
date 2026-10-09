"""Simple launch entry for source-time references and four-times physical replay."""
import argparse
import subprocess
from pathlib import Path

root = Path(__file__).resolve().parents[1]
parser = argparse.ArgumentParser(description=__doc__)
parser.add_argument('--isaac-python', required=True, help='Isaac Python executable or python.sh')
parser.add_argument('--mode', choices=['original','accelerated','repaired','head-forward'], default='head-forward')
parser.add_argument('--headless', action='store_true')
parser.add_argument('--out', type=Path)
parser.add_argument('--max-source-seconds', type=float)
args = parser.parse_args()
references = {'original':'original.npz', 'accelerated':'original.npz',
              'repaired':'head_forward_source.npz', 'head-forward':'head_forward_source.npz'}
command = [args.isaac_python, str(root/'scripts/114_replay_ros1_collision_fixed.py'),
           '--reference', str(root/'trajectories'/references[args.mode]),
           '--motion-speed', '1' if args.mode=='original' else '4']
if args.mode in ['repaired','head-forward']:
    command.append('--smooth-targets')
if args.headless:
    command += ['--headless', '--wall-speed', '10']
if args.out is not None:
    command += ['--out', str(args.out.resolve())]
if args.max_source_seconds is not None:
    command += ['--max-source-seconds', str(args.max_source_seconds)]
subprocess.run(command, check=True)

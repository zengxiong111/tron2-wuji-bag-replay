"""Launch the published retimed recorded path at one-times file time."""
import argparse
from pathlib import Path
import subprocess

root = Path(__file__).resolve().parents[1]
parser = argparse.ArgumentParser(description=__doc__)
parser.add_argument('--isaac-python', required=True)
parser.add_argument('--headless', action='store_true')
parser.add_argument('--out', type=Path)
parser.add_argument('--max-seconds', type=float)
args = parser.parse_args()
command = [args.isaac_python, str(root/'scripts/replay_recorded_path.py')]
if args.headless:
    command.append('--headless')
if args.out:
    command += ['--out', str(args.out.resolve())]
if args.max_seconds is not None:
    command += ['--max-seconds', str(args.max_seconds)]
subprocess.run(command, check=True)

"""Launch preserved6x reference display, or explicitly rerun repaired targets in PhysX."""
import argparse
from pathlib import Path
import subprocess

root = Path(__file__).resolve().parents[1]
parser = argparse.ArgumentParser(description=__doc__)
parser.add_argument('--isaac-python', required=True)
parser.add_argument('--headless', action='store_true')
parser.add_argument('--physics', action='store_true', help='Execute repaired6x targets with PhysX instead of setting recorded poses')
parser.add_argument('--wall-speed', type=float, default=1, help='Physical-run wall pacing only; action speed stays6x')
parser.add_argument('--out', type=Path)
parser.add_argument('--max-seconds', type=float)
args = parser.parse_args()
command = [args.isaac_python, str(root/('scripts/replay_physics.py' if args.physics else 'scripts/replay_recorded_path.py'))]
if args.physics:
    command += ['--wall-speed', str(args.wall_speed)]
elif args.wall_speed !=1:
    parser.error('--wall-speed requires --physics; recorded timing is fixed')
if args.headless:
    command.append('--headless')
if args.out:
    command += ['--out', str(args.out.resolve())]
if args.max_seconds is not None:
    command += ['--max-seconds', str(args.max_seconds)]
subprocess.run(command, check=True)

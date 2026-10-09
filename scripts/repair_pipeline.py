"""Rebuild every accepted repair stage from the original ROS1 bag."""
import argparse
import hashlib
import json
import subprocess
import sys
from pathlib import Path

import numpy as np
from export_trajectory import export

root = Path(__file__).resolve().parents[1]
parser = argparse.ArgumentParser(description=__doc__)
parser.add_argument('--out', type=Path, default=root.parent/(root.name+'-runs')/'repair')
parser.add_argument('--reference-only', action='store_true', help='Use the delivered extracted feedback instead of decoding the bag again')
args = parser.parse_args()
args.out = args.out.resolve()
args.out.mkdir(parents=True, exist_ok=False)

def run(script, *arguments):
    subprocess.run([sys.executable, str(root/'scripts'/script), *map(str, arguments)], check=True)

if args.reference_only:
    original = root/'trajectories/original.npz'
else:
    run('109_extract_ros1_feedback.py', '--bag', root/'data/2026-10-08-17-54-13.bag',
        '--usd', root/'outputs/robot/tron2_wuji_hand2.usd', '--urdf', root/'outputs/robot/tron2_wuji_hand2.urdf',
        '--out', args.out/'extracted')
    original = args.out/'extracted/feedback_30hz.npz'
run('118_smooth_bag_physics_reference.py', '--input', original, '--out', args.out/'filter',
    '--motion-speed', '4', '--head-forward')
run('117_repair_bag_wrist_clearance.py', '--input', args.out/'filter/reference.npz',
    '--out', args.out/'head_forward', '--safety-margin', '.04')
source = args.out/'head_forward/reference.npz'
result = export(source, args.out/'head_forward_4x.npz', True, 4)
(args.out/'exports.json').write_text(json.dumps({'head_forward_4x':result}, indent=2))
print('Reference rebuilt. New physics and collision checks are required before runtime acceptance.')

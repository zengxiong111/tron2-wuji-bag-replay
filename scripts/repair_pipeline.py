"""Rebuild every accepted repair stage from the original ROS1 bag."""
import argparse
import hashlib
import json
import subprocess
import sys
from pathlib import Path

import numpy as np
from export_5x import export

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
run('117_repair_bag_wrist_clearance.py', '--input', original, '--out', args.out/'clearance', '--safety-margin', '.03')
run('118_smooth_bag_physics_reference.py', '--input', args.out/'clearance/reference.npz', '--out', args.out/'filter')
run('117_repair_bag_wrist_clearance.py', '--input', args.out/'filter/reference.npz', '--out', args.out/'repaired', '--safety-margin', '.03')
head = args.out/'head_forward'
head.mkdir()
source = args.out/'repaired/reference.npz'
data = np.load(source)
q = data['q'].copy()
names = data['joint_names'].tolist()
q[:, [names.index('head_yaw_Joint'), names.index('head_pitch_Joint')]] = 0
np.savez_compressed(head/'reference.npz', **{k:data[k] for k in data.files if k!='q'}, q=q)
parent = json.loads((source.parent/'repair.json').read_text())
binding = {'source_sha256': parent['source_sha256'],
           'parent_reference_sha256': hashlib.sha256(source.read_bytes()).hexdigest(),
           'output_sha256': hashlib.sha256((head/'reference.npz').read_bytes()).hexdigest(),
           'head_position_rad': [0,0], 'operation': 'only head yaw/pitch targets set to zero'}
(head/'repair.json').write_text(json.dumps(binding, indent=2))
exports = {}
for label, source, smooth in [('accelerated_5x', original, False),
                               ('repaired_5x', args.out/'repaired/reference.npz', True),
                               ('head_forward_5x', head/'reference.npz', True)]:
    exports[label] = export(source, args.out/(label+'.npz'), smooth)
(args.out/'exports.json').write_text(json.dumps(exports, indent=2))
print('References rebuilt. Run physics and collision checks before claiming new runtime acceptance.')

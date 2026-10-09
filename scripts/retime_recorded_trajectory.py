"""Reproduce the published trajectory from its hash-bound 5x actual-state trace."""
import argparse
import hashlib
import json
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
parser = argparse.ArgumentParser(description=__doc__)
parser.add_argument('--tracking', type=Path, required=True)
parser.add_argument('--out', type=Path, required=True)
args = parser.parse_args()
origin = json.loads((ROOT/'configs/retiming.json').read_text())
sha = lambda path: hashlib.sha256(path.read_bytes()).hexdigest()
if sha(args.tracking) != origin['source_tracking_sha256']:
    raise ValueError('Source trace differs from the published retiming source')
output = args.out.expanduser().resolve()
if output == ROOT or ROOT in output.parents:
    raise ValueError('Write regenerated candidates outside the repository')
data = np.load(args.tracking, allow_pickle=False)
time = data['source_time']/4
q = data['q_actual'].astype(float)
output.parent.mkdir(parents=True, exist_ok=True)
with output.open('xb') as stream:
    np.savez_compressed(stream, time=time, source_time=data['source_time'],
                        q=data['q_actual'], joint_names=data['joint_names'],
                        q_velocity=np.gradient(q, time, axis=0))
if sha(output) != origin['output_sha256']:
    raise ValueError('Regenerated file differs; inspect environment and revalidate before use')
print('Reproduced trajectory SHA-256:', sha(output))

"""Explicitly rerun the bound reference in Isaac PhysX using the accepted 5x profile."""
import argparse
import json
import os
from pathlib import Path
import subprocess
import sys

ROOT = Path(__file__).resolve().parents[1]
parser = argparse.ArgumentParser(description=__doc__)
parser.add_argument('--isaac-python', default=os.environ.get('ISAAC_PYTHON', sys.executable))
parser.add_argument('--headless', action='store_true')
parser.add_argument('--out', type=Path)
parser.add_argument('--max-source-seconds', type=float, help='Bounded diagnostic only; omit to cover the full accepted phase')
args = parser.parse_args()

# verify_bundle validates installed bytes, acceptance identity, trajectory bindings, and the model lineage.
subprocess.run([sys.executable, str(ROOT/'scripts/verify_bundle.py')], check=True)
binding = json.loads((ROOT/'configs/trajectory.json').read_text())
if binding.get('motion_speed') != 5 or binding.get('physics_hz') != 480:
    raise ValueError('This entrypoint is bound to the accepted overall-5x, 480 Hz profile')
profile = binding.get('simulation_profile', {})
if (profile.get('arm_force_drive') or profile.get('rnea_feedforward')
        or profile.get('left_thumb_flex_gain_scale') != 4
        or profile.get('hand_velocity_limit_scale') != 0.94):
    raise ValueError('Fresh-physics actuator settings do not match accepted provenance')
prov = ROOT/binding['provenance']['path']
reference = ROOT/binding['reference']['path']
execution = ROOT/binding['execution']['path']
raw = ROOT/binding['raw_feedback']['path']
usd = ROOT/binding['usd']['path']
command = [args.isaac_python, str(ROOT/'scripts/portable_physx_replay.py'),
           '--provenance', str(prov), '--reference', str(reference),
           '--execution', str(execution), '--raw-source', str(raw), '--usd', str(usd),
           '--motion-speed', '5', '--physics-hz', '480',
           '--solver-position-iterations', '32', '--solver-velocity-iterations', '16',
           '--drive-gain-scale', '4', '--arm-drive-gain-scale', '8',
           '--hand-drive-gain-scale', '1', '--left-thumb-flex-gain-scale', '4',
           '--hand-velocity-limit-scale', '0.94', '--contact-offset', '0.001',
           '--rest-offset', '0', '--official-wuji-assembly-policy']
if args.headless:
    command.append('--headless')
if args.out:
    command += ['--out', str(args.out.resolve())]
if args.max_source_seconds is not None:
    command += ['--max-source-seconds', str(args.max_source_seconds)]
subprocess.run(command, check=True)

"""Validate the explicit official model assembly-overlap policy; no device I/O."""
from pathlib import Path
import hashlib
import json


def load_assembly_policy(path: Path, root: Path, usd_path: Path | None = None) -> dict:
    """Load an explicit, model-bound official assembly-overlap allowlist."""
    policy = json.loads(path.read_text())
    digest = lambda p: hashlib.sha256(p.read_bytes()).hexdigest()
    if digest(root / 'outputs/robot/tron2_wuji_hand2.urdf') != policy['urdf_sha256']:
        raise ValueError('Assembly policy URDF binding mismatch')
    if digest(usd_path or root / 'outputs/robot/tron2_wuji_hand2.usd') not in policy['allowed_usd_sha256']:
        raise ValueError('Assembly policy USD binding mismatch')
    for relative, expected in policy['mesh_sha256'].items():
        if digest(root / relative) != expected:
            raise ValueError(f'Assembly policy mesh binding mismatch: {relative}')
    expected = {frozenset((f'{side}_wrist', f'{side}_{digit}_{segment}'))
                for side in ('l', 'r') for digit in ('thumb', 'index_finger', 'middle_finger', 'ring_finger', 'pinky')
                for segment in ('proximal', 'proximal_abd')}
    actual = {frozenset(pair) for pair in policy['link_pairs']}
    if len(policy['link_pairs']) != 20 or actual != expected:
        raise ValueError('Expected the 20 official Wuji mechanical assembly pairs')
    return policy


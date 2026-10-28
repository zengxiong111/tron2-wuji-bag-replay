"""Load the pinned Wuji Beta2 assembly-overlap exclusions from bundled MJCF."""
import hashlib
import re
import xml.etree.ElementTree as ET
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
UPSTREAM = ROOT/'assets/wuji-description'
EXPECTED_UPSTREAM_COMMIT = 'c2cd7f8d1ef8b6dc8cb907c17daa5a88b4442d95'
EXPECTED_SOURCE_SHA256 = {
    'assets/wuji-description/hand2/hand2_beta2/body/mjcf/left_with_mount.xml': '903d80d3fa00ff78efa7f7684cedb768b0a1b960db9f68fdb1d28c03eee3ba22',
    'assets/wuji-description/hand2/hand2_beta2/body/mjcf/right_with_mount.xml': '9bce131e7cf7147b2fdf822cf762ac6ed7a313c449e65c7ff0db3120a617254a',
}
EXPECTED_README_SHA256 = '63d4ec18255205c73cca9def466abfb4cc3291a0f05cfaf6cd3f5ec34e43de53'


def sha(path):
    digest = hashlib.sha256()
    with Path(path).open('rb') as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b''):
            digest.update(block)
    return digest.hexdigest()


def load_official_policy():
    """Return exactly the 20 upstream-authored pair exclusions and source hashes."""
    for relative, expected in EXPECTED_SOURCE_SHA256.items():
        if sha(ROOT/relative) != expected:
            raise ValueError(f'Pinned official contact-policy source changed: {relative}')
    pairs = []
    sources = {}
    for side in ('left', 'right'):
        relative = f'assets/wuji-description/hand2/hand2_beta2/body/mjcf/{side}_with_mount.xml'
        path = ROOT/relative
        root = ET.parse(path).getroot()
        found = [tuple(sorted((node.attrib['body1'], node.attrib['body2'])))
                 for node in root.findall('./contact/exclude')]
        prefix = 'l' if side == 'left' else 'r'
        expected = {tuple(sorted((f'{prefix}_{digit}_{part}', f'{prefix}_wrist')))
                    for digit in ('thumb', 'index_finger', 'middle_finger', 'ring_finger', 'pinky')
                    for part in ('proximal', 'proximal_abd')}
        if len(found) != 10 or set(found) != expected:
            raise ValueError(f'Unexpected official {side} contact exclusions')
        pairs.extend(found)
        sources[relative] = sha(path)
    return {'enabled': True, 'model': 'Wuji hand2_beta2 with_mount',
            'body_pairs': [list(pair) for pair in sorted(set(pairs))], 'official_pair_count': 20,
            'source_sha256': sources, 'expected_source_commit': EXPECTED_UPSTREAM_COMMIT,
            'readme_source_sha256': EXPECTED_README_SHA256,
            'semantics': 'Official model-specific internal assembly-overlap exclusions; no cross-digit, distal/palm, cross-hand, arm or table pair is removed.',
            'boundary': 'Excluded assembly mesh intersections remain geometric overlaps and must not be described as zero all-mesh penetration.'}


def geometry_body_name(model, obj):
    name = model.frames[obj.parentFrame].name if 0 <= obj.parentFrame < len(model.frames) else ''
    return name if name else re.sub(r'_\d+$', '', obj.name)


def apply_geometry_policy(model, geometry):
    """Remove only authored official assembly pairs from an already-built pair set."""
    import pinocchio as pin
    policy = load_official_policy()
    allowed = {tuple(pair) for pair in policy['body_pairs']}
    removed = []
    for pair in list(geometry.collisionPairs):
        first, second = geometry.geometryObjects[pair.first], geometry.geometryObjects[pair.second]
        bodies = tuple(sorted((geometry_body_name(model, first), geometry_body_name(model, second))))
        if bodies in allowed:
            removed.append({'first': first.name, 'second': second.name, 'body_pair': list(bodies),
                            'reason': 'official_wuji_beta2_assembly_overlap'})
            geometry.removeCollisionPair(pin.CollisionPair(pair.first, pair.second))
    policy['additional_removed_pairs'] = removed
    policy['additional_removed_pair_count'] = len(removed)
    return policy

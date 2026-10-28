"""Verify installed asset hashes and acceptance-bound 5x runtime lineage."""
import hashlib
import json
from pathlib import Path
import xml.etree.ElementTree as ET

ROOT = Path(__file__).resolve().parents[1]
CANONICAL_ACCEPTANCE = '2b3f670ebc45de276d1c5729028bbf8f6d4d58119ae68d1a826a0039f160ac8d'
ARTIFACT_ID = '20261027-bag-5x-restored-thumb-v9'
REQUIRED_GATES = (
    'overall_5x', 'complete_source_interval', 'nominal_c2_full_coverage',
    'nominal_c2_all_pairs', 'nominal_c2_zero_collision', 'physics_completed',
    'all_physics_steps_recorded', 'audit_bound_to_actual', 'physics_bound_to_actual',
    'physics_bound_to_reference', 'physics_bound_to_usd', 'runtime_self_collision_enabled',
    'runtime_collision_shapes_retained', 'no_actual_mesh_collision',
    'no_actual_mesh_penetration', 'physical_base_table_gap', 'actual_gap_max',
    'actual_gap_rms', 'actual_native_speed', 'actual_fd_speed',
)


def sha256(path):
    digest = hashlib.sha256()
    with Path(path).open('rb') as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b''):
            digest.update(block)
    return digest.hexdigest()


def local_file(root, relative):
    relative_path = Path(relative)
    if relative_path.is_absolute() or '..' in relative_path.parts or not relative_path.parts:
        raise ValueError(f'Unsafe repository path: {relative}')
    candidate = root / relative_path
    cursor = candidate
    while cursor != root:
        if cursor.is_symlink():
            raise ValueError(f'Symlink in bound path: {relative}')
        cursor = cursor.parent
    path = candidate.resolve(strict=False)
    if root.resolve() not in path.parents:
        raise ValueError(f'Path escapes repository: {relative}')
    return path



def urdf_semantics_sha256(root, urdf_relative):
    """Hash normalized URDF structure while binding each mesh URI to installed mesh bytes."""
    path = local_file(root, urdf_relative)
    tree = ET.parse(path)

    def node_value(node):
        attributes = []
        for key, value in sorted(node.attrib.items()):
            if node.tag == 'mesh' and key == 'filename':
                relative = value.removeprefix('package://').removeprefix('file://')
                mesh = local_file(root, relative)
                if not mesh.is_file():
                    raise FileNotFoundError(f'Missing URDF mesh {relative}')
                value = 'sha256:' + sha256(mesh)
            attributes.append((key, value))
        return (node.tag, tuple(attributes), (node.text or '').strip(),
                tuple(node_value(child) for child in list(node)))

    canonical = json.dumps(node_value(tree.getroot()), ensure_ascii=False, separators=(',', ':')).encode()
    return hashlib.sha256(canonical).hexdigest()


def verify(root=ROOT):
    root = Path(root).resolve()
    manifest = json.loads((root / 'release_manifest.json').read_text())
    verified = 0
    for asset in manifest.get('assets', []):
        for relative, expected in asset.get('files', {}).items():
            path = local_file(root, relative)
            if not path.is_file():
                raise FileNotFoundError(f'Missing {relative}; run scripts/download_assets.py')
            if sha256(path) != expected:
                raise ValueError(f'Checksum mismatch: {relative}')
            verified += 1

    binding = json.loads((root / 'configs/trajectory.json').read_text())
    if binding.get('schema_version') != 2 or binding.get('motion_speed') != 5 or binding.get('file_time_multiplier') != 1:
        raise ValueError('Runtime binding must declare schema v2, overall 5x, and 1x execution-time playback')
    if binding.get('execution_time_is_final') is not True:
        raise ValueError('Execution trajectory is already retimed; final execution-time flag is required')
    profile = binding.get('simulation_profile', {})
    expected_profile = {
        'solver_position_iterations': 32, 'solver_velocity_iterations': 16,
        'global_drive_gain_scale': 4, 'arm_drive_gain_scale': 8,
        'hand_drive_gain_scale': 1, 'hand_velocity_limit_scale': 0.94,
        'arm_force_drive': False, 'rnea_feedforward': False,
        'target_interpolation': 'clamped C2 cubic', 'contact_offset_m': 0.001,
        'rest_offset_m': 0.0, 'official_wuji_assembly_policy_pair_count': 20,
        'left_thumb_flex_gain_scale': 4,
    }
    if profile != expected_profile:
        raise ValueError('Fresh-physics profile differs from the accepted 480 Hz settings')
    artifacts = ('reference', 'execution', 'actual', 'raw_feedback', 'urdf', 'usd', 'provenance', 'retiming_manifest')
    for name in artifacts:
        item = binding.get(name)
        if not isinstance(item, dict) or not item.get('path') or not item.get('sha256'):
            raise ValueError(f'Missing path/hash binding for {name}')
        path = local_file(root, item['path'])
        if not path.is_file() or sha256(path) != item['sha256']:
            raise ValueError(f'Runtime binding mismatch: {name} ({item["path"]})')

    lineage_path = root / 'configs/lineage.json'
    if not lineage_path.is_file():
        raise FileNotFoundError('Missing configs/lineage.json acceptance source/installed binding')
    lineage = json.loads(lineage_path.read_text())
    for group in ('runtime_files', 'imported_evidence'):
        entries = lineage.get(group)
        if not isinstance(entries, dict) or not entries:
            raise ValueError(f'Lineage must hash every file in {group}')
        for relative, expected in entries.items():
            if isinstance(expected, dict):
                expected = expected.get('sha256')
            path = local_file(root, relative)
            if not expected or not path.is_file() or sha256(path) != expected:
                raise ValueError(f'Lineage {group} hash mismatch: {relative}')
    acceptance = lineage.get('acceptance', {})
    source = acceptance.get('source', {})
    installed = acceptance.get('installed', {})
    expected_source = acceptance.get('canonical_source_sha256', CANONICAL_ACCEPTANCE)
    artifact_id = acceptance.get('artifact_id', ARTIFACT_ID)
    if artifact_id != ARTIFACT_ID or expected_source != CANONICAL_ACCEPTANCE:
        raise ValueError('Canonical acceptance artifact identity changed')
    for label, item in (('canonical source', source), ('installed acceptance', installed)):
        if not item.get('path') or not item.get('sha256'):
            raise ValueError(f'Missing {label} path/hash binding')
        path = local_file(root, item['path'])
        if not path.is_file() or sha256(path) != item['sha256']:
            raise ValueError(f'{label} installed bytes do not match lineage')
    if source.get('sha256') != CANONICAL_ACCEPTANCE or installed.get('sha256') != CANONICAL_ACCEPTANCE:
        raise ValueError('Canonical source and installed acceptance must preserve the original acceptance bytes')
    report = acceptance.get('report', {})
    report_path = local_file(root, report.get('path', ''))
    if not report_path.is_file() or sha256(report_path) != report.get('sha256'):
        raise ValueError('Imported authoritative acceptance report hash mismatch')
    report_data = json.loads(report_path.read_text())
    gates = report_data.get('gates', {})
    if report_data.get('accepted') is not True or any(gates.get(name) is not True for name in REQUIRED_GATES):
        raise ValueError('Authoritative acceptance report is not fully accepted for the required 5x gates')
    physics_binding = lineage.get('physics_report', {})
    if not physics_binding.get('path') or not physics_binding.get('sha256'):
        raise ValueError('Lineage must bind the detailed accepted physics report')
    physics_path = local_file(root, physics_binding['path'])
    if not physics_path.is_file() or sha256(physics_path) != physics_binding['sha256']:
        raise ValueError('Detailed physics report hash mismatch')
    physics = json.loads(physics_path.read_text())
    source_installed = lineage.get('source_to_installed_binding', {})
    expected_physics_hashes = {
        'reference_sha256': 'reference', 'execution_sha256': 'execution',
        'raw_source_sha256': 'raw_feedback', 'usd_sha256': 'usd',
        'tracking_actual_sha256': 'actual',
    }
    for report_key, name in expected_physics_hashes.items():
        if physics.get(report_key) != source_installed.get(name, {}).get('source_sha256'):
            raise ValueError(f'Detailed physics report does not bind source {name} bytes')
    expected_physics_values = {
        'status': 'complete', 'physics_hz': 480, 'physics_steps': 33093,
        'saved_states_including_initial_and_final': 33094, 'motion_speed': 5.0,
        'replay_timing_profile': 'overall5/shared_nonuniform_time_map',
        'physical_base_to_table_m': .45, 'solver_iterations': [32, 16],
        'contact_offset_m': .001, 'rest_offset_m': 0.0,
        'hand_velocity_limit_scale': .94, 'drive_gain_scale': 4.0,
        'arm_drive_gain_scale': 8.0, 'hand_drive_gain_scale': 1.0,
        'left_thumb_flex_gain_scale': 4.0,
    }
    for key, expected in expected_physics_values.items():
        if physics.get(key) != expected:
            raise ValueError(f'Detailed physics report differs from accepted setting: {key}')
    if physics.get('runtime_self_collision') != ['True']:
        raise ValueError('Detailed physics report must confirm runtime self-collision')
    drives = physics.get('runtime_drive_settings', {})
    if len(drives) != 56 or any(row.get('drive_type') != 'acceleration' for row in drives.values()):
        raise ValueError('Detailed physics report must preserve acceleration-mode drives across 56 axes')
    ff = physics.get('arm_force_drive_rnea_feedforward', {})
    if ff.get('force_drive_enabled') is not False or ff.get('feedforward_enabled') is not False:
        raise ValueError('Detailed physics report must keep arm force drive and RNEA feedforward disabled')
    policy = physics.get('official_wuji_assembly_policy', {})
    if policy.get('enabled') is not True or policy.get('official_pair_count') != 20:
        raise ValueError('Detailed physics report does not bind the official 20-pair assembly policy')
    # Match installed inputs to the historical source-path keys stored in the accepted report.
    source_keys = acceptance.get('binding_sources', {})
    installed_map = lineage.get('source_to_installed_binding', {})
    for name in ('reference', 'actual', 'raw_feedback', 'urdf', 'usd'):
        source_key = source_keys.get(name)
        mapping = installed_map.get(name, {})
        source_sha = mapping.get('source_sha256')
        installed_sha = mapping.get('installed_sha256')
        if installed_sha != binding[name]['sha256']:
            raise ValueError(f'Lineage installed hash mismatch: {name}')
        if name != 'urdf' and source_sha != installed_sha:
            raise ValueError(f'Source and installed {name} bytes must be identical')
        if name == 'urdf' and mapping.get('transformation') != 'mesh_uri_normalization_only':
            raise ValueError('URDF differences require the declared mesh URI normalization transformation')
        if name in ('reference', 'actual', 'raw_feedback', 'urdf', 'usd'):
            if not source_key or report_data.get('binding', {}).get(source_key) != source_sha:
                raise ValueError(f'Acceptance report does not bind source {name} bytes')
    provenance_data = json.loads(local_file(root, binding['provenance']['path']).read_text())
    provenance_hash_keys = {
        'reference': 'reference_sha256', 'execution': 'execution_sha256',
        'raw_feedback': 'raw_source_sha256', 'urdf': 'urdf_sha256',
        'usd': 'usd_sha256', 'retiming_manifest': 'retiming_manifest_sha256',
    }
    for name, key in provenance_hash_keys.items():
        if provenance_data.get(key) != binding[name]['sha256']:
            raise ValueError(f'Normalized provenance does not bind installed {name} bytes')
    recorded = lineage.get('runtime_bindings', {})
    for name in artifacts:
        expected = binding[name]['sha256']
        if recorded.get(name) != expected:
            raise ValueError(f'Lineage runtime hash mismatch: {name}')
    model_dir = local_file(root, binding['usd']['path']).parent
    model_changes = json.loads((model_dir/'model_changes.json').read_text())
    asset_binding = json.loads((model_dir/'asset_binding.json').read_text())
    semantic_hash = urdf_semantics_sha256(root, binding['urdf']['path'])
    model_identity = lineage.get('model_identity', {})
    if (model_identity.get('urdf_path_only_conversion') is not True
            or model_identity.get('installed_sha256') != binding['urdf']['sha256']
            or model_identity.get('semantic_sha256') != semantic_hash
            or model_identity.get('source_sha256') != installed_map.get('urdf', {}).get('source_sha256')):
        raise ValueError('Installed URDF and mesh semantics differ from the source model identity binding')
    if model_changes.get('usd_sha256') != binding['usd']['sha256']:
        raise ValueError('Current model change metadata does not bind the installed USD')
    if (asset_binding.get('replay_usd_sha256') != binding['usd']['sha256']
            or asset_binding.get('mesh_geometry_and_authored_world_transforms_equal') is not True
            or abs(float(asset_binding.get('physical_base_above_table_m', 0)) - .45) > 1e-8):
        raise ValueError('Current model metadata does not bind the USD geometry and fixed 0.45 m tabletop frame')
    return verified


if __name__ == '__main__':
    print(f'Verified {verify()} release files and accepted 5x runtime lineage')

"""Verify every downloaded runtime input using only the Python standard library."""
import hashlib
import json
from pathlib import Path

root = Path(__file__).resolve().parents[1]
manifest = json.loads((root/'release_manifest.json').read_text())
count = 0
for asset in manifest['assets']:
    for relative, expected in asset['files'].items():
        path = root/relative
        if not path.is_file():
            raise FileNotFoundError('Missing '+relative+'; run scripts/download_assets.py')
        value = hashlib.sha256()
        with path.open('rb') as stream:
            for block in iter(lambda: stream.read(1024*1024), b''):
                value.update(block)
        if value.hexdigest() != expected:
            raise ValueError('Checksum mismatch: '+relative)
        count += 1
print('Verified',count,'runtime files')

# Verify relocated evidence and retain its canonical source-byte identity.
binding = json.loads((root/'evidence/reference/import_binding.json').read_text())
for relative, record in binding['files'].items():
    if hashlib.sha256((root/relative).read_bytes()).hexdigest() != record['installed_sha256']:
        raise ValueError('Imported evidence mismatch: '+relative)

# Bind the accepted run to the installed defaults and video.
a = json.loads((root/'evidence/reference/acceptance.json').read_text())
for field, relative in [('trajectory_sha256', 'trajectories/trajectory_6x.npz'),
                        ('reference_sha256', 'trajectories/repaired_source.npz'),
                        ('actual_trace_sha256', 'trajectories/physical_tracking_6x.npz')]:
    if hashlib.sha256((root/relative).read_bytes()).hexdigest() != a[field]:
        raise ValueError('Acceptance input mismatch: '+relative)
physics = json.loads((root/'configs/physics.json').read_text())
target = json.loads((root/'configs/trajectory.json').read_text())
video = json.loads((root/'evidence/video/recording.json').read_text())
if (physics['reference_sha256'] != a['reference_sha256']
        or target['trajectory_sha256'] != a['trajectory_sha256']
        or physics['usd_sha256'] != a['usd_sha256']
        or video['actual_trace_sha256'] != a['actual_trace_sha256']
        or hashlib.sha256((root/'media/new_6x_full.mp4').read_bytes()).hexdigest() != video['video_sha256']):
    raise ValueError('Configuration/video binding mismatch')
import math
events = json.loads((root/'evidence/physics/digit_contact_events.json').read_text())
if not all(math.isfinite(e[2]) and math.isfinite(e[3]) and e[2] > 0 and e[3] == 0 for e in events):
    raise ValueError('Invalid finger proximity evidence')
if not (a['accepted'] and a['finger_nonzero_impulse_pairs'] == 0
        and a['finger_nonzero_impulse_points'] == 0
        and a['whole_target_mesh_intersections'] == 0
        and a['whole_actual_mesh_intersections'] == 0
        and a['target_linear_path_certified'] and a['recorded_actual_linear_path_certified']
        and all(a['body_head_clock_unchanged'].values()) and a['physics_config_unchanged']):
    raise ValueError('Acceptance conditions failed')
print('Accepted run, configuration and actual-state video bindings verified')

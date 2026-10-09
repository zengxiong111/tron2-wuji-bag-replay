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

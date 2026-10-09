"""Download private GitHub Release assets, verify hashes, and install without overwriting."""
import hashlib
import json
import subprocess
import tarfile
import tempfile
import time
from pathlib import Path

root = Path(__file__).resolve().parents[1]
manifest = json.loads((root/'release_manifest.json').read_text())

def digest(path):
    value = hashlib.sha256()
    with path.open('rb') as stream:
        for block in iter(lambda: stream.read(1024*1024), b''):
            value.update(block)
    return value.hexdigest()

with tempfile.TemporaryDirectory(prefix='tron2-download-') as temporary:
    temporary = Path(temporary)
    for asset in manifest['assets']:
        installed = True
        for relative, expected in asset['files'].items():
            target = root/relative
            parent = target.parent
            unsafe = target.is_symlink()
            while parent != root:
                unsafe = unsafe or parent.is_symlink()
                parent = parent.parent
            if unsafe or not target.is_file() or digest(target) != expected:
                installed = False
                break
        if installed:
            print('Already installed and verified:', asset['name'], flush=True)
            continue
        archive = temporary/asset['name']
        for attempt in range(3):
            archive.unlink(missing_ok=True)
            result = subprocess.run(['gh','release','download',manifest['tag'],'--repo',manifest['repository'],
                                     '--pattern',asset['name'],'--dir',str(temporary)], capture_output=True, text=True)
            if result.returncode == 0:
                break
            if attempt == 2:
                raise RuntimeError('Download failed after 3 attempts: '+asset['name']+'; check GitHub access and network, then rerun.')
            print('Retrying download:', asset['name'], flush=True)
            time.sleep(2*(attempt+1))
        if archive.stat().st_size != asset['size_bytes'] or digest(archive) != asset['sha256']:
            raise RuntimeError('Release asset checksum mismatch: '+asset['name'])
        staged = temporary/'unpack'
        staged.mkdir()
        with tarfile.open(archive, 'r:gz') as tar:
            for member in tar.getmembers():
                path = Path(member.name)
                if path.is_absolute() or '..' in path.parts or not member.isfile():
                    raise ValueError('Unsupported archive member: '+member.name)
            tar.extractall(staged, filter='data')
        # Validate the entire asset before moving any file into the checkout.
        for relative, expected in asset['files'].items():
            file = staged/relative
            if not file.is_file() or digest(file) != expected:
                raise RuntimeError('Installed file checksum mismatch: '+relative)
            target = root/relative
            if target.is_symlink() or (target.exists() and (not target.is_file() or digest(target) != expected)):
                raise FileExistsError('Refusing to overwrite modified input: '+str(target))
            parent = target.parent
            while parent != root:
                if parent.is_symlink():
                    raise ValueError('Symlink in destination path: '+str(parent))
                parent = parent.parent
        for relative in asset['files']:
            target = root/relative
            target.parent.mkdir(parents=True, exist_ok=True)
            if not target.exists():
                # Create exclusively; works even when /tmp and the checkout are different filesystems.
                import shutil
                with target.open('xb') as destination, (staged/relative).open('rb') as source:
                    shutil.copyfileobj(source, destination)
        import shutil
        shutil.rmtree(staged)
        print('Installed and verified:',asset['name'])
print('Ready. Run python3 scripts/verify_bundle.py')

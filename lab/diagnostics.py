"""Small, bounded evidence bundle. Never collect JPEGs, tokens or SSH keys."""
import argparse
import base64
import io
import json
from pathlib import Path
import subprocess
from datetime import datetime, timezone
from zipfile import ZipFile, ZIP_DEFLATED


def tail(path, limit=256*1024):
    with path.open('rb') as f:
        offset = max(0, path.stat().st_size-limit)
        f.seek(offset); data = f.read(limit)
    return data.partition(b'\n')[2] if offset else data


def remote_bundle(rig):
    root = Path(rig['wsl']['data_root'])
    runs = sorted(root.glob('*_LIVE_*/run.json'), reverse=True)[:4]
    buffer = io.BytesIO()
    with ZipFile(buffer, 'w', ZIP_DEFLATED) as z:
        for run in runs:
            folder = run.parent
            paths = [run, folder/'events.jsonl', folder/'camera/capture.json', folder/'camera/frames.csv']
            for pattern in ('motor/*/events.jsonl', 'motor/*/samples.csv', 'motor/*/session.json'):
                paths.extend(sorted(folder.glob(pattern))[:2])
            for p in paths:
                if p.is_file() and not p.is_symlink():
                    z.writestr('laptop/'+p.relative_to(root).as_posix(), tail(p))
    return buffer.getvalue()


def collect(root, rig):
    from connection import wsl_command
    remote, remote_error = None, None
    try:
        reply = subprocess.run(['ssh','-T','-o','BatchMode=yes','-o','ConnectTimeout=10',rig['ssh_host'],
                                wsl_command(rig, 'diagnostics.py')], check=True, capture_output=True, timeout=30)
        remote = base64.b64decode(reply.stdout.strip(), validate=True)
        if len(remote)>12*1024*1024: raise ValueError('Diagnostic reply exceeds size limit')
    except (OSError, ValueError, subprocess.SubprocessError) as exc:
        remote = None
        remote_error = type(exc).__name__+': '+str(exc)
    stamp=datetime.now(timezone.utc).strftime('%Y%m%dT%H%M%S_%fZ')
    target = Path.home()/('Downloads/SpiRob_diagnostics_v070_'+stamp+'.zip')
    target.parent.mkdir(parents=True, exist_ok=True)
    with ZipFile(target, 'w', ZIP_DEFLATED) as out:
        if remote is not None:
            with ZipFile(io.BytesIO(remote)) as z:
                if sum(i.file_size for i in z.infolist())>12*1024*1024:
                    raise ValueError('Expanded diagnostics exceed size limit')
                for info in z.infolist(): out.writestr(info.filename, z.read(info))
        out.writestr('collection_status.json',json.dumps({'remote_error':remote_error},indent=2))
        paths = [root/'config.json', root/'lab.local.json']
        for pattern in ('connection.log', '*_analysis.jsonl', 'run_review.json'):
            paths += sorted((root/'logs').rglob(pattern), key=lambda p:p.stat().st_mtime, reverse=True)[:4]
        for p in paths:
            if p.is_file() and not p.is_symlink(): out.writestr('workstation/'+p.relative_to(root).as_posix(), tail(p))
    print('Created:', target)
    print('No images or credential files included. Size: %.0f KB' % (target.stat().st_size/1024))
    if remote_error:print('Laptop logs unavailable; workstation settings and logs were still collected.')


if __name__ == '__main__':
    p=argparse.ArgumentParser();p.add_argument('--rig',type=Path,required=True)
    args=p.parse_args()
    print(base64.b64encode(remote_bundle(json.loads(args.rig.read_text()))).decode('ascii'))

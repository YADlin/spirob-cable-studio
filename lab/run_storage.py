"""Run retention: collect first, stop hardware, then explicitly keep or discard.

This standard-library module is also deployed to WSL. It never opens a camera
or a serial port. The same lock as the motor owner prevents cleanup during a run.
"""
import argparse
import json
import re
import shutil
import subprocess
import time
from pathlib import Path


def write_json(path, value):
    path = Path(path)
    temporary = path.with_suffix('.tmp')
    temporary.write_text(json.dumps(value, indent=2, allow_nan=False)+'\n')
    for attempt in range(5):
        try:
            temporary.replace(path)
            return
        except PermissionError:
            if attempt == 4: raise
            time.sleep(.1)


def endpoint(rig):
    return dict(host=rig['ssh_host'], data_root=rig['wsl']['data_root'])


def checked_path(root, identifier):
    root = Path(root).resolve()
    if not re.fullmatch(r'[A-Za-z0-9_-]{1,100}', identifier) or '_LIVE_' not in identifier:
        raise ValueError('Invalid live-session identifier')
    path = root/identifier
    if path.is_symlink() or path.resolve().parent != root:
        raise ValueError('Session path is not a direct, owned run folder')
    return path


def shutdown_confirmed(run):
    if run.get('state') not in ('COMPLETE', 'FAILED'):
        return False
    motor, camera = run.get('motor_shutdown') or {}, run.get('camera_shutdown') or {}
    motor_ok = run.get('motor_opened') is False or (
        motor.get('worker_exited') is True and motor.get('disable_confirmed') == [True, True])
    camera_ok = run.get('camera_requested') is False or camera.get('thread_exited') is True
    return bool(motor_ok and camera_ok)


def describe_run(run):
    capture = run.get('camera_shutdown') or {}
    return dict(session=run['session_id'], state=run.get('state'), error=run.get('error'),
                shutdown_confirmed=shutdown_confirmed(run),
                recording=capture.get('recording', {}),
                decision=run.get('retention', {}).get('decision', 'pending'))


def pending_remote(root):
    result = []
    for file in sorted(Path(root).glob('*_LIVE_*/run.json')):
        if file.is_symlink() or file.parent.is_symlink(): continue
        run = json.loads(file.read_text())
        if run.get('session_id') != file.parent.name: continue
        if run.get('retention', {}).get('decision') in ('pending', 'discard_requested'):
            result.append(describe_run(run))
    for receipt in sorted((Path(root)/'.spirob-run-decisions').glob('*.json')):
        if receipt.is_symlink():continue
        previous=json.loads(receipt.read_text())
        if previous.get('decision')=='discard_requested' and not any(r['session']==previous.get('session') for r in result):
            checked_path(root,previous['session'])
            result.append(dict(session=previous['session'],state='FAILED',shutdown_confirmed=True,
                               decision='discard_requested',error='Previously requested discard was interrupted'))
    return result


def decide_remote(root, identifier, decision):
    """Caller holds the rig owner lock. Deletion requires explicit, final consent."""
    if decision not in ('keep', 'discard'): raise ValueError('Choose keep or discard')
    path = checked_path(root, identifier)
    receipts = Path(root)/'.spirob-run-decisions'
    if receipts.is_symlink(): raise ValueError('Receipt folder cannot be a symlink')
    receipt = receipts/(identifier+'.json')
    if receipt.is_symlink(): raise ValueError('Receipt cannot be a symlink')
    if not path.exists():
        if receipt.is_file():
            previous = json.loads(receipt.read_text())
            if previous.get('session') == identifier and (previous.get('decision') == decision or
                    decision=='discard' and previous.get('decision')=='discard_requested'):
                previous['decision']=decision
                write_json(receipt, previous)
                return previous  # A lost reply can be retried without deleting anything else.
        raise ValueError('Session missing; no matching completed decision receipt')
    file = path/'run.json'
    if not file.exists() and receipt.is_file() and decision=='discard':
        previous=json.loads(receipt.read_text())
        if previous.get('session')==identifier and previous.get('decision')=='discard_requested':
            shutil.rmtree(path)
            previous['decision']='discard';write_json(receipt,previous)
            return previous
    if file.is_symlink(): raise ValueError('Run metadata cannot be a symlink')
    run = json.loads(file.read_text())
    if run.get('session_id') != identifier or 'retention' not in run:
        raise ValueError('This folder is not a pending run created by this retention workflow')
    previous = run['retention'].get('decision')
    if previous == 'keep' and decision != 'keep':
        raise ValueError('Previously saved runs are not selected for discard by this workflow')
    if decision == 'discard' and not shutdown_confirmed(run):
        raise ValueError('Run retained: motor/camera shutdown is unconfirmed; discard is unavailable')
    run['retention'] = dict(decision='discard_requested' if decision=='discard' else 'keep',
                            decided_utc_ns=time.time_ns())
    write_json(file, run)
    result = dict(session=identifier, decision=decision, path=str(path))
    if decision == 'discard':
        # Write the consent receipt before deletion, so partial cleanup is retryable.
        receipts.mkdir(exist_ok=True)
        write_json(receipt, dict(result, decision='discard_requested'))
        for attempt in range(5):
            try:
                shutil.rmtree(path)
                break
            except PermissionError:
                if attempt == 4: raise
                time.sleep(.1)
        write_json(receipt, result)
    return result


def remote_request(rig, identifier=None, decision=None):
    from connection import wsl_command
    args = ('--session', identifier, '--decision', decision) if identifier else ('--list',)
    reply = subprocess.run(['ssh', '-T', '-o', 'BatchMode=yes', '-o', 'ConnectTimeout=10',
        rig['ssh_host'], wsl_command(rig, 'run_storage.py', *args)],
        capture_output=True, text=True, timeout=30)
    if reply.returncode:
        raise RuntimeError((reply.stderr or reply.stdout).strip() or 'Run review connection failed')
    return json.loads(reply.stdout)


def local_pending(logs, rig):
    result = []
    for path in sorted((Path(logs)/'runs').glob('*/run_review.json')):
        if path.is_symlink() or path.parent.is_symlink(): continue
        entry = json.loads(path.read_text())
        if entry.get('decision') == 'pending' and entry.get('endpoint') == endpoint(rig):
            result.append(dict(entry, local_path=str(path.parent.resolve())))
    return result


def review_entries(logs, rig):
    local = local_pending(logs, rig)
    remote = remote_request(rig)
    entries = []
    for item in remote:
        matches = [r for r in local if r.get('session') == item['session']]
        entries.append(dict(item, local_paths=[r['local_path'] for r in matches],
                            requested_decision=next((r.get('requested_decision') for r in matches if r.get('requested_decision')),None)))
    for item in local:
        if not any(e.get('session') == item.get('session') for e in entries):
            entries.append(dict(item, local_paths=[item['local_path']], shutdown_confirmed=(
                shutdown_confirmed(item['final']) if item.get('final') else not item.get('remote_started'))))
    return entries


def finish_review(logs, rig, entry, decision):
    """Complete remote action before local deletion. Failures remain recoverable."""
    allowed_root = (Path(logs)/'runs').resolve()
    local = []
    for value in entry.get('local_paths', []):
        folder = Path(value)
        if folder.is_symlink() or folder.resolve().parent != allowed_root:
            raise ValueError('Local review path is outside this run-log folder')
        journal = folder/'run_review.json'
        if journal.is_symlink(): raise ValueError('Review metadata cannot be a symlink')
        data = json.loads(journal.read_text())
        if data.get('endpoint') != endpoint(rig) or data.get('session') != entry.get('session'):
            raise ValueError('Review identity does not match this rig and run')
        local.append((folder, journal, data))
    if decision not in ('keep', 'discard'): raise ValueError('Choose keep or discard')
    if decision=='discard' and not entry.get('session') and any(data.get('remote_started') for _,_,data in local):
        raise ValueError('Run identity was not received. Keep these logs for recovery and review the laptop runs first.')
    for _, journal, data in local:
        data['requested_decision'] = decision
        write_json(journal, data)
    result = remote_request(rig, entry['session'], decision) if entry.get('session') else {'decision':decision}
    for folder, journal, data in local:
        if decision == 'discard':
            # Keep the recovery journal until every data file has been removed.
            for file in folder.iterdir():
                if file==journal:continue
                if file.is_dir() and not file.is_symlink():shutil.rmtree(file)
                else:file.unlink()
            journal.unlink();folder.rmdir()
        else:
            data['decision'] = 'keep'; write_json(journal, data)
    return result


def main():
    import fcntl
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--rig', type=Path, required=True)
    parser.add_argument('--list', action='store_true')
    parser.add_argument('--session')
    parser.add_argument('--decision', choices=('keep','discard'))
    args = parser.parse_args()
    rig = json.loads(args.rig.read_text())['wsl']
    if args.list:
        print(json.dumps(pending_remote(rig['data_root'])))
        return
    with (Path(rig['motor_repo'])/'.spirob-live.lock').open('a') as lock:
        fcntl.flock(lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
        print(json.dumps(decide_remote(rig['data_root'], args.session, args.decision)))


if __name__ == '__main__':
    try:
        main()
    except Exception as exc:
        import sys
        print(str(exc), file=sys.stderr)
        sys.exit(1)

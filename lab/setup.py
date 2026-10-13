"""One-time setup and occasional connection maintenance. Standard library only."""
import argparse
import json
from pathlib import Path, PureWindowsPath
import re
import shutil
import subprocess
import tempfile
from connection import powershell, ps_quote, wsl_command

ROOT = Path(__file__).resolve().parent
RIG = ROOT/'rig.local.json'


def ssh(rig, command, tty=False):
    return subprocess.run(['ssh', '-t' if tty else '-T', '-o', 'ConnectTimeout=10',
                           rig['ssh_host'], command], check=True)


def camera_maintenance(rig, stop=False):
    """Address only the authenticated SpiRob service; never kill a port owner."""
    token_path=ps_quote(rig['windows']['token_file'])
    command = (
        "if (-not (Get-NetTCPConnection -State Listen -LocalPort 8765 -ErrorAction SilentlyContinue)) "
        "{ Write-Output 'No camera service is listening'; exit 0 }; "
        "$token=(Get-Content -Raw "+token_path+").Trim(); "
        "$headers=@{Authorization=('Bearer '+$token)}; "
        "$health=Invoke-RestMethod -Uri 'http://127.0.0.1:8765/health' -Headers $headers -TimeoutSec 3; "
        "if ($health.service -ne 'spirob-live') { throw 'Port 8765 is not the authenticated SpiRob camera service' }; "
        "Write-Output ('Camera version '+$health.version+'; capture '+$health.capture_state); ")
    if stop:
        command += (
            "if ($health.capture_state -in @('STARTING','STREAMING','RECORDING')) "
            "{ throw 'Capture is active. Use Disconnect in its control window, then retry.' }; "
            "Invoke-RestMethod -Method Post -Uri 'http://127.0.0.1:8765/shutdown-service' "
            "-Headers $headers -Body '{}' -ContentType 'application/json' -TimeoutSec 4 | Out-Null; "
            "for ($attempt=0; $attempt -lt 30; $attempt++) { "
            "if (-not (Get-NetTCPConnection -State Listen -LocalPort 8765 -ErrorAction SilentlyContinue)) "
            "{ Write-Output 'Idle camera service closed'; exit 0 }; Start-Sleep -Milliseconds 100 }; "
            "throw 'Service shutdown requested but port remains occupied; inspect camera-status'")
    ssh(rig,powershell(command))


def deploy(rig):
    camera_maintenance(rig, stop=True)
    windows = rig['windows']
    runtime = PureWindowsPath(windows['runtime'])
    if runtime.name != 'SpiRob_Simple_Runtime' or runtime.parent != PureWindowsPath(windows['profile']):
        raise ValueError('This version deploys only to the dedicated SpiRob_Simple_Runtime folder')
    script = ("$ErrorActionPreference='Stop'; if ($env:USERPROFILE -ne "+ps_quote(windows['profile'])+
        ") { throw 'SSH profile does not match the configured Windows profile' }; "
        "if (Get-NetTCPConnection -State Listen -LocalPort 8765 -ErrorAction SilentlyContinue) "
        "{ throw 'Finish acquisition and stop the existing camera service before deployment' }; "
        'New-Item -ItemType Directory -Force '+ps_quote(runtime)+' | Out-Null')
    ssh(rig, powershell(script))
    with tempfile.TemporaryDirectory() as tmp:
        stage = Path(tmp)/runtime.name
        (stage/'spirob_live').mkdir(parents=True)
        for name in ('connection.py', 'actuation.py', 'motion_limits.py', 'motion_profile.py', 'run_storage.py', 'unlock_camera.sh', 'diagnostics.py'):
            shutil.copy2(ROOT/name, stage/name)
        for name in ('__init__.py', 'server.py', 'packets.py', 'capture.py'):
            shutil.copy2(ROOT/'spirob_live'/name, stage/'spirob_live'/name)
        shutil.copy2(RIG, stage/'rig.local.json')
        (stage/'DEPLOYMENT.txt').write_text('SpiRob Lab 0.9.0. Generated copy; edit on workstation and redeploy.\n')
        subprocess.run(['scp', '-r', str(stage), rig['ssh_host']+':.'], check=True)
    token = Path(rig['workstation_token_file']).expanduser()
    if not token.is_file():
        token.parent.mkdir(parents=True, exist_ok=True)
        relative = PureWindowsPath(windows['token_file']).relative_to(PureWindowsPath(windows['profile'])).as_posix()
        with tempfile.NamedTemporaryFile(dir=token.parent, delete=False) as f:
            temporary = Path(f.name)
        try:
            temporary.chmod(0o600)
            subprocess.run(['scp', rig['ssh_host']+':'+relative, str(temporary)], check=True)
            if len(temporary.read_text().strip()) < 32:
                raise ValueError('Invalid camera token')
            temporary.replace(token)
        finally:
            temporary.unlink(missing_ok=True)
    print('Deployed to', windows['runtime'])
    print('Existing motor, camera addon and previous runtime were preserved.')


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    commands = parser.add_subparsers(dest='command', required=True)
    p = commands.add_parser('import'); p.add_argument('--from', dest='source', type=Path, required=True)
    for name in ('deploy', 'unlock', 'check', 'usb-list', 'diagnostics', 'camera-status', 'camera-stop'):
        commands.add_parser(name)
    for name in ('usb-attach', 'usb-detach'):
        p = commands.add_parser(name); p.add_argument('--busid', required=True)
    p = commands.add_parser('fetch'); p.add_argument('session')
    args = parser.parse_args()
    if args.command == 'import':
        if RIG.exists():
            raise SystemExit('Keeping existing rig.local.json; edit it explicitly if paths need changing.')
        rig = json.loads(args.source.expanduser().read_text())
        old_name = PureWindowsPath(rig['windows']['runtime']).name
        rig['windows']['runtime'] = str(PureWindowsPath(rig['windows']['profile'])/'SpiRob_Simple_Runtime')
        parent = rig['wsl']['runtime'].rstrip('/').rsplit('/', 1)[0]
        rig['wsl']['runtime'] = parent+'/SpiRob_Simple_Runtime'
        RIG.write_text(json.dumps(rig, indent=2)+'\n'); RIG.chmod(0o600)
        print('Imported settings. New deployment folder: SpiRob_Simple_Runtime. Previous:', old_name)
        return
    rig = json.loads(RIG.read_text())
    if args.command == 'deploy':
        deploy(rig)
    elif args.command in ('camera-status','camera-stop'):
        camera_maintenance(rig, stop=args.command=='camera-stop')
    elif args.command == 'diagnostics':
        from diagnostics import collect
        collect(ROOT, rig)
    elif args.command == 'check':
        profile=None
        for path in (ROOT/'config.json',ROOT/'lab.local.json'):
            if path.is_file():profile=json.loads(path.read_text()).get('motion_profile',profile)
        extra=[]
        if profile:
            from motion_profile import encode_profile
            extra=['--motion-profile-b64',encode_profile(profile)]
        ssh(rig, wsl_command(rig, 'actuation.py', '--check-only',*extra))
    elif args.command == 'unlock':
        words = ['wsl.exe', '-u', rig['wsl']['user'], '--exec', '/bin/bash',
                 rig['wsl']['runtime']+'/unlock_camera.sh', rig['wsl']['camera_ssh_key']]
        if any(not re.fullmatch(r'[A-Za-z0-9_./-]+', word) for word in words):
            raise ValueError('Use WSL paths without spaces or shell characters')
        ssh(rig, ' '.join(words), tty=True)
    elif args.command == 'usb-list':
        ssh(rig, 'usbipd.exe list')
    elif args.command in ('usb-attach', 'usb-detach'):
        if not re.fullmatch(r'\d+-\d+(?:\.\d+)*', args.busid):
            raise ValueError('Invalid USB bus ID')
        command = 'usbipd.exe '+('attach --wsl' if args.command == 'usb-attach' else 'detach')+' --busid '+args.busid
        ssh(rig, command)
    elif args.command == 'fetch':
        if not re.fullmatch(r'[A-Za-z0-9_-]{1,100}', args.session):
            raise ValueError('Use the exact session folder name from the window')
        remote = PureWindowsPath(rig['windows']['data_root']).relative_to(PureWindowsPath(rig['windows']['profile'])).as_posix()
        (ROOT/'data').mkdir(exist_ok=True)
        subprocess.run(['scp', '-r', rig['ssh_host']+':'+remote+'/'+args.session, str(ROOT/'data')], check=True)


if __name__ == '__main__':
    try:
        main()
    except subprocess.CalledProcessError as exc:
        raise SystemExit(f'Remote command failed (exit {exc.returncode}). See the plain-text message above.')
    except (OSError, ValueError) as exc:
        raise SystemExit(str(exc))

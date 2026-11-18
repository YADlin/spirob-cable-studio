"""One-time setup and occasional connection maintenance. Standard library only.

The motor owner is the laptop (Windows + WSL) or a Raspberry Pi ("host": "pi" in
rig.local.json). Commands that only make sense for one of them say so.
"""
import argparse
import io
import json
from pathlib import Path, PureWindowsPath
import re
import secrets
import shlex
import shutil
import subprocess
import tarfile
import tempfile
from connection import powershell, ps_quote, wsl_command
from hosts import is_pi, remote_section

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
        for name in ('connection.py', 'hosts.py', 'actuation.py', 'motion_limits.py', 'motion_profile.py', 'run_storage.py', 'unlock_camera.sh', 'diagnostics.py'):
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


# --- Raspberry Pi -------------------------------------------------------------------

PI_LAB_FILES = ('connection.py', 'hosts.py', 'actuation.py', 'motion_limits.py', 'motion_profile.py',
                'run_storage.py', 'diagnostics.py', 'camera_maintenance.py', 'requirements-pi.txt',
                'spirob_live/__init__.py', 'spirob_live/server.py', 'spirob_live/packets.py',
                'spirob_live/capture.py')
PI_PACKAGES = ('spirob_cable', 'spirob_experiment')


def pi_bundle():
    """The workstation's own code, so the Pi always runs exactly what the GUI expects."""
    buffer = io.BytesIO()
    with tarfile.open(fileobj=buffer, mode='w:gz') as tar:
        for name in PI_LAB_FILES:
            tar.add(ROOT/name, arcname='lab/'+name)
        for package in PI_PACKAGES:
            for path in sorted((ROOT.parent/package).glob('*.py')):
                tar.add(path, arcname=package+'/'+path.name)
        tar.add(RIG, arcname='lab/rig.local.json')
    return buffer.getvalue()


def pi_token(rig):
    """The workstation and the Pi share one camera token. Create it once, then copy it over."""
    pi = remote_section(rig)
    token = Path(rig['workstation_token_file']).expanduser()
    if not token.is_file():
        token.parent.mkdir(parents=True, exist_ok=True)
        token.write_text(secrets.token_hex(32)+'\n')
        token.chmod(0o600)
        print('Created camera token', token)
    remote = shlex.quote(pi['token_file'])
    subprocess.run(['ssh', '-T', '-o', 'ConnectTimeout=10', rig['ssh_host'],
                    f'umask 077 && mkdir -p "$(dirname {remote})" && cat > {remote}'],
                   input=token.read_bytes(), check=True)


def pi_deploy(rig):
    pi = remote_section(rig)
    root, lab, python = (shlex.quote(pi[k]) for k in ('root', 'runtime', 'python'))
    # An idle camera service would keep running the old code; a busy one must not be touched.
    ssh(rig, f'if [ -f {lab}/camera_maintenance.py ] && [ -x {python} ]; then '
             f'cd {lab} && {python} camera_maintenance.py --rig {lab}/rig.local.json --stop; fi')
    subprocess.run(['ssh', '-T', '-o', 'ConnectTimeout=10', rig['ssh_host'],
                    f'mkdir -p {root} && tar -xzf - -C {root}'], input=pi_bundle(), check=True)
    uv = '"$HOME/.local/bin/uv"'
    ssh(rig, f'{uv} venv --quiet --python 3.12 --allow-existing {root}/.venv && '
             f'{uv} pip sync --quiet --python {python} {lab}/requirements-pi.txt && '
             f'mkdir -p {shlex.quote(pi["data_root"])}')
    pi_token(rig)
    missing = subprocess.run(['ssh', '-T', '-o', 'ConnectTimeout=10', rig['ssh_host'],
                              f'test -f {root}/config.json'], check=False).returncode != 0
    print('Deployed the Lab runtime to', pi['root'], 'on', rig['ssh_host'])
    if missing:
        print('Next: the Pi has no motor configuration yet. Copy the laptop\'s config.json to',
              pi['root']+'/config.json', 'and set each axis port to the /dev/serial/by-id/... path.')
    else:
        print('Next: python3 setup.py check')


def pi_only_on_laptop(command):
    raise ValueError(f'"{command}" is for the laptop (Windows/WSL); the Raspberry Pi does not need it.')


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    commands = parser.add_subparsers(dest='command', required=True)
    p = commands.add_parser('import'); p.add_argument('--from', dest='source', type=Path, required=True)
    for name in ('deploy', 'unlock', 'check', 'usb-list', 'diagnostics', 'camera-status', 'camera-stop', 'pi-token'):
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
    if is_pi(rig) and args.command in ('unlock', 'usb-list', 'usb-attach', 'usb-detach'):
        pi_only_on_laptop(args.command)
    if args.command == 'deploy':
        pi_deploy(rig) if is_pi(rig) else deploy(rig)
    elif args.command == 'pi-token':
        pi_token(rig)
    elif args.command in ('camera-status','camera-stop'):
        if is_pi(rig):
            ssh(rig, wsl_command(rig, 'camera_maintenance.py', *(['--stop'] if args.command == 'camera-stop' else [])))
        else:
            camera_maintenance(rig, stop=args.command=='camera-stop')
    elif args.command == 'diagnostics':
        from diagnostics import collect
        collect(ROOT, rig)
    elif args.command == 'check':
        from model_check import check_config
        model_ok, model_lines = check_config(ROOT/'config.json')
        print('Model XML check:\n  ' + '\n  '.join(model_lines))
        if not model_ok: print('WARNING: model/spirob_robot.xml does not match the horizontal rig.')
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
        if is_pi(rig):
            remote = remote_section(rig)['data_root'].rstrip('/')
        else:
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

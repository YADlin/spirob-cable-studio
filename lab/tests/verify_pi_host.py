"""The Raspberry Pi host mode: remote commands, the deployed bundle, and a GUI-free runtime.

Builds the exact bundle `setup.py deploy` sends, unpacks it, and runs the Pi's read-only
motor check and camera-service status/stop with the current interpreter.
"""
import io
import json
from pathlib import Path
import secrets
import subprocess
import sys
import tarfile
import tempfile
import time

LAB = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(LAB))
import connection
import hosts
import run_storage

checks = []


def check(name, ok, detail=''):
    checks.append(ok)
    print(('PASS ' if ok else 'FAIL ') + name + ('' if ok else '  ' + str(detail)))


pi_rig = json.loads((LAB/'rig.pi.example.json').read_text())
laptop_rig = json.loads((LAB/'rig.example.json').read_text())

# 1. Commands for each host
pi_cmd = connection.wsl_command(pi_rig, 'actuation.py', '--check-only')
check('Pi command runs the deployed runtime with the Pi venv',
      pi_cmd == 'cd /home/user/spirob-lab/lab && exec /home/user/spirob-lab/.venv/bin/python -u '
                '/home/user/spirob-lab/lab/actuation.py --rig /home/user/spirob-lab/lab/rig.local.json --check-only', pi_cmd)
check('Pi camera command starts spirob_live.server locally',
      connection.camera_command(pi_rig).endswith('-m spirob_live.server --rig /home/user/spirob-lab/lab/rig.local.json'))
lap_cmd = connection.wsl_command(laptop_rig, 'actuation.py', '--check-only')
check('Laptop command is unchanged', lap_cmd.startswith('wsl.exe -u user --exec ') and lap_cmd.endswith(' --check-only'), lap_cmd)
check('Laptop camera command is still PowerShell', connection.camera_command(laptop_rig).startswith('powershell.exe '))
check('Hosts default to the laptop', not hosts.is_pi(laptop_rig) and hosts.remote_section(laptop_rig) is laptop_rig['wsl'])
try:
    connection.wsl_command(dict(pi_rig, pi=dict(pi_rig['pi'], root='/home/user/my lab')), 'x.py')
    check('Paths with spaces are refused', False)
except ValueError:
    check('Paths with spaces are refused', True)
check('Run storage points at the Pi data folder',
      run_storage.endpoint(pi_rig) == {'host': 'user@100.73.176.88', 'data_root': '/home/user/SpiRob_Data'})

# 2. The deployed bundle, unpacked into a scratch Pi root
with tempfile.TemporaryDirectory() as scratch:
    root = Path(scratch)
    rig_file = LAB/'rig.local.json'
    existing = rig_file.read_bytes() if rig_file.exists() else None
    rig_file.write_text(json.dumps(pi_rig))
    try:
        import setup
        bundle = setup.pi_bundle()
    finally:
        rig_file.write_bytes(existing) if existing is not None else rig_file.unlink()
    with tarfile.open(fileobj=io.BytesIO(bundle)) as tar:
        names = tar.getnames()
        tar.extractall(root, filter='data')
    needed = {'lab/' + n for n in setup.PI_LAB_FILES} | {'lab/rig.local.json', 'spirob_cable/worker.py',
                                                          'spirob_experiment/camera.py'}
    check('Bundle holds the runtime, motor and capture code', needed <= set(names), needed - set(names))
    check('Bundle carries no credentials', not any('token' in n or n.endswith('.key') for n in names))

    rig = json.loads((root/'lab/rig.local.json').read_text())
    rig['pi'].update(root=str(root), data_root=str(root/'data'), token_file=str(root/'token.txt'),
                     python=sys.executable)
    (root/'lab/rig.local.json').write_text(json.dumps(rig))
    (root/'token.txt').write_text(secrets.token_hex(32))
    (root/'data').mkdir()
    config = json.loads((LAB.parent/'config.example.json').read_text())
    for axis, device in zip(config['axes'], (5, 7)):
        axis.update(device_id=device, payout_sign=1, rest_length_mm=220.0, diameter_basis='core')
    (root/'config.json').write_text(json.dumps(config))

    run = lambda *args, **kw: subprocess.run([sys.executable, '-u', *args, '--rig', 'rig.local.json'],
                                             cwd=root/'lab', capture_output=True, text=True, timeout=60, **kw)
    result = run('actuation.py', '--check-only')
    reply = json.loads(result.stdout.strip().splitlines()[-1]) if result.stdout.strip() else {}
    check('Read-only motor check works on the Pi layout',
          reply.get('type') == 'check' and reply.get('host') == 'pi'
          and [a['device_id'] for a in reply.get('axes', [])] == [5, 7], result.stdout + result.stderr)

    result = run('camera_maintenance.py')
    check('Camera status reports an idle Pi cleanly', result.returncode == 0 and 'No camera service' in result.stdout,
          result.stdout + result.stderr)
    # The real camera service code from the bundle, with a synthetic camera instead of /dev/video0.
    serve_code = (
        f"import sys; sys.path[:0] = [{str(root/'lab')!r}, {str(root)!r}]\n"
        "from spirob_live.server import add_observer\n"
        "from spirob_live.capture import LiveCameraService\n"
        "from spirob_experiment.camera import SyntheticCamera, serve\n"
        f"svc = LiveCameraService({str(root/'data')!r}, open({str(root/'token.txt')!r}).read().strip(), "
        "lambda: SyntheticCamera(fps=25))\n"
        "add_observer(serve(svc, '127.0.0.1', 8765), svc).serve_forever()\n")
    server = subprocess.Popen([sys.executable, '-u', '-c', serve_code], cwd=root/'lab',
                              stdout=subprocess.DEVNULL, stderr=subprocess.PIPE, text=True)
    try:
        for _ in range(100):
            if run('camera_maintenance.py').stdout.startswith('Camera version'):
                break
            time.sleep(.1)
        status = run('camera_maintenance.py')
        check('Camera status reads the authenticated service', status.stdout.startswith('Camera version 0.7.0'),
              status.stdout + status.stderr)
        stop = run('camera_maintenance.py', '--stop')
        check('Camera stop closes an idle service', 'Idle camera service closed' in stop.stdout, stop.stdout + stop.stderr)
        server.wait(10)
    finally:
        if server.poll() is None:
            server.kill()

print(f'{sum(checks)}/{len(checks)} checks passed')
sys.exit(0 if all(checks) else 1)

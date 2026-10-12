"""1. Connection: SSH transports and the camera link; no motor calculations.

WorkstationSession runs on Ubuntu. CameraTunnel runs inside laptop WSL.
Commands are newline-delimited JSON over SSH stdin; replies use SSH stdout.
The camera uses the existing authenticated HTTP service through SSH.
"""
import base64
import json
import queue
import re
import socket
import subprocess
import threading
import time
import tempfile
import uuid
from run_storage import write_json, endpoint
from pathlib import Path
from urllib.request import Request, build_opener, ProxyHandler


def camera_get(url, token, route='/health', timeout=1):
    request = Request(url + route, headers={'Authorization': 'Bearer ' + token})
    with build_opener(ProxyHandler({})).open(request, timeout=timeout) as reply:
        data = reply.read(12 * 1024 * 1024 + 1)
    if len(data) > 12 * 1024 * 1024:
        raise ValueError('Camera response exceeds size limit')
    return json.loads(data)


def camera_post(url, token, route, data, timeout=4):
    request = Request(url+route, json.dumps(data, allow_nan=False).encode(),
        {'Authorization':'Bearer '+token, 'Content-Type':'application/json'})
    with build_opener(ProxyHandler({})).open(request, timeout=timeout) as reply:
        return json.loads(reply.read(16384))


def windows_host():
    try:
        mode = subprocess.check_output(['wslinfo', '--networking-mode'], text=True).strip()
        if mode == 'mirrored':
            return '127.0.0.1'
    except (FileNotFoundError, subprocess.CalledProcessError):
        pass
    routes = json.loads(subprocess.check_output(['ip', '-j', 'route', 'show', 'default'], text=True))
    return routes[0]['gateway']


class CameraTunnel:
    """WSL -> Windows camera. Close only the SSH process we start."""
    def __init__(self, rig, token):
        self.rig, self.token, self.process = rig, token, None

    def __enter__(self):
        try:
            if camera_get('http://127.0.0.1:8766', self.token)['service'] == 'spirob-live':
                return self
        except Exception:
            pass
        with socket.socket() as sock:
            if sock.connect_ex(('127.0.0.1', 8766)) == 0:
                raise RuntimeError('WSL port 8766 is occupied by an unhealthy camera tunnel.')
        self.process = subprocess.Popen([
            'ssh', '-N', '-T', '-i', self.rig['camera_ssh_key'],
            '-o', 'IdentitiesOnly=yes', '-o', 'BatchMode=yes',
            '-o', 'IdentityAgent=' + str(Path.home() / '.ssh/spirob-camera-agent.sock'),
            '-o', 'ExitOnForwardFailure=yes', '-o', 'ConnectTimeout=10',
            '-o', 'ServerAliveInterval=10', '-o', 'ServerAliveCountMax=3',
            '-L', '127.0.0.1:8766:127.0.0.1:8765',
            self.rig['windows_ssh_user'] + '@' + windows_host()], stdout=subprocess.DEVNULL)
        try:
            deadline = time.monotonic() + 15
            while time.monotonic() < deadline:
                if self.process.poll() is not None:
                    raise RuntimeError('Camera SSH failed. Run: python3 setup.py unlock')
                try:
                    if camera_get('http://127.0.0.1:8766', self.token)['service'] == 'spirob-live':
                        return self
                except Exception:
                    pass
                time.sleep(.1)
            raise RuntimeError('Camera tunnel did not become ready')
        except BaseException:
            self.__exit__(None, None, None)
            raise

    def __exit__(self, *_):
        if self.process:
            self.process.terminate()
            try:
                self.process.wait(5)
            except subprocess.TimeoutExpired:
                self.process.kill()
                self.process.wait()


def wsl_command(rig, filename, *arguments):
    """Avoid nested PowerShell/bash quoting for WSL Python commands."""
    w = rig['wsl']
    words = ['wsl.exe', '-u', w['user'], '--exec',
             w['motor_repo'].rstrip('/') + '/.venv/bin/python', '-u',
             w['runtime'].rstrip('/') + '/' + filename, '--rig',
             w['runtime'].rstrip('/') + '/rig.local.json', *arguments]
    if any(not re.fullmatch(r'[A-Za-z0-9_./-]+', word) for word in words):
        raise ValueError('WSL paths must use simple names without spaces or shell characters.')
    return ' '.join(words)


def powershell(script):
    script = "$ProgressPreference='SilentlyContinue'; $ErrorActionPreference='Stop'; try { "+script+" } catch { [Console]::Error.WriteLine($_.Exception.Message); exit 1 }"
    return 'powershell.exe -NoProfile -OutputFormat Text -EncodedCommand ' + base64.b64encode(script.encode('utf-16-le')).decode()


def ps_quote(value):
    return "'" + str(value).replace("'", "''") + "'"


class WorkstationSession:
    """One live session. The GUI calls touch(); a stalled GUI stops its lease.

    No motion is retried after a lost acknowledgement. A new session starts disabled.
    """
    def __init__(self, rig, log_dir):
        self.rig = rig
        self.token = Path(rig['workstation_token_file']).expanduser().read_text().strip()
        self.url = 'http://127.0.0.1:8765'
        self.events = queue.Queue()
        self.closed = threading.Event()
        self.cancel = threading.Event()
        self.write_lock = threading.Lock()
        self.control = self.camera = None
        self.last_ui = time.monotonic()
        self.last_preview = 0.
        self.sequence = 0
        self.final = None
        self.max_seconds = 600
        self.difference_limit_mm = 70.
        self.ssh_directory = tempfile.TemporaryDirectory(prefix='spirob-ssh-')
        self.log_dir = Path(log_dir).resolve()/'runs'/('session_'+uuid.uuid4().hex)
        self.log_dir.mkdir(parents=True, exist_ok=False)
        self.journal_lock = threading.Lock()
        self.journal = dict(decision='pending', session=None, remote_started=False,
                            endpoint=endpoint(rig), created_utc_ns=time.time_ns())
        self.review_path = self.log_dir/'run_review.json'
        self.update_journal()
        self.stderr = (self.log_dir / 'connection.log').open('a', buffering=1)

    def update_journal(self, **changes):
        with self.journal_lock:
            self.journal.update(changes)
            write_json(self.review_path, self.journal)

    def touch(self):
        self.last_ui = time.monotonic()

    def preview_received(self, age):
        self.last_preview = time.monotonic() - max(0., age)

    def connect(self):
        """Start camera service and WSL owner. Neither operation enables motors."""
        common = self.launch_camera()
        # Persist intent before launch; even a lost session ID remains reviewable.
        self.update_journal(remote_started=True)
        self.control = subprocess.Popen(common + [self.rig['ssh_host'], wsl_command(self.rig, 'actuation.py',
            '--max-seconds', str(self.max_seconds), '--difference-limit-mm', str(self.difference_limit_mm))],
            stdin=subprocess.PIPE, stdout=subprocess.PIPE, stderr=self.stderr, text=True, bufsize=1)
        threading.Thread(target=self._read, daemon=True).start()
        threading.Thread(target=self._heartbeat, daemon=True).start()

    def launch_camera(self):
        with socket.socket() as sock:
            if sock.connect_ex(('127.0.0.1', 8765)) == 0:
                raise RuntimeError('Close the previous Terminal A camera command before Connect.')
        self.events.put({'type':'starting', 'message':'Starting camera service; first launch may take up to 60 seconds'})
        w = self.rig['windows']
        script = ('Set-Location ' + ps_quote(w['runtime']) +
                  '; & uv --system-certs run --offline --no-project --python 3.12 '
                  '--with opencv-python-headless==4.11.0.86 python -m spirob_live.server --rig ' +
                  ps_quote(w['runtime'] + '\\rig.local.json') + '; exit $LASTEXITCODE')
        common = ['ssh', '-T', '-o', 'ConnectTimeout=10', '-o', 'ServerAliveInterval=10',
                  '-o', 'ServerAliveCountMax=3', '-o', 'ControlMaster=auto',
                  '-o', 'ControlPath=' + str(Path(self.ssh_directory.name)/'control')]
        self.camera = subprocess.Popen(common + ['-o', 'ExitOnForwardFailure=yes',
            '-L', '127.0.0.1:8765:127.0.0.1:8765', self.rig['ssh_host'], powershell(script)],
            stdin=subprocess.DEVNULL, stdout=self.stderr, stderr=self.stderr)
        deadline = time.monotonic() + 60
        while time.monotonic() < deadline:
            if self.cancel.is_set():
                raise RuntimeError('Connection cancelled')
            if self.camera.poll() is not None:
                raise RuntimeError('Camera launch failed; inspect logs/connection.log')
            try:
                health = camera_get(self.url, self.token)
                if health.get('version') != '0.7.0':
                    raise RuntimeError('Camera runtime is old; run python3 setup.py deploy')
                break
            except (OSError, ValueError):
                time.sleep(.1)
        else:
            raise RuntimeError('Camera service did not become ready; inspect logs/connection.log')
        return common

    def send(self, action, **values):
        if self.control is None or self.control.poll() is not None:
            raise RuntimeError('Motor connection is not running')
        with self.write_lock:
            self.sequence += 1
            self.control.stdin.write(json.dumps(dict(id=self.sequence, action=action, **values), allow_nan=False) + '\n')
            self.control.stdin.flush()
            return self.sequence

    def _read(self):
        try:
            for line in self.control.stdout:
                try:
                    event = json.loads(line)
                except ValueError:
                    self.stderr.write(line)
                    continue
                if event.get('type') == 'closed':
                    self.final = event
                if event.get('session') and (event['session'] != self.journal.get('session') or event.get('type')=='closed'):
                    try:
                        self.update_journal(session=event['session'], final=self.final)
                    except OSError as exc:
                        self.stderr.write('Recovery journal write failed; laptop run is retained: '+str(exc)+'\n')
                self.events.put(event)
        finally:
            self.closed.set()
            self.events.put({'type': 'transport_closed', 'confirmed': self.final is not None})

    def _heartbeat(self):
        while not self.closed.wait(.25):
            if time.monotonic() - self.last_ui > 1:
                continue
            try:
                self.send('ping', preview_ok=time.monotonic() - self.last_preview < 1.5)
            except (OSError, RuntimeError):
                return

    def disconnect(self):
        """Await motor shutdown, then close camera service and our SSH transports."""
        self.cancel.set()
        if self.control is not None:
            try:
                self.send('disconnect')
            except (OSError, RuntimeError):
                pass
            self.closed.wait(25)
        if self.camera is not None:
            try:
                request = Request(self.url + '/shutdown-service', b'{}',
                                  {'Authorization': 'Bearer ' + self.token})
                with build_opener(ProxyHandler({})).open(request, timeout=4):
                    pass
            except OSError:
                pass
        for process in (self.control, self.camera):
            if process and process.poll() is None:
                process.terminate()
                try:
                    process.wait(3)
                except subprocess.TimeoutExpired:
                    process.kill()
                    process.wait()
        self.closed.set()
        self.ssh_directory.cleanup()
        return self.final

    def finish_storage(self):
        """Close writers only. Retention is a separate, explicit post-run action."""
        if not self.stderr.closed:self.stderr.close()
        self.update_journal(final=self.final)

    def review_entry(self):
        from run_storage import shutdown_confirmed
        return dict(self.journal, local_paths=[str(self.log_dir)],
                    shutdown_confirmed=shutdown_confirmed(self.final) if self.final else self.control is None,
                    recording=((self.final or {}).get('camera_shutdown') or {}).get('recording', {}))


class CameraSetupSession(WorkstationSession):
    """Camera transport only: never launches WSL or creates a motor run."""
    def __init__(self, rig):
        self.rig = rig
        self.token = Path(rig['workstation_token_file']).expanduser().read_text().strip()
        self.url = 'http://127.0.0.1:8765'
        self.events = queue.Queue(); self.cancel = threading.Event(); self.closed = threading.Event()
        self.control = self.camera = None; self.final = None; self.log_dir = None
        self.ssh_directory = tempfile.TemporaryDirectory(prefix='spirob-camera-setup-')
        self.stderr = (Path(self.ssh_directory.name)/'camera.log').open('a', buffering=1)
        self.last_ui = time.monotonic()
        self.identity = dict(session_id='SETUP_'+uuid.uuid4().hex, nonce=uuid.uuid4().hex)

    def connect(self):
        self.launch_camera()
        camera_post(self.url,self.token,'/preview/start',self.identity)
        threading.Thread(target=self._heartbeat,daemon=True).start()

    def _heartbeat(self):
        while not self.closed.wait(.5):
            if time.monotonic()-self.last_ui > 1: continue
            try:
                result=camera_post(self.url,self.token,'/preview/status',self.identity,timeout=2)
                if result['state'] in ('FAILED','COMPLETE'): raise RuntimeError(result.get('error') or 'Preview ended')
            except Exception as exc:
                self.events.put(dict(type='preview_failed',message=str(exc)));return

    def preview_received(self, age): pass

    def disconnect(self):
        self.closed.set(); self.cancel.set()
        try:
            if self.camera is not None:
                deadline=time.monotonic()+8
                while True:
                    result=camera_post(self.url,self.token,'/preview/stop',self.identity)
                    if result.get('thread_exited'): break
                    if time.monotonic()>deadline: raise RuntimeError('Camera setup stop was not confirmed')
                self.final=result
        finally:
            super().disconnect()
            self.stderr.close()
        return self.final

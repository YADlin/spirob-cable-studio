"""2. Actuation: the laptop's single owner of motor commands and recording.

Read targets(), dispatch(), then run_session(). Hardware communication remains
in the existing spirob_cable.Worker, including its limits and watchdog.
No MuJoCo, image fitting, GUI, or automatic motion sequence runs here.
"""
import argparse
import fcntl
import json
import math
import os
import queue
import signal
import sys
import threading
import time
import uuid
from datetime import datetime, timezone
from pathlib import Path
from connection import CameraTunnel
from motion_limits import limits, worker_limits


def targets(mean_mm, difference_mm, speed_mm_s, rests, difference_limit_mm=70.):
    """Full difference D = L1-L2. The legacy driver expects D/2."""
    values = (mean_mm, difference_mm, speed_mm_s)
    if any(isinstance(v, bool) or not isinstance(v, (int, float)) or not math.isfinite(v) for v in values):
        raise ValueError('Mean, difference and speed must be finite numbers')
    if not .05 <= speed_mm_s <= 5:
        raise ValueError('This pilot uses speeds from 0.05 to 5 mm/s')
    policy = limits(rests, difference_limit_mm)
    if abs(difference_mm) > policy['difference_mm']:
        raise ValueError(f"Full cable difference exceeds the selected ±{policy['difference_mm']:g} mm limit")
    lengths = [mean_mm + difference_mm / 2, mean_mm - difference_mm / 2]
    if any(length <= 0 or abs(length - rest) > policy['travel_mm'] for length, rest in zip(lengths, rests)):
        raise ValueError(f"A target exceeds ±{policy['travel_mm']:g} mm travel from rest or is nonpositive")
    return lengths


def independent_targets(lengths, speeds, rests, difference_limit_mm=70.):
    """Absolute cable endpoints with separate speeds; no mean/difference target."""
    if not isinstance(lengths, (list, tuple)) or not isinstance(speeds, (list, tuple)) or len(lengths) != 2 or len(speeds) != 2:
        raise ValueError('Provide two cable lengths and two speeds.')
    if any(isinstance(v, bool) or not isinstance(v, (int, float)) or not math.isfinite(v) for v in [*lengths, *speeds]):
        raise ValueError('Cable lengths and speeds must be finite numbers.')
    if any(not .05 <= speed <= 5. for speed in speeds):
        raise ValueError('Each cable speed must be 0.05..5 mm/s.')
    travel = limits(rests, difference_limit_mm)['travel_mm']
    for i, (length, rest) in enumerate(zip(lengths, rests)):
        if length <= 0 or abs(length-rest) > travel:
            raise ValueError(f'Cable {i+1}: use {max(0.,rest-travel):g}..{rest+travel:g} mm (positive lengths only).')
    return list(lengths)


class Inbox:
    """Bounded normal command queue; Hold/Disconnect never wait behind Move."""
    def __init__(self, stream):
        self.commands = queue.Queue(maxsize=1)
        self.stop = threading.Event()
        self.close = threading.Event()
        self.last_ping = time.monotonic()
        self.last_preview = 0.
        self.reason = 'User disconnected'
        self.last_id = 0
        threading.Thread(target=self.read, args=(stream,), daemon=True).start()

    def clear(self):
        while True:
            try:
                self.commands.get_nowait()
            except queue.Empty:
                return

    def read(self, stream):
        try:
            while True:
                line = stream.readline(4097)
                if not line:
                    self.reason = 'Control SSH input closed'
                    break
                if len(line) > 4096:
                    raise ValueError('Oversized control message')
                message = json.loads(line)
                identifier, action = message['id'], message['action']
                if type(identifier) is not int or identifier <= self.last_id:
                    raise ValueError('Control IDs must increase')
                self.last_id = identifier
                if action == 'ping':
                    self.last_ping = time.monotonic()
                    if message.get('preview_ok') is True:
                        self.last_preview = self.last_ping
                elif action == 'disconnect':
                    self.clear()
                    break
                elif action == 'hold':
                    self.clear()
                    self.stop.set()
                else:
                    message['received_s'] = time.monotonic()
                    try:
                        self.commands.put_nowait(message)
                    except queue.Full:
                        emit(type='rejected', id=identifier, message='Previous command still pending')
        except Exception as exc:
            self.reason = 'Control input failed: ' + str(exc)
        finally:
            self.close.set()


_output_lock = threading.Lock()


def emit(**data):
    try:
        with _output_lock:
            print(json.dumps(data, allow_nan=False), flush=True)
    except (BrokenPipeError, OSError):
        pass  # Local cleanup must run even after the workstation disappears.


def dispatch(message, motor, rests, preview_fresh, difference_limit_mm=70.):
    """Validate a user command before handing it to the existing motor owner."""
    action = message['action']
    if time.monotonic() - message['received_s'] > 1:
        raise ValueError('Command expired; it was not executed')
    if not preview_fresh:
        raise ValueError('Wait for fresh camera video on the workstation')
    if action == 'independent':
        axes = motor.snapshot.get('axes', [])
        if (not motor.referenced or motor.pending or len(axes) != 2 or
                not all(a['state'] == 'HOLDING' and a['enabled'] for a in axes)):
            raise ValueError('Declare rest and wait for both motors to hold before length control.')
        lengths = independent_targets(message.get('lengths_mm'), message.get('speeds_mm_s'), rests, difference_limit_mm)
        motor.submit('independent', {'lengths_mm': lengths, 'speeds_mm_s': list(message['speeds_mm_s'])})
        return 'Independent targets: %.3f / %.3f mm; speeds %.3f / %.3f mm/s' % (*lengths, *message['speeds_mm_s'])
    if action == 'setup_enable':
        if not motor.connected:
            raise ValueError('Setup enable requires both motors connected and disabled')
        motor.submit('enable', {'indices':[0,1]})
        return 'Enabled for small setup jogs. Absolute lengths remain unreferenced.'
    if action in ('jog', 'axis', 'mean'):
        speed=message.get('speed_mm_s')
        if isinstance(speed,bool) or not isinstance(speed,(int,float)) or not math.isfinite(speed) or not .05<=speed<=5:
            raise ValueError('Speed must be 0.05..5 mm/s')
        axes=motor.snapshot.get('axes', [])
        if len(axes)!=2 or motor.pending or not all(a['state']=='HOLDING' and a['enabled'] for a in axes):
            raise ValueError('Wait for both motors to hold before another command')
        if action=='mean':
            if not motor.referenced: raise ValueError('Declare physical rest first')
            mean=message.get('mean_mm')
            diff=axes[0]['absolute_mm']-axes[1]['absolute_mm']
            target=targets(mean,diff,speed,rests,difference_limit_mm)
            motor.submit('modes', {'mode':'mean','mean_mm':mean,'difference_mm':diff/2,'speed_mm_s':speed})
            return 'Mean target %.3f mm; preserve measured full difference %.3f mm' % (mean,diff)
        axis=message.get('axis')
        if type(axis) is not int or axis not in (0,1): raise ValueError('Select cable 1 or 2')
        field='delta_mm' if action=='jog' else 'length_mm'
        value=message.get(field)
        if isinstance(value,bool) or not isinstance(value,(int,float)) or not math.isfinite(value):
            raise ValueError('Cable target must be finite')
        if action=='jog':
            if not 0<abs(value)<=5: raise ValueError('Jog step must be greater than zero and at most 5 mm')
            if motor.referenced:
                lengths=[a['absolute_mm'] for a in axes];lengths[axis]+=value
                targets(sum(lengths)/2,lengths[0]-lengths[1],speed,rests,difference_limit_mm)
            motor.submit('jog', {'axis':axis,'delta_mm':value,'speed_mm_s':speed})
        else:
            if not motor.referenced: raise ValueError('Declare physical rest before absolute cable targets')
            lengths=[a['absolute_mm'] for a in axes];lengths[axis]=value
            independent_targets(lengths,[speed,speed],rests,difference_limit_mm)
            motor.submit('move', {'axis':axis,'value_mm':value,'absolute':True,'speed_mm_s':speed})
        return 'Cable %d: %s %.3f mm; other cable keeps holding' % (axis+1,field,value)
    if action == 'arm':
        if message.get('at_rest') is not True or not motor.connected:
            raise ValueError('Enable requires a disabled connection and confirmation of physical rest')
        motor.submit('enable', {'indices': [0, 1], 'reference_lengths': dict(enumerate(rests))})
        return 'Referencing at declared rest; no homing motion requested'
    if action == 'move':
        if not motor.referenced:
            raise ValueError('Wait until both motors are referenced and holding')
        mean, difference, speed = (message[k] for k in ('mean_mm', 'difference_mm', 'speed_mm_s'))
        lengths = targets(mean, difference, speed, rests,difference_limit_mm)
        motor.submit('modes', {'mode': 'both', 'mean_mm': mean,
                               'difference_mm': difference / 2, 'speed_mm_s': speed})
        return 'Requested targets: %.3f, %.3f mm' % tuple(lengths)
    raise ValueError('Unknown motor command: ' + str(action))


def run_session(rig, cfg, camera, inbox, demo=False, max_seconds=600, difference_limit_mm=70.):
    """Camera on -> motors disabled -> user commands -> motors off -> camera off."""
    from spirob_experiment.motor import Motor
    from spirob_experiment.schema import Trial, Step
    from spirob_experiment.storage import Session as BaseSession
    class Session(BaseSession):
        def save(self):
            for attempt in range(4):
                try:
                    return super().save()
                except PermissionError:
                    if attempt == 3: raise
                    time.sleep(.02)
    cfg.validate()
    if cfg.active_axes != 2 or any(a.rest_length_mm is None for a in cfg.selected):
        raise ValueError('Two configured motors and physical rest lengths are required')
    rests = tuple(float(a.rest_length_mm) for a in cfg.selected)
    policy = limits(rests,difference_limit_mm)
    if not math.isfinite(max_seconds) or not 0 < max_seconds <= 3600:
        raise ValueError('Session duration must be positive and no more than 3600 seconds')
    trial = Trial('interactive_live', rests, 1., 1, 0., 0., max_seconds,
                  (Step(*rests, 0.),))  # Metadata only: this procedure is never executed.
    identifier = datetime.now(timezone.utc).strftime('%Y%m%dT%H%M%S_%fZ') + '_LIVE_' + uuid.uuid4().hex[:6]
    session = Session(rig['data_root'], identifier, trial, demo)
    session.data.update(mode='INTERACTIVE', trial_metadata_only=True, motor_config=cfg.data(),
                        retention={'decision':'pending'}, motor_opened=False, camera_requested=False,
                        command_convention='difference_mm = cable_1_mm - cable_2_mm',
                        session_limit_s=max_seconds, motion_limits=policy, control_lease_s=3)
    session.save()  # Crash recovery must know this run is pending before startup I/O.
    motor, camera_requested, failure = None, False, None
    last_camera, last_report, last_clock_update = 0., 0., 0.
    camera_context = None
    clock_candidates = []
    status = {}
    stage = 'clock synchronization'
    starting_deadline = time.monotonic()+40
    stopping_since = None
    rest_sequence = []

    def pump(starting=False):
        nonlocal last_camera, status, last_clock_update
        if inbox.close.is_set():
            raise InterruptedError(inbox.reason)
        if time.monotonic() - inbox.last_ping > 3:
            raise RuntimeError('Workstation heartbeat lost')
        if time.monotonic() - session.started > max_seconds:
            raise RuntimeError('Interactive session duration limit reached')
        if motor:
            motor.tick()
            # Events are already on disk. Do not accumulate an unbounded duplicate.
            motor.events.clear()
            if any(a['enabled'] for a in motor.snapshot.get('axes', [])) and time.monotonic() - inbox.last_preview > 2:
                raise RuntimeError('Workstation camera preview is stale')
        if time.monotonic() - last_camera >= .25:
            try:
                request_start = time.monotonic()
                status = camera.status()
                request_end = time.monotonic()
            except (TimeoutError, OSError) as exc:
                if starting and time.monotonic() < starting_deadline:
                    emit(type='starting', message='Camera warming up; motors disabled. '+str(exc))
                    last_camera = time.monotonic()
                    return
                raise
            last_camera = time.monotonic()
            if camera_context is not None and 'camera_monotonic_s' in status:
                clock_candidates.append(dict(round_trip_s=request_end-request_start,
                    offset_s=status['camera_monotonic_s']-(request_start+request_end)/2))
                if request_end-last_clock_update >= 1.:
                    best=min(clock_candidates,key=lambda x:x['round_trip_s'])
                    sync={'best':best,'network_uncertainty_s':best['round_trip_s']/2}
                    camera_context['clock_before']=sync
                    camera.call('/context', dict(camera.identity,context=camera_context))
                    session.event('clock_update', **best)
                    clock_candidates.clear();last_clock_update=request_end
            if status['state'] in ('FAILED', 'COMPLETE'):
                raise RuntimeError('Camera stopped: ' + str(status.get('error')))
            if not starting and (status['last_frame_age_s'] is None or status['last_frame_age_s'] > 1):
                raise RuntimeError('Camera frames are stale')
            if status['frame_count'] >= 10 and (status['observed_fps'] or 0) < rig.get('min_fps', 15):
                raise RuntimeError('Camera delivery below configured minimum FPS')

    def wait_for(predicate, seconds, starting=False):
        deadline = time.monotonic() + seconds
        while True:
            pump(starting)
            if predicate():
                return
            if time.monotonic() >= deadline:
                raise TimeoutError('Camera/motor startup timed out')
            time.sleep(.02)

    try:
        emit(type='starting', session=identifier, message='Opening camera; motors remain disabled')
        session.data['clock_before'] = camera.clock()
        session.save()
        nonce = uuid.uuid4().hex
        camera_requested = True
        session.data['camera_requested'] = True
        session.save()
        start_timeout = getattr(camera, 'timeout', None)
        try:
            if start_timeout is not None: camera.timeout = 3.
            stage = 'camera start request'
            try:
                camera.start(identifier, nonce)
            except (TimeoutError, OSError):
                emit(type='starting', message='Camera start reply delayed; checking the same session, without restarting it')
            stage = 'waiting for camera frames'
            wait_for(lambda: status.get('frame_count', 0) >= 10, 35, True)
        finally:
            if start_timeout is not None: camera.timeout = start_timeout
        capture_file = session.path / 'camera/capture.json'
        if not capture_file.exists() or json.loads(capture_file.read_text())['nonce'] != nonce:
            raise RuntimeError('Camera and motor coordinator must share the same data folder')
        if not demo and status.get('source', {}).get('source') != 'opencv':
            raise RuntimeError('Hardware mode requires a real camera source')
        stage = 'connecting motors disabled'
        emit(type='starting', message='Camera frames ready; connecting motors with torque disabled')
        session.data['motor_opened'] = True
        session.save()
        # Worker constructs its Rig in its thread; keep the factory scoped until ready.
        with worker_limits(rests,difference_limit_mm):
            motor = Motor(cfg, demo, session)
            wait_for(lambda: motor.connected, 15)
        camera_context={k: session.data[k] for k in
            ('coordinator_monotonic_s','clock_before','trial','motor_origin_s','motor_log')}
        camera.call('/context', dict(camera.identity,context=camera_context))
        stage = 'interactive session'
        session.data['state'] = 'READY_DISABLED'
        session.save()
        while True:
            pump()
            if inbox.stop.is_set():
                inbox.clear()
                rest_sequence.clear()
                motor.worker.submit('stop')  # Priority path bypasses normal pending command.
                motor.pending = False  # A cleared queued command will not send command_done.
                stopping_since = time.monotonic()
                inbox.stop.clear()
                session.event('interactive_hold')
                emit(type='accepted', message='Hold requested; waiting for stationary motor feedback')
            elif rest_sequence:
                if not motor.pending:
                    index=rest_sequence.pop(0)
                    motor.submit('rest', {'axis':index,'length_mm':rests[index]})
            else:
                try:
                    command = inbox.commands.get_nowait()
                except queue.Empty:
                    command = None
                if command:
                    try:
                        if stopping_since is not None:
                            raise ValueError('Hold is still waiting for stationary feedback')
                        if command['action']=='reference':
                            axes=motor.snapshot.get('axes',[])
                            if (command.get('at_rest') is not True or motor.pending or len(axes)!=2 or
                                any(a['state']!='HOLDING' or not a['enabled'] for a in axes) or
                                any(a['rest_count'] is not None for a in axes) or
                                time.monotonic()-inbox.last_preview>=1.5 or time.monotonic()-command['received_s']>1):
                                raise ValueError('Reference requires setup hold, fresh video and physical-rest confirmation')
                            rest_sequence[:]=[0,1]
                            message='Declaring the two current positions as physical rest; no movement requested'
                        elif command['action'] == 'record':
                            if time.monotonic()-command['received_s'] > 1:
                                raise ValueError('Recording command expired')
                            reply = camera.call('/record', dict(camera.identity, enabled=command.get('enabled')))
                            message = 'Recording: '+reply['reason']
                        else:
                            message = dispatch(command, motor, rests, time.monotonic() - inbox.last_preview < 1.5,difference_limit_mm)
                        session.event('interactive_command', command=command, result=message)
                        emit(type='accepted', id=command['id'], message=message)
                    except (ValueError, RuntimeError, KeyError) as exc:
                        session.event('interactive_rejection', command=command, reason=str(exc))
                        emit(type='rejected', id=command['id'], message=str(exc))
            if time.monotonic() - last_report >= .2:
                axes = motor.snapshot.get('axes', [])
                if stopping_since is not None and len(axes)==2 and all(a['state']=='HOLDING' for a in axes) and not motor.worker.stop_request.is_set() and all(
                        a['last_read'] is not None and a['last_read'] > stopping_since for a in axes):
                    stopping_since = None
                phase = ('STOPPING' if stopping_since is not None else 'BUSY' if rest_sequence or motor.pending
                         else 'HOLDING' if motor.referenced else 'READY_DISABLED' if motor.connected
                         else 'SETUP_HOLD' if len(axes)==2 and all(a['state']=='HOLDING' and a['enabled'] and a['rest_count'] is None for a in axes)
                         else 'PARTIAL_REFERENCE_DISCONNECT' if len(axes)==2 and all(a['state']=='HOLDING' for a in axes) and any(a['rest_count'] is None for a in axes)
                         else 'MOVING_OR_REFERENCING')
                if session.data['state'] != phase:
                    session.data['state'] = phase
                    session.save()
                emit(type='status', session=identifier, phase=phase, axes=axes,
                     motor_ids=[a.device_id for a in cfg.selected], retention='pending',
                     pending=motor.pending, rest_lengths_mm=rests, session_limit_s=max_seconds, motion_limits=policy,
                     camera_fps=status.get('observed_fps'), recording=status.get('recording', {}), elapsed_s=time.monotonic() - session.started)
                last_report = time.monotonic()
            time.sleep(.02)
    except InterruptedError as exc:
        if str(exc) != 'User disconnected':
            failure = str(exc)
    except BaseException as exc:
        failure = f'{stage}: {type(exc).__name__}: {exc}'
    finally:
        emit(type='closing', message='Stopping motors, then releasing torque and closing camera')
        if motor:
            try:
                session.data['motor_shutdown'] = motor.shutdown()
                closed = session.data['motor_shutdown']
                if not closed['worker_exited'] or closed['disable_confirmed'] != [True, True]:
                    failure = (failure or '') + '; motor shutdown UNCONFIRMED'
                session.save()
            except BaseException as exc:
                failure = (failure or '') + '; motor shutdown failed: ' + str(exc)
        if camera_requested:
            try:
                camera_deadline = time.monotonic()+8
                while True:
                    try:
                        session.data['camera_shutdown'] = camera.stop()
                        if session.data['camera_shutdown'].get('thread_exited'): break
                    except (TimeoutError, OSError):
                        if time.monotonic() >= camera_deadline: raise
                    if time.monotonic() >= camera_deadline: break
                    time.sleep(.1)
                if not session.data['camera_shutdown'].get('thread_exited') or session.data['camera_shutdown'].get('state') != 'COMPLETE':
                    failure = (failure or '') + '; camera shutdown incomplete or failed'
                session.data['clock_after'] = camera.clock()
            except BaseException as exc:
                failure = (failure or '') + '; camera shutdown failed: ' + str(exc)
        session.data.update(state='FAILED' if failure else 'COMPLETE', error=failure)
        session.close()
        emit(type='closed', session=identifier, path=str(session.path), state=session.data['state'],
             error=failure, motor_shutdown=session.data.get('motor_shutdown'),
             camera_shutdown=session.data.get('camera_shutdown'),
             motor_opened=session.data['motor_opened'], camera_requested=camera_requested,
             data_saved=True, retention='pending',
             storage_message='Run retained until you choose Save or Discard after shutdown')
    return session.data


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--rig', type=Path, required=True)
    parser.add_argument('--check-only', action='store_true')
    parser.add_argument('--max-seconds', type=int, choices=(600,1200,1800,2700,3600), default=600)
    parser.add_argument('--difference-limit-mm', type=float, default=70.)
    args = parser.parse_args()
    rig = json.loads(args.rig.read_text())['wsl']
    repo = Path(rig['motor_repo'])
    os.chdir(repo)
    sys.path.insert(0, str(repo))
    from spirob_cable.config import RigSettings
    from spirob_experiment.camera_client import CameraClient
    cfg = RigSettings.load(repo / 'config.json')
    if args.check_only:
        cfg.validate()
        emit(type='check', motor_repo=str(repo), axes=[{'port': a.port, 'device_id': a.device_id,
             'rest_length_mm': a.rest_length_mm, 'usb_present': Path(a.port).exists()} for a in cfg.selected],
             camera_key_present=Path(rig['camera_ssh_key']).is_file(),
             camera_agent_socket_present=(Path.home()/'.ssh/spirob-camera-agent.sock').is_socket(),
             message='Read-only configuration check; motors and camera were not opened')
        return 0
    for axis in cfg.selected:
        if not os.access(axis.port, os.R_OK | os.W_OK):
            raise RuntimeError(f'Motor USB unavailable: {axis.port}. Attach the CP2102 to WSL.')
    token = Path(rig['token_file']).read_text().strip()
    inbox = Inbox(sys.stdin)
    def interrupted(*_):
        inbox.reason = 'Coordinator received termination signal'
        inbox.close.set()
    for signum in (signal.SIGTERM, signal.SIGHUP, signal.SIGINT):
        signal.signal(signum, interrupted)
    with (repo / '.spirob-live.lock').open('a') as lock:
        fcntl.flock(lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
        with CameraTunnel(rig, token):
            result = run_session(rig, cfg, CameraClient('http://127.0.0.1:8766', token), inbox,
                                 max_seconds=args.max_seconds,difference_limit_mm=args.difference_limit_mm)
    return 0 if result['state'] == 'COMPLETE' else 1


if __name__ == '__main__':
    try:
        sys.exit(main())
    except Exception as exc:
        emit(type='startup_failed', message=str(exc))
        sys.exit(1)

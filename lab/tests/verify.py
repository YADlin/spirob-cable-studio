"""Focused checks: scientific invariants, actual demo worker, control-link failure.

No serial devices are opened. Pass existing motor/capture source paths for the
full demo-driver checks. The core scientific checks need only this project.
"""
import argparse
import copy
import io
import json
import os
from pathlib import Path
import queue
import sys
import tempfile
import threading
import time
import importlib.util
import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
os.chdir(ROOT)
from analysis import Comparator, point_cost
from actuation import targets, Inbox, dispatch
from spirob_analysis.measurement import measure
from spirob_analysis.session import Session


def require(condition, message):
    if not condition:
        raise AssertionError(message)


def packet_for(step, config):
    session = Session(config['session'])
    jpeg, provenance = session.held_frame(step)
    t = provenance['t_s']; rest = session.trial['rest_lengths_mm']
    rows = []
    for axis, (times, lengths) in enumerate(session.motor_series()):
        rows.extend(dict(axis=axis, t_s=float(ti), length_mm=float(length+rest[axis]),
                    target_mm=float(session.displacement_mm(step)[axis]+rest[axis]), state='HOLDING')
                    for ti, length in zip(times, lengths) if t-4 <= ti <= t+1)
    return dict(session='recorded', state='RECORDING', t_s=t, motor=rows, motor_active=True,
                rest_lengths_mm=rest, frame_age_s=0, motor_age_s=[0, 0]), jpeg


def science_checks(config, fit):
    model = Comparator(config, fit)
    errors = []
    observations = []
    for step in (1, 2, 3, 4):
        packet, jpeg = packet_for(step, config)
        _, observed = measure(jpeg, config)
        observations.append(observed)
        result = model.compare_still(packet, observed)
        errors.append(result['rmse_px'])
    # The recorded photos show the previous robot; lab/model now holds only the current print,
    # so these errors no longer have reference values. Check that the pipeline still runs.
    require(np.all(np.isfinite(errors)) and len(errors) == 4, 'Recorded static comparison failed')
    require(point_cost([[0, 0], [0, 0]], [[3, 4], [0, 0]]) == 12.5, 'Point cost normalization')
    require(targets(220, 20, 1, [220, 220]) == [230, 210], 'Full-difference conversion')
    for values in [(220, float('nan'), 1), (220, 20, 0), (400, 20, 1), (220, 71, 1)]:
        try:
            targets(*values, [220, 220])
        except ValueError:
            pass
        else:
            raise AssertionError('Invalid motor command accepted')
    original_k = model.model.m.jnt_stiffness[0]
    original_d = model.model.m.dof_damping[0]
    model.set_parameters(.5, .1)
    require(model.model.m.jnt_stiffness[0] == original_k and model.model.m.dof_damping[0] == original_d, 'Boundary joint changed')
    require(np.allclose(model.model.m.jnt_stiffness[1:], .5*model.model.k_weights), 'Wrong stiffness scaling')
    packet, jpeg = packet_for(2, config)
    _, observed = measure(jpeg, config)
    r1 = model.compare_still(packet, observed)
    model.set_parameters(.5, .2)
    r2 = model.compare_still(packet, observed)
    require(abs(r1['cost_px2']-r2['cost_px2']) < 1e-7, 'Damping affected static equilibrium')
    scan = model.scan(r2, count=7)
    require(len(scan['cost_px2']) >= 7 and model.p['k2'] == .5, 'Sweep modified selected k')
    return {'recorded_hold_rmse_px': errors, 'scaling_cost_bounds_static_damping': 'PASS'}


def dynamic_checks(config, fit):
    # Physical observations are held fixed here to test gates, not to claim a real motion fit.
    base, jpeg = packet_for(1, config); _, observed = measure(jpeg, config)
    model = Comparator(config, fit)
    def packet(t, moving=False):
        times = np.arange(max(0., t-4), t+.151, .05)
        rows = [dict(axis=a, t_s=float(x), length_mm=220.+(1 if a==0 else -1)*max(0., x-5)*.3,
                     target_mm=221. if a==0 else 219., state='MOVING' if moving else 'HOLDING')
                for a in (0, 1) for x in times]
        return dict(base, session='synthetic_inputs', t_s=t, motor=rows, frame_age_s=0, motor_age_s=[0, 0])
    valid = None
    for t in np.arange(.2, 5.01, .1):
        try:
            valid = model.compare(packet(float(t)), observed)
        except ValueError as exc:
            require(str(exc).startswith('Initialization:'), str(exc))
    require(valid and valid['settled'], 'Never reached settled dynamic comparison')
    initial = np.array(valid['predicted_curve_px'])
    for t in np.arange(5.1, 6.01, .1):
        moving = model.compare(packet(float(t), True), observed)
        require(moving['cost_px2'] is None, 'Moving frame received a stiffness score')
    require(np.max(np.abs(np.array(moving['predicted_curve_px'])-initial)) > .1, 'MuJoCo backbone did not move')
    q = model.model.d.qpos.copy()
    model.set_parameters(.9, .03)
    require(np.array_equal(q, model.model.d.qpos), 'Parameter edit teleported simulated state')
    stale = packet(6.1, True); stale['frame_age_s'] = 2
    try:
        model.compare(stale, observed)
    except ValueError:
        pass
    else:
        raise AssertionError('Stale image was scored')
    return {'moving_backbone': 'PASS', 'moving_stiffness_score_suppressed': 'PASS', 'stale_reset': 'PASS'}


def driver_checks(motor_source, capture_source):
    sys.path[:0] = [str(Path(motor_source).resolve()), str(Path(capture_source).resolve())]
    import actuation
    from spirob_cable.config import RigSettings
    from spirob_experiment.camera import SyntheticCamera
    from spirob_live.capture import LiveCameraService as CameraService
    outcomes = {}
    for scenario in ('normal', 'discard', 'discard_then_record', 'setup_controls', 'independent_controls', 'cold_start', 'heartbeat_lost', 'preview_lost', 'camera_failure', 'eof', 'timeout'):
        with tempfile.TemporaryDirectory() as tmp:
            events = queue.Queue()
            original_emit = actuation.emit
            actuation.emit = lambda **data: events.put(data)
            read_fd, write_fd = os.pipe()
            read = os.fdopen(read_fd, 'r'); write = os.fdopen(write_fd, 'w', buffering=1)
            inbox = Inbox(read)
            service = CameraService(tmp, 'x'*40, lambda: SyntheticCamera(fps=25))
            class Camera:
                identity = None
                fail = False
                starts = 0
                warming = 2 if scenario=='cold_start' else 0
                def clock(self):
                    return {'best': {'offset_s': 0.}, 'network_uncertainty_s': 0.}
                def start(self, session_id, nonce):
                    self.identity = dict(session_id=session_id, nonce=nonce)
                    self.starts += 1
                    reply=service.start(self.identity)
                    if scenario=='cold_start': raise TimeoutError('Injected lost start reply')
                    return reply
                def status(self):
                    if self.warming:
                        self.warming-=1
                        raise TimeoutError('Injected camera warm-up')
                    if self.fail:
                        raise RuntimeError('Injected camera failure')
                    return service.current(self.identity).status()
                def call(self, endpoint, data):
                    if endpoint == '/context':
                        service.current(self.identity).context = data['context']
                        return {'context_received': True}
                    if endpoint == '/record':
                        return service.current(self.identity).record(data['enabled'])
                    raise ValueError(endpoint)
                def stop(self):
                    # Motor disable must have been reported to the session before camera closure.
                    run = json.loads((Path(tmp)/self.identity['session_id']/'run.json').read_text())
                    require(run.get('motor_shutdown', {}).get('disable_confirmed') == [True, True],
                            'Camera closed before motor disable confirmation was saved')
                    return service.current(self.identity).stop()
            camera = Camera(); cfg = RigSettings.demo()
            result = {}
            def owner():
                result.update(actuation.run_session({'data_root': tmp, 'min_fps': 15}, cfg, camera, inbox, demo=True, max_seconds=6 if scenario=='timeout' else 30))
            thread = threading.Thread(target=owner); thread.start()
            sequence = 0
            def send(action, **values):
                nonlocal sequence
                sequence += 1
                write.write(json.dumps(dict(id=sequence, action=action, **values))+'\n'); write.flush()
            def until(predicate, seconds=10, heartbeat=True, preview=True):
                deadline = time.monotonic()+seconds
                while time.monotonic()<deadline:
                    if heartbeat:
                        send('ping', preview_ok=preview)
                    try:
                        event = events.get(timeout=.1)
                        if predicate(event): return event
                        if event['type']=='closed': raise AssertionError(str(event))
                    except queue.Empty:
                        pass
                raise AssertionError('Timed out in scenario '+scenario)
            until(lambda e: e.get('phase')=='READY_DISABLED')
            send('move', mean_mm=220, difference_mm=20, speed_mm_s=1)
            until(lambda e: e['type']=='rejected')
            if scenario not in ('heartbeat_lost', 'setup_controls', 'timeout'):
                send('arm', at_rest=True)
                until(lambda e: e.get('phase')=='HOLDING')
            if scenario in ('discard','discard_then_record'):
                require(not list((service.capture.path/'frames').glob('*.jpg')), 'Discard run saved images')
                if scenario=='discard_then_record':
                    send('record',enabled=True)
                    until(lambda e:e.get('recording',{}).get('bytes',0)>0)
                send('disconnect')
            elif scenario=='normal':
                require(not list((service.capture.path/'frames').glob('*.jpg')), 'Preview unexpectedly saved JPEGs')
                send('record', enabled=True)
                until(lambda e:e['type']=='accepted')
                until(lambda e:e.get('recording',{}).get('bytes',0)>0)
                send('record', enabled=False)
                until(lambda e:e['type']=='accepted')
                send('move', mean_mm=220, difference_mm=2, speed_mm_s=5)
                until(lambda e: e['type']=='accepted')
                held = until(lambda e: e.get('phase')=='HOLDING' and all(abs(a['absolute_mm']-target)<.05 for a,target in zip(e['axes'], [221,219])))
                send('move', mean_mm=220, difference_mm=20, speed_mm_s=1)
                until(lambda e: e['type']=='accepted')
                send('hold')
                until(lambda e: e.get('phase')=='HOLDING')
                send('disconnect')
            elif scenario=='independent_controls':
                send('independent',lengths_mm=[221,291],speeds_mm_s=[2,.7]);until(lambda e:e['type']=='rejected')
                send('independent',lengths_mm=[221,219.5],speeds_mm_s=[2,.7])
                until(lambda e:e['type']=='accepted')
                held=until(lambda e:e.get('phase')=='HOLDING' and all(abs(a['absolute_mm']-v)<.03 for a,v in zip(e['axes'],[221,219.5])))
                require(held['axes'][0]['motor_rpm_command']!=held['axes'][1]['motor_rpm_command'],'Independent speeds were collapsed to one speed')
                send('independent',lengths_mm=[225,217],speeds_mm_s=[2,1])
                until(lambda e:e['type']=='accepted')
                send('hold');until(lambda e:e.get('phase')=='HOLDING')
                send('disconnect')
            elif scenario=='cold_start':
                require(camera.starts==1,'Ambiguous camera start was retried')
                send('disconnect')
            elif scenario=='setup_controls':
                send('setup_enable')
                setup=until(lambda e:e.get('phase')=='SETUP_HOLD')
                require(all(a['rest_count'] is None for a in setup['axes']),'Setup enable invented a rest reference')
                send('axis',axis=0,length_mm=220,speed_mm_s=1)
                until(lambda e:e['type']=='rejected')
                before=setup['axes'][1]['count']
                send('jog',axis=0,delta_mm=.5,speed_mm_s=5)
                until(lambda e:e['type']=='accepted')
                jogged=until(lambda e:e.get('phase')=='SETUP_HOLD' and e['axes'][0]['connection_mm']>.45)
                require(jogged['axes'][1]['count']==before,'Single-axis jog moved other motor')
                send('reference',at_rest=False);until(lambda e:e['type']=='rejected')
                send('reference',at_rest=True)
                rested=until(lambda e:e.get('phase')=='HOLDING')
                require(all(abs(a['absolute_mm']-220)<.01 for a in rested['axes']),'Rest declaration failed')
                send('axis',axis=0,length_mm=221,speed_mm_s=5)
                until(lambda e:e['type']=='accepted')
                single=until(lambda e:e.get('phase')=='HOLDING' and abs(e['axes'][0]['absolute_mm']-221)<.03)
                require(abs(single['axes'][1]['absolute_mm']-220)<.03,'Absolute single-axis command moved other motor')
                difference=single['axes'][0]['absolute_mm']-single['axes'][1]['absolute_mm']
                send('mean',mean_mm=221.5,speed_mm_s=5)
                until(lambda e:e['type']=='accepted')
                paired=until(lambda e:e.get('phase')=='HOLDING' and abs(sum(a['absolute_mm'] for a in e['axes'])/2-221.5)<.03)
                require(abs(paired['axes'][0]['absolute_mm']-paired['axes'][1]['absolute_mm']-difference)<.03,'Mean-only changed measured difference')
                send('jog',axis=0,delta_mm=1001,speed_mm_s=1);until(lambda e:e['type']=='rejected')
                send('disconnect')
            elif scenario=='camera_failure':
                camera.fail=True
            elif scenario=='eof':
                write.close()
            if scenario=='heartbeat_lost':
                final=until(lambda e:e['type']=='closed', heartbeat=False)
            elif scenario=='preview_lost':
                # A formerly valid preview must expire while heartbeats continue.
                final=until(lambda e:e['type']=='closed', preview=False)
            elif scenario=='eof':
                final=until(lambda e:e['type']=='closed', heartbeat=False)
            else:
                final=until(lambda e:e['type']=='closed')
            thread.join(3)
            require(not thread.is_alive(), 'Coordinator did not exit')
            require(final['motor_shutdown']['worker_exited'] and final['motor_shutdown']['disable_confirmed']==[True, True], 'Motor shutdown unconfirmed')
            require(final['camera_shutdown']['thread_exited'], 'Camera writer did not exit')
            require(final['state']==('COMPLETE' if scenario in ('normal','discard','discard_then_record','setup_controls','independent_controls','cold_start') else 'FAILED'), 'Unexpected session outcome')
            require(final['retention']=='pending' and Path(final['path']).is_dir(), 'Run removed before post-run decision')
            from run_storage import decide_remote, pending_remote
            require(any(r['session']==final['session'] for r in pending_remote(tmp)), 'Run missing from recovery list')
            decision='discard' if scenario=='discard' else 'keep'
            outcome=decide_remote(tmp,final['session'],decision)
            require(Path(final['path']).exists()==(decision=='keep'), 'Post-run decision failed')
            if scenario == 'heartbeat_lost':
                require('heartbeat lost' in final['error'], 'Heartbeat lease was not tested independently')
            outcomes[scenario] = {'state': final['state'], 'error': final['error'], 'motor_shutdown': final['motor_shutdown'], 'camera_writer_exited': True}
            if not write.closed: write.close()
            read.close()
            actuation.emit = original_emit
    return outcomes


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--motor-source'); parser.add_argument('--capture-source')
    args=parser.parse_args()
    config=json.loads(Path('config.json').read_text());config['model']='model/spirob_robot.xml';config.pop('comparison_setup',None)
    fit=json.loads(Path('calibration/pilot_fit.json').read_text())
    result={'science':science_checks(config,fit), 'dynamics':dynamic_checks(config,fit)}
    if args.motor_source and args.capture_source:
        result['demo_driver']=driver_checks(args.motor_source,args.capture_source)
    else:
        result['demo_driver']='Not run: source paths were not supplied'
    Path('verification.json').write_text(json.dumps(result,indent=2)+'\n')
    print(json.dumps(result,indent=2))


if __name__=='__main__': main()

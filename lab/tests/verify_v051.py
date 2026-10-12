"""Regression checks for live Apply, missing vision and end-of-run retention.

Only recorded images, synthetic inputs, temporary folders and fake transports
are used. No real hardware or SSH is opened.
"""
import base64
import copy
import json
import os
from pathlib import Path
import queue
import sys
import tempfile
import threading
import time
from unittest.mock import patch
import numpy as np
os.environ.setdefault('QT_QPA_PLATFORM','offscreen')
ROOT=Path(__file__).resolve().parents[1];sys.path.insert(0,str(ROOT));os.chdir(ROOT)
from analysis import Comparator
from interface import Window, QApplication, recorded_packet
from run_storage import (write_json, decide_remote, pending_remote, finish_review,
                         review_entries, endpoint, shutdown_confirmed)
from spirob_analysis.measurement import measure

config=json.loads(Path('config.json').read_text());fit=json.loads(Path('calibration/pilot_fit.json').read_text())
recorded=recorded_packet(config,1)
_,observed=measure(base64.b64decode(recorded['image_b64']),config)

def packet(t):
    times=np.arange(t-3,t+.11,.05)
    rows=[dict(axis=a,t_s=float(s),length_mm=220.,target_mm=220.,state='HOLDING')
          for a in (0,1) for s in times]
    return dict(recorded,session='LIVE_TEST',t_s=t,motor=rows,frame_age_s=0.,motor_age_s=[0.,0.])

engine=Comparator(config,fit)
slow=packet(4.)
slow['motor']=[dict(axis=a,t_s=float(t),length_mm=220.,target_mm=220.,state='HOLDING')
               for a in (0,1) for t in np.arange(1.13,4.41,.3)]
assert engine.encoders_settled(slow), 'Irregular serial samples prevented held-input initialization'
for t in np.arange(3,5.31,.1):
    result=engine.compare(packet(float(t)),observed)
assert result['settled']
q=engine.model.d.qpos.copy();qd=engine.model.d.qvel.copy();when=engine.dynamic_t
old_curve=np.asarray(result['predicted_curve_px'])
setup=engine.setup();setup['translation_x_px']+=60;setup['translation_y_px']+=35
engine.set_setup(setup)
assert engine.dynamic_t==when and np.array_equal(engine.model.d.qpos,q) and np.array_equal(engine.model.d.qvel,qd)
assert np.allclose(engine.transform(engine.model.curve())-old_curve,[60,35])
updated=engine.compare(packet(float(when+.1)),None)
assert updated['predicted_curve_px'] and updated['cost_px2'] is None and not updated['settled']
# Restarting with the saved alignment must also render with no camera extraction.
restarted=Comparator(dict(config,comparison_setup=setup),fit)
fresh=restarted.compare(packet(4.),None)
assert fresh['predicted_curve_px'] and fresh['cost_px2'] is None
assert np.allclose(fresh['base_px'],updated['base_px'])
# A bad correspondence / scale may invalidate scoring but must not erase physics.
setup['pixels_per_m']=100;restarted.set_setup(setup)
small=restarted.compare(packet(4.1),observed)
assert small['predicted_curve_px'] and small['cost_px2'] is None and 'J paused' in small['message']
json.dumps(small,allow_nan=False)
# Wiring is different: require held inputs again, and never score stale feedback.
setup['mapping']=[1,0];restarted.set_setup(setup);assert restarted.dynamic_t is None
stale=packet(4.2);stale['motor_age_s']=[2.,2.]
try:restarted.compare(stale,observed)
except ValueError:pass
else:raise AssertionError('Stale feedback was displayed as a current prediction')
print('PASS: live Apply preserves physics; absent/invalid vision pauses J only; restart, mapping and stale gates')

with tempfile.TemporaryDirectory() as tmp:
    root=Path(tmp);rig={'ssh_host':'fake-host','wsl':{'data_root':str(root/'remote')}}
    remote=Path(rig['wsl']['data_root']);remote.mkdir()
    logs=root/'logs';local=logs/'runs'/'session_test';local.mkdir(parents=True)
    sid='20261003T120000_000000Z_LIVE_aabbcc'
    path=remote/sid;path.mkdir()
    run=dict(session_id=sid,state='COMPLETE',retention={'decision':'pending'},motor_opened=True,camera_requested=True,
        motor_shutdown=dict(worker_exited=True,disable_confirmed=[True,True]),
        camera_shutdown=dict(thread_exited=True,recording={'frames':5,'bytes':10000}))
    write_json(path/'run.json',run);(path/'image.jpg').write_bytes(b'synthetic')
    write_json(local/'run_review.json',dict(decision='pending',session=sid,endpoint=endpoint(rig),remote_started=True,final=run))
    (local/'analysis.jsonl').write_text('evidence')
    def fake_request(rig,identifier=None,decision=None):
        return decide_remote(remote,identifier,decision) if identifier else pending_remote(remote)
    with patch('run_storage.remote_request',fake_request):
        entries=review_entries(logs,rig);assert len(entries)==1 and entries[0]['shutdown_confirmed']
        # Read-only recovery and Later leave every byte available.
        assert (path/'image.jpg').exists() and (local/'analysis.jsonl').exists()
        finish_review(logs,rig,entries[0],'keep')
        assert path.exists() and local.exists() and not review_entries(logs,rig)
        try:decide_remote(remote,sid,'discard')
        except ValueError:pass
        else:raise AssertionError('An earlier kept run was deleted')
        # Separate pending run: lost remote reply leaves local logs pending;
        # retry completes the same explicitly requested action idempotently.
        sid2='20261003T120001_000000Z_LIVE_ddeeff';p2=remote/sid2;p2.mkdir()
        run2=dict(run,session_id=sid2);write_json(p2/'run.json',run2)
        l2=logs/'runs'/'session_next';l2.mkdir()
        write_json(l2/'run_review.json',dict(decision='pending',session=sid2,endpoint=endpoint(rig),remote_started=True,final=run2))
        entry=review_entries(logs,rig)[0]
        def lost_reply(rig,identifier,decision):
            decide_remote(remote,identifier,decision)
            raise TimeoutError('injected lost reply')
        with patch('run_storage.remote_request',lost_reply):
            try:finish_review(logs,rig,entry,'discard')
            except TimeoutError:pass
            else:raise AssertionError('Lost reply not exercised')
        assert l2.exists() and not p2.exists()
        finish_review(logs,rig,entry,'discard');assert not l2.exists()
        assert path.exists(), 'Earlier saved run was touched'
    bad_id='20261003T120002_000000Z_LIVE_112233';bad=remote/bad_id;bad.mkdir()
    write_json(bad/'run.json',dict(run,session_id=bad_id,motor_shutdown={'disable_confirmed':[True,False]}))
    try:decide_remote(remote,bad_id,'discard')
    except ValueError:pass
    else:raise AssertionError('Unconfirmed shutdown allowed discard')
    link=remote/'20261003T120003_000000Z_LIVE_445566';link.symlink_to(path,target_is_directory=True)
    try:decide_remote(remote,link.name,'discard')
    except ValueError:pass
    else:raise AssertionError('Symlink target could be deleted')
    # Failed-start attempt with no remote owner can be discarded locally.
    local_only=logs/'runs'/'startup';local_only.mkdir()
    write_json(local_only/'run_review.json',dict(endpoint=endpoint(rig),session=None,remote_started=False,decision='pending'))
    finish_review(logs,rig,dict(session=None,local_paths=[str(local_only)]),'discard')
    assert not local_only.exists()
print('PASS: crash recovery, Save/Discard, no action before choice, lost replies, unknown shutdown and path guards')

# Exercise the deployed helper's owner lock, plus persistent workstation logs.
with tempfile.TemporaryDirectory() as tmp:
    import fcntl
    import subprocess
    from connection import WorkstationSession
    root=Path(tmp);(root/'motor').mkdir();(root/'token').write_text('x'*40)
    rig=dict(ssh_host='fake-host',workstation_token_file=str(root/'token'),
             wsl=dict(data_root=str(root/'remote'),motor_repo=str(root/'motor')))
    (root/'remote').mkdir();path=root/'remote'/'TEST_LIVE_lock';path.mkdir()
    write_json(path/'run.json',dict(run,session_id=path.name))
    write_json(root/'rig.json',rig)
    command=[sys.executable,str(ROOT/'run_storage.py'),'--rig',str(root/'rig.json'),
             '--session',path.name,'--decision','discard']
    with (root/'motor'/'.spirob-live.lock').open('a') as lock:
        fcntl.flock(lock,fcntl.LOCK_EX|fcntl.LOCK_NB)
        reply=subprocess.run(command,capture_output=True)
        assert reply.returncode!=0 and path.exists()
    reply=subprocess.run(command,capture_output=True)
    assert reply.returncode==0 and not path.exists()
    link=WorkstationSession(rig,root/'logs')
    folder=link.log_dir
    link.stderr.write('connection-attempt evidence\n')
    link.disconnect();link.finish_storage()
    assert (folder/'connection.log').is_file()
    assert json.loads((folder/'run_review.json').read_text())['decision']=='pending'
    # A partially completed remote discard is discoverable after restart.
    partial=root/'remote'/'TEST_LIVE_partial';partial.mkdir();(partial/'leftover').write_text('bytes')
    receipt=root/'remote'/'.spirob-run-decisions'/(partial.name+'.json')
    write_json(receipt,dict(session=partial.name,decision='discard_requested',path=str(partial)))
    assert any(r['session']==partial.name for r in pending_remote(root/'remote'))
    decide_remote(root/'remote',partial.name,'discard');assert not partial.exists()
    # Diagnostic collection still includes alignment when the laptop is offline.
    from diagnostics import collect
    from zipfile import ZipFile
    rig['wsl'].update(user='test',runtime='/runtime')
    write_json(root/'lab.local.json',{'comparison_setup':setup})
    with patch('diagnostics.subprocess.run',side_effect=OSError('offline')),patch('diagnostics.Path.home',return_value=root):
        collect(root,rig)
    archive=next((root/'Downloads').glob('*.zip'))
    with ZipFile(archive) as z:
        assert 'workstation/lab.local.json' in z.namelist() and 'collection_status.json' in z.namelist()
        assert not any('token' in name for name in z.namelist())
print('PASS: active-owner lock, persistent workstation journal, interrupted discard recovery and offline diagnostics')

app=QApplication.instance() or QApplication([])
def until(predicate,seconds=20):
    deadline=time.monotonic()+seconds
    while time.monotonic()<deadline:
        app.processEvents()
        if predicate():return
        time.sleep(.02)
    raise AssertionError('GUI condition timed out')

out=Path(os.environ.get('SPIROB_TEST_OUTPUT','/tmp/spirob-v051-verification'));out.mkdir(parents=True,exist_ok=True)
window=Window(config,fit,None,True);window.show()
try:
    until(lambda:window.latest and window.latest.get('result'))
    window.open_setup();dialog=window.setup_dialog
    dialog.picked(890,55);draft=dialog.values();dialog.apply()
    until(lambda:window.latest and window.latest.get('result') and
          np.allclose(window.latest['result']['base_px'],[890,55],atol=.02))
    assert window.camera.predicted
    saved=dict(config,comparison_setup=draft);check=Comparator(saved,fit)
    assert np.allclose(check.transform(check.model.base_point()),[890,55],atol=.02)
    dialog.mapping.setCurrentIndex(1);dialog.load_supplied_alignment()
    assert dialog.mapping.currentIndex()==1
    bad=copy.deepcopy(draft);bad['translation_x_px']=9000;bad['translation_y_px']=9000
    try:window.apply_setup(bad)
    except ValueError:pass
    else:raise AssertionError('Wholly off-image alignment accepted')
    dialog.close();window.refresh_summary(window.inspector_snapshot());window.tick()
    window.grab().save(str(out/'alignment.png'))
finally:window.close();app.processEvents()

class FakeSession:
    def __init__(self,rig,logs):
        self.events=queue.Queue();self.cancel=threading.Event();self.commands=[];self.stopped=False;self.writers_closed=False
    def connect(self):pass
    def touch(self):pass
    def send(self,action,**values):self.commands.append((action,values))
    def disconnect(self):
        self.stopped=True
        return dict(session='TEST_LIVE_abc',path='SIMULATED',state='COMPLETE',error=None,
          motor_shutdown=dict(worker_exited=True,disable_confirmed=[True,True]),camera_shutdown=dict(thread_exited=True))
    def finish_storage(self):self.writers_closed=True
    def review_entry(self):return dict(session='TEST_LIVE_abc',shutdown_confirmed=True,local_paths=[])

for action,choice in [('disconnect','keep'),('close','discard'),('timeout','later')]:
    live=Window(config,fit,{},False);live.show();live.start_preview=lambda:None
    asked=[];decisions=[]
    def ask(entry):
        assert session.stopped and session.writers_closed and live.session is None
        asked.append(entry);return choice
    live.ask_run_decision=ask
    with (patch('interface.WorkstationSession',FakeSession),
          patch('interface.QMessageBox.question',side_effect=AssertionError('Unexpected pre-run prompt')),
          patch('interface.finish_review',lambda logs,rig,entry,decision:decisions.append(decision) or {'decision':decision})):
        live.connect();until(lambda:not live.connecting and live.session is not None)
        session=live.session
        assert not asked and not session.commands
        live.toggle_recording();assert session.commands==[('record',{'enabled':True})] and not asked
        if action=='disconnect':live.disconnect()
        elif action=='close':live.close()
        else:session.events.put({'type':'transport_closed','confirmed':True})
        until(lambda:bool(asked) and not live.review_busy)
        assert len(asked)==1 and decisions==([] if choice=='later' else [choice])
    live.close();app.processEvents()

live=Window(config,fit,{},False);live.show();seen=[]
live.ask_run_decision=lambda entry:seen.append(entry) or 'later'
with patch('interface.review_entries',return_value=[{'session':'INTERRUPTED_LIVE_a','shutdown_confirmed':False}]):
    live.recover_runs();until(lambda:bool(seen) and not live.review_busy)
assert len(seen)==1
live.close();app.processEvents()
print('PASS: actual Apply button, saved base, off-image rejection, no Connect prompt; disconnect/close/timeout prompt after shutdown; restart recovery')

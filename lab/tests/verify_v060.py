"""Rigid calibration, preview-only lifecycle, expanded limits and frozen exports.

No real camera, serial device or SSH. Uses synthetic camera and recorded images.
"""
import argparse
import base64
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
from unittest.mock import patch
import numpy as np
os.environ.setdefault('QT_QPA_PLATFORM','offscreen')
ROOT=Path(__file__).resolve().parents[1];sys.path.insert(0,str(ROOT));os.chdir(ROOT)
parser=argparse.ArgumentParser();parser.add_argument('--motor-source',required=True);parser.add_argument('--capture-source',required=True)
args=parser.parse_args()
sys.path[:0]=[str(Path(args.motor_source).resolve()),str(Path(args.capture_source).resolve())]
from landmarks import placement_profile,correspondences,rigid_points,scale_from_profile
from analysis import Comparator,point_cost
from interface import QApplication,Window,Preview,recorded_packet
from connection import camera_get,camera_post,CameraSetupSession,WorkstationSession
from spirob_analysis.measurement import measure
from actuation import targets
from motion_limits import worker_limits
from spirob_cable.config import RigSettings
from spirob_cable import worker
from spirob_cable.drive import DemoDrive,DriveFault
from spirob_live.capture import LiveCameraService
from spirob_live.server import add_observer
from spirob_experiment.camera import SyntheticCamera,serve

config=json.loads(Path('config.json').read_text());fit=json.loads(Path('calibration/pilot_fit.json').read_text())
for name in ('model','session'):config[name]=str(ROOT/config[name])
out=Path('/tmp/spirob-v060-verification');out.mkdir(exist_ok=True)
# A bent polyline with genuinely rigid links. Unlike fixed arc distances, chord
# constraints survive a change of joint angle, translation and image rotation.
names=['j_018','j_019','j_020','j_021','tip'];lengths=np.array([30,25,20,15.])
def chain(angles):
    return np.vstack([[100.,100.],[100.,100.]+np.cumsum(lengths[:,None]*np.c_[np.cos(angles),np.sin(angles)],axis=0)])
points=chain([.2,.5,.7,1.]);rows=[dict(id=n,included=True,physical_px=p.tolist()) for n,p in zip(names,points)]
profile=placement_profile({'curve_px':points.tolist()},rows,'joints',1,(1080,1920,3),True,rigid=True)
for angles in ([.2,.5,.7,1.],[-.8,-.4,.2,.7],[1.5,1.1,.9,.5]):
    bent=chain(angles)+[20,10]
    found=rigid_points(bent,profile)
    assert np.allclose(list(found.values()),bent,atol=1e-7)
    assert np.allclose(np.linalg.norm(np.diff(list(found.values()),axis=0),axis=1),lengths)
short=rigid_points(points[2:],profile);assert short[names[0]] is None
layout=dict(ids=names,tail_m=(np.r_[lengths[::-1].cumsum()[::-1],0]/4000).tolist())
assert abs(scale_from_profile(profile,layout)-4000)<1e-7
bad=copy.deepcopy(rows);bad[1]['physical_px']=bad[2]['physical_px']
try:placement_profile({'curve_px':points.tolist()},bad,'joints',1,(1080,1920),True,rigid=True)
except ValueError:pass
else:raise AssertionError('Unordered manual joints accepted')
print('PASS rigid distances across deformation; missing coverage; physical-only scale')

packet=recorded_packet(config,2);rgb,observed=measure(base64.b64decode(packet['image_b64']),config)
engine=Comparator(config,fit);result=engine.compare_still(packet,observed)
rows=correspondences(observed,result['landmarks'])
profile=placement_profile(observed,rows,'joints',packet['frame_index'],rgb.shape,True,rigid=True)
setup=engine.setup();setup['profile']=profile;engine.set_setup(setup)
joint=engine.compare_still(packet,observed)
assert joint['settled'] and joint['observation']['landmark_ids']
assert abs(joint['cost_px2']-point_cost(joint['observation']['points_px'],joint['predicted_points_px']))<1e-8
preview=engine.compare_setup(dict(packet,motor=[],motor_active=False),observed)
assert preview['predicted_curve_px'] and preview['landmark_rows']
assert preview['cost_px2'] is None and preview['observation'] is None and not preview['settled']
assert preview['lengths_mm']==[None,None]
print('PASS calibrated J uses matching model anchors; setup reference never invents feedback or J')

# Simulated driver's actual Rig validates expanded endpoints and measured travel.
assert targets(220,220,1,[220,220],220)==[330,110]
for values in [(220,221,220),(330,20,220),(220,120,70)]:
    try:targets(values[0],values[1],1,[220,220],values[2])
    except ValueError:pass
    else:raise AssertionError('Motor envelope not enforced')
class Recorder:
    def event(self,*a,**k):pass
    def sample(self,*a,**k):pass
    def pair(self,*a,**k):pass
cfg=RigSettings.demo();original=worker.Rig
with worker_limits([220,220],220):
    r=worker.Rig(cfg,[DemoDrive(c,Recorder()) for c in cfg.selected],Recorder())
assert worker.Rig is original
now=time.monotonic()
for a in r.axes:
    a.rest=a.connection=a.count=0;a.rest_length=220;a.enabled=True;a.state='HOLDING'
    a.last_read=now;a.stable.extend([0,0,0])
moves=r.plan_modes(220,110,1)
assert np.allclose([a.absolute(m.count) for a,m in zip(r.axes,moves)],[330,110],atol=.001)
assert all(a.envelope()[1]==110 for a in r.axes)
try:r.plan_modes(220,110.01,1)
except ValueError:pass
else:raise AssertionError('Driver accepted excessive full difference')
# Measured envelope uses the new bound, including after stop; no watchdog is disabled.
a=r.axes[0];a.count=round(110.2/a.cfg.mm_per_count);a.last_read=time.monotonic()-.1
a.drive.position=lambda:a.count
try:a.poll()
except DriveFault as exc:assert 'travel exceeded' in str(exc)
else:raise AssertionError('Measured travel bypassed')
print('PASS full-rest difference endpoints; driver conversion and feedback envelope; scoped adapter')

with tempfile.TemporaryDirectory() as directory:
    service=LiveCameraService(directory,'x'*40,lambda:SyntheticCamera(fps=30))
    server=add_observer(serve(service,'127.0.0.1',0),service)
    thread=threading.Thread(target=server.serve_forever);thread.start()
    url=f'http://127.0.0.1:{server.server_port}';identity=dict(session_id='SETUP_TEST',nonce='a'*32)
    try:
        camera_post(url,'x'*40,'/preview/start',identity)
        deadline=time.monotonic()+4
        while camera_post(url,'x'*40,'/preview/status',identity)['frame_count']<5 and time.monotonic()<deadline:time.sleep(.03)
        capture=service.capture;capture.started-=4000
        status=camera_post(url,'x'*40,'/preview/status',identity)
        q=camera_get(url,'x'*40,'/packet');assert q['image_b64'] and not q['motor_active'] and not q['motor']
        assert capture.max_s==float('inf') and list(Path(directory).iterdir())==[]
        lease=capture.lease;camera_get(url,'x'*40,'/packet');assert capture.lease==lease
        try:camera_post(url,'x'*40,'/record',dict(identity,enabled=True))
        except Exception:pass
        else:raise AssertionError('Setup preview recording accepted')
        assert camera_post(url,'x'*40,'/preview/stop',identity)['thread_exited']
        assert not capture.path.exists()
        # Connected capture can pass the former 620-second cap; owner controls selected duration.
        service.start(dict(session_id='LONG',nonce='b'*32));capture=service.capture
        capture.started-=1800
        time.sleep(.15);assert capture.status()['state'] in ('STARTING','STREAMING') and capture.max_s==3720
        capture.lease-=20
        deadline=time.monotonic()+3
        while capture.thread.is_alive() and time.monotonic()<deadline:time.sleep(.02)
        assert capture.status(renew=False)['state']=='FAILED' and not capture.thread.is_alive()
        print('PASS authenticated setup camera; no run data; no session timer; long camera cap; lease expiry')
    finally:
        if service.capture:service.capture.stop()
        if getattr(service,'preview_directory',None):service.preview_directory.cleanup()
        server.shutdown();thread.join();server.server_close()

app=QApplication.instance() or QApplication([])
from PySide6.QtWidgets import QMessageBox
def warning(*args):
    raise AssertionError('Unexpected UI warning: '+str(args[2]))
QMessageBox.warning=warning
def until(predicate, seconds=30):
    deadline=time.monotonic()+seconds
    while time.monotonic()<deadline:
        app.processEvents()
        if predicate():return
        time.sleep(.015)
    raise AssertionError('UI condition timed out')
window=Window(config,fit,None,True);window.show()
try:
    until(lambda:window.latest and window.latest.get('result'))
    assert [window.duration.itemData(i) for i in range(window.duration.count())]==[600,1200,1800,2700,3600]
    window.difference_limit.setValue(220);assert window.difference.maximum()==220
    window.inspect_points();inspector=window.inspector
    assert inspector.rigid.isChecked() and inspector.score_profile.isChecked()
    # Actual visible buttons transfer manual placement and freeze that exact pose.
    first=next(r for r in inspector.rows if r['physical_px'] is not None)
    path=np.asarray(inspector.snapshot['observed']['curve_px']);p=np.array(first['physical_px'])
    nearest=int(np.argmin(np.linalg.norm(path-p,axis=1)))
    inspector.manual[first['id']]=path[min(nearest+2,len(path)-1)].tolist();inspector.rebuild()
    window.scan_min.setValue(.4);window.scan_max.setValue(1.3);window.scan_count.setValue(7)
    inspector.sweep_profile_button.click();until(lambda:window.sweep_result is not None)
    assert window.comparison_setup['profile']['kind']=='rigid_links'
    saved_frame=window.sweep_snapshot['packet']['frame_index']
    # Change the displayed pose; export must still use the captured pose.
    window.steps.setCurrentIndex(2);until(lambda:window.latest and window.latest.get('result') and window.latest['packet']['frame_index']!=saved_frame)
    destination=window.export_pose(out/'pose.png')
    data=json.loads(destination.with_suffix('.json').read_text())
    assert data['frame_index']==saved_frame and data['sweep']['observation']==data['result']['observation']
    assert all(destination.with_name(destination.stem+s).exists() for s in ('_frame.png','_joints.csv','_sweep.csv'))
    # Match scale from calibrated physical links with a fixed base.
    window.open_setup();dialog=window.setup_dialog;before=dialog.values()
    dialog.match_length();after=dialog.values()
    check=Comparator(config,fit);check.set_setup(after)
    assert np.allclose(check.joint_preview()['base_px'],[dialog.x.value(),dialog.y.value()],atol=.01)
    with tempfile.TemporaryDirectory() as tmp:
        os.chdir(tmp);inspector.save_profile_button.click()
        preferences=json.loads(Path('lab.local.json').read_text())
        reloaded=Comparator(dict(config,comparison_setup=preferences['comparison_setup']),fit)
        assert reloaded.profile['kind']=='rigid_links'
    os.chdir(ROOT)
    dialog.close();inspector.grab().save(str(out/'inspector.png'));inspector.close()
    window.grab().save(str(out/'window.png'))
    print('PASS manual joints -> live J -> same-frame sweep/export; restart calibration; fixed-base scale; UI duration choices')
finally:window.close();app.processEvents();os.chdir(ROOT)

class FakeSetup:
    def __init__(self,rig):self.events=queue.Queue();self.cancel=threading.Event();self.stopped=False
    def connect(self):pass
    def touch(self):pass
    def disconnect(self):self.stopped=True
class FakeRun:
    def __init__(self,rig,logs):self.events=queue.Queue();self.cancel=threading.Event()
    def connect(self):assert setup_link.stopped
    def touch(self):pass
    def disconnect(self):return None
live=Window(config,fit,{},False);live.start_preview=lambda:None
with patch('interface.CameraSetupSession',FakeSetup),patch('interface.WorkstationSession',FakeRun):
    live.toggle_setup_preview();until(lambda:live.setup_session and not live.setup_connecting)
    setup_link=live.setup_session
    assert live.session is None
    live.duration.setCurrentIndex(4);live.difference_limit.setValue(220)
    live.connect();until(lambda:live.session and not live.connecting)
    assert setup_link.stopped and live.setup_session is None
    assert live.session.max_seconds==3600 and live.session.difference_limit_mm==220
    live.disconnect();until(lambda:live.session is None and not live.closing)
live.close();app.processEvents()
print('PASS setup-to-Connect closes camera first, launches motor session only on Connect, propagates 1-hour/220-mm settings')
print('All v0.6.0 checks passed. No hardware was operated.')

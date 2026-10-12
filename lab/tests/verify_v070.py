"""Independent cable commands and vision-independent manual pose analysis.

Only recorded images, offscreen widgets and simulated motor drives are used.
"""
import argparse
import base64
import copy
import json
import os
from pathlib import Path
import sys
import tempfile
import time
from unittest.mock import patch
import numpy as np
os.environ.setdefault('QT_QPA_PLATFORM','offscreen')
ROOT=Path(__file__).resolve().parents[1];sys.path.insert(0,str(ROOT));os.chdir(ROOT)
parser=argparse.ArgumentParser();parser.add_argument('--motor-source',required=True);args=parser.parse_args()
sys.path.insert(0,str(Path(args.motor_source).resolve()))
from actuation import independent_targets,dispatch,targets
from motion_limits import worker_limits
from spirob_cable import worker
from spirob_cable.config import RigSettings
from spirob_cable.drive import DemoDrive
from analysis import Comparator,point_cost
from interface import QApplication,Window,Preview,recorded_packet
from landmarks import correspondences
from manual_analysis import prepare_capture,complete_capture
from spirob_analysis.measurement import measure
from PySide6.QtWidgets import QMessageBox

config=json.loads(Path('config.json').read_text());fit=json.loads(Path('calibration/pilot_fit.json').read_text())
for key in ('model','session'):config[key]=str(ROOT/config[key])
out=Path('/tmp/spirob-v070-verification');out.mkdir(exist_ok=True)
assert independent_targets([150,290],[.05,5],[220,220])==[150,290]
assert independent_targets([110,330],[1,2],[220,220],220)==[110,330]
try:targets(220,140,1,[220,220])
except ValueError:pass
else:raise AssertionError('Mean/difference limit bypassed')
class MockMotor:
    referenced=True;pending=False;connected=False
    snapshot={'axes':[dict(enabled=True,state='HOLDING',absolute_mm=220.) for _ in range(2)]}
    def __init__(self):self.commands=[]
    def submit(self,*args):self.commands.append(args)
motor=MockMotor()
def command(lengths,speeds):return dict(action='independent',lengths_mm=lengths,speeds_mm_s=speeds,received_s=time.monotonic())
for lengths,speeds in [([150,291],[1,1]),([150,290],[1,5.1]),([True,220],[1,1]),([220,float('nan')],[1,1]),([220,220],[1]),([220],[1,1])]:
    try:dispatch(command(lengths,speeds),motor,[220,220],True)
    except ValueError:pass
    else:raise AssertionError('Bad independent target accepted')
assert not motor.commands
dispatch(command([150,290],[.5,2]),motor,[220,220],True)
assert motor.commands==[('independent',dict(lengths_mm=[150,290],speeds_mm_s=[.5,2]))]
for changes,fresh in [({'referenced':False},True),({'pending':True},True),({},False)]:
    m=MockMotor()
    for k,v in changes.items():setattr(m,k,v)
    try:dispatch(command([220,220],[1,1]),m,[220,220],fresh)
    except ValueError:pass
    else:raise AssertionError('Reference/pending/camera gate bypassed')
    assert not m.commands
class Recorder:
    def __init__(self):self.events=[]
    def event(self,*args,**kwargs):self.events.append((args,kwargs))
    def sample(self,*args,**kwargs):pass
    def pair(self,*args,**kwargs):pass
cfg=RigSettings.demo();original_rig,original_worker=worker.Rig,worker.Worker
recorder=Recorder()
with worker_limits([220,220],70):
    r=worker.Rig(cfg,[DemoDrive(c,recorder) for c in cfg.selected],recorder)
    assert worker.Worker is not original_worker
assert (worker.Rig,worker.Worker)==(original_rig,original_worker)
for a in r.axes:
    position=a.drive.position()
    a.count=a.connection=a.rest=a.target=position;a.rest_length=220.;a.enabled=True;a.state='HOLDING'
    a.last_read=time.monotonic();a.stable.extend([position]*3);a.position_mode=True
    a.drive.enable()
try:r.move_independent([150,291],[.5,2])
except ValueError:pass
else:raise AssertionError('Driver allowed invalid endpoint')
assert not recorder.events
r.move_independent([150,290],[.5,2])
assert np.allclose([a.absolute(a.target) for a in r.axes],[150,290],atol=.001)
assert r.axes[0].rpm!=r.axes[1].rpm
assert [e[1]['requested_mm_s'] for e in recorder.events if e[0][0]=='move_requested']==[.5,2]
print('PASS independent endpoints/speeds; complete validation before writes; scoped worker; original mean/difference limits')

packet=recorded_packet(config,2);rgb,observed=measure(base64.b64decode(packet['image_b64']),config)
engine=Comparator(config,fit);baseline=engine.compare_still(packet,observed)
captured=engine.compare_still(packet,None)
other=recorded_packet(config,3);engine.compare_still(other,None)
attached=engine.attach_measurement(captured,observed,packet,recorded=True)
assert abs(attached['cost_px2']-baseline['cost_px2'])<1e-8
assert attached['t_s']==packet['t_s']
source=dict(rgb=rgb,packet=packet,result=None,observed=None,setup=engine.setup(),
    parameters=dict(engine.p),d2=.15,message='No image measurement',recorded_demo=True,setup_preview=False,
    displayed_age_s=0.,received=time.monotonic(),landmarks=baseline['landmarks'])
manual=prepare_capture(source,config,fit)
assert manual['observed'] is None and manual['result']['manual_capture_ready']
assert manual['result']['d2']==.15
rows=correspondences(None,manual['result']['landmarks'])
visible=[r for r in correspondences(observed,baseline['landmarks']) if r['physical_px'] is not None]
chosen=visible[::5]
for row in rows:
    found=next((r for r in chosen if r['id']==row['id']),None)
    if found:row['physical_px']=(np.array(found['physical_px'])+[25.,3.]).tolist()
complete=complete_capture(manual,rows,'joints',True)
assert len(complete['result']['observation']['points_px'])==len(chosen)>=4
assert complete['result']['cost_px2']==point_cost(complete['result']['observation']['points_px'],complete['result']['predicted_points_px'])
assert complete['result']['observation']['measurement_source']=='manual frozen coordinates'
scan=engine.scan(complete['result'],count=5,lower=.4,upper=1.3)
assert scan['observation']==complete['result']['observation']
for image_source in [dict(source,setup_preview=True),dict(source,recorded_demo=False,displayed_age_s=5),
                     dict(source,recorded_demo=False,packet=dict(packet,motor_active=False)),
                     dict(source,recorded_demo=False,packet=dict(packet,motor=[dict(r,state='MOVING') for r in packet['motor']]))]:
    rejected=prepare_capture(image_source,config,fit)
    assert rejected['capture_error'] and not rejected['result']['manual_capture_ready']
    try:complete_capture(rejected,rows,'joints',True)
    except ValueError:pass
    else:raise AssertionError('Invalid captured inputs scored')
try:complete_capture(manual,rows,'joints',False)
except ValueError:pass
else:raise AssertionError('Missing stationary confirmation accepted')
print('PASS manual points with no extraction/texture/tracking; same captured model; stale/unreferenced/moving inputs explained')

app=QApplication.instance() or QApplication([])
def until(predicate,seconds=30):
    deadline=time.monotonic()+seconds
    while time.monotonic()<deadline:
        app.processEvents()
        if predicate():return
        time.sleep(.01)
    raise AssertionError('UI condition timed out')
def warning(*args):raise AssertionError('Unexpected UI warning: '+str(args[2]))
QMessageBox.warning=warning
def forbidden(*args,**kwargs):raise AssertionError('Manual mode invoked image processing')
with patch('interface.measure',forbidden),patch('joint_tracking.ManualJointTracker.update',forbidden):
    window=Window(dict(config,analysis_mode='manual'),copy.deepcopy(fit),None,True);window.show()
    try:
        until(lambda:window.last_model is not None)
        window.tick();assert window.scan_button.isEnabled() and window.latest['observed'] is None
        assert 'No image estimation' in window.sweep_ready.text()
        assert window.same_speed.isChecked() and not window.length_speeds[1].isEnabled()
        window.length_speeds[0].setValue(2.3);assert window.length_speeds[1].value()==2.3
        window.same_speed.setChecked(False);window.length_speeds[1].setValue(.7)
        assert window.length_speeds[0].value()==2.3 and window.length_speeds[1].isEnabled()
        window.motor_mode.setCurrentIndex(1)
        sent=[];window.command=lambda action,**values:sent.append((action,values))
        window.length_targets[0].setValue(150);window.length_targets[1].setValue(290)
        window.length_move.setEnabled(True);window.length_move.click()
        assert sent[-1]==('independent',dict(lengths_mm=[150.,290.],speeds_mm_s=[2.3,.7]))
        window.length_buttons[1].setEnabled(True);window.length_buttons[1].click()
        assert sent[-1]==('axis',dict(axis=1,length_mm=290.,speed_mm_s=.7))
        window.scan_min.setValue(.4);window.scan_max.setValue(1.3);window.scan_count.setValue(5)
        window.scan_button.click();until(lambda:window.inspector is not None and not window.manual_capture_busy)
        inspector=window.inspector;assert inspector.manual_mode and inspector.snapshot['observed'] is None
        assert not inspector.sweep_profile_button.isEnabled() and 'four' in inspector.manual_status.text()
        inspector.manual={r['id']:(np.array(r['physical_px'])+[25.,3.]).tolist() for r in chosen}
        inspector.rebuild();inspector.stationary.setChecked(True)
        assert inspector.sweep_profile_button.isEnabled()
        saved_frame=inspector.source_item['packet']['frame_index']
        window.steps.setCurrentIndex(2)
        until(lambda:window.last_model['packet']['frame_index']!=saved_frame)
        inspector.sweep_profile_button.click();until(lambda:window.sweep_result is not None)
        assert window.sweep_snapshot['packet']['frame_index']==saved_frame
        assert window.sweep_snapshot['observed']['measurement_source']=='manual frozen coordinates'
        destination=window.export_pose(out/'manual_pose.png')
        metadata=json.loads(destination.with_suffix('.json').read_text())
        assert metadata['result']['observation']==metadata['sweep']['observation']
        assert metadata['frame_index']==saved_frame
        assert metadata['result']['observation']['points_px']==list(inspector.manual.values())
        inspector.grab().save(str(out/'manual_editor.png'));inspector.close()
        window.grab().save(str(out/'manual_window.png'))
        with tempfile.TemporaryDirectory() as tmp:
            os.chdir(tmp);window.analysis_mode_box.setCurrentIndex(0)
            assert json.loads(Path('lab.local.json').read_text())['analysis_mode']=='vision'
        os.chdir(ROOT)
        until(lambda:window.last_vision is not None)
        window.tick();assert not window.scan_button.isEnabled() and 'Manual points mode' in window.sweep_ready.text()
        assert window.camera.predicted
        print('PASS actual manual capture/points/sweep/export buttons; independent linked speeds; mode preference; failed extraction has a visible explanation')
    finally:window.close();app.processEvents();os.chdir(ROOT)

# Image processing deliberately takes 350 ms. Physics and camera must continue
# at their own cadence; delayed measurements retain their original frame/model.
class Link:url='test';token='x'*40
started=time.monotonic();counter=[0]
def source_packet(*args,**kwargs):
    p=copy.deepcopy(packet);counter[0]+=1
    dt=time.monotonic()-started;p['t_s']+=dt;p['frame_index']=counter[0]
    p['motor']=[dict(r,t_s=r['t_s']+dt) for r in p['motor']]
    return p
def slow_measure(*args):time.sleep(.35);return rgb,copy.deepcopy(observed)
items=[];raw=[]
with patch('interface.camera_get',source_packet),patch('interface.measure',slow_measure):
    preview=Preview(config,fit,Link(),False)
    preview.updated.connect(items.append);preview.video.connect(raw.append);preview.start()
    try:until(lambda:any(i.get('vision_result') and i.get('result',{}).get('settled') for i in items),seconds=12)
    finally:preview.stop.set();assert preview.wait(3500)
models=[i for i in items if i.get('result') and not i.get('vision_result')]
images=[i for i in items if i.get('vision_result') and i.get('result')]
assert len(models)>2*len(images) and len(models)>15,(len(models),len(images))
assert len(raw)>=len(models)
assert all(i['result']['t_s']==i['packet']['t_s'] for i in images)
assert all(abs(i['result']['cost_px2']-baseline['cost_px2'])<.02 for i in images if i['result']['cost_px2'] is not None)
assert preview.vision.pending.qsize()<=1 and preview.vision.finished.qsize()<=1
print(f'PASS slow vision isolation: {len(models)} model frames / {len(images)} measured frames; automatic settled sweep recovered; queues bounded')
print('All v0.7.0 checks passed. No physical hardware was operated.')

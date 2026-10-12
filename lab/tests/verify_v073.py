"""No startup retention modal; exact frozen stiffness surfaces and native Qt flow.

Recorded images, synthetic physical points and fake storage only. No hardware.
"""
import base64,copy,csv,hashlib,json,os,sys,threading,time
from pathlib import Path
from unittest.mock import patch
import numpy as np
os.environ.setdefault('QT_QPA_PLATFORM','offscreen')
ROOT=Path(__file__).resolve().parents[1];sys.path.insert(0,str(ROOT));os.chdir(ROOT)
from analysis import Comparator
from spirob_analysis.model import SpiRobModel
from spirob_analysis.fitting import predict
from stiffness_sweep import sample_surface, SurfaceCancelled
from surface_review import export_surface, draw_surface
from interface import Window,QApplication,recorded_packet
from PySide6.QtWidgets import QMessageBox
from matplotlib.backend_bases import MouseEvent
from matplotlib.figure import Figure

config=json.loads(Path('config.json').read_text());fit=json.loads(Path('calibration/pilot_fit.json').read_text())
model=SpiRobModel(config['model']);old=model.original_k.copy();d=model.original_d.copy()
model.set_stiffness_profile();model.scale(model.k2,model.d2)
assert np.array_equal(model.k_weights,old[1:]/model.k2)
assert np.allclose(model.m.jnt_stiffness,old,rtol=1e-15,atol=0) and np.allclose(model.m.dof_damping,d,rtol=1e-15,atol=0)
model.set_stiffness_profile(.92,1.5);model.scale(.7,.02)
assert np.allclose(model.m.jnt_stiffness[1:],.7*.92**(1.5*np.arange(20)),rtol=1e-13)
assert model.m.jnt_stiffness[0]==old[0] and model.m.dof_damping[0]==d[0]
assert np.allclose(model.m.dof_damping[1:],.02*model.d_weights)
for r,b in [(float('nan'),1),(.4,1),(.9,-.1),(1.,4)]:
 try:model.set_stiffness_profile(r,b)
 except ValueError:pass
 else:raise AssertionError('Invalid profile accepted')

engine=Comparator(config,fit);pkt=recorded_packet(config,2)
from analysis import inputs_at
lengths,_=inputs_at(pkt,pkt['t_s'],config['live']['max_input_gap_s'])
obs=dict(displacement_mm=(lengths-np.array(pkt['rest_lengths_mm'])).tolist(),
         landmark_ids=['j_002','j_005','j_010','j_015','j_020','tip'],landmark_mode='joints',
         tail_distance_px=[0]*6,points_px=[[0,0]]*6)
truth=dict(engine.p,k2=float(np.sqrt(.5)),stiffness_ratio=.92,stiffness_exponent=1.)
obs['points_px']=predict(model,obs,truth)[0].tolist()
source=dict(fixed_parameters=copy.deepcopy(engine.p),observation=obs,lower=.5,upper=1.,count=3,
            frame_t_s=pkt['t_s'],session=pkt['session'],frame_index=pkt['frame_index'],
            setup_revision=0,setup=engine.setup(),frozen_d2=engine.d2,recorded_demo=True,
            model_sha256=hashlib.sha256(Path(config['model']).read_bytes()).hexdigest(),
            frame_sha256=hashlib.sha256(base64.b64decode(pkt['image_b64'])).hexdigest())
before=json.dumps(source,sort_keys=True);progress=[]
surface=sample_surface(config['model'],source,'stiffness_ratio',.90,.94,3,.5,1.,3,
                       progress=lambda a,b:progress.append((a,b)))
best=surface['minimum']
assert abs(best['k2']-truth['k2'])<1e-12 and abs(best['constant_value']-.92)<1e-12
assert best['cost_px2']<1e-14 and not best['at_boundary']
assert json.dumps(source,sort_keys=True)==before and progress[-1][0]==surface['actual_sample_count']
# Constant row at the current profile agrees sample-for-sample with the old 1D sweep.
result=dict(settled=True,cost_px2=1.,observation=obs,t_s=pkt['t_s'])
line=engine.scan(result,3,.5,1.)
r=surface['constant_values'].index(engine.p['stiffness_ratio'])
assert np.allclose(surface['cost_px2'][r],line['cost_px2'],atol=1e-8,rtol=1e-8)
# Review the other constant, holding r fixed. Exact synthetic truth at b=.75.
truth_b=dict(engine.p,k2=float(np.sqrt(.5)),stiffness_exponent=.75)
source_b=copy.deepcopy(source);source_b['observation']['points_px']=predict(model,obs,truth_b)[0].tolist()
surface_b=sample_surface(config['model'],source_b,'stiffness_exponent',.5,1.,3,.5,1.,3)
assert abs(surface_b['minimum']['constant_value']-.75)<1e-12 and surface_b['minimum']['cost_px2']<1e-14
# r and b are confounded through r**b; equal effective ratios give equal predictions.
p1=dict(engine.p,stiffness_ratio=.94,stiffness_exponent=1.5)
p2=dict(engine.p,stiffness_ratio=.94**1.5,stiffness_exponent=1.)
assert np.allclose(predict(model,obs,p1)[0],predict(model,obs,p2)[0],atol=1e-5)
q=engine.model.d.qpos.copy();qd=engine.model.d.qvel.copy();when=engine.model.d.time
engine.set_parameters(.8,.02,.93,1.2)
assert np.array_equal(q,engine.model.d.qpos) and np.array_equal(qd,engine.model.d.qvel) and when==engine.model.d.time
assert np.isclose(engine.model.m.jnt_stiffness[2]/engine.model.m.jnt_stiffness[1],.93**1.2)
print('PASS exact XML defaults, profile equation, fixed joint 1/damping, both surface axes, synthetic minima, 1D equivalence, r^b degeneracy and live-state preservation')

original_predict=predict
def partial(m,o,p):
 if p['k2']>=.99:raise RuntimeError('injected invalid equilibrium')
 return original_predict(m,o,p)
with patch('stiffness_sweep.predict',partial):
 invalid=sample_surface(config['model'],source,'stiffness_ratio',.92,.96,3,.5,1.,3)
assert invalid['invalid_sample_count']==len(invalid['constant_values'])
assert invalid['minimum']['k2']<1. and invalid['minimum']['at_boundary']
json.dumps(invalid,allow_nan=False)
with patch('stiffness_sweep.predict',side_effect=RuntimeError('injected all invalid')):
 try:sample_surface(config['model'],source,'stiffness_ratio',.9,.94,3,.5,1.,3)
 except ValueError as exc:assert 'No valid equilibrium' in str(exc)
 else:raise AssertionError('All-invalid grid accepted')
cancel=threading.Event()
try:sample_surface(config['model'],source,'stiffness_ratio',.9,.94,3,.5,1.,3,
                   progress=lambda done,total:cancel.set(),cancel=cancel)
except SurfaceCancelled:pass
else:raise AssertionError('Cancellation ignored')
bad=dict(source,model_sha256='wrong')
try:sample_surface(config['model'],bad,'stiffness_ratio',.9,.94,3,.5,1.,3)
except ValueError as exc:assert 'XML changed' in str(exc)
else:raise AssertionError('Changed XML accepted')
print('PASS invalid equilibria omitted, boundary flagged, all-invalid grid refused, cancellation, unchanged frozen input and model hash gate')

app=QApplication.instance() or QApplication([])
def until(predicate,seconds=30):
 deadline=time.monotonic()+seconds
 while time.monotonic()<deadline:
  app.processEvents()
  if predicate():return
  time.sleep(.01)
 raise AssertionError('GUI condition timed out')

# Reproduce the real launch condition: a configured SSH host and an older run
# without shutdown acknowledgement. Launch must do no remote recovery or modal.
with (patch('interface.review_entries',return_value=[{'session':'OLD_LIVE_test','shutdown_confirmed':False}]) as review,
      patch('interface.Window.ask_run_decision',return_value='later') as ask):
 w=Window(config,fit,{'ssh_host':'fake-host'},False);w.show()
 deadline=time.monotonic()+.7
 until(lambda:time.monotonic()>deadline)
 assert review.call_count==0 and ask.call_count==0 and not w.review_busy
 w.review_button.click();until(lambda:not w.review_busy and ask.call_count==1)
 assert review.call_count==1
 w.close();app.processEvents()
# Never turn an unknown hardware shutdown into permission to delete logs.
w=Window(config,fit,{},False)
buttons={}
def inspect_box(box):
 for b in box.buttons():buttons[b.text()]=b.isEnabled()
 return 0
with patch.object(QMessageBox,'exec',inspect_box):
 assert w.ask_run_decision({'session':'OLD_LIVE_test','shutdown_confirmed':False})=='later'
assert buttons['Save'] and not buttons['Discard']
w.close()
print('PASS configured-host startup has no modal or SSH recovery; explicit pending review works; unconfirmed shutdown cannot discard')

out=Path(os.environ.get('SPIROB_TEST_OUTPUT','/tmp/spirob-v073-verification'));out.mkdir(parents=True,exist_ok=True)
w=Window(config,fit,None,True);w.show()
try:
 until(lambda:w.latest and w.latest.get('result') and w.latest['result'].get('settled') and w.scan_button.isEnabled())
 assert not w.surface_check.isChecked() and not w.profile_edit.isChecked()
 # 3D without a completed 1D sweep explains what is missing without a modal.
 w.surface_check.setChecked(True);assert not w.surface_check.isChecked() and 'normal frozen-pose' in w.command_status.text()
 w.scan_min.setValue(.5);w.scan_max.setValue(1.);w.scan_count.setValue(3)
 w.scan_button.click();until(lambda:w.sweep_result is not None and not w.scan_busy)
 snapshot=copy.deepcopy(w.sweep_result)
 w.analysis_tabs.setCurrentIndex(1);w.surface_check.setChecked(True)
 dialog=w.surface_dialog;assert dialog.source==snapshot
 dialog.low.setValue(.90);dialog.high.setValue(.94);dialog.count.setValue(3);dialog.kcount.setValue(3)
 dialog.run_button.click();until(lambda:dialog.result is not None and not dialog.running,60)
 assert dialog.result['source']['frame_sha256']==snapshot['frame_sha256']
 # Rotation is actual interactive Matplotlib 3D navigation.
 dialog.canvas.draw();ax=dialog.figure.axes[0];before_angle=(ax.elev,ax.azim)
 bounds=ax.bbox;x=bounds.x0+.55*bounds.width;y=bounds.y0+.5*bounds.height
 for name,dx,dy in [('button_press_event',0,0),('motion_notify_event',25,15),('button_release_event',25,15)]:
  event=MouseEvent(name,dialog.canvas,x+dx,y+dy,button=1,buttons={1} if name=='motion_notify_event' else None)
  dialog.canvas.callbacks.process(name,event)
 assert (ax.elev,ax.azim)!=before_angle
 for size in [(1280,720),(1366,768),(1920,1080)]:
  w.resize(*size);dialog.resize(min(1100,size[0]-60),size[1]-80);app.processEvents();dialog.canvas.draw()
  for widget in (dialog.run_button,dialog.use_button,dialog.status,dialog.canvas,dialog.toolbar):
   pos=widget.mapTo(dialog,widget.rect().topLeft());assert pos.x()>=0 and pos.y()>=0
   assert pos.x()+widget.width()<=dialog.width() and pos.y()+widget.height()<=dialog.height()
  dialog.grab().save(str(out/f'surface_{size[0]}x{size[1]}.png'))
 dialog.grab().save(str(out/'surface_light.png'))
 before_p=copy.deepcopy(w.preview.engine.p)
 dialog.use_button.click();app.processEvents()
 assert w.preview.engine.p==before_p and w.profile_edit.isChecked()
 p=dialog.result['minimum']['parameters']
 assert np.isclose(w.rbox.value(),p['stiffness_ratio']) and np.isclose(w.bbox.value(),p['stiffness_exponent'])
 w.apply_button.click();until(lambda:not w.pending_comparison_change and np.isclose(w.preview.engine.p['stiffness_ratio'],p['stiffness_ratio']))
 assert np.isclose(w.preview.engine.model.m.jnt_stiffness[2]/w.preview.engine.model.m.jnt_stiffness[1],p['stiffness_ratio']**p['stiffness_exponent'])
 # Changing the surface selector retains frozen source, does not change live parameters.
 dialog.parameter.setCurrentIndex(1);assert dialog.source==snapshot
 dialog.low.setValue(.8);dialog.high.setValue(1.2);dialog.count.setValue(3)
 dialog.run_button.click();until(lambda:not dialog.running,60)
 assert dialog.result['parameter']=='stiffness_exponent'
 w.change_theme('Dark');app.processEvents();dialog.canvas.draw();dialog.grab().save(str(out/'surface_dark.png'))
 export_surface(out/'surface.png',dialog.result)
 saved=json.loads((out/'surface.json').read_text());rows=list(csv.DictReader((out/'surface.csv').open()))
 assert saved==dialog.result and len(rows)==saved['actual_sample_count']
 assert sum(row['lowest_sample']=='True' for row in rows)==1
 assert saved['observation']==snapshot['observation']
 w.setup_revision+=1
 try:w.use_surface_minimum(dialog.result)
 except ValueError:pass
 else:raise AssertionError('Changed setup permitted stale surface selection')
 dialog.close();app.processEvents();assert not w.surface_check.isChecked()
 w.change_theme('Light')
finally:w.close();app.processEvents()
fig=Figure(figsize=(8,6));draw_surface(fig,invalid);fig.savefig(out/'surface_gaps.png')
print('PASS actual 3D dialog, both axes, frozen capture, rotation, staging/explicit Apply, three window sizes, both themes and PNG/CSV/JSON exports')
print('All v0.7.3 checks passed. No physical hardware was operated.')

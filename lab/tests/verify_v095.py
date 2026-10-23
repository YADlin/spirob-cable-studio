"""Hand-edited per-joint XML loads; joint editor applies and saves values. No hardware.

Run: QT_QPA_PLATFORM=offscreen python tests/verify_v095.py
"""
import copy,json,os,sys,tempfile
from pathlib import Path
import numpy as np
os.environ.setdefault('QT_QPA_PLATFORM','offscreen')
ROOT=Path(__file__).resolve().parents[1];sys.path.insert(0,str(ROOT));os.chdir(ROOT)
import mujoco
from analysis import Comparator
from interface import Window,QApplication,recorded_packet
from joint_editor import write_joint_values
from spirob_analysis.model import SpiRobModel
from spirob_analysis.fitting import predict

XML=Path('model/spirob_robot.xml')
names=[f'j_{i:03d}' for i in range(2,22)]
k=np.r_[np.linspace(.2,.05,12),np.linspace(.06,.12,8)]          # U-shaped, not exponential
d=np.r_[.004,np.full(19,.002)]
custom=XML.with_name('verify_custom_tmp.xml')
try:
    write_joint_values(XML,names,k,d,custom)
    m=mujoco.MjModel.from_xml_path(str(custom))
    assert np.allclose(m.jnt_stiffness[1:],k) and np.allclose(m.dof_damping[1:],d) and m.jnt_stiffness[0]==100
    assert XML.read_text().count('<joint')==custom.read_text().count('<joint')
    model=SpiRobModel(custom);assert not model.xml_exponential
    cfg=json.loads(Path('config.json').read_text());cfg['model']=str(custom);cfg.pop('comparison_setup',None)
    fit=json.loads(Path('calibration/new_model_initial.json').read_text())
    e=Comparator(cfg,fit)
    assert e.p['stiffness_profile']=='xml' and np.allclose(e.model.m.jnt_stiffness[1:],k) and np.allclose(e.model.m.dof_damping[1:],d)
    e.set_parameters(2*k[0],2*d[0])                               # scaling keeps the per-joint shape
    assert np.allclose(e.model.m.jnt_stiffness[1:],2*k) and np.allclose(e.model.m.dof_damping[1:],2*d)
    k2=k*1.5;d2=d*3;e.set_joint_values(k2,d2)
    assert np.allclose(e.model.m.jnt_stiffness[1:],k2) and np.allclose(e.model.m.dof_damping[1:],d2) and e.p['stiffness_profile']=='joints'
    obs=dict(displacement_mm=[-10.,10.],landmark_ids=['link_005','link_010','link_015','link_020'],landmark_mode='centres',tail_distance_px=[0]*4,points_px=[[0,0]]*4)
    M=SpiRobModel(custom);M.set_joint_values(k2,d2)
    ref=predict(M,obs,dict(e.p,joint_weights=None,stiffness_ratio=None) if False else e.p)[0]
    M2=SpiRobModel(custom);M2.set_joint_values(k2,d2);direct=M2.equilibrium(M2.controls([-10,10]),k2=k2[0])['curve_m']
    assert np.all(np.isfinite(ref))
    e.set_parameters(.18,.009,.9,1.)                              # explicit exponential replaces per-joint values
    assert e.p['stiffness_profile']=='exponential' and 'joint_stiffness' not in e.p and np.isclose(e.model.m.jnt_stiffness[2]/e.model.m.jnt_stiffness[1],.9)
    print('PASS hand-edited XML loads; per-joint k/d kept, scaled, replaced and reset to exponential on request',flush=True)
    app=QApplication.instance() or QApplication([])
    w=Window(cfg,fit,{},False);w.show()
    assert np.isclose(w.kbox.value(),k[0]) and np.isclose(w.dbox.value(),d[0])
    w.open_joint_editor();ed=w.joint_editor
    kv,dv=ed.values();assert np.allclose(kv,k,rtol=1e-5) and np.allclose(dv,d,rtol=1e-5)
    ed.k2.setValue(.2);ed.r.setValue(.9);ed.fill_exponential();kv,_=ed.values();assert np.isclose(kv[3]/kv[2],.9,rtol=1e-4)
    sent=[]
    class Q:
        def put(self,item):sent.append(item)
    class P: requests=Q()
    w.preview=P();ed.apply();assert sent and sent[-1][0]=='joint_values' and np.isclose(sent[-1][1][0][0],.2)
    out=custom.with_name('verify_custom_saved_tmp.xml')
    try:
        write_joint_values(custom,ed.names,*ed.values(),out);mm=mujoco.MjModel.from_xml_path(str(out));assert np.isclose(mm.jnt_stiffness[2],.2*.9,rtol=1e-6)
    finally:out.unlink(missing_ok=True)
    w.preview=None;ed.close();w.close();app.processEvents()
    from interface import Preview
    pv=Preview(cfg,fit);pv.engine=Comparator(cfg,fit);got=[];pv.applied.connect(got.append)
    pv.requests.put(('joint_values',(list(k*2),list(d))));assert pv.handle_requests()
    app.processEvents();assert np.allclose(pv.engine.model.m.jnt_stiffness[1:],2*k)
    print('PASS simulation thread applies per-joint values on request',flush=True)
    print('PASS joint editor: loads XML values, fills exponential, sends values to the simulation, saves an XML',flush=True)
finally:custom.unlink(missing_ok=True)
print('PASS all v0.9.5 per-joint checks',flush=True)

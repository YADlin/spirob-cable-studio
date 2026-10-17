"""Model, limits, real demo-driver dispatch, STOP, preview and GUI regressions.

Run: python tests/verify_v090.py --motor-source /path/to/spirob-cable-studio
No serial ports, SSH sessions or physical camera are opened.
"""
import argparse,copy,json,os,sys,tempfile,threading,time
from dataclasses import replace
from pathlib import Path
from unittest.mock import patch,Mock
import numpy as np
ROOT=Path(__file__).resolve().parents[1];sys.path.insert(0,str(ROOT));os.chdir(ROOT)
os.environ.setdefault('QT_QPA_PLATFORM','offscreen')
from spirob_analysis.model import SpiRobModel
from motion_profile import validate_profile,encode_profile,decode_profile
from motion_settings import model_profile,model_bounds
from motion_limits import worker_limits
from actuation import targets,independent_targets,dispatch,Inbox
from campaign import expand_plan,CampaignRunner
from campaign_preview import random_plan,simulate,review_context,digest,require_review,PreviewCancelled


def rejected(fn):
    try:fn()
    except (ValueError,RuntimeError):return
    raise AssertionError('Invalid input was accepted')


def main():
    parser=argparse.ArgumentParser();parser.add_argument('--motor-source',required=True);args=parser.parse_args()
    sys.path.insert(0,str(Path(args.motor_source).resolve()))
    cfg=json.loads(Path('config.json').read_text());fit=json.loads(Path(cfg.get('fit','calibration/new_model_initial.json')).read_text())
    model=SpiRobModel(cfg['model']);p=model_profile(model);rests=p['rest_lengths_mm']
    assert model.m.njnt==21 and np.allclose(model.neutral_lengths*1000,227.353314410471)
    assert np.allclose(model.m.actuator_ctrlrange,[[.15,.3],[.15,.3]])
    assert np.allclose(model.kp,2500) and np.allclose(model.m.actuator_biasprm[:,2],-20)
    assert np.allclose(model.m.actuator_forcerange,[[-1000,0],[-1000,0]])
    assert np.ptp(model.joint_curve()[:,1])>.2 and np.ptp(model.joint_curve()[:,0])<1e-9
    assert decode_profile(encode_profile(p))==p
    assert np.allclose(model_bounds(model,rests),[[150,300],[150,300]])
    assert targets(225,150,20,rests,p)==[300,150]
    assert independent_targets([150,300],[20,15],rests,p)==[150,300]
    for fn in (lambda:targets(225,151,20,rests,p),lambda:independent_targets([149,250],[1,1],rests,p),
               lambda:independent_targets([250,250],[20.01,1],rests,p),lambda:validate_profile(dict(p,rest_lengths_mm=[220,float('nan')]))):rejected(fn)
    shifted=[240.,235.];assert np.allclose(model_bounds(model,shifted),np.array([[150,300],[150,300]])+np.array(shifted)[:,None]-np.array(rests)[:,None])
    print('PASS model geometry, position PD, mapping, reference serialization and limits',flush=True)

    from spirob_cable.config import RigSettings
    from spirob_cable.drive import DemoDrive,DriveFault
    from spirob_cable.records import NullRecorder,AxisLog
    from spirob_cable.model import Rejected
    from spirob_cable import worker
    setup=RigSettings.demo();setup=replace(setup,axes=tuple(replace(a,rest_length_mm=r,setup_travel_mm=p['setup_travel_mm']) for a,r in zip(setup.axes,rests)))
    now=[100.]
    with patch('time.monotonic',lambda:now[0]),worker_limits(rests,p):
        log=Mock(wraps=NullRecorder());drives=[DemoDrive(a,AxisLog(log,i)) for i,a in enumerate(setup.selected)]
        rig=worker.Rig(setup,drives,log)
        def poll(n=1):
            for _ in range(n):now[0]+=.05;rig.poll_all()
        def settle():
            poll(3)
            for _ in range(2000):
                poll()
                if all(a.state in ('HOLDING','DISABLED') for a in rig.axes):return
            raise AssertionError('Move did not settle')
        rig.connect();poll(3);rig.enable([0,1]);settle()
        rig.jog_pair([25,25],20);settle()
        assert np.allclose([a.snapshot()['connection_mm'] for a in rig.axes],[25,25],atol=.003)
        for i,r in enumerate(rests):rig.set_rest(i,r)
        rig.jog_pair([-15,-15],15);settle()
        assert np.allclose([a.absolute() for a in rig.axes],np.array(rests)-15,atol=.003)
        stages=[]
        for i,d in enumerate(drives):
            d.stage_target=Mock(wraps=d.stage_target);d.commit_target=Mock(wraps=d.commit_target)
        try:rig.move_independent([180,301],[10,10])
        except (Rejected,ValueError):pass
        else:raise AssertionError('Out-of-bounds second endpoint accepted')
        assert not any(d.stage_target.called or d.commit_target.called for d in drives)
        # 300 is inside the symmetric ±77.353 envelope but 303 is not inside
        # the requested asymmetric 150..300 physical bounds.
        try:rig.move_axis(1,301,10,True)
        except Rejected:pass
        else:raise AssertionError('Asymmetric endpoint bound bypassed')
        rig.move_independent([150,300],[20,15]);poll(4)
        rig.stop_all();settle()
        held=[a.absolute() for a in rig.axes];assert abs(held[0]-150)>10 and abs(held[1]-300)>10
        poll(30);assert np.allclose([a.absolute() for a in rig.axes],held,atol=.002)
        assert all(a.enabled for a in rig.axes)
        rig.move_independent([180,270],[20,20]);settle()
        assert np.allclose([a.absolute() for a in rig.axes],[180,270],atol=.003)
        # Both preparation phases are cancelled before commit if STOP arrives.
        from spirob_cable.rig import Cancelled
        stop=[False];rig.cancelled=lambda:stop[0]
        original=drives[0].stage_target
        def stage(*a):result=original(*a);stop[0]=True;return result
        drives[0].stage_target=stage
        for d in drives:d.commit_target.reset_mock()
        try:rig.jog_pair([10,10],20)
        except Cancelled:pass
        else:raise AssertionError('STOP did not cancel staged paired motion')
        assert not any(d.commit_target.called for d in drives)
    # Inbox STOP uses its priority channel and flushes queued motion.
    import io
    inbox=Inbox(io.StringIO('\n'.join(json.dumps(m) for m in [dict(id=1,action='jog_pair'),dict(id=2,action='hold')])+'\n'))
    deadline=time.monotonic()+2
    while not inbox.close.is_set() and time.monotonic()<deadline:time.sleep(.001)
    assert inbox.stop.is_set() and inbox.commands.empty()
    print('PASS actual demo-driver paired setup/reference jog, 20 mm/s, asymmetric bounds, STOP and no partial pair commit',flush=True)

    rows=random_plan(p,'Both cables',[[rests[0]-1,rests[0]+1],[rests[1]-1,rests[1]+1]],[10,20],3,713,linked=False)
    assert rows==random_plan(p,'Both cables',[[rests[0]-1,rests[0]+1],[rests[1]-1,rests[1]+1]],[10,20],3,713,linked=False)
    plan=expand_plan(rows,rests,rests,p,dwell_s=0)
    result=simulate(plan,cfg['model'],fit['candidate']['parameters'],model.d2,{})
    assert len(result['summary']['poses'])==4 and len(result['frames'])>30
    assert result['summary']['poses'][-1]['role']=='return'
    assert np.isfinite(np.array([f['curve_mm'] for f in result['frames']])).all()
    event=threading.Event();event.set()
    try:simulate(plan,cfg['model'],fit['candidate']['parameters'],model.d2,{},event)
    except PreviewCancelled:pass
    else:raise AssertionError('Preview cancellation failed')
    context=review_context(rows,0,p,cfg['model'],fit['candidate']['parameters'],model.d2,{})
    review=dict(fingerprint=digest(context),plan=plan)
    assert require_review(review,context,rests)==plan
    for key,value in [('dwell_s',1),('d2',model.d2*2),('setup',{'signs':[-1,1]})]:
        changed=copy.deepcopy(context);changed[key]=value;rejected(lambda:require_review(review,changed,rests))
    rejected(lambda:require_review(review,context,[rests[0]+1,rests[1]]))
    rejected(lambda:require_review(None,context,rests))
    print('PASS seeded generation, dynamic MuJoCo preview, cancellation and approval invalidation',flush=True)

    from interface import Window,QApplication
    from campaign_window import CampaignWindow
    from campaign_preview_window import RandomCampaign,CampaignPreview
    from motion_settings import MotionSettings
    app=QApplication.instance() or QApplication([])
    with patch('interface.QTimer.singleShot'):window=Window(cfg,fit,None,False)
    window.show();window.timer.stop();app.processEvents()
    assert abs(window.mean.value()-sum(rests)/2)<.001 and window.jog_step.maximum()==1000 and window.speed.maximum()==20
    assert len(window.jog_buttons)==6
    dialog=MotionSettings(window);dialog.fields[0][0].setValue(240);dialog.fields[1][0].setValue(240);dialog.map_bounds();dialog.save()
    window.motion_profile=dialog.profile;window.update_motion_widgets();assert window.mean.value()==240
    window.motion_profile=p;window.update_motion_widgets()
    campaign=CampaignWindow(window);window.campaign_window=campaign;campaign.set_rows(rows);campaign.dwell.setValue(0)
    draft,_=campaign.build_plan();context=campaign.review_context()
    campaign.approve_preview(dict(fingerprint=digest(context),context=context,plan=draft,summary=result['summary']))
    assert campaign.approved_review is not None
    campaign.table.item(0,3).setText('12');assert campaign.approved_review is None
    # Draw the actual simulator result and exercise the approval callback without
    # repeating physics. The integration itself was checked above.
    reviews=[]
    with patch('campaign_preview_window.simulate',return_value=result):
        preview=CampaignPreview(campaign,plan,cfg['model'],context,reviews.append)
        preview.worker.join(3);preview.poll();preview.show();app.processEvents();preview.slider.setValue(preview.slider.maximum());preview.approve()
        assert len(reviews)==1 and reviews[0]['plan']==plan
        out=Path(os.environ.get('SPIROB_TEST_OUTPUT','/tmp/spirob-v090-verification'));out.mkdir(parents=True,exist_ok=True)
        preview.grab().save(str(out/'simulation_preview.png'));preview.close()
    campaign.close();window.hide()
    print('PASS GUI references, editable ranges, six jog buttons, preview rendering and explicit approval',flush=True)
    result['summary']['tests']='Model, motion bounds, paired jog, STOP, random plans, approval and headless GUI passed; no physical hardware tested.'
    (out/'verification.json').write_text(json.dumps(result['summary'],indent=2)+'\n')


if __name__=='__main__':main()

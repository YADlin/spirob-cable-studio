"""Mixed campaigns, stepped sweeps and acknowledged return; no hardware I/O.

Run: QT_QPA_PLATFORM=offscreen python tests/verify_v091.py
"""
import base64
import copy
import io
import json
import os
from pathlib import Path
import sys
import tempfile
from unittest.mock import patch

import numpy as np
from PIL import Image

ROOT=Path(__file__).resolve().parents[1];sys.path.insert(0,str(ROOT));os.chdir(ROOT)
os.environ.setdefault('QT_QPA_PLATFORM','offscreen')
from campaign import MODES,CampaignRunner,expand_plan
from campaign_preview import random_plan,difference_sweep,simulate,review_context,digest,require_review
from campaign_storage import CampaignStore
from spirob_analysis.model import SpiRobModel


def rejected(fn):
    try:fn()
    except ValueError:return
    raise AssertionError('Invalid input accepted')


def main():
    cfg=json.loads(Path('config.json').read_text());fit=json.loads(Path(cfg['fit']).read_text())
    profile=cfg['motion_profile'];rests=profile['rest_lengths_mm'];mean=sum(rests)/2
    bounds=[[r-1,r+1] for r in rests];speeds=[10.,20.]
    rows=random_plan(profile,'Mixed',bounds,speeds,8,912,mean,False,[-1,1])
    assert rows==random_plan(profile,'Mixed',bounds,speeds,8,912,mean,False,[-1,1])
    before=copy.deepcopy((profile,rows))
    initial=[rests[0]+.25,rests[1]-.15]
    plan=expand_plan(rows,initial,rests,profile,dwell_s=0,return_speed_mm_s=2.)
    for seed in range(40):
        draft=random_plan(profile,'Mixed',bounds,speeds,17,seed,mean,False,[-1,1])
        assert draft[0]['mode']=='Both cables'
        for offset in range(0,16,4):assert {r['mode'] for r in draft[offset:offset+4]}==set(MODES)
        expanded=expand_plan(draft,initial,rests,profile,dwell_s=0)
        previous=initial
        for pose in expanded['poses'][:-1]:
            assert all(lo<=v<=hi for v,(lo,hi) in zip(pose['lengths_mm'],bounds))
            assert all(speeds[0]<=s<=speeds[1] for s in pose['speeds_mm_s'])
            if pose['mode']=='Cable 1':assert pose['lengths_mm'][1]==previous[1]
            if pose['mode']=='Cable 2':assert pose['lengths_mm'][0]==previous[0]
            if pose['mode']=='Difference':
                assert abs(sum(pose['lengths_mm'])/2-mean)<1e-9
                assert -1<=pose['command']['difference_mm']<=1
                assert pose['speeds_mm_s'][0]==pose['speeds_mm_s'][1]
            previous=pose['lengths_mm']
    linked=random_plan(profile,'Mixed',bounds,[1,4],4,1,mean,True,[-1,1])
    assert all(r['s1']==r['s2'] for r in linked)
    assert any(r['s1']!=r['s2'] for r in rows)
    for mode,ranges in [('Difference',[[-1,1]]),('Both cables',bounds)]:
        assert len(random_plan(profile,mode,ranges,[1,2],2,1,mean))==2
    for fn in [lambda:random_plan(profile,'Mixed',bounds,speeds,3,1,mean),
        lambda:random_plan(profile,'Mixed',bounds,speeds,4,1,mean,True,[-3,3]),
        lambda:random_plan(profile,'Mixed',bounds,[1,21],4,1,mean),
        lambda:random_plan(profile,'Mixed',bounds,speeds,4,1,mean+10),
        lambda:expand_plan(rows,[140,230],rests,profile),
        lambda:expand_plan(rows,initial,rests,profile,return_speed_mm_s=21)]:rejected(fn)
    sweep=difference_sweep(profile,-40,40,mean,4,15)
    sweep_plan=expand_plan(sweep,initial,rests,profile,dwell_s=0)
    assert len(sweep_plan['poses'])==17
    assert np.allclose([p['lengths_mm'][0]-p['lengths_mm'][1] for p in sweep_plan['poses'][:-1]],np.linspace(-40,40,16))
    reverse=expand_plan(difference_sweep(profile,40,-40,mean,4,15),initial,rests,profile)
    assert np.allclose([p['lengths_mm'][0]-p['lengths_mm'][1] for p in reverse['poses'][:-1]],np.linspace(40,-40,16))
    end=plan['poses'][-1]
    assert end['lengths_mm']==initial and end['lengths_mm']!=rests
    assert end['command']==dict(action='independent',lengths_mm=initial,speeds_mm_s=[2.,2.])
    assert len(plan['poses'])==9 and (profile,rows)==before
    total=sum(p['travel_s']+p['settle_s']+p['dwell_s']+1.5 for p in plan['poses'])
    assert plan['estimated_seconds']==total and plan['reserved_seconds']==total*1.3+30
    # Exactly 500 includes the return; adding one data pose must fail preflight.
    caprows=[dict(rows[0],steps=1),*[dict(rows[0],steps=100) for _ in range(4)],dict(rows[0],steps=98)]
    assert len(expand_plan(caprows,initial,rests,profile)['poses'])==500
    rejected(lambda:expand_plan([*caprows,rows[0]],initial,rests,profile))
    print('PASS mixed modes, single-axis isolation, seed/ranges/speeds, bidirectional sweep, return target and budgets',flush=True)

    now=[0.];sent=[];captured=[];states=[]
    remote=dict(phase='HOLDING',pending=False,axes=[dict(enabled=True,rest_count=0,state='HOLDING') for _ in (0,1)])
    def item(t,lengths):
        return dict(displayed_age_s=0.,packet=dict(session='test',frame_index=int(t*10),t_s=t,state='STREAMING',
            motor_active=True,motor_age_s=[0.,0.],rest_lengths_mm=rests,
            motor=[dict(axis=a,t_s=float(ti),state='HOLDING',length_mm=lengths[a],target_mm=lengths[a])
                   for a in (0,1) for ti in np.arange(t-5,t+.31,.2)]))
    def send(action,**values):sent.append(dict(action=action,**values));return len(sent)
    runner=CampaignRunner(plan,send,lambda *v:captured.append(v),lambda s,m:states.append(s),clock=lambda:now[0])
    runner.start();current=initial
    for index,pose in enumerate(plan['poses']):
        runner.tick(remote,0,item(now[0],current));assert runner.state=='ack'
        runner.event(dict(type='accepted',id=9999));assert runner.state=='ack'
        runner.event(dict(type='accepted',id=runner.command_id));now[0]+=3
        if pose.get('role')=='return':
            assert 'complete' not in states
            runner.tick(remote,0,item(now[0],current));assert runner.state=='moving'
        runner.tick(remote,0,item(now[0],pose['lengths_mm']));assert runner.state=='saving'
        assert 'complete' not in states
        runner.captured(index);current=pose['lengths_mm'];now[0]+=.1
    assert runner.state=='complete' and len(sent)==9 and len(captured)==9
    assert sent[-1]==end['command']
    # STOP, feedback loss, write failure and timeout never trigger a return or retry.
    for reason in ('stop','stale','closed','rejected','write','ack_timeout','return_stop'):
        now[0]=0.;sent.clear()
        r=CampaignRunner(plan,send,lambda *v:None,clock=lambda:now[0]);r.start()
        if reason=='return_stop':r.index=len(plan['poses'])-1
        r.tick(remote,0,item(0,initial));r.event(dict(type='accepted',id=r.command_id))
        if reason in ('stop','return_stop'):r.abort('STOP')
        elif reason=='stale':r.tick(remote,2,item(0,initial))
        elif reason=='closed':r.event(dict(type='closing',message='session timeout'))
        elif reason=='write':
            now[0]=3;r.tick(remote,0,item(3,plan['poses'][0]['lengths_mm']));r.captured(0,'disk full')
        else:
            r.state='ack'
            if reason=='rejected':r.event(dict(type='rejected',id=r.command_id,message='injected'))
            else:now[0]=4;r.tick(remote,0,item(4,initial))
        assert r.state=='aborted' and sent[-1]['action']=='hold' and len(sent)==2
        r.tick(remote,0,item(10,initial));r.captured(r.index);assert len(sent)==2
    print('PASS return acknowledgement, settling, capture before completion, STOP/fault/timeout abort without return/retry',flush=True)

    model=SpiRobModel(cfg['model']);parameters=fit['candidate']['parameters']
    result=simulate(plan,cfg['model'],parameters,model.d2,{})
    assert len(result['summary']['poses'])==9 and result['summary']['poses'][-1]['command_mm']==initial
    assert result['summary']['poses'][-1]['role']=='return'
    assert np.allclose(result['frames'][-1]['lengths_mm'],initial)
    assert np.isfinite(np.array([f['curve_mm'] for f in result['frames']])).all()
    context=review_context(rows,0,profile,cfg['model'],parameters,model.d2,{},2.)
    review=dict(fingerprint=digest(context),context=context,plan=plan,summary=result['summary'])
    assert require_review(review,context,initial)==plan
    changed=copy.deepcopy(context);changed['return_to_start']['speed_mm_s']=3.
    rejected(lambda:require_review(review,changed,initial))
    changed=copy.deepcopy(context);changed['settle_s']=3.
    rejected(lambda:require_review(review,changed,initial))
    rejected(lambda:require_review(review,context,[initial[0]+1,initial[1]]))
    # Storage records the exact return and image, including write-before-index recovery.
    buf=io.BytesIO();Image.new('RGB',(16,16),(120,150,180)).save(buf,format='JPEG')
    def saved_item(t,lengths):
        value=item(t,lengths);value.update(parameters=parameters,d2=model.d2,setup={})
        value['packet']['image_b64']=base64.b64encode(buf.getvalue()).decode();return value
    with tempfile.TemporaryDirectory() as td:
        store=CampaignStore.create(td,plan,rows,saved_item(0,initial),cfg,fit,'manual')
        for i,pose in enumerate(plan['poses']):store.capture(i,saved_item(10+i*10,pose['lengths_mm']),pose)
        assert store.doc['poses'][-1]['excluded'] and not any(p['excluded'] for p in store.doc['poses'][:-1])
        assert store.doc['image_bytes']==len(plan['poses'])*len(buf.getvalue())
        store.doc['poses'].pop();store.doc['image_bytes']-=len(buf.getvalue());store.save()
        recovered=CampaignStore(store.root);assert recovered.doc['poses'][-1]['excluded']
        assert recovered.doc['poses'][-1]['command']==end
        recovered.exclude(len(plan['poses'])-1,False);assert not recovered.doc['poses'][-1]['excluded']
    print('PASS dynamic MuJoCo mixed preview and return, approval invalidation, JPEG retention/recovery and optional return inclusion',flush=True)

    from interface import Window,QApplication
    from campaign_window import CampaignWindow
    from campaign_preview_window import RandomCampaign,DifferenceSweep,CampaignPreview
    app=QApplication.instance() or QApplication([])
    with patch('interface.QTimer.singleShot'):host=Window(cfg,fit,None,False)
    host.timer.stop();host.show();app.processEvents()
    window=CampaignWindow(host);host.campaign_window=window;window.timer.stop()
    window.set_rows(rows);window.dwell.setValue(0);window.return_speed.setValue(2)
    draft,_=window.build_plan();assert draft['poses'][-1]['role']=='return'
    c=window.review_context();window.approve_preview(dict(fingerprint=digest(c),context=c,plan=draft,summary=result['summary']))
    assert window.approved_review
    window.return_speed.setValue(3);assert window.approved_review is None
    random=RandomCampaign(window,profile);assert random.mode.currentText()=='Mixed' and random.count.minimum()==4
    assert random.mean.isEnabled() and random.bounds[0][0].isEnabled()
    random.generate();assert random.rows is not None and set(r['mode'] for r in random.rows)==set(MODES)
    sweep_dialog=DifferenceSweep(window,profile);sweep_dialog.generate();assert sweep_dialog.rows[1]['steps']==15
    out=Path(os.environ.get('SPIROB_TEST_OUTPUT','/tmp/spirob-v091-verification'));out.mkdir(parents=True,exist_ok=True)
    window.resize(960,620);window.show();app.processEvents()
    for widget in (window.table,window.return_speed,window.start_button,window.preflight_text):
        at=widget.mapTo(window,widget.rect().topLeft())
        assert 0<=at.x() and 0<=at.y() and at.x()+widget.width()<=window.width() and at.y()+widget.height()<=window.height()
    window.grab().save(str(out/'campaign_plan.png'))
    random.show();app.processEvents();random.grab().save(str(out/'mixed_generator.png'));random.close()
    with patch('campaign_preview_window.simulate',return_value=result):
        preview=CampaignPreview(window,plan,cfg['model'],context,lambda _:None)
        preview.worker.join(3);preview.poll();preview.show();app.processEvents();preview.slider.setValue(preview.slider.maximum())
        assert preview.pose_ax.get_title().startswith('Return to start')
        preview.grab().save(str(out/'return_preview.png'));preview.close()
    window.close();host.hide()
    (out/'verification.json').write_text(json.dumps(dict(version='0.9.1',status='PASS',
        tests='40 mixed seeds; constraints; stepped sweeps; return timing, capture, STOP and faults; storage; approval; dynamic MuJoCo; offscreen GUI',
        hardware_tested=False,simulation=result['summary']),indent=2)+'\n')
    print('PASS mixed/sweep dialogs, adjustable return speed, approval invalidation and 960×620 layout',flush=True)


if __name__=='__main__':main()

"""Frozen campaign measurements; shared k2 is calculated before per-pose minima."""
import copy
import csv
import threading
from pathlib import Path
import numpy as np
from analysis import Comparator, inputs_at, point_cost
from spirob_analysis.fitting import predict
from spirob_analysis.model import SpiRobModel
from spirob_analysis.measurement import measure
from campaign_storage import atomic, encoded


class CampaignCancelled(Exception):pass


def check_cancel(cancel):
    if cancel is not None and cancel.is_set():raise CampaignCancelled('Analysis stopped; saved measurements remain available.')


def auto_measure(store,progress=None,cancel=None):
    """Process bounded saved frames only. Never block acquisition or move hardware."""
    store.verify_model();engine=Comparator(store.config(),store.doc['fit'])
    from joint_tracking import ManualJointTracker
    profile=engine.profile
    tracker=ManualJointTracker(profile,store.config()) if profile else None
    for i,pose in enumerate(store.doc['poses']):
        check_cancel(cancel)
        if pose['observation'] is not None or pose['excluded']:continue
        try:
            item=store.load_item(i);packet=item['packet']
            if tracker:
                observed=tracker.update(item['rgb'],(packet['session'],packet['frame_index']))
            else:_,observed=measure(item['rgb'],store.config())
            lengths,_=inputs_at(packet,packet['t_s'],store.config()['live']['max_input_gap_s'])
            obs=engine.observation(observed,lengths,packet)
            obs.update(measurement_source='automatic saved image; encoder hold plus dwell',stationary_confirmed=False)
            store.set_observation(i,obs,'Automatic — inspect image if fit is poor')
        except (ValueError,RuntimeError,OSError) as exc:
            pose.update(status='Needs points',reason=str(exc));store.save()
            if profile:tracker=ManualJointTracker(profile,store.config())
        if progress:progress(i+1,len(store.doc['poses']),'Measuring retained images')


def fit_campaign(store,lower=.01,upper=4.,count=31,per_pose=False,progress=None,cancel=None):
    if not np.isfinite([lower,upper,count]).all() or not .001<=lower<upper<=10 or int(count)!=count or not 3<=count<=101:
        raise ValueError('Use 0.001 ≤ k₂ minimum < maximum ≤ 10 and 3..101 samples.')
    store.verify_model();doc=copy.deepcopy(store.doc)
    selected=[p for p in doc['poses'] if p['observation'] is not None and not p['excluded']]
    if not selected:raise ValueError('No usable physical measurements yet. Open Poses and assign joints.')
    fixed=doc['fit']['candidate']['parameters'];model=SpiRobModel(store.config()['model'])
    grid=np.geomspace(lower,upper,int(count))
    if lower<=fixed['k2']<=upper:grid=np.unique(np.r_[grid,fixed['k2']])
    costs=[];errors=[];total=len(grid)*len(selected)
    for i,pose in enumerate(selected):
        row=[];why=[]
        for j,k in enumerate(grid):
            check_cancel(cancel)
            try:
                points,_,_,coverage=predict(model,pose['observation'],dict(fixed,k2=float(k)))
                if np.max(coverage)>.01:raise ValueError('Observation exceeds model length')
                cost=point_cost(pose['observation']['points_px'],points)
                if not np.isfinite(cost):raise ValueError('Nonfinite fit')
                row.append(cost);why.append(None)
            except (ValueError,RuntimeError,FloatingPointError) as exc:row.append(None);why.append(str(exc))
            if progress:progress(i*len(grid)+j+1,total,'Shared k₂ sweep')
        costs.append(row);errors.append(why)
    # Every selected pose contributes at every eligible shared k. Never drop
    # a difficult pose for only part of the grid: that would bias the optimum.
    shared=[float(np.mean([row[j] for row in costs])) if all(row[j] is not None for row in costs) else None for j in range(len(grid))]
    valid=[j for j,v in enumerate(shared) if v is not None]
    if not valid:
        atomic(store.root/'fit_attempt.json',encoded(dict(k2=grid.tolist(),poses=[p['number'] for p in selected],cost_px2=costs,errors=errors)))
        raise ValueError('No grid value is valid for every included pose. Per-pose failures are saved in fit_attempt.json; review points or change the k₂ interval.')
    best=min(valid,key=lambda j:shared[j])
    records=[]
    for pose,row,why in zip(selected,costs,errors):
        item=store.load_item(pose['number']-1);packet=item['packet'];lengths,_=inputs_at(packet,packet['t_s'],store.config()['live']['max_input_gap_s'])
        obs=pose['observation'];xy=np.array(obs['points_px'])
        # A geometric deflection independent of k: the most distal measured
        # landmark's lateral offset from the calibrated neutral axis.
        tail=np.asarray(obs['tail_distance_px']);tip=int(np.argmin(tail))
        rotation=fixed['rotation_rad'];delta=xy[tip]-[fixed['translation_x_px'],fixed['translation_y_px']]
        lateral=float((np.cos(rotation)*delta[0]+np.sin(rotation)*delta[1])/fixed['pixels_per_m']*1000)
        records.append(dict(number=pose['number'],label=pose['command']['label'],mode=pose['command']['mode'],
            frame_sha256=pose['frame_sha256'],annotation_revision=pose['annotation_revision'],measurement_source=pose['reason'],
            observation=obs,lengths_mm=lengths.tolist(),mean_mm=float(np.mean(lengths)),difference_mm=float(lengths[0]-lengths[1]),
            measured_lateral_mm=lateral,distal_landmark=(obs.get('landmark_ids') or ['sample']*len(xy))[tip],
            cost_px2=row,errors=why,shared_cost_px2=row[best],shared_rmse_px=float(np.sqrt(row[best]))))
    missing=[p['number'] for p in doc['poses'] if p['observation'] is None and not p['excluded']]
    return_numbers=[p['number'] for p in doc['plan']['poses'] if p.get('role')=='return']
    included_returns=[p['number'] for p in doc['poses'] if p['number'] in return_numbers and not p['excluded']]
    expected_fit=len(doc['plan']['poses'])-len(return_numbers)+len(included_returns)
    result=dict(version='0.9.1',kind='campaign_fit',campaign=store.root.name,k2=grid.tolist(),shared_cost_px2=shared,
        shared_minimum=dict(k2=float(grid[best]),cost_px2=shared[best],rmse_px=float(np.sqrt(shared[best])),
                            index=best,at_boundary=best in (0,len(grid)-1)),poses=records,
        expected_poses=len(doc['plan']['poses']),captured_poses=len(doc['poses']),included_poses=len(selected),missing_points=missing,
        expected_fit_poses=expected_fit,return_poses=return_numbers,
        excluded_poses=[p['number'] for p in doc['poses'] if p['excluded']],
        complete=len(doc['poses'])==len(doc['plan']['poses']) and not missing and
                 not any(p['excluded'] and p['number'] not in return_numbers for p in doc['poses']),
        fixed_parameters=fixed,config=doc['config'],model_hashes=doc['model_hashes'],per_pose=False,
        meaning='Equal weight per pose; each J is mean squared Euclidean pixel error per included point. Lowest sampled k₂, '
                'not a guaranteed global optimum. All alignment, scale, offsets, r and b stay fixed. '
                'No pass/fail threshold for a shared k₂ is assumed. Inspect residuals and repeated/reverse poses. '
                'Per-pose improvements use extra fitted parameters and are not independent validation of variable stiffness.')
    # Commit the shared baseline first, even if later per-pose review is cancelled.
    check_cancel(cancel);save_result(store,result)
    if per_pose:result=add_per_pose(store,result)
    return result


def add_per_pose(store,result=None):
    result=copy.deepcopy(result or store.doc.get('analysis'))
    if result is None:raise ValueError('Calculate the shared baseline first.')
    for row in result['poses']:
        current=store.doc['poses'][row['number']-1]
        if current['annotation_revision']!=row['annotation_revision'] or current['excluded']:raise ValueError('Measurements changed. Recalculate the shared baseline.')
        valid=[i for i,c in enumerate(row['cost_px2']) if c is not None]
        j=min(valid,key=lambda i:row['cost_px2'][i]);best=row['cost_px2'][j]
        row['minimum']=dict(k2=result['k2'][j],cost_px2=best,rmse_px=float(np.sqrt(best)),index=j,at_boundary=j in (0,len(result['k2'])-1))
        row['improvement_px2']=row['shared_cost_px2']-best
    result['per_pose']=True;save_result(store,result);return result


def save_result(store,result):
    atomic(store.root/'fit_results.json',encoded(result))
    store.doc['analysis']=result;store.save()
    with (store.root/'fit_results.csv').open('w',newline='') as f:
        fields=['pose','label','L1_mm','L2_mm','mean_mm','difference_mm','distal_landmark','lateral_mm','shared_k2','shared_J_px2','shared_RMSE_px','pose_k2','pose_J_px2','boundary','measurement']
        writer=csv.writer(f);writer.writerow(fields)
        for p in result['poses']:
            best=p.get('minimum',{})
            writer.writerow([p['number'],p['label'],*p['lengths_mm'],p['mean_mm'],p['difference_mm'],p['distal_landmark'],p['measured_lateral_mm'],
                result['shared_minimum']['k2'],p['shared_cost_px2'],p['shared_rmse_px'],best.get('k2'),best.get('cost_px2'),best.get('at_boundary'),p['measurement_source']])

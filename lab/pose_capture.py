"""Export a single frozen observation and its own stiffness sweep together."""
import copy
import csv
import hashlib
import base64
import json
from pathlib import Path
import numpy as np
from PIL import Image
from matplotlib.figure import Figure


def export_pose(destination, item, scan):
    packet=item['packet'];result=item['result']
    digest=hashlib.sha256(base64.b64decode(packet['image_b64'])).hexdigest()
    if (scan['session']!=packet['session'] or scan['frame_index']!=packet['frame_index'] or
        scan['frame_sha256']!=digest or scan['observation']!=result['observation']):
        raise ValueError('Pose and sweep must come from the same frozen observation.')
    path=Path(destination).with_suffix('.png')
    figure=Figure(figsize=(14,8),layout='constrained')
    ax,graph=figure.subplots(1,2,gridspec_kw={'width_ratios':[1.5,1]})
    ax.imshow(item['rgb'])
    physical=np.asarray(item['observed']['curve_px'])
    virtual=np.asarray([result['base_px']]+result['predicted_curve_px'])
    ax.plot(*physical.T,color='#00a776',lw=1.8,label='Manual physical points (joined)' if item.get('manual_frozen') else 'Physical centreline')
    ax.plot(*virtual.T,color='#ed493e',lw=1.8,label='MuJoCo at capture k₂')
    rows=result.get('landmark_rows',[])
    for row in rows:
        if not row['included']:continue
        p,q=row['physical_px'],row['model_px']
        if p is None or q is None:continue
        ax.plot([p[0],q[0]],[p[1],q[1]],color='#e7a900',lw=.7)
        ax.scatter(*p,s=13,color='#00a776',zorder=4)
        ax.scatter(*q,s=25,facecolors='none',edgecolors='#ed493e',zorder=4)
        label=row['id'].replace('j_0','J')
        ax.annotate(label,p,xytext=(4,-7),textcoords='offset points',fontsize=6,
                    bbox=dict(facecolor='white',alpha=.65,pad=.1,edgecolor='none'))
    # Show the actual scored point connectors as well when J uses curve samples.
    if not result['observation'].get('landmark_ids'):
        for p,q in zip(result['observation']['points_px'],result['predicted_points_px']):
            ax.plot([p[0],q[0]],[p[1],q[1]],color='#e7a900',lw=.6,alpha=.7)
    h,w=item['rgb'].shape[:2]
    points=np.vstack([physical,virtual]);lo=points.min(0)-45;hi=points.max(0)+45
    ax.set(xlim=(max(0,lo[0]),min(w,hi[0])),ylim=(min(h,hi[1]),max(0,lo[1])),
        xlabel='Native image x [px]',ylabel='Native image y [px]',
        title=f"Frozen pose · J = {result['cost_px2']:.5g} px²/point\nk₂ at capture = {result['k2']:.7g} N m/rad")
    ax.legend(loc='best',fontsize=8)
    costs=[np.nan if v is None else v for v in scan['cost_px2']]
    graph.semilogx(scan['k2'],costs,'o-',ms=3,color='#7451aa')
    best=scan['minimum'];graph.plot(best['k2'],best['cost_px2'],'*',ms=14,color='#d25d00')
    graph.set(title=f"Same-pose equilibrium sweep\nLowest sampled J = {best['cost_px2']:.5g} at k₂ = {best['k2']:.7g}",
              xlabel='k₂ [N m/rad]',ylabel='J [px² / point]')
    graph.grid(alpha=.25)
    profile=result['setup'].get('profile') or {}
    score='manual frozen points' if item.get('manual_frozen') else 'calibrated joints' if result['observation'].get('landmark_ids') else 'curve samples'
    warning='Boundary minimum: extend the interval.\n' if best['at_boundary'] else ''
    graph.text(.02,.98,warning+f"J uses {score}; n = {len(result['observation']['points_px'])}\n"
        +f"Cable lengths: {result['lengths_mm'][0]:.3f}, {result['lengths_mm'][1]:.3f} mm\n"
        +f"Scale: {result['setup']['pixels_per_m']:.3f} px/m\n"
        +f"Stiffness profile r = {scan['fixed_parameters']['stiffness_ratio']:.7g}, b = {scan['fixed_parameters']['stiffness_exponent']:.7g}\n"
        +('Fixed adjacent joint distances\n' if not item.get('manual_frozen') and profile.get('kind')=='rigid_links' else '')
        +'Alignment and cable offsets held fixed.\nSingle-pose sweep; damping is not fitted.',
        transform=graph.transAxes,va='top',fontsize=8,
        bbox=dict(facecolor='white',alpha=.85,edgecolor='#dddddd'))
    figure.suptitle(('RECORDED EXAMPLE · ' if item.get('recorded_demo') else '')+
        f"{packet['session']} · frame {packet['frame_index']} · t = {packet['t_s']:.3f} s",fontsize=11)
    figure.savefig(path,dpi=180)
    Image.fromarray(item['rgb']).save(path.with_name(path.stem+'_frame.png'))
    with path.with_name(path.stem+'_joints.csv').open('w',newline='') as stream:
        writer=csv.writer(stream);writer.writerow(['point','included','physical_x_px','physical_y_px','model_x_px','model_y_px','distance_px','source'])
        for row in rows:writer.writerow([row['id'],row['included'],*(row['physical_px'] or ['','']),*(row['model_px'] or ['','']),row['distance_px'],row['source']])
    with path.with_name(path.stem+'_sweep.csv').open('w',newline='') as stream:
        writer=csv.writer(stream);writer.writerow(['k2_Nm_rad','J_px2_per_point','error'])
        writer.writerows(zip(scan['k2'],scan['cost_px2'],scan['errors']))
    metadata=dict(version='0.8.0',session=packet['session'],frame_index=packet['frame_index'],
        frame_sha256=digest,parameters=copy.deepcopy(item['parameters']),result=copy.deepcopy(result),
        sweep=copy.deepcopy(scan),recorded_demo=bool(item.get('recorded_demo')),
        meaning='All overlays, J, and sweep use this one frozen image and its captured encoder inputs. Green joints record their measurement source: automatic estimate, exact manual placement, or image tracking.')
    path.with_suffix('.json').write_text(json.dumps(metadata,indent=2,allow_nan=False)+'\n')
    return path

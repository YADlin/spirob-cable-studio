"""Frozen manual measurements: no segmentation, optical flow or saved pixels.

The captured encoder inputs determine the equilibrium model. The user supplies
the matching physical points and confirms that the photographed pose was held.
"""
import copy
import numpy as np
from analysis import Comparator, inputs_at, point_cost
from landmarks import correspondences


def prepare_capture(item, config, fit, allow_missing_model=False):
    """Freeze one camera/encoder packet, even if automatic vision never worked."""
    result = copy.deepcopy(item)
    result.update(manual_frozen=True, observed=None, capture_error=None)
    settings = copy.deepcopy(config)
    settings['comparison_setup'] = copy.deepcopy(item.get('setup') or (item.get('result') or {}).get('setup') or config.get('comparison_setup'))
    candidate = copy.deepcopy(fit)
    candidate['candidate']['parameters'] = copy.deepcopy(item['parameters'])
    engine = Comparator(settings, candidate)
    engine.set_parameters(item['parameters']['k2'],item['d2'])
    packet = item['packet']
    inputs_ready=False
    try:
        if item.get('setup_preview'):
            raise ValueError('Setup preview has no encoder inputs. Connect, enable and declare rest, then capture again.')
        if not item.get('recorded_demo'):
            if item['displayed_age_s'] > config['live']['max_age_s']:
                raise ValueError('The camera frame was stale at capture. Wait for fresh video and capture again.')
            if (not packet.get('motor_active') or packet.get('state') not in ('STREAMING','RECORDING') or
                    any(v is None or not -.1 <= v <= config['live']['max_age_s'] for v in packet['motor_age_s'])):
                raise ValueError('Fresh, enabled and referenced encoder feedback is required. Declare rest and capture again.')
        lengths,targets=inputs_at(packet, packet['t_s'], config['live']['max_input_gap_s'])
        if not item.get('recorded_demo') and not engine.encoders_settled(packet):
            raise ValueError('The captured cables were moving or had not held for two seconds. Hold the pose and capture again.')
        inputs_ready=True
        frozen = engine.compare_still(packet, None)
        frozen.update(manual_capture_ready=True, encoders_settled=True,
            message='Manual capture ready: place physical points and confirm that the photographed pose was stationary.',
            model_source='equilibrium at captured measured cable lengths')
        result['result'] = frozen
        result['message'] = frozen['message']
    except (ValueError, RuntimeError) as exc:
        result['capture_error'] = str(exc)
        result['result'] = engine.compare_setup(packet, None)
        result['result'].update(manual_capture_ready=False, model_source='reference only', message=str(exc))
        result['message'] = str(exc)
        if inputs_ready and allow_missing_model:
            # Batch annotation must not depend on the current trial k having a
            # valid free equilibrium. Keep real inputs; show no false prediction.
            result['capture_error']=None
            result['result'].update(manual_capture_ready=True,lengths_mm=lengths.tolist(),targets_mm=targets,
                model_source='equilibrium unavailable',predicted_curve_px=[],setup_preview=False)
            result['result']['landmarks']['model_px']=None
            result['message']='Manual placement is available. Current k₂ has no valid equilibrium: '+str(exc)+'. The later sweep will test each k₂ independently.'
            result['result']['message']=result['message']
    result['landmarks'] = result['result']['landmarks']
    result['setup'] = engine.setup()
    # Captured age is immutable. Time spent placing points does not make this
    # historical, paired image/encoder observation stale after the fact.
    return result


def complete_capture(item, rows, mode, stationary_confirmed, allow_missing_model=False):
    """Use exactly the selected coordinates and named model anchors for J."""
    if item.get('capture_error'):
        raise ValueError(item['capture_error'])
    if not item.get('result', {}).get('manual_capture_ready'):
        raise ValueError('Capture a held pose with valid measured cable inputs first.')
    if not stationary_confirmed:
        raise ValueError('Confirm that the physical SpiRob was stationary in this captured frame.')
    selected = [r for r in rows if r['included'] and r['physical_px'] is not None]
    if len(selected) < 4:
        raise ValueError('Place and include at least four matching physical joints or element centres.')
    names = [r['id'] for r in selected]
    xy = np.asarray([r['physical_px'] for r in selected], float)
    h, w = item['rgb'].shape[:2]
    if len(set(names)) != len(names) or not np.isfinite(xy).all() or np.any(xy < 0) or np.any(xy >= [w,h]):
        raise ValueError('Each named point must have a finite position inside the image.')
    distances = np.linalg.norm(xy[:,None,:]-xy[None,:,:],axis=2)
    np.fill_diagonal(distances,np.inf)
    if np.min(distances) < 1.:
        raise ValueError('Two named points coincide. Check the physical joint identities.')
    frozen = copy.deepcopy(item)
    result = frozen['result']
    layout = result['landmarks']
    pairs = correspondences(None,layout,mode=mode,manual=dict(zip(names,xy.tolist())),
        excluded=[r['id'] for r in rows if r['id'] not in names])
    for row in pairs:row['source']='manual frozen' if row['physical_px'] is not None else 'not placed'
    scored = [r for r in pairs if r['included'] and r['physical_px'] is not None]
    model_available=all(r['model_px'] is not None for r in scored)
    if len(scored) != len(names) or (not model_available and not allow_missing_model):
        raise ValueError('A placed point does not have a corresponding MuJoCo anchor.')
    # Keep manual points tied to THIS image, never to a running tracker/profile.
    tails = np.asarray(layout['tail_m'])*layout['pixels_per_m']
    ids = layout['ids']
    if mode == 'centres':
        tails = (tails[:-1]+tails[1:])/2
        ids = ['link_'+n.split('_')[-1] for n in ids[:-1]]
    tail_map = dict(zip(ids,tails.tolist()))
    setup = result['setup']
    displacement = (np.array(result['lengths_mm'])-item['packet']['rest_lengths_mm'])[setup['mapping']]*setup['signs']
    observed = dict(points_px=xy.tolist(), curve_px=xy.tolist(), landmark_ids=names, landmark_mode=mode,
        tail_distance_px=[tail_map[n] for n in names], image_shape=[h,w],
        visible_arc_px=float(np.linalg.norm(np.diff(xy,axis=0),axis=1).sum()),
        joint_points_px=dict(zip(names,xy.tolist())), measurement_source='manual frozen coordinates',
        stationary_confirmed=True, displacement_mm=displacement.tolist())
    predicted_map = {r['id']:r['model_px'] for r in scored}
    predicted = [predicted_map[n] for n in names]
    cost = point_cost(observed['points_px'],predicted) if model_available else None
    result.update(observation=copy.deepcopy(observed),landmark_rows=pairs,predicted_points_px=predicted,
        cost_px2=cost,rmse_px=None if cost is None else float(np.sqrt(cost)),motion_error_px2=cost,settled=True,
        measurement_mode='manual_frozen',settled_source='held encoder inputs + user-confirmed stationary photograph',
        message='Manual frozen pose: exact selected points; no centreline estimation or tracking.')
    frozen.update(observed=observed,message=result['message'])
    return frozen

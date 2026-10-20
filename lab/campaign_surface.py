"""Campaign-wide (k2, stiffness ratio) surface: shared J over every included pose.

kᵢ = k₂ · r^(i−2) for i ≥ 2; joint 1 unchanged. Alignment, scale, offsets and
damping stay fixed exactly as in the shared k₂ fit. Optionally the cable
position-servo gain kp (the model's only cable compliance) can be overridden,
because a static shape constrains k₂ only relative to kp and gravity.

Writes fit_surface.json / .csv / .png next to campaign.json. It never touches
fit_results.json, the live model, motors or the camera.
"""
import copy
import csv
import numpy as np
from analysis import point_cost
from spirob_analysis.fitting import predict
from spirob_analysis.model import SpiRobModel
from campaign_storage import atomic, encoded

FORMULA = 'kᵢ = k₂ · r^(i−2), i ≥ 2; joint 1 unchanged'


class SurfaceCancelled(Exception):
    pass


def set_cable_kp(model, kp):
    """Position actuator: force = kp·(ctrl − length), tension-only. Keep that form."""
    kp = float(kp)
    if not np.isfinite(kp) or not 10 <= kp <= 1e7:
        raise ValueError('Cable kp must be between 10 and 10,000,000 N/m.')
    model.m.actuator_gainprm[:, 0] = kp
    model.m.actuator_biasprm[:, 1] = -kp
    model.kp[:] = kp


def fit_surface(store, k_lower=.03, k_upper=3., k_count=21, r_lower=.90, r_upper=1.10, r_count=21,
                cable_kp=None, progress=None, cancel=None):
    if (not np.isfinite([k_lower, k_upper, k_count, r_lower, r_upper, r_count]).all()
            or not .001 <= k_lower < k_upper <= 10 or not .5 <= r_lower < r_upper <= 1.5
            or int(k_count) != k_count or int(r_count) != r_count
            or not 3 <= k_count <= 61 or not 3 <= r_count <= 61):
        raise ValueError('Use 0.001 ≤ k₂ min < max ≤ 10, 0.5 ≤ ratio min < max ≤ 1.5 and 3..61 samples per axis.')
    store.verify_model()
    doc = copy.deepcopy(store.doc)
    selected = [p for p in doc['poses'] if p['observation'] is not None and not p['excluded']]
    if not selected:
        raise ValueError('No usable physical measurements yet. Open Poses and assign joints.')
    fixed = copy.deepcopy(doc['fit']['candidate']['parameters'])
    model = SpiRobModel(store.config()['model'])
    model_kp = float(model.kp[0])
    if cable_kp is not None:
        set_cable_kp(model, cable_kp)
    fixed.pop('stiffness_exponent', None)
    ks = np.geomspace(k_lower, k_upper, int(k_count))
    rs = np.linspace(r_lower, r_upper, int(r_count))
    # Always include the campaign's own (k₂, r) so the old baseline is a grid point.
    if k_lower <= fixed['k2'] <= k_upper: ks = np.unique(np.r_[ks, fixed['k2']])
    model_ratio = float(fixed.get('stiffness_ratio', model.rk))
    if r_lower <= model_ratio <= r_upper: rs = np.unique(np.r_[rs, model_ratio])
    # cost[pose][row=r][col=k]
    cost = np.full((len(selected), len(rs), len(ks)), np.nan)
    first_error = {}
    total = cost.size
    n = 0
    for a, r in enumerate(rs):
        for b, k in enumerate(ks):
            for i, pose in enumerate(selected):
                if cancel is not None and cancel.is_set():
                    raise SurfaceCancelled('Surface calculation stopped; nothing was saved.')
                parameters = dict(fixed, k2=float(k), stiffness_ratio=float(r), stiffness_exponent=1.)
                try:
                    points, _, _, coverage = predict(model, pose['observation'], parameters)
                    if np.max(coverage) > .01: raise ValueError('Observation exceeds model length')
                    c = point_cost(pose['observation']['points_px'], points)
                    if not np.isfinite(c): raise ValueError('Non-finite equilibrium cost')
                    cost[i, a, b] = c
                except (ValueError, RuntimeError, FloatingPointError) as exc:
                    first_error.setdefault(pose['number'], str(exc))
                n += 1
                if progress and (n % 25 == 0 or n == total):
                    progress(n, total, 'Shared (k₂, ratio) surface')
    # Same rule as the shared k₂ fit: a grid point counts only if EVERY pose is valid there.
    valid = ~np.isnan(cost).any(axis=0)
    shared = cost.mean(axis=0)  # NaN wherever any pose is invalid
    if not valid.any():
        raise ValueError('No grid point is valid for every included pose (contact or non-convergence). '
                         'Narrow the ranges or review points. First errors: ' +
                         '; '.join(f'pose {k}: {v}' for k, v in list(first_error.items())[:3]))
    a, b = np.unravel_index(np.nanargmin(shared), shared.shape)
    boundary = [name for name, hit in (('k2', b in (0, len(ks) - 1)), ('ratio', a in (0, len(rs) - 1))) if hit]
    # Best k₂ for each ratio (a "valley" profile) and the old exponential baseline.
    valley = []
    for row in range(len(rs)):
        if valid[row].any():
            col = int(np.nanargmin(shared[row]))
            valley.append(dict(ratio=float(rs[row]), k2=float(ks[col]), cost_px2=float(shared[row, col]),
                               rmse_px=float(np.sqrt(shared[row, col]))))
    baseline = None
    if model_ratio in rs and fixed['k2'] in ks:
        ra, kb = int(np.where(rs == model_ratio)[0][0]), int(np.where(ks == fixed['k2'])[0][0])
        if valid[ra, kb]:
            baseline = dict(ratio=model_ratio, k2=float(fixed['k2']), cost_px2=float(shared[ra, kb]),
                            rmse_px=float(np.sqrt(shared[ra, kb])))
    poses = []
    for i, pose in enumerate(selected):
        pc = cost[i]
        if np.isnan(pc).all():
            best = None
        else:
            pa, pb = np.unravel_index(np.nanargmin(pc), pc.shape)
            best = dict(ratio=float(rs[pa]), k2=float(ks[pb]), cost_px2=float(pc[pa, pb]))
        poses.append(dict(number=pose['number'], label=pose['command']['label'],
                          annotation_revision=pose['annotation_revision'],
                          cost_at_shared_minimum_px2=None if np.isnan(pc[a, b]) else float(pc[a, b]),
                          rmse_at_shared_minimum_px=None if np.isnan(pc[a, b]) else float(np.sqrt(pc[a, b])),
                          own_minimum=best, first_error=first_error.get(pose['number'])))
    best_cost = float(shared[a, b])
    to_list = lambda arr: [[None if np.isnan(v) else float(v) for v in row] for row in arr]
    result = dict(version='0.9.2', kind='campaign_stiffness_surface', campaign=store.root.name, formula=FORMULA,
        k2=ks.tolist(), ratio=rs.tolist(), shared_cost_px2=to_list(shared),
        valid_points=int(valid.sum()), total_points=int(valid.size),
        minimum=dict(k2=float(ks[b]), ratio=float(rs[a]), cost_px2=best_cost, rmse_px=float(np.sqrt(best_cost)),
                     row=int(a), column=int(b), at_boundary=bool(boundary), boundary_axes=boundary),
        baseline_exponential=baseline, valley=valley, poses=poses,
        cable_kp_N_per_m=float(model.kp[0]), model_cable_kp_N_per_m=model_kp,
        cable_kp_overridden=cable_kp is not None,
        included_poses=len(selected), excluded_poses=[p['number'] for p in doc['poses'] if p['excluded']],
        missing_points=[p['number'] for p in doc['poses'] if p['observation'] is None and not p['excluded']],
        fixed_parameters=fixed, model_hashes=doc['model_hashes'],
        requested_grid=dict(k_lower=k_lower, k_upper=k_upper, k_count=int(k_count),
                            r_lower=r_lower, r_upper=r_upper, r_count=int(r_count)),
        meaning='Equal weight per pose; J = mean squared Euclidean pixel error per point, averaged over poses. '
                'A grid point is valid only if every included pose converges without contact. Lowest sampled point, '
                'not a guaranteed global optimum. Camera alignment, scale, cable offsets and damping are fixed. '
                'k₂ is identified only relative to cable kp and gravity; report it as an effective value.')
    save_surface(store, result)
    return result


def surface_is_current(store, result):
    """True only if no pose was re-annotated, excluded or included after the sweep."""
    now = {p['number']: p['annotation_revision'] for p in store.doc['poses']
           if p['observation'] is not None and not p['excluded']}
    then = {p['number']: p['annotation_revision'] for p in result['poses']}
    return now == then


def save_surface(store, result):
    atomic(store.root / 'fit_surface.json', encoded(result))
    with (store.root / 'fit_surface.csv').open('w', newline='') as f:
        w = csv.writer(f)
        w.writerow(['k2_Nm_per_rad', 'stiffness_ratio', 'shared_J_px2', 'shared_RMSE_px', 'cable_kp_N_per_m'])
        for a, r in enumerate(result['ratio']):
            for b, k in enumerate(result['k2']):
                c = result['shared_cost_px2'][a][b]
                w.writerow([k, r, c, None if c is None else float(np.sqrt(c)), result['cable_kp_N_per_m']])

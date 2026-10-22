"""Independent joint stiffness from a campaign (no exponential law). No hardware.

Every free joint j_002 … j_021 gets its own stiffness k_i = k₂ · w_i. Joint 1 (the
clamp) is untouched. The weights are fitted to the saved physical points of every
included pose by least squares (Gauss–Newton/trust region), optionally with a
cable difference offset and a smoothness penalty on neighbouring log-stiffnesses.

Identifiability: with position-controlled cables and no in-plane gravity, scaling
every k_i by the same factor leaves the static shape unchanged unless the cable
itself is compliant. So the shape fixes the *ratios* w_i, not the absolute level;
k₂ is held at a reference value you choose (absolute k needs a known force, e.g.
the pulley-and-mass test). Per-joint log-uncertainty is reported from the
Jacobian; a joint with σ(log k) > 0.7 (factor 2) is not pinned by the data.
"""
import copy
import csv
import hashlib
from pathlib import Path
import numpy as np
from scipy.optimize import least_squares
from analysis import point_cost
from spirob_analysis.fitting import predict
from spirob_analysis.model import SpiRobModel
from campaign_storage import atomic, encoded
from campaign_surface import set_cable_kp


SLACK_MM = 50.   # lengthened cable paid out far enough to stay slack in every pose


class JointFitCancelled(Exception):
    pass


def _poses(store):
    return [p for p in store.doc['poses'] if p['observation'] is not None and not p['excluded']]


def fit_joints(store, model_path=None, cable_kp=2.5e5, k2_reference=None, smooth=0., profile='independent',
               fit_gain=True, fit_offset=True, start='exponential', max_evaluations=60,
               optimizer='cmaes', cma_evaluations=None, seed=0, slack_antagonist=True, progress=None, cancel=None):
    """profile: 'independent' (one stiffness per free joint) or 'exponential' (one ratio r).

    Transmission per cable c: effective displacement = gain_c · encoder displacement + offset_c,
    i.e. how much of the commanded cable travel actually reaches the body.

    slack_antagonist: the lengthened cable is given ample slack, so the shape is set by
    the pulling cable alone (as observed on the rig). Without it, two inextensible,
    position-controlled cables "lock" the body once both are taut: tensions jump to
    tens of newtons and the shape changes by ~100 mm per mm of cable — a cliff no
    optimiser can cross. Tension-limited/compliant cables are needed to model that regime.

    optimizer: 'least_squares' (local Gauss–Newton from the start point) or 'cmaes'
    (PACE-style global CMA-ES over the bounded box, then a least-squares polish).
    The objective has kinks (slack ↔ taut cables, rejected contact, equilibrium
    branch changes), where a purely local method can stall.
    """
    store.verify_model()
    selected = _poses(store)
    if len(selected) < 3:
        raise ValueError('Use at least three included poses with points.')
    if profile not in ('independent', 'exponential'):
        raise ValueError('profile must be independent or exponential.')
    xml = Path(model_path).resolve() if model_path else Path(store.config()['model'])
    model = SpiRobModel(xml)
    if cable_kp is not None:
        set_cable_kp(model, cable_kp)
    fixed = copy.deepcopy(store.doc['fit']['candidate']['parameters'])
    fixed.pop('stiffness_exponent', None)
    k2 = float(fixed['k2'] if k2_reference is None else k2_reference)
    if not np.isfinite(k2) or k2 <= 0:
        raise ValueError('Reference k₂ must be positive.')
    n = len(model.original_k) - 1                     # free joints j_002 … j_N
    ratio0 = float(fixed.get('stiffness_ratio', model.rk))
    if profile == 'independent':
        x_w = np.zeros(n - 1) if start == 'uniform' else (np.arange(1, n) * np.log(ratio0))
        lo_w, hi_w = np.full(n - 1, -8.), np.full(n - 1, 8.)
    else:
        x_w, lo_w, hi_w = np.array([ratio0]), np.array([.5]), np.array([1.5])
    g0, o0 = [1., 1.], None
    previous = store.root / 'joint_fit.json'
    if start == 'previous':
        if not previous.exists():
            raise ValueError('No previous joint fit in this campaign; start from exponential or uniform.')
        import json
        last = json.loads(previous.read_text())
        w_last = np.array([j['weight'] for j in last['joints']])
        x_w = np.log(w_last[1:]) if profile == 'independent' else np.array([last.get('ratio') or ratio0])
        g0 = last['transmission']['gain']; o0 = last['transmission']['offset_mm']
    nw = len(x_w)
    x0, lo, hi = [x_w], [lo_w], [hi_w]
    if fit_gain:
        x0.append(g0); lo.append([.3, .3]); hi.append([1.5, 1.5])
    if fit_offset:
        d0 = float(fixed.get('difference_offset_mm', 0.))
        x0.append(o0 if o0 is not None else [d0 / 2, -d0 / 2]); lo.append([-15., -15.]); hi.append([15., 15.])
    x0, lo, hi = np.concatenate(x0), np.concatenate(lo), np.concatenate(hi)
    obs = [p['observation'] for p in selected]
    count = [0]
    total = max_evaluations * (len(x0) + 1)

    def unpack(x):
        weights = np.exp(np.r_[0., x[:nw]]) if profile == 'independent' else x[0] ** np.arange(n)
        i = nw
        gain = np.ones(2); offset = np.zeros(2)
        if fit_gain:
            gain = np.asarray(x[i:i + 2]); i += 2
        if fit_offset:
            offset = np.asarray(x[i:i + 2])
        return weights, gain, offset

    def params(x):
        weights, _, _ = unpack(x)
        # Offsets are applied per cable below, so the shared offsets are zeroed only when fitted.
        extra = dict(difference_offset_mm=0., mean_offset_mm=0.) if fit_offset else {}
        return dict(fixed, k2=k2, joint_weights=weights, **extra)

    def effective(o, x):
        _, gain, offset = unpack(x)
        return effective_observation(o, dict(transmission=dict(gain=gain, offset_mm=offset,
                                                               slack_antagonist=slack_antagonist)))

    def residual(x, per_pose=False):
        if cancel is not None and cancel.is_set():
            raise JointFitCancelled('Joint fit stopped; nothing was saved.')
        p = params(x)
        out, costs = [], []
        for o in obs:
            try:
                pts = predict(model, effective(o, x), p)[0]
                e = pts - np.asarray(o['points_px'])
                out.append(e.ravel()); costs.append(point_cost(o['points_px'], pts))
            except (ValueError, RuntimeError):
                out.append(np.full(2 * len(o['points_px']), 150.)); costs.append(None)
        if smooth > 0 and profile == 'independent':
            curv = np.diff(np.r_[0., x[:nw]], 2)
            out.append(np.sqrt(smooth) * 100. * curv)   # 100 px per unit log-curvature at weight 1
        count[0] += 1
        if progress and count[0] % 5 == 0:
            progress(min(count[0], total), total, 'Joint / transmission fit')
        return (np.concatenate(out), costs) if per_pose else np.concatenate(out)

    r0, before = residual(x0, True)
    search = None
    if optimizer == 'cmaes':
        from cmaes import minimize as cma_minimize
        budget = int(cma_evaluations or (600 if profile == 'exponential' else 3000))
        def objective(x):
            return float(np.sum(residual(x) ** 2))
        def report(n_eval, total_eval, best):
            if progress:progress(n_eval, total_eval, f'CMA-ES search (best RMS {np.sqrt(best / max(1, len(r0) / 2)):.1f} px)')
        search = cma_minimize(objective, x0, lo, hi, sigma0=.25 if start != 'previous' else .05, max_evaluations=budget, seed=seed,
                              callback=report, cancel=cancel)
        if cancel is not None and cancel.is_set():
            raise JointFitCancelled('Joint fit stopped; nothing was saved.')
        x0 = np.clip(search['x'], lo, hi)
    elif optimizer != 'least_squares':
        raise ValueError('optimizer must be cmaes or least_squares.')
    fit = least_squares(residual, x0, bounds=(lo, hi), diff_step=1e-3, x_scale='jac', max_nfev=max_evaluations)
    r, after = residual(fit.x, True)
    m = sum(2 * len(o['points_px']) for o in obs)
    # Include the smoothness rows: with regularisation, σ describes the regularised estimate.
    J = fit.jac
    dof = max(1, m - len(fit.x))
    cov = np.linalg.pinv(J.T @ J) * float(np.sum(r[:m] ** 2)) / dof
    sd = np.sqrt(np.clip(np.diag(cov), 0, None))
    weights, gain, offset = unpack(fit.x)
    if profile == 'independent':
        sig = np.r_[0., sd[:nw]]
    else:  # σ(log w_i) = (i−2)·σ(r)/r
        sig = np.arange(n) * sd[0] / fit.x[0]
    i = nw
    gain_sd = sd[i:i + 2].tolist() if fit_gain else [0., 0.]
    i += 2 if fit_gain else 0
    offset_sd = sd[i:i + 2].tolist() if fit_offset else [0., 0.]
    names = model.names[1:]
    joints = [dict(joint=names[j], weight=float(weights[j]), k_Nm_per_rad=float(k2 * weights[j]),
                   sigma_log=float(sig[j]), determined=bool(sig[j] <= .7),
                   exponential_weight=float(ratio0 ** j)) for j in range(n)]
    valid_after = [c for c in after if c is not None]
    valid_before = [c for c in before if c is not None]
    fixed_out = {k: v for k, v in fixed.items() if k != 'joint_weights'}
    result = dict(version='0.9.4', kind='campaign_independent_joints', campaign=store.root.name,
        profile=profile, ratio=float(fit.x[0]) if profile == 'exponential' else None,
        ratio_sigma=float(sd[0]) if profile == 'exponential' else None,
        joints=joints, k2_reference=k2,
        transmission=dict(fitted_gain=bool(fit_gain), fitted_offset=bool(fit_offset),
                          gain=gain.tolist(), gain_sigma=gain_sd, offset_mm=offset.tolist(), offset_sigma_mm=offset_sd,
                          slack_antagonist=bool(slack_antagonist),
                          meaning='Per model cable: effective displacement = gain · encoder displacement + offset.'),
        difference_offset_mm=float(fixed.get('difference_offset_mm', 0.)) if not fit_offset else 0.,
        rmse_px=float(np.sqrt(np.mean(valid_after))) if valid_after else None,
        start_rmse_px=float(np.sqrt(np.mean(valid_before))) if valid_before else None,
        poses=[dict(number=p['number'], label=p['command']['label'], annotation_revision=p['annotation_revision'],
                    start_rmse_px=None if b is None else float(np.sqrt(b)),
                    rmse_px=None if a is None else float(np.sqrt(a)))
               for p, b, a in zip(selected, before, after)],
        cable_kp_N_per_m=float(model.kp[0]), smooth=float(smooth), start=start,
        evaluations=int(fit.nfev), converged=bool(fit.success), message=str(fit.message), optimizer=optimizer,
        cma=None if search is None else dict(evaluations=search['evaluations'], generations=search['generations'], best_cost=search['f']),
        model_xml=str(xml), model_xml_sha256=hashlib.sha256(xml.read_bytes()).hexdigest(),
        model_source='campaign snapshot' if model_path is None else 'external',
        fixed_parameters=fixed_out,
        meaning='Shapes fix the ratios k_i/k₂ (and the cable transmission); k₂ is held at the reference. '
                'σ(log k) > 0.7 means the data do not pin that joint (factor-2 uncertainty). '
                'Static fit: damping, speed and loading history are not used.')
    save_joint_fit(store, result)
    return result


def effective_observation(observation, result):
    t = result.get('transmission') or {}
    o = dict(observation)
    gain = np.asarray(t.get('gain', [1., 1.])); offset = np.asarray(t.get('offset_mm', [0., 0.]))
    d = np.asarray(o['displacement_mm'], float) * gain + offset
    if t.get('slack_antagonist'):
        j = int(np.argmax(d))
        d[j] = max(d[j], SLACK_MM)
    o['displacement_mm'] = d.tolist()
    return o


def fitted_parameters(result):
    p = dict(result['fixed_parameters'], k2=result['k2_reference'],
             joint_weights=[j['weight'] for j in result['joints']])
    if (result.get('transmission') or {}).get('fitted_offset'):
        p.update(difference_offset_mm=0., mean_offset_mm=0.)
    return p


def save_joint_fit(store, result):
    atomic(store.root / 'joint_fit.json', encoded(result))
    with (store.root / 'joint_fit.csv').open('w', newline='') as f:
        w = csv.writer(f)
        w.writerow(['joint', 'k_Nm_per_rad', 'k_over_k2', 'sigma_log_k', 'determined', 'exponential_k_over_k2'])
        for j in result['joints']:
            w.writerow([j['joint'], j['k_Nm_per_rad'], j['weight'], j['sigma_log'], j['determined'], j['exponential_weight']])


def export_xml(result, destination):
    """Copy of the fitted model with j_002 … j_N stiffness replaced (damping unchanged)."""
    import xml.etree.ElementTree as ET
    src = Path(result['model_xml'])
    if hashlib.sha256(src.read_bytes()).hexdigest() != result['model_xml_sha256']:
        raise ValueError('The model used for this fit has changed since. Fit again.')
    text = src.read_text()
    tree = ET.ElementTree(ET.fromstring(text))
    stiff = {j['joint']: j['k_Nm_per_rad'] for j in result['joints']}
    for joint in tree.iter('joint'):
        if joint.get('name') in stiff:
            joint.set('stiffness', repr(stiff[joint.get('name')]))
    destination = Path(destination)
    tree.write(destination, encoding='unicode')
    # Meshes are referenced relative to the XML; keep the file next to the source model.
    return destination


def render_overlays(store, result):
    """Physical points, independent-joint fit and the campaign exponential on each pose photo."""
    from matplotlib.figure import Figure
    from PIL import Image
    xml = Path(result['model_xml'])
    if hashlib.sha256(xml.read_bytes()).hexdigest() != result['model_xml_sha256']:
        raise ValueError('The model used for this fit has changed since. Fit again.')
    model = SpiRobModel(xml)
    set_cable_kp(model, result['cable_kp_N_per_m'])
    fixed = dict(result['fixed_parameters'])
    label = 'Independent joints' if result.get('profile', 'independent') == 'independent' else 'Exponential r = %.3f' % result['ratio']
    if (result.get('transmission') or {}).get('fitted_gain'):
        label += ' + transmission %.2f / %.2f' % tuple(result['transmission']['gain'])
    cases = [(label, fitted_parameters(result), '#1e88e5', '-'),
             ('Campaign exponential r = %.3f' % fixed.get('stiffness_ratio', model.rk),
              dict(fixed, stiffness_exponent=1.), '#e53935', '--')]
    by_number = {p['number']: p for p in store.doc['poses']}
    paths = []
    for row in result['poses']:
        pose = by_number.get(row['number'])
        if pose is None or pose['observation'] is None:
            continue
        obs = pose['observation']
        phys = np.asarray(obs['points_px'], float)
        fig = Figure(figsize=(12, 7), layout='constrained')
        ax = fig.subplots()
        frame = store.path(pose['path']) / 'frame.jpg'
        if frame.is_file():
            ax.imshow(Image.open(frame).convert('RGB'))
        ax.plot(*phys.T, 'o', ms=6, mfc='none', mec='#00e676', mew=1.8, label='Physical points')
        for label, p, colour, style in cases:
            try:
                pts, curve, _, _ = predict(model, effective_observation(obs, result) if style == '-' else obs, p)
            except (ValueError, RuntimeError) as exc:
                ax.plot([], [], style, color=colour, label=label + ' — failed: ' + str(exc)[:40]); continue
            err = np.sqrt(point_cost(phys, pts))
            ax.plot(*np.asarray(curve).T, style, color=colour, lw=2, label=f'{label} (RMSE {err:.1f} px)')
            ax.plot(*pts.T, 's', ms=3, color=colour)
        lo, hi = phys.min(axis=0) - 90, phys.max(axis=0) + 90
        ax.set_xlim(lo[0], hi[0]); ax.set_ylim(hi[1], lo[1])
        D = obs['displacement_mm']
        ax.set_title(f"Pose {pose['number']} · cable Δ = ({D[0]:+.2f}, {D[1]:+.2f}) mm · cable kp {result['cable_kp_N_per_m']:.6g} N/m")
        ax.legend(loc='lower left', fontsize=8)
        path = store.root / f"joint_overlay_pose_{pose['number']:03d}.png"
        fig.savefig(path, dpi=110); paths.append(path)
    if not paths:
        raise ValueError('No included pose has points to overlay.')
    return paths

"""Fit k2 with a shared camera transform and cable differential offset.

The common cable offset is fixed for the headline fit. A separate profile explicitly
shows how freeing that offset changes identifiability. No per-pose camera alignment.
"""
import csv
import json
from pathlib import Path
import numpy as np
from scipy.optimize import least_squares


def predict(model, observation, parameters):
    p = parameters
    model.set_stiffness_profile(p.get('stiffness_ratio', model.rk), p.get('stiffness_exponent', 1.))
    ctrl = model.controls(observation['displacement_mm'], p['mean_offset_mm'], p['difference_offset_mm'])
    state = model.equilibrium(ctrl, p['k2'])
    curve = state['curve_m']
    arc = np.r_[0, np.cumsum(np.linalg.norm(np.diff(curve, axis=0), axis=1))]
    locations = arc[-1] - np.array(observation['tail_distance_px']) / p['pixels_per_m']
    sampled = np.column_stack([np.interp(locations, arc, curve[:, j]) for j in [0, 1]])
    if observation.get('landmark_ids'):
        sampled = model.landmark_points(observation['landmark_ids'], observation.get('landmark_mode', 'joints'))
    c, s = np.cos(p['rotation_rad']), np.sin(p['rotation_rad'])
    rotation = np.array([[c, -s], [s, c]])
    translation = np.array([p['translation_x_px'], p['translation_y_px']])
    points = p['pixels_per_m'] * (sampled @ rotation.T) + translation
    full_curve = p['pixels_per_m'] * (curve @ rotation.T) + translation
    coverage_penalty = np.maximum(-locations, 0) * p['pixels_per_m']
    if observation.get('landmark_ids'):
        coverage_penalty = np.zeros(len(sampled))
    return points, full_curve, state, coverage_penalty


def optimize(model, observations, steps, fixed_k2=None, fixed_mean=0., initial=None):
    # All nuisance parameters are shared across all fitting poses.
    names = []
    x0, low, high = [], [], []
    for name, value, lo, hi, enabled in [
        ('log_k2', np.log(model.k2), np.log(.001), np.log(10), fixed_k2 is None),
        ('mean_offset_mm', 0., -5., 5., fixed_mean is None),
        ('difference_offset_mm', -1., -7., 7., True),
        ('log_pixels_per_m', np.log(4200.), np.log(3000.), np.log(6000.), True),
        ('rotation_rad', -.08, -.3, .3, True),
        ('translation_x_px', 875., 700., 1000., True),
        ('translation_y_px', 0., -150., 150., True),
    ]:
        if enabled:
            names.append(name)
            if initial is not None:
                value = np.log(initial[name[4:]]) if name.startswith('log_') else initial[name]
            x0.append(np.clip(value, lo+1e-8, hi-1e-8))
            low.append(lo)
            high.append(hi)

    def unpack(x):
        result = {'k2': fixed_k2, 'mean_offset_mm': fixed_mean}
        for name, value in zip(names, x):
            result[name[4:] if name.startswith('log_') else name] = float(np.exp(value) if name.startswith('log_') else value)
        return result

    training = [o for o in observations if o['step'] in steps]
    if not training:
        raise ValueError('No selected fitting observations.')

    def residual(x):
        p = unpack(x)
        values = []
        for observation in training:
            predicted, _, _, coverage = predict(model, observation, p)
            values.extend((predicted - np.array(observation['points_px'])).ravel())
            values.extend(coverage)
        return np.array(values)

    # Absolute perturbations avoid a near-zero cable offset receiving a tiny, noisy
    # finite-difference step through the nested equilibrium optimizer.
    increments = {'log_k2': 1e-3, 'mean_offset_mm': .002, 'difference_offset_mm': .002,
                  'log_pixels_per_m': 1e-5, 'rotation_rad': 1e-5,
                  'translation_x_px': .01, 'translation_y_px': .01}
    def jacobian(x):
        columns = []
        for i, name in enumerate(names):
            a, b = np.array(x), np.array(x)
            a[i] = max(low[i], x[i]-increments[name])
            b[i] = min(high[i], x[i]+increments[name])
            columns.append((residual(b)-residual(a))/(b[i]-a[i]))
        return np.column_stack(columns)

    result = least_squares(residual, x0, bounds=(low, high), jac=jacobian, method="trf", loss="linear",
                           xtol=1e-8, ftol=1e-8, gtol=1e-6, max_nfev=100)
    if not result.success:
        raise RuntimeError(f'Fit failed to converge: {result.message}')
    p = unpack(result.x)
    errors, poses = [], []
    for observation in observations:
        points, curve, state, coverage = predict(model, observation, p)
        if np.max(coverage) > .01:
            raise RuntimeError('Image curve extends beyond the model under the fitted scale.')
        error = points - np.array(observation['points_px'])
        if observation['step'] in steps:
            errors.extend(np.sum(error**2, axis=1))
        poses.append({'step': observation['step'], 'used_for_fit': observation['step'] in steps,
                      'point_rmse_px': float(np.sqrt(np.mean(np.sum(error**2, axis=1)))),
                      'predicted_points_px': points.tolist(), 'predicted_curve_px': curve.tolist(),
                      **{name: value.tolist() if isinstance(value, np.ndarray) else value
                         for name, value in state.items()}})
    return {'parameters': p, 'point_rmse_px': float(np.sqrt(np.mean(errors))), 'poses': poses,
            'optimizer': {'method': 'trf', 'loss': 'linear', 'objective': 'half sum of squared residuals', 'success': bool(result.success), 'evaluations': int(result.nfev),
                          'message': str(result.message), 'active_bounds': result.active_mask.tolist()}}


def fit(model, observation_document, config, out):
    out = Path(out)
    out.mkdir(parents=True, exist_ok=True)
    observations = observation_document['observations']
    steps = config['fit_steps']
    fixed_mean = config['fixed_mean_offset_mm']
    baseline = optimize(model, observations, steps, fixed_k2=model.k2, fixed_mean=fixed_mean)
    candidate = optimize(model, observations, steps, fixed_mean=fixed_mean, initial=baseline['parameters'])
    ks = sorted(set([.01, .03, .06, .1, model.k2, .3, .5, candidate['parameters']['k2'], 1., 2., 4.]))
    profiles = []
    for k2 in ks:
        conditional = optimize(model, observations, steps, fixed_k2=k2, fixed_mean=fixed_mean,
                               initial=candidate['parameters'])
        free_candidates = []
        for starting_mean in [0., 1.5, -1.5]:
            initial = {**conditional['parameters'], 'mean_offset_mm': starting_mean}
            free_candidates.append(optimize(model, observations, steps, fixed_k2=k2,
                                             fixed_mean=None, initial=initial))
        free = min(free_candidates, key=lambda item: item['point_rmse_px'])
        profiles.append({'k2': k2, 'fixed_mean_rmse_px': conditional['point_rmse_px'],
                         'free_mean_rmse_px': free['point_rmse_px'],
                         'free_mean_offset_mm': free['parameters']['mean_offset_mm']})
    output = {'schema_version': 1, 'model_audit': model.audit(), 'fit_steps': steps,
              'return_check_steps': config['check_steps'], 'baseline': baseline, 'candidate': candidate,
              'stiffness_profile': profiles,
              'interpretation': 'Conditional shape-fit candidate; not a unique physical stiffness identification.',
              'damping_status': 'Preserved from source XML. No damping fit has been performed.',
              'common_offset_assumption_mm': fixed_mean,
              'length_mapping': 'simulation neutral tendon length + recorded displacement + shared offsets',
              'camera_model': 'one planar similarity transform shared across all poses',
              'point_definition': 'silhouette medial-curve samples labelled by distance from distal cap',
              'joint1_policy': 'Original j_001 stiffness and damping preserved exactly.'}
    (out / 'fit.json').write_text(json.dumps(output, indent=2))
    with (out / 'stiffness_profile.csv').open('w', newline='') as f:
        writer = csv.DictWriter(f, fieldnames=list(profiles[0]))
        writer.writeheader()
        writer.writerows(profiles)
    model.export(out / 'candidate_model', candidate['parameters']['k2'])
    with (out / 'joint_parameters.csv').open('w', newline='') as f:
        writer = csv.writer(f)
        writer.writerow(['joint', 'role', 'original_stiffness_Nm_rad', 'candidate_stiffness_Nm_rad',
                         'original_damping_Nm_s_rad', 'candidate_damping_Nm_s_rad'])
        for i, name in enumerate(model.names):
            writer.writerow([name, 'mount' if i == 0 else 'flexible', model.original_k[i],
                             model.m.jnt_stiffness[i], model.original_d[i], model.m.dof_damping[i]])
    return output

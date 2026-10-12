"""Two-parameter equilibrium review of one immutable, measured pose.

Only k2 and ONE exponential-profile constant vary. No camera, motor, Qt or live
model is touched. Invalid equilibria remain missing samples, never minima.
"""
import copy
import hashlib
from pathlib import Path
import numpy as np
from analysis import point_cost
from spirob_analysis.model import SpiRobModel
from spirob_analysis.fitting import predict

PROFILE_AXES = {
    'stiffness_ratio': ('Ratio r', .5, 1.5),
    'stiffness_exponent': ('Exponent multiplier b', 0., 3.),
}
FORMULA = 'kᵢ = k₂ · r^[b(i−2)], i ≥ 2; joint 1 unchanged'


class SurfaceCancelled(Exception):
    pass


def sample_surface(model_path, source, parameter, lower, upper, count,
                   k_lower, k_upper, k_count, progress=None, cancel=None):
    """Rows vary the chosen constant; columns vary logarithmically spaced k2."""
    source = copy.deepcopy(source)
    if parameter not in PROFILE_AXES:
        raise ValueError('Choose one profile constant: ratio r or exponent multiplier b.')
    _, lo, hi = PROFILE_AXES[parameter]
    if (not np.isfinite([lower, upper, k_lower, k_upper, count, k_count]).all()
        or not lo <= lower < upper <= hi or not .001 <= k_lower < k_upper <= 10
        or int(count) != count or int(k_count) != k_count
        or not 3 <= count <= 31 or not 3 <= k_count <= 31):
        raise ValueError(f'Use {lo:g} ≤ constant minimum < maximum ≤ {hi:g}, '
                         '0.001 ≤ k₂ minimum < maximum ≤ 10 and 3..31 samples per axis.')
    digest = hashlib.sha256(Path(model_path).read_bytes()).hexdigest()
    if source.get('model_sha256', digest) != digest:
        raise ValueError('Model XML changed after capture. Capture and sweep again.')
    model = SpiRobModel(model_path)
    fixed = copy.deepcopy(source['fixed_parameters'])
    fixed.setdefault('stiffness_ratio', model.rk)
    fixed.setdefault('stiffness_exponent', 1.)
    ks = np.geomspace(k_lower, k_upper, int(k_count))
    constants = np.linspace(lower, upper, int(count))
    if k_lower <= fixed['k2'] <= k_upper: ks = np.unique(np.r_[ks, fixed['k2']])
    if lower <= fixed[parameter] <= upper: constants = np.unique(np.r_[constants, fixed[parameter]])
    observation = source['observation']
    costs, errors = [], []
    total = len(ks)*len(constants)
    for row, value in enumerate(constants):
        row_costs, row_errors = [], []
        for col, k in enumerate(ks):
            if cancel is not None and cancel.is_set(): raise SurfaceCancelled('Surface calculation cancelled.')
            parameters = dict(fixed, k2=float(k), **{parameter: float(value)})
            try:
                points, _, _, coverage = predict(model, observation, parameters)
                if np.max(coverage) > .01: raise ValueError('Observation exceeds model length')
                cost = point_cost(observation['points_px'], points)
                if not np.isfinite(cost): raise ValueError('Non-finite equilibrium cost')
                row_costs.append(cost); row_errors.append(None)
            except (ValueError, RuntimeError, FloatingPointError) as exc:
                row_costs.append(None); row_errors.append(str(exc))
            if progress: progress(row*len(ks)+col+1, total)
        costs.append(row_costs); errors.append(row_errors)
    if cancel is not None and cancel.is_set(): raise SurfaceCancelled('Surface calculation cancelled.')
    valid = [(r,c) for r in range(len(constants)) for c in range(len(ks)) if costs[r][c] is not None]
    if not valid: raise ValueError('No valid equilibrium in this grid. Narrow the ranges or review the captured pose.')
    row, col = min(valid, key=lambda pair: costs[pair[0]][pair[1]])
    best_parameters = dict(fixed, k2=float(ks[col]), **{parameter: float(constants[row])})
    boundary_axes = []
    if col in (0,len(ks)-1): boundary_axes.append('k2')
    if row in (0,len(constants)-1): boundary_axes.append(parameter)
    return dict(version='0.8.0', kind='stiffness_surface', formula=FORMULA,
        parameter=parameter, parameter_label=PROFILE_AXES[parameter][0],
        k2=ks.tolist(), constant_values=constants.tolist(), cost_px2=costs, errors=errors,
        fixed_parameters=fixed, observation=observation, frozen_d2=source.get('frozen_d2'),
        minimum=dict(row=row, column=col, k2=float(ks[col]), constant_value=float(constants[row]),
                     cost_px2=costs[row][col], parameters=best_parameters,
                     at_boundary=bool(boundary_axes), boundary_axes=boundary_axes),
        requested_grid=dict(k_lower=k_lower,k_upper=k_upper,k_count=int(k_count),
                            lower=lower,upper=upper,count=int(count)),
        actual_sample_count=total, valid_sample_count=len(valid), invalid_sample_count=total-len(valid),
        source={key:copy.deepcopy(source[key]) for key in ('session','frame_index','frame_t_s','frame_sha256',
                'model_sha256','setup_revision','setup','recorded_demo') if key in source},
        meaning='Lowest valid sampled J for one frozen pose and the requested grid; not a guaranteed global minimum. '
                'Alignment, offsets, damping and the other profile constant are fixed. The live model is unchanged.',
        identifiability='Only r**b determines adjacent stiffness ratio. r and b are alternative ways to review the same '
                        'profile slope; they cannot be independently identified from this stiffness law.')

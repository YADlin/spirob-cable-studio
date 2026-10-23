"""3. Analysis: measured cable inputs -> MuJoCo -> image points -> squared error.

Read compare() first. It advances physics; it never commands the real robot.
The existing multi-pose static fit is in spirob_analysis/fitting.py.
"""
from collections import deque
import copy
import numpy as np
import mujoco
from spirob_analysis.model import SpiRobModel
from spirob_analysis.fitting import predict
from landmarks import correspondences


def point_cost(observed, predicted):
    """Mean squared Euclidean point distance, in px² per point."""
    error = np.asarray(predicted) - np.asarray(observed)
    return float(np.mean(np.sum(error ** 2, axis=1)))


def inputs_at(packet, t, max_gap):
    """Interpolate encoder-derived lengths only between bracketing samples."""
    lengths, targets = [], []
    for axis in (0, 1):
        rows = sorted((r for r in packet['motor'] if r['axis'] == axis), key=lambda r: r['t_s'])
        times = np.array([r['t_s'] for r in rows])
        if len(times) < 2 or t < times[0] or t > times[-1]:
            raise ValueError('Waiting for encoder samples to bracket the image time')
        j = int(np.clip(np.searchsorted(times, t), 1, len(times)-1))
        if times[j] - times[j-1] > max_gap:
            raise ValueError('Encoder sample gap; comparison paused')
        lengths.append(float(np.interp(t, times, [r['length_mm'] for r in rows])))
        row = rows[max(0, int(np.searchsorted(times, t, side='right'))-1)]
        targets.append(row['target_mm'])
    return np.array(lengths), targets


class Comparator:
    def __init__(self, config, fit):
        self.config = config
        self.p = copy.deepcopy(fit['candidate']['parameters'])
        self.model_path = config['model']
        self.model = SpiRobModel(self.model_path)
        self.d2 = self.model.d2
        self.p.setdefault('stiffness_ratio', self.model.rk)
        self.p.setdefault('stiffness_exponent', 1.)
        self.model.set_stiffness_profile(self.p['stiffness_ratio'], self.p['stiffness_exponent'])
        self.revision = 0
        self.session = None
        self.history = deque()
        self.virtual_history = deque()
        self.dynamic_t = None
        self.mapping = [0, 1]
        self.mapping_signs = [1, 1]
        self.profile = None
        if config.get('comparison_setup'):
            self.set_setup(config['comparison_setup'])

    def reset(self):
        self.history.clear()
        self.virtual_history.clear()
        self.dynamic_t = None

    def set_parameters(self, k2, d2, stiffness_ratio=None, stiffness_exponent=None):
        if not np.isfinite(k2) or not .001 <= k2 <= 10 or not np.isfinite(d2) or not 0 <= d2 <= 2:
            raise ValueError('Use 0.001 <= k2 <= 10 and 0 <= d2 <= 2 in SI units')
        ratio = self.p['stiffness_ratio'] if stiffness_ratio is None else stiffness_ratio
        exponent = self.p['stiffness_exponent'] if stiffness_exponent is None else stiffness_exponent
        self.model.set_stiffness_profile(ratio, exponent)
        self.p.update(k2=float(k2), stiffness_ratio=float(ratio), stiffness_exponent=float(exponent))
        self.d2 = float(d2)
        self.model.scale(k2, d2)
        self.revision += 1
        # Preserve q and qdot: the virtual robot responds to the parameter change.
        self.history.clear()
        self.virtual_history.clear()

    def controls(self, lengths, packet):
        displacement = lengths - np.array(packet['rest_lengths_mm'])
        displacement = displacement[self.mapping] * self.mapping_signs
        return self.model.controls(displacement, self.p['mean_offset_mm'], self.p['difference_offset_mm'])

    def set_setup(self, setup):
        """Explicit user calibration; never estimated from a bent-pose mismatch."""
        mapping = list(setup.get('mapping', [0, 1]))
        signs = list(setup.get('signs', [1, 1]))
        if sorted(mapping) != [0, 1] or len(signs) != 2 or any(s not in (-1, 1) for s in signs):
            raise ValueError('Map each physical cable once; signs must be +1 or −1.')
        p = dict(self.p)
        for key in ('translation_x_px', 'translation_y_px', 'rotation_rad', 'pixels_per_m'):
            value = float(setup.get(key, p[key]))
            if not np.isfinite(value): raise ValueError('Alignment must be finite.')
            p[key] = value
        if not 100 <= p['pixels_per_m'] <= 20000:
            raise ValueError('Use a scale of 100..20000 pixels/metre.')
        profile = copy.deepcopy(setup.get('profile'))
        if profile:
            names = self.model.names[1:]+['tip'] if profile['mode']=='joints' else ['link_'+n.split('_')[-1] for n in self.model.names[1:]]
            tails = profile['tail_px']
            if (len(tails)<4 or any(n not in names for n in tails) or
                any(not np.isfinite(v) or not 0 <= v <= 2000 for v in tails.values())):
                raise ValueError('Invalid landmark spacing profile.')
            if profile.get('kind') == 'rigid_links':
                chain = profile['chain_ids']; lengths = profile['link_lengths_px']
                start = names.index(chain[0]) if chain and chain[0] in names else -1
                if (profile['mode'] != 'joints' or start < 0 or chain != names[start:] or
                    len(chain) < 4 or len(lengths) != len(chain)-1 or
                    any(not np.isfinite(x) or not 1 <= x <= 2000 for x in lengths) or
                    not np.isfinite(profile['anchor_tail_px']) or not 0 <= profile['anchor_tail_px'] <= 2000 or
                    len(set(profile['score_ids'])) < 4 or any(n not in chain for n in profile['score_ids'])):
                    raise ValueError('Invalid rigid joint calibration.')
            if profile.get('measurement_mode') == 'manual_joints':
                from joint_tracking import validate_profile
                validate_profile(profile)
        wiring_changed = mapping != self.mapping or signs != self.mapping_signs
        self.p, self.mapping, self.mapping_signs, self.profile = p, mapping, signs, profile
        self.revision += 1
        # Projection and scoring do not alter physical state or its time base.
        # Only a different tendon input convention requires reinitialization.
        self.history.clear()
        self.virtual_history.clear()
        if wiring_changed:
            self.dynamic_t = None

    def setup(self):
        return dict(mapping=self.mapping, signs=self.mapping_signs, profile=copy.deepcopy(self.profile),
                    **{k:self.p[k] for k in ('translation_x_px','translation_y_px','rotation_rad','pixels_per_m')})

    def observation(self, measured, lengths, packet, layout=None):
        if measured is None: return None
        if measured.get('score_error'): raise ValueError(measured['score_error'])
        if measured.get('measurement_mode') == 'marker_dots':
            # Dots carry their own element identities; no manual profile applies.
            displacement = (lengths-np.asarray(packet['rest_lengths_mm']))[self.mapping]*self.mapping_signs
            return dict(copy.deepcopy(measured), displacement_mm=displacement.tolist())
        if self.profile and self.profile.get('measurement_mode') == 'manual_joints' and measured.get('calibration_id') != self.profile['calibration_id']:
            raise ValueError('Manual joint tracking is unavailable; capture and Apply joints again.')
        if self.profile and measured.get('image_shape') and list(measured['image_shape']) != self.profile['image_shape']:
            raise ValueError('Image size changed; recalibrate joint placement.')
        result = copy.deepcopy(measured)
        if self.profile and self.profile.get('use_for_score'):
            rows = correspondences(measured,layout or self.joint_preview(),mode=self.profile['mode'],profile=self.profile)
            rows = [r for r in rows if r['included']]
            if any(r['physical_px'] is None for r in rows):
                raise ValueError('A calibrated landmark is unavailable; scoring paused.')
            result.update(points_px=[r['physical_px'] for r in rows],
                landmark_ids=[r['id'] for r in rows], landmark_mode=self.profile['mode'],
                tail_distance_px=[self.profile['tail_px'][r['id']] for r in rows])
        displacement = (lengths-np.asarray(packet['rest_lengths_mm']))[self.mapping]*self.mapping_signs
        return dict(result, displacement_mm=displacement.tolist())

    def landmark_rows(self, measured):
        if measured is None: return []
        profile = self.profile
        return correspondences(measured, self.joint_preview(),
            mode=profile['mode'] if profile else 'joints', profile=profile)

    def tendon_telemetry(self):
        m, d = self.model.m, self.model.d
        ids = m.actuator_trnid[:, 0]
        return dict(names=[m.tendon(int(i)).name for i in ids],
                    demanded_mm=(d.ctrl*1000).tolist(), actual_mm=(d.ten_length[ids]*1000).tolist(),
                    physical_axes=list(self.mapping), signs=list(self.mapping_signs),
                    input_source='encoder displacement from physical rest, then mapping and fixed offsets')

    def transform(self, points):
        p = self.p
        c, s = np.cos(p['rotation_rad']), np.sin(p['rotation_rad'])
        return p['pixels_per_m']*(np.asarray(points) @ np.array([[c,s],[-s,c]])) + [p['translation_x_px'],p['translation_y_px']]

    def stable_shape(self, history, t, points):
        settings = self.config['live']
        history.append((t, np.asarray(points)))
        while len(history) > 1 and history[1][0] <= t - settings['settle_window_s']:
            history.popleft()
        if t - history[0][0] < settings['settle_window_s']:
            return False
        if any(b[0]-a[0] > settings['max_age_s'] for a, b in zip(history, list(history)[1:])):
            return False
        return bool(max(np.linalg.norm(p-points, axis=1).max() for _, p in history) <= settings['settle_motion_px'])

    def physical_settled(self, packet, observed):
        if observed is None:
            self.history.clear()
            return False
        t = packet['t_s']
        if not self.stable_shape(self.history, t, np.array(observed['points_px'])):
            return False
        return self.encoders_settled(packet)

    def encoders_settled(self, packet):
        """Initialize the prediction from held inputs, independently of vision."""
        t = packet['t_s']
        window = self.config['live']['settle_window_s']
        for axis in (0, 1):
            stream = sorted((r for r in packet['motor'] if r['axis']==axis),key=lambda r:r['t_s'])
            times = [r['t_s'] for r in stream]
            first = max(0,int(np.searchsorted(times,t-window,side='right'))-1)
            last = int(np.searchsorted(times,t,side='left'))+1
            rows = stream[first:last]  # Bracket the hold interval despite irregular serial sampling.
            if len(rows) < 3 or any(r['state'] != 'HOLDING' for r in rows):
                return False
            if rows[0]['t_s'] > t-window or rows[-1]['t_s'] < t or any(
                    b['t_s']-a['t_s'] > self.config['live']['max_input_gap_s']
                    for a,b in zip(rows,rows[1:])):
                return False
            if np.ptp([r['length_mm'] for r in rows]) > .15:
                return False
        return True

    def score_geometry(self, observed, lengths, packet):
        """A missing measurement disables J, never the independent backbone."""
        if observed is None:
            return None, np.empty((0,2)), [], 'Camera centreline unavailable; model prediction continues, J paused'
        try:
            observation = self.observation(observed, lengths, packet)
            if observation.get('landmark_ids'):
                points = self.transform(self.model.landmark_points(observation['landmark_ids'], observation['landmark_mode']))
            else:
                points, _ = self.project(self.model.curve(), observed['tail_distance_px'])
            return observation, points, self.landmark_rows(observed), None
        except (ValueError, KeyError) as exc:
            return None, np.empty((0,2)), [], 'Model prediction continues; J paused: '+str(exc)

    def advance(self, packet, t):
        """Integrate at the XML timestep using the measured input time history."""
        live, model = self.config['live'], self.model
        if t < self.dynamic_t or t-self.dynamic_t > live['max_age_s']:
            raise ValueError('Display gap; wait at rest to initialize the simulation again')
        streams = []
        for axis in (0, 1):
            rows = sorted((r for r in packet['motor'] if r['axis'] == axis), key=lambda r: r['t_s'])
            times = np.array([r['t_s'] for r in rows])
            if self.dynamic_t < times[0] or t > times[-1] or any(
                    b-a > live['max_input_gap_s'] and b > self.dynamic_t and a < t
                    for a, b in zip(times, times[1:])):
                raise ValueError('Input history has a gap; comparison paused')
            streams.append((times, np.array([r['length_mm'] for r in rows])))
        original_dt = model.m.opt.timestep
        try:
            while model.d.time < t-1e-10:
                dt = min(original_dt, t-model.d.time)
                model.m.opt.timestep = dt
                lengths = np.array([np.interp(model.d.time+dt/2, ts, ls) for ts, ls in streams])
                model.d.ctrl[:] = self.controls(lengths, packet)
                mujoco.mj_step(model.m, model.d)
                if model.d.ncon or not np.all(np.isfinite(model.d.qpos)) or np.any(model.d.warning.number):
                    raise ValueError('Simulation reached contact or an invalid numerical state')
        finally:
            model.m.opt.timestep = original_dt
        self.dynamic_t = t
        mujoco.mj_forward(model.m, model.d)

    def project(self, curve, tails):
        """The same frozen camera transformation applies at every deformation."""
        p = self.p
        arc = np.r_[0., np.cumsum(np.linalg.norm(np.diff(curve, axis=0), axis=1))]
        sample_at = arc[-1] - np.array(tails)/p['pixels_per_m']
        if np.min(sample_at) < 0:
            raise ValueError('Observed arc exceeds model length')
        sampled = np.c_[np.interp(sample_at, arc, curve[:, 0]), np.interp(sample_at, arc, curve[:, 1])]
        c, s = np.cos(p['rotation_rad']), np.sin(p['rotation_rad'])
        rotation = np.array([[c, -s], [s, c]])
        def transform(x):
            return p['pixels_per_m']*(x @ rotation.T) + [p['translation_x_px'], p['translation_y_px']]
        return transform(sampled), transform(curve)

    def joint_preview(self, available=True):
        p = self.p
        c, s = np.cos(p['rotation_rad']), np.sin(p['rotation_rad'])
        xy = p['pixels_per_m'] * (self.model.joint_curve() @ np.array([[c, s], [-s, c]]))
        xy += [p['translation_x_px'], p['translation_y_px']]
        return dict(ids=self.model.names[1:] + ['tip'],
                    tail_m=self.model.joint_tail_m.tolist(),
                    model_px=xy.tolist() if available else None,
                    pixels_per_m=p['pixels_per_m'], base_px=self.transform(self.model.base_point()).tolist())

    def compare(self, packet, observed):
        """One live update: check timing, advance physics, project, then score."""
        settings = self.config['live']
        if self.session != packet['session']:
            self.reset()
            self.session = packet['session']
        if not packet.get('motor_active') or packet['state'] not in ('RECORDING', 'STREAMING') or packet['frame_age_s'] > settings['max_age_s'] or any(
                v is None or v > settings['max_age_s'] or v < -.1 for v in packet['motor_age_s']):
            self.reset()
            raise ValueError('Waiting for fresh video and enabled, referenced motor feedback')
        t = packet['t_s']
        lengths, targets = inputs_at(packet, t, settings['max_input_gap_s'])
        if self.dynamic_t is None:
            if not self.encoders_settled(packet):
                raise ValueError('Initialization: hold both referenced cables still for two seconds')
            self.model.equilibrium(self.controls(lengths, packet), self.p['k2'])
            self.model.scale(self.p['k2'], self.d2)
            self.model.d.qvel[:] = 0
            self.model.d.time = self.dynamic_t = t
        try:
            self.advance(packet, t)
        except Exception:
            self.reset()
            raise
        curve = self.transform(self.model.curve())
        observation, points, rows, score_error = self.score_geometry(observed, lengths, packet)
        input_hold = self.encoders_settled(packet)
        physical_hold = self.physical_settled(packet, observation) if observed is not None else False
        virtual_hold = self.stable_shape(self.virtual_history, t, curve)
        max_velocity = float(np.max(np.abs(self.model.d.qvel)))
        settled = physical_hold and virtual_hold and max_velocity < .01
        instantaneous = point_cost(observation['points_px'], points) if observation else None
        cost = instantaneous if settled else None
        return dict(t_s=t, lengths_mm=lengths.tolist(), targets_mm=targets,
                    landmarks=self.joint_preview(),
                    landmark_rows=rows, tendons=self.tendon_telemetry(), setup=self.setup(),
                    base_px=self.transform(self.model.base_point()).tolist(),
                    predicted_points_px=points.tolist(), predicted_curve_px=curve.tolist(),
                    settled=settled, physical_settled=physical_hold, virtual_settled=virtual_hold,
                    encoders_settled=input_hold,
                    cost_px2=cost, rmse_px=None if cost is None else float(np.sqrt(cost)),
                    motion_error_px2=instantaneous, max_joint_speed_rad_s=max_velocity,
                    k2=self.p['k2'], d2=self.d2, stiffness_ratio=self.p['stiffness_ratio'],
                    stiffness_exponent=self.p['stiffness_exponent'], revision=self.revision,
                    observation=observation,
                    message=score_error or ('Ready to compare: both shapes settled' if settled else 'Moving / settling: stiffness score pending'))

    def attach_measurement(self, captured, observed, packet, recorded=False):
        """Score a delayed vision result against its SAME-frame model snapshot.

        Physics has moved on; never read the current model geometry here.
        """
        result=copy.deepcopy(captured)
        if captured.get('revision') != self.revision:
            raise ValueError('Comparison settings changed while this image was processed.')
        layout=captured['landmarks'];profile=self.profile
        rows=correspondences(observed,layout,mode=profile['mode'] if profile else 'joints',profile=profile)
        result['landmark_rows']=rows
        if captured.get('setup_preview'):
            return result
        observation=self.observation(observed,np.array(captured['lengths_mm']),packet,layout)
        if observation is None:return result
        if observation.get('landmark_ids'):
            all_points=dict(zip(layout['ids'],layout['model_px']))
            if observation['landmark_mode']=='centres':
                all_points={'link_'+a.split('_')[-1]:((np.array(all_points[a])+all_points[b])/2).tolist()
                    for a,b in zip(layout['ids'],layout['ids'][1:])}
            points=np.array([all_points[n] for n in observation['landmark_ids']])
        else:
            curve=np.array(captured['predicted_curve_px'])
            arc=np.r_[0.,np.cumsum(np.linalg.norm(np.diff(curve,axis=0),axis=1))]
            at=arc[-1]-np.array(observation['tail_distance_px'])
            if np.min(at)<0:raise ValueError('Observed arc exceeds model length.')
            points=np.c_[np.interp(at,arc,curve[:,0]),np.interp(at,arc,curve[:,1])]
        physical_hold=bool(recorded or self.physical_settled(packet,observation))
        virtual_hold=bool(recorded or (captured.get('virtual_settled') and captured.get('max_joint_speed_rad_s',1.)<.01))
        settled=physical_hold and virtual_hold
        cost=point_cost(observation['points_px'],points)
        if settled:message='Ready to compare: measured and model shapes settled'
        elif not captured.get('encoders_settled',recorded):message='Sweep waiting: cable encoders have not held for two seconds'
        elif not virtual_hold:message='Sweep waiting: MuJoCo backbone is still settling'
        else:message='Sweep waiting: image points are moving, intermittent or not yet stable for two seconds'
        result.update(observation=observation,predicted_points_px=points.tolist(),
            cost_px2=cost if settled else None,rmse_px=float(np.sqrt(cost)) if settled else None,
            motion_error_px2=cost,settled=settled,physical_settled=physical_hold,message=message)
        return result

    def compare_still(self, packet, observed):
        """An explicitly labelled recorded hold, for learning without hardware."""
        lengths, targets = inputs_at(packet, packet['t_s'], self.config['live']['max_input_gap_s'])
        self.model.equilibrium(self.controls(lengths, packet), self.p['k2'])
        observation, points, rows, score_error = self.score_geometry(observed, lengths, packet)
        curve = self.transform(self.model.curve())
        cost = point_cost(observation['points_px'], points) if observation else None
        return dict(t_s=packet['t_s'], lengths_mm=lengths.tolist(), targets_mm=targets,
                    landmarks=self.joint_preview(),
                    landmark_rows=rows, tendons=self.tendon_telemetry(), setup=self.setup(),
                    base_px=self.transform(self.model.base_point()).tolist(),
                    predicted_points_px=points.tolist(), predicted_curve_px=curve.tolist(),
                    settled=cost is not None, cost_px2=cost, rmse_px=None if cost is None else float(np.sqrt(cost)), motion_error_px2=cost,
                    observation=observation, k2=self.p['k2'], d2=self.d2,
                    stiffness_ratio=self.p['stiffness_ratio'], stiffness_exponent=self.p['stiffness_exponent'], revision=self.revision,
                    message=score_error or 'Recorded settled image: offline static comparison; not a live stream')

    def compare_setup(self, packet, observed):
        """Unloaded XML reference shape; no fabricated encoder data or score."""
        mujoco.mj_resetData(self.model.m, self.model.d)
        self.model.scale(self.p['k2'], self.d2)
        mujoco.mj_forward(self.model.m, self.model.d)
        return dict(t_s=packet['t_s'], lengths_mm=[None,None], targets_mm=[None,None],
            landmarks=self.joint_preview(), landmark_rows=self.landmark_rows(observed),
            setup=self.setup(), base_px=self.transform(self.model.base_point()).tolist(),
            predicted_curve_px=self.transform(self.model.joint_curve()).tolist(), predicted_points_px=[],
            observation=None, cost_px2=None, rmse_px=None, motion_error_px2=None,
            settled=False, k2=self.p['k2'], d2=self.d2, stiffness_ratio=self.p['stiffness_ratio'],
            stiffness_exponent=self.p['stiffness_exponent'], revision=self.revision, setup_preview=True,
            message='SETUP PREVIEW — MuJoCo reference pose; motors disconnected; J / k sweep unavailable')

    def scan(self, result, count=23, lower=.02, upper=3.):
        """Freeze ONE settled observation; sample J(k2) with alignment fixed."""
        if not result['settled'] or result['cost_px2'] is None:
            raise ValueError('Wait for a valid settled comparison before scanning k2')
        if (not np.isfinite(lower) or not np.isfinite(upper) or
                not .001 <= lower < upper <= 10 or not 3 <= count <= 201):
            raise ValueError('Use 0.001 ≤ minimum < maximum ≤ 10 and 3..201 samples.')
        model = SpiRobModel(self.model_path)
        fixed = dict(self.p)
        ks = np.geomspace(lower, upper, count)
        if lower <= fixed['k2'] <= upper:
            ks = np.unique(np.r_[ks, fixed['k2']])
        costs, errors = [], []
        for k in ks:
            try:
                predicted, _, _, coverage = predict(model, result['observation'], dict(fixed, k2=float(k)))
                if max(coverage) > .01: raise ValueError('Observation exceeds model length')
                costs.append(point_cost(result['observation']['points_px'], predicted)); errors.append(None)
            except (ValueError, RuntimeError) as exc:
                costs.append(None); errors.append(str(exc))
        valid = [i for i,c in enumerate(costs) if c is not None and np.isfinite(c)]
        if not valid: raise ValueError('No valid equilibrium in the selected k₂ interval.')
        best = min(valid, key=lambda i:costs[i])
        return dict(k2=ks.tolist(), cost_px2=costs, frame_t_s=result['t_s'],
                    minimum=dict(k2=float(ks[best]), cost_px2=costs[best], index=best,
                                 at_boundary=best in (0, len(ks)-1)), errors=errors,
                    lower=lower, upper=upper, count=count, actual_sample_count=len(ks), setup=self.setup(),
                    fixed_parameters=fixed, observation=result['observation'],
                    meaning='Single-pose equilibrium sweep. Selected live parameters are unchanged.')

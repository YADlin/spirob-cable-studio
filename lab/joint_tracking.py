"""Manual image measurements and conservative, image-only joint tracking.

The frozen clicks define the measurement, not a prior segmentation or MuJoCo.
Future frames need image evidence; a failed track never replays saved pixels.
"""
import base64
import uuid
import cv2
import numpy as np
from scipy.optimize import least_squares


def gray_image(rgb):
    return cv2.cvtColor(np.asarray(rgb, dtype=np.uint8), cv2.COLOR_RGB2GRAY)


def manual_profile(rows, mode, frame_index, rgb, use_for_score=True, rigid=True):
    """Store the exact displayed/clicked coordinates, without curve projection."""
    if rigid and mode != 'joints':
        raise ValueError('Rigid lengths require joint anchors, not element centres.')
    visible = [r for r in rows if r['physical_px'] is not None]
    if len(visible) < 4:
        raise ValueError('Place at least four visible points.')
    names = [r['id'] for r in visible]
    if rigid:
        start = next(i for i, r in enumerate(rows) if r['physical_px'] is not None)
        if names != [r['id'] for r in rows[start:]] or names[-1] != 'tip':
            raise ValueError('Place every joint from the first visible joint through the tip. Hidden proximal joints may be omitted.')
    points = np.asarray([r['physical_px'] for r in visible], float)
    h, w = rgb.shape[:2]
    if points.shape != (len(names), 2) or not np.isfinite(points).all() or np.any(points < 0) or np.any(points >= [w, h]):
        raise ValueError('Place finite joint coordinates inside the image.')
    lengths = np.linalg.norm(np.diff(points, axis=0), axis=1)
    if np.any(lengths < 1.):
        raise ValueError('Adjacent points must be at least 1 pixel apart. Check their identities in base-to-tip table order.')
    score_ids = [r['id'] for r in visible if r['included']]
    if len(score_ids) < 4:
        raise ValueError('Include at least four visible points in J.')
    tails = np.r_[np.cumsum(lengths[::-1])[::-1], 0.]
    gray = gray_image(rgb)
    templates = {}
    for name, point in zip(names, points):
        patch = cv2.getRectSubPix(gray, (41, 41), tuple(point))
        ok, encoded = cv2.imencode('.png', patch)
        if not ok: raise ValueError('Could not store joint image references.')
        templates[name] = base64.b64encode(encoded).decode('ascii')
    return dict(mode=mode, measurement_mode='manual_joints', calibration_id=uuid.uuid4().hex,
        kind='rigid_links' if rigid else 'image_points', chain_ids=names,
        link_lengths_px=lengths.tolist(), anchor_tail_px=0.,
        tail_px=dict(zip(names, tails.tolist())), score_ids=score_ids,
        calibration_points_px=dict(zip(names, points.tolist())),
        templates_png=templates, image_shape=[h, w], source_frame=frame_index,
        use_for_score=bool(use_for_score),
        method='Exact manual coordinates on the frozen frame; image flow on subsequent frames. '
               + ('Fixed adjacent pixel lengths constrain planar motion. ' if rigid else '')
               + 'Tracking loss pauses J; no snapping to the automatic centreline or model.')


def validate_profile(profile):
    names = profile.get('chain_ids', [])
    if not names or len(names) != len(set(names)) or len(names) > 100:
        raise ValueError('Invalid manual joint identities.')
    points = np.asarray([profile.get('calibration_points_px', {}).get(n, []) for n in names], float)
    h, w = profile['image_shape']
    if (points.shape != (len(names), 2) or not np.isfinite(points).all() or
            np.any(points < 0) or np.any(points >= [w, h]) or not profile.get('calibration_id')):
        raise ValueError('Invalid manual calibration coordinates.')
    if len(profile.get('score_ids', [])) < 4 or any(n not in names for n in profile['score_ids']):
        raise ValueError('Invalid scored joint identities.')
    if set(profile['tail_px']) != set(names): raise ValueError('Invalid manual point spacing.')
    if profile.get('kind') == 'rigid_links' and not np.allclose(
            np.linalg.norm(np.diff(points, axis=0), axis=1), profile['link_lengths_px'], atol=1e-6):
        raise ValueError('Manual link lengths do not match the calibrated coordinates.')
    for name in names:
        encoded = profile.get('templates_png', {}).get(name, '')
        if not isinstance(encoded, str) or len(encoded) > 16000:
            raise ValueError('Invalid saved joint image reference.')
        try:
            patch = cv2.imdecode(np.frombuffer(base64.b64decode(encoded, validate=True), np.uint8), cv2.IMREAD_GRAYSCALE)
        except (ValueError, cv2.error) as exc:
            raise ValueError('Invalid saved joint image reference.') from exc
        if patch is None or patch.shape != (41, 41):
            raise ValueError('Missing joint image reference; capture and Apply joints again.')


def manual_observation(profile, points, config, source='manual calibration'):
    points = np.asarray(points, float)
    arc = np.r_[0., np.cumsum(np.linalg.norm(np.diff(points, axis=0), axis=1))]
    named = dict(zip(profile['chain_ids'], points.tolist()))
    measured = dict(curve_px=points.tolist(), visible_arc_px=float(arc[-1]),
        joint_points_px=named, calibration_id=profile['calibration_id'],
        image_shape=list(profile['image_shape']), measurement_source=source)
    if profile['use_for_score']:
        ids = profile['score_ids']
        measured.update(points_px=[named[n] for n in ids],
            tail_distance_px=[profile['tail_px'][n] for n in ids])
    else:
        tails = np.asarray(config['live']['tail_distance_px'], float)
        at = arc[-1] - tails
        measured.update(points_px=np.c_[np.interp(at, arc, points[:, 0]), np.interp(at, arc, points[:, 1])].tolist(),
            tail_distance_px=tails.tolist())
        if np.any(at < 0):
            measured['score_error'] = 'Corrected visible chain is too short for the original curve samples. Enable verified points for J or place more proximal joints.'
    return measured


def constrain_lengths(points, profile):
    """Fit image positions to the measured link lengths, never to model shape."""
    if profile.get('kind') != 'rigid_links': return points
    lengths = np.asarray(profile['link_lengths_px'])
    angles = np.arctan2(*np.diff(points, axis=0)[:, ::-1].T)
    def decode(x):
        steps = lengths[:, None] * np.c_[np.cos(x[2:]), np.sin(x[2:])]
        return np.vstack([x[:2], x[:2] + np.cumsum(steps, axis=0)])
    fit = least_squares(lambda x: (decode(x)-points).ravel(), np.r_[points[0], angles], max_nfev=50)
    constrained = decode(fit.x)
    if not fit.success or np.max(np.linalg.norm(constrained-points, axis=1)) > 3.:
        raise ValueError('Image joints no longer satisfy the calibrated link lengths.')
    return constrained


class ManualJointTracker:
    """Bounded memory, forward/backward flow and explicit tracking-loss state."""
    def __init__(self, profile, config, reference=None):
        validate_profile(profile)
        self.profile, self.config = profile, config
        self.points = np.asarray([profile['calibration_points_px'][n] for n in profile['chain_ids']], float)
        self.gray = self.key = self.last_observation = None
        self.lost = None
        if reference is not None:
            self.gray = gray_image(reference['rgb'])
            if list(self.gray.shape) != profile['image_shape']:
                raise ValueError('Reference image size does not match joint placement.')
            self.key = (reference['packet']['session'], reference['packet']['frame_index'])
            self.last_observation = manual_observation(profile, self.points, config)

    def reacquire(self, gray):
        """A saved calibration needs unambiguous image matches, not old pixels."""
        h, w = gray.shape
        matched = []
        for name in self.profile['chain_ids']:
            point = np.asarray(self.profile['calibration_points_px'][name])
            patch = cv2.imdecode(np.frombuffer(base64.b64decode(self.profile['templates_png'][name]), np.uint8), cv2.IMREAD_GRAYSCALE)
            if np.std(patch) < 6.: raise ValueError(f'{name} has too little image texture to reacquire.')
            x0, y0 = np.maximum(np.floor(point).astype(int)-70, 0)
            x1, y1 = np.minimum(np.ceil(point).astype(int)+71, [w, h])
            search = gray[y0:y1, x0:x1]
            if min(search.shape) < 41: raise ValueError(f'{name} is too close to the image edge to reacquire.')
            scores = cv2.matchTemplate(search, patch, cv2.TM_CCOEFF_NORMED)
            _, best, _, (x, y) = cv2.minMaxLoc(scores)
            scores[max(0,y-6):y+7, max(0,x-6):x+7] = -1
            if best < .8 or best-float(scores.max()) < .04:
                raise ValueError(f'{name} has no unambiguous image match.')
            matched.append([x0+x+20., y0+y+20.])
        return constrain_lengths(np.asarray(matched), self.profile)

    def update(self, rgb, key):
        if list(rgb.shape[:2]) != self.profile['image_shape']:
            self.lost = 'Image size changed.'
        if self.lost:
            raise ValueError(self.lost + ' Tracking paused; capture a fresh frame and Apply joints again.')
        if key == self.key and self.last_observation is not None:
            return self.last_observation
        gray = gray_image(rgb)
        try:
            if self.gray is None or (self.key is not None and key[0] != self.key[0]):
                points = self.reacquire(gray)
                source = 'image reacquired'
            else:
                old = self.points.astype(np.float32).reshape(-1, 1, 2)
                options = dict(winSize=(41, 41), maxLevel=3,
                    criteria=(cv2.TERM_CRITERIA_EPS | cv2.TERM_CRITERIA_COUNT, 30, .01))
                new, status, error = cv2.calcOpticalFlowPyrLK(self.gray, gray, old, None, **options)
                if new is None: raise ValueError('No image flow at the placed joints.')
                back, back_status, _ = cv2.calcOpticalFlowPyrLK(gray, self.gray, new, None, **options)
                if (back is None or not np.all(status) or not np.all(back_status) or
                    not np.isfinite(new).all() or np.max(error) > 20. or
                    np.max(np.linalg.norm(back-old, axis=2)) > 1.5):
                    raise ValueError('Joint image flow is missing or inconsistent.')
                points = new[:, 0, :].astype(float)
                if np.max(np.linalg.norm(points-self.points, axis=1)) > 60.:
                    raise ValueError('Joints moved too far between measured frames.')
                # Each point must still have local image evidence before the
                # fixed-length constraint is applied.
                for before, after in zip(self.points, points):
                    a = cv2.getRectSubPix(self.gray, (31, 31), tuple(before))
                    b = cv2.getRectSubPix(gray, (31, 31), tuple(after))
                    if min(float(a.std()), float(b.std())) < 6. or float(cv2.matchTemplate(b, a, cv2.TM_CCOEFF_NORMED)[0, 0]) < .65:
                        raise ValueError('A joint is occluded or lacks consistent image texture.')
                points = constrain_lengths(points, self.profile)
                source = 'image tracked, fixed lengths' if self.profile['kind'] == 'rigid_links' else 'image tracked'
            h, w = gray.shape
            if np.any(points < 0) or np.any(points >= [w, h]):
                raise ValueError('A tracked joint left the image.')
            observed = manual_observation(self.profile, points, self.config, source)
        except (ValueError, cv2.error) as exc:
            self.lost = str(exc)
            raise ValueError(self.lost + ' Tracking paused; capture a fresh frame and Apply joints again.') from exc
        self.gray, self.points, self.key = gray, points, key
        self.last_observation = observed
        return observed

"""Markerless pilot measurements, kept separate from physics and optimization.

Points are samples of an approximate silhouette medial curve, NOT measured joint axes.
ROIs and endpoint seeds are explicit inputs because the scene has no fiducials.
"""
import heapq
import io
import json
from pathlib import Path
import cv2
import numpy as np
from PIL import Image
from scipy.ndimage import gaussian_filter1d
from skimage.morphology import skeletonize


def extract_curve(image_bytes, spec, settings):
    rgb = np.array(Image.open(io.BytesIO(image_bytes)).convert('RGB'))
    r, g, b = np.moveaxis(rgb.astype(float), 2, 0)
    seed = np.array(spec['seed_curve_px'], dtype=int)
    corridor = np.zeros(r.shape, np.uint8)
    for a, end in zip(seed[:-1], seed[1:]):
        cv2.line(corridor, tuple(a), tuple(end), 255, settings['corridor_width_px'])
    mask = ((b-r > settings['blue_minus_red_min']) & (g-r > settings['green_minus_red_min'])
            & (g > settings['green_min']) & (b > settings['blue_min']) & (corridor > 0))
    yy = np.indices(r.shape)[0]
    mask &= (yy >= spec['min_y_px']) & (yy <= spec.get('max_y_px', r.shape[0]-1))
    mask = cv2.morphologyEx(mask.astype('uint8'), cv2.MORPH_CLOSE,
                           np.ones((settings['closing_size_px'],)*2, np.uint8))
    num, labels, stats, _ = cv2.connectedComponentsWithStats(mask)
    if num < 2:
        raise ValueError('No foreground robot component; adjust the explicit vision ROI/thresholds.')
    mask = labels == (1 + np.argmax(stats[1:, cv2.CC_STAT_AREA]))
    yy, xx = np.where(skeletonize(mask))
    xy = np.c_[xx, yy]
    lookup = {tuple(p): i for i, p in enumerate(xy)}
    start = int(np.argmin(np.sum((xy-seed[0])**2, axis=1)))
    end = int(np.argmin(np.sum((xy-seed[-1])**2, axis=1)))
    distance = np.full(len(xy), np.inf)
    previous = np.full(len(xy), -1)
    distance[start] = 0
    queue = [(0., start)]
    neighbors = [(dx, dy) for dx in [-1, 0, 1] for dy in [-1, 0, 1] if dx or dy]
    while queue:
        current_distance, i = heapq.heappop(queue)
        if current_distance > distance[i]:
            continue
        if i == end:
            break
        x, y = xy[i]
        for dx, dy in neighbors:
            j = lookup.get((x+dx, y+dy))
            if j is None:
                continue
            candidate = current_distance + np.hypot(dx, dy)
            if candidate < distance[j]:
                distance[j], previous[j] = candidate, i
                heapq.heappush(queue, (candidate, j))
    if not np.isfinite(distance[end]):
        raise ValueError('Robot mask is disconnected between the supplied endpoint seeds.')
    path, i = [], end
    while i >= 0:
        path.append(xy[i])
        i = previous[i]
    path = np.array(path[::-1], float)
    if len(path) < 100:
        raise ValueError('Extracted path is too short; review the mask and seeds.')
    path = gaussian_filter1d(path, settings['smooth_sigma_px'], axis=0, mode='nearest')
    tangent = path[-1] - path[max(0, len(path)-45)]
    tangent /= np.linalg.norm(tangent)
    extension = []
    for distance in np.arange(.5, settings['max_tip_extension_px'], .5):
        point = path[-1] + distance*tangent
        x, y = np.round(point).astype(int)
        if not (0 <= y < mask.shape[0] and 0 <= x < mask.shape[1] and mask[y, x]):
            break
        extension.append(point)
    if extension:
        path = np.vstack([path, extension])
    arc = np.r_[0, np.cumsum(np.linalg.norm(np.diff(path, axis=0), axis=1))]
    samples = np.linspace(settings['proximal_trim_px'], arc[-1]-settings['distal_trim_px'],
                          settings['point_count'])
    if samples[-1] <= samples[0]:
        raise ValueError('Trim settings remove the whole curve.')
    points = np.column_stack([np.interp(samples, arc, path[:, j]) for j in [0, 1]])
    return rgb, mask, {'points_px': points.tolist(), 'tail_distance_px': (arc[-1]-samples).tolist(),
                       'curve_px': path.tolist(), 'visible_arc_px': float(arc[-1]),
                       'distal_cap_px': path[-1].tolist()}


def extract(session, config, out):
    out = Path(out)
    out.mkdir(parents=True, exist_ok=True)
    observations = []
    for spec in config['vision']['poses']:
        step = spec['step']
        frame, provenance = session.held_frame(step)
        rgb, mask, observation = extract_curve(frame, spec, config['vision']['settings'])
        (out / f'pose_{step:03d}.jpg').write_bytes(frame)
        for a, b in zip(observation['curve_px'][:-1], observation['curve_px'][1:]):
            cv2.line(rgb, tuple(np.round(a).astype(int)), tuple(np.round(b).astype(int)), (255, 70, 60), 2)
        for x, y in observation['points_px']:
            cv2.circle(rgb, (round(x), round(y)), 4, (60, 255, 120), -1)
        Image.fromarray(rgb).save(out / f'extraction_{step:03d}.png')
        Image.fromarray((mask*255).astype('uint8')).save(out / f'mask_{step:03d}.png')
        observation.update(step=step, frame=provenance,
                           displacement_mm=session.displacement_mm(step).tolist())
        observations.append(observation)
    result = {'schema_version': 1, 'measurement': 'approximate silhouette medial curve',
              'image_units': 'native image pixels', 'anatomical_joint_landmarks': False,
              'session_id': session.run['session_id'], 'observations': observations}
    (out / 'observations.json').write_text(json.dumps(result, indent=2))
    return result

"""Image-only centreline extraction in a fixed, visible region of interest.

No simulated shape is used to select pixels: changing k cannot change the measurement.
"""
import heapq
import io
import cv2
import numpy as np
from PIL import Image
from scipy.ndimage import gaussian_filter1d
from skimage.morphology import skeletonize


def measure(image, config):
    rgb = image if isinstance(image, np.ndarray) else np.array(Image.open(io.BytesIO(image)).convert('RGB'))
    if rgb.shape[:2] != (1080, 1920):
        raise ValueError('Calibration expects native 1920 x 1080 frames; review alignment before scoring.')
    settings, live = config['vision']['settings'], config['live']
    r, g, b = np.moveaxis(rgb.astype(np.int16), 2, 0)
    roi = np.zeros(r.shape, np.uint8)
    cv2.fillPoly(roi, [np.array(live['roi_polygon_px'], np.int32)], 1)
    mask = ((b-r > settings['blue_minus_red_min']) & (g-r > settings['green_minus_red_min']) &
            (g > settings['green_min']) & (b > settings['blue_min']) & (roi > 0)).astype(np.uint8)
    mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, np.ones((settings['closing_size_px'],)*2, np.uint8))
    count, labels, stats, _ = cv2.connectedComponentsWithStats(mask)
    root = np.array(live['root_seed_px'])
    candidates = []
    for label in range(1, count):
        if stats[label, cv2.CC_STAT_AREA] < 1000:
            continue
        yy, xx = np.where(labels == label)
        candidates.append((np.min((xx-root[0])**2 + (yy-root[1])**2), label))
    if not candidates or min(candidates)[0] > 80**2:
        raise ValueError('No robot component near the fixed base seed.')
    label = min(candidates)[1]
    mask = labels == label
    # Empty pixels cannot affect thinning. Crop to the component plus a zero rim.
    x0, y0, width, height = stats[label, :4]
    cropped = np.pad(mask[y0:y0+height, x0:x0+width], 1)
    yy, xx = np.where(skeletonize(cropped))
    xy = np.c_[xx+x0-1, yy+y0-1]
    lookup = {tuple(p): i for i, p in enumerate(xy)}
    start = int(np.argmin(np.sum((xy-root)**2, axis=1)))
    distances, previous = np.full(len(xy), np.inf), np.full(len(xy), -1)
    distances[start] = 0
    queue = [(0., start)]
    while queue:
        distance, i = heapq.heappop(queue)
        if distance > distances[i]:
            continue
        x, y = xy[i]
        for dx, dy in ((-1,-1),(-1,0),(-1,1),(0,-1),(0,1),(1,-1),(1,0),(1,1)):
            j = lookup.get((x+dx, y+dy))
            if j is not None and distance + np.hypot(dx, dy) < distances[j]:
                distances[j], previous[j] = distance + np.hypot(dx, dy), i
                heapq.heappush(queue, (distances[j], j))
    end = int(np.argmax(np.where(np.isfinite(distances), distances, -1)))
    indices = []
    while end >= 0:
        indices.append(end)
        end = previous[end]
    path = gaussian_filter1d(xy[indices[::-1]].astype(float), settings['smooth_sigma_px'], axis=0, mode='nearest')
    if len(path) < 100:
        raise ValueError('Robot centreline too short.')
    tangent = path[-1] - path[-min(45, len(path))]
    tangent /= np.linalg.norm(tangent)
    extension = []
    for distance in np.arange(.5, settings['max_tip_extension_px'], .5):
        point = path[-1] + tangent * distance
        x, y = np.round(point).astype(int)
        if not (0 <= x < 1920 and 0 <= y < 1080 and mask[y, x]):
            break
        extension.append(point)
    if extension:
        path = np.vstack([path, extension])
    arc = np.r_[0., np.cumsum(np.linalg.norm(np.diff(path, axis=0), axis=1))]
    tails = np.array(live['tail_distance_px'])
    if arc[-1] < max(tails) + 50 or arc[-1] > 1400:
        raise ValueError('Implausible centreline length; inspect the mask/ROI.')
    points = np.c_[np.interp(arc[-1]-tails, arc, path[:, 0]), np.interp(arc[-1]-tails, arc, path[:, 1])]
    return rgb, {'points_px': points.tolist(), 'tail_distance_px': tails.tolist(),
                 'curve_px': path.tolist(), 'visible_arc_px': float(arc[-1]), 'image_shape':list(rgb.shape[:2])}

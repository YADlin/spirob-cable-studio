"""E1 marker-dot measurement: one black dot at the centre of each element's top face.

Image evidence only. The MuJoCo model contributes nothing but the expected
spacing between neighbouring element centres (a search window), so changing k₂,
the stiffness ratio or cable lengths cannot move a measured point.

Marking convention (NYK, 22 Oct 2026):
  * link_001 (fixed base element): two dots along the body axis,
  * link_002 … link_021: one dot at each element centre,
  * link_021 (tip element) may carry a second dot near its end; it is ignored.
Measured points are reported as landmark_mode='centres' with ids link_002…link_021,
which the Lab already compares with the midpoints between model joints.
"""
import numpy as np
import cv2

LINK_IDS = [f'link_{i:03d}' for i in range(2, 22)]


def detect_dots(rgb, min_area=8, max_area=260, contrast=None, white=None):
    """Dark blobs fully enclosed by bright (white TPU) material.

    Black-hat isolates small dark features; the enclosure ring rejects the dark
    notches between elements and the background at element edges.
    """
    g = cv2.cvtColor(np.asarray(rgb, np.uint8), cv2.COLOR_RGB2GRAY).astype(np.float32)
    # Exposure-relative thresholds: the white robot is the brightest large surface.
    level = float(np.percentile(g, 99.5))
    white = .7 * level if white is None else white
    contrast = .25 * level if contrast is None else contrast
    bh = cv2.morphologyEx(g, cv2.MORPH_BLACKHAT, cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (21, 21)))
    n, lab, st, cen = cv2.connectedComponentsWithStats((bh > contrast).astype(np.uint8))
    h_img, w_img = g.shape
    theta = np.linspace(0, 2 * np.pi, 24, endpoint=False)
    out = []
    for i in range(1, n):
        x, y, w, h, a = st[i]
        if not (min_area <= a <= max_area and w <= 22 and h <= 22 and .4 < w / h < 2.5):
            continue
        cx, cy = cen[i]
        rad = max(5., np.sqrt(a / np.pi) + 3.)
        px = np.clip(np.round(cx + rad * np.cos(theta)).astype(int), 0, w_img - 1)
        py = np.clip(np.round(cy + rad * np.sin(theta)).astype(int), 0, h_img - 1)
        if np.mean(g[py, px] > white) < .9:
            continue
        weights = bh[y:y + h, x:x + w] * (lab[y:y + h, x:x + w] == i)
        yy, xx = np.mgrid[y:y + h, x:x + w]
        s = weights.sum()
        out.append([float((xx * weights).sum() / s), float((yy * weights).sum() / s), float(a)])
    return np.asarray(out, float).reshape(-1, 3)


def expected_spacing_px(model, pixels_per_m):
    """Image distance between consecutive element centres (link_002 → link_021) at rest."""
    centres = model.landmark_points(LINK_IDS, 'centres')
    return np.linalg.norm(np.diff(centres, axis=0), axis=1) * pixels_per_m


def _rotate(v, angle):
    c, s = np.cos(angle), np.sin(angle)
    return np.array([c * v[0] - s * v[1], s * v[0] + c * v[1]])


def _grow(points, start, direction, spacing, first_gap, max_missing, weak=None):
    """Walk from the base along the chain, predicting each next element centre.

    gaps[k] = expected image distance from element k−1 to element k (k=0: from
    the base dot to link_002). A dot may be skipped (unmarked/occluded) at most
    max_missing times; the prediction then spans the summed gaps.
    """
    gaps = np.r_[first_gap, spacing]
    weak = np.zeros(len(points), bool) if weak is None else np.asarray(weak, bool)
    used = np.zeros(len(points), bool)
    chain, ids, missing = [], [], 0
    last, d, turn = np.asarray(start, float), direction / np.linalg.norm(direction), 0.
    k = 0
    while k < len(LINK_IDS):
        found = None
        for skip in range(0, max_missing - missing + 1):
            if k + skip >= len(LINK_IDS):
                break
            span = gaps[k:k + skip + 1]
            gap = float(span.sum())
            # Curl continues at roughly the last observed turn per element.
            predicted = last + gap * _rotate(d, turn * (skip + 1))
            dist = np.linalg.norm(points[:, :2] - predicted, axis=1)
            dist[used] = np.inf
            far = np.linalg.norm(points[:, :2] - last, axis=1) > .4 * gap
            # Clear dots first; faint (often small distal) dots only close to the prediction.
            strong_ok = (~weak) & far & (dist < .45 * gap + 4)
            weak_ok = weak & far & (dist < .3 * gap + 2)
            for ok in (strong_ok, weak_ok):
                if ok.any():
                    j = int(np.argmin(np.where(ok, dist, np.inf)))
                    found = (j, skip)
                    break
            if found:
                break
        if found is None:
            break
        j, skip = found
        missing += skip
        k += skip
        p = points[j, :2]
        new_d = (p - last) / np.linalg.norm(p - last)
        if chain:
            turn = .5 * turn + .5 * np.arctan2(d[0] * new_d[1] - d[1] * new_d[0], d @ new_d) / (skip + 1)
        chain.append(p.tolist()); ids.append(LINK_IDS[k]); used[j] = True
        last, d = p, new_d
        k += 1
    return chain, ids, missing, used


def find_base_pairs(points, base_hint=None, radius=80., pair=(10., 35.)):
    """Candidate base-element dot pairs; restricted near base_hint when given."""
    pairs = []
    for i in range(len(points)):
        for j in range(i + 1, len(points)):
            d = np.linalg.norm(points[i, :2] - points[j, :2])
            if not pair[0] <= d <= pair[1]:
                continue
            mid = (points[i, :2] + points[j, :2]) / 2
            if base_hint is not None and np.linalg.norm(mid - base_hint) > radius:
                continue
            pairs.append((i, j))
    return pairs


def measure_dots(rgb, spacing_px, base_hint=None, min_links=12, max_missing=2):
    """Return an observation compatible with the Lab, or raise ValueError."""
    rgb = np.asarray(rgb)
    strong = detect_dots(rgb)
    level = float(np.percentile(cv2.cvtColor(rgb.astype(np.uint8), cv2.COLOR_RGB2GRAY), 99.5))
    faint = detect_dots(rgb, min_area=3, contrast=.1 * level, white=.6 * level)
    if len(strong) and len(faint):
        near = np.min(np.linalg.norm(faint[:, None, :2] - strong[None, :, :2], axis=2), axis=1)
        faint = faint[near > 4]
    points = np.vstack([strong, faint]) if len(faint) else strong
    weak = np.r_[np.zeros(len(strong), bool), np.ones(len(points) - len(strong), bool)]
    if len(points) < min_links + 2:
        raise ValueError(f'Only {len(points)} marker dots visible; need at least {min_links + 2}.')
    spacing = np.asarray(spacing_px, float)
    best = None
    for i, j in find_base_pairs(strong, base_hint):
        for a, b in ((i, j), (j, i)):          # b = the base dot nearer link_002
            direction = strong[b, :2] - strong[a, :2]
            # link_002 centre lies ~1–5 base-pair spacings beyond the base pair
            for factor in (2., 3.5, 5.):
                chain, ids, missing, used = _grow(points, strong[b, :2], direction,
                                                  spacing, factor * np.linalg.norm(direction), max_missing, weak)
                score = (len(ids), -missing)
                if best is None or score > best[0]:
                    best = (score, chain, ids, missing, used, (a, b))
    if best is None:
        raise ValueError('Base element dots (two dots on the fixed element) not found.')
    _, chain, ids, missing, used, (a, b) = best
    if len(ids) < min_links:
        raise ValueError(f'Marker chain broke after {len(ids)} elements ({ids[-1] if ids else "none"}); '
                         'check lighting, occlusion or a missing dot.')
    chain = np.asarray(chain)
    base = strong[[a, b], :2]
    curve = np.vstack([base, chain])
    seg = np.linalg.norm(np.diff(chain, axis=0), axis=1)
    tails = np.r_[np.cumsum(seg[::-1])[::-1], 0.]
    return dict(points_px=chain.tolist(), landmark_ids=ids, landmark_mode='centres',
                tail_distance_px=tails.tolist(), curve_px=curve.tolist(),
                visible_arc_px=float(seg.sum()), image_shape=list(rgb.shape[:2]),
                measurement_mode='marker_dots', measurement_source='marker dots (automatic)',
                base_dots_px=base.tolist(), missing_links=[n for n in LINK_IDS if n not in ids],
                missing_count=int(missing), unused_dots=int((~used[:len(strong)]).sum()) - 2,
                faint_used=int((used & weak).sum()))


class DotTracker:
    """Stateless per-frame dot measurement with the VisionWorker tracker interface."""
    profile = {'calibration_id': 'marker_dots'}

    def __init__(self, spacing_px, base_hint=None):
        self.spacing = np.asarray(spacing_px, float)
        self.base_hint = None if base_hint is None else np.asarray(base_hint, float)

    def update(self, rgb, key=None):
        result = measure_dots(rgb, self.spacing, self.base_hint)
        # Keep searching near the last base position: the base element is fixed.
        self.base_hint = np.mean(result['base_dots_px'], axis=0)
        return result


def draw(rgb, result, model_points=None):
    """Annotated copy for review: detected chain, ids and optional model points."""
    out = np.ascontiguousarray(np.asarray(rgb, np.uint8).copy())
    pts = np.asarray(result['points_px'])
    for p, q in zip(pts[:-1], pts[1:]):
        cv2.line(out, tuple(np.round(p).astype(int)), tuple(np.round(q).astype(int)), (0, 200, 80), 2)
    for (x, y), name in zip(pts, result['landmark_ids']):
        cv2.circle(out, (round(x), round(y)), 7, (0, 230, 90), 2)
        cv2.putText(out, name[-2:], (round(x) + 9, round(y) + 4), cv2.FONT_HERSHEY_SIMPLEX, .4, (0, 120, 40), 1)
    for x, y in result['base_dots_px']:
        cv2.drawMarker(out, (round(x), round(y)), (255, 140, 0), cv2.MARKER_CROSS, 14, 2)
    if model_points is not None:
        for x, y in np.asarray(model_points):
            cv2.circle(out, (round(x), round(y)), 4, (230, 40, 40), -1)
    return out

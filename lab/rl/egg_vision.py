"""Simple, reviewable colour detectors for the overhead camera (no learning).

Print colours (only white and black PLA are available):
  egg      black PLA, matte  (sim rgba 0.06 0.06 0.06)  - dark against the brown floor and the white robot
  pedestal white PLA         (sim rgba 0.93 0.93 0.93)  - never confused with the black egg
Each colour model is an HSV box sampled by the operator clicking the object once. For grey-scale
objects (black, white) hue is meaningless and saturation is noisy, so those channels are opened up
and the box is decided by brightness (and low saturation for white).

Detectors never guess: no matching component -> None.
SceneStillness reports how long consecutive frames have been nearly identical (no hand moving).
"""

from __future__ import annotations

import json
import math
from dataclasses import asdict, dataclass
from pathlib import Path

import cv2
import numpy as np

CAP_RADIUS_M = 0.020      # egg cap sphere radius
CUP_SIDE_M = 0.028        # pedestal cup square side
DARK_V = 80               # sampled 95th-percentile brightness below this -> treat as black
GREY_S = 50               # sampled 95th-percentile saturation below this -> treat as white/grey


@dataclass
class EggColour:      # kept for compatibility: any HSV colour box
    h_lo: int = 0
    h_hi: int = 179
    s_lo: int = 0
    s_hi: int = 60
    v_lo: int = 170
    v_hi: int = 255

    @classmethod
    def sample(cls, rgb: np.ndarray, px, radius_px: int = 12, margin=(8, 40, 45)) -> "EggColour":
        hsv = cv2.cvtColor(rgb, cv2.COLOR_RGB2HSV)
        mask = np.zeros(hsv.shape[:2], np.uint8)
        cv2.circle(mask, (int(px[0]), int(px[1])), radius_px, 255, -1)
        vals = hsv[mask > 0].astype(int)
        if len(vals) < 20:
            raise ValueError("Click closer to the centre of the object")
        lo, hi = np.percentile(vals, 5, axis=0), np.percentile(vals, 95, axis=0)
        dh, ds, dv = margin
        h_lo, h_hi = int(max(0, lo[0] - dh)), int(min(179, hi[0] + dh))
        s_lo, s_hi = int(max(0, lo[1] - ds)), int(min(255, hi[1] + ds))
        if hi[2] < DARK_V:                       # black: hue and saturation are noise; use brightness only
            h_lo, h_hi, s_lo, s_hi = 0, 179, 0, 255
        elif hi[1] < GREY_S:                     # white/grey: any hue, low saturation
            h_lo, h_hi, s_lo = 0, 179, 0
        return cls(h_lo, h_hi, s_lo, s_hi, int(max(0, lo[2] - dv)), int(min(255, hi[2] + dv)))

    def mask(self, rgb: np.ndarray) -> np.ndarray:
        hsv = cv2.cvtColor(rgb, cv2.COLOR_RGB2HSV)
        return cv2.inRange(hsv, (self.h_lo, self.s_lo, self.v_lo), (self.h_hi, self.s_hi, self.v_hi))


Colour = EggColour


@dataclass
class Detection:
    px: tuple[float, float]
    area_px: float
    expected_area_px: float
    circularity: float
    confidence: float
    n_candidates: int
    # Radius supported by the filled colour component; None for edge proposals.
    inscribed_radius_px: float | None = None


class BlobDetector:
    def __init__(self, colour: Colour, expected_area_px: float, roi_polygon_px=None, area_ratio=(0.35, 2.2),
                 min_confidence: float = 0.5, shape: str = "circle"):
        self.colour, self.expected_area = colour, expected_area_px
        self.roi = None if roi_polygon_px is None else np.asarray(roi_polygon_px, np.int32)
        self.area_ratio, self.min_confidence, self.shape = area_ratio, min_confidence, shape
        self.kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (5, 5))

    def detect(self, rgb: np.ndarray, near_px=None) -> Detection | None:
        m = self.colour.mask(rgb)
        if self.roi is not None:
            roi = np.zeros_like(m)
            cv2.fillPoly(roi, [self.roi], 255)
            m &= roi
        m = cv2.morphologyEx(cv2.morphologyEx(m, cv2.MORPH_OPEN, self.kernel), cv2.MORPH_CLOSE, self.kernel)
        n, labels, stats, cents = cv2.connectedComponentsWithStats(m, 8)
        lo, hi = (r * self.expected_area for r in self.area_ratio)
        best, best_score, count = None, -1.0, 0
        ideal = 1.0 if self.shape == "circle" else math.pi / 4
        for i in range(1, n):
            area = float(stats[i, cv2.CC_STAT_AREA])
            if not lo <= area <= hi:
                continue
            count += 1
            contours, _ = cv2.findContours((labels == i).astype(np.uint8), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            perim = max(cv2.arcLength(contours[0], True), 1.0)
            circ = float(min(1.0, 4 * math.pi * area / perim ** 2))
            shape_score = float(max(0.0, 1 - abs(circ - ideal) / ideal))
            size = float(math.exp(-abs(math.log(area / self.expected_area))))
            conf = 0.6 * shape_score + 0.4 * size
            centre = cents[i]
            inscribed_radius = None
            if self.shape == "circle":
                # The cap is circular; its stem and attached shadow can pull the
                # component centroid sideways. Centre the largest inscribed disk
                # instead. Fill specular holes before the distance transform.
                x, y, w, h = stats[i, :4]
                filled = np.zeros((h + 2, w + 2), np.uint8)
                local = [c - np.array([[[x - 1, y - 1]]]) for c in contours]
                cv2.drawContours(filled, local, -1, 255, cv2.FILLED)
                dist = cv2.distanceTransform(filled, cv2.DIST_L2, cv2.DIST_MASK_PRECISE)
                inscribed_radius = float(dist.max())
                ys, xs = np.nonzero(dist >= dist.max() - 0.25)
                centre = np.array([xs.mean() + x - 1, ys.mean() + y - 1])
            score = conf - (math.dist(centre, near_px) / (8 * math.sqrt(self.expected_area)) if near_px is not None else 0)
            if score > best_score:
                best_score = score
                best = Detection((float(centre[0]), float(centre[1])), area, self.expected_area,
                                 circ, conf, 0, inscribed_radius)
        if best is None or best.confidence < self.min_confidence:
            return None
        best.n_candidates = count
        return best


class EggDetector(BlobDetector):
    def __init__(self, colour: Colour, pixels_per_m: float, **kw):
        super().__init__(colour, math.pi * (CAP_RADIUS_M * pixels_per_m) ** 2, shape="circle", **kw)

    def detect(self, rgb: np.ndarray, near_px=None) -> Detection | None:
        """Keep supported cap centres; recover a black cap fragmented by reflections.

        A specular highlight can cut the black mask into irregular pieces even
        though the cap's outer edge is visible. Circle proposals alone are not
        detections: require sampled black pixels, a dark interior, a brighter
        surrounding ring, and contrast around most of the proposed boundary.
        This does not solve an egg merged into a dark background.
        """
        blob = super().detect(rgb, near_px)
        c = self.colour
        if not (c.h_lo == 0 and c.h_hi == 179 and c.s_lo == 0 and
                c.s_hi == 255 and c.v_lo == 0 and c.v_hi <= 120):
            return blob  # This recovery is for the sampled black egg only.
        expected_radius = math.sqrt(self.expected_area / math.pi)
        # A bright reflection can leave an accepted dark crescent joined to the
        # stem. Its largest inscribed disk then points at the crescent/stem,
        # not the cap centre. Require support spanning most of the cap radius;
        # otherwise use the same independently checked outer-edge recovery as
        # for a miss. Do not fall back to the unsupported blob if recovery fails.
        if (blob is not None and blob.inscribed_radius_px is not None and
                blob.inscribed_radius_px >= .75 * expected_radius):
            return blob
        scale = min(1.0, 640.0 / rgb.shape[1])
        small = cv2.resize(rgb, (round(rgb.shape[1] * scale), round(rgb.shape[0] * scale)),
                           interpolation=cv2.INTER_AREA) if scale < 1 else rgb
        gray = cv2.cvtColor(small, cv2.COLOR_RGB2GRAY)
        expected_r = math.sqrt(self.expected_area / math.pi) * scale
        if expected_r < 8:
            return None
        circles = cv2.HoughCircles(cv2.GaussianBlur(gray, (5, 5), 1), cv2.HOUGH_GRADIENT,
                                  dp=1, minDist=expected_r, param1=100, param2=25,
                                  minRadius=max(6, round(0.65 * expected_r)),
                                  maxRadius=round(1.2 * expected_r))
        if circles is None:
            return None
        mask = c.mask(small) > 0
        roi = np.ones(gray.shape, np.uint8)
        if self.roi is not None:
            roi.fill(0)
            polygon = np.rint((self.roi + .5) * scale - .5).astype(np.int32)
            cv2.fillPoly(roi, [polygon], 1)
        angles = np.arange(72) * (2 * math.pi / 72)
        directions = np.stack([np.cos(angles), np.sin(angles)], axis=1)
        candidates = []
        for cx, cy, radius in circles[0]:
            # Need the whole ring: a partial frame-edge object is ambiguous.
            extent = int(math.ceil(1.4 * radius))
            x0, y0 = int(cx) - extent, int(cy) - extent
            x1, y1 = int(cx) + extent + 2, int(cy) + extent + 2
            if x0 < 0 or y0 < 0 or x1 > gray.shape[1] or y1 > gray.shape[0]:
                continue
            yy, xx = np.ogrid[y0:y1, x0:x1]
            distance2 = (xx - cx) ** 2 + (yy - cy) ** 2
            inside = distance2 <= (0.9 * radius) ** 2
            ring = (distance2 >= (1.15 * radius) ** 2) & (distance2 <= (1.4 * radius) ** 2)
            if np.mean(roi[y0:y1, x0:x1][inside]) < .95:
                continue
            dark_fraction = float(np.mean(mask[y0:y1, x0:x1][inside]))
            inner_level = float(np.median(gray[y0:y1, x0:x1][inside]))
            ring_level = float(np.median(gray[y0:y1, x0:x1][ring]))
            if dark_fraction < .3 or inner_level > c.v_hi + 30 or ring_level - inner_level < 30:
                continue
            inner = np.rint([cx, cy] + .85 * radius * directions).astype(int)
            outer = np.rint([cx, cy] + 1.15 * radius * directions).astype(int)
            difference = (gray[outer[:, 1], outer[:, 0]].astype(float)
                          - gray[inner[:, 1], inner[:, 0]].astype(float))
            edge_support = float(np.mean(difference > 15))
            if edge_support < .55:
                continue
            size_score = min(radius / expected_r, expected_r / radius)
            confidence = float(.6 * edge_support + .2 * dark_fraction + .2 * size_score)
            if confidence < self.min_confidence:
                continue
            centre = (np.asarray([cx, cy]) + .5) / scale - .5
            score = confidence - (math.dist(centre, near_px) / (8 * math.sqrt(self.expected_area))
                                  if near_px is not None else 0)
            # Circularity describes the proposed disk, not a reconstructed mask.
            found = Detection(tuple(float(v) for v in centre), float(math.pi * (radius / scale) ** 2),
                              self.expected_area, 1.0, confidence, 0)
            candidates.append((score, found))
        if not candidates:
            return None
        candidates.sort(key=lambda item: item[0], reverse=True)
        if len(candidates) > 1 and candidates[0][0] - candidates[1][0] < .1:
            return None  # Do not choose arbitrarily between similar caps.
        best = candidates[0][1]
        best.n_candidates = len(candidates)
        return best


class PedestalDetector(BlobDetector):
    """White cup seen from above (visible only when the egg is not on it)."""

    def __init__(self, colour: Colour, pixels_per_m: float, **kw):
        kw.setdefault("area_ratio", (0.4, 1.8))
        super().__init__(colour, (CUP_SIDE_M * pixels_per_m) ** 2, shape="square", **kw)


class SceneStillness:
    """Seconds for which consecutive frames have been nearly identical: the 99.5th percentile of the
    absolute grey-level change (160x90 thumbnail) stays below `threshold`. A percentile, not a mean,
    so a hand moving in a small part of the image still counts as motion."""

    def __init__(self, threshold: float = 15.0, size=(160, 90), percentile: float = 99.5):
        self.threshold, self.size, self.percentile = threshold, size, percentile
        self.prev = None
        self.still_since = None
        self.last_level = None

    def update(self, rgb: np.ndarray, t: float) -> float:
        g = cv2.resize(cv2.cvtColor(rgb, cv2.COLOR_RGB2GRAY), self.size, interpolation=cv2.INTER_AREA).astype(np.float32)
        if self.prev is not None:
            self.last_level = float(np.percentile(np.abs(g - self.prev), self.percentile))
            if self.last_level < self.threshold:
                self.still_since = t if self.still_since is None else self.still_since
            else:
                self.still_since = None
        self.prev = g
        return 0.0 if self.still_since is None else t - self.still_since

    def reset(self):
        self.prev = self.still_since = self.last_level = None


def load_settings(path: Path) -> dict:
    return json.loads(path.read_text()) if path.is_file() else {}


def save_settings(path: Path, colour: Colour | None = None, extra: dict | None = None, key: str = "colour") -> None:
    data = load_settings(path)
    if colour is not None:
        data[key] = asdict(colour)
    data.update(extra or {})
    tmp = path.with_suffix(".tmp")
    tmp.write_text(json.dumps(data, indent=2))
    tmp.replace(path)

"""Simple, reviewable colour detectors for the overhead camera (no learning).

Recommended print colours (contrast with the white robot and brown cardboard floor):
  egg      saturated green  (sim rgba 0.16 0.67 0.31, #29AB4F)
  pedestal saturated blue   (sim rgba 0.12 0.35 0.78, #1F59C7)
Each colour model is an HSV box sampled by the operator clicking the object once.

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
        return cls(int(max(0, lo[0] - dh)), int(min(179, hi[0] + dh)), int(max(0, lo[1] - ds)),
                   int(min(255, hi[1] + ds)), int(max(0, lo[2] - dv)), int(min(255, hi[2] + dv)))

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
            score = conf - (math.dist(cents[i], near_px) / (8 * math.sqrt(self.expected_area)) if near_px is not None else 0)
            if score > best_score:
                best_score = score
                best = Detection((float(cents[i][0]), float(cents[i][1])), area, self.expected_area, circ, conf, 0)
        if best is None or best.confidence < self.min_confidence:
            return None
        best.n_candidates = count
        return best


class EggDetector(BlobDetector):
    def __init__(self, colour: Colour, pixels_per_m: float, **kw):
        super().__init__(colour, math.pi * (CAP_RADIUS_M * pixels_per_m) ** 2, shape="circle", **kw)


class PedestalDetector(BlobDetector):
    """Blue cup seen from above (visible only when the egg is not on it)."""

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

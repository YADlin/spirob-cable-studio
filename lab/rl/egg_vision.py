"""Simple, reviewable egg detector for the overhead camera (no learning).

Colour model: an HSV box sampled by the operator clicking the egg once
(``EggColour.sample``) and saved to ``rl_vision.json``. Detection keeps
connected components inside a pixel ROI whose area matches the expected egg
cap size (derived from pixels_per_m), and reports a confidence in [0, 1].

The detector never guesses: no component -> ``None`` (the runner treats this
as UNKNOWN, never as success).
"""

from __future__ import annotations

import json
import math
from dataclasses import asdict, dataclass
from pathlib import Path

import cv2
import numpy as np

CAP_RADIUS_M = 0.020  # spirob-mjlab egg cap sphere radius


@dataclass
class EggColour:
    h_lo: int = 0
    h_hi: int = 179
    s_lo: int = 0
    s_hi: int = 60
    v_lo: int = 170
    v_hi: int = 255

    @classmethod
    def sample(cls, rgb: np.ndarray, px, radius_px: int = 12, margin=(8, 40, 45)) -> "EggColour":
        """Fit an HSV box to a disc around a clicked egg pixel."""
        hsv = cv2.cvtColor(rgb, cv2.COLOR_RGB2HSV)
        mask = np.zeros(hsv.shape[:2], np.uint8)
        cv2.circle(mask, (int(px[0]), int(px[1])), radius_px, 255, -1)
        vals = hsv[mask > 0].astype(int)
        if len(vals) < 20:
            raise ValueError("Click closer to the centre of the egg")
        lo = np.percentile(vals, 5, axis=0)
        hi = np.percentile(vals, 95, axis=0)
        dh, ds, dv = margin
        return cls(int(max(0, lo[0] - dh)), int(min(179, hi[0] + dh)), int(max(0, lo[1] - ds)),
                   int(min(255, hi[1] + ds)), int(max(0, lo[2] - dv)), int(min(255, hi[2] + dv)))

    def mask(self, rgb: np.ndarray) -> np.ndarray:
        hsv = cv2.cvtColor(rgb, cv2.COLOR_RGB2HSV)
        return cv2.inRange(hsv, (self.h_lo, self.s_lo, self.v_lo), (self.h_hi, self.s_hi, self.v_hi))


@dataclass
class Detection:
    px: tuple[float, float]
    area_px: float
    expected_area_px: float
    circularity: float
    confidence: float
    n_candidates: int


class EggDetector:
    def __init__(self, colour: EggColour, pixels_per_m: float, roi_polygon_px=None,
                 area_ratio=(0.35, 2.2), min_confidence: float = 0.5):
        self.colour = colour
        self.expected_area = math.pi * (CAP_RADIUS_M * pixels_per_m) ** 2
        self.roi = None if roi_polygon_px is None else np.asarray(roi_polygon_px, np.int32)
        self.area_ratio = area_ratio
        self.min_confidence = min_confidence
        self.kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (5, 5))

    def detect(self, rgb: np.ndarray, near_px=None) -> Detection | None:
        m = self.colour.mask(rgb)
        if self.roi is not None:
            roi = np.zeros_like(m)
            cv2.fillPoly(roi, [self.roi], 255)
            m &= roi
        m = cv2.morphologyEx(m, cv2.MORPH_OPEN, self.kernel)
        m = cv2.morphologyEx(m, cv2.MORPH_CLOSE, self.kernel)
        n, labels, stats, cents = cv2.connectedComponentsWithStats(m, 8)
        lo, hi = (r * self.expected_area for r in self.area_ratio)
        best, best_score, count = None, -1.0, 0
        for i in range(1, n):
            area = float(stats[i, cv2.CC_STAT_AREA])
            if not lo <= area <= hi:
                continue
            count += 1
            comp = (labels == i).astype(np.uint8)
            contours, _ = cv2.findContours(comp, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            perim = max(cv2.arcLength(contours[0], True), 1.0)
            circ = float(min(1.0, 4 * math.pi * area / perim**2))
            size = float(math.exp(-abs(math.log(area / self.expected_area))))
            conf = 0.6 * circ + 0.4 * size
            score = conf
            if near_px is not None:
                d = math.dist(cents[i], near_px)
                score -= d / (8 * math.sqrt(self.expected_area))
            if score > best_score:
                best_score, best = score, Detection((float(cents[i][0]), float(cents[i][1])), area,
                                                     self.expected_area, circ, conf, 0)
        if best is None or best.confidence < self.min_confidence:
            return None
        best.n_candidates = count
        return best


def load_settings(path: Path) -> dict:
    return json.loads(path.read_text()) if path.is_file() else {}


def save_settings(path: Path, colour: EggColour, extra: dict | None = None) -> None:
    data = load_settings(path)
    data["colour"] = asdict(colour)
    data.update(extra or {})
    tmp = path.with_suffix(".tmp")
    tmp.write_text(json.dumps(data, indent=2))
    tmp.replace(path)

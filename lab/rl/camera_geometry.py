"""Overhead camera geometry, shared verbatim between the Lab and simulation.

The similarity calibration is on the robot plane. Egg blob coordinates use the
upright cap centre (87 mm), not the top (107 mm); the empty cup uses 46 mm.
"""
from __future__ import annotations

import hashlib
import json
from dataclasses import dataclass

import numpy as np


@dataclass(frozen=True)
class CameraMap:
    """Similarity calibration (robot plane) plus a pinhole parallax correction for the egg.

    The Lab's alignment maps frame-C points ON THE ROBOT PLANE (z = plane_z) to
    pixels: px = s R [u, v] + t. The egg feature seen from above sits at
    z = feature_z. For a pinhole camera at height H above the table, points at a
    different height appear scaled about the principal point by
    (H - plane_z) / (H - feature_z). We undo that scale.
    """
    pixels_per_m: float
    rotation_rad: float
    translation_px: tuple[float, float]
    plane_z_m: float = 0.09
    camera_height_m: float | None = None       # MEASURE on the rig; None disables the correction
    principal_point_px: tuple[float, float] | None = None
    feature_z_m: float = 0.087                 # egg cap centre height on the pedestal

    @classmethod
    def from_lab(cls, setup: dict, rl_vision: dict | None = None, plane_z_m: float = 0.09) -> "CameraMap":
        v = rl_vision or {}
        pp = v.get("principal_point_px")
        return cls(float(setup["pixels_per_m"]), float(setup["rotation_rad"]),
                   (float(setup["translation_x_px"]), float(setup["translation_y_px"])), plane_z_m,
                   v.get("camera_height_m"), tuple(pp) if pp else None, float(v.get("egg_feature_z_m", 0.087)))

    def _R(self):
        c, s = np.cos(self.rotation_rad), np.sin(self.rotation_rad)
        return np.array([[c, -s], [s, c]])

    def c_to_px(self, uv) -> np.ndarray:
        return self.pixels_per_m * (np.asarray(uv, dtype=float) @ self._R().T) + np.asarray(self.translation_px)

    def px_to_c_plane(self, px) -> np.ndarray:
        return ((np.asarray(px, dtype=float) - np.asarray(self.translation_px)) @ self._R()) / self.pixels_per_m

    def px_to_c(self, px, feature_z_m: float | None = None) -> np.ndarray:
        """Pixel -> frame C at the egg feature height (parallax-corrected when calibrated)."""
        uv = self.px_to_c_plane(px)
        if self.camera_height_m is None or self.principal_point_px is None:
            return uv
        z = self.feature_z_m if feature_z_m is None else feature_z_m
        H = self.camera_height_m
        if not H > max(z, self.plane_z_m) + 0.05:
            raise ValueError("Camera height must exceed the feature and plane heights")
        uv_pp = self.px_to_c_plane(self.principal_point_px)
        return uv_pp + (uv - uv_pp) * (H - z) / (H - self.plane_z_m)

    def pixels_per_m_at(self, feature_z_m: float) -> float:
        if self.camera_height_m is None:
            return self.pixels_per_m
        if self.camera_height_m <= max(feature_z_m, self.plane_z_m) + 0.05:
            raise ValueError("Camera height must exceed the feature and plane heights")
        return self.pixels_per_m * (self.camera_height_m - self.plane_z_m) / (self.camera_height_m - feature_z_m)

    def fingerprint(self) -> str:
        return hashlib.sha256(json.dumps(self.__dict__, sort_keys=True, default=list).encode()).hexdigest()[:16]

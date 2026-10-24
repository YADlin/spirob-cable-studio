"""Hardware boundary for the RL mode: model metres <-> physical millimetres, camera pixels <-> frame C.

Everything here reuses the Lab's existing calibration as the authority:
  * cable mapping/signs and image alignment: ``comparison_setup`` (Alignment / cable mapping dialog)
  * mean/difference offsets: ``fit['candidate']['parameters']``
  * physical rest references and bounds: the session motion profile
The forward cable map is identical to ``Comparator.controls`` (analysis.py) and
``motion_settings.model_bounds``; tests/test_rl_knockdown.py checks this.

No motor or camera I/O happens in this module.
"""

from __future__ import annotations

import hashlib
import json
from dataclasses import dataclass

import numpy as np


@dataclass(frozen=True)
class CableMap:
    mapping: tuple[int, int]          # model cable j -> physical axis mapping[j]
    signs: tuple[int, int]
    rests_mm: tuple[float, float]     # physical rest reference per physical axis
    neutral_m: tuple[float, float]    # model neutral payout per model cable
    mean_offset_mm: float = 0.0
    difference_offset_mm: float = 0.0

    @classmethod
    def from_lab(cls, setup: dict, parameters: dict, rests_mm, neutral_m) -> "CableMap":
        mapping = tuple(int(v) for v in setup.get("mapping", [0, 1]))
        signs = tuple(int(v) for v in setup.get("signs", [1, 1]))
        if sorted(mapping) != [0, 1] or any(s not in (-1, 1) for s in signs):
            raise ValueError("Invalid cable mapping/signs in the Lab setup")
        return cls(mapping, signs, tuple(float(r) for r in rests_mm), tuple(float(n) for n in neutral_m),
                   float(parameters.get("mean_offset_mm", 0.0)), float(parameters.get("difference_offset_mm", 0.0)))

    @property
    def offsets_mm(self) -> np.ndarray:
        m, d = self.mean_offset_mm, self.difference_offset_mm
        return np.array([m + d / 2, m - d / 2])

    def model_to_physical_mm(self, model_m) -> np.ndarray:
        """Model-order payout [m] -> physical-axis lengths [mm] (inverse of Comparator.controls)."""
        model_m = np.asarray(model_m, dtype=float)
        out = np.empty(2)
        for j in range(2):
            p = self.mapping[j]
            out[p] = self.rests_mm[p] + self.signs[j] * (1000.0 * (model_m[j] - self.neutral_m[j]) - self.offsets_mm[j])
        return out

    def physical_to_model_m(self, physical_mm) -> np.ndarray:
        """Physical-axis lengths [mm] -> model-order payout [m] (exactly Comparator.controls)."""
        physical_mm = np.asarray(physical_mm, dtype=float)
        disp = (physical_mm - np.asarray(self.rests_mm))[list(self.mapping)] * np.asarray(self.signs)
        return np.asarray(self.neutral_m) + (disp + self.offsets_mm) / 1000.0

    def fingerprint(self) -> str:
        return hashlib.sha256(json.dumps(self.__dict__, sort_keys=True).encode()).hexdigest()[:16]


# Re-export for existing Lab callers. Shared verbatim with simulation.
from rl.camera_geometry import CameraMap  # noqa: E402,F401

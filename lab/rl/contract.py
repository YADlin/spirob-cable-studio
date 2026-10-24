"""Deployable policy contract v2 for SpiRob egg knockdown (camera + motor encoders only).

Single definition of what the actor sees and what its output means. Imported by the mjlab task
and copied verbatim into the SpiRob Lab (``lab/rl/contract.py``); the Lab refuses a policy whose
export records a different SHA-256 for this file.

Dependencies: Python standard library at import time; array helpers accept NumPy arrays or torch
tensors (duck-typed) so simulation and hardware run the same arithmetic.

Units: SI (m, s). Millimetres appear only at the hardware boundary in the Lab.

What changed from v1 (24 Oct, NYK): one action = the cable DIFFERENCE D = L0 - L1 at a fixed
mean payout (the motion the knockdown model was validated on, D in [-80, +80] mm); speed is a
deployment setting (5, then 10, then 15 mm/s per cable) with small moves slowed down; the egg may
leave the camera frame (actor gets a visibility flag and the last seen position); episodes may
start from any robot shape (the robot stops where it knocked the egg and continues from there).

Frames
------
Base frame C = the Lab's calibrated planar frame: origin at joint link_002 at rest, u = model +x
(lateral), v = model -y (from the mount toward the tip at rest). Sign: D > 0 curls toward -u.

Decision (waypoint semantics, matching the Lab motor owner)
-----------------------------------------------------------
  1. observe: settled encoders + fresh camera frame
  2. raw action a -> clipped a in [-1, 1]
  3. desired D  = 80 mm * a
  4. limited D  = prev D + clip(desired - prev, +-8 mm); |limited - prev| < 0.5 mm -> no move
  5. cables move to (mean + D/2, mean - D/2), each at a speed proportional to the step:
        speed = clip(v_max * |D step| / 8 mm, 0.5 mm/s, v_max)      (v_max: 5 | 10 | 15 mm/s)
     so every move takes the same time (4 mm / v_max) and a 1 mm step at 15 mm/s runs at 1.9 mm/s
  6. wait for both axes to hold; next decision
"""

from __future__ import annotations

import hashlib
import json
from dataclasses import asdict, dataclass, field
from pathlib import Path

CONTRACT_ID = "spirob-egg-knockdown/contract-v2"

# Knockdown model (assets/knockdown). Recomputed from the XML by tests.
SOURCE_MODEL_SHA256 = "fe7d13cd476a"  # prefix of NYK's spirob_robot_k2_2.148_20261024_0318.xml (Lab model)
MEAN_PAYOUT_M = 0.2273533144104711
C_ORIGIN_MODEL_XY = (0.0, 0.21796009)


@dataclass(frozen=True)
class ActionSpec:
    semantics: str = "difference_waypoint_v2"
    raw_clip: float = 1.0
    mean_m: float = MEAN_PAYOUT_M
    diff_limit_m: float = 0.080
    max_diff_step_m: float = 0.008
    deadband_diff_m: float = 0.0005
    min_cable_speed_m_s: float = 0.0005
    cable_speed_max_m_s: float = 0.005       # deployment default (NYK: 5, then 10, then 15 mm/s)
    train_cable_speed_range_m_s: tuple[float, float] = (0.0045, 0.015)
    decision_period_s: float = 1.2           # sim; the rig takes move + settle (logged)


@dataclass(frozen=True)
class CameraSpec:
    """Overhead camera (provisional geometry; the Lab's alignment is the authority on the rig)."""
    height_m: float = 0.321
    centre_uv_m: tuple[float, float] = (0.0, 0.128)   # 14 cm from the base point (link_001, v = -0.012)
    visible_half_extent_uv_m: tuple[float, float] = (0.20, 0.118)  # 1920x1080 at ~4500 px/m; recheck after re-alignment
    egg_bias_m: float = 0.003
    egg_noise_m: float = 0.001


@dataclass(frozen=True)
class ObservationTerm:
    name: str
    size: int
    source: str            # camera | encoder | policy_memory
    scale: float           # obs = SI value / scale
    description: str


PERMITTED_ACTOR_SOURCES = frozenset({"camera", "encoder", "policy_memory"})

ACTOR_TERMS: tuple[ObservationTerm, ...] = (
    ObservationTerm("egg_uv", 2, "camera", 0.10, "Egg position in frame C; last seen position when not visible [m]"),
    ObservationTerm("egg_visible", 1, "camera", 1.0, "1 if the egg is detected in the current frame, else 0"),
    ObservationTerm("egg_uv_start", 2, "camera", 0.10, "Egg position latched at episode start [m]"),
    ObservationTerm("diff", 1, "encoder", 0.080, "Measured cable difference D = payout0 - payout1 [m]"),
    ObservationTerm("prev_action", 1, "policy_memory", 1.0, "Previous clipped action (start: measured D / 80 mm)"),
)


@dataclass(frozen=True)
class SuccessSpec:
    sim_support_top_z_m: float = 0.046       # knockdown pedestal cup top
    sim_fall_margin_m: float = 0.005
    hw_off_radius_m: float = 0.035           # egg seen this far from its start ...
    hw_confirm_s: float = 0.5                # ... for this long
    hw_empty_cup_s: float = 0.7              # or: blue cup visible at the start, no egg near it, this long
    hw_cup_match_m: float = 0.015


@dataclass(frozen=True)
class AutoStartSpec:
    """Hardware: after a knockdown the robot holds; the next episode starts when a new egg has been
    placed and the scene has been still (no hand) for still_s."""
    still_s: float = 2.0
    egg_stable_m: float = 0.002
    clearance_m: float = 0.025               # egg centre to robot surface in the predicted shape


@dataclass(frozen=True)
class Contract:
    contract_id: str = CONTRACT_ID
    action: ActionSpec = field(default_factory=ActionSpec)
    camera: CameraSpec = field(default_factory=CameraSpec)
    success: SuccessSpec = field(default_factory=SuccessSpec)
    autostart: AutoStartSpec = field(default_factory=AutoStartSpec)
    episode_decisions: int = 30
    source_model_sha256_prefix: str = SOURCE_MODEL_SHA256
    c_origin_model_xy: tuple[float, float] = C_ORIGIN_MODEL_XY

    @property
    def actor_terms(self):
        return ACTOR_TERMS

    @property
    def actor_dim(self) -> int:
        return sum(t.size for t in ACTOR_TERMS)

    def to_json(self) -> dict:
        d = asdict(self)
        d.update(actor_terms=[asdict(t) for t in ACTOR_TERMS], actor_dim=self.actor_dim, action_dim=1,
                 permitted_actor_sources=sorted(PERMITTED_ACTOR_SOURCES))
        return d


DEFAULT = Contract()


def source_file_sha256() -> str:
    return hashlib.sha256(Path(__file__).read_bytes()).hexdigest()


def validate_actor_schema(terms=ACTOR_TERMS) -> None:
    bad = [t.name for t in terms if t.source not in PERMITTED_ACTOR_SOURCES]
    if bad:
        raise ValueError(f"Actor terms use non-permitted sensing sources: {bad}")


# ---------------------------------------------------------------- array helpers (NumPy or torch)
def _is_torch(x) -> bool:
    return hasattr(x, "clamp")


def _clip(x, lo, hi):
    if _is_torch(x):
        if _is_torch(lo) or _is_torch(hi):   # per-env bounds (e.g. sampled speed limit)
            import torch
            lo_t = torch.as_tensor(lo, dtype=x.dtype, device=x.device)
            hi_t = torch.as_tensor(hi, dtype=x.dtype, device=x.device)
            return torch.minimum(torch.maximum(x, lo_t), hi_t)
        return x.clamp(min=lo, max=hi)
    import numpy as np
    return np.clip(x, lo, hi)


def _abs(x):
    if _is_torch(x):
        return x.abs()
    import numpy as np
    return np.abs(x)


def _where(c, a, b):
    if _is_torch(c):
        import torch
        return torch.where(c, a, b)
    import numpy as np
    return np.where(c, a, b)


# ---------------------------------------------------------------- frames
def model_xy_to_c(x, y, origin=C_ORIGIN_MODEL_XY):
    return x - origin[0], origin[1] - y


def c_to_model_xy(u, v, origin=C_ORIGIN_MODEL_XY):
    return u + origin[0], origin[1] - v


# ---------------------------------------------------------------- action transform
def clip_raw(raw, spec: ActionSpec = DEFAULT.action):
    return _clip(raw, -spec.raw_clip, spec.raw_clip)


def desired_diff(raw, spec: ActionSpec = DEFAULT.action):
    return clip_raw(raw, spec) * spec.diff_limit_m


def limited_diff(desired, prev, spec: ActionSpec = DEFAULT.action):
    """Step-limit, bound, and suppress sub-deadband moves (target stays at prev)."""
    step = _clip(desired - prev, -spec.max_diff_step_m, spec.max_diff_step_m)
    out = _clip(prev + step, -spec.diff_limit_m, spec.diff_limit_m)
    return _where(_abs(out - prev) < spec.deadband_diff_m, prev, out)


def payouts_from_diff(diff, spec: ActionSpec = DEFAULT.action):
    """(..., 1) or (...) difference -> (..., 2) payouts [m], model cable order."""
    if _is_torch(diff):
        import torch
        d = diff if diff.dim() and diff.shape[-1] == 1 else diff.unsqueeze(-1)
        return torch.cat((spec.mean_m + d / 2, spec.mean_m - d / 2), dim=-1)
    import numpy as np
    d = np.asarray(diff, dtype=float)
    if d.ndim == 0 or d.shape[-1] != 1:
        d = d[..., None]
    return np.concatenate((spec.mean_m + d / 2, spec.mean_m - d / 2), axis=-1)


def diff_from_payouts(payout):
    return payout[..., 0:1] - payout[..., 1:2]


def cable_speed(step_diff, vmax_m_s, spec: ActionSpec = DEFAULT.action):
    """Per-cable speed for a decision whose difference changes by step_diff [m]: proportional to the
    step, reaching vmax only for a full max_diff_step move (all moves take ~equal time)."""
    return _clip(_abs(step_diff) / spec.max_diff_step_m * vmax_m_s, spec.min_cable_speed_m_s, vmax_m_s)


# ---------------------------------------------------------------- observation assembly
def build_actor_obs(egg_uv, egg_visible, egg_uv_start, diff, prev_action, spec: Contract = DEFAULT):
    t = {term.name: term for term in ACTOR_TERMS}
    parts = (egg_uv / t["egg_uv"].scale, egg_visible / t["egg_visible"].scale, egg_uv_start / t["egg_uv_start"].scale,
             diff / t["diff"].scale, prev_action / t["prev_action"].scale)
    if _is_torch(diff):
        import torch
        return torch.cat(parts, dim=-1)
    import numpy as np
    return np.concatenate([np.atleast_1d(np.asarray(p, dtype=float)) for p in parts], axis=-1)


# ---------------------------------------------------------------- predicted robot shape (pose table)
def load_pose_table(path) -> dict:
    return json.loads(Path(path).read_text())


def robot_clearance(point_uv, diff_m, table: dict) -> float:
    """Distance from a point to the robot outline (link centres minus half-widths) at the tabulated
    shape nearest to diff_m. Negative = inside the robot. NumPy only."""
    import numpy as np
    rows = table["rows"]
    k = int(np.argmin([abs(r["diff_mm"] / 1000 - diff_m) for r in rows]))
    pts = np.asarray(rows[k]["link_uv"])
    hw = np.asarray(table["link_halfwidth_m"])
    return float(np.min(np.linalg.norm(pts - np.asarray(point_uv), axis=1) - hw))


def write_contract_json(path, spec: Contract = DEFAULT) -> dict:
    data = spec.to_json()
    data["contract_source_sha256"] = source_file_sha256()
    Path(path).write_text(json.dumps(data, indent=2, sort_keys=True))
    return data


if __name__ == "__main__":
    validate_actor_schema()
    print(json.dumps(DEFAULT.to_json(), indent=2))
    print("source sha256:", source_file_sha256())

"""Deployable policy contract for SpiRob egg knockdown (camera + motor encoders only).

This file is the single definition of what the actor sees and what its output
means. It is imported by the mjlab task AND copied verbatim into the SpiRob Lab
(``lab/rl/contract.py``). The exported policy manifest stores this file's
SHA-256; the Lab refuses to run a policy whose contract hash differs.

Dependencies: Python standard library only. Every array function accepts either
NumPy arrays or torch tensors (duck-typed), so simulation and hardware execute
the same arithmetic.

Units: SI (metres, seconds) everywhere in this file. Millimetres appear only at
the hardware boundary in the Lab (``lab/rl/hardware_io.py``).

Frames
------
Base frame C (the Lab's calibrated planar frame, ``spirob_analysis.model``):
  origin  = joint ``link_002`` at the rest pose, in the robot's bending plane
  u axis  = model-world +x (lateral)
  v axis  = model-world -y (from the mount toward the tip at rest)
The Lab camera calibration maps C to image pixels with a similarity transform
(pixels_per_m, rotation_rad, translation_px). Simulation computes the same C
coordinates from the robot entity origin. Nothing in the actor uses the global
simulator origin.

Cables
------
Model order: index 0 = tendon ``cable_0`` (actuator ``act_c0``), index 1 =
``cable_1``. The Lab maps model order to physical motors with its saved cable
mapping/signs; the contract never assumes physical order.

Timing (waypoint semantics, chosen to match the Lab motor owner)
----------------------------------------------------------------
The Lab's motor worker accepts a new target only when both axes are HOLDING
with three stable encoder samples. A policy decision is therefore one bounded
point-to-point move followed by a settle, not a streamed setpoint:

  1. read observation (settled encoders + fresh camera frame)
  2. policy -> raw action a in [-1, 1]^2
  3. desired  = neutral + half_span * a                (absolute target)
  4. limited  = prev_limited + clip(desired - prev_limited, +-max_step)
  5. both motors move to `limited` at the fixed speed `move_speed` (per axis,
     same speed; the shorter move finishes first), then hold
  6. wait until the decision period has elapsed and the robot has settled

Raw action, desired target, limited command and measured payout are distinct
quantities and are logged separately.
"""

from __future__ import annotations

import hashlib
import json
from dataclasses import asdict, dataclass, field
from pathlib import Path

CONTRACT_ID = "spirob-egg-knockdown/contract-v1"

# ---------------------------------------------------------------------------
# Constants measured from the Lab model (lab/model/spirob_robot.xml,
# sha256 ac8d9f9d...). tests/test_knockdown_contract.py recomputes them from
# the XML and fails if they drift.
# ---------------------------------------------------------------------------
LAB_MODEL_SHA256 = "ac8d9f9dd36cc656228616904dcc64c409f3c17f6d917afe2115f13124f4a310"
NEUTRAL_PAYOUT_M = (0.22735331441047116, 0.2273533144104711)
C_ORIGIN_MODEL_XY = (0.0, 0.21796009)  # link_002 at rest, model world


@dataclass(frozen=True)
class ActionSpec:
    semantics: str = "waypoint_absolute_v1"
    raw_clip: float = 1.0
    neutral_m: tuple[float, float] = NEUTRAL_PAYOUT_M
    half_span_m: float = 0.060          # desired targets: neutral +- 60 mm
    max_step_m: float = 0.008           # per decision, per cable
    move_speed_m_s: float = 0.015       # fixed hardware speed (Lab ceiling 0.020)
    decision_period_s: float = 1.0      # PROVISIONAL until measured on the rig

    @property
    def bounds_m(self) -> tuple[tuple[float, float], tuple[float, float]]:
        return tuple((n - self.half_span_m, n + self.half_span_m) for n in self.neutral_m)  # type: ignore[return-value]


@dataclass(frozen=True)
class ObservationTerm:
    name: str
    size: int
    source: str            # camera | encoder | policy_memory
    scale: float           # obs = raw_SI_value / scale
    description: str


PERMITTED_ACTOR_SOURCES = frozenset({"camera", "encoder", "policy_memory"})

ACTOR_TERMS: tuple[ObservationTerm, ...] = (
    ObservationTerm("egg_uv", 2, "camera", 0.10,
                    "Egg position in base frame C from the overhead camera [m]"),
    ObservationTerm("egg_uv_start", 2, "camera", 0.10,
                    "Egg position in C latched at episode start (pedestal estimate) [m]"),
    ObservationTerm("payout_offset", 2, "encoder", 0.060,
                    "Motor payout minus neutral, model cable order [m]; NOT geometric tendon length"),
    ObservationTerm("prev_action", 2, "policy_memory", 1.0,
                    "Previous clipped raw action (zeros after reset)"),
)


@dataclass(frozen=True)
class SuccessSpec:
    # Simulation (privileged): egg COM below the pedestal support surface, after
    # the robot has touched the egg at least once in this episode.
    sim_support_top_z_m: float = 0.066       # pedestal cup top in the spirob-mjlab asset
    sim_fall_margin_m: float = 0.005
    # Hardware (camera): egg seen outside the pedestal footprint, sustained.
    hw_off_radius_m: float = 0.035
    hw_confirm_s: float = 0.5
    hw_max_frame_age_s: float = 0.5


@dataclass(frozen=True)
class Contract:
    contract_id: str = CONTRACT_ID
    action: ActionSpec = field(default_factory=ActionSpec)
    success: SuccessSpec = field(default_factory=SuccessSpec)
    episode_decisions: int = 24
    lab_model_sha256: str = LAB_MODEL_SHA256
    c_origin_model_xy: tuple[float, float] = C_ORIGIN_MODEL_XY

    @property
    def actor_terms(self) -> tuple[ObservationTerm, ...]:
        return ACTOR_TERMS

    @property
    def actor_dim(self) -> int:
        return sum(t.size for t in ACTOR_TERMS)

    def to_json(self) -> dict:
        d = asdict(self)
        d["actor_terms"] = [asdict(t) for t in ACTOR_TERMS]
        d["actor_dim"] = self.actor_dim
        d["permitted_actor_sources"] = sorted(PERMITTED_ACTOR_SOURCES)
        return d


DEFAULT = Contract()


def source_file_sha256() -> str:
    return hashlib.sha256(Path(__file__).read_bytes()).hexdigest()


def validate_actor_schema(terms=ACTOR_TERMS) -> None:
    bad = [t.name for t in terms if t.source not in PERMITTED_ACTOR_SOURCES]
    if bad:
        raise ValueError(f"Actor terms use non-permitted sensing sources: {bad}")


# ---------------------------------------------------------------------------
# Array helpers (NumPy or torch)
# ---------------------------------------------------------------------------
def _clip(x, lo, hi):
    if hasattr(x, "clamp"):
        return x.clamp(min=lo, max=hi)
    import numpy as np
    return np.clip(x, lo, hi)


# ---------------------------------------------------------------------------
# Frames
# ---------------------------------------------------------------------------
def model_xy_to_c(x, y, origin=C_ORIGIN_MODEL_XY):
    """Model-world planar coordinates -> base frame C (u, v)."""
    return x - origin[0], origin[1] - y


def c_to_model_xy(u, v, origin=C_ORIGIN_MODEL_XY):
    return u + origin[0], origin[1] - v


# ---------------------------------------------------------------------------
# Action transform (identical in sim and on hardware)
# ---------------------------------------------------------------------------
def clip_raw(raw, spec: ActionSpec = DEFAULT.action):
    return _clip(raw, -spec.raw_clip, spec.raw_clip)


def desired_target(raw, spec: ActionSpec = DEFAULT.action):
    """Absolute desired payout [m] from a raw action, model cable order."""
    a = clip_raw(raw, spec)
    n0, n1 = spec.neutral_m
    out = a * spec.half_span_m
    # neutral differs per cable only in the 15th decimal; keep per-cable form.
    if hasattr(out, "clone"):
        out = out.clone()
        out[..., 0] += n0
        out[..., 1] += n1
    else:
        import numpy as np
        out = np.array(out, dtype=float, copy=True)
        out[..., 0] += n0
        out[..., 1] += n1
    return out


def limited_target(desired, prev_limited, spec: ActionSpec = DEFAULT.action):
    """Rate-limit the absolute target by max_step per decision, then bound."""
    step = _clip(desired - prev_limited, -spec.max_step_m, spec.max_step_m)
    out = prev_limited + step
    (lo0, hi0), (lo1, hi1) = spec.bounds_m
    if hasattr(out, "clone"):
        out = out.clone()
        out[..., 0] = out[..., 0].clamp(lo0, hi0)
        out[..., 1] = out[..., 1].clamp(lo1, hi1)
    else:
        import numpy as np
        out = np.array(out, dtype=float, copy=True)
        out[..., 0] = np.clip(out[..., 0], lo0, hi0)
        out[..., 1] = np.clip(out[..., 1], lo1, hi1)
    return out


def move_duration_s(limited, payout_now, spec: ActionSpec = DEFAULT.action) -> float:
    """Longest per-axis travel time at the fixed move speed (NumPy only)."""
    import numpy as np
    return float(np.max(np.abs(np.asarray(limited) - np.asarray(payout_now))) / spec.move_speed_m_s)


# ---------------------------------------------------------------------------
# Observation assembly (identical in sim and on hardware)
# ---------------------------------------------------------------------------
def build_actor_obs(egg_uv, egg_uv_start, payout, prev_action, spec: Contract = DEFAULT):
    """Concatenate actor terms in contract order with contract scaling.

    egg_uv, egg_uv_start: (..., 2) metres in frame C (camera-derived)
    payout: (..., 2) metres, encoder payout in model cable order
    prev_action: (..., 2) clipped raw action
    """
    n = spec.action.neutral_m
    t = {term.name: term for term in ACTOR_TERMS}
    if hasattr(payout, "clone"):
        import torch
        neutral = torch.tensor(n, dtype=payout.dtype, device=payout.device)
        parts = [egg_uv / t["egg_uv"].scale, egg_uv_start / t["egg_uv_start"].scale,
                 (payout - neutral) / t["payout_offset"].scale, prev_action / t["prev_action"].scale]
        return torch.cat(parts, dim=-1)
    import numpy as np
    neutral = np.asarray(n, dtype=float)
    parts = [np.asarray(egg_uv) / t["egg_uv"].scale, np.asarray(egg_uv_start) / t["egg_uv_start"].scale,
             (np.asarray(payout) - neutral) / t["payout_offset"].scale, np.asarray(prev_action) / t["prev_action"].scale]
    return np.concatenate(parts, axis=-1)


def write_contract_json(path: str | Path, spec: Contract = DEFAULT) -> dict:
    data = spec.to_json()
    data["contract_source_sha256"] = source_file_sha256()
    Path(path).write_text(json.dumps(data, indent=2, sort_keys=True))
    return data


if __name__ == "__main__":
    validate_actor_schema()
    print(json.dumps(DEFAULT.to_json(), indent=2))
    print("source sha256:", source_file_sha256())

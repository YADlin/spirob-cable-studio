"""Explicit millimetre targets; serialization does not depend on the GUI."""
from dataclasses import dataclass, asdict
import json
import math
from pathlib import Path


def numeric(value, name, low, high):
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        raise ValueError(f"{name} must be a number")
    if not math.isfinite(value) or not low <= value <= high:
        raise ValueError(f"{name} must be in [{low}, {high}]")
    return float(value)


@dataclass(frozen=True)
class Step:
    cable_1_mm: float
    cable_2_mm: float
    hold_s: float


@dataclass(frozen=True)
class Trial:
    name: str
    rest_lengths_mm: tuple
    speed_mm_s: float
    repetitions: int
    pre_roll_s: float
    post_roll_s: float
    timeout_s: float
    steps: tuple

    @classmethod
    def load(cls, path):
        return cls.from_dict(json.loads(Path(path).read_text(encoding="utf-8")))

    @classmethod
    def from_dict(cls, data):
        required = {"schema_version", "name", "rest_lengths_mm", "speed_mm_s",
                    "repetitions", "pre_roll_s", "post_roll_s", "timeout_s", "steps"}
        if not isinstance(data, dict) or set(data) != required or data["schema_version"] != 1:
            raise ValueError("Expected trial schema_version 1 and its exact documented fields")
        name = data["name"]
        if not isinstance(name, str) or not 1 <= len(name) <= 120:
            raise ValueError("name must contain 1..120 characters")
        rests = data["rest_lengths_mm"]
        if not isinstance(rests, list) or len(rests) != 2:
            raise ValueError("Two measured rest_lengths_mm are required")
        rests = tuple(numeric(v, "rest length", 70.001, 10000) for v in rests)
        repeats = data["repetitions"]
        rows = data["steps"]
        if type(repeats) is not int or not 1 <= repeats <= 20:
            raise ValueError("repetitions must be an integer in 1..20")
        if not isinstance(rows, list) or not 1 <= len(rows) <= 100 or len(rows)*repeats > 200:
            raise ValueError("Use 1..100 steps and at most 200 expanded steps")
        steps = []
        for row in rows:
            if not isinstance(row, dict) or set(row) != {"cable_1_mm", "cable_2_mm", "hold_s"}:
                raise ValueError("Each step requires cable_1_mm, cable_2_mm, hold_s")
            lengths = [numeric(row[f"cable_{i+1}_mm"], f"cable {i+1}", r-70, r+70)
                       for i, r in enumerate(rests)]
            steps.append(Step(*lengths, numeric(row["hold_s"], "hold_s", 0, 120)))
        return cls(name, rests, numeric(data["speed_mm_s"], "speed_mm_s", .05, 20),
                   repeats, numeric(data["pre_roll_s"], "pre_roll_s", 0, 120),
                   numeric(data["post_roll_s"], "post_roll_s", 0, 120),
                   numeric(data["timeout_s"], "timeout_s", 5, 3600), tuple(steps))

    def data(self):
        data = asdict(self)
        data["schema_version"] = 1
        data["rest_lengths_mm"] = list(self.rest_lengths_mm)
        data["steps"] = [asdict(step) for step in self.steps]
        return data

    def expanded(self):
        return self.steps * self.repetitions

    def procedure(self):
        # Only this adapter translates to the legacy half-difference format.
        return {"kind": "pair", "rows": [
            [(s.cable_1_mm+s.cable_2_mm)/2, (s.cable_1_mm-s.cable_2_mm)/2, s.hold_s]
            for s in self.steps], "repeats": self.repetitions, "speed_mm_s": self.speed_mm_s}

    def validate_rig(self, cfg):
        cfg.validate()
        if cfg.active_axes != 2:
            raise ValueError("This acquisition version requires two selected cables")
        for i, axis in enumerate(cfg.selected):
            if axis.rest_length_mm is None or abs(axis.rest_length_mm-self.rest_lengths_mm[i]) > .001:
                raise ValueError(f"Cable {i+1}: plan and motor config rest lengths disagree")
            axis.rpm_for_speed(self.speed_mm_s)

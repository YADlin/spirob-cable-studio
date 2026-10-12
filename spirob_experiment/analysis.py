"""Offline extension contracts and a cost function independent of MuJoCo."""
from bisect import bisect_left
import csv
import json
import math
from pathlib import Path
from typing import Protocol


class SimulationBackend(Protocol):
    """Map explicit input semantics to model-specific actuators and named sites."""
    def simulate(self, trial_path: Path, input_csv: Path, parameters: dict,
                 output_dir: Path) -> Path:
        """Return a keypoint CSV; also save model, mappings, solver and parameter metadata."""
        ...


class KeypointExtractor(Protocol):
    def extract(self, session_dir: Path, calibration_path: Path, output_dir: Path) -> Path: ...


def stiffness_profile(k0, beta, coordinates):
    """k_i=k0*exp(-beta*s_i). Caller must supply the generator's actual s_i."""
    if not math.isfinite(k0) or k0 <= 0 or not math.isfinite(beta):
        raise ValueError("k0 must be positive and beta finite")
    coords = list(coordinates)
    if not coords or coords[0] != 0 or any(not math.isfinite(x) for x in coords):
        raise ValueError("Explicit finite coordinates starting at base coordinate 0 are required")
    if any(b <= a for a, b in zip(coords, coords[1:])):
        raise ValueError("Joint coordinates must increase from base to tip")
    values = [k0*math.exp(-beta*x) for x in coords]
    if any(not math.isfinite(x) or x <= 0 for x in values):
        raise ValueError("Stiffness profile overflowed or underflowed")
    return values


def read_keypoints(path):
    rows = []
    with Path(path).open(newline="", encoding="utf-8") as stream:
        reader = csv.DictReader(stream)
        if set(reader.fieldnames or []) != {"t_s", "point", "x", "y", "valid"}:
            raise ValueError("Keypoint CSV columns must be t_s,point,x,y,valid")
        seen = set()
        for row in reader:
            if row["valid"] not in ("0", "1") or not row["point"]:
                raise ValueError("Use named points and valid=0 or 1")
            t = float(row["t_s"])
            if not math.isfinite(t) or (row["point"], t) in seen:
                raise ValueError("Non-finite or duplicate point timestamp")
            seen.add((row["point"], t))
            valid = row["valid"] == "1"
            xy = [float(row[k]) for k in ("x", "y")] if valid else [None, None]
            if valid and any(not math.isfinite(v) for v in xy):
                raise ValueError("Valid keypoints must have finite coordinates")
            rows.append({"t": t, "point": row["point"], "xy": xy, "valid": valid})
    if not rows:
        raise ValueError("Empty keypoint file")
    return rows


def keypoint_cost(observed, simulated, metadata, max_gap_s=.1, min_coverage=.95):
    """SSE at measured times; interpolate simulation only, never extrapolate.

    A missing simulated point is an error, so an optimizer cannot improve cost
    by dropping difficult points. Physical missingness is reported and gated.
    """
    if not 0 <= min_coverage <= 1 or not math.isfinite(max_gap_s) or max_gap_s <= 0:
        raise ValueError("Invalid coverage/gap threshold")
    meta = json.loads(Path(metadata).read_text(encoding="utf-8"))
    if set(meta) != {"units", "coordinate_frame", "time_origin", "required_points"}:
        raise ValueError("Comparison metadata must explicitly name units, frame, time origin and required_points")
    if meta["units"] not in ("m", "mm", "pixel") or not meta["coordinate_frame"] or not meta["time_origin"]:
        raise ValueError("Invalid comparison coordinate metadata")
    points = meta["required_points"]
    if not isinstance(points, list) or not points or len(set(points)) != len(points):
        raise ValueError("required_points must be a nonempty unique list")
    obs, sim = read_keypoints(observed), read_keypoints(simulated)
    groups = {}
    for row in sim:
        groups.setdefault(row["point"], []).append(row)
    for rows in groups.values():
        rows.sort(key=lambda r: r["t"])
    total, squared, valid_total = 0, [], 0
    per_point = {}
    for point in points:
        measured = [r for r in obs if r["point"] == point]
        if not measured:
            raise ValueError(f"No observations for required point {point}")
        samples = groups.get(point, [])
        times = [r["t"] for r in samples]
        point_valid = 0
        for row in measured:
            total += 1
            if not row["valid"]:
                continue
            point_valid += 1
            idx = bisect_left(times, row["t"])
            if idx < len(times) and abs(times[idx]-row["t"]) < 1e-9:
                if not samples[idx]["valid"]:
                    raise ValueError(f"Invalid simulated point {point} at {row['t']}")
                predicted = samples[idx]["xy"]
            else:
                if idx == 0 or idx == len(times):
                    raise ValueError(f"Simulation does not cover {point} at {row['t']}; no extrapolation")
                a, b = samples[idx-1], samples[idx]
                if not a["valid"] or not b["valid"] or b["t"]-a["t"] > max_gap_s:
                    raise ValueError(f"Simulation gap for {point} at {row['t']}")
                fraction = (row["t"]-a["t"])/(b["t"]-a["t"])
                predicted = [x+fraction*(y-x) for x, y in zip(a["xy"], b["xy"])]
            squared.append(sum((a-b)**2 for a, b in zip(row["xy"], predicted)))
            valid_total += 1
        coverage = point_valid/len(measured)
        if coverage < min_coverage or not point_valid:
            raise ValueError(f"Insufficient physical tracking coverage for {point}: {coverage:.1%}")
        per_point[point] = {"valid": point_valid, "expected": len(measured), "coverage": coverage}
    return {"sse": sum(squared), "sse_units": meta["units"]+"^2",
            "point_distance_rmse": math.sqrt(sum(squared)/valid_total),
            "rmse_units": meta["units"], "matched_points": valid_total, "expected_points": total,
            "coverage": valid_total/total, "per_point": per_point,
            "coordinate_metadata": meta, "max_interpolation_gap_s": max_gap_s}

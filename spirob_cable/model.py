"""Units and calibration, independent of GUI and hardware.

Positive cable displacement = payout (longer free cable).
Counts are incremental base-motor quadrature counts. The spool is assumed to
be rigidly attached 1:1 to the documented 100:1 gearbox output.
"""
from dataclasses import asdict, dataclass, fields
import json
import math
from pathlib import Path

COUNTS_PER_REV = 133600
GEAR_RATIO = 100
MAX_SPEED_MM_S = 20.0  # Application limit, not a motor/mechanism rating.
MAX_MOTOR_RPM = 2000  # Also bound commands for unusually small calibrations.
TRAVEL_MM = 70.0
I32_MIN, I32_MAX = -(1 << 31), (1 << 31) - 1


class Rejected(ValueError):
    """Invalid user request: reject without changing a valid motor state."""


def finite(value, name):
    if isinstance(value, bool):
        raise Rejected(f"{name} must be a finite number")
    try:
        result = float(value)
    except (ValueError, TypeError) as exc:
        raise Rejected(f"{name} must be a finite number") from exc
    if not math.isfinite(result):
        raise Rejected(f"{name} must be a finite number")
    return result


def signed_count(value):
    if type(value) is not int or not I32_MIN <= value <= I32_MAX:
        raise Rejected("Encoder target outside signed 32-bit range; establish a new session")
    return value


@dataclass(frozen=True)
class Settings:
    port: str = "/dev/ttyUSB0"
    device_id: int | None = None
    spool_diameter_mm: float = 27.0
    cable_diameter_mm: float | None = 0.405
    diameter_basis: str | None = None
    calibrated_mm_per_rev: float | None = None
    payout_sign: int | None = None
    rest_length_mm: float | None = None
    speed_mm_s: float = 0.5
    jog_speed_mm_s: float | None = None  # Older configs inherit speed_mm_s.
    acceleration_raw: int = 1000
    setup_travel_mm: float = 70.0  # Explicit envelope about connection count, before rest.

    @classmethod
    def load(cls, path):
        data = json.loads(Path(path).read_text())
        unknown = set(data) - {f.name for f in fields(cls)}
        if unknown:
            raise Rejected(f"Unknown settings: {sorted(unknown)}")
        return cls(**data)

    def save(self, path):
        path = Path(path)
        tmp = path.with_suffix(path.suffix + ".tmp")
        tmp.write_text(json.dumps(asdict(self), indent=2) + "\n")
        tmp.replace(path)

    def validate_connection(self):
        if not isinstance(self.port, str) or not self.port.strip():
            raise Rejected("Enter the actual serial port")
        if type(self.device_id) is not int or not 1 <= self.device_id <= 247:
            raise Rejected("Device ID must be an integer from 1 to 247")

    def validate(self):
        self.validate_connection()
        if type(self.payout_sign) is not int or self.payout_sign not in (-1, 1):
            raise Rejected("Select which encoder direction pays cable out (+1 or -1)")
        for name in ("spool_diameter_mm", "cable_diameter_mm", "calibrated_mm_per_rev"):
            value = getattr(self, name)
            if value is not None and finite(value, name) <= 0:
                raise Rejected(f"{name} must be positive")
        if self.calibrated_mm_per_rev is None:
            if self.diameter_basis not in ("core", "effective"):
                raise Rejected("Specify whether 27 mm is the bare core or effective winding diameter")
            if self.diameter_basis == "core" and self.cable_diameter_mm is None:
                raise Rejected("Enter cable diameter for a bare-core spool")
        if self.rest_length_mm is not None and finite(self.rest_length_mm, "Rest length") <= TRAVEL_MM:
            raise Rejected("An optional absolute rest length must exceed 70 mm")
        if not 0.5 <= finite(self.setup_travel_mm, "Setup travel") <= 1000:
            raise Rejected("Setup travel must be 0.5..1000 mm from connection position")
        self.rpm_for_speed(self.speed_mm_s)
        self.rpm_for_speed(self.jog_speed)
        if type(self.acceleration_raw) is not int or not 1 <= self.acceleration_raw <= 20000:
            raise Rejected("Acceleration must be an integer from 1 to 20000 (raw units)")
        if self.mm_per_count > 0.01:
            raise Rejected("Calibration implies over 0.01 mm per count; check dimensions")

    @property
    def mm_per_rev(self):
        if self.calibrated_mm_per_rev is not None:
            return finite(self.calibrated_mm_per_rev, "Calibrated mm/rev")
        diameter = finite(self.spool_diameter_mm, "Spool diameter")
        if self.diameter_basis == "core":
            diameter += finite(self.cable_diameter_mm, "Cable diameter")
        elif self.diameter_basis != "effective":
            raise Rejected("Diameter basis is unknown")
        return math.pi * diameter

    @property
    def mm_per_count(self):
        return self.mm_per_rev / COUNTS_PER_REV

    @property
    def motor_rpm(self):
        return self.rpm_for_speed(self.speed_mm_s)

    @property
    def jog_speed(self):
        return self.speed_mm_s if self.jog_speed_mm_s is None else self.jog_speed_mm_s

    def rpm_for_speed(self, speed_mm_s):
        speed = finite(speed_mm_s, "Cable speed")
        if not 0.05 <= speed <= MAX_SPEED_MM_S:
            raise Rejected(f"Cable speed must be 0.05 to {MAX_SPEED_MM_S:g} mm/s")
        # Round DOWN, so nominal commanded cable speed does not exceed request.
        rpm = math.floor(speed * 60 * GEAR_RATIO / self.mm_per_rev)
        if not 1 <= rpm <= MAX_MOTOR_RPM:
            raise Rejected(f"Speed and calibration imply {rpm} motor RPM; allowed 1..{MAX_MOTOR_RPM}")
        return rpm

    def speed_for_rpm(self, rpm):
        return rpm * self.mm_per_rev / (60 * GEAR_RATIO)

    @property
    def actual_speed_mm_s(self):
        return self.speed_for_rpm(self.motor_rpm)

    def offset_mm(self, count, reference):
        # Deliberately NO modulo 360 or modulo COUNTS_PER_REV here.
        return self.payout_sign * (count - reference) * self.mm_per_count

    def target_count(self, offset_mm, reference, limit_mm=TRAVEL_MM):
        offset_mm = finite(offset_mm, "Cable target")
        if abs(offset_mm) > limit_mm:
            raise Rejected(f"Target must be within +/-{limit_mm:g} mm")
        delta = round(self.payout_sign * offset_mm / self.mm_per_count)
        max_delta = math.floor(limit_mm / self.mm_per_count)
        delta = min(max_delta, max(-max_delta, delta))
        return signed_count(reference + delta)


def turns(count, reference):
    """Signed total spool turns, complete turns, signed remainder degrees."""
    total = (count - reference) / COUNTS_PER_REV
    whole = math.trunc(total)
    return total, whole, (total - whole) * 360

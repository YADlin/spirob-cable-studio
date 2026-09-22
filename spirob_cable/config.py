"""Two-axis configuration and explicit mean/half-difference coordinates."""
from dataclasses import asdict, dataclass, fields, replace
import json
from pathlib import Path
from .model import Settings, Rejected, finite, TRAVEL_MM


@dataclass(frozen=True)
class RigSettings:
    axes: tuple[Settings, Settings] = (Settings(), Settings())
    active_axes: int = 2

    @classmethod
    def load(cls, path):
        data = json.loads(Path(path).read_text())
        if 'axes' not in data:  # Import the working single-motor v1/v2 config.
            allowed = {f.name for f in fields(Settings)}
            if set(data)-allowed:
                raise Rejected(f'Unknown settings: {sorted(set(data)-allowed)}')
            first = Settings(**data)
            return cls((first, replace(first, device_id=None, rest_length_mm=None)), 1)
        if set(data)-{'schema_version', 'active_axes', 'axes'} or data.get('schema_version') != 3:
            raise Rejected('Expected schema_version 3 configuration')
        if not isinstance(data['axes'], list) or len(data['axes']) != 2:
            raise Rejected('Configuration stores two cable definitions')
        return cls(tuple(Settings(**a) for a in data['axes']), data['active_axes'])

    def validate(self):
        if type(self.active_axes) is not int or self.active_axes not in (1, 2):
            raise Rejected('Select one or two active cables')
        for i, cfg in enumerate(self.selected):
            try:
                cfg.validate()
            except Rejected as exc:
                raise Rejected(f'Cable {i+1}: {exc}') from exc
        if len({c.device_id for c in self.selected}) != self.active_axes:
            raise Rejected('The two motors must have distinct device IDs')

    @property
    def selected(self):
        return self.axes[:self.active_axes]

    def data(self):
        return {'schema_version': 3, 'active_axes': self.active_axes,
                'axes': [asdict(a) for a in self.axes]}

    def save(self, path):
        path = Path(path); path.parent.mkdir(parents=True, exist_ok=True)
        temp = path.with_suffix(path.suffix+'.tmp')
        temp.write_text(json.dumps(self.data(), indent=2)+'\n'); temp.replace(path)

    @classmethod
    def demo(cls, count=2):
        return cls(tuple(Settings(port='SIMULATED', device_id=i+1, diameter_basis='core',
                    payout_sign=1 if i == 0 else -1, rest_length_mm=220,
                    speed_mm_s=5, jog_speed_mm_s=5, setup_travel_mm=70) for i in range(2)), count)


def lengths_from_modes(mean_mm, half_difference_mm):
    a = finite(mean_mm, 'Mean length A')
    d = finite(half_difference_mm, 'Half-difference D')
    return a+d, a-d


def modes_from_lengths(length_1, length_2):
    first, second = finite(length_1, 'Cable 1 length'), finite(length_2, 'Cable 2 length')
    return (first+second)/2, (first-second)/2


def difference_bounds(mean_mm, rest_1, rest_2):
    a = finite(mean_mm, 'Mean length A')
    return max(rest_1-TRAVEL_MM-a, a-rest_2-TRAVEL_MM), min(rest_1+TRAVEL_MM-a, a-rest_2+TRAVEL_MM)

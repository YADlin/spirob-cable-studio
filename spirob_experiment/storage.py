"""Local session ownership. Cloud upload can consume a closed session later."""
import hashlib
from importlib.metadata import PackageNotFoundError, version
import json
from pathlib import Path
import platform
import time
from typing import Protocol


def software_provenance():
    """Record installed versions and source hashes, including uncommitted code."""
    root = Path(__file__).resolve().parent.parent
    sources = {}
    for package in ("spirob_experiment", "spirob_cable"):
        for path in sorted((root/package).glob("*.py")):
            sources[path.relative_to(root).as_posix()] = hashlib.sha256(path.read_bytes()).hexdigest()
    dependencies = {}
    for name in ("opencv-python-headless", "numpy", "pymodbus", "pyserial", "PySide6", "pyqtgraph"):
        try:
            dependencies[name] = version(name)
        except PackageNotFoundError:
            pass
    return {"python": platform.python_version(), "platform": platform.platform(),
            "dependencies": dependencies, "source_sha256": sources}


def atomic_json(path, data):
    path = Path(path)
    tmp = path.with_name(path.name+".tmp")
    tmp.write_text(json.dumps(data, indent=2, allow_nan=False)+"\n", encoding="utf-8")
    tmp.replace(path)


class ArchiveSink(Protocol):
    """Future Drive adapter: upload an already closed, hashed directory."""
    def publish(self, session_dir: Path, checksums: dict) -> str: ...


def checksums(root):
    result = {}
    for path in sorted(Path(root).rglob("*")):
        if path.is_file() and path.name != "checksums.json":
            digest = hashlib.sha256()
            with path.open("rb") as stream:
                for block in iter(lambda: stream.read(1024*1024), b""):
                    digest.update(block)
            result[path.relative_to(root).as_posix()] = digest.hexdigest()
    return result


class Session:
    def __init__(self, root, session_id, trial, demo):
        self.path = Path(root)/session_id
        self.path.mkdir(parents=True, exist_ok=False)
        self.started = time.monotonic()
        self.data = {"schema_version": 1, "session_id": session_id, "demo": demo,
                     "state": "STARTING", "created_utc_ns": time.time_ns(),
                     "coordinator_monotonic_s": self.started, "trial": trial.data(),
                     "camera_timing": "Host arrival only; no sensor exposure timestamps",
                     "end_policy": "stop, then disconnect and release motor torque"}
        self.data["software"] = software_provenance()
        atomic_json(self.path/"trial.json", trial.data())
        self.events = (self.path/"events.jsonl").open("x", encoding="utf-8")
        self.save()

    def save(self):
        atomic_json(self.path/"run.json", self.data)

    def event(self, event, **values):
        row = {"t_s": time.monotonic()-self.started, "event": event, **values}
        self.events.write(json.dumps(row, allow_nan=False)+"\n")
        self.events.flush()

    def close(self):
        self.save()
        self.events.close()

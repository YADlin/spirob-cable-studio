"""Derived timing, inputs, and event-selected frames; originals stay intact."""
import csv
import json
from pathlib import Path
import shutil
from .storage import atomic_json


def camera_to_coordinator(value, before, after=None):
    a = before["best"]
    if after is None:
        return value-a["offset_s"]
    b = after["best"]
    elapsed = b["coordinator_mid_s"]-a["coordinator_mid_s"]
    if elapsed <= 0:
        raise ValueError("Clock calibration endpoints are not ordered")
    slope = (b["offset_s"]-a["offset_s"])/elapsed
    if abs(slope) > .005:
        raise ValueError("Host clock mapping changed by >5000 ppm; inspect clock/suspend events")
    return (value-a["offset_s"]+slope*a["coordinator_mid_s"])/(1+slope)


def write_rows(path, fields, rows):
    with Path(path).open("w", newline="", encoding="utf-8") as stream:
        writer = csv.DictWriter(stream, fieldnames=fields)
        writer.writeheader()
        writer.writerows(rows)


def derive_session(path):
    path = Path(path)
    run = json.loads((path/"run.json").read_text(encoding="utf-8"))
    before, after = run["clock_before"], run.get("clock_after")
    origin = run["coordinator_monotonic_s"]
    derived = path/"derived"
    derived.mkdir(exist_ok=True)
    frames = []
    with (path/"camera/frames.csv").open(newline="", encoding="utf-8") as stream:
        for frame in csv.DictReader(stream):
            frames.append({**frame, "t_s": camera_to_coordinator(float(frame["read_end_s"]), before, after)-origin})
    if not frames:
        raise ValueError("No recorded camera frames")
    write_rows(derived/"frames_aligned.csv", list(frames[0]), frames)
    gaps = [b["t_s"]-a["t_s"] for a, b in zip(frames, frames[1:])]
    atomic_json(derived/"timing_quality.json", {
        "frame_count": len(frames), "max_arrival_gap_s": max(gaps, default=None),
        "observed_average_fps": ((len(frames)-1)/(frames[-1]["t_s"]-frames[0]["t_s"])
                                 if len(frames) > 1 and frames[-1]["t_s"] > frames[0]["t_s"] else None),
        "clock_mapping": "two-endpoint affine" if after else "initial offset only",
        "network_uncertainty_before_s": before["network_uncertainty_s"],
        "network_uncertainty_after_s": after["network_uncertainty_s"] if after else None,
        "unmeasured": ["sensor exposure time", "camera buffering delay", "sensor frames dropped before read"],
        "interpretation": "Host timing validation required before dynamic parameter identification"})
    selections = [("first_frame", frames[0]["t_s"]), ("last_frame", frames[-1]["t_s"])]
    if run.get("motor_log"):
        motor_path = path/run["motor_log"]
        motor_origin = run["motor_origin_s"]
        events = [json.loads(line) for line in (motor_path/"events.jsonl").read_text().splitlines()]
        cfg = run["motor_config"]["axes"]
        inputs, commands = [], []
        with (motor_path/"samples.csv").open(newline="") as stream:
            for row in csv.DictReader(stream):
                inputs.append({"axis": row["axis"],
                               "read_start_s": motor_origin+float(row["read_start_s"]),
                               "read_end_s": motor_origin+float(row["read_end_s"]),
                               "actual_length_m": float(row["absolute_mm"])/1000 if row["absolute_mm"] else "",
                               "target_length_m": float(row["target_absolute_mm"])/1000 if row["target_absolute_mm"] else "",
                               "state": row["state"]})
        for event in events:
            t = motor_origin+event["t_s"]
            if event["event"] == "move_requested":
                from spirob_cable.model import Settings
                settings = Settings(**cfg[event["axis"]])
                length = settings.rest_length_mm + settings.offset_mm(event["target_count"], event["rest_count"])
                commands.append({"axis": event["axis"], "request_t_s": t,
                                 "target_length_m": length/1000, "speed_mm_s": event["requested_mm_s"],
                                 "timing": "request event before serial dispatch, not hardware start"})
            if event["event"] == "experiment_hold":
                selections.append((f"step_{event['step']:03d}_settled", t+event["duration_s"]/2))
        write_rows(derived/"motor_inputs.csv",
                   ["axis", "read_start_s", "read_end_s", "actual_length_m", "target_length_m", "state"], inputs)
        write_rows(derived/"commands.csv",
                   ["axis", "request_t_s", "target_length_m", "speed_mm_s", "timing"], commands)
    selected = derived/"selected_frames"
    selected.mkdir(exist_ok=True)
    index = []
    for label, target in selections:
        frame = min(frames, key=lambda r: abs(r["t_s"]-target))
        source = (path/"camera"/frame["file"]).resolve()
        if not source.is_relative_to((path/"camera/frames").resolve()):
            raise ValueError("Camera index references a file outside its frame directory")
        destination = selected/(label+".jpg")
        shutil.copyfile(source, destination)
        index.append({"label": label, "requested_t_s": target, "frame_t_s": frame["t_s"],
                      "selection_error_s": frame["t_s"]-target, "frame_index": frame["frame_index"],
                      "file": destination.relative_to(path).as_posix()})
    write_rows(derived/"selected_frames.csv",
               ["label", "requested_t_s", "frame_t_s", "selection_error_s", "frame_index", "file"], index)
    return derived

"""python -m spirob_experiment --help"""
import argparse
from dataclasses import replace
import json
import os
from pathlib import Path
import secrets
import signal
import threading
from .camera import CameraService, SyntheticCamera, OpenCVCamera, serve
from .camera_client import CameraClient
from .runner import acquire
from .schema import Trial
from .storage import atomic_json, checksums


ROOT = Path(__file__).resolve().parent.parent
EXAMPLE = ROOT/"examples/experiment_pilot.json"


def token_file(path):
    return Path(path).read_text(encoding="utf-8").strip()


def parser():
    root = argparse.ArgumentParser(description="SpiRob modular acquisition and offline comparison")
    sub = root.add_subparsers(dest="command", required=True)
    token = sub.add_parser("make-token")
    token.add_argument("path", type=Path)
    demo = sub.add_parser("demo", help="Simulated motors and synthetic camera; no devices opened")
    demo.add_argument("--root", type=Path, required=True)
    demo.add_argument("--plan", type=Path, default=EXAMPLE)
    cam = sub.add_parser("camera-serve", help="Native Windows camera service or synthetic test service")
    cam.add_argument("--root", type=Path, required=True)
    cam.add_argument("--token-file", type=Path, required=True)
    cam.add_argument("--bind", default="127.0.0.1")
    cam.add_argument("--port", type=int, default=8765)
    cam.add_argument("--device", type=int, default=0)
    cam.add_argument("--backend", choices=["auto", "dshow", "msmf", "v4l2"], default="auto")
    cam.add_argument("--width", type=int, default=1920)
    cam.add_argument("--height", type=int, default=1080)
    cam.add_argument("--fps", type=float, default=30)
    cam.add_argument("--synthetic", action="store_true")
    for name in ("camera-check", "run"):
        cmd = sub.add_parser(name)
        cmd.add_argument("--root", type=Path, required=True)
        cmd.add_argument("--camera-url", required=True)
        cmd.add_argument("--token-file", type=Path, required=True)
        cmd.add_argument("--min-fps", type=float, default=20)
        if name == "run":
            cmd.add_argument("--config", type=Path, required=True)
            cmd.add_argument("--plan", type=Path, required=True)
            cmd.add_argument("--execute", action="store_true")
            cmd.add_argument("--at-rest", action="store_true")
        else:
            cmd.add_argument("--seconds", type=float, default=10)
    val = sub.add_parser("validate", help="Read-only plan/config checks; never open a serial port")
    val.add_argument("--config", type=Path, required=True)
    val.add_argument("--plan", type=Path, required=True)
    derive = sub.add_parser("derive", help="Regenerate aligned indices and selected frames")
    derive.add_argument("session", type=Path)
    seal = sub.add_parser("seal", help="Hash files of a closed session for later archival")
    seal.add_argument("session", type=Path)
    cost = sub.add_parser("cost", help="Offline keypoint SSE; requires aligned coordinate data")
    cost.add_argument("--observed", type=Path, required=True)
    cost.add_argument("--simulated", type=Path, required=True)
    cost.add_argument("--metadata", type=Path, required=True)
    cost.add_argument("--max-gap", type=float, default=.1)
    cost.add_argument("--min-coverage", type=float, default=.95)
    cost.add_argument("--output", type=Path, required=True)
    return root


def main():
    args = parser().parse_args()
    def interrupted(*_):
        raise KeyboardInterrupt("Termination requested")
    signal.signal(signal.SIGTERM, interrupted)
    if hasattr(signal, "SIGHUP"):
        signal.signal(signal.SIGHUP, interrupted)
    if args.command == "make-token":
        args.path.parent.mkdir(parents=True, exist_ok=True)
        fd = os.open(args.path, os.O_WRONLY | os.O_CREAT | os.O_EXCL, 0o600)
        with os.fdopen(fd, "w") as stream:
            stream.write(secrets.token_hex(32)+"\n")
        print(f"Token created at {args.path}; keep it outside Git")
        return 0
    if args.command == "camera-serve":
        if args.width <= 0 or args.height <= 0 or not 1 <= args.fps <= 120:
            raise ValueError("Invalid camera dimensions/frame rate")
        factory = (lambda: SyntheticCamera(args.width, args.height, args.fps)) if args.synthetic else (
            lambda: OpenCVCamera(args.device, args.backend, args.width, args.height, args.fps))
        service = CameraService(args.root, token_file(args.token_file), factory)
        server = serve(service, args.bind, args.port)
        print(f"Camera service at {args.bind}:{args.port}; root={args.root.resolve()}; no camera opened until requested",
              flush=True)
        try:
            server.serve_forever()
        except KeyboardInterrupt:
            pass
        finally:
            if service.capture:
                service.capture.stop()
            server.server_close()
        return 0
    if args.command == "demo":
        from spirob_cable.config import RigSettings
        trial = Trial.load(args.plan)
        cfg = RigSettings.demo()
        cfg = replace(cfg, axes=tuple(replace(a, rest_length_mm=r)
                                     for a, r in zip(cfg.axes, trial.rest_lengths_mm)))
        token = secrets.token_hex(32)
        service = CameraService(args.root, token, lambda: SyntheticCamera())
        server = serve(service, port=0)
        thread = threading.Thread(target=server.serve_forever, daemon=True)
        thread.start()
        try:
            path = acquire(trial, args.root, CameraClient(f"http://127.0.0.1:{server.server_port}", token),
                           cfg, demo=True, min_fps=10)
            print(f"COMPLETE: {path}")
        finally:
            server.shutdown()
            server.server_close()
            thread.join(3)
        return 0
    if args.command in ("validate", "run"):
        from spirob_cable.config import RigSettings
        trial, cfg = Trial.load(args.plan), RigSettings.load(args.config)
        trial.validate_rig(cfg)
        print(json.dumps({"name": trial.name, "rest_lengths_mm": trial.rest_lengths_mm,
                          "expanded_steps": len(trial.expanded()), "speed_mm_s": trial.speed_mm_s,
                          "targets_mm": [[s.cable_1_mm, s.cable_2_mm] for s in trial.steps],
                          "end_policy": "Stop then disconnect; torque is released"}, indent=2), flush=True)
        if args.command == "validate":
            return 0
        if not args.execute or not args.at_rest:
            raise ValueError("Review the plan; hardware execution requires --execute --at-rest. Close the motor GUI first.")
        path = acquire(trial, args.root, CameraClient(args.camera_url, token_file(args.token_file)),
                       cfg, min_fps=args.min_fps)
        print(f"COMPLETE: {path}")
        return 0
    if args.command == "camera-check":
        if not 1 <= args.seconds <= 120:
            raise ValueError("Camera check duration must be 1..120 seconds")
        trial = replace(Trial.load(EXAMPLE), name="camera-check", pre_roll_s=args.seconds,
                        post_roll_s=0, timeout_s=args.seconds+30)
        print(f"COMPLETE: {acquire(trial, args.root, CameraClient(args.camera_url, token_file(args.token_file)), min_fps=args.min_fps)}")
        return 0
    if args.command == "derive":
        from .derive import derive_session
        print(derive_session(args.session))
        return 0
    if args.command == "seal":
        run = json.loads((args.session/"run.json").read_text())
        if run["state"] not in ("COMPLETE", "FAILED", "POSTPROCESS_FAILED"):
            raise ValueError("Only closed sessions may be sealed")
        if not run.get("camera_shutdown", {}).get("thread_exited"):
            raise ValueError("Camera writer exit was not confirmed; do not archive a live recording")
        if "motor_shutdown" in run and not run["motor_shutdown"]["worker_exited"]:
            raise ValueError("Motor writer exit was not confirmed")
        atomic_json(args.session/"checksums.json", checksums(args.session))
        print(args.session/"checksums.json")
        return 0
    if args.command == "cost":
        from .analysis import keypoint_cost
        result = keypoint_cost(args.observed, args.simulated, args.metadata, args.max_gap, args.min_coverage)
        atomic_json(args.output, result)
        print(json.dumps(result, indent=2))
        return 0


if __name__ == "__main__":
    try:
        raise SystemExit(main())
    except (Exception, KeyboardInterrupt) as exc:
        import sys
        print(f"FAILED: {exc}", file=sys.stderr, flush=True)
        raise SystemExit(1)

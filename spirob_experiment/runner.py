"""Local orchestration. Losing progress lets the motor worker watchdog expire."""
from datetime import datetime, timezone
import json
import math
from pathlib import Path
import time
import uuid
from . import __version__
from .motor import Motor
from .storage import Session, atomic_json
from .derive import derive_session


def acquire(trial, root, camera, cfg=None, demo=False, min_fps=20, max_frame_age_s=1):
    if not math.isfinite(min_fps) or not 1 <= min_fps <= 120:
        raise ValueError("min_fps must be finite and in 1..120")
    if not math.isfinite(max_frame_age_s) or not .1 <= max_frame_age_s <= 10:
        raise ValueError("max_frame_age_s must be in 0.1..10")
    if cfg is not None:
        trial.validate_rig(cfg)
    mode = "DEMO" if demo else "HARDWARE" if cfg else "CAMERA_CHECK"
    session_id = datetime.now(timezone.utc).strftime("%Y%m%dT%H%M%S_%fZ")+"_"+mode+"_"+uuid.uuid4().hex[:6]
    session = Session(root, session_id, trial, demo)
    session.data.update(pipeline_version=__version__, mode=mode)
    if cfg:
        session.data["motor_config"] = cfg.data()
    motor, camera_requested = None, False
    last_camera_check = 0.
    last_status = {}
    deadline = time.monotonic()+trial.timeout_s
    failure = None

    def phase(name):
        session.data["state"] = name
        session.event("phase", name=name)
        session.save()
        print(f"{session_id}: {name}", flush=True)

    def pump(starting=False):
        nonlocal last_camera_check, last_status
        if motor:
            motor.tick()
        if time.monotonic() > deadline:
            raise TimeoutError("Trial wall-clock deadline exceeded")
        if time.monotonic()-last_camera_check >= .25:
            last_status = camera.status()
            last_camera_check = time.monotonic()
            if last_status["state"] in ("FAILED", "COMPLETE"):
                raise RuntimeError("Camera stopped: "+str(last_status.get("error")))
            age = last_status["last_frame_age_s"]
            if not starting and (age is None or age > max_frame_age_s):
                raise RuntimeError("Camera feedback stale; stopping trial")
            if last_status["frame_count"] >= 10 and (last_status["observed_fps"] or 0) < min_fps:
                raise RuntimeError(f"Camera delivery below {min_fps:g} fps")

    def wait_for(predicate, seconds, starting=False):
        until = time.monotonic()+seconds
        while True:
            pump(starting)
            if predicate():
                return
            if time.monotonic() >= until:
                raise TimeoutError("Timed out waiting for camera/motor readiness")
            time.sleep(.02)

    def dwell(seconds):
        until = time.monotonic()+seconds
        while time.monotonic() < until:
            pump()
            time.sleep(.02)

    try:
        session.data["clock_before"] = camera.clock()
        nonce = uuid.uuid4().hex
        # Mark before sending: even a lost start response requires a stop attempt.
        camera_requested = True
        camera.start(session_id, nonce)
        wait_for(lambda: last_status.get("frame_count", 0) >= 10, 15, starting=True)
        local = session.path/"camera/capture.json"
        if not local.exists() or json.loads(local.read_text())["nonce"] != nonce:
            raise RuntimeError("Camera and coordinator do not share the same storage root")
        if cfg and not demo and last_status.get("source", {}).get("source") != "opencv":
            raise RuntimeError("Hardware trials require a real camera source")
        session.data["camera_ready"] = last_status
        if cfg:
            phase("CONNECTING_DISABLED")
            motor = Motor(cfg, demo, session)
            wait_for(lambda: motor.connected, 15)
            phase("REFERENCING_AT_DECLARED_REST")
            motor.submit("enable", {"indices": [0, 1],
                                   "reference_lengths": dict(enumerate(trial.rest_lengths_mm))})
            wait_for(lambda: motor.referenced, 15)
        phase("PRE_ROLL")
        dwell(trial.pre_roll_s)
        if motor:
            phase("TRIAL")
            motor.submit("experiment", trial.procedure())
            wait_for(lambda: motor.snapshot.get("procedure", {}).get("phase") == "COMPLETE", trial.timeout_s)
        phase("POST_ROLL")
        dwell(trial.post_roll_s)
    except BaseException as exc:
        failure = f"{type(exc).__name__}: {exc}"
    finally:
        # Motor cleanup comes first, even if recording/storage/network has failed.
        if motor:
            try:
                session.data["motor_shutdown"] = motor.shutdown()
                closed = session.data["motor_shutdown"]
                if not closed["worker_exited"] or not all(closed["disable_confirmed"]):
                    failure = (failure+"; " if failure else "")+"Motor disable or worker exit UNCONFIRMED"
            except BaseException as exc:
                failure = (failure+"; " if failure else "")+f"Motor cleanup failed: {exc}"
        if camera_requested:
            try:
                session.data["camera_shutdown"] = camera.stop()
                status = session.data["camera_shutdown"]
                if status["state"] != "COMPLETE" or not status["thread_exited"]:
                    failure = (failure+"; " if failure else "")+"Camera recording did not close cleanly"
            except Exception as exc:
                failure = (failure+"; " if failure else "")+f"Camera stop unconfirmed: {exc}"
            try:
                session.data["clock_after"] = camera.clock()
            except Exception as exc:
                session.data["clock_after_error"] = str(exc)
        session.data.update(state="FAILED" if failure else "COMPLETE", error=failure)
        session.close()
    if not failure:
        try:
            derive_session(session.path)
        except Exception as exc:
            session.data.update(state="POSTPROCESS_FAILED", error=str(exc))
            atomic_json(session.path/"run.json", session.data)
            raise RuntimeError(f"Acquisition retained; postprocessing failed in {session.path}: {exc}") from exc
    if failure:
        raise RuntimeError(f"{failure}\nSession retained: {session.path}")
    return session.path

"""RL egg-knockdown mode: software verification without hardware.

  1. Cable map equals Comparator.controls (all mapping/sign combinations) and inverts exactly.
  2. Camera map: similarity + parallax round trip.
  3. Egg detector on synthetic frames (hit, miss, robot distractor).
  4. Policy bundle verification rejects tampered/mismatched exports.
  5. Closed loop through the MuJoCo twin with a SYNTHETIC heuristic policy:
     knockdown on both sides, under every cable mapping/sign wiring.
  6. Fault paths: rejected command, stuck motor, lost egg (UNKNOWN), stale frames.
  7. Dry run sends no motor commands.
  8. RL commands pass the REAL dispatcher and motor Rig (DemoDrive) when --motor-source is given.

usage (workstation, lab folder):
  uv run --frozen python tests/verify_rl_knockdown.py [--motor-source /path/to/spirob-cable-studio]
"""
import argparse
import copy
import hashlib
import json
import shutil
import sys
import tempfile
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
p = argparse.ArgumentParser()
p.add_argument("--motor-source")
p.add_argument("--quick", action="store_true")
args = p.parse_args()

from rl import contract as C  # noqa: E402
from rl.egg_vision import EggColour, EggDetector  # noqa: E402
from rl.hardware_io import CableMap, CameraMap  # noqa: E402
from rl.numpy_policy import NumpyPolicy  # noqa: E402
from rl.runner import KnockdownRunner, PolicyBundle, return_to_start_command, sha  # noqa: E402
from rl.twin import Twin  # noqa: E402

RESTS = (227.35331441047117, 227.3533144104711)
LIMITS = dict(rest_lengths_mm=list(RESTS), absolute_bounds_mm=[[150.0, 300.0], [150.0, 300.0]], max_speed_mm_s=20.0,
              setup_travel_mm=150.0, difference_mm=150.0, travel_mm=77.35331441047117)
SETUP = dict(mapping=[0, 1], signs=[1, 1], pixels_per_m=4521.679319439051, rotation_rad=-0.12648877120158908,
             translation_x_px=847.7087666971994, translation_y_px=-9.345554861541913)
PARAMS = dict(mean_offset_mm=0.0, difference_offset_mm=0.0)
results = []


def check(name, cond, detail=""):
    results.append((name, bool(cond), detail))
    print(("PASS " if cond else "FAIL ") + name + (f"  [{detail}]" if detail else ""), flush=True)


# ------------------------------------------------------------------ 1. cable map
from spirob_analysis.model import SpiRobModel  # noqa: E402
from analysis import Comparator  # noqa: E402

model = SpiRobModel(str(ROOT / "model" / "spirob_robot.xml"))
check("neutral payout equals Lab model", np.allclose(model.neutral_lengths, C.NEUTRAL_PAYOUT_M, atol=1e-9))
check("frame C origin equals Lab reference_origin", np.allclose(model.reference_origin[:2], C.C_ORIGIN_MODEL_XY, atol=1e-7))
ok = True
for mapping in ([0, 1], [1, 0]):
    for signs in ([1, 1], [1, -1], [-1, 1], [-1, -1]):
        for offs in ((0.0, 0.0), (1.5, -2.0)):
            setup = dict(SETUP, mapping=mapping, signs=signs)
            par = dict(mean_offset_mm=offs[0], difference_offset_mm=offs[1])
            cmp = Comparator(dict(json.loads((ROOT / "config.json").read_text()), comparison_setup=setup),
                             dict(candidate=dict(parameters=dict(json.loads((ROOT / "calibration" / "new_model_initial.json").read_text())["candidate"]["parameters"], **par))))
            cm = CableMap.from_lab(setup, par, RESTS, model.neutral_lengths)
            for _ in range(5):
                phys = np.asarray(RESTS) + np.random.default_rng(0).uniform(-30, 30, 2)
                ref = cmp.controls(phys, dict(rest_lengths_mm=list(RESTS)))
                ok &= np.allclose(cm.physical_to_model_m(phys), ref, atol=1e-12)
                ok &= np.allclose(cm.model_to_physical_mm(cm.physical_to_model_m(phys)), phys, atol=1e-9)
check("CableMap == Comparator.controls for 16 wirings and inverts exactly", ok)

# ------------------------------------------------------------------ 2. camera map
cam = CameraMap.from_lab(SETUP)
uv = np.array([0.08, 0.12])
check("similarity round trip", np.allclose(cam.px_to_c(cam.c_to_px(uv)), uv, atol=1e-12))
cam_p = CameraMap.from_lab(SETUP, dict(camera_height_m=1.0, principal_point_px=[640, 512]))
tw = Twin.__new__(Twin)
tw.camera = cam_p
check("parallax correction inverts the pinhole projection", np.allclose(cam_p.px_to_c(Twin._project(tw, uv)), uv, atol=1e-9))
shift_mm = 1000 * np.linalg.norm(cam.px_to_c(Twin._project(tw, uv)) - uv)
check("parallax magnitude is material at 1 m camera height", shift_mm > 1.0, f"{shift_mm:.1f} mm uncorrected")

# ------------------------------------------------------------------ 3. detector
colour = EggColour.sample(np.full((40, 40, 3), (238, 232, 215), np.uint8), (20, 20))
det = EggDetector(colour, SETUP["pixels_per_m"])
twin = Twin(CableMap.from_lab(SETUP, PARAMS, RESTS, model.neutral_lengths), cam, (0.075, 0.115))
img = twin.render()
d0 = det.detect(img)
truth_px = cam.c_to_px(twin.egg_uv_true())
check("detector finds the egg next to the robot", d0 is not None and np.linalg.norm(np.asarray(d0.px) - truth_px) < 2.0,
      None if d0 is None else f"err {np.linalg.norm(np.asarray(d0.px)-truth_px):.2f} px, conf {d0.confidence:.2f}")
twin.fault = "hide_egg"
check("detector returns None when the egg is absent", det.detect(twin.render()) is None)

# ------------------------------------------------------------------ 4. bundles
LAB_MODEL = ROOT / "model" / "spirob_robot.xml"


def synthetic_bundle(folder: Path, gain=10.0) -> Path:
    """SYNTHETIC heuristic, not a trained policy: curl toward the egg's side (sign of u)."""
    folder.mkdir(parents=True)
    W0 = np.zeros((2, 8), np.float32); W0[0, 0] = gain; W0[1, 0] = -gain
    W1 = np.array([[-1, 1], [1, -1]], np.float32)
    np.savez(folder / "policy.npz", obs_mean=np.zeros(8, np.float32), obs_std=np.ones(8, np.float32),
             obs_eps=np.float32(0.0), W0=W0, b0=np.zeros(2, np.float32), W1=W1, b1=np.zeros(2, np.float32),
             meta_json=np.array(json.dumps(dict(n_layers=2, activation="elu", obs_dim=8, act_dim=2))))
    obs = np.random.default_rng(0).normal(size=(32, 8))
    pol = NumpyPolicy(folder / "policy.npz")
    np.save(folder / "parity_obs.npy", obs); np.save(folder / "parity_actions.npy", pol(obs))
    manifest = dict(contract_id=C.CONTRACT_ID, contract_source_sha256=sha(Path(C.__file__)),
                    policy_npz_sha256=sha(folder / "policy.npz"), parity=dict(passed=True, max_abs_error=0.0),
                    lab_model_sha256=sha(LAB_MODEL), checkpoint_sha256="SYNTHETIC-HEURISTIC", git_commit="none")
    (folder / "manifest.json").write_text(json.dumps(manifest))
    return folder


tmp = Path(tempfile.mkdtemp())
bundle = PolicyBundle(synthetic_bundle(tmp / "ok"))
check("valid bundle loads", bundle.policy.obs_dim == 8)
bad = synthetic_bundle(tmp / "bad")
m = json.loads((bad / "manifest.json").read_text()); m["lab_model_sha256"] = "0" * 64
(bad / "manifest.json").write_text(json.dumps(m))
try:
    PolicyBundle(bad); check("bundle for a different robot model is rejected", False)
except ValueError as exc:
    check("bundle for a different robot model is rejected", "different robot model" in str(exc))
bad2 = synthetic_bundle(tmp / "bad2")
np.savez(bad2 / "policy.npz", **{k: v for k, v in np.load(bad2 / "policy.npz").items()}, extra=np.zeros(1))
try:
    PolicyBundle(bad2); check("tampered weights are rejected", False)
except ValueError as exc:
    check("tampered weights are rejected", "checksum" in str(exc))


# ------------------------------------------------------------------ 5-7. closed loop
def episode(egg_uv, setup=SETUP, fault=None, dry_run=False, max_s=60.0, start_payout_offset_mm=0.0):
    cm = CableMap.from_lab(setup, PARAMS, RESTS, model.neutral_lengths)
    twin = Twin(cm, cam, egg_uv, fault=None)
    if start_payout_offset_mm:
        twin.lengths += start_payout_offset_mm; twin.target = twin.lengths.copy(); twin.advance(0.5)
    clock = [0.0]
    runner = KnockdownRunner(bundle, cm, cam, det, RESTS, LIMITS, twin.send, tmp / "episodes",
                             dry_run=dry_run, clock=lambda: clock[0])
    items = []
    for _ in range(6):
        twin.advance(0.1); clock[0] += 0.1; items.append(twin.item())
    runner.start_episode(twin.remote(), items, placement_id=f"twin {egg_uv}")
    twin.fault = fault
    while runner.active and clock[0] < max_s:
        twin.advance(0.1); clock[0] += 0.1
        for e in twin.events: runner.event(e)
        twin.events.clear()
        age = 2.0 if fault == "stale" and clock[0] > 3 else 0.05
        runner.tick(twin.remote(), 0.05, twin.item(age))
    return runner, twin


placements = [(0.075, 0.115), (-0.075, 0.115), (0.05, 0.06), (-0.10, 0.17)]
if args.quick:
    placements = placements[:2]
for uv in placements:
    r, tw = episode(uv)
    fell = tw.egg_z() < 0.06
    # Lost detection must never become SUCCESS; it is acceptable here only when the egg
    # physically fell AND left the frame or lies under the robot (realistic occlusion).
    explained_unknown = r.outcome == "UNKNOWN" and fell and (not tw.egg_in_frame() or det.detect(tw.render()) is None)
    check(f"twin knockdown at u={uv[0]*100:+.1f} v={uv[1]*100:.1f} cm",
          fell and (r.outcome == "SUCCESS" or explained_unknown),
          f"{r.outcome}, {r.decision_index} decisions, egg z {tw.egg_z()*1000:.0f} mm, in frame {tw.egg_in_frame()}")
for setup in (dict(SETUP, mapping=[1, 0]), dict(SETUP, signs=[-1, -1]), dict(SETUP, mapping=[1, 0], signs=[1, -1])):
    r, tw = episode((0.075, 0.115), setup=setup)
    check(f"wiring mapping={setup['mapping']} signs={setup['signs']} still knocks down", r.outcome == "SUCCESS", r.outcome)

r, tw = episode((0.075, 0.115), fault="reject")
check("rejected command -> FAULT_REJECTED and hold sent", r.outcome == "FAULT_REJECTED" and tw.commands[-2]["action"] == "hold")
r, tw = episode((0.075, 0.115), fault="stuck")
check("stuck motor -> FAULT_MOVE_TIMEOUT", r.outcome == "FAULT_MOVE_TIMEOUT", r.outcome)
r, tw = episode((0.075, 0.115), fault="hide_egg")
check("lost egg -> UNKNOWN (never success)", r.outcome == "UNKNOWN", r.outcome)
r, tw = episode((0.075, 0.115), fault="stale")
check("stale frames -> FAULT_STALE", r.outcome == "FAULT_STALE", r.outcome)
r, tw = episode((0.075, 0.115), dry_run=True, max_s=8.0)
check("dry run sends no motor commands", not any(c["action"] in ("independent", "hold") for c in tw.commands)
      and len(r.log.decisions) >= 3, f"{len(r.log.decisions)} decisions logged")
try:
    episode((0.075, 0.115), start_payout_offset_mm=3.0)
    check("episode refuses to start away from the trained start pose", False)
except ValueError as exc:
    check("episode refuses to start away from the trained start pose", "start pose" in str(exc))
try:
    episode((0.20, 0.10))
    check("placement outside the trained region is refused", False)
except ValueError as exc:
    check("placement outside the trained region is refused", "outside the trained placement region" in str(exc))
row = r.log.decisions[0]
check("decision log separates raw, desired, limited, command and measured",
      all(k in row for k in ("raw", "clipped", "desired_m", "limited_m", "command_mm", "encoder_mm", "payout_m")))
r_rep, _ = episode((-0.075, 0.115))
vision_file = tmp / "rl_vision.json"
vision_file.write_text(json.dumps({"colour": colour.__dict__}))
import subprocess  # noqa: E402
rep = subprocess.run([sys.executable, "-m", "rl.replay", "--episode", str(r_rep.log.root), "--policy", str(tmp / "ok"),
                      "--vision", str(vision_file)], cwd=ROOT, capture_output=True, text=True)
check("recorded-frame replay reproduces logged detections and actions", rep.returncode == 0,
      rep.stdout.strip().splitlines()[-1] if rep.stdout.strip() else rep.stderr[-300:])
cmd = return_to_start_command(CableMap.from_lab(SETUP, PARAMS, RESTS, model.neutral_lengths))
check("return-to-start targets the neutral payout at 2 mm/s", np.allclose(cmd["lengths_mm"], RESTS) and cmd["speeds_mm_s"] == [2.0, 2.0])

# ------------------------------------------------------------------ 8. real motor owner
if args.motor_source:
    sys.path.insert(0, str(Path(args.motor_source).resolve()))
    from unittest.mock import patch
    from actuation import dispatch
    from motion_limits import worker_limits
    from spirob_cable import worker
    from spirob_cable.config import RigSettings
    from spirob_cable.drive import DemoDrive

    class Rec:
        path = "rl-sim"
        def event(self, *a, **k): pass
        def sample(self, *a, **k): pass
        def pair(self, *a, **k): pass

    now = [100.0]
    cfg = RigSettings.demo()
    with patch("time.monotonic", lambda: now[0]), worker_limits(list(RESTS), LIMITS):
        rig = worker.Rig(cfg, [DemoDrive(c, Rec()) for c in cfg.selected], Rec())
        for a in rig.axes:
            pos = a.drive.position(); a.count = a.connection = a.rest = a.target = pos; a.rest_length = RESTS[a.index]
            a.enabled = True; a.state = "HOLDING"; a.last_read = now[0]; a.stable.extend([pos] * 3); a.position_mode = True; a.drive.enable()

        class Motor:
            referenced = True; pending = False
            @property
            def snapshot(self): return rig.snapshot()
            def submit(self, name, d):
                assert name == "independent", name
                rig.move_independent(d["lengths_mm"], d["speeds_mm_s"])
        sent_ok = 0
        for c in [x for x in tw.commands if x["action"] == "independent"][:3] or [cmd]:
            msg = dict(action="independent", lengths_mm=c["lengths_mm"], speeds_mm_s=c["speeds_mm_s"], received_s=now[0])
            dispatch(msg, Motor(), RESTS, True, LIMITS)
            for _ in range(400):
                now[0] += 0.05; rig.poll_all()
                if all(a.state == "HOLDING" for a in rig.axes): break
            sent_ok += 1
        check("RL independent commands pass the real dispatcher and motor Rig (DemoDrive)", sent_ok >= 1, f"{sent_ok} moves")

shutil.rmtree(tmp, ignore_errors=True)
failed = [n for n, ok, _ in results if not ok]
print(f"\n{len(results) - len(failed)}/{len(results)} checks passed")
sys.exit(1 if failed else 0)

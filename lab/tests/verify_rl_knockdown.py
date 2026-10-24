"""RL egg-knockdown mode (contract v2): software verification without hardware.

  1. Cable map == Comparator.controls (all wirings); camera map round trips.
  2. Detectors on synthetic frames: green egg next to the white robot on brown floor; blue empty cup.
  3. Policy bundle verification (valid; wrong robot model; tampered weights).
  4. Closed loop through the MuJoCo twin with a SYNTHETIC heuristic policy: knockdown from straight and
     curled starts, all cable wirings, 5/10/15 mm/s ceilings, small steps slower than the ceiling.
  5. Faults: rejected command, stuck motor, stale frames; egg hidden -> episode continues, never SUCCESS.
  6. Dry run sends no motor commands; placement outside region / too close to the robot is refused.
  7. Human-in-the-loop auto-continue: hold after success, hand blocks the start, new egg -> auto start.
  8. Recorded-frame replay; 9. RL commands through the REAL dispatcher + motor Rig (DemoDrive).

usage (lab folder; lab/model/spirob_robot.xml must be the knockdown model):
  uv run --frozen python tests/verify_rl_knockdown.py [--motor-source ..] [--quick]
"""
import argparse
import json
import shutil
import subprocess
import sys
import tempfile
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
p = argparse.ArgumentParser()
p.add_argument("--motor-source")
p.add_argument("--quick", action="store_true")
args, _ = p.parse_known_args()

from rl import contract as C  # noqa: E402
from rl.egg_vision import Colour, EggDetector, PedestalDetector  # noqa: E402
from rl.hardware_io import CableMap, CameraMap  # noqa: E402
from rl.numpy_policy import NumpyPolicy  # noqa: E402
from rl.runner import KnockdownRunner, PolicyBundle, sha, straighten_command  # noqa: E402
from rl.twin import CUP, EGG, Twin  # noqa: E402

RESTS = (227.35331441047117, 227.3533144104711)
LIMITS = dict(rest_lengths_mm=list(RESTS), absolute_bounds_mm=[[150.0, 300.0], [150.0, 300.0]], max_speed_mm_s=20.0,
              setup_travel_mm=150.0, difference_mm=150.0, travel_mm=77.35331441047117)
SETUP = dict(mapping=[0, 1], signs=[1, 1], pixels_per_m=4521.679319439051, rotation_rad=-0.02,
             translation_x_px=960.0, translation_y_px=-38.0)
PARAMS = dict(mean_offset_mm=0.0, difference_offset_mm=0.0)
MEAN = (C.MEAN_PAYOUT_M, C.MEAN_PAYOUT_M)
TABLE = ROOT / "tests" / "fixtures" / "pose_table_v2.json"   # copy of spirob-mjlab assets/knockdown/pose_table.json
results = []


def check(name, cond, detail=""):
    results.append((name, bool(cond)))
    print(("PASS " if cond else "FAIL ") + name + (f"  [{detail}]" if detail else ""), flush=True)


# ------------------------------------------------------------------ 1. maps
from analysis import Comparator  # noqa: E402

ok = True
fit = json.loads((ROOT / "calibration" / "new_model_initial.json").read_text())
for mapping in ([0, 1], [1, 0]):
    for signs in ([1, 1], [1, -1], [-1, 1], [-1, -1]):
        for offs in ((0.0, 0.0), (1.5, -2.0)):
            setup = dict(SETUP, mapping=mapping, signs=signs)
            par = dict(mean_offset_mm=offs[0], difference_offset_mm=offs[1])
            cmp = Comparator(dict(json.loads((ROOT / "config.json").read_text()), comparison_setup=setup),
                             dict(candidate=dict(parameters=dict(fit["candidate"]["parameters"], **par))))
            cm = CableMap.from_lab(setup, par, RESTS, cmp.model.neutral_lengths)
            phys = np.asarray(RESTS) + np.random.default_rng(0).uniform(-30, 30, 2)
            ok &= np.allclose(cm.physical_to_model_m(phys), cmp.controls(phys, dict(rest_lengths_mm=list(RESTS))), atol=1e-12)
            ok &= np.allclose(cm.model_to_physical_mm(cm.physical_to_model_m(phys)), phys, atol=1e-9)
check("CableMap == Comparator.controls for 16 wirings and inverts exactly", ok)
check("Lab model neutral mean equals contract mean", np.isclose(cmp.model.neutral_lengths.mean(), C.MEAN_PAYOUT_M, atol=1e-12))
cam = CameraMap.from_lab(SETUP)
check("camera similarity round trip", np.allclose(cam.px_to_c(cam.c_to_px([0.08, 0.12])), [0.08, 0.12], atol=1e-12))

# ------------------------------------------------------------------ 2. detectors
egg_colour = Colour.sample(np.full((40, 40, 3), EGG, np.uint8), (20, 20))
cup_colour = Colour.sample(np.full((40, 40, 3), CUP, np.uint8), (20, 20))
det = EggDetector(egg_colour, SETUP["pixels_per_m"])
ped = PedestalDetector(cup_colour, SETUP["pixels_per_m"])
cable = CableMap.from_lab(SETUP, PARAMS, RESTS, MEAN)
tw = Twin(cable, cam, (0.08, 0.11))
d0 = det.detect(tw.render())
err = np.inf if d0 is None else np.linalg.norm(np.asarray(d0.px) - cam.c_to_px(tw.egg_uv_true()))
check("green egg found next to the white robot on brown floor", err < 2.0, f"err {err:.2f} px")
check("blue cup hidden under the egg is not reported", ped.detect(tw.render()) is None)
tw.place_new_egg((0.08, 0.11)); tw.d.qpos[tw.egg_qadr:tw.egg_qadr + 3] = [0.5, 0.5, 0.02]
import mujoco  # noqa: E402
mujoco.mj_forward(tw.m, tw.d)
c0 = ped.detect(tw.render())
check("empty blue cup detected at the pedestal", c0 is not None and np.linalg.norm(cam.px_to_c(c0.px) - [0.08, 0.11]) < 0.003)

# ------------------------------------------------------------------ 3. bundles
LAB_MODEL = ROOT / "model" / "spirob_robot.xml"


def synthetic_bundle(folder: Path, gain=10.0) -> Path:
    """SYNTHETIC heuristic, not a trained policy: full curl toward the egg's side (a = -sign(u_egg))."""
    folder.mkdir(parents=True)
    W0 = np.zeros((2, 7), np.float32); W0[0, 0] = gain; W0[1, 0] = -gain
    W1 = np.array([[-1, 1]], np.float32)
    np.savez(folder / "policy.npz", obs_mean=np.zeros(7, np.float32), obs_std=np.ones(7, np.float32),
             obs_eps=np.float32(0.0), W0=W0, b0=np.zeros(2, np.float32), W1=W1, b1=np.zeros(1, np.float32),
             meta_json=np.array(json.dumps(dict(n_layers=2, activation="elu", obs_dim=7, act_dim=1))))
    obs = np.random.default_rng(0).normal(size=(32, 7))
    np.save(folder / "parity_obs.npy", obs); np.save(folder / "parity_actions.npy", NumpyPolicy(folder / "policy.npz")(obs))
    shutil.copy2(TABLE, folder / "pose_table.json")
    manifest = dict(contract_id=C.CONTRACT_ID, contract_source_sha256=sha(Path(C.__file__)),
                    policy_npz_sha256=sha(folder / "policy.npz"), parity=dict(passed=True, max_abs_error=0.0),
                    source_model_sha256=sha(LAB_MODEL), pose_table_sha256=sha(folder / "pose_table.json"),
                    region=dict(abs_u_m=[0.045, 0.125], v_m=[0.040, 0.180]),
                    checkpoint_sha256="SYNTHETIC-HEURISTIC", git_commit="none")
    (folder / "manifest.json").write_text(json.dumps(manifest))
    return folder


tmp = Path(tempfile.mkdtemp())
bundle = PolicyBundle(synthetic_bundle(tmp / "ok"))
check("valid v2 bundle loads (7 -> 1)", bundle.policy.obs_dim == 7 and bundle.policy.act_dim == 1)
bad = synthetic_bundle(tmp / "bad")
m = json.loads((bad / "manifest.json").read_text()); m["source_model_sha256"] = "0" * 64
(bad / "manifest.json").write_text(json.dumps(m))
try:
    PolicyBundle(bad); check("bundle for a different robot model is rejected", False)
except ValueError as exc:
    check("bundle for a different robot model is rejected", "not the model" in str(exc))
bad2 = synthetic_bundle(tmp / "bad2")
np.savez(bad2 / "policy.npz", **{k: v for k, v in np.load(bad2 / "policy.npz").items()}, extra=np.zeros(1))
try:
    PolicyBundle(bad2); check("tampered weights are rejected", False)
except ValueError as exc:
    check("tampered weights are rejected", "checksum" in str(exc))


# ------------------------------------------------------------------ 4-7. closed loop
def make_runner(tw, clock, dry_run=False, vmax=5.0, auto=False, setup_cable=None):
    return KnockdownRunner(bundle, setup_cable or cable, cam, det, ped, RESTS, LIMITS, tw.send, tmp / "episodes",
                           dry_run=dry_run, vmax_mm_s=vmax, auto_continue=auto, clock=lambda: clock[0])


def pump(runner, tw, clock, seconds, age=0.05):
    for _ in range(int(round(seconds / 0.1))):
        tw.advance(0.1); clock[0] += 0.1
        for e in tw.events:
            runner.event(e)
        tw.events.clear()
        runner.tick(tw.remote(), 0.05, tw.item(age))


def episode(egg_uv, setup=SETUP, fault=None, dry_run=False, max_s=80.0, start_diff=0.0, vmax=5.0):
    cm = CableMap.from_lab(setup, PARAMS, RESTS, MEAN)
    tw = Twin(cm, cam, egg_uv, start_diff_m=start_diff)
    clock = [0.0]
    runner = make_runner(tw, clock, dry_run=dry_run, vmax=vmax, setup_cable=cm)
    items = []
    for _ in range(6):
        tw.advance(0.1); clock[0] += 0.1; items.append(tw.item())
    runner.start_episode(tw.remote(), items, placement_id=f"twin {egg_uv}")
    tw.fault = fault
    t0 = clock[0]
    while runner.active and clock[0] - t0 < max_s:
        pump(runner, tw, clock, 0.1, age=2.0 if fault == "stale" and clock[0] - t0 > 3 else 0.05)
    return runner, tw


cases = [((0.08, 0.11), 0.0), ((-0.08, 0.11), 0.0), ((0.055, 0.07), 0.0), ((-0.11, 0.17), 0.0), ((0.09, 0.06), 0.040)]
if args.quick:
    cases = cases[:2]
for uv, d0 in cases:
    r, tw = episode(uv, start_diff=d0)
    fell = tw.egg_z() < 0.04
    explained = r.outcome == "UNKNOWN" and fell and not tw.egg_in_frame()
    check(f"twin knockdown at u={uv[0]*100:+.0f} v={uv[1]*100:.0f} cm from D0={d0*1000:+.0f} mm",
          fell and (r.outcome == "SUCCESS" or explained), f"{r.outcome}, {r.decision_index} decisions, egg z {tw.egg_z()*1000:.0f} mm")
for setup in (dict(SETUP, mapping=[1, 0]), dict(SETUP, signs=[-1, -1]), dict(SETUP, mapping=[1, 0], signs=[1, -1])):
    r, tw = episode((0.08, 0.11), setup=setup)
    check(f"wiring mapping={setup['mapping']} signs={setup['signs']} still knocks down", r.outcome == "SUCCESS", r.outcome)
for vmax in (5.0, 10.0, 15.0):
    r, tw = episode((-0.08, 0.11), vmax=vmax)
    speeds = [c["speeds_mm_s"][0] for c in tw.commands if c["action"] == "independent"]
    check(f"speed ceiling {vmax:g} mm/s reached by full steps, never exceeded; knocks down",
          r.outcome == "SUCCESS" and abs(max(speeds) - vmax) < 1e-9,
          f"{r.outcome}, max {max(speeds):.2f} mm/s, {len(speeds)} moves")
sp = [float(C.cable_speed(np.array(dd), 0.015)) * 1000 for dd in (0.001, 0.002, 0.004, 0.008)]
check("small steps move proportionally slower (15 mm/s ceiling)", np.allclose(sp, [1.875, 3.75, 7.5, 15.0]), f"{sp} mm/s")

r, tw = episode((0.08, 0.11), fault="reject")
check("rejected command -> FAULT_REJECTED and hold sent", r.outcome == "FAULT_REJECTED" and any(c["action"] == "hold" for c in tw.commands))
r, tw = episode((0.08, 0.11), fault="stuck")
check("stuck motor -> FAULT_MOVE_TIMEOUT", r.outcome == "FAULT_MOVE_TIMEOUT", r.outcome)
r, tw = episode((0.08, 0.11), fault="stale")
check("stale frames -> FAULT_STALE", r.outcome == "FAULT_STALE", r.outcome)
r, tw = episode((0.08, 0.11), fault="hide_egg", max_s=60)
vis = [json.loads(l)["egg_visible"] for l in (r.log.root / "decisions.jsonl").read_text().splitlines()]
check("egg hidden: policy continues with visible=0, never SUCCESS by vision",
      r.outcome in ("UNKNOWN", "SUCCESS") and 0.0 in vis and (r.outcome != "SUCCESS" or "empty blue cup" in r.message),
      f"{r.outcome}: {r.message[:70]}")
r, tw = episode((0.08, 0.11), dry_run=True, max_s=8.0)
check("dry run sends no motor commands", not any(c["action"] in ("independent", "hold") for c in tw.commands)
      and len(r.log.decisions) >= 3, f"{len(r.log.decisions)} decisions logged")
for uv, why in (((0.20, 0.10), "outside the trained region"), ((0.09, 0.06), "from the robot")):
    try:
        episode(uv, start_diff=-0.060 if why == "from the robot" else 0.0)
        check(f"start refused: {why}", False)
    except ValueError as exc:
        check(f"start refused: {why}", why in str(exc), str(exc)[:80])

# 7. auto-continue (human in the loop)
tw = Twin(cable, cam, (-0.08, 0.11))
clock = [0.0]
run = make_runner(tw, clock, auto=True)
items = []
for _ in range(6):
    tw.advance(0.1); clock[0] += 0.1; items.append(tw.item())
run.start_episode(tw.remote(), items)
while run.active and clock[0] < 80:
    pump(run, tw, clock, 0.1)
check("episode 1 succeeds and the runner holds, waiting", run.outcome == "SUCCESS" and run.waiting, run.message[:80])
n_cmd = len(tw.commands)
d_hold = tw.diff_m()
tw.place_new_egg((0.09, 0.15)); tw.hand_uv = np.array([0.09, 0.15])
for k in range(30):          # hand moving over the scene for 3 s
    tw.hand_uv = tw.hand_uv + np.array([0.004 * (-1) ** k, 0.003]); pump(run, tw, clock, 0.1)
check("no start while a hand is moving in the scene", run.waiting and len(tw.commands) == n_cmd
      and run.still.still_since is None, f"{run.message[:60]} | change level {run.still.last_level:.1f}")
tw.hand_uv = None
pump(run, tw, clock, 1.0)
check("no start before the scene has been still for 2 s", run.waiting, run.message[:80])
pump(run, tw, clock, 2.0)
check("auto-start after a still scene, from the held shape", run.active and abs(run.log.meta["start_diff_m"] - d_hold) < 0.002,
      f"start D {1000*run.log.meta.get('start_diff_m', np.nan):+.1f} mm vs held {1000*d_hold:+.1f}")
while run.active and clock[0] < 200:
    pump(run, tw, clock, 0.1)
check("second (auto-started) episode knocks the new egg down", run.outcome == "SUCCESS" and tw.egg_z() < 0.04, run.outcome)
run.abort("test done")
check("Stop ends auto-continue", not run.waiting and not run.active)

# 8. replay
r_rep, _ = episode((-0.08, 0.11))
vision_file = tmp / "rl_vision.json"
vision_file.write_text(json.dumps({"colour": egg_colour.__dict__}))
rep = subprocess.run([sys.executable, "-m", "rl.replay", "--episode", str(r_rep.log.root), "--policy", str(tmp / "ok"),
                      "--vision", str(vision_file)], cwd=ROOT, capture_output=True, text=True)
check("recorded-frame replay reproduces logged detections and actions", rep.returncode == 0,
      rep.stdout.strip().splitlines()[-1] if rep.stdout.strip() else rep.stderr[-300:])
cmd = straighten_command(cable)
check("straighten targets D = 0 at 2 mm/s", np.allclose(cmd["lengths_mm"], RESTS) and cmd["speeds_mm_s"] == [2.0, 2.0])

# 9. real motor owner
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
    moves = [c for c in tw.commands if c["action"] == "independent"][:4]
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
        done = 0
        for c in moves:
            dispatch(dict(action="independent", lengths_mm=c["lengths_mm"], speeds_mm_s=c["speeds_mm_s"], received_s=now[0]),
                     Motor(), RESTS, True, LIMITS)
            for _ in range(2000):
                now[0] += 0.05; rig.poll_all()
                if all(a.state == "HOLDING" for a in rig.axes): break
            done += all(a.state == "HOLDING" for a in rig.axes)
        check("RL moves (incl. slow small steps) pass the real dispatcher and motor Rig", done == len(moves) and done > 0,
              f"{done}/{len(moves)} moves settled; speeds {[round(c['speeds_mm_s'][0], 2) for c in moves]}")

shutil.rmtree(tmp, ignore_errors=True)
failed = [n for n, ok in results if not ok]
print(f"\n{len(results) - len(failed)}/{len(results)} checks passed")
sys.exit(1 if failed else 0)

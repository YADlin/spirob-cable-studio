"""Recorded-camera replay: re-run detection + policy on a logged RL episode.

For every decision in <episode>/decisions.jsonl this re-detects the egg in the
saved frame dNN.jpg, rebuilds the contract observation from the LOGGED payout and
previous action, runs the policy, and compares with what was logged.

usage (lab folder):
  uv run --frozen python -m rl.replay --episode logs/runs/<session>/rl_episodes/<id> --policy <export folder>
Exit status 1 if any decision differs by more than the tolerances.
"""

from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path

import cv2
import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from rl import contract as C  # noqa: E402
from rl.egg_vision import EggColour, EggDetector  # noqa: E402
from rl.hardware_io import CameraMap  # noqa: E402
from rl.runner import PolicyBundle  # noqa: E402


def main():
    p = argparse.ArgumentParser()
    p.add_argument("--episode", required=True)
    p.add_argument("--policy", required=True)
    p.add_argument("--vision", default=str(Path(__file__).resolve().parents[1] / "rl_vision.json"))
    p.add_argument("--px-tol", type=float, default=3.0, help="pixels; logged frames are JPEG (q95)")
    p.add_argument("--act-tol", type=float, default=0.05,
                   help="raw-action tolerance; frames are re-read from JPEG, so centroids shift slightly")
    a = p.parse_args()
    ep = Path(a.episode)
    meta = json.loads((ep / "episode.json").read_text())
    vision = json.loads(Path(a.vision).read_text())
    camera = CameraMap(**{k: (tuple(v) if isinstance(v, list) else v) for k, v in meta["camera_map"].items()})
    det = EggDetector(EggColour(**vision["colour"]), camera.pixels_per_m)
    bundle = PolicyBundle(a.policy)
    if bundle.manifest["checkpoint_sha256"] != meta["policy"]["checkpoint_sha256"]:
        print("WARNING: replaying with a different policy than the one logged")
    start = np.asarray(meta["egg_start_uv_m"])
    worst_px = worst_act = 0.0
    rows = [json.loads(l) for l in (ep / "decisions.jsonl").read_text().splitlines()]
    for r in rows:
        img = cv2.imread(str(ep / f"d{r['decision']:02d}.jpg"))
        if img is None:
            print(f"decision {r['decision']}: frame missing"); continue
        d = det.detect(cv2.cvtColor(img, cv2.COLOR_BGR2RGB))
        if d is None:
            print(f"decision {r['decision']}: egg NOT re-detected"); worst_px = float("inf"); continue
        dpx = float(np.linalg.norm(np.asarray(d.px) - np.asarray(r["egg_px"])))
        prev = np.asarray(rows[r["decision"] - 1]["clipped"]) if r["decision"] > 0 else np.zeros(2)
        obs = C.build_actor_obs(camera.px_to_c(d.px), start, np.asarray(r["payout_m"]), prev)
        raw = bundle.policy(obs)
        dact = float(np.max(np.abs(raw - np.asarray(r["raw"]))))
        worst_px, worst_act = max(worst_px, dpx), max(worst_act, dact)
        print(f"decision {r['decision']:2d}: egg px diff {dpx:6.2f}  action diff {dact:.2e}")
    ok = worst_px <= a.px_tol and worst_act <= a.act_tol
    print(f"worst egg px diff {worst_px:.2f}, worst action diff {worst_act:.2e} -> {'OK' if ok else 'DIFFERS'}")
    sys.exit(0 if ok else 1)


if __name__ == "__main__":
    main()

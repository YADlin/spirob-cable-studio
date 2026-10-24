"""RL egg-knockdown episode runner for the Lab (workstation side, GUI thread).

Modelled on campaign.CampaignRunner: it only sends acknowledged
``independent`` moves through the existing motor owner and never enables,
references or retries. One decision =

    observe (settled encoders + fresh frame) -> detect egg -> contract obs
    -> NumpyPolicy -> contract transform -> physical mm -> send -> wait for hold

States: idle, observe, ack, moving, done (outcome set).
Outcomes: SUCCESS, FAILURE_TIMEOUT, UNKNOWN (perception), ABORTED (operator Stop),
          FAULT_* (infrastructure). Only SUCCESS counts as a knockdown.
"""

from __future__ import annotations

import hashlib
import json
import math
import time
import uuid
from collections import deque
from dataclasses import dataclass, field
from pathlib import Path

import numpy as np

from rl import contract as C
from rl.hardware_io import CableMap, CameraMap
from rl.numpy_policy import NumpyPolicy

LAB_DIR = Path(__file__).resolve().parents[1]
START_TOLERANCE_M = 0.0010          # robot must be within 1 mm of neutral payout at episode start
START_FRAMES = 5                    # stable egg detections required to start
START_SPREAD_M = 0.002
REGION_TOLERANCE_M = 0.005
DETECTION_WAIT_S = 1.0              # missing egg for this long at a decision -> UNKNOWN
SETTLE_WINDOW_S = 0.3
ACK_TIMEOUT_S = 3.0


def sha(path: Path) -> str:
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


class PolicyBundle:
    """A verified export folder from spirob-mjlab tools/knockdown/export_policy.py."""

    def __init__(self, folder: str | Path, lab_model: Path | None = None):
        self.folder = Path(folder)
        m = json.loads((self.folder / "manifest.json").read_text())
        problems = []
        if m.get("contract_id") != C.CONTRACT_ID:
            problems.append(f"contract id {m.get('contract_id')} != Lab {C.CONTRACT_ID}")
        if m.get("contract_source_sha256") != sha(Path(C.__file__)):
            problems.append("contract.py differs between the export and the Lab (lab/rl/contract.py)")
        if sha(self.folder / "policy.npz") != m.get("policy_npz_sha256"):
            problems.append("policy.npz checksum mismatch")
        if not m.get("parity", {}).get("passed"):
            problems.append("export parity check did not pass")
        lab_model = lab_model or LAB_DIR / "model" / "spirob_robot.xml"
        if sha(lab_model) != m.get("lab_model_sha256"):
            problems.append("policy was trained on a different robot model than lab/model/spirob_robot.xml")
        self.policy = NumpyPolicy(self.folder / "policy.npz")
        if self.policy.obs_dim != C.DEFAULT.actor_dim or self.policy.act_dim != 2:
            problems.append("policy input/output size does not match the contract")
        obs_p, act_p = self.folder / "parity_obs.npy", self.folder / "parity_actions.npy"
        if obs_p.is_file():
            err = float(np.max(np.abs(self.policy(np.load(obs_p)) - np.load(act_p))))
            if err > 1e-4:
                problems.append(f"Lab-side parity error {err:.2e} exceeds 1e-4")
            self.lab_parity_error = err
        if problems:
            raise ValueError("Policy rejected: " + "; ".join(problems))
        self.manifest = m

    def describe(self) -> str:
        return (f"{self.folder.name}: checkpoint {self.manifest['checkpoint_sha256'][:12]}, "
                f"commit {str(self.manifest.get('git_commit'))[:10]}, parity {self.manifest['parity']['max_abs_error']:.1e}")


@dataclass
class EpisodeLog:
    root: Path
    episode_id: str
    meta: dict
    decisions: list = field(default_factory=list)

    def __post_init__(self):
        self.root.mkdir(parents=True, exist_ok=False)
        self._write("episode.json", self.meta)

    def _write(self, name, data):
        tmp = self.root / (name + ".tmp")
        tmp.write_text(json.dumps(data, indent=2, default=_jsonable))
        tmp.replace(self.root / name)

    def decision(self, row: dict):
        self.decisions.append(row)
        with (self.root / "decisions.jsonl").open("a") as f:
            f.write(json.dumps(row, default=_jsonable) + "\n")

    def frame(self, name: str, rgb):
        try:
            import cv2
            cv2.imwrite(str(self.root / name), cv2.cvtColor(rgb, cv2.COLOR_RGB2BGR), [cv2.IMWRITE_JPEG_QUALITY, 95])
        except Exception:
            pass

    def close(self, **fields):
        self.meta.update(fields)
        self._write("episode.json", self.meta)


def _jsonable(x):
    if isinstance(x, np.ndarray):
        return x.tolist()
    if isinstance(x, (np.floating, np.integer)):
        return x.item()
    return str(x)


class KnockdownRunner:
    def __init__(self, bundle: PolicyBundle, cable: CableMap, camera: CameraMap, detector, rests_mm, limits: dict,
                 send, log_root: Path, dry_run: bool = True, clock=time.monotonic, region=None):
        self.bundle, self.cable, self.camera, self.detector = bundle, cable, camera, detector
        self.rests_mm, self.limits, self.send = tuple(rests_mm), limits, send
        self.log_root, self.dry_run, self.clock = Path(log_root), dry_run, clock
        self.spec = C.DEFAULT
        self.region = region or {"abs_u_m": (0.045, 0.105), "v_m": (0.055, 0.175)}
        self.state, self.outcome, self.message = "idle", None, "Load a policy, then Run episode"
        self.events = deque(maxlen=2000)
        self.log: EpisodeLog | None = None
        self._check_contract_fits_limits()

    # ------------------------------------------------------------------ setup
    def _check_contract_fits_limits(self):
        from actuation import independent_targets
        corners = [(lo0, lo1) for lo0 in self.spec.action.bounds_m[0] for lo1 in self.spec.action.bounds_m[1]]
        v = 1000 * self.spec.action.move_speed_m_s
        if v > self.limits["max_speed_mm_s"] + 1e-9:
            raise ValueError("Contract move speed exceeds this session's speed ceiling")
        for c in corners:
            independent_targets(list(self.cable.model_to_physical_mm(c)), [v, v], self.rests_mm, self.limits)

    @property
    def active(self) -> bool:
        return self.state in ("observe", "ack", "moving")

    def _update(self, state, message):
        self.state, self.message = state, message
        self.events.append(dict(state=state, message=message, t=self.clock()))

    # ------------------------------------------------------------------ episode lifecycle
    def start_episode(self, remote: dict, items: list[dict], placement_id: str, operator_note: str = ""):
        if self.active:
            raise ValueError("An episode is already running")
        item = items[-1]
        self.camera_only = False
        try:
            payout = self._payout_m(item)
        except (ValueError, KeyError) as exc:
            if not self.dry_run:
                raise ValueError("Referenced encoder samples are required: " + str(exc)) from exc
            self.camera_only = True          # shadow mode without motors: payout assumed neutral, flagged in logs
            payout = np.asarray(self.spec.action.neutral_m, dtype=float)
        if not self.camera_only:
            self._require_motor_ready(remote)
        if np.max(np.abs(payout - np.asarray(self.spec.action.neutral_m))) > START_TOLERANCE_M:
            raise ValueError("Robot is not at the trained start pose (neutral payout +-1 mm). Use Return to start first.")
        uvs = []
        for it in items[-START_FRAMES:]:
            det = self.detector.detect(it["rgb"])
            if det is None:
                raise ValueError("Egg not detected in every recent frame; check colour calibration and lighting")
            uvs.append(self.camera.px_to_c(det.px))
        uvs = np.array(uvs)
        if len(uvs) < START_FRAMES or np.max(np.linalg.norm(uvs - uvs.mean(0), axis=1)) > START_SPREAD_M:
            raise ValueError("Egg detection is not yet stable (need 5 frames within 2 mm)")
        start = uvs.mean(0)
        au, v = abs(start[0]), start[1]
        (a0, a1), (v0, v1) = self.region["abs_u_m"], self.region["v_m"]
        tol = REGION_TOLERANCE_M
        if not (a0 - tol <= au <= a1 + tol and v0 - tol <= v <= v1 + tol):
            raise ValueError(f"Egg at u={start[0]*100:.1f} cm, v={v*100:.1f} cm is outside the trained placement region")
        self.egg_start = start
        self.prev_action = np.zeros(2)
        self.prev_limited = payout.copy()      # reconcile with the measured robot state
        self.decision_index = 0
        self.last_command_t = -1e9
        self.target_mm = None if self.camera_only else [float(x) for x in self._lengths_mm(item)]
        self.off_since = None
        self.missing_since = None
        self.command_id = None
        eid = time.strftime("%Y%m%dT%H%M%S") + "_" + uuid.uuid4().hex[:6]
        self.log = EpisodeLog(self.log_root / eid, eid, dict(
            episode_id=eid, placement_id=placement_id, operator_note=operator_note, dry_run=self.dry_run,
            policy=self.bundle.manifest, contract_id=C.CONTRACT_ID, contract_sha256=sha(Path(C.__file__)),
            cable_map=self.cable.__dict__, cable_map_fingerprint=self.cable.fingerprint(),
            camera_map=self.camera.__dict__, camera_map_fingerprint=self.camera.fingerprint(),
            egg_start_uv_m=start, rests_mm=self.rests_mm, limits=self.limits, camera_only=self.camera_only,
            started_wall=time.time(),
            started_monotonic=self.clock(), outcome=None))
        self.log.frame("start.jpg", item["rgb"])
        if not self.dry_run:
            try:
                self.send("record", enabled=True)
            except Exception:
                pass
        self.outcome = None
        self._update("observe", f"Episode {eid} started ({'DRY RUN — no motor writes' if self.dry_run else 'ARMED'})")

    def finish(self, outcome: str, message: str, hold: bool = True):
        if hold and not self.dry_run:
            try:
                self.send("hold")
            except Exception:
                pass
        if not self.dry_run:
            try:
                self.send("record", enabled=False)
            except Exception:
                pass
        self.outcome = outcome
        if self.log:
            self.log.close(outcome=outcome, terminal_message=message, decisions=self.decision_index,
                           ended_monotonic=self.clock(), duration_s=self.clock() - self.log.meta["started_monotonic"])
        self._update("done", f"{outcome}: {message}")

    def abort(self, reason: str):
        if self.active:
            self.finish("ABORTED", reason)

    def operator_label(self, knocked_off: bool, note: str = ""):
        """Resolve an UNKNOWN episode from the recorded video (kept separate from automatic SUCCESS)."""
        if self.outcome != "UNKNOWN" or not self.log:
            raise ValueError("Only an UNKNOWN episode can be labelled manually")
        self.log.close(operator_label="KNOCKED_OFF" if knocked_off else "NOT_KNOCKED_OFF", operator_label_note=note,
                       operator_label_wall=time.time())

    # ------------------------------------------------------------------ messages from the motor owner
    def event(self, event: dict):
        if not self.active:
            return
        if event.get("type") in ("closing", "closed", "startup_failed", "transport_closed"):
            self.finish("FAULT_CONNECTION", "Connection ended: " + str(event.get("message", event["type"])), hold=False)
            return
        if self.state == "ack" and event.get("id") == self.command_id:
            if event["type"] == "rejected":
                self.finish("FAULT_REJECTED", "Motor owner rejected the command: " + str(event.get("message")))
            elif event["type"] == "accepted":
                self._update("moving", f"Decision {self.decision_index}: moving")

    # ------------------------------------------------------------------ main loop (called ~10 Hz)
    def tick(self, remote: dict, remote_age: float, item: dict | None, max_age: float = 0.5, max_gap: float = 0.65):
        if not self.active:
            return
        now = self.clock()
        if item is None or item.get("displayed_age_s", 1e9) > max_age or (not self.camera_only and remote_age > 1.5):
            self.finish("FAULT_STALE", "Fresh camera frame or motor status lost")
            return
        if not self.camera_only:
            try:
                self._require_motor_ready(remote, allow_moving=self.state in ("ack", "moving"))
            except ValueError as exc:
                self.finish("FAULT_MOTOR", str(exc))
                return
        det = self.detector.detect(item["rgb"])
        if self._check_success(det, now, item):
            return
        if self.state == "ack" and now - self.sent_t > ACK_TIMEOUT_S:
            self.finish("FAULT_ACK_TIMEOUT", "Command acknowledgement timed out; never retried")
            return
        if self.state == "moving":
            if now > self.move_deadline:
                self.finish("FAULT_MOVE_TIMEOUT", "Target not reached and settled in time")
                return
            from campaign import held_packet
            if remote.get("phase") == "HOLDING" and not remote.get("pending") and held_packet(
                    item["packet"], self.target_mm, SETTLE_WINDOW_S, max_gap):
                self._update("observe", f"Decision {self.decision_index}: settled")
            return
        if self.state != "observe":
            return
        if now - self.last_command_t < self.spec.action.decision_period_s:
            return
        if self.decision_index >= self.spec.episode_decisions:
            self.finish("FAILURE_TIMEOUT", f"No knockdown after {self.decision_index} decisions")
            return
        if det is None:
            self.missing_since = self.missing_since or now
            if now - self.missing_since > DETECTION_WAIT_S:
                self.finish("UNKNOWN", "Egg not detected (occlusion or out of view); label from video")
            return
        self.missing_since = None
        self._decide(item, det, now)

    # ------------------------------------------------------------------ internals
    def _require_motor_ready(self, remote, allow_moving=False):
        if self.dry_run:
            return
        axes = remote.get("axes", [])
        if len(axes) != 2 or any(not a.get("enabled") or a.get("rest_count") is None or a.get("state") in ("FAULT", "DISABLED")
                                 for a in axes):
            raise ValueError("Both motors must be enabled, referenced and healthy")
        if not allow_moving and (remote.get("phase") != "HOLDING" or remote.get("pending")):
            raise ValueError("Motors are not holding")
        if list(remote.get("rest_lengths_mm", self.rests_mm)) != list(self.rests_mm):
            raise ValueError("Rest references changed")

    def _lengths_mm(self, item) -> np.ndarray:
        from analysis import inputs_at
        lengths, _ = inputs_at(item["packet"], item["packet"]["t_s"], 0.65)
        return np.asarray(lengths)

    def _payout_m(self, item) -> np.ndarray:
        return self.cable.physical_to_model_m(self._lengths_mm(item))

    def _check_success(self, det, now, item) -> bool:
        s = self.spec.success
        if det is None:
            self.off_since = None
            return False
        uv = self.camera.px_to_c(det.px)
        if np.linalg.norm(uv - self.egg_start) > s.hw_off_radius_m:
            self.off_since = self.off_since or now
            if now - self.off_since >= s.hw_confirm_s:
                self.log.frame("success.jpg", item["rgb"])
                self.finish("SUCCESS", f"Egg seen {np.linalg.norm(uv-self.egg_start)*1000:.0f} mm from its start for "
                                       f"{now-self.off_since:.1f} s")
                return True
        else:
            self.off_since = None
        return False

    def _decide(self, item, det, now):
        a = self.spec.action
        payout = np.asarray(a.neutral_m, dtype=float) if self.camera_only else self._payout_m(item)
        egg_uv = self.camera.px_to_c(det.px)
        obs = C.build_actor_obs(egg_uv, self.egg_start, payout, self.prev_action)
        try:
            raw = self.bundle.policy(obs)
        except ValueError as exc:
            self.finish("FAULT_NUMERIC", str(exc))
            return
        clipped = C.clip_raw(raw, a)
        desired = C.desired_target(clipped, a)
        limited = C.limited_target(desired, payout if self.dry_run else self.prev_limited, a)
        lengths_mm = self.cable.model_to_physical_mm(limited)
        speed = 1000 * a.move_speed_m_s
        row = dict(decision=self.decision_index, t=now, frame_t_s=item["packet"].get("t_s"),
                   frame_age_s=item.get("displayed_age_s"), motor_age_s=item["packet"].get("motor_age_s"),
                   encoder_mm=None if self.camera_only else self._lengths_mm(item), payout_assumed=self.camera_only,
                   payout_m=payout, egg_px=det.px, egg_uv_m=egg_uv,
                   egg_confidence=det.confidence, egg_candidates=det.n_candidates, obs=obs, raw=raw,
                   clipped=clipped, desired_m=desired, limited_m=limited, command_mm=lengths_mm,
                   speed_mm_s=speed, dry_run=self.dry_run,
                   since_last_command_s=now - self.last_command_t if self.last_command_t > 0 else None)
        try:
            from actuation import independent_targets
            if not self.camera_only:
                independent_targets(list(lengths_mm), [speed, speed], self.rests_mm, self.limits)
        except ValueError as exc:
            row["rejected_locally"] = str(exc)
            self.log.decision(row)
            self.finish("FAULT_LIMIT", "Policy target failed Lab limit validation: " + str(exc))
            return
        self.log.frame(f"d{self.decision_index:02d}.jpg", item["rgb"])
        self.prev_action = np.asarray(clipped, dtype=float)
        self.prev_limited = np.asarray(limited, dtype=float)
        self.decision_index += 1
        self.last_command_t = now
        if self.dry_run:
            row["command_id"] = None
            self.log.decision(row)
            self._update("observe", f"Decision {self.decision_index} computed (dry run, not sent)")
            return
        travel = float(np.max(np.abs(lengths_mm - self._lengths_mm(item))) / speed)
        try:
            self.command_id = self.send("independent", lengths_mm=[float(x) for x in lengths_mm], speeds_mm_s=[speed, speed])
        except Exception as exc:
            row["send_error"] = str(exc)
            self.log.decision(row)
            self.finish("FAULT_SEND", "Could not send motion: " + str(exc))
            return
        row.update(command_id=self.command_id, expected_travel_s=travel)
        self.log.decision(row)
        self.target_mm = [float(x) for x in lengths_mm]
        self.sent_t = now
        self.move_deadline = now + 2 * travel + 6.0
        self._update("ack", f"Decision {self.decision_index}: awaiting acknowledgement")


def return_to_start_command(cable: CableMap, speed_mm_s: float = 2.0) -> dict:
    """Explicit, operator-confirmed move back to the trained start pose (never automatic)."""
    lengths = cable.model_to_physical_mm(np.asarray(C.NEUTRAL_PAYOUT_M))
    return dict(action="independent", lengths_mm=[float(x) for x in lengths], speeds_mm_s=[speed_mm_s, speed_mm_s])

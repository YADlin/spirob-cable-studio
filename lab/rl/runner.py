"""RL egg-knockdown episode runner for the Lab (contract v3; workstation side, GUI thread).

Sends only acknowledged 'independent' moves through the existing motor owner; never enables,
references or retries. One decision =

  observe (settled encoders + fresh frame) -> egg detector -> contract obs (7)
  -> NumpyPolicy -> difference target -> payouts -> physical mm, per-cable speed -> send -> wait for hold

States: idle, observe, ack, moving, await_egg (human-in-the-loop), done.
Outcomes: SUCCESS, FAILURE_TIMEOUT, UNKNOWN (egg not seen at the end), ABORTED, FAULT_*.

Human-in-the-loop (auto-continue): after an episode the robot holds where it is. The next episode
starts by itself when a new egg is detected, has been still for `still_s`, the whole image has
been still for `still_s` (no hand moving), and the egg is inside the trained region and clear of
the robot's current shape (pose table). After a non-success the egg must also have been moved.
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
from rl.egg_vision import SceneStillness
from rl.hardware_io import CableMap, CameraMap
from rl.numpy_policy import NumpyPolicy
from rl.live_feed import encoder_packet, age_item

LAB_DIR = Path(__file__).resolve().parents[1]
START_FRAMES = 5
START_SPREAD_M = 0.002
REGION_TOLERANCE_M = 0.005
SETTLE_WINDOW_S = 0.3
ACK_TIMEOUT_S = 3.0
# Fresh data is mandatory for policy decisions and success/settle checks.
# A bounded move is closed-loop inside the native motor owner. A brief delivery
# delay does not cancel that move; loss beyond this absolute age DOES send Hold.
# These are end-to-end delivery limits, not the interpolation gap or serial timeout.
SENSOR_DELIVERY_TIMEOUT_S = 1.0
FEEDBACK_RECOVERY_TIMEOUT_S = 1.0
LAB_MIN_SPEED_MM_S = 0.05
RETRY_MOVE_M = 0.020          # after a non-success, the egg must be moved this far before auto-start


def sha(path: Path) -> str:
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


class PolicyBundle:
    """A verified export folder from spirob-mjlab tools/knockdown/export_policy.py (contract v3)."""

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
        if sha(lab_model) != m.get("source_model_sha256"):
            problems.append("lab/model/spirob_robot.xml is not the model this policy was trained on "
                            f"(expected sha256 {str(m.get('source_model_sha256'))[:12]}...)")
        table_path = self.folder / "pose_table.json"
        if not table_path.is_file() or sha(table_path) != m.get("pose_table_sha256"):
            problems.append("pose_table.json missing or checksum mismatch")
        self.policy = NumpyPolicy(self.folder / "policy.npz")
        if self.policy.obs_dim != C.DEFAULT.actor_dim or self.policy.act_dim != 1:
            problems.append("policy input/output size does not match contract v3 (7 -> 1)")
        obs_p, act_p = self.folder / "parity_obs.npy", self.folder / "parity_actions.npy"
        self.lab_parity_error = float("nan")
        if obs_p.is_file():
            self.lab_parity_error = float(np.max(np.abs(self.policy(np.load(obs_p)) - np.load(act_p))))
            if self.lab_parity_error > 1e-4:
                problems.append(f"Lab-side parity error {self.lab_parity_error:.2e} exceeds 1e-4")
        for name, expected in m.get("perception_files", {}).items():
            if name not in ("egg_vision.py", "camera_geometry.py") or sha(Path(__file__).with_name(name)) != expected:
                problems.append("Perception source differs from evaluated export: " + str(name))
        if problems:
            raise ValueError("Policy rejected: " + "; ".join(problems))
        self.manifest = m
        self.table = C.load_pose_table(table_path)
        self.region = m.get("region", {"abs_u_m": (0.045, 0.125), "v_m": (0.040, 0.180)})

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

    def event(self, row: dict):
        with (self.root / "motor_events.jsonl").open("a") as f:
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
    def __init__(self, bundle: PolicyBundle, cable: CableMap, camera: CameraMap, egg_detector, pedestal_detector,
                 rests_mm, limits: dict, send, log_root: Path, dry_run: bool = True, vmax_mm_s: float = 5.0,
                 auto_continue: bool = False, clock=time.monotonic):
        if not np.isfinite(vmax_mm_s) or not 5.0 <= vmax_mm_s <= 20.0:
            raise ValueError("Common cable speed must be between 5 and 20 mm/s")
        self.bundle, self.cable, self.camera = bundle, cable, camera
        self.egg_det, self.ped_det = egg_detector, pedestal_detector
        self.rests_mm, self.limits, self.send = tuple(rests_mm), limits, send
        self.log_root, self.dry_run, self.clock = Path(log_root), dry_run, clock
        self.vmax_m_s = vmax_mm_s / 1000
        self.auto_continue = auto_continue
        self.spec = C.DEFAULT
        self.region = bundle.region
        self.state, self.outcome, self.message = "idle", None, "Load a policy, then Run episode"
        self.events = deque(maxlen=2000)
        self.recent = deque(maxlen=60)
        self.still = SceneStillness()
        self.log: EpisodeLog | None = None
        self.episodes: list[dict] = []
        self.last_start = None
        self.camera_only = False
        self._check_limits()

    # ------------------------------------------------------------------ setup checks
    def _check_limits(self):
        from actuation import independent_targets
        v = 1000 * self.vmax_m_s
        if v > self.limits["max_speed_mm_s"] + 1e-9:
            raise ValueError("Selected cable speed exceeds this session's speed ceiling")
        for d in (-self.spec.action.diff_limit_m, 0.0, self.spec.action.diff_limit_m):
            mm = self.cable.model_to_physical_mm(C.payouts_from_diff(d))
            independent_targets(list(mm), [v, v], self.rests_mm, self.limits)

    @property
    def active(self) -> bool:
        return self.state in ("observe", "ack", "moving")

    @property
    def waiting(self) -> bool:
        return self.state == "await_egg"

    def _update(self, state, message):
        self.state, self.message = state, message
        self.events.append(dict(state=state, message=message, t=self.clock()))

    # ------------------------------------------------------------------ measurements
    def _lengths_mm(self, item) -> np.ndarray:
        from analysis import inputs_at
        packet = encoder_packet(item)
        lengths, _ = inputs_at(packet, packet["t_s"], 0.65)
        return np.asarray(lengths)

    def _diff_m(self, item) -> float:
        if self.camera_only:
            return 0.0
        return float(C.diff_from_payouts(self.cable.physical_to_model_m(self._lengths_mm(item)))[0])

    def _egg_uv(self, item):
        det = item["rl_egg"] if item.get("rl_detected") else self.egg_det.detect(item["rgb"])
        return (None, None) if det is None else (self.camera.px_to_c(det.px), det)

    # ------------------------------------------------------------------ episode lifecycle
    def start_episode(self, remote: dict, items: list[dict], placement_id: str = "free", operator_note: str = ""):
        if self.active:
            raise ValueError("An episode is already running")
        item = items[-1]
        self.camera_only = False
        try:
            d_meas = self._diff_m(item)
        except (ValueError, KeyError) as exc:
            if not self.dry_run:
                raise ValueError("Referenced encoder samples are required: " + str(exc)) from exc
            self.camera_only, d_meas = True, 0.0
        if not self.camera_only:
            self._require_motor_ready(remote)
        uvs = []
        for it in items[-START_FRAMES:]:
            uv, _ = self._egg_uv(it)
            if uv is None:
                raise ValueError("Egg not detected in every recent frame; check colour calibration and lighting")
            uvs.append(uv)
        uvs = np.array(uvs)
        if len(uvs) < START_FRAMES or np.max(np.linalg.norm(uvs - uvs.mean(0), axis=1)) > START_SPREAD_M:
            raise ValueError("Egg detection is not yet stable (need 5 frames within 2 mm)")
        start = uvs.mean(0)
        self._check_placement(start, d_meas)
        self.egg_start = start
        self.last_seen, self.visible = start.copy(), 1.0
        self.prev_limited = d_meas
        self.prev_action = d_meas / self.spec.action.diff_limit_m
        self.decision_index = 0
        self.last_command_t = -1e9
        self.target_mm = None if self.camera_only else [float(x) for x in self._lengths_mm(item)]
        self.camera_max_age = 0.5
        self.encoder_max_age = 0.65
        self.feedback_wait_started = None
        self.feedback_wait_reason = None
        self.feedback_wait_count = 0
        self.off_since = self.empty_since = None
        self.command_id = None
        eid = time.strftime("%Y%m%dT%H%M%S") + "_" + uuid.uuid4().hex[:6]
        self.log = EpisodeLog(self.log_root / eid, eid, dict(
            episode_id=eid, placement_id=placement_id, operator_note=operator_note, dry_run=self.dry_run,
            camera_only=self.camera_only, vmax_mm_s=1000 * self.vmax_m_s, auto_started=self.waiting,
            policy=self.bundle.manifest, contract_id=C.CONTRACT_ID, contract_sha256=sha(Path(C.__file__)),
            cable_map=self.cable.__dict__, camera_map=self.camera.__dict__, egg_start_uv_m=start,
            start_diff_m=d_meas, rests_mm=self.rests_mm, limits=self.limits, started_wall=time.time(),
            started_monotonic=self.clock(), outcome=None,
            runtime_sources={name: sha(Path(__file__).with_name(name))
                             for name in ("runner.py", "live_feed.py", "rl_window.py")}))
        self.log.frame("start.jpg", item["rgb"])
        self.last_perception_key = None
        self.log.close(image_recording="Start/decision/success JPEGs; continuous recording is operator-controlled",
                       feedback_policy=dict(camera_decision_max_age_s=0.5, encoder_decision_max_age_s=0.65,
                           delivery_stop_age_s=SENSOR_DELIVERY_TIMEOUT_S,
                           recovery_wait_limit_s=FEEDBACK_RECOVERY_TIMEOUT_S,
                           delayed_data_behavior="No new decision, settle or success check; existing bounded move remains under native control"))
        self.outcome = None
        self.last_start = start
        self._update("observe", f"Episode {eid} started ({'DRY RUN — no motor writes' if self.dry_run else 'ARMED'}), "
                                f"D = {1000 * d_meas:+.1f} mm")

    def _check_placement(self, uv, d_meas):
        (a0, a1), (v0, v1) = self.region["abs_u_m"], self.region["v_m"]
        t = REGION_TOLERANCE_M
        if not (a0 - t <= abs(uv[0]) <= a1 + t and v0 - t <= uv[1] <= v1 + t):
            raise ValueError(f"Egg at u={uv[0]*100:+.1f} cm, v={uv[1]*100:.1f} cm is outside the trained region")
        clear = C.robot_clearance(uv, d_meas, self.bundle.table)
        if clear <= self.spec.autostart.clearance_m:
            raise ValueError(f"Egg is {clear*1000:.0f} mm from the robot in its current shape (need > "
                             f"{self.spec.autostart.clearance_m*1000:.0f} mm)")

    def _send_quiet(self, action, **kw):
        try:
            return self.send(action, **kw)
        except Exception:
            return None

    def finish(self, outcome: str, message: str, hold: bool = True):
        if hold and not self.dry_run and self.state in ("ack", "moving"):
            self._send_quiet("hold")
        self.outcome = outcome
        if self.log:
            self.log.close(outcome=outcome, terminal_message=message, decisions=self.decision_index,
                           ended_monotonic=self.clock(), duration_s=self.clock() - self.log.meta["started_monotonic"],
                           feedback_wait_count=self.feedback_wait_count)
            self.episodes.append(dict(episode=self.log.episode_id, outcome=outcome, decisions=self.decision_index,
                                      start_uv=self.egg_start.tolist(), auto_started=self.log.meta["auto_started"]))
        resumable = outcome in ("SUCCESS", "FAILURE_TIMEOUT", "UNKNOWN")
        if self.auto_continue and resumable:
            self.still.reset()
            self._update("await_egg", f"{outcome}: {message}. Holding; place a new egg and take your hand away.")
        else:
            self._update("done", f"{outcome}: {message}")

    def abort(self, reason: str):
        if self.active:
            self.finish("ABORTED", reason)
        elif self.waiting:
            self._update("done", "Auto-continue stopped: " + reason)

    def operator_label(self, knocked_off: bool, note: str = ""):
        if self.outcome != "UNKNOWN" or not self.log:
            raise ValueError("Only an UNKNOWN episode can be labelled manually")
        self.log.close(operator_label="KNOCKED_OFF" if knocked_off else "NOT_KNOCKED_OFF", operator_label_note=note,
                       operator_label_wall=time.time())

    # ------------------------------------------------------------------ motor-owner messages
    def event(self, event: dict):
        if not (self.active or self.waiting):
            return
        if self.active and self.log:
            self.log.event(dict(received_monotonic=self.clock(), runner_state=self.state, event=event))
        if event.get("type") in ("closing", "closed", "startup_failed", "transport_closed"):
            if self.active:
                self.finish("FAULT_CONNECTION", "Connection ended: " + str(event.get("message", event["type"])), hold=False)
            else:
                self._update("done", "Connection ended while waiting")
            return
        if self.state == "ack" and event.get("id") == self.command_id:
            if event["type"] == "rejected":
                self.finish("FAULT_REJECTED", "Motor owner rejected the command: " + str(event.get("message")))
            elif event["type"] == "accepted":
                self._update("moving", f"Decision {self.decision_index}: moving")

    # ------------------------------------------------------------------ main loop (~10 Hz)
    def tick(self, remote: dict, remote_age: float, item: dict | None, max_age: float = 0.5, max_gap: float = 0.65):
        if not (self.active or self.waiting):
            return
        now = self.clock()
        self.camera_max_age, self.encoder_max_age = max_age, max_gap
        # Control faults and existing move/ACK deadlines never wait for a camera.
        if self.active and not self.camera_only:
            try:
                self._require_motor_ready(remote, allow_moving=self.state in ("ack", "moving"))
            except ValueError as exc:
                if self.active:
                    self.finish("FAULT_MOTOR", str(exc))
                else:
                    self._update("done", "Auto-continue stopped: " + str(exc))
                return
        if self.state == "ack" and now - self.sent_t > ACK_TIMEOUT_S:
            self.finish("FAULT_ACK_TIMEOUT", "Command acknowledgement timed out; never retried")
            return
        if self.state == "moving" and now > self.move_deadline:
            self.finish("FAULT_MOVE_TIMEOUT", "Target not reached and settled in time")
            return
        if not self._feedback_ready(item, now, remote, remote_age):
            return
        key = (item["packet"].get("session"), item["packet"].get("frame_index", item["packet"].get("t_s")))
        if not self.recent or key != (self.recent[-1]["packet"].get("session"),
                self.recent[-1]["packet"].get("frame_index", self.recent[-1]["packet"].get("t_s"))):
            self.recent.append(item)
        if self.waiting:
            self._tick_waiting(remote, item, now)
            return
        uv, det = self._egg_uv(item)
        if uv is not None:
            self.last_seen, self.visible = uv, 1.0
        else:
            self.visible = 0.0
        if key != self.last_perception_key:
            self.last_perception_key = key
            if self._check_success(uv, item, now):
                return
        if self.state == "moving":
            from campaign import held_packet
            if remote.get("phase") == "HOLDING" and not remote.get("pending") and held_packet(
                    encoder_packet(item, max_gap), self.target_mm, SETTLE_WINDOW_S, max_gap):
                self._update("observe", f"Decision {self.decision_index}: settled")
            return
        if self.state != "observe" or now - self.last_command_t < self.spec.action.decision_period_s:
            return
        if self.decision_index >= self.spec.episode_decisions:
            near = self.visible and np.linalg.norm(self.last_seen - self.egg_start) < self.spec.success.hw_off_radius_m
            self.finish("FAILURE_TIMEOUT" if near else "UNKNOWN",
                        f"No confirmed knockdown after {self.decision_index} decisions"
                        + ("" if near else "; egg not seen on its pedestal — label from video"))
            return
        self._decide(item, det, now, remote, remote_age)

    def _camera_fault(self, item, remote_age, message):
        if self.active:
            self.finish("FAULT_STALE", message)
            if self.log:
                self.log.close(freshness_at_fault=dict(
                    camera_frame_available=item is not None,
                    camera_age_s=None if item is None else item.get("displayed_age_s"),
                    camera_decision_limit_s=self.camera_max_age,
                    camera_limit_s=SENSOR_DELIVERY_TIMEOUT_S,
                    motor_status_age_s=remote_age, motor_status_limit_s=1.5,
                    camera_only=self.camera_only,
                    pipeline_timing=None if item is None else item.get("rl_timing")))
        else:
            self._update("done", "Auto-continue stopped: " + message)

    def _feedback_ready(self, item, now, remote=None, remote_age=0.0):
        """Gate observations strictly; allow only a short, bounded delivery wait.

        No stale observation enters the actor, success detector or settle check.
        Absolute sample ages continue ageing in LiveFeed. Repeated replies cannot
        renew a sample. A second deadline prevents endless moderately-old replies.
        The native owner still supervises the already-dispatched finite target.
        """
        def finite_age(value):
            return isinstance(value, (int, float)) and not isinstance(value, bool) and math.isfinite(value) and value >= 0
        cam_age = None if item is None else item.get("displayed_age_s")
        if (item is None or not finite_age(cam_age) or
                (not self.camera_only and (not finite_age(remote_age) or remote_age > 1.5))):
            self._camera_fault(item, remote_age, "Camera unavailable/invalid or motor-status connection stale")
            return False
        if cam_age > SENSOR_DELIVERY_TIMEOUT_S:
            self._camera_fault(item, remote_age,
                f"Camera age {cam_age:.3f} s exceeds delivery stop limit {SENSOR_DELIVERY_TIMEOUT_S:.3f} s")
            return False
        encoder_delayed = False
        encoder_message = ""
        if not self.camera_only:
            try:
                # Structural validity, active state and the absolute delivery bound.
                encoder_packet(item, SENSOR_DELIVERY_TIMEOUT_S)
            except ValueError as exc:
                if self.active:
                    self._encoder_fault(item, str(exc), remote, remote_age)
                else:
                    self.message = "Waiting for referenced encoder feedback: " + str(exc)
                return False
            try:
                encoder_packet(item, self.encoder_max_age)
            except ValueError as exc:
                encoder_delayed, encoder_message = True, str(exc)
        camera_delayed = cam_age > self.camera_max_age
        if (self.feedback_wait_started is not None and
                now - self.feedback_wait_started >= FEEDBACK_RECOVERY_TIMEOUT_S):
            message = "Fresh observations did not recover within 1.000 s"
            if self.feedback_wait_reason == "encoders" and self.active:
                self._encoder_fault(item, message, remote, remote_age)
            else:
                self._camera_fault(item, remote_age, message)
            return False
        if camera_delayed or encoder_delayed:
            if self.feedback_wait_started is None:
                self.feedback_wait_started = now
                self.feedback_wait_reason = "encoders" if encoder_delayed else "camera"
                self.feedback_wait_count += 1
                self.off_since = self.empty_since = None
                if self.log and self.active:
                    self.log.event(dict(received_monotonic=now, runner_state=self.state,
                        event=dict(type="feedback_wait", camera_age_s=cam_age,
                            encoder_age_s=item["packet"].get("encoder_age_s"),
                            command_id=self.command_id)))
            self.message = "Waiting for fresh " + ("camera and encoders" if camera_delayed and encoder_delayed
                else "camera" if camera_delayed else "encoders") + "; no new target sent"
            return False
        if self.feedback_wait_started is not None:
            if self.log and self.active:
                self.log.event(dict(received_monotonic=now, runner_state=self.state,
                    event=dict(type="feedback_recovered", wait_s=now-self.feedback_wait_started,
                        camera_age_s=cam_age, encoder_age_s=item["packet"].get("encoder_age_s"))))
            self.feedback_wait_started = None
            self.message = "Fresh observations restored; " + self.state
        return True

    def _encoder_fault(self, item, message, remote=None, remote_age=None):
        packet = item['packet']
        if self.log:
            self.log.close(freshness_at_fault=dict(
                runner_state=self.state, command_id=self.command_id,
                camera_age_s=item.get('displayed_age_s'), motor_status_age_s=remote_age,
                encoder_age_s=packet.get('encoder_age_s'), motor_age_s=packet.get('motor_age_s'),
                encoder_decision_limit_s=self.encoder_max_age, encoder_delivery_limit_s=SENSOR_DELIVERY_TIMEOUT_S,
                encoder_t_s=packet.get('encoder_t_s'), motor_active=packet.get('motor_active'),
                motor_states=packet.get('motor_states'), feedback_source=packet.get('feedback_source'),
                feedback_error=item.get('rl_feedback_error'), pipeline_timing=item.get('rl_timing'),
                remote_phase=None if remote is None else remote.get('phase')))
        self.finish('FAULT_ENCODER_STALE', message)

    def _tick_waiting(self, remote, item, now):
        still_s = self.still.update(item["rgb"], now)
        a = self.spec.autostart
        uvs = [self._egg_uv(it)[0] for it in list(self.recent)[-START_FRAMES:]]
        if any(u is None for u in uvs) or len(uvs) < START_FRAMES:
            self.message = "Waiting: no egg detected"
            return
        uvs = np.array(uvs)
        if np.max(np.linalg.norm(uvs - uvs.mean(0), axis=1)) > a.egg_stable_m or still_s < a.still_s:
            self.message = f"Waiting: scene still for {still_s:.1f}/{a.still_s:.0f} s"
            return
        if self.outcome != "SUCCESS" and np.linalg.norm(uvs.mean(0) - self.last_start) < RETRY_MOVE_M:
            self.message = "Waiting: move the egg to a new place (or press Run episode to retry here)"
            return
        try:
            self.start_episode(remote, list(self.recent), placement_id="auto")
            self.message = "Auto-started: " + self.message
        except ValueError as exc:
            self.message = "Waiting: " + str(exc)

    def _require_motor_ready(self, remote, allow_moving=False):
        if self.dry_run and self.camera_only:
            return
        axes = remote.get("axes", [])
        if len(axes) != 2 or any(not a.get("enabled") or a.get("rest_count") is None or a.get("state") in ("FAULT", "DISABLED")
                                 for a in axes):
            raise ValueError("Both motors must be enabled, referenced and healthy")
        if not allow_moving and (remote.get("phase") != "HOLDING" or remote.get("pending")):
            raise ValueError("Motors are not holding")
        if list(remote.get("rest_lengths_mm", self.rests_mm)) != list(self.rests_mm):
            raise ValueError("Rest references changed")

    def _check_success(self, uv, item, now) -> bool:
        s = self.spec.success
        if uv is not None and np.linalg.norm(uv - self.egg_start) > s.hw_off_radius_m:
            self.off_since = self.off_since or now
            if now - self.off_since >= s.hw_confirm_s:
                return self._success(item, f"egg seen {np.linalg.norm(uv - self.egg_start)*1000:.0f} mm from its start")
        else:
            self.off_since = None
        egg_near = uv is not None and np.linalg.norm(uv - self.egg_start) <= s.hw_off_radius_m
        cup = (item["rl_cup"] if item.get("rl_detected") else
               self.ped_det.detect(item["rgb"]) if self.ped_det is not None else None)
        cup_here = cup is not None and np.linalg.norm(self.camera.px_to_c(cup.px, feature_z_m=s.sim_support_top_z_m) - self.egg_start) <= s.hw_cup_match_m
        if cup_here and not egg_near:
            self.empty_since = self.empty_since or now
            if now - self.empty_since >= s.hw_empty_cup_s:
                return self._success(item, "empty white cup visible at the egg's start position")
        else:
            self.empty_since = None
        return False

    def _success(self, item, why) -> bool:
        self.log.frame("success.jpg", item["rgb"])
        self.finish("SUCCESS", why)
        return True

    def _decide(self, item, det, now, remote=None, remote_age=0.0):
        a = self.spec.action
        d_meas = self._diff_m(item)
        obs = C.build_actor_obs(self.last_seen, np.array([self.visible]), self.egg_start, np.array([d_meas]),
                                np.array([self.prev_action]))
        try:
            raw = self.bundle.policy(obs)
        except ValueError as exc:
            self.finish("FAULT_NUMERIC", str(exc))
            return
        clipped = float(C.clip_raw(raw, a)[0])
        desired = float(C.desired_diff(np.array([clipped]), a)[0])
        base = d_meas if self.dry_run else self.prev_limited
        limited = float(C.limited_diff(np.array([desired]), np.array([base]), a)[0])
        moved = abs(limited - base) >= a.deadband_diff_m
        payouts = C.payouts_from_diff(limited)
        lengths_mm = self.cable.model_to_physical_mm(payouts)
        speed = max(LAB_MIN_SPEED_MM_S, 1000 * float(C.cable_speed(np.array(limited - d_meas), self.vmax_m_s, a)))
        row = dict(decision=self.decision_index, t=now, frame_t_s=item["packet"].get("t_s"),
                   frame_age_s=item.get("displayed_age_s"), motor_age_s=item["packet"].get("motor_age_s"),
                   encoder_t_s=item["packet"].get("encoder_t_s"),
                   encoder_age_s=item["packet"].get("encoder_age_s"), pipeline_timing=item.get("rl_timing"),
                   encoder_mm=None if self.camera_only else self._lengths_mm(item), payout_assumed=self.camera_only,
                   diff_measured_m=d_meas, egg_visible=self.visible, egg_uv_m=self.last_seen,
                   egg_px=None if det is None else det.px, egg_confidence=None if det is None else det.confidence,
                   obs=obs, raw=raw, clipped=clipped, desired_diff_m=desired, limited_diff_m=limited, moved=moved,
                   command_mm=lengths_mm, speed_mm_s=speed, vmax_mm_s=1000 * self.vmax_m_s, dry_run=self.dry_run,
                   since_last_command_s=now - self.last_command_t if self.last_command_t > 0 else None)
        if moved and not self.camera_only:
            try:
                from actuation import independent_targets
                independent_targets(list(lengths_mm), [speed, speed], self.rests_mm, self.limits)
            except ValueError as exc:
                row["rejected_locally"] = str(exc)
                self.log.decision(row)
                self.finish("FAULT_LIMIT", "Policy target failed Lab limit validation: " + str(exc))
                return
        self.log.frame(f"d{self.decision_index:02d}.jpg", item["rgb"])
        if item.get("rl_detected"):
            commit_time = self.clock()
            item = age_item(item, commit_time)
            if not self._feedback_ready(item, commit_time, remote, remote_age + max(0.0, commit_time-now)):
                return
        self.prev_action = clipped
        self.prev_limited = limited if moved else base
        self.decision_index += 1
        self.last_command_t = now
        if self.dry_run or not moved:
            row["command_id"] = None
            self.log.decision(row)
            self._update("observe", f"Decision {self.decision_index}: " + ("computed (dry run, not sent)" if self.dry_run
                                                                            else "no move (inside 0.5 mm deadband)"))
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
        self._update("ack", f"Decision {self.decision_index}: D -> {1000*limited:+.1f} mm at {speed:.2f} mm/s")


def straighten_command(cable: CableMap, speed_mm_s: float = 2.0) -> dict:
    """Optional, operator-confirmed move to D = 0 (never automatic; not required by contract v3)."""
    lengths = cable.model_to_physical_mm(C.payouts_from_diff(0.0))
    return dict(action="independent", lengths_mm=[float(x) for x in lengths], speeds_mm_s=[speed_mm_s, speed_mm_s])

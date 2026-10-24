"""RL egg-knockdown window: Load policy, Dry run, Arm, Run episode, Stop/Hold, Return to start.

Single command owner: while an episode is active, manual motion and campaigns
are refused by the host (interface.py) and Stop / Hold aborts the episode.
The window never enables, references or disconnects the motors.
"""

from __future__ import annotations

import math
import time
from collections import deque
from pathlib import Path

import cv2
import numpy as np
from PySide6.QtCore import Qt
from PySide6.QtGui import QImage, QPixmap
from PySide6.QtWidgets import (QCheckBox, QComboBox, QDoubleSpinBox, QFileDialog, QGridLayout, QGroupBox, QHBoxLayout, QLabel,
                               QLineEdit, QMessageBox, QPushButton, QTableWidget, QTableWidgetItem, QVBoxLayout, QWidget)

from rl import contract as C
from rl.egg_vision import EggColour, EggDetector, load_settings, save_settings
from rl.hardware_io import CableMap, CameraMap
from rl.placements import PLACEMENTS, TARGET_TOLERANCE_M
from rl.runner import KnockdownRunner, PolicyBundle, return_to_start_command

LAB = Path(__file__).resolve().parents[1]
SETTINGS = LAB / "rl_vision.json"          # local, like lab.local.json (git-ignored)


def wilson(k, n, z=1.96):
    if n == 0:
        return (float("nan"), float("nan"))
    p = k / n; d = 1 + z * z / n; c = (p + z * z / (2 * n)) / d
    h = z * math.sqrt(p * (1 - p) / n + z * z / (4 * n * n)) / d
    return (c - h, c + h)


class ClickImage(QLabel):
    def __init__(self, on_click):
        super().__init__()
        self.on_click, self.scale = on_click, 1.0
        self.setMinimumSize(480, 300)
        self.setAlignment(Qt.AlignCenter)

    def mousePressEvent(self, e):
        if self.pixmap() is None:
            return
        pm = self.pixmap()
        ox = (self.width() - pm.width()) / 2
        oy = (self.height() - pm.height()) / 2
        x, y = (e.position().x() - ox) / self.scale, (e.position().y() - oy) / self.scale
        self.on_click((x, y))


class RLWindow(QWidget):
    def __init__(self, host):
        super().__init__()
        self.host = host
        self.setWindowTitle("RL — egg knockdown (camera + encoders)")
        self.resize(980, 760)
        self.bundle = None
        self.runner: KnockdownRunner | None = None
        self.settings = load_settings(SETTINGS)
        self.items = deque(maxlen=12)
        self.last_t = None
        self.armed = False
        self.pick_colour = False
        self.history = []
        self.on_target = None
        lay = QVBoxLayout(self)

        top = QHBoxLayout(); lay.addLayout(top)
        left = QVBoxLayout(); top.addLayout(left, 2)
        self.image = ClickImage(self.clicked); left.addWidget(self.image, 1)
        self.vision_status = QLabel("Camera: waiting"); left.addWidget(self.vision_status)

        right = QVBoxLayout(); top.addLayout(right, 1)
        g = QGroupBox("1. Policy"); gl = QVBoxLayout(g); right.addWidget(g)
        b = QPushButton("Load exported policy folder…"); b.clicked.connect(self.load_policy); gl.addWidget(b)
        self.policy_status = QLabel("No policy loaded"); self.policy_status.setWordWrap(True); gl.addWidget(self.policy_status)

        g = QGroupBox("2. Egg camera"); gl = QGridLayout(g); right.addWidget(g)
        b = QPushButton("Sample egg colour (then click the egg)"); b.clicked.connect(self.start_pick); gl.addWidget(b, 0, 0, 1, 2)
        gl.addWidget(QLabel("Camera height above table [m] (0 = off)"), 1, 0)
        self.cam_h = QDoubleSpinBox(); self.cam_h.setRange(0, 5); self.cam_h.setDecimals(3)
        self.cam_h.setValue(float(self.settings.get("camera_height_m") or 0)); gl.addWidget(self.cam_h, 1, 1)
        b = QPushButton("Save camera settings"); b.clicked.connect(self.save_camera); gl.addWidget(b, 2, 0, 1, 2)

        g = QGroupBox("3. Episode"); gl = QGridLayout(g); right.addWidget(g)
        self.dry = QCheckBox("Dry run (compute + log, no motor writes)"); self.dry.setChecked(True)
        self.dry.toggled.connect(self.dry_changed); gl.addWidget(self.dry, 0, 0, 1, 2)
        self.arm_button = QPushButton("Arm motor commands…"); self.arm_button.clicked.connect(self.arm); gl.addWidget(self.arm_button, 1, 0, 1, 2)
        gl.addWidget(QLabel("Placement"), 2, 0); self.placement = QComboBox(); self.placement.setEditable(True)
        self.placement.addItems(list(PLACEMENTS)); gl.addWidget(self.placement, 2, 1)
        gl.addWidget(QLabel("Note"), 3, 0); self.note = QLineEdit(); gl.addWidget(self.note, 3, 1)
        self.run_button = QPushButton("Run episode"); self.run_button.clicked.connect(self.run_episode); gl.addWidget(self.run_button, 4, 0)
        self.stop_button = QPushButton("STOP / HOLD"); self.stop_button.setStyleSheet("font-weight:bold;color:#b00020")
        self.stop_button.clicked.connect(lambda: self.host.command("hold")); gl.addWidget(self.stop_button, 4, 1)
        self.return_button = QPushButton("Return to start pose (2 mm/s)…"); self.return_button.clicked.connect(self.return_start)
        gl.addWidget(self.return_button, 5, 0, 1, 2)
        self.yes = QPushButton("UNKNOWN → knocked off"); self.yes.clicked.connect(lambda: self.label(True)); gl.addWidget(self.yes, 6, 0)
        self.no = QPushButton("UNKNOWN → still on"); self.no.clicked.connect(lambda: self.label(False)); gl.addWidget(self.no, 6, 1)
        self.status = QLabel("Load a policy. Dry run is on."); self.status.setWordWrap(True); right.addWidget(self.status)
        self.summary = QLabel(""); right.addWidget(self.summary); right.addStretch()

        self.table = QTableWidget(0, 6); lay.addWidget(self.table)
        self.table.setHorizontalHeaderLabels(["Episode", "Placement", "Mode", "Outcome", "Decisions", "Label"])

    # ---------------------------------------------------------------- state
    @property
    def active(self) -> bool:
        return bool(self.runner and self.runner.active)

    def dry_changed(self, on):
        if on:
            self.armed = False
        self.arm_button.setEnabled(not on)

    def arm(self):
        if self.dry.isChecked():
            return
        if QMessageBox.question(self, "Arm RL motor commands",
                                "The policy will command both cables at "
                                f"{1000*C.DEFAULT.action.move_speed_m_s:g} mm/s within the session limits.\n"
                                "Keep your hand near STOP / HOLD. Continue?") == QMessageBox.Yes:
            self.armed = True
            self.status.setText("ARMED: Run episode will move the robot.")

    def load_policy(self):
        folder = QFileDialog.getExistingDirectory(self, "Exported knockdown policy folder", str(LAB))
        if not folder:
            return
        try:
            self.bundle = PolicyBundle(folder)
            self.policy_status.setText("Loaded " + self.bundle.describe() +
                                       f"\nLab-side parity {getattr(self.bundle, 'lab_parity_error', float('nan')):.1e}")
        except Exception as exc:
            self.bundle = None
            self.policy_status.setText(str(exc))

    def start_pick(self):
        self.pick_colour = True
        self.vision_status.setText("Click the centre of the egg in the image")

    def clicked(self, px):
        if not self.pick_colour or not self.items:
            return
        try:
            colour = EggColour.sample(self.items[-1]["rgb"], px)
            save_settings(SETTINGS, colour)
            self.settings = load_settings(SETTINGS)
            self.vision_status.setText(f"Egg colour saved: {colour}")
        except Exception as exc:
            self.vision_status.setText(str(exc))
        self.pick_colour = False

    def save_camera(self):
        rgb = self.items[-1]["rgb"] if self.items else None
        extra = dict(camera_height_m=self.cam_h.value() or None)
        if rgb is not None:
            h, w = rgb.shape[:2]
            extra["principal_point_px"] = [w / 2, h / 2]
        save_settings(SETTINGS, EggColour(**self.settings.get("colour", {})), extra)
        self.settings = load_settings(SETTINGS)
        self.vision_status.setText("Camera settings saved (principal point = image centre)")

    def maps(self):
        remote = self.host.remote
        rests = remote.get("rest_lengths_mm") or self.host.motion_profile["rest_lengths_mm"]
        limits = remote.get("motion_limits") or self.host.motion_profile
        params = self.host.fit["candidate"]["parameters"]
        cable = CableMap.from_lab(self.host.comparison_setup, params, rests, C.NEUTRAL_PAYOUT_M)
        camera = CameraMap.from_lab(self.host.comparison_setup, self.settings,
                                    self.host.config.get("model_check", {}).get("plane_height_m", 0.09))
        detector = EggDetector(EggColour(**self.settings.get("colour", {})), camera.pixels_per_m)
        return cable, camera, detector, rests, limits

    def run_episode(self):
        if self.active:
            return
        try:
            if self.bundle is None:
                raise ValueError("Load a verified policy first")
            if self.host.campaign_active():
                raise ValueError("A campaign owns the motors. Stop it first.")
            if not self.dry.isChecked() and not self.armed:
                raise ValueError("Arm motor commands first (or enable Dry run)")
            if not self.dry.isChecked() and not self.host.session:
                raise ValueError("Connect the motor session first")
            if "colour" not in self.settings:
                raise ValueError("Sample the egg colour first")
            cable, camera, detector, rests, limits = self.maps()
            log_root = (Path(self.host.session.log_dir) if self.host.session else LAB / "logs" / "rl_dry") / "rl_episodes"
            self.runner = KnockdownRunner(self.bundle, cable, camera, detector, rests, limits,
                                          self.host.session.send if self.host.session else self._no_send,
                                          log_root, dry_run=self.dry.isChecked())
            pid = self.placement.currentText().strip()
            if pid in PLACEMENTS and self.on_target is False:
                raise ValueError(f"Egg is not within {TARGET_TOLERANCE_M*1000:.0f} mm of {pid}; move it onto the green cross")
            self.runner.start_episode(self.host.remote, list(self.items), pid, self.note.text())
            self.status.setText(self.runner.message)
        except Exception as exc:
            self.status.setText("Episode not started: " + str(exc))

    @staticmethod
    def _no_send(*_a, **_k):
        raise RuntimeError("No motor session (dry run only)")

    def return_start(self):
        if self.active or self.dry.isChecked() or not self.host.session:
            self.status.setText("Return to start needs a connected session, no running episode and Dry run off")
            return
        if QMessageBox.question(self, "Return to start", "Move both cables to the trained start pose at 2 mm/s?\n"
                                "Make sure the egg/pedestal will not be hit.") != QMessageBox.Yes:
            return
        cable, *_ = self.maps()
        cmd = return_to_start_command(cable)
        try:
            self.host.session.send(cmd.pop("action"), **cmd)
            self.status.setText("Return-to-start move sent")
        except Exception as exc:
            self.status.setText(str(exc))

    def label(self, knocked):
        if not self.runner:
            return
        try:
            self.runner.operator_label(knocked, self.note.text())
            self.history[-1]["label"] = "KNOCKED_OFF" if knocked else "NOT_KNOCKED_OFF"
            self.refresh_table()
        except ValueError as exc:
            self.status.setText(str(exc))

    # ---------------------------------------------------------------- host hooks
    def abort(self, reason):
        if self.runner:
            self.runner.abort(reason)
            self._finished()

    def motor_event(self, event):
        if self.runner:
            self.runner.event(event)

    def tick(self):
        item = self.host.manual_source()
        if item is not None and "rgb" in item:
            t = item["packet"].get("t_s")
            if t != self.last_t:
                self.items.append(item); self.last_t = t
                self.draw(item)
        was = self.active
        if self.runner and self.runner.active:
            self.runner.tick(self.host.remote, time.monotonic() - self.host.remote_received, item,
                             max_age=min(0.5, self.host.config["live"]["max_age_s"]),
                             max_gap=self.host.config["live"]["max_input_gap_s"])
            self.status.setText(self.runner.message)
        if was and not self.active:
            self._finished()

    def _finished(self):
        r = self.runner
        if r and r.log and not any(h["episode"] == r.log.episode_id for h in self.history):
            self.history.append(dict(episode=r.log.episode_id, placement=r.log.meta["placement_id"],
                                     mode="dry" if r.dry_run else "armed", outcome=r.outcome,
                                     decisions=r.decision_index, label=""))
            self.refresh_table()

    def refresh_table(self):
        self.table.setRowCount(len(self.history))
        for i, h in enumerate(self.history):
            for j, k in enumerate(("episode", "placement", "mode", "outcome", "decisions", "label")):
                self.table.setItem(i, j, QTableWidgetItem(str(h[k])))
        armed = [h for h in self.history if h["mode"] == "armed"]
        k = sum(h["outcome"] == "SUCCESS" for h in armed)
        lo, hi = wilson(k, len(armed))
        unknown = sum(h["outcome"] == "UNKNOWN" for h in armed)
        faults = sum(str(h["outcome"]).startswith("FAULT") or h["outcome"] == "ABORTED" for h in armed)
        self.summary.setText(f"Armed episodes: {len(armed)} | automatic successes {k} "
                             f"(95% CI {lo:.2f}–{hi:.2f}) | UNKNOWN {unknown} | faults/aborts {faults} | "
                             f"placements {len({h['placement'] for h in armed})}")

    def draw(self, item):
        rgb = item["rgb"].copy()
        if "colour" in self.settings:
            try:
                _, camera, detector, *_ = self.maps()
                det = detector.detect(rgb)
                if det:
                    cv2.circle(rgb, tuple(int(v) for v in det.px), int(math.sqrt(det.expected_area_px / math.pi)), (0, 255, 0), 3)
                    uv = camera.px_to_c(det.px)
                    text = f"Egg u={uv[0]*100:+.1f} cm v={uv[1]*100:.1f} cm conf {det.confidence:.2f}"
                    target = PLACEMENTS.get(self.placement.currentText().strip())
                    if target is not None and not self.active:
                        err = float(np.linalg.norm(uv - np.asarray(target)))
                        self.on_target = err <= TARGET_TOLERANCE_M
                        text += f" | {self.placement.currentText()} offset {err*1000:.0f} mm" + (" ✓" if self.on_target else "")
                    self.vision_status.setText(text)
                else:
                    self.on_target = None
                    self.vision_status.setText("Egg not detected")
                target = PLACEMENTS.get(self.placement.currentText().strip())
                if target is not None:
                    tp = tuple(int(v) for v in camera.c_to_px(target))
                    col = (0, 200, 0) if self.on_target else (255, 160, 0)
                    cv2.drawMarker(rgb, tp, col, cv2.MARKER_CROSS, 60, 4)
                    cv2.circle(rgb, tp, int(TARGET_TOLERANCE_M * camera.pixels_per_m), col, 2)
            except Exception as exc:
                self.vision_status.setText("Vision: " + str(exc))
        h, w = rgb.shape[:2]
        scale = min(self.image.width() / w, self.image.height() / h)
        self.image.scale = scale
        qimg = QImage(rgb.data, w, h, 3 * w, QImage.Format_RGB888)
        self.image.setPixmap(QPixmap.fromImage(qimg).scaled(int(w * scale), int(h * scale), Qt.KeepAspectRatio))

    def closeEvent(self, e):
        if self.active:
            self.abort("RL window closed")
        super().closeEvent(e)

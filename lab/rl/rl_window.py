"""RL egg-knockdown window (contract v2).

Load policy · sample egg/pedestal colours · Dry run / Arm · cable speed 5|10|15 mm/s ·
Run episode · STOP / HOLD · Auto-continue (human replaces the egg; the robot waits for a still scene).
Single command owner: while an episode runs or auto-continue waits, the host locks manual motion and
campaigns; STOP / HOLD aborts both. The window never enables, references or disconnects motors.
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
from PySide6.QtWidgets import (QCheckBox, QComboBox, QDoubleSpinBox, QFileDialog, QGridLayout, QGroupBox, QHBoxLayout,
                               QLabel, QLineEdit, QMessageBox, QPushButton, QTableWidget, QTableWidgetItem, QVBoxLayout,
                               QWidget)

from rl import contract as C
from rl.egg_vision import Colour, EggDetector, PedestalDetector, load_settings, save_settings
from rl.hardware_io import CableMap, CameraMap
from rl.placements import PLACEMENTS, TARGET_TOLERANCE_M
from rl.runner import KnockdownRunner, PolicyBundle, straighten_command

LAB = Path(__file__).resolve().parents[1]
SETTINGS = LAB / "rl_vision.json"          # local (git-ignored)


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
        pm = self.pixmap()
        if pm is None:
            return
        ox, oy = (self.width() - pm.width()) / 2, (self.height() - pm.height()) / 2
        self.on_click(((e.position().x() - ox) / self.scale, (e.position().y() - oy) / self.scale))


class RLWindow(QWidget):
    def __init__(self, host):
        super().__init__()
        self.host = host
        self.setWindowTitle("RL — egg knockdown (camera + encoders, difference action)")
        self.resize(1000, 780)
        self.bundle = None
        self.runner: KnockdownRunner | None = None
        self.settings = load_settings(SETTINGS)
        self.items = deque(maxlen=12)
        self.last_t = None
        self.armed = False
        self.pick = None
        self.on_target = None
        lay = QVBoxLayout(self)
        top = QHBoxLayout(); lay.addLayout(top)
        left = QVBoxLayout(); top.addLayout(left, 2)
        self.image = ClickImage(self.clicked); left.addWidget(self.image, 1)
        self.vision_status = QLabel("Camera: waiting"); self.vision_status.setWordWrap(True); left.addWidget(self.vision_status)
        right = QVBoxLayout(); top.addLayout(right, 1)

        g = QGroupBox("1. Policy"); gl = QVBoxLayout(g); right.addWidget(g)
        b = QPushButton("Load exported policy folder…"); b.clicked.connect(self.load_policy); gl.addWidget(b)
        self.policy_status = QLabel("No policy loaded"); self.policy_status.setWordWrap(True); gl.addWidget(self.policy_status)

        g = QGroupBox("2. Camera"); gl = QGridLayout(g); right.addWidget(g)
        b = QPushButton("Sample EGG colour (click the egg)"); b.clicked.connect(lambda: self.start_pick("colour")); gl.addWidget(b, 0, 0, 1, 2)
        b = QPushButton("Sample PEDESTAL colour (click the empty cup)"); b.clicked.connect(lambda: self.start_pick("pedestal_colour"))
        gl.addWidget(b, 1, 0, 1, 2)
        gl.addWidget(QLabel("Camera height above table [m] (0 = off)"), 2, 0)
        self.cam_h = QDoubleSpinBox(); self.cam_h.setRange(0, 5); self.cam_h.setDecimals(3)
        self.cam_h.setValue(float(self.settings.get("camera_height_m") or 0.321)); gl.addWidget(self.cam_h, 2, 1)
        b = QPushButton("Save camera settings"); b.clicked.connect(self.save_camera); gl.addWidget(b, 3, 0, 1, 2)

        g = QGroupBox("3. Episodes"); gl = QGridLayout(g); right.addWidget(g)
        self.dry = QCheckBox("Dry run (compute + log, no motor writes)"); self.dry.setChecked(True)
        self.dry.toggled.connect(self.dry_changed); gl.addWidget(self.dry, 0, 0, 1, 2)
        self.arm_button = QPushButton("Arm motor commands…"); self.arm_button.setEnabled(False)
        self.arm_button.clicked.connect(self.arm); gl.addWidget(self.arm_button, 1, 0, 1, 2)
        gl.addWidget(QLabel("Cable speed ceiling"), 2, 0)
        self.speed = QComboBox(); self.speed.addItems(["5 mm/s", "10 mm/s", "15 mm/s"]); gl.addWidget(self.speed, 2, 1)
        self.auto = QCheckBox("Auto-continue: wait for a new egg + still scene, then start"); gl.addWidget(self.auto, 3, 0, 1, 2)
        gl.addWidget(QLabel("Placement"), 4, 0)
        self.placement = QComboBox(); self.placement.setEditable(True)
        self.placement.addItems(["free", *PLACEMENTS]); gl.addWidget(self.placement, 4, 1)
        gl.addWidget(QLabel("Note"), 5, 0); self.note = QLineEdit(); gl.addWidget(self.note, 5, 1)
        self.run_button = QPushButton("Run episode"); self.run_button.clicked.connect(self.run_episode); gl.addWidget(self.run_button, 6, 0)
        self.stop_button = QPushButton("STOP / HOLD"); self.stop_button.setStyleSheet("font-weight:bold;color:#b00020")
        self.stop_button.clicked.connect(lambda: self.host.command("hold")); gl.addWidget(self.stop_button, 6, 1)
        self.straight_button = QPushButton("Straighten robot (D = 0 at 2 mm/s)…"); self.straight_button.clicked.connect(self.straighten)
        gl.addWidget(self.straight_button, 7, 0, 1, 2)
        self.yes = QPushButton("UNKNOWN → knocked off"); self.yes.clicked.connect(lambda: self.label(True)); gl.addWidget(self.yes, 8, 0)
        self.no = QPushButton("UNKNOWN → still on"); self.no.clicked.connect(lambda: self.label(False)); gl.addWidget(self.no, 8, 1)
        self.status = QLabel("Load a policy. Dry run is on."); self.status.setWordWrap(True); right.addWidget(self.status)
        self.summary = QLabel(""); self.summary.setWordWrap(True); right.addWidget(self.summary); right.addStretch()
        self.table = QTableWidget(0, 6); lay.addWidget(self.table)
        self.table.setHorizontalHeaderLabels(["Episode", "Placement", "Mode", "Outcome", "Decisions", "Label / start"])
        self.history = []

    # ---------------------------------------------------------------- state
    @property
    def active(self) -> bool:
        """True while the runner owns motion: an episode or auto-continue waiting."""
        return bool(self.runner and (self.runner.active or self.runner.waiting))

    def vmax(self) -> float:
        return float(self.speed.currentText().split()[0])

    def dry_changed(self, on):
        if on:
            self.armed = False
        self.arm_button.setEnabled(not on)

    def arm(self):
        if self.dry.isChecked():
            return
        if QMessageBox.question(self, "Arm RL motor commands",
                                f"The policy will move the cable difference at up to {self.vmax():g} mm/s per cable "
                                "within the session limits.\nKeep your hand near STOP / HOLD. Continue?") == QMessageBox.Yes:
            self.armed = True
            self.status.setText("ARMED: Run episode will move the robot.")

    def load_policy(self):
        folder = QFileDialog.getExistingDirectory(self, "Exported knockdown policy folder", str(LAB))
        if not folder:
            return
        try:
            self.bundle = PolicyBundle(folder)
            self.policy_status.setText("Loaded " + self.bundle.describe() + f"\nLab-side parity {self.bundle.lab_parity_error:.1e}")
        except Exception as exc:
            self.bundle = None
            self.policy_status.setText(str(exc))

    def start_pick(self, key):
        self.pick = key
        self.vision_status.setText("Click the centre of the " + ("egg" if key == "colour" else "empty blue cup"))

    def clicked(self, px):
        if not self.pick or not self.items:
            return
        try:
            save_settings(SETTINGS, Colour.sample(self.items[-1]["rgb"], px), key=self.pick)
            self.settings = load_settings(SETTINGS)
            self.vision_status.setText(f"Saved {self.pick.replace('_', ' ')}")
        except Exception as exc:
            self.vision_status.setText(str(exc))
        self.pick = None

    def save_camera(self):
        extra = dict(camera_height_m=self.cam_h.value() or None)
        if self.items:
            h, w = self.items[-1]["rgb"].shape[:2]
            extra["principal_point_px"] = [w / 2, h / 2]
        save_settings(SETTINGS, None, extra)
        self.settings = load_settings(SETTINGS)
        self.vision_status.setText("Camera settings saved (principal point = image centre)")

    def maps(self):
        remote = self.host.remote
        rests = remote.get("rest_lengths_mm") or self.host.motion_profile["rest_lengths_mm"]
        limits = remote.get("motion_limits") or self.host.motion_profile
        cable = CableMap.from_lab(self.host.comparison_setup, self.host.fit["candidate"]["parameters"], rests,
                                  (C.MEAN_PAYOUT_M, C.MEAN_PAYOUT_M))
        camera = CameraMap.from_lab(self.host.comparison_setup, self.settings,
                                    self.host.config.get("model_check", {}).get("plane_height_m", 0.09))
        egg = EggDetector(Colour(**self.settings.get("colour", {})), camera.pixels_per_m)
        ped = PedestalDetector(Colour(**self.settings["pedestal_colour"]), camera.pixels_per_m) \
            if "pedestal_colour" in self.settings else None
        return cable, camera, egg, ped, rests, limits

    def run_episode(self):
        if self.runner and self.runner.active:
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
            pid = self.placement.currentText().strip() or "free"
            if pid in PLACEMENTS and self.on_target is False:
                raise ValueError(f"Egg is not within {TARGET_TOLERANCE_M*1000:.0f} mm of {pid}; move it onto the cross")
            if self.runner is None or not self.runner.waiting:
                cable, camera, egg, ped, rests, limits = self.maps()
                log_root = (Path(self.host.session.log_dir) if self.host.session else LAB / "logs" / "rl_dry") / "rl_episodes"
                self.runner = KnockdownRunner(self.bundle, cable, camera, egg, ped, rests, limits,
                                              self.host.session.send if self.host.session else self._no_send, log_root,
                                              dry_run=self.dry.isChecked(), vmax_mm_s=self.vmax(),
                                              auto_continue=self.auto.isChecked())
            self.runner.start_episode(self.host.remote, list(self.items), pid, self.note.text())
            self.status.setText(self.runner.message)
        except Exception as exc:
            self.status.setText("Episode not started: " + str(exc))

    @staticmethod
    def _no_send(*_a, **_k):
        raise RuntimeError("No motor session (dry run only)")

    def straighten(self):
        if self.active or self.dry.isChecked() or not self.host.session:
            self.status.setText("Straighten needs a connected session, nothing running, and Dry run off")
            return
        if QMessageBox.question(self, "Straighten", "Move to D = 0 at 2 mm/s?\nMake sure no egg is in the robot's path.") != QMessageBox.Yes:
            return
        cmd = straighten_command(self.maps()[0])
        try:
            self.host.session.send(cmd.pop("action"), **cmd)
            self.status.setText("Straighten move sent")
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
            self._sync_history()

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
        if self.runner and (self.runner.active or self.runner.waiting):
            self.runner.tick(self.host.remote, time.monotonic() - self.host.remote_received, item,
                             max_age=min(0.5, self.host.config["live"]["max_age_s"]),
                             max_gap=self.host.config["live"]["max_input_gap_s"])
            self.status.setText(self.runner.message)
        self._sync_history()

    def _sync_history(self):
        if not self.runner:
            return
        known = {h["episode"] for h in self.history}
        added = False
        for e in self.runner.episodes:
            if e["episode"] not in known:
                self.history.append(dict(episode=e["episode"], placement="auto" if e["auto_started"] else
                                         self.placement.currentText(), mode="dry" if self.runner.dry_run else "armed",
                                         outcome=e["outcome"], decisions=e["decisions"],
                                         label=f"start u={e['start_uv'][0]*100:+.1f} v={e['start_uv'][1]*100:.1f} cm"))
                added = True
        if added:
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
        self.summary.setText(f"Armed episodes: {len(armed)} | automatic successes {k} (95% CI {lo:.2f}–{hi:.2f}) | "
                             f"UNKNOWN {unknown} | faults/aborts {faults}")

    def draw(self, item):
        rgb = item["rgb"].copy()
        if "colour" in self.settings:
            try:
                _, camera, egg, ped, *_ = self.maps()
                det = egg.detect(rgb)
                text = "Egg not detected"
                target = PLACEMENTS.get(self.placement.currentText().strip())
                if det:
                    cv2.circle(rgb, tuple(int(v) for v in det.px), int(math.sqrt(det.expected_area_px / math.pi)), (0, 255, 0), 3)
                    uv = camera.px_to_c(det.px)
                    text = f"Egg u={uv[0]*100:+.1f} cm v={uv[1]*100:.1f} cm conf {det.confidence:.2f}"
                    if target is not None and not self.active:
                        err = float(np.linalg.norm(uv - np.asarray(target)))
                        self.on_target = err <= TARGET_TOLERANCE_M
                        text += f" | {self.placement.currentText()} offset {err*1000:.0f} mm" + (" ✓" if self.on_target else "")
                else:
                    self.on_target = None
                if ped is not None:
                    cd = ped.detect(rgb)
                    if cd:
                        cv2.rectangle(rgb, (int(cd.px[0]) - 25, int(cd.px[1]) - 25), (int(cd.px[0]) + 25, int(cd.px[1]) + 25),
                                      (0, 120, 255), 3)
                        text += " | empty cup visible"
                if target is not None:
                    tp = tuple(int(v) for v in camera.c_to_px(target))
                    cv2.drawMarker(rgb, tp, (0, 200, 0) if self.on_target else (255, 160, 0), cv2.MARKER_CROSS, 60, 4)
                if self.runner and self.runner.waiting:
                    text += " | " + self.runner.message
                self.vision_status.setText(text)
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

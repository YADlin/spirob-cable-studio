"""Simulated driver ("digital twin") for closed-loop tests of the RL hardware path.

Emulates exactly what the RL runner consumes from the real Lab:
  * ``remote``  : the laptop status event (phase, axes, pending, rest_lengths_mm)
  * ``item``    : a camera item with an RGB frame and a packet of encoder rows
  * ``send``    : the command channel (``independent``, ``hold``, ``record``)
Physical cable lengths are converted to model controls with the SAME CableMap
used by the runner, so a wrong mapping/sign makes the twin robot curl the wrong
way. The camera renders a synthetic overhead image through the SAME CameraMap,
so the real EggDetector and pixel->frame-C conversion are exercised.

This is a software test harness, not evidence of physical transfer.
"""

from __future__ import annotations

import itertools
from pathlib import Path

import cv2
import mujoco
import numpy as np

from rl import contract as C
from rl.hardware_io import CableMap, CameraMap

LAB = Path(__file__).resolve().parents[1]
ASSETS = Path(__file__).resolve().parent / "assets"
EGG_SETTLED_Z = 0.0950
PEDESTAL_Z = 0.002


def build_scene(egg_uv, dt=1e-3):
    s = mujoco.MjSpec.from_file(str(LAB / "model" / "spirob_robot.xml"))
    s.option.timestep = dt
    s.option.integrator = mujoco.mjtIntegrator.mjINT_IMPLICITFAST
    x, y = C.c_to_model_xy(*egg_uv)
    for prefix, name, z in (("ped/", "spirob_pedestal.xml", PEDESTAL_Z), ("egg/", "spirob_egg.xml", EGG_SETTLED_Z)):
        child = mujoco.MjSpec.from_file(str(ASSETS / name))
        frame = s.worldbody.add_frame(pos=[x, y, z])
        frame.attach_body(child.worldbody.first_body(), prefix, "")
    m = s.compile()
    return m, mujoco.MjData(m)


class Twin:
    def __init__(self, cable: CableMap, camera: CameraMap, egg_uv, image_size=(1920, 1080), physics_dt=1e-3,
                 fault: str | None = None, frame_noise_px: float = 0.0, seed: int = 0):
        self.cable, self.camera = cable, camera
        self.m, self.d = build_scene(egg_uv, physics_dt)
        self.egg = self.m.body("egg/egg").id
        self.lengths = np.asarray(cable.model_to_physical_mm(C.NEUTRAL_PAYOUT_M), dtype=float)
        self.target = self.lengths.copy()
        self.speed = np.zeros(2)
        self.t = 0.0
        self.rows: list[dict] = []
        self.ids = itertools.count(1)
        self.events: list[dict] = []
        self.pending = False
        self.state = ["HOLDING", "HOLDING"]
        self.fault = fault
        self.size = image_size
        self.rng = np.random.default_rng(seed)
        self.frame_noise_px = frame_noise_px
        self.commands: list[dict] = []
        self._apply_ctrl()
        for _ in range(5):
            self.advance(0.05)

    # -------------------------------------------------- command channel
    def send(self, action, **kw):
        cid = next(self.ids)
        self.commands.append(dict(id=cid, action=action, **kw))
        if action == "hold":
            self.target = self.lengths.copy()
            return cid
        if action == "record":
            return cid
        if action != "independent":
            self.events.append(dict(type="rejected", id=cid, message="twin supports independent/hold/record only"))
            return cid
        if self.fault == "reject":
            self.events.append(dict(type="rejected", id=cid, message="injected rejection"))
            return cid
        self.target = np.asarray(kw["lengths_mm"], dtype=float)
        self.speed = np.asarray(kw["speeds_mm_s"], dtype=float)
        if self.fault != "no_ack":
            self.events.append(dict(type="accepted", id=cid, message="ok"))
        return cid

    # -------------------------------------------------- physics
    def _apply_ctrl(self):
        self.d.ctrl[:] = self.cable.physical_to_model_m(self.lengths)

    def advance(self, seconds: float):
        n = int(round(seconds / self.m.opt.timestep))
        for _ in range(n):
            if self.fault != "stuck":
                step = self.speed * self.m.opt.timestep
                self.lengths += np.clip(self.target - self.lengths, -step, step)
            self._apply_ctrl()
            mujoco.mj_step(self.m, self.d)
            self.t += self.m.opt.timestep
            if int(round(self.t * 1000)) % 50 == 0:  # 20 Hz encoder samples, like the worker poll
                moving = np.abs(self.target - self.lengths) > 1e-3
                self.state = ["MOVING" if mv else "HOLDING" for mv in moving]
                for ax in (0, 1):
                    self.rows.append(dict(axis=ax, t_s=self.t, length_mm=float(self.lengths[ax]),
                                          target_mm=float(self.target[ax]), state=self.state[ax]))
                self.rows = self.rows[-400:]

    # -------------------------------------------------- what the runner sees
    def remote(self) -> dict:
        axes = [dict(enabled=True, rest_count=0, state=self.state[i], absolute_mm=float(self.lengths[i])) for i in (0, 1)]
        phase = "HOLDING" if all(s == "HOLDING" for s in self.state) else "BUSY"
        return dict(type="status", phase=phase, pending=phase != "HOLDING", axes=axes,
                    rest_lengths_mm=list(self.cable.rests_mm))

    def egg_uv_true(self) -> np.ndarray:
        x, y = self.d.xpos[self.egg, :2]
        return np.asarray(C.model_xy_to_c(x, y))

    def egg_z(self) -> float:
        return float(self.d.xpos[self.egg, 2])

    def render(self) -> np.ndarray:
        w, h = self.size
        img = np.full((h, w, 3), (70, 70, 72), np.uint8)
        # robot backbone in blue (distractor for the detector). An egg on the floor
        # lies BELOW the robot plane, so the robot is drawn over it (occlusion).
        names = [f"link_{i:03d}" for i in range(1, 22)]
        pts = [self.camera.c_to_px(C.model_xy_to_c(*self.d.body(n).xpos[:2])) for n in names]
        egg_below_robot = self.egg_z() < 0.06
        if not egg_below_robot:
            cv2.polylines(img, [np.asarray(pts, np.int32)], False, (40, 80, 200), 25)
        if self.fault != "hide_egg":
            uv = self.egg_uv_true()
            # feature height: perspective is the inverse of CameraMap.px_to_c's correction
            px = self.camera.c_to_px(uv) if self.camera.camera_height_m is None else self._project(uv)
            px = px + self.rng.normal(0, self.frame_noise_px, 2)
            r = int(round(0.020 * self.camera.pixels_per_m))
            cv2.circle(img, (int(px[0]), int(px[1])), r, (238, 232, 215), -1)
        if egg_below_robot:
            cv2.polylines(img, [np.asarray(pts, np.int32)], False, (40, 80, 200), 25)
        return img

    def egg_in_frame(self) -> bool:
        px = self.camera.c_to_px(self.egg_uv_true())
        r = 0.020 * self.camera.pixels_per_m
        return r <= px[0] <= self.size[0] - r and r <= px[1] <= self.size[1] - r

    def _project(self, uv):
        cam = self.camera
        H, z = cam.camera_height_m, cam.feature_z_m
        uv_pp = cam.px_to_c_plane(cam.principal_point_px)
        apparent = uv_pp + (uv - uv_pp) * (H - cam.plane_z_m) / (H - z)
        return cam.c_to_px(apparent)

    def item(self, age_s: float = 0.05) -> dict:
        motor = [r for r in self.rows if r["t_s"] <= self.t + 1e-9]
        # bracket the frame time with the next encoder sample like the real packet builder
        frame_t = motor[-1]["t_s"] if motor else self.t
        packet = dict(t_s=frame_t, motor=motor, motor_active=True, state="STREAMING", motor_age_s=[0.02, 0.02],
                      frame_age_s=age_s, rest_lengths_mm=list(self.cable.rests_mm), session="twin")
        return dict(rgb=self.render(), packet=packet, displayed_age_s=age_s)

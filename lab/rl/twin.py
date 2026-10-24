"""Simulated driver ("digital twin") for closed-loop tests of the RL hardware path (contract v2).

Emulates exactly what the runner consumes from the real Lab: the laptop status event (`remote`),
camera items (RGB frame + encoder rows) and the command channel (`send`). Physical cable lengths
become model controls through the SAME CableMap as the runner, so a wrong mapping/sign makes the twin
robot curl the wrong way. The synthetic overhead image (brown floor, white robot and its shadow, white cup, black egg,
optional hand) goes through the SAME CameraMap and detectors.

A software test harness, not evidence of physical transfer.
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
EGG_SETTLED_Z = 0.0750
PEDESTAL_Z = 0.002
FLOOR, ROBOT, CUP, EGG, HAND = (150, 112, 72), (238, 238, 236), (236, 236, 236), (16, 16, 16), (205, 160, 130)
SHADOW = (92, 68, 44)        # robot shadow on the cardboard: darker brown, must not look like the black egg
IMAGE_NOISE = 3.0            # grey-level noise (sigma) so colour sampling sees realistic pixels


def build_scene(egg_uv, model_xml: Path, dt=1e-3):
    s = mujoco.MjSpec.from_file(str(model_xml))
    s.option.timestep = dt
    s.option.integrator = mujoco.mjtIntegrator.mjINT_IMPLICITFAST
    x, y = C.c_to_model_xy(*egg_uv)
    for prefix, name, z in (("ped/", "spirob_pedestal.xml", PEDESTAL_Z), ("egg/", "spirob_egg.xml", EGG_SETTLED_Z)):
        child = mujoco.MjSpec.from_file(str(ASSETS / name))
        frame = s.worldbody.add_frame(pos=[x, y, z])
        body = frame.attach_body(child.worldbody.first_body(), prefix, "")
        if prefix == "ped/":
            body.mocap = True
    m = s.compile()
    return m, mujoco.MjData(m)


class Twin:
    def __init__(self, cable: CableMap, camera: CameraMap, egg_uv, image_size=(1920, 1080), physics_dt=1e-3,
                 fault: str | None = None, frame_noise_px: float = 0.0, seed: int = 0, start_diff_m: float = 0.0,
                 model_xml: Path | None = None):
        self.cable, self.camera = cable, camera
        self.m, self.d = build_scene(egg_uv, model_xml or LAB / "model" / "spirob_robot.xml", physics_dt)
        self.egg = self.m.body("egg/egg").id
        self.ped_mocap = self.m.body_mocapid[self.m.body("ped/pedestal").id]
        self.egg_qadr = self.m.jnt_qposadr[self.m.body_jntadr[self.egg]]
        self.lengths = np.asarray(cable.model_to_physical_mm(C.payouts_from_diff(start_diff_m)), dtype=float)
        self.target = self.lengths.copy()
        self.speed = np.zeros(2)
        self.t = 0.0
        self.rows: list[dict] = []
        self.ids = itertools.count(1)
        self.events: list[dict] = []
        self.state = ["HOLDING", "HOLDING"]
        self.fault = fault
        self.size = image_size
        self.rng = np.random.default_rng(seed)
        self.frame_noise_px = frame_noise_px
        self.commands: list[dict] = []
        self.hand_uv = None
        self._apply_ctrl()
        if start_diff_m:  # let the robot settle into the start shape without disturbing the egg
            saved = self.d.qpos[self.egg_qadr:self.egg_qadr + 7].copy()
            self.d.qpos[self.egg_qadr:self.egg_qadr + 3] = [1.0, 1.0, EGG_SETTLED_Z]
            self.advance(2.0)
            self.d.qpos[self.egg_qadr:self.egg_qadr + 7] = saved
            self.d.qvel[:] = 0
            mujoco.mj_forward(self.m, self.d)
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
        if any(not 0.05 <= s <= 20.0 for s in kw["speeds_mm_s"]):
            self.events.append(dict(type="rejected", id=cid, message="speed outside 0.05..20 mm/s"))
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
        for _ in range(int(round(seconds / self.m.opt.timestep))):
            if self.fault != "stuck":
                step = self.speed * self.m.opt.timestep
                self.lengths += np.clip(self.target - self.lengths, -step, step)
            self._apply_ctrl()
            mujoco.mj_step(self.m, self.d)
            self.t += self.m.opt.timestep
            if int(round(self.t * 1000)) % 50 == 0:  # 20 Hz encoder samples, like the worker poll
                self.state = ["MOVING" if mv else "HOLDING" for mv in np.abs(self.target - self.lengths) > 1e-3]
                for ax in (0, 1):
                    self.rows.append(dict(axis=ax, t_s=self.t, length_mm=float(self.lengths[ax]),
                                          target_mm=float(self.target[ax]), state=self.state[ax]))
                self.rows = self.rows[-400:]

    def place_new_egg(self, uv):
        """The human replaces the egg: pedestal + egg at a new place, robot untouched."""
        x, y = C.c_to_model_xy(*uv)
        self.d.mocap_pos[self.ped_mocap] = [x, y, PEDESTAL_Z]
        self.d.qpos[self.egg_qadr:self.egg_qadr + 7] = [x, y, EGG_SETTLED_Z, 1, 0, 0, 0]
        self.d.qvel[self.m.jnt_dofadr[self.m.body_jntadr[self.egg]]:][:6] = 0
        mujoco.mj_forward(self.m, self.d)

    # -------------------------------------------------- what the runner sees
    def remote(self) -> dict:
        axes = [dict(enabled=True, rest_count=0, state=self.state[i], absolute_mm=float(self.lengths[i])) for i in (0, 1)]
        phase = "HOLDING" if all(s == "HOLDING" for s in self.state) else "BUSY"
        return dict(type="status", phase=phase, pending=phase != "HOLDING", axes=axes, rest_lengths_mm=list(self.cable.rests_mm))

    def diff_m(self) -> float:
        return float(C.diff_from_payouts(self.cable.physical_to_model_m(self.lengths))[0])

    def egg_uv_true(self) -> np.ndarray:
        x, y = self.d.xpos[self.egg, :2]
        return np.asarray(C.model_xy_to_c(x, y))

    def egg_z(self) -> float:
        return float(self.d.xpos[self.egg, 2])

    def _px(self, uv):
        return tuple(int(v) for v in self.camera.c_to_px(uv))

    def render(self) -> np.ndarray:
        w, h = self.size
        img = np.full((h, w, 3), FLOOR, np.uint8)
        names = [f"link_{i:03d}" for i in range(1, 22)]
        pts = np.asarray([self.camera.c_to_px(C.model_xy_to_c(*self.d.body(n).xpos[:2])) for n in names], np.int32)
        ppm = self.camera.pixels_per_m
        cup = self.d.mocap_pos[self.ped_mocap][:2]
        c = self._px(C.model_xy_to_c(*cup))
        half = int(0.014 * ppm)
        cv2.rectangle(img, (c[0] - half, c[1] - half), (c[0] + half, c[1] + half), CUP, -1)
        egg_below = self.egg_z() < 0.04
        cv2.polylines(img, [pts + np.int32(round(0.012 * ppm))], False, SHADOW, 25)
        if not egg_below:
            cv2.polylines(img, [pts], False, ROBOT, 25)
        if self.fault != "hide_egg":
            px = self.camera.c_to_px(self.egg_uv_true()) + self.rng.normal(0, self.frame_noise_px, 2)
            cv2.circle(img, (int(px[0]), int(px[1])), int(round(0.020 * ppm)), EGG, -1)
        if egg_below:
            cv2.polylines(img, [pts], False, ROBOT, 25)
        if self.hand_uv is not None:
            hp = self._px(self.hand_uv)
            cv2.ellipse(img, hp, (int(0.05 * ppm), int(0.09 * ppm)), 20, 0, 360, HAND, -1)
        noise = self.rng.normal(0, IMAGE_NOISE, img.shape)
        return np.clip(img + noise, 0, 255).astype(np.uint8)

    def egg_in_frame(self) -> bool:
        px = self.camera.c_to_px(self.egg_uv_true())
        r = 0.020 * self.camera.pixels_per_m
        return r <= px[0] <= self.size[0] - r and r <= px[1] <= self.size[1] - r

    def item(self, age_s: float = 0.05) -> dict:
        motor = [r for r in self.rows if r["t_s"] <= self.t + 1e-9]
        frame_t = motor[-1]["t_s"] if motor else self.t
        packet = dict(t_s=frame_t, motor=motor, motor_active=True, state="STREAMING", motor_age_s=[0.02, 0.02],
                      frame_age_s=age_s, rest_lengths_mm=list(self.cable.rests_mm), session="twin")
        return dict(rgb=self.render(), packet=packet, displayed_age_s=age_s)

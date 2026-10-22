"""MuJoCo model adapter. Joint 2 is the reference; joint 1 is never fitted."""
from pathlib import Path
import shutil
import xml.etree.ElementTree as ET
import mujoco
import numpy as np
from scipy.optimize import minimize, root


class SpiRobModel:
    def __init__(self, xml):
        self.xml = Path(xml).resolve()
        self.m = mujoco.MjModel.from_xml_path(str(self.xml))
        self.d = mujoco.MjData(self.m)
        m = self.m
        self.names = [m.joint(i).name for i in range(m.njnt)]
        expected = [f"j_{i:03d}" for i in range(1, m.njnt + 1)]
        if self.names != expected or m.nq != m.nv or m.nu != 2:
            raise ValueError("This adapter expects the supplied ordered, planar, two-cable hinge model.")
        if not np.all(m.jnt_type == int(mujoco.mjtJoint.mjJNT_HINGE)):
            raise ValueError("Only hinge joints are supported by this static adapter.")
        if not np.allclose(m.actuator_gear[:, 0], 1):
            raise ValueError("This adapter expects unit tendon actuator gear.")
        self.original_k = m.jnt_stiffness.copy()
        self.original_d = m.dof_damping.copy()
        self.k2 = float(self.original_k[1])
        self.d2 = float(self.original_d[1])
        self.k_weights = self.original_k[1:] / self.k2
        self.d_weights = self.original_d[1:] / self.d2
        self.rk = float(np.median(self.k_weights[1:] / self.k_weights[:-1]))
        self.rd = float(np.median(self.d_weights[1:] / self.d_weights[:-1]))
        if not np.allclose(self.k_weights, self.rk ** np.arange(len(self.k_weights)), rtol=1e-8):
            raise ValueError("Uploaded stiffness weights are not exponential; review the model adapter.")
        if not np.allclose(self.d_weights, self.rd ** np.arange(len(self.d_weights)), rtol=1e-8):
            raise ValueError("Uploaded damping weights are not exponential; review the model adapter.")
        self.stiffness_ratio, self.stiffness_exponent = self.rk, 1.
        self.qspring = m.qpos_spring.copy()
        mujoco.mj_forward(m, self.d)
        self.neutral_lengths = self.d.actuator_length.copy()
        self.reference_origin = self.d.body("link_002").xpos.copy()
        # Project onto the robot's bending plane, including upright XZ models.
        self.vertical = self.d.site('tip_site').xpos-self.d.xanchor[0]
        self.vertical /= np.linalg.norm(self.vertical)
        normal=self.d.xaxis[1].copy()
        self.lateral=np.cross(self.vertical,normal)
        self.lateral/=np.linalg.norm(self.lateral)
        self.vertical=np.cross(normal,self.lateral)
        if not np.allclose(np.abs(self.d.xaxis@normal),1,atol=1e-6):
            raise ValueError('The static adapter requires parallel planar hinge axes.')
        self.reference_gravity_energy = -np.sum(m.body_mass[:, None] * self.d.xipos * m.opt.gravity)
        self.body_ids = [m.body(f"link_{i:03d}").id for i in range(2, m.njnt + 1)]
        # Physical diagnostic landmarks use joint anchors, not uniformly spaced
        # samples of the fitted curve. Joint 1 is the fixed boundary and omitted.
        neutral = self.joint_curve()
        arc = np.r_[0., np.cumsum(np.linalg.norm(np.diff(neutral, axis=0), axis=1))]
        self.joint_tail_m = arc[-1] - arc
        self.kp = m.actuator_gainprm[:, 0].copy()
        if not (np.all(self.kp > 0) and np.allclose(m.actuator_biasprm[:, 1], -self.kp)
                and np.allclose(m.actuator_forcerange[:, 1], 0)):
            raise ValueError("Expected tension-only position actuators.")

    def set_stiffness_profile(self, ratio=None, exponent=1.):
        """k_i = k2 * ratio**(exponent*(i-2)); never alter the damping profile.

        The XML supplies only the adjacent ratio. The exponent multiplier is a
        new review parameter, defaulting to one, not a recovered generator value.
        """
        ratio = self.rk if ratio is None else float(ratio)
        exponent = float(exponent)
        if not np.isfinite([ratio, exponent]).all() or not .5 <= ratio <= 1.5 or not 0 <= exponent <= 3:
            raise ValueError('Use stiffness ratio 0.5..1.5 and exponent multiplier 0..3.')
        # Preserve the supplied weights exactly for the default profile.
        self.k_weights = (self.original_k[1:] / self.k2 if ratio == self.rk and exponent == 1.
                          else ratio ** (exponent * np.arange(len(self.original_k)-1)))
        self.stiffness_ratio, self.stiffness_exponent = ratio, exponent

    def set_stiffness_weights(self, weights):
        """Independent profile: k_i = k2 * weights[i-2] for j_002 … j_N (weights[0] == 1)."""
        w = np.asarray(weights, float)
        if w.shape != (len(self.original_k)-1,) or not np.all(np.isfinite(w)) or np.any(w <= 0):
            raise ValueError(f'Need {len(self.original_k)-1} positive joint stiffness weights.')
        self.k_weights = w / w[0]
        self.stiffness_ratio, self.stiffness_exponent = None, None

    def scale(self, k2=None, d2=None):
        k2 = self.k2 if k2 is None else float(k2)
        d2 = self.d2 if d2 is None else float(d2)
        if not np.isfinite([k2, d2]).all() or k2 <= 0 or d2 < 0:
            raise ValueError("k2 must be positive and d2 nonnegative.")
        self.m.jnt_stiffness[1:] = k2 * self.k_weights
        self.m.dof_damping[1:] = d2 * self.d_weights
        self.m.jnt_stiffness[0] = self.original_k[0]
        self.m.dof_damping[0] = self.original_d[0]

    def controls(self, displacement_mm, mean_offset_mm=0, difference_offset_mm=0):
        offsets = np.array([mean_offset_mm + difference_offset_mm / 2,
                            mean_offset_mm - difference_offset_mm / 2])
        ctrl = self.neutral_lengths + (np.asarray(displacement_mm) + offsets) / 1000
        if not np.isfinite(ctrl).all() or np.any(ctrl < self.m.actuator_ctrlrange[:, 0]-1e-9) or np.any(ctrl > self.m.actuator_ctrlrange[:, 1]+1e-9):
            raise ValueError("Mapped cable lengths exceed this model's actuator range.")
        return np.clip(ctrl,self.m.actuator_ctrlrange[:,0],self.m.actuator_ctrlrange[:,1])

    def curve(self):
        p = np.vstack([self.d.xpos[self.body_ids], self.d.site("tip_site").xpos])
        # Use the neutral bending plane; preserve the model orientation and gravity.
        relative=p-self.reference_origin
        return np.column_stack([relative@self.lateral,relative@self.vertical])

    def joint_curve(self):
        p = np.vstack([self.d.xanchor[1:], self.d.site("tip_site").xpos])
        relative=p-self.reference_origin
        return np.column_stack([relative@self.lateral,relative@self.vertical])

    def base_point(self):
        """Fixed joint-1 anchor in the same planar coordinates as curve()."""
        p = self.d.xanchor[0]
        return np.array([(p-self.reference_origin)@self.lateral, (p-self.reference_origin)@self.vertical])

    def landmark_points(self, names, mode='joints'):
        curve = self.joint_curve()
        ids = self.names[1:] + ['tip']
        if mode == 'centres':
            curve = (curve[:-1]+curve[1:])/2
            ids = ['link_'+name.split('_')[-1] for name in ids[:-1]]
        return np.array([curve[ids.index(name)] for name in names])

    def equilibrium(self, ctrl, k2=None):
        """Minimize the free-space static potential using MuJoCo lengths and forces.

        Contact/friction equilibria are deliberately rejected, not silently approximated.
        Dynamic damping is absent from this static objective because qvel is zero.
        """
        self.scale(k2)
        m, d = self.m, self.d
        d.ctrl[:] = ctrl
        weights = np.sqrt(m.jnt_stiffness)
        cap = -m.actuator_forcerange[:, 0]

        def potential(x):
            d.qpos[:] = self.qspring + x / weights
            d.qvel[:] = 0
            mujoco.mj_forward(m, d)
            extension = np.maximum(d.actuator_length - d.ctrl, 0)
            elastic = np.minimum(extension, cap / self.kp)
            tendon_energy = np.sum(.5 * self.kp * elastic**2 + cap * (extension - elastic))
            gravity_energy = -np.sum(m.body_mass[:, None] * d.xipos * m.opt.gravity)
            energy = .5 * np.dot(x, x) + tendon_energy + gravity_energy - self.reference_gravity_energy
            gradient = (d.qfrc_bias - d.qfrc_passive - d.qfrc_actuator) / weights
            return energy, gradient

        bounds = list(zip((m.jnt_range[:, 0] - self.qspring) * weights,
                          (m.jnt_range[:, 1] - self.qspring) * weights))
        result = minimize(potential, np.zeros(m.nq), jac=True, method="L-BFGS-B", bounds=bounds,
                          options={"gtol": 1e-11, "ftol": 1e-15, "maxiter": 500, "maxls": 40})
        potential(result.x)
        residual = float(np.max(np.abs(d.qfrc_bias - d.qfrc_passive - d.qfrc_actuator)))
        if residual > 1e-7:
            # Refine stationarity when potential-value precision terminates L-BFGS early.
            energy_before = potential(result.x)[0]
            refined = root(lambda x: potential(x)[1], result.x, method='hybr', options={'xtol': 1e-9})
            lo, hi = np.array(bounds).T
            energy_after = potential(refined.x)[0]
            if np.any(refined.x < lo) or np.any(refined.x > hi) or energy_after > energy_before+1e-12:
                potential(result.x)
            residual = float(np.max(np.abs(d.qfrc_bias-d.qfrc_passive-d.qfrc_actuator)))
        if not np.all(np.isfinite(d.qpos)) or residual > 2e-6:
            raise RuntimeError(f"Static equilibrium did not converge: residual {residual:.3g} N m")
        if d.ncon:
            raise RuntimeError("Contact detected: this free-space static solver cannot score this pose.")
        return {"curve_m": self.curve().copy(), "qpos": d.qpos.copy(),
                "controls_m": np.array(ctrl), "tendon_lengths_m": d.ten_length.copy(),
                "actuator_force_N": d.actuator_force.copy(), "residual_Nm": residual,
                "contacts": int(d.ncon), "iterations": int(result.nit)}

    def audit(self):
        return {"mujoco_version": mujoco.__version__, "joint_count": self.m.njnt,
                "reference_joint": "j_002", "original_k2_Nm_per_rad": self.k2,
                "original_d2_Nm_s_per_rad": self.d2, "stiffness_ratio": self.rk,
                "damping_ratio": self.rd, "joint1_stiffness": float(self.original_k[0]),
                "joint1_damping": float(self.original_d[0]),
                "neutral_tendon_lengths_m": self.neutral_lengths.tolist(),
                "compiled_mass_kg": float(self.m.body_mass.sum()),
                "gravity_m_s2": self.m.opt.gravity.tolist(),
                "actuator_kp_N_m": self.kp.tolist(),
                "actuator_kv_N_s_m": (-self.m.actuator_biasprm[:, 2]).tolist(),
                "stiffness_weights_from_joint2": self.k_weights.tolist(),
                "damping_weights_from_joint2": self.d_weights.tolist()}

    def export(self, destination, k2, d2=None):
        destination = Path(destination)
        destination.mkdir(parents=True, exist_ok=True)
        self.scale(k2, d2)
        tree = ET.parse(self.xml)
        for node in tree.findall(".//worldbody//joint"):
            name = node.get("name")
            if name not in self.names[1:]:
                continue
            i = self.names.index(name)
            node.set("stiffness", format(self.m.jnt_stiffness[i], ".17g"))
            node.set("damping", format(self.m.dof_damping[i], ".17g"))
        shutil.copytree(self.xml.parent / "meshes", destination / "meshes", dirs_exist_ok=True)
        path = destination / "spirob_robot.xml"
        tree.write(path, encoding="utf-8", xml_declaration=True)
        return path

# SpiRob Lab 0.9.0 — new model and reviewed campaigns

Historical release notes: [0.9.1](UPDATE_0_9_1.md) adds mixed generation, a difference-sweep draft, and a reviewed return to starting lengths at normal campaign completion.

This update uses the supplied new geometry and meshes. It activates a separate model at `model/new_20261013/spirob_robot.xml` and keeps the old model and calibration files. It does not move motors during installation or simulation preview.

## Model and physical references

| Setting | New model / default |
|---|---|
| Ordered hinge joints | 21; joint 1 remains the boundary |
| Neutral tendon lengths | 227.353314410471 mm on both tendons |
| Position actuator command range | 0.15–0.30 m (150–300 mm) |
| Position gains | kp = 2500 N/m; kv = 20 N·s/m |
| Tension-only actuator force range | −1000 to 0 N, retained from the previous position model |
| Initial k₂ | 0.18302833187063192 N·m/rad, from the new XML |
| Initial d₂ | 0.009151416593531596 N·m·s/rad, from the new XML |
| Adjacent stiffness ratio | 1.03⁻³; the displayed exponent multiplier starts at 1 |

The source geometry, mass, inertia, gravity, joints, tendon routing and joint gains are retained. Only cable actuators are converted to the previous position-PD convention. The adapter now projects the bending plane correctly for the new upright XZ model as well as the previous XY model. Old fitted k₂ and cable offsets are not applied to the new robot. Camera alignment is retained as a starting point; the old joint-spacing profile is cleared because the geometry changed. Recheck base placement, scale and cable mapping.

Before Connect, open **Session / data → Cable references / limits**. Enter each physical cable's measured length at the pose you will declare as rest, each minimum/maximum, the speed ceiling, and unreferenced setup travel. Save updates the mean and the independent cable targets immediately and persists the values. These values are sent to WSL for that session; editing the remote `config.json` no longer silently competes with a hard-coded GUI reference. Use **Read reference lengths from WSL config** to import remote edits explicitly. The remote file is not overwritten.

The default physical references are the model's neutral lengths, not measurements of your installed cables. Correct them when needed. Model controls follow:

`control[j] = neutral[j] + (sign[j] * (physical[mapping[j]] - physical_rest[mapping[j]]) + offset[j]) / 1000`

The mapping dialog determines cable order/signs. Offsets are in millimetres. **Map model bounds to entered physical references** inverts this equation to propose physical limits; review/correct them before saving. Settings lock at Connect and require disconnecting to change.

`setup.py check` reports the effective next-session GUI references and separately reports the stored WSL references. USB/key checks still do not open the motors or camera.

## Motion and STOP

- Regular and jog speed controls support **0.05–20 mm/s**, with a configurable session ceiling. This is the existing motor driver's application limit; its RPM, feedback, encoder and stop checks remain active.
- Jog step supports **0.05–1000 mm**, subject to the actual endpoints. Before reference, cumulative travel stays within the configured setup envelope (default ±150 mm from connection position). After reference, both physical min/max bounds are enforced. A 1000 mm widget maximum does not authorize a 1000 mm move outside those bounds.
- **Both − / Both +** jog both cables by the same selected increment. Both targets are validated and staged before their serial commits. This is coordinated dispatch, not a claim of perfectly simultaneous mechanical starts.
- **STOP — hold both**, including Escape, interrupts manual/campaign motion through the existing priority stop path and clears pending motion. Completed campaign photographs remain retained. The motors keep holding torque; Disconnect requests torque release. Software STOP is not a hard-wired emergency stop.
- Mean/difference controls also enforce the selected full-difference limit. Independent cable mode uses each cable's absolute bounds.

## Random campaign and simulation review

1. Before Connect, choose references, limits and session duration. Open **Campaign → Plan / run campaign**.
2. Keep the existing 73-pose draft, edit waypoint rows, load a plan, or choose **Random generator**. Random generation supports both-cable or difference mode, length/difference bounds, a speed range, linked or separate cable speeds, pose count (2–200), and a reproducible seed. The default is 12 poses within ±20 mm of each reference at 1–4 mm/s, with equal speeds per pose.
3. Inspect **Preflight** to see expanded endpoints, estimated duration and JPEG budget. Then click **Simulate / review**. MuJoCo integrates the original 0.0001 s timestep and uses the selected per-cable rates. Progress and cancellation remain available. No SSH or motor commands are issued by preview.
4. Review the animated backbone and commanded-versus-measured tendon lengths. Scrub the entire journey or play it at adjustable speed. The display reports observed `mjData.ten_length` extrema and contacts. It conditions the initial model pose for five seconds; actual motor ramps, backlash and load history are not represented.
5. Click **Approve this simulated plan**. Then Connect, confirm/reference the physical rest, and click **Start approved campaign**. The expanded, approved commands are reused; the generator is never rerun at Start. Starting encoder lengths must match the preview within 0.5 mm. If they differ, simulate again from the current held pose.
6. Editing the plan, motion limits, model files, selected model parameters, or mapping requires a new preview/approval. Sufficient session time, fresh camera/encoders and healthy referenced motors are still required. Approval never enables or starts motors by itself.
7. Each reached pose still requires target arrival and stable encoder readings, plus dwell, before saving one original JPEG. Manual campaigns retain images for later batch joint placement; automatic campaigns attempt a shared k₂ fit after acquisition. Complete the shared fit before comparing per-pose k₂ values.

The campaign saves the approved plan, model fingerprint, parameters and preview summary alongside its usual photographs and motor inputs. The preview animation stays in memory; it is not recorded as a video. Existing limits remain 500 expanded captures and 256 MiB of campaign JPEGs. The random generator does not add an unrequested return-to-rest move.

## What the tendon range means

`ten_length` measures the tendon path at the current simulated pose. A position actuator's command range is a separate input limit. A tension-only cable can be slack, so neither the commanded value nor neutral length gives a universal reachable or mechanically permissible range.

The 12-pose default random journey was simulated without contacts. A separate full-range test at (150,300), (300,150), then neutral produced up to 18 model contacts; its sampled tendon range was approximately 153.59–297.00 mm. These are results for those journeys, not global extrema or hardware validation. The existing free-space k₂ solver rejects contact equilibria. Use the preview to identify and reduce such poses before a fitting campaign, or explicitly exclude affected images later.

## Install and rollback

Close the running GUI after disconnecting and handling its pending data. Use the installer included in `spirob-lab-v0.9.0-update.zip`, then **deploy** the changed WSL runtime before starting. The installer targets the supplied 0.8.0 project, checks source/payload hashes before modifying files, backs up every changed file, preserves old models/calibrations and user data, and prints a rollback command. It does not use the old generated `RELEASE_SHA256.json` as an update gate. Re-running an already installed update preserves subsequent model selection and motion-setting edits.

```bash
unzip -n "$HOME/Downloads/spirob-lab-v0.9.0-update.zip" -d "$HOME/Downloads" &&
python3 "$HOME/Downloads/spirob-lab-v0.9.0-update/install.py" \
  --project "$HOME/Research_Work/SpiRob_Analysis/spirob-lab" &&
cd "$HOME/Research_Work/SpiRob_Analysis/spirob-lab" &&
python3 setup.py deploy &&
python3 setup.py check &&
bash start.sh
```

`bash start.sh --demo` deliberately continues to use the old recorded robot, model and fitted calibration. It does not validate this new model or physical rig. Use the campaign simulation preview to review the new model without connecting.

Verification details and executable checks are in `release_metadata/v0.9.0/verification.json` and `tests/verify_v090.py`. All validation here is software/simulation; the physical rig was not operated.

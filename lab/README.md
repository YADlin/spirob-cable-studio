# SpiRob Lab — connect, move, compare

A small desktop experiment for the existing Windows-camera / WSL-motor rig.
The workstation displays the video, advances MuJoCo, and calculates image error.
The laptop owns the motors and recording. Version 0.9.1. This update has recorded-image, offscreen UI and simulated-driver checks; the new workflow still needs verification on the physical rig.

**Start with the [Operating manual](docs/OPERATING_MANUAL.html)** for installation, daily use, setup jogs, individual/mean actuation, point inspection, recovery and shutdown. The same manual is available from the application.

## RL egg knockdown (feature branch)

An **RL** tab runs an exported `spirob-mjlab` knockdown policy (camera + encoders only) with Dry run, Arm, Run episode, Stop / Hold and Return to start. Workstation-only; no WSL redeploy. Read [docs/RL_KNOCKDOWN.md](docs/RL_KNOCKDOWN.md) before arming.

## Mixed campaigns and return to start (0.9.1)

Use **Campaign → Difference sweep…** for uniformly spaced poses between two cable-difference extremes. Use **Mixed / random… → Mixed** to combine difference, cable 1 only, cable 2 only and independent two-cable moves in one seeded journey.

Every new campaign ends with a simulated and reviewed return to its starting cable lengths at the selected **Return [mm/s]** speed (default 1 mm/s). Completion requires stable encoders and a retained return image. STOP, faults, disconnect and timeout abort the journey without automatic return. The return image is excluded from fitting by default and can be included during review.

See [the 0.9.1 guide](docs/UPDATE_0_9_1.md). This workstation update preserves your current model, PD gains, references, limits, calibration and data. No WSL deployment is needed if 0.9.0 is already deployed.

## New model, lengths and campaigns (0.9.0)

Read [the 0.9.0 guide](docs/UPDATE_0_9_0.md) before running the new 21-joint model. Set measured references and bounds in **Session / data → Cable references / limits**; defaults are 227.353314 mm and 150–300 mm. Speeds reach 20 mm/s and jogs support larger bounded steps, including both cables together. **STOP — hold both** remains visible.

Campaigns now require **Simulate / review → Approve → Start approved campaign**. A reproducible random generator provides selectable ranges, speeds and pose counts. Preview shows the MuJoCo backbone, actual tendon lengths and contact warnings. It never commands hardware. The approved expanded plan is reused for acquisition.

The installer preserves the previous model/calibration and writes a backup. **Run `python3 setup.py deploy` when first upgrading the runtime to 0.9.0** to send the changed motion profile and jog support to WSL. Existing photographs and runs are retained. The recorded demo below uses the current model with photographs of the previous robot.

**Model files.** `model/spirob_robot.xml` and `model/meshes/` are the only model. To use another print, replace these files (keep the names). `python model_check.py` confirms the XML matches the rig: horizontal bending plane, gravity along −Z, plane height from `config.json → model_check`. The Lab runs the same check at start-up.

## Start with the recorded example

Extract this folder alongside `spirob-analysis-pilot`; keep the old project.
On the Ubuntu workstation, inside this folder:

```bash
uv sync --frozen --extra live
bash start.sh --demo
```

This opens a real recorded held image. It does not connect to SSH, camera or motors.
Try another Hold in the selector. Change k₂, click **Apply to simulation**, then
**Freeze measured pose + sweep k₂**. The lowest sampled k₂ is selected; **Apply selection** changes the simulation. Choose the interval and grid size before sweeping, and **Save sweep…** for PNG/CSV/JSON.
Changing d₂ has no effect on this static example because velocity is zero.

Read [READ_THE_CODE.md](docs/READ_THE_CODE.md) next. It follows one experiment through
three modules and gives the exact functions to read, in order.

## Use the physical robot

Follow the [Operating manual](docs/OPERATING_MANUAL.html) once. It imports your existing
paths and deploys a separate `SpiRob_Simple_Runtime` on Windows. It does not replace
the motor controller, the camera addon, or the previous analysis project.

After setup, launch:

```bash
bash start.sh
```

Camera ON + MuJoCo backbone ON → calibrate joints and scale → select session duration/maximum difference → Connect → inspect video → confirm physical rest → Enable at rest → wait for the
red model → Move → wait for both shapes to settle → compare → Disconnect → Save / Discard / Decide later.

The window accepts a **full difference** D = L1 − L2. At mean 220 mm and D=20 mm,
the targets are 230 and 210 mm. **Stop / Hold** interrupts motion and retains torque;
**Disconnect** releases torque. Returning D to zero is a move, not a re-reference.

## What is where

| Location | Purpose |
|---|---|
| `connection.py` | SSH, camera connection, control heartbeat, shutdown transport |
| `actuation.py` | Laptop session loop; requested lengths to existing motor worker |
| `campaign.py`, `campaign_storage.py`, `campaign_analysis.py` | Campaign planning, acquisition, frozen records and shared/per-pose sweeps |
| `campaign_window.py`, `campaign_review.py` | Plan, preflight, thumbnails, batch annotation and results |
| `analysis.py` | Live MuJoCo update, settled comparison, explicit stiffness sweep |
| `stiffness_sweep.py`, `surface_review.py` | Optional frozen J(k₂, one profile constant) calculation, 3D review and export |
| `manual_analysis.py` | Frozen user-placed points and their captured encoder-based equilibrium model; no vision processing |
| `live_vision.py` | Bounded background image processing; never blocks camera I/O or model stepping |
| `main.py`, `interface.py` | Start the window; buttons and drawing |
| `point_inspector.py`, `landmarks.py` | Frozen inspection, calibrated rigid joint distances, live J and same-pose sweep |
| `display_setup.py`, `appearance.py` | Fixed base alignment, cable mapping, high-contrast themes |
| `lab.local.json` | Saved theme and explicitly saved comparison setup; created locally |
| `spirob_analysis/` | Existing model, image measurement and offline fitting calculations |
| `spirob_live/` | Small Windows adapter around the installed camera addon |
| `setup.py` | Import paths, deploy, unlock key, attach USB, fetch data |
| `rig.local.json` | Your machine addresses and paths; created locally |
| `config.json` | Scientific settings, image region and timing gates |
| `calibration/pilot_fit.json` | The existing conditional k₂ fit and fixed camera mapping |
| `data/quasistatic_review.zip` | Supplied recorded pilot used by the offline example |
| `logs/runs/` | Connection and analysis logs, retained pending your end-of-run decision |
| Windows `SpiRob_Data/<session>/` | Motor logs and explicitly recorded images; retained pending Save / Discard |
| `run_storage.py` | Post-run decisions and recovery; never opens hardware |

[SCIENTIFIC_METHOD.md](docs/SCIENTIFIC_METHOD.md) explains the two original optimizations,
the current cost, parameter scaling, and the limits of the live comparison.
[VERIFICATION.md](docs/VERIFICATION.md) separates software checks from pending rig checks.

## Campaigns in 0.8.0

Use **Campaign → Plan / run campaign** for autonomous waypoint and interpolated
motion acquisition. The default contains 73 moderate poses with reverse sweeps,
independent cable moves and repeated references. Select a 30-minute session
before Connect, inspect Preflight, then Start. One original JPEG is retained per
pose (256 MiB campaign image cap); no continuous recording is required.

Choose automatic shared-k₂ fitting after the journey, or collect for later
manual marking in one sitting. The batch editor preserves zoom, advances joint
selection, saves drafts and offers reviewed image-flow suggestions. Calculate
one shared k₂ baseline before requesting per-pose minima from the same grid.
Missing observations are explicitly queued, and subset fits are labelled.
Campaign files are included in the normal end-of-run Save/Discard decision.
See [UPDATE_0_8_0.md](docs/UPDATE_0_8_0.md) for the complete workflow and safeguards.

## Changes in 0.7.3

Launching no longer opens the old pending-run Save/Discard dialog. The current
run still prompts after shutdown; older runs are reviewed explicitly through
**Session / data → Review pending run data…**.

The existing J–k₂ sweep remains the default. After completing it, check **3D
profile review** in **J / k₂ plots** to explore J against k₂ and either the ratio
r or exponent multiplier b, one at a time: `k_i = k_2 × r^[b(i−2)]`.
Defaults reproduce the XML profile with b=1. Rotate the surface, inspect the
lowest valid grid point and export all samples. Only r^b determines the slope;
r and b are not independent identifiable parameters. See
[UPDATE_0_7_3.md](docs/UPDATE_0_7_3.md) for the workflow and interpretation.

## Retained from 0.7.2

Jog clicks keep the camera and controls in place. Use the **Jog / rest** tab
under the viewer; **Motion**, **Lengths / timing** and **Session / data** hold
the other motor/session controls. Connect, Stop / Hold and Disconnect are fixed
at the top. The right-hand tabs separate **Fit / capture** from **J / k₂ plots**.

Point placement preserves zoom and pan. Zoom with the image toolbar, then click
**Place selected** and place the physical feature. **Fit view** explicitly resets
the viewport. The table also preserves its scroll position during edits.
See [UPDATE_0_7_2.md](docs/UPDATE_0_7_2.md) for the complete workflow and layout map.

## Retained from 0.7.1

**Camera** and **MuJoCo backbone** now have separate ON/OFF switches.
Use either view alone, both together, or neither. Before Connect, Camera OFF
closes the setup camera; backbone alone shows a local reference without opening
camera/SSH or starting the timer. During a motor run, the switches control
visibility while acquisition, model calculations and freshness checks continue.
The status text states this explicitly. See [UPDATE_0_7_1.md](docs/UPDATE_0_7_1.md).

## Retained from 0.7.0

See [UPDATE_0_7_0.md](docs/UPDATE_0_7_0.md) for the complete workflow.

- **Independent cable lengths:** explicit targets for cable 1 and cable 2,
  single-cable or paired moves, and independent speeds with Same speed checked
  by default. At 220 mm rest and default travel, each target is 150–290 mm.
- **Manual points — no image processing:** capture a held pose, place at least
  four named points, confirm that the photographed pose was stationary, and
  sweep. No extracted curve or optical-flow track is needed.
- **Image estimation — live green backbone:** vision runs separately from
  camera delivery and physics. Delayed measurements use their own captured
  model geometry. A visible readiness line explains every unmet sweep condition.

## Retained from 0.6.1 (image-estimation mode)

See [UPDATE_0_6_1.md](docs/UPDATE_0_6_1.md). Manual placement now keeps your exact
coordinates, replaces the green curve, and supplies live J and frozen-pose
sweeps. Clicks are no longer rejected or snapped to an incorrect automatic
centreline. Subsequent frames use image tracking with fixed link lengths;
tracking loss pauses J until a fresh placement. Saved calibration includes small
joint image references for verified reacquisition, not blind coordinate replay.

For backbone length: Alignment / cable mapping → Image scale [px/m]. Increase
scale to lengthen the red overlay; decrease it to shorten it. Or apply the
physical joints, then Match model length to calibrated physical joints → Apply
+ save setup. The selected base stays fixed. Mechanical dimensions and k stay
unchanged. Calibrate at rest with a fixed camera and planar motion.

## Retained from 0.6.0

See [UPDATE_0_6_0.md](docs/UPDATE_0_6_0.md) for the new controls and workflow.

- Place physical joints, Apply to live J, or Apply + save setup. Rigid mode keeps adjacent measured pixel distances fixed using image tracking.
- Preview the real camera and MuJoCo reference pose before Connect, without opening motors or consuming session time. Match scale from calibrated physical link lengths.
- Choose 10, 20, 30, 45 or 60 minutes and a maximum full difference up to the shorter configured rest length (220 mm on the current rig). Defaults remain 10 min / 70 mm.
- Save a combined frozen pose screenshot with both curves, calibrated joints, correspondence lines, J and the same-pose k₂ sweep, plus raw-frame PNG / joint CSV / sweep CSV / JSON.

## Retained from 0.5.1

Base alignment edits preserve the simulated pose and time. MuJoCo can initialize
from held encoder inputs without a camera centreline; missing/invalid image
measurements pause J while the independent backbone continues. The alignment
dialog can restore the supplied image alignment without changing cable mapping.

Connect starts camera acquisition and preserves the two display-switch choices. Disconnect, window close, faults and timeout
stop hardware before asking Save / Discard / Decide later. A crash cannot show a
dialog: pending data survives; open Session / data → Review pending run data to review it explicitly. Discard requires
confirmed shutdown and targets only that run. Network failure leaves the decision
recoverable. Explicit graph/inspector exports are separate and remain untouched.

Preview images stay in memory. Use Start recording to collect footage under the
60-second / 250-MiB cap; Save cannot reconstruct unrecorded video. See the updated
Operating manual for the exact recovery steps.

# SpiRob Lab 0.6.0

This update adds physical joint calibration, camera setup before Connect, longer
sessions, configurable full-difference limits, and a combined pose/sweep capture.

## Before Connect

1. Click **Camera + MuJoCo setup preview**. The real camera opens through the
   established SSH arrangement. Motors are not opened and the run timer has not
   started. The red curve is explicitly the unloaded XML reference pose, not a
   prediction based on encoder feedback. J and k sweeps become available in a
   connected, referenced, settled run.
2. Open **Alignment / cable mapping…**, pick the fixed joint-1 base, and adjust
   rotation if needed. The base marker is distinct from joint 2.
3. Open **Inspect joint / element points…** when the green curve is available.
   Select **Joints + tip**, select a table row, tick **Click image to place selected
   physical point**, and click that physical joint. Repeat for joints that need
   correction, including joints 2 and 3 if visible. Keep point identities ordered
   from base to tip; a hidden proximal joint can remain unavailable.
4. Leave **Keep joint-to-joint distances fixed** and **Also use verified points
   for J / k₂ sweep** checked. Click **Apply joints + save setup for next run**.
   The buttons are in the inspector's right panel, below the point table.
5. To fix virtual/physical length, reopen **Alignment / cable mapping…**, click
   **Match model length to calibrated physical joints**, then **Apply + save
   setup**. It compares the sum of corresponding link lengths, not the straight
   base-to-tip distance of a bent robot. The picked base stays fixed. Scale is
   retained until explicitly changed; it is not refitted during a k sweep.
6. Choose **Session duration** (10, 20, 30, 45 or 60 min) and **Maximum |full
   difference|** before Connect. Click **Connect** when ready. The setup capture
   closes before the motor session starts; the camera may briefly reopen.

## What the joint calibration means

Clicks are projected onto the measured green curve (maximum cross-track distance
15 px). Rigid mode stores the distance between each adjacent pair of visible
joints, including the final joint-to-tip interval. Every subsequent image follows
that same chain of fixed pixel distances backwards from its measured tip. The
model pose and k do not determine green point placement.

These are **estimated joint positions**, not independently detected hinges.
Fixed projected lengths assume planar motion and a fixed camera. An incorrect
curve, occlusion or out-of-plane motion can invalidate them. If a scored joint
cannot fit within the visible curve, J pauses; it does not silently drop that
joint. Include at least four scored points and place every visible joint from
the first included part of the chain through the tip. Hidden proximal joints may
be omitted. Excluding an interior joint from J retains it in the rigid chain.

The original curve-sample objective is preserved until placement is applied
with scoring checked. Element-centre mode retains the older arc-spacing method:
centre-to-centre distances across a hinge are not rigid link lengths. Switching
point definitions changes J's meaning; do not directly compare those scores.

## Motion and duration controls

The default remains 10 minutes and a 70 mm maximum full difference. A selected
limit can be raised as far as the shorter configured cable rest length: **220 mm
for the current 220/220 mm rig**. Set it before connecting; it is fixed for that
run. Both the session coordinator and motor driver validate it.

For a selected maximum difference B, each cable endpoint must also remain within
`rest ± max(70, B/2)` mm and be positive. At B=220, mean=220 and full difference
220, the endpoints are 330 and 110 mm. Changing the mean can therefore reduce the
available difference. The selected limit is a software envelope, not a mechanical
rating. Speed remains 0.05–5 mm/s; unreferenced setup jog bounds, feedback checks,
Stop / Hold, watchdogs and torque-release handling remain in effect.

The Windows capture duration ceiling now covers a full hour. Each connected run
still ends at its selected duration. A setup preview has no experiment timer; it
closes if its workstation lease disappears. Longer sessions do not extend the
separate explicit video-recording cap of 60 seconds / 250 MiB.

## Save a pose with J and k sweep

For existing applied joints: hold a connected pose until settled, choose the k₂
range, and click **Freeze settled pose + sweep k₂**.

To correct joints on that exact captured pose: open the inspector while settled,
place the physical joints, then click **Apply joints + sweep this frozen pose**.
The sweep uses this captured image and its encoder inputs, even if the live
camera subsequently changes. Changing camera alignment or cable mapping after
capture requires another capture.

After the sweep, click **Save pose screenshot + J + k₂ sweep…** in the main window.
For a chosen filename `pose.png`, the export contains:

- `pose.png`: camera image with green curve/joints, red MuJoCo curve/joints,
  corresponding-point lines, captured J, and the same-pose k₂ sweep.
- `pose_frame.png`: original decoded camera image without annotations.
- `pose_joints.csv`: physical/model positions, distances and inclusion status.
- `pose_sweep.csv`: sampled k₂, J and any rejected equilibrium values.
- `pose.json`: frame identity/hash, encoder inputs, camera/joint calibration,
  captured parameters, point geometry and sweep results.

The red curve and its J use k₂ **at capture**. The graph identifies the lowest
sampled J and its k₂ separately. Selecting a best k does not silently repaint the
captured model as if it had been the live prediction. Explicit exports remain
separate from the run's Save / Discard decision.

## Installation and verification

Close SpiRob Lab first. Install `spirob-lab-v0.6.0-update.zip` using its installer,
then run `python3 setup.py deploy`, `python3 setup.py check`, and `bash start.sh`.
The update supports published 0.5.0 and 0.5.1 sources, including both 0.5.1
installers. Local rig settings, scientific configuration, calibration, recorded
data, previous generated reports and the external controller repository are
preserved. Source edits are checked before replacement; replaced files are
backed up. Release reports are under `release_metadata/v0.6.0`.

Recorded-image, geometry, offscreen UI, HTTP, simulated-driver, shutdown and
installer checks are documented in VERIFICATION.md. No physical rig was operated
while preparing this update. The end-of-run Save / Discard / Decide later and
interrupted-run recovery behavior from 0.5.1 remains active.

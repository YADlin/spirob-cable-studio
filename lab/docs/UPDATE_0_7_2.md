# SpiRob Lab 0.7.2 — stable jogging, precise point placement and compact layout

## Jog controls stay in place

The previous jog jump was reproduced: disabling the focused jog button at motion
start moved Qt focus to a control at the top of the shared scroll area, scrolling
from the bottom back to zero. The whole-window scroll area is now removed.
Jog controls have a fixed page beneath the camera, and focus moves to the fixed
Stop / Hold button when the clicked jog becomes temporarily unavailable.

Open **Jog / rest**, set the step and speed, then use either cable's **− Step**
or **+ Step**. Wait for holding feedback before another click. The selected tab,
camera and button positions remain in place during command/status updates.

## Zoom stays fixed while placing points

1. Open **Inspect / place points…** or capture a manual pose.
2. Use the image toolbar's zoom rectangle or pan tool to inspect the joint.
3. Click **Place selected** to leave the navigation tool while retaining its
   zoom/pan. Select the correct point row, then click the physical feature.
4. Continue selecting rows and placing points at the same zoom. Coordinate
   edits, inclusion changes and point resets also keep the current viewport.

**Fit view** explicitly resets the view. **Full frame** chooses whether that fit
shows the complete image or the available curve/points. A new capture resets
the viewport and navigation history for its new image. Toolbar Home/Back/Forward
continue to work after point edits. Resetting points alone does not reset zoom.

The point table keeps its scroll position during edits. Its default columns
show the point identity, physical coordinates, paired distance and source.
Enable **Show model coordinates and residual components** to reveal the remaining
columns. The selected point's x/y fields sit directly below the table.
Automatic tracking controls are under **Apply / sweep** and **Image spacing**;
manual mode shows its stationary-pose confirmation and sweep action.

## Window organization

| Area | Controls |
|---|---|
| Fixed top bar | Connect, Enable at rest, Stop / Hold, Disconnect and rest confirmation |
| Viewer | Separate Camera and MuJoCo backbone switches; Inspect / place points |
| Motion | Mean/difference or independent absolute lengths and speeds |
| Jog / rest | Setup enable, jog step/speed, per-cable steps, rest declaration |
| Lengths / timing | Physical/model cable telemetry and optional timing details |
| Session / data | Session duration, motion envelope, recording and pending-run review |
| Fit / capture | Measurement mode, k₂/d₂, sweep interval and pose capture |
| J / k₂ plots | Live image-error history and the frozen-pose sweep |

Sweep selection and export buttons remain below the right-hand tabs. Drag the
divider between the viewer and analysis areas to change their widths. The
camera and motor controls do not scroll away. Only the fit form may scroll on
a smaller window. Changing status messages uses fixed-height regions with full
text in tooltips, so it cannot push nearby controls around. The model stroke
and base label remain readable at the smaller viewer sizes.

Native clicks were checked at 1280×720, 1366×768 and 1920×1080, including 24 jog
cycles with moving/holding status changes and actual image-toolbar zoom/pan
followed by precise point placement. Manual sweep/export, off-curve calibration,
independent view switches and shutdown/recovery UI regressions pass. Recorded
images and simulated links were used; no physical rig was operated.

This workstation update retains runtime protocol 0.7.0. An already-deployed
0.7.0 or 0.7.1 installation does not need laptop redeployment. Older releases
need `python3 setup.py deploy` after the cumulative installation. Local settings,
calibration, model and recordings are preserved; the installer provides a backup
and rollback command.

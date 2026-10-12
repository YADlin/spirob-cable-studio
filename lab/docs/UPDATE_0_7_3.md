# SpiRob Lab 0.7.3 — end-of-run decisions and optional stiffness surfaces

## Launch without a save dialog

The first-launch dialog was the automatic review of an older pending run, not a
save decision for the new run. The launch-time recovery timer has been removed.
Launching `bash start.sh` does not open a retention dialog or contact the laptop
for recovery. The current run still asks **Save / Discard / Decide later** after
Disconnect, window close, timeout or detected failure, once its writers stop.

Earlier pending runs remain intact. Open **Session / data → Review pending run
data…** when you want to review them. **Decide later** also uses this explicit
review path; reopening the app no longer opens a modal automatically. A forced
kill or power loss cannot show a dialog, so its pending files remain available.
Discard remains disabled for a run whose motor/camera shutdown is unconfirmed.

## The exact stiffness law

The previous implementation used, for flexible joint i ≥ 2:

`k_i = k_2 × r^(i−2)`

The supplied XML gives `r = 0.9151416593531595` approximately. It stores per-joint
stiffness values; the adapter verifies their exponential sequence. It does not
recover separate generator constants from those values.

The optional review writes the same law as:

`k_i = k_2 × r^[b(i−2)]`

Defaults are the ratio read from your XML and **b = 1**. The exponent multiplier
b is a new review parameter, not a claimed original generator constant. k₂ remains
the actual stiffness at joint 2 in N m/rad. Joint 1 and the damping profile remain
unchanged. Ratio r and multiplier b are dimensionless.

Only `r^b` determines the adjacent stiffness ratio. Changing r or b explores the
same shape slope in different coordinates. They cannot be identified independently
from this law, so the surface varies **one** of them alongside k₂. An extra overall
multiplier on k₂ would duplicate k₂ and is not introduced.

## Review a surface

1. Capture a held pose and complete the normal k₂ sweep, using either image
   estimation or manually placed points. The existing **J versus k₂** plot remains
   the default.
2. Open **J / k₂ plots** and check **3D profile review (after a completed sweep)**.
   A separate window uses that sweep's exact frozen points, cable inputs and
   alignment. It does not recapture the moving live view.
3. Choose **Ratio r** or **Exponent multiplier b**. The other constant remains at
   its captured value. Choose each axis's minimum, maximum and number of samples.
   k₂ uses logarithmic spacing; the chosen constant uses linear spacing. Current
   values are included when inside the intervals.
4. Click **Calculate surface**. Progress is displayed; **Cancel calculation**
   stops after the current equilibrium solve. Camera and motor controls continue.
   Closing the review window or pressing Escape also requests cancellation.
5. Drag the graph to rotate it. The orange star and numerical readout identify
   the lowest valid sampled J, k₂ and selected constant. Boundary minima are
   flagged. Invalid equilibria are missing cells with reasons in the exported
   data, never zero-valued minima.
6. Optionally click **Use minimum in parameter fields**, then **Apply to
   simulation** in the main window. The first click only fills k₂/r/b; it does
   not change the model. The explicit Apply changes virtual stiffness while
   preserving the current pose and velocity. It sends no physical motor command.
7. **Save surface…** writes a surface PNG, CSV with every grid sample and error,
   and JSON with parameters, exact observations and image/model hashes. These
   exports remain separate from the combined pose + 1D sweep export.

The optional **Edit exponential stiffness profile (advanced)** controls under
**Fit / capture** let you edit r and b directly. The applied-parameter line shows
acknowledged k₂, d₂, r and b. The normal 1D sweep then holds the applied r/b fixed.
The normal **Apply selection** restores the profile captured by that 1D sweep,
so its selected k₂ is not accidentally combined with a different profile.

Each surface allows 3–31 requested samples per axis (up to 32 after including a
current value). Software bounds are k₂ 0.001–10, r 0.5–1.5 and b 0–3. These bounds
are not evidence that every combination gives a valid or physically appropriate
model. The ranges used, number of valid samples and failures are exported.

The lowest sample is conditional on this pose, grid, points, scale, mapping and
cable offsets. It is not a guaranteed global optimum or a uniquely identified
material stiffness. Refine the grid and compare multiple held poses before
adopting a profile. Static sweeps do not identify damping.

## Installation and verification

This is a workstation update; runtime protocol remains 0.7.0. An already-deployed
0.7.0, 0.7.1 or 0.7.2 does not need laptop redeployment. Older installations need
`python3 setup.py deploy` before check/start. Settings, calibration, model XML
and run data are preserved, with backup/rollback supplied by the installer.

Verification uses recorded images, synthetic known-parameter poses, native Qt
controls and simulated links. It checks default 1D equivalence, both surface
axes, minima, invalid cells, cancellation, frozen provenance, explicit Apply,
exports, launch recovery and retained shutdown workflows. No physical hardware
was operated.

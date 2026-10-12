# Verification — SpiRob Lab 0.7.3

Checks use Python 3.12, MuJoCo 3.3.7, recorded images, synthetic
cameras and the supplied motor worker with simulated drives. No physical motor,
serial port, Windows host or camera was operated here.

## 0.7.3 startup and profile surfaces (6 October 2026)

`tests/verify_v073.py` checks launch with a configured SSH host and an older
unconfirmed pending run: no automatic remote recovery or modal occurs. Explicit
review still works, and unknown shutdown still disables Discard. The retained
v0.5.1 suite checks disconnect, close and timeout decisions after writer closure.

The original XML stiffness weights and default 1D costs are retained. Synthetic
known-parameter observations locate the expected grid minimum for each surface
axis. The ratio/exponent equation, r^b degeneracy, fixed joint 1/damping, preserved
live state, invalid-equilibrium gaps, boundary flag, cancellation and model-hash
gate pass. Native Qt checks exercise both axes, actual 3D rotation, immutable
capture, staging followed by explicit Apply, three display sizes, both themes,
and PNG/CSV/JSON export. Rendered surfaces were visually inspected.

Core recorded science/dynamics, current window, v0.7.2 jog/zoom/layout, v0.7.0
manual/independent-motion/slow-vision and v0.5.1 retention regressions pass.
Physical hardware was not operated. The current core run did not repeat the
older full simulated-driver shutdown scenarios; those remain historical below.

All fifteen supported cumulative installation paths pass check-only, installation,
repeat, rollback and source-edit refusal. User settings, model, calibration,
run data and generated root reports are preserved. See INSTALLER_VERIFICATION.json
in the update archive.

## 0.7.2 layout and interaction correction (6 October 2026)

The old Qt focus/scroll failure was reproduced: a native + Step click followed
by MOVING feedback scrolled the shared viewport from 557 to 0. The corrected
window has no scroll parent around the viewer/jogs, and explicitly moves focus
to Stop / Hold before disabling the clicked jog.

`tests/verify_v072.py` checks 24 native clicks across 1280×720, 1366×768 and
1920×1080. Button/camera coordinates, the chosen tab and analysis scroll offset
stay fixed through moving/holding status and changing messages. Required motion
controls and graphs fit the tested windows. It also drives the real Matplotlib
box-zoom and pan callbacks, then places exact point coordinates and checks that
row selection, numerical edits, inclusions and table updates retain the view.
Home/navigation history, Fit view and new-capture reset behavior are checked.
The rendered window and zoomed editor were visually inspected.

The current window, v0.7.1, v0.7.0, v0.6.1 and v0.5.1 regressions pass, covering
separate views, manual capture/sweep/export, independent motor controls,
off-curve calibration, image tracking and shutdown/recovery. This is a
workstation UI update; runtime protocol, motor commands and scientific
calculations remain as before. Physical hardware was not operated.

All thirteen cumulative installer paths pass, including direct upgrade from
0.7.1 and the complete incremental chain through 0.7.1. Check-only performs no
writes; install, repeated install, backup/rollback and refusal of local source
edits pass. Local settings, calibration, model, data and generated root reports
are preserved. See the archive's `INSTALLER_VERIFICATION.json`.

## 0.7.1 view-control correction (6 October 2026)

`tests/verify_v071.py` checks actual rendered pixels for all four camera/backbone
combinations, including physical/model markers and paired connecting lines.
Fake camera/run links verify standalone reference without camera/SSH/timer,
camera-only setup acquisition, setup shutdown before Connect, independent view
choices across that transition, retained freshness checks when video is hidden,
and reference recovery after Disconnect or a camera-open failure.

All eleven cumulative installer paths pass, including direct upgrade from
0.7.0 and the complete incremental chain through 0.7.0. Dry-run, repeat install,
rollback and source-edit refusal are checked; local configuration, calibration,
model, data and generated root-level reports are preserved. See the archive's
`INSTALLER_VERIFICATION.json`.

The current window, v0.7.0, v0.6.0 and v0.5.1 suites cover the retained manual
sweep/export workflow, independent cable controls, slow vision, setup transition,
alignment and post-run decisions. The 11 simulated-drive scenarios documented
below were run for 0.7.0; this correction does not change motor commands or the
camera protocol. No physical hardware was operated for either release.

## 0.7.0 checks (6 October 2026)

`tests/verify_v070.py --motor-source ABSOLUTE_MOTOR_REPO` verifies:

- Independent 150–290 mm targets at 220 mm rest, separate speeds, linked-speed
  UI defaults, single- and two-cable commands, expanded travel bounds, and
  unchanged mean/full-difference limits. Invalid lengths, speeds, references,
  pending commands and stale previews are rejected before drive writes.
- The actual controller's simulated drives receive individual speed settings
  and encoder-converted targets through the scoped worker extension. Original
  worker classes are restored after a session; the external source is unchanged.
- A manual frozen capture and exact named-point J work with extraction and
  tracking disabled. Non-contiguous visible points are accepted; missing,
  moving, stale or unreferenced motor inputs give explicit reasons. A stationary
  physical-pose confirmation remains required. Frozen inputs do not expire
  during point placement.
- The actual Qt mode selector, independent controls, manual capture, placement,
  sweep and export buttons. Export keeps the same image, encoder inputs and
  points even after the live pose changes. Failed automatic extraction leaves
  the model/reference visible and a readable explanation by the sweep button.
- Delayed measurements use their captured model geometry even if the live
  model has moved. An injected 350 ms vision calculation allows continuing
  camera/model updates (38 model frames and 7 completed measurements in the
  final check). Automatic settled scoring becomes ready, and queues remain
  bounded. This is an isolation test, not a physical-camera throughput claim.

Current `verify.py` science/dynamics checks and **all 11 simulated-driver
scenarios** pass, including independent commands with unequal speeds and
Stop/Hold, normal move/hold, discard, explicit recording, setup controls, cold
camera start, lost heartbeat, stale preview, camera failure, control EOF and
session timeout. Both drive disables and camera-writer exit are confirmed in
every scenario. The four recorded RMSEs remain unchanged.

The v0.5.0, v0.5.1, v0.6.0, v0.6.1, window, transport and slow-analysis/recording
regression checks also pass against this release. Manual editor, window and
combined pose export were rendered and visually inspected; readiness text is
visible and manual mode uses the plot area for its frozen sweep.

The test environment used PySide6 6.8.3, NumPy 2.5.3, SciPy 1.18.1,
OpenCV 5.0.0 and scikit-image 0.26.0 alongside MuJoCo 3.3.7; it is not a fresh
installation of every locked deployment dependency. The shipped dependency
constraints are unchanged. Physical motor travel, actual image quality and
physical point identities require checking on the rig.

Installer verification covers nine supported paths: full 0.5.0, incremental
0.4.0 → 0.5.0, each 0.5.1 installer, incremental through 0.5.1 revision 2,
0.6.0, incremental through 0.6.0, 0.6.1, and the complete incremental chain
through 0.6.1. Check-only performs no writes; install, repeat, rollback and
refusal of edited source pass. User settings, model, calibration, data and
arbitrary legacy generated checksum/verification reports are preserved.
See `INSTALLER_VERIFICATION.json` in the update archive.

## Passed checks retained from earlier releases

- Four recorded holds: RMSE 7.293832, 6.391024, 4.860399 and 9.811050 px.
  Boundary-joint settings, exponential profiles and static damping independence
  are unchanged. Dynamic integration, settled-only J and stale-data gates pass.
- Live Apply preserves q, qdot and simulation time. Actual Qt base Apply and
  reloaded setup project joint 1 to the selected native pixel. Entirely off-image
  setup is rejected. Camera-alignment reset preserves cable mapping.
- Missing vision and invalid scoring geometry leave the backbone visible with J
  disabled. Held encoder inputs initialize without image extraction, including
  irregular serial sampling. Mapping changes still require initialization.
- Mapping/signs with asymmetric rests, tendon telemetry, transferred joint
  spacing, joint scoring, bounded sweep/apply/export, and both themes pass the
  retained 0.5.0 feature checks.
- GUI Connect opens with no save prompt or automatic recording. Disconnect,
  window close and remote timeout ask Save/Discard/Later after shutdown and
  closed writers. Later and restart recovery preserve pending data.
- Real filesystem tests cover persistent journals, active-owner locking,
  unconfirmed shutdown, symlinks, earlier saved runs, interrupted discard and
  retry after a lost reply. Offline diagnostics still collect saved alignment.
- Ten simulated-drive cases: normal move/hold, post-run discard, explicit
  recording, setup/individual/mean controls, cold start, lost heartbeat, stale
  preview, camera failure, control EOF and session timeout. Both drives acknowledge
  disable and the camera writer exits in every case.
- Synthetic preview writes zero JPEGs; explicit recording respects byte/time caps.
  Slow or failing analysis leaves video responsive. Authenticated HTTP preview,
  observer lease behavior and active-service shutdown protection pass.
- Installer precheck/install/repeat install/rollback and refusal of local edits;
  rig settings, saved alignment, calibration and data remain untouched.

## Reproduce

Run with the installed dependencies from the project directory:

```bash
uv run --offline --frozen --extra live python tests/verify.py
QT_QPA_PLATFORM=offscreen uv run --offline --frozen --extra live python tests/verify_v073.py
QT_QPA_PLATFORM=offscreen uv run --offline --frozen --extra live python tests/verify_v072.py
QT_QPA_PLATFORM=offscreen uv run --offline --frozen --extra live python tests/verify_v051.py
QT_QPA_PLATFORM=offscreen uv run --offline --frozen --extra live python tests/verify_v050.py
QT_QPA_PLATFORM=offscreen uv run --offline --frozen --extra live python tests/verify_window.py
QT_QPA_PLATFORM=offscreen uv run --offline --frozen --extra live python tests/verify_v070.py --motor-source ABSOLUTE_MOTOR_REPO
```

Full driver checks use `tests/verify.py --motor-source ABSOLUTE_MOTOR_REPO
--capture-source ABSOLUTE_CAPTURE_SOURCE`. Camera regression and transport scripts
use `--capture-source` with an absolute path. They need temporary localhost socket
permission, but no hardware or external network. Test screenshots/results are
verification artifacts, not experimental recordings.

## Remaining rig check

The uploaded run records motor disable and camera exit, but omits workstation
analysis logs and alignment. Its precise display-failure trigger is unconfirmed.
Check base placement, existing mapping and one end-of-run Save/Discard choice on
the actual Windows/WSL rig. The diagnostic command collects workstation evidence
if the problem persists. No damping-identification claim is made.

## Installer revision 2 regression

Verified the actual 0.4.0 → 0.5.0 update → 0.5.1 path, legacy generated reports,
fresh 0.5.0 installation, and an already-installed first 0.5.1 package. Repeated
installation, rollback, source-edit refusal and preserved user files pass. The
original failure occurred before file replacement, so no rollback was necessary.


## 0.6.0 checks (6 October 2026)

`tests/verify_v060.py` takes absolute `--motor-source` and `--capture-source` paths.
It verifies:

- Fixed adjacent joint distances across bending and translation; missing coverage is not clamped; length matching uses corresponding physical/model links.
- Calibrated J uses actual matching MuJoCo joint anchors; the preview reference supplies no fabricated encoder data or stiffness score.
- A full-difference limit of 220 mm gives 330/110 mm endpoints at mean 220, while invalid endpoints are rejected in both coordinator and actual driver's Rig. Measured-travel checking uses the same expanded bound.
- Authenticated camera-only preview, bounded RAM frames, no motor-run directory, no recording, temporary-data cleanup, unchanged observer lease semantics, and lease expiry.
- Camera duration beyond the former 620-second cap; UI choices 10/20/30/45/60 minutes and propagation of the selected one-hour duration. Long-duration behavior was checked with an advanced test clock; no one-hour wall-clock hardware run was performed.
- Actual manual-placement, apply/save/reload, fixed-base length matching, and same-frame screenshot/J/sweep exports even after the displayed pose changes.
- Transition from setup preview to Connect closes setup capture before the motor session; duration and difference settings reach the session transport.

The existing full `verify.py` checks pass: recorded RMSEs remain 7.293832,
6.391024, 4.860399, 9.811050 px; dynamics/stale gates pass. All 10 simulated-driver
scenarios confirm both drive disables and camera-writer exit (normal, discard,
discard-then-record, setup controls, delayed camera startup, heartbeat loss,
preview loss, camera failure, EOF and timeout). The v0.5.0, v0.5.1, window,
transport and slow-analysis/recording regression suites also pass.

The combined pose PNG and inspector/window were rendered offscreen and visually
inspected. This is software verification with recorded images and synthetic
cameras/drives, not evidence that the physical robot tolerates the newly allowed
travel. Verify physical joint identities and unchanged camera geometry before
using calibrated J for a stiffness experiment. The existing XML actuator/contact
limits can still reject a simulated pose at larger physical motion; a larger
motor command envelope does not expand or refit the scientific model.

Installer verification: published full 0.5.0, incremental 0.4.0 → 0.5.0, 0.5.1 first installer, 0.5.1 revision 2, and the complete incremental path all pass check-only, installation, repeat, rollback and source-edit refusal without writes. User settings/calibration/data and arbitrary legacy generated root reports are preserved. See INSTALLER_VERIFICATION.json in the update archive.


## 0.6.1 checks (6 October 2026)

`tests/verify_v061.py` uses recorded frames, synthetic textured image joints and
offscreen Qt, with no serial devices, physical camera or SSH. It verifies:

- Exact clicks 35–45 px away from the automatic trace are retained in the
  corrected curve, named correspondence, frozen J, sweep and exported JSON.
- Known translating/bending image joints remain within 1 px in the synthetic
  sequence, with exact fixed adjacent lengths. No model pose is an input.
- Saved calibration reacquires from image patches; blank, occluded, resized or
  ambiguous images pause tracking instead of replaying old pixels.
- Alignment changes preserve the image tracker; the independent red model
  remains available when scoring has no valid manual measurement.
- Manual placement works without automatic extraction or motor feedback.
- The actual Apply/sweep/export UI keeps the corrected frozen image and J
  together, and length matching changes scale while preserving the picked base.
- Settings save/reload preserves exact calibration coordinates.

Core recorded science and dynamics, v0.5.0, v0.5.1 and v0.6.0 checks pass against
this update. The four original recorded RMSEs are unchanged. The v0.6.0 suite
includes the actual driver geometry checks, authenticated synthetic camera,
selected-duration propagation and setup-to-run transition. No transport or
motor-command behavior was changed for this correction. The ten shutdown-driver
scenarios documented for 0.6.0 were run for that release.

Tracking has not been validated on the physical rig. Synthetic tracking accuracy
is not a physical accuracy claim. Inspect the green overlay at each held pose;
fixed pixel lengths require planar motion and a fixed camera. The frozen manual
measurement is exact user placement, while subsequent image positions are
estimates and may drift or fail.

The combined pose and inspector images were rendered and visually inspected.
Installer checks cover supported published source versions, check-only, repeat
installation, backup/rollback, preserved user files and generated reports, and
refusal of edited source before writes. See INSTALLER_VERIFICATION.json.


## Version 0.8.0

Recorded/synthetic campaign checks and native Qt batch editing pass. The full
73-pose default passes through the existing motor dispatcher, Rig and DemoDrive
with an accelerated clock. Automatic acquisition/analysis and manual queuing,
shared-first fitting, invalid-equilibrium gaps, exact JPEG retention, budget
failures, draft reopening and shutdown/deletion order are exercised. Current
v0.7.3, v0.7.2, v0.7.0, v0.5.1 and main-window regressions pass. No physical
hardware was operated. See UPDATE_0_8_0.md and release_metadata/v0.8.0.

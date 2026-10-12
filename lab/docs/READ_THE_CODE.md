# Read one experiment, in this order

The purpose is to understand this sentence:

> I command cable lengths; MuJoCo receives the encoder-derived cable changes;
> I compare its predicted backbone with the physical backbone in the camera.

There are three experiment responsibilities, rather than three enormous files.
Window drawing and the existing device/scientific calculations have their own
named files. No framework, plugin registry or automatic fitting runs in the background.

## 1. Start with one calculation: actuation.targets

Read `targets()` near the top of `actuation.py`.

Inputs: mean A, full difference D, speed, and physical rest lengths.
Calculation: L1 = A + D/2; L2 = A − D/2.
Output: two requested physical cable lengths in millimetres.
Checks: finite numbers; speed 0.05–5 mm/s; full difference within the selected session limit (default 70 mm, at most the shorter configured rest);
and each cable within ±max(70, selected_difference_limit/2) mm of its rest.
`motion_limits.worker_limits()` supplies this envelope to the existing driver’s endpoint conversion and measured-travel checks. It leaves the external repository unchanged.

Example: A=220, D=20 → [230, 210]. A=220, D=−20 → [210, 230].
The underlying motor driver's coordinate is **half-difference**, so `dispatch()`
passes D/2 to it. This is the only conversion boundary.

Read `dispatch()` next. Its core commands are:

- `arm`: enable at the explicitly declared physical rest position;
- `move`: submit the two-cable request after checking readiness and fresh preview.

`setup_enable`, `jog`, `axis` and `mean` extend these operations without changing the worker owner. `reference` is handled in the session loop.

For the independent analysis mode, `independent_targets()` validates two absolute
lengths and two speeds. Each endpoint uses the same per-cable travel envelope;
the mean/difference limit does not constrain their difference. With 220 mm rests
and the default envelope, either cable may be commanded from 150 to 290 mm.
`motion_limits.worker_limits()` temporarily supplies a `SessionWorker` and
`SessionRig` that accept `independent` and validate both moves before any drive
writes. Both targets are staged, then committed back-to-back using the existing
driver. Individual speeds do not promise simultaneous arrival. The original
driver repository is unchanged, and its feedback, Stop/Hold and watchdog paths
remain responsible for execution.

A submitted command is not proof of completed motion. The subsequent encoder
feedback determines when both axes are holding. Hold and Disconnect have priority.

## 2. Follow the operating sequence: actuation.run_session

Read the `try` block, then the `finally` block before reading its helper functions.

1. Create a uniquely named session and record clock correspondence.
2. Start the Windows camera; check that camera and WSL share the session directory.
3. Connect the existing motor worker with torque disabled.
4. Repeatedly read feedback, check the connection, and dispatch user commands.
5. On exit, stop the worker, disable both drives, and record acknowledgements.
6. Stop the camera writer. Retain the run pending an explicit Save / Discard / Decide later choice after confirmed shutdown.

The `pump()` helper keeps the existing worker watchdog alive only while this
coordinator makes progress. It also checks camera freshness and the workstation
lease. The same cleanup runs for a requested disconnect and for a detected failure.

The important distinction: `Motor.tick()` updates feedback; `Motor.submit()` requests
an action. Both reuse the controller already installed with your robot.

## 3. Follow one comparison: analysis.compare

Read these functions in this order:

| Function | Question it answers |
|---|---|
| `inputs_at` | What were the two encoder-derived lengths at this image's time? |
| `controls` | What changes from physical rest should enter the XML tendon controls? |
| `advance` | Where does MuJoCo predict the backbone after integrating those inputs? |
| `project` | Where should that backbone appear in camera pixels? |
| `point_cost` | How far apart are corresponding observed and predicted points? |
| `compare` | Are the data fresh, and are both shapes settled enough to report a stiffness score? |

`advance()` repeatedly calls `mujoco.mj_step` at the XML timestep, shortening only
the last step to reach the image time. It does not solve an equilibrium at every
video frame. That is what makes the red backbone a moving mechanical prediction.

At the start of a live session the simulation needs an initial state. After the
encoder lengths are stationary, it initializes at the predicted equilibrium for the
current cable lengths. The camera shape does not set the simulated joint angles.
After a data gap, initialization is required again and the missing interval is not scored.

Changing k₂ or d₂ retains the current simulated position and velocity. The model
then responds to its new parameters. The settled comparison window restarts.

## 4. Inspect the cost with actual numbers

Run this without hardware:

```bash
uv run --offline --frozen python science_walkthrough.py
```

It prints one observed point, one predicted point, their x/y residuals,
the squared distance, the mean over points, and the RMSE. It then writes a
fixed-alignment stiffness sweep. Open the script and follow its numbered comments.
It uses the original offline point selection, which differs from the live selection.

The live **Freeze settled pose + sweep k₂** button uses `analysis.scan()`:
freeze the measured points and cable inputs, try a visible list of k₂ values,
find an equilibrium for each, and plot their errors. It selects the lowest sampled k₂ in the window. `apply_sweep()` explicitly requests that value; the worker acknowledges it before the Applied label changes. `export_sweep()` writes PNG/CSV/JSON only when requested.

## 5. Finally read the connections

`connection.WorkstationSession.connect()` starts the Windows camera service through
SSH and then starts `actuation.py` inside WSL using its existing Python environment.
Commands are small JSON lines through SSH stdin; status messages return on stdout.
There is no public motor-control web endpoint.

`connection.CameraTunnel` is the existing WSL-to-Windows camera route, using the
fixed ssh-agent socket. The key stays encrypted on disk. `setup.py unlock` loads
it for up to eight hours; WSL shutdown or key expiry requires another unlock.

A timer in the window calls `touch()`. The connection thread sends heartbeats
only while that timer remains responsive. The laptop closes the session if those
heartbeats stop for three seconds. The existing motor worker also retains its
own watchdog. These are software guards, not hard real-time guarantees.

## 6. Read the window only when you want to change its appearance

`main.py` loads the settings and creates `Window`.
`interface.Window.command()` forwards button actions; `apply_parameters()` forwards
simulation edits. `Preview.run()` obtains the latest frame and its motor samples
and calls `analysis.compare()` without waiting for image extraction.
`live_vision.VisionWorker` processes at most one active and one pending image;
new arrivals replace the pending image. `Comparator.attach_measurement()` joins
the completed image measurement to the model geometry saved with that same
frame, never to the current model pose. `CameraView.paintEvent()` draws each
overlay on its corresponding image.

Preview and vision run outside the GUI thread, so slow extraction does not hold
up camera delivery, MuJoCo updates or Stop/Hold. A separate worker performs a
frozen sweep. Changing the measurement mode or comparison settings invalidates
older pending measurements. The viewer does not build a backlog of old video.

You can understand the experiment without first studying Qt drawing or Modbus
register transactions. Both remain inspectable when you need those details.

## 7. Follow the new calibration controls

- `Comparator.controls`: subtract each physical rest, reorder channels, apply displacement signs, then add fixed model offsets.
- `DisplaySetup.values`: convert a physical base pixel, image rotation and scale into one fixed camera translation.
- `Comparator.tendon_telemetry`: read the MuJoCo control target and geometric tendon length separately.
- `placement_profile`: project verified physical clicks onto the measured curve and retain tip-relative arc distances. No red model point influences this calculation.
- `Comparator.observation`: optionally replace default arc samples with verified joint/centre identities.
- `run_storage.decide_remote`: after the run, verify the owner lock, session identity and shutdown before an explicitly requested discard.
- `run_storage.review_entries`: find pending runs after a crash without changing their data.
- `Comparator.set_setup`: change image projection without resetting physical state; only cable mapping changes require initialization.
- `Comparator.score_geometry`: pause the score if measurement/correspondence fails while keeping the independent model visible.

## Camera setup, rigid joints and frozen pose exports in 0.6.0

`CameraSetupSession` launches only the Windows service. Authenticated preview routes start a temporary, memory-buffered capture; a GUI lease releases the camera if the workstation disappears. Connect closes this capture before creating a motor run. `Comparator.compare_setup()` projects the XML reference pose and deliberately returns no encoder data, J or sweep.

The legacy `landmarks.placement_profile(..., rigid=True)` and `rigid_points()` retain compatibility with old saved profiles. Since 0.6.1, the inspector uses `joint_tracking.manual_profile()` instead: exact frozen coordinates define a new polyline and adjacent chord lengths, independently of the automatic curve. `ManualJointTracker` uses forward/backward optical flow, image patch consistency and a fixed-length fit on subsequent frames. It has no model-pose or stiffness input. Small saved image patches permit bounded, correlation-checked reacquisition when a camera session restarts; missing or ambiguous evidence pauses J and requires a fresh capture. The image constraints assume planar motion and a fixed camera.

`manual_observation()` supplies the corrected curve and named points to both the live `Comparator` and frozen sweep in image-estimation mode. A calibration ID prevents old automatic measurements from being reused with a new manual profile. Full calibration, including patch references, is logged only when the comparison revision changes, rather than on every frame. Version 0.7.0 requires deployment of its paired runtime for independent-speed motor commands.

`scale_from_profile()` divides the sum of physical link lengths in pixels by the sum of the corresponding model link lengths in metres; matching scale preserves the picked base.

The inspector’s explicit frozen-sweep action uses the captured image, captured encoder inputs and new joint placement together. `pose_capture.export_pose()` checks frame identity and observation consistency before exporting the combined annotated image and sweep. The live feed can move on without changing the saved pose.

## Manual frozen analysis in 0.7.0

`manual_analysis.prepare_capture()` copies one camera packet and checks the
captured measured cable inputs. An independent `Comparator` calculates their
equilibrium at the acknowledged k₂; the live simulation is not reset. No image
extraction, image tracking or visual-settling detector is called. Missing,
unreferenced, stale, moving or unbracketed motor inputs produce an explicit
capture error; the user can still view and annotate the image.

`PointInspector` in manual mode starts with no inferred physical points. The
user selects landmark identities and clicks at least four visible physical
points, then confirms that the robot was stationary in that frame.
`complete_capture()` builds the observation from those exact coordinates and
the corresponding model anchors. It does not require contiguous joints, a
visible tip, image textures or proximity to an automatic curve. The green
polyline is only a display joining the chosen points; J uses named point pairs.
`analysis.scan()` and `pose_capture.export_pose()` retain that same frozen image,
placement and measured cable input even while the live camera moves on.

`Window.tick()` and the inspector show the current readiness reason beside their
buttons. A manual frozen capture remains valid while the user places points;
it is not invalidated just because the live frame has aged.


## Independent views in 0.7.1

`Window.toggle_camera()` owns setup-camera start/stop before Connect. During a
run it changes visibility only, preserving acquisition and its freshness checks.
`CameraView.set_visibility()` independently gates the camera/physical marks and
red model/base marks; correspondence lines require both views. Switching the
backbone has no camera/session side effects. A separately cached local reference
is rendered without a synthetic image or encoder packet when no camera is open.
It cannot enter the scoring or capture path. `Window.tick()` preserves display
choices across setup-to-run transition and restores the local reference after
camera shutdown or failure. This frontend change keeps runtime protocol 0.7.0.


## Stable GUI interaction in 0.7.2

`Window` uses a horizontal splitter, a fixed session bar and compact task tabs.
The camera and jog controls have no scroll-area parent. `motion_enabled()` moves
focus to the fixed Stop control before disabling an active jog, avoiding Qt's
focus-driven auto-scroll. `StatusLabel` reserves a fixed number of lines and
keeps complete status text in a tooltip so updates do not resize the controls.

`PointInspector.redraw()` retains the current x/y limits while redrawing points.
Only first/new capture, Fit view or a changed full-frame fit option initializes
the viewport and navigation stack. Existing toolbar history survives edits.
`begin_placement()` exits the active navigation tool without changing its limits.
`rebuild()` restores the point-table scrollbar values and uses stable column
widths instead of repeatedly resizing them to changing coordinate strings.


## Optional profile review and explicit recovery in 0.7.3

`SpiRobModel.set_stiffness_profile(r, b)` updates only the flexible-joint
stiffness weights. Default values retain the original XML weights; damping and
joint 1 are preserved. `Comparator.set_parameters()` acknowledges r/b together
with k₂/d₂ while retaining q, qdot and time. `predict()` uses the supplied profile
for every independent equilibrium, including both 1D and surface calculations.

`stiffness_sweep.sample_surface()` has no UI or hardware access. It copies the
completed 1D sweep's frozen parameters/observation, checks the model hash and
varies k₂ plus exactly one constant. Invalid costs remain null with errors;
cancellation is checked between solves. `surface_review.SurfaceReview` handles
progress, interactive 3D plotting and explicit PNG/CSV/JSON export. Its minimum
button stages fields; the existing Apply action sends the virtual-model request.

`Window` no longer schedules `recover_runs()` on startup. That function runs only
from the explicit review button. Current-run shutdown still calls `begin_review`
after remote shutdown and local writer closure. No retention or deletion guard
in `run_storage.py` is bypassed or weakened.


## Campaigns

`campaign.py` expands waypoint/segment rows and runs the acknowledgement → target
arrival → encoder stability/dwell → committed capture sequence. It sends the
existing guarded commands and never enables hardware. `campaign_storage.py`
retains raw JPEGs and frozen input/model provenance within the session folder.
`campaign_analysis.py` measures saved images, calculates a shared k₂ baseline,
then optional per-pose minima with fixed nuisance parameters. `campaign_window.py`
coordinates Qt/background work and writer shutdown; `campaign_review.py` extends
the existing point editor with drafts, reviewed image-only suggestions and
Save & next. See tests/verify_v080.py, verify_campaign_flow.py and
verify_campaign_driver.py. Runtime protocol remains 0.7.0.

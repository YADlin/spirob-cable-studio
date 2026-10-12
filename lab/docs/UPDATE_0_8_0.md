# SpiRob Lab 0.8.0 — autonomous campaigns and batch joint review

This workstation update uses the working camera/SSH/WSL arrangement and the
existing motor coordinator, limits and watchdog. Runtime protocol stays 0.7.0;
a deployed 0.7.x installation does not need laptop redeployment.

## First campaign

1. Install while SpiRob Lab is disconnected and closed. Launch normally.
2. Check the camera, independent MuJoCo display, base alignment, physical scale
   and cable mapping. These are frozen for the campaign, together with r, b,
   damping and the cable offsets. Keep the camera fixed during acquisition.
3. In **Session / data**, choose **30 min** before Connect. Connect, establish
   physical rest, enable and wait for HOLDING, as in the working manual workflow.
4. Open **Campaign → Plan / run campaign**. The editable default is already
   present. Choose **Automatic shared k₂ after journey** or **Collect now; place
   joints afterward**. Manual collection also selects the live manual-points
   mode, so image extraction is off during that journey.
5. Click **Preflight** to inspect every expanded cable endpoint and speed, capture
   count, estimated duration and storage. Then click **Start campaign**. Start
   does not enable motors or move until the existing referenced HOLDING gates
   are satisfied. It refuses a plan that exceeds the remaining session budget.
6. The journey runs without point-placement interruptions. **Stop campaign +
   hold**, the main **Stop / Hold**, closing the campaign window, disconnect,
   stale camera/feedback, command rejection or timeout stop the sequence.
   Completed poses remain retained. Stop retains torque; Disconnect releases it.
   There is no automatic motion retry or motion resumption from a saved file.
7. After completion, automatic mode extracts/tracks physical image features from
   the saved frames and calculates the shared-k₂ result. Manual mode waits for
   your batch review. No current green centreline or live model-settled flag is
   required to acquire a pose.
8. Disconnect and **Save** at the normal end-of-run prompt to retain the campaign.
   **Decide later** retains it pending a decision; **Discard** includes the
   campaign images, drafts and results. Closing or a timeout follows the same
   shutdown-first sequence. An application/process crash leaves completed files
   pending; it cannot display a prompt while the process is gone.

## Default journey

For 220 mm rests, the default contains **73 captures**:

| Part | Commands | Captures |
|---|---|---:|
| Initial reference | D = 0, mean = 220 mm | 1 |
| Difference map | D: 0 → +40 → −40 → +40 → 0 mm; 15 intervals per segment | 60 |
| Cable 1 | 200 → 240 → 220 mm; cable 2 holds | 3 |
| Cable 2 | 200 → 240 → 220 mm; cable 1 holds | 3 |
| Common lengths | (210, 210), (230, 230) mm | 2 |
| Unequal lengths/speeds | (205, 230) at (1, 0.5) mm/s; (230, 205) at (0.5, 1) mm/s | 2 |
| Final references | (220, 220), repeated | 2 |

Other rows use 1 mm/s. The default extra dwell is 2 s in addition to a 2 s
encoder-stability window. Its planning estimate is 13.4 minutes with an
18-minute reservation; select 30 minutes to leave setup time. This estimate is
not a guarantee of hardware timing. The default deliberately exercises moderate
travel, rather than assuming the entire physical motion envelope is verified.
The default generator centres the lengths on the current known rests; before
connection it uses 220 mm when rest lengths are unavailable. Reload **Default
73 poses** after connection if your rests differ, then inspect Preflight.

## Editing waypoints and maps

| Motion column | x | y | Speeds |
|---|---|---|---|
| Difference | Full D = L₁ − L₂ | Mean = (L₁ + L₂)/2 | Speed 1 applies to both; Speed 2 is ignored |
| Both cables | L₁ absolute length | L₂ absolute length | Separate Speed 1 and Speed 2 |
| Cable 1 | L₁ absolute length | Ignored; cable 2 keeps its previous target | Speed 1 |
| Cable 2 | Ignored; cable 1 keeps its previous target | L₂ absolute length | Speed 2 |

All lengths are mm and speeds are mm/s (0.05–5). A row with **Intervals = 1** is a
single waypoint. A later row with N intervals divides the segment from the
previous row into N equally spaced cable-length steps, capturing every arrival.
The first row has one capture. Thus P1 → P2 → P3 with 15 intervals per segment
produces 31 images; the shared endpoint is not captured twice unless you add an
explicit repeat row. Each leg uses the destination row's speeds. Different
motor speeds can reach their targets at different times; both must finish.
Use Save/Load plan for reusable JSON plans. Maximum 200 rows / 500 expanded poses.

## What qualifies a capture

Each command must receive its own acknowledgement. Both motors must be enabled,
referenced and HOLDING, with no pending command. The encoder samples must
bracket the camera timestamp, have no gap above the configured input limit,
remain within 0.5 mm of the requested targets and vary by no more than 0.15 mm
over the full stability-plus-dwell window. The window starts after the command.
The last JPEG is never substituted for a missing fresh frame. A stationary
encoder at the wrong target is not accepted. A failed disk write aborts and
holds before any next move.

Encoder stability plus dwell is a quasi-static acquisition criterion; it does
not prove the body has stopped vibrating or creeping. Increase dwell when
needed and inspect the image during review. Manual annotation requires explicit
stationary-image confirmation. Automatic results are labelled as image
measurements at an encoder hold, and do not silently claim visual confirmation.

## Mark all the images in one sitting

Open **Captured poses / batch review → Review next unfinished**, also available
after disconnect or reopening a campaign.json inside the saved run folder.
There is no need to reconnect hardware. The thumbnail list shows Captured,
Needs points, Ready, Analysed or Excluded, with the reason and fitted residuals.

- Select a joint and click its physical location. By default the next joint is
  selected after each click. Zoom, pan and numeric edits retain the current view.
- Every included joint/tip (or element centre) must be placed. Uncheck obscured
  points explicitly; at least four distinct physical points are required.
- **Suggest from previous annotated image** uses image optical flow with
  forward/backward and local texture-agreement checks. It fills unplaced points only. Suggestions can be
  wrong, especially for large moves, repeated textures or occlusions: review
  each one. No MuJoCo coordinate is ever used as a physical-point suggestion.
- Drafts save after every edit. Close and reopen an unfinished pose to resume.
  Confirm that the photograph was stationary, then **Save & next pose**.
- An invalid equilibrium at the current trial k₂ does not block batch marking.
  The unavailable prediction is hidden and explained. The subsequent sweep
  evaluates each candidate independently.

For accuracy, use the same identifiable joints across poses where visible. The
suggestion tool speeds up placement; it does not replace your final review.
Repeated reference poses help distinguish actual response changes from marking
variation, drift and motion history.

## Shared baseline first, individual fits second

**1. Fit shared k₂** evaluates one common logarithmic grid for all included poses:

`J_p(k₂) = mean over physical points of squared Euclidean pixel error`

`J_shared(k₂) = mean over included poses of J_p(k₂)`

Every pose has equal weight, including repeated references. A shared grid value
is eligible only if its equilibrium and geometry are valid for every included
pose. A failed pose is never dropped just for selected k₂ values. An all-invalid
attempt records the per-pose errors in fit_attempt.json. Failed automatic image
measurements remain in the manual queue. A fit with missing captures, missing
measurements or user exclusions is visibly **INCOMPLETE / SUBSET**, with coverage
counts and exact omissions in its JSON. It is not presented as a successful
all-pose fit.

Then **2. Fit each pose** reports each pose's minimum from those same frozen
measurements and grid. It never silently changes camera alignment, scale, cable
offsets, r or b to improve an individual result. The normal single-pose J/k₂
and optional 3D profile tools remain available in the main window.

Results show the shared J–k₂ curve and per-pose k₂ against measured cable
difference, measured distal lateral offset or acquisition order. The lateral
offset uses the most distal *measured* landmark relative to the calibrated
neutral axis; when that landmark differs between poses, it is not a like-for-like
tip-deflection measure. Its identity is exported. For independent cable control,
retain both lengths and their mean as covariates rather than treating D alone
as the full state. The fit is the lowest sampled value, not a guaranteed global
optimum. Grid-boundary minima are flagged. A lower individual residual is
expected with more fitted parameters and alone does not establish variable
material stiffness. Use repeat/reverse poses and later held-out validation
before building a constitutive k₂(deflection) law. No arbitrary shared-fit
pass/fail threshold or automatic simulation update is imposed.

## Storage and recovery

Each capture stores exactly **one original full-resolution JPEG**, the bounded
encoder history around it, image/metadata hashes, timestamp, camera age, cable
command and speeds. The campaign freezes its model files, calibration, profile
constants and mapping. Images are not duplicated as base64 in JSON. There is no
continuous video requirement. Estimated JPEG storage is shown in Preflight;
images have a hard **256 MiB per-campaign cap** and an **8 MiB per-frame cap**.
Metadata, the model snapshot and small plots are additional. At most 500 frames
can be requested. A low-disk or cap failure preserves prior poses and stops.

Campaign folders live under:

`logs/runs/session_<id>/campaigns/campaign_<timestamp>_<id>/`

The normal session Save/Discard decision includes that directory. Writers are
stopped/joined before discard is possible. Open data folder exposes the files:

- campaign.json: plan, frozen configuration, capture index, annotations and fit.
- poses/0001/frame.jpg and snapshot.json: original image and synchronized inputs.
- poses/0001/draft.json: unfinished annotation, saved independently.
- fit_results.json: shared curve, every per-pose sweep, errors and provenance.
- fit_results.csv: per-pose shared/individual results, cable inputs and geometry.
- fit_results.png: overview plot.

Changing annotations or exclusions invalidates the current result. The last JSON
is retained as fit_results_previous.json; stale current CSV/PNG are removed.
An image/snapshot committed just before an interrupted index write is recovered
for manual review when its session and held encoder inputs validate.
Unfinished campaigns reopen for review and subset analysis; physical movement
never resumes automatically. A future journey starts as a new campaign.

## Verification

The 73-pose default completed through the original motor Rig/DemoDrive and real
command dispatcher with an accelerated clock. Additional checks cover mixed
commands and interpolation, acknowledgement identity, correct-target/stability
checks, stale data, rejection, acknowledgement timeout, Stop, failed storage,
shared/individual known-parameter recovery, invalid equilibria, interrupted
analysis, raw image integrity, offline drafts, native zoom/click/save-next,
automatic completion versus manual queuing, and shutdown/deletion ordering.
Existing v0.7.3, v0.7.2, v0.7.0, window and v0.5.1 regressions were rerun.
No physical motors or camera were operated while producing this update.

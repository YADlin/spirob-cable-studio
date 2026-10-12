# SpiRob Lab 0.7.0 — independent lengths and two analysis modes

This release separates pose analysis from automatic vision. It also makes
absolute cable-length control available as an analysis motion mode, alongside
mean/full-difference control and the separate rest-setting jog controls.

## Independent cable lengths

After Connect, enable the motors and declare the physical rest as usual. Under
**Analysis motion control**, select **Independent cable lengths**.

- Enter **Target length [mm]** for each cable. At 220/220 mm rest with the
  default travel setting, each target is constrained to **150–290 mm**.
- Enter **Speed [mm/s]**. **Same speed for both motors** starts checked: cable
  1's speed is copied to cable 2. Uncheck it to set the speeds independently.
  Each speed is 0.05–5 mm/s.
- **Move cable 1 only** or **Move cable 2 only** moves that cable while the other
  holds. **Move both to selected lengths** submits both targets and speeds in
  one command. Both endpoints/speeds are checked before either target is written.
- Stop / Hold, feedback checks, travel bounds, reference requirements, command
  expiry and the disconnect/watchdog shutdown sequence remain active.

Independent mode constrains each cable to the displayed absolute range. Thus
150/290 mm is allowed at the default per-cable range even though its full
difference is 140 mm. The separate **Difference-mode limit |L₁−L₂|** constrains
mean/difference commands. Its selected setting also defines the shared travel
envelope `rest ± max(70, difference_limit/2)` mm. With a selected difference
limit of 220 mm, the per-cable envelope becomes 110–330 mm for 220 mm rests.
These are configured software limits, not a measured mechanical rating.

The motor protocol commits the two targets in sequence; this does not promise
perfectly simultaneous starts or arrivals. Each selected speed remains an
individual speed request. MuJoCo receives the measured encoder-derived lengths,
with the established mapping, rest subtraction and fixed cable offsets. It
does not assume that a commanded endpoint has already been reached.

**Show setup jog / rest controls** remains the separate panel for preparing and
declaring physical rest. Its jog speed is independent of analysis target speeds.

## Manual points — no image processing

1. Select **Measurement mode → Manual points — no image processing**. Camera
   delivery and the MuJoCo backbone continue; no segmentation, centreline fitting
   or optical-flow tracking is requested in this mode.
2. Move using either cable-control mode. Hold the physical robot still, then
   click **Capture pose + place points for k₂**. A frozen frame is paired with
   its measured cable inputs. A separate equilibrium model is calculated at
   those captured inputs; the live simulation is not reset.
3. In the editor, choose **Joints + tip** or **Element centres**. Select each row
   and click the corresponding physical point on the image. Place at least four
   identifiable points, preferably spread over the visible body. You may place
   more. Unplaced points are omitted; there is no requirement to fill every
   joint through the tip, no curve-distance threshold and no texture test.
4. Check **The physical SpiRob was stationary in this captured frame**, then
   click **Sweep k₂ using these manual points**. This confirmation replaces
   automatic visual settling detection. It does not override invalid encoder
   data or a moving cable capture.
5. After completion, use **Save pose screenshot + J + k₂ sweep…** in the main
   window. The image, manually measured points, red equilibrium model,
   correspondence lines, J and sweep all use that one frozen capture. The live
   camera may move on without altering it. **Capture current frame** starts a
   new placement for the next pose.

The green line in a manual export joins selected points for display. It is not
an extracted centreline; J uses the exact named points only. The positions are
never tracked or replayed onto later frames. Four points are a software minimum,
not a guarantee of parameter identifiability or placement accuracy. A single
pose still cannot identify damping.

The capture button is available without a valid automatic curve or an
automatically settled visual score. An explanation is always shown near it.
To perform a valid static sweep, the captured inputs must be fresh, referenced,
time-bracketed encoder samples held for two seconds. The user must also identify
at least four points and confirm a stationary physical pose. Setup preview can
capture an image, but cannot supply an encoder-based J before Connect/reference.
If a requirement is missing, the editor states it and offers a new capture.
Time spent placing points does not invalidate an otherwise valid frozen capture.

## Image estimation — live green backbone

This mode retains automatic extraction and the previously applied image joint
calibration/tracking. A separate worker processes images with one waiting frame,
so expensive extraction does not block the live camera reader or MuJoCo stepping.
Old queued images are replaced rather than accumulating delay.

A vision result is paired with the model geometry captured for that exact image
time. It is never scored against the newer model state or drawn over a different
image. A valid measured pair stays visible briefly while the next measurement
is calculated. The readiness text shows its frame number and age. The latest
measured pair can be frozen for analysis for up to five seconds; this is a
historical paired capture, not a stale measurement claimed to describe new video.

The readiness line explains missing extraction, unstable image points, moving
encoder inputs, model settling, or waiting for changed settings. If image
features cannot be measured reliably, switch to Manual points mode. Automatic
vision can still fail on occlusion, ambiguous features, an unsuitable fixed ROI,
or camera/lighting changes; asynchronous processing cannot repair those pixels.

When encoder-based prediction is unavailable, the window labels the visible
backbone **Reference backbone only**. It is not presented as a valid live
prediction and is not used for a scored sweep. The separate camera setup preview
remains available before Connect without starting a motor session timer.

## Install and verify

Close SpiRob Lab, install the supplied update, then run:

```bash
python3 setup.py deploy
python3 setup.py check
bash start.sh
```

Deployment is needed because absolute paired commands now support two speeds.
The camera/runtime version check is updated to 0.7.0. Published 0.5.0, both
0.5.1 updates, 0.6.0 and 0.6.1 are accepted by the cumulative installer. Local
settings, XML/calibration, recordings and generated root checksum reports are
preserved; changed source is checked before any replacement and a rollback
backup is created. The established end-of-run Save/Discard/Later and interrupted
run recovery behavior remains in effect.

Verification uses recorded images, offscreen Qt, synthetic camera feeds and the
actual controller with simulated drives. Tests include independent endpoints and
speeds, both-target validation, Stop/Hold and all 11 shutdown/session scenarios;
manual sweep/export with image processing disabled; delayed-vision/model frame
pairing; and recovery of automatic settled scoring under slow vision. No physical
hardware was operated. Verify the new controls and manual point identities on
the rig before collecting accepted experiment results.

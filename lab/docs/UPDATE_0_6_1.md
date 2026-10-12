# SpiRob Lab 0.6.1 — manual curve correction and length matching

The 15 px rejection assumed the automatic centreline was correct. That was the
wrong assumption for correcting extraction errors. The inspector now accepts
your exact physical coordinates without projecting them onto that curve.

## Correct the physical backbone

1. Start **Camera + MuJoCo setup preview** before Connect, or hold a connected
   pose. Open **Inspect joint / element points…**. A green automatic curve is
   no longer required to open the inspector.
2. Select **Joints + tip**. Select a joint row, enable **Click image to place
   selected physical point**, and click the physical hinge centre. Verify all
   visible joints through the tip, particularly joints 2 and 3. Unavailable
   proximal joints can remain unplaced. Use the image features, not the red
   model, to identify each joint. Table order defines joint identity.
3. Leave **Keep joint-to-joint distances fixed** and **Also use verified points
   for J / k₂ sweep** checked. Click **Apply joints to live view + J**, or
   **Apply joints + save setup for next run**. The solid green polyline joins
   your points exactly. The former image trace is only a grey dashed reference
   in the inspector. There is no distance-to-automatic-curve rejection.
4. On subsequent frames, image flow tracks these joints. The rigid option
   constrains adjacent distances to the lengths measured from your clicks.
   No red model geometry or stiffness value determines the tracked green points.
   If a point is occluded, lacks image texture, leaves the image, or no longer
   satisfies the image/length checks, J pauses. **Capture current frame**, place
   the visible joints and Apply again. It will not silently reuse old coordinates
   or fall back to the incorrect automatic curve.

Keep the camera fixed and motion in the calibrated plane. Image tracking can
drift or fail; inspect the overlay at each held pose before accepting J. The
frozen manual coordinates are exact user measurements, but later image tracks
are estimates. Element centres can also be placed and tracked; their distances
are not constrained as rigid links.

Saving setup retains small 41×41 grayscale image references around the points,
as well as the point coordinates and link lengths. A later session searches for
unambiguous matches near that calibration. If the pose/camera changed too much,
or the features are ambiguous, capture and apply a fresh placement. Old saved
0.6.0 profiles keep their previous behavior until you reapply joints with 0.6.1.

## Match MuJoCo backbone length to the physical robot

Use an unobstructed, resting, nearly straight pose with a fixed camera.

1. Open **Alignment / cable mapping…**. **Pick physical base in video** sets
   j_001. Adjust **Image rotation [degrees]** to align the resting direction.
2. After applying the physical joints, click **Match model length to calibrated
   physical joints**. It divides the sum of clicked adjacent link lengths in
   pixels by the sum of those same model link lengths in metres.
3. Click **Apply + save setup**. The selected base stays fixed while scale
   changes. Keep this calibration fixed while comparing deformations and k.

You can also directly edit **Image scale [px/m]**: increase it to lengthen the
red backbone and decrease it to shorten it. A 10% scale increase produces a 10%
longer overlay. For matching corresponding lengths,
`new scale = current scale × physical pixel length / current model pixel length`.
Measure summed joint-to-joint lengths, not the end-to-end chord of a bent robot.

This is camera projection calibration. It does not change XML segment lengths,
cable rests, actuation limits or stiffness. Uniform image scaling cannot repair
incorrect relative segment dimensions, perspective or out-of-plane bending.

## Frozen pose, J and sweep

For a settled connected capture, **Apply joints + sweep this frozen pose** uses
those exact corrected coordinates, that image, and its captured encoder inputs.
Then use **Save pose screenshot + J + k₂ sweep…** in the main window. The green
curve, joint table, connecting lines, J and k sweep share that measurement.
Setup preview has no encoder-based J; after Connect, enable/reference and wait
for a valid settled comparison. If initial automatic extraction failed, apply
the joints first and wait for tracking to produce a settled comparison.

## Installation and checks

Close SpiRob Lab and install the provided update using `install.py`, then run
`python3 setup.py deploy`, `python3 setup.py check`, and `bash start.sh`.
Published 0.5.0, both 0.5.1 packages and 0.6.0 are supported. User settings,
calibration, model and recorded data are preserved. Source checks run before
replacement; a rollback backup is created. Mutable root-level release reports
are not installation prerequisites.

The new regression test is `tests/verify_v061.py`. It checks >15 px corrections,
exact frozen J/sweep/export, image-only rigid tracking, saved-image reacquisition,
tracking loss, placement with no automatic curve, and fixed-base scale matching.
Existing recorded science, dynamics, 0.5.0, 0.5.1 and 0.6.0 checks also pass.
No physical hardware was operated to prepare this update.

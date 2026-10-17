# SpiRob Lab 0.9.1 — mixed campaigns, difference sweeps and return to start

This update builds on 0.9.0 in `spirob-cable-studio/lab`. It changes workstation campaign planning and acquisition. Your model, position PD gains, cable references, motion limits, camera settings, calibration and existing runs are preserved. An already deployed 0.9.0 runtime does not need redeployment.

## Difference sweep for stiffness fitting

Open **Campaign → Plan / preflight → Difference sweep…**. Choose the first and second full-difference extremes, fixed mean length, speed and number of intervals. The draft has two rows: the first captures the starting extreme, and the second interpolates to the other extreme.

For example, `D = −40 → +40 mm`, mean `227.353 mm`, speed `1 mm/s`, and **15 intervals** gives **16 measurement poses including both extremes**, then one return capture. The robot also travels from its initial held pose to the first extreme. Swap the extremes to reverse direction. Edit or append rows for repeat/reverse sweeps.

The convention stays `D = L₁ − L₂`, `L₁ = mean + D/2`, `L₂ = mean − D/2`. Every intermediate endpoint is checked against the configured limits. Fixed mean and an ordered sweep help separate deflection dependence from changes in common cable length; forward/reverse repeats can reveal history dependence. Shared k₂ fitting, then separate per-pose fitting, is unchanged.

## Mixed random shape comparisons

Choose **Mixed / random… → Mixed**. This is the new generator default. Its measurement rows combine:

| Mode | Command |
|---|---|
| Difference | A random full D at the selected fixed mean; equal speed for both cables |
| Cable 1 | A random cable 1 target; cable 2 stays at its preceding target |
| Cable 2 | A random cable 2 target; cable 1 stays at its preceding target |
| Both cables | Independent random length targets, with linked or separate speeds |

Set both cable ranges, the difference range and mean, speed range, measurement count, and seed. The same inputs and seed generate the same rows. Mixed requires **4–200 measurement poses**. It begins with a both-cable target inside the requested ranges; each complete group of four contains all four modes in a varied order. Targets and speeds are randomized. Difference endpoints must fit both of the selected cable ranges as well as the session limits; incompatible inputs show an explanation instead of silently widening a range.

The default remains 12 measurement poses, ±20 mm around each reference, D from −40 to +40 mm, speeds 1–4 mm/s, linked speeds and seed 20261013. **Both cables** and **Difference** are still available as pure random modes. The extra return capture is not counted in the generator's measurement count.

## Return at normal completion

Every newly expanded campaign includes a final **Return to campaign start** move using both cables. It targets the initial cable lengths recorded for the previewed journey, rather than a hard-coded 220 mm or the first waypoint. Set **Return [mm/s]** in the plan controls; the default is 1 mm/s, limited by the current speed ceiling.

When connected, preview reads the measured cable lengths. Before connection, it uses the configured reference lengths. At Start, the existing starting-pose check requires measured lengths to agree with the approved initial lengths within 0.5 mm; otherwise preview again. The actual start feedback is also recorded in the campaign. To finish at physical rest, start and preview from that rest pose.

The return is included in Preflight, the MuJoCo animation, the approved hardware plan, the time reservation and the image budget. Completion and automatic shared-k₂ analysis happen only after its command is acknowledged, both encoders reach the target within the existing tolerance, their readings remain stable through settling/dwell, and the return image is saved. Changing the return speed or settling settings invalidates preview approval.

**STOP/Hold, a fault, lost camera/motor feedback, disconnect, session timeout, or a failed capture abort the campaign without any automatic return or retry.** A return that has already started is stopped by the same STOP button. A stopped or interrupted campaign never resumes on reopening.

Returning to the same cable lengths does not guarantee the same physical shape: friction, slack and hysteresis can leave a different pose. The return photograph is retained for comparison. It is **excluded from k₂ fitting by default** to avoid adding an extra reference sample and manual annotation task; use **Exclude / include selected** to include it if desired. The 500-capture and 256 MiB image budgets include this final photograph. Preview animations are not saved as videos.

## Run workflow

1. Generate or edit a draft and set dwell and return speed.
2. Inspect **Preflight**, including its final return and time reservation.
3. **Simulate / review → Approve this simulated plan**.
4. Connect, enable/reference, wait for fresh held feedback, then **Start approved campaign**. If already connected, keep the same starting pose used for preview.
5. On normal completion, the robot holds its starting lengths. Review photographs and fit shared k₂, then individual k₂. Save/discard remains an end-of-session decision.

Existing saved plans loaded for a new acquisition also receive the final return. Older captured campaigns remain readable and are never replayed automatically.

## Install on the workstation

Disconnect and close the running application. Save `spirob-lab-v0.9.1-update.zip` in Downloads, then:

```bash
unzip -n "$HOME/Downloads/spirob-lab-v0.9.1-update.zip" -d "$HOME/Downloads" &&
python3 "$HOME/Downloads/spirob-lab-v0.9.1-update/install.py" \
  --project "$HOME/Two_Cable_Motor_Setup/spirob-cable-studio/lab" &&
cd "$HOME/Two_Cable_Motor_Setup/spirob-cable-studio/lab" &&
bash start.sh
```

The installer verifies 0.9.0 source and payload hashes before writing, backs up replaced files, and prints a rollback command. Use `--dry-run` to check compatibility without writing. Unexpected edits to files being updated are reported without changes; mutable configuration and data are not update targets. A repeat install is harmless. Rollback checks for subsequent edits and preserves run data. The ZIP changes your local checkout; it does not push a Git commit.

## Verification

`tests/verify_v091.py` exercises deterministic mixed generation, mode coverage, endpoint/speed bounds, single-cable isolation, sweeps in both directions, return timing and capture ordering, STOP/fault/timeout paths, preview approval, real MuJoCo integration, retained-image recovery, and the offscreen GUI. The prior campaign/science and v0.9.0 model/demo-driver checks are also run. Results are recorded in `release_metadata/v0.9.1/verification.json`.

These checks use simulation and synthetic or recorded feedback. No physical motors, SSH session, or live camera are operated by them.

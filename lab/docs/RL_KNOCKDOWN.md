# RL egg knockdown — operator guide (contract v3)

**What this adds.** An **RL** tab that runs a policy trained in `spirob-mjlab`
(`Mjlab-SpiRob-EggKnockdown-Camera`, contract v3) on the rig. The policy sees only the
overhead camera (egg position, visible or not) and the motor encoders (cable
difference D). It commands **only D**; the cable mean stays at 227.35 mm. Each
decision is one ordinary `independent` move of at most 8 mm in D, through the
existing motor owner and its limits. Workstation-only change: **no WSL redeploy**.

```
camera ─► black egg detector ─► egg (u,v), visible ┐
camera ─► white cup detector ─► empty-cup check     │ (success rule 2)
encoders ─► D = payout0 − payout1 ─────────────────┼─► 7 numbers ─► policy ─► new D ─► L0 = 227.35 + D/2
previous action ───────────────────────────────────┘                               L1 = 227.35 − D/2
```

**Speed.** Set **Common cable speed** between 5 and 20 mm/s, within the session's
limits. Both motors use that value for every move, including small corrections.
D changes under 0.5 mm are skipped. The existing move → HOLD → settled-encoder
sequence and 1.2 s minimum decision interval remain in effect.

V2 exports are intentionally incompatible with this v3 contract. Evaluate the
checkpoint with rendered camera observations and the new speed rule before
exporting. The original v2 branch remains available for its original bundles.

## Before the first session (once)

1. **Robot model.** `lab/model/spirob_robot.xml` must be the knockdown model
   (`spirob_robot_k2_2.148_20261024_0318.xml`, renamed). Check:
   `sha256sum model/spirob_robot.xml` → must start with `fe7d13cd476a`.
   The Lab refuses a policy exported against a different file.
2. **Print the egg and pedestal** from the drawing (`spirob-mjlab/docs/knockdown/egg_pedestal.png`):
   egg in **black** PLA (matte if possible), pedestal in **white** PLA. Pedestal cup top 46 mm above the
   table (20 mm lower than the original, so the drooping tip meets the egg, not the
   cup). Fix the pedestal to the table. Egg ≈ 12 g.
   Why this way round: a black egg stays a separate dark blob even when the white robot
   touches it, and an empty white cup can never be mistaken for the black egg. Avoid
   strong side lighting: a hard black robot shadow is the only thing that could look
   like the egg (the detector also checks size and roundness).
3. **Camera.** Lens 321 mm above the table, looking down, centre 140 mm from the
   base point along the rest axis (`rig_geometry.png`). After any camera move, redo
   the Lab camera alignment before RL work.
4. Copy the export folder (`spirob-mjlab/exports/<name>`) to the workstation.

## Each session

1. `bash start.sh` → Camera ON → Connect → confirm physical rest → Enable at rest.
   Check the alignment overlay.
2. **RL** tab → **Load exported policy folder…**. It is refused if the contract,
   robot model hash, pose table or weights do not match.
3. **Sample EGG colour** (click the black egg), **Sample PEDESTAL colour** (click the
   empty white cup). Enter camera height 0.321 → **Save camera settings**. The default upright egg
   feature height is **0.087 m**; **0.107 m is its top**, not its centre. Remove an
   old explicit `egg_feature_z_m: 0.107` override from `rl_vision.json` if present.
   Empty-cup mapping uses 0.046 m. Recheck coordinates at several known placements
   after alignment; lens distortion and actual lighting still need rig validation.
4. **Dry run first** (default). Placement `free` (anywhere) or P01–P12 (cross on
   screen). The egg must be > 25 mm from the robot. **Run episode** computes and logs
   decisions without moving. **STOP / HOLD** ends it.
5. **Armed run:** untick Dry run → **Arm motor commands…** → confirm → choose the speed
   → **Run episode**. Keep a hand near **STOP / HOLD**. The episode ends on success,
   30 decisions, a fault, or UNKNOWN.
   Success = egg seen > 35 mm from its start for 0.5 s, **or** the empty white cup seen
   at the start position with no egg near it for 0.7 s.
6. **Next egg.** The robot stays where it stopped. Either press **Run episode** again
   after placing the egg, or tick **Auto-continue**: put the egg on the pedestal at a
   new spot and take your hand away. The next episode starts when the image has been
   still for 2 s and the egg is > 25 mm from the robot. After a non-success the
   egg must be moved at least 20 mm before auto-continue starts again (no retry at
   the same spot).
7. **Straighten robot (D = 0 at 2 mm/s)…** when you want the robot straight again.
8. UNKNOWN episodes: check the video, then **UNKNOWN → knocked off** or
   **UNKNOWN → still on** (kept separate from automatic results).
9. Disconnect and **Save** the run as usual.

## Armed trial plan (to run only after dry runs look right)

1. 10 episodes at 5 mm/s, placement `free`, auto-continue on.
2. If ≥ 8/10 succeed, run the pre-registered 30 (seed 2026) at the same speed:
   `P01 P03 P01 P03 P09 P06 P10 P12 P11 P05 P06 P02 P04 P01 P07 P03 P04 P08 P07 P02 P06 P11 P08 P02 P12 P09 P10 P05 P05 P04`
3. Only if 5 mm/s fails because moves are too slow, repeat at 10, then 15.

Keep every outcome, including faults and aborts. Report successes with the 95 %
Wilson interval, UNKNOWN labels and faults separately.

## Where the evidence is

`logs/runs/session_<id>/rl_episodes/<episode>/`: `episode.json` (policy, contract,
model hash, common speed, outcome), `decisions.jsonl` (raw action, desired, limited
and measured D, per-cable speeds, egg pixels/position, visible flag, frame ages),
`start.jpg`, `dNN.jpg`, `success.jpg`.

Offline replay: `uv run --frozen python -m rl.replay --episode <episode folder> --policy <export folder>`

## Verification (no hardware)

```bash
uv run --frozen python tests/verify_rl_knockdown.py --motor-source ..
QT_QPA_PLATFORM=offscreen uv run --frozen --extra live python tests/verify_rl_window.py
```

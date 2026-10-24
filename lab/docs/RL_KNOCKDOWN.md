# RL egg knockdown — operator guide (contract v2)

**What this adds.** An **RL** tab that runs a policy trained in `spirob-mjlab`
(`Mjlab-SpiRob-EggKnockdown`, contract v2) on the rig. The policy sees only the
overhead camera (egg position, visible or not) and the motor encoders (cable
difference D). It commands **only D**; the cable mean stays at 227.35 mm. Each
decision is one ordinary `independent` move of at most 8 mm in D, through the
existing motor owner and its limits. Workstation-only change: **no WSL redeploy**.

```
camera ─► green egg detector ─► egg (u,v), visible ┐
camera ─► blue cup detector  ─► empty-cup check     │ (success rule 2)
encoders ─► D = payout0 − payout1 ─────────────────┼─► 7 numbers ─► policy ─► new D ─► L0 = 227.35 + D/2
previous action ───────────────────────────────────┘                               L1 = 227.35 − D/2
```

**Speed.** Pick the ceiling in *Cable speed ceiling*: start at **5 mm/s**, then 10,
and 15 only if needed. The speed is scaled to the move: a full 8 mm step in D runs at
the ceiling, a 1 mm step at 1/8 of it (e.g. 1.9 mm/s at 15 mm/s), never below
0.5 mm/s. Moves under 0.5 mm are skipped. Every move therefore takes the same time.

## Before the first session (once)

1. **Robot model.** `lab/model/spirob_robot.xml` must be the knockdown model
   (`spirob_robot_k2_2.148_20261024_0318.xml`, renamed). Check:
   `sha256sum model/spirob_robot.xml` → must start with `fe7d13cd476a`.
   The Lab refuses a policy exported against a different file.
2. **Print the egg and pedestal** from the drawing (`spirob-mjlab/docs/knockdown/egg_pedestal.png`):
   egg green (#29AB4F), pedestal blue (#1F59C7). Pedestal cup top 46 mm above the
   table (20 mm lower than the original, so the drooping tip meets the egg, not the
   cup). Fix the pedestal to the table. Egg ≈ 12 g.
3. **Camera.** Lens 321 mm above the table, looking down, centre 140 mm from the
   base point along the rest axis (`rig_geometry.png`). After any camera move, redo
   the Lab camera alignment before RL work.
4. Copy the export folder (`spirob-mjlab/exports/<name>`) to the workstation.

## Each session

1. `bash start.sh` → Camera ON → Connect → confirm physical rest → Enable at rest.
   Check the alignment overlay.
2. **RL** tab → **Load exported policy folder…**. It is refused if the contract,
   robot model hash, pose table or weights do not match.
3. **Sample EGG colour** (click the green egg), **Sample PEDESTAL colour** (click the
   empty blue cup). Enter camera height 0.321 → **Save camera settings**.
4. **Dry run first** (default). Placement `free` (anywhere) or P01–P12 (cross on
   screen). The egg must be > 25 mm from the robot. **Run episode** computes and logs
   decisions without moving. **STOP / HOLD** ends it.
5. **Armed run:** untick Dry run → **Arm motor commands…** → confirm → choose the speed
   → **Run episode**. Keep a hand near **STOP / HOLD**. The episode ends on success,
   30 decisions, a fault, or UNKNOWN.
   Success = egg seen > 35 mm from its start for 0.5 s, **or** the empty blue cup seen
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
model hash, speed ceiling, outcome), `decisions.jsonl` (raw action, desired, limited
and measured D, per-cable speeds, egg pixels/position, visible flag, frame ages),
`start.jpg`, `dNN.jpg`, `success.jpg`.

Offline replay: `uv run --frozen python -m rl.replay --episode <episode folder> --policy <export folder>`

## Verification (no hardware)

```bash
uv run --frozen python tests/verify_rl_knockdown.py --motor-source ..
QT_QPA_PLATFORM=offscreen uv run --frozen --extra live python tests/verify_rl_window.py
```

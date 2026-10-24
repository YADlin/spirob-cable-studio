# RL egg knockdown — operator guide

**What this adds.** An **RL** tab in the Lab that runs a policy trained in
`spirob-mjlab` (`Mjlab-SpiRob-EggKnockdown`) on the rig. The policy sees only the
overhead camera (egg position) and the motor encoders. It moves the cables through
the existing motor owner with ordinary `independent` moves at 15 mm/s, one bounded
move (≤ 8 mm per cable) per decision. Workstation-only change: **no WSL
redeploy is needed** (`setup.py deploy` files are untouched).

```
camera frame ─► egg detector ─► egg (u,v) ┐
encoders ─────► payout (model metres) ────┼─► contract obs (8) ─► policy ─► target ─► mm ─► 'independent' move
previous action ──────────────────────────┘                                            (Lab limits, Stop/Hold, watchdog)
```

## Before the first session (once)

1. **Measure** with a tape and write down: camera lens height above the table [m];
   egg cap diameter; pedestal height and cup size. The trained scene assumes a
   40 mm cap on a pedestal whose top is 66 mm above the table, with the robot plane
   at 90 mm. Report big differences before armed trials.
2. Check the camera sees the whole placement region plus a margin: with the current
   alignment the frame reaches only ≈ 8 cm beyond the −u edge. Knocked eggs that
   leave the frame become UNKNOWN and need manual labels.
3. Copy the export folder (from `spirob-mjlab/exports/<name>`) to the workstation.

## Each session

1. `bash start.sh` → Camera ON → Connect → confirm physical rest → Enable at rest
   (the existing flow). Check alignment as usual.
2. **RL** tab → *Egg knockdown policy…* → **Load exported policy folder**. The Lab
   refuses the policy if its contract, robot model or weights do not match.
3. **Sample egg colour**, then click the centre of the egg. Enter the camera height
   and **Save camera settings** (enables the parallax correction; 0 = off).
4. **Dry run first** (default): choose a placement (P01–P12), put the egg on the
   green cross (✓ appears within 5 mm), **Run episode**. Decisions are computed and
   logged; no motor command is sent. Stop with STOP / HOLD.
5. **Armed run:** untick Dry run → **Arm motor commands…** → confirm → **Run episode**.
   Keep a hand near **STOP / HOLD**. The episode ends on success (egg seen > 35 mm
   from its start for 0.5 s), 24 decisions, a fault, or UNKNOWN.
6. Between episodes: **Return to start pose (2 mm/s)** *before* placing the next
   egg (the episode will not start unless both cables are within 1 mm of the start
   pose). Then place the egg on the next cross.
7. UNKNOWN episodes: review the recorded video, then press **UNKNOWN → knocked off**
   or **UNKNOWN → still on**. These labels are kept separate from automatic results.
8. Disconnect and **Save** the run as usual.

## Pre-registered armed trial order (30 episodes, seed 2026)

`P01 P03 P01 P03 P09 P06 P10 P12 P11 P05 P06 P02 P04 P01 P07 P03 P04 P08 P07 P02 P06 P11 P08 P02 P12 P09 P10 P05 P05 P04`

Run every episode in this order and keep every outcome, including faults and
aborts. Target: ≥ 27/30 automatic successes over ≥ 10 placements; report counts,
the 95% Wilson interval, UNKNOWN labels and faults separately.

## Where the evidence is

`logs/runs/session_<id>/rl_episodes/<episode>/`: `episode.json` (policy, contract,
calibration fingerprints, outcome), `decisions.jsonl` (raw, desired, limited,
command mm, encoder mm, egg pixels/position, frame ages), `start.jpg`, `dNN.jpg`,
`success.jpg`. Continuous video: Windows `SpiRob_Data/<session>/` (recording starts
and stops with each armed episode).

Replay a logged episode offline:
`uv run --frozen python -m rl.replay --episode <episode folder> --policy <export folder>`

## Verification (no hardware)

```bash
uv run --frozen python tests/verify_rl_knockdown.py --motor-source ..
QT_QPA_PLATFORM=offscreen uv run --frozen --extra live python tests/verify_rl_window.py
```

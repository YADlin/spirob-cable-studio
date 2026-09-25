# Length sliders and paired starts — 3.1.0

This patch targets the Studio version with the contrast and serial-deadline
updates, **before** the optional intermittent-diagnostics patch. You do not need
to apply that optional patch first. Local configuration and research logs are
not part of this update. The existing communication fault latch is retained;
this change does not repair electrical corruption or ignore malformed replies.

## Apply and inspect

Close Studio. Download `spirob-length-sliders.patch` to Downloads:

```bash
cd ~/Two_Cable_Motor_Setup/spirob-cable-studio
git apply --check ~/Downloads/spirob-length-sliders.patch
git apply ~/Downloads/spirob-length-sliders.patch
uv run app.py --demo
```

If `git apply --check` fails, stop and report it. Do not force the patch or
overwrite your local configuration.

In the demo: Connect → Enable at rest → Lengths. Try mean 0.220 m and difference
0, then difference +20 mm; targets become 0.230 and 0.210 m. Change mean to
0.180 m and targets become 0.190 and 0.170 m. Enter individual targets 0.240
and 0.200 m to see mean 0.220 m and difference +40 mm. The Live column follows
encoder feedback and is separate from the editable Target column.

Dragging a slider previews the resulting pair; release sends one paired move
when the checkbox is selected. Numeric edits require Move both cables. Equal
lengths and Copy live lengths change the preview only. While a move is active,
the pair controls wait for both drives to settle rather than queueing old edits.

Exit the demo, then start hardware mode:

```bash
uv run app.py
```

Re-establish the measured rest lengths. For an initial comparison use a small
feasible paired displacement and a low cable speed, so its motion lasts longer
than a serial transaction. Check that both motors move in the intended directions
and overlap in time. Inspect the displayed start-write request gap. It measures
host command timing, not the actual motor start times. A shared 9600-baud link
still has a transaction gap, normally tens of milliseconds; very short moves may
finish inside it. Do not interpret the change as a hardware synchronized start,
a common arrival time, or a fix for communication faults.

## Record and publish the change

```bash
git status --short
git add spirob_cable/pair_controls.py spirob_cable/ui.py spirob_cable/drive.py spirob_cable/rig.py spirob_cable/__init__.py
git add tests/test_protocol.py tests/test_rig.py tests/test_ui.py
git add pyproject.toml uv.lock README.md OPERATIONS.md ENGINEERING.md CHANGELOG.md docs/LENGTH_CONTROLS_UPDATE.md
git diff --cached --stat
git diff --cached
git commit -m "Add paired length sliders and reduce motor start delay"
git push
```

Review the complete staged diff; these commands also stage any earlier uncommitted
changes in the listed files. Do not commit `config.json`, logs or credentials.
Use your existing SSH remote and branch.

## Verification and implementation

68 local tests pass. They include Qt slider release behavior, unequal cable
targets and reference lengths, limits, full/half-difference procedure conversion,
low/high register ordering, partial failures, STOP during staging/commit, and
resuming from STOP's hold mode with the present encoder count preloaded.
No physical drive is available in the development environment.

The manual's register-18 commit behavior is used to stage each register-16 low
word first, then commit each high word. While already in position mode the
extra enable write is omitted. The second drive's commit is not delayed until
the first drive reaches its endpoint. If the second commit fails, the existing
fault path attempts to disable both motors; motion already initiated cannot be
rolled back.

All UI differences now mean L1−L2. Saved procedures, internal modes and existing
research CSVs retain half-difference D=(L1−L2)/2 for compatibility. UI conversions
are explicit and tested; previously saved procedures keep the same targets.

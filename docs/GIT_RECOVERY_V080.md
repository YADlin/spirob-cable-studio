# SpiRob consolidated source and recovery

The workstation's existing spirob-cable-studio repository is the source of truth.
Motor source is in spirob_cable/, acquisition source in spirob_experiment/, and
Lab v0.8.0 in lab/. Keep their separate environments: their pinned NumPy and
OpenCV versions differ. Never install both dependency sets into one environment.

## Evidence from the supplied snapshots, 12 October 2026

- All 59 common workstation/WSL files match byte-for-byte.
- WSL contributes three additional experiment plans in examples/.
- All 10 Windows camera-addon files match the workstation/WSL files.
- All 10 Windows runtime source/script files match the Lab archive.
- DEPLOYMENT.txt matches the generated 0.7.0 label (LF newline). It is metadata;
  matching runtime code does not establish that every part was deployed together.
- All 83 entries in the Lab v0.8.0 release checksum manifest match.
- These are file comparisons, not a new hardware or scientific validation.

## What was imported

Reviewed Lab release files, including models, meshes, calibration, source,
tests, scientific config.json, docs, locks and the 15,355,801-byte offline
data/quasistatic_review.zip. results/observations.json is an existing reference
fixture listed in the release manifest; other generated results are excluded.
Historical release_metadata/ and preview.png are preserved. The stale root
RELEASE_SHA256.json and verification.json are omitted; use the versioned records.
Only Lab .gitignore and the repository .gitignore change; scientific code is
copied without edits. rig.local.json and lab.local.json are copied privately
when present, and are ignored. Original standalone Lab files remain untouched.

## Finish this import on the workstation

Inspect `git diff --cached --stat` and `git diff --cached`. The import is staged
on integrate-spirob-lab-v0.8.0, but has not been committed or pushed.

From the repository root:

```bash
uv sync --frozen --extra capture
QT_QPA_PLATFORM=offscreen uv run --frozen --extra capture python -m unittest discover -s tests -v
QT_QPA_PLATFORM=offscreen uv run --frozen --extra capture python -m unittest discover -s tests_pipeline -v
cd lab
uv sync --frozen --extra live
QT_QPA_PLATFORM=offscreen uv run --frozen --extra live python tests/verify_v080.py
bash start.sh --demo
```

Close the demo and return to the repository root. If checks pass:

```bash
git commit -m "Consolidate motor, camera addon and SpiRob Lab v0.8.0"
git push -u origin integrate-spirob-lab-v0.8.0
```

Open a pull request against main and merge after reviewing the changes. Do not
force-push. GitHub's main branch contains the full setup only after that merge.
This import does not change the existing CI workflow; Lab/addon checks above
must be run explicitly. Dependency availability on a fresh machine still needs
verification with uv sync; this source import is not evidence of that check.

## New Ubuntu workstation after merge

Install Git and uv, then:

```bash
mkdir -p ~/Two_Cable_Motor_Setup
cd ~/Two_Cable_Motor_Setup
git clone git@github.com:YADlin/spirob-cable-studio.git
cd spirob-cable-studio
uv sync --frozen --extra capture
cd lab
uv sync --frozen --extra live
bash start.sh --demo
```

SSH cloning requires GitHub authentication. For this public repository, HTTPS
cloning is another option. Restore private settings before live operation and
adjust usernames, machine addresses and paths for the replacement machine.
Follow lab/docs/OPERATING_MANUAL.html and docs/SSH_WINDOWS_WSL.md for OS setup.
Git does not configure Windows OpenSSH, WSL, USB forwarding, serial permissions,
camera access, camera tokens or SSH keys.

## Windows and WSL alignment

Do not overwrite or delete either working laptop installation during this import.
After the integration commit is published, a fresh checkout of that commit can
be made alongside each old installation. WSL needs the root motor environment;
Windows camera operation needs the root capture environment. Restore the WSL
motor config.json, and update lab/rig.local.json to the actual checkout paths,
Python executables and existing token locations before deploying from lab/.
Check `git rev-parse HEAD` in each checkout to record the common revision.

`python3 setup.py deploy` in lab/ copies the Lab runtime only. It does NOT
synchronize spirob_cable/ or spirob_experiment/. Those must come from the chosen
Git revision independently. Finish active acquisition and disconnect before
deployment. Recheck camera, USB and motor startup on the physical rig afterward.

## Private and experimental backups

Copy the importer-created backup directory to another device/private storage.
It contains the pre-import Git bundle, reviewed root source snapshot, original
.gitignore, and available private root/Lab settings. It is NOT a complete data
backup and is NOT encrypted. Back up recordings, fitted results and campaign
data separately; the original standalone Lab keeps those files.

Also keep secure backups of camera tokens and required SSH credentials outside
Git. Machine settings can be recovered from root/config.json, lab/rig.local.json
and lab/lab.local.json within the private backup. Never publish this backup.
Use a new local settings backup whenever calibration or hardware paths change.

For ongoing work, edit the consolidated source, test, commit and push. Do not
continue separate source development in the old standalone folder. Retain that
folder for its historical data until those data have a separate verified backup.

# SpiRob experiment pipeline 0.1.0

This additive package works beside Cable Studio 3.1.0. The existing GUI is for manual setup and jogging; the new command-line coordinator owns the motor connection during a recorded experiment. Close the GUI before using the coordinator. No existing application files are replaced by this package.

## What is implemented

- A versioned trial file with explicit cable 1 and cable 2 targets in millimetres, speed, repetitions, settled holds, pre-roll, post-roll and a total deadline.
- The existing serial worker, register protocol, limits, feedback validation and stop behavior, accessed through a headless adapter.
- A native Windows OpenCV camera service, authenticated and bound to loopback by default. The instructions use an SSH tunnel from WSL, so no camera port is exposed on the LAN.
- Timestamped full-resolution JPEG frames and a frame index; original motor CSV/event logs; a shared session manifest and trial/config snapshots.
- Camera startup/rate/staleness checks before and during motor execution; failure records and attempted motor stop/disconnect on cancellation or camera loss.
- Initial and final host clock measurements. Derived frame times use an affine mapping between the Windows camera clock and WSL coordinator clock.
- Automatic selection of the first frame, last frame, and a frame near the middle of every settled hold; selection time errors are recorded.
- Offline keypoint SSE/RMSE, coverage checks, and interpolation-gap checks.
- Interfaces for a future keypoint extractor, MuJoCo backend, and archive destination.

Not implemented yet: automatic physical keypoint detection, lens/workspace calibration, a model-specific MuJoCo actuator/site adapter, an optimizer, or Google Drive upload. No stiffness estimate is produced by acquisition alone.

The synthetic camera in the demo is a test pattern, not a simulated image of SpiRob. The demo checks acquisition coordination and file handling; it does not validate tracking or robot physics.

## Module boundaries

| File | Responsibility |
|---|---|
| `schema.py` | Validate and expand cable-length trials; translate once to the old half-difference API |
| `motor.py` | Own the existing Worker; service its heartbeat; receive state and attempt shutdown |
| `camera.py` | Real/synthetic camera sources, frame recording, HTTP service and coordinator lease |
| `camera_client.py` | Authentication, bounded requests, host clock measurements |
| `runner.py` | Camera readiness, motor reference, trial execution, cleanup |
| `storage.py` | Local session metadata, checksums, future archive interface |
| `derive.py` | Align host timestamps, export motor inputs, choose review frames |
| `analysis.py` | Keypoint and simulation interfaces; offline SSE; explicit exponential profile |

## 1. Install in the WSL motor repository

Extract the add-on ZIP into a temporary directory. Copy its `spirob_experiment`, `examples`, `tests_pipeline`, and `docs` directories, plus `PIPELINE_START_HERE.md`, into the existing repository. Its documentation filenames are new. Preserve the current `config.json` and existing app files.

Inside WSL:

```bash
cd ~/Research_Work/Two_Cable_HW/spirob-cable-studio
uv add --optional capture opencv-python-headless==4.11.0.86
uv sync --extra capture
uv run --extra capture python -m unittest discover -s tests_pipeline -v
uv run --extra capture python -m spirob_experiment demo --root "$HOME/SpiRob_Demo_Data"
```

`uv add` records the optional dependency in `pyproject.toml` and updates `uv.lock`. The headless OpenCV wheel provides capture/encoding without another Qt GUI installation. Your original app still starts with `uv run app.py` (and your chosen Qt display setting).

The demo opens no USB device and requires no camera, SSH or robot. It prints the resulting session directory. Inspect `run.json`, `derived/selected_frames`, and the motor logs.

## 2. Windows camera process

Keep CP2102 attached to WSL as already configured. Keep the camera available to Windows. Close Windows Camera/other camera applications before starting capture.

First set up Windows OpenSSH using `SSH_WINDOWS_WSL.md`. Then extract the same add-on ZIP in Windows to:

```text
C:\Users\YOUR_WINDOWS_ACCOUNT\spirob-experiment-addon
```

This folder must directly contain `spirob_experiment` and `examples`. It does not need the motor repository: the native camera command has no motor dependencies.

Open ordinary PowerShell in the logged-in Windows desktop. Install Windows uv if it is not already available:

```powershell
winget install --exact --id astral-sh.uv
```

Open a new PowerShell after installation, then:

```powershell
Set-Location "$env:USERPROFILE\spirob-experiment-addon"
$spirobData = "$env:USERPROFILE\SpiRob_Data"
$spirobToken = "$env:USERPROFILE\.spirob\camera-token.txt"
uv run --no-project --python 3.12 --with opencv-python-headless==4.11.0.86 python -m spirob_experiment make-token $spirobToken
uv run --no-project --python 3.12 --with opencv-python-headless==4.11.0.86 python -m spirob_experiment camera-serve --root $spirobData --token-file $spirobToken --bind 127.0.0.1 --device 0 --backend dshow --width 1920 --height 1080 --fps 30
```

Create the token once. Later starts use the existing token. It is never printed or stored in session metadata. Keep its file outside Git and inside your Windows user profile. On Windows, its access follows the profile's ACL.

Camera index 0 may be the laptop webcam. Confirm the actual image using the camera check below; stop the service and change `--device 1` or another index if necessary. The DirectShow backend is requested explicitly. If that backend cannot open this camera, test `--backend msmf` and inspect the captured settings. Do not infer capture success solely from a property setter returning true.

The agent remains running in this PowerShell window; it opens the camera only when the coordinator requests a session. Ctrl+C stops the agent. Initially launch it in the logged-in Windows desktop. Camera access from a Windows SSH session depends on that machine's camera permissions/backend and has not been tested here.

## 3. WSL-to-Windows camera tunnel

Open another WSL terminal. In the usual WSL NAT networking mode:

```bash
spirob_win_host=$(ip route show default | awk '{print $3; exit}')
read -r -p 'Windows account name (not the WSL account): ' spirob_win_user
ssh -N -T -o ExitOnForwardFailure=yes -o ServerAliveInterval=10 -o ServerAliveCountMax=3 \
  -L 127.0.0.1:8766:127.0.0.1:8765 "$spirob_win_user@$spirob_win_host"
```

Use the Windows account password, not its Windows Hello PIN. Check the host key fingerprint as described in the SSH guide. Leave this terminal running. In mirrored WSL networking, use `spirob_win_host=127.0.0.1` instead of the gateway query. You do not need to change networking mode.

This makes Windows port 8765 reachable at WSL `http://127.0.0.1:8766`. Frames are saved directly to the shared local disk; only small control/status messages cross this tunnel. If the tunnel exits, camera checks fail and an active coordinator attempts to stop the trial.

## 4. Check the real camera before moving motors

In a third WSL terminal, locate the shared Windows profile paths:

```bash
cd ~/Research_Work/Two_Cable_HW/spirob-cable-studio
spirob_win_profile=$(powershell.exe -NoProfile -Command '$env:USERPROFILE' | tr -d '\r')
spirob_win_profile_wsl=$(wslpath -u "$spirob_win_profile")
spirob_data="$spirob_win_profile_wsl/SpiRob_Data"
spirob_token="$spirob_win_profile_wsl/.spirob/camera-token.txt"

uv run --extra capture python -m spirob_experiment camera-check \
  --root "$spirob_data" --camera-url http://127.0.0.1:8766 \
  --token-file "$spirob_token" --seconds 10 --min-fps 20
```

This command does not instantiate the motor adapter. It saves a CAMERA_CHECK session. Inspect:

- `camera/frames`: correct camera, full workspace, reference object, sharp visible robot features.
- `camera/source.json`: reported resolution, FOURCC, FPS and backend.
- `derived/timing_quality.json`: measured average delivery rate and largest arrival gap.
- `camera/capture.json`: total encoded bytes and clean completion state.

A source reporting 30 FPS can still deliver fewer frames. At 1080p, the DS-U02 family needs the appropriate compressed capture mode to reach its advertised high frame rate. Treat a failed rate check as a setup issue to investigate, not a reason to lower the threshold without considering the motion bandwidth.

The two processes verify a session nonce in the same directory before enabling the motors. A wrong shared root causes an error before the motor worker starts.

## 5. Prepare and run a hardware trial

Close Cable Studio after jogging the physical robot to its intended rest. The coordinator establishes a new encoder reference at the current position; it does not physically home the robot. Closing the old app releases torque, so check the physical position again before declaring rest in the new run.

Copy the example and edit it:

```bash
cp examples/experiment_pilot.json examples/my_trial.json
nano examples/my_trial.json
```

The example's 220 mm rest lengths and 0.5 mm per-cable movement are illustrative. Use your measured rest lengths and an already verified movement. Both values in `rest_lengths_mm` must match the current `config.json`. Every target is an absolute cable length in mm and must lie inside its own rest +/-70 mm window. The example is full difference 1 mm at its bent step, not 1 mm per cable.

Check without opening a serial port:

```bash
uv run --extra capture python -m spirob_experiment validate \
  --config config.json --plan examples/my_trial.json
```

When the robot is at the declared physical rest and the camera check passed:

```bash
uv run --extra capture python -m spirob_experiment run \
  --config config.json --plan examples/my_trial.json \
  --root "$spirob_data" --camera-url http://127.0.0.1:8766 \
  --token-file "$spirob_token" --execute --at-rest
```

The sequence is: start recording -> verify delivery/storage -> connect with torque disabled -> verify stationary feedback -> enable at current position and establish the declared rest -> pre-roll -> move/settle/hold steps -> post-roll -> stop -> disconnect/release -> stop camera -> derive review frames.

**The end policy releases motor torque.** An automatic return is performed only if your plan includes that target. Hold duration starts when both motor axes satisfy the existing settling criterion; it does not certify that the flexible body has stopped vibrating. Choose adequate holds for the experiment.

Ctrl+C requests cancellation. The local worker's existing 1.5-second heartbeat remains active; the camera has a 10-second coordinator lease. Failure/cancellation never automatically resumes. Python, SSH and USB cannot guarantee a stop after a process/host failure; retain the independent motor-power disconnect used with the existing controller.

Only one program may own the serial adapter. This add-on reuses one worker and one client per port; it does not make the old GUI and coordinator safe to run concurrently. Do not launch another hardware controller while it runs.

## 6. Stored data and timing

Each run has a UTC/unique/mode identifier, so files cannot silently overwrite another run.

| Artifact | Meaning |
|---|---|
| `trial.json` | Requested physical cable targets, speed and repetitions |
| `run.json` | States, config snapshot, software version, clock mapping inputs, cleanup results |
| `events.jsonl` | Coordinator phase transitions |
| `camera/source.json` | Requested/reported capture settings |
| `camera/frames/*.jpg` | Every image successfully acquired and saved, JPEG quality 95 |
| `camera/frames.csv` | Frame index, read start/end, write end, dimensions |
| `motor/<session>/...` | Existing raw motor samples, events, pair logs and config metadata |
| `derived/frames_aligned.csv` | Camera host-arrival times mapped into coordinator session time |
| `derived/motor_inputs.csv` | Per-axis measured length and target, metres, individual read intervals |
| `derived/commands.csv` | Per-axis target requests and requested speeds |
| `derived/selected_frames/` | First, last and mid-hold review images |
| `derived/selected_frames.csv` | Provenance and requested/selected time difference |
| `derived/timing_quality.json` | Delivery and clock-mapping limitations |

JPEG files are decoded/re-encoded camera images, not sensor-raw data or bit-for-bit source MJPEG. There is deliberately no constant-FPS MP4 treated as a measurement clock. An image sequence plus timestamps retains irregular delivery and survives interruption without a container-repair step. It can be converted to a review video later.

Coordinator and camera metadata record their own Python/platform/dependency versions and SHA-256 hashes of available pipeline/controller source files. The two processes may run in different environments; retain both records. Source hashes identify the code used even when changes have not yet been committed. Keep that code in Git as well so it can be recovered later.

Clock pings choose the minimum round-trip sample at start and end. Half the round-trip is a network uncertainty indicator only. The affine mapping estimates clock drift; sensor exposure time, USB buffering and driver latency remain unknown. Static holds can be useful before precise optical timing is available. Dynamic identification needs a visible/logged synchronization measurement and a check that the camera resolves the motion.

If acquisition fails, the session remains FAILED. If acquisition succeeds but derived exports fail, it is POSTPROCESS_FAILED and originals remain available. Rebuild derived outputs using:

```bash
uv run --extra capture python -m spirob_experiment derive /absolute/path/to/session
```

After inspection, hash the closed files before copying to your workstation:

```bash
uv run --extra capture python -m spirob_experiment seal /absolute/path/to/session
```

Keep data and tokens outside Git. The archive interface is reserved for a future Google Drive uploader. Upload closed sessions in a separate process so network upload cannot stall acquisition. No Google account access or upload is implemented in this version.

## 7. MuJoCo and cost-function extension

The common experiment input is a pair of physical cable lengths. Preserve THREE distinctions:

1. The requested plan is shared experiment intent; hold/move timing is resolved during acquisition.
2. Command events describe when targets were requested; serial writes and motor motion are sequential, not simultaneous.
3. Encoder-derived lengths describe observed spool travel; they are not direct measurements of tendon stretch, slack or tension.

The future MuJoCo adapter must explicitly map the physical cable length convention to the model's tendon paths and actuator controls. A motor-controlled external cable span may differ from the full model tendon length. Rest offsets, payout signs, actuator gear and the actuator's input units must be checked. Do not write metre values into `data.ctrl` until that mapping is established.

For a controlled actuator comparison, replay the recorded target events with the same speed/acceleration model. For isolating robot-body dynamics, replay measured per-axis cable histories as imposed boundary trajectories, with documented interpolation and actuator assumptions. Record which input mode was used. They answer different questions and must not be silently mixed.

Implement `SimulationBackend.simulate(...)` in `analysis.py` for your current XML and meshes. Record model checksum, site names, cable mapping, masses/inertias, gravity, spring references, damping, timestep, solver and input mode. Use the previously verified stable timestep as a starting point; check numerical convergence for the selected motion.

Implement `KeypointExtractor.extract(...)` to produce:

```csv
t_s,point,x,y,valid
0.000,base,0.0,0.0,1
0.000,tip,0.012,0.180,1
0.033,tip,,,0
```

Use identical, documented units, coordinate frame, point identities and time origin in physical and simulated data. A missing physical observation is an explicit `valid=0` row. For pixel coordinates, project simulated points through the calibrated camera. For workspace millimetres/metres, transform observations using the corresponding calibration. Plain 2D distance is not a substitute for treating out-of-plane motion.

The implemented cost is the sum of squared 2D point distances at the observed timestamps. Simulation points are interpolated only within supported intervals. Missing simulation coverage fails evaluation; it cannot reduce the score by dropping difficult points. Physical tracking coverage is reported and must exceed the configured threshold for every required point. The supplied example demonstrates units and is not experimental data:

```bash
uv run python -m spirob_experiment cost \
  --observed examples/keypoints_observed.csv \
  --simulated examples/keypoints_simulated.csv \
  --metadata examples/keypoint_comparison.json \
  --output /tmp/spirob_cost_example.json
```

Expected SSE is 0.000002 m^2. Point-distance RMSE is approximately 0.000816497 m.

The helper `stiffness_profile(k0, beta, coordinates)` implements `k_i=k0*exp(-beta*s_i)`. Supply the exact coordinates used by the current generator; it never guesses whether beta is per joint or over normalized length. Positive beta decreases stiffness with increasing coordinate. If your generator uses the opposite sign, translate explicitly in the future adapter.

Fit over multiple body points and trials; reserve a separate trial for validation. Inspect the two-parameter error surface and sensitivity before reporting a unique k0/beta. Quasi-static cable-constrained shapes may weakly identify overall stiffness; known loading or adequately sampled dynamics may be needed. Do not let incorrect rest angles, masses, damping, cable offsets or camera scaling be absorbed into k0 and beta.

## 8. Git

From the WSL motor repository, inspect and stage the additive implementation and dependency changes:

```bash
git status --short
git add spirob_experiment tests_pipeline examples/experiment_pilot.json \
  examples/keypoints_observed.csv examples/keypoints_simulated.csv \
  examples/keypoint_comparison.json docs/EXPERIMENT_PIPELINE.md \
  docs/SSH_WINDOWS_WSL.md docs/PIPELINE_VALIDATION.md PIPELINE_START_HERE.md pyproject.toml uv.lock
git diff --cached --check
git diff --cached --stat
git commit -m "Add modular camera and motor experiment pipeline"
git push origin main
```

Use your actual working branch if it is not main. Existing unstaged changes should be reviewed separately. No remote commit or push was performed for this package.

## References

- OpenCV capture API and property caveats: https://docs.opencv.org/4.x/d8/dfe/classcv_1_1VideoCapture.html
- uv installation: https://docs.astral.sh/uv/getting-started/installation/
- Windows/WSL networking: https://learn.microsoft.com/en-us/windows/wsl/networking
- MuJoCo spring, actuator and tendon semantics: https://mujoco.readthedocs.io/en/stable/XMLreference.html

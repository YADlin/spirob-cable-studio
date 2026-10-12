# SpiRob experiment add-on 0.1.0

This package adds repeatable acquisition to the working Cable Studio 3.1.0 repository. It contains only new files. It does not replace the motor app, your hardware configuration, or your Git history.

## Start with the offline demo

Extract this ZIP to a temporary folder. In WSL, copy these four directories into your current `spirob-cable-studio` repository: `spirob_experiment`, `tests_pipeline`, `examples`, and `docs`. Merge the directories; the add-on's example and documentation filenames are new. Copy this `PIPELINE_START_HERE.md` file too.

Then, inside the repository:

```bash
uv add --optional capture opencv-python-headless==4.11.0.86
uv sync --extra capture
uv run --extra capture python -m spirob_experiment demo --root "$HOME/SpiRob_Demo_Data"
```

This opens neither the serial adapter nor a real camera. It uses simulated motors and a synthetic image source. Open the printed session folder and inspect `derived/selected_frames` and `run.json`.

## Then connect the camera and motors

Follow [docs/EXPERIMENT_PIPELINE.md](docs/EXPERIMENT_PIPELINE.md) in order. It includes the native Windows camera service, the WSL-to-Windows tunnel, shared data paths, a camera-only check, plan validation, and hardware execution.

Follow [docs/SSH_WINDOWS_WSL.md](docs/SSH_WINDOWS_WSL.md) to enter the laptop's existing WSL environment from the Ubuntu workstation. The coordinator runs on the laptop, even when its terminal is accessed remotely.

Read [docs/PIPELINE_VALIDATION.md](docs/PIPELINE_VALIDATION.md) for what was tested and what still needs checking on the physical setup.

## The experiment contract

`examples/experiment_pilot.json` specifies explicit cable lengths in **millimetres**. There is no mean/difference ambiguity in this file. The example assumes two 220 mm rests, moves to 220.5/219.5 mm, returns, and repeats three times. Replace those illustrative values with the measured references and motion already verified on your setup.

The coordinator starts recording, checks camera delivery and shared storage, references the current physical rest, executes the validated trial, and saves requested targets, measured motor lengths, frames, and timing metadata. At the end it stops and disconnects, **releasing torque**. A physical return occurs only when the plan requests it.

Use the existing GUI for manual setup; close it before running hardware acquisition. The CLI and GUI must not own the same serial adapter concurrently.

## Analysis grows separately

The offline distance cost is implemented. `SimulationBackend`, `KeypointExtractor`, and `ArchiveSink` define the extension points for MuJoCo, image analysis, and later Google Drive upload. Automatic tracking, your model's actuator/site mapping, parameter optimization, and cloud upload are not included yet.

The next physical milestone is one short CAMERA_CHECK followed by one already verified trial. Review the selected hold frames before automating keypoint detection or fitting stiffness. Keep original recordings; host frame-arrival timestamps do not establish sensor exposure time.

No Git commit or push is performed by installing this package. The acquisition guide includes explicit staging, commit, and push commands.

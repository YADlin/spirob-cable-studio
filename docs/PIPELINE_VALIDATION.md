# Validation record — experiment pipeline 0.1.0

Validated on 2026-09-26 in a Linux execution environment against the supplied Cable Studio 3.1.0 source, including its local slider-control changes. No physical serial adapter or camera was connected to this environment.

## Automated checks

- **68 existing application tests passed** with Qt's offscreen backend: geometry, register protocol, bounded ASCII transport, paired targets, stop/fault behavior, worker heartbeat, recording, and UI behavior.
- **16 pipeline tests passed**: strict plan validation, length/half-difference conversion, rest mismatch, host clock mapping, exponential profile coordinates, a complete repeated trial using the existing demo motor worker, camera loss during motion, shared-root mismatch, rejection of synthetic capture in hardware mode, authentication/path validation, camera lease expiry, lost start acknowledgement cleanup, keypoint interpolation/SSE, tracking coverage, and missing simulation coverage.
- The supplied nine-step CLI demo completed. The inspected run recorded **372 frames**, selected **11 review frames** (nine holds plus endpoints), and recorded confirmed worker exit and both simulated disable acknowledgements. Its measured synthetic delivery was approximately **19.96 FPS**. These numbers describe a software exercise, not the physical camera or motor performance.
- The cost example returned SSE **2.0000000000000003e-06 m²**, point-distance RMSE **0.0008164965809277262 m**, and full coverage for its three points in time.
- A separate camera-only run completed from the extracted add-on directory with the motor package absent. It recorded 47 synthetic frames and created no motor log. This checks import/dependency separation; it does not test native Windows capture.

Commands:

```bash
QT_QPA_PLATFORM=offscreen uv run --extra capture python -m unittest discover -s tests -v
uv run --extra capture python -m unittest discover -s tests_pipeline -v
uv run --extra capture python -m spirob_experiment demo --root "$HOME/SpiRob_Demo_Data"
```

Implementation testing used `uv run --with opencv-python-headless==4.11.0.86` before adding the optional dependency to the user's project. The installation guide records that dependency using `uv add --optional capture`.

## Physical acceptance still required

1. Verify native Windows camera opening, correct device selection, reported resolution, focus/exposure, actual frame delivery, and disk throughput using CAMERA_CHECK. This camera/backend combination was not available for testing here.
2. Verify Windows OpenSSH login, WSL tunnel access, and shared Windows/WSL paths on the laptop. The instructions follow Microsoft documentation; these operating systems were not available for a live connection test.
3. Verify one attended small trial on the existing hardware after setting the physical rest. The pipeline reuses the controller's worker, but simulated tests cannot establish real motor behavior or stop reliability.
4. Check the coordinate/time reference before comparing camera and simulation. The measured camera timestamp is host delivery, not sensor exposure. USB/driver buffering is unmeasured.
5. A successful encoder hold means the motors met the existing settling criterion. It does not prove the flexible robot has reached mechanical equilibrium.

## Research limits of this version

- Sequential serial dispatch/feedback is logged. There is no simultaneous hardware trigger across the two motors and the camera.
- Length from spool encoder counts is an actuator estimate. Cable stretch, slack, spool build-up, and routing must be assessed separately.
- Automatic keypoint extraction, calibration, MuJoCo model mapping, parameter search, and Drive upload are future modules. No k0/beta estimate is claimed.
- The cost function assumes a fixed observation set, shared units/frame/time origin, and explicit missing observations. An extractor must output `valid=0` rows for missed expected points; omitting rows would make the coverage denominator incomplete.
- Raw session folders remain useful when derived processing fails. A session with an unconfirmed live writer cannot be sealed for archival by the CLI.
- Process/OS/power failures can defeat software cleanup. Hardware trials retain the independent motor-power disconnect used with Cable Studio.

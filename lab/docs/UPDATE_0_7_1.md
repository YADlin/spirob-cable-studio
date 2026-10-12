# SpiRob Lab 0.7.1 — independent camera and backbone switches

The combined setup-preview switch is replaced by two separate switches at the
top of the viewer: **Camera: ON/OFF** and **MuJoCo backbone: ON/OFF**.

| Camera | Backbone | Before Connect |
|---|---|---|
| OFF | OFF | Neither view is displayed; no camera or motor session is opened. |
| ON | OFF | Camera setup preview, with image measurements when available. |
| OFF | ON | Local MuJoCo reference on a plain background; camera stays closed. |
| ON | ON | Camera with the MuJoCo reference overlay for alignment. |

Neither switch starts the motor-session timer. **Connect** still starts the run.
Camera defaults to OFF on launch; backbone defaults to ON. The standalone
reference is fitted to its viewport for visibility. Once the camera is open,
the overlay uses the saved native-pixel alignment and scale. No calibration is
changed by toggling either view.

Before Connect, Camera OFF closes the setup camera. Backbone OFF hides the red
curve, red joint/base marks and correspondence lines, while the camera can keep
running. Switching the backbone alone opens no SSH connection or camera lease.
Both display choices are preserved when Connect transitions from setup preview
to a motor run. Disconnect closes the run, turns Camera OFF and keeps the
backbone choice. Camera failure also leaves the local reference available.

During a motor run, the switches independently show or hide the two views.
Camera acquisition, feedback-freshness checks, MuJoCo calculations and analysis
continue even if a view is hidden. The text below the switches explains this.
Use **Disconnect** to close acquisition and end the motor run. Hidden video
does not disable stale-data protection or change motor commands.

Before Connect the backbone is explicitly labelled **reference**. It has no
measured motor inputs and cannot produce a physical J or stiffness sweep. Base
picking requires Camera ON and a fresh image. A captured manual analysis and
pose export still contain their complete image/model evidence regardless of
which layers are currently visible in the live viewer.

This is a workstation UI correction. The laptop runtime protocol remains
0.7.0; deployment is unnecessary if 0.7.0 is already installed and deployed.
The cumulative update also supports older published releases; those users
should run `python3 setup.py deploy` after installing, as in the supplied
installation command. Local settings, calibration, model and data are retained.

Checks cover rendered layers, all four combinations, no-camera reference,
camera-only setup, setup-to-Connect ownership, hidden-view freshness checks,
disconnect and camera-open failure. Recorded/synthetic inputs and fake links
were used; no physical hardware was operated.

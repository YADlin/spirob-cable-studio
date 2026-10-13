# Connect and run from the Ubuntu workstation

For version 0.9.0 read [new model / references / reviewed campaigns](UPDATE_0_9_0.md), then use the complete [Operating manual](OPERATING_MANUAL.html), also available from the window. See [stable jogging, zoom and the reorganized layout](UPDATE_0_7_3.md), [separate camera/backbone switches](UPDATE_0_7_1.md) and [independent cable lengths and the two measurement modes](UPDATE_0_7_0.md). For analysis without automatic vision, select Manual points, capture a held pose, place points and sweep.

The code has been checked with recorded images and simulated motors. The new
Windows/WSL/USB route still needs your first hardware check. The camera stays on
Windows; attach only the CP2102 motor adapter to WSL.

## One-time setup

Finish any previous trial and inspect its shutdown acknowledgement. Close the
old motor GUI/controller, old live observer and Terminal A camera command.
Only one program should own the motor serial port and camera.

From the new `spirob-lab` directory on `yadu@wiprob`:

```bash
python3 setup.py import --from "$HOME/Research_Work/SpiRob_Analysis/spirob-analysis-pilot/rig.local.json"
python3 setup.py deploy
python3 setup.py unlock
python3 setup.py check
```

`import` copies your established addresses and paths. The only redirected paths
are the generated runtime: Windows `C:\Users\DELL\SpiRob_Simple_Runtime` and its
WSL `/mnt/c/...` equivalent. The previous runtime remains available.
`deploy` copies the small laptop adapter and privately copies the camera token
only if the workstation token is missing. It gracefully closes an authenticated idle camera service before deployment, and refuses to overwrite an active acquisition.
`unlock` prompts for the WSL camera key passphrase. It is not the Windows login
password. `check` reads the configuration without opening the motors or camera.
Expect your configured motor IDs, effective GUI reference lengths (initially 227.353314 mm for the new model), and the actual serial port to be present.

If USB is absent:

```bash
python3 setup.py usb-list
```

Identify the CP2102 (10c4:ea60) bus ID. For example, if its current ID is `2-3`:

```bash
python3 setup.py usb-attach --busid 2-3
python3 setup.py check
```

Use the actual current bus ID. Binding/admin setup was already done in your
working usbipd installation; these shortcuts do not change it.

Session duration, the difference limit, recording and pending-run review are
under **Session / data**. Motor commands are under **Motion**; rest preparation
is under **Jog / rest**. The top bar keeps Stop / Hold and Disconnect visible.
Use **Fit / capture** for stiffness settings, and **J / k₂ plots** for graphs.

## First live check

```bash
bash start.sh
```

1. Turn **Camera ON** and **MuJoCo backbone ON** for setup preview, then click **Connect**. There is no save decision at startup. The camera begins streaming without saving images while the motor
   connection starts with torque disabled. An SSH passphrase prompt, if needed,
   appears in the terminal used to launch the window. The camera and WSL command
   channels share that workstation SSH connection during the session.
2. Verify the video is current and the robot is in its declared physical rest.
   Check the confirmation box, then click **Enable at rest**. This does not home.
3. Hold still until the red MuJoCo backbone appears. Use **Alignment / cable mapping** to place the fixed base and verify tendon-to-motor correspondence; save that setup after checking it. Then wait for the settled score.
4. For a first small check, keep mean **220 mm**, full difference **2 mm**, speed **0.5 mm/s**. Check the
   displayed targets **221 and 219 mm**, then click **Move mean + difference**.
5. Observe the motion. Wait for the physical and virtual shapes to settle before
   interpreting the blue stiffness score. The gray error trace during motion is
   diagnostic; d₂ has not been fitted.
6. Enter difference **0 mm** and click **Move mean + difference** to return both cables to 220 mm.
7. Click **Disconnect**. Wait for motor torque release confirmation, then choose **Save**, **Discard** or **Decide later** for this run. Detach USB only after shutdown.

**Stop / Hold** interrupts a move and keeps torque. It does not return to rest.
If shutdown is unconfirmed, the window says so; inspect the laptop and session
`run.json` before assuming the motors are disabled.

Keep parameters fixed while checking a different deformation. Applying k₂ or d₂
changes only the virtual model. A stiffness sweep freezes one observation while
the video and motor controls remain active; its result is a historical comparison.

## Subsequent sessions

After the one-time setup, use `bash start.sh`. Run `python3 setup.py unlock` again
when the eight-hour key lease expires or WSL restarts. USB may need reattachment
after unplugging, rebooting or WSL shutdown. The program reports these failures;
it does not reinstall dependencies or start hardware movement to repair them.

Windows camera startup uses the same cached OpenCV environment and the working
`uv --system-certs run --offline` setting. Linux startup uses the supplied lockfile.
If `start.sh` reports a missing cached package, run `uv sync --frozen --extra live`
once on the workstation, then retry. If Qt specifically reports missing
`libxcb-cursor0`, install that Ubuntu library with `sudo apt install libxcb-cursor0`.

## Save and retrieve

All collected logs remain pending until you decide at the end of the run. Disconnect,
window close and timeout stop the hardware before showing **Save / Discard / Decide later**.
After a crash or lost connection, reopen the app and choose **Session / data → Review pending run data…**. Launch does not open an automatic review dialog.
Discard is unavailable until motor and camera shutdown is confirmed. A failed review
retains the remaining files for retry. Existing saved runs are not selected for deletion.
Preview stays in memory; Save cannot reconstruct video that was never recorded.
Click **Start recording** only when you want timestamped JPEGs saved, then **Stop recording**.
Recording has a cumulative **60-second / 250-MiB JPEG limit per session**; reaching a limit
stops saving images while preview continues. The **selected session limit** still applies.
Use `python3 setup.py diagnostics` to collect a small evidence ZIP without images.

The window reports the exact session name and path after shutdown. Fetch that
closed session using its reported name, for example:

```bash
python3 setup.py fetch YOUR_EXACT_SESSION_NAME
```

It arrives under this project's `data/`. Retained workstation comparisons are under `logs/`. Frozen sweeps stay in memory until **Save sweep…**, which writes PNG/CSV/JSON to the chosen path. A successfully discarded run cannot be fetched. No Drive or Git setup is required for this step.

## Rollback

Disconnect the new session and confirm motor shutdown. Close the new window.
Your previous `spirob-analysis-pilot`, `SpiRob_Lab_Runtime`, camera addon and motor
repository have not been replaced; their previous commands remain available.
Do not run both camera/controller workflows at once.


## Autonomous campaigns (0.8.0)

For the supplied 73-pose campaign, choose 30 min before Connect. After the normal
rest/enable steps, open Campaign → Plan / run campaign. Inspect Preflight and
Start. Automatic mode fits the shared k₂ after acquisition; manual mode queues
all retained images for one batch review afterward. Disconnect and Save retains
images, annotations and results. Full instructions: UPDATE_0_8_0.md.

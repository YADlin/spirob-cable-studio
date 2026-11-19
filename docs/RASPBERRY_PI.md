# Running the Lab with a Raspberry Pi instead of the laptop

The Pi takes over everything the laptop did: it owns the two RMCS-2303 drives and the overhead camera, runs the session coordinator (`actuation.py`) and the camera service, and keeps each run's data. The GUI on the workstation is unchanged: Connect, setup jogs, rest, length control, campaigns, recording and Save/Discard all work as before.

```
Workstation (GUI)  --SSH-->  Raspberry Pi: actuation.py  -> USB-RS485 -> RMCS-2303 x2
                             Raspberry Pi: spirob_live    -> USB camera
```

The Pi runs the same motor, safety and recording code the laptop ran; `setup.py deploy` copies it from the workstation, so the two never drift apart.

> **The Lab and `octa serve` cannot run at the same time**: both need the serial port and the camera. Stop Octa on the Pi before Connect (Ctrl-C, or `sudo systemctl stop octa`).

## One-time setup

All commands run on the workstation in `spirob-cable-studio/lab/` unless marked **Pi**.

1. **Passwordless SSH to the Pi.** The GUI cannot answer a password prompt.
   ```bash
   ssh-copy-id user@100.73.176.88
   ssh user@100.73.176.88 true        # must return without asking for a password
   ```
2. **Point the Lab at the Pi.** Keep the laptop settings so you can switch back.
   ```bash
   cp rig.local.json rig.laptop.json
   cp rig.pi.example.json rig.local.json
   ```
   Edit `rig.local.json` if your Pi user, address or folders differ:

   | Key | Meaning |
   |---|---|
   | `ssh_host` | `user@<pi-address>` |
   | `pi.root` | Folder on the Pi that holds the deployed Lab, motor code and `config.json` |
   | `pi.data_root` | Where the Pi keeps each run |
   | `pi.token_file` | Camera token on the Pi (setup creates it) |
   | `pi.device`, `pi.backend` | Camera: `0` and `v4l2` for `/dev/video0` |
   | `pi.width`, `pi.height`, `pi.fps` | Optional; default 1920 × 1080 at 30 fps |
   | `pi.min_fps` | Session stops if delivered frame rate falls below this (default 15) |
3. **Deploy.** This copies the Lab runtime, builds the Pi's Python 3.12 environment with uv from `lab/requirements-pi.txt` (no GUI packages), and installs the shared camera token on both machines.
   ```bash
   python3 setup.py deploy
   ```
4. **Motor configuration, from Octa's recipe card (Pi).** No old laptop file is needed: Octa writes the Lab's `config.json` from `~/robot.yaml` (drive IDs, port, payout signs, spool, travel window). Only the two rest lengths come from the Lab: on the workstation,
   ```bash
   python3 -c "import json;print(json.load(open('config.json'))['motion_profile']['rest_lengths_mm'])"
   ```
   then on the Pi, with those two numbers in the card's motor order (`m_left`, `m_right`):
   ```bash
   git -C ~/octa pull
   octa lab-config --card ~/robot.yaml --rest-mm 220 220
   ```
   It writes `~/spirob-lab/config.json` and prints which drive became axis 1 and 2.
5. **Check, without opening motors or camera:**
   ```bash
   python3 setup.py check
   ```
   The last line should be a `"type": "check"` reply with `"host": "pi"` and both axes.

## Every session

1. Make sure `octa serve` is not running on the Pi.
2. Start the Lab as usual and press **Connect**. The first camera start can take up to a minute.
3. After Disconnect, choose Save or Discard as before. To copy a kept run to the workstation:
   ```bash
   python3 setup.py fetch <session-folder-name>
   ```

## After changing Lab code

Run `python3 setup.py deploy` again. It refuses while a camera capture is active, and it never touches `config.json` or the run data on the Pi.

## Switching back to the laptop

```bash
cp rig.laptop.json rig.local.json
```

The laptop path is unchanged (`setup.py unlock`, `usb-attach` and the WSL/PowerShell steps still apply there; on the Pi they are not needed and say so).

## Troubleshooting

| Symptom | Fix |
|---|---|
| Connect hangs or asks for a password in the terminal | Step 1: `ssh-copy-id`, then `ssh user@<pi> true` |
| `Camera launch failed` | `python3 setup.py camera-status`; if an idle service is left over, `python3 setup.py camera-stop`. Is `octa serve` running? |
| `Motor USB unavailable` | Adapter unplugged, wrong port in the Pi's `config.json`, or the Pi user is not in `dialout` |
| `Camera delivery below configured minimum FPS` | Add `"width": 1280, "height": 720` to the `pi` section and deploy again |
| Session stops with `Workstation camera preview is stale` | Network too slow for the preview; prefer Ethernet or lower the resolution |

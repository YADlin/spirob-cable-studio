# SpiRob Lab 0.5.1

Install over 0.5.0. Disconnect and confirm motor shutdown, then close the app
before installing and deploying. The installer checks source hashes, backs up
changed files and prints a rollback command. Settings, calibration and data remain.

## Recover the backbone

1. Connect, establish physical rest, Enable at rest, and hold the cables still.
2. Open **Alignment / cable mapping…**. If needed, **Load supplied image alignment**
   then **Apply setup**. This preserves the selected cable mapping.
3. **Pick physical base in video**, click the fixed joint-1 anchor, then **Apply setup**.
4. Check the result and **Apply + save setup** for the next launch.

Projection edits preserve the simulated pose and time. Missing image measurements
or invalid scoring correspondences pause J while the independent model continues.
Initialization uses held encoder inputs. Stale input, missing time bracketing or
invalid physics still pause the prediction with a reason. An entirely off-image
setup is rejected before Apply.

Your uploaded run reports clean motor/camera shutdown but does not contain the
workstation analysis log or alignment. These code failure paths were reproduced;
the exact trigger on the rig remains unconfirmed.

## Keep or discard after the run

Disconnect, window close, faults and timeout stop hardware first. Then choose:

- **Save:** keep collected motor/analysis logs and explicitly recorded images.
- **Discard:** remove only this pending run after confirmed motor/camera shutdown.
- **Decide later:** keep the run pending. Closing the question does the same.

Preview stays in a bounded memory buffer. **Start recording** explicitly collects
footage under the cumulative 60-second / 250-MiB cap. Save cannot recreate
unrecorded video. Inspector and sweep exports retain separate Save buttons and
are outside run discard.

A crash, forced kill or power loss cannot show a question. Written files remain
pending and are offered on the next launch. **Review pending run data…** retries
after an unavailable laptop or interrupted decision. Review opens no hardware.
Unconfirmed shutdown disables Discard. Saved and pending runs accumulate until
reviewed; unanswered runs are never removed automatically.

## If a problem remains

Run `python3 setup.py diagnostics` and share the timestamped
`SpiRob_diagnostics_v051_….zip` from Downloads. It includes saved alignment and
bounded workstation/laptop logs, without images or credentials. It still collects
workstation diagnostics if the laptop is unreachable.

The Operating manual button contains the full workflow. Testing used recorded
images, simulated drives and synthetic cameras; the Windows/WSL rig check remains.

## Installer revision 2

The earlier updater incorrectly required generated checksum and verification
files from a fresh 0.5.0 installation. Prior updates left older copies in place.
Revision 2 preserves those local files and puts the 0.5.1 release reports under
`release_metadata/v0.5.1/`. Application-code checksum checks and backups remain.
Use `spirob-lab-v0.5.1-update-r2.zip` and its matching extracted folder; no force
flag or manual deletion is needed. This changes packaging only, not the runtime.

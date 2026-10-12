# SpiRob Lab 0.5.0

This update is for a working 0.4.0 installation. Disconnect and confirm motor
shutdown, then close the window before installing and deploying. The updater
checks source hashes, backs up changed files and prints its rollback command.
It preserves your rig settings, original scientific config, calibration, model
and all existing recordings. Saved display preferences use `lab.local.json`.

The Operating manual button contains the complete updated instructions.

1. Pick Light or Dark at the top right.
2. Connect and answer Save this run? No is suitable for the first alignment check.
3. Establish physical rest. Open Alignment / cable mapping. Pick the fixed
   physical joint-1 base, then Apply. Check scale/rotation using known geometry.
4. Select the swapped physical-to-MuJoCo cable mapping if a small individual
   movement demonstrates opposite virtual bending. Apply while holding. Save
   the setup after verification. Do not alter physical motor signs to correct
   the overlay.
5. Read physical demand, physical encoder length, model demand and model actual
   tendon length in the table. The model target follows encoder displacement.
6. In the inspector, correct physical point identities, then Use placement in
   live view. These become tip-relative spacing estimates along new image curves.
   Enable the optional J/sweep checkbox only after verifying the correspondence.
7. Set the k₂ interval and grid size, freeze a settled pose and sweep. The lowest
   sampled J/k₂ is selected; Apply sweep selection changes the simulation and
   displays acknowledgement. Save sweep writes PNG, CSV and JSON only on request.
8. Disconnect and inspect the saved/discarded outcome. No runs are removed only
   after confirmed shutdown; earlier saved experiments are never selected.

Saved setup is fixed across deformations. A transferred landmark profile is an
estimate, not a joint detector. The software cannot determine your physical base
or cable wiring without the short rig check. Recorded-image and simulated-drive
checks passed; no real Windows/WSL camera or physical motor was tested here.

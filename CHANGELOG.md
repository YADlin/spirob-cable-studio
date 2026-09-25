# Changes

## 3.1.0 — paired length controls

- Replace separate A/D apply actions with one target pair, mean/full-difference
  sliders, editable cable lengths and independent live feedback.
- Slider release optionally sends one finite paired move; dragging only previews.
  Numeric edits require Move both cables. Travel limits use each physical rest.
- Stage both low target words before committing high words back-to-back. Remove
  redundant enable writes in position mode; safely preload current position when
  resuming from STOP's torque-hold mode.
- Record and display the host gap between start-write requests. This reduces
  serial start delay but does not promise hardware-synchronized starts or arrival.
- Show full difference in the Trials table, retaining old half-difference values
  in saved procedures and logs through explicit conversion.
- This update is based on the version before the optional intermittent-diagnostics
  patch; that patch is not required or included. Communication fault policy is unchanged.

## Unreleased — interface contrast fix

- Apply a complete light Qt palette, including active, inactive and disabled states,
  instead of combining fixed dark text with inherited desktop-theme backgrounds.
- Cover dropdowns, selection highlights, tables and headers, diagnostics, menus,
  tooltips and Qt dialogs; keep disabled controls legible.
- Improve button text contrast and keyboard focus visibility. Motor control,
  limits, calibration, device IDs and recordings are unchanged.

## 3.0.0

- Two independently addressed motors, shared or separate serial ports; one owner per client.
- Independent numeric step/speed jogging, also before reference; explicit configurable setup envelope.
- Mean and half-difference commands, paired endpoint validation and failure handling.
- Immediate connection-frame plotting and a larger graph with optional error panel.
- Combined enable-at-rest workflow, reference-aware absolute lengths and unchanged 100:1 gearing.
- One/two-cable experiments, per-axis records, pair feedback skew and recorded-run review.
- Repository structure, pinned environment and GitHub Actions tests.

The release evolves the 2.1 single-cable Research Studio. It retains the established low-level RMCS register protocol but changes the multi-drive orchestration and reference model. Physical two-drive validation is still required.

# Changes

## 3.0.0

- Two independently addressed motors, shared or separate serial ports; one owner per client.
- Independent numeric step/speed jogging, also before reference; explicit configurable setup envelope.
- Mean and half-difference commands, paired endpoint validation and failure handling.
- Immediate connection-frame plotting and a larger graph with optional error panel.
- Combined enable-at-rest workflow, reference-aware absolute lengths and unchanged 100:1 gearing.
- One/two-cable experiments, per-axis records, pair feedback skew and recorded-run review.
- Repository structure, pinned environment and GitHub Actions tests.

The release evolves the 2.1 single-cable Research Studio. It retains the established low-level RMCS register protocol but changes the multi-drive orchestration and reference model. Physical two-drive validation is still required.

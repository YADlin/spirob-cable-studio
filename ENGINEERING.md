# Engineering record — 3.0.0

## Hardware/protocol basis

The project retains the working RMCS-2303 transport: Modbus ASCII, 9600 baud, 8 data bits, no parity, 1 stop bit, 0.3 s timeout and zero retries. The user's previous connection problem was an incorrect ID. IDs are entered/imported explicitly; a hardware ID is never inferred from the demo.

The supplied RMCS-2303 manual defines register 14 as base-motor RPM, position command low/high words at 16/18, actual-position words at 20/22, enable 0x0201, disable 0x0700 and stop/hold 0x0701 at register 2. The high command word commits the target. LPR must read 334 before enabling. The RMCS-5024 specification and user confirmation establish the 100:1 gearbox and 133600 output counts/revolution.

A shared port uses one PyModbus client and one worker. Different ports use separate clients owned by the same worker. All operations are sequential; no concurrent calls access a shared client. Distinct IDs alone do not make an electrically unsuitable wiring arrangement into a bus; use the established valid interface/wiring. No EEPROM, ID or gain writes are added.

## State and references

Each cable has DISABLED, HOLDING, MOVING, STOPPING or FAULT state. Its physical rest reference is a separate optional value. HOLDING without rest allows setup jogging. HOLDING with rest allows rest-relative targets; with a measured rest length it also allows absolute cable coordinates.

The setup origin is the first count on connection, and the configurable setup envelope defaults to ±70 mm. It persists through release/re-enable. The physical-rest envelope is always ±70 mm. No 0.5 mm step cap, ±2 mm window or 2 mm/s setup-speed cap remains. GUI step values are 0.01..2000 mm; backend acceptance depends on endpoint limits, finite arithmetic and signed count range. The UI does not make a non-moveable large option appear to work by clipping it.

Enable preloads the current stationary count before turning on the position loop. Set rest is an explicit physical reference action. Enable at rest waits for stable enabled feedback before capturing the entered physical lengths; Stop cancels any pending capture. Release/disconnect/fault invalidates physical references. Known power/reset events require re-referencing; a reset that produces a plausible nearby count may not be distinguishable from normal movement by host software.

## Coordinated commands and failure semantics

A/D transforms are invertible: A=(L1+L2)/2, D=(L1−L2)/2. A uses metres in the UI and millimetres in the backend/logs; D uses millimetres. Apply A preserves measured D at command processing; Apply D preserves measured A. Both axes must be stationary for pair requests.

All requested count endpoints, speeds and state requirements are checked before any target write. Speed writes are acknowledged and read back; verified unchanged speeds avoid redundant transactions. Cancellation during a speed write invalidates the cache. Targets then commit sequentially. A partial target failure triggers both-drive disable attempts and a latched fault. There is no atomic multi-drive commit, rollback, synchronized arrival or guarantee of constant A/D during transient motion. Combined A/D moves use the selected cable speed on each axis, not an interpolated trajectory.

Stop/release-all/disconnect are priority flags that clear the queued normal command. Cancellation is checked before register writes, between low/high target words and between axes. An in-flight serial transaction remains uninterruptible. Both stop/disable attempts are made even if the first fails. Logging failure cannot prevent the low-level emergency disable attempt. A serial stop/disable acknowledgement is not physical proof of stopped motion; stop settling uses encoder feedback, and no independent hardware safety function is added.

Read duration is limited to 0.65 s per axis. Enabled feedback gaps are limited to 2 s, allowing the two-drive serial sequence; the GUI heartbeat limit is 1.5 s. These are host software watchdog thresholds, not guarantees of stopping latency. Maximum cable speed is 20 mm/s and maximum base-motor command 2000 RPM; these are application ceilings, not validated load-specific ratings. Movement timeout and encoder-jump checks use applied RPM, circumference and measured elapsed time.

## Measurement and recording

Position words are read high/low/high to detect split-word rollover. The manual does not document an atomic snapshot. Each cable's acquisition start/end and raw counts are recorded; pair coordinates use the latest two readings with their skew. Host dispatch timing is recorded separately and must not be mistaken for physical motor-start skew.

Session schema 4 uses long-form sample rows with an axis index. Each row includes connection, rest and absolute coordinates where known. The connection frame is available immediately; absolute length is withheld until referenced. Reference epochs break rest-frame plot lines when reference changes. Curves retain at most 8000 samples per axis; CSV receives all acquired samples. Freeze affects drawing only. Offline legacy single-cable CSV reading remains supported.

A constant circumference assumes single-layer winding and no slip. Counts alone cannot measure stretch, tension, backlash, SpiRob shape or base-to-tip pose. Force sensing, camera feedback, hardware synchronized trajectories and continuous press-and-hold jogging are not included in this release.

## Validation

48 local tests passed using fake register replies, a controllable clock, two simulated motors and actual Qt widgets/event loops. Coverage includes:

- Counts/gear/sign arithmetic and complete turns; legacy configuration migration with no guessed second ID.
- One client on a shared port, two clients on separate ports and distinct-ID rejection.
- Immediate pre-enable plotting, >0.5 mm and >2 mm setup jogging, configurable envelopes and references surviving only as intended.
- Independent overlapping jogs, A/D examples, unequal-rest bounds and rejection of a bad second target before either write.
- Speed readback failure, stop during speed update, cancellation between commits, partial target failure, both-disable attempts and GUI-heartbeat loss.
- Enable-at-rest capture/cancellation, procedure validation/completion/abort, recording before rest and pair timestamp skew.
- Qt configuration import, manual/coordinate workflow, fault indication, review and a plot at least 430 pixels tall in a 1080 × 760 test window.

No hardware device was connected during development. Simulation is constant-speed and ideal; it does not model cable forces, acceleration, gearing compliance or friction. The GitHub Actions configuration is included but has not yet been executed on GitHub. The two-drive integration requires physical bench validation.

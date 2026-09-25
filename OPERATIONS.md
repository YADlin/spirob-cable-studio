# Operating and understanding the two-cable workstation

## First understand the references

The encoder measures motor turns. It does not know cable length, the SpiRob rest configuration or whether the cable is tight. The software needs a physical reference to translate counts into absolute length.

1. **Connection origin:** the first encoder count after opening the drive. The plot immediately shows travel relative to this point, even before enabling. The origin does not assert that the robot is at rest.
2. **Physical rest:** a stationary configuration that you identify. Set rest records its count. The ±70 mm motion window is then centred on this count.
3. **Physical rest length:** the cable length you measure at that configuration. Enter this in `Rest L` in millimetres. A value of 220 mm is only correct if it corresponds to your actual reference length. Consistently define which cable path/endpoints you are measuring.

If the reference count is 300000 and the reference length is 220 mm, advancing by one payout millimetre gives estimated length 221 mm, regardless of whether the raw count is positive or negative. Each motor's payout sign handles the winding direction.

## Use jogging to set up the mechanism

1. Close other motor-control programs. In Setup, import the old config. Verify cable 1's ID; choose Two cables and enter cable 2's different ID and actual port.
2. Check both spool calibrations and payout signs. The same physical cable motion can correspond to opposite encoder signs on mirrored motors.
3. Read the setup envelope. Default ±70 mm means 70 mm either way from the connection position; the previous hidden ±2 mm rule has been removed. Change the setup envelope before connection if your setup requires a different range.
4. Connect. The motors are disabled and both encoder traces are visible. A stationary trace is a horizontal line, not missing data.
5. Enable the cable you want to move. Enter its jog distance and speed. Rest need not be set. Try a small movement first to verify the physical direction, then select a larger step as appropriate.
6. Jog either cable independently. If a requested endpoint exceeds the displayed travel window, it is rejected instead of silently shortened.
7. Once the mechanism is at the intended rest and preload, enter the measured cable rest lengths and press each Set rest here. For an already-resting mechanism, the global Enable at rest button combines enabling with stationary reference capture.

The old 0.5 mm cap was a commissioning choice, not a motor requirement. The old ±2 mm limit was also a software setup guard, measured from the enable position. This release removes both and displays the replacement envelope and its origin explicitly.

Enabling torque and defining a length reference remain distinct physical concepts. Combining their buttons does not eliminate the need to know whether the robot is actually at rest. Do not use Set rest merely to bypass a rejected target; establish the intended physical configuration.

## Calculate the conversion once

For a 27 mm bare spool and a single layer of 0.405 mm cable:

```text
Effective cable-centreline diameter = 27 + 0.405 = 27.405 mm
Cable per spool revolution = π × 27.405 = 86.09535 mm
Motor encoder counts/revolution = 334 × 4 = 1336
Spool counts/revolution = 1336 × 100 = 133600
Cable mm/count = 86.09535 / 133600
```

One cable radius is added on each side of the core; together they add one cable diameter. If 27 mm already measures the cable centreline diameter, select Cable centreline and do not add 0.405 again. If the cable stacks in layers, this constant-radius model becomes inaccurate.

The calculation deliberately keeps complete turns. A 140 mm span is about 1.626 spool revolutions. Reducing the angle modulo 360 degrees would lose cable travel.

Register 14 is base-motor RPM:

```text
motor RPM = floor(cable speed mm/s × 60 × 100 / mm per spool revolution)
output RPM = motor RPM / 100
```

At 5 mm/s the command is 348 motor RPM, 3.48 output RPM and nominally 4.9935 mm/s after rounding. The gear ratio appears once in speed conversion and once in the count scale; do not divide either result by 100 again.

## Mean controls common length; difference controls redistribution

Imagine L1 and L2 as two numbers on rulers. Their centre is the mean. Their separation is the full difference displayed in the Lengths tab:

```text
Mean = (L1 + L2) / 2
Difference = L1 - L2
L1 = Mean + Difference/2
L2 = Mean - Difference/2
```

A change in mean adds the same amount to both target lengths. Increasing difference by 2 mm adds 1 mm to cable 1 and subtracts 1 mm from cable 2. These are cable coordinates; how the SpiRob moves in space must be observed or modelled separately.

In the demo:

1. Connect, then Enable at rest. Both simulated rest lengths are 220 mm.
2. Open Lengths. Mean 0.220 m and difference 0 give equal 0.220 m targets. Numeric edits preview; Move both cables executes.
3. Set difference to +20 mm. Targets show 0.230 and 0.210 m. Move both cables, or use the slider with Move when a slider is released selected.
4. Set mean to 0.180 m. The entered difference stays +20 mm, so targets show 0.190 and 0.170 m.
5. Set difference to −10 mm. Targets show 0.175 and 0.185 m.
6. Directly enter targets 0.240 and 0.200 m. Mean updates to 0.220 m and difference to +40 mm.
7. Equal lengths sets the difference to zero at the entered mean. Click Move both cables to execute. Copy live lengths also only changes the preview.
8. Mean 0.180 m with difference +80 mm is rejected because cable 2 would be 0.140 m, below its 0.150 m lower bound.

These examples interpret 0.22/0.18 as metres. Live values come from encoders and spool calibration; targets are desired lengths, not measurements. Sliders step by 0.1 mm, while numeric fields allow finer entry subject to encoder quantization. Rest capture initializes the editors from both live lengths, including unequal lengths. Later feedback updates live displays without replacing your draft.

With unequal physical rests, bounds are not symmetric about zero difference. At a proposed mean A, in millimetres:

```text
Difference_min = 2 * max(L1_rest − 70 − A, A − L2_rest − 70)
Difference_max = 2 * min(L1_rest + 70 − A, A − L2_rest + 70)
```

If the minimum exceeds the maximum, no pair at that mean fits both windows. Slider bounds show the permissible interval. Invalid numeric targets disable Move both cables. Each slider requires both cables enabled, referenced and stationary; you do not build up a queue of moves while dragging.

The Trials table also displays full difference. Existing procedure files and research logs retain the earlier HALF-difference D in millimetres; load/save converts automatically, so existing files keep their meaning.

## Why the path is not exactly synchronized

Both endpoint counts and speed settings are checked before dispatch. Both low target words are written first; their high words then trigger the moves in quick succession. This removes an intervening low-word write and redundant enable write from the gap between starts. It does not wait for cable 1 to arrive before starting cable 2. If cable 1 accepts its commit and cable 2 then fails, cable 1 may already have begun moving. The fault response attempts to disable both drives and clears both references. There is no software rollback of physical motion.

The panel shows the host time between the two commit requests, also recorded in pair_dispatched events. This is not an encoder-measured start delay. At 9600 baud the remaining write-and-acknowledgement transaction normally takes tens of milliseconds; very short moves may still finish within that interval. Physical validation is necessary to measure actual overlap.

Equal cable speed commands also do not guarantee equal durations for different travel distances. This application does not yet generate a common time-parameterized trajectory or provide a hardware synchronized start. Use the graphs and per-axis timestamps to quantify transient mean/difference error if your experiment depends on it.

## Calibrate from a physical measurement

In Setup, capture stationary encoder count A, move a feasible amount, capture count B and enter the independently measured cable travel magnitude. The calculated coefficient is:

```text
mm per spool revolution = measured travel × 133600 / abs(B − A)
```

Apply a coefficient only after disconnecting, then save, reconnect and re-reference. Repeat in payout and take-in directions. Backlash, stretch and slack cannot be corrected simply by treating the commanded travel as a measurement.

## Reading the source

Read `model.py` for units, `config.py` for internal A/D conversion, `drive.py` for stage/commit transactions, `rig.py` for movement authorization, `worker.py` for sequencing and stop priority, `records.py` for timing/data, `pair_controls.py` for the target editor, then `ui.py` for integration. `procedure.py` implements move–settle–hold experiments. `analysis.py` and `review.py` contain offline CSV operations.

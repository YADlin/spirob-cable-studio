# What the comparison means

## The mechanical hypothesis

The XML is a multi-link approximation to the printed flexible robot. Joint 1
keeps its boundary settings (k₁=100 N m/rad, d₁=50 N m s/rad). Joint 2 is the
reference flexible joint. For i≥2:

- kᵢ = k₂ · rₖ^(i−2)
- dᵢ = d₂ · r_d^(i−2)

The supplied XML has rₖ≈r_d≈0.9151416594. The code reads these ratios and verifies
the exponential sequence. It does not fit each joint independently.
The current conditional candidate is k₂≈0.8115955 N m/rad. d₂ remains the original
XML value, approximately 0.0091514166 N m s/rad; it has not been identified.

## Optional profile-shape review (0.7.3)

The default above is unchanged. Optional review extends its notation to
`k_i = k_2 × r^[b(i−2)]`, with b=1 by default. The XML supplies r, not separate
original generator constants. Here b is an explicitly introduced exponent
multiplier. The effective adjacent ratio is r^b, so the two shape constants
cannot be independently identified; only one is swept alongside k₂ at a time.
An extra amplitude multiplier would duplicate k₂ and is not fitted.

The surface uses one immutable observation from the completed 1D sweep. Image
points, measured cable inputs, transform, cable offsets and the other profile
constant stay fixed. Joint 1 and damping remain unchanged. Each grid point is
solved independently from the original spring reference; failed equilibria have
null cost and an error string. They are excluded from minimum selection.

The display reports the lowest valid sample within the requested bounds, flags
boundary minima and reports missing cells. It does not establish a global
minimum, confidence interval, multi-pose fit or unique material parameters.
Exports contain the full grid, exact observation and source hashes. Applying a
selected profile is explicit and changes only virtual joint stiffness. Normal
1D sweeps subsequently use the acknowledged r/b values as fixed constants.

## What enters MuJoCo

The physical motors receive requested target lengths. The simulation receives
**encoder-derived cable displacement histories**, so a motor's actual progress,
rather than instantaneous arrival at its command target, drives the model.
Encoder-derived length assumes the configured spool geometry and no cable slip;
it is not a direct optical or tension measurement of free cable length.

The XML neutral tendon length is not exactly the declared physical rest length.
The selectable cable mapping subtracts each physical rest first, then reorders and optionally reverses the displacement signs in model-tendon order. Therefore `model.controls()` maps displacement from physical rest onto the XML's
neutral tendon lengths, with the existing fixed cable-zero offsets. It does not
insert 220 mm as an absolute XML tendon length.

The model keeps the supplied orientation and gravity. It assumes free movement,
with no board contact. Camera mapping and cable offsets remain frozen when testing
a different deformation. The measured backbone never corrects simulated joint angles.

## The cost

For N corresponding points in the camera plane:

J = Σᵢ [(x̂ᵢ−xᵢ)² + (ŷᵢ−yᵢ)²] / N

Units: px² per point. RMSE = √J, in pixels. Division is by N points, not 2N
coordinates. The original least-squares fitter uses coordinate residuals; for
a fixed number of points, minimizing their sum of squares gives the same minimum
as minimizing J.

The live extractor uses colour thresholds inside a fixed region, follows the
skeleton from the base and samples fixed distances back from the tip. Neither
pixel selection nor camera alignment depends on the selected k₂. It rejects
frames with missing components or implausible backbone length. Its point selection
is different from the original guided offline extraction, so their RMSE values
are not numerically interchangeable.

## What the original static fit optimized

There are two nested calculations. Neither is polynomial curve fitting.

1. **Mechanical equilibrium for a proposed k₂.** `spirob_analysis/model.py` minimizes
   the robot's static potential energy using bounded L-BFGS-B, with the generalized
   force residual supplying the gradient. A root solve can refine stationarity.
   Excessive force residual, invalid state or contact rejects the equilibrium.
2. **Agreement with held images.** `spirob_analysis/fitting.py` uses bounded
   nonlinear least squares (`least_squares`, TRF, linear loss). The original fit
   adjusted log(k₂), differential cable offset, image scale, rotation and translation.
   The mean cable offset stayed fixed. Camera transformation was shared across poses.

That fit produced a useful candidate under the chosen cable-zero convention.
It did not establish a unique material stiffness independent of tendon offsets.
The alignment and offsets from it are now frozen during live cross-checks.

The original offline workflow remains available:

```bash
uv run --offline --frozen python -m spirob_analysis demo --out results_refit
```

This extracts the supplied held poses, repeats the fit and writes its report.
It does not connect to hardware. The recorded live-window example and its sweep
provide a shorter first lesson; use `science_walkthrough.py` for actual residual arithmetic.

## What the live window computes

It initializes at mechanical equilibrium after observing a stationary physical
pose, then integrates MuJoCo through the measured input history at the XML timestep
(0.0001 s in this model). It displays a moving backbone, not a sequence of newly
fitted shapes. Parameter edits preserve simulated positions and velocities.

The gray instantaneous error can be displayed during motion. The blue stiffness
score requires all of these conditions:

- Fresh image and encoder samples, with encoder samples bracketing image time.
- Motor states HOLDING and cable-length variation ≤0.15 mm over the hold window.
- Measured point movement ≤2 px over the two-second window.
- Simulated point movement ≤2 px over the same duration, and all |q̇|<0.01 rad/s.

These are practical tolerances, not a claim of mathematically exact zero velocity.
After a data gap the simulated state is reinitialized at a subsequent held pose;
the missing interval cannot be used for damping identification.

A frozen stiffness sweep evaluates a visible grid of k₂ values for one held
observation. Each uses an equilibrium solve with the same frozen alignment and
cable offsets. The curve is a conditional single-pose cost, not a confidence
interval or an automatic multi-pose refit. It selects the lowest sampled value; the explicit Apply action changes live k₂. The interval and log-grid size are user inputs, and a boundary minimum is flagged.

## Timing and damping

Camera timestamps are host read-call times. The system estimates the WSL-to-Windows
clock offset and brackets each image with motor samples; exposure latency, USB
buffering and long-session clock drift remain imperfectly known. Displaying an
image and its prediction together does not remove that acquisition uncertainty.

The camera recording minimum remains **15 fps**. The display polls up to 15 Hz
and runs at the speed of image measurement and simulation. Actual image age and
processing duration are shown. Display rate is not recording rate, and the
software does not claim hard real-time synchronization.

Damping influences transient simulation. A visually plausible overlay is an
exploratory cross-check; the present version does not optimize d₂ or claim validated
damping. A later damping experiment needs a fixed k₂, uninterrupted input history,
useful transients and a timing treatment appropriate to their frequencies.

## Optional verified landmark objective

The inspector can transfer image-only tip-relative arc distances. Each live image
is sampled again at those distances, giving estimates of the same physical joints
or element centres. Actual MuJoCo joint anchors/midpoints supply the corresponding
predictions. Enable this objective explicitly; otherwise default samples are used.
A required point outside the visible curve pauses scoring. Frozen clicks are never
replayed as fixed x/y coordinates, and model predictions never select image points.
This assumes a valid measured centreline, stable scale and tip, and approximately
preserved projected arc positions. Out-of-plane motion, camera movement, extension
or incorrect joint identity can invalidate it. Inspect a second held pose before
using a transferred profile for stiffness selection. The old and new objectives
have different point sets; compare J only with that distinction recorded.


## Campaign shared and pose-specific k₂ (0.8.0)

Campaign fitting freezes all nuisance parameters and evaluates a common k₂ grid.
The shared objective is the equally weighted mean of the per-pose mean squared
pixel errors. Invalid equilibria at a candidate k₂ exclude that candidate for
the entire shared set; missing physical observations instead produce a labelled
incomplete/subset result. Per-pose minima use the exact same observations and
candidate grid after the shared baseline. Repeated references carry equal pose
weight. Compare consistent landmark sets and account for mean cable length,
direction/history and marking uncertainty before interpreting effective fitted
k₂ as intrinsic stiffness. The estimator does not infer damping, validate a
constitutive law or declare a fit acceptable from an arbitrary threshold.
UPDATE_0_8_0.md documents acquisition, deflection fields and reproducibility.

"""Inspectable static outputs; no renderer or GPU is needed for the fitting pipeline."""
import html
import json
from pathlib import Path
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import numpy as np
from PIL import Image


def report(result, observations, out, image_dir=None):
    out = Path(out)
    image_dir = Path(image_dir or out)
    obs = {o['step']: o for o in observations['observations']}
    candidate, baseline = result['candidate'], result['baseline']
    fig, axes = plt.subplots(2, 2, figsize=(14, 8.8), layout='constrained')
    for ax, pose in zip(axes.flat, candidate['poses']):
        step = pose['step']
        ax.imshow(Image.open(image_dir / f'pose_{step:03d}.jpg'))
        measured = np.array(obs[step]['points_px'])
        curve = np.array(pose['predicted_curve_px'])
        ax.plot(curve[:, 0], curve[:, 1], color='#ffae31', lw=1.5, label='MuJoCo candidate curve')
        ax.scatter(measured[:, 0], measured[:, 1], color='#58ffd0', s=7, label='Image curve samples')
        displacement = obs[step]['displacement_mm']
        role = 'fit pose' if pose['used_for_fit'] else 'return-pose check'
        ax.set_title(f"Step {step}: cable changes {displacement[0]:+g} / {displacement[1]:+g} mm\n"
                     f"{role}; point RMS {pose['point_rmse_px']:.1f} px", fontsize=11)
        ax.set_xlim(350, 1500)
        ax.set_ylim(1000, 0)
        ax.axis('off')
    axes.flat[0].legend(loc='lower left', fontsize=8, facecolor='#273544', labelcolor='white')
    p = candidate['parameters']
    fig.suptitle(f"SpiRob static pilot | k₂ = {p['k2']:.4g} N m/rad | fixed joint-to-joint scaling", fontsize=15)
    fig.savefig(out / 'static_overlay.png', dpi=150)
    plt.close(fig)

    profile = result['stiffness_profile']
    fig, ax = plt.subplots(figsize=(8, 4.8), layout='constrained')
    ax.semilogx([r['k2'] for r in profile], [r['fixed_mean_rmse_px'] for r in profile], 'o-',
                label='Common cable offset fixed at 0 mm')
    ax.semilogx([r['k2'] for r in profile], [r['free_mean_rmse_px'] for r in profile], 's--',
                label='Common cable offset also fitted')
    ax.axvline(result['model_audit']['original_k2_Nm_per_rad'], ls=':', color='gray', label='Uploaded XML k₂')
    ax.set(xlabel='Joint 2 stiffness k₂ (N m/rad)', ylabel='2D point RMS error (native pixels)',
           title='Sensitivity to k₂ and cable-zero assumptions')
    ax.grid(alpha=.2)
    ax.legend(fontsize=9)
    fig.savefig(out / 'stiffness_sensitivity.png', dpi=160)
    plt.close(fig)

    audit = result['model_audit']
    rows = ''.join(f"<tr><td>{x['step']}</td><td>{'Fit' if x['used_for_fit'] else 'Return check'}</td>"
                   f"<td>{x['point_rmse_px']:.2f}</td></tr>" for x in candidate['poses'])
    document = f'''<!doctype html><meta charset="utf-8"><title>SpiRob static pilot</title>
<style>body{{max-width:1100px;margin:36px auto;padding:0 24px;font:17px/1.55 system-ui;color:#193046}}
h1,h2{{line-height:1.2}}img{{max-width:100%}}table{{border-collapse:collapse}}td,th{{padding:8px 18px;border-bottom:1px solid #ccd4dc;text-align:left}}
code{{background:#edf2f6;padding:2px 5px}}.note{{background:#eef5f9;padding:16px;border-left:4px solid #4386a8}}</style>
<h1>SpiRob: first static comparison pipeline</h1>
<p>Session {html.escape(observations['session_id'])}. These are approximate markerless shape measurements from the existing completed trial.</p>
<p><b>Parameter convention:</b> joint 2 is the first flexible reference joint.
For i ≥ 2, kᵢ = k₂ × {audit['stiffness_ratio']:.8f}<sup>i−2</sup> and dᵢ = d₂ × {audit['damping_ratio']:.8f}<sup>i−2</sup>.
Joint 1 stays at stiffness {audit['joint1_stiffness']:g} and damping {audit['joint1_damping']:g}.</p>
<table><tr><th>Quantity</th><th>Value</th></tr>
<tr><td>Uploaded XML k₂</td><td>{audit['original_k2_Nm_per_rad']:.8g} N m/rad</td></tr>
<tr><td>Conditional candidate k₂</td><td>{p['k2']:.8g} N m/rad</td></tr>
<tr><td>d₂ (preserved, not fitted)</td><td>{audit['original_d2_Nm_s_per_rad']:.8g} N m s/rad</td></tr>
<tr><td>Uploaded stiffness, fitted alignment/offset</td><td>{baseline['point_rmse_px']:.2f} px point RMS</td></tr>
<tr><td>Candidate stiffness, fitted alignment/offset</td><td>{candidate['point_rmse_px']:.2f} px point RMS</td></tr>
<tr><td>Shared differential cable offset</td><td>{p['difference_offset_mm']:.3f} mm</td></tr>
<tr><td>Shared common cable offset</td><td>{p['mean_offset_mm']:.3f} mm (fixed assumption)</td></tr></table>
<p>Both scores use a single camera similarity transform shared across the three fitting poses; each candidate may refit that shared transform. There is no independent alignment per pose.</p>
<img src="static_overlay.png" alt="Measured silhouette samples and simulated curves for four poses">
<table><tr><th>Step</th><th>Use</th><th>Point RMS (px)</th></tr>{rows}</table>
<h2>What this pilot establishes</h2>
<p>The data-to-image-points-to-MuJoCo-to-cost-to-XML path runs end to end. The same exponential joint pattern can be assessed against both bending directions. The return pose is retained as a repeatability check, not a new actuation level.</p>
<div class="note"><b>Interpret the candidate as a working model, not a uniquely measured material stiffness.</b>
The common cable-zero offset and k₂ can trade off. The second curve below demonstrates that directly. Camera scale, cable compliance, actuator position gain, geometry and the fixed scaling law are also conditional assumptions. A close fit on these poses does not establish policy-transfer performance.</div>
<img src="stiffness_sensitivity.png" alt="Stiffness sensitivity with fixed and free common cable offset">
<h2>Measurement and physics details</h2>
<p>The green samples come from a thresholded robot silhouette and a smoothed medial path, with explicit ROIs and endpoint seeds in config.json. They are approximate shape samples, not fiducial markers or anatomical joint axes. Correspondence is based on distance along the curve from its distal cap. Inspect extraction_*.png and mask_*.png before reusing on another scene.</p>
<p>Lengths map as neutral simulated tendon length + recorded cable displacement + shared offsets. The two neutral simulated lengths are {audit['neutral_tendon_lengths_m'][0]*1000:.3f} mm. The physical session's declared 220 mm zero is not asserted as a calibrated geometric length.</p>
<p>The uploaded robot orientation, gravity, masses, contacts, position-actuator gains and joint-1 values are retained. The static backend minimizes the free-space potential using MuJoCo tendon geometry and generalized forces. Solutions with contact or excessive residual force are rejected. A time-integration check verifies the accepted equilibrium separately.</p>
<p>Damping remains at the uploaded values. Its scaling is exposed for trajectory replay, but static images cannot fit it. <a href="https://mujoco.readthedocs.io/en/3.3.7/computation/index.html">MuJoCo force-model reference</a>.</p>
<h2>Files</h2><p><a href="fit.json">Complete fit</a> · <a href="observations.json">Image measurements</a> · <a href="joint_parameters.csv">Per-joint values</a> · <a href="stiffness_profile.csv">Sensitivity sweep</a> · <a href="candidate_model/spirob_robot.xml">Candidate XML</a></p>
'''
    (out / 'report.html').write_text(document)

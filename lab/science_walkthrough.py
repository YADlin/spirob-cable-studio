"""A short, executable lesson: one observation -> one mechanical prediction -> one cost.

Run: uv run python science_walkthrough.py
Read this file top to bottom. It does not connect to the robot or fit camera alignment.
"""
import csv
import json
from pathlib import Path
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import numpy as np
from spirob_analysis.model import SpiRobModel
from spirob_analysis.fitting import predict

config = json.loads(Path('config.json').read_text())
fit = json.loads(Path('calibration/pilot_fit.json').read_text())
observations = json.loads(Path('results/observations.json').read_text())['observations']
model = SpiRobModel(config['model'])
parameters = dict(fit['candidate']['parameters'])
output = Path('results_walkthrough'); output.mkdir(exist_ok=True)

# 1. The fitted scalar applies to joint 2. Later joints inherit fixed XML ratios.
k2 = parameters['k2']
print('Joint 1 stays:', model.original_k[0], 'N m/rad and', model.original_d[0], 'N m s/rad')
for joint in range(2, 6):
    print(f'Joint {joint}: k = {k2:.6g} * {model.rk:.6g}^{joint-2} = {k2*model.rk**(joint-2):.6g}')

# 2. Use the +20 mm difference held image. v0.1 used this observation in its fit.
observation = next(o for o in observations if o['step'] == 2)
observed = np.array(observation['points_px'])
print('Cable displacement from declared rest [mm]:', observation['displacement_mm'])

# 3. predict() first finds mechanical equilibrium, then projects it into camera pixels.
predicted, curve, state, coverage = predict(model, observation, parameters)
assert max(coverage) < .01
residual = predicted-observed
squared_distance = np.sum(residual**2, axis=1)
J = np.mean(squared_distance)
print('First point observed [px]:', observed[0])
print('First point predicted [px]:', predicted[0])
print('First point residual [px]:', residual[0])
print(f'First squared distance = dx² + dy² = {squared_distance[0]:.4f} px²')
print(f'J = sum of the {len(observed)} squared distances / {len(observed)} = {J:.4f} px²/point')
print(f'RMSE = sqrt(J) = {np.sqrt(J):.4f} px')
print('Equilibrium force residual [N m]:', state['residual_Nm'])

# 4. A deliberately simple grid search. Only k2 changes; all other values stay fixed.
# This is a teaching experiment, not a repeat of the joint camera-and-k fit.
ks = np.unique(np.r_[np.geomspace(.02, 3, 35), k2])
rows = []
for k in ks:
    p = dict(parameters, k2=float(k))
    values = []
    for pose in observations:
        points, _, _, penalty = predict(model, pose, p)
        assert max(penalty) < .01
        values.append(float(np.mean(np.sum((points-np.array(pose['points_px']))**2, axis=1))))
    rows.append([float(k), *values])
with (output/'fixed_alignment_sweep.csv').open('w',newline='') as stream:
    writer=csv.writer(stream);writer.writerow(['k2','J_step1','J_step2','J_step3','J_step4']);writer.writerows(rows)
fig,ax=plt.subplots(figsize=(8,4.5))
for i,label in enumerate(('Initial rest','+20 mm difference','−20 mm difference','Return rest'),1):
    ax.semilogx(ks,[r[i] for r in rows],label=label)
ax.axvline(k2,color='black',linestyle=':',label='Current multi-pose candidate')
ax.set(xlabel='k2 [N m / rad]',ylabel='Mean squared point error J [px² / point]',
       title='Change one physical parameter; keep the camera mapping fixed')
ax.grid(alpha=.2);ax.legend(fontsize=8);fig.tight_layout()
fig.savefig(output/'fixed_alignment_sweep.png',dpi=150)
print('Wrote',output/'fixed_alignment_sweep.csv','and',output/'fixed_alignment_sweep.png')
print('Next: edit one k value or choose another observation and predict the effect before running it.')

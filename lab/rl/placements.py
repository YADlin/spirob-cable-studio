"""Hardware evaluation placements in frame C [m] (inside the simulation evaluation region).

Evaluation region: |u| in [0.050, 0.100], v in [0.060, 0.170]. Twelve placements,
six per side, covering near/far and inner/outer corners. Planned 30 armed episodes:
P01-P06 x3 and P07-P12 x2 (or the reverse), fixed before any armed trial.
"""
PLACEMENTS = {
    "P01": (+0.055, 0.070), "P02": (+0.095, 0.070), "P03": (+0.075, 0.115),
    "P04": (+0.095, 0.115), "P05": (+0.055, 0.160), "P06": (+0.095, 0.160),
    "P07": (-0.055, 0.070), "P08": (-0.095, 0.070), "P09": (-0.075, 0.115),
    "P10": (-0.095, 0.115), "P11": (-0.055, 0.160), "P12": (-0.095, 0.160),
}
TARGET_TOLERANCE_M = 0.005

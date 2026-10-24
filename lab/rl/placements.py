"""Hardware evaluation placements in frame C [m] (contract v2; inside the evaluation region).

Evaluation region: |u| in [0.050, 0.130] m, v in [0.060, 0.170] m (spirob-mjlab env_cfg EVAL_*).
Six placements per side: near/far rows x inner/outer columns plus the centre-outer point.
"""
PLACEMENTS = {
    "P01": (+0.055, 0.070), "P02": (+0.125, 0.070), "P03": (+0.090, 0.115),
    "P04": (+0.125, 0.115), "P05": (+0.055, 0.160), "P06": (+0.125, 0.160),
    "P07": (-0.055, 0.070), "P08": (-0.125, 0.070), "P09": (-0.090, 0.115),
    "P10": (-0.125, 0.115), "P11": (-0.055, 0.160), "P12": (-0.125, 0.160),
}
TARGET_TOLERANCE_M = 0.005

"""Dependency-light deterministic actor for deployment (NumPy only).

Copied verbatim into the SpiRob Lab as ``lab/rl/numpy_policy.py``. Loads the
``policy.npz`` written by ``tools/knockdown/export_policy.py`` and reproduces
RSL-RL's deterministic actor output:

    a = MLP((obs - mean) / (std + eps))      (Gaussian head: mean action)
"""

from __future__ import annotations

import hashlib
import json
from pathlib import Path

import numpy as np

ACTIVATIONS = {
    "elu": lambda x: np.where(x > 0, x, np.expm1(np.minimum(x, 0.0))),
    "tanh": np.tanh,
    "relu": lambda x: np.maximum(x, 0.0),
}


class NumpyPolicy:
    def __init__(self, npz_path: str | Path):
        path = Path(npz_path)
        self.sha256 = hashlib.sha256(path.read_bytes()).hexdigest()
        with np.load(path, allow_pickle=False) as z:
            self.mean = z["obs_mean"].astype(np.float64)
            self.std = z["obs_std"].astype(np.float64)
            self.eps = float(z["obs_eps"])
            meta = json.loads(str(z["meta_json"]))
            self.layers = [(z[f"W{i}"].astype(np.float64), z[f"b{i}"].astype(np.float64))
                           for i in range(meta["n_layers"])]
        self.activation = ACTIVATIONS[meta["activation"]]
        self.obs_dim = int(meta["obs_dim"])
        self.act_dim = int(meta["act_dim"])

    def __call__(self, obs) -> np.ndarray:
        x = np.asarray(obs, dtype=np.float64)
        if x.shape[-1] != self.obs_dim:
            raise ValueError(f"Observation has {x.shape[-1]} values; policy expects {self.obs_dim}")
        if not np.isfinite(x).all():
            raise ValueError("Non-finite observation rejected")
        x = (x - self.mean) / (self.std + self.eps)
        for i, (W, b) in enumerate(self.layers):
            x = x @ W.T + b
            if i < len(self.layers) - 1:
                x = self.activation(x)
        if not np.isfinite(x).all():
            raise ValueError("Non-finite policy output rejected")
        return x

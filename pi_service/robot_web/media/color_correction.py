"""Radial BGR edge-colour correction independent of camera lifecycle."""
from __future__ import annotations

from .profiles import EDGE_BGR_GAINS, RADIAL_FALLOFF_EXPONENT


def build_radial_gain_map(width: int, height: int, strength: float):
    import numpy as np

    x = np.linspace(-1.0, 1.0, int(width), dtype=np.float32)
    y = np.linspace(-1.0, 1.0, int(height), dtype=np.float32)
    radius = np.minimum(
        1.0,
        np.sqrt(y[:, None] ** 2 + x[None, :] ** 2) / np.sqrt(2.0),
    )
    falloff = radius ** RADIAL_FALLOFF_EXPONENT
    edge = np.asarray(EDGE_BGR_GAINS, dtype=np.float32).reshape(1, 1, 3)
    gain = 1.0 + (edge - 1.0) * (falloff[..., None] * float(strength))
    return gain.astype(np.float32)

import numpy as np
from dataclasses import dataclass, field
from typing import Any, Dict


@dataclass
class WithCovariance:
    value: np.ndarray
    covariance: np.ndarray


@dataclass
class AuxData:
    data: Dict[Any, Any] = field(default_factory=dict)

    def get(self, key, default=None):
        if self.data is None:
            return default
        return self.data.get(key, default)


import numpy as np


def inv(x):
    x = np.asarray(x)

    # Scalar
    if x.ndim == 0:
        return 1.0 / x

    # 1D array: elementwise inverse
    elif x.ndim == 1:
        return 1.0 / x

    # 2D+ array: matrix inverse or batch inverse
    elif x.shape[-2:] == (1, 1):  # Treat 1x1 as scalar
        return 1.0 / x[..., 0, 0]

    else:
        try:
            return np.linalg.inv(x)
        except np.linalg.LinAlgError:
            # Fallback for singular/ill-conditioned matrices
            return np.linalg.pinv(x)


def skew(v):
    v = v.flatten()
    return np.array([[0, -v[2], v[1]], [v[2], 0, -v[0]], [-v[1], v[0], 0]])

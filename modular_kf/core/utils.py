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


def skew(v):
    v = v.flatten()
    return np.array([[0, -v[2], v[1]], [v[2], 0, -v[0]], [-v[1], v[0], 0]])

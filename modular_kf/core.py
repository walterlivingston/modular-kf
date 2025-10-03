import numpy as np
from dataclasses import dataclass
from typing import Any


@dataclass
class WithCovariance:
    value: np.ndarray
    covariance: np.ndarray


@dataclass
class AuxData:
    data: dict[Any, Any]

    def get(self, key, default=None):
        if self.data is None:
            return default
        return self.data.get(key, default)

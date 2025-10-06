import numpy as np
from typing import Optional, Any

from ...core.base import BaseMeasurementModel
from ...core import AuxData


class PendulumThetaMeasModel(BaseMeasurementModel):
    def update_observation_matrix(
        self, x: np.ndarray, aux: Optional[AuxData] = None
    ) -> np.ndarray:
        return np.array([1, 0])

    def meas_estimate(self, x: np.ndarray, aux: Optional[AuxData] = None) -> np.ndarray:
        return x[0]


class PendulumThetaDotMeasModel(BaseMeasurementModel):
    def update_observation_matrix(
        self, x: np.ndarray, aux: Optional[AuxData] = None
    ) -> np.ndarray:
        return np.array([0, 1])

    def meas_estimate(self, x: np.ndarray, aux: Optional[AuxData] = None) -> np.ndarray:
        return x[1]


class PendulumFullMeasModel(BaseMeasurementModel):
    def update_observation_matrix(
        self, x: np.ndarray, aux: Optional[AuxData] = None
    ) -> np.ndarray:
        return np.eye(2)

    def meas_estimate(self, x: np.ndarray, aux: Optional[AuxData] = None) -> np.ndarray:
        return x

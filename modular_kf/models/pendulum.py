import numpy as np
from typing import Optional, Any

from ..core.base import BaseSystemModel, BaseMeasurementModel
from ..core import AuxData


class PendulumSystemModel(BaseSystemModel):
    g: float = -9.81
    m: float
    l: float
    b: float

    def __init__(
        self,
        x_i: np.ndarray,
        sigmas: np.ndarray,
        m: float = 0.1,
        l: float = 1.0,
        b: float = 0.1,
    ) -> None:
        self.m = m
        self.l = l
        self.b = b
        super().__init__(x_i, sigmas)

    def update_state_transition_matrix(
        self, x: np.ndarray, dt: float, aux: Optional[AuxData] = None
    ) -> np.ndarray:

        return np.array([[0, 1], [(self.m * self.g * self.l * x[0]), -self.b * x[1]]])

    def update_noise_input_matrix(
        self, x: np.ndarray, aux: Optional[AuxData] = None
    ) -> np.ndarray:
        return np.eye(2)

    def propogate_state(
        self, x: np.ndarray, dt: float, aux: Optional[AuxData] = None
    ) -> np.ndarray:
        return np.array(
            [
                x[0] + x[1] * dt,
                x[1] + ((self.m * self.g * self.l * np.sin(x[0])) - self.b * x[1]) * dt,
            ]
        )


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

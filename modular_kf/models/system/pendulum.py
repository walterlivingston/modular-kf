import numpy as np
from typing import Optional, Any

from ...core.base import BaseSystemModel
from ...core import AuxData


class PendulumSystemModel(BaseSystemModel):
    g: float = 9.81
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

        return np.array([[0, 1], [-self.g / self.l, -self.b]])

    def update_noise_input_matrix(
        self, x: np.ndarray, aux: Optional[AuxData] = None
    ) -> np.ndarray:
        return np.eye(2)

    def propogate_state(
        self, x: np.ndarray, dt: float, aux: Optional[AuxData] = None
    ) -> np.ndarray:
        theta_dot = x[1]
        theta_ddot = (-self.b * x[1] - self.m * self.g * self.l * np.sin(x[0])) / (
            self.m * self.l**2
        )
        return np.array([[theta_dot], [theta_ddot]])

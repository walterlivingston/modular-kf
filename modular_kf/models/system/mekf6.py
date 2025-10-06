import numpy as np
from typing import Optional

from ...core.base import BaseSystemModel
from ...core import AuxData
from ...core.utils import skew


class MEKF6StateSystemModel(BaseSystemModel):
    def update_state_transition_matrix(
        self, x: np.ndarray, dt: float, aux: Optional[AuxData] = None
    ) -> np.ndarray:
        if aux is None:
            raise ValueError(
                "AuxData is required for computing the state transition matrix"
            )

        I = np.eye(3)
        O = np.zeros(3)
        omega = aux.get("gyro")

        return np.array([[-skew(omega), O], [O, O]])

    def update_noise_input_matrix(
        self, x: np.ndarray, aux: Optional[AuxData] = None
    ) -> np.ndarray:
        I = np.eye(3)
        O = np.zeros(3)
        return np.ndarray([[-I, O], [O, I]])

    def propogate_state(
        self, x: np.ndarray, dt: float, aux: Optional[AuxData] = None
    ) -> np.ndarray:
        pass

    def covariance(self) -> np.ndarray:
        pass

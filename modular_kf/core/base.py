from abc import ABC, abstractmethod
from typing import Optional
import numpy as np

from .utils import AuxData


class BaseSystemModel(ABC):
    xi: np.ndarray
    F: np.ndarray
    Bw: np.ndarray
    sigmas: np.ndarray

    def __init__(self, xi: np.ndarray, sigmas: np.ndarray) -> None:
        self.xi = xi
        self.F = self.update_state_transition_matrix(xi, 0)
        self.Bw = self.update_noise_input_matrix(xi)
        self.sigmas = sigmas

    @abstractmethod
    def update_state_transition_matrix(
        self, x: np.ndarray, dt: float, aux: Optional[AuxData] = None
    ) -> np.ndarray:
        pass

    @abstractmethod
    def update_noise_input_matrix(
        self, x: np.ndarray, aux: Optional[AuxData] = None
    ) -> np.ndarray:
        pass

    @abstractmethod
    def propogate_state(
        self, x: np.ndarray, dt: float, aux: Optional[AuxData] = None
    ) -> np.ndarray:
        pass

    def covariance(self) -> np.ndarray:
        return np.diag(self.sigmas)


class BaseMeasurementModel(ABC):
    H: np.ndarray
    sigmas: np.ndarray

    def __init__(self, x_i: np.ndarray, sigmas: np.ndarray) -> None:
        self.H = self.update_observation_matrix(x_i)
        self.sigmas = sigmas

    @abstractmethod
    def update_observation_matrix(
        self, x: np.ndarray, aux: Optional[AuxData] = None
    ) -> np.ndarray:
        pass

    @abstractmethod
    def meas_estimate(self, x: np.ndarray, aux: Optional[AuxData] = None) -> np.ndarray:
        pass

    def covariance(self) -> np.ndarray:
        return np.diag(self.sigmas)


class BaseCovarianceManager(ABC):
    @abstractmethod
    def process_covariance(
        self, sys_model: BaseSystemModel, x: np.ndarray, aux: Optional[AuxData] = None
    ) -> np.ndarray:
        pass

    @abstractmethod
    def measurement_covariance(
        self,
        meas_model: BaseMeasurementModel,
        x: np.ndarray,
        aux: Optional[AuxData] = None,
    ) -> np.ndarray:
        pass

    @abstractmethod
    def innovation_covariance(
        self,
        meas_model: BaseMeasurementModel,
        x: np.ndarray,
        P: np.ndarray,
        aux: Optional[AuxData] = None,
    ) -> np.ndarray:
        pass


# (TODO) Move
class DefaultCovarianceManager(BaseCovarianceManager):
    def process_covariance(
        self, sys_model: BaseSystemModel, x: np.ndarray, aux: Optional[AuxData] = None
    ) -> np.ndarray:
        return sys_model.covariance()

    def measurement_covariance(
        self,
        meas_model: BaseMeasurementModel,
        x: np.ndarray,
        aux: Optional[AuxData] = None,
    ) -> np.ndarray:
        return meas_model.covariance()

    def innovation_covariance(
        self,
        meas_model: BaseMeasurementModel,
        x: np.ndarray,
        P: np.ndarray,
        aux: Optional[AuxData] = None,
    ) -> np.ndarray:
        return meas_model.H @ P @ meas_model.H.T + self.measurement_covariance(
            meas_model, x, aux
        )

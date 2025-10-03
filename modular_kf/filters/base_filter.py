import numpy as np
from abc import ABC, abstractmethod
from typing import Optional

from ..models.base import (
    BaseSystemModel,
    BaseMeasurementModel,
    BaseCovarianceManager,
    DefaultCovarianceManager,
)
from ..core import AuxData, WithCovariance


class BaseFilter(ABC):
    sys_model: BaseSystemModel
    meas_model: BaseMeasurementModel
    cov_manager: BaseCovarianceManager

    state: WithCovariance
    innovation: WithCovariance

    aux: AuxData

    def __init__(
        self,
        system_model: BaseSystemModel,
        measurement_model: BaseMeasurementModel,
        covariance_manager: Optional[BaseCovarianceManager] = None,
    ) -> None:
        self.system_model = system_model
        self.measurement_model = measurement_model
        self.covariance_manager = covariance_manager or DefaultCovarianceManager()

    @abstractmethod
    def predict(
        self, dt: float, custom_model: Optional[BaseSystemModel] = None
    ) -> WithCovariance:
        pass

    @abstractmethod
    def update(
        self, y: np.ndarray, custom_model: Optional[BaseMeasurementModel] = None
    ) -> WithCovariance:
        pass

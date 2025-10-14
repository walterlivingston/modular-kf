import numpy as np
from abc import ABC, abstractmethod
from typing import Optional

from ..core.base import (
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
        sys_model: BaseSystemModel,
        meas_model: BaseMeasurementModel,
        cov_manager: Optional[BaseCovarianceManager] = None,
    ) -> None:
        self.sys_model = sys_model
        self.meas_model = meas_model
        self.cov_manager = cov_manager or DefaultCovarianceManager()
        self.state = WithCovariance(
            self.sys_model.xi,
            self.cov_manager.process_covariance(self.sys_model, self.sys_model.xi),
        )
        self.aux = AuxData()

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

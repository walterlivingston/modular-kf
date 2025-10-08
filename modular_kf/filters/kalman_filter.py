import numpy as np
from typing import Optional
from scipy.linalg import expm

from .base_filter import BaseFilter
from ..core.base import (
    BaseSystemModel,
    BaseMeasurementModel,
    BaseCovarianceManager,
    DefaultCovarianceManager,
)
from ..core import AuxData, WithCovariance
from ..core.utils import inv


class KalmanFilter(BaseFilter):
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

    def predict(
        self, dt: float, custom_model: Optional[BaseSystemModel] = None
    ) -> WithCovariance:
        if custom_model:
            self.sys_model = custom_model

        x = self.state.value
        P = self.state.covariance

        F_ = self.sys_model.F
        Phi_ = expm(F_ * dt)
        x = Phi_ @ x

        Bw = self.sys_model.Bw
        Q = self.cov_manager.process_covariance(self.sys_model, x, self.aux)

        P = Phi_ @ P @ Phi_.T + Bw @ Q @ Bw.T * dt

        self.state = WithCovariance(x, P)

        return self.state

    def update(
        self, y: np.ndarray, custom_model: Optional[BaseMeasurementModel] = None
    ) -> WithCovariance:
        if custom_model:
            self.meas_model = custom_model

        x = self.state.value
        P = self.state.covariance

        H_ = self.meas_model.H
        yhat_ = H_ @ x

        R = self.cov_manager.measurement_covariance(self.meas_model, x, self.aux)
        S = self.cov_manager.innovation_covariance(self.meas_model, x, P, self.aux)

        S_inv = inv(S)
        L = P @ H_.T * S_inv if np.isscalar(S_inv) else P @ H_.T @ S_inv

        self.z = y - yhat_
        x = x + L.T @ self.z
        P = (np.eye(len(x)) - L * H_) @ P @ (np.eye(len(x)) - L @ H_).T + L @ R @ L.T

        self.state = WithCovariance(x, P)

        return self.state

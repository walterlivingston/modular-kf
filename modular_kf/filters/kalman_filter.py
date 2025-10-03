import numpy as np
from typing import Optional
from scipy.linalg import expm

from .base_filter import BaseFilter
from ..base import (
    BaseSystemModel,
    BaseMeasurementModel,
    BaseCovarianceManager,
    DefaultCovarianceManager,
)
from ..core import AuxData, WithCovariance


class KalmanFilter:
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

    def predict(
        self, dt: float, custom_model: Optional[BaseSystemModel] = None
    ) -> WithCovariance:
        if custom_model:
            self.sys_model = custom_model

        F_ = self.sys_model.F
        Phi_ = expm(F_ * dt)
        self.x = Phi_ * self.x

        Bw = self.sys_model.Bw
        Q = self.cov_manager.process_covariance(self.sys_model, self.x, self.aux)

        self.P = Phi_ @ self.P @ Phi_.T + Bw @ Q @ Bw.T * dt

        return WithCovariance(self.x, self.P)

    def update(
        self, y: np.ndarray, custom_model: Optional[BaseMeasurementModel] = None
    ) -> WithCovariance:
        if custom_model:
            self.meas_model = custom_model

        H_ = self.meas_model.H
        yhat_ = H_ * self.x

        R = self.cov_manager.measurement_covariance(self.meas_model, self.x, self.aux)
        S = self.cov_manager.innovation_covariance(
            self.meas_model, self.x, self.P, self.aux
        )

        L = self.P @ H_.T / S

        self.z = y - yhat_
        self.x = self.x + L * self.z
        self.P = (np.eye(len(self.x)) - L @ H_) @ self.P @ (
            np.eye(len(self.x)) - L @ H_
        ).T + L @ R @ L.T

        return WithCovariance(self.x, self.P)

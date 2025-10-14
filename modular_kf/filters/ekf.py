import numpy as np
from typing import Optional
from scipy.linalg import expm

from .base_filter import BaseFilter
from ..core.base import BaseSystemModel, BaseMeasurementModel
from ..core import WithCovariance
from ..core.utils import inv


class ExtendedKalmanFilter(BaseFilter):
    def predict(
        self, dt: float, custom_model: Optional[BaseSystemModel] = None
    ) -> WithCovariance:
        if custom_model:
            self.sys_model = custom_model

        x = self.state.value
        P = self.state.covariance

        F_ = self.sys_model.update_state_transition_matrix(x, dt, self.aux)
        Phi_ = expm(F_ * dt)
        x = self.sys_model.propogate_state(x, dt, self.aux)

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

        H_ = self.meas_model.update_observation_matrix(x, self.aux)
        yhat_ = self.meas_model.meas_estimate(x, self.aux)

        R = self.cov_manager.measurement_covariance(self.meas_model, x, self.aux)
        S = self.cov_manager.innovation_covariance(self.meas_model, x, P, self.aux)

        S_inv = inv(S)
        L = P @ H_.T * S_inv if np.isscalar(S_inv) else P @ H_.T @ S_inv

        self.z = y - yhat_
        if self.z.size == 1:
            x = x + L * self.z
            P = (np.eye(len(x)) - L * H_) @ P @ (
                np.eye(len(x)) - L @ H_
            ).T + L * R * L.T
        else:
            x = x + L @ self.z
            P = (np.eye(len(x)) - L * H_) @ P @ (
                np.eye(len(x)) - L @ H_
            ).T + L @ R @ L.T

        self.state = WithCovariance(x, P)

        return self.state

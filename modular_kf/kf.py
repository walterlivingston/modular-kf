from __future__ import annotations
from typing import TYPE_CHECKING

if TYPE_CHECKING:
    from modular_kf import CovarianceBlock

import numpy as np
from modular_kf import StateBlock, MeasurementBlock
from modular_kf.helpers import *

class KalmanFilter:
    state_block: StateBlock
    measurement_block: MeasurementBlock
    covariance_block: CovarianceBlock
    x: vec = vec0
    X: vec = vec0
    z: vec = vec0
    P: mat = mat0
    mode: str = ""

    def __init__(self, state_block, measurement_block, covariance_block, **options):
        self.state_block: StateBlock = state_block
        self.measurement_block: MeasurementBlock = measurement_block
        self.covariance_block = covariance_block

        self.x = options.get('x_i', np.zeros((state_block.num_states, 1)))
        self.dt = options.get('dt', 0)
        self.mode = options.get('mode', "linear")

        self.P = self.covariance_block.calcProcessCovarianceMatrix(self.dt)

        match self.mode:
            case "linear":
                self.state_block.F = self.state_block.updateStateTransitionMatrix(self.x)
                self.measurement_block.H = self.measurement_block.updateObservationMatrix(self.x, vec0)
            case "extended":
                pass
            case "error":
                self.X = options.get('X_i', np.zeros((state_block.num_states, 1)))

                sAux = self.state_block.aux
                sAux.X = self.X

                mAux = self.measurement_block.aux
                mAux.X = self.X

                self.state_block = self.state_block.processAuxData(sAux)
                self.measurement_block = self.measurement_block.processAuxData(mAux)

                self.state_block.applyError(self.x, self.X, self.dt)
                self.measurement_block.applyError(self.x, self.X)
            case _:
                raise ValueError("This type of Kalman Filter is undefined!")
            
    def process(self, dt: float, customStateBlock: StateBlock | None = None, customCovBlock: CovarianceBlock | None = None):
        if customStateBlock is not None:
            self.state_block = customStateBlock

        if customCovBlock is not None:
            self.covariance_block = customCovBlock

        [self.x, Phi] = self.state_block.propagate(self.x, dt, self.mode == 'extended' or self.mode == 'error')
        Qd = self.covariance_block.calcProcessCovariance(self, dt, self.state_block)
        P = Phi*self.P*Phi.T + Qd
        self.P = np.matrix(P)

        if self.mode == 'error':
            ewn = self.state_block.applyError(self.x, self.X, dt)
            self.X = ewn.E
            self.x = ewn.E

            aux = self.state_block.aux
            aux.X = self.X
            self.state_block.processAuxData(aux)

    def update(self, y: vec, customMeasBlock: MeasurementBlock | None = None, customCovBlock: CovarianceBlock | None = None):
        if customMeasBlock is not None:
            self.measurement_block = customMeasBlock

        if customCovBlock is not None:
            self.covariance_block = customCovBlock

        [yhat, H] = self.measurement_block.update(self.x, y, self.mode == 'extended' or self.mode == 'error')
        R = self.covariance_block.calcMeasurementCovariance(self, self.measurement_block)
        oldz = self.z
        self.z = (y - yhat)
        [S,reject] = self.covariance_block.calcInnovationCovariance(self)

        if not reject:
            L = self.P*H.T/S

            self.x = self.x + L*self.z
            P = (np.eye(self.state_block.num_states) - L*H)*self.P*(np.eye(self.state_block.num_states) - L@H).T + L*R*L.T
            self.P = np.matrix(P)

            if self.mode == 'error':
                [self.x, self.X] = self.measurement_block.applyError(self.x, self.X)
                aux = self.measurement_block.aux
                aux.X = self.X
                aux.y = y
                self.measurement_block.processAuxData(aux)
        else:
            self.z = oldz



import numpy as np
from modular_kf import MeasurementBlock
from modular_kf.helpers import *
import quaternions as q

class mekfAccMeasBlock(MeasurementBlock):
    g = np.array([0, 0, -9.81]).T

    def update(self, x_, y_, relinearize):
        if relinearize or np.isnan(self.H).any:
            self.H = np.asmatrix(self.updateObservationMatrix(x_, y_))
            H = self.H
            X = self.aux.X
            C = q.q2DCM(X[:4]).T
            yhat = C@self.g - H*x_
        else:
            H = self.H
            yhat = H*x_

        return (yhat, H)

    def updateObservationMatrix(self, x_, y_):
        X = self.aux.X
        C = q.q2DCM(X[:4]).T
        return np.hstack([skew(C@self.g), np.zeros((3, 3))])
    
    def calcMeasurementCovarianceMatrix(self):
        return np.diag(self.meas_sigmas**2)

    def applyError(self, x_, X_):
        alpha = x_[:3]*0.5
        alpha = alpha.reshape((3,1))
        qa = np.vstack((1, alpha))
        X = np.zeros((len(X_),1))
        X[:4] = q.qMult(X_[:4], qa).reshape((4,1))
        X[4:] = x_[3:6]
        x = np.concatenate([np.zeros((3, 1)), x_[3:6].reshape(-1, 1)])

        return (x, X)

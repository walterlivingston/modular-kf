import numpy as np
from modular_kf import MeasurementBlock
from modular_kf.helpers import *
import quaternions as q

class mekfMagMeasBlock(MeasurementBlock):

    def update(self, x_, y_, relinearize):
        if relinearize or np.isnan(self.H).any:
            self.H = np.asmatrix(self.updateObservationMatrix(x_, y_))
            H = self.H
            X = self.aux.X
            C = q.q2DCM(X[:4]).T
            b = C@y_
            b = np.array([np.sqrt(b(1)^2 + b(2)^2), 0, b(3)]).T
            yhat = C@b - H@x_
        else:
            H = self.H
            yhat = H@x_

        return (yhat, H)

    def updateObservationMatrix(self, x_, y_):
        X = self.aux.X
        C = q.q2DCM(X[:4]).T
        C = q.q2DCM(X[:4]).T
        b = C@y_
        b = np.array([np.sqrt(b(1)^2 + b(2)^2), 0, b(3)]).T
        return np.hstack([skew(C@b), np.zeros((3, 3))])
    
    def calcMeasurementCovarianceMatrix(self):
        return np.diag(self.meas_sigmas**2)

    def applyError(self, x_, X_):
        alpha = x_[:3]*0.5
        qa = np.array([1, alpha.T])
        X = np.zeros((len(X_),1))
        X[:4] = q.qMult(X_[:4], qa)
        X[4:] = x_[3:6]
        x = np.concatenate([np.zeros((3, 1)), x_[3:6].reshape(-1, 1)])

        return (x, X)

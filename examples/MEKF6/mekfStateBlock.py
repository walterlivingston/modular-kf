import numpy as np
from scipy.linalg import expm
from modular_kf import StateBlock
from modular_kf.helpers import *
import quaternions as q

class mekfStateBlock(StateBlock):
    
    def propagate(self, x_, dt, relinearize):
        if relinearize or np.isnan(self.F).any():
            self.F = np.asmatrix(self.updateStateTransitionMatrix(x_))
            x = self.updateState(x_, dt)
            Phi = expm(self.F*dt)
        else:
            Phi = expm(self.F*dt)
            x = Phi@x_

        return (x, Phi)
    
    def updateStateTransitionMatrix(self, x_):
        return np.block([[-skew(self.aux.y[0:3]),      -np.eye(3)],
                      [  np.zeros((3,3)), np.zeros((3,3))]])
    
    def updateState(self, x_, dt):
        return np.concatenate([np.zeros((3,1)), x_[3:6]])
    
    def calcProcessCovarianceMatrix(self, dt):
        Q = np.diag(self.state_sigmas**2)*np.sqrt(dt)
        Bw = np.block([[      -np.eye(3), np.zeros((3, 3))],
                       [np.zeros((3, 3)),        np.eye(3)]])
        if np.isnan(self.F).any:
            self.F = np.asmatrix(self.updateStateTransitionMatrix(np.zeros((self.num_states, 1))))
        Phi = expm(self.F*dt)
        return Phi@Bw@Q@Bw.T@Phi.T*dt**2

    def applyError(self, x_, X_, dt):
        X = np.zeros((len(X_), 1))

        if q.qNorm(X_[:4]) < 1e-10:
            X_[:4] = np.array([1, 0, 0, 0])
        if len(self.aux.y) > 3:
            w = self.aux.y[:3]
            w1 = self.aux.y[3:6]
            w_ = (w + w1)/2
        else:
            w_ = self.aux.y[:3]

        q_dot = 0.5*q.qMult(X_[:4], np.vstack(([0], w_)))*dt
        q_dot = q_dot.reshape((4,1))

        X[:4] = X_[:4] + q_dot
        X[:4] = q.qNormalize(X[:4])

        X[4:] = X_[4:]
        
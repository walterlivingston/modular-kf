import numpy as np
from modular_kf import CovarianceBlock
from modular_kf.helpers import *

class BasicCovBlock(CovarianceBlock):
    def calcInnovationCovariance(self, filter) -> tuple[mat, bool]:
        H = filter.measurement_block.H
        P = filter.P
        R = filter.measurement_block.calcMeasurementCovarianceMatrix()
        z = filter.z
        self.S = np.asmatrix(H*P*H.T + R)
        self.r = np.sqrt(z.T*(self.S^(-1))*z)

        reject = False

        return (self.S, reject)
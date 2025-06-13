import numpy as np
from abc import ABC, abstractmethod
from modular_kf.helpers import *

class MeasurementBlock(ABC):
    meas_sigmas: vec = vec0
    H: mat = mat0
    aux: Aux = Aux(x = vec0, X = vec0, y = vec0)

    def __init__(self, meas_sigmas):
        self.meas_sigmas = meas_sigmas

    @abstractmethod
    def updateObservationMatrix(self, x_: vec, y_:vec) -> mat:
        pass

    @abstractmethod
    def calcMeasurementCovarianceMatrix(self) -> mat:
        pass

    @abstractmethod
    def calcInnovationCovarianceMatrix(self) -> mat:
        pass

    @abstractmethod
    def update(self, x_: vec, y_: vec, relinearize: bool = True) -> tuple[vec, mat]:
        pass

    @abstractmethod
    def applyError(self, x_: vec, X_: vec) -> estWithNominal:
        pass

    def processAuxData(self, aux_: Aux):
        self.aux = aux_
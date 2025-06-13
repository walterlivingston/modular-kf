import numpy as np
from abc import ABC, abstractmethod
from modular_kf.helpers import *
from modular_kf import KalmanFilter, StateBlock, MeasurementBlock

class CovarianceBlock(ABC):
    S: mat = mat0
    r: vec = vec0

    def __init__(self):
        pass

    def calcProcessCovariance(self, filter: KalmanFilter, dt: float, customStateBlock: StateBlock | None = None):
        pass

    def calcMeasurementCovariance(self, filter: KalmanFilter, customMeasBlock: MeasurementBlock | None = None):
        pass

    @abstractmethod
    def calcInnovationCovariance(self, filter: KalmanFilter) -> tuple[mat, bool]:
        pass
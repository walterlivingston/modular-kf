from __future__ import annotations
from typing import TYPE_CHECKING

if TYPE_CHECKING:
    from modular_kf import KalmanFilter

import numpy as np
from abc import ABC, abstractmethod
from modular_kf.helpers import *
from modular_kf import StateBlock, MeasurementBlock

class CovarianceBlock(ABC):
    S: mat = mat0
    r: vec = vec0

    def __init__(self):
        pass

    def calcProcessCovariance(self, filter: KalmanFilter, dt: float, customStateBlock: StateBlock | None = None) -> mat:
        sBlock: StateBlock
        if customStateBlock is not None:
            sBlock = customStateBlock
        else:
            sBlock = filter.state_block

        return sBlock.calcProcessCovarianceMatrix(dt)

    def calcMeasurementCovariance(self, filter: KalmanFilter, customMeasBlock: MeasurementBlock | None = None) -> mat:
        mBlock: MeasurementBlock
        if customMeasBlock is not None:
            mBlock = customMeasBlock
        else:
            mBlock = filter.measurement_block

        return mBlock.calcMeasurementCovarianceMatrix()

    @abstractmethod
    def calcInnovationCovariance(self, filter: KalmanFilter) -> tuple[mat, bool]:
        pass
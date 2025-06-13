import numpy as np
from abc import ABC, abstractmethod
from modular_kf.helpers import *

class StateBlock(ABC):
    num_states: int = 0
    state_sigmas: vec = vec0
    F: mat = mat0
    aux: Aux = Aux(x = vec0, X = vec0, y = vec0)

    def __init__(self, num_states: int, state_sigmas: vec):
        self.num_states = num_states
        self.state_sigmas = state_sigmas

    @abstractmethod
    def updateStateTransitionMatrix(self, x_: vec) -> mat:
        pass

    @abstractmethod
    def updateState(self, x_: vec, dt: float) -> vec:
        pass

    @abstractmethod
    def calcProcessCovarianceMatrix(self, dt: float) -> mat:
        pass

    @abstractmethod
    def propagate(self, x_: vec, dt: float, relinearize: bool = True) -> tuple[vec, mat]: 
        pass

    @abstractmethod
    def applyError(self, x_: vec, X_: vec, dt: float) -> estWithNominal:
        pass

    def processAuxData(self, aux_: Aux):
        self.aux = aux_
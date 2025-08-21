from dataclasses import dataclass
import numpy as np
from collections import namedtuple
from typing import Any

vec = np.ndarray
mat = np.matrix

vec0 = np.zeros((1,1))
mat0 = np.matrix(np.zeros((1,1)))

estWithCov = namedtuple("estWithCov", ["e", "P"])
estWithNominal = namedtuple("estWithNominal", ["e", "E"])

@dataclass
class Aux:
    x: vec = vec0
    X: vec = vec0
    y: vec = vec0

def skew(v):
    v = v.flatten()
    return np.array([[    0, -v[2],  v[1]],
                     [ v[2],     0, -v[0]],
                    [ -v[1],  v[0],     0]])
#!/usr/bin/env python3
from filterpy.common import Q_discrete_white_noise
from filterpy.kalman import KalmanFilter
from scipy.linalg import block_diag
import numpy as np

class Tracker:
    def __init__(self, dt=1.0/20.0):
        self.dt = dt
        self.kalman = KalmanFilter(dim_x=4, dim_z=2)
        self.kalman.x = np.array([[0], [0], [0], [0]])
        uncertaintyInit = 500
        self.kalman.P = np.array([[1, 0, 0, 0],
                                  [0, 1, 0, 0],
                                  [0, 0, 1, 0],
                                  [0, 0, 0, 1]]) * uncertaintyInit
        processVariance = 30
        q = Q_discrete_white_noise(dim=2, dt=self.dt, var=processVariance)
        self.kalman.Q = block_diag(q, q)
        self.kalman.R = np.array([[0.5, 0], [0, 0.5]])
        self.kalman.H = np.array([[1, 0, 0, 0], [0, 0, 1, 0]])
        self.kalman.F = np.array([[1, dt, 0, 0], [0, 1, 0, 0], [0, 0, 1, dt], [0, 0, 0, 1]])

    def add(self, poly):
        self.kalman.predict()
        if poly:
            m = poly.coeffs[0]
            b = poly.coeffs[1]
            measurement = np.array([[m], [b]], np.float32)
            self.kalman.update(measurement)
        line = np.poly1d([self.kalman.x[0, 0], self.kalman.x[2, 0]])
        return line

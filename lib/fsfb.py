import numpy as np
from scipy.linalg import solve_continuous_are, solve_discrete_are
from scipy.signal import cont2discrete

class FSFB:
    def __init__(self, A, B, C, D, dt: float):
        self.A = A
        self.B = B
        self.C = C
        self.D = D
        self.dt = dt

        self.discretize()
        self.lqr(np.eye(A.shape[1]),np.eye(B.shape[1]))

    def discretize(self):
        dlti = cont2discrete((self.A,self.B,self.C,self.D),self.dt)
        self.Ad = dlti[0]
        self.Bd = dlti[1]

    def lqr(self, Q, R):
        self.Q = Q
        self.R = R
        S = solve_continuous_are(self.A, self.B, Q, R)
        Sd = solve_discrete_are(self.Ad, self.Bd, Q, R)

        self.K = np.linalg.inv(R) @ self.B.T @ S
        self.Kd = np.linalg.inv(R) @ np.transpose(self.Bd) @ Sd
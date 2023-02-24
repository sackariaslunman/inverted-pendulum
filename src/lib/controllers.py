from scipy.signal import cont2discrete
import numpy as np
from scipy.linalg import solve_continuous_are, solve_discrete_are

class FSFB:
    def __init__(self, A, B, C, D, dt: float):
        self.A = A
        self.B = B
        self.C = C
        self.D = D
        self.dt = dt

        self.discretize()

    def discretize(self):
        dlti = cont2discrete((self.A,self.B,self.C,self.D),self.dt)
        self.A_d = np.array(dlti[0])
        self.B_d = np.array(dlti[1])

class LQR(FSFB):
    def calculate_K_lqr(self, Q, R):
        self.Q = Q
        self.R = R
        S = solve_continuous_are(self.A, self.B, Q, R)
        S_d = solve_discrete_are(self.A_d, self.B_d, Q, R)

        self.K = np.linalg.inv(R) @ self.B.T @ S
        self.K_d = np.linalg.inv(R + self.B_d.T @ S_d @ self.B_d) @ (self.B_d.T @ S_d @ self.A_d)

    def calculate_K_r(self):
        K_r = np.true_divide(1, self.D + self.C @ np.linalg.inv(-self.A + self.B @ self.K) @ self.B)
        K_r[K_r == np.inf] = 0
        K_r = np.nan_to_num(K_r)
        self.K_r = K_r.T

        K_r_d = np.true_divide(1, self.D + self.C @ np.linalg.inv(np.eye(self.A_d.shape[0]) - self.A_d + self.B_d @ self.K_d) @ self.B_d)      
        K_r_d[K_r_d == np.inf] = 0
        K_r_d = np.nan_to_num(K_r_d)
        self.K_r_d = K_r_d.T

    def feedback_d(self, state, r):
        u_d = self.K_r_d @ r - self.K_d @ state
        return u_d

class Kalman:
    def __init__(self):
        pass
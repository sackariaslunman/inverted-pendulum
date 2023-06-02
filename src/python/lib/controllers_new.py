from scipy.signal import cont2discrete
import numpy as np
from scipy.linalg import solve_continuous_are, solve_discrete_are

class FSFB:
    def __init__(self, dt: float):
        self.dt = dt

    def discretize(self, A: np.ndarray, B: np.ndarray, C: np.ndarray, D: np.ndarray):
        dlti = cont2discrete((A,B,C,D),self.dt)
        A_d = np.array(dlti[0])
        B_d = np.array(dlti[1])
        return A_d, B_d

class LQR(FSFB):
    def calculate_K_lqr(self, A: np.ndarray, B: np.ndarray, Q: np.ndarray, R: np.ndarray) -> np.ndarray:
        S = solve_continuous_are(A, B, Q, R)
        K = np.linalg.inv(R) @ B.T @ S
        return K

    def calculate_K_lqr_d(self, A_d: np.ndarray, B_d: np.ndarray, Q: np.ndarray, R: np.ndarray) -> np.ndarray:
        S_d = solve_discrete_are(A_d, B_d, Q, R)
        K_d = np.linalg.inv(R + B_d.T @ S_d @ B_d) @ (B_d.T @ S_d @ A_d)
        return K_d

    def feedback(self, K: np.ndarray, error: np.ndarray) -> np.ndarray:
        u = error @ K.T
        return u

class Kalman:
    def __init__(self):
        pass
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
    def error_augment(self, A: np.ndarray, B: np.ndarray, C: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
        AC = np.concatenate((A, -C))
        ZZ = np.concatenate((np.zeros(C.T.shape), np.zeros((C.shape[0], C.shape[0])))) #type: ignore
        A_aug = np.concatenate((AC, ZZ), axis=1)
        B_aug = np.concatenate((B, np.zeros((C.shape[0],B.shape[1]))))
        return A_aug, B_aug

    def calculate_K(self, A: np.ndarray, B: np.ndarray, Q: np.ndarray, R: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
        P = solve_continuous_are(A, B, Q, R)
        K = np.linalg.inv(R) @ B.T @ P
        return P, K

    def calculate_K_d(self, A_d: np.ndarray, B_d: np.ndarray, Q: np.ndarray, R: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
        P_d = solve_discrete_are(A_d, B_d, Q, R)
        K_d = np.linalg.inv(R + B_d.T @ P_d @ B_d) @ (B_d.T @ P_d @ A_d)
        return P_d, K_d
    
    def calculate_finite_K_ds(self, A_ds: np.ndarray, B_ds: np.ndarray, Q: np.ndarray, R: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
        P_d_last, K_d_last = self.calculate_K_d(A_ds[-1], B_ds[-1], Q, R)

        K_ds = np.zeros((A_ds.shape[0], K_d_last.shape[0], K_d_last.shape[1]))
        K_ds[-1] = K_d_last

        P_ds = np.zeros((A_ds.shape[0], P_d_last.shape[0], P_d_last.shape[1]))
        P_ds[-1] = P_d_last

        for k in range(A_ds.shape[0]-2, -1, -1):
            K_ds[k] = np.linalg.inv(R + B_ds[k].T @ P_ds[k+1] @ B_ds[k]) @ (B_ds[k].T @ P_ds[k+1] @ A_ds[k])
            P_ds[k] = A_ds[k].T @ P_ds[k+1] @ A_ds[k] - (A_ds[k].T @ P_ds[k+1] @ B_ds[k]) @ K_ds[k] + Q
        
        return P_ds, K_ds


    def feedback(self, K: np.ndarray, error: np.ndarray) -> np.ndarray:
        u = error @ K.T 
        return u

class Kalman:
    def __init__(self):
        pass
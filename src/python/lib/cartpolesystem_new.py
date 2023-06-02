from __future__ import annotations
import numpy as np
from numpy import sin, cos, pi
from numpy.random import multivariate_normal
from .colors import color
from scipy.signal import cont2discrete
import sympy as sp

class Cart:
    def __init__(
        self, 
        mass: float, 
        track_friction: float, 
        x_bounds: tuple[float, float]
    ):
        self.m = mass
        self.u_c = track_friction
        self.x_bounds = x_bounds

class DCMotor:
    def __init__(
        self, 
        torque_constant: float,
        velocity_constant: float,
        resistance: float,
        radius: float,
        motor_inertia: float,
        motor_friction: float,
        Va_bounds: tuple[float, float],
        Ia_bounds: tuple[float, float],
    ):
        self.Kt = torque_constant
        self.Ke = velocity_constant
        self.Ra = resistance
        self.r = radius
        self.Jm = motor_inertia
        self.Bm = motor_friction
        self.Va_bounds = Va_bounds
        self.Ia_bounds = Ia_bounds

class Pole:
    def __init__(
        self, 
        mass: float, 
        length: float, 
        joint_friction: float, 
    ):
        self.m = mass
        self.l = length
        self.u_p = joint_friction

class CartPoleSystem:
    def __init__(
        self,
        cart: Cart,
        motor: DCMotor,
        poles: list[Pole],
        g: float
    ):
        self.cart = cart
        self.motor = motor
        self.poles = poles
        self.g = g
        self.num_poles = len(poles)
        self.num_states = 2+self.num_poles*2
        self.num_controls = 1

        self.M = self.cart.m + sum(pole.m for pole in self.poles)
        self.ms = np.array([pole.m for pole in self.poles])
        self.ls = np.array([pole.l for pole in self.poles])
        self.L = sum(pole.l for pole in self.poles)
        self.u_ps = np.array([pole.u_p for pole in self.poles])

        self.state_lower_bound = np.array([cart.x_bounds[0], np.finfo(np.float64).min] + [-2*pi, np.finfo(np.float64).min]*self.num_poles)
        self.state_upper_bound = np.array([cart.x_bounds[1], np.finfo(np.float64).max] + [ 2*pi, np.finfo(np.float64).max]*self.num_poles)

        self.control_lower_bound = np.array([motor.Va_bounds[0]])
        self.control_upper_bound = np.array([motor.Va_bounds[1]])
    
        self.vars = self.get_sp_vars()
        self.eqs = self.get_sp_equations()
        self.F = self.get_sp_jacobian()

    def constraints(self, state: np.ndarray, control: np.ndarray) -> np.ndarray:
        Va = control[0]
        d_x = state[1]
        Ia = (Va-self.motor.Ke*d_x/self.motor.r)/self.motor.Ra
        return np.concatenate([state-self.state_lower_bound, self.state_upper_bound-state, control-self.control_lower_bound, self.control_upper_bound-control, [Ia-self.motor.Ia_bounds[0], self.motor.Ia_bounds[1]-Ia]])
    
    def clip(self, state: np.ndarray, control: np.ndarray) -> tuple[np.ndarray,np.ndarray]:
        x = state[0]
        d_x = state[1]
        x_clipped = np.clip(x, self.state_lower_bound[0], self.state_upper_bound[0])
        thetas = state[2::2]
        d_thetas = state[3::2]
        clipped_thetas = (thetas + np.pi) % (2*np.pi) - np.pi

        clipped_state = np.concatenate((np.array([x_clipped, d_x]), np.column_stack((clipped_thetas, d_thetas)).flatten()))
        
        Va = control[0]
        Va_Ia_max = self.motor.Ra*self.motor.Ia_bounds[1] + self.motor.Ke*d_x/self.motor.r
        Va_Ia_min = self.motor.Ra*self.motor.Ia_bounds[0] + self.motor.Ke*d_x/self.motor.r

        Va_clipped = np.clip(Va, self.control_lower_bound[0], self.control_upper_bound[0])
        Va_Ia_clipped = np.clip(Va_clipped, Va_Ia_min, Va_Ia_max)

        clipped_control = np.array([Va_Ia_clipped])

        return clipped_state, clipped_control

    def end_height(self, state: np.ndarray) -> float:
        thetas = state[2::2]
        return (self.ls*sin(thetas)).sum()
        
    def calculate_error(self, state: np.ndarray, reference: np.ndarray) -> np.ndarray:
        error = reference-state
        error[2::2] = np.arctan2(np.sin(error[2::2]), np.cos(error[2::2]))
        return error

    def get_sp_vars(self):
        return sp.symbols("x dx " + " ".join([f"theta{i} d_theta{i}" for i in range(self.num_poles)]) + " Va")

    def get_sp_equations(self):
        # x = self.vars[0]
        d_x = self.vars[1]
        Va = self.vars[-1]
        thetas = self.vars[2::2]
        d_thetas = self.vars[3::2]

        f1 = d_x

        sp_sum1 = sum([m*sp.sin(theta)*sp.cos(theta) for m, theta in zip(self.ms, thetas)])
        sp_sum2 = sum([m*(l/2)*d_theta**2*sp.sin(theta) for m, l, d_theta, theta in zip(self.ms, self.ls, d_thetas, thetas)])
        sp_sum3 = sum([u_p*d_theta*sp.cos(theta)/(l/2) for u_p, d_theta, theta, l in zip(self.u_ps, d_thetas, thetas, self.ls)])
        sp_sum4 = sum([m*sp.cos(theta)**2 for m, theta in zip(self.ms, thetas)])
                   
        h2 = (self.g*sp_sum1-7/3*(1/self.motor.r**2*(self.motor.Kt/self.motor.Ra*(Va*self.motor.r-self.motor.Ke*d_x)-self.motor.Bm*d_x)-sp_sum2-self.cart.u_c*d_x)-sp_sum3)
        g2 = (sp_sum4-7/3*(self.M+self.motor.Jm/self.motor.r**2))
        dd_x = h2/g2
        f2 = dd_x

        f_d_thetas = d_thetas       
        dd_thetas = [3/(7*l/2)*(self.g*sp.sin(theta)-dd_x*sp.cos(theta)-u_p*d_theta/(m*l/2)) for l, theta, u_p, m, d_theta in zip(self.ls, thetas, self.u_ps, self.ms, d_thetas)]
        f_dd_thetas = dd_thetas

        return [f1, f2] + [val for pair in zip(f_d_thetas, f_dd_thetas) for val in pair] #type: ignore
    
    def get_sp_jacobian(self):
        return sp.Matrix(self.eqs).jacobian(self.vars)
    
    def linearize(self, state0: np.ndarray, control0: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
        operating_point = {var: value for var, value in zip(self.vars, np.concatenate((state0, control0)))}
        F_evaluated = self.F.subs(operating_point)
        A = np.array(F_evaluated[:,:-1]).astype(np.float64)
        B = np.array(F_evaluated[:,-1]).astype(np.float64)
        return A, B
    
    def differentiate(self, state: np.ndarray, control: np.ndarray) -> np.ndarray:
        # x = state[0]
        d_x = state[1]
        Va = control[-1]
        thetas = state[2::2]
        d_thetas = state[3::2]

        sum1 = (self.ms*sin(thetas)*cos(thetas)).sum()
        sum2 = (self.ms*(self.ls/2)*d_thetas**2*sin(thetas)).sum()
        sum3 = ((self.u_ps*d_thetas*cos(thetas))/(self.ls/2)).sum()
        sum4 = (self.ms*cos(thetas)**2).sum()

        Ia = (Va-self.motor.Ke*d_x/self.motor.r)/self.motor.Ra
        h2 = (self.g*sum1-7/3*(1/self.motor.r**2*(self.motor.Kt*self.motor.r*Ia-self.motor.Bm*d_x)-sum2-self.cart.u_c*d_x)-sum3)
        g2 = (sum4-7/3*(self.M+self.motor.Jm/self.motor.r**2))
        dd_x = h2/g2

        dd_thetas = 3/(7*self.ls/2)*(self.g*sin(thetas)-dd_x*cos(thetas)-self.u_ps*d_thetas/(self.ms*self.ls/2))

        d_state = np.concatenate((np.array([d_x, dd_x]), np.column_stack((d_thetas, dd_thetas)).flatten()))
            
        return d_state
    
    def linear_differentiate(self, state: np.ndarray, control: np.ndarray, state0: np.ndarray, control0: np.ndarray) -> np.ndarray:
        A, B = self.linearize(state0, control0)
        d_state = state @ A.T + control @ B.T
        return d_state.T

    def discretize(self, dt: float, A: np.ndarray, B: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
        dlti = cont2discrete((A,B,None,None), dt)
        A_d = np.array(dlti[0])
        B_d = np.array(dlti[1])
        return A_d, B_d
from __future__ import annotations
import numpy as np 
from numpy import sin, cos, pi
import sympy as sp 
from .motors import DCMotor, StepperMotor

class Cart:
    def __init__(
        self, 
        mass: float, 
        track_friction: float, 
        x_bounds: tuple[float, float],
        x_bound_margin = 0.1
    ):
        self.m = mass
        self.u_c = track_friction
        self.x_bounds = x_bounds

        x_margin_clipped = np.clip(x_bound_margin, 0, 0.5)
        self.x_margin = x_margin_clipped

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

class CartPoleDCMotorSystem:
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
        self.num_constraint_states = 1

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

    def constraint_states(self, state: np.ndarray, control: np.ndarray) -> np.ndarray:
        Va = control[0]
        d_x = state[1]
        Ia = (Va-self.motor.Ke*d_x/self.motor.r)/self.motor.Ra
        return np.array([Ia])

    def constraints(self, state: np.ndarray, control: np.ndarray) -> np.ndarray:
        c_state = self.constraint_states(state, control)
        Ia = c_state[0]
        constraints = np.concatenate([state[:1]-self.state_lower_bound[:1]*(1-self.cart.x_margin), self.state_upper_bound[:1]*(1-self.cart.x_margin)-state[:1], control-self.control_lower_bound*(1-self.motor.Ia_margin), self.control_upper_bound*(1-self.motor.Ia_margin)-control, [Ia-self.motor.Ia_bounds[0]*(1-self.motor.Ia_margin), self.motor.Ia_bounds[1]*(1-self.motor.Ia_margin)-Ia]])
        return constraints
    
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
        return sum(self.ls*sin(thetas))
        
    def calculate_error(self, state: np.ndarray, reference: np.ndarray) -> np.ndarray:
        error = reference-state
        error[2::2] = np.arctan2(np.sin(error[2::2]), np.cos(error[2::2]))
        return error

    def get_sp_vars(self):
        variables = "x dx " + " ".join([f"theta{i} d_theta{i}" for i in range(self.num_poles)]) + " Va"
        return sp.symbols(variables)

    def get_sp_equations(self):
        # x = self.vars[0]
        d_x = self.vars[1]
        Va = self.vars[-1]
        thetas = self.vars[2::2]
        d_thetas = self.vars[3::2]

        f1 = d_x

        sp_sum1 = sum(m*sp.sin(theta)*sp.cos(theta) for m, theta in zip(self.ms, thetas))
        sp_sum2 = sum(m*(l/2)*d_theta**2*sp.sin(theta) for m, l, d_theta, theta in zip(self.ms, self.ls, d_thetas, thetas))
        sp_sum3 = sum(u_p*d_theta*sp.cos(theta)/(l/2) for u_p, d_theta, theta, l in zip(self.u_ps, d_thetas, thetas, self.ls))
        sp_sum4 = sum(m*sp.cos(theta)**2 for m, theta in zip(self.ms, thetas))
                   
        h2 = (self.g*sp_sum1-7/3*(1/self.motor.r**2*(self.motor.Kt/self.motor.Ra*(Va*self.motor.r-self.motor.Ke*d_x)-self.motor.Bm*d_x)-sp_sum2-self.cart.u_c*d_x)-sp_sum3)
        g2 = (sp_sum4-7/3*(self.M+self.motor.Jm/self.motor.r**2))
        dd_x = h2/g2
        f2 = dd_x

        f_d_thetas = d_thetas       
        dd_thetas = [3/(7*l/2)*(self.g*sp.sin(theta)-dd_x*sp.cos(theta)-u_p*d_theta/(m*l/2)) for l, theta, u_p, m, d_theta in zip(self.ls, thetas, self.u_ps, self.ms, d_thetas)]
        f_dd_thetas = dd_thetas

        eqs = [f1, f2] + [val for pair in zip(f_d_thetas, f_dd_thetas) for val in pair] #type: ignore
        return eqs
    
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

        sum1 = sum(self.ms*sin(thetas)*cos(thetas))
        sum2 = sum(self.ms*(self.ls/2)*d_thetas**2*sin(thetas))
        sum3 = sum((self.u_ps*d_thetas*cos(thetas))/(self.ls/2))
        sum4 = sum(self.ms*cos(thetas)**2)

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
    
class CartPoleStepperMotorSystem:
    def __init__(
        self,
        cart: Cart,
        motor: StepperMotor,
        poles: list[Pole],
        g: float,
    ):
        self.cart = cart
        self.motor = motor
        self.poles = poles
        self.g = g
        self.num_poles = len(poles)
        self.num_states = 2+self.num_poles*2
        self.num_controls = 1
        self.num_constraint_states = 1

        self.M = self.cart.m + sum(pole.m for pole in self.poles)
        self.ms = np.array([pole.m for pole in self.poles])
        self.ls = np.array([pole.l for pole in self.poles])
        self.L = sum(pole.l for pole in self.poles)
        self.u_ps = np.array([pole.u_p for pole in self.poles])

        self.state_lower_bound = np.array([cart.x_bounds[0], motor.v_bounds[0]] + [-2*pi, -np.inf]*self.num_poles)
        self.state_upper_bound = np.array([cart.x_bounds[1], motor.v_bounds[1]] + [ 2*pi, np.inf]*self.num_poles)

        self.control_lower_bound = np.array([-np.inf])
        self.control_upper_bound = np.array([np.inf])

        self.state_margin = np.array([cart.x_margin*(self.state_upper_bound[0]-self.state_lower_bound[0]), motor.v_margin*(self.state_upper_bound[1]-self.state_lower_bound[1])] + [0, 0]*self.num_poles)
        self.control_margin = np.array([0])
    
        self.vars = self.get_sp_vars()
        self.eqs = self.get_sp_equations()
        self.F = self.get_sp_jacobian()

    def constraint_states(self, state: np.ndarray, control: np.ndarray) -> np.ndarray:
        d_s = state[1]
        dd_s = control[0]
        thetas = state[2::2]
        d_thetas = state[3::2]

        sum1 = sum(self.ms*sin(thetas)*cos(thetas))
        sum2 = sum(self.ms*(self.ls/2)*d_thetas**2*sin(thetas))
        sum3 = sum((self.u_ps*d_thetas*cos(thetas))/(self.ls/2))
        sum4 = sum(self.ms*cos(thetas)**2)

        f = 3/7*(self.g*sum1-7/3*(sum2-self.cart.u_c*d_s)-sum3-(sum4-7/3*self.M)*dd_s)
        torque = f*self.motor.r
        
        return np.array([torque])

    def constraints(self, state: np.ndarray, control: np.ndarray) -> np.ndarray:
        s = state[0]
        d_s = state[1]
        c_state = self.constraint_states(state, control)
        torque = c_state[0]
        
        constraints = np.array([s-(self.state_lower_bound[0]+self.state_margin[0]), self.state_upper_bound[0]-self.state_margin[0]-s, d_s-(self.state_lower_bound[1]+self.state_margin[1]), self.state_upper_bound[1]-self.state_margin[1]-d_s, torque-self.motor.torque_bounds[0], self.motor.torque_bounds[1]-torque])
        return constraints
    
    def clip(self, state: np.ndarray, control: np.ndarray) -> tuple[np.ndarray,np.ndarray]:
        s_ds_clipped = np.clip(state[:2], self.state_lower_bound[:2], self.state_upper_bound[:2])

        d_s = state[1]
        thetas = state[2::2]
        d_thetas = state[3::2]
        clipped_thetas = (thetas + np.pi) % (2*np.pi) - np.pi

        clipped_state = np.concatenate((s_ds_clipped, np.column_stack((clipped_thetas, d_thetas)).flatten()))

        sum1 = sum(self.ms*sin(thetas)*cos(thetas))
        sum2 = sum(self.ms*(self.ls/2)*d_thetas**2*sin(thetas))
        sum3 = sum((self.u_ps*d_thetas*cos(thetas))/(self.ls/2))
        sum4 = sum(self.ms*cos(thetas)**2)

        f_max = self.motor.torque_bounds[1]/self.motor.r
        f_min = self.motor.torque_bounds[0]/self.motor.r

        dd_s_max = (self.g*sum1-7/3*(f_max+sum2-self.cart.u_c*d_s)-sum3)/(sum4-7/3*self.M)
        dd_s_min = (self.g*sum1-7/3*(f_min+sum2-self.cart.u_c*d_s)-sum3)/(sum4-7/3*self.M)

        clipped_control = np.clip(control, np.array([dd_s_min]), np.array([dd_s_max]))

        return clipped_state, clipped_control

    def end_height(self, state: np.ndarray) -> float:
        thetas = state[2::2]
        return sum(self.ls*sin(thetas))
        
    def calculate_error(self, state: np.ndarray, reference: np.ndarray) -> np.ndarray:
        error = reference-state
        error[2::2] = np.arctan2(np.sin(error[2::2]), np.cos(error[2::2]))
        return error

    def get_sp_vars(self):
        return sp.symbols("s d_s " + " ".join([f"theta{i} d_theta{i}" for i in range(self.num_poles)]) + " dd_s")

    def get_sp_equations(self):
        d_s = self.vars[1]
        dd_s = self.vars[-1]
        thetas = self.vars[2::2]
        d_thetas = self.vars[3::2]

        f1 = d_s
        f2 = dd_s
        f_d_thetas = d_thetas       
        dd_thetas = [3/(7*l/2)*(self.g*sp.sin(theta)-dd_s*sp.cos(theta)-u_p*d_theta/(m*l/2)) for l, theta, u_p, m, d_theta in zip(self.ls, thetas, self.u_ps, self.ms, d_thetas)]
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
        d_s = state[1]
        dd_s = control[-1]
        thetas = state[2::2]
        d_thetas = state[3::2]
        dd_thetas = 3/(7*self.ls/2)*(self.g*sin(thetas)-dd_s*cos(thetas)-self.u_ps*d_thetas/(self.ms*self.ls/2))
        d_state = np.concatenate((np.array([d_s, dd_s]), np.column_stack((d_thetas, dd_thetas)).flatten()))

        return d_state

    def linear_differentiate(self, state: np.ndarray, control: np.ndarray, state0: np.ndarray, control0: np.ndarray) -> np.ndarray:
        A, B = self.linearize(state0, control0)
        d_state = state @ A.T + control @ B.T
        return d_state.T
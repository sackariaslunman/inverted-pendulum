from __future__ import annotations
import os
import numpy as np
import casadi as ca
import sympy as sp 
import sympy.physics.mechanics as me
from .utils import sympy2casadi

HASH_MOD = 31
HASH_MAX = 2**32-1

class StepperMotor:
    def __init__(
        self, 
        radius: float,
        velocity_bounds: tuple[float, float],
        velocity_margin: float,
        torque_bounds: tuple[float, float],
        torque_margin: float = 0.1
    ):
        self.r = radius
        self.v_bounds = velocity_bounds
        self.v_margin = velocity_margin
        self.torque_bounds = torque_bounds
        self.torque_margin = torque_margin

    def copy(self) -> StepperMotor:
        return StepperMotor(
            self.r,
            self.v_bounds,
            self.v_margin,
            self.torque_bounds,
            self.torque_margin
        )
    
    def __hash__(self) -> int:
        value = HASH_MOD
        value = HASH_MOD*(hash(self.r) + value)
        value = HASH_MOD*(hash(self.v_bounds[0]) + value)
        value = HASH_MOD*(hash(self.v_bounds[1]) + value)
        value = HASH_MOD*(hash(self.v_margin) + value)
        value = HASH_MOD*(hash(self.torque_bounds[0]) + value)
        value = HASH_MOD*(hash(self.torque_bounds[1]) + value)
        value = HASH_MOD*(hash(self.torque_margin) + value)
        return value % HASH_MAX

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

    def copy(self) -> Cart:
        return Cart(self.m, self.u_c, self.x_bounds, self.x_margin)
    
    def __hash__(self) -> int:
        value = HASH_MOD
        value = HASH_MOD*(hash(self.m) + value)
        value = HASH_MOD*(hash(self.u_c) + value)
        value = HASH_MOD*(hash(self.x_bounds[0]) + value)
        value = HASH_MOD*(hash(self.x_bounds[1]) + value)
        value = HASH_MOD*(hash(self.x_margin) + value)
        return value % HASH_MAX
    
class Pole:
    def __init__(
        self, 
        m: float, 
        l: float, 
        a: float,
        d: float, 
        J: float,
    ):
        self.m = m
        self.l = l
        self.a = a
        self.d = d
        self.J = J

    def copy(self) -> Pole:
        return Pole(self.m, self.l, self.a, self.d, self.J)
    
    def __hash__(self) -> int:
        value = HASH_MOD
        value = HASH_MOD*(hash(self.m) + value)
        value = HASH_MOD*(hash(self.l) + value)
        value = HASH_MOD*(hash(self.a) + value)
        value = HASH_MOD*(hash(self.d) + value)
        value = HASH_MOD*(hash(self.J) + value)
        return value % HASH_MAX

class CartPoleSystem:
    def __init__(
        self,
        cart: Cart,
        motor: StepperMotor,
        poles: list[Pole],
        g: float,
        set_equations: bool = True
    ):
        self.cart = cart
        self.motor = motor
        self.poles = poles
        self.g = g
        self.num_poles = len(poles)
        self.num_states = 2+self.num_poles*2
        self.num_controls = 1
        self.num_constraint_states = 1

        self.m_c = self.cart.m
        self.M = self.cart.m + sum(pole.m for pole in self.poles)
        self.L = sum(pole.l for pole in self.poles)

        self.pole_ms = np.array([pole.m for pole in self.poles])
        self.pole_ls = np.array([pole.l for pole in self.poles])
        self.pole_as = np.array([pole.a for pole in self.poles])
        self.pole_ds = np.array([pole.d for pole in self.poles])
        self.pole_Js = np.array([pole.J for pole in self.poles])

        self.state_lower_bound = np.array([cart.x_bounds[0], motor.v_bounds[0]] + [-2*np.pi, -np.inf]*self.num_poles)
        self.state_upper_bound = np.array([cart.x_bounds[1], motor.v_bounds[1]] + [ 2*np.pi, np.inf]*self.num_poles)

        self.control_lower_bound = np.array([-np.inf])
        self.control_upper_bound = np.array([np.inf])

        self.state_margin = np.array([cart.x_margin*(self.state_upper_bound[0]-self.state_lower_bound[0]), motor.v_margin*(self.state_upper_bound[1]-self.state_lower_bound[1])] + [0, 0]*self.num_poles)
        self.control_margin = np.array([0])

        if set_equations:
            self.set_equations()
    
    def set_equations(self):
        self.set_sp_equations()
        self.set_ca_equations()

    def constraint_states(self, state: np.ndarray, control: np.ndarray) -> np.ndarray:
        values = np.concatenate((state, control))
        c_states = self.ca_constraint_states(*values)
        f = np.array(c_states).flatten().astype(np.float64)[0]      #type: ignore
        torque = f*self.motor.r
        
        return np.array([torque])

    def constraints(self, state: np.ndarray, control: np.ndarray) -> np.ndarray:
        s = state[0]
        d_s = state[1]
        c_state = self.constraint_states(state, control)
        torque = c_state[0]
        
        constraints = np.array([s-(self.state_lower_bound[0]+self.state_margin[0]), \
            self.state_upper_bound[0]-self.state_margin[0]-s, \
            d_s-(self.state_lower_bound[1]+self.state_margin[1]), \
            self.state_upper_bound[1]-self.state_margin[1]-d_s, \
            torque-self.motor.torque_bounds[0], \
            self.motor.torque_bounds[1]-torque])
        return constraints
    
    def clip(self, state: np.ndarray, control: np.ndarray) -> tuple[np.ndarray,np.ndarray]:
        s_ds_clipped = np.clip(state[:2], self.state_lower_bound[:2], self.state_upper_bound[:2])

        d_s = state[1]
        thetas = state[2::2]
        d_thetas = state[3::2]
        clipped_thetas = (thetas + np.pi) % (2*np.pi) - np.pi

        clipped_state = np.concatenate((s_ds_clipped, np.column_stack((clipped_thetas, d_thetas)).flatten()))

        f_max = self.motor.torque_bounds[1]/self.motor.r
        f_min = self.motor.torque_bounds[0]/self.motor.r

        dd_s_max = f_max/self.m_c
        dd_s_min = f_min/self.m_c

        clipped_control = np.clip(control, np.array([dd_s_min]), np.array([dd_s_max]))

        return clipped_state, clipped_control

    def end_height(self, state: np.ndarray) -> float:
        thetas = state[2::2]
        return sum(self.pole_ls*np.sin(thetas))
        
    def calculate_error(self, state: np.ndarray, reference: np.ndarray) -> np.ndarray:
        error = reference-state
        error[2::2] = np.arctan2(np.sin(error[2::2]), np.cos(error[2::2]))
        return error

    def set_sp_equations(self):
        s = me.dynamicsymbols("s")
        d_s = sp.diff(s)
        dd_s = sp.diff(d_s)
        thetas = [me.dynamicsymbols(f"theta{i+1}") for i in range(self.num_poles)]      #type: ignore
        d_thetas = [sp.diff(theta) for theta in thetas]
        dd_thetas = [sp.diff(d_theta) for d_theta in d_thetas]
        tau = sp.symbols("tau")

        pole_pc1s = []
        pole_pc2s = []
        for i, (theta, a) in enumerate(zip(thetas, self.pole_as)):
            prev_1 = 0
            prev_2 = 0
            for prev_l, prev_theta in list(zip(self.pole_ls, thetas))[:i]:
                prev_1 += -prev_l*sp.sin(-prev_theta)       #type: ignore
                prev_2 += prev_l*sp.cos(-prev_theta)        #type: ignore
            pole_pc1s.append(s-a*sp.sin(-theta)+prev_1)     #type: ignore
            pole_pc2s.append(a*sp.cos(-theta)+prev_2)       #type: ignore

        T = 1/2*self.m_c*d_s**2         #type: ignore
        for m, pc1, pc2, J, d_theta in zip(self.pole_ms, pole_pc1s, pole_pc2s, self.pole_Js, d_thetas):
            d_pc1 = sp.diff(pc1)
            d_pc2 = sp.diff(pc2)
            T += 1/2*m*(d_pc1**2 + d_pc2**2) + 1/2*J*d_theta**2     #type: ignore
            
        V = 0   
        for m, pc2 in zip(self.pole_ms, pole_pc2s):
            V += self.g*m*pc2

        R = 0
        prev_w = 0
        for d, d_theta in zip(self.pole_ds, d_thetas):
            R += 1/2*d*(d_theta-prev_w)**2     #type: ignore
            prev_w = d_theta

        eqs = []
        L = T-V
        lh = sp.diff(sp.diff(L, d_s)) - sp.diff(L, s) + sp.diff(R, d_s)     #type: ignore
        rh = tau
        eqs = [lh-rh]
        for theta, d_theta in zip(thetas, d_thetas):
            L = T-V
            lh = sp.diff(sp.diff(L, d_theta)) - sp.diff(L, theta) + sp.diff(R, d_theta)     #type: ignore
            rh = 0
            eqs.append(lh-rh)

        sp_vars = [s, d_s] + [item for pair in zip(thetas, d_thetas) for item in pair] + [dd_s] + dd_thetas
        sols = sp.solve(eqs, dd_thetas+[tau])
        sp_sols = [sp.simplify(sols[dd_theta]) for dd_theta in dd_thetas] + [sp.simplify(sols[tau])]
    
        pure_s, pure_d_s, pure_dd_s = sp.symbols("s d_s dd_s")
        pure_thetas = [sp.symbols(f"theta{i+1}") for i in range(self.num_poles)]        #type: ignore
        pure_d_thetas = [sp.symbols(f"d_theta{i+1}") for i in range(self.num_poles)]    #type: ignore
        pure_dd_thetas = [sp.symbols(f"dd_theta{i+1}") for i in range(self.num_poles)]  #type: ignore

        self.sp_vars = [pure_s, pure_d_s] + [item for pair in zip(pure_thetas, pure_d_thetas) for item in pair] + [pure_dd_s] + pure_dd_thetas
        subs_dict = dict(zip(sp_vars, self.sp_vars))
        self.sp_sols = [sol.subs(subs_dict) for sol in sp_sols]
    
    def set_ca_equations(self):
        s = ca.SX.sym("s") #type: ignore
        d_s = ca.SX.sym("d_s") #type: ignore
        dd_s = ca.SX.sym("dd_s") #type: ignore
        thetas = [ca.SX.sym(f"theta{i+1}") for i in range(self.num_poles)]          #type: ignore
        d_thetas = [ca.SX.sym(f"d_theta{i+1}") for i in range(self.num_poles)]      #type: ignore

        self.ca_vars = [s, d_s] + [item for pair in zip(thetas, d_thetas) for item in pair] + [dd_s]
        self.ca_state_vars = [s, d_s] + [item for pair in zip(thetas, d_thetas) for item in pair]
        self.ca_control_vars = [dd_s]
        self.ca_d_state_vars = [d_s, dd_s]
        self.ca_constraint_vars = [sympy2casadi(self.sp_sols[-1], sp.Matrix(self.sp_vars[:2+2*self.num_poles+1]), ca.vertcat(*self.ca_vars[:2+2*self.num_poles+1]))]

        for d_theta, sol in zip(d_thetas, self.sp_sols[:-1]):
            dd_theta = sympy2casadi(sol, sp.Matrix(self.sp_vars[:2+2*self.num_poles+1]), ca.vertcat(*self.ca_vars[:2+2*self.num_poles+1]))
            self.ca_vars.append(dd_theta)
            self.ca_d_state_vars.extend([d_theta, dd_theta])
            
        self.ca_differentiate = ca.Function("differentiate", self.ca_state_vars + [dd_s], self.ca_d_state_vars)
        self.ca_constraint_states = ca.Function("constraint_states", self.ca_state_vars + [dd_s], self.ca_constraint_vars)     

        vars = ca.vertcat(*(self.ca_state_vars + self.ca_control_vars))
        eqs = ca.vertcat(*self.ca_d_state_vars)
        self.F = ca.jacobian(eqs, vars)

    def linearize(self, state0: np.ndarray, control0: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
        vals = ca.vertcat(*state0, *control0)
        vars = ca.vertcat(*(self.ca_state_vars + self.ca_control_vars))
        F_eval = ca.substitute(self.F, vars, vals)
        F_eval_np = np.array(ca.DM(F_eval))
        A = np.array(F_eval_np[:,:-1]).astype(np.float64)
        B = np.vstack(np.array(F_eval_np[:,-1]).astype(np.float64))     #type: ignore
        return A, B
    
    def differentiate(self, state: np.ndarray, control: np.ndarray) -> np.ndarray:
        values = np.concatenate((state, control))
        d_state = self.ca_differentiate(*values)
        return np.array(d_state).flatten().astype(np.float64)

    def linear_differentiate(self, state: np.ndarray, control: np.ndarray, state0: np.ndarray, control0: np.ndarray) -> np.ndarray:
        A, B = self.linearize(state0, control0)
        d_state = state @ A.T + control @ B.T
        return d_state.T
    
    def copy(self):
        return CartPoleSystem(self.cart.copy(), self.motor.copy(), [pole.copy() for pole in self.poles], self.g)
    
    def __hash__(self) -> int:
        value = HASH_MOD
        value = HASH_MOD*(hash(self.cart) + value)
        value = HASH_MOD*(hash(self.motor) + value)
        for pole in self.poles:
            value = HASH_MOD*(hash(pole) + value)
        value = HASH_MOD*(hash(self.g) + value)
        return value % HASH_MAX
    
    def export_equations(self, path: str):
        vars = []
        for var in self.sp_vars:
            vars.append(str(var))

        sols = []
        for sol in self.sp_sols:
            sols.append(str(sol))

        with open(f"{path}/vars_{hash(self)}.txt", "w") as f:   
            f.write("\n".join(vars))
        with open(f"{path}/sols_{hash(self)}.txt", "w") as f:
            f.write("\n".join(sols))

    def check_equations(self, path: str) -> bool:
        return os.path.exists(f"{path}/vars_{hash(self)}.txt") and os.path.exists(f"{path}/sols_{hash(self)}.txt")

    def import_equations(self, path: str):
        with open(f"{path}/vars_{hash(self)}.txt", "r") as f:
            vars = f.read().split("\n")
        with open(f"{path}/sols_{hash(self)}.txt", "r") as f:
            sols = f.read().split("\n")
        
        self.sp_vars = [sp.Symbol(var) for var in vars]
        self.sp_sols = [sp.sympify(sol) for sol in sols]

        self.set_ca_equations()
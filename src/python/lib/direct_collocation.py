import numpy as np
from scipy.interpolate import CubicSpline #type: ignore
from .cartpolesystem import CartPoleSystem
import casadi as ca
import sympy as sp
from .utils import sympy2casadi

class CartPoleStepperMotorDirectCollocation():
    def __init__(self, N: int, N_collocation: int, system: CartPoleSystem, tolerance=1e-6):
        self.N = N
        self.N_collocation = N_collocation
        self.N_states = system.num_states
        self.N_controls = system.num_controls
        self.tolerance = tolerance
        self.system = system

    def differentiate(self, x, u):
        s = x[0]
        d_s = x[1]
        dd_s = u[0]

        d_state = [d_s, dd_s]
        ca_vars = [s, d_s]
        for i in range(self.system.num_poles):
            theta = x[2+2*i]
            d_theta = x[3+2*i]
            ca_vars.extend([theta, d_theta])
        ca_vars.append(dd_s)

        for i, sol in enumerate(self.system.sp_sols):
            d_theta = x[3+2*i]
            dd_theta = sympy2casadi(sol, sp.Matrix(self.system.sp_vars[:2+2*self.system.num_poles+1]), ca.vertcat(*ca_vars[:2+2*self.system.num_poles+1]))
            d_state.extend([d_theta, dd_theta])          

        return d_state
    
    def constraint_states(self, x, u):
        dd_s = u[0]
        f = self.system.m_c*dd_s
        torque = f*self.system.motor.r
        
        return np.array([torque])

    def make_guess(self, x0: np.ndarray, r: np.ndarray, x_guess: np.ndarray, u_guess: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
        self.x0 = x0
        self.r = r

        if x_guess.size == 0 or u_guess.size == 0:
            x_guess = np.linspace(x0, r, self.N_collocation)
            u_guess = np.zeros((self.N_collocation, self.system.num_controls))
        
        self.x_guess = x_guess
        self.u_guess = u_guess
        self.opti.set_initial(self.xs, x_guess)
        self.opti.set_initial(self.us, u_guess)

        return x_guess, u_guess

    def set_objective_function(self):
        obj = 0
        for i in range(self.N_collocation-1):
            obj += (self.us[i,0]**2+self.us[i+1,0]**2)*self.h/2
        self.opti.minimize(obj)

    def set_eq_constraints(self):
        self.opti.subject_to(self.xs[0,0] == self.x0[0])
        self.opti.subject_to(self.xs[-1,0] == self.r[0])
        self.opti.subject_to(self.xs[0,1] == self.x0[1])
        self.opti.subject_to(self.xs[-1,1] == self.r[1])

        for i in range(self.system.num_poles*2):
            if i % 2 == 0:
                self.opti.subject_to(ca.arctan2(ca.sin(self.x0[i+2]-self.xs[0,i+2]),ca.cos(self.x0[i+2]-self.xs[0,i+2])) == 0)
                self.opti.subject_to(ca.arctan2(ca.sin(self.r[i+2]-self.xs[-1,i+2]),ca.cos(self.r[i+2]-self.xs[-1,i+2])) == 0)
            else:
                self.opti.subject_to(self.xs[0,i+2] == self.x0[i+2])
                self.opti.subject_to(self.xs[-1,i+2] == self.r[i+2])

    def set_ineq_constraints(self):
        for i in range(self.N_collocation):
            s = self.xs[i,0]
            last_s = self.xs[i-1,0]
            d_s = self.xs[i,1]
            last_d_s = self.xs[i-1,1]
            dd_s = self.us[i,0]
            last_dd_s = self.us[i-1,0]
            self.opti.subject_to(self.opti.bounded(self.system.state_lower_bound[0]+self.system.state_margin[0],s,self.system.state_upper_bound[0]-self.system.state_margin[0]))
            self.opti.subject_to(self.opti.bounded(self.system.state_lower_bound[1]+self.system.state_margin[1],d_s,self.system.state_upper_bound[1]-self.system.state_margin[1]))

            d_x = self.differentiate(self.xs[i,:],self.us[i,:])
            last_d_x = self.differentiate(self.xs[i-1,:],self.us[i-1,:])

            if i > 0:
                self.opti.subject_to((d_s+last_d_s)*self.h/2 == (s-last_s)) 
                self.opti.subject_to((dd_s+last_dd_s)*self.h/2 == (d_s-last_d_s)) 

                for j in range(self.system.num_poles):
                    theta = self.xs[i,2+2*j]
                    last_theta = self.xs[i-1,2+2*j]
                    d_theta = self.xs[i,3+2*j]
                    last_d_theta = self.xs[i-1,3+2*j]
                    
                    dd_theta = d_x[3+2*j]
                    last_dd_theta = last_d_x[3+2*j]

                    self.opti.subject_to((d_theta+last_d_theta)*self.h/2 == (theta-last_theta))
                    self.opti.subject_to((dd_theta+last_dd_theta)*self.h/2 == (d_theta-last_d_theta))
            
            constraint_state = self.constraint_states(self.xs[i,:],self.us[i,:])
            f = constraint_state[0]
            torque = f*self.system.motor.r
            # self.opti.subject_to(self.opti.bounded(self.system.motor.torque_bounds[0]*(1-self.system.motor.torque_margin),torque,self.system.motor.torque_bounds[1]*(1-self.system.motor.torque_margin))) #type: ignore

    def make_solver(self, end_time: float, x0, r, x_guess = np.array([]), u_guess = np.array([])):
        self.opti = ca.Opti()   
        self.xs = self.opti.variable(self.N_collocation,self.system.num_states)
        self.us = self.opti.variable(self.N_collocation,self.system.num_controls)
        self.h = end_time/self.N_collocation
        self.end_time = end_time
    
        self.x0 = x0
        self.r = r

        self.make_guess(x0, r, x_guess, u_guess)
        self.set_objective_function()
        self.set_eq_constraints()
        self.set_ineq_constraints()

        self.opti.solver("ipopt")
        sol = self.opti.solve()

        x_optimal_raw = sol.value(self.xs)
        u_optimal_raw = sol.value(self.us)

        time_collocation = np.linspace(0, self.end_time, self.N_collocation)
        time = np.linspace(0, self.end_time, self.N)

        states = np.vstack([
            CubicSpline(time_collocation, s_row)(time) for s_row in x_optimal_raw.T
        ]).T
        controls = np.vstack(np.interp(time, time_collocation, u_optimal_raw))

        return states, controls
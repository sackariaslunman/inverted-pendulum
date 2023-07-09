import numpy as np
from scipy.optimize import minimize #type: ignore
from scipy.interpolate import CubicSpline #type: ignore
from typing import Callable
from .cartpolesystem import CartPoleStepperMotorSystem
import casadi as ca

class DirectCollocation():
    def __init__(self, N: int, dynamics: Callable[[np.ndarray,np.ndarray],np.ndarray], N_states: int, N_controls: int, constraints: Callable[[np.ndarray,np.ndarray],np.ndarray], calculate_boundary_error: Callable[[np.ndarray,np.ndarray],np.ndarray], tolerance=1e-6):
        self.N = N
        self.dynamics = dynamics
        self.vectorized_dynamics = np.vectorize(dynamics, signature='(n),(m)->(n)')
        self.calculate_boundary_error = calculate_boundary_error
        self.N_states = N_states
        self.N_controls = N_controls

        self.vectorized_constraints = np.vectorize(constraints, signature='(n),(m)->(l)')
        self.tolerance = tolerance

    def make_guess(self, initial_state, final_state, state_guess: np.ndarray, control_guess: np.ndarray):
        self.initial_state = initial_state
        self.final_state = final_state

        if state_guess.size == 0 or control_guess.size == 0:
            state_guess = np.linspace(initial_state, final_state, self.N).flatten()
            control_guess = np.zeros((self.N, self.N_controls)).flatten()
        self.initial_variables = np.hstack([state_guess, control_guess])

        return self.initial_variables
    
    def variables_to_state_control(self, variables):
        states = variables[:self.N_states*self.N].reshape((self.N, self.N_states))
        controls = variables[self.N_states*self.N:].reshape((self.N, self.N_controls))
        return states, controls

    def objective_function(self, variables):
        states, controls = self.variables_to_state_control(variables)
        cost = (controls[:-1]**2+controls[1:]**2).sum()*self.h/2
        return cost

    def eq_constraints(self, variables):
        states, controls = self.variables_to_state_control(variables)

        constraints = np.zeros((2+self.N-1, self.N_states))

        constraints[0] = self.calculate_boundary_error(states[0], self.initial_state)
        constraints[1] = self.calculate_boundary_error(states[-1], self.final_state)

        dynamics = self.vectorized_dynamics(states, controls)

        next_states = states[1:]
        current_states = states[:-1]
        next_dynamics = dynamics[1:]
        current_dynamics = dynamics[:-1]

        dynamic_constraints = next_states-current_states-self.h/2*(next_dynamics+current_dynamics)
        constraints[2:] = dynamic_constraints

        return constraints.flatten()

    def ineq_constraints(self, variables):
        states, controls = self.variables_to_state_control(variables)
        result = (self.vectorized_constraints(states, controls)).flatten()
        return result

    def make_controller(self, time, initial_state, final_state, N_spline=None, state_guess = np.array([]), control_guess = np.array([])):
        self.time = time
        self.N_spline = N_spline
        self.h = (self.time)/self.N

        self.initial_state = initial_state
        self.final_state = final_state

        self.make_guess(initial_state, final_state, state_guess, control_guess)

        constraints = [
            {"type": "eq", "fun": self.eq_constraints},
            {"type": "ineq", "fun": self.ineq_constraints},
        ]
        sol = minimize(
            fun=self.objective_function,
            x0=self.initial_variables,
            method="SLSQP",
            constraints=constraints,
            tol=self.tolerance
        )
        states, controls = self.variables_to_state_control(sol.x)
        N_time = np.linspace(0, time, self.N)

        if N_spline:
            N_spline_time = np.linspace(0, time, N_spline)

            state_spline = np.vstack([
                CubicSpline(N_time, s_row)(N_spline_time) for s_row in states.T
            ]).T
            control_spline = np.vstack([
                np.interp(N_spline_time, N_time, c_row) for c_row in controls.T
            ]).T
        else:
            state_spline, control_spline = states, controls

        return state_spline, control_spline

class CartPoleStepperMotorDirectCollocation():
    def __init__(self, N: int, N_collocation: int, system: CartPoleStepperMotorSystem, tolerance=1e-6):
        self.N = N
        self.N_collocation = N_collocation
        self.N_states = system.num_states
        self.N_controls = system.num_controls
        self.tolerance = tolerance
        self.system = system
        self.calculate_jacobian()

    def differentiate(self, x, u):
        d_s = x[1]
        dd_s = u[0]
        next_x = [0 for _ in range(self.system.num_states)]
        next_x[0] = d_s
        next_x[1] = dd_s
        for i in range(self.system.num_poles):
            theta = x[2+2*i]
            d_theta = x[3+2*i]
            u_p = self.system.u_ps[i]
            l = self.system.ls[i]
            m = self.system.ms[i]
            dd_theta = 3/(7*l/2)*(self.system.g*ca.sin(theta)-dd_s*ca.cos(theta)-u_p*d_theta/(m*l/2))
            next_x[2+2*i] = d_theta
            next_x[3+2*i] = dd_theta
        return next_x

    def constraint_states(self, x, u):
        d_s = x[1]
        dd_s = u[0]
        sum1 = 0
        sum2 = 0
        sum3 = 0
        sum4 = 0

        for j in range(self.system.num_poles):
            theta = x[2+2*j]
            d_theta = x[3+2*j]
            l = self.system.ls[j]
            m = self.system.ms[j]
            u_p = self.system.u_ps[j]

            sum1 += m*ca.sin(theta)*ca.cos(theta)
            sum2 += m*l/2*d_theta**2*ca.sin(theta)
            sum3 += u_p*d_theta*ca.cos(theta)/(l/2)
            sum4 += m*ca.cos(theta)**2

        f = 3/7*(self.system.g*sum1-7/3*(sum2-self.system.cart.u_c*d_s)-sum3-(sum4-7/3*self.system.M)*dd_s)
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
            self.opti.subject_to(self.opti.bounded(self.system.motor.torque_bounds[0]*(1-self.system.motor.torque_margin),torque,self.system.motor.torque_bounds[1]*(1-self.system.motor.torque_margin))) #type: ignore

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
    
    def calculate_jacobian(self):
        x_vars = []
        x_vars.append(ca.SX.sym("s")) #type: ignore
        x_vars.append(ca.SX.sym("d_s"))  #type: ignore
        for i in range(self.system.num_poles):
            x_vars.append(ca.SX.sym(f"theta_{i}")) #type: ignore
            x_vars.append(ca.SX.sym(f"d_theta_{i}")) #type: ignore
        u_vars = [ca.SX.sym("u")] #type: ignore

        eqs = self.differentiate(x_vars, u_vars)
        vars = x_vars + u_vars
        vars = ca.vertcat(*vars)
        eqs = ca.vertcat(*eqs)

        J = ca.jacobian(eqs, vars)
        self.J = J
        self.vars = vars
        self.eqs = eqs

    def linearize(self, x_vals: np.ndarray, u_vals: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
        vals = ca.vertcat(*x_vals, *u_vals)
        J_eval = ca.substitute(self.J, self.vars, vals)
        J_eval_np = np.array(ca.DM(J_eval))
        A = np.array(J_eval_np[:,:-1]).astype(np.float64)
        B = np.vstack(np.array(J_eval_np[:,-1]).astype(np.float64)) # type: ignore
        return A, B
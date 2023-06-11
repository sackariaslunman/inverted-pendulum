import numpy as np
from scipy.optimize import minimize
from scipy.interpolate import CubicSpline
from typing import Callable
from time import perf_counter

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

        self.times = {
            "variables": [],
            "objective": [],
            "eq_constraints": [],
            "ineq_constraints": [],
        }

    def make_guess(self, initial_state, final_state, state_guess: np.ndarray, control_guess: np.ndarray):
        self.initial_state = initial_state
        self.final_state = final_state

        if state_guess.size == 0 or control_guess.size == 0:
            state_guess = np.linspace(initial_state, final_state, self.N).flatten()
            control_guess = np.zeros((self.N, self.N_controls)).flatten()
        self.initial_variables = np.hstack([state_guess, control_guess])

        return self.initial_variables
    
    def variables_to_state_control(self, variables):
        start_time = perf_counter()
        states = variables[:self.N_states*self.N].reshape((self.N, self.N_states))
        controls = variables[self.N_states*self.N:].reshape((self.N, self.N_controls))
        self.times["variables"].append(perf_counter()-start_time)
        return states, controls

    def objective_function(self, variables):
        _, controls = self.variables_to_state_control(variables)
        start_time = perf_counter()
        cost = (controls[:-1]**2+controls[1:]**2).sum()*self.h/2
        self.times["objective"].append(perf_counter()-start_time)
        return cost

    def eq_constraints(self, variables):
        states, controls = self.variables_to_state_control(variables)
        start_time = perf_counter()

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
        self.times["eq_constraints"].append(perf_counter()-start_time)

        return constraints.flatten()

    def ineq_constraints(self, variables):
        states, controls = self.variables_to_state_control(variables)
        start_time = perf_counter()
        result = (self.vectorized_constraints(states, controls)).flatten()
        self.times["ineq_constraints"].append(perf_counter()-start_time)
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
            # method="SLSQP",
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

        keys = list(self.times.keys())
        for key in keys:
            self.times[f"{key}_mean"] = np.mean(self.times[key])
            self.times[f"{key}_total"] = sum(self.times[key])
            self.times[key] = len(self.times[key])

        print(self.times)

        return state_spline, control_spline
import numpy as np
from scipy.optimize import minimize
from scipy.interpolate import CubicSpline

class DirectCollocation():
    def __init__(self, N: int, dynamics, N_states: int, N_controls: int, state_lower_bound, state_upper_bound, control_lower_bound, control_upper_bound, tolerance=1e-6):
        self.N = N
        self.dynamics = dynamics
        self.N_states = N_states
        self.N_controls = N_controls

        self.state_lower_bound = state_lower_bound.T[0]
        self.state_upper_bound = state_upper_bound.T[0]
        self.control_lower_bound = control_lower_bound.T[0]
        self.control_upper_bound = control_upper_bound.T[0]
        self.tolerance = tolerance

    def make_guess(self, initial_state, final_state):
        self.initial_state = initial_state.T[0]
        self.final_state = final_state.T[0]

        self.initial_variables = np.hstack([
            np.hstack([np.linspace(initial, final, self.N) for initial, final in zip(self.initial_state, self.final_state)]),
            np.zeros(self.N_controls*self.N),
        ])
        return self.initial_variables
    
    def variables_to_state_control(self, variables):
        state = variables[:self.N_states*self.N]
        control = variables[self.N_states*self.N:(self.N_states+self.N_controls)*self.N]
        return state, control

    def objective_function(self, variables):
        state, control = self.variables_to_state_control(variables)
        result = 0
        for i in range(self.N_controls):
            for k in range(self.N-1):
                result += (self.h/2)*(control[self.N*i+k]**2+control[self.N*i+k+1]**2)
        return result

    def eq_constraints(self, variables):
        state, control = self.variables_to_state_control(variables)
        constraints = []

        for k in range(self.N_states):
            constraints.append(state[self.N*k]-self.initial_state[k])
            constraints.append(state[self.N*k+self.N-1]-self.final_state[k])

        state_current = state[0::self.N]
        control_current = control[0::self.N]
        dynamics_current = self.dynamics(np.vstack(state_current), np.vstack(control_current)).T[0]

        for k in range(self.N-1):
            state_next = state[k+1::self.N]
            control_next = control[k+1::self.N]
            dynamics_next = self.dynamics(np.vstack(state_next), np.vstack(control_next)).T[0]

            dynamic_constraint = state_next-state_current-self.h/2*(dynamics_next+dynamics_current)
            constraints.extend(list(dynamic_constraint))

            state_current = state_next
            control_current = control_next
            dynamics_current = dynamics_next
        
        return constraints

    def ineq_constraints(self, variables):
        state, control = self.variables_to_state_control(variables)
        constraints = []

        for k in range(self.N):
            current_state = state[k::self.N]
            current_control = control[k::self.N]

            constraints.extend(list(np.array(current_state)-self.state_lower_bound))
            constraints.extend(list(self.state_upper_bound-np.array(current_state)))
            constraints.extend(list(np.array(current_control)-self.control_lower_bound))
            constraints.extend(list(self.control_upper_bound-np.array(current_control)))

        return constraints

    def make_controller(self, time, initial_state, final_state, N_spline=None):
        self.time = time
        self.N_spline = N_spline
        self.h = (self.time)/self.N

        self.make_guess(initial_state, final_state)
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
        state, control = self.variables_to_state_control(sol.x)
        N_time = np.linspace(0, time, self.N)

        if N_spline:
            N_spline_time = np.linspace(0, time, N_spline)

            state_spline = np.vstack([
                CubicSpline(N_time, s_row)(N_spline_time) for s_row in state.reshape((self.N_states, self.N))
            ])
            control_spline = np.vstack([
                np.interp(N_spline_time, N_time, c_row) for c_row in control.reshape((self.N_controls, self.N))
            ])
        else:
            state_spline, control_spline = np.vstack(state), np.vstack(control)

        return state_spline, control_spline
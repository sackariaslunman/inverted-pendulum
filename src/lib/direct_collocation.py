import numpy as np
from scipy.optimize import minimize
from scipy.interpolate import CubicSpline

class DirectCollocation():
    def __init__(self, N: int, dynamics, N_states: int, N_controls: int):
        self.N = N
        self.dynamics = dynamics
        self.N_states = N_states
        self.N_controls = N_controls

    def make_guess(self, start_state, final_state, start_control, final_control):
        self.start_state = start_state.T[0]
        self.final_state = final_state.T[0]
        self.start_control = start_control.T[0]
        self.final_control = final_control.T[0]

        self.initial_variables = np.hstack([
            np.hstack([np.linspace(start, final, self.N) for start, final in zip(self.start_state, self.final_state)]),
            np.hstack([np.linspace(start, final, self.N) for start, final in zip(self.start_control, self.final_control)]),
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
            constraints.append(state[self.N*k]-self.start_state[k])
            constraints.append(state[self.N*k+self.N-1]-self.final_state[k])

        for k in range(self.N_controls):
            constraints.append(control[self.N*k]-self.start_control[k])
            constraints.append(control[self.N*k+self.N-1]-self.final_control[k])

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
        pass

    def make_controller(self, time, N_spline, start_state, final_state, start_control, final_control):
        self.time = time
        self.N_spline = N_spline
        self.h = (self.time)/self.N
        self.make_guess(start_state, final_state, start_control, final_control)
        constraints_dict = {"type": "eq", "fun": self.eq_constraints}

        sol = minimize(
            fun=self.objective_function,
            x0=self.initial_variables,
            method="SLSQP",
            constraints=constraints_dict
        )
        state, control = self.variables_to_state_control(sol.x)
        N_time = np.linspace(0, time, self.N)
        N_spline_time = np.linspace(0, time, N_spline)

        state_spline = np.vstack([
            CubicSpline(N_time, s_row)(N_spline_time) for s_row in state.reshape((self.N_states, self.N))
        ])
        control_spline = np.vstack([
            np.interp(N_spline_time, N_time, c_row) for c_row in control.reshape((self.N_controls, self.N))
        ])

        return state_spline, control_spline
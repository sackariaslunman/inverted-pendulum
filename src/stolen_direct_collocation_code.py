import numpy as np 
import scipy.optimize

class DoubleIntegratorMinimumForceTrapezoidal(object):
    """
    @brief      
    A class that contains the functions that define the following optimization problem 
    for a double integrator moving to position 1 with minimum squared acceleration
    @details
    The continuous version of this problem is:
    \min_{u(t)} \int_u(t)^2 dt
    s.t. x(0) = 0, x(1) = 1
         v(0) = 0, v(1) = 0
         \dot{x}(t) = v(t)
         \dot{v}(t) = u(t)
    This class formulates the discrete version by replacing integrals with trapezoidal approximations with N knot points
    \min{\{u_0, \hdots, u_{N-1}\}, \{x_0, \hdots, x_{N-1}\}, \{v_0, \hdots, v_{N-1}\}} 
        \sum_{k=0}^{N-1} 0.5 * (t_{k+1} - t_{k}) * (u_{k}^2 + u_{k+1}^2)
    s.t. x_{0} = 0, x_{N-1} = 1
         v_{0} = 0, v_{N-1} = 0
         x_{k+1}-x_k = 0.5*(t_{k+1}-t_k)*(v_{k+1} + v_k) for k=0...N-1
         v_{k+1}-v_k = 0.5*(t_{k+1}-t_k)*(u_{k+1} + u_k) for k=0...N-1
    All the decision variables {u, x, v} will be represented by one big vector of length 3*N
    where the first N points (variables[:N]) is u_0 through u_{N-1}
    Then, the next 2N points (variables[N:]) is a the tuples (x_0, v_0) through (x_{N-1}, v_{N-1})
    thus, variables[N::2] is x_0 through x_{N-1}
    and variables[N+1::2] is v_0 through v_{N-1}
    It is layed out this way, so that the state variables (x, v) for a single timestep are all together
    """

    def __init__(self, N):
        """
        @brief      constructs the discrete problem
        @param      N     number of knot points to use
        """
        self.N = N

        # the times, {t_0, ..., t_{N-1}}
        self.t = np.linspace(0, 1, N) 

        # the delta times
        self.h = self.t[1:] - self.t[:-1]

    def obj_func(self, variables):
        """
        @brief      the function to minimize
        \sum_{k=0}^{N-1} 0.5 * (t_{k+1} - t_{k}) * (u_{k}^2 + u_{k+1}^2)
        @param      variables  The decision variables (see class documentation for how its layed out)
        @return     the value of the objective function
        """
        N = self.N
        u = variables[0:self.N]

        obj = 0
        for i in range(N-1):
            obj += 0.5*(self.h[i]) * (u[i]**2 + u[i+1]**2)
        return obj

    def eq_constraint_func(self, variables):
        """
        @brief      the equality constraints 
        x[0] = 0
        v[0] = 0
        x[N-1] = 1
        v[N-1] = 0
        x_{k+1}-x_k = 0.5*(t_{k+1}-t_k)*(v_{k+1} + v_k) for k=0...N-1
        v_{k+1}-v_k = 0.5*(t_{k+1}-t_k)*(u_{k+1} + u_k) for k=0...N-1
        
        @param      variables  The decision variables (see class documentation for how its layed out)
        @return     the value of the constraint function. It is a vector valued function
        where the constraints[0:4] are the initial condition constraints
        and the rest are the dynamics feasibility constraints
        """
        N = self.N
        u = variables[0:N]
        pos = variables[N::2]
        vel = variables[N+1::2]

        constraints = np.zeros(4 + (N-1)*2)
        constraints[0] = pos[0] # x[0] = 0
        constraints[1] = vel[0] # v[0] = 0
        constraints[2] = pos[N-1] - 1 # x[N] = 1
        constraints[3] = vel[N-1] # v[N] = 0
        for i in range(N-1):
            constraints[4 + i] = (pos[i+1]-pos[i]) - 0.5*(self.h[i])*(vel[i] + vel[i+1])
            constraints[4 + (N-1) + i] = (vel[i+1]-vel[i]) - 0.5*(self.h[i])*(u[i] + u[i+1])

        return constraints

    def initialize_variables(self):
        """
        @brief      initializes variables for the optimization problem
        u is initialized to all zeros
        x is set to be the same as t
        v is set to be 0 for all time
        
        @return     Initial variables
        length 3*N np.array 
        [0:N] controls u
        [N::2] positions x
        [N+1::2] velocities v
        """
        N = self.N
        variables = np.zeros(3*N)

        # populating positions
        variables[N::2] = self.t
        variables[N+1::2] = 0

        return variables



def solve_problem(N):
    """
    @brief      convience function to solve the problem with scipy.optimize
    as well as compute the absolute error
    @param      N     number of knot points
    @return     times, scipy solution, position error, velocity error
    """

    problem = DoubleIntegratorMinimumForceTrapezoidal(N)
    variables = problem.initialize_variables()

    constraints_dict = {'type': 'eq',
                        'fun': problem.eq_constraint_func}

    sol = scipy.optimize.minimize(
        fun=problem.obj_func,
        x0=variables,
        method="SLSQP",
        constraints=constraints_dict)

    if not sol.success:
        print("Solution not found.")
        print(sol.message)
        raise Exception("Solution not found.")

    # print("Solution found in " + str(sol.nit) + " iterations.")
    sol_vars = sol.x
    sol_u = sol_vars[:N]
    sol_pos = sol_vars[N::2]
    sol_vel = sol_vars[N+1::2]

    import matplotlib.pyplot as plt

    analytic_control = 6 - 12*problem.t
    analytic_pos = 3*np.square(problem.t) - 2*np.power(problem.t, 3)
    analytic_vel = 6*problem.t - 6*np.square(problem.t)

    # computing errors
    # \int |pos - analytic_pos| dt
    # \int |vel - analytic_vel| dt
    # using trapezoidal approximation
    pos_err_abs = np.abs(sol_pos - analytic_pos)
    pos_err = np.sum((pos_err_abs[1:] + pos_err_abs[:-1]) * problem.h)
    vel_err_abs = np.abs(sol_vel - analytic_vel)
    vel_err = np.sum((vel_err_abs[1:] + vel_err_abs[:-1]) * problem.h)

    return problem.t, sol, pos_err, vel_err


if __name__ == '__main__':
    import matplotlib.pyplot as plt

    # for plotting a single trajectory
    N = 11
    times, sol, _, _ = solve_problem(N)
    sol_vars = sol.x
    sol_u = sol_vars[:N]
    sol_pos = sol_vars[N::2]
    sol_vel = sol_vars[N+1::2]
    print("Solved with " + str(N) + " knot points in " + str(sol.nit) + " iterations")

    analytic_t_plot = np.linspace(0, 1, 100)
    analytic_control_plot = 6 - 12*analytic_t_plot
    analytic_pos_plot = 3*np.square(analytic_t_plot) - 2*np.power(analytic_t_plot, 3)
    analytic_vel_plot = 6*analytic_t_plot - 6*np.square(analytic_t_plot)

    plt.figure(1)

    plt.subplot(3, 1, 1)

    plt.title("Trajectory with "  + str(N) + " knot points")

    plt.plot(analytic_t_plot, analytic_control_plot, label='analytic')
    plt.plot(times, sol_u, linestyle="--", label='optimizer')
    plt.ylabel("control (m/s/s)")
    plt.legend()

    plt.subplot(3, 1, 2)
    plt.plot(analytic_t_plot, analytic_pos_plot, label='analytic')
    plt.plot(times, sol_pos, linestyle="--", label='optimizer')
    plt.ylabel("position (m)")

    plt.subplot(3, 1, 3)
    plt.plot(analytic_t_plot, analytic_vel_plot, label='analytic')
    plt.plot(times, sol_vel, linestyle="--", label='optimizer')
    plt.ylabel("velocity (m)")
    plt.xlabel("time (s)")

    # for plotting error with number of knot positions
    pos_errors = []
    vel_errors = []
    sol_nit = []
    N_list = np.arange(5, 20)
    for N in N_list:
        _, sol, pos_err, vel_err = solve_problem(N)
        pos_errors.append(pos_err)
        vel_errors.append(vel_err)
        sol_nit.append(sol.nit)

    plt.figure(2)
    plt.subplot(2, 1, 1)
    plt.plot(N_list, pos_errors, label="pos err")
    plt.plot(N_list, vel_errors, label="vel err")
    plt.ylabel("err")
    plt.legend()

    plt.subplot(2, 1, 2)
    plt.plot(N_list, sol_nit, label="pos err")
    plt.ylabel("Number of optimizer iterations")
    plt.xlabel("N (knot points)")

    plt.show()
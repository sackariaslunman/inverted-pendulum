import numpy as np
from numpy import radians
from threading import Thread
from .cartpolesystem import CartPoleStepperMotorSystem
from .cartpolesimulator import CartPoleSimulator
from .direct_collocation import CartPoleStepperMotorDirectCollocation
from .regulators import LQR

class CartPoleController:
    def __init__(self, simulator: CartPoleSimulator, dt: float):
        self._simulator = simulator
        self._system = simulator.system
        self._simulator.get_control = self.calculate_control
        self._thread = Thread(target=self._run_loop)
        self._is_running = False
        self._dt = dt

        self._control_enabled = False
        self._is_in_trajectory = False

        self._trajectory_calculating = False
        self._trajectory_count = 0
        self._trajectory_max = 0
        self._trajectory_states = np.array([])
        self._trajectory_controls = np.array([])
        self._trajectory_K_ds = np.array([])
        self._target_state = np.array([])
        self._target_K = np.array([])

    @property
    def dt(self) -> float:
        return self._dt
    
    def disable_control(self):
        if not self._is_running:
            return
        self._control_enabled = False

    def create_trajectory(self, pos: float, pole_pos: list[bool]):
        if self._trajectory_calculating or not self._is_running or self._is_in_trajectory or not self._simulator.running:
            return
        
        self._trajectory_calculating = True
        end_time = 3
        dt_collocation = 0.03
        N_collocation = int(end_time/dt_collocation)+1

        pole_states = np.array([[float(0 if pos else radians(180)), 0.0] for pos in pole_pos]).flatten()
        target_state = np.array([pos, 0] + pole_states.tolist())

        N = int(end_time/self.dt)

        system = self._system
        direct_collocation = CartPoleStepperMotorDirectCollocation(N, N_collocation, system, 0.0001)

        x0 = self._simulator.state
        states, controls = direct_collocation.make_solver(end_time, x0, target_state)

        C = np.diag([1, 1]+[1, 1]*system.num_poles)
        D = np.zeros((1, 1))
        Q = np.diag([100, 10]+[100, 10]*system.num_poles)
        R = np.diag([1])

        As, Bs = np.vectorize(direct_collocation.linearize, signature='(n),(m)->(n,n),(n,m)')(states, controls)
        A_ds, B_ds = np.vectorize(LQR.discretize, signature='(),(n,n),(n,m),(a,b),(c,d)->(n,n),(n,m)')(self.dt, As, Bs, C, D)
        _, K_ds = LQR.calculate_finite_K_ds(A_ds, B_ds, Q, R)

        self._trajectory_states = states
        self._trajectory_controls = controls
        self._trajectory_K_ds = K_ds
        self._trajectory_max = states.shape[0]
        self._trajectory_count = 0
        self._target_state = states[-1]
        self._target_K = K_ds[-1]
        self._control_enabled = True
        self._is_in_trajectory = False
        self._trajectory_calculating = False

    def calculate_control(self, state: np.ndarray):
        control = np.zeros(self._system.num_controls)
        if self._control_enabled:
            if self._trajectory_count < self._trajectory_max:
                error = self._system.calculate_error(state, self._trajectory_states[self._trajectory_count])
                u_ff = self._trajectory_controls[self._trajectory_count]
                u_fb = LQR.feedback(self._trajectory_K_ds[self._trajectory_count], error)
                control = u_ff + u_fb
                self._trajectory_count += 1
                self._is_in_trajectory = True
            else:
                error = self._system.calculate_error(state, self._target_state)
                control = LQR.feedback(self._target_K, error)
                self._is_in_trajectory = False
        return control

    def run(self):
        if self._is_running:
            return
        self._is_running = True
        self._thread.start()

    def _run_loop(self):
        while self._is_running:
            command = input()
            if command == 'q':
                self.stop()
            elif command == 'c':
                self.disable_control()
            elif command == 't':
                try:
                    pos = float(input('Enter target position (-0.5 to 0.5): '))
                    if pos > 0.5:
                        pos = 0.5
                    elif pos < -0.5:
                        pos = -0.5
                    pole_pos = [bool(int(input(f"Enter pole {i} position ('0' for down or '1' for up): "))) for i in range(self._system.num_poles)]
                    trajectory_process = Thread(target=self.create_trajectory, args=(pos, pole_pos))
                    trajectory_process.start()
                except ValueError:
                    print('Value error: Failed to parse value to number')

            if not self._simulator.running:
                self.stop()
                break

    def stop(self):
        self._is_running = False
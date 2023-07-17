import numpy as np
from numpy import radians
from threading import Thread
from .cartpolesimulator import CartPoleSimulator
from .direct_collocation import CartPoleStepperMotorDirectCollocation
from .regulators import LQR
from enum import Enum

class ControlType(Enum):
    LQR = 0
    TRAJECTORY = 1
    COS = 2

class CartPoleController:
    def __init__(self, simulator: CartPoleSimulator, dt: float):
        self._simulator = simulator
        self._system = simulator.system
        self._simulator.get_control = self.calculate_control
        self._thread = Thread(target=self._run_loop)
        self._is_running = False
        self._dt = dt

        self._control_enabled = False
        self._control_type = ControlType.LQR

        self._control_calculating = False
        self._trajectory_count = 0
        self._trajectory_max = 0
        self._trajectory_states = np.array([])
        self._trajectory_controls = np.array([])
        self._trajectory_K_ds = np.array([])
        self._target_K = np.array([])
        self._last_pole_pos = [True for _ in range(self._system.num_poles)]

        pole_pos = self._last_pole_pos
        pole_states = np.array([[float(0 if pos else radians(180)), 0.0] for pos in pole_pos]).flatten()
        self._target_state = np.array([0, 0] + pole_states.tolist())

        self._cos_count = 0

    @property
    def dt(self) -> float:
        return self._dt
    
    def disable_control(self):
        if not self._is_running:
            return
        self._control_enabled = False
        self._control_type = ControlType.LQR

    def create_trajectory(self, pos: float, pole_pos: list[bool], end_time: float):
        if self._control_calculating or not self._is_running or self._control_type == ControlType.TRAJECTORY or not self._simulator.running:
            return
        
        self._control_calculating = True
        dt_collocation = 0.03
        N_collocation = int(end_time/dt_collocation)+1

        self._last_pole_pos = pole_pos
        pole_states = np.array([[float(0 if pos else radians(180)), 0.0] for pos in pole_pos]).flatten()
        target_state = np.array([pos, 0] + pole_states.tolist())

        N = int(end_time/self.dt)

        system = self._system
        direct_collocation = CartPoleStepperMotorDirectCollocation(N, N_collocation, system, 0.0001)

        x0 = self._target_state
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
        self._control_type = ControlType.TRAJECTORY
        self._control_calculating = False

    def create_reference(self, pos: float):
        if self._control_calculating or not self._is_running or self._control_type == ControlType.TRAJECTORY or not self._simulator.running:
            return
        
        self._control_calculating = True

        pole_pos = self._last_pole_pos
        pole_states = np.array([[float(0 if pos else radians(180)), 0.0] for pos in pole_pos]).flatten()
        target_state = np.array([pos, 0] + pole_states.tolist())
        u0 = np.zeros(self._system.num_controls)

        C = np.diag([1, 1]+[1, 1]*self._system.num_poles)
        D = np.zeros((1, 1))
        Q = np.diag([100, 10]+[100, 10]*self._system.num_poles)
        R = np.diag([1])

        A, B = self._system.linearize(target_state, u0)
        A_d, B_d = LQR.discretize(self.dt, A, B, C, D)
        _, K_d = LQR.calculate_K_d(A_d, B_d, Q, R)

        self._target_state = target_state
        self._control_enabled = True
        self._control_type = ControlType.LQR
        self._target_K = K_d
        self._control_calculating = False

    def create_cos(self, amplitude: float, period: float):
        if self._control_calculating or not self._is_running or self._control_type == ControlType.TRAJECTORY or not self._simulator.running:
            return
        
        self._cos_count = 0
        self._cos_amplitude = amplitude
        self._cos_period = period
        self._control_type = ControlType.COS
        self._control_enabled = True

    def calculate_control(self, state: np.ndarray):
        control = np.zeros(self._system.num_controls)
        if self._control_enabled:
            if self._control_type == ControlType.TRAJECTORY:
                error = self._system.calculate_error(state, self._trajectory_states[self._trajectory_count])
                u_ff = self._trajectory_controls[self._trajectory_count]
                u_fb = LQR.feedback(self._trajectory_K_ds[self._trajectory_count], error)
                control = u_ff + u_fb
                self._trajectory_count += 1

                if self._trajectory_count >= self._trajectory_max:
                    self._control_type = ControlType.LQR
            elif self._control_type == ControlType.LQR:
                error = self._system.calculate_error(state, self._target_state)
                control = LQR.feedback(self._target_K, error)
                self._is_in_trajectory = False
            elif self._control_type == ControlType.COS:
                control = np.array([self._cos_amplitude * np.cos(2*np.pi*self._cos_count*self.dt/self._cos_period)])
                self._cos_count += 1
        return control
    
    def run(self):
        if self._is_running:
            return
        self._is_running = True
        self._thread.start()

    def _run_loop(self):
        while self._is_running:
            command = input()
            if command == 'c':
                self.disable_control()
            elif command == "r":
                try:
                    pos = float(input('Enter target position (-0.4 to 0.4): '))
                    if pos > 0.4:
                        pos = 0.4
                    elif pos < -0.4:
                        pos = -0.4
                    trajectory_process = Thread(target=self.create_reference, args=(pos,))
                    trajectory_process.start()
                except ValueError:
                    print('Value error: Failed to parse value to number')
            elif command == 't':
                try:
                    pos = float(input('Enter target position (-0.4 to 0.4): '))
                    if pos > 0.4:
                        pos = 0.4
                    elif pos < -0.4:
                        pos = -0.4
                    pole_pos = [bool(int(input(f"Enter pole {i} position ('0' for down or '1' for up): "))) for i in range(self._system.num_poles)]
                    end_time = float(input('Enter end time: '))
                    trajectory_process = Thread(target=self.create_trajectory, args=(pos, pole_pos, end_time))
                    trajectory_process.start()
                except ValueError:
                    print('Value error: Failed to parse value to number')
            elif command == "f":
                try:
                    amplitude = float(input('Enter cos amplitude: '))
                    period = float(input('Enter cos period: '))
                    self.create_cos(amplitude, period)
                except ValueError:
                    print('Value error: Failed to parse value to number')
            elif command == "q":
                self._simulator.stop()

                if input('Save data to file? (y/n):') == 'y':
                    name = input('Enter file name: ')
                    if name == '':
                        name = 'data'
                    df = self._simulator.export()
                    df.to_csv(f'{name}.csv', index=False)
                self.stop()

            if not self._simulator.running:
                self.stop()
                break

    def stop(self):
        self._is_running = False
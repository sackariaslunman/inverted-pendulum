import numpy as np
from numpy import radians
import pandas as pd
from enum import Enum
from threading import Thread
from multiprocessing import Process, Queue
from .cartpolesimulator import CartPoleSimulator
from .direct_collocation import CartPoleStepperMotorDirectCollocation
from .regulators import LQR
from .cartpolesystem import CartPoleSystem

def make_solver(N: int, N_collocation: int, system: CartPoleSystem, end_time: float, x0: np.ndarray, target_state: np.ndarray, output: Queue):
    direct_collocation = CartPoleStepperMotorDirectCollocation(N, N_collocation, system, 0.0001)
    states, controls = direct_collocation.make_solver(end_time, x0, target_state)
    As, Bs = np.vectorize(direct_collocation.linearize, signature='(n),(m)->(n,n),(n,m)')(states, controls)
    output.put([states, controls, As, Bs])

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
        self._dt = dt
        self._reset()

    def _reset(self):
        self._is_running = False
        self._control_enabled = False
        self._control_type = ControlType.LQR

        self._control_calculating = False
        self._trajectory_count = 0
        self._trajectory_max = 0
        self._trajectory_states = np.array([])
        self._trajectory_controls = np.array([])
        self._trajectory_K_ds = np.array([])
        self._target_K = np.array([])
        self._last_pole_pos = [False for _ in range(self._system.num_poles)]

        pole_pos = self._last_pole_pos
        pole_states = np.array([[float(0 if pos else radians(180)), 0.0] for pos in pole_pos]).flatten()
        self._target_state = np.array([0, 0] + pole_states.tolist())

        self._cos_count = 0

        self.C = np.diag([1, 1]+[1, 1]*self._system.num_poles)
        self.D = np.zeros([1, 1])
        self.Q = np.diag([100, 1]+[100, 1]*self._system.num_poles)
        self.R = np.diag([1])

        self._desired_controls = []
        self._desired_states = []
        self._errors = []
    
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
        x0 = self._target_state

        output = Queue()
        process = Process(target=make_solver, args=(N, N_collocation, system, end_time, x0, target_state, output))
        process.start()

        while output.empty():
            pass
        result = output.get()
        states, controls, As, Bs = result

        A_ds, B_ds = np.vectorize(LQR.discretize, signature='(),(n,n),(n,m),(a,b),(c,d)->(n,n),(n,m)')(self.dt, As, Bs, self.C, self.D)
        _, K_ds = LQR.calculate_finite_K_ds(A_ds, B_ds, self.Q, self.R)

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

        A, B = self._system.linearize(target_state, u0)
        A_d, B_d = LQR.discretize(self.dt, A, B, self.C, self.D)
        _, K_d = LQR.calculate_K_d(A_d, B_d, self.Q, self.R)

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
        desired_control = np.zeros(self._system.num_controls)
        desired_state = np.zeros(self._system.num_states)
        error = np.zeros(self._system.num_states)

        if self._control_enabled:
            if self._control_type == ControlType.TRAJECTORY:
                desired_state = self._trajectory_states[self._trajectory_count]
                error = self._system.calculate_error(state, desired_state)
                u_ff = self._trajectory_controls[self._trajectory_count]
                desired_control = u_ff
                u_fb = LQR.feedback(self._trajectory_K_ds[self._trajectory_count], error)
                control = u_ff
                self._trajectory_count += 1

                if self._trajectory_count >= self._trajectory_max:
                    self._control_type = ControlType.LQR
            elif self._control_type == ControlType.LQR:
                desired_state = self._target_state
                error = self._system.calculate_error(state, desired_state)
                control = LQR.feedback(self._target_K, error)
                self._is_in_trajectory = False
            elif self._control_type == ControlType.COS:
                control = np.array([self._cos_amplitude * np.cos(2*np.pi*self._cos_count*self.dt/self._cos_period)])
                self._cos_count += 1

        self._desired_controls.append(desired_control)
        self._desired_states.append(desired_state)
        self._errors.append(error)
        return control
    
    def _adjust_gains(self):
        pos_gain = input(f'Enter position gain (enter to keep {self.Q[0,0]}): ')
        if pos_gain == '':
            pos_gain = self.Q[0,0]
        else:
            pos_gain = float(pos_gain)
        vel_gain = input(f'Enter velocity gain (enter to keep {self.Q[1,1]}): ')
        if vel_gain == '':
            vel_gain = self.Q[1,1]
        else:
            vel_gain = float(vel_gain)
        self.Q[0, 0] = pos_gain
        self.Q[1, 1] = vel_gain
        for i in range(self._system.num_poles):
            theta_gain = input(f'Enter pole {i+1} angle gain (enter to keep {self.Q[2*i+2,2*i+2]}): ')
            if theta_gain == '':
                theta_gain = self.Q[2*i+2,2*i+2]
            else:
                theta_gain = float(theta_gain)
            d_theta_gain = input(f'Enter pole {i+1} angle velocity gain (enter to keep {self.Q[2*i+3,2*i+3]}): ')
            if d_theta_gain == '':
                d_theta_gain = self.Q[2*i+3,2*i+3]
            else:
                d_theta_gain = float(d_theta_gain)
            self.Q[2*i+2, 2*i+2] = theta_gain
            self.Q[2*i+3, 2*i+3] = d_theta_gain
    
    def run(self):
        if self._is_running:
            return
        self._reset()
        self._is_running = True
        self._thread.start()

    def _run_loop(self):
        while self._is_running:
            command = input()
            if command == 'c':
                self.disable_control()
            elif command == "r":
                try:
                    min_pos = self._system.state_lower_bound[0]+self._system.state_margin[0]+0.05
                    max_pos = self._system.state_upper_bound[0]-self._system.state_margin[0]-0.05

                    pos = float(input(f'Enter target position ({min_pos} to {max_pos}): '))
                    if pos > max_pos:
                        pos = max_pos
                    elif pos < min_pos:
                        pos = min_pos
                    trajectory_process = Thread(target=self.create_reference, args=(pos,))
                    trajectory_process.start()
                except ValueError:
                    print('Value error: Failed to parse value to number')
            elif command == 't':
                try:
                    min_pos = self._system.state_lower_bound[0]+self._system.state_margin[0]+0.05
                    max_pos = self._system.state_upper_bound[0]-self._system.state_margin[0]-0.05
                    pos = float(input(f'Enter target position ({min_pos} to {max_pos}): '))
                    if pos > max_pos:
                        pos = max_pos
                    elif pos < min_pos:
                        pos = min_pos

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
                    self.export(name, True)
                self.stop()
            
            elif command == "j":
                try:
                    self._adjust_gains()
                except ValueError:
                    print('Value error: Failed to parse value to number')

            if not self._simulator.running:
                self.stop()
                break
        
    def export(self, name: str, save_to_file: bool) -> pd.DataFrame:
        df_env = self._simulator.export()
        data = {}
        desired_states = np.array(self._desired_states)
        desired_controls = np.array(self._desired_controls)
        errors = np.array(self._errors)

        data["desired_s"] = desired_states[:,0]
        data["error_s"] = errors[:,0]

        data["desired_d_s"] = desired_states[:,1]
        data["error_d_s"] = errors[:,1]

        for i in range(self._system.num_poles):
            data[f"desired_theta_{i+1}"] = desired_states[:,2*i+2]
            data[f"error_theta_{i+1}"] = errors[:,2*i+2]

            data[f"desired_d_theta_{i+1}"] = desired_states[:,2*i+3]
            data[f"error_d_theta_{i+1}"] = errors[:,2*i+3]
        data["desired_u"] = desired_controls[:,0]

        df_desired = pd.DataFrame(data)
        df = pd.concat([df_env, df_desired], axis=1)

        if save_to_file:
            df.to_csv(f'{name}.csv', index=False)
        return df

    def stop(self):
        self._is_running = False
import time
import numpy as np
from numpy import radians
from abc import ABC, abstractmethod
from threading import Thread
from .cartpoleenv import CartPoleEnv
from .cartpolesystem import CartPoleStepperMotorSystem
from .regulators import LQR
from .numerical import rk4_step
import random

class CartPoleSimulator(ABC):
    @property
    @abstractmethod
    def system(self) -> CartPoleStepperMotorSystem:
        ...

    @property
    @abstractmethod
    def dt(self) -> float:
        ...

    @property
    @abstractmethod
    def control_enabled(self) -> bool:
        ...

    @abstractmethod
    def disable_control(self) -> None:
        ...

    @property
    @abstractmethod
    def running(self) -> bool:
        ...

    @abstractmethod
    def send_trajectory(self, states: np.ndarray, controls: np.ndarray, K_ds: np.ndarray) -> None:
        ...

    @abstractmethod
    def send_reference(self, state: np.ndarray) -> None:
        ...

    @property
    @abstractmethod
    def state(self) -> np.ndarray:
        ...

    @property
    @abstractmethod
    def is_in_trajectory(self) -> bool:
        ...

    @property
    @abstractmethod
    def render_enabled(self) -> bool:
        ...

    @render_enabled.setter
    @abstractmethod
    def render_enabled(self, value: bool) -> None:
        ...

class CartPoleEnvSimulator(CartPoleSimulator):
    def __init__(self, dt: float, system: CartPoleStepperMotorSystem, max_time: float = 60*10):
        self._system = system
        env = CartPoleEnv(system, dt, rk4_step)
        self._env = env
        self._dt = dt
        self._system = env.system
        self._max_time = max_time
        self._N_max = int(max_time/env.dt_sim)
        self._control_enabled = False
        self._running = False
        self._is_in_trajectory = False
        self._render_enabled = True

        self._trajectory_calculating = False
        self._trajectory_count = 0
        self._trajectory_max = 0
        self._trajectory_states = np.array([])
        self._trajectory_controls = np.array([])
        self._trajectory_K_ds = np.array([])
        self._target_state = np.array([])
        self._target_K = np.array([])

        self.run_thread = Thread(target=self.run_loop)

    @property
    def dt(self):
        return self._dt

    @property
    def control_enabled(self):
        return self._control_enabled
    
    @property
    def running(self):
        return self._running
    
    @property
    def is_in_trajectory(self):
        return self._is_in_trajectory

    @property
    def state(self):
        return self._env.get_state()
    
    @property
    def system(self) -> CartPoleStepperMotorSystem:
        return self._system #type: ignore

    @property
    def render_enabled(self):
        return self._render_enabled
    
    def disable_control(self):
        self._control_enabled = False
        self._is_in_trajectory = False

    def send_trajectory(self, states: np.ndarray, controls: np.ndarray, K_ds: np.ndarray):
        self._trajectory_states = states
        self._trajectory_controls = controls
        self._trajectory_K_ds = K_ds
        self._trajectory_max = states.shape[0]
        self._trajectory_count = 0
        self._target_state = states[-1]
        self._target_K = K_ds[-1]
        self._control_enabled = True

    def send_reference(self, state: np.ndarray) -> None:
        self._target_state = state
        self._control_enabled = True

    def run(self):
        self._running = True
        self.run_thread.start()

    def stop(self):
        self._running = False
        self.run_thread.join()

    def run_loop(self):
        initial_state = np.array([0,0] + [radians(180), 0] * self._system.num_poles)
        state, _ = self._env.reset(initial_state)
        if self._render_enabled:
            self._env.render()

        self._running = True
        self.step = 0

        while self._running: 
            state = self._env.get_state()
            self.step += 1
            control = np.zeros(self._system.num_controls)
            if self.control_enabled:
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
                    
            self._env.step(control)
            if self._render_enabled:
                self._env.render(state)

            if self.step >= self._N_max:
                self._running = False
            time.sleep(self._dt)

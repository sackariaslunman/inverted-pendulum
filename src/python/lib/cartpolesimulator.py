from time import perf_counter
import numpy as np
from numpy import radians
from abc import ABC, abstractmethod
from typing import Callable
from serial import Serial
from threading import Thread
from multiprocessing import Process
from .cartpoleenv import CartPoleEnv
from .cartpolesystem import CartPoleSystem
from .numerical import rk4_step
import pandas as pd

class CartPoleSimulator(ABC):
    def __init__(self, dt: float, system: CartPoleSystem, get_control: Callable[[np.ndarray], np.ndarray] | None = None):
        self._system = system
        self._dt = dt
        self.get_control = get_control

    @property
    @abstractmethod
    def system(self) -> CartPoleSystem:
        ...

    @property
    @abstractmethod
    def dt(self) -> float:
        ...

    @property
    @abstractmethod
    def running(self) -> bool:
        ...

    @property
    @abstractmethod
    def state(self) -> np.ndarray:
        ...

    @property
    @abstractmethod
    def render_enabled(self) -> bool:
        ...

    @render_enabled.setter
    @abstractmethod
    def render_enabled(self, value: bool) -> None:
        ...

    @abstractmethod
    def export(self) -> pd.DataFrame:
        ...
    
    @abstractmethod
    def stop(self) -> None:
        ...

class CartPoleSerialSimulator(CartPoleSimulator):
    def __init__(self, dt: float, system: CartPoleSystem, get_control: Callable[[np.ndarray],np.ndarray] | None = None):
        super().__init__(dt, system, get_control)
        env = CartPoleEnv(system, dt, rk4_step)
        self._env = env
        self._running = False
        self._render_enabled = True
        self._run_process = Thread(target=self.run_loop)
        self._state = np.zeros(system.num_states, dtype=np.float64)
        self._control = np.zeros(system.num_controls, dtype=np.float64)

    @property
    def dt(self):
        return self._dt
    
    @property
    def running(self):
        return self._running

    @property
    def state(self) -> np.ndarray:
        return self._state
    
    @property
    def system(self) -> CartPoleSystem:
        return self._system #type: ignore

    @property
    def render_enabled(self):
        return self._render_enabled
    
    def run(self, port: str, baudrate: int, timeout: float = 1):
        self._running = True
        self._port = port
        self._baudrate = baudrate
        self._timeout = timeout
        self._run_process.start()

    def stop(self):
        self._running = False
        self._run_process.join()

    def run_loop(self):
        if self.get_control is None:
            raise ValueError("get_control is None")

        last_update = perf_counter()
        counter = 0
        with Serial(self._port, self._baudrate, timeout=self._timeout) as ser:
            while ser.is_open and self._running:
                if ser.in_waiting == 0:
                    continue

                ser.read_until(b"xst")
                state_bytes = ser.read(self._state.nbytes)
                state = np.frombuffer(state_bytes, dtype=self._state.dtype)
                # state = list(state).copy()

                # rolling_n = 5
                # if counter > rolling_n+1:
                #     for i in range(self._system.num_poles):
                #         # low pass filter d_theta
                #         prev_d_thetas = [state[3+2*i]]
                #         prev_d_thetas.extend(self._env.get_state(-j)[3+2*i] for j in range(1, rolling_n+1))
                #         rolling_d_theta = np.mean(prev_d_thetas, axis=0)
                #         state[3+2*i] = rolling_d_theta

                self._state = np.array(state, dtype=self._state.dtype)

                self._control = self.get_control(self._state)

                ser.write(self._control.tobytes())
                dt = perf_counter() - last_update
                self._env.step(self._control, self._state, dt)
                last_update = perf_counter()

                counter += 1

                if self._render_enabled:
                    self._env.render()
    
    def export(self):
        return self._env.export()

class CartPoleEnvSimulator(CartPoleSimulator):
    def __init__(self, dt: float, system: CartPoleSystem, get_control: Callable[[np.ndarray],np.ndarray] | None = None, max_time: float = 60*10):
        super().__init__(dt, system, get_control)
        env = CartPoleEnv(system, dt, rk4_step)
        self._env = env
        self._system = env.system
        self._max_time = max_time
        self._N_max = int(max_time/env.dt_sim)
        self._running = False
        self._render_enabled = True
        self._run_process = Thread(target=self.run_loop, daemon=True)

    @property
    def dt(self):
        return self._dt
    
    @property
    def running(self):
        return self._running

    @property
    def state(self) -> np.ndarray:
        return self._env.get_state()
    
    @property
    def system(self) -> CartPoleSystem:
        return self._system #type: ignore

    @property
    def render_enabled(self):
        return self._render_enabled
    
    def run(self):
        self._running = True
        self._run_process.start()

    def stop(self):
        self._running = False
        self._run_process.join()

    def run_loop(self):
        initial_state = np.array([0,0] + [radians(180), 0] * self._system.num_poles)
        state, _ = self._env.reset(initial_state)
        if self._render_enabled:
            self._env.render()

        self._running = True
        self.step = 0
        last_update = perf_counter()

        while self._running: 
            state = self._env.get_state()
            self.step += 1
            if self.get_control is None:
                raise ValueError("No control function provided")
            control = self.get_control(state)

            self._env.step(control)
            if self._render_enabled:
                self._env.render(state)

            if self.step >= self._N_max:
                self._running = False
            while perf_counter() - last_update < self._dt:
                pass

            last_update = (perf_counter() // self._dt) * self._dt

    def export(self):
        return self._env.export()
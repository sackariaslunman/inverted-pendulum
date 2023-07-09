import asyncio
import time
import numpy as np
from numpy import radians

from .direct_collocation import CartPoleStepperMotorDirectCollocation
from .cartpoleenv import CartPoleEnv
from .cartpolesystem import CartPoleStepperMotorSystem
from .controllers import LQR

class CartPoleSimulator:
    def __init__(self, env: CartPoleEnv, max_time: float = 60*10, dt_collocation: float = 0.05):
        self._env = env
        self._dt = env.dt_sim
        self._system = env.system
        self._max_time = max_time
        self._N_max = int(max_time/env.dt_sim)
        self._enable_control = False
        self._dt_collocation = dt_collocation
        self._running = False

        self._trajectory_calculating = False
        self._trajectory_count = 0
        self._trajectory_max = 0
        self._trajectory_states = np.array([])
        self._trajectory_controls = np.array([])
        self._trajectory_K_ds = np.array([])
        self._target_state = np.array([])
        self._target_K = np.array([])

    @property
    def dt(self):
        return self._dt
    
    @property
    def max_time(self):
        return self._max_time
    
    @property
    def N_max(self):
        return self._N_max

    @property
    def enable_control(self):
        return self._enable_control
    
    @property
    def dt_collocation(self):
        return self._dt_collocation
        
    async def trajectory(self, initial_state: np.ndarray, target_state: np.ndarray, end_time: float = 10):
        if not self.enable_control or self._trajectory_calculating:
            return

        N_collocation = int(end_time/self.dt_collocation)+1
        N = int(end_time/self.dt)

        if self._system is not CartPoleStepperMotorSystem:
            raise NotImplementedError("Only CartPoleStepperMotorSystem is supported for now.")
        
        direct_collocation = CartPoleStepperMotorDirectCollocation(N, N_collocation, end_time, self._system, 0.001) # type: ignore
        states, controls = direct_collocation.make_controller(initial_state, target_state)

        C = np.diag([1, 1]+[1, 1]*self._system.num_poles)
        D = np.zeros((1, 1))
        Q = np.diag([100, 10]+[100, 10]*self._system.num_poles)
        R = np.diag([1])

        As, Bs = np.vectorize(direct_collocation.linearize, signature='(n),(m)->(n,n),(n,m)')(states, controls)
        A_ds, B_ds = np.vectorize(LQR.discretize, signature='(),(n,n),(n,m),(a,b),(c,d)->(n,n),(n,m)')(self.dt, As, Bs, C, D)
        _, K_ds = LQR.calculate_finite_K_ds(A_ds, B_ds, Q, R)

        self._trajectory_states = states
        self._trajectory_controls = controls
        self._trajectory_K_ds = K_ds
        self._trajectory_count = 0
        self._trajectory_max = len(K_ds)

        self._target_state = target_state
        self._target_K = K_ds[-1]

        self._trajectory_calculating = False

    def run(self, initial_state: np.ndarray | None = None):
        if initial_state is None:
            initial_state = np.array([0,0] + [radians(180), 0] * self._system.num_poles)
        state, _ = self._env.reset(initial_state)
        self._env.render()

        self._running = True
        self.paused = False
        self.step = 0

        while self._running: 
            state = self._env.get_state()
            self.step += 1
            control = np.zeros(self._system.num_controls)
            if self.enable_control:
                if self._trajectory_count < self._trajectory_max:
                    error = self._system.calculate_error(state, self._trajectory_states[self._trajectory_count])
                    u_ff = self._trajectory_controls[self._trajectory_count]
                    u_fb = LQR.feedback(self._trajectory_K_ds[self._trajectory_count], error)
                    control = u_ff + u_fb
                    self._trajectory_count += 1
                else:
                    error = self._system.calculate_error(state, self._target_state)
                    control = LQR.feedback(self._target_K, error)

            self._env.step(control)
            self._env.render(state)

            if self.step >= self._N_max:
                self._running = False
            time.sleep(self.dt)

    def on_run(self, event):
        self.run()

    def on_stop(self):
        self._running = False
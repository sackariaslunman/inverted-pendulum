import gymnasium as gym
import jax
import jax.random as random
import jax.numpy as jnp
import numpy as np
from cartpolesystem import CartPoleSystem, generate_random_cartpole_system
from typing import Literal

class CartPoleEnv(gym.Env):
    def __init__(self, n_poles: int, max_steps: int, dt: float, render_mode: Literal["rgb_array", "human"] = "human"):
        assert n_poles > 0
        self._n_poles = n_poles
        assert max_steps > 0
        self._max_steps = max_steps
        assert dt > 0
        self._dt = dt
        self._render_mode = render_mode

        # 2 cart states (position and velocity)
        # 2 states for each pole (angle and angular velocity)
        self._n_states = 2 + 2 * n_poles
        # 1 observation state (applied force on cart (tau)) 
        self._n_observations = 1

        # Note that the observation space is the state space

        min_float32 = jnp.finfo(jnp.float32).min
        max_float32 = jnp.finfo(jnp.float32).max
        min_action = min_float32
        max_action = max_float32
        min_observation = np.full((self._n_states,), min_float32)
        max_observation = np.full((self._n_states,), max_float32)
        self._action_space = gym.spaces.Box(low=min_action, high=max_action, shape=(1,))
        self._observation_space = gym.spaces.Box(low=min_observation, high=max_observation, shape=(self._n_states,))

    @property
    def action_space(self) -> gym.spaces.Box:
        return self._action_space
    
    @property
    def observation_space(self) -> gym.spaces.Box:
        return self._observation_space

    def reset(self, key, init_state: jnp.ndarray | None = None) -> tuple[jnp.ndarray, dict[str, jnp.ndarray]]:
        # Some ranges for the random system
        cart_mass_range = (0.5, 1.5)
        gravity_range = (9.5, 10.5)
        masses_range = (0.1, 0.2)
        lengths_range = (0.5, 1.0)
        frictions_range = (0.0, 0.1)
        inertias_range = (0.0, 0.1)
        self._system: CartPoleSystem = generate_random_cartpole_system(
            key, self._n_poles, cart_mass_range, gravity_range, masses_range, lengths_range, frictions_range, inertias_range
        )
        if init_state is None:
            init_state = jnp.zeros((self._n_states,))
        self._state = init_state
        action = jnp.zeros((1,))
        self._observation = self._system.observe(self._state, action)

        info = {"observation_state": self._observation}
        return self._state, info
    
    def step(self, action: jnp.ndarray) -> tuple[jnp.ndarray, float, bool, dict[str, jnp.ndarray]]:
        dstate = self._system(self._state, action)
        self._state += dstate * self._dt
        self._observation = self._system.observe(self._state, action)
        done = False
        truncated = False
        reward = 0
        info = {"observation_state"}
        return self._state, reward, done, truncated, info
    
    def _setup_render(self) -> None:
        if self._render_mode == "rgb_array":
            return
        elif self._render_mode == "human":
            # import matplotlib
            # matplotlib.use('Qt5Agg')
            # from PyQt6 import QtCore, QtWidgets
            pass
    
    def render(self) -> None:
        pass
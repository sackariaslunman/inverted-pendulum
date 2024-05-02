from functools import partial
import gymnasium as gym
import jax.numpy as jnp
import numpy as np
import jax
from cartpolesystem import CartPoleSystem, generate_random_cartpole_system
from typing import Literal
from time import perf_counter

class CartPoleEnv(gym.Env):
    def __init__(self, n_poles: int, max_steps: int, dt: float, use_noise: bool = True, render_mode: None | Literal["human"] = "human"):
        assert n_poles > 0
        self._n_poles = n_poles
        assert max_steps > 0
        self._max_steps = max_steps
        assert dt > 0
        self._dt = dt
        self._render_mode = render_mode
        self._use_noise = use_noise

        ### States, actions and observations

        # 2 cart states (position and velocity)
        # 2 states for each pole (angle and angular velocity)
        self._n_states = 2 + 2 * n_poles
        # 1 observation state (applied force on cart (tau)) 
        self._n_observations = 1
        self._n_actions = 1

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
        cart_mass_range = (0.1, 1)
        gravity_range = (9, 10.62) # mean is 9.81
        gravity_orientation_range = (-0.1, 0.1) # mean is 0
        masses_range = (0.1, 0.2)
        lengths_range = (0.1, 0.3)
        frictions_range = (0.0, 0.1)
        inertias_range = (0.0, 0.1)

        self._rail_length = 1.0

        # split key
        key, new_key = jax.random.split(key, 2)
        self._key = new_key

        self._system: CartPoleSystem = generate_random_cartpole_system(
            key, self._n_poles, cart_mass_range, gravity_range, gravity_orientation_range, masses_range, lengths_range, frictions_range, inertias_range
        )
        if init_state is None:
            init_state = jnp.concatenate([jnp.zeros((2,)), jnp.tile(jnp.array([jnp.pi, 0]), (self._n_poles,))])
        self._step = 1
        self._state = init_state
        action = jnp.zeros((self._n_actions,))

        self._observation = self._system.observe(self._state, action)
        info = {"observation_state": self._observation, "step": 0}

        ### Reset goal
        self._goal_state = jnp.zeros((self._n_states,))
        s_goal_dist = 0.1 # rewarded if 10 cm from the goal
        v_goal_dist = 0.1 # rewarded if 0.01 [speed] from the goal
        angle_goal_dist = 10*jnp.pi/180 # rewarded if 10 degrees from the goal
        self._goal_distance = jnp.concatenate([jnp.array([s_goal_dist, v_goal_dist]), jnp.tile(jnp.array([angle_goal_dist, v_goal_dist]), self._n_poles)])
        self._action_cost = jnp.ones((self._n_actions,))

        ### Reset noise variance and bias
        max_variance = jnp.sqrt(0.001)
        self._noise_variances = jax.random.uniform(key, (self._n_states,), minval=0.0, maxval=max_variance)
        self._noise_function = lambda key: jax.random.normal(key, shape=(self._n_states,)) * self._noise_variances
        
        return self._state, info
    
    def step(self, action: jnp.ndarray) -> tuple[jnp.ndarray, float, bool, dict[str, jnp.ndarray]]:
        dstate = self._system(self._state, action)
        key, self._key = jax.random.split(self._key)
        noise = self._noise_function(key) if self._use_noise else 0

        self._state += dstate * self._dt
        reward = self._reward(self._state, action).item()

        state = self._state + noise
        self._observation = self._system.observe(state, action)
        info = {"observation_state": self._observation}

        done = False
        if self._step >= self._max_steps:
            done = True
        info["step"] = self._step
        self._step += 1
        truncated = False
        return state, reward, done, truncated, info
    
    def _reward(self, state: jnp.ndarray, action: jnp.ndarray) -> jnp.ndarray:
        # the reward distribution will be expanded upon
        # to have more objectives
        # now, the pole is just incentivized to go from the bottom to upright
        reward = 0

        # If all distances from the goal are less than the maximum distance, reward the agent
        goal_dist_differences = self._goal_distance-jnp.abs(self._system.distance(state, self._goal_state))
        if jnp.all(goal_dist_differences >= 0):
            reward += 1

        action_cost = jnp.abs(action)*self._action_cost
        action_cost = -action_cost.sum()
        reward += action_cost

        return reward
    
    def _setup_render(self) -> None:
        if self._render_mode is None:
            return
        elif self._render_mode == "human":
            import pygame
            self._pygame = pygame
            pygame.init()
            self._screen_size = (1280, 720)
            self._screen = pygame.display.set_mode(self._screen_size)
            sum_lengths = self._system.lengths.sum().item()*2 # times two since we want equal distance above and below the rail
            height_ratio = self._screen_size[1] / sum_lengths
            width_ratio = self._screen_size[0] / self._rail_length
            self._meters_to_pixels_ratio = min([height_ratio, width_ratio])
            self._last_render_time = perf_counter()

    def close(self) -> None:
        if self._render_mode is None:
            return
        elif self._render_mode == "human":
            if hasattr(self, "_pygame") and self._pygame is not None:
                self._pygame.quit()

    def _meters_to_pixels(self, metres: float) -> int:
        return int(metres * self._meters_to_pixels_ratio)
    
    def _mass_to_radius(self, mass: float) -> int:
        return int(jnp.sqrt(mass)*20)
    
    def render(self) -> np.ndarray | None:
        if self._render_mode == "rgb_array":
            return None
        if self._render_mode == "human":

            if not hasattr(self, "_pygame"):
                self._setup_render()
            
            for event in self._pygame.event.get():
                if event.type == self._pygame.QUIT:
                    pass

            while perf_counter() - self._last_render_time < self._dt:
                pass
            self._last_render_time = self._last_render_time + self._dt
            
            # Draw background
            self._screen.fill("white")

            # draw circle at cart
            width = self._screen_size[0]
            height = self._screen_size[1]
            cart_x = self._meters_to_pixels(self._state[0]) + int(width / 2)
            cart_y = int(height / 2)
            cart_radius = self._mass_to_radius(self._system.cart_mass)
            self._pygame.draw.circle(self._screen, "red", (cart_x, cart_y), cart_radius)

            last_x = cart_x
            last_y = cart_y
            colors = ["green", "blue", "purple"]
            for k, (l, a, color) in enumerate(zip(self._system.lengths, self._system.centres_of_mass, colors)):
                angle = self._state[2 + 2*k]
                # +k for slight offset to tell poles apart
                pole_x = self._meters_to_pixels(l*jnp.sin(-angle)) + last_x 
                pole_y = -self._meters_to_pixels(l*jnp.cos(-angle)) + last_y
                mass_x = self._meters_to_pixels(a*jnp.sin(-angle)) + last_x
                mass_y = -self._meters_to_pixels(a*jnp.cos(-angle)) + last_y
                pole_radius = self._mass_to_radius(self._system.masses[k])
                self._pygame.draw.circle(self._screen, color, (mass_x+k, mass_y+k), pole_radius)
                self._pygame.draw.line(self._screen, color, (last_x+k, last_y+k), (pole_x+k, pole_y+k), 5)
                last_x = pole_x
                last_y = pole_y

            # Draw screen
            self._pygame.display.flip()
from __future__ import annotations
from typing import Callable
import numpy as np
from numpy import radians, sin, cos
from .colors import Colors
from random import uniform
from gym import spaces, Env
import pygame
from time import perf_counter
from .cartpolesystem import CartPoleSystem

class CartPoleEnv(Env):
  def __init__(
    self, 
    system: CartPoleSystem, 
    dt: float,
    N: int,
    integration_method: Callable[[float, Callable[[np.ndarray, np.ndarray], np.ndarray], np.ndarray, np.ndarray], tuple[np.ndarray, np.ndarray]]
  ):
    super(CartPoleEnv, self).__init__()
    self.system = system
    self.max_height = system.L
    self.dt = dt
    self.N = N
    self.max_time = dt*N
    self.g = system.g
    
    self.counter_down = 0
    self.counter_up = 0

    self.screen: pygame.surface.Surface | None = None
    self.font: pygame.font.Font
    self.i : int
    self.width = 1200
    self.height = 700
    self.size = self.width, self.height

    self.action_space = spaces.Box(
      low=system.control_lower_bound,
      high=system.control_upper_bound, 
      dtype=np.float64
    )
    
    self.observation_space = spaces.Box(
      low=system.state_lower_bound,
      high=system.state_upper_bound,
      dtype=np.float64
    )
    self.integration_method = integration_method
    self.reset()

  def get_state(self, index: int = -1) -> np.ndarray:
    if index < 0:
      index = self.iterations-1

    return self.states[index]

  def reset(self, initial_state: np.ndarray | None = None) -> tuple[np.ndarray, dict]:
    self.close()

    self.states = np.zeros((self.N, self.system.num_states))
    self.controls = np.zeros((self.N, self.system.num_controls))
    self.iterations = 0

    if initial_state is None:
      initial_state = np.zeros(self.system.num_states)
      initial_state[0] = uniform(self.system.state_lower_bound[0], self.system.state_upper_bound[0])*0.8
      self.counter_down = 0
      self.counter_up = 0

      for i in range(self.system.num_poles):
        initial_state[2+i*2] = radians(uniform(-50, 50))

    self.states[0] = initial_state
    self.iterations += 1

    return self.get_state(), {"Msg": "Reset env"}

  def step(self, action: np.ndarray) -> tuple[np.ndarray, float, bool, dict, bool]:
    reward = 0
    last_state = self.get_state()
    _, clipped_action = self.system.clip(last_state, action)
    
    raw_state, d_state = self.integration_method(self.dt, self.system.differentiate, last_state, clipped_action)
    state, _ = self.system.clip(raw_state, clipped_action)

    x = state[0]
    y = self.system.end_height(state)
    d_thetas = state[3::2]

    lost = bool(
      x >= self.system.state_upper_bound[0] or
      x <= self.system.state_lower_bound[0]
    )

    won = bool(
      self.counter_up > int(5/self.dt) # has been up for more than 5 seconds
    )

    done = bool(
      self.counter_down > int(5/self.dt) # has been down for more than 5 seconds
    )

    if y > self.max_height * 0.9 and all([abs(d_theta) < 1 for d_theta in d_thetas]):
      reward += 0
      self.counter_down = 0
      self.counter_up += 1
    else:
      reward -= 1
      self.counter_down += 1
      self.counter_up = 0
    
    if lost:
      reward -= 10
    elif won:
      reward += 10

    done = won or lost or done

    self.states[self.iterations] = state
    self.controls[self.iterations] = clipped_action
    self.iterations += 1
    
    return state, reward, done, {"won": won, "lost": lost}, False

  def si_to_pixels(self, x: float):
    return int(x * 500)

  def render(self, state: np.ndarray = np.array([]), optional_text: list[str] = []):
    if not self.screen:
      pygame.init()

      self.screen = pygame.display.set_mode(self.size)
      self.font = pygame.font.Font('freesansbold.ttf', 20)
      self.start_time = perf_counter()
      self.i = 0

    for event in pygame.event.get():
      if event.type == pygame.QUIT: 
        self.close()

    self.screen.fill(Colors.gray)

    if state.size == 0:
      state = self.get_state()
    x = state[0]
    d_x = state[1]
    thetas = state[2::2]
    d_thetas = state[3::2]

    x0 = self.si_to_pixels(state[0]) + self.width//2
    y0 = self.height//2
    pygame.draw.rect(self.screen, Colors.red, (x0, y0, 20, 10))

    max_x = self.width//2 + self.si_to_pixels(self.system.state_upper_bound[0])
    min_x = self.width//2 + self.si_to_pixels(self.system.state_lower_bound[0])
    pygame.draw.rect(self.screen, Colors.red, (min_x-10, y0, 10, 10))
    pygame.draw.rect(self.screen, Colors.red, (max_x+20, y0, 10, 10))

    motor_x0 = min_x-100
    theta_m = x/self.system.motor.r
    motor_sin = self.si_to_pixels(sin(-theta_m)*0.05)
    motor_cos = self.si_to_pixels(cos(-theta_m)*0.05)

    pygame.draw.polygon(self.screen, Colors.black, [
      (motor_x0+motor_sin, y0+motor_cos),
      (motor_x0+motor_cos, y0-motor_sin),
      (motor_x0-motor_sin, y0-motor_cos),
      (motor_x0-motor_cos, y0+motor_sin),
    ])

    x0 += 10
    pole_colors = [Colors.green, Colors.blue, Colors.purple, Colors.yellow]
    for pole, theta, color in zip(self.system.poles, thetas, pole_colors[:self.system.num_poles]): #type: ignore
      l = pole.l
      x1 = x0 + self.si_to_pixels(l * sin(theta))
      y1 = y0 + self.si_to_pixels(-l * cos(theta))
      pygame.draw.line(self.screen, color, (x0, y0), (x1, y1), 10)
      x0 = x1
      y0 = y1
  
    texts = [
      f"Time: {round(self.i*self.dt,2)} s",
      "",
      "Cart:",
      f"Position: {round(state[0],2)} m",
      f"Velocity: {round(state[1],2)} m/s",
      "",
      "Motor:",
      f"Angle: {round(theta_m,2)} rad",
    ]

    for k, (theta, d_theta) in enumerate(zip(thetas, d_thetas)):
      texts.extend([
        f"Pole {k+1} angle: {round(theta,2)} rad",
        f"Pole {k+1} angular velocity: {round(d_theta,2)} rad/s",
      ])

    texts.extend(optional_text)
    
    for k, text_k in enumerate(texts):
      text = self.font.render(text_k, True, Colors.black, Colors.gray)
      text_rect = text.get_rect()
      self.screen.blit(text,(0,20*k,text_rect.width,text_rect.height))

    pygame.display.flip()
    self.i += 1
    
  def close(self):
    pygame.quit()
    self.screen = None
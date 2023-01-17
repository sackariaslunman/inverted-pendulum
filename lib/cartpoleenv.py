from __future__ import annotations
import numpy as np
from numpy import radians, pi, sin, cos
from lib.colors import Colors
from random import uniform
import gym
from gym import spaces
import pygame
from time import perf_counter
from lib.cartpolesystem import CartPoleSystem

class CartPoleEnv(gym.Env):
  def __init__(
    self, 
    system: CartPoleSystem, 
    dt: float,
    g: float
  ):

    super(CartPoleEnv, self).__init__()
    self.system = system
    self.max_height = system.max_height()
    self.dt = dt
    self.g = g
    
    self.counter_down = 0
    self.counter_up = 0

    self.screen: pygame.surface.Surface | None = None
    self.font: pygame.font.Font
    self.i : int
    self.width = 1200
    self.height = 700
    self.size = self.width, self.height

    self.action_space = spaces.Box(
      low=system.min_Va,
      high=system.max_Va, 
      dtype=np.float32
    )

    num_of_poles = system.num_poles

    obs_lower_bound = np.array([ np.hstack([np.array([system.min_x, np.finfo(np.float32).min]), np.tile(np.array([0.0, np.finfo(np.float32).min]), num_of_poles)]) ]).T
    
    obs_upper_bound = np.array([ np.hstack([np.array([system.max_x, np.finfo(np.float32).max]), np.tile(np.array([2*pi, np.finfo(np.float32).max]), num_of_poles)]) ]).T
    
    self.observation_space = spaces.Box(
      low=obs_lower_bound,
      high=obs_upper_bound, 
      shape=obs_lower_bound.shape, 
      dtype=np.float32
    )

  def get_state(self):
    state = self.system.get_state()
    return state

  def reset(self, initial_state = None):
    self.close()

    if initial_state is None:
      initial_state = [uniform(self.system.min_x, self.system.max_x)*0.8, 0]
      self.counter_down = 0
      self.counter_up = 0

      for _ in range(self.system.num_poles):
        initial_state.extend([radians(uniform(-15, 15)), 0])

      initial_state = np.array([initial_state]).T

    self.system.reset(initial_state)
    return self.get_state(), {"Msg": "Reset env"}

  def step(self, action):
    reward = 0
    Va = action

    state, d_state = self.system.step(self.dt, Va, True)

    state = state.T[0]

    x = state[0]
    y = self.system.end_height()
    d_thetas = state[3::2]

    lost = bool(
      x >= self.system.max_x or
      x <= self.system.min_x
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
    
    return self.get_state(), reward, done, {"won": won, "lost": lost}, False

  def si_to_pixels(self, x: float):
    return int(x * 500)

  def render(self, optional_text: list[str] = []):
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

    state = self.system.get_state()
    state = state.T[0]
    x = state[0]

    x0 = self.si_to_pixels(state[0]) + self.width//2
    y0 = self.height//2
    pygame.draw.rect(self.screen, self.system.cart_color, (x0, y0, 20, 10))

    max_x = self.width//2 + self.si_to_pixels(self.system.max_x)
    min_x = self.width//2 + self.si_to_pixels(self.system.min_x)
    pygame.draw.rect(self.screen, self.system.cart_color, (min_x-10, y0, 10, 10))
    pygame.draw.rect(self.screen, self.system.cart_color, (max_x+20, y0, 10, 10))

    motor_x0 = min_x-100
    theta_m = x/self.system.r
    motor_sin = self.si_to_pixels(sin(-theta_m)*0.05)
    motor_cos = self.si_to_pixels(cos(-theta_m)*0.05)

    pygame.draw.polygon(self.screen, self.system.motor_color, [
      (motor_x0+motor_sin, y0+motor_cos),
      (motor_x0+motor_cos, y0-motor_sin),
      (motor_x0-motor_sin, y0-motor_cos),
      (motor_x0-motor_cos, y0+motor_sin),
    ])

    x0 += 10
    for k, ((_,_,l,_),color) in enumerate(zip(self.system.poles,self.system.pole_colors)):
      x1 = x0 + self.si_to_pixels(l * sin(state[2+k*2]))
      y1 = y0 + self.si_to_pixels(-l * cos(state[2+k*2]))
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

    for k in range(self.system.num_poles):
      theta, d_theta = state[2+k*2:2+k*2+2]
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
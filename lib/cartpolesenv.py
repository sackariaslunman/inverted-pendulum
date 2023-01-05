from math import radians, pi, sin, cos
from random import uniform
import gym
from gym import spaces
import numpy as np
from lib.cartpolesystem import CartPoleSystem
import pygame
from time import perf_counter
from lib.colors import Colors

class CartPolesEnv(gym.Env):
  def __init__(
    self, 
    system: CartPoleSystem, 
    dt: float,
    g: float, 
  ):

    super(CartPolesEnv, self).__init__()
    self.system = system
    self.max_height = system.max_height()
    self.dt = dt
    self.g = g

    self.counter_steps_up = 0
    self.counter_steps_not_up = 0

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
    obs_lower_bound = np.hstack([np.array([system.min_x, np.finfo(np.float32).min]), np.tile(np.array([0.0, np.finfo(np.float32).min]), num_of_poles)])
    obs_upper_bound = np.hstack([np.array([system.max_x, np.finfo(np.float32).max]), np.tile(np.array([2*pi, np.finfo(np.float32).max]), num_of_poles)])
    self.observation_space = spaces.Box(
      low=obs_lower_bound,
      high=obs_upper_bound, 
      shape=(2+2*num_of_poles,), 
      dtype=np.float32
    )

  def step(self, action: float):
    reward = 0
    Va = action
    state = self.system.step(self.dt, Va)
    x = state[0]
    y = self.system.end_height()

    terminated = bool(
      x >= self.system.max_x or
      x <= self.system.min_x
    )

    reward += 1/(1+abs(10*(self.max_height-y))**2) + 1/(1+abs(10*(x-0))**2)
    
    if terminated:
      reward -= 10
      self.counter_steps_not_up = 0
      self.counter_steps_up = 0
        
    return self.get_state(), reward, terminated, {}, False

  def reset(self):
    self.close()
    initial_state = [uniform(self.system.min_x, self.system.max_x)*0.8, 0, 0]

    for _ in range(self.system.num_poles):
      initial_state.extend([radians(uniform(-15, 15)), 0])

    self.system.reset(initial_state)
    return self.get_state(), {"Msg": "Reset env"}

  def si_to_pixels(self, x: float):
    return int(x * 500)

  def get_state(self):
    state = np.delete(self.system.get_state(), 3, axis=0)
    return state

  def render(self, optional_text: list[str] = []):
    if not self.screen:
      pygame.init()

      self.screen = pygame.display.set_mode(self.size)
      self.font = pygame.font.Font('freesansbold.ttf', 20)
      self.start_time = perf_counter()
      self.i = 0

    self.screen.fill(Colors.gray)

    state = self.system.get_state()

    x0 = self.si_to_pixels(state[0]) + self.width//2
    y0 = self.height//2
    pygame.draw.rect(self.screen, self.system.cart_color, (x0, y0, 20, 10))

    max_x = self.width//2 + self.si_to_pixels(self.system.max_x)
    min_x = self.width//2 + self.si_to_pixels(self.system.min_x)
    pygame.draw.rect(self.screen, self.system.cart_color, (min_x-10, y0, 10, 10))
    pygame.draw.rect(self.screen, self.system.cart_color, (max_x+20, y0, 10, 10))

    motor_x0 = min_x-100
    theta_m = state[2]
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
      x1 = x0 + self.si_to_pixels(l * sin(state[3+k*2]))
      y1 = y0 + self.si_to_pixels(-l * cos(state[3+k*2]))
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
from math import radians, pi, sin, cos
from random import uniform
import gym
from gym import spaces
import numpy as np
from cartpoles import Cart
import sys, pygame
from time import perf_counter
from colors import Colors

class CartPolesEnv(gym.Env):
  def __init__(
    self, 
    cart: Cart, 
    dt: float,
    g: float, 
  ):

    super(CartPolesEnv, self).__init__()
    self.cart = cart
    self.max_height = cart.max_height()
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
      low=cart.motor.min_Va,
      high=cart.motor.max_Va, 
      dtype=np.float32
    )

    num_of_poles = cart.num_of_poles()
    obs_lower_bound = np.hstack([np.array([cart.min_x, np.finfo(np.float32).min]), np.tile(np.array([0.0, np.finfo(np.float32).min]), num_of_poles)])
    obs_upper_bound = np.hstack([np.array([cart.max_x, np.finfo(np.float32).max]), np.tile(np.array([2*pi, np.finfo(np.float32).max]), num_of_poles)])
    self.observation_space = spaces.Box(
      low=obs_lower_bound,
      high=obs_upper_bound, 
      shape=(2+2*num_of_poles,), 
      dtype=np.float32
    )

  def get_state(self):
    state = [self.cart.position(), self.cart.velocity()]
    for pole in self.cart:
      state.extend([pole.angle(), pole.angular_velocity()])
    return np.array(state, dtype=np.float32)


  def step(self, action: float):
    reward = 0
    Va = action
    self.cart.update(self.dt, Va, self.g)
    state = self.get_state()
    x = state[0]
    y = self.cart.end_height()

    terminated = bool(
      x > self.cart.max_x or
      x < self.cart.min_x or 
      self.counter_steps_not_up > int(0.1/self.dt)
    )

    if y > self.max_height * 0.95:
      reward += 0
      self.counter_steps_not_up = 0
      # if y > self.max_height * 0.95:
      reward += 1/(1+abs(x-0))
      if abs(x-0) < 0.1:
        self.counter_steps_up += 1
        reward += 1
      else:
        self.counter_steps_up = 0

    else:
      reward -= 1.0
      self.counter_steps_not_up += 1
      self.counter_steps_up = 0

    if terminated:
      reward -= 10
      self.counter_steps_not_up = 0
      self.counter_steps_up = 0

    won_terminated = bool(
      self.counter_steps_up > int(10/self.dt)
    )

    if won_terminated and not terminated:
      terminated = won_terminated
      reward += 10
      self.counter_steps_not_up = 0
      self.counter_steps_up = 0
        
    return self.get_state(), reward, terminated, {}, False

  def reset(self):
    self.close()
    self.cart.reset(uniform(self.cart.min_x, self.cart.max_x)*0.8)
    for pole in self.cart:
      pole.reset(radians(uniform(-15, 15)))
    self.cart.motor.reset()
    return self.get_state(), {"Msg": "Reset env"}

  def si_to_pixels(self, x: float):
    return int(x * 500)

  def render(self, optional_text: list[str] = []):
    if not self.screen:
      pygame.init()

      self.screen = pygame.display.set_mode(self.size)
      self.font = pygame.font.Font('freesansbold.ttf', 20)
      self.start_time = perf_counter()
      self.i = 0

    self.screen.fill(Colors.gray)

    x0 = self.si_to_pixels(self.cart.position()) + self.width//2
    y0 = self.height//2
    pygame.draw.rect(self.screen, self.cart.color, (x0, y0, 20, 10))

    max_x = self.width//2 + self.si_to_pixels(self.cart.max_x)
    min_x = self.width//2 + self.si_to_pixels(self.cart.min_x)
    pygame.draw.rect(self.screen, self.cart.color, (min_x-10, y0, 10, 10))
    pygame.draw.rect(self.screen, self.cart.color, (max_x+20, y0, 10, 10))

    motor_x0 = min_x-100
    motor_sin = self.si_to_pixels(sin(-self.cart.motor.angle())*0.05)
    motor_cos = self.si_to_pixels(cos(-self.cart.motor.angle())*0.05)

    pygame.draw.polygon(self.screen, self.cart.motor.color, [
      (motor_x0+motor_sin, y0+motor_cos),
      (motor_x0+motor_cos, y0-motor_sin),
      (motor_x0-motor_sin, y0-motor_cos),
      (motor_x0-motor_cos, y0+motor_sin),
    ])

    x0 += 10
    for pole in self.cart:
      x1 = x0 + self.si_to_pixels(pole.l * sin(pole.angle()))
      y1 = y0 + self.si_to_pixels(-pole.l * cos(pole.angle()))
      pygame.draw.line(self.screen, pole.color, (x0, y0), (x1, y1), 10)
      x0 = x1
      y0 = y1
  
    texts = [
      f"Time: {round(self.i*self.dt,2)} s",
      "",
      "Cart:",
      f"Position: {round(self.cart.position(),2)} m",
      f"Velocity: {round(self.cart.velocity(),2)} m/s",
      f"Acceleration: {round(self.cart.acceleration(),2)} m/s^2",
      "",
      "Motor:",
      f"Angle: {round(self.cart.motor.angle(),2)} rad",
      f"Angular velocity: {round(self.cart.motor.angular_velocity(),2)} rad/s",
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
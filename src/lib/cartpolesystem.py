from __future__ import annotations
import numpy as np
from numpy import sin, cos, pi
from numpy.random import multivariate_normal
from lib.colors import color
from lib.numerical import fe_step, rk4_step
from scipy.signal import cont2discrete

class CartPoleSystem:
  def __init__(
    self,
    cart: tuple[float, float, float, float, float, color],
    motor: tuple[float, float, float, float, float, float, color],
    poles: list[tuple[float, float, float, float, color]],
    g: float,
    dt: float,
    integrator: str = "rk4",
    dynamics: str = "nonlinear",
    system_noise_covariance = None,
  ):

    self.cart = np.array(cart[:-1], dtype=np.float64)
    self.dt = dt
    x0, m, u_c, min_x, max_x, cart_color = cart
    self.m = m
    self.u_c = u_c
    self.min_x = min_x
    self.max_x = max_x
    self.cart_color = cart_color
    
    self.motor = np.array(motor[:-1], dtype=np.float64)
    Ra, Jm, Bm, K, r, max_Va, motor_color = motor
    self.Ra = Ra
    self.Jm = Jm
    self.Bm = Bm
    self.K = K
    self.r = r
    self.max_Va = max_Va
    self.motor_color = motor_color

    self.num_poles = len(poles)
    self.poles = np.array([pole[:-1] for pole in poles], dtype=np.float64)
    self.pole_colors = [pole[-1] for pole in poles]
    self.M = self.m + sum(mp for (_, mp, _, _) in self.poles)
    self.g = g

    self.state_lower_bound = np.array([ np.hstack([np.array([min_x, np.finfo(np.float64).min]), np.tile(np.array([-2*pi, np.finfo(np.float64).min]), self.num_poles)]) ]).T
    self.state_upper_bound = np.array([ np.hstack([np.array([max_x, np.finfo(np.float64).max]), np.tile(np.array([ 2*pi, np.finfo(np.float64).max]), self.num_poles)]) ]).T
   
    self.control_lower_bound = np.vstack([-max_Va])
    self.control_upper_bound = np.vstack([max_Va])

    self.integrator = integrator
    self.dynamics = dynamics

    if system_noise_covariance is None:
      system_noise_covariance = np.diag(np.zeros(2+2*self.num_poles))

    self.system_noise_covariance = system_noise_covariance

    self.reset(self.get_initial_state())

    x0 = np.zeros((2+2*self.num_poles, 1))
    u0 = np.vstack([0])

    self.linearize(x0, u0)
    self.discretize()
    
  def reset(self, initial_state):
    self.state = np.hstack([
      initial_state
    ])
    self.d_state = np.hstack([
      np.zeros(initial_state.shape)
    ])

  def clip_state(self, state):
    for k in range(self.num_poles):
      if state[2+k*2] > 2*pi:
        state[2+k*2] -= 2*pi
      elif state[2+k*2] < -2*pi:
        state[2+k*2] += 2*pi

    state = np.clip(state, self.state_lower_bound, self.state_upper_bound)

    return state

  def get_initial_state(self):
    state = np.array([np.hstack([np.array([self.cart[0], 0]), np.hstack([[angle0, 0] for angle0 in self.poles.T[0]])])]).T
    return state

  def differentiate(self, state, u):
    Va = u[0][0]
    state = state.T[0]
    sum1 = sum([m*sin(state[2+k*2])*cos(state[2+k*2]) for k,(_,m,_,_) in enumerate(self.poles)])
    sum2 = sum([m*(l/2)*state[2+k*2+1]**2*sin(state[2+k*2]) for k,(_,m,l,_) in enumerate(self.poles)])
    sum3 = sum([(u_p*state[2+k*2+1]*cos(state[2+k*2]))/(l/2) for k,(_,_,l,u_p) in enumerate(self.poles)])
    sum4 = sum([m*cos(state[2+k*2])**2 for k,(m,_,_,_) in enumerate(self.poles)])

    d_x = state[1]
    dd_x = (self.g*sum1-(7/3)*((1/self.r**2)*((self.K/self.Ra)*(Va*self.r-self.K*d_x)-self.Bm*d_x)+sum2-self.u_c*d_x)-sum3)/(sum4-(7/3)*(self.M+self.Jm/(self.r**2)))

    dd_thetas = np.hstack([[state[2+k*2+1],(3/(7*l/2)*(self.g*sin(state[2+k*2])-dd_x*cos(state[2+k*2])-u_p*state[2+k*2+1]/(m*l/2)))] for k,(_,m,l,u_p) in enumerate(self.poles)])
    
    new_state = np.array([np.hstack([d_x, dd_x, dd_thetas])]).T
    return new_state

  def linear_differentiate(self, state, u):
    Ax_d = self.A_d @ state
    Bu_d = self.B_d @ u
    next_state = Ax_d + Bu_d
    return next_state

  def get_state(self, t=-1):
    return np.array([self.state[:,t]]).T

  def step(self, dt: float, u, update=True):
    state = self.get_state()

    integration_scheme = rk4_step

    if self.dynamics == "linear":
      next_state = self.linear_differentiate(state, u)
      d_state = np.zeros(next_state.shape)

    else:
      if self.integrator == "fe":
        integration_scheme = fe_step
      elif self.integrator == "rk4":
        integration_scheme = rk4_step

      next_state, d_state = integration_scheme(dt, self.differentiate, state, u)

    next_state = self.clip_state(next_state)

    system_noise = np.array([multivariate_normal(np.zeros(len(state)), self.system_noise_covariance)]).T
    next_state += system_noise

    if update:
      self.update(next_state, d_state)

    return next_state, d_state

  def update(self, next_state, d_state):
    self.state = np.hstack([self.state, next_state])
    self.d_state = np.hstack([self.d_state, d_state])

  def max_height(self) -> float:
    return sum(l for _,_,l,_ in self.poles)

  def end_height(self) -> float:
    state = self.get_state()
    state = state.T[0]
    return sum(l*cos(state[2+k*2]) for k, (_,_,l,_) in enumerate(self.poles))

  def linearize(self, initial_state, initial_control):
    initial_state_raw = initial_state
    initial_control_raw = initial_control

    initial_state = initial_state.T[0]
    initial_control = initial_control.T[0]

    x = initial_state[0]
    d_x = initial_state[1]

    Va = initial_control[0]

    r = self.r
    K = self.K
    Ra = self.Ra
    Bm = self.Bm
    u_c = self.u_c
    M = self.M
    Jm = self.Jm
    g = self.g
    n = self.num_poles

    f1s = np.hstack([np.array([0, 1]), np.tile([0, 0], n)])
    f2_dx = 7/3*(1/r**2*(K**2/Ra+Bm)+u_c)/(sum([m*cos(initial_state[2+2*i])**2 for i,(_,m,_,_) in enumerate(self.poles)])-7/3*(M+Jm/r**2))
    
    f2_theta_g = g*sum([m*cos(initial_state[2+2*i])*sin(initial_state[2+2*i]) for i,(_,m,_,_) in enumerate(self.poles)])+7/3*(1/r**2*(K*(K*d_x-Va*r)/Ra+Bm*d_x)-sum([m*l/2*initial_state[2+2*i+1]**2*sin(initial_state[2+2*i]) for i,(_,m,l,_) in enumerate(self.poles)])+u_c*d_x)-sum([u_p*initial_state[2+2*i+1]*cos(initial_state[2+2*i])/(l/2) for i,(_,_,l,u_p) in enumerate(self.poles)])
    f2_theta_h = sum([m*cos(initial_state[2+2*i])**2 for i,(_,m,_,_) in enumerate(self.poles)])-7/3*(M+Jm/r**2)

    def f2_theta_dg(k):
      theta_k = initial_state[2+2*k]
      d_theta_k = initial_state[2+2*k+1]
      _, m_k, l_k, u_p_k = self.poles[k]
      return g*m_k*(cos(theta_k)**2-sin(theta_k)**2)+7/3*(-m_k*l_k/2*d_theta_k**2*cos(theta_k))+u_p_k*d_theta_k*sin(theta_k)/(l_k/2)

    def f2_theta_dh(k):
      theta_k = initial_state[2+2*k]
      d_theta_k = initial_state[2+2*k+1]
      _, m_k, l_k, u_p_k = self.poles[k]
      return -2*m_k*sin(theta_k)*cos(theta_k)

    def f2_theta(k):
      return (f2_theta_dg(k)*f2_theta_h-f2_theta_g*f2_theta_dh(k))/f2_theta_h**2

    def f2_dtheta(k):
      theta_k = initial_state[2+2*k]
      d_theta_k = initial_state[2+2*k+1]
      _, m_k, l_k, u_p_k = self.poles[k]
      return (7/3*(-2*m_k*l_k/2*d_theta_k*sin(theta_k))-u_p_k*cos(theta_k)/(l_k/2))/(sum([m*cos(initial_state[2+2*i])**2 for i,(_,m,_,_) in enumerate(self.poles)])-7/3*(M+Jm/r**2))

    f2_Va = (-7*K/(3*r*Ra))/(sum([m*cos(initial_state[2+2*i])**2 for i,(_,m,_,_) in enumerate(self.poles)])-7/3*(M+Jm/r**2))

    f2s = np.hstack([np.array([0, f2_dx]), np.hstack([[f2_theta(k), f2_dtheta(k)] for k in range(n)])])

    f2 = (self.differentiate(initial_state_raw, initial_control_raw)).T[0][1]

    def fk2_dx(k):
      theta_k = initial_state[2+2*k]
      d_theta_k = initial_state[2+2*k+1]
      _, m_k, l_k, u_p_k = self.poles[k]
      return (-3/(7*l_k/2))*cos(theta_k)*f2_dx

    def fk2_theta_k(k):
      theta_k = initial_state[2+2*k]
      d_theta_k = initial_state[2+2*k+1]
      _, m_k, l_k, u_p_k = self.poles[k]
      return (3/(7*l_k/2))*(g*cos(theta_k)-f2_theta(k)*cos(theta_k)+f2*sin(theta_k))
    
    def fk2_theta_j(k, j):
      theta_k = initial_state[2+2*k]
      d_theta_k = initial_state[2+2*k+1]
      _, m_k, l_k, u_p_k = self.poles[k]
      return (-3/(7*l_k/2))*cos(theta_k)*f2_theta(j)

    def fk2_dtheta_k(k):
      theta_k = initial_state[2+2*k]
      d_theta_k = initial_state[2+2*k+1]
      _, m_k, l_k, u_p_k = self.poles[k]
      return (-3/(7*l_k/2))*(f2_dtheta(k)*cos(theta_k)+u_p_k/(m_k*l_k/2))

    def fk2_dtheta_j(k, j):
      theta_k = initial_state[2+2*k]
      d_theta_k = initial_state[2+2*k+1]
      _, m_k, l_k, u_p_k = self.poles[k]
      return (-3/(7*l_k/2))*cos(theta_k)*f2_dtheta(j)

    def fk2_theta(k,j):
      if k == j:
        return fk2_theta_k(k)
      else:
        return fk2_theta_j(k,j)

    def fk2_dtheta(k,j):
      if k == j:
        return fk2_dtheta_k(k)
      else:
        return fk2_dtheta_j(k,j)

    f_rest = np.vstack([
      [
        np.hstack([np.zeros(2+2*k+1), [1], np.zeros(2*(n-k-1))]),
        np.hstack([[0, fk2_dx(k)], np.hstack([[fk2_theta(k,j), fk2_dtheta(k,j)] for j in range(n)])])
      ] for k in range(n)
    ])

    A = np.array(np.vstack([
      np.array([
        f1s,
        f2s,
      ]),
      f_rest
    ]), dtype=np.float64)

    def fk2_Va(k):
      theta_k = initial_state[2+2*k]
      d_theta_k = initial_state[2+2*k+1]
      _, m_k, l_k, u_p_k = self.poles[k]
      return (-3*cos(theta_k)/(7*l_k/2))*f2_Va

    B = np.array([np.hstack([
      [0, f2_Va], 
      np.hstack([[0, fk2_Va(k)] for k in range(n)])
    ])], dtype=np.float64).T

    self.A = A
    self.B = B

    return A, B

  def discretize(self):
    dlti = cont2discrete((self.A,self.B,None,None),self.dt)
    A_d = np.array(dlti[0])
    B_d = np.array(dlti[1])
    self.A_d = A_d
    self.B_d = B_d
    return A_d, B_d
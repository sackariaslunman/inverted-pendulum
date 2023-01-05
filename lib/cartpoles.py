from __future__ import annotations
import numpy as np
from numpy import sin, cos, pi
from lib.numerical import fe_step, rk4_step

class CartPoles:
    def __init__(
        self,
        cart: tuple[float, float, float, float, float],
        motor: tuple[float, float, float, float, float, float, float],
        poles: list[tuple[float, float, float, float]],
        g: float,
        integrator: str = "rk4"
    ):
        self.cart = np.array(cart, dtype=np.float32)
        x0, m, u_c, min_x, max_x = cart
        self.m = m
        self.u_c = u_c
        self.min_x = min_x
        self.max_x = max_x
        
        self.motor = np.array(motor, dtype=np.float32)
        Ra, Jm, Bm, K, r, min_Va, max_Va = motor
        self.Ra = Ra
        self.Jm = Jm
        self.Bm = Bm
        self.K = K
        self.r = r
        self.min_Va = min_Va
        self.max_Va = max_Va

        self.num_poles = len(poles)
        self.poles = np.array(poles, dtype=np.float32)
        self.M = self.m + sum(mp for (_, mp, _, _) in self.poles)
        self.g = g

        self.integrator = integrator

        self.reset(self.get_initial_state())

    def reset(self, initial_state):
        self.state = np.array([
            initial_state
        ])
        self.d_state = np.array([
            np.zeros(len(initial_state))
        ])

    def max_height(self) -> float:
        return sum(l for _,_,l,_ in self.poles)

    def end_height(self) -> float:
        state = self.get_state()
        return sum(l*cos(state[3+k*2]) for k, (_,_,l,_) in enumerate(self.poles))

    def get_state(self, t=-1):
        return self.state[t]

    def get_initial_state(self):
        return np.hstack([np.array([self.cart[0], 0, 0]), np.hstack([[angle0, 0] for angle0 in self.poles.T[0]])])

    def forward(self, state, u):
        Va = u[0]
        sum1 = sum([m*sin(state[3+k*2])*cos(state[3+k*2]) for k,(_,m,_,_) in enumerate(self.poles)])
        sum2 = sum([m*(l/2)*state[3+k*2+1]**2*sin(state[3+k*2]) for k,(_,m,l,_) in enumerate(self.poles)])
        sum3 = sum([(u_p*state[3+k*2+1]*cos(state[3+k*2]))/(l/2) for k,(_,_,l,u_p) in enumerate(self.poles)])
        sum4 = sum([m*cos(state[3+k*2])**2 for k,(m,_,_,_) in enumerate(self.poles)])

        d_x = state[1]
        dd_x = (self.g*sum1-(7/3)*((1/self.r**2)*((self.K/self.Ra)*(Va*self.r-self.K*state[1])-self.Bm*state[1])+sum2-self.u_c*state[1])-sum3)/(sum4-(7/3)*(self.M+self.Jm/(self.r**2)))
        d_omega = d_x / self.r

        dd_thetas = np.hstack([[state[3+k*2+1],(3/(7*l/2)*(self.g*sin(state[3+k*2])-dd_x*cos(state[3+k*2])-u_p*state[3+k*2+1]/(m*l/2)))] for k,(_,m,l,u_p) in enumerate(self.poles)])
        return np.hstack([d_x, dd_x, d_omega, dd_thetas])

    def step(self, dt: float, Va: float):
        state = self.get_state()
        next_state, d_state = None, None
        
        if self.integrator == "fe":
            next_state, d_state = fe_step(dt, self.forward, state, [Va])
        else:
            next_state, d_state = rk4_step(dt, self.forward, state, [Va])
        next_state = self.clamp(next_state)

        self.update(next_state, d_state)

        return next_state

    def clamp(self, state):
        x = state[0]
        if x > self.max_x:
            x = self.max_x
        elif x < self.min_x:
            x = self.min_x
        state[0] = x

        for k in range(self.num_poles):
            state[3+k*2] %= 2*pi

        return state
    
    def update(self, next_state, d_state):
        self.state = np.vstack([self.state, next_state])
        self.d_state = np.vstack([self.d_state, d_state])
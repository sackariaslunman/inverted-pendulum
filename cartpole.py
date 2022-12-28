from __future__ import annotations
from math import sin, cos, pi

class DCMotor:
    def __init__(self, max_V: float, min_V: float, resistance: float, inductance: float, ke: float, kt: float, inertia: float, friction: float, radius: float, color: tuple[int,int,int]):
        self.max_V = max_V
        self.min_V = min_V

        self.Ra = resistance
        self.La = inductance
        self.Ke = ke
        self.Kt = kt
        self.Jm = inertia
        self.Bm = friction

        self.color = color
        self.r = radius

        self.state = {
            "omega": [0],
            "d_omega": [0]
        }

    def angle(self, t=None):
        if not t:
            t = len(self.state["omega"])-1
        return self.state["omega"][t]

    def angular_velocity(self, t=None):
        if not t:
            t = len(self.state["d_omega"])-1
        return self.state["d_omega"][t]

    def update(self, dt, d_x):
        d_omega = d_x / self.r
        omega = self.angle() + d_omega * dt

        self.state["d_omega"].append(d_omega)
        self.state["omega"].append(omega)

class Cart:
    def __init__(self, mass: float, cart_friction: float, x0: float, min_x: float, max_x: float, color: tuple[int,int,int], motor: DCMotor, pole: Pole):
        self.m = mass
        self.u_c = cart_friction
        self.min_x = min_x
        self.max_x = max_x
        self.color = color

        self.motor = motor
        
        self.state = {
            "x": [x0],
            "d_x": [0],
            "dd_x": [0]
        }

        self.pole = pole

    def __iter__(self):
        return iter(self.pole)

    def total_mass(self):
        M = self.m
        for pole in self:
            M += pole.m
        return M

    def x(self, t=None):
        if not t:
            t = len(self.state["x"])-1
        return self.state["x"][t]

    def velocity(self, t=None):
        if not t:
            t = len(self.state["d_x"])-1
        return self.state["d_x"][t]

    def acceleration(self, t=None):
        if not t:
            t = len(self.state["dd_x"])-1
        return self.state["dd_x"][t]

    def clamp(self, x: float):
        if x > self.max_x:
            return self.max_x
        elif x < self.min_x:
            return self.min_x
        return x

    def update(self, dt: float, Va: float, g: float):
        x = self.x()
        d_x = self.velocity()
        M = self.total_mass()

        sum1 = sum([pole.m*sin(pole.angle())*cos(pole.angle()) for pole in self])
        sum2 = sum([pole.m*pole.lh()*pole.angular_velocity()**2*sin(pole.angle()) for pole in self])
        sum3 = sum([pole.u_p*pole.angular_velocity()*cos(pole.angle())/pole.lh() for pole in self])
        sum4 = sum([pole.m*cos(pole.angle())**2 for pole in self])

        dd_x = (g*sum1-7/3*((1/self.motor.r**2)*(self.motor.Kt/self.motor.Ra*(Va*self.motor.r-self.motor.Ke*d_x)-self.motor.Bm*d_x)+sum2-self.u_c*d_x)-sum3)/(sum4-7/3*(M+self.motor.Jm/self.motor.r**2))
        d_x = d_x + dd_x * dt
        x = self.clamp(x + d_x * dt)

        self.state["dd_x"].append(dd_x)
        self.state["d_x"].append(d_x)
        self.state["x"].append(x)

        for pole in self:
            pole.update(dt, dd_x, g)
        self.motor.update(dt, d_x)

class Pole:
    def __init__(self, mass: float, angle0: float, length: float, pole_friction: float, color: tuple[int,int,int], child: Pole | None):
        self.m = mass
        self.l = length
        self.u_p = pole_friction
        self.color = color  

        self.state = {
            "theta": [angle0],
            "d_theta": [0],
        }

        self.child = child

    def lh(self):
        return self.l/2

    def __iter__(self):
        poles = [self]
        pole = self.child
        while pole:
            poles.append(pole)
            pole = pole.child
        return iter(poles)

    def angle(self, t=None):
        if not t:
            t = len(self.state["theta"])-1
        return self.state["theta"][t]
    
    def angular_velocity(self, t=None):
        if not t:
            t = len(self.state["d_theta"])-1
        return self.state["d_theta"][t]
    
    def update(self, dt: float, dd_x: float, g: float):
        theta = self.angle()
        d_theta = self.angular_velocity()

        dd_theta = 3/(7*self.lh())*(g*sin(theta)-dd_x*cos(theta)-self.u_p*d_theta/(self.m*self.lh()))
        d_theta = d_theta + dd_theta * dt
        theta = theta + d_theta * dt

        self.state["d_theta"].append(d_theta)
        self.state["theta"].append(theta)

    def __str__(self):
        return f"Pole:\n  Mass: {self.m}\n  Length: {self.l}\n  Friction: {self.u_p}\n  Child: {self.child is not None}"
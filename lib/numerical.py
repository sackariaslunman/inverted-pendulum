#!/usr/bin/env python3
def fe_step(dt: float, differentiate, x0, u):
    d_x = differentiate(x0, u)
    x = x0 + dt * d_x
    return x, d_x

def rk4_step(dt: float, differentiate, x0, u):
    f1 = differentiate(x0, u)
    f2 = differentiate(x0 + (dt/2) * f1, u)
    f3 = differentiate(x0 + (dt/2) * f2, u)
    f4 = differentiate(x0 + dt * f3, u)

    d_x = f1 + 2*f2 + 2*f3 + f4
    x = x0 + (dt/6) * d_x
    return x, d_x
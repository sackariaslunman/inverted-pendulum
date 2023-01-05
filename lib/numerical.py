def rk4_step(dt: float, function, x0, u):
    f1 = function(x0, u)
    f2 = function(x0 + (dt/2) * f1, u)
    f3 = function(x0 + (dt/2) * f2, u)
    f4 = function(x0 + dt * f3, u)

    d_x = f1 + 2*f2 + 2*f3 + f4
    x = x0 + (dt/6) * d_x
    return x, d_x

def fe_step(dt: float, function, x0, u):
    d_x = function(x0, u)
    x = x0 + dt * d_x
    return x, d_x
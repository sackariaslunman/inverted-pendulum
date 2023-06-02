import numpy as np
from lib.cartpolesystem_new import CartPoleSystem as CartPoleSystemNew, Cart, DCMotor, Pole
from time import perf_counter

cart = Cart(2, 1, (-1, 1))
motor = DCMotor(1, 1, 1, 1, 1, 1, (-24, 24), (-10, 10))
poles = [Pole(1, 1, 1), Pole(1, 1, 1)]
g = 9.81
dt = 0.01

dt = 0.05
g = 9.81

state0 = np.array([10,5,1,2,0,1])
control0 = np.array([10])

cart_pole_system_new = CartPoleSystemNew(cart, motor, poles, g)
# print("Vars:", cart_pole_system_new.vars)
# print("Eqs:", cart_pole_system_new.eqs)
# print("Jacobian:", cart_pole_system_new.F.shape)

# start_time = perf_counter()
# print("Linearized:", cart_pole_system_new.linearize(state0, control0))
# stop_time = perf_counter()

# print("Time:", stop_time - start_time)

state0 = np.array([10,0.1,1,0,2,0])
control0 = np.array([3])
state = state0

start_time = perf_counter()

for i in range(1):
    d_state = cart_pole_system_new.differentiate(state, control0)
    state = state + d_state*dt
    # print("state", state)
stop_time = perf_counter()

print("Final state:", state)
print("Nonlinear Time:", stop_time - start_time)

state0 = np.array([10,0.1,1,0,2,0])
control0 = np.array([3])
state = state0

start_time = perf_counter()

for i in range(1):
    d_state = cart_pole_system_new.linear_differentiate(state, control0, state, control0)
    state = state + d_state*dt
    # print("state", state)
stop_time = perf_counter()

print("Final state:", state)
print("linear Time:", stop_time - start_time)
# import multiprocessing as mp
from lib.cartpolecontroller import CartPoleController
from lib.cartpolesimulator import CartPoleEnvSimulator, CartPoleSerialSimulator
from lib.cartpolesystem import CartPoleStepperMotorSystem, Pole, Cart
from lib.motors import StepperMotor

def main():
    # mp.set_start_method('spawn')
    
    dt = 0.01
    g = 9.81
    r = 0.04456
    x_max = 1.15/2
    l1 = 0.225
    m1 = 0.0446
    m = 0.2167
    
    cart = Cart(m, 0.01, (-x_max, x_max), 0.1)
    motor = StepperMotor(r, (-5, 5), 0.2, (-3, 3), 0.1)
    poles = [Pole(m1, l1, 0.001)]
    system = CartPoleStepperMotorSystem(cart, motor, poles, g)
    sim = CartPoleEnvSimulator(dt, system)
    controller = CartPoleController(sim, dt)

    # sim.run("COM3", 500000)
    sim.run()
    controller.run()

if __name__ == '__main__':
    main()
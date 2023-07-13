from lib.cartpolecontroller import CartPoleController
from lib.cartpolesimulator import CartPoleEnvSimulator
from lib.cartpolesystem import CartPoleStepperMotorSystem, Pole, Cart
from lib.motors import StepperMotor

def main():
    dt = 0.01
    g = 9.81

    cart = Cart(0.1, 0.01, (-0.8, 0.8), 0.1)
    motor = StepperMotor(0.06, (-5, 5), 0.2, (-2, 2), 0.1)
    poles = [Pole(0.05, 0.15, 0.001), Pole(0.05, 0.2, 0.001)]
    system = CartPoleStepperMotorSystem(cart, motor, poles, g)
    sim = CartPoleEnvSimulator(dt, system)
    sim.run()

    controller = CartPoleController(sim)
    controller.run()

if __name__ == '__main__':
    main()
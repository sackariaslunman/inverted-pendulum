from serial.tools.list_ports import comports
from lib.cartpolecontroller import CartPoleController
from lib.cartpolesimulator import CartPoleEnvSimulator, CartPoleSerialSimulator
from lib.cartpolesystem import CartPoleSystem, Pole, Cart
from lib.motors import StepperMotor

def main():
    dt = 0.005
    g = 9.81
    r = 0.04456
    x_max = 1.15/2
    l1 = 0.225
    m1 = 0.0445
    m = 0.2167
    d1 = 0.0003
    # J = 8.25e-4
    J = 2.23e-4

    cart = Cart(m, 0.01, (-x_max, x_max), 0.1)
    motor = StepperMotor(r, (-2.7, 2.7), 0.1, (-2, 2), 0.1)
    poles = [Pole(m1, l1, l1/2, d1, J)]
    system = CartPoleSystem(cart, motor, poles, g)

    if (input("Simulate (y/n)?") == "y"):
        sim = CartPoleEnvSimulator(dt, system)
        controller = CartPoleController(sim, dt)
        sim.run()
        controller.run()
    else:
        sim = CartPoleSerialSimulator(dt, system)
        controller = CartPoleController(sim, dt)
        # open ports
        print("Available ports: ")
        ports = [port.name for port in comports()]
        if len(ports) == 0:
            raise ValueError("No ports available")

        for port in ports:
            print(f"\t{port}")

        port = input(f"Port (press enter to select {ports[0]}): ")
        if port == "":
            port = ports[0]
        sim.run(port, 500000)
        controller.run()

if __name__ == '__main__':
    main()
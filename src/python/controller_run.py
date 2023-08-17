from serial.tools.list_ports import comports
from lib.cartpolecontroller import CartPoleController
from lib.cartpolesimulator import CartPoleEnvSimulator, CartPoleSerialSimulator
from lib.cartpolesystem import CartPoleSystem, Pole, Cart, StepperMotor

def main():
    dt = 0.01
    g = 9.81
    r = 0.04456
    m = 0.2167
    x_max = 1.15/2

    l1 = 0.225
    a1 = l1/2
    m1 = 0.0445 #+0.069*2+0.040
    d1 = 0.0003
    J1 = 2.23e-4#*(m1/0.0445)

    # m2 = 0.062
    # l2 = 0.115
    # a2 = 0.046
    # d2 = 0.00001
    # J2 = 1.694e-4

    print("Calculating equations (1-5 min)...")
    cart = Cart(m, 0.01, (-x_max, x_max), 0.1)
    motor = StepperMotor(r, (-2.7, 2.7), 0.1, (-2, 2), 0.1)
    poles = [Pole(m1, l1, a1, d1, J1)]
    path = "./cartpolesystems"
    
    system = CartPoleSystem(cart, motor, poles, g, False)

    if system.check_equations(path):
        system.import_equations(path)
    else:
        system.set_equations()
        system.export_equations(path)
    
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
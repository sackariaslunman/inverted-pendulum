from serial.tools.list_ports import comports
from lib.cartpolecontroller import CartPoleController
from lib.cartpolesimulator import CartPoleEnvSimulator, CartPoleSerialSimulator
from lib.cartpolesystem import CartPoleSystem, Pole, Cart, StepperMotor

def main():
    dt = 0.005
    g = 9.81
    r = 0.04456
    m = 0.2167
    x_max = 1.15/2

    d = 0.0001

    # 45.4 g outer
    l1 = 0.200
    a1 = 0.108709
    m1 = 0.13725
    d1 = d
    J1 = 0.00092347
    pole1 = Pole(m1, l1, a1, d1, J1)

    # 55 g outer
    l2 = 0.200
    a2 = 0.116161
    m2 = 0.14945
    d2 = d
    J2 = 0.001017455
    pole2 = Pole(m2, l2, a2, d2, J2)

    print("Calculating equations (1-5 min)...")
    cart = Cart(m, 0.01, (-x_max, x_max), 0.15)
    motor = StepperMotor(r, (-2.7, 2.7), 0.15, (-2, 2), 0.15)
    poles = [
        pole1,
        pole2,
    ]
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
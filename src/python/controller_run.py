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

    # 115 mm, 45.4 g outer
    # l1 = 0.115
    # a1 = 0.066506
    # m1 = 0.123919
    # d1 = 0.006
    # J1 = 0.0002898662297
    # pole1 = Pole(m1, l1, a1, d1, J1)

    # # 200 mm, 45.4 g outer
    # l1 = 0.200
    # a1 = 0.108709
    # m1 = 0.13725
    # d1 = 0.003
    # J1 = 0.00092347
    # pole1 = Pole(m1, l1, a1, d1, J1)

    # # 200 mm, 55 g outer
    # l2 = 0.200
    # a2 = 0.116161
    # m2 = 0.14945
    # d2 = 0.001
    # J2 = 0.001017455
    # pole2 = Pole(m2, l2, a2, d2, J2)

    # 200 mm, 0 g outer
    l2 = 0.200
    a2 = 0.067341
    m2 = 0.09445
    d2 = 0.0001
    J2 = 0.00040300
    pole1 = Pole(m2, l2, a2, d2, J2)

    # # 200 mm, 55 g outer + 72.5 g middle
    # l2 = 0.200
    # a2 = 0.100214
    # m2 = 0.22195
    # d2 = d
    # J2 = 0.001133810
    # pole2 = Pole(m2, l2, a2, d2, J2)

    print("Calculating equations (1-5 min)...")
    cart = Cart(m, 0.01, (-x_max, x_max), 0.2)
    motor = StepperMotor(r, (-2.7, 2.7), 0.2, (-2, 2), 0.2)
    poles = [
        pole1,
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
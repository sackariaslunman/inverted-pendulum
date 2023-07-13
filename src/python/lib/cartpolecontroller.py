import numpy as np
from numpy import radians
from threading import Thread
from .cartpolesimulator import CartPoleSimulator
from .direct_collocation import CartPoleStepperMotorDirectCollocation
from .regulators import LQR

class CartPoleController:
    def __init__(self, simulator: CartPoleSimulator):
        self.__simulator = simulator
        self.__thread = Thread(target=self.__run_loop)
        self.__is_running = False

    def disable_control(self):
        if not self.__simulator.running:
            return
        self.__simulator.disable_control()

    def create_trajectory(self, pos: float, pole_pos: list[bool]):
        if not self.__simulator.running or self.__simulator.is_in_trajectory:
            return
        
        end_time = 3
        dt_collocation = 0.03
        N_collocation = int(end_time/dt_collocation)+1

        pole_states = np.array([[float(0 if pos else radians(180)), 0.0] for pos in pole_pos]).flatten()
        target_state = np.array([pos, 0] + pole_states.tolist())

        N = int(end_time/self.__simulator.dt)

        system = self.__simulator.system

        direct_collocation = CartPoleStepperMotorDirectCollocation(N, N_collocation, system, 0.0001)

        x0 = self.__simulator.state
        states, controls = direct_collocation.make_solver(end_time, x0, target_state)

        C = np.diag([1, 1]+[1, 1]*system.num_poles)
        D = np.zeros((1, 1))
        Q = np.diag([100, 10]+[100, 10]*system.num_poles)
        R = np.diag([1])

        As, Bs = np.vectorize(direct_collocation.linearize, signature='(n),(m)->(n,n),(n,m)')(states, controls)
        A_ds, B_ds = np.vectorize(LQR.discretize, signature='(),(n,n),(n,m),(a,b),(c,d)->(n,n),(n,m)')(self.__simulator.dt, As, Bs, C, D)
        _, K_ds = LQR.calculate_finite_K_ds(A_ds, B_ds, Q, R)

        self.__simulator.send_trajectory(states, controls, K_ds)

    def run(self):
        if self.__is_running:
            return
        self.__is_running = True
        self.__thread.start()

    def __run_loop(self):
        while self.__is_running:
            command = input()
            if command == 'q':
                self.stop()
            elif command == 'c':
                self.disable_control()
            elif command == 't':
                try:
                    pos = float(input('Enter target position (-0.5 to 0.5): '))
                    if pos > 0.5:
                        pos = 0.5
                    elif pos < -0.5:
                        pos = -0.5
                    pole_pos = [bool(int(input(f"Enter pole {i} position ('0' for down or '1' for up): "))) for i in range(self.__simulator.system.num_poles)]
                    self.create_trajectory(pos, pole_pos)
                except ValueError:
                    print('Value error: Failed to parse value to number')
            if not self.__simulator.running:
                self.stop()

    def stop(self):
        self.__is_running = False
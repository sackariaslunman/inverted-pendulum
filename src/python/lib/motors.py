class DCMotor:
    def __init__(
        self, 
        torque_constant: float,
        velocity_constant: float,
        resistance: float,
        radius: float,
        motor_inertia: float,
        motor_friction: float,
        Va_bounds: tuple[float, float],
        Va_margin: float,
        Ia_bounds: tuple[float, float],
        Ia_margin: float = 0.1
    ):
        self.Kt = torque_constant
        self.Ke = velocity_constant
        self.Ra = resistance
        self.r = radius
        self.Jm = motor_inertia
        self.Bm = motor_friction
        self.Va_bounds = Va_bounds
        self.Va_margin = Va_margin
        self.Ia_bounds = Ia_bounds
        self.Ia_margin = Ia_margin

class StepperMotor:
    def __init__(
        self, 
        radius: float,
        velocity_bounds: tuple[float, float],
        velocity_margin: float,
        torque_bounds: tuple[float, float],
        torque_margin: float = 0.1
    ):
        self.r = radius
        self.v_bounds = velocity_bounds
        self.v_margin = velocity_margin
        self.torque_bounds = torque_bounds
        self.torque_margin = torque_margin
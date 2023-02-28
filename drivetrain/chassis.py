from ntcore import *


class Chassis:

    state: NetworkTableInstance

    throttle_multiplier: float
    rotation_multiplier: float
    in_LX: FloatSubscriber
    in_LY: FloatSubscriber
    in_RX: FloatSubscriber
    in_RY: FloatSubscriber

    # TODO: Add useful constants
    talon_srx_revolution_steps: int = 4096
    talon_fx_revolution_steps: int = 2048

    def __init__(self, inherited_state: NetworkTableInstance):
        self.state = inherited_state
        self.config = self.state.getTable("drivetrain")

        self.in_LX = self.state.getFloatTopic("controller/LX").subscribe(0.0)
        self.in_LY = self.state.getFloatTopic("controller/LY").subscribe(0.0)
        self.in_RX = self.state.getFloatTopic("controller/RX").subscribe(0.0)
        self.in_RY = self.state.getFloatTopic("controller/RY").subscribe(0.0)

        self.throttle_multiplier = self.config.getNumber("control/throttle", 1.0)
        self.rotation_multiplier = self.config.getNumber("control/rotation", 1.0)

    def drive(self):
        print("No logic connected to drivetrain")

from ntcore import *
from wpilib import *


class Chassis:

    state: NetworkTableInstance
    controller: XboxController

    throttle_multiplier: float
    rotation_multiplier: float

    # TODO: Add useful constants
    talon_srx_revolution_steps: int = 4096
    talon_fx_revolution_steps: int = 4096

    def __init__(self, inherited_state: NetworkTableInstance, controller: XboxController):
        self.state = inherited_state
        self.config = self.state.getTable("drivetrain")
        self.controller = controller

        self.throttle_multiplier = self.config.getNumber("control/throttle", 1.0)
        self.rotation_multiplier = self.config.getNumber("control/rotation", 1.0)

    def drive(self):
        print("No logic connected to drivetrain")

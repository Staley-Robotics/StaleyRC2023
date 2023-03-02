from wpilib import *


class Chassis:

    controller: XboxController

    throttle_multiplier: float = 0.7
    rotation_multiplier: float = 0.4
    direction_multiplier: float = 0.3
    talon_srx_resolution: int = 4096
    talon_fx_resolution: int = 4096

    def __init__(self, controller: XboxController):
        self.controller = controller

    def drive(self):
        print("No logic connected to drivetrain")

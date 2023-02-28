from wpilib import *
from ntcore import *


class Controller(XboxController):
    state: NetworkTableInstance

    def __init__(self, port: int, inherited_state: NetworkTableInstance):
        super().__init__(port)
        self.state = inherited_state
        self.controller = self.state.getTable("controller")

    def update(self):
        self.controller.set("LX", self.getLeftX())
        self.controller.set("LY", self.getLeftY())
        self.controller.set("RX", self.getRightX())
        self.controller.set("RY", self.getRightY())

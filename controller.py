from wpilib import *
from ntcore import *


class Controller(XboxController):
    state: NetworkTableInstance

    def __init__(self, port: int, inherited_state: NetworkTableInstance):
        super().__init__(port)
        self.state = inherited_state
        controller = self.state.getTable("controller")
        self.LX = controller.getFloatTopic("LX").publish()
        self.LY = controller.getFloatTopic("LY").publish()
        self.RX = controller.getFloatTopic("RX").publish()
        self.RY = controller.getFloatTopic("RY").publish()

    def update(self):
        self.LX.set(self.getLeftX())
        self.LY.set(self.getLeftY())
        self.RX.set(self.getRightX())
        self.RY.set(self.getRightY())

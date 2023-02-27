from wpilib import *
from ntcore import *


class Controller(XboxController):
    state: NetworkTableInstance

    def __init__(self, port: int, inherited_state: NetworkTableInstance):
        super().__init__(port)
        self.state = inherited_state
        controller = self.state.getTable("controller")
        self.leftX = controller.getFloatTopic("leftX").publish()
        self.leftY = controller.getFloatTopic("leftY").publish()

    def update(self):
        self.leftX.set(self.getLeftX())
        self.leftY.set(self.getLeftY())

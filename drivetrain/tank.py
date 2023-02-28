from wpilib import *
from ctre import *
from ntcore import *

from drivetrain.chassis import Chassis


class Tank(Chassis):
    config: NetworkTableInstance
    leftModule: MotorControllerGroup
    rightModule: MotorControllerGroup

    def __init__(self, inherited_state: NetworkTableInstance):
        super().__init__(inherited_state)

        self.config = self.state.getTable("tank")

        self.leftModule = MotorControllerGroup(WPI_TalonFX(self.config.getNumber("modules/left/0", 1), "rio"),
                                               WPI_TalonFX(self.config.getNumber("modules/left/1", 2), "rio"))
        self.rightModule = MotorControllerGroup(WPI_TalonFX(self.config.getNumber("modules/right/0", 3), "rio"),
                                                WPI_TalonFX(self.config.getNumber("modules/right/1", 4), "rio"))

    def drive(self):
        speed = self.in_LY.get() * self.throttle_multiplier
        rotation = self.in_LX.get() * self.rotation_multiplier
        self.leftModule.set(rotation - speed)
        self.rightModule.set(rotation + speed)

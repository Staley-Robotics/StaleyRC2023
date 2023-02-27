from wpilib import *
from ctre import *
from ntcore import *

from drivetrain.drivetrain import Drivetrain


class Tank(Drivetrain):
    drivetrain: NetworkTableInstance
    leftModule: MotorControllerGroup
    rightModule: MotorControllerGroup

    def __init__(self, inherited_state: NetworkTableInstance):
        super().__init__(inherited_state)

        self.drivetrain = self.state.getTable("tank")

        self.leftModule = MotorControllerGroup(WPI_TalonFX(self.drivetrain.getNumber("modules/left/0", 1), "rio"),
                                               WPI_TalonFX(self.drivetrain.getNumber("modules/left/1", 2), "rio"))
        self.rightModule = MotorControllerGroup(WPI_TalonFX(self.drivetrain.getNumber("modules/right/0", 3), "rio"),
                                                WPI_TalonFX(self.drivetrain.getNumber("modules/right/1", 4), "rio"))

    def drive(self):
        speed = self.throttle_input.get() * self.throttle_multiplier
        rotation = self.rotation_input.get() * self.rotation_multiplier
        self.leftModule.set(rotation - speed)
        self.rightModule.set(rotation + speed)

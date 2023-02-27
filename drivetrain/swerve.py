import math
from typing import Tuple

from wpilib import *
from ctre import *
from ntcore import *

from drivetrain.drivetrain import Drivetrain


class Swerve(Drivetrain):

    throttle_pinions: MotorControllerGroup
    rotation_pinions: Tuple[WPI_TalonFX, WPI_TalonFX, WPI_TalonFX, WPI_TalonFX]

    def __init__(self, inherited_state: NetworkTableInstance):
        super().__init__(inherited_state)

        self.drivetrain = self.state.getTable("swerve")

        self.throttle_pinions = MotorControllerGroup(
            WPI_TalonFX(self.drivetrain.getNumber("modules/throttle/0", 1), "rio"),
            WPI_TalonFX(self.drivetrain.getNumber("modules/throttle/1", 3), "rio"),
            WPI_TalonFX(self.drivetrain.getNumber("modules/throttle/2", 5), "rio"),
            WPI_TalonFX(self.drivetrain.getNumber("modules/throttle/3", 7), "rio")
        )
        self.rotation_pinions = (
            WPI_TalonFX(self.drivetrain.getNumber("modules/rotation/0", 2), "rio"),
            WPI_TalonFX(self.drivetrain.getNumber("modules/rotation/1", 4), "rio"),
            WPI_TalonFX(self.drivetrain.getNumber("modules/rotation/2", 6), "rio"),
            WPI_TalonFX(self.drivetrain.getNumber("modules/rotation/3", 8), "rio")
        )

    def drive(self):
        speed = self.in_LY.get() * self.throttle_multiplier
        rotation = -math.atan2(self.in_RX.get(), self.in_RY.get()) * self.rotation_multiplier
        self.throttle_pinions.set(speed)
        self.rotation_pinions[0].set(ControlMode.Position, rotation)

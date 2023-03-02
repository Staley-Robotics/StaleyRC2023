from wpilib import *
from ctre import *

from drivetrain.chassis import Chassis


class Tank(Chassis):
    leftModule: MotorControllerGroup
    rightModule: MotorControllerGroup

    def __init__(self, controller: XboxController):
        super().__init__(controller)

        self.leftModule = MotorControllerGroup(WPI_TalonFX(1, "rio"), WPI_TalonFX(2, "rio"))
        self.rightModule = MotorControllerGroup(WPI_TalonFX(3, "rio"), WPI_TalonFX(4, "rio"))

    def drive(self):
        speed = self.pipeline.throttle() * self.throttle_multiplier
        rotation = self.pipeline.rotation() * self.rotation_multiplier
        self.leftModule.set(rotation - speed)
        self.rightModule.set(rotation + speed)

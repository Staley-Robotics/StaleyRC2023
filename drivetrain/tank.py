from wpilib import *
from ctre import *
from ntcore import *


class Tank:
    state: NetworkTableInstance
    drivetrain: NetworkTableInstance
    leftModule: MotorControllerGroup
    rightModule: MotorControllerGroup
    controller: MultiSubscriber

    def __init__(self, inherited_state: NetworkTableInstance):
        self.state = inherited_state
        self.drivetrain = self.state.getTable("drivetrain")

        self.throttleInput = self.state.getFloatTopic("controller/leftY").subscribe(0.0)
        self.rotationInput = self.state.getFloatTopic("controller/leftX").subscribe(0.0)

        self.leftModule = MotorControllerGroup(WPI_TalonFX(self.drivetrain.getNumber("modules/left/0", 1), "rio"), WPI_TalonFX(self.drivetrain.getNumber("modules/left/1", 2), "rio"))
        self.rightModule = MotorControllerGroup(WPI_TalonFX(self.drivetrain.getNumber("modules/right/0", 3), "rio"), WPI_TalonFX(self.drivetrain.getNumber("modules/right/1", 4), "rio"))

    def drive(self):
        speed = self.throttleInput.get() * self.drivetrain.getNumber("control/throttle", 0.0)
        rotation = self.rotationInput.get() * self.drivetrain.getNumber("control/rotation", 0.0)
        self.leftModule.set(rotation - speed)
        self.rightModule.set(rotation + speed)

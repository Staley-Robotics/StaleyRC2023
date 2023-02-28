from wpilib.shuffleboard import Shuffleboard
from wpilib import *
from ntcore import *

from controller import Controller
from drivetrain.chassis import Chassis
from drivetrain.swerve import Swerve
from drivetrain.tank import Tank
from optics.limelight import Limelight


class Robot(TimedRobot):

    # TODO: Merge and incorporate other classes
    time: Timer
    state: NetworkTableInstance
    controller: Controller
    drivetrain: Chassis
    limelight: Limelight

    def _simulationPeriodic(self) -> None:
        Shuffleboard.update()

    def robotInit(self) -> None:
        self.time = Timer()
        self.state = NetworkTableInstance.getDefault()
        self.controller = Controller(0, self.state)
        self.drivetrain = Swerve(self.state)
        self.limelight = Limelight(self.state)

    def teleopInit(self) -> None:
        self.time.start()

    def teleopPeriodic(self) -> None:
        self.controller.update()
        self.drivetrain.drive()
        print(self.limelight.val("tx"))

    def teleopExit(self) -> None:
        self.time.stop()

    # TODO: Implement Auto
    def autonomousInit(self) -> None: ...
    def autonomousPeriodic(self) -> None: ...
    def autonomousExit(self) -> None: ...


if __name__ == "__main__":
    run(Robot)

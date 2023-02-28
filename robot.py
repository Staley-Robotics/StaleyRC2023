from wpilib import *
from ntcore import *

from drivetrain.chassis import Chassis
from drivetrain.swerve import Swerve
from drivetrain.tank import Tank
from optics.limelight import Limelight


class Robot(TimedRobot):

    # TODO: Merge and incorporate other classes
    time: Timer
    state: NetworkTableInstance
    pilot: XboxController
    drivetrain: Chassis
    limelight: Limelight

    def robotInit(self) -> None:
        self.time = Timer()
        self.state = NetworkTableInstance.getDefault()
        self.pilot = XboxController(0)
        self.drivetrain = Swerve(self.state, self.pilot)
        self.limelight = Limelight(self.state)

    def teleopInit(self) -> None:
        self.time.start()

    def teleopPeriodic(self) -> None:
        self.drivetrain.drive()

    def teleopExit(self) -> None:
        self.time.stop()

    # TODO: Implement Auto
    def autonomousInit(self) -> None: ...
    def autonomousPeriodic(self) -> None: ...
    def autonomousExit(self) -> None: ...


if __name__ == "__main__":
    run(Robot)

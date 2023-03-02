from wpilib import *

from appendage.arm import Arm
from appendage.claw import Claw
from drivetrain.chassis import Chassis
from drivetrain.swerve import Swerve
from drivetrain.tank import Tank
from optics.limelight import Limelight


class Robot(TimedRobot):

    # TODO: Merge and incorporate other classes
    time: Timer
    pilot: XboxController
    drivetrain: Chassis
    limelight: Limelight
    arm: Arm
    claw: Claw

    def robotInit(self) -> None:
        self.time = Timer()
        self.pilot = XboxController(0)
        self.drivetrain = Swerve(self.pilot)
        self.limelight = Limelight()
        self.arm = Arm(self.pilot)
        self.claw = Claw(self.pilot)

    def teleopInit(self) -> None:
        self.time.start()

    def teleopPeriodic(self) -> None:
        self.drivetrain.drive()
        self.arm.run_checks()
        self.claw.run_checks()

    def teleopExit(self) -> None:
        self.time.stop()

    # TODO: Implement Auto
    def autonomousInit(self) -> None: ...
    def autonomousPeriodic(self) -> None: ...
    def autonomousExit(self) -> None: ...


if __name__ == "__main__":
    run(Robot)

import wpimath
from wpilib import *

from drivetrain.drivetrain import *
from appendage.arm import Arm
from appendage.claw import Claw
from drivetrain.chassis import Chassis
from drivetrain.swerve_raw import Swerve
from drivetrain.tank import Tank
from optics.limelight import Limelight
from tools import PipelineManager, Mode


class Robot(TimedRobot):
    # TODO: Merge and incorporate other classes
    time: Timer
    pilot: XboxController
    pipeline: PipelineManager
    drivetrain: Chassis
    limelight: Limelight
    arm: Arm
    claw: Claw
    swerve: Drivetrain

    def robotInit(self) -> None:
        self.time = Timer()
        self.pilot = XboxController(0)
        self.pipeline = PipelineManager(self.pilot)
        self.drivetrain = Swerve(self.pipeline)
        self.limelight = Limelight()
        self.arm = Arm(self.pipeline)
        self.claw = Claw(self.pipeline)

    def teleopInit(self) -> None:
        self.time.start()
        self.pipeline.set_mode(Mode.TELEOP)
        self.claw.compressor.enableDigital()

    def teleopPeriodic(self) -> None:
        # self.drivetrain.drive()
        # self.arm.run_checks()
        self.claw.run_checks()

    def teleopExit(self) -> None:
        self.time.stop()

    # TODO: Implement Auto
    def autonomousInit(self) -> None: ...

    def autonomousPeriodic(self) -> None: ...

    def autonomousExit(self) -> None: ...

    def testInit(self) -> None:
        self.swerve = Drivetrain()

    def testPeriodic(self) -> None:
        def clamp(num, min_value):
            if abs(num) < min_value:
                return 0
            return num

        leftx1 = wpimath.applyDeadband(self.pipeline.rotation(), 0.05, 1)
        lefty1 = wpimath.applyDeadband(self.pipeline.throttle(), 0.05, 1)
        rightx1 = wpimath.applyDeadband(self.pipeline.direction_x(), 0.05, 1)

        self.swerve.drive(leftx1, lefty1, rightx1, True)


if __name__ == "__main__":
    run(Robot)

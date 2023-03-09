from typing import List

from drivetrain.swerve_drivetrain import *
from appendage.arm import Arm
from appendage.claw import Claw
# from autonomous.step import Step
# from drivetrain.chassis import Chassis
# from drivetrain.swerve_raw import Swerve
from optics.limelight import Limelight
from tools import PipelineManager, Mode


class Robot(TimedRobot):
    # TODO: Merge and incorporate other classes
    pilot: XboxController
    other: XboxController
    pipeline: PipelineManager
    drivetrain: Chassis
    limelight: Limelight
    arm: Arm
    claw: Claw
    # auto_steps: List[Step]
    step_index: int

    def robotInit(self) -> None:
        self.pilot = XboxController(0)
        self.other = XboxController(1)
        self.pipeline = PipelineManager(Timer(), self.pilot, self.other)
        # self.drivetrain = Swerve(self.pipeline)
        self.limelight = Limelight()
        self.arm = Arm(self.pipeline)
        self.claw = Claw(self.pipeline)
        # self.auto_steps = [
        #     Step(self.drivetrain.goto, 0.0, 12.0)
        # ]
        # self.step_index = 0

    # def robotPeriodic(self) -> None:
        # self.drivetrain.updatePosition()

    def teleopInit(self) -> None:
        self.pipeline.set_mode(Mode.TELEOP)
        self.claw.compressor.enableDigital()

    def teleopPeriodic(self) -> None:
        self.drivetrain.drive()
        self.arm.run_checks()
        self.claw.run_checks()

    def teleopExit(self) -> None:
        self.pipeline.set_mode(Mode.DISABLED)

    # TODO: Implement Auto
    def autonomousInit(self) -> None:
        self.pipeline.set_mode(Mode.AUTO)

    def autonomousPeriodic(self) -> None:
        if self.auto_steps[self.step_index].callback():
            self.step_index += 1

    def autonomousExit(self) -> None:
        self.pipeline.set_mode(Mode.DISABLED)

    def testInit(self) -> None:
        self.pipeline.set_mode(Mode.TELEOP)
        # self.drivetrain = Swerve(self.pipeline)

    def testPeriodic(self) -> None:
        # self.drivetrain.drive()
        self.arm.run_checks()


if __name__ == "__main__":
    run(Robot)
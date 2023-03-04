from wpilib import *

from appendage.arm import Arm
from appendage.claw import Claw
from autonomous.step import Step
from drivetrain.chassis import Chassis
from drivetrain.swerve_raw import Swerve
from optics.limelight import Limelight
from tools import PipelineManager, Mode
from autonomous.process import Process


class Robot(TimedRobot):
    # TODO: Merge and incorporate other classes
    time: Timer
    pilot: XboxController
    other: XboxController
    pipeline: PipelineManager
    drivetrain: Chassis
    limelight: Limelight
    arm: Arm
    claw: Claw
    auto_steps: list[Step]
    step_index: int
    process: Process

    def robotInit(self) -> None:
        self.time = Timer()
        self.pilot = XboxController(0)
        self.other = XboxController(1)
        self.pipeline = PipelineManager(self.pilot, self.other)
        self.drivetrain = Swerve(self.pipeline)
        self.limelight = Limelight()
        self.arm = Arm(self.pipeline)
        self.claw = Claw(self.pipeline)
        self.process = Process([
            Step(
                self.drivetrain.goto, 0, 12
            )
        ])

    def robotPeriodic(self) -> None:
        self.drivetrain.updatePosition()

    def teleopInit(self) -> None:
        self.time.reset()
        self.time.start()
        self.pipeline.set_mode(Mode.TELEOP)
        self.claw.compressor.enableDigital()

    def teleopPeriodic(self) -> None:
        self.drivetrain.drive()
        self.arm.run_checks()
        self.claw.run_checks()

    def teleopExit(self) -> None:
        self.time.stop()

    # TODO: Implement Auto
    def autonomousInit(self) -> None:
        self.time.reset()
        self.time.start()
        self.pipeline.set_mode(Mode.AUTO)
        self.process.begin()

    def autonomousPeriodic(self) -> None:
        # if self.auto_steps[self.step_index].callback():
        #     self.step_index += 1
        self.process.update()

    def autonomousExit(self) -> None:
        self.time.stop()

    def testInit(self) -> None:
        self.pipeline.set_mode(Mode.TELEOP)
        self.drivetrain = Swerve(self.pipeline)

    def testPeriodic(self) -> None:
        self.drivetrain.drive()


if __name__ == "__main__":
    run(Robot)

from typing import List

from wpilib import *

from drivetrain.swerve_drivetrain import *
from appendage.arm import Arm
from appendage.claw import Claw
from autonomous.step import Step
from drivetrain.chassis import Chassis
from drivetrain.swerve_drivetrain import Swerve2
from drivetrain.swerve_raw import Swerve
from drivetrain.tank import Tank
from optics.limelight import Limelight
from tools import PipelineManager, Mode
import time

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
    auto_steps: List[Step]
    step_index: int

    def robotInit(self) -> None:
        time.sleep(5)
        self.time = Timer()
        self.pilot = XboxController(0)
        self.other = XboxController(1)
        self.pipeline = PipelineManager(self.pilot, self.other)
        self.drivetrain = Swerve(self.pipeline)
        self.limelight = Limelight()
        self.arm = Arm(self.pipeline)
        self.claw = Claw(self.pipeline)

        # def step_1():
        #     self.pipeline.throttle_constant(0.5)
        #     if self.drivetrain.m_odometry == None:
        #         return self.time.get() > 4
        #     else:
        #         return self.drivetrain.m_odometry.getPose().translation().Y() >= 3.6576
        #
        # def step_2():
        #     self.pipeline.throttle_constant(-0.5)
        #     if self.drivetrain.m_odometry == None:
        #         return self.time.get() > 10
        #     else:
        #         return self.drivetrain.m_odometry.getPose().translation().Y() <= 1.2446
        #
        # self.auto_steps = [
        #     Step(step_1),
        #     # Step(step_2)
        # ]

        self.step_index = 0

    def robotPeriodic(self) -> None: ...
        # self.drivetrain.updateOdometry()

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

    def autonomousPeriodic(self) -> None:
        pass
        # if self.auto_steps[self.step_index].callback():
        #     self.step_index += 1

    def autonomousExit(self) -> None:
        self.time.stop()

    def testInit(self) -> None:
        self.pipeline.set_mode(Mode.TELEOP)
        self.drivetrain = Swerve(self.pipeline)

    def testPeriodic(self) -> None:
        self.drivetrain.drive()


if __name__ == "__main__":
    run(Robot)

from wpimath.geometry import Pose2d

import wpilib

from subsystems.arm import Arm
from subsystems.claw import Claw
from subsystems.swervedrive import SwerveDrive4


class RobotState:
    pos_x: float
    pos_y: float
    rot: float
    arm_state: int
    grip: bool
    time: int

    def __init__(self, pos_x: float = None, pos_y: float = None, rot: float = None, arm: int = None, grip: bool = None,
                 time: int = 0):
        self.pos_x = pos_x
        self.pos_y = pos_y
        self.rot = rot
        self.arm_state = arm
        self.grip = grip
        self.time = time

    @staticmethod
    def getDefault():
        return RobotState(0.0, 0.0, 0.0, 0, True)


class Auto:
    timer: wpilib.Timer
    steps: dict[str, list[RobotState]]
    mode: str = "autonomousLeft"
    index: int = 0
    is_new_step: bool = True

    def __init__(self, swerve: SwerveDrive4, arm: Arm, claw: Claw):
        self.swerve = swerve
        self.arm = arm
        self.claw = claw
        self.timer = wpilib.Timer()
        self.steps = {
            "autonomousGeneral": [
                RobotState.getDefault(),
                RobotState(pos_x=-0.4572, arm=Arm.MID),
                RobotState(grip=False, time=250),
                RobotState(pos_x=0.5, arm=Arm.BAY, grip=True)
            ]
        }

    def pose_comp(self, pose_a: Pose2d, pose_b: Pose2d, threshold_pos: float, threshold_rot: float = 360) -> bool:
        if abs(pose_a.X() - pose_b.X()) > threshold_pos:
            return False
        if abs(pose_a.Y() - pose_b.Y()) > threshold_pos:
            return False
        if abs(pose_a.rotation().degrees() - pose_b.rotation().degrees()) > threshold_rot:
            return False
        return True

    def drive(self, pose: Pose2d) -> bool:
        self.swerve.driveToPose(pose)
        return self.pose_comp(self.swerve.odometry.getPose(), pose, 0.05, 5)

    def run(self):
        if self.is_new_step:
            self.timer.reset()
            self.timer.start()
            self.is_new_step = False
        goal: RobotState = self.steps[self.mode][self.index]
        self.drive(Pose2d(goal.pos_x, goal.pos_y, goal.rot))
        self.claw.set(goal.grip)
        self.arm.set_mode(goal.arm_state)
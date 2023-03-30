import json
import math

from wpilib import SmartDashboard
from wpimath._controls._controls.trajectory import Trajectory
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
    delay: float

    def __init__(self, pos_x: float = None, pos_y: float = None, rot: float = None, arm: int = None, grip: bool = None,
                 delay: float = 0.0):
        self.pos_x = pos_x
        self.pos_y = pos_y
        self.rot = rot
        self.arm_state = arm
        self.grip = grip
        self.delay = delay

    @staticmethod
    def getDefault():
        return RobotState(0.0, 0.0, 0.0, 0, True)


class Auto:
    timer: wpilib.Timer
    steps: dict[str, list[RobotState]]
    mode: str
    index: int
    using_path: bool
    path: any
    path_timer: wpilib.Timer
    path_index: int

    def __init__(self, swerve: SwerveDrive4, arm: Arm, claw: Claw):
        self.swerve = swerve
        self.arm = arm
        self.claw = claw
        self.timer = wpilib.Timer()
        self.set_mode("AutonomousLeft")
        self.index = 0
        self.steps = {
            "AutonomousLeft": [
                RobotState.getDefault(),
                RobotState(pos_x=6.85, arm=Arm.MID),
                RobotState(grip=False, delay=0.250),
                RobotState(pos_x=0.5, arm=Arm.BAY, grip=True)
            ]
        }
        self.using_path = False
        self.path_timer = wpilib.Timer()
        self.path_timer.start()
        self.path_index = 0
        self.path = []

    def pose_comp(self, pose_a: Pose2d, pose_b: Pose2d, threshold_pos: float, threshold_rot: float = math.pi) -> bool:
        if abs(pose_a.X() - pose_b.X()) > threshold_pos:
            return False
        if abs(pose_a.Y() - pose_b.Y()) > threshold_pos:
            return False
        if abs(pose_a.rotation().radians() - pose_b.rotation().radians()) > threshold_rot:
            return False
        return True

    def set_mode(self, mode: str):
        self.mode = mode
        self.path = json.load(open("./deploy/pathplanner/generatedJSON/" + self.mode + ".wpilib.json"))

    def run(self):
        goal: RobotState = self.steps[self.mode][self.index]
        current_pose: Pose2d = self.swerve.odometry.getEstimatedPosition()
        if self.using_path:
            while self.path_timer.get() > self.path[self.path_index]["time"]:
                self.path_index += 1
            collected_pose = self.path[self.path_index]["pose"]
            goal_pose = Pose2d(collected_pose["translation"]["x"], collected_pose["translation"]["y"],
                               math.degrees(collected_pose["rotation"]["radians"]))
        else:
            goal_pose: Pose2d = Pose2d(goal.pos_x or current_pose.X(), goal.pos_y or current_pose.Y(), goal.rot or current_pose.rotation().radians())

        if self.pose_comp(goal_pose, current_pose, 0.1) and self.index < len(self.steps[self.mode]):
            self.index += 1
            goal = self.steps[self.mode][self.index]
            self.timer.reset()
            self.timer.start()
            # self.swerve.driveToPose(goal_pose)
            self.swerve.driveToState(Trajectory.State(acceleration=self.swerve.speed_angular_maxacceleration,
                                                      velocity=self.swerve.speed_linear_maxvelocity / 2, pose=goal_pose))
        self.claw.set(goal.grip)
        self.arm.set_mode(goal.arm_state)

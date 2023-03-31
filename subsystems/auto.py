import json
import math

from wpimath._controls._controls.trajectory import Trajectory
from wpimath.geometry import Pose2d

from wpilib import TimedRobot, Timer

from subsystems.arm import Arm


class RobotState:
    pos_x: float
    pos_y: float
    rot: float
    arm_state: int
    grip: bool
    min_time: float
    max_time: float
    balance: bool

    def __init__(self, pos_x: float = None, pos_y: float = None, rot: float = None, arm: int = None, grip: bool = None,
                 balance: bool = False, min_time: float = 0.0, max_time: float = 15.0):
        self.pos_x = pos_x
        self.pos_y = pos_y
        self.rot = rot
        self.arm_state = arm
        self.grip = grip
        self.balance = balance
        self.min_time = min_time
        self.max_time = max_time

    @staticmethod
    def getDefault():
        return RobotState(0.0, 0.0, 0.0, 0, True)


def pose_comp(pose_a: Pose2d, pose_b: Pose2d, threshold_pos: float, threshold_rot: float = 360) -> bool:
    if abs(pose_a.X() - pose_b.X()) > threshold_pos:
        return False
    if abs(pose_a.Y() - pose_b.Y()) > threshold_pos:
        return False
    if abs(pose_a.rotation().degrees() - pose_b.rotation().degrees()) > threshold_rot:
        return False
    return True


class Auto:
    robot: TimedRobot
    timer: Timer
    steps: dict[str, list[RobotState]]
    mode: str
    index: int
    using_path: bool
    path: any
    path_timer: Timer
    path_index: int

    def __init__(self, robot: TimedRobot):
        self.robot = robot
        self.timer = Timer()
        self.timer.start()
        self.set_mode("none")
        self.index = 0
        self.steps = {
            "none": [],
            "red-left": [
                RobotState.getDefault(),
                RobotState(arm=Arm.MID, min_time=0.25),
                RobotState(pos_x=1.3, pos_y=4.4),
                RobotState(grip=False, min_time=0.25),
                RobotState(pos_x=2.5, pos_y=4.75, arm=Arm.BAY, grip=True),
                RobotState(pos_x=6, pos_y=4.6, rot=180),
                RobotState(arm=Arm.LOW, grip=False, min_time=0.25),
                RobotState(pos_x=6.5),
                RobotState(grip=True, min_time=0.25),
                RobotState(arm=Arm.BAY, min_time=0.25),
                RobotState(pos_x=5.5, pos_y=3.5, rot=90),
                RobotState(pos_x=3.8),
                RobotState(balance=True)
            ],
            "red-center": [
                RobotState.getDefault(),
                RobotState(arm=Arm.MID, min_time=0.25),
                RobotState(pos_x=1.3, pos_y=2.75),
                RobotState(grip=False, min_time=0.25),
                RobotState(pos_x=2.5, pos_y=2.75, arm=Arm.BAY, grip=True),
                RobotState(rot=90),
                RobotState(pos_x=6),
                RobotState(pos_x=3.8),
                RobotState(balance=True)
            ],
            "red-right": [
                RobotState.getDefault(),
                RobotState(arm=Arm.MID, min_time=0.25),
                RobotState(pos_x=1.3, pos_y=1),
                RobotState(grip=False, min_time=0.25),
                RobotState(pos_x=2.5, pos_y=0.75, arm=Arm.BAY, grip=True),
                RobotState(pos_x=6, pos_y=0.9, rot=180),
                RobotState(arm=Arm.LOW, grip=False, min_time=0.25),
                RobotState(pos_x=6.5),
                RobotState(grip=True, min_time=0.25),
                RobotState(arm=Arm.BAY, min_time=0.25),
                RobotState(pos_x=5.5, pos_y=2, rot=270),
                RobotState(pos_x=3.8, min_time=15),
                RobotState(balance=True)
            ],
            "blue-left": [
                RobotState.getDefault(),
                RobotState(arm=Arm.MID, min_time=0.25),
                RobotState(pos_x=1.3, pos_y=4.4),
                RobotState(grip=False, min_time=0.25),
                RobotState(pos_x=2.5, pos_y=4.75, arm=Arm.BAY, grip=True),
                RobotState(pos_x=6, pos_y=4.6, rot=180),
                RobotState(arm=Arm.LOW, grip=False, min_time=0.25),
                RobotState(pos_x=6.5),
                RobotState(grip=True, min_time=0.25),
                RobotState(arm=Arm.BAY, min_time=0.25),
                RobotState(pos_x=5.5, pos_y=3.5, rot=90),
                RobotState(pos_x=3.8),
                RobotState(balance=True)
            ],
            "blue-center": [
                RobotState.getDefault(),
                RobotState(arm=Arm.MID, min_time=0.25),
                RobotState(pos_x=1.3, pos_y=2.75),
                RobotState(grip=False, min_time=0.25),
                RobotState(pos_x=2.5, pos_y=2.75, arm=Arm.BAY, grip=True),
                RobotState(rot=90),
                RobotState(pos_x=6),
                RobotState(pos_x=3.8),
                RobotState(balance=True)
            ],
            "blue-right": [
                RobotState.getDefault(),
                RobotState(arm=Arm.MID, min_time=0.25),
                RobotState(pos_x=1.3, pos_y=1),
                RobotState(grip=False, min_time=0.25),
                RobotState(pos_x=2.5, pos_y=0.75, arm=Arm.BAY, grip=True),
                RobotState(pos_x=6, pos_y=0.9, rot=180),
                RobotState(arm=Arm.LOW, grip=False, min_time=0.25),
                RobotState(pos_x=6.5),
                RobotState(grip=True, min_time=0.25),
                RobotState(arm=Arm.BAY, min_time=0.25),
                RobotState(pos_x=5.5, pos_y=2, rot=270),
                RobotState(pos_x=3.8, min_time=15),
                RobotState(balance=True)
            ]
        }
        self.using_path = False
        self.path_timer = Timer()
        self.path_timer.start()
        self.path_index = 0
        self.path = []

    def set_mode(self, mode: str):
        self.mode = mode
        self.path = json.load(open("./deploy/pathplanner/generatedJSON/" + self.mode + ".wpilib.json"))

    def run(self):
        goal: RobotState = self.steps[self.mode][self.index]
        current_pose: Pose2d = self.robot.swerve.odometry.getEstimatedPosition()
        if self.using_path:
            while self.path_timer.get() > self.path[self.path_index]["time"]:
                self.path_index += 1
            collected_pose = self.path[self.path_index]["pose"]
            goal_pose = Pose2d(collected_pose["translation"]["x"], collected_pose["translation"]["y"],
                               math.degrees(collected_pose["rotation"]["radians"]))
        else:
            goal_pose: Pose2d = Pose2d(goal.pos_x or current_pose.X(), goal.pos_y or current_pose.Y(),
                                       goal.rot or current_pose.rotation().degrees())

        if pose_comp(goal_pose, current_pose, 0.1) and self.timer.get() >= goal.min_time or self.timer.get() >= goal.max_time:
            self.index += 1
            goal = self.steps[self.mode][self.index]
            self.timer.reset()
            self.timer.start()

        if goal.balance:
            self.robot.swerve.balance()
        else:
            # self.swerve.driveToPose(goal_pose)
            self.robot.swerve.driveToState(
                Trajectory.State(acceleration=self.robot.swerve.speed_angular_maxacceleration,
                                 velocity=self.robot.swerve.speed_linear_maxvelocity / 4, pose=goal_pose))
        self.robot.claw.set(goal.grip)
        self.robot.arm.set_mode(goal.arm_state)

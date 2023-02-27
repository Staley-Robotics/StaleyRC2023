from wpilib import *
from wpimath.geometry import *
from ctre import *
from ntcore import *


class SwerveModule:

    pose: Pose2d
    driving_pinion: WPI_TalonFX
    rotation_pinion: WPI_TalonFX

    def __init__(self, driving_pinion_pid: int, rotation_pinion_pid: int):
        self.driving_pinion = WPI_TalonFX(driving_pinion_pid)
        self.rotation_pinion = WPI_TalonFX(rotation_pinion_pid)
        self.pose = Pose2d()


class Swerve:

    state: NetworkTableInstance
    rotation_pinions: MotorControllerGroup




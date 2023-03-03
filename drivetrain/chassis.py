from wpimath.kinematics import SwerveDrive4Odometry

from tools import PipelineManager


class Chassis:

    pipeline: PipelineManager

    throttle_multiplier: float = 0.3
    rotation_multiplier: float = 0.2
    direction_multiplier: float = 0.3
    talon_srx_resolution: int = 4096
    talon_fx_resolution: int = 2048

    m_odometry: SwerveDrive4Odometry = None

    def __init__(self, pipeline: PipelineManager):
        self.pipeline = pipeline

    def drive(self):
        print("No logic connected to drivetrain")

    def updateOdometry(self):
        print("No logic connected to drivetrain")

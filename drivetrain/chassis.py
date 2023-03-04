from wpimath.kinematics import SwerveDrive4Odometry

from tools import PipelineManager


class Chassis:

    pipeline: PipelineManager
    position: list[float, float] = [0.0, 0.0]
    head: float = 0.0

    throttle_multiplier: float = 0.3
    rotation_multiplier: float = 0.2
    direction_multiplier: float = 0.3
    talon_srx_resolution: int = 4096
    talon_fx_resolution: int = 2048
    distance_at_max: float = 131.5181176503937

    def velocity_to_distance(self, velocity: float):
        return velocity * self.distance_at_max

    def __init__(self, pipeline: PipelineManager):
        self.pipeline = pipeline

    def drive(self):
        print("No logic connected to drivetrain")

    def updatePosition(self):
        print("No logic connected to drivetrain")

    def goto(self, x: float, y: float, r: float = 0):
        print("No logic connected to drivetrain")

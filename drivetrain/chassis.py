from tools import PipelineManager


class Chassis:

    pipeline: PipelineManager

    throttle_multiplier: float = 0.7
    rotation_multiplier: float = 0.4
    direction_multiplier: float = 0.3
    talon_srx_resolution: int = 4096
    talon_fx_resolution: int = 2048

    def __init__(self, pipeline: PipelineManager):
        self.pipeline = pipeline

    def drive(self):
        print("No logic connected to drivetrain")

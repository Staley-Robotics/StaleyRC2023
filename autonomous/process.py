from autonomous.step import Step
from drivetrain.swerve_raw import Swerve
from drivetrain.chassis import Chassis
from tools import PipelineManager, Mode


class Process:
    index: int = 0
    steps: list[Step] = []

    def __init__(self, steps):
        self.steps = steps

    def begin(self):
        self.steps[0].callback(self.steps[0].params)

    def update(self):
        if self.steps[self.index].complete:
            self.index += 1
            self.steps[self.index].callback(self.steps[self.index].params)


        # if self.control < 2.5:
        #     self.pipeline.throttle_constant(0.5)
        #     if self.drivetrain.odometry is None:
        #         return self.time.get() > 5
        #     else:
        #         return self.drivetrain.odometry.getPose().translation().Y() >= 3.6576
        #
        # elif 2.5 < self.control < 5:
        #     self.pipeline.throttle_constant(-0.5)
        #
        # else:
        #     print("switching from for to backwards")
        #
        # self.control += 0.2







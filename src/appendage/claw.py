from wpilib import *

from tools import PipelineManager


class Claw:
    pipeline: PipelineManager
    module: PneumaticsControlModule
    compressor: Compressor
    solenoid: Solenoid

    def __init__(self, pipeline: PipelineManager):
        self.pipeline = pipeline
        self.module = PneumaticsControlModule(0)
        self.solenoid = self.module.makeSolenoid(1)

        self.compressor = self.module.makeCompressor()
        self.compressor.disable()

    def run_checks(self):
        self.solenoid.set(self.pipeline.grip())

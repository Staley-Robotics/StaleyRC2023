from typing import List

from wpilib import *

from tools import PipelineManager


class Claw:
    pipeline: PipelineManager
    module: PneumaticsControlModule
    compressor: Compressor
    solenoid: DoubleSolenoid

    def __init__(self, pipeline: PipelineManager):
        self.pipeline = pipeline
        self.module = PneumaticsControlModule(0)
        self.solenoid = self.module.makeDoubleSolenoid(0, 4)

        self.compressor = self.module.makeCompressor()
        self.compressor.disable()

    def run_checks(self):
        print(self.compressor.getPressureSwitchValue())
        if self.compressor.isEnabled() and self.compressor.getPressureSwitchValue():
            self.compressor.disable()
        elif not self.compressor.isEnabled():
            self.compressor.enableDigital()

        if self.pipeline.grip():
            self.solenoid.set(self.solenoid.Value.kForward)
        elif self.pipeline.release():
            self.solenoid.set(self.solenoid.Value.kReverse)
        else:
            self.solenoid.set(self.solenoid.Value.kOff)

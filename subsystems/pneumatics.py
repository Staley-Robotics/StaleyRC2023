from . import Subsystems

from wpilib import PneumaticsControlModule, Compressor


class Pneumatics:
    module: PneumaticsControlModule
    compressor: Compressor

    def __init__(self):
        self.module = PneumaticsControlModule(0)
        self.module.clearAllStickyFaults()

        self.compressor = self.module.makeCompressor()
        self.compressor.disable()

    def run(self):
        if self.compressor.isEnabled() and self.compressor.getPressureSwitchValue():
            self.compressor.disable()
        if not self.compressor.isEnabled() and not self.compressor.getPressureSwitchValue():
            self.compressor.enableDigital()

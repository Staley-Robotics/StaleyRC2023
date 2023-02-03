import wpilib


class Pcm:
    module: wpilib.PneumaticsControlModule = None
    compressor: wpilib.Compressor = None

    def __init__(self, module):
        self.module = module
        self.compressor = self.module.makeCompressor()
        self.compressor.disable()

    def update(self):
        if self.compressor.isEnabled() and not self.compressor.getPressureSwitchValue():
            self.compressor.disable()
        if not self.compressor.isEnabled() and self.compressor.getPressureSwitchValue():
            self.compressor.enableDigital()

    def getSolendoid(self, channelNumber):
        return self.module.makeSolenoid(channelNumber)

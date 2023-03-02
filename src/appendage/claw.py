from wpilib import *


class Claw:
    controller: XboxController
    module: PneumaticsControlModule
    compressor: Compressor
    solenoid: Solenoid

    def __init__(self, controller: XboxController):
        self.controller = controller
        self.module = PneumaticsControlModule(0)
        self.solenoid = self.module.makeSolenoid(1)

        self.compressor = self.module.makeCompressor()
        self.compressor.disable()

    def run_checks(self):
        self.solenoid.set(self.controller.getRightTriggerAxis() > 0.7)

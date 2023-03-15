from . import Subsystems

from wpilib import *


class Claw(Subsystems):
    module: PneumaticsControlModule
    compressor: Compressor
    solenoid: DoubleSolenoid
    op2: XboxController
    clawOpen: bool = False

    def initVariables(self):
        self.op2 = XboxController(1)
        self.module = PneumaticsControlModule(0)
        self.module.clearAllStickyFaults()
        # self.solenoid = self.module.makeDoubleSolenoid(4, 0)
        self.solenoid = DoubleSolenoid(0, PneumaticsModuleType.CTREPCM, 4, 7) #self.module.makeDoubleSolenoid(4, 0)
        #self.solenoid.set(DoubleSolenoid.Value.kForward)

        self.compressor = self.module.makeCompressor()
        self.compressor.disable()

    def getInputs(self) -> bool:
        grip = self.op2.getAButtonPressed()
        return grip

    def runInit(self):
        self.solenoid.set( DoubleSolenoid.Value.kForward )

    def run(self):
        # print(self.compressor.getPressureSwitchValue())
        if self.compressor.isEnabled() and self.compressor.getPressureSwitchValue():
            self.compressor.disable()
        if not self.compressor.isEnabled() and not self.compressor.getPressureSwitchValue():
            self.compressor.enableDigital()
        self.toggle(self.getInputs())

    def toggle(self, toggle_value: bool):
        if toggle_value:
            if self.clawOpen:
                self.clawOpen = False
                self.solenoid.set( DoubleSolenoid.Value.kForward )
                print("Open")
            else:
                self.clawOpen = True
                self.solenoid.set( DoubleSolenoid.Value.kReverse )
                print("Close")

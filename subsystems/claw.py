from . import Subsystems

from wpilib import *

from .pneumatics import Pneumatics


class Claw(Subsystems):
    solenoid: DoubleSolenoid
    op2: XboxController
    clawOpen: bool = False
    pneumatics: Pneumatics

    def __init__(self, pneumatics: Pneumatics):
        super().__init__()
        self.pneumatics = pneumatics


    def initVariables(self):
        self.op2 = XboxController(1)
        # self.solenoid = self.module.makeDoubleSolenoid(4, 0)
        self.solenoid = DoubleSolenoid(0, PneumaticsModuleType.CTREPCM, 3, 5) #self.module.makeDoubleSolenoid(4, 0)
        #self.solenoid.set(DoubleSolenoid.Value.kForward)

    def getInputs(self) -> bool:
        grip = self.op2.getAButtonPressed()
        return grip

    def runInit(self):
        self.solenoid.set( DoubleSolenoid.Value.kForward )

    def run(self):
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